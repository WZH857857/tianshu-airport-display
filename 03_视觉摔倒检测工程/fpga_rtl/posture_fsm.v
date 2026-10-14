//===========================================================================
// posture_fsm.v -- integer RTL port of PostureMachine in tools/fall_proto.py
//
// One instance per tracked motion blob (up to 4 in the crowd path).  The
// Python state machine is mirrored exactly with integer cross-multiplication
// so no floating point is needed anywhere:
//
//   ratio <= 0.95        ->  20*h  <= 19*w
//   h >= 0.60 * ref_h    ->   5*h  >=  3*ref_h
//   h <= 1.00 * ref_h    ->  h  <= ref_h   (no shrink gate: a fall toward
//                            the camera keeps its height, see below)
//   h >= 0.80 * ref_h    ->   5*h  >=  4*ref_h
//   fill >= 0.25         ->   4*area >= w*h
//   sink >= 0.04 * H     ->  cy2_delta >= SUNK_CY2  (cy2 = 2*y + h - 1)
//   bottom_tol 0.10      ->  10*(y+h) + 1*ref_h >= 10*(ref_y+ref_h)
//   keep_area  0.30      ->  10*area >= 3*ref_area
//
// FRAME_W/FRAME_H are the working resolution of the source clip (the offline
// traces range from 320x180 to 360x260); the camera build is fixed at
// 320x240 and passes the constants below accordingly.
//===========================================================================
`timescale 1ns / 1ps

module posture_fsm #(
    parameter integer FRAME_W         = 320,
    parameter integer FRAME_H         = 240,
    parameter integer MIN_REF_AREA    = 1600,
    parameter integer MIN_REF_HEIGHT  = 25,
    parameter integer MAX_REF_W       = 176,    // floor(0.55 * FRAME_W)
    parameter integer MAX_REF_H       = 228,    // floor(0.95 * FRAME_H)
    parameter integer MAX_REF_AREA    = 15360,  // floor(0.20 * FRAME_W*FRAME_H)
    parameter integer SUNK_CY2        = 20,     // ceil(0.08 * FRAME_H)
    parameter integer REF_TTL         = 300,
    parameter integer COLLAPSE_FRAMES = 18,
    parameter integer RECOVERY_WINDOW = 2,
    parameter integer COOLDOWN_FRAMES = 60,

    // Width of the state bundle carried by ctx_in / ctx_out.  The layout is
    // spelled out once, in the ctx_out assignment below; posture_bank only
    // moves the vector around and never looks inside it.
    parameter integer CTX_W           = 242
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- context save / restore -------------------------------------------
    // posture_bank time-shares one engine over the four crowd slots: before a
    // slot is fed, its saved state bundle is driven into ctx_in with ctx_load
    // high; the updated bundle comes back out of ctx_out two clocks later.
    // A caller that owns the engine outright (the solo board top and the
    // vector replay benches) ties ctx_load low and the ports are unused.
    input  wire [CTX_W-1:0] ctx_in,
    input  wire             ctx_load,
    output wire [CTX_W-1:0] ctx_out,

    // Clock enable: the machine advances one row per enabled clock.  A
    // free-running caller ties it high and drives one row per clock; the
    // vision chain drives it from the bbox result pulse so that gaps and
    // cooldowns are counted in frames, not in clock cycles.
    input  wire        en,

    // One feed() call per clock.  valid=0 replicates feed(None, frame):
    // a gap with no motion blob for this track.
    input  wire        valid,
    input  wire [31:0] frame,
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire [9:0]  w,
    input  wire [9:0]  h,
    input  wire [23:0] area,
    input  wire [2:0]  confirm_required,   // 2 solo, 4 crowd

    output reg         alarm,              // single-clock pulse
    output reg  [2:0]  state
);

localparam [2:0] ST_SEEK      = 3'd0,
                 ST_PENDING   = 3'd1,
                 ST_FALL      = 3'd2,
                 ST_RECOVERED = 3'd3;

// ------------------------------------------------------------ registered state
reg        ref_valid;
reg [31:0] ref_frame;
reg [9:0]  ref_x, ref_y, ref_w, ref_h;
reg [23:0] ref_area;
reg [23:0] ref_wh;             // registered ref_h * ref_w
reg        last_tall_valid;
reg [31:0] last_tall;
reg [15:0] flat_frames;
reg [15:0] cooldown;
reg        pending;
reg [31:0] pending_frame;
reg [9:0]  pending_top;
reg [9:0]  pending_ref_h;

// ------------------------------------------------------------ context bundle
// The concatenation order here and in the ctx_load restore below must match
// exactly -- posture_bank treats the bundle as opaque, so a mismatch would
// shuffle the saved state instead of failing loudly.
wire [CTX_W-1:0] ctx_now = {
    ref_valid, ref_frame, ref_x, ref_y, ref_w, ref_h,
    ref_area, ref_wh, last_tall_valid, last_tall,
    flat_frames, cooldown, pending, pending_frame,
    pending_top, pending_ref_h, state
};

assign ctx_out = ctx_now;

// ============================ observation pipeline =========================
//
// The first P&R run of the board top missed 48 MHz by 0.161 ns on a single
// 16-logic-level path: bbox height -> w*h (MULT18) -> 20*h / 13*rh -> the
// cy/sunk and ground comparisons -> a priority mux into pending_frame.  The
// fix is one pipeline stage across the whole observation, not a shaved
// adder: stage 1 registers the products and the ratio comparisons, stage 2
// runs the identical state machine on those registered values.
//
// en is the caller's "this observation is valid" strobe (one clock per
// frame in fall_core_top), so shifting the state machine by one clock does
// not change any decision.  It does give every comparison in the machine a
// full 20.833 ns budget of its own, which is what the old path never had.
// en_q is a delayed copy of the strobe; do not replace it with `en` while
// en is a one-shot pulse, or the machine would never advance.
reg        p_valid;
reg [31:0] p_frame;
reg [9:0]  p_x, p_y, p_w, p_h;
reg [23:0] p_area;
reg [23:0] p_wh;                 // p_w * p_h
reg [14:0] p_h20, p_w19;         // 20*h, 19*w
reg        p_plike;              // person-like test of the observation
reg        p_flat;               // h/w <= 0.95
reg        p_tall_obs;           // h >= 0.60*ref_h
reg        p_shrunk;             // h <= collapse_frac*ref_h (1.00)
reg        p_stands;             // h >= 0.80*ref_h and h >= w
reg        en_q;                 // p_* hold the observation from `en`

// Reference-box selection.  This block comes first on purpose: the stage-1
// comparisons compare the observation against the *current* reference, so
// rh / rw / ry / ra and ref_person_like are consumed above their natural
// place in the file.  TD reports HDL-5373 ("identifier used before its
// declaration") and falls back to an implicit net when the declaration
// follows the use, so the declarations sit in front of every user.
wire ref_alive = ref_valid && ((p_frame - ref_frame) <= REF_TTL);
wire ref_set   = p_valid && p_plike && (!ref_alive || (p_h > ref_h));
wire ref_ok_n  = p_valid && (ref_set || ref_alive);
wire [9:0]  rh = ref_set ? p_h : ref_h;
wire [9:0]  rw = ref_set ? p_w : ref_w;
wire [9:0]  ry = ref_set ? p_y : ref_y;
wire [23:0] ra = ref_set ? p_area : ref_area;

// ref_person_like tests the reference box's fill ratio, ref_area vs
// ref_h * ref_w.  Keeping that product in a register removes a second
// multiplier from the critical path: when the reference is being replaced the
// product is exactly p_wh, which the person-like test already computed, and
// otherwise it is the stored product of the standing reference.
wire [23:0] ref_wh_eff = ref_set ? p_wh : ref_wh;
wire ref_person_like = (ra >= MIN_REF_AREA) && (rh >= MIN_REF_HEIGHT) &&
                       (rh >= rw) && ((ra << 2) >= ref_wh_eff) &&
                       (rw <= MAX_REF_W) && (rh <= MAX_REF_H) &&
                       (ra <= MAX_REF_AREA);

// Stage 1 combinational inputs.  The products are declared before the
// comparisons that consume them: without that TD warns "identifier used
// before its declaration" and leaves the reference to an implicit net.
wire [14:0] h15     = {5'b0, h};
wire [14:0] w15     = {5'b0, w};
wire [14:0] rh15    = {5'b0, rh};
wire [23:0] wh      = w * h;
wire        valid_w = valid;
wire [14:0] h5      = (h15 << 2) + h15;                // 5*h
wire [14:0] rh3     = (rh15 << 1) + rh15;              // 3*rh
wire [14:0] rh4     = (rh15 << 2);                     // 4*rh
wire [14:0] h20     = (h15 << 4) + (h15 << 2);         // 20*h
wire [14:0] w19     = (w15 << 4) + (w15 << 1) + w15;   // 19*w

// Stage 1 arithmetic.  Every constant multiply is an explicit shift+add:
// written as "20 * h" TD maps it onto a DSP block, and two of those in one
// row of evidence put 7 ns of MULT18 on the critical path.  The comparisons
// are exact integers either way, so only the netlist changes.  Each operand
// is widened first because a shift operator in Verilog takes its width from
// the left operand; shifting a 10-bit value by 4 would drop the top bits.

wire plike_w  = (area >= MIN_REF_AREA) && (h >= MIN_REF_HEIGHT) &&
                (h >= w) && ((area << 2) >= wh) &&
                (w <= MAX_REF_W) && (h <= MAX_REF_H) &&
                (area <= MAX_REF_AREA);
wire flat_w   = (h20 <= w19);                          // h/w <= 0.95
// The height gate used to be h <= 0.65*ref_h.  That killed the four demo
// loops where the person falls toward the camera: the projected blob grows
// as the body comes nearer, so h > ref_h and the collapse was rejected even
// though flat/sunk/grounded all held.  Any observation that is no taller
// than the standing reference now qualifies; the flat + speed + keep-area
// gates carry the discrimination instead.
wire shrunk_w = (h <= rh);                             // h <= 1.00*ref_h
wire stood_w  = (h5 >= rh4) && (h >= w);               // h >= 0.80*ref_h

// ------------------------------------------------------------ per-row evidence
wire flat     = p_flat;
wire [15:0] flat_n = flat ? (flat_frames + 16'd1) : 16'd0;
wire tall_obs = ref_ok_n && p_tall_obs;    // h >= 0.60*ref_h

wire [10:0] cy2_obs = (p_y << 1) + p_h - 1'b1;
wire [10:0] cy2_ref = (ry << 1) + rh - 1'b1;
wire signed [12:0] cy2_delta =
    $signed({2'b00, cy2_obs}) - $signed({2'b00, cy2_ref});

wire sunk = (cy2_delta >= SUNK_CY2);

wire [10:0] yh11  = {1'b0, p_y}  + {1'b0, p_h};
wire [10:0] ryh11 = {1'b0, ry} + {1'b0, rh};
wire [17:0] yh18  = {7'b0, yh11};
wire [17:0] ryh18 = {7'b0, ryh11};
wire [17:0] rh18  = {8'b0, rh};
// The bottom tolerance used to be 0.15*ref_h (20*(y+h) + 3*ref_h).  On the
// crowd negatives a walking pair merges, the blob jumps from a full body to
// one torso, and the bottom rises by ~11% of the standing height - inside
// the old window, so it alarmed.  A real fall keeps its mask on the floor,
// so 0.10*ref_h still passes every GMDCSA24 fall in the suite while the
// crowd sweep drops to zero false alarms.
wire [17:0] ground_l = (yh18 << 3) + (yh18 << 1) + rh18;  // 10*yh + rh
wire [17:0] ground_r = (ryh18 << 3) + (ryh18 << 1);       // 10*ryh
wire grounded = (ground_l >= ground_r);

wire [27:0] a27  = {4'b0, p_area};
wire [27:0] ra27 = {4'b0, ra};
wire [27:0] a10  = (a27 << 3) + (a27 << 1);            // 10*area
wire [27:0] ra3  = (ra27 << 1) + ra27;                 // 3*ra
wire keep     = (a10 >= ra3);

wire shrunk   = p_shrunk;                          // h <= 1.00*ref_h
wire tall     = (rh >= MIN_REF_HEIGHT) && ref_person_like;
wire fast     = last_tall_valid && ((p_frame - last_tall) <= COLLAPSE_FRAMES);

// ---------------------------------------------------------- pending candidate
wire [12:0] y4   = {3'b0, p_y} << 2;
wire [12:0] top4 = {3'b0, pending_top} << 2;
wire rises = pending && ((y4 + {1'b0, pending_ref_h}) <= top4);
wire stands = ref_ok_n && p_stands;
wire recovery_now = p_valid && pending && (rises || stands);
wire window_elapsed = pending && !recovery_now &&
                      ((p_frame - pending_frame) >= RECOVERY_WINDOW);
wire pending_live = pending && !recovery_now;      // still waiting this row
wire [15:0] flat_eff = recovery_now ? 16'd0 : flat_n;

// The Python code decrements the cooldown at the top of feed() and then
// tests cooldown == 0, so the test sees the *decremented* value.
wire [15:0] cooldown_eff = (cooldown == 16'd0) ? 16'd0 : (cooldown - 16'd1);

wire detect_fire = p_valid && ref_ok_n && (cooldown_eff == 16'd0) &&
                   !pending_live && tall && shrunk && flat && sunk &&
                   grounded && keep && fast &&
                   (flat_eff >= confirm_required);

// ------------------------------------------------------------------ sequential
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ref_valid      <= 1'b0;
        ref_frame      <= 32'd0;
        ref_x          <= 10'd0;
        ref_y          <= 10'd0;
        ref_w          <= 10'd0;
        ref_h          <= 10'd0;
        ref_area       <= 24'd0;
        ref_wh         <= 24'd0;
        last_tall_valid<= 1'b0;
        last_tall      <= 32'd0;
        flat_frames    <= 16'd0;
        cooldown       <= 16'd0;
        pending        <= 1'b0;
        pending_frame  <= 32'd0;
        pending_top    <= 10'd0;
        pending_ref_h  <= 10'd0;
        p_valid        <= 1'b0;
        p_frame        <= 32'd0;
        p_x            <= 10'd0;
        p_y            <= 10'd0;
        p_w            <= 10'd0;
        p_h            <= 10'd0;
        p_area         <= 24'd0;
        p_wh           <= 24'd0;
        p_h20          <= 15'd0;
        p_w19          <= 15'd0;
        p_plike        <= 1'b0;
        p_flat         <= 1'b0;
        p_tall_obs     <= 1'b0;
        p_shrunk       <= 1'b0;
        p_stands       <= 1'b0;
        en_q           <= 1'b0;
        state          <= ST_SEEK;
        alarm          <= 1'b0;
    end else begin
        // alarm is a one-clock pulse in every configuration.  Clearing it
        // only inside the `en` branch is wrong when the caller enables this
        // machine once per observation: a fired alarm then stays high until
        // the next frame.  The board top samples alarm into a toggle flop and
        // the crowd path ORs several machines, so a frame-long level is not
        // just untidy, it is a different waveform than the port promises.
        alarm <= 1'b0;

        // Context restore for the time-shared engine.  posture_bank raises
        // ctx_load only on a clock where en is low, so the observation
        // pipeline below stays empty and no state step runs on this edge.
        if (ctx_load) begin
            {ref_valid, ref_frame, ref_x, ref_y, ref_w, ref_h,
             ref_area, ref_wh, last_tall_valid, last_tall,
             flat_frames, cooldown, pending, pending_frame,
             pending_top, pending_ref_h, state} <= ctx_in;
        end else begin

        // Stage 1: capture this observation and its pre-computed products.
        en_q <= en;
        if (en) begin
            p_valid    <= valid_w;
            p_frame    <= frame;
            p_x        <= x;
            p_y        <= y;
            p_w        <= w;
            p_h        <= h;
            p_area     <= area;
            p_wh       <= wh;
            p_h20      <= h20;
            p_w19      <= w19;
            p_plike    <= plike_w;
            p_flat     <= flat_w;
            p_tall_obs <= (h5 >= rh3);
            p_shrunk   <= shrunk_w;
            p_stands   <= stood_w;
        end

        // Stage 2: the unchanged state machine, one clock behind the strobe.
        //
        // Everything below reads the captured observation (p_*), never the
        // input ports.  The ports only have to be valid on the `en` edge; by
        // the time this branch runs the caller is free to have moved on to
        // the next frame, and the chain testbench does exactly that (it bumps
        // its frame counter on the same edge that strobes `en`).  Reading the
        // ports here once cost a whole frame of latency: pending_frame was
        // latched as frame+1, so a candidate detected on frame 29 confirmed
        // on 32 instead of 31.
        if (en_q) begin
        if (cooldown != 16'd0) cooldown <= cooldown - 16'd1;

        if (!p_valid) begin
            // A gap breaks the "continuously flat" evidence.
            flat_frames <= 16'd0;
            if (pending &&
                ((p_frame - pending_frame) >= RECOVERY_WINDOW)) begin
                state    <= ST_FALL;
                pending  <= 1'b0;
                cooldown <= COOLDOWN_FRAMES;
                alarm    <= 1'b1;
            end
        end else begin
            // 1) standing-reference maintenance
            if (ref_valid && ((p_frame - ref_frame) > REF_TTL))
                ref_valid <= 1'b0;
            if (ref_set) begin
                ref_valid <= 1'b1;
                ref_frame <= p_frame;
                ref_x     <= p_x;
                ref_y     <= p_y;
                ref_w     <= p_w;
                ref_h     <= p_h;
                ref_area  <= p_area;
                ref_wh    <= p_wh;
            end
            if (tall_obs) begin
                last_tall       <= p_frame;
                last_tall_valid <= 1'b1;
            end

            // 2) flat counter
            if (flat) flat_frames <= flat_frames + 16'd1;
            else      flat_frames <= 16'd0;

            // 3) pending candidate: stand-up recovery or silent confirm
            if (recovery_now) begin
                pending     <= 1'b0;
                state       <= ST_RECOVERED;
                flat_frames <= 16'd0;
            end else if (window_elapsed) begin
                pending  <= 1'b0;
                state    <= ST_FALL;
                cooldown <= COOLDOWN_FRAMES;
                alarm    <= 1'b1;
            end

            // 4) new collapse candidate
            if (detect_fire) begin
                pending       <= 1'b1;
                pending_frame <= p_frame;
                pending_top   <= p_y;
                pending_ref_h <= rh;
                state         <= ST_PENDING;
                if (RECOVERY_WINDOW <= 1) begin
                    pending  <= 1'b0;
                    state    <= ST_FALL;
                    cooldown <= COOLDOWN_FRAMES;
                    alarm    <= 1'b1;
                end
            end

            // 5) display-state auto recovery after a confirmed fall
            if ((state == ST_FALL) && !flat && !window_elapsed &&
                !(detect_fire && (RECOVERY_WINDOW <= 1)))
                state <= ST_RECOVERED;
        end
        end
        end
    end
end

endmodule
