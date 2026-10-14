//===========================================================================
// posture_bank.v -- one posture engine, four saved contexts
//
// The crowd path used to instantiate four copies of posture_fsm, one per
// slot.  Each copy costs about 800 LUT columns (468 logic + 329 carry), so
// the four of them were 3,200 of the 15,076 LUTs the board top needs.  At
// 76.9% utilisation the router ran out of room: placement seeds 2, 3 and 4
// all ended with one or two PHY-8023 shorts inside u_core/u_bbox, and the
// vendor CongestionHigh parameter set did not change that.
//
// This module keeps the four state bundles in registers and feeds them
// through a single posture_fsm, one slot at a time.  The machines only
// advance once per frame (en is the bbox_valid pulse) and a frame is 1.6 M
// clocks at 48 MHz, so the serial sweep costs 18 of them.
//
// Port-level behaviour is the same as four parallel machines: state[g] is
// settled before the next frame, and the fired slots raise alarm together.
// The alarm level is held until the next snapshot strobe rather than pulsed
// for one clock, because the whole sweep finishes ~18 clocks after the strobe
// and a one-clock pulse would be easy for a slow reader to miss; the board
// top edge-detects it into its 24 MHz toggle handshake.  Only the latency
// after the en strobe moves, from 2 clocks to about 18.
//
// Per slot the sweep is:
//   LOAD    ctx_load=1, en=0   saved bundle -> engine registers
//   FEED    en=1               engine captures this slot's observation
//   UPDATE  en=0               engine steps the state machine
//   CAPTURE ctx_load=0         latch ctx_out and the alarm pulse
//===========================================================================
`timescale 1ns / 1ps

module posture_bank #(
    parameter integer NUM_BLOBS       = 4,
    parameter integer CTX_W           = 242,
    parameter integer FRAME_W         = 320,
    parameter integer FRAME_H         = 180,
    parameter integer MIN_REF_AREA    = 1600,
    parameter integer MIN_REF_HEIGHT  = 25,
    parameter integer MAX_REF_W       = 176,
    parameter integer MAX_REF_H       = 171,
    parameter integer MAX_REF_AREA    = 11520,
    parameter integer SUNK_CY2        = 15,
    parameter integer REF_TTL         = 300,
    parameter integer COLLAPSE_FRAMES = 36,
    parameter integer RECOVERY_WINDOW = 2,
    parameter integer COOLDOWN_FRAMES = 60
)(
    input  wire        clk,
    input  wire        rst_n,

    // One snapshot strobe per frame, together with that frame's number.
    input  wire        en,
    input  wire [31:0] frame,

    // Per-slot observations, published by multi_bbox.
    input  wire [NUM_BLOBS-1:0]    valid,
    input  wire [NUM_BLOBS*10-1:0] bx,
    input  wire [NUM_BLOBS*10-1:0] by,
    input  wire [NUM_BLOBS*10-1:0] bw,
    input  wire [NUM_BLOBS*10-1:0] bh,
    input  wire [NUM_BLOBS*24-1:0] barea,
    input  wire [2:0]              confirm_required,

    output wire [NUM_BLOBS-1:0]   alarm,
    output wire [NUM_BLOBS*3-1:0] state
);

    localparam [2:0] PH_IDLE = 3'd0,
                     PH_LOAD = 3'd1,
                     PH_FEED = 3'd2,
                     PH_UPD  = 3'd3,
                     PH_CAP  = 3'd4,
                     PH_DONE = 3'd5;

    reg [2:0]  phase;
    reg        busy;
    reg [1:0]  slot;
    reg [31:0] obs_frame;

    // Per-slot latched observation and saved context.  Four entries each, so
    // TD builds these out of registers and muxes rather than RAM.
    reg        obs_valid [0:NUM_BLOBS-1];
    reg [9:0]  obs_x     [0:NUM_BLOBS-1];
    reg [9:0]  obs_y     [0:NUM_BLOBS-1];
    reg [9:0]  obs_w     [0:NUM_BLOBS-1];
    reg [9:0]  obs_h     [0:NUM_BLOBS-1];
    reg [23:0] obs_area  [0:NUM_BLOBS-1];
    reg [CTX_W-1:0] ctx_q [0:NUM_BLOBS-1];

    reg [NUM_BLOBS-1:0] alarm_lat;
    reg [NUM_BLOBS-1:0] alarm_r;

    integer i;

    // ------------------------------------------------------- engine feed mux
    wire             fsm_en   = (phase == PH_FEED);
    wire             fsm_load = (phase == PH_LOAD);
    wire [CTX_W-1:0] ctx_in   = ctx_q[slot];
    wire [CTX_W-1:0] ctx_out;
    wire             fsm_alarm;

    posture_fsm #(
        .FRAME_W         (FRAME_W),
        .FRAME_H         (FRAME_H),
        .MIN_REF_AREA    (MIN_REF_AREA),
        .MIN_REF_HEIGHT  (MIN_REF_HEIGHT),
        .MAX_REF_W       (MAX_REF_W),
        .MAX_REF_H       (MAX_REF_H),
        .MAX_REF_AREA    (MAX_REF_AREA),
        .SUNK_CY2        (SUNK_CY2),
        .REF_TTL         (REF_TTL),
        .COLLAPSE_FRAMES (COLLAPSE_FRAMES),
        .RECOVERY_WINDOW (RECOVERY_WINDOW),
        .COOLDOWN_FRAMES (COOLDOWN_FRAMES),
        .CTX_W           (CTX_W)
    ) u_fsm (
        .clk              (clk),
        .rst_n            (rst_n),
        .ctx_in           (ctx_in),
        .ctx_load         (fsm_load),
        .ctx_out          (ctx_out),
        .en               (fsm_en),
        .valid            (obs_valid[slot]),
        .frame            (obs_frame),
        .x                (obs_x[slot]),
        .y                (obs_y[slot]),
        .w                (obs_w[slot]),
        .h                (obs_h[slot]),
        .area             (obs_area[slot]),
        .confirm_required (confirm_required),
        .alarm            (fsm_alarm),
        .state            ()
    );

    // ----------------------------------------------------------- sequencer
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase     <= PH_IDLE;
            busy      <= 1'b0;
            slot      <= 2'd0;
            obs_frame <= 32'd0;
            alarm_lat <= {NUM_BLOBS{1'b0}};
            alarm_r   <= {NUM_BLOBS{1'b0}};
            for (i = 0; i < NUM_BLOBS; i = i + 1) begin
                // An all-zero bundle is the engine's own reset state:
                // ref_valid=0, last_tall_valid=0, pending=0, state=SEEK.
                ctx_q[i]    <= {CTX_W{1'b0}};
                obs_valid[i] <= 1'b0;
                obs_x[i]    <= 10'd0;
                obs_y[i]    <= 10'd0;
                obs_w[i]    <= 10'd0;
                obs_h[i]    <= 10'd0;
                obs_area[i] <= 24'd0;
            end
        end else begin
            if (en && !busy) begin
                // Latch the whole snapshot first: the sweep below reads the
                // latched copy, so the caller may drop the box ports at once.
                for (i = 0; i < NUM_BLOBS; i = i + 1) begin
                    obs_valid[i] <= valid[i];
                    obs_x[i]    <= bx[i*10 +: 10];
                    obs_y[i]    <= by[i*10 +: 10];
                    obs_w[i]    <= bw[i*10 +: 10];
                    obs_h[i]    <= bh[i*10 +: 10];
                    obs_area[i] <= barea[i*24 +: 24];
                end
                obs_frame <= frame;
                alarm_lat <= {NUM_BLOBS{1'b0}};
                alarm_r   <= {NUM_BLOBS{1'b0}};
                slot      <= 2'd0;
                phase     <= PH_LOAD;
                busy      <= 1'b1;
            end else if (busy) begin
                case (phase)
                    PH_LOAD: phase <= PH_FEED;
                    PH_FEED: phase <= PH_UPD;
                    PH_UPD:  phase <= PH_CAP;
                    PH_CAP: begin
                        ctx_q[slot]     <= ctx_out;
                        alarm_lat[slot] <= fsm_alarm;
                        if (slot == NUM_BLOBS - 1) begin
                            phase <= PH_DONE;
                        end else begin
                            slot  <= slot + 2'd1;
                            phase <= PH_LOAD;
                        end
                    end
                    PH_DONE: begin
                        // All four alarm pulses are issued together, so the
                        // waveform matches four parallel machines.
                        alarm_r <= alarm_lat;
                        busy    <= 1'b0;
                        phase   <= PH_IDLE;
                    end
                    default: phase <= PH_IDLE;
                endcase
            end
        end
    end

    genvar g;
    generate
        for (g = 0; g < NUM_BLOBS; g = g + 1) begin : g_out
            // The low three bits of the bundle are the posture state, so the
            // published state comes straight out of the saved context.
            wire [CTX_W-1:0] ctx_g = ctx_q[g];
            assign state[g*3 +: 3] = ctx_g[2:0];
        end
    endgenerate

    assign alarm = alarm_r;

endmodule
