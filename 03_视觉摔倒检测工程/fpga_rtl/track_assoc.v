//===========================================================================
// track_assoc.v -- per-frame IoU association for the four crowd slots
//
// multi_bbox publishes its four boxes in area order.  That is cheap, but it
// makes the slot number change whenever two people swap strength or cross.
// This module keeps the four slot numbers stable across frames:
//
//   1. latch this frame's four boxes and keep last frame's tracked boxes,
//   2. build the 4x4 IoU matrix with exact integer cross products,
//   3. greedily pick the strongest remaining pair,
//   4. publish the current boxes in the old track order,
//   5. give any unmatched new box the lowest free slot.
//
// The implementation is deliberately serial: a frame has ~1.6 M clocks at
// 24/48 MHz and this association takes at most 4 x 4 x 6 = 384 clocks.  One
// 18x18 multiplier is time-shared for the cross products, so the area cost
// stays close to the four box registers and the small finite-state machine.
//
// Threshold: a pair needs inter != 0 and 4*inter >= union (IoU >= 0.25).
// The greedy scan is i-major then j-minor, so an exact tie keeps the earlier
// pair.  track_model.py mirrors this exact ordering and arithmetic.
//
// Lock-on hold, second generation.  The first version glued every blob to
// the screen for 300 frames, which turned one-frame flicker (the wall lamp
// in the fall clip) into a ten-second ghost and left three stale boxes on a
// single person.  The board captures showed a familiar-looking display that
// was mostly frozen leftovers, so the policy is now:
//
//   * est: a slot earns the right to be held only after three consecutive
//     matches or one person-sized detection (AREA_EST).  A flicker blob
//     never qualifies and dies on its first missed frame.
//   * HOLD_FRAMES = 120 (~4 s at 30 fps): long enough to cover the largest
//     measurement gap in the test material (117 frames), short enough that a
//     stale box cannot survive on screen for ten seconds.
//   * retire: while a slot is being held, if a matched box of the same frame
//     sits within CLAIM_QUIET/CLAIM_NEAR of it, the held box is a leftover
//     pose of that same person -- drop it instead of publishing a duplicate.
//   * claim/evict: an unmatched detection adopts the lowest-index stale slot
//     inside that radius; failing that it takes a free slot; failing that it
//     evicts the stale slot with the least hold left.  Only a frame with
//     four live tracks drops a detection.
//
// Lock-on hold, third generation (2026-10-07).  The board showed boxes that
// blinked on and off a person who was clearly still there: a slot only
// earned the right to be held after three consecutive matches, so a single
// dropped frame in the first three of a track killed it outright (measured
// on crowd07: 81 tracks in 500 frames, median lifetime 1 frame, 95% of the
// tracks three frames or shorter).  Two changes:
//
//   * GRACE_FRAMES: a track that has not yet earned a full hold still
//     survives GRACE_FRAMES missed frames.  Same counter, different reload
//     value -- no new state.  Median lifetime went 1 -> 7 frames and the
//     number of published boxes 526 -> 798 over the same 500 frames.
//   * elastic lock box: the published rectangle is no longer the raw
//     detection.  It always grows instantly to cover the detection and
//     relaxes SHRINK px per frame toward it, so a limb-sized blob settles
//     into a body-sized box that follows the person.  Four registers per
//     slot; the association arithmetic is untouched.
//
// The radius is 80 px in a quiet scene (two detections or fewer) and 40 px
// in a crowd, so two neighbours in dense footage are not glued together.
//===========================================================================
`timescale 1ns / 1ps

module track_assoc #(
    parameter integer NUM_BLOBS = 4,
    parameter integer XW        = 9,
    parameter integer AREAW     = 17,
    parameter integer HOLD_FRAMES = 120,
    parameter integer EST_MATCHES = 3,
    parameter integer AREA_EST    = 700,
    parameter integer CLAIM_NEAR  = 40,
    parameter integer CLAIM_QUIET = 80,
    parameter integer GRACE_FRAMES = 6,
    parameter integer SHRINK       = 2,
    parameter integer LOCK_MARGIN  = 8
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // One pulse from multi_bbox.
    input  wire                    in_valid,
    input  wire [NUM_BLOBS-1:0]    in_found,
    input  wire [NUM_BLOBS*XW-1:0] in_x,
    input  wire [NUM_BLOBS*XW-1:0] in_y,
    input  wire [NUM_BLOBS*XW-1:0] in_w,
    input  wire [NUM_BLOBS*XW-1:0] in_h,
    input  wire [NUM_BLOBS*AREAW-1:0] in_area,

    // One pulse after the frames have been placed in stable slots.
    output reg                     out_valid,
    output reg  [NUM_BLOBS-1:0]    out_found,
    output reg  [NUM_BLOBS*XW-1:0] out_x,
    output reg  [NUM_BLOBS*XW-1:0] out_y,
    output reg  [NUM_BLOBS*XW-1:0] out_w,
    output reg  [NUM_BLOBS*XW-1:0] out_h,
    output reg  [NUM_BLOBS*AREAW-1:0] out_area,

    // Per-slot person-confidence percentage (1..99).  The four slots are
    // walked through one shared person_score ROM after the boxes settle.
    output reg  [NUM_BLOBS*7-1:0] out_score
);

    localparam [3:0] S_IDLE      = 4'd0,
                     S_SCAN      = 4'd1,
                     S_CAND      = 4'd2,
                     S_PROD_A    = 4'd3,
                     S_PROD_B    = 4'd4,
                     S_PROD_C    = 4'd5,
                     S_DECIDE    = 4'd6,
                     S_ADVANCE   = 4'd7,
                     S_ASSIGN    = 4'd8,
                     S_OUT_MATCH = 4'd9,
                     S_OUT_FILL  = 4'd10,
                     S_OUT_PULSE = 4'd11,
                     S_OUT_DONE  = 4'd12,
                     S_CAND_M    = 4'd13,   // register the intersection product
                     S_CAND_U    = 4'd14,   // register union, then test IoU
                     S_SCORE     = 4'd15;   // four-cycle score-table walk

    reg [3:0] state;

    // ------------------------------------------------------------- snapshots
    reg [XW-1:0]    cur_x [0:NUM_BLOBS-1];
    reg [XW-1:0]    cur_y [0:NUM_BLOBS-1];
    reg [XW-1:0]    cur_w [0:NUM_BLOBS-1];
    reg [XW-1:0]    cur_h [0:NUM_BLOBS-1];
    reg [AREAW-1:0] cur_area [0:NUM_BLOBS-1];
    reg [NUM_BLOBS-1:0] cur_found;

    reg             prev_valid [0:NUM_BLOBS-1];
    reg [XW-1:0]    prev_x [0:NUM_BLOBS-1];
    reg [XW-1:0]    prev_y [0:NUM_BLOBS-1];
    reg [XW-1:0]    prev_w [0:NUM_BLOBS-1];
    reg [XW-1:0]    prev_h [0:NUM_BLOBS-1];
    reg [AREAW-1:0] prev_area [0:NUM_BLOBS-1];

    // --------------------------------------------------------------- matching
    reg [NUM_BLOBS-1:0] cur_used;
    reg [NUM_BLOBS-1:0] prev_used;
    reg [1:0]           map_i [0:NUM_BLOBS-1];

    reg [1:0] scan_i;
    reg [1:0] scan_j;
    reg [1:0] round;
    reg       best_found;
    reg [1:0] best_i;
    reg [1:0] best_j;

    // 18 bits cover both 57600-area products and 115200 unions exactly.
    reg [17:0] best_inter;
    reg [17:0] best_union;

    reg [1:0]      cand_i;
    reg [1:0]      cand_j;
    reg [XW-1:0]   cand_cx;
    reg [XW-1:0]   cand_cy;
    reg [XW-1:0]   cand_cw;
    reg [XW-1:0]   cand_ch;
    reg [XW-1:0]   cand_px;
    reg [XW-1:0]   cand_py;
    reg [XW-1:0]   cand_pw;
    reg [XW-1:0]   cand_ph;
    // The candidate geometry is pipelined over three clocks (S_CAND ->
    // S_CAND_M -> S_CAND_U).  It used to be one 21 ns combinational run of
    // add -> min -> subtract -> 10x10 multiply -> add/sub -> compare, which
    // missed 48 MHz by 445 ps once the HDMI picture path filled the die.
    reg [XW:0]     cand_iw_r;
    reg [XW:0]     cand_ih_r;
    reg [17:0]     cand_carea_r;
    reg [17:0]     cand_parea_r;
    reg [17:0]     cand_inter_r;
    reg [17:0]     cand_union_r;

    // ------------------------------------------------------- score multiplier
    reg  [17:0] mul_x;
    reg  [17:0] mul_y;
    wire [35:0] mul_p = mul_x * mul_y;
    reg  [35:0] prod_a;
    reg  [35:0] prod_b;

    // ------------------------------------------------------------ output side
    reg [1:0]           fill_i;
    reg [1:0]           score_i;

    // Lock-on hold countdown, one per slot (`~4 s at 30 fps) plus the
    // evidence counters: run_cnt saturates at EST_MATCHES consecutive
    // matches and est remembers that the slot has earned a hold.
    localparam [15:0] HOLD_MAX = HOLD_FRAMES;
    localparam [15:0] GRACE_MAX = GRACE_FRAMES;
    reg [15:0] hold_cnt [0:NUM_BLOBS-1];
    reg [1:0]  run_cnt  [0:NUM_BLOBS-1];
    reg        est      [0:NUM_BLOBS-1];

    // Elastic lock box: the rectangle that actually gets published.
    reg          lock_valid [0:NUM_BLOBS-1];
    reg [XW-1:0] lock_x     [0:NUM_BLOBS-1];
    reg [XW-1:0] lock_y     [0:NUM_BLOBS-1];
    reg [XW-1:0] lock_w     [0:NUM_BLOBS-1];
    reg [XW-1:0] lock_h     [0:NUM_BLOBS-1];

    // Edge arithmetic runs one bit wider than XW: x+w can reach 320.
    localparam [XW:0] FRAME_W_C = 320;
    localparam [XW:0] FRAME_H_C = 180;
    localparam [XW:0] SHRINK_C  = SHRINK;
    // Hard cap on how far the lock box may trail the detection: without it a
    // fast walker inflates the box along its whole path (measured on the
    // board: 96x37 -> 234x113), which is the "box too big" complaint again.
    localparam [XW:0] MARGIN_C  = LOCK_MARGIN;
    reg [XW:0] ld_x0, ld_y0, ld_x1, ld_y1;   // detection edges
    reg [XW:0] lk_x0, lk_y0, lk_x1, lk_y1;   // lock edges
    reg        strong_m;

    // ------------------------------------------------ person-score walk
    // The features are exactly the integer buckets in tools/person_roi.py.
    // Only one multiplier is needed for the box-area fill test; the three
    // size buckets are plain compares.  The shared case ROM in person_score
    // is evaluated once for each settled slot, so this costs four clocks per
    // frame and no extra multiplier farm.
    wire [1:0]  score_evid = !out_found[score_i] ? 2'd0 :
                             (est[score_i] ? 2'd3 :
                              (run_cnt[score_i] >= 2'd2 ? 2'd2 : 2'd1));
    wire [8:0]  score_w = out_w[score_i*XW +: XW];
    wire [8:0]  score_h = out_h[score_i*XW +: XW];
    wire [16:0] score_area = out_area[score_i*AREAW +: AREAW];
    wire [9:0]  score_w2 = {1'b0, score_w} << 1;
    wire [9:0]  score_h2 = {1'b0, score_h} << 1;
    wire [1:0]  score_aspect = (score_h2 < {1'b0, score_w}) ? 2'd0 :
                               (score_h < score_w) ? 2'd1 :
                               ({1'b0, score_h} < score_w2) ? 2'd2 : 2'd3;
    wire [19:0] score_box_area = {2'b0, score_w} * {2'b0, score_h};
    wire [19:0] score_area4 = {3'b0, score_area} << 2;
    wire [19:0] score_area2 = {3'b0, score_area} << 1;
    wire [19:0] score_box3 = score_box_area +
                             (score_box_area << 1);
    wire [1:0]  score_fill = (score_area4 < score_box_area) ? 2'd0 :
                             (score_area2 < score_box_area) ? 2'd1 :
                             (score_area4 < score_box3) ? 2'd2 : 2'd3;
    wire [1:0]  score_size = (score_area < 17'd160) ? 2'd0 :
                             (score_area < 17'd400) ? 2'd1 :
                             (score_area < 17'd900) ? 2'd2 : 2'd3;
    wire [6:0]  score_now;

    person_score u_person_score (
        .evid   (score_evid),
        .aspect (score_aspect),
        .fill   (score_fill),
        .size   (score_size),
        .score  (score_now)
    );

    // Centres of the published boxes and of this frame's detections, so the
    // proximity rules are plain 10-bit comparisons.
    wire [9:0] out_cx [0:NUM_BLOBS-1];
    wire [9:0] out_cy [0:NUM_BLOBS-1];
    wire [9:0] cur_cx [0:NUM_BLOBS-1];
    wire [9:0] cur_cy [0:NUM_BLOBS-1];
    genvar gc;
    generate
        for (gc = 0; gc < NUM_BLOBS; gc = gc + 1) begin : g_centre
            wire [XW-1:0] ow = out_w[gc*XW +: XW];
            wire [XW-1:0] oh = out_h[gc*XW +: XW];
            wire [XW-1:0] cw = cur_w[gc];
            wire [XW-1:0] ch = cur_h[gc];
            assign out_cx[gc] = {1'b0, out_x[gc*XW +: XW]} +
                                {2'b0, ow[XW-1:1]};
            assign out_cy[gc] = {1'b0, out_y[gc*XW +: XW]} +
                                {2'b0, oh[XW-1:1]};
            assign cur_cx[gc] = {1'b0, cur_x[gc]} +
                                {2'b0, cw[XW-1:1]};
            assign cur_cy[gc] = {1'b0, cur_y[gc]} +
                                {2'b0, ch[XW-1:1]};
        end
    endgenerate

    // Part-selects of an integer parameter are awkward for some tools, so the
    // radii get their own 10-bit constants.
    localparam [9:0] CLAIM_Q = CLAIM_QUIET;
    localparam [9:0] CLAIM_N = CLAIM_NEAR;

    wire [2:0] n_cur = {2'b0, cur_found[0]} + {2'b0, cur_found[1]} +
                       {2'b0, cur_found[2]} + {2'b0, cur_found[3]};
    wire [9:0] claim_r = (n_cur <= 3'd2) ? CLAIM_Q : CLAIM_N;

    // Is detection ``d`` centre-close to published slot ``s``?
    function near_pub_cur;
        input [1:0] s;
        input [1:0] d;
        reg [9:0] dx;
        reg [9:0] dy;
        begin
            dx = (out_cx[s] > cur_cx[d]) ? (out_cx[s] - cur_cx[d])
                                         : (cur_cx[d] - out_cx[s]);
            dy = (out_cy[s] > cur_cy[d]) ? (out_cy[s] - cur_cy[d])
                                         : (cur_cy[d] - out_cy[s]);
            near_pub_cur = (dx <= claim_r) && (dy <= claim_r);
        end
    endfunction

    // Stale slot ``s`` while a matched box of this frame sits next to it:
    // that is the same person in two poses, not two people.
    function retire_hit;
        input [1:0] s;
        integer kk;     // must be wide: a 2-bit index wraps 4 -> 0 forever
        reg [9:0] dx;
        reg [9:0] dy;
        reg       hit;
        begin
            hit = 1'b0;
            for (kk = 0; kk < NUM_BLOBS; kk = kk + 1) begin
                if (prev_used[kk]) begin
                    dx = (cur_cx[map_i[kk]] > out_cx[s])
                         ? (cur_cx[map_i[kk]] - out_cx[s])
                         : (out_cx[s] - cur_cx[map_i[kk]]);
                    dy = (cur_cy[map_i[kk]] > out_cy[s])
                         ? (cur_cy[map_i[kk]] - out_cy[s])
                         : (out_cy[s] - cur_cy[map_i[kk]]);
                    if ((dx <= claim_r) && (dy <= claim_r))
                        hit = 1'b1;
                end
            end
            retire_hit = hit;
        end
    endfunction

    // Lowest-index stale slot near detection ``fi``: {hit, index}.
    function [2:0] claim_sel;
        input [1:0] fi;
        integer ss;     // must be wide: a 2-bit index wraps 4 -> 0 forever
        reg       hit;
        reg [1:0] sel;
        begin
            hit = 1'b0;
            sel = 2'd0;
            for (ss = 0; ss < NUM_BLOBS; ss = ss + 1) begin
                if (!hit && !prev_used[ss] && out_found[ss] &&
                    near_pub_cur(ss, fi)) begin
                    hit = 1'b1;
                    sel = ss;
                end
            end
            claim_sel = {hit, sel};
        end
    endfunction

    // Lowest free slot: {hit, index}.  The argument is unused; Verilog
    // requires functions to have at least one input port.
    function [2:0] free_sel;
        input [1:0] fi;
        integer ss;     // must be wide: a 2-bit index wraps 4 -> 0 forever
        reg       hit;
        reg [1:0] sel;
        begin
            hit = 1'b0;
            sel = 2'd0;
            for (ss = 0; ss < NUM_BLOBS; ss = ss + 1) begin
                if (!hit && !out_found[ss]) begin
                    hit = 1'b1;
                    sel = ss;
                end
            end
            free_sel = {hit, sel};
        end
    endfunction

    // Stale slot with the least hold left: {hit, index}.  A later stale slot
    // wins only on a strictly smaller count, so ties keep the lower index --
    // the same tie-break the Python model uses.
    function [2:0] evict_sel;
        input [1:0] fi;
        integer    ss;  // must be wide: a 2-bit index wraps 4 -> 0 forever
        reg        hit;
        reg [1:0]  sel;
        reg [15:0] best;
        begin
            hit  = 1'b0;
            sel  = 2'd0;
            best = 16'hFFFF;
            for (ss = 0; ss < NUM_BLOBS; ss = ss + 1) begin
                if (out_found[ss] && !prev_used[ss] &&
                    (!hit || (hold_cnt[ss] < best))) begin
                    hit  = 1'b1;
                    best = hold_cnt[ss];
                    sel  = ss;
                end
            end
            evict_sel = {hit, sel};
        end
    endfunction

    wire [2:0] fill_claim = claim_sel(fill_i);
    wire [2:0] fill_free  = free_sel(fill_i);
    wire [2:0] fill_evict = evict_sel(fill_i);
    wire [2:0] fill_tgt   = fill_claim[2] ? fill_claim :
                            (fill_free[2]  ? fill_free  : fill_evict);

    integer i;

    // ------------------------------------------------------- scan arithmetic
    wire [XW-1:0] scan_cx = cur_x[scan_i];
    wire [XW-1:0] scan_cy = cur_y[scan_i];
    wire [XW-1:0] scan_cw = cur_w[scan_i];
    wire [XW-1:0] scan_ch = cur_h[scan_i];
    wire [XW-1:0] scan_px = prev_x[scan_j];
    wire [XW-1:0] scan_py = prev_y[scan_j];
    wire [XW-1:0] scan_pw = prev_w[scan_j];
    wire [XW-1:0] scan_ph = prev_h[scan_j];

    wire        scan_pair_ok = cur_found[scan_i] &&
                               prev_valid[scan_j] &&
                               !cur_used[scan_i] &&
                               !prev_used[scan_j];

    // ------------------------------------------------ candidate arithmetic
    wire [XW:0] cand_cx1 = {1'b0, cand_cx} + {1'b0, cand_cw};
    wire [XW:0] cand_cy1 = {1'b0, cand_cy} + {1'b0, cand_ch};
    wire [XW:0] cand_px1 = {1'b0, cand_px} + {1'b0, cand_pw};
    wire [XW:0] cand_py1 = {1'b0, cand_py} + {1'b0, cand_ph};
    wire [XW:0] cand_ix0 = (cand_cx > cand_px) ? cand_cx : cand_px;
    wire [XW:0] cand_iy0 = (cand_cy > cand_py) ? cand_cy : cand_py;
    wire [XW:0] cand_ix1 = (cand_cx1 < cand_px1) ? cand_cx1 : cand_px1;
    wire [XW:0] cand_iy1 = (cand_cy1 < cand_py1) ? cand_cy1 : cand_py1;
    wire [XW:0] cand_iw = (cand_ix1 > cand_ix0) ? (cand_ix1 - cand_ix0)
                                                  : {XW+1{1'b0}};
    wire [XW:0] cand_ih = (cand_iy1 > cand_iy0) ? (cand_iy1 - cand_iy0)
                                                  : {XW+1{1'b0}};
    // Use the box area w*h, not the motion-pixel area, for IoU: the latter
    // can be smaller than the geometric intersection of two boxes.
    wire [17:0] cand_carea = cand_cw * cand_ch;
    wire [17:0] cand_parea = cand_pw * cand_ph;

    // ---- stage 2 / stage 3 arithmetic on the registered geometry ----
    wire [17:0] cand_inter_q  = cand_iw_r * cand_ih_r;
    wire [17:0] cand_union_q  = cand_carea_r + cand_parea_r - cand_inter_q;
    wire [19:0] cand_inter4_q = {2'b0, cand_inter_q} << 2;
    wire        cand_ok_q     = (cand_inter_q != 18'd0) &&
                                (cand_inter4_q >= {2'b0, cand_union_q});

    wire scan_last = (scan_i == NUM_BLOBS - 1) &&
                     (scan_j == NUM_BLOBS - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            out_valid    <= 1'b0;
            out_found    <= {NUM_BLOBS{1'b0}};
            out_x        <= {(NUM_BLOBS*XW){1'b0}};
            out_y        <= {(NUM_BLOBS*XW){1'b0}};
            out_w        <= {(NUM_BLOBS*XW){1'b0}};
            out_h        <= {(NUM_BLOBS*XW){1'b0}};
            out_area     <= {(NUM_BLOBS*AREAW){1'b0}};
            out_score    <= {(NUM_BLOBS*7){1'b0}};
            cur_found    <= {NUM_BLOBS{1'b0}};
            cur_used     <= {NUM_BLOBS{1'b0}};
            prev_used    <= {NUM_BLOBS{1'b0}};
            scan_i       <= 2'd0;
            scan_j       <= 2'd0;
            round        <= 2'd0;
            fill_i       <= 2'd0;
            score_i      <= 2'd0;
            best_found   <= 1'b0;
            best_i       <= 2'd0;
            best_j       <= 2'd0;
            best_inter   <= 18'd0;
            best_union   <= 18'd0;
            cand_i       <= 2'd0;
            cand_j       <= 2'd0;
            cand_iw_r    <= {(XW+1){1'b0}};
            cand_ih_r    <= {(XW+1){1'b0}};
            cand_carea_r <= 18'd0;
            cand_parea_r <= 18'd0;
            cand_inter_r <= 18'd0;
            cand_union_r <= 18'd0;
            mul_x        <= 18'd0;
            mul_y        <= 18'd0;
            prod_a       <= 36'd0;
            prod_b       <= 36'd0;
            for (i = 0; i < NUM_BLOBS; i = i + 1) begin
                cur_x[i]    <= {XW{1'b0}};
                cur_y[i]    <= {XW{1'b0}};
                cur_w[i]    <= {XW{1'b0}};
                cur_h[i]    <= {XW{1'b0}};
                cur_area[i] <= {AREAW{1'b0}};
                prev_valid[i] <= 1'b0;
                prev_x[i]    <= {XW{1'b0}};
                prev_y[i]    <= {XW{1'b0}};
                prev_w[i]    <= {XW{1'b0}};
                prev_h[i]    <= {XW{1'b0}};
                prev_area[i] <= {AREAW{1'b0}};
                hold_cnt[i]  <= 16'd0;
                run_cnt[i]   <= 2'd0;
                est[i]       <= 1'b0;
                lock_valid[i] <= 1'b0;
                lock_x[i]    <= {XW{1'b0}};
                lock_y[i]    <= {XW{1'b0}};
                lock_w[i]    <= {XW{1'b0}};
                lock_h[i]    <= {XW{1'b0}};
                map_i[i]     <= 2'd0;
            end
        end else begin
            case (state)
                S_IDLE: begin
                    out_valid <= 1'b0;
                    if (in_valid) begin
                        cur_found <= in_found;
                        for (i = 0; i < NUM_BLOBS; i = i + 1) begin
                            cur_x[i]    <= in_x[i*XW +: XW];
                            cur_y[i]    <= in_y[i*XW +: XW];
                            cur_w[i]    <= in_w[i*XW +: XW];
                            cur_h[i]    <= in_h[i*XW +: XW];
                            cur_area[i] <= in_area[i*AREAW +: AREAW];
                            map_i[i]    <= 2'd0;
                        end
                        cur_used   <= {NUM_BLOBS{1'b0}};
                        prev_used  <= {NUM_BLOBS{1'b0}};
                        scan_i     <= 2'd0;
                        scan_j     <= 2'd0;
                        round      <= 2'd0;
                        best_found <= 1'b0;
                        best_inter <= 18'd0;
                        best_union <= 18'd0;
                        state      <= S_SCAN;
                    end
                end

                S_SCAN: begin
                    if (scan_pair_ok) begin
                        cand_i     <= scan_i;
                        cand_j     <= scan_j;
                        cand_cx    <= scan_cx;
                        cand_cy    <= scan_cy;
                        cand_cw    <= scan_cw;
                        cand_ch    <= scan_ch;
                        cand_px    <= scan_px;
                        cand_py    <= scan_py;
                        cand_pw    <= scan_pw;
                        cand_ph    <= scan_ph;
                        state      <= S_CAND;
                    end else begin
                        state <= S_ADVANCE;
                    end
                end

                S_CAND: begin
                    cand_iw_r    <= cand_iw;
                    cand_ih_r    <= cand_ih;
                    cand_carea_r <= cand_carea;
                    cand_parea_r <= cand_parea;
                    state        <= S_CAND_M;
                end

                S_CAND_M: begin
                    cand_inter_r <= cand_inter_q;
                    state        <= S_CAND_U;
                end

                S_CAND_U: begin
                    cand_union_r <= cand_union_q;
                    if (cand_ok_q) begin
                        state <= S_PROD_A;
                    end else begin
                        state <= S_ADVANCE;
                    end
                end

                S_PROD_A: begin
                    // cross product: candidate_inter * best_union
                    mul_x <= cand_inter_r;
                    mul_y <= best_union;
                    state <= S_PROD_B;
                end

                S_PROD_B: begin
                    prod_a <= mul_p;
                    // cross product: best_inter * candidate_union
                    mul_x <= best_inter;
                    mul_y <= cand_union_r;
                    state <= S_PROD_C;
                end

                S_PROD_C: begin
                    prod_b <= mul_p;
                    state  <= S_DECIDE;
                end

                S_DECIDE: begin
                    if (!best_found || (prod_a > prod_b)) begin
                        best_found <= 1'b1;
                        best_i     <= cand_i;
                        best_j     <= cand_j;
                        best_inter <= cand_inter_r;
                        best_union <= cand_union_r;
                    end
                    state <= S_ADVANCE;
                end

                S_ADVANCE: begin
                    if (scan_last) begin
                        if (best_found)
                            state <= S_ASSIGN;
                        else
                            state <= S_OUT_MATCH;
                    end else begin
                        if (scan_j == NUM_BLOBS - 1) begin
                            scan_i <= scan_i + 2'd1;
                            scan_j <= 2'd0;
                        end else begin
                            scan_j <= scan_j + 2'd1;
                        end
                        state <= S_SCAN;
                    end
                end

                S_ASSIGN: begin
                    cur_used[best_i]  <= 1'b1;
                    prev_used[best_j] <= 1'b1;
                    map_i[best_j]     <= best_i;
                    if (round == NUM_BLOBS - 1) begin
                        state <= S_OUT_MATCH;
                    end else begin
                        round      <= round + 2'd1;
                        scan_i     <= 2'd0;
                        scan_j     <= 2'd0;
                        best_found <= 1'b0;
                        best_inter <= 18'd0;
                        best_union <= 18'd0;
                        state      <= S_SCAN;
                    end
                end

                S_OUT_MATCH: begin
                    // Matched boxes keep their old slot number and reload the
                    // hold.  A missed slot may re-publish its last box only
                    // when it has earned est, and only while no matched box
                    // of this frame sits next to it (otherwise the held box
                    // is an old pose of that same person and retires).
                    out_found <= {NUM_BLOBS{1'b0}};
                    out_x     <= {(NUM_BLOBS*XW){1'b0}};
                    out_y     <= {(NUM_BLOBS*XW){1'b0}};
                    out_w     <= {(NUM_BLOBS*XW){1'b0}};
                    out_h     <= {(NUM_BLOBS*XW){1'b0}};
                    out_area  <= {(NUM_BLOBS*AREAW){1'b0}};
                    out_score <= {(NUM_BLOBS*7){1'b0}};
                    // prev_used stays the match marker for the retire test;
                    // S_OUT_FILL re-uses it to mark stale slots it consumes.
                    for (i = 0; i < NUM_BLOBS; i = i + 1) begin
                        if (prev_used[i]) begin
                            // Elastic lock box: grow to cover this detection,
                            // then pull every untouched side in by SHRINK.
                            ld_x0 = {1'b0, cur_x[map_i[i]]};
                            ld_y0 = {1'b0, cur_y[map_i[i]]};
                            ld_x1 = ld_x0 + {1'b0, cur_w[map_i[i]]};
                            ld_y1 = ld_y0 + {1'b0, cur_h[map_i[i]]};
                            if (lock_valid[i]) begin
                                lk_x0 = {1'b0, lock_x[i]};
                                lk_y0 = {1'b0, lock_y[i]};
                                lk_x1 = lk_x0 + {1'b0, lock_w[i]};
                                lk_y1 = lk_y0 + {1'b0, lock_h[i]};
                            end else begin
                                lk_x0 = ld_x0;
                                lk_y0 = ld_y0;
                                lk_x1 = ld_x1;
                                lk_y1 = ld_y1;
                            end
                            if (ld_x0 < lk_x0) lk_x0 = ld_x0;
                            if (ld_y0 < lk_y0) lk_y0 = ld_y0;
                            if (ld_x1 > lk_x1) lk_x1 = ld_x1;
                            if (ld_y1 > lk_y1) lk_y1 = ld_y1;
                            lk_x0 = (lk_x0 + SHRINK_C >= ld_x0) ? ld_x0
                                                                : lk_x0 + SHRINK_C;
                            lk_y0 = (lk_y0 + SHRINK_C >= ld_y0) ? ld_y0
                                                                : lk_y0 + SHRINK_C;
                            lk_x1 = (lk_x1 <= ld_x1 + SHRINK_C) ? ld_x1
                                                                : lk_x1 - SHRINK_C;
                            lk_y1 = (lk_y1 <= ld_y1 + SHRINK_C) ? ld_y1
                                                                : lk_y1 - SHRINK_C;
                            // ...but never trail the detection by more than
                            // LOCK_MARGIN px on any side (see MARGIN_C).
                            if (lk_x0 + MARGIN_C < ld_x0)
                                lk_x0 = ld_x0 - MARGIN_C;
                            if (lk_y0 + MARGIN_C < ld_y0)
                                lk_y0 = ld_y0 - MARGIN_C;
                            if (lk_x1 > ld_x1 + MARGIN_C)
                                lk_x1 = ld_x1 + MARGIN_C;
                            if (lk_y1 > ld_y1 + MARGIN_C)
                                lk_y1 = ld_y1 + MARGIN_C;
                            if (lk_x1 > FRAME_W_C) lk_x1 = FRAME_W_C;
                            if (lk_y1 > FRAME_H_C) lk_y1 = FRAME_H_C;

                            lock_valid[i] <= 1'b1;
                            lock_x[i] <= lk_x0[XW-1:0];
                            lock_y[i] <= lk_y0[XW-1:0];
                            lock_w[i] <= lk_x1[XW-1:0] - lk_x0[XW-1:0];
                            lock_h[i] <= lk_y1[XW-1:0] - lk_y0[XW-1:0];

                            out_found[i]     <= 1'b1;
                            out_x[i*XW +: XW] <= lk_x0[XW-1:0];
                            out_y[i*XW +: XW] <= lk_y0[XW-1:0];
                            out_w[i*XW +: XW] <= lk_x1[XW-1:0] -
                                                 lk_x0[XW-1:0];
                            out_h[i*XW +: XW] <= lk_y1[XW-1:0] -
                                                 lk_y0[XW-1:0];
                            out_area[i*AREAW +: AREAW] <=
                                cur_area[map_i[i]];
                            strong_m = (cur_area[map_i[i]] >= AREA_EST) ||
                                       (run_cnt[i] >= (EST_MATCHES - 1));
                            if (strong_m)
                                est[i] <= 1'b1;
                            hold_cnt[i] <= (est[i] || strong_m) ? HOLD_MAX
                                                                : GRACE_MAX;
                            if (run_cnt[i] != 2'd3)
                                run_cnt[i] <= run_cnt[i] + 2'd1;
                        end else if (prev_valid[i] &&
                                     hold_cnt[i] != 16'd0) begin
                            if (retire_hit(i)) begin
                                // Duplicate of a live track: free the slot.
                                hold_cnt[i] <= 16'd0;
                                run_cnt[i]  <= 2'd0;
                                est[i]      <= 1'b0;
                                lock_valid[i] <= 1'b0;
                            end else begin
                                // Keep publishing the locked box, count down.
                                out_found[i]     <= 1'b1;
                                out_x[i*XW +: XW] <= lock_valid[i] ? lock_x[i]
                                                                   : prev_x[i];
                                out_y[i*XW +: XW] <= lock_valid[i] ? lock_y[i]
                                                                   : prev_y[i];
                                out_w[i*XW +: XW] <= lock_valid[i] ? lock_w[i]
                                                                   : prev_w[i];
                                out_h[i*XW +: XW] <= lock_valid[i] ? lock_h[i]
                                                                   : prev_h[i];
                                out_area[i*AREAW +: AREAW] <= prev_area[i];
                                hold_cnt[i] <= hold_cnt[i] - 16'd1;
                            end
                        end else begin
                            hold_cnt[i] <= 16'd0;
                            run_cnt[i]  <= 2'd0;
                            est[i]      <= 1'b0;
                            lock_valid[i] <= 1'b0;
                        end
                    end
                    fill_i   <= 2'd0;
                    state    <= S_OUT_FILL;
                end

                S_OUT_FILL: begin
                    // Unmatched detections, in the order multi_bbox published
                    // them: adopt the lowest-index stale slot of the same
                    // person when one is close, else take a free slot, else
                    // evict the stale slot with the least hold left.  Only a
                    // frame with four live tracks drops a detection.
                    if (cur_found[fill_i] && !cur_used[fill_i] &&
                        fill_tgt[2]) begin
                        out_found[fill_tgt[1:0]]     <= 1'b1;
                        out_x[fill_tgt[1:0]*XW +: XW] <= cur_x[fill_i];
                        out_y[fill_tgt[1:0]*XW +: XW] <= cur_y[fill_i];
                        out_w[fill_tgt[1:0]*XW +: XW] <= cur_w[fill_i];
                        out_h[fill_tgt[1:0]*XW +: XW] <= cur_h[fill_i];
                        out_area[fill_tgt[1:0]*AREAW +: AREAW]
                            <= cur_area[fill_i];
                        // The slot stops being stale for the rest of this
                        // pass; prev_used is idle here and serves as that mark.
                        prev_used[fill_tgt[1:0]] <= 1'b1;
                        // A fresh lock starts exactly on the detection.
                        lock_valid[fill_tgt[1:0]] <= 1'b1;
                        lock_x[fill_tgt[1:0]] <= cur_x[fill_i];
                        lock_y[fill_tgt[1:0]] <= cur_y[fill_i];
                        lock_w[fill_tgt[1:0]] <= cur_w[fill_i];
                        lock_h[fill_tgt[1:0]] <= cur_h[fill_i];
                        if (!fill_claim[2]) begin
                            // Fresh track, not a continuation: restart the
                            // evidence counters from this one detection.
                            run_cnt[fill_tgt[1:0]] <= 2'd1;
                            est[fill_tgt[1:0]]     <=
                                (cur_area[fill_i] >= AREA_EST);
                        end
                        // Same reload rule as a matched slot: an established
                        // track gets the full hold, a new one the grace.
                        strong_m = fill_claim[2] ? est[fill_tgt[1:0]]
                                                 : (cur_area[fill_i] >= AREA_EST);
                        hold_cnt[fill_tgt[1:0]] <= strong_m ? HOLD_MAX
                                                            : GRACE_MAX;
                    end

                    if (fill_i == NUM_BLOBS - 1) begin
                        score_i <= 2'd0;
                        state   <= S_SCORE;
                    end
                    else
                        fill_i <= fill_i + 2'd1;
                end

                S_SCORE: begin
                    out_score[score_i*7 +: 7] <= out_found[score_i]
                                                 ? score_now : 7'd0;
                    if (score_i == NUM_BLOBS - 1)
                        state <= S_OUT_PULSE;
                    else
                        score_i <= score_i + 2'd1;
                end

                S_OUT_PULSE: begin
                    // Publish and make this frame's slots the next reference.
                    for (i = 0; i < NUM_BLOBS; i = i + 1) begin
                        prev_valid[i] <= out_found[i];
                        prev_x[i]     <= out_x[i*XW +: XW];
                        prev_y[i]     <= out_y[i*XW +: XW];
                        prev_w[i]     <= out_w[i*XW +: XW];
                        prev_h[i]     <= out_h[i*XW +: XW];
                        prev_area[i]  <= out_area[i*AREAW +: AREAW];
                    end
                    out_valid <= 1'b1;
                    state     <= S_OUT_DONE;
                end

                S_OUT_DONE: begin
                    out_valid <= 1'b0;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
