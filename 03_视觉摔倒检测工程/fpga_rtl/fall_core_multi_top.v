//===========================================================================
// fall_core_multi_top.v -- crowd path: one front end, four tracked slots
//
//   gray_front -> frame_diff -> motion_morph
//        -> multi_bbox (4 exclusion passes)
//        -> track_assoc (IoU re-slotting, identity first)
//        -> motion_hist (2-bit motion memory -> whole-person boxes)
//        -> posture_bank (one posture_fsm engine time-shared over 4 contexts)
//
// This is the airport-crowd variant of fall_core_top.v.  The solo top keeps
// its single proj_bbox because that bitstream is what the 45-frame / 14-track
// regressions pin down; this module is the upgrade path and is verified by
// multi_bbox_check.py plus its own board-level run.
//
// The four posture machines used to be four separate posture_fsm instances.
// That cost about 3,200 LUTs and pushed the board top to 76.9% utilisation,
// where P&R stopped closing (three seeds, each ending with one or two
// PHY-8023 shorts).  posture_bank runs the same engine over four saved state
// bundles instead, which brings the design back to the size of the solo top.
// The machines only step once per frame, so the serial sweep costs ~20 clocks
// out of the 1.6 M a frame has.
//
// Slot semantics: multi_bbox orders boxes by area within the frame.
// track_assoc then maps those boxes onto the previous frame's four tracks
// with an exact integer IoU greedy match.  Matched people keep their slot
// number; new detections fill the lowest free slot.
//
// All four machines advance on the same bbox_valid pulse and share
// frame_idx, so their TTL / cooldown windows stay in step.  Each machine
// confirms with confirm_required = 2: a single missed frame does not fire.
//===========================================================================
`timescale 1ns / 1ps

module fall_core_multi_top #(
    parameter integer NUM_BLOBS      = 4,
    parameter integer MARGIN         = 8,
    // Defaults mirror the board build (fall_camera_hdmi_multi_top), which is
    // also what fpga/tools/multi_model.py and multi_top_check.py model.
    parameter integer DIFF_THRESH    = 12,
    parameter integer BBOX_MIN_PIX   = 4,
    parameter integer BBOX_MIN_AREA  = 160
)(
    input  wire        clk,           // OV5640 PCLK domain
    input  wire        rst_n,
    input  wire        frame_start,   // one clk pulse per frame, in blanking
    input  wire        de_in,         // 640x480 RGB565 pixel valid
    input  wire [15:0] pix_in,        // {R[4:0], G[5:0], B[4:0]}

    // ---- per-slot bounding boxes, published together once per frame ----
    output wire                    bbox_valid,
    output wire [NUM_BLOBS-1:0]    bbox_found,
    output wire [NUM_BLOBS*9-1:0]  bbox_x,
    output wire [NUM_BLOBS*9-1:0]  bbox_y,
    output wire [NUM_BLOBS*9-1:0]  bbox_w,
    output wire [NUM_BLOBS*9-1:0]  bbox_h,
    output wire [NUM_BLOBS*17-1:0] bbox_area,
    output wire [NUM_BLOBS*7-1:0]  bbox_score,

    // ---- per-slot person boxes for the stick figure overlay ----
    // The tracker's box only covers the moving part; motion_hist grows it
    // into the whole person (see rtl/motion_hist.v).  Published once per
    // frame, a few thousand clocks after bbox_valid.
    output wire                    pbox_valid,
    output wire [NUM_BLOBS-1:0]    pbox_found,
    output wire [NUM_BLOBS*9-1:0]  pbox_x,
    output wire [NUM_BLOBS*9-1:0]  pbox_y,
    output wire [NUM_BLOBS*9-1:0]  pbox_w,
    output wire [NUM_BLOBS*9-1:0]  pbox_h,
    output wire [NUM_BLOBS*13-1:0] pbox_cells,
    output wire [NUM_BLOBS-1:0]    pbox_dir,
    output wire                    pbox_pass_hit,   // sticky: pass saw a cell
    output wire                    pbox_sweep_hit,  // sticky: sweep read a trail

    // ---- per-slot posture state and alarm pulses ----
    output wire [NUM_BLOBS*3-1:0]  posture_state,
    output wire [NUM_BLOBS-1:0]    alarm
);

    wire        de_gray, de_diff, de_morph;
    wire [7:0]  gray_out;
    wire        mask_diff, mask_morph;
    wire        bbox_busy;

    // Raw multi_bbox snapshot, before identity re-slotting.
    wire        raw_valid;
    wire [NUM_BLOBS-1:0]    raw_found;
    wire [NUM_BLOBS*9-1:0]  raw_x;
    wire [NUM_BLOBS*9-1:0]  raw_y;
    wire [NUM_BLOBS*9-1:0]  raw_w;
    wire [NUM_BLOBS*9-1:0]  raw_h;
    wire [NUM_BLOBS*17-1:0] raw_area;

    reg [31:0] frame_idx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)           frame_idx <= 32'd0;
        else if (frame_start) frame_idx <= frame_idx + 32'd1;
    end

    gray_front u_gray (
        .clk         (clk),
        .rst_n       (rst_n),
        .frame_start (frame_start),
        .de_in       (de_in),
        .pix_in      (pix_in),
        .de_out      (de_gray),
        .gray_out    (gray_out),
        .x_out       (),
        .y_out       ()
    );

    frame_diff #(
        .DIFF_THRESH (DIFF_THRESH)
    ) u_diff (
        .clk         (clk),
        .rst_n       (rst_n),
        .frame_start (frame_start),
        .de_in       (de_gray),
        .gray_in     (gray_out),
        .de_out      (de_diff),
        .mask_out    (mask_diff)
    );

    motion_morph u_morph (
        .clk      (clk),
        .rst_n    (rst_n),
        .de_in    (de_diff),
        .mask_in  (mask_diff),
        .de_out   (de_morph),
        .mask_out (mask_morph)
    );

    multi_bbox #(
        .WIDTH     (320),
        .HEIGHT    (180),
        .MIN_PIX   (BBOX_MIN_PIX),
        .MIN_AREA  (BBOX_MIN_AREA),
        .NUM_BLOBS (NUM_BLOBS),
        .MARGIN    (MARGIN)
    ) u_bbox (
        .clk       (clk),
        .rst_n     (rst_n),
        .de_in     (de_morph),
        .mask_in   (mask_morph),
        .busy      (bbox_busy),
        .out_valid (raw_valid),
        .out_found (raw_found),
        .out_x     (raw_x),
        .out_y     (raw_y),
        .out_w     (raw_w),
        .out_h     (raw_h),
        .out_area  (raw_area)
    );

    // The tracker keeps the box vectors in the same 9-bit/17-bit convention.
    // A matched person is published in the same slot as the previous frame;
    // unmatched detections take the lowest free slot.  This is the point
    // where a slot number becomes a track number.
    track_assoc #(
        .NUM_BLOBS (NUM_BLOBS),
        .XW        (9),
        .AREAW     (17)
    ) u_track (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (raw_valid),
        .in_found   (raw_found),
        .in_x       (raw_x),
        .in_y       (raw_y),
        .in_w       (raw_w),
        .in_h       (raw_h),
        .in_area    (raw_area),
        .out_valid  (bbox_valid),
        .out_found  (bbox_found),
        .out_x      (bbox_x),
        .out_y      (bbox_y),
        .out_w      (bbox_w),
        .out_h      (bbox_h),
        .out_area   (bbox_area),
        .out_score  (bbox_score)
    );

    // The person box: the bounding box of the live cells of the 80x45
    // motion memory inside a window grown around the tracked box, united
    // with the tracked box itself.  RTL twin of tools/hist_model.py.
    motion_hist #(
        .NSLOT        (NUM_BLOBS)
    ) u_hist (
        .clk        (clk),
        .rst_n      (rst_n),
        .de_in      (de_morph),
        .mask_in    (mask_morph),
        .trk_valid  (bbox_valid),
        .trk_found  (bbox_found),
        .trk_x      (bbox_x),
        .trk_y      (bbox_y),
        .trk_w      (bbox_w),
        .trk_h      (bbox_h),
        .out_valid  (pbox_valid),
        .out_found  (pbox_found),
        .out_x      (pbox_x),
        .out_y      (pbox_y),
        .out_w      (pbox_w),
        .out_h      (pbox_h),
        .out_cells  (pbox_cells),
        .out_dir    (pbox_dir),
        .dbg_pass_hit  (pbox_pass_hit),
        .dbg_sweep_hit (pbox_sweep_hit)
    );

    // The posture engine works on 10-bit names and a 24-bit area, so widen
    // after the tracker rather than inside either module.
    wire [NUM_BLOBS*10-1:0] bank_x, bank_y, bank_w, bank_h;
    wire [NUM_BLOBS*24-1:0] bank_area;

    genvar g;
    generate
        for (g = 0; g < NUM_BLOBS; g = g + 1) begin : g_pad
            assign bank_x[g*10 +: 10] = {1'b0, bbox_x[g*9 +: 9]};
            assign bank_y[g*10 +: 10] = {1'b0, bbox_y[g*9 +: 9]};
            assign bank_w[g*10 +: 10] = {1'b0, bbox_w[g*9 +: 9]};
            assign bank_h[g*10 +: 10] = {1'b0, bbox_h[g*9 +: 9]};
            assign bank_area[g*24 +: 24] = {7'b0, bbox_area[g*17 +: 17]};
        end
    endgenerate

    posture_bank #(
        .NUM_BLOBS       (NUM_BLOBS),
        .FRAME_W         (320),
        .FRAME_H         (180),
        .MIN_REF_AREA    (1600),
        .MIN_REF_HEIGHT  (25),
        .MAX_REF_W       (176),
        .MAX_REF_H       (171),
        .MAX_REF_AREA    (11520),
        .SUNK_CY2        (15),
        .REF_TTL         (300),
        .COLLAPSE_FRAMES (36),
        .RECOVERY_WINDOW (2),
        .COOLDOWN_FRAMES (60)
    ) u_bank (
        .clk              (clk),
        .rst_n            (rst_n),
        .en               (bbox_valid),
        .frame            (frame_idx),
        .valid            (bbox_found),
        .bx               (bank_x),
        .by               (bank_y),
        .bw               (bank_w),
        .bh               (bank_h),
        .barea            (bank_area),
        .confirm_required (3'd2),
        .alarm            (alarm),
        .state            (posture_state)
    );

endmodule
