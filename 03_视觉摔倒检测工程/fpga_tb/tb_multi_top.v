//===========================================================================
// tb_multi_top.v -- end-to-end crowd path on RGB565 frames:
//
//   gray_front -> frame_diff -> motion_morph
//        -> multi_bbox (4 exclusion passes) -> 4 x posture_fsm
//
// This drives the real fall_core_multi_top module, not a re-wired copy of its
// stages, so the four-slot publish and the shared frame counter are part of
// what is under test.
//
// Input : 640x480 RGB565 little-endian frames, one de_in pixel per clock.
// Output: BOX lines in +OUT, four per frame in slot order (this is the same
//         bbox_check tuple order: found x y w h area), plus per-frame ROW
//         lines on stdout:
//
//             ROW <slot> <frame> <found> <state> <alarm>
//
//         and the matching rule-2 probes as DBG2 lines.  An alarm that is
//         high at the sampled edge also prints ALARM <slot> <frame>, which
//         is what the driver compares against the reference alarm list.
//
//         motion_hist's whole-person boxes print as PBOX lines, four per
//         publish.  The module publishes one frame behind the tracker, so
//         publish k holds the boxes of model frame k:
//
//             PBOX <publish> <slot> <found> <x> <y> <w> <h> <cells>
//
// The bench inserts a realistic vertical blanking (BLANK clocks) between
// frames so the write side can swap banks and the read side can run its four
// passes inside the next frame, then drains at the end so the last frame's
// passes can finish.
//
// plusargs:
//   +IN=<file>     RGB565 640x480 frames
//   +OUT=<file>    BOX lines
//   +FRAMES=<n>
//   +MORPH=<file>  optional packed motion_morph output (front-end debug)
//   +DIFF=<file>   optional packed frame_diff mask (front-end debug)
//   +BLANK=<n>     clocks between frames (default 24000)
//===========================================================================
`timescale 1ns / 1ps

module tb_multi_top;

localparam integer NB    = 4;
localparam integer SW    = 640;
localparam integer SH    = 480;
localparam integer NPIX  = SW * SH;
localparam integer BLANK = 24000;
localparam integer DRAIN = 400000;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg frame_start = 1'b0;
reg de_in = 1'b0;
reg [15:0] pix_in = 16'd0;

wire                    bbox_valid;
wire [NB-1:0]           bbox_found;
wire [NB*9-1:0]         bbox_x;
wire [NB*9-1:0]         bbox_y;
wire [NB*9-1:0]         bbox_w;
wire [NB*9-1:0]         bbox_h;
wire [NB*17-1:0]        bbox_area;
wire [NB*3-1:0]         posture_state;
wire [NB-1:0]           alarm;
wire                    bbox_busy;
wire                    pbox_valid;
wire [NB-1:0]           pbox_found;
wire [NB*9-1:0]         pbox_x, pbox_y, pbox_w, pbox_h;
wire [NB*13-1:0]        pbox_cells;
wire [NB-1:0]           pbox_dir;

fall_core_multi_top #(
    .NUM_BLOBS (NB),
    .MARGIN    (8)
) u_dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .frame_start  (frame_start),
    .de_in        (de_in),
    .pix_in       (pix_in),
    .bbox_valid   (bbox_valid),
    .bbox_found   (bbox_found),
    .bbox_x       (bbox_x),
    .bbox_y       (bbox_y),
    .bbox_w       (bbox_w),
        .bbox_h       (bbox_h),
        .bbox_area    (bbox_area),
        .pbox_valid   (pbox_valid),
        .pbox_found   (pbox_found),
        .pbox_x       (pbox_x),
        .pbox_y       (pbox_y),
        .pbox_w       (pbox_w),
        .pbox_h       (pbox_h),
        .pbox_cells   (pbox_cells),
        .pbox_dir     (pbox_dir),
        .posture_state(posture_state),
        .alarm        (alarm)
);

// Front-end taps for stage-by-stage diagnosis.  These are the same nets the
// solo bench dumps, referenced hierarchically so the DUT stays untouched.
wire de_morph   = u_dut.de_morph;
wire mask_morph = u_dut.mask_morph;
wire de_diff    = u_dut.de_diff;
wire mask_diff  = u_dut.mask_diff;

always #5 clk = ~clk;

reg [1023:0] in_path, out_path, morph_path, diff_path;
integer fd_in, fd_out, fd_morph, fd_diff;
integer nframes, f, n, got, k, blank_n;
reg [15:0] fbuf [0:NPIX-1];

reg dump_morph, dump_diff;
reg [7:0] mbyte, dbyte;
integer mcnt, dcnt;

integer res_count;
integer alarm_count;
integer x_diff, x_morph;
reg [31:0] frame_cnt;      // bbox publications so far
reg [31:0] pbox_cnt;       // motion_hist publications so far
reg [31:0] samp_frame;     // frame number of the pending observation
reg [5:0]  samp_wait;      // rule 1: wait for the posture bank to settle
reg [5:0]  samp2_wait;     // rule 2: one edge later (probe only)
reg        samp_valid, samp2_valid;

// The crowd path now runs its four posture contexts through one engine
// (posture_bank): the sweep needs 16 clocks and the alarm level is raised at
// clock 18 and held until the next frame.  Rule 1 therefore samples 25 edges
// after the bbox pulse, which is well after both the state write-back and the
// alarm raise; rule 2 reads one edge later and is printed as DBG2 so a
// mismatch can be told apart from a sampling error.
always @(posedge clk) begin
    if (pbox_valid) begin
        pbox_cnt <= pbox_cnt + 32'd1;
        for (k = 0; k < NB; k = k + 1)
            $display("PBOX %0d %0d %0d %0d %0d %0d %0d %0d",
                     pbox_cnt + 32'd1, k, pbox_found[k],
                     pbox_x[k*9 +: 9], pbox_y[k*9 +: 9],
                     pbox_w[k*9 +: 9], pbox_h[k*9 +: 9],
                     pbox_cells[k*13 +: 13]);
        for (k = 0; k < NB; k = k + 1)
            $display("PDIR %0d %0d %0d", pbox_cnt + 32'd1, k, pbox_dir[k]);
    end

    if (bbox_valid) begin
        for (k = 0; k < NB; k = k + 1)
            $fdisplay(fd_out, "%0d %0d %0d %0d %0d %0d",
                      bbox_found[k],
                      bbox_x[k*9 +: 9], bbox_y[k*9 +: 9],
                      bbox_w[k*9 +: 9], bbox_h[k*9 +: 9],
                      bbox_area[k*17 +: 17]);
        frame_cnt  <= frame_cnt + 32'd1;
        samp_frame <= frame_cnt + 32'd1;
        samp_valid <= 1'b1;
        samp_wait  <= 6'd24;
        samp2_valid <= 1'b1;
        samp2_wait  <= 6'd25;
        res_count  <= res_count + 1;
    end else begin
        if (samp_valid) begin
            if (samp_wait == 2'd0) begin
                for (k = 0; k < NB; k = k + 1) begin
                    $display("ROW %0d %0d %0d %0d %0d",
                             k, samp_frame, bbox_found[k],
                             posture_state[k*3 +: 3], alarm[k]);
                    if (alarm[k]) begin
                        $display("ALARM %0d %0d", k, samp_frame);
                        alarm_count <= alarm_count + 1;
                    end
                end
                samp_valid <= 1'b0;
            end else begin
                samp_wait <= samp_wait - 2'd1;
            end
        end
        if (samp2_valid) begin
            if (samp2_wait == 2'd0) begin
                for (k = 0; k < NB; k = k + 1)
                    $display("DBG2 %0d %0d %0d %0d %0d",
                             k, samp_frame, bbox_found[k],
                             posture_state[k*3 +: 3], alarm[k]);
                samp2_valid <= 1'b0;
            end else begin
                samp2_wait <= samp2_wait - 2'd1;
            end
        end
    end

    if (dump_morph && de_morph) begin
        mbyte = {mbyte[6:0], mask_morph};
        if (mcnt == 7) begin
            $fwrite(fd_morph, "%c", mbyte);
            mcnt = 0;
            mbyte = 8'd0;
        end else begin
            mcnt = mcnt + 1;
        end
    end
    if (dump_diff && de_diff) begin
        dbyte = {dbyte[6:0], mask_diff};
        if (dcnt == 7) begin
            $fwrite(fd_diff, "%c", dbyte);
            dcnt = 0;
            dbyte = 8'd0;
        end else begin
            dcnt = dcnt + 1;
        end
    end
    if (de_diff  && (mask_diff  === 1'bx)) x_diff  = x_diff + 1;
    if (de_morph && (mask_morph === 1'bx)) x_morph = x_morph + 1;
end

initial begin
    if (!$value$plusargs("IN=%s", in_path) ||
        !$value$plusargs("OUT=%s", out_path) ||
        !$value$plusargs("FRAMES=%d", nframes)) begin
        $display("ERROR missing plusarg (need IN, OUT, FRAMES)");
        $finish;
    end

    fd_in = $fopen(in_path, "rb");
    if (fd_in == 0) begin
        $display("ERROR cannot open input");
        $finish;
    end
    fd_out = $fopen(out_path, "w");
    dump_morph = $value$plusargs("MORPH=%s", morph_path);
    if (dump_morph) fd_morph = $fopen(morph_path, "wb");
    dump_diff = $value$plusargs("DIFF=%s", diff_path);
    if (dump_diff) fd_diff = $fopen(diff_path, "wb");
    if (!$value$plusargs("BLANK=%d", blank_n)) blank_n = BLANK;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    res_count   = 0;
    alarm_count = 0;
    mbyte = 8'd0;
    mcnt  = 0;
    dbyte = 8'd0;
    dcnt  = 0;
    x_diff  = 0;
    x_morph = 0;
    frame_cnt   = 32'd0;
    pbox_cnt    = 32'd0;
    samp_valid  = 1'b0;
    samp2_valid = 1'b0;
    samp_wait   = 2'd0;
    samp2_wait  = 2'd0;

    for (f = 0; f < nframes; f = f + 1) begin
        got = $fread(fbuf, fd_in);
        if (got != NPIX * 2) begin
            $display("ERROR short read at frame %0d: got %0d bytes, want %0d",
                     f, got, NPIX * 2);
            $finish;
        end

        // One-cycle frame_start pulse between two negedges.
        @(negedge clk);
        frame_start = 1'b1;
        de_in = 1'b0;
        @(negedge clk);
        frame_start = 1'b0;

        for (n = 0; n < NPIX; n = n + 1) begin
            @(negedge clk);
            pix_in = {fbuf[n][7:0], fbuf[n][15:8]};
            de_in = 1'b1;
            @(posedge clk); #1;
        end
        de_in = 1'b0;
        $display("frame %0d streamed: frame_cnt=%0d x_diff=%0d x_morph=%0d",
                 f, frame_cnt, x_diff, x_morph);

        for (n = 0; n < blank_n; n = n + 1) begin
            @(negedge clk);
            @(posedge clk); #1;
        end
    end

    for (n = 0; n < DRAIN; n = n + 1) begin
        @(negedge clk);
        @(posedge clk); #1;
    end

    $fclose(fd_in);
    $fclose(fd_out);
    if (dump_morph) $fclose(fd_morph);
    if (dump_diff) $fclose(fd_diff);
    $display("DONE results=%0d alarms=%0d xdiff=%0d xmorph=%0d",
             res_count, alarm_count, x_diff, x_morph);
    $finish;
end

endmodule
