//===========================================================================
// tb_vision_chain.v -- end-to-end vision chain on RGB565 frames:
//
//   gray_front -> frame_diff -> motion_morph -> proj_bbox -> posture_fsm
//
// Input : 640x480 RGB565 little-endian frames, one de_in pixel per clock.
//         gray_front emits 320x180 after the 2x2 box + 2-of-3 row gate.
// Output: one line per frame: found x y w h area (same order as bbox_check),
//         plus ROW / ALARM lines from the single-target posture machine.
//
// The posture machine is enabled by the bbox result pulse, so it consumes
// exactly one observation per frame and counts gaps and cooldowns in frames.
// Its size-dependent constants come from sim/gen/params_chain.vh, which
// chain_check.py writes straight from the Python Params object.
//
// The bench inserts a realistic vertical blanking (BLANK clocks) between
// frames so motion_morph can run its 7 internal flush rows, and drains at
// the end so the last ping-pong bbox pass can finish.
//
// plusargs:
//   +IN=<file>   RGB565 640x480 frames
//   +OUT=<file>  per-frame bbox lines
//   +FRAMES=<n>
//   +MORPH=<file> optional: packed motion_morph output stream (one row per
//                 320 pixels, MSB first) for stage-by-stage debugging
//   +DIFF=<file>  optional: packed frame_diff motion mask (same format)
//===========================================================================
`timescale 1ns / 1ps
`include "params_chain.vh"

module tb_vision_chain;

localparam integer SW    = 640;
localparam integer SH    = 480;
localparam integer NPIX  = SW * SH;
localparam integer BLANK = 24000;
localparam integer DRAIN = 200000;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg frame_start = 1'b0;
reg de_in = 1'b0;
reg [15:0] pix_in = 16'd0;

wire de_gray, de_mask, de_morph;
wire [7:0] gray_out;
wire mask_diff, mask_morph;
wire de_bbox, bbox_found;
wire [8:0] bx, by, bw, bh;
wire [16:0] barea;
wire bbox_busy;

wire [2:0] fsm_state;
wire       fsm_alarm;
reg [31:0] fsm_frame;

// Single-target camera path: a lone person only needs the solo confirmation
// count (Params.confirm_frames at 30 fps).
localparam [2:0] SOLO_CONFIRM = 3'd2;

gray_front u_gray (
    .clk (clk), .rst_n (rst_n), .frame_start (frame_start),
    .de_in (de_in), .pix_in (pix_in),
    .de_out (de_gray), .gray_out (gray_out),
    .x_out (), .y_out ()
);

// DIFF_THRESH tracks the board build (fall_camera_hdmi_multi_top.v) so the
// chain check exercises the threshold that actually ships.
frame_diff #(.DIFF_THRESH(12)) u_diff (
    .clk (clk), .rst_n (rst_n), .frame_start (frame_start),
    .de_in (de_gray), .gray_in (gray_out),
    .de_out (de_mask), .mask_out (mask_diff)
);

motion_morph u_morph (
    .clk (clk), .rst_n (rst_n),
    .de_in (de_mask), .mask_in (mask_diff),
    .de_out (de_morph), .mask_out (mask_morph)
);

proj_bbox u_bbox (
    .clk (clk), .rst_n (rst_n),
    .de_in (de_morph), .mask_in (mask_morph),
    .busy (bbox_busy),
    .out_valid (de_bbox), .out_found (bbox_found),
    .out_x (bx), .out_y (by), .out_w (bw), .out_h (bh),
    .out_area (barea)
);

posture_fsm #(
    .FRAME_W         (320),
    .FRAME_H         (180),
    .MIN_REF_AREA    (`CHAIN_MIN_REF_AREA),
    .MIN_REF_HEIGHT  (`CHAIN_MIN_REF_HEIGHT),
    .MAX_REF_W       (`CHAIN_MAX_REF_W),
    .MAX_REF_H       (`CHAIN_MAX_REF_H),
    .MAX_REF_AREA    (`CHAIN_MAX_REF_AREA),
    .SUNK_CY2        (`CHAIN_SUNK_CY2),
    .REF_TTL         (`CHAIN_REF_TTL),
    .COLLAPSE_FRAMES (`CHAIN_COLLAPSE_FRAMES),
    .RECOVERY_WINDOW (`CHAIN_RECOVERY_WINDOW),
    .COOLDOWN_FRAMES (`CHAIN_COOLDOWN_FRAMES)
) u_fsm (
    .clk              (clk),
    .rst_n            (rst_n),
    .en               (de_bbox),
    .valid            (bbox_found),
    .frame            (fsm_frame),
    .x                ({1'b0, bx}),
    .y                ({1'b0, by}),
    .w                ({1'b0, bw}),
    .h                ({1'b0, bh}),
    .area             ({7'b0, barea}),
    .confirm_required (SOLO_CONFIRM),
    .alarm            (fsm_alarm),
    .state            (fsm_state),
    .ctx_in           ({242{1'b0}}),
    .ctx_load         (1'b0),
    .ctx_out          ()
);

always #5 clk = ~clk;

reg [1023:0] in_path, out_path;
reg [1023:0] morph_path;
reg [1023:0] diff_path;
integer fd_in, fd_out;
integer fd_morph;
integer fd_diff;
integer nframes, f, n, rc;
// The whole frame is pulled in with a single $fread into this array: the
// per-byte variant needed 13.8 M $fread calls for a 45-frame clip and made
// the long regression unusable, while one call per frame costs nothing.
reg [15:0] fbuf [0:NPIX-1];
integer got;
integer res_count;
reg [7:0] mbyte;
integer mcnt;

reg dump_morph;
reg dump_diff;
reg [7:0] dbyte;
integer dcnt;
integer x_diff, x_morph;
integer alarms;
// posture_fsm has a one-clock input stage, so the row for frame N is only
// complete at the edge that follows the bbox pulse.  Two candidate sampling
// rules run side by side while the bench is being characterised:
//
//   rule 1: read two edges after the de_bbox edge (the edge right after the
//           posture_fsm stage-2 update; the one-clock alarm is still high)
//   rule 2: read three edges after the de_bbox edge (one edge later)
//
// +SAMPLE=<1|2> selects which rule prints the ROW / ALARM lines the driver
// compares; the other rule prints DBG<n> lines carrying the same fields plus
// the raw alarm, so "wrong state" can be told apart from "sampled an edge too
// late".  CAP lines dump what the pipeline captured for this frame, which
// catches a capture that happened on the wrong observation.
integer sample_sel;
integer dbg_from;         // +DBG=<frame>: dump posture_fsm internals from here
reg        rec1_valid, rec2_valid;
reg [31:0] rec1_frame, rec2_frame;
reg        rec1_found, rec2_found;
reg [1:0]  rec1_wait,  rec2_wait;

task sample;
    input integer rule;
    input [31:0]  fr;
    input         found;
    begin
        if (rule == sample_sel) begin
            if (fsm_alarm) begin
                $display("ALARM %0d", fr);
                alarms <= alarms + 1;
            end
            $display("ROW %0d %0d %0d", fr, found, fsm_state);
        end else begin
            $display("DBG%0d %0d %0d %0d %0d", rule, fr, found,
                     fsm_state, fsm_alarm);
        end
    end
endtask

always @(posedge clk) begin
    if (de_bbox) begin
        $fdisplay(fd_out, "%0d %0d %0d %0d %0d %0d",
                  bbox_found, bx, by, bw, bh, barea);
        res_count <= res_count + 1;
        rec1_frame <= fsm_frame;
        rec1_found <= bbox_found;
        rec1_valid <= 1'b1;
        rec1_wait  <= 2'd1;
        rec2_frame <= fsm_frame;
        rec2_found <= bbox_found;
        rec2_valid <= 1'b1;
        rec2_wait  <= 2'd2;
        fsm_frame <= fsm_frame + 32'd1;
    end else begin
        if (rec1_valid) begin
            if (rec1_wait == 2'd0) begin
                sample(1, rec1_frame, rec1_found);
                rec1_valid <= 1'b0;
            end else begin
                if (rec1_wait == 2'd1)
                    $display("CAP %0d %0d %0d %0d", u_fsm.p_frame,
                             u_fsm.p_valid, u_fsm.en_q, fsm_state);
                // The active-region read at this edge sees the values of the
                // cycle the state machine is about to consume: p_* hold this
                // observation and the combinationals below are the verdict it
                // is about to act on.  reg_* are the pre-update state.
                if (dbg_from != 0 && rec1_frame >= dbg_from)
                    $display("FSMDBG %0d v=%0d x=%0d y=%0d w=%0d h=%0d a=%0d | ref_v=%0d ref_f=%0d ref_h=%0d ref_a=%0d | pend=%0d pf=%0d flat=%0d cool=%0d st=%0d | det=%0d rec=%0d win=%0d rises=%0d stands=%0d tall=%0d shr=%0d flatc=%0d sunk=%0d gnd=%0d keep=%0d fast=%0d feff=%0d",
                             rec1_frame, u_fsm.p_valid, u_fsm.p_x, u_fsm.p_y,
                             u_fsm.p_w, u_fsm.p_h, u_fsm.p_area,
                             u_fsm.ref_valid, u_fsm.ref_frame, u_fsm.ref_h,
                             u_fsm.ref_area, u_fsm.pending,
                             u_fsm.pending_frame, u_fsm.flat_frames,
                             u_fsm.cooldown, fsm_state,
                             u_fsm.detect_fire, u_fsm.recovery_now,
                             u_fsm.window_elapsed, u_fsm.rises,
                             u_fsm.stands, u_fsm.tall, u_fsm.shrunk,
                             u_fsm.flat, u_fsm.sunk, u_fsm.grounded,
                             u_fsm.keep, u_fsm.fast, u_fsm.flat_eff);
                rec1_wait <= rec1_wait - 2'd1;
            end
        end
        if (rec2_valid) begin
            if (rec2_wait == 2'd0) begin
                sample(2, rec2_frame, rec2_found);
                rec2_valid <= 1'b0;
            end else rec2_wait <= rec2_wait - 2'd1;
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
    if (dump_diff && de_mask) begin
        dbyte = {dbyte[6:0], mask_diff};
        if (dcnt == 7) begin
            $fwrite(fd_diff, "%c", dbyte);
            dcnt = 0;
            dbyte = 8'd0;
        end else begin
            dcnt = dcnt + 1;
        end
    end
    if (de_mask  && (mask_diff  === 1'bx)) x_diff  = x_diff + 1;
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
    if (!$value$plusargs("SAMPLE=%d", sample_sel)) sample_sel = 1;
    if (!$value$plusargs("DBG=%d", dbg_from)) dbg_from = 0;
    dump_morph = $value$plusargs("MORPH=%s", morph_path);
    if (dump_morph) fd_morph = $fopen(morph_path, "wb");
    dump_diff = $value$plusargs("DIFF=%s", diff_path);
    if (dump_diff) fd_diff = $fopen(diff_path, "wb");

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    res_count = 0;
    mbyte = 8'd0;
    mcnt  = 0;
    dbyte = 8'd0;
    dcnt  = 0;
    x_diff  = 0;
    x_morph = 0;
    alarms  = 0;
    rec1_valid = 1'b0;
    rec2_valid = 1'b0;
    rec1_wait  = 2'd0;
    rec2_wait  = 2'd0;
    fsm_frame = 32'd1;

    for (f = 0; f < nframes; f = f + 1) begin
        got = $fread(fbuf, fd_in);
        if (got != NPIX * 2) begin
            $display("ERROR short read at frame %0d: got %0d bytes, want %0d",
                     f, got, NPIX * 2);
            $finish;
        end

        // One-cycle frame_start pulse between two negedges; anything set
        // before the negedge would be seen by two posedges.
        @(negedge clk);
        frame_start = 1'b1;
        de_in = 1'b0;
        @(negedge clk);
        frame_start = 1'b0;

        for (n = 0; n < NPIX; n = n + 1) begin
            // Negative-edge drive, positive-edge sample: driving the pixel
            // before the wait would let the first pixel be sampled twice.
            @(negedge clk);
            // The file is little-endian 565 pairs, so the low byte of the
            // 16-bit memory word is the first byte on disk.
            pix_in = {fbuf[n][7:0], fbuf[n][15:8]};
            de_in = 1'b1;
            @(posedge clk); #1;
        end
        de_in = 1'b0;
        $display("frame %0d streamed: frame_cnt=%0d x_diff=%0d x_morph=%0d",
                 f, u_diff.frame_cnt, x_diff, x_morph);

        for (n = 0; n < BLANK; n = n + 1) begin
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
    $display("DONE results=%0d xdiff=%0d xmorph=%0d alarms=%0d",
             res_count, x_diff, x_morph, alarms);
    $finish;
end

endmodule
