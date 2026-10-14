//===========================================================================
// tb_multi_bbox.v -- replay packed 320x180 masks through multi_bbox and dump
// all NUM_BLOBS slots for the Python driver.
//
// plusargs:
//   +IN=<file>     packed masks, 7200 B per frame, MSB first per row
//   +OUT=<file>    text lines: per slot "found x y w h area"
//   +FRAMES=<n>    number of frames
//===========================================================================
`timescale 1ns / 1ps

module tb_multi_bbox;

localparam integer WIDTH     = 320;
localparam integer HEIGHT    = 180;
localparam integer NPIX      = WIDTH * HEIGHT;
localparam integer NBYTES    = WIDTH / 8 * HEIGHT;
localparam integer NUM_BLOBS = 4;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg de_in = 1'b0;
reg mask_in = 1'b0;

wire busy;
wire out_valid;
wire [NUM_BLOBS-1:0]    out_found;
wire [NUM_BLOBS*9-1:0]  out_x, out_y, out_w, out_h;
wire [NUM_BLOBS*17-1:0] out_area;

multi_bbox #(
    .WIDTH     (WIDTH),
    .HEIGHT    (HEIGHT),
    .NUM_BLOBS (NUM_BLOBS)
) uut (
    .clk       (clk),
    .rst_n     (rst_n),
    .de_in     (de_in),
    .mask_in   (mask_in),
    .busy      (busy),
    .out_valid (out_valid),
    .out_found (out_found),
    .out_x     (out_x),
    .out_y     (out_y),
    .out_w     (out_w),
    .out_h     (out_h),
    .out_area  (out_area)
);

always #5 clk = ~clk;

reg [1023:0] in_path, out_path;
reg dbg_on = 1'b0;
integer fd_in, fd_out;
integer nframes, f, n, rc, k;
reg [7:0] frame_buf [0:NBYTES-1];
integer res_count;
integer busy_seen;
integer fd_mask;
reg [1023:0] mask_path;
integer dump_n = 0;
integer frames_done = 0;
integer dump_pending = 0;

// Debug: snapshot one pass's histogram the cycle after it is fully built.
// +HIST=<file> is used by tools/hist_diff.py when bisecting a mismatch.
reg        hist_dbg_armed = 1'b0;
reg        hist_dbg_seen = 1'b0;
integer    hist_dbg_pass = 0;
integer    fd_hist = 0;
reg [1023:0] hist_path;
always @(posedge clk) begin
    if (uut.state != 2'd3)
        hist_dbg_seen <= 1'b0;
    // ST_SPAN is entered on the clock after the last histogram write, so the
    // array reads back complete here and pass_idx has not moved on yet.
    if (uut.state == 2'd3 && !hist_dbg_seen) begin
        hist_dbg_seen  <= 1'b1;
        hist_dbg_armed <= 1'b1;
        hist_dbg_pass  <= uut.pass_idx;
    end
    if (hist_dbg_armed && !hist_dbg_seen) begin
        if (fd_hist != 0) begin
            $fdisplay(fd_hist, "HIST %0d", hist_dbg_pass);
            for (k = 0; k < WIDTH; k = k + 1)
                $fdisplay(fd_hist, "%0d", uut.hist[k]);
        end
        hist_dbg_armed <= 1'b0;
    end
end

// Optional: dump one bank of the on-chip mask RAM so the harness can verify
// the write side is bit-identical to the packed input.
task dump_bank;
    input integer bank;
    integer a;
    begin
        $fdisplay(fd_mask, "BANK %0d", bank);
        for (a = 0; a < NBYTES; a = a + 1)
            $fdisplay(fd_mask, "%02x", uut.mask_ram[bank * NBYTES + a]);
    end
endtask

always @(posedge clk) begin
    if (dump_pending != 0) begin
        dump_bank((frames_done - 1) % 2);
        dump_pending <= 0;
    end
    if (uut.frame_done) begin
        frames_done <= frames_done + 1;
        if (frames_done < dump_n)
            dump_pending <= 1;
    end
end

always @(posedge clk) begin
    if (out_valid) begin
        for (k = 0; k < NUM_BLOBS; k = k + 1)
            $fdisplay(fd_out, "%0d %0d %0d %0d %0d %0d",
                      out_found[k],
                      out_x[k*9 +: 9], out_y[k*9 +: 9],
                      out_w[k*9 +: 9], out_h[k*9 +: 9],
                      out_area[k*17 +: 17]);
        if (dbg_on) begin
            for (k = 0; k < NUM_BLOBS; k = k + 1)
                $display("EXSLOT %0d valid=%0d x=%0d y=%0d x1=%0d y1=%0d",
                         k, uut.ex_valid[k], uut.ex_x[k], uut.ex_y[k],
                         uut.ex_x1[k], uut.ex_y1[k]);
        end
        res_count <= res_count + 1;
    end
    if (busy)
        busy_seen <= busy_seen + 1;
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
    dbg_on = $test$plusargs("DBG");
    fd_mask = 0;
    dump_pending = 0;
    frames_done = 0;
    if ($value$plusargs("MASKD=%s", mask_path)) begin
        fd_mask = $fopen(mask_path, "w");
        if (!$value$plusargs("DUMPN=%d", dump_n))
            dump_n = 1;
    end
    if ($value$plusargs("HIST=%s", hist_path))
        fd_hist = $fopen(hist_path, "w");

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    res_count = 0;
    busy_seen = 0;

    for (f = 0; f < nframes; f = f + 1) begin
        rc = $fread(frame_buf, fd_in);

        for (n = 0; n < NPIX; n = n + 1) begin
            @(negedge clk);
            mask_in = frame_buf[n >> 3] >> (7 - (n & 7));
            de_in = 1'b1;
            @(posedge clk); #1;
        end
        de_in = 1'b0;
        mask_in = 1'b0;

        while (res_count <= f) begin
            @(negedge clk);
            @(posedge clk); #1;
        end
    end

    $fclose(fd_in);
    $fclose(fd_out);
    if (fd_mask != 0)
        $fclose(fd_mask);
    if (fd_hist != 0)
        $fclose(fd_hist);
    $display("DONE results=%0d busy_cycles=%0d", res_count, busy_seen);
    $finish;
end

endmodule
