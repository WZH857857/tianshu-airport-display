//===========================================================================
// tb_proj_bbox.v -- replay packed 320x180 masks through proj_bbox and dump
// the per-frame bounding box for the Python driver.
//
// plusargs:
//   +IN=<file>     packed masks, 7200 B per frame, MSB first per row
//   +OUT=<file>    text lines: found x y w h area
//   +FRAMES=<n>    number of frames
//
// One pixel per clock.  After the last pixel of a frame the bench waits for
// the post-frame pass (busy) to emit out_valid before starting the next
// frame.  In the real system the pass must fit inside vertical blanking.
//===========================================================================
`timescale 1ns / 1ps

module tb_proj_bbox;

localparam integer WIDTH  = 320;
localparam integer HEIGHT = 180;
localparam integer NPIX   = WIDTH * HEIGHT;
localparam integer NBYTES = WIDTH / 8 * HEIGHT;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg de_in = 1'b0;
reg mask_in = 1'b0;

wire busy, out_valid, out_found;
wire [8:0] out_x, out_y, out_w, out_h;
wire [16:0] out_area;

proj_bbox uut (
    .clk         (clk),
    .rst_n       (rst_n),
    .de_in       (de_in),
    .mask_in     (mask_in),
    .busy        (busy),
    .out_valid   (out_valid),
    .out_found   (out_found),
    .out_x       (out_x),
    .out_y       (out_y),
    .out_w       (out_w),
    .out_h       (out_h),
    .out_area    (out_area)
);

always #5 clk = ~clk;

reg [1023:0] in_path, out_path;
integer fd_in, fd_out;
integer nframes, f, n, rc;
reg [7:0] frame_buf [0:NBYTES-1];
integer res_count;
integer busy_seen;

always @(posedge clk) begin
    if (out_valid) begin
        $fdisplay(fd_out, "%0d %0d %0d %0d %0d %0d",
                  out_found, out_x, out_y, out_w, out_h, out_area);
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
    $display("DONE results=%0d busy_cycles=%0d", res_count, busy_seen);
    $finish;
end

endmodule
