//===========================================================================
// tb_pixel_front.v -- stream RGB565 frames through gray_front + frame_diff
// and dump both the downsampled luma and the motion mask for the Python
// driver to compare byte-for-byte.
//
// plusargs:
//   +IN=<file>     RGB565 little-endian pixels, 640x480 per frame
//   +GRAY=<file>   output: 8-bit luma       (320x180 per frame)
//   +MASK=<file>   output: packed mask bits (MSB = first pixel, 7200 B/frame)
//   +FRAMES=<n>    number of frames to stream
//===========================================================================
`timescale 1ns / 1ps

module tb_pixel_front;

localparam integer SRC_W  = 640;
localparam integer SRC_H  = 480;
localparam integer NPIX   = SRC_W * SRC_H;
localparam integer FLUSH  = 8;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg frame_start = 1'b0;
reg de_in = 1'b0;
reg [15:0] pix_in = 16'd0;

wire de_gray, de_mask;
wire [7:0] gray_out;
wire [9:0] x_out;
wire [8:0] y_out;
wire mask_out;

gray_front u_gray (
    .clk         (clk),
    .rst_n       (rst_n),
    .frame_start (frame_start),
    .de_in       (de_in),
    .pix_in      (pix_in),
    .de_out      (de_gray),
    .gray_out    (gray_out),
    .x_out       (x_out),
    .y_out       (y_out)
);

frame_diff #(
    .DIFF_THRESH (10)
) u_diff (
    .clk         (clk),
    .rst_n       (rst_n),
    .frame_start (frame_start),
    .de_in       (de_gray),
    .gray_in     (gray_out),
    .de_out      (de_mask),
    .mask_out    (mask_out)
);

always #5 clk = ~clk;

reg [1023:0] in_path, gray_path, mask_path;
integer fd_in, fd_gray, fd_mask;
integer nframes;
integer f, n, rc;
reg [7:0] lo, hi;
reg [7:0] mbyte;
integer mcnt;
integer cap_count;
integer verbose;
integer gray_count, mask_count;

task capture;
    begin
        // The gray stream and the mask stream are one clock apart now:
        // frame_diff registers its ERAM read, so its de_out/de_mask trail
        // gray_front by one clock.  Capture each stream on its own valid --
        // the byte order inside each file is what the checker compares.
        if (de_gray) begin
            $fwrite(fd_gray, "%c", gray_out);
            gray_count = gray_count + 1;
        end
        if (de_mask) begin
            mask_count = mask_count + 1;
            if (verbose && cap_count < 6)
                $display("CAP %0d: t=%0t n=%0d de_mask=%b gray=%0d mask=%b",
                         cap_count, $time, n, de_mask, gray_out, mask_out);
            cap_count = cap_count + 1;
            mbyte = {mbyte[6:0], mask_out};
            if (mcnt == 7) begin
                $fwrite(fd_mask, "%c", mbyte);
                mcnt = 0;
            end else begin
                mcnt = mcnt + 1;
            end
        end
    end
endtask

initial begin
    if (!$value$plusargs("IN=%s", in_path) ||
        !$value$plusargs("GRAY=%s", gray_path) ||
        !$value$plusargs("MASK=%s", mask_path) ||
        !$value$plusargs("FRAMES=%d", nframes)) begin
        $display("ERROR missing plusarg (need IN, GRAY, MASK, FRAMES)");
        $finish;
    end

    fd_in = $fopen(in_path, "rb");
    if (fd_in == 0) begin
        $display("ERROR cannot open input");
        $finish;
    end
    fd_gray = $fopen(gray_path, "wb");
    fd_mask = $fopen(mask_path, "wb");

    if (!$value$plusargs("VERBOSE=%d", verbose)) verbose = 0;
    cap_count = 0;
    gray_count = 0;
    mask_count  = 0;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    for (f = 0; f < nframes; f = f + 1) begin
        // One-cycle frame_start pulse during blanking.  It must be set
        // *after* a negedge and cleared after the next one: a value assigned
        // before a negedge is still high for the following posedge and then
        // for the intended one, i.e. it would be seen twice.
        // One-cycle frame_start pulse, asserted between two negedges so that
        // exactly one posedge sees it.
        @(negedge clk);
        frame_start = 1'b1;
        de_in = 1'b0;
        @(negedge clk);
        frame_start = 1'b0;
        mbyte = 8'd0;
        mcnt  = 0;
        gray_count = 0;
        mask_count = 0;

        for (n = 0; n < NPIX + FLUSH; n = n + 1) begin
            // Drive on the negedge, sample on the following posedge: setting
            // de_in before the wait would feed the first pixel twice.
            @(negedge clk);
            if (n < NPIX) begin
                rc = $fread(lo, fd_in);
                rc = $fread(hi, fd_in);
                pix_in = {hi, lo};
                de_in  = 1'b1;
            end else begin
                de_in = 1'b0;
            end
            @(posedge clk); #1;
            capture;
        end
        $display("frame %0d: gray=%0d mask=%0d", f, gray_count, mask_count);
    end

    $fclose(fd_in);
    $fclose(fd_gray);
    $fclose(fd_mask);
    $display("DONE");
    $finish;
end

endmodule
