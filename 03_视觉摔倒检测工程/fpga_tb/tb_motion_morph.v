//===========================================================================
// tb_motion_morph.v -- replay packed 320x180 binary masks through
// motion_morph and dump the packed result for the Python driver.
//
// Input format matches the frame_diff output: one bit per pixel, MSB first,
// 40 bytes per row, 180 rows per frame (7200 bytes/frame).
//
// plusargs:
//   +IN=<file>     packed input masks,  7200 B per frame
//   +OUT=<file>    packed output masks, 7200 B per frame
//   +FRAMES=<n>    number of frames
//   +GAP=<n>       idle clocks between pixels (default 0; the real chain
//                  delivers one de_in pulse every second clock)
//   +ROW_GAP=<n>   idle clocks between rows (default 6; the real chain has
//                  a long inter-row gap)
//
// Timing follows the module contract: per row 320 de_in pixels + 6 idle
// slots; after the last real row the bench keeps idling long enough for the
// internal 7 flush rows (7*326 clocks) to drain the vertical pipeline.
//===========================================================================
`timescale 1ns / 1ps

module tb_motion_morph;

localparam integer WIDTH      = 320;
localparam integer HEIGHT     = 180;
localparam integer ROW_BYTES  = WIDTH / 8;
localparam integer CLEAR_IDLE = 336;
localparam integer FLUSH_ROWS = 8;
localparam integer DRAIN      = 512;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg de_in = 1'b0;
reg mask_in = 1'b0;

wire de_out, mask_out;

motion_morph uut (
    .clk         (clk),
    .rst_n       (rst_n),
    .de_in       (de_in),
    .mask_in     (mask_in),
    .de_out      (de_out),
    .mask_out    (mask_out)
);

always #5 clk = ~clk;

reg [1023:0] in_path, out_path;
integer fd_in, fd_out;
integer nframes, f, row, col, b, rc;
integer gap, row_gap, g;
reg [7:0] rowbuf [0:ROW_BYTES-1];
reg [7:0] tmp_byte;
reg [7:0] mbyte;
integer mcnt;
integer out_bytes;
integer t;

task capture;
    begin
        if (de_out) begin
            mbyte = {mbyte[6:0], mask_out};
            if (mcnt == 7) begin
                $fwrite(fd_out, "%c", mbyte);
                mcnt = 0;
                out_bytes = out_bytes + 1;
            end else begin
                mcnt = mcnt + 1;
            end
        end
    end
endtask

task tick_idle;
    begin
        de_in  = 1'b0;
        mask_in = 1'b0;
        @(negedge clk);
        @(posedge clk); #1;
        capture;
    end
endtask

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
    fd_out = $fopen(out_path, "wb");

    if (!$value$plusargs("GAP=%d", gap)) gap = 0;
    if (!$value$plusargs("ROW_GAP=%d", row_gap)) row_gap = 6;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    out_bytes = 0;

    for (f = 0; f < nframes; f = f + 1) begin
        de_in = 1'b0;
        mbyte = 8'd0;
        mcnt  = 0;

        for (t = 0; t < CLEAR_IDLE; t = t + 1)
            tick_idle;

        for (row = 0; row < HEIGHT; row = row + 1) begin
            for (b = 0; b < ROW_BYTES; b = b + 1)
                begin
                    rc = $fread(tmp_byte, fd_in);
                    rowbuf[b] = tmp_byte;
                end

            for (col = 0; col < WIDTH; col = col + 1) begin
                mask_in = rowbuf[col >> 3] >> (7 - (col & 7));
                de_in = 1'b1;
                @(negedge clk);
                @(posedge clk); #1;
                capture;
                for (g = 0; g < gap; g = g + 1)
                    tick_idle;
            end
            for (t = 0; t < row_gap; t = t + 1)
                tick_idle;
        end

        // Extra idle clocks let the internal flush rows finish the bottom
        // border and drain the output pipeline.
        for (t = 0; t < FLUSH_ROWS * 326 + DRAIN; t = t + 1)
            tick_idle;
    end

    $fclose(fd_in);
    $fclose(fd_out);
    $display("DONE bytes=%0d rows=%0d", out_bytes, out_bytes / (WIDTH / 8));
    $finish;
end

endmodule
