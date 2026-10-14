// Minimal check of multi_bbox's stream write side: push a known first row and
// dump the bank, so "is the first pixel of the frame lost?" is unambiguous.
`timescale 1ns / 1ps

module tb_mask_write;
localparam integer WIDTH  = 320;
localparam integer HEIGHT = 180;
localparam integer NBYTES = WIDTH / 8 * HEIGHT;

reg clk = 0, rst_n = 0, de_in = 0, mask_in = 0;
wire busy, out_valid;
wire [3:0] out_found;
wire [3*9:0] out_x, out_y, out_w, out_h;
wire [3*17:0] out_area;

multi_bbox #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .NUM_BLOBS(4)) uut (
    .clk(clk), .rst_n(rst_n), .de_in(de_in), .mask_in(mask_in),
    .busy(busy), .out_valid(out_valid), .out_found(out_found),
    .out_x(out_x), .out_y(out_y), .out_w(out_w), .out_h(out_h),
    .out_area(out_area));

always #5 clk = ~clk;

integer n, y, x, fd;
initial begin
    fd = $fopen("fpga/td/gen/mask_write.txt", "w");
    repeat (4) @(negedge clk);
    rst_n = 1;
    // One full frame; pixel (0,0) alone is set on the first row, and a solid
    // 40-pixel run on rows 100..110 so the band picker has something to find.
    for (y = 0; y < HEIGHT; y = y + 1) begin
        for (x = 0; x < WIDTH; x = x + 1) begin
            @(negedge clk);
            de_in = 1;
            mask_in = (y == 0 && x == 0) || (y >= 100 && y <= 110 && x < 40);
            @(posedge clk); #1;
        end
    end
    de_in = 0; mask_in = 0;
    wait (uut.frame_done == 1);
    @(posedge clk); #1;
    $fdisplay(fd, "b0=%02x", uut.mask_ram[0]);
    $fdisplay(fd, "b1=%02x", uut.mask_ram[1]);
    $fdisplay(fd, "b1600=%02x", uut.mask_ram[100 * 40]);
    $fdisplay(fd, "b1601=%02x", uut.mask_ram[100 * 40 + 1]);
    $fclose(fd);
    $display("done");
    $finish;
end
endmodule
