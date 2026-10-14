//===========================================================================
// tb_probe_store.v -- debug probe: push one 320-pixel row into proj_bbox's
// stream side and dump the first packed bytes of both mask banks.
//
// Pattern: only pixel 0 and pixel 9 of the row are set.  Expected packing
// (MSB first) is byte0 = 0x80 and byte1 = 0x40.
//===========================================================================
`timescale 1ns / 1ps

module tb_probe_store;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg de_in = 1'b0;
reg mask_in = 1'b0;

wire busy, out_valid, out_found;
wire [8:0] out_x, out_y, out_w, out_h;
wire [16:0] out_area;

proj_bbox uut (
    .clk (clk), .rst_n (rst_n), .de_in (de_in), .mask_in (mask_in),
    .busy (busy), .out_valid (out_valid), .out_found (out_found),
    .out_x (out_x), .out_y (out_y), .out_w (out_w), .out_h (out_h),
    .out_area (out_area)
);

always #5 clk = ~clk;

integer n, i;
integer cyc;

always @(posedge clk) begin
    if (rst_n && cyc < 12) begin
        $display("cyc %0d de=%b m=%b wbit=%0d waddr=%0d wshift=%02b sxx=%0d",
                 cyc, de_in, mask_in, uut.wbit, uut.waddr, uut.wshift, uut.sx);
        cyc = cyc + 1;
    end
end

initial begin
    cyc = 0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    for (n = 0; n < 320; n = n + 1) begin
        @(negedge clk);
        mask_in = (n == 0 || n == 9);
        de_in = 1'b1;
        @(posedge clk); #1;
    end
    de_in = 1'b0;

    $display("bank0 bytes:");
    for (i = 0; i < 4; i = i + 1)
        $display("  [%0d] = %02x", i, uut.mask_ram[i]);
    $display("bank1 bytes:");
    for (i = 0; i < 4; i = i + 1)
        $display("  [%0d] = %02x", 9600 + i, uut.mask_ram[9600 + i]);
    $display("DONE");
    $finish;
end

endmodule
