`timescale 1ns / 1ps
//**********************************************************************
// Description: FIFO Generator for line buffer (depth=1024 for 640 pixels)
//*********************************************************************/

module fifo_generator_0 (
	rst,
	di, re, clk, we,
	do,
	empty_flag,full_flag,rdusedw,wrusedw
);

	input rst;
	input [7:0] di;
	input clk, we;
	input re;

	output [7:0] do;
	output empty_flag;
	output full_flag;
	output [10:0] rdusedw;
	output [10:0] wrusedw;

EG_LOGIC_RAMFIFO #(
 	.DATA_WIDTH(8),
	.ADDR_WIDTH(10),  // 2^10 = 1024 depth
	.SHOWAHEAD(1),
	.IMPLEMENT("AUTO")
)
logic_ramfifo(
	.rst(rst),
	.di(di),
	.clk(clk),
	.we(we),
	.do(do),
	.re(re),
	.empty_flag(empty_flag),
	.full_flag(full_flag),
	.rdusedw(rdusedw),
	.wrusedw(wrusedw)
);

endmodule
