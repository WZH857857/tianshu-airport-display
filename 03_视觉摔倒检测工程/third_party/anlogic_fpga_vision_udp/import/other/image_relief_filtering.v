`timescale 1ns / 1ps
//********************************************************************** 
// -------------------------------------------------------------------
// Disclaimers
// -------------------------------------------------------------------
// When you use this source file, please note that the author assumes 
// no legal or non-legal responsibility for any consequences of your use 
// of it including, but not limited to, the specific use of the code or 
// liability to you for infringement of any patent, copyright or other 
// intellectual property rights.
// If you do not agree to the terms, please do not use the file and 
// delete the file promptly.
// -------------------------------------------------------------------
// Copyright Notice
// -------------------------------------------------------------------
// This source file may be used for personal study, provided that this 
// copyright notice is not removed from the file.
// and that derivative works of this source file contain the original 
// copyright notice and related disclaimers.
// For commercial use, please contact the author for authorization, 
// otherwise the author reserves all rights.
// ------------------------------------------------------------------- 
// Author: Geeker_FPGA 
// Email:geeker_fpga@uisrc.com v
// Date:2021/03/06
// Description: 
// image_relief_filtering
//
// 
// Web:http://www.uisrc.com
//------------------------------------------------------------------- 
//*********************************************************************/
module image_relief_filtering
(
	input   wire				i_clk,
	input   wire				i_rst_n,

	input	wire				i_hsyn,
	input	wire				i_vsyn,
	input	wire				i_en,
	input	wire [7:0]			i_gray,
	
	input   wire [7:0]      	value,	
	
	output	wire 				o_hs,
	output	wire 				o_vs,
	output	wire 				o_en,	
	output  wire [7:0]			o_gray
);

reg [1:0]	hsyn_reg;
reg [1:0]	vsyn_reg;
reg [1:0]	en_reg;
reg [7:0]	gray_reg;
reg [7:0]	gray_reg_1d;
wire signed [9:0]	relief;


assign 		o_hs		= hsyn_reg[1];
assign 		o_vs		= vsyn_reg[1];
assign 		o_en		= en_reg[1];		
assign 		o_gray		= gray_reg_1d;

assign 		relief 		= i_gray - gray_reg + value;

always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
        hsyn_reg	<= 'd0;
		vsyn_reg	<= 'd0;
		en_reg		<= 'd0;
		gray_reg	<= 'd0;	
    end
    else 
	begin
        hsyn_reg	<= {hsyn_reg[0],i_hsyn};
		vsyn_reg	<= {vsyn_reg[0],i_vsyn};
		en_reg		<= {en_reg[0],i_en};
		gray_reg	<= i_gray;	      	
    end
end

always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
        gray_reg_1d	<= 'd0;
    end
    else if(relief > 255)
	begin
        gray_reg_1d	<= 255;   	
    end
    else if(relief < 0)
	begin
        gray_reg_1d	<= 0;   	
    end	
    else 
	begin
        gray_reg_1d	<= relief;   	
    end	
end

endmodule