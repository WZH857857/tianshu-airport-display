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
// Date:2021/02
// Description: 
// image_laplacian_filtering
//
// 
// Web:http://www.uisrc.com
//------------------------------------------------------------------- 
//*********************************************************************/
module image_laplacian_filtering
(
	input   wire				i_clk,
	input   wire				i_rst_n,

	input	wire				i_hsyn,
	input	wire				i_vsyn,
	input	wire				i_en,
	input	wire [7:0]			i_r,
	input	wire [7:0] 			i_g,
	input	wire [7:0] 			i_b,		
	
	output	wire 				o_hs,
	output	wire 				o_vs,
	output	wire 				o_en,	
	output  wire [7:0]			o_r,
	output  wire [7:0]			o_g,
	output  wire [7:0]			o_b	
);

reg	[2:0]		i_hsyn_d;
reg	[2:0]		i_vsyn_d;
reg	[2:0]		i_en_d;

wire [11:0]		sum_r;
wire [11:0]		sum_g;
wire [11:0]		sum_b;

reg [11:0]		sum_r1;
reg [11:0]		sum_g1;
reg [11:0]		sum_b1;

reg [11:0]		sum_r2;
reg [11:0]		sum_g2;
reg [11:0]		sum_b2;

reg [7:0]		lap_r;
reg [7:0]		lap_g;
reg [7:0]		lap_b;

wire [7:0]	r_temp_11;
wire [7:0]	r_temp_12;
wire [7:0]	r_temp_13;
wire [7:0]	r_temp_21;
wire [7:0]	r_temp_22;
wire [7:0]	r_temp_23;
wire [7:0]	r_temp_31;
wire [7:0]	r_temp_32;
wire [7:0]	r_temp_33;

wire [7:0]	g_temp_11;
wire [7:0]	g_temp_12;
wire [7:0]	g_temp_13;
wire [7:0]	g_temp_21;
wire [7:0]	g_temp_22;
wire [7:0]	g_temp_23;
wire [7:0]	g_temp_31;
wire [7:0]	g_temp_32;
wire [7:0]	g_temp_33;

wire [7:0]	b_temp_11;
wire [7:0]	b_temp_12;
wire [7:0]	b_temp_13;
wire [7:0]	b_temp_21;
wire [7:0]	b_temp_22;
wire [7:0]	b_temp_23;
wire [7:0]	b_temp_31;
wire [7:0]	b_temp_32;
wire [7:0]	b_temp_33;

assign o_hs = i_hsyn_d[2];
assign o_vs = i_vsyn_d[2];
assign o_en = i_en_d[2]	;
assign o_r	= lap_r;
assign o_g	= lap_g;
assign o_b	= lap_b;

always@(posedge i_clk ) 
begin
	i_hsyn_d <=	{i_hsyn_d[1:0],i_hsyn};
	i_vsyn_d <=	{i_vsyn_d[1:0],i_vsyn};
	i_en_d	 <=	{i_en_d[1:0],i_en};
     	
end

image_template u_r_template
(
	.i_clk			(i_clk				),
	.i_rst_n		(i_rst_n			),
	.i_en			(i_en				),
	.i_data			(i_r				),
	.o_en			(					),
	.o_temp_11		(r_temp_11			),
	.o_temp_12		(r_temp_12			),
	.o_temp_13		(r_temp_13			),	
	.o_temp_21		(r_temp_21			),
	.o_temp_22		(r_temp_22			),
	.o_temp_23		(r_temp_23			),		
	.o_temp_31		(r_temp_31			),
	.o_temp_32		(r_temp_32			),
	.o_temp_33		(r_temp_33			)
);

image_template u_g_template
(
	.i_clk			(i_clk				),
	.i_rst_n		(i_rst_n			),
	.i_en			(i_en				),
	.i_data			(i_g				),
	.o_en			(					),
	.o_temp_11		(g_temp_11			),
	.o_temp_12		(g_temp_12			),
	.o_temp_13		(g_temp_13			),	
	.o_temp_21		(g_temp_21			),
	.o_temp_22		(g_temp_22			),
	.o_temp_23		(g_temp_23			),		
	.o_temp_31		(g_temp_31			),
	.o_temp_32		(g_temp_32			),
	.o_temp_33		(g_temp_33			)
);

image_template u_b_template
(
	.i_clk			(i_clk				),
	.i_rst_n		(i_rst_n			),
	.i_en			(i_en				),
	.i_data			(i_b				),
	.o_en			(					),
	.o_temp_11		(b_temp_11			),
	.o_temp_12		(b_temp_12			),
	.o_temp_13		(b_temp_13			),	
	.o_temp_21		(b_temp_21			),
	.o_temp_22		(b_temp_22			),
	.o_temp_23		(b_temp_23			),		
	.o_temp_31		(b_temp_31			),
	.o_temp_32		(b_temp_32			),
	.o_temp_33		(b_temp_33			)
);

always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n) 
	begin
        sum_r1 <= 12'd0;
        sum_g1 <= 12'd0;
        sum_b1 <= 12'd0;
		sum_r2 <= 12'd0;
        sum_g2 <= 12'd0;
        sum_b2 <= 12'd0;
    end
    else 
	begin
        sum_r1 <= r_temp_12 + r_temp_21 + r_temp_23 + r_temp_32;
		sum_r2 <= {2'd0,r_temp_22,2'd0} + {4'd0,r_temp_22};
		
        sum_g1 <= g_temp_12 + g_temp_21 + g_temp_23 + g_temp_32;
		sum_g2 <= {2'd0,g_temp_22,2'd0} + {4'd0,g_temp_22};	
		
        sum_b1 <= b_temp_12 + b_temp_21 + b_temp_23 + b_temp_32;
		sum_b2 <= {2'd0,b_temp_22,2'd0} + {4'd0,b_temp_22};
    end
end

assign  sum_r = sum_r2 - sum_r1;
assign 	sum_g = sum_g2 - sum_g1;
assign 	sum_b = sum_b2 - sum_b1;

always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n) 
	begin
        lap_r <= 8'd0;
    end
    else if(sum_r[11:8] > 0)
	begin
        lap_r <= 8'd255;
    end
    else if(sum_r2 < sum_r1)
	begin
        lap_r <= 8'd0;
    end
	else
	begin
        lap_r <= sum_r[7:0];
    end	
end

always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n) 
	begin
        lap_g <= 8'd0;
    end
    else if(sum_g[11:8] > 0)
	begin
        lap_g <= 8'd255;
    end
    else if(sum_g2 < sum_g1)
	begin
        lap_g <= 8'd0;
    end
	else
	begin
        lap_g <= sum_g[7:0];
    end	
end

always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n) 
	begin
        lap_b <= 8'd0;
    end
    else if(sum_b[11:8] > 0)
	begin
        lap_b <= 8'd255;
    end
    else if(sum_b2 < sum_b1)
	begin
        lap_b <= 8'd0;
    end
	else
	begin
        lap_b <= sum_b[7:0];
    end	
end

endmodule