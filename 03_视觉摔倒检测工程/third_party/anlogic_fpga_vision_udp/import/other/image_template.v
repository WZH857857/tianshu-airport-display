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
// image_template
//
//
// Web:http://www.uisrc.com
//-------------------------------------------------------------------
//*********************************************************************/

`define TEMPLATE_3X3
// `define TEMPLATE_5X5

module image_template
(
	input   wire				i_clk,
	input   wire				i_rst_n,

	input	wire				i_en,
	input	wire [7:0]			i_data,

	output	reg 				o_en,
	`ifdef TEMPLATE_3X3
	output  reg [7:0]			o_temp_11,
	output  reg [7:0]			o_temp_12,
	output  reg [7:0]			o_temp_13,
	output  reg [7:0]			o_temp_21,
	output  reg [7:0]			o_temp_22,
	output  reg [7:0]			o_temp_23,
	output  reg [7:0]			o_temp_31,
	output  reg [7:0]			o_temp_32,
	output  reg [7:0]			o_temp_33
	`endif
	`ifdef TEMPLATE_5X5
	output  reg [7:0]			o_temp_11,
	output  reg [7:0]			o_temp_12,
	output  reg [7:0]			o_temp_13,
	output  reg [7:0]			o_temp_14,
	output  reg [7:0]			o_temp_15,
	output  reg [7:0]			o_temp_21,
	output  reg [7:0]			o_temp_22,
	output  reg [7:0]			o_temp_23,
	output  reg [7:0]			o_temp_24,
	output  reg [7:0]			o_temp_25,
	output  reg [7:0]			o_temp_31,
	output  reg [7:0]			o_temp_32,
	output  reg [7:0]			o_temp_33,
	output  reg [7:0]			o_temp_34,
	output  reg [7:0]			o_temp_35,
	output  reg [7:0]			o_temp_41,
	output  reg [7:0]			o_temp_42,
	output  reg [7:0]			o_temp_43,
	output  reg [7:0]			o_temp_44,
	output  reg [7:0]			o_temp_45,
	output  reg [7:0]			o_temp_51,
	output  reg [7:0]			o_temp_52,
	output  reg [7:0]			o_temp_53,
	output  reg [7:0]			o_temp_54,
	output  reg [7:0]			o_temp_55
	`endif
);
parameter  H_ACTIVE = 640; //图像宽度 (Modified from 1024)
parameter  V_ACTIVE = 480; //图像高度 (Modified from 600)

reg  [10:0]	h_cnt;
reg  [10:0]	v_cnt;

wire [7:0]	fifo_1_in;
wire 		fifo_1_wr_en;
wire 		fifo_1_rd_en;
wire [7:0]	fifo_1_out;

wire [7:0]	fifo_2_in;
wire 		fifo_2_wr_en;
wire 		fifo_2_rd_en;
wire [7:0]	fifo_2_out;

`ifdef TEMPLATE_5X5
wire [7:0]	fifo_3_in;
wire 		fifo_3_wr_en;
wire 		fifo_3_rd_en;
wire [7:0]	fifo_3_out;

wire [7:0]	fifo_4_in;
wire 		fifo_4_wr_en;
wire 		fifo_4_rd_en;
wire [7:0]	fifo_4_out;
`endif


//显示区域行计数
always@(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
	begin
        h_cnt <= 11'd0;
    end
    else if(i_en)
	begin
		if(h_cnt == H_ACTIVE - 1'b1)
			h_cnt <= 11'd0;
		else
			h_cnt <= h_cnt + 11'd1;
    end
end

//显示区域场计数
always@(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
	begin
        v_cnt <= 11'd0;
    end
    else if(h_cnt == H_ACTIVE - 1'b1)
	begin
		if(v_cnt == V_ACTIVE - 1'b1)
			v_cnt <= 11'd0;
		else
			v_cnt <= v_cnt + 11'd1;
    end
end

assign fifo_1_in	= i_data;
assign fifo_1_wr_en	= (v_cnt < V_ACTIVE - 1) ? i_en : 1'b0;
assign fifo_1_rd_en	= (v_cnt > 0 ) ? i_en : 1'b0;

assign fifo_2_in	= fifo_1_out;
assign fifo_2_wr_en	= fifo_1_rd_en && (v_cnt < V_ACTIVE - 2);
assign fifo_2_rd_en	= (v_cnt > 1 ) ? i_en : 1'b0;

`ifdef TEMPLATE_5X5
assign fifo_3_in	= fifo_2_out;
assign fifo_3_wr_en	= fifo_2_rd_en && (v_cnt < V_ACTIVE - 3);
assign fifo_3_rd_en	= (v_cnt > 2) ? i_en : 1'b0;

assign fifo_4_in	= fifo_3_out;
assign fifo_4_wr_en	= fifo_3_rd_en && (v_cnt < V_ACTIVE - 4);
assign fifo_4_rd_en	= (v_cnt > 3) ? i_en : 1'b0;
`endif

fifo_generator_0 u_fifo_1(
	.clk        (i_clk			),
	.rst        (!i_rst_n		),
	.di         (fifo_1_in		),
	.we         (fifo_1_wr_en	),
	.re         (fifo_1_rd_en	),
	.do         (fifo_1_out		),
	.full_flag  (				),
	.empty_flag (               ),
	.rdusedw    (               ),
	.wrusedw    (               )
);

fifo_generator_0 u_fifo_2(
	.clk        (i_clk			),
	.rst        (!i_rst_n		),
	.di         (fifo_2_in		),
	.we         (fifo_2_wr_en	),
	.re         (fifo_2_rd_en	),
	.do         (fifo_2_out		),
	.full_flag  (				),
	.empty_flag (               ),
	.rdusedw    (               ),
	.wrusedw    (               )
);

`ifdef TEMPLATE_5X5
fifo_generator_0 u_fifo_3(
	.clk        (i_clk			),
	.rst        (!i_rst_n		),
	.di         (fifo_3_in		),
	.we         (fifo_3_wr_en	),
	.re         (fifo_3_rd_en	),
	.do         (fifo_3_out		),
	.full_flag  (				),
	.empty_flag (               ),
	.rdusedw    (               ),
	.wrusedw    (               )
);

fifo_generator_0 u_fifo_4(
	.clk        (i_clk			),
	.rst        (!i_rst_n		),
	.di         (fifo_4_in		),
	.we         (fifo_4_wr_en	),
	.re         (fifo_4_rd_en	),
	.do         (fifo_4_out		),
	.full_flag  (				),
	.empty_flag (               ),
	.rdusedw    (               ),
	.wrusedw    (               )
);
`endif

`ifdef TEMPLATE_3X3
always@(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
	begin
		o_temp_11	<= 8'd0;
		o_temp_12	<= 8'd0;
		o_temp_13	<= 8'd0;

		o_temp_21	<= 8'd0;
		o_temp_22	<= 8'd0;
		o_temp_23	<= 8'd0;

		o_temp_31	<= 8'd0;
		o_temp_32	<= 8'd0;
		o_temp_33	<= 8'd0;
    end
	else if(v_cnt == 0)
	begin
		if(h_cnt == 0)
		begin
			o_temp_11	<= i_data;
			o_temp_12	<= i_data;
			o_temp_13	<= i_data;
			o_temp_21	<= i_data;
			o_temp_22	<= i_data;
			o_temp_23	<= i_data;
			o_temp_31	<= i_data;
			o_temp_32	<= i_data;
			o_temp_33	<= i_data;
		end
		else
		begin
			o_temp_11	<= o_temp_12;
			o_temp_12	<= o_temp_13;
			o_temp_13	<= i_data;
			o_temp_21	<= o_temp_22;
			o_temp_22	<= o_temp_23;
			o_temp_23	<= i_data;
			o_temp_31	<= o_temp_32;
			o_temp_32	<= o_temp_33;
			o_temp_33	<= i_data;
		end
	end
	else if(v_cnt == 1)
	begin
		if(h_cnt == 0)
		begin
			o_temp_11	<= fifo_1_out;
			o_temp_12	<= fifo_1_out;
			o_temp_13	<= fifo_1_out;
			o_temp_21	<= fifo_1_out;
			o_temp_22	<= fifo_1_out;
			o_temp_23	<= fifo_1_out;
			o_temp_31	<= i_data;
			o_temp_32	<= i_data;
			o_temp_33	<= i_data;
		end
		else
		begin
			o_temp_11	<= o_temp_12;
			o_temp_12	<= o_temp_13;
			o_temp_13	<= fifo_1_out;
			o_temp_21	<= o_temp_22;
			o_temp_22	<= o_temp_23;
			o_temp_23	<= fifo_1_out;
			o_temp_31	<= o_temp_32;
			o_temp_32	<= o_temp_33;
			o_temp_33	<= i_data;
		end
	end
	else
	begin
		if(h_cnt == 0)
		begin
			o_temp_11	<= fifo_2_out;
			o_temp_12	<= fifo_2_out;
			o_temp_13	<= fifo_2_out;
			o_temp_21	<= fifo_1_out;
			o_temp_22	<= fifo_1_out;
			o_temp_23	<= fifo_1_out;
			o_temp_31	<= i_data;
			o_temp_32	<= i_data;
			o_temp_33	<= i_data;
		end
		else
		begin
			o_temp_11	<= o_temp_12;
			o_temp_12	<= o_temp_13;
			o_temp_13	<= fifo_2_out;
			o_temp_21	<= o_temp_22;
			o_temp_22	<= o_temp_23;
			o_temp_23	<= fifo_1_out;
			o_temp_31	<= o_temp_32;
			o_temp_32	<= o_temp_33;
			o_temp_33	<= i_data;
		end
	end
end

// 修复：让输出使能始终跟随输入使能，避免数据错位
// 对于边界像素（前2行、前2列），窗口已通过像素复制填充
always@(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
	begin
		o_en	<= 1'b0;
    end
	else
	begin
		o_en	<= i_en;  // 始终跟随输入使能，保持输出尺寸与输入一致
	end
end
`endif

`ifdef TEMPLATE_5X5
always@(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
	begin
		o_temp_11	<= 8'd0;
		o_temp_12	<= 8'd0;
		o_temp_13	<= 8'd0;
		o_temp_14	<= 8'd0;
		o_temp_15	<= 8'd0;
		o_temp_21	<= 8'd0;
		o_temp_22	<= 8'd0;
		o_temp_23	<= 8'd0;
		o_temp_24	<= 8'd0;
		o_temp_25	<= 8'd0;
		o_temp_31   <= 8'd0;
		o_temp_32   <= 8'd0;
		o_temp_33   <= 8'd0;
		o_temp_34   <= 8'd0;
		o_temp_35   <= 8'd0;
		o_temp_41   <= 8'd0;
		o_temp_42   <= 8'd0;
		o_temp_43   <= 8'd0;
		o_temp_44	<= 8'd0;
		o_temp_45   <= 8'd0;
		o_temp_51   <= 8'd0;
		o_temp_52   <= 8'd0;
		o_temp_53   <= 8'd0;
		o_temp_54   <= 8'd0;
		o_temp_55   <= 8'd0;
    end
	else
	begin
		o_temp_11	<= o_temp_12;
		o_temp_12	<= o_temp_13;
		o_temp_13	<= o_temp_14;
		o_temp_14	<= o_temp_15;
		o_temp_15	<= fifo_4_out;

		o_temp_21	<= o_temp_22;
		o_temp_22	<= o_temp_23;
		o_temp_23	<= o_temp_24;
		o_temp_24	<= o_temp_25;
		o_temp_25	<= fifo_3_out;

		o_temp_31   <= o_temp_32;
		o_temp_32   <= o_temp_33;
		o_temp_33   <= o_temp_34;
		o_temp_34   <= o_temp_35;
		o_temp_35   <= fifo_2_out;

		o_temp_41   <= o_temp_42;
		o_temp_42   <= o_temp_43;
		o_temp_43   <= o_temp_44;
		o_temp_44	<= o_temp_45;
		o_temp_45   <= fifo_1_out;

		o_temp_51   <= o_temp_52;
		o_temp_52   <= o_temp_53;
		o_temp_53   <= o_temp_54;
		o_temp_54   <= o_temp_55;
		o_temp_55   <= i_data;
	end
end

// 修复：让输出使能始终跟随输入使能，避免数据错位
// 对于边界像素（前4行、前4列），窗口已通过像素复制填充
always@(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
	begin
		o_en	<= 1'b0;
    end
	else
	begin
		o_en	<= i_en;  // 始终跟随输入使能，保持输出尺寸与输入一致
	end
end
`endif
endmodule
