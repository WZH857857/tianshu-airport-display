`timescale 1ns / 1ps
//**********************************************************************
// Description: Image Erosion Filtering (腐蚀滤波)
// 修改：使用 line_buffer 替代 image_template，避免数据错位
//*********************************************************************/
module image_erode_filtering
(
	input   wire				i_clk,
	input   wire				i_rst_n,

	input	wire				i_hsyn,
	input	wire				i_vsyn,
	input	wire				i_en,
	input	wire [7:0]			i_binary,

	output	wire 				o_hs,
	output	wire 				o_vs,
	output	wire 				o_en,
	output  wire [7:0]			o_binary
);

// ========================================================================
// 信号延迟对齐（line_buffer有3周期延迟）
// ========================================================================
reg [2:0] i_hsyn_d;
reg [2:0] i_vsyn_d;

always@(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        i_hsyn_d <= 3'b0;
        i_vsyn_d <= 3'b0;
    end
    else begin
        i_hsyn_d <= {i_hsyn_d[1:0], i_hsyn};
        i_vsyn_d <= {i_vsyn_d[1:0], i_vsyn};
    end
end

assign o_hs = i_hsyn_d[2];
assign o_vs = i_vsyn_d[2];

// ========================================================================
// line_buffer 实例化：生成3x3窗口
// ========================================================================
wire window_valid;
wire [7:0] win_data_00, win_data_01, win_data_02;
wire [7:0] win_data_10, win_data_11, win_data_12;
wire [7:0] win_data_20, win_data_21, win_data_22;

line_buffer #(
    .H_PIXEL    (640),
    .DATA_WIDTH (8)
) u_line_buffer (
    .clk            (i_clk),
    .rst_n          (i_rst_n),
    .data_in_valid  (i_en),
    .data_in        (i_binary),
    .frame_vsync    (i_vsyn),
    .frame_href     (i_hsyn),
    .window_valid   (window_valid),
    .win_data_00    (win_data_00),
    .win_data_01    (win_data_01),
    .win_data_02    (win_data_02),
    .win_data_10    (win_data_10),
    .win_data_11    (win_data_11),
    .win_data_12    (win_data_12),
    .win_data_20    (win_data_20),
    .win_data_21    (win_data_21),
    .win_data_22    (win_data_22)
);

// ========================================================================
// 腐蚀算法：3x3窗口内全为255时输出255，否则输出0
// ========================================================================
reg erode_result;
reg [7:0] binary_reg;

always@(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        erode_result <= 1'b0;
    end
    else if(window_valid) begin
        // 腐蚀：窗口内全为255时输出1，否则输出0
        erode_result <= (win_data_00 == 8'd255 && win_data_01 == 8'd255 && win_data_02 == 8'd255 &&
                         win_data_10 == 8'd255 && win_data_11 == 8'd255 && win_data_12 == 8'd255 &&
                         win_data_20 == 8'd255 && win_data_21 == 8'd255 && win_data_22 == 8'd255);
    end
end

always@(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        binary_reg <= 8'd0;
    end
    else begin
        binary_reg <= erode_result ? 8'd255 : 8'd0;
    end
end

assign o_en = window_valid;
assign o_binary = binary_reg;

endmodule
