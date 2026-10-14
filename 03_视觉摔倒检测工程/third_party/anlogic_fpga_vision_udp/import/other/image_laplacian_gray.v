`timescale 1ns / 1ps
//**********************************************************************
// Description: 拉普拉斯锐化算法 - 灰度图版本
// 使用 line_buffer 生成3x3窗口
//
// Laplacian 核：
//    0  -1   0
//   -1   5  -1
//    0  -1   0
//
// 计算：output = 5*center - (up + down + left + right)
//*********************************************************************/

module image_laplacian_gray (
    input   wire                i_clk,
    input   wire                i_rst_n,

    input   wire                i_hsyn,
    input   wire                i_vsyn,
    input   wire                i_en,
    input   wire [7:0]          i_gray,     // 灰度输入

    output  wire                o_hs,
    output  wire                o_vs,
    output  wire                o_en,
    output  wire [7:0]          o_gray      // 灰度输出
);

// ===== 3x3窗口生成 =====
wire        window_valid;
wire [7:0]  win_00, win_01, win_02;
wire [7:0]  win_10, win_11, win_12;
wire [7:0]  win_20, win_21, win_22;

line_buffer #(
    .H_PIXEL    (640),
    .DATA_WIDTH (8)
) u_line_buffer (
    .clk            (i_clk),
    .rst_n          (i_rst_n),
    .data_in_valid  (i_en),
    .data_in        (i_gray),
    .frame_vsync    (i_vsyn),
    .frame_href     (i_hsyn),
    .window_valid   (window_valid),
    .win_data_00    (win_00),
    .win_data_01    (win_01),
    .win_data_02    (win_02),
    .win_data_10    (win_10),
    .win_data_11    (win_11),
    .win_data_12    (win_12),
    .win_data_20    (win_20),
    .win_data_21    (win_21),
    .win_data_22    (win_22)
);

// ===== 时序信号延迟匹配 =====
// line_buffer 有多级延迟，需要延迟时序信号
reg [2:0] hsyn_d;
reg [2:0] vsyn_d;
reg [2:0] en_d;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        hsyn_d <= 3'b0;
        vsyn_d <= 3'b0;
        en_d <= 3'b0;
    end else begin
        hsyn_d <= {hsyn_d[1:0], i_hsyn};
        vsyn_d <= {vsyn_d[1:0], i_vsyn};
        en_d <= {en_d[1:0], window_valid};
    end
end

// ===== 拉普拉斯计算 =====
// Laplacian = 5*center - (up + down + left + right)
reg [11:0] sum_neighbor;  // 4个邻居的和
reg [11:0] center_x5;     // 中心像素×5
reg [11:0] laplacian;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        sum_neighbor <= 12'd0;
        center_x5 <= 12'd0;
    end else begin
        // 上下左右四个邻居的和
        sum_neighbor <= win_01 + win_21 + win_10 + win_12;
        // 中心像素×5 = center×4 + center
        center_x5 <= {2'd0, win_11, 2'd0} + {4'd0, win_11};
    end
end

// 做差并饱和处理
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        laplacian <= 12'd0;
    end else begin
        if (center_x5 >= sum_neighbor) begin
            laplacian <= center_x5 - sum_neighbor;
        end else begin
            laplacian <= 12'd0;  // 负值钳位到0
        end
    end
end

// ===== 输出饱和处理 =====
reg [7:0] lap_out;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        lap_out <= 8'd0;
    end else begin
        if (laplacian[11:8] > 0) begin
            lap_out <= 8'd255;  // 溢出钳位到255
        end else begin
            lap_out <= laplacian[7:0];
        end
    end
end

// ===== 输出赋值 =====
assign o_hs = hsyn_d[2];
assign o_vs = vsyn_d[2];
assign o_en = en_d[2];
assign o_gray = lap_out;

endmodule
