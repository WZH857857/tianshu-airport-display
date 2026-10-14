`timescale 1ns / 1ps
//**********************************************************************
// Description: Sobel边缘检测算法 - 灰度图版本
// 使用 line_buffer 生成3x3窗口
//
// Sobel X核：              Sobel Y核：
//   -1   0   1              -1  -2  -1
//   -2   0   2               0   0   0
//   -1   0   1               1   2   1
//
// 计算：Gx = (右列和 - 左列和)
//       Gy = (下行和 - 上行和)
//       Edge = |Gx| + |Gy|
//*********************************************************************/

module image_sobel_gray (
    input   wire                i_clk,
    input   wire                i_rst_n,

    input   wire                i_hsyn,
    input   wire                i_vsyn,
    input   wire                i_en,
    input   wire [7:0]          i_gray,         // 灰度输入
    input   wire [7:0]          threshold,      // 边缘阈值

    output  wire                o_hs,
    output  wire                o_vs,
    output  wire                o_en,
    output  wire [7:0]          o_gray          // 边缘二值图输出
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

// ===== Sobel X 梯度计算 =====
// Gx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20)
reg [11:0] sum_x_left;   // 左列和
reg [11:0] sum_x_right;  // 右列和
reg [11:0] sobel_x;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        sum_x_left <= 12'd0;
        sum_x_right <= 12'd0;
    end else begin
        // 左列：p00 + 2*p10 + p20
        sum_x_left <= win_00 + win_10 + win_10 + win_20;
        // 右列：p02 + 2*p12 + p22
        sum_x_right <= win_02 + win_12 + win_12 + win_22;
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        sobel_x <= 12'd0;
    end else begin
        if (sum_x_right >= sum_x_left) begin
            sobel_x <= sum_x_right - sum_x_left;
        end else begin
            sobel_x <= sum_x_left - sum_x_right;  // 取绝对值
        end
    end
end

// ===== Sobel Y 梯度计算 =====
// Gy = (p20 + 2*p21 + p22) - (p00 + 2*p01 + p02)
reg [11:0] sum_y_top;     // 上行和
reg [11:0] sum_y_bottom;  // 下行和
reg [11:0] sobel_y;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        sum_y_top <= 12'd0;
        sum_y_bottom <= 12'd0;
    end else begin
        // 上行：p00 + 2*p01 + p02
        sum_y_top <= win_00 + win_01 + win_01 + win_02;
        // 下行：p20 + 2*p21 + p22
        sum_y_bottom <= win_20 + win_21 + win_21 + win_22;
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        sobel_y <= 12'd0;
    end else begin
        if (sum_y_bottom >= sum_y_top) begin
            sobel_y <= sum_y_bottom - sum_y_top;
        end else begin
            sobel_y <= sum_y_top - sum_y_bottom;  // 取绝对值
        end
    end
end

// ===== 边缘强度合成与阈值化 =====
reg [11:0] edge_magnitude;
reg [7:0] edge_out;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        edge_magnitude <= 12'd0;
        edge_out <= 8'd0;
    end else begin
        // Edge = |Gx| + |Gy|
        edge_magnitude <= sobel_x + sobel_y;

        // 阈值化为二值图
        if (edge_magnitude > {4'd0, threshold}) begin
            edge_out <= 8'd255;  // 边缘
        end else begin
            edge_out <= 8'd0;    // 非边缘
        end
    end
end

// ===== 输出赋值 =====
assign o_hs = hsyn_d[2];
assign o_vs = vsyn_d[2];
assign o_en = en_d[2];
assign o_gray = edge_out;

endmodule
