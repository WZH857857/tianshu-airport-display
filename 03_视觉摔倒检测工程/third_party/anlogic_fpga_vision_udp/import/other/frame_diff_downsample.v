`timescale 1ns / 1ps
//********************************************************************************
// Module: frame_diff_downsample
// Description: 帧差法模块（4×4降采样，使用BRAM存储前一帧）
//
// 功能：
//   - 输入：灰度图像（640×480，8bit）
//   - 降采样：4×4取左上角像素 → 160×120
//   - 帧差计算：|当前帧 - 前一帧| (绝对值)
//   - 输出：差值图像（灰度）
//
// 资源：
//   - BRAM: 19200×8bit（使用推理的BRAM，约5个BRAM 32k）
//********************************************************************************

module frame_diff_downsample (
    input   wire        i_clk,
    input   wire        i_rst_n,
    input   wire        i_hsyn,
    input   wire        i_vsyn,
    input   wire        i_en,
    input   wire [7:0]  i_gray,      // 输入灰度值（来自image_proc_wrapper）

    output  reg         o_hs,
    output  reg         o_vs,
    output  reg         o_en,
    output  reg  [7:0]  o_diff,      // 输出差值（绝对值）
    output  reg         o_motion     // 运动检测标志（1=有运动，0=无运动）
);

// 像素位置计数器（640×480）
reg [9:0] h_cnt;  // 0-639
reg [8:0] v_cnt;  // 0-479

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        h_cnt <= 10'd0;
        v_cnt <= 9'd0;
    end else if (i_en) begin
        if (h_cnt == 10'd639) begin
            h_cnt <= 10'd0;
            if (v_cnt == 9'd479)
                v_cnt <= 9'd0;
            else
                v_cnt <= v_cnt + 1'b1;
        end else begin
            h_cnt <= h_cnt + 1'b1;
        end
    end else if (i_hsyn) begin
        h_cnt <= 10'd0;
    end
end

// 降采样判断（每4×4取左上角）
wire is_sample_point = (h_cnt[1:0] == 2'b00) && (v_cnt[1:0] == 2'b00) && i_en;

// 降采样地址（160×120）
wire [14:0] ds_addr = {v_cnt[8:2], h_cnt[9:2]};  // 7位v + 8位h = 15位

// BRAM读写
wire [7:0] prev_gray;

bram_19200x8 u_prev_frame_buffer (
    .clk    (i_clk),
    .we     (is_sample_point),      // 采样点才写入
    .addr   (ds_addr),
    .din    (i_gray),               // 写入当前灰度值
    .dout   (prev_gray)             // 读出前一帧灰度值
);

// 帧差计算（绝对值）
wire signed [8:0] diff_signed = $signed({1'b0, i_gray}) - $signed({1'b0, prev_gray});
wire [7:0] diff_abs = diff_signed[8] ? (~diff_signed[7:0] + 1'b1) : diff_signed[7:0];

// 差值增强（×2，增强对比度）
wire [8:0] diff_enhanced = {diff_abs, 1'b0};  // 左移1位 = ×2
wire [7:0] diff_final = (diff_enhanced > 9'd255) ? 8'd255 : diff_enhanced[7:0];

// 运动检测阈值（可调整）
parameter MOTION_THRESHOLD = 8'd30;  // 差值>30认为有运动
wire is_motion = (diff_final > MOTION_THRESHOLD);

// 采样点延迟（匹配BRAM读延迟）
reg is_sample_point_d1;
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n)
        is_sample_point_d1 <= 1'b0;
    else
        is_sample_point_d1 <= is_sample_point;
end

// 差值寄存器和运动标志寄存器（保持4×4周期）
reg [7:0] diff_hold;
reg motion_hold;
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        diff_hold <= 8'd0;
        motion_hold <= 1'b0;
    end else if (is_sample_point_d1) begin  // 采样点更新（延迟后）
        diff_hold <= diff_final;
        motion_hold <= is_motion;
    end
end

// 输出时序（延迟1个周期匹配BRAM读延迟）
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_hs <= 1'b0;
        o_vs <= 1'b0;
        o_en <= 1'b0;
        o_diff <= 8'd0;
        o_motion <= 1'b0;
    end else begin
        o_hs <= i_hsyn;
        o_vs <= i_vsyn;
        o_en <= i_en;
        // 最近邻插值：保持采样点的值，直到下一个采样点
        o_diff <= diff_hold;
        o_motion <= motion_hold;
    end
end

endmodule
