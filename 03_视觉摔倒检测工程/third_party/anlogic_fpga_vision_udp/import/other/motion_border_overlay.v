`timescale 1ns / 1ps
//********************************************************************************
// Module: motion_border_overlay
// Description: 运动区域边框叠加模块
//
// 功能：
//   - 在原始RGB888图像上叠加运动区域的边框
//   - 运动区域边缘绘制绿色边框
//   - 非边缘区域保持原图
//
// 边框检测：
//   - 检测运动标志的边缘（当前像素有运动，且相邻像素无运动）
//   - 在边缘位置绘制边框色
//********************************************************************************

module motion_border_overlay (
    input   wire        i_clk,
    input   wire        i_rst_n,

    // 输入视频流
    input   wire        i_hsyn,
    input   wire        i_vsyn,
    input   wire        i_en,
    input   wire [31:0] i_data,      // 原始RGB888图像 [31:24]=R, [23:16]=G, [15:8]=B
    input   wire        i_motion,    // 运动检测标志

    // 输出视频流（叠加边框后）
    output  reg         o_hsyn,
    output  reg         o_vsyn,
    output  reg         o_en,
    output  reg  [31:0] o_data
);

// 边框颜色（绿色）
parameter BORDER_R = 8'd0;
parameter BORDER_G = 8'd255;
parameter BORDER_B = 8'd0;

// 延迟链（检测边缘需要3×3窗口）
reg [2:0] motion_shift_reg;  // 水平方向3个像素的motion标志
reg motion_prev_line;        // 上一行的motion标志

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        motion_shift_reg <= 3'b000;
        motion_prev_line <= 1'b0;
    end else if (i_en) begin
        // 移位寄存器：[2]=左, [1]=中, [0]=右
        motion_shift_reg <= {motion_shift_reg[1:0], i_motion};
    end
end

// 边缘检测（简化版：检测水平和垂直边缘）
wire is_border = i_motion && (
    // 左边缘：当前有运动，左侧无运动
    (motion_shift_reg[2] == 1'b0) ||
    // 右边缘：当前有运动，右侧无运动（预测）
    (motion_shift_reg[0] == 1'b0) ||
    // 上边缘：当前有运动，上方无运动（简化，使用延迟）
    (motion_prev_line == 1'b0)
);

// 延迟匹配
reg [31:0] data_d1, data_d2;
reg hsyn_d1, hsyn_d2;
reg vsyn_d1, vsyn_d2;
reg en_d1, en_d2;
reg is_border_d1;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        data_d1 <= 32'd0;
        data_d2 <= 32'd0;
        hsyn_d1 <= 1'b0;
        hsyn_d2 <= 1'b0;
        vsyn_d1 <= 1'b0;
        vsyn_d2 <= 1'b0;
        en_d1 <= 1'b0;
        en_d2 <= 1'b0;
        is_border_d1 <= 1'b0;
    end else begin
        // 2级延迟匹配边缘检测延迟
        data_d1 <= i_data;
        data_d2 <= data_d1;
        hsyn_d1 <= i_hsyn;
        hsyn_d2 <= hsyn_d1;
        vsyn_d1 <= i_vsyn;
        vsyn_d2 <= vsyn_d1;
        en_d1 <= i_en;
        en_d2 <= en_d1;
        is_border_d1 <= is_border;
    end
end

// 输出叠加
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_hsyn <= 1'b0;
        o_vsyn <= 1'b0;
        o_en <= 1'b0;
        o_data <= 32'd0;
    end else begin
        o_hsyn <= hsyn_d2;
        o_vsyn <= vsyn_d2;
        o_en <= en_d2;

        // 边框叠加
        if (is_border_d1) begin
            // 绘制绿色边框
            o_data <= {BORDER_R, BORDER_G, BORDER_B, 8'd0};
        end else begin
            // 保持原图
            o_data <= data_d2;
        end
    end
end

endmodule
