`timescale 1ns / 1ps
//**********************************************************************
// 简化版边框检测模块
// 功能：检测二值化图像中的白色区域，在原始图像上绘制边框
//
// 工作原理：
// 1. 接收外部二值化结果（i_r/i_g/i_b，白色255表示检测到的区域）
// 2. 统计白色像素的边界框坐标
// 3. 在原始彩色图像上绘制边框
//
// 延迟：1个时钟周期
//**********************************************************************

module image_skin_select_simplified #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480,
    parameter R_VALUE = 8'd0,      // 边框颜色R分量（默认绿色）
    parameter G_VALUE = 8'd255,    // 边框颜色G分量
    parameter B_VALUE = 8'd0       // 边框颜色B分量
)(
    input   wire            i_clk,
    input   wire            i_rst_n,

    // 输入时序信号
    input   wire            i_hsyn,
    input   wire            i_vsyn,
    input   wire            i_de,

    // 二值化图像输入（255=白色目标区域，0=黑色背景）
    input   wire [7:0]      i_r,
    input   wire [7:0]      i_g,
    input   wire [7:0]      i_b,

    // 原始RGB图像输入（用于绘制边框）
    input   wire [7:0]      i_r_original,
    input   wire [7:0]      i_g_original,
    input   wire [7:0]      i_b_original,

    // RGB输出（原始图像+边框）
    output  reg  [7:0]      o_r,
    output  reg  [7:0]      o_g,
    output  reg  [7:0]      o_b
);

//=============================================================================
// 像素坐标计数器
//=============================================================================
reg [10:0] x_cnt, y_cnt;
reg vsyn_d;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        x_cnt <= 11'd0;
        y_cnt <= 11'd0;
        vsyn_d <= 1'b0;
    end else begin
        vsyn_d <= i_vsyn;

        if (i_de) begin
            if (x_cnt == H_ACTIVE - 1) begin
                x_cnt <= 11'd0;
                if (y_cnt == V_ACTIVE - 1)
                    y_cnt <= 11'd0;
                else
                    y_cnt <= y_cnt + 1'b1;
            end else begin
                x_cnt <= x_cnt + 1'b1;
            end
        end
    end
end

//=============================================================================
// 帧起始检测
//=============================================================================
wire frame_start = i_vsyn && !vsyn_d;

//=============================================================================
// 白色像素检测（阈值设为200以上认为是白色）
//=============================================================================
wire white_pixel = (i_r > 8'd200) && (i_g > 8'd200) && (i_b > 8'd200);

//=============================================================================
// 边界框计算（在当前帧实时计算）
//=============================================================================
reg [10:0] bbox_x_min, bbox_x_max, bbox_y_min, bbox_y_max;
reg bbox_valid;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        bbox_x_min <= 11'd0;
        bbox_x_max <= 11'd0;
        bbox_y_min <= 11'd0;
        bbox_y_max <= 11'd0;
        bbox_valid <= 1'b0;
    end else begin
        if (frame_start) begin
            // 帧开始时重置边界框
            bbox_x_min <= H_ACTIVE - 1;
            bbox_x_max <= 11'd0;
            bbox_y_min <= V_ACTIVE - 1;
            bbox_y_max <= 11'd0;
            bbox_valid <= 1'b0;
        end else if (i_de && white_pixel) begin
            // 检测到白色像素，更新边界框
            bbox_valid <= 1'b1;
            if (x_cnt < bbox_x_min) bbox_x_min <= x_cnt;
            if (x_cnt > bbox_x_max) bbox_x_max <= x_cnt;
            if (y_cnt < bbox_y_min) bbox_y_min <= y_cnt;
            if (y_cnt > bbox_y_max) bbox_y_max <= y_cnt;
        end
    end
end

//=============================================================================
// 边界框延迟寄存器（用于下一帧显示）
// 这样可以避免边框闪烁，使用上一帧统计的边界框
//=============================================================================
reg [10:0] bbox_x_min_d, bbox_x_max_d, bbox_y_min_d, bbox_y_max_d;
reg bbox_valid_d;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        bbox_x_min_d <= 11'd0;
        bbox_x_max_d <= 11'd0;
        bbox_y_min_d <= 11'd0;
        bbox_y_max_d <= 11'd0;
        bbox_valid_d <= 1'b0;
    end else if (frame_start) begin
        // 帧开始时，锁存上一帧的边界框
        bbox_x_min_d <= bbox_x_min;
        bbox_x_max_d <= bbox_x_max;
        bbox_y_min_d <= bbox_y_min;
        bbox_y_max_d <= bbox_y_max;
        bbox_valid_d <= bbox_valid;
    end
end

//=============================================================================
// 边框绘制判断
// 在边界框的四条边上绘制边框
//=============================================================================
wire on_border_x = ((x_cnt == bbox_x_min_d) || (x_cnt == bbox_x_max_d)) &&
                   (y_cnt >= bbox_y_min_d) && (y_cnt <= bbox_y_max_d);
wire on_border_y = ((y_cnt == bbox_y_min_d) || (y_cnt == bbox_y_max_d)) &&
                   (x_cnt >= bbox_x_min_d) && (x_cnt <= bbox_x_max_d);
wire draw_border = bbox_valid_d && (on_border_x || on_border_y);

//=============================================================================
// 输出控制（1个时钟周期延迟）
// 如果在边框位置，输出边框颜色；否则输出原始图像
//=============================================================================
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_r <= 8'd0;
        o_g <= 8'd0;
        o_b <= 8'd0;
    end else begin
        if (draw_border) begin
            // 绘制边框
            o_r <= R_VALUE;
            o_g <= G_VALUE;
            o_b <= B_VALUE;
        end else begin
            // 显示原始图像
            o_r <= i_r_original;
            o_g <= i_g_original;
            o_b <= i_b_original;
        end
    end
end

endmodule
