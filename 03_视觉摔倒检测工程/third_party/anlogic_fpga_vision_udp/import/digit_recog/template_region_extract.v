//****************************************Copyright (c)***********************************//
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取FPGA & STM32资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           template_region_extract
// Last modified Date:  2025/01/05
// Last Version:        V1.0
// Descriptions:        从视频流中提取32×32区域用于模板匹配
//----------------------------------------------------------------------------------------
// Created by:          Claude
// Created date:        2025/01/05
// Version:             V1.0
// Descriptions:        Extract 32×32 region from video stream for template matching
//                      - Extract from top-left corner (0,0) to (31,31)
//                      - Binarize grayscale to 1-bit (threshold 128)
//                      - Output pixel address and data for real-time matching
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module template_region_extract (
    input   wire        i_clk,
    input   wire        i_rst_n,

    // 视频流输入（来自image_proc_wrapper的灰度输出）
    input   wire        i_hsyn,
    input   wire        i_vsyn,
    input   wire        i_en,
    input   wire [7:0]  i_gray,      // 灰度值（0-255）

    // 提取区域输出（32×32，用于模板匹配）
    output  reg         o_region_valid,  // 区域像素有效信号
    output  reg  [9:0]  o_pixel_addr,    // 像素地址 0-1023 (32×32)
    output  reg         o_pixel_data,    // 像素数据（二值化：1=白, 0=黑）

    // 帧同步信号输出
    output  reg         o_frame_vsync    // 帧同步信号（用于触发匹配）
);

// ===== 参数定义 =====
parameter REGION_X_START = 10'd0;    // 提取区域左上角X坐标
parameter REGION_Y_START = 9'd0;     // 提取区域左上角Y坐标
parameter REGION_WIDTH   = 10'd32;   // 区域宽度
parameter REGION_HEIGHT  = 9'd32;    // 区域高度
parameter BINARY_THRESHOLD = 8'd128; // 二值化阈值

// ===== 像素位置计数器 =====
reg [9:0] h_cnt;  // 水平计数器 0-639
reg [8:0] v_cnt;  // 垂直计数器 0-479

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

// ===== 判断是否在提取区域内 =====
wire in_region_x = (h_cnt >= REGION_X_START) && (h_cnt < REGION_X_START + REGION_WIDTH);
wire in_region_y = (v_cnt >= REGION_Y_START) && (v_cnt < REGION_Y_START + REGION_HEIGHT);
wire in_region = in_region_x && in_region_y && i_en;

// ===== 计算区域内像素地址 =====
wire [4:0] region_x = h_cnt[4:0];  // 0-31 (相对于区域左上角)
wire [4:0] region_y = v_cnt[4:0];  // 0-31
wire [9:0] pixel_addr = {region_y, region_x};  // 行优先地址 = y*32 + x

// ===== 二值化 =====
wire pixel_binary = (i_gray > BINARY_THRESHOLD);

// ===== 输出区域像素 =====
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_region_valid <= 1'b0;
        o_pixel_addr <= 10'd0;
        o_pixel_data <= 1'b0;
    end else if (in_region) begin
        o_region_valid <= 1'b1;
        o_pixel_addr <= pixel_addr;
        o_pixel_data <= pixel_binary;
    end else begin
        o_region_valid <= 1'b0;
    end
end

// ===== 帧同步信号输出 =====
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n)
        o_frame_vsync <= 1'b0;
    else
        o_frame_vsync <= i_vsyn;
end

endmodule
