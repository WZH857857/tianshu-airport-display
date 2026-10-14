//****************************************Copyright (c)***********************************//
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取FPGA & STM32资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           template_match_realtime
// Last modified Date:  2025/01/05
// Last Version:        V1.0
// Descriptions:        实时视频流模板匹配模块
//----------------------------------------------------------------------------------------
// Modified by:         Claude
// Modified date:       2025/01/05
// Version:             V1.0
// Descriptions:        Real-time template matching from video stream
//                      - Template uploaded once via UDP
//                      - Real-time comparison with extracted 32×32 region
//                      - Output match result every frame
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module template_match_realtime #(
    parameter H_PIXEL = 32,              // 图像宽度
    parameter V_PIXEL = 32,              // 图像高度
    parameter DEPBIT  = 10               // 地址位宽（1024像素=10位地址）
)(
    //module clock
    input                      clk               ,    // 时钟信号
    input                      rst_n             ,    // 复位信号（低有效）

    //Template image interface (模板图像接口 - UDP上传)
    input                      template_we       ,    // 模板写使能
    input      [DEPBIT-1:0]    template_addr     ,    // 模板地址 (0-1023)
    input                      template_data     ,    // 模板数据 (1-bit: 1=白, 0=黑)

    //Real-time video region interface (实时视频流区域接口)
    input                      region_valid      ,    // 区域像素有效
    input      [DEPBIT-1:0]    region_addr       ,    // 区域像素地址 (0-1023)
    input                      region_data       ,    // 区域像素数据 (1-bit)

    //Frame sync interface (帧同步接口)
    input                      frame_vsync       ,    // 帧同步信号

    //Control interface (控制接口)
    input      [DEPBIT-1:0]    match_threshold   ,    // 匹配阈值（不同像素数量阈值）

    //Result interface (结果接口)
    output reg                 match_result      ,    // 匹配结果 (1=匹配, 0=不匹配)
    output reg                 match_result_valid,    // 匹配结果有效标志（每帧1次）
    output reg [DEPBIT-1:0]    hamming_distance       // 汉明距离（不同像素的数量）
);

//localparam define
localparam TOTAL_PIXELS = H_PIXEL * V_PIXEL;  // 1024

//reg define
reg [DEPBIT-1:0]    diff_count          ;  // 不同像素计数器
reg [DEPBIT-1:0]    pixel_count         ;  // 像素计数器
reg                 frame_vsync_d1      ;  // 帧同步延迟
reg                 comparing           ;  // 比较进行中标志

//wire define
wire                frame_vsync_rise    ;  // 帧同步上升沿
wire                template_pixel      ;  // 模板像素值
wire                pixel_diff          ;  // 像素差异（XOR）

//*****************************************************
//**                    main code
//*****************************************************

assign frame_vsync_rise = frame_vsync & ~frame_vsync_d1;
assign pixel_diff = region_data ^ template_pixel;

//模板图像存储 (Template image RAM)
myram #(
    .WIDTH(1  ),                         // 1-bit per pixel
    .DEPTH(H_PIXEL * V_PIXEL),           // 32×32 = 1024 pixels
    .DEPBIT(DEPBIT)
)u_template_ram(
    //module clock
    .clk(clk),
    //ram interface
    .we(template_we),
    .waddr(template_addr),
    .raddr(region_addr),                 // 实时读取对应位置的模板像素
    .dq_i(template_data),
    .dq_o(template_pixel)
);

//帧同步延迟（用于边沿检测）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        frame_vsync_d1 <= 1'b0;
    else
        frame_vsync_d1 <= frame_vsync;
end

//实时比较状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        diff_count         <= 10'd0;
        pixel_count        <= 10'd0;
        comparing          <= 1'b0;
        match_result       <= 1'b0;
        match_result_valid <= 1'b0;
        hamming_distance   <= 10'd0;
    end else begin
        // 默认清除有效标志
        match_result_valid <= 1'b0;

        if (frame_vsync_rise) begin
            // 帧开始，复位计数器，开始新一轮比较
            diff_count  <= 10'd0;
            pixel_count <= 10'd0;
            comparing   <= 1'b1;
        end else if (comparing && region_valid) begin
            // 逐像素比较
            pixel_count <= pixel_count + 1'b1;

            // 累计不同像素数量
            if (pixel_diff)
                diff_count <= diff_count + 1'b1;

            // 最后一个像素（1023），输出匹配结果
            if (pixel_count == (TOTAL_PIXELS - 1)) begin
                hamming_distance <= diff_count;
                // 判断匹配：不同像素数 <= 阈值
                if (diff_count <= match_threshold)
                    match_result <= 1'b1;    // 匹配
                else
                    match_result <= 1'b0;    // 不匹配

                match_result_valid <= 1'b1;  // 结果有效
                comparing <= 1'b0;            // 本帧比较结束
            end
        end
    end
end

endmodule
