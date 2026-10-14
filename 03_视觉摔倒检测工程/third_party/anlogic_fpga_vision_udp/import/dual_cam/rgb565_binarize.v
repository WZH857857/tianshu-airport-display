//****************************************Copyright (c)***********************************//
// File name:           rgb565_binarize
// Last modified Date:  2025/01/05
// Last Version:        V1.0
// Descriptions:        RGB565转灰度并二值化（用于双目测距）
//----------------------------------------------------------------------------------------
// Created by:          Claude
// Created date:        2025/01/05
// Version:             V1.0
// Descriptions:        Convert RGB565 to grayscale and binarize
//                      - Input: RGB565 format [15:0] = {R[4:0], G[5:0], B[4:0]}
//                      - Output: Binary 8-bit (0x00=black, 0xFF=white)
//                      - Also output 1-bit for bounding_box
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module rgb565_binarize (
    input wire          clk,
    input wire          rst_n,

    // 控制信号
    input wire          enable,            // 使能信号（0=bypass, 1=二值化）
    input wire [7:0]    threshold_r_low,   // R通道低阈值
    input wire [7:0]    threshold_r_high,  // R通道高阈值
    input wire [7:0]    threshold_g_low,   // G通道低阈值
    input wire [7:0]    threshold_g_high,  // G通道高阈值
    input wire [7:0]    threshold_b_low,   // B通道低阈值
    input wire [7:0]    threshold_b_high,  // B通道高阈值

    // 视频流输入
    input wire          vsync_i,
    input wire          hsync_i,
    input wire          data_en_i,
    input wire [15:0]   rgb565_i,          // RGB565输入

    // 二值化输出
    output reg          vsync_o,
    output reg          hsync_o,
    output reg          data_en_o,
    output reg [7:0]    binary_o,          // 8位二值化输出 (0x00/0xFF)
    output reg          binary_bit_o       // 1位二值化输出 (0/1)
);

//=============================================================================
// 内部信号
//=============================================================================
reg [7:0] gray;

// RGB565提取
wire [4:0] r5 = rgb565_i[15:11];
wire [5:0] g6 = rgb565_i[10:5];
wire [4:0] b5 = rgb565_i[4:0];

// 扩展到8位
wire [7:0] r8 = {r5, r5[4:2]};
wire [7:0] g8 = {g6, g6[5:4]};
wire [7:0] b8 = {b5, b5[4:2]};

//=============================================================================
// 同步信号（1级延迟）
//=============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vsync_o   <= 1'b0;
        hsync_o   <= 1'b0;
        data_en_o <= 1'b0;
    end else begin
        vsync_o   <= vsync_i;
        hsync_o   <= hsync_i;
        data_en_o <= data_en_i;
    end
end

//=============================================================================
// RGB颜色阈值二值化
//=============================================================================
// RGB颜色匹配判断：三个通道都在阈值范围内则为白色
wire rgb_in_range = (r8 >= threshold_r_low) && (r8 <= threshold_r_high) &&
                    (g8 >= threshold_g_low) && (g8 <= threshold_g_high) &&
                    (b8 >= threshold_b_low) && (b8 <= threshold_b_high);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        gray         <= 8'd0;
        binary_o     <= 8'd0;
        binary_bit_o <= 1'b0;
    end else if (data_en_i) begin
        // 灰度转换（用于未使能时的显示）
        // Gray = 0.299*R + 0.587*G + 0.114*B
        // 近似：Gray = (R*77 + G*150 + B*29) >> 8
        gray <= ((r8 * 77) + (g8 * 150) + (b8 * 29)) >> 8;

        // RGB颜色阈值二值化
        if (enable) begin
            if (rgb_in_range) begin
                binary_o     <= 8'hFF;  // 白色
                binary_bit_o <= 1'b1;
            end else begin
                binary_o     <= 8'h00;  // 黑色
                binary_bit_o <= 1'b0;
            end
        end else begin
            // 未使能时输出灰度图
            binary_o     <= gray;
            binary_bit_o <= (gray > 8'd128) ? 1'b1 : 1'b0;
        end
    end
end

endmodule
