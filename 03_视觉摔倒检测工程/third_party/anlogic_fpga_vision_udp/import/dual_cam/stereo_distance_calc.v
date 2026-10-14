//****************************************Copyright (c)***********************************//
// File name:           stereo_distance_calc
// Last modified Date:  2025/01/05
// Last Version:        V1.0
// Descriptions:        双目测距计算模块
//----------------------------------------------------------------------------------------
// Created by:          Claude
// Created date:        2025/01/05
// Version:             V1.0
// Descriptions:        Stereo distance calculation based on disparity
//                      Formula: Distance = (Baseline × Focal_Length) / Disparity
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module stereo_distance_calc #(
    parameter BASELINE     = 120,   // 基线距离（mm）
    parameter FOCAL_LENGTH = 350    // 焦距（像素）
)(
    input wire          clk,
    input wire          rst_n,

    // 边界框中心坐标输入
    input wire [10:0]   left_coor_x,      // 左目边界框中心X坐标
    input wire [10:0]   right_coor_x,     // 右目边界框中心X坐标
    input wire          coor_valid,       // 坐标有效信号（帧同步）

    // 距离输出
    output reg [7:0]    distance,         // 距离（cm，0-255）
    output reg          distance_valid    // 距离有效标志
);

//=============================================================================
// 内部信号
//=============================================================================
reg signed [11:0] disparity;       // 视差（带符号，左-右）
reg signed [11:0] disparity_d1;    // 打拍
reg [31:0]        distance_mm;     // 距离（mm）
reg               calc_stage1;     // 计算阶段1：计算视差
reg               calc_stage2;     // 计算阶段2：计算mm距离
reg               calc_stage3;     // 计算阶段3：转换为cm

// 常量：BASELINE * FOCAL_LENGTH = 120 * 350 = 42000
localparam BASELINE_FOCAL = BASELINE * FOCAL_LENGTH;

//=============================================================================
// 流水线第一级：计算视差
//=============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        disparity       <= 12'd0;
        calc_stage1     <= 1'b0;
    end else if (coor_valid) begin
        // 计算视差：左目中心X - 右目中心X
        disparity <= $signed({1'b0, left_coor_x}) - $signed({1'b0, right_coor_x});
        calc_stage1 <= 1'b1;
    end else begin
        calc_stage1 <= 1'b0;
    end
end

//=============================================================================
// 流水线第二级：计算距离（mm）
//=============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        disparity_d1    <= 12'd0;
        distance_mm     <= 32'd0;
        calc_stage2     <= 1'b0;
    end else if (calc_stage1) begin
        disparity_d1 <= disparity;
        // 距离计算：D(mm) = (BASELINE * FOCAL_LENGTH) / disparity
        if (disparity > 12'sd16 && disparity < 12'sd640) begin
            distance_mm <= BASELINE_FOCAL / disparity;
            calc_stage2 <= 1'b1;
        end else begin
            distance_mm <= 32'd0;
            calc_stage2 <= 1'b0;
        end
    end else begin
        calc_stage2 <= 1'b0;
    end
end

//=============================================================================
// 流水线第三级：转换为cm并输出
//=============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        distance        <= 8'd0;
        distance_valid  <= 1'b0;
        calc_stage3     <= 1'b0;
    end else if (calc_stage2) begin
        // 转换为cm（除以10）
        if (distance_mm >= 32'd2550) begin
            distance <= 8'd255;
        end else if (distance_mm >= 32'd10) begin
            // 用乘法逼近除以10：x/10 ≈ (x * 205) >> 11
            distance <= ((distance_mm * 205) >> 11);
        end else begin
            distance <= 8'd0;
        end
        distance_valid <= 1'b1;
        calc_stage3 <= 1'b1;
    end else begin
        distance_valid <= 1'b0;
        calc_stage3 <= 1'b0;
    end
end

endmodule
