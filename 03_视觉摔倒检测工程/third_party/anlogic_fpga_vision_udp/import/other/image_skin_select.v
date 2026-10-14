`timescale 1ns / 1ps
//**********************************************************************
// 肤色识别模块 - 基于椭圆模型
// 算法：在YCbCr空间中，肤色像素在CrCb平面近似椭圆分布
// 椭圆参数：中心(113, 155.6), 半轴(23.4, 15.2), 旋转43度
// 延迟：6级流水线（匹配系统延迟）
//**********************************************************************
module image_skin_select
(
	input   wire				i_clk,
	input   wire				i_rst_n,

	input	wire				i_hsyn,
	input	wire				i_vsyn,
	input	wire				i_de,
	input	wire [7:0]			i_r,
	input	wire [7:0] 			i_g,
	input	wire [7:0] 			i_b,
	input	wire [7:0]			i_r_original,
	input	wire [7:0] 			i_g_original,
	input	wire [7:0] 			i_b_original,
	input	wire [10:0]			i_x_pos,
	input	wire [10:0] 		i_y_pos,
	output  reg  [7:0]			o_r,
	output  reg  [7:0]			o_g,
	output  reg  [7:0]			o_b,
	output  reg					o_de,
	output  reg					o_hsyn,
	output  reg					o_vsyn
);

parameter H_ACTIVE = 640;
parameter V_ACTIVE = 480;

/******************************************************************************
 * 椭圆肤色模型参数（可调节）
 *
 * 基本原理：
 * 在YCbCr颜色空间中，肤色像素在CrCb平面上近似呈椭圆分布。
 * 白色物体的Cb≈128, Cr≈128（中性色），肤色的Cr值明显高于128。
 *
 * 椭圆参数说明：
 * - CB_CENTER, CR_CENTER: 椭圆中心坐标
 * - A_SQ, B_SQ: 椭圆半轴的平方（a^2, b^2）
 * - COS_43, SIN_43: 旋转角度的三角函数值（*256定点化）
 *
 * 调参指南：
 ******************************************************************************/

//-----------------------------------------------------------------------------
// 参数1: 椭圆中心位置（最重要！）
//-----------------------------------------------------------------------------
// 调节原则：
// - CB_CENTER: 控制蓝色分量中心（范围通常在100-120）
//   * 增大 → 识别偏蓝的肤色
//   * 减小 → 识别偏黄的肤色
//
// - CR_CENTER: 控制红色分量中心（范围通常在145-165）
//   * 增大 → 识别更红的肤色
//   * 减小 → 过滤红色，减少误检
//
// 常见问题解决：
// ✗ 白色误检严重 → CR_CENTER增大（远离128），CB_CENTER保持或略减
// ✗ 肤色漏检     → CR_CENTER减小，CB_CENTER增大
// ✗ 红色物体误检 → CR_CENTER减小
// ✗ 黄色物体误检 → CB_CENTER增大
//
// 推荐配置：
// - 标准亚洲肤色：CB_CENTER=113, CR_CENTER=156
// - 减少白色误检：CB_CENTER=110, CR_CENTER=160  ← 当前使用
// - 浅色肤色：    CB_CENTER=115, CR_CENTER=150
// - 深色肤色：    CB_CENTER=108, CR_CENTER=158

localparam signed [8:0] CB_CENTER = 9'd110;  // Cb中心（调整范围: 100-120）
localparam signed [8:0] CR_CENTER = 9'd160;  // Cr中心（调整范围: 145-165）

//-----------------------------------------------------------------------------
// 参数2: 椭圆大小（半轴长度）
//-----------------------------------------------------------------------------
// 调节原则：
// - A_SQ = a^2: Cr方向半轴的平方
//   * 减小 → 收紧椭圆，减少误检（推荐）
//   * 增大 → 扩大椭圆，减少漏检
//
// - B_SQ = b^2: Cb方向半轴的平方
//   * 减小 → 收紧椭圆，减少误检（推荐）
//   * 增大 → 扩大椭圆，减少漏检
//
// 常见配置：
// - 标准大小：   a=23 (A_SQ=529), b=15 (B_SQ=225)
// - 严格检测：   a=20 (A_SQ=400), b=13 (B_SQ=169)  ← 当前使用
// - 宽松检测：   a=26 (A_SQ=676), b=17 (B_SQ=289)
// - 极严格：     a=18 (A_SQ=324), b=11 (B_SQ=121)

localparam [15:0] A_SQ = 16'd400;    // a=20, Cr方向半轴（调整范围: 324-676）
localparam [15:0] B_SQ = 16'd169;    // b=13, Cb方向半轴（调整范围: 121-289）

//-----------------------------------------------------------------------------
// 参数3: 椭圆旋转角度（一般不需要调整）
//-----------------------------------------------------------------------------
// 旋转43度是基于统计数据得出的最优角度
// 如需调整，参考下表：
// - 40度: COS=196, SIN=164
// - 43度: COS=187, SIN=175  ← 推荐
// - 45度: COS=181, SIN=181
// - 48度: COS=171, SIN=188

localparam [15:0] COS_43 = 16'd187;  // cos(43°)*256（一般不改）
localparam [15:0] SIN_43 = 16'd175;  // sin(43°)*256（一般不改）

/******************************************************************************
 * 快速调参步骤：
 *
 * 问题: 白色/浅色物体误检严重
 * 解决: 1. CR_CENTER 从 156 → 160 (提高5-10)
 *       2. CB_CENTER 从 113 → 110 (降低3-5)
 *       3. A_SQ 从 529 → 400 (收紧)
 *       4. B_SQ 从 225 → 169 (收紧)
 *
 * 问题: 肤色区域检测不完整（漏检）
 * 解决: 1. A_SQ 和 B_SQ 都增大约20%
 *       2. CR_CENTER 减小5左右
 *
 * 问题: 红色物体（如红色衣服）被误识别
 * 解决: 1. CR_CENTER 减小10左右
 *       2. A_SQ 减小约100
 *
 * 问题: 黄色/棕色物体被误识别
 * 解决: 1. CB_CENTER 增大5左右
 *       2. B_SQ 减小约50
 ******************************************************************************/

//=============================================================================
// Stage 0: 输入缓存和坐标计数
//=============================================================================
reg [7:0] rgb_s0[2:0];
reg [3:0] sync_s0;
reg [10:0] x_cnt, y_cnt;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        rgb_s0[0] <= 8'd0; rgb_s0[1] <= 8'd0; rgb_s0[2] <= 8'd0;
        sync_s0 <= 4'd0;
        x_cnt <= 11'd0; y_cnt <= 11'd0;
    end else begin
        rgb_s0[0] <= i_r; rgb_s0[1] <= i_g; rgb_s0[2] <= i_b;
        sync_s0 <= {i_hsyn, i_vsyn, i_de, 1'b0};

        if (i_de) begin
            if (x_cnt == H_ACTIVE - 1) begin
                x_cnt <= 11'd0;
                y_cnt <= (y_cnt == V_ACTIVE - 1) ? 11'd0 : y_cnt + 1'b1;
            end else
                x_cnt <= x_cnt + 1'b1;
        end
    end
end

//=============================================================================
// Stage 1: RGB→YCbCr 乘法
//=============================================================================
reg [15:0] ycbcr_s1[8:0];
reg [7:0] rgb_s1[2:0];
reg [3:0] sync_s1;
reg [10:0] xy_s1[1:0];

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        ycbcr_s1[0] <= 16'd0; ycbcr_s1[1] <= 16'd0; ycbcr_s1[2] <= 16'd0;
        ycbcr_s1[3] <= 16'd0; ycbcr_s1[4] <= 16'd0; ycbcr_s1[5] <= 16'd0;
        ycbcr_s1[6] <= 16'd0; ycbcr_s1[7] <= 16'd0; ycbcr_s1[8] <= 16'd0;
        rgb_s1[0] <= 8'd0; rgb_s1[1] <= 8'd0; rgb_s1[2] <= 8'd0;
        sync_s1 <= 4'd0; xy_s1[0] <= 11'd0; xy_s1[1] <= 11'd0;
    end else begin
        ycbcr_s1[0] <= 66  * rgb_s0[0]; ycbcr_s1[1] <= 129 * rgb_s0[1]; ycbcr_s1[2] <= 25  * rgb_s0[2];
        ycbcr_s1[3] <= 38  * rgb_s0[0]; ycbcr_s1[4] <= 74  * rgb_s0[1]; ycbcr_s1[5] <= 112 * rgb_s0[2];
        ycbcr_s1[6] <= 112 * rgb_s0[0]; ycbcr_s1[7] <= 94  * rgb_s0[1]; ycbcr_s1[8] <= 18  * rgb_s0[2];

        rgb_s1[0] <= rgb_s0[0]; rgb_s1[1] <= rgb_s0[1]; rgb_s1[2] <= rgb_s0[2];
        sync_s1 <= sync_s0; xy_s1[0] <= x_cnt; xy_s1[1] <= y_cnt;
    end
end

//=============================================================================
// Stage 2: YCbCr 加法和计算差值
//
// 说明：将RGB转换为YCbCr后，计算Cb和Cr与椭圆中心的偏移量
// 这个偏移量将用于后续的椭圆内部判断
//=============================================================================
reg [15:0] cb_val, cr_val;
reg signed [8:0] cb_diff, cr_diff;
reg [7:0] rgb_s2[2:0];
reg [3:0] sync_s2;
reg [10:0] xy_s2[1:0];

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        cb_val <= 16'd0; cr_val <= 16'd0;
        cb_diff <= 9'd0; cr_diff <= 9'd0;
        rgb_s2[0] <= 8'd0; rgb_s2[1] <= 8'd0; rgb_s2[2] <= 8'd0;
        sync_s2 <= 4'd0; xy_s2[0] <= 11'd0; xy_s2[1] <= 11'd0;
    end else begin
        cb_val <= ycbcr_s1[5] - ycbcr_s1[3] - ycbcr_s1[4] + 16'd32768;
        cr_val <= ycbcr_s1[6] - ycbcr_s1[7] - ycbcr_s1[8] + 16'd32768;

        // 计算与椭圆中心的差值
        cb_diff <= $signed({1'b0, cb_val[15:8]}) - CB_CENTER;
        cr_diff <= $signed({1'b0, cr_val[15:8]}) - CR_CENTER;

        rgb_s2[0] <= rgb_s1[0]; rgb_s2[1] <= rgb_s1[1]; rgb_s2[2] <= rgb_s1[2];
        sync_s2 <= sync_s1; xy_s2[0] <= xy_s1[0]; xy_s2[1] <= xy_s1[1];
    end
end

//=============================================================================
// Stage 3: 旋转坐标系（计算 u, v）
//
// 说明：由于椭圆旋转了43度，需要将坐标系也旋转43度
// 旋转公式：
//   u = (Cb-CB_CENTER)*cos43 + (Cr-CR_CENTER)*sin43
//   v = (Cr-CR_CENTER)*cos43 - (Cb-CB_CENTER)*sin43
// 旋转后，椭圆的主轴与坐标轴对齐，便于判断
//=============================================================================
reg signed [24:0] u_mul1, u_mul2, v_mul1, v_mul2;
reg [7:0] rgb_s3[2:0];
reg [3:0] sync_s3;
reg [10:0] xy_s3[1:0];

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        u_mul1 <= 25'd0; u_mul2 <= 25'd0;
        v_mul1 <= 25'd0; v_mul2 <= 25'd0;
        rgb_s3[0] <= 8'd0; rgb_s3[1] <= 8'd0; rgb_s3[2] <= 8'd0;
        sync_s3 <= 4'd0; xy_s3[0] <= 11'd0; xy_s3[1] <= 11'd0;
    end else begin
        u_mul1 <= cb_diff * $signed({1'b0, COS_43});
        u_mul2 <= cr_diff * $signed({1'b0, SIN_43});
        v_mul1 <= cr_diff * $signed({1'b0, COS_43});
        v_mul2 <= cb_diff * $signed({1'b0, SIN_43});

        rgb_s3[0] <= rgb_s2[0]; rgb_s3[1] <= rgb_s2[1]; rgb_s3[2] <= rgb_s2[2];
        sync_s3 <= sync_s2; xy_s3[0] <= xy_s2[0]; xy_s3[1] <= xy_s2[1];
    end
end

//=============================================================================
// Stage 4: 计算 u, v（除以256并取整）
//=============================================================================
reg signed [16:0] u_val, v_val;
reg [7:0] rgb_s4[2:0];
reg [3:0] sync_s4;
reg [10:0] xy_s4[1:0];

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        u_val <= 17'd0; v_val <= 17'd0;
        rgb_s4[0] <= 8'd0; rgb_s4[1] <= 8'd0; rgb_s4[2] <= 8'd0;
        sync_s4 <= 4'd0; xy_s4[0] <= 11'd0; xy_s4[1] <= 11'd0;
    end else begin
        u_val <= (u_mul1 + u_mul2) >>> 8;  // 右移8位（除以256）
        v_val <= (v_mul1 - v_mul2) >>> 8;

        rgb_s4[0] <= rgb_s3[0]; rgb_s4[1] <= rgb_s3[1]; rgb_s4[2] <= rgb_s3[2];
        sync_s4 <= sync_s3; xy_s4[0] <= xy_s3[0]; xy_s4[1] <= xy_s3[1];
    end
end

//=============================================================================
// Stage 5: 计算 u^2 和 v^2
//=============================================================================
reg [31:0] u_sq, v_sq;
reg [7:0] rgb_s5[2:0];
reg [3:0] sync_s5;
reg [10:0] xy_s5[1:0];

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        u_sq <= 32'd0; v_sq <= 32'd0;
        rgb_s5[0] <= 8'd0; rgb_s5[1] <= 8'd0; rgb_s5[2] <= 8'd0;
        sync_s5 <= 4'd0; xy_s5[0] <= 11'd0; xy_s5[1] <= 11'd0;
    end else begin
        u_sq <= u_val * u_val;
        v_sq <= v_val * v_val;

        rgb_s5[0] <= rgb_s4[0]; rgb_s5[1] <= rgb_s4[1]; rgb_s5[2] <= rgb_s4[2];
        sync_s5 <= sync_s4; xy_s5[0] <= xy_s4[0]; xy_s5[1] <= xy_s4[1];
    end
end

//=============================================================================
// Stage 6: 椭圆判断和边界检测
//
// 椭圆方程：(u^2/a^2) + (v^2/b^2) <= 1
// 为避免除法，变形为：u^2*b^2 + v^2*a^2 <= a^2*b^2
//
// 工作原理：
// - 如果像素点在椭圆内（包括边界），则判定为肤色
// - 椭圆的中心、大小、旋转角度由前面的参数决定
// - 白色物体(Cb≈128, Cr≈128)因为远离椭圆中心而被排除
//=============================================================================
reg skin_pixel;
reg [31:0] ellipse_left;
reg [7:0] rgb_s6[2:0];
reg [3:0] sync_s6;
reg [10:0] xy_s6[1:0];

localparam [31:0] ELLIPSE_THRESHOLD = A_SQ * B_SQ;  // 529 * 225 = 119025

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        skin_pixel <= 1'b0;
        ellipse_left <= 32'd0;
        rgb_s6[0] <= 8'd0; rgb_s6[1] <= 8'd0; rgb_s6[2] <= 8'd0;
        sync_s6 <= 4'd0; xy_s6[0] <= 11'd0; xy_s6[1] <= 11'd0;
    end else begin
        ellipse_left <= u_sq * B_SQ + v_sq * A_SQ;
        skin_pixel <= (ellipse_left <= ELLIPSE_THRESHOLD);

        rgb_s6[0] <= rgb_s5[0]; rgb_s6[1] <= rgb_s5[1]; rgb_s6[2] <= rgb_s5[2];
        sync_s6 <= sync_s5; xy_s6[0] <= xy_s5[0]; xy_s6[1] <= xy_s5[1];
    end
end

//=============================================================================
// 边界框计算
//=============================================================================
reg [10:0] bbox_x_min, bbox_x_max, bbox_y_min, bbox_y_max;
reg bbox_valid;
reg vsyn_d;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n)
        vsyn_d <= 1'b0;
    else
        vsyn_d <= sync_s6[2];
end

wire frame_start = sync_s6[2] && !vsyn_d;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        bbox_x_min <= 11'd0; bbox_x_max <= 11'd0;
        bbox_y_min <= 11'd0; bbox_y_max <= 11'd0;
        bbox_valid <= 1'b0;
    end else begin
        if (frame_start) begin
            bbox_x_min <= H_ACTIVE - 1; bbox_x_max <= 11'd0;
            bbox_y_min <= V_ACTIVE - 1; bbox_y_max <= 11'd0;
            bbox_valid <= 1'b0;
        end else if (sync_s6[1] && skin_pixel) begin
            bbox_valid <= 1'b1;
            if (xy_s6[0] < bbox_x_min) bbox_x_min <= xy_s6[0];
            if (xy_s6[0] > bbox_x_max) bbox_x_max <= xy_s6[0];
            if (xy_s6[1] < bbox_y_min) bbox_y_min <= xy_s6[1];
            if (xy_s6[1] > bbox_y_max) bbox_y_max <= xy_s6[1];
        end
    end
end

reg [10:0] bbox_x_min_d, bbox_x_max_d, bbox_y_min_d, bbox_y_max_d;
reg bbox_valid_d;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        bbox_x_min_d <= 11'd0; bbox_x_max_d <= 11'd0;
        bbox_y_min_d <= 11'd0; bbox_y_max_d <= 11'd0;
        bbox_valid_d <= 1'b0;
    end else if (frame_start) begin
        bbox_x_min_d <= bbox_x_min; bbox_x_max_d <= bbox_x_max;
        bbox_y_min_d <= bbox_y_min; bbox_y_max_d <= bbox_y_max;
        bbox_valid_d <= bbox_valid;
    end
end

//=============================================================================
// 输出和边框绘制
//=============================================================================
wire on_border_x = ((xy_s6[0] == bbox_x_min_d) || (xy_s6[0] == bbox_x_max_d)) &&
                   (xy_s6[1] >= bbox_y_min_d) && (xy_s6[1] <= bbox_y_max_d);
wire on_border_y = ((xy_s6[1] == bbox_y_min_d) || (xy_s6[1] == bbox_y_max_d)) &&
                   (xy_s6[0] >= bbox_x_min_d) && (xy_s6[0] <= bbox_x_max_d);
wire draw_border = bbox_valid_d && (on_border_x || on_border_y);

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_r <= 8'd0; o_g <= 8'd0; o_b <= 8'd0;
        o_de <= 1'b0; o_hsyn <= 1'b0; o_vsyn <= 1'b0;
    end else begin
        o_de <= sync_s6[1]; o_hsyn <= sync_s6[3]; o_vsyn <= sync_s6[2];

        if (draw_border) begin
            o_r <= 8'd255; o_g <= 8'd0; o_b <= 8'd0;
        end else begin
            o_r <= rgb_s6[0]; o_g <= rgb_s6[1]; o_b <= rgb_s6[2];
        end
    end
end

endmodule
