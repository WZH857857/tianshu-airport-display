`timescale 1ns / 1ps
//**********************************************************************
// Description: Image Processing Wrapper for OV5640 Camera (640x480 RGB888)
// Supports: Binarization, Erosion (Dilation), Opening operation, Relief effect, Laplacian, Sobel, Border detection
//
// Processing modes (i_proc_mode):
// 3'b000: Original RGB image (bypass)
// 3'b001: Binarized image
// 3'b010: Binarized + Dilation (显示为"腐蚀")
// 3'b011: Opening operation (Dilation then Erosion)
// 3'b100: Relief effect (浮雕效果)
// 3'b101: Laplacian sharpening (拉普拉斯锐化)
// 3'b110: Sobel edge detection (Sobel边缘检测)
//
// Border enable (i_border_enable):
// When enabled, draws bounding box around white regions on original image
//*********************************************************************/

module image_proc_wrapper (
    input   wire                i_clk,
    input   wire                i_rst_n,

    // Input video timing signals
    input   wire                i_hsyn,
    input   wire                i_vsyn,
    input   wire                i_en,
    input   wire [31:0]         i_data,      // RGB888: [31:24]=R, [23:16]=G, [15:8]=B

    // Processing mode control
    input   wire [2:0]          i_proc_mode, // 000:original, 001:binary, 010:dilate, 011:opening, 100:relief, 101:laplacian, 110:sobel

    // Binarization threshold inputs (新增)
    input   wire [7:0]          i_threshold_low,  // 低阈值 (0-255)
    input   wire [7:0]          i_threshold_high, // 高阈值 (0-255)

    // Border detection enable (新增)
    input   wire                i_border_enable,  // 边框检测使能

    // Output video signals
    output  wire                o_hsyn,
    output  wire                o_vsyn,
    output  wire                o_en,
    output  wire [31:0]         o_data,

    // Grayscale output (for external processing, e.g., frame difference)
    output  wire [7:0]          o_gray      // 灰度输出（用于外部帧差法等）
);

// ===== Parameters =====
parameter H_ACTIVE = 11'd640;
parameter V_ACTIVE = 11'd480;

// ===== Extract RGB components =====
wire [7:0] R = i_data[31:24];
wire [7:0] G = i_data[23:16];
wire [7:0] B = i_data[15:8];

// ===== RGB to Grayscale Conversion =====
// Gray = (77*R + 150*G + 29*B) >> 8
wire [15:0] gray_mult_r = R * 8'd77;
wire [15:0] gray_mult_g = G * 8'd150;
wire [15:0] gray_mult_b = B * 8'd29;
wire [17:0] gray_sum = gray_mult_r + gray_mult_g + gray_mult_b;
wire [7:0] gray_value = gray_sum[15:8];

// ===== Binarization =====
// 动态阈值计算：取高低阈值的平均值作为二值化阈值
wire [8:0] threshold_sum = i_threshold_low + i_threshold_high;  // 9位以避免溢出
wire [7:0] threshold = threshold_sum[8:1];  // 除以2取平均值

reg [7:0] binary_value;
reg hsyn_d1, vsyn_d1, en_d1;
reg [31:0] data_d1;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        binary_value <= 8'd0;
        hsyn_d1 <= 1'b0;
        vsyn_d1 <= 1'b0;
        en_d1 <= 1'b0;
        data_d1 <= 32'd0;
    end else begin
        // 数据和时序信号必须同步更新，不能加条件判断
        // 使用动态计算的阈值进行二值化
        binary_value <= (gray_value >= threshold) ? 8'd255 : 8'd0;
        data_d1 <= i_data;
        hsyn_d1 <= i_hsyn;
        vsyn_d1 <= i_vsyn;
        en_d1 <= i_en;
    end
end

// ===== Dilation (膨胀) =====
wire dilate_hs, dilate_vs, dilate_en;
wire [7:0] dilate_binary;

image_dilate_filtering u_dilate (
    .i_clk      (i_clk),
    .i_rst_n    (i_rst_n),
    .i_hsyn     (hsyn_d1),
    .i_vsyn     (vsyn_d1),
    .i_en       (en_d1),
    .i_binary   (binary_value),
    .o_hs       (dilate_hs),
    .o_vs       (dilate_vs),
    .o_en       (dilate_en),
    .o_binary   (dilate_binary)
);

// ===== Erosion (腐蚀) =====
wire erode_hs, erode_vs, erode_en;
wire [7:0] erode_binary;

image_erode_filtering u_erode (
    .i_clk      (i_clk),
    .i_rst_n    (i_rst_n),
    .i_hsyn     (dilate_hs),
    .i_vsyn     (dilate_vs),
    .i_en       (dilate_en),
    .i_binary   (dilate_binary),
    .o_hs       (erode_hs),
    .o_vs       (erode_vs),
    .o_en       (erode_en),
    .o_binary   (erode_binary)
);

// ===== Relief Effect (浮雕效果) =====
// 注意：浮雕模块使用原始输入信号（无延迟），因为gray_value是组合逻辑
// 模块内部会自动处理延迟对齐
wire relief_hs, relief_vs, relief_en;
wire [7:0] relief_gray;

image_relief_filtering u_relief (
    .i_clk      (i_clk),
    .i_rst_n    (i_rst_n),
    .i_hsyn     (i_hsyn),          // 使用原始信号，0延迟
    .i_vsyn     (i_vsyn),          // 使用原始信号，0延迟
    .i_en       (i_en),            // 使用原始信号，0延迟
    .i_gray     (gray_value),      // 组合逻辑，0延迟
    .value      (8'd128),          // 浮雕偏移值，硬编码为128
    .o_hs       (relief_hs),
    .o_vs       (relief_vs),
    .o_en       (relief_en),
    .o_gray     (relief_gray)
);

// ===== Laplacian Sharpening (拉普拉斯锐化) =====
wire laplacian_hs, laplacian_vs, laplacian_en;
wire [7:0] laplacian_gray;

image_laplacian_gray u_laplacian (
    .i_clk      (i_clk),
    .i_rst_n    (i_rst_n),
    .i_hsyn     (i_hsyn),
    .i_vsyn     (i_vsyn),
    .i_en       (i_en),
    .i_gray     (gray_value),
    .o_hs       (laplacian_hs),
    .o_vs       (laplacian_vs),
    .o_en       (laplacian_en),
    .o_gray     (laplacian_gray)
);

// ===== Sobel Edge Detection (Sobel边缘检测) =====
wire sobel_hs, sobel_vs, sobel_en;
wire [7:0] sobel_gray;

image_sobel_gray u_sobel (
    .i_clk      (i_clk),
    .i_rst_n    (i_rst_n),
    .i_hsyn     (i_hsyn),
    .i_vsyn     (i_vsyn),
    .i_en       (i_en),
    .i_gray     (gray_value),
    .threshold  (8'd100),            // Sobel边缘检测阈值，硬编码为100
    .o_hs       (sobel_hs),
    .o_vs       (sobel_vs),
    .o_en       (sobel_en),
    .o_gray     (sobel_gray)
);

// ===== 像素位置计数已移除 =====
// 边框检测模块使用内部计数器，不需要外部位置

// ===== Delay matching (延迟匹配膨胀+腐蚀的处理延迟) =====
// 注意：膨胀和腐蚀模块会引入延迟，需要匹配
// 但要注意只在enable有效时才更新延迟链
reg [31:0] rgb_delay [0:5];
reg hsyn_delay [0:5];
reg vsyn_delay [0:5];
reg en_delay [0:5];
reg [7:0] binary_delay [0:5];

integer i;
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        for (i = 0; i < 6; i = i + 1) begin
            rgb_delay[i] <= 32'd0;
            hsyn_delay[i] <= 1'b0;
            vsyn_delay[i] <= 1'b0;
            en_delay[i] <= 1'b0;
            binary_delay[i] <= 8'd0;
        end
    end else begin
        // 延迟链条 - 只在enable时推进
        if (i_en) begin
            rgb_delay[0] <= data_d1;
            binary_delay[0] <= binary_value;

            for (i = 1; i < 6; i = i + 1) begin
                rgb_delay[i] <= rgb_delay[i-1];
                binary_delay[i] <= binary_delay[i-1];
            end
        end

        // 时序信号始终传递
        hsyn_delay[0] <= i_hsyn;
        vsyn_delay[0] <= i_vsyn;
        en_delay[0] <= i_en;

        for (i = 1; i < 6; i = i + 1) begin
            hsyn_delay[i] <= hsyn_delay[i-1];
            vsyn_delay[i] <= vsyn_delay[i-1];
            en_delay[i] <= en_delay[i-1];
        end
    end
end

// ===== Border Detection (边框检测) =====
// 使用膨胀后的二值化结果检测白色区域，在原始图像上绘制边框
// 使用简化版模块，内部自带位置计数器
wire [7:0] border_r, border_g, border_b;

image_skin_select_simplified #(
    .H_ACTIVE   (640),
    .V_ACTIVE   (480),
    .R_VALUE    (8'd0),     // 黑色边框
    .G_VALUE    (8'd255),   // 绿色边框
    .B_VALUE    (8'd0)
) u_border_detect (
    .i_clk          (i_clk),
    .i_rst_n        (i_rst_n),
    .i_hsyn         (dilate_hs),
    .i_vsyn         (dilate_vs),
    .i_de           (dilate_en),
    .i_r            (dilate_binary),        // 膨胀后的二值化结果
    .i_g            (dilate_binary),
    .i_b            (dilate_binary),
    .i_r_original   (rgb_delay[3][31:24]),  // 延迟匹配的原始图像
    .i_g_original   (rgb_delay[3][23:16]),
    .i_b_original   (rgb_delay[3][15:8]),
    .o_r            (border_r),
    .o_g            (border_g),
    .o_b            (border_b)
);

// ===== Mode Selection Output =====
reg [31:0] o_data_reg;
reg o_hsyn_reg, o_vsyn_reg, o_en_reg;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_data_reg <= 32'd0;
        o_hsyn_reg <= 1'b0;
        o_vsyn_reg <= 1'b0;
        o_en_reg <= 1'b0;
    end else begin
        // 如果边框使能，优先输出边框检测结果
        if (i_border_enable) begin
            o_data_reg <= {border_r, border_g, border_b, 8'd0};
            o_hsyn_reg <= dilate_hs;
            o_vsyn_reg <= dilate_vs;
            o_en_reg <= dilate_en;
        end else begin
            case (i_proc_mode)
                3'b000: begin  // Original RGB (无延迟)
                    o_data_reg <= i_data;
                    o_hsyn_reg <= i_hsyn;
                    o_vsyn_reg <= i_vsyn;
                    o_en_reg <= i_en;
                end
                3'b001: begin  // Binary (延迟1个时钟)
                    o_data_reg <= {binary_value, binary_value, binary_value, 8'd0};
                    o_hsyn_reg <= hsyn_d1;
                    o_vsyn_reg <= vsyn_d1;
                    o_en_reg <= en_d1;
                end
                3'b010: begin  // Dilation (膨胀效果)
                    o_data_reg <= {dilate_binary, dilate_binary, dilate_binary, 8'd0};
                    o_hsyn_reg <= dilate_hs;
                    o_vsyn_reg <= dilate_vs;
                    o_en_reg <= dilate_en;
                end
                3'b011: begin  // Opening (开运算: Dilation -> Erosion)
                    o_data_reg <= {erode_binary, erode_binary, erode_binary, 8'd0};
                    o_hsyn_reg <= erode_hs;
                    o_vsyn_reg <= erode_vs;
                    o_en_reg <= erode_en;
                end
                3'b100: begin  // Relief Effect (浮雕效果，灰度图转RGB显示)
                    o_data_reg <= {relief_gray, relief_gray, relief_gray, 8'd0};
                    o_hsyn_reg <= relief_hs;
                    o_vsyn_reg <= relief_vs;
                    o_en_reg <= relief_en;
                end
                3'b101: begin  // Laplacian Sharpening (拉普拉斯锐化，灰度图转RGB显示)
                    o_data_reg <= {laplacian_gray, laplacian_gray, laplacian_gray, 8'd0};
                    o_hsyn_reg <= laplacian_hs;
                    o_vsyn_reg <= laplacian_vs;
                    o_en_reg <= laplacian_en;
                end
                3'b110: begin  // Sobel Edge Detection (Sobel边缘检测，二值图转RGB显示)
                    o_data_reg <= {sobel_gray, sobel_gray, sobel_gray, 8'd0};
                    o_hsyn_reg <= sobel_hs;
                    o_vsyn_reg <= sobel_vs;
                    o_en_reg <= sobel_en;
                end
                default: begin
                    o_data_reg <= i_data;
                    o_hsyn_reg <= i_hsyn;
                    o_vsyn_reg <= i_vsyn;
                    o_en_reg <= i_en;
                end
            endcase
        end
    end
end

assign o_data = o_data_reg;
assign o_hsyn = o_hsyn_reg;
assign o_vsyn = o_vsyn_reg;
assign o_en = o_en_reg;
assign o_gray = gray_value;  // 输出灰度值（供外部帧差法使用）

endmodule
