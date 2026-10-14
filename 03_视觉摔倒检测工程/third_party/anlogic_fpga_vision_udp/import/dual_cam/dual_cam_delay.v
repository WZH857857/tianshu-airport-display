module dual_cam_delay(
    // 左目摄像头输入
    input                   cmos_pclk_l,        // 左目像素时钟
    input                   cmos_frame_vsync_l, // 左目场同步
    input                   cmos_frame_href_l,  // 左目行同步
    input                   cmos_frame_valid_l, // 左目数据有效
    input       [15:0]      cmos_wr_data_l,     // 左目RGB565数据

    // 右目摄像头输入
    input                   cmos_pclk_r,        // 右目像素时钟
    input                   cmos_frame_vsync_r, // 右目场同步
    input                   cmos_frame_href_r,  // 右目行同步
    input                   cmos_frame_valid_r, // 右目数据有效
    input       [15:0]      cmos_wr_data_r,     // 右目RGB565数据

    // 复位信号
    input                   rst_n,              // 复位信号

    // 双目测距控制（新增）
    input                   stereo_ranging_enable,  // 双目测距使能
    input       [7:0]       binary_threshold_r_low,  // R通道低阈值
    input       [7:0]       binary_threshold_r_high, // R通道高阈值
    input       [7:0]       binary_threshold_g_low,  // G通道低阈值
    input       [7:0]       binary_threshold_g_high, // G通道高阈值
    input       [7:0]       binary_threshold_b_low,  // B通道低阈值
    input       [7:0]       binary_threshold_b_high, // B通道高阈值

    // 输出接口
    output                  cam_write_en,       // 写使能
    output      [31:0]      cam_write_data,     // 32位输出数据
    output reg              cam_write_req,      // 写请求
    input                   cam_write_req_ack,  // 写请求应答

    // 双目测距输出（新增）
    output      [7:0]       stereo_distance     // 测距结果（cm，0-255）
);

//==============================================================================
// 左目数据处理（直通，打拍对齐）
//==============================================================================
reg                     cmos_frame_href_l_d0;
reg                     cmos_frame_vsync_l_d0;
reg                     cmos_frame_valid_l_d0;
reg [15:0]              cmos_wr_data_l_d0;
reg                     cmos_frame_href_l_d1;
reg                     cmos_frame_vsync_l_d1;
reg                     cmos_frame_valid_l_d1;
reg [15:0]              cmos_wr_data_l_d1;

// 左目数据打拍（使用左目像素时钟）
always@(posedge cmos_pclk_l or negedge rst_n) begin
    if(~rst_n) begin
        cmos_frame_href_l_d0 <= 1'b0;
        cmos_frame_vsync_l_d0 <= 1'b0;
        cmos_frame_valid_l_d0 <= 1'b0;
        cmos_wr_data_l_d0 <= 16'd0;
    end
    else begin
        cmos_frame_href_l_d0 <= cmos_frame_href_l;
        cmos_frame_vsync_l_d0 <= cmos_frame_vsync_l;
        cmos_frame_valid_l_d0 <= cmos_frame_valid_l;
        cmos_wr_data_l_d0 <= cmos_wr_data_l;
    end
end

always@(posedge cmos_pclk_l or negedge rst_n) begin
    if(~rst_n) begin
        cmos_frame_href_l_d1 <= 1'b0;
        cmos_frame_vsync_l_d1 <= 1'b0;
        cmos_frame_valid_l_d1 <= 1'b0;
        cmos_wr_data_l_d1 <= 16'd0;
    end
    else begin
        cmos_frame_href_l_d1 <= cmos_frame_href_l_d0;
        cmos_frame_vsync_l_d1 <= cmos_frame_vsync_l_d0;
        cmos_frame_valid_l_d1 <= cmos_frame_valid_l_d0;
        cmos_wr_data_l_d1 <= cmos_wr_data_l_d0;
    end
end

//==============================================================================
// 右目数据通过异步FIFO同步
//==============================================================================
// 右目写入FIFO的数据格式：[17:0] = {VSYNC, HREF, data[15:0]}
reg [17:0]              fifo_wr_data_r;
reg                     fifo_wr_en_r;
wire [17:0]             fifo_rd_data_r;
wire                    fifo_rd_en_r;
wire                    fifo_empty_r;
wire                    fifo_full_r;

// 右目数据写入FIFO（在右目像素时钟域）
reg                     cmos_frame_valid_r_d1;
always@(posedge cmos_pclk_r or negedge rst_n) begin
    if(~rst_n) begin
        cmos_frame_valid_r_d1 <= 1'b0;
        fifo_wr_data_r <= 18'd0;
        fifo_wr_en_r <= 1'b0;
    end
    else begin
        cmos_frame_valid_r_d1 <= cmos_frame_valid_r;
        fifo_wr_en_r <= cmos_frame_valid_r;
        // 将同步信号和数据一起打包写入FIFO
        fifo_wr_data_r <= {cmos_frame_vsync_r, cmos_frame_href_r, cmos_wr_data_r};
    end
end

// 从FIFO读出右目数据（在左目像素时钟域）
// 当左目数据有效时，同步读取右目数据
assign fifo_rd_en_r = cmos_frame_valid_l_d1 & (~fifo_empty_r);

// 从FIFO读出的右目数据
wire                    cmos_frame_vsync_r_sync;
wire                    cmos_frame_href_r_sync;
wire [15:0]             cmos_wr_data_r_sync;

assign {cmos_frame_vsync_r_sync, cmos_frame_href_r_sync, cmos_wr_data_r_sync} = fifo_rd_data_r;

// 异步FIFO实例化
async_fifo #(
    .DATA_WIDTH     (18),           // 数据宽度：18位（2位同步信号 + 16位RGB565）
    .ADDR_WIDTH     (13),           // 地址宽度：13位，深度8192
    .FULL_AHEAD     (1),
    .SHOWAHEAD_EN   (0)
) async_fifo_right (
    .reset          (~rst_n),
    // 写端口（右目像素时钟域）
    .wrclk          (cmos_pclk_r),
    .wren           (fifo_wr_en_r),
    .wrdata         (fifo_wr_data_r),
    .full           (fifo_full_r),
    .almost_full    (),
    .wrusedw        (),
    // 读端口（左目像素时钟域）
    .rdclk          (cmos_pclk_l),
    .rden           (fifo_rd_en_r),
    .rddata         (fifo_rd_data_r),
    .empty          (fifo_empty_r),
    .rdusedw        ()
);

//==============================================================================
// 输出数据合并：根据测距模式选择输出RGB或二值化数据
//==============================================================================
// 使用左目的同步信号
assign cam_write_en = cmos_frame_valid_l_d1 & (~fifo_empty_r);

// 二值化输出信号
wire [7:0]  left_binary_out;
wire [7:0]  right_binary_out;
wire        left_binary_bit;
wire        right_binary_bit;

// 输出32位数据：
// - RGB模式：高16位=左目RGB565，低16位=右目RGB565
// - 测距模式：高16位=左目二值化（扩展），低16位=右目二值化（扩展）
assign cam_write_data = stereo_ranging_enable ?
                        {left_binary_out, left_binary_out, right_binary_out, right_binary_out} :  // 测距模式：2字节灰度
                        {cmos_wr_data_l_d1, cmos_wr_data_r_sync};                                  // RGB模式：原始数据

//==============================================================================
// 双目测距处理链（新增）
//==============================================================================

// 左目RGB565 → 二值化（RGB颜色阈值）
wire        left_vsync_bin, left_hsync_bin, left_data_en_bin;
rgb565_binarize u_left_binarize (
    .clk                (cmos_pclk_l),
    .rst_n              (rst_n),
    // 控制
    .enable             (stereo_ranging_enable),
    .threshold_r_low    (binary_threshold_r_low),
    .threshold_r_high   (binary_threshold_r_high),
    .threshold_g_low    (binary_threshold_g_low),
    .threshold_g_high   (binary_threshold_g_high),
    .threshold_b_low    (binary_threshold_b_low),
    .threshold_b_high   (binary_threshold_b_high),
    // 视频流输入（使用打拍后的左目数据）
    .vsync_i            (cmos_frame_vsync_l_d1),
    .hsync_i            (cmos_frame_href_l_d1),
    .data_en_i          (cmos_frame_valid_l_d1),
    .rgb565_i           (cmos_wr_data_l_d1),
    // 二值化输出
    .vsync_o            (left_vsync_bin),
    .hsync_o            (left_hsync_bin),
    .data_en_o          (left_data_en_bin),
    .binary_o           (left_binary_out),
    .binary_bit_o       (left_binary_bit)
);

// 右目RGB565 → 二值化（RGB颜色阈值）
wire        right_vsync_bin, right_hsync_bin, right_data_en_bin;
rgb565_binarize u_right_binarize (
    .clk                (cmos_pclk_l),
    .rst_n              (rst_n),
    // 控制
    .enable             (stereo_ranging_enable),
    .threshold_r_low    (binary_threshold_r_low),
    .threshold_r_high   (binary_threshold_r_high),
    .threshold_g_low    (binary_threshold_g_low),
    .threshold_g_high   (binary_threshold_g_high),
    .threshold_b_low    (binary_threshold_b_low),
    .threshold_b_high   (binary_threshold_b_high),
    // 视频流输入（使用同步后的右目数据）
    .vsync_i            (cmos_frame_vsync_r_sync),
    .hsync_i            (cmos_frame_href_r_sync),
    .data_en_i          (fifo_rd_en_r),
    .rgb565_i           (cmos_wr_data_r_sync),
    // 二值化输出
    .vsync_o            (right_vsync_bin),
    .hsync_o            (right_hsync_bin),
    .data_en_o          (right_data_en_bin),
    .binary_o           (right_binary_out),
    .binary_bit_o       (right_binary_bit)
);

// 左目边界框检测
wire [10:0] left_bounding_coor;
bounding_box u_left_bounding_box (
    .rst_n      (rst_n),
    .sclk       (cmos_pclk_l),
    // 输入：二值化数据
    .vsync_i    (left_vsync_bin),
    .hsync_i    (left_hsync_bin),
    .idata      (left_binary_bit),
    .rgb_data_i (24'd0),
    // 输出（仅使用坐标）
    .vsync_o    (),
    .hsync_o    (),
    .rgb_data_o (),
    .coor_data  (left_bounding_coor)
);

// 右目边界框检测
wire [10:0] right_bounding_coor;
bounding_box u_right_bounding_box (
    .rst_n      (rst_n),
    .sclk       (cmos_pclk_l),
    // 输入：二值化数据
    .vsync_i    (right_vsync_bin),
    .hsync_i    (right_hsync_bin),
    .idata      (right_binary_bit),
    .rgb_data_i (24'd0),
    // 输出（仅使用坐标）
    .vsync_o    (),
    .hsync_o    (),
    .rgb_data_o (),
    .coor_data  (right_bounding_coor)
);

// 双目测距计算
wire        distance_valid;
stereo_distance_calc #(
    .BASELINE       (120),      // 基线距离120mm（根据实际相机调整）
    .FOCAL_LENGTH   (350)       // 焦距350像素（根据实际调整）
) u_stereo_distance (
    .clk            (cmos_pclk_l),
    .rst_n          (rst_n),
    // 边界框中心坐标
    .left_coor_x    (left_bounding_coor),
    .right_coor_x   (right_bounding_coor),
    .coor_valid     (left_vsync_bin),   // 每帧更新一次
    // 距离输出
    .distance       (stereo_distance),
    .distance_valid (distance_valid)
);

//==============================================================================
// 写请求控制（使用左目的场同步信号）
//==============================================================================
always@(posedge cmos_pclk_l or negedge rst_n) begin
    if(~rst_n) begin
        cam_write_req <= 1'b0;
    end
    // 检测左目场同步信号的边沿
    else if(cmos_frame_vsync_l_d0 & ~cmos_frame_vsync_l) begin
        cam_write_req <= 1'b1;
    end
    else if(cam_write_req_ack) begin
        cam_write_req <= 1'b0;
    end
end

endmodule
