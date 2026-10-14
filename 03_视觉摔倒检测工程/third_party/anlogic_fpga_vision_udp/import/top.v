`timescale 1ns / 1ps
//================================================================================
// 模块名称: top_final
// 功能描述: 双模式视频处理系统顶层模块
//
// 系统架构:
//   - 项目0: 图像接收模式 - UDP接收→SDRAM缓存→HDMI显示
//   - 项目1: 摄像头采集模式 - SD卡/摄像头采集→SDRAM缓存→UDP发送
//
// 核心功能:
//   - UDP命令或按键动态切换工作模式
//   - SDRAM地址空间隔离，硬件仲裁保证互不干扰
//   - 多分辨率支持 (VGA/720p/1080p)
//   - 双目立体视觉与距离测量
//   - 实时图像处理 (二值化/腐蚀/形态学/浮雕/帧差法)
//   - 模板匹配目标识别
//   - 运动检测与边框标注
//================================================================================

module top_final(
    // 系统时钟与复位
    input                       clk_50,
    input                       key1,              // 系统复位 (低有效)
    input                       key2,              // SD卡触发
    input                       key3,              // 模式切换

    // SD卡接口 (项目1)
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,

    // 数码管与LED接口 (共用)
    output [7:0]                tub,
    output [7:0]                dled,
    output [3:0]                led_data,

    // 左目摄像头OV5640接口 (项目1)
    input                       cam_pclk,
    input                       cam_vsync,
    input                       cam_href,
    input   [7:0]               cam_data,
    output                      cam_rst_n,
    output                      cam_pwdn,
    output                      cam_scl,
    inout                       cam_sda,

    // 右目摄像头OV5640接口 (项目1，双目立体视觉)
    input                       cam_r_pclk,
    input                       cam_r_vsync,
    input                       cam_r_href,
    input   [7:0]               cam_r_data,
    output                      cam_r_rst_n,
    output                      cam_r_pwdn,
    output                      cam_r_scl,
    inout                       cam_r_sda,

    // 千兆以太网RGMII接口 (共用)
    input                       phy1_rgmii_rx_clk,
    input                       phy1_rgmii_rx_ctl,
    input  [3:0]                phy1_rgmii_rx_data,
    output wire                 phy1_rgmii_tx_clk,
    output wire                 phy1_rgmii_tx_ctl,
    output wire [3:0]           phy1_rgmii_tx_data,

    // HDMI输出接口 (项目0)
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P
);

// ========================================================================
// 参数定义
// ========================================================================
parameter DEVICE               = "EG4";          // FPGA器件型号
parameter MEM_DATA_BITS        = 32;             // 存储器数据位宽
parameter ADDR_BITS            = 21;             // 地址位宽
parameter BURST_BITS           = 10;             // 突发长度位宽

// 以太网网络配置
parameter LOCAL_UDP_PORT_NUM   = 16'h1773;
parameter LOCAL_IP_ADDRESS     = 32'hc0a8f001;  // 192.168.240.1
parameter LOCAL_MAC_ADDRESS    = 48'h0123456789ab;
parameter DST_UDP_PORT_NUM     = 16'h1773;
parameter DST_IP_ADDRESS       = 32'hc0a8f002;  // 192.168.240.2

// 摄像头分辨率 (VGA 640×480)
parameter V_CMOS_DISP          = 11'd480;
parameter H_CMOS_DISP          = 11'd640;
parameter TOTAL_H_PIXEL        = H_CMOS_DISP + 12'd1216;
parameter TOTAL_V_PIXEL        = V_CMOS_DISP + 12'd504;

// ========================================================================
// 模式控制信号 (UDP可配置)
// ========================================================================
wire        udp_mode_project;           // 工程选择: 0=图像接收, 1=摄像头采集
wire        udp_mode_q3v3_internal;     // 数据源选择: 0=SD卡, 1=实时摄像头
wire        udp_mode_bp12_led_seg;      // 显示模式: 0=LED, 1=数码管
wire        udp_sd_trigger;             // SD卡读取触发
wire [2:0]  udp_image_proc_mode;        // 图像处理模式: 0=原始/1=二值化/2=腐蚀/3=开运算/4=浮雕/7=帧差法
wire [7:0]  udp_binary_threshold_low;   // 灰度二值化低阈值
wire [7:0]  udp_binary_threshold_high;  // 灰度二值化高阈值

// RGB颜色阈值 (双目测距用)
reg  [7:0]  udp_binary_threshold_r_low;
reg  [7:0]  udp_binary_threshold_r_high;
reg  [7:0]  udp_binary_threshold_g_low;
reg  [7:0]  udp_binary_threshold_g_high;
reg  [7:0]  udp_binary_threshold_b_low;
reg  [7:0]  udp_binary_threshold_b_high;

wire        udp_border_enable;          // 运动边框叠加使能
wire        udp_q3v3_cam_dual_enable;   // 双目模式: 0=单目, 1=双目
wire        udp_q3v3_cam_right_select;  // 摄像头选择: 0=左目, 1=右目
wire [1:0]  resolution_sel;             // 分辨率选择: 0=VGA, 1=720p, 2=1080p
wire [20:0] frame_pixel_count;          // 帧像素计数

// ========================================================================
// 模板匹配信号 (32×32二值化图像)
// ========================================================================
wire        template_match_done;
wire        template_match_result;      // 匹配结果: 1=匹配, 0=不匹配
wire [9:0]  template_hamming_distance;  // 汉明距离

// ========================================================================
// 双目测距信号
// ========================================================================
reg         stereo_ranging_enable;      // 测距模式: 0=RGB模式, 1=测距模式
wire [7:0]  stereo_distance_cam;        // 距离输出 (cam_pclk域, 单位:cm)
reg  [7:0]  stereo_distance_udp_sync1;  // CDC同步级1
reg  [7:0]  stereo_distance_udp_sync2;  // CDC同步级2
reg         stereo_ranging_enable_cam_sync1;
reg         stereo_ranging_enable_cam_sync2;
wire        stereo_ranging_enable_cam;

// ========================================================================
// SD卡状态
// ========================================================================
wire [3:0]  sd_state_code;

// ========================================================================
// 时钟信号
// ========================================================================
wire clk_125m_0_1;     // 125MHz (SDRAM, TEMAC)
wire clk_125m_0_2;     // 125MHz (HDMI 5x)
wire clk_125m_90;      // 125MHz 90度相移 (SDRAM IP)
wire clk_25m_0;        // 25MHz (HDMI像素时钟)
wire clk_12_5m_0;      // 12.5MHz
wire clk_50m_0;        // 50MHz
wire clk_50m_180;      // 50MHz 180度
wire clk_1_25m_0;      // 1.25MHz
wire pll_locked;

// 动态分辨率时钟
wire video_clk_selected;      // 根据分辨率选择的像素时钟
wire video_clk_5x_selected;   // 根据分辨率选择的5x时钟
wire video_pll_locked;        // video_pll锁定信号

wire mem_clk         = clk_125m_0_1;
wire mem_clk_sft     = clk_125m_90;
wire video_clk       = video_clk_selected;       // 使用动态选择的像素时钟
wire video_clk_5x    = video_clk_5x_selected;    // 使用动态选择的5x时钟
wire temac_clk       = clk_125m_0_1;
wire udp_clk;

// ========================================================================
// 复位信号
// ========================================================================
wire reset;
wire reset_reg;
assign reset_reg = ~pll_locked;
assign reset = ~key1 || reset_reg;

// ========================================================================
// SDRAM控制信号
// ========================================================================
wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

// SDRAM真实接口（连接到SDRAM IP核）
wire        sdram_App_rd_en;
wire [ADDR_BITS-1:0] sdram_App_rd_addr;
wire        sdram_App_wr_en;
wire [ADDR_BITS-1:0] sdram_App_wr_addr;
wire [MEM_DATA_BITS-1:0] sdram_App_wr_din;
wire [3:0]  sdram_App_wr_dm;
wire        sdram_Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] sdram_Sdr_rd_dout;

// ========================================================================
// 项目0: 图像接收模式信号
// ========================================================================
// SDRAM接口
wire        prj0_App_rd_en;
wire [ADDR_BITS-1:0] prj0_App_rd_addr;
wire        prj0_App_wr_en;
wire [ADDR_BITS-1:0] prj0_App_wr_addr;
wire [MEM_DATA_BITS-1:0] prj0_App_wr_din;
wire [3:0]  prj0_App_wr_dm;
wire        prj0_Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] prj0_Sdr_rd_dout;

// UDP写入与视频读取控制信号
wire        prj0_write_req;
wire        prj0_write_req_ack;
wire        prj0_write_en;
wire [31:0] prj0_write_data;
wire        prj0_video_read_req;
wire        prj0_video_read_req_ack;
wire        prj0_video_read_en;
wire [31:0] prj0_video_read_data;
wire        prj0_hs, prj0_vs, prj0_de;
wire [23:0] prj0_vout_data;

// ========================================================================
// 项目1: 摄像头采集模式信号
// ========================================================================
// SDRAM接口
wire        prj1_App_rd_en;
wire [ADDR_BITS-1:0] prj1_App_rd_addr;
wire        prj1_App_wr_en;
wire [ADDR_BITS-1:0] prj1_App_wr_addr;
wire [MEM_DATA_BITS-1:0] prj1_App_wr_din;
wire [3:0]  prj1_App_wr_dm;
wire        prj1_Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] prj1_Sdr_rd_dout;

// SD卡与摄像头写入控制信号
wire        prj1_sd_write_req;
wire        prj1_sd_write_req_ack;
wire        prj1_sd_write_en;
wire [31:0] prj1_sd_write_data;
wire        prj1_cam_write_req;
wire        prj1_cam_write_req_ack;
wire        prj1_cam_write_en;
wire [31:0] prj1_cam_write_data;

// 左目图像处理模块信号
wire        prj1_left_write_en;
wire [31:0] prj1_left_write_data;
wire        prj1_left_write_req;
wire        prj1_left_write_req_ack;

// 右目图像处理模块信号（新增）
wire        prj1_right_write_en;
wire [31:0] prj1_right_write_data;
wire        prj1_right_write_req;
wire        prj1_right_write_req_ack;

// 双目合并模块信号（新增）
wire        prj1_dual_write_en;
wire [31:0] prj1_dual_write_data;
wire        prj1_dual_write_req;
wire        prj1_dual_write_req_ack;

// 图像处理参数CDC同步（左目）
reg  [2:0]  image_proc_mode_sync1, image_proc_mode_sync2;  // CDC同步
reg  [7:0]  threshold_low_sync1, threshold_low_sync2;      // 低阈值CDC同步
reg  [7:0]  threshold_high_sync1, threshold_high_sync2;    // 高阈值CDC同步
reg         border_enable_sync1, border_enable_sync2;      // 边框使能CDC同步

// 右目图像处理参数CDC同步（新增）
reg  [2:0]  image_proc_mode_sync1_r, image_proc_mode_sync2_r;
reg  [7:0]  threshold_low_sync1_r, threshold_low_sync2_r;
reg  [7:0]  threshold_high_sync1_r, threshold_high_sync2_r;
reg         border_enable_sync1_r, border_enable_sync2_r;

// q3v3 UDP读取信号（单目和双目共用）
wire        prj1_video_read_req;
wire        prj1_video_read_req_ack;
wire        prj1_video_read_en;
wire [31:0] prj1_video_read_data;

// q3v3数据源选择信号
wire prj1_write_req_sel;
wire prj1_write_req_ack_sel;
wire prj1_write_en_sel;
wire [31:0] prj1_write_data_sel;
wire prj1_write_clk_sel;

// q3v3左目摄像头数据
wire cmos_frame_vsync;
wire cmos_frame_href;
wire cmos_frame_valid;
wire [15:0] cmos_wr_data;

// q3v3右目摄像头数据（新增）
wire cmos_frame_vsync_r;
wire cmos_frame_href_r;
wire cmos_frame_valid_r;
wire [15:0] cmos_wr_data_r;

// q3v3 SD卡时钟
wire sd_card_clk;

// ========================================================================
// UDP相关信号（两个工程共用UDP协议栈）
// ========================================================================
wire        app_rx_data_valid;
wire [7:0]  app_rx_data;
wire [15:0] app_rx_data_length;
wire [15:0] app_rx_port_num;

wire        udp_tx_ready;
wire        app_tx_ack;
wire        app_tx_data_request;
wire        app_tx_data_valid;
wire [7:0]  app_tx_data;
wire [15:0] udp_data_length;

// bp12_2的UDP发送信号
wire [7:0]  prj0_app_tx_data;
wire        prj0_app_tx_data_valid;
wire        prj0_app_tx_data_request;
wire [15:0] prj0_udp_data_length;

// q3v3单目的UDP发送信号
wire [7:0]  prj1_single_app_tx_data;
wire        prj1_single_app_tx_data_valid;
wire        prj1_single_app_tx_data_request;
wire [15:0] prj1_single_udp_data_length;
wire        prj1_single_read_req;
wire        prj1_single_read_req_ack;
wire        prj1_single_read_en;

// q3v3双目的UDP发送信号（新增）
wire [7:0]  prj1_dual_app_tx_data;
wire        prj1_dual_app_tx_data_valid;
wire        prj1_dual_app_tx_data_request;
wire [15:0] prj1_dual_udp_data_length;
wire        prj1_dual_read_req;
wire        prj1_dual_read_req_ack;
wire        prj1_dual_read_en;

// q3v3最终的UDP发送信号（多路选择后）
wire [7:0]  prj1_app_tx_data;
wire        prj1_app_tx_data_valid;
wire        prj1_app_tx_data_request;
wire [15:0] prj1_udp_data_length;

// UDP发送接口多路选择（根据UDP控制的工程模式选择）
// UDP发送接口多路选择（根据UDP控制的工程模式选择）
// [这三行是数据流，从工程(prj) -> 协议栈(app)]
assign app_tx_data         = udp_mode_project ? prj1_app_tx_data : prj0_app_tx_data;
assign app_tx_data_valid   = udp_mode_project ? prj1_app_tx_data_valid : prj0_app_tx_data_valid;
assign udp_data_length     = udp_mode_project ? prj1_udp_data_length : prj0_udp_data_length;

// [这是请求流，也是从工程(prj) -> 协议栈(app)]
// [!! 新增这一行 !!]
assign app_tx_data_request = udp_mode_project ? prj1_app_tx_data_request : prj0_app_tx_data_request;

// [!! 这两行必须删除 !!]
// assign prj0_app_tx_data_request = mode_project ? 1'b0 : app_tx_data_request; 
// assign prj1_app_tx_data_request = mode_project ? app_tx_data_request : 1'b0;


// ========================================================================
// TEMAC以太网信号
// ========================================================================
wire        temac_tx_ready;
wire        temac_tx_valid;
wire [7:0]  temac_tx_data;
wire        temac_tx_sof;
wire        temac_tx_eof;

wire        temac_rx_ready;
wire        temac_rx_valid;
wire [7:0]  temac_rx_data;
wire        temac_rx_sof;
wire        temac_rx_eof;

wire        rx_clk_int;
wire        rx_clk_en_int;
wire        tx_clk_int;
wire        tx_clk_en_int;
wire        rx_valid;
wire [7:0]  rx_data;
wire [7:0]  tx_data;
wire        tx_valid;
wire        tx_rdy;
wire        tx_collision;
wire        tx_retransmit;
wire        rx_correct_frame;
wire        rx_error_frame;

wire [1:0]  TRI_speed = 2'b10;  // 千兆网速度

// TEMAC配置信号
wire        tx_stop = 1'b0;
wire [7:0]  tx_ifg_val = 8'h00;
wire        pause_req = 1'b0;
wire [15:0] pause_val = 16'h0;
wire [47:0] pause_source_addr = 48'h5af1f2f3f4f5;
wire [47:0] unicast_address;
wire [19:0] mac_cfg_vector;

assign unicast_address = {
    LOCAL_MAC_ADDRESS[7:0],
    LOCAL_MAC_ADDRESS[15:8],
    LOCAL_MAC_ADDRESS[23:16],
    LOCAL_MAC_ADDRESS[31:24],
    LOCAL_MAC_ADDRESS[39:32],
    LOCAL_MAC_ADDRESS[47:40]
};

assign mac_cfg_vector = {1'b0, 2'b00, TRI_speed, 8'b00000010, 7'b0000010};

// ========================================================================
// bp12_2工程: 数据包长度判断逻辑
// ========================================================================
wire        is_seg_data;
wire        seg_data_valid;
wire        img_data_valid;

assign is_seg_data = (app_rx_data_length <= 16'd50);
assign seg_data_valid = app_rx_data_valid && is_seg_data && !udp_mode_project;
assign img_data_valid = app_rx_data_valid && !is_seg_data && !udp_mode_project;

// ========================================================================
// UDP模式控制模块（解析上位机控制命令，支持按键控制）
// ========================================================================
udp_mode_ctrl u_udp_mode_ctrl (
    .udp_rx_clk             (udp_clk),
    .reset                  (key1),
    .app_rx_data_valid      (app_rx_data_valid),
    .app_rx_data            (app_rx_data),
    .app_rx_data_length     (app_rx_data_length),
    .key2                   (key2),                   // SD卡触发按键
    .key3                   (key3),                   // 模式切换按键
    .mode_project           (udp_mode_project),
    .mode_q3v3_internal     (udp_mode_q3v3_internal),
    .mode_bp12_led_seg      (udp_mode_bp12_led_seg),
    .sd_trigger             (udp_sd_trigger),
    .image_proc_mode        (udp_image_proc_mode),
    .binary_threshold_low   (udp_binary_threshold_low),
    .binary_threshold_high  (udp_binary_threshold_high),
    .border_enable          (udp_border_enable),
    .q3v3_cam_dual_enable   (udp_q3v3_cam_dual_enable),    // 新增
    .q3v3_cam_right_select  (udp_q3v3_cam_right_select),   // 新增
    .resolution_sel         (resolution_sel),              // 分辨率选择输出
    .frame_pixel_count      (frame_pixel_count)            // 帧像素数输出
);

// ========================================================================
// 双目测距模式控制（UDP命令0x00F5解析 - 新增）
// ========================================================================
// 状态机解析UDP命令
reg [1:0] stereo_cmd_state;
reg [7:0] cmd_byte_buf;
localparam ST_CMD_IDLE   = 2'd0;
localparam ST_CMD_BYTE0  = 2'd1;
localparam ST_CMD_BYTE1  = 2'd2;
localparam ST_CMD_BYTE2  = 2'd3;

always @(posedge udp_clk or negedge key1) begin
    if (!key1) begin
        stereo_ranging_enable <= 1'b0;  // 默认RGB模式
        stereo_cmd_state <= ST_CMD_IDLE;
        cmd_byte_buf <= 8'd0;
    end else if (app_rx_data_valid) begin
        case (stereo_cmd_state)
            ST_CMD_IDLE: begin
                // 检测第一个字节 0x00
                if (app_rx_data == 8'h00 && app_rx_data_length >= 16'd3) begin
                    stereo_cmd_state <= ST_CMD_BYTE0;
                end
            end

            ST_CMD_BYTE0: begin
                // 检测第二个字节 0xF5
                if (app_rx_data == 8'hF5) begin
                    stereo_cmd_state <= ST_CMD_BYTE1;
                end else begin
                    stereo_cmd_state <= ST_CMD_IDLE;
                end
            end

            ST_CMD_BYTE1: begin
                // 接收模式字节（0=RGB, 1=测距）
                stereo_ranging_enable <= app_rx_data[0];
                stereo_cmd_state <= ST_CMD_IDLE;
            end

            default: stereo_cmd_state <= ST_CMD_IDLE;
        endcase
    end
end

// ========================================================================
// RGB颜色阈值配置（UDP命令0x0009解析 - 新增）
// ========================================================================
// 命令格式：[0x00][0x09][R_low][R_high][G_low][G_high][B_low][B_high]
reg [2:0] rgb_threshold_cmd_state;

localparam RGB_CMD_IDLE   = 3'd0;
localparam RGB_CMD_BYTE0  = 3'd1;
localparam RGB_CMD_R_LOW  = 3'd2;
localparam RGB_CMD_R_HIGH = 3'd3;
localparam RGB_CMD_G_LOW  = 3'd4;
localparam RGB_CMD_G_HIGH = 3'd5;
localparam RGB_CMD_B_LOW  = 3'd6;
localparam RGB_CMD_B_HIGH = 3'd7;

always @(posedge udp_clk or negedge key1) begin
    if (!key1) begin
        rgb_threshold_cmd_state <= RGB_CMD_IDLE;
        // RGB阈值初始化（默认识别红色：R高，G低，B低）
        udp_binary_threshold_r_low  <= 8'd100;
        udp_binary_threshold_r_high <= 8'd255;
        udp_binary_threshold_g_low  <= 8'd0;
        udp_binary_threshold_g_high <= 8'd100;
        udp_binary_threshold_b_low  <= 8'd0;
        udp_binary_threshold_b_high <= 8'd100;
    end else if (app_rx_data_valid) begin
        case (rgb_threshold_cmd_state)
            RGB_CMD_IDLE: begin
                // 检测第一个字节 0x00
                if (app_rx_data == 8'h00 && app_rx_data_length >= 16'd8) begin
                    rgb_threshold_cmd_state <= RGB_CMD_BYTE0;
                end
            end

            RGB_CMD_BYTE0: begin
                // 检测第二个字节 0x09
                if (app_rx_data == 8'h09) begin
                    rgb_threshold_cmd_state <= RGB_CMD_R_LOW;
                end else begin
                    rgb_threshold_cmd_state <= RGB_CMD_IDLE;
                end
            end

            RGB_CMD_R_LOW: begin
                udp_binary_threshold_r_low <= app_rx_data;
                rgb_threshold_cmd_state <= RGB_CMD_R_HIGH;
            end

            RGB_CMD_R_HIGH: begin
                udp_binary_threshold_r_high <= app_rx_data;
                rgb_threshold_cmd_state <= RGB_CMD_G_LOW;
            end

            RGB_CMD_G_LOW: begin
                udp_binary_threshold_g_low <= app_rx_data;
                rgb_threshold_cmd_state <= RGB_CMD_G_HIGH;
            end

            RGB_CMD_G_HIGH: begin
                udp_binary_threshold_g_high <= app_rx_data;
                rgb_threshold_cmd_state <= RGB_CMD_B_LOW;
            end

            RGB_CMD_B_LOW: begin
                udp_binary_threshold_b_low <= app_rx_data;
                rgb_threshold_cmd_state <= RGB_CMD_B_HIGH;
            end

            RGB_CMD_B_HIGH: begin
                udp_binary_threshold_b_high <= app_rx_data;
                rgb_threshold_cmd_state <= RGB_CMD_IDLE;
            end

            default: rgb_threshold_cmd_state <= RGB_CMD_IDLE;
        endcase
    end
end

// CDC同步：udp_clk → cam_pclk（测距使能）
always @(posedge cam_pclk or negedge key1) begin
    if (!key1) begin
        stereo_ranging_enable_cam_sync1 <= 1'b0;
        stereo_ranging_enable_cam_sync2 <= 1'b0;
    end else begin
        stereo_ranging_enable_cam_sync1 <= stereo_ranging_enable;
        stereo_ranging_enable_cam_sync2 <= stereo_ranging_enable_cam_sync1;
    end
end
assign stereo_ranging_enable_cam = stereo_ranging_enable_cam_sync2;

// CDC同步：cam_pclk → udp_clk（测距距离）
always @(posedge udp_clk or negedge key1) begin
    if (!key1) begin
        stereo_distance_udp_sync1 <= 8'd0;
        stereo_distance_udp_sync2 <= 8'd0;
    end else begin
        stereo_distance_udp_sync1 <= stereo_distance_cam;
        stereo_distance_udp_sync2 <= stereo_distance_udp_sync1;
    end
end

// ========================================================================
// 模板匹配模块（32×32二值化图像 - 实时视频流模式）
// ========================================================================
// CDC跨时钟域同步：UDP上传信号从udp_clk到cam_pclk
reg [15:0] template_byte_count;
reg        app_rx_data_valid_d1;
reg [7:0]  template_cmd_byte1;
reg [15:0] template_cmd;

// UDP时钟域的字节计数和命令提取
always @(posedge udp_clk or negedge key1) begin
    if (!key1) begin
        template_byte_count <= 16'd0;
        app_rx_data_valid_d1 <= 1'b0;
        template_cmd_byte1 <= 8'd0;
        template_cmd <= 16'd0;
    end else begin
        app_rx_data_valid_d1 <= app_rx_data_valid;

        if (app_rx_data_valid) begin
            // 字节计数
            template_byte_count <= template_byte_count + 1'b1;

            // 提取命令码（前两个字节）
            if (template_byte_count == 16'd0) begin
                template_cmd_byte1 <= app_rx_data;
            end else if (template_byte_count == 16'd1) begin
                template_cmd <= {template_cmd_byte1, app_rx_data};
            end
        end else if (!app_rx_data_valid && app_rx_data_valid_d1) begin
            // 数据包结束，复位计数器
            template_byte_count <= 16'd0;
        end
    end
end

// CDC同步到cam_pclk（简化处理，UDP上传时cam_pclk也在运行）
reg        app_rx_data_valid_sync1, app_rx_data_valid_sync2;
reg [7:0]  app_rx_data_sync1, app_rx_data_sync2;
reg [15:0] template_byte_count_sync1, template_byte_count_sync2;
reg [15:0] template_cmd_sync1, template_cmd_sync2;

always @(posedge cam_pclk or negedge key1) begin
    if (!key1) begin
        app_rx_data_valid_sync1 <= 1'b0;
        app_rx_data_valid_sync2 <= 1'b0;
        app_rx_data_sync1 <= 8'd0;
        app_rx_data_sync2 <= 8'd0;
        template_byte_count_sync1 <= 16'd0;
        template_byte_count_sync2 <= 16'd0;
        template_cmd_sync1 <= 16'd0;
        template_cmd_sync2 <= 16'd0;
    end else begin
        // 两级同步
        app_rx_data_valid_sync1 <= app_rx_data_valid;
        app_rx_data_valid_sync2 <= app_rx_data_valid_sync1;
        app_rx_data_sync1 <= app_rx_data;
        app_rx_data_sync2 <= app_rx_data_sync1;
        template_byte_count_sync1 <= template_byte_count;
        template_byte_count_sync2 <= template_byte_count_sync1;
        template_cmd_sync1 <= template_cmd;
        template_cmd_sync2 <= template_cmd_sync1;
    end
end

// 实例化实时模板匹配顶层模块（连接到左目视频流）
vip_template_match u_vip_template_match (
    .clk                    (cam_pclk),              // 使用cam_pclk时钟
    .rst_n                  (key1),

    // UDP接收接口（跨时钟域同步后）
    .udp_rx_valid           (app_rx_data_valid_sync2),
    .udp_rx_cmd             (template_cmd_sync2),
    .udp_rx_data            (app_rx_data_sync2),
    .udp_rx_byte_cnt_valid  (app_rx_data_valid_sync2),
    .udp_rx_byte_cnt        (template_byte_count_sync2),

    // 视频流接口（连接到左目OV5640灰度输出）
    .video_hsyn             (prj1_left_hsyn),        // 左目行同步
    .video_vsyn             (prj1_left_vsyn),        // 左目帧同步
    .video_en               (prj1_left_write_en),    // 左目数据有效
    .video_gray             (prj1_left_gray),        // 左目灰度值

    // 匹配结果输出
    .match_result           (template_match_result),
    .match_result_valid     (template_match_result_valid),
    .hamming_distance       (template_hamming_distance)
);

// 匹配结果跨时钟域同步（cam_pclk → mem_clk用于SDRAM写入）
reg        template_match_result_sync1, template_match_result_sync2;
reg        template_match_result_valid_sync1, template_match_result_valid_sync2;

always @(posedge mem_clk or negedge key1) begin
    if (!key1) begin
        template_match_result_sync1 <= 1'b0;
        template_match_result_sync2 <= 1'b0;
        template_match_result_valid_sync1 <= 1'b0;
        template_match_result_valid_sync2 <= 1'b0;
    end else begin
        // 两级同步
        template_match_result_sync1 <= template_match_result;
        template_match_result_sync2 <= template_match_result_sync1;
        template_match_result_valid_sync1 <= template_match_result_valid;
        template_match_result_valid_sync2 <= template_match_result_valid_sync1;
    end
end

// 匹配结果跨时钟域同步（cam_pclk → udp_clk用于UDP发送）
reg        template_match_result_udp_sync1, template_match_result_udp_sync2;
reg [9:0]  template_hamming_distance_udp_sync1, template_hamming_distance_udp_sync2;

always @(posedge udp_clk or negedge key1) begin
    if (!key1) begin
        template_match_result_udp_sync1 <= 1'b0;
        template_match_result_udp_sync2 <= 1'b0;
        template_hamming_distance_udp_sync1 <= 10'd0;
        template_hamming_distance_udp_sync2 <= 10'd0;
    end else begin
        // 两级同步
        template_match_result_udp_sync1 <= template_match_result;
        template_match_result_udp_sync2 <= template_match_result_udp_sync1;
        template_hamming_distance_udp_sync1 <= template_hamming_distance;
        template_hamming_distance_udp_sync2 <= template_hamming_distance_udp_sync1;
    end
end

// ========================================================================
// q3v3数据源选择（4路：SD卡/左目/右目/双目）
// ========================================================================
// 时钟选择
assign prj1_write_clk_sel = udp_mode_q3v3_internal ?
                           (udp_q3v3_cam_right_select ? cam_r_pclk : cam_pclk) :
                           sd_card_clk;

// 写请求选择
assign prj1_write_req_sel = !udp_mode_q3v3_internal ? prj1_sd_write_req :           // SD卡
                           udp_q3v3_cam_dual_enable ? prj1_dual_write_req :         // 双目
                           udp_q3v3_cam_right_select ? prj1_right_write_req :       // 右目
                                                       prj1_left_write_req;         // 左目

// 写使能选择（左目使用final信号，支持帧差法）
assign prj1_write_en_sel = !udp_mode_q3v3_internal ? prj1_sd_write_en :
                          udp_q3v3_cam_dual_enable ? prj1_dual_write_en :
                          udp_q3v3_cam_right_select ? prj1_right_write_en :
                                                      prj1_left_final_en;

// 写数据选择（左目使用final信号，支持帧差法）
assign prj1_write_data_sel = !udp_mode_q3v3_internal ? prj1_sd_write_data :
                            udp_q3v3_cam_dual_enable ? prj1_dual_write_data :
                            udp_q3v3_cam_right_select ? prj1_right_write_data :
                                                        prj1_left_final_data;

// 应答信号分配
assign prj1_sd_write_req_ack = (!udp_mode_q3v3_internal) ? prj1_write_req_ack_sel : 1'b0;
assign prj1_left_write_req_ack = (udp_mode_q3v3_internal && !udp_q3v3_cam_dual_enable && !udp_q3v3_cam_right_select) ?
                                 prj1_write_req_ack_sel : 1'b0;
assign prj1_right_write_req_ack = (udp_mode_q3v3_internal && !udp_q3v3_cam_dual_enable && udp_q3v3_cam_right_select) ?
                                  prj1_write_req_ack_sel : 1'b0;
assign prj1_dual_write_req_ack = (udp_mode_q3v3_internal && udp_q3v3_cam_dual_enable) ?
                                 prj1_write_req_ack_sel : 1'b0;

// ========================================================================
// 时钟生成模块
// ========================================================================
time_top u_time_top (
    .clk_50m            (clk_50),
    .rst_n              (key1),
    .resolution_sel     (resolution_sel),        // 分辨率选择输入
    .clk_125m_0_1       (clk_125m_0_1),
    .clk_125m_0_2       (clk_125m_0_2),
    .clk_125m_90        (clk_125m_90),
    .clk_25m_0          (clk_25m_0),
    .clk_12_5m_0        (clk_12_5m_0),
    .clk_50m_0          (clk_50m_0),
    .clk_50m_180        (clk_50m_180),
    .clk_1_25m_0        (clk_1_25m_0),
    .pll_locked         (pll_locked),
    .video_clk_out      (video_clk_selected),    // 选择后的像素时钟
    .video_clk_5x_out   (video_clk_5x_selected), // 选择后的5x时钟
    .video_pll_locked   (video_pll_locked)       // video_pll锁定信号
);

// q3v3的系统PLL（用于SD卡时钟）
sys_pll sys_pll_m0(
    .refclk         (clk_50),
    .clk0_out       (sd_card_clk),
    .clk1_out       (),
    .clk2_out       (),
    .reset          (1'b0)
);

// UDP时钟生成
udp_clk_gen #(
    .DEVICE(DEVICE)
) u_udp_clk_gen (
    .reset          (~key1),
    .tri_speed      (TRI_speed),
    .clk_125_in     (clk_125m_0_1),
    .clk_12_5_in    (clk_12_5m_0),
    .clk_1_25_in    (clk_1_25m_0),
    .udp_clk_out    (udp_clk)
);

// ========================================================================
// SDRAM IP核
// ========================================================================
sdram u_sdram (
    .Clk            (mem_clk),
    .Clk_sft        (mem_clk_sft),
    .Rst            (reset),
    .Sdr_init_done  (Sdr_init_done),
    .Sdr_init_ref_vld(Sdr_init_ref_vld),
    .Sdr_busy       (Sdr_busy),
    .App_wr_en      (sdram_App_wr_en),
    .App_wr_addr    (sdram_App_wr_addr),
    .App_wr_dm      (sdram_App_wr_dm),
    .App_wr_din     (sdram_App_wr_din),
    .App_rd_en      (sdram_App_rd_en),
    .App_rd_addr    (sdram_App_rd_addr),
    .Sdr_rd_en      (sdram_Sdr_rd_en),
    .Sdr_rd_dout    (sdram_Sdr_rd_dout)
);

// ========================================================================
// SDRAM仲裁器（关键！确保两个工程隔离）
// ========================================================================
sdram_arbiter #(
    .ADDR_BITS          (ADDR_BITS),
    .DATA_BITS          (MEM_DATA_BITS),
    .ADDR_OFFSET_PROJECT1(21'd2073600)  // q3v3地址偏移（支持1080p最大尺寸）
) u_sdram_arbiter (
    .mode_project       (udp_mode_project),
    // bp12_2工程接口
    .prj0_App_rd_en     (prj0_App_rd_en),
    .prj0_App_rd_addr   (prj0_App_rd_addr),
    .prj0_App_wr_en     (prj0_App_wr_en),
    .prj0_App_wr_addr   (prj0_App_wr_addr),
    .prj0_App_wr_din    (prj0_App_wr_din),
    .prj0_App_wr_dm     (prj0_App_wr_dm),
    .prj0_Sdr_rd_en     (prj0_Sdr_rd_en),
    .prj0_Sdr_rd_dout   (prj0_Sdr_rd_dout),
    // q3v3工程接口
    .prj1_App_rd_en     (prj1_App_rd_en),
    .prj1_App_rd_addr   (prj1_App_rd_addr),
    .prj1_App_wr_en     (prj1_App_wr_en),
    .prj1_App_wr_addr   (prj1_App_wr_addr),
    .prj1_App_wr_din    (prj1_App_wr_din),
    .prj1_App_wr_dm     (prj1_App_wr_dm),
    .prj1_Sdr_rd_en     (prj1_Sdr_rd_en),
    .prj1_Sdr_rd_dout   (prj1_Sdr_rd_dout),
    // 真实SDRAM接口
    .sdram_App_rd_en    (sdram_App_rd_en),
    .sdram_App_rd_addr  (sdram_App_rd_addr),
    .sdram_App_wr_en    (sdram_App_wr_en),
    .sdram_App_wr_addr  (sdram_App_wr_addr),
    .sdram_App_wr_din   (sdram_App_wr_din),
    .sdram_App_wr_dm    (sdram_App_wr_dm),
    .sdram_Sdr_rd_en    (sdram_Sdr_rd_en),
    .sdram_Sdr_rd_dout  (sdram_Sdr_rd_dout)
);

// ========================================================================
// 项目0: 图像接收模式模块实例化
// ========================================================================

// 数码管/LED控制
wire [7:0] bp12_tub;
wire [7:0] bp12_dled;

led u_img_rx_led (
    .udp_rx_clk         (udp_clk),
    .reset              (key1),
    .app_rx_data_valid  (seg_data_valid),
    .app_rx_data        (app_rx_data),
    .app_rx_data_length (app_rx_data_length),
    .mode_bp12_led_seg  (udp_mode_bp12_led_seg),
    .dled               (bp12_dled),
    .led_data_1         (led_data),
    .tub                (bp12_tub)
);

// ========================================================================
// 项目1: 数码管状态显示 (SD卡状态)
// ========================================================================
wire [6:0] q3v3_seg_data_0;
wire [7:0] q3v3_tub;
wire [7:0] q3v3_dled;

// 状态码解码器
seg_decoder u_cam_seg_decoder (
    .bin_data           (sd_state_code),
    .seg_data           (q3v3_seg_data_0)
);

// 数码管扫描驱动
seg_scan u_cam_seg_scan (
    .clk                (clk_50),
    .rst_n              (key1),
    .seg_sel            (q3v3_dled[5:0]),         // 6位数码管位选
    .seg_data           (q3v3_tub),               // 8位数码管段选
    .seg_data_0         ({1'b1, q3v3_seg_data_0}), // 最右边：显示SD卡状态码
    .seg_data_1         (8'b11111111),
    .seg_data_2         (8'b11111111),
    .seg_data_3         (8'b11111111),
    .seg_data_4         (8'b11111111),
    .seg_data_5         ({1'b1, udp_mode_q3v3_internal ? 7'b0001110 : 7'b0010010}) // 最左边：显示模式: C(摄像头) 或 S(SD卡)
);

// 将6位扩展为8位（高2位置1）
assign q3v3_dled[7:6] = 2'b11;

// ========================================================================
// LED和数码管输出选择（根据工程模式）
// ========================================================================
assign tub  = udp_mode_project ? q3v3_tub  : bp12_tub;
assign dled = udp_mode_project ? q3v3_dled : bp12_dled;

// UDP接收数据写入SDRAM (支持动态分辨率)
udp_to_sdram_writer #(
    .MAX_PIXEL_COUNT(2073600)  // 最大支持1920×1080
) u_img_rx_writer (
    .clk                (udp_clk),
    .reset              (reset),
    .pixel_count_in     (frame_pixel_count),  // 动态像素数输入
    .udp_data_valid     (img_data_valid),
    .udp_data           (app_rx_data),
    .write_req          (prj0_write_req),
    .write_req_ack      (prj0_write_req_ack),
    .write_en           (prj0_write_en),
    .write_data         (prj0_write_data)
);

// SDRAM读写控制 (连接到仲裁器)
frame_read_write #(
    .ADDR_BITS          (ADDR_BITS),
    .READ_DATA_BITS     (32),
    .WRITE_DATA_BITS    (32)
) u_img_rx_frame_rw (
    .mem_clk            (mem_clk),
    .rst                (reset),
    .Sdr_init_done      (Sdr_init_done),
    .Sdr_init_ref_vld   (1'b0),
    .Sdr_busy           (1'b0),
    // 读接口
    .App_rd_en          (prj0_App_rd_en),
    .App_rd_addr        (prj0_App_rd_addr),
    .Sdr_rd_en          (prj0_Sdr_rd_en),
    .Sdr_rd_dout        (prj0_Sdr_rd_dout),
    .read_clk           (video_clk),
    .read_req           (prj0_video_read_req),
    .read_req_ack       (prj0_video_read_req_ack),
    .read_finish        (),
    .read_addr_0        (21'd0),
    .read_addr_1        (21'd0),
    .read_addr_2        (21'd0),
    .read_addr_3        (21'd0),
    .read_addr_index    (2'd0),
    .read_len           (frame_pixel_count),  // 使用动态像素数
    .read_en            (prj0_video_read_en),
    .read_data          (prj0_video_read_data),
    // 写接口
    .App_wr_en          (prj0_App_wr_en),
    .App_wr_addr        (prj0_App_wr_addr),
    .App_wr_din         (prj0_App_wr_din),
    .App_wr_dm          (prj0_App_wr_dm),
    .write_clk          (udp_clk),
    .write_req          (prj0_write_req),
    .write_req_ack      (prj0_write_req_ack),
    .write_finish       (),
    .write_addr_0       (21'd0),
    .write_addr_1       (21'd0),
    .write_addr_2       (21'd0),
    .write_addr_3       (21'd0),
    .write_addr_index   (2'd0),
    .write_len          (frame_pixel_count),  // 使用动态像素数
    .write_en           (prj0_write_en),
    .write_data         (prj0_write_data)
);

// 视频时序生成器
video_timing_data u_img_rx_video_timing (
    .video_clk          (video_clk),
    .rst                (reset),
    .resolution_sel     (resolution_sel),  // 分辨率选择
    .read_req           (prj0_video_read_req),
    .read_req_ack       (prj0_video_read_req_ack),
    .hs                 (prj0_hs),
    .vs                 (prj0_vs),
    .de                 (prj0_de)
);

// 视频数据延迟对齐
wire prj0_hs_r, prj0_vs_r, prj0_de_r;
video_delay u_img_rx_video_delay (
    .video_clk          (video_clk),
    .rst                (reset),
    .read_en            (prj0_video_read_en),
    .read_data          (prj0_video_read_data[23:0]),
    .hs                 (prj0_hs),
    .vs                 (prj0_vs),
    .de                 (prj0_de),
    .hs_r               (prj0_hs_r),
    .vs_r               (prj0_vs_r),
    .de_r               (prj0_de_r),
    .vout_data          (prj0_vout_data)
);

// HDMI视频发送器
hdmi_tx #(
    .FAMILY("EG4")
) u_img_rx_hdmi_tx (
    .PXLCLK_I           (video_clk),
    .PXLCLK_5X_I        (video_clk_5x),
    .RST_N              (~reset),
    .VGA_HS             (prj0_hs_r),
    .VGA_VS             (prj0_vs_r),
    .VGA_DE             (prj0_de_r),
    .VGA_RGB            (prj0_vout_data),
    .HDMI_CLK_P         (HDMI_CLK_P),
    .HDMI_D2_P          (HDMI_D2_P),
    .HDMI_D1_P          (HDMI_D1_P),
    .HDMI_D0_P          (HDMI_D0_P)
);

// UDP回环测试
udp_loopback #(
    .DEVICE(DEVICE)
) u_img_rx_loopback (
    .app_rx_clk         (udp_clk),
    .app_tx_clk         (udp_clk),
    .reset              (reset),
    .app_rx_data        (app_rx_data),
    .app_rx_data_valid  (app_rx_data_valid),
    .app_rx_data_length (app_rx_data_length),
    .udp_tx_ready       (udp_tx_ready),
    .app_tx_ack         (app_tx_ack),
    .app_tx_data        (prj0_app_tx_data),
    .app_tx_data_request(prj0_app_tx_data_request),
    .app_tx_data_valid  (prj0_app_tx_data_valid),
    .udp_data_length    (prj0_udp_data_length)
);

// ========================================================================
// 项目1: 摄像头采集模式模块实例化
// ========================================================================

// SD卡BMP图像读取
sd_card_bmp u_cam_sd_card (
    .clk                (sd_card_clk),
    .rst                (~key1),
    .key                (udp_sd_trigger),
    .state_code         (sd_state_code),          // 获取SD卡状态码
    .bmp_width          (16'd640),
    .write_req          (prj1_sd_write_req),
    .write_req_ack      (prj1_sd_write_req_ack),
    .write_en           (prj1_sd_write_en),
    .write_data         (prj1_sd_write_data),
    .SD_nCS             (sd_ncs),
    .SD_DCLK            (sd_dclk),
    .SD_MOSI            (sd_mosi),
    .SD_MISO            (sd_miso)
);

// ========================================================================
// 左目OV5640摄像头驱动
// ========================================================================
ov5640_dri u_cam_ov5640_left (
    .clk                (clk_50),
    .rst_n              (key1),
    .cam_pclk           (cam_pclk),
    .cam_vsync          (cam_vsync),
    .cam_href           (cam_href),
    .cam_data           (cam_data),
    .cam_rst_n          (cam_rst_n),
    .cam_pwdn           (cam_pwdn),
    .cam_scl            (cam_scl),
    .cam_sda            (cam_sda),
    // 在单目左、单目右、双目模式下都启动（需要左目数据）
    .capture_start      (Sdr_init_done && udp_mode_project && udp_mode_q3v3_internal),
    .cmos_h_pixel       (H_CMOS_DISP),
    .cmos_v_pixel       (V_CMOS_DISP),
    .total_h_pixel      (TOTAL_H_PIXEL),
    .total_v_pixel      (TOTAL_V_PIXEL),
    .cmos_frame_vsync   (cmos_frame_vsync),
    .cmos_frame_href    (cmos_frame_href),
    .cmos_frame_valid   (cmos_frame_valid),
    .cmos_frame_data    (cmos_wr_data)
);

// ========================================================================
// 右目OV5640摄像头驱动 (双目立体视觉)
// ========================================================================
ov5640_dri u_cam_ov5640_right (
    .clk                (clk_50),
    .rst_n              (key1),
    .cam_pclk           (cam_r_pclk),
    .cam_vsync          (cam_r_vsync),
    .cam_href           (cam_r_href),
    .cam_data           (cam_r_data),
    .cam_rst_n          (cam_r_rst_n),
    .cam_pwdn           (cam_r_pwdn),
    .cam_scl            (cam_r_scl),
    .cam_sda            (cam_r_sda),
    // 在单目右或双目模式下启动
    .capture_start      (Sdr_init_done && udp_mode_project && udp_mode_q3v3_internal &&
                        (udp_q3v3_cam_right_select || udp_q3v3_cam_dual_enable)),
    .cmos_h_pixel       (H_CMOS_DISP),
    .cmos_v_pixel       (V_CMOS_DISP),
    .total_h_pixel      (TOTAL_H_PIXEL),
    .total_v_pixel      (TOTAL_V_PIXEL),
    .cmos_frame_vsync   (cmos_frame_vsync_r),
    .cmos_frame_href    (cmos_frame_href_r),
    .cmos_frame_valid   (cmos_frame_valid_r),
    .cmos_frame_data    (cmos_wr_data_r)
);

// ========================================================================
// CDC时钟域同步: UDP控制信号 → 左目摄像头时钟域
// ========================================================================
always @(posedge cam_pclk or negedge key1) begin
    if (!key1) begin
        image_proc_mode_sync1 <= 3'b000;
        image_proc_mode_sync2 <= 3'b000;
        threshold_low_sync1 <= 8'd50;
        threshold_low_sync2 <= 8'd50;
        threshold_high_sync1 <= 8'd200;
        threshold_high_sync2 <= 8'd200;
        border_enable_sync1 <= 1'b0;
        border_enable_sync2 <= 1'b0;
    end else begin
        // 图像处理模式同步
        image_proc_mode_sync1 <= udp_image_proc_mode;
        image_proc_mode_sync2 <= image_proc_mode_sync1;

        // 二值化阈值同步（两级触发器防止亚稳态）
        threshold_low_sync1 <= udp_binary_threshold_low;
        threshold_low_sync2 <= threshold_low_sync1;
        threshold_high_sync1 <= udp_binary_threshold_high;
        threshold_high_sync2 <= threshold_high_sync1;

        // 边框使能同步（两级触发器防止亚稳态）
        border_enable_sync1 <= udp_border_enable;
        border_enable_sync2 <= border_enable_sync1;
    end
end

// ========================================================================
// CDC时钟域同步: UDP控制信号 → 右目摄像头时钟域
// ========================================================================
always @(posedge cam_r_pclk or negedge key1) begin
    if (!key1) begin
        image_proc_mode_sync1_r <= 3'b000;
        image_proc_mode_sync2_r <= 3'b000;
        threshold_low_sync1_r <= 8'd50;
        threshold_low_sync2_r <= 8'd50;
        threshold_high_sync1_r <= 8'd200;
        threshold_high_sync2_r <= 8'd200;
        border_enable_sync1_r <= 1'b0;
        border_enable_sync2_r <= 1'b0;
    end else begin
        image_proc_mode_sync1_r <= udp_image_proc_mode;
        image_proc_mode_sync2_r <= image_proc_mode_sync1_r;
        threshold_low_sync1_r <= udp_binary_threshold_low;
        threshold_low_sync2_r <= threshold_low_sync1_r;
        threshold_high_sync1_r <= udp_binary_threshold_high;
        threshold_high_sync2_r <= threshold_high_sync1_r;
        border_enable_sync1_r <= udp_border_enable;
        border_enable_sync2_r <= border_enable_sync1_r;
    end
end

// ========================================================================
// 左目图像处理流水线 (RGB565→RGB888 + 多种图像处理算法)
// ========================================================================
wire [7:0] prj1_left_gray;  // 灰度输出 (供帧差法使用)

ov5640_delay u_cam_left_delay (
    .clk                (cam_pclk),
    .rst_n              (key1),

    // 输入：左目摄像头RGB565数据
    .cmos_frame_vsync   (cmos_frame_vsync),
    .cmos_frame_href    (cmos_frame_href),
    .cmos_frame_valid   (cmos_frame_valid),
    .cmos_wr_data       (cmos_wr_data),

    // 图像处理控制（CDC同步后的信号）
    .i_proc_mode        (image_proc_mode_sync2),
    .i_threshold_low    (threshold_low_sync2),
    .i_threshold_high   (threshold_high_sync2),
    .i_border_enable    (border_enable_sync2),

    // 输出：处理后的RGB888数据（时序已对齐）
    .cam_write_req      (prj1_left_write_req),
    .cam_write_req_ack  (prj1_left_write_req_ack),
    .cam_write_en       (prj1_left_write_en),
    .cam_write_data     (prj1_left_write_data),
    .cam_gray_out       (prj1_left_gray)  // 灰度输出
);

// ========================================================================
// 帧差法运动检测 (基于左目灰度图像)
// ========================================================================
wire        prj1_framediff_hs, prj1_framediff_vs, prj1_framediff_en;
wire [7:0]  prj1_framediff_gray;
wire        prj1_framediff_motion;

frame_diff_downsample u_cam_left_framediff (
    .i_clk      (cam_pclk),
    .i_rst_n    (key1),
    .i_hsyn     (cmos_frame_href),
    .i_vsyn     (cmos_frame_vsync),
    .i_en       (cmos_frame_valid),
    .i_gray     (prj1_left_gray),       // 使用左目灰度输出
    .o_hs       (prj1_framediff_hs),
    .o_vs       (prj1_framediff_vs),
    .o_en       (prj1_framediff_en),
    .o_diff     (prj1_framediff_gray),
    .o_motion   (prj1_framediff_motion) // 运动检测标志输出
);

// ========================================================================
// 运动边框叠加 (在原图上标注运动区域)
// ========================================================================
wire        prj1_border_hsyn, prj1_border_vsyn, prj1_border_en;
wire [31:0] prj1_border_data;

motion_border_overlay u_cam_motion_border (
    .i_clk      (cam_pclk),
    .i_rst_n    (key1),
    .i_hsyn     (cmos_frame_href),
    .i_vsyn     (cmos_frame_vsync),
    .i_en       (cmos_frame_valid),
    .i_data     (prj1_left_write_data),      // 原始RGB888图像
    .i_motion   (prj1_framediff_motion),     // 运动检测标志
    .o_hsyn     (prj1_border_hsyn),
    .o_vsyn     (prj1_border_vsyn),
    .o_en       (prj1_border_en),
    .o_data     (prj1_border_data)           // 叠加边框后的图像
);

// ========================================================================
// 左目最终输出选择 (图像处理结果 or 运动边框标注)
// ========================================================================
wire        prj1_left_final_en;
wire [31:0] prj1_left_final_data;

assign prj1_left_final_en = (image_proc_mode_sync2 == 3'b111) ?
                             prj1_border_en : prj1_left_write_en;

assign prj1_left_final_data = (image_proc_mode_sync2 == 3'b111) ?
                               prj1_border_data :
                               prj1_left_write_data;

// ========================================================================
// 右目图像处理流水线 (RGB565→RGB888 + 多种图像处理算法)
// ========================================================================
wire [7:0] prj1_right_gray;

ov5640_delay u_cam_right_delay (
    .clk                (cam_r_pclk),
    .rst_n              (key1),

    // 输入：右目摄像头RGB565数据
    .cmos_frame_vsync   (cmos_frame_vsync_r),
    .cmos_frame_href    (cmos_frame_href_r),
    .cmos_frame_valid   (cmos_frame_valid_r),
    .cmos_wr_data       (cmos_wr_data_r),

    // 图像处理控制（CDC同步后的信号，与左目共用配置）
    .i_proc_mode        (image_proc_mode_sync2_r),
    .i_threshold_low    (threshold_low_sync2_r),
    .i_threshold_high   (threshold_high_sync2_r),
    .i_border_enable    (border_enable_sync2_r),

    // 输出：处理后的RGB888数据（时序已对齐）
    .cam_write_req      (prj1_right_write_req),
    .cam_write_req_ack  (prj1_right_write_req_ack),
    .cam_write_en       (prj1_right_write_en),
    .cam_write_data     (prj1_right_write_data),
    .cam_gray_out       (prj1_right_gray)  // 灰度输出（右目不使用帧差法）
);

// ========================================================================
// 双目同步与立体视觉处理 (左右目数据合并 + 测距)
// ========================================================================
dual_cam_delay u_cam_dual_delay (
    // 左目输入（原始数据，不经过图像处理）
    .cmos_pclk_l        (cam_pclk),
    .cmos_frame_vsync_l (cmos_frame_vsync),
    .cmos_frame_href_l  (cmos_frame_href),
    .cmos_frame_valid_l (cmos_frame_valid),
    .cmos_wr_data_l     (cmos_wr_data),

    // 右目输入（原始数据）
    .cmos_pclk_r        (cam_r_pclk),
    .cmos_frame_vsync_r (cmos_frame_vsync_r),
    .cmos_frame_href_r  (cmos_frame_href_r),
    .cmos_frame_valid_r (cmos_frame_valid_r),
    .cmos_wr_data_r     (cmos_wr_data_r),

    // 复位
    .rst_n              (key1),

    // 双目测距控制（新增）
    .stereo_ranging_enable      (stereo_ranging_enable_cam),
    .binary_threshold_r_low     (udp_binary_threshold_r_low),
    .binary_threshold_r_high    (udp_binary_threshold_r_high),
    .binary_threshold_g_low     (udp_binary_threshold_g_low),
    .binary_threshold_g_high    (udp_binary_threshold_g_high),
    .binary_threshold_b_low     (udp_binary_threshold_b_low),
    .binary_threshold_b_high    (udp_binary_threshold_b_high),

    // 输出：32位双目数据（左16bit + 右16bit）
    .cam_write_req      (prj1_dual_write_req),
    .cam_write_req_ack  (prj1_dual_write_req_ack),
    .cam_write_en       (prj1_dual_write_en),
    .cam_write_data     (prj1_dual_write_data),

    // 双目测距输出（新增）
    .stereo_distance    (stereo_distance_cam)
);

// ========================================================================
// UDP发送路径选择 (单目 or 双目)
// ========================================================================
// 读请求选择（发送给frame_read_write）
assign prj1_video_read_req = udp_q3v3_cam_dual_enable ?
                             prj1_dual_read_req : prj1_single_read_req;

assign prj1_video_read_en = udp_q3v3_cam_dual_enable ?
                            prj1_dual_read_en : prj1_single_read_en;

// 应答和数据分发（两个模块都连接，通过ack信号区分谁在工作）
assign prj1_single_read_req_ack = udp_q3v3_cam_dual_enable ? 1'b0 : prj1_video_read_req_ack;
assign prj1_dual_read_req_ack = udp_q3v3_cam_dual_enable ? prj1_video_read_req_ack : 1'b0;

// UDP发送数据选择（发送给UDP协议栈）
assign prj1_app_tx_data = udp_q3v3_cam_dual_enable ?
                         prj1_dual_app_tx_data : prj1_single_app_tx_data;
assign prj1_app_tx_data_valid = udp_q3v3_cam_dual_enable ?
                               prj1_dual_app_tx_data_valid : prj1_single_app_tx_data_valid;
assign prj1_app_tx_data_request = udp_q3v3_cam_dual_enable ?
                                 prj1_dual_app_tx_data_request : prj1_single_app_tx_data_request;
assign prj1_udp_data_length = udp_q3v3_cam_dual_enable ?
                             prj1_dual_udp_data_length : prj1_single_udp_data_length;

// ========================================================================
// 单目UDP发送控制 (SDRAM读取 + UDP封包, RGB888格式)
// ========================================================================
udp_ov5640_send u_ov5640_send (
    .clk                (udp_clk),
    .rst_n              (key1 & Sdr_init_done),
    .read_req           (prj1_single_read_req),
    .read_req_ack       (prj1_single_read_req_ack),
    .read_en            (prj1_single_read_en),
    .read_data          (prj1_video_read_data),
    .udp_tx_ready       (udp_tx_ready),
    .app_tx_ack         (app_tx_ack),
    .app_tx_data_request(prj1_single_app_tx_data_request),
    .app_tx_data_valid  (prj1_single_app_tx_data_valid),
    .app_tx_data        (prj1_single_app_tx_data),
    .udp_data_length    (prj1_single_udp_data_length)
);

// ========================================================================
// 双目UDP发送控制 (SDRAM读取 + UDP封包 + 测距数据)
// ========================================================================
dual_cam_send u_cam_dual_send (
    .clk                (udp_clk),
    .rst_n              (key1 & Sdr_init_done),
    .read_req           (prj1_dual_read_req),
    .read_req_ack       (prj1_dual_read_req_ack),
    .read_en            (prj1_dual_read_en),
    .read_data          (prj1_video_read_data),
    // 模板匹配结果输入（CDC同步后）
    .match_result       (template_match_result_udp_sync2),
    .hamming_distance   (template_hamming_distance_udp_sync2),
    // 双目测距输入（新增）
    .stereo_ranging_enable  (stereo_ranging_enable),
    .stereo_distance        (stereo_distance_udp_sync2),
    // UDP发送接口
    .udp_tx_ready       (udp_tx_ready),
    .app_tx_ack         (app_tx_ack),
    .app_tx_data_request(prj1_dual_app_tx_data_request),
    .app_tx_data_valid  (prj1_dual_app_tx_data_valid),
    .app_tx_data        (prj1_dual_app_tx_data),
    .udp_data_length    (prj1_dual_udp_data_length)
);

// SDRAM读写控制 (连接到仲裁器)
frame_read_write #(
    .ADDR_BITS          (ADDR_BITS),
    .READ_DATA_BITS     (32),
    .WRITE_DATA_BITS    (32)
) u_cam_frame_rw (
    .mem_clk            (mem_clk),
    .rst                (~key1),
    .Sdr_init_done      (Sdr_init_done),
    .Sdr_init_ref_vld   (Sdr_init_ref_vld),
    .Sdr_busy           (Sdr_busy),
    // 读接口（UDP发送）
    .App_rd_en          (prj1_App_rd_en),
    .App_rd_addr        (prj1_App_rd_addr),
    .Sdr_rd_en          (prj1_Sdr_rd_en),
    .Sdr_rd_dout        (prj1_Sdr_rd_dout),
    .read_clk           (udp_clk),
    .read_req           (prj1_video_read_req),
    .read_req_ack       (prj1_video_read_req_ack),
    .read_finish        (),
    .read_addr_0        (24'd0),
    .read_addr_1        (24'd0),
    .read_addr_2        (24'd0),
    .read_addr_3        (24'd0),
    .read_addr_index    (2'd0),
    .read_len           (24'd307200),
    .read_en            (prj1_video_read_en),
    .read_data          (prj1_video_read_data),
    // 写接口（SD卡/摄像头）
    .App_wr_en          (prj1_App_wr_en),
    .App_wr_addr        (prj1_App_wr_addr),
    .App_wr_din         (prj1_App_wr_din),
    .App_wr_dm          (prj1_App_wr_dm),
    .write_clk          (prj1_write_clk_sel),
    .write_req          (prj1_write_req_sel),
    .write_req_ack      (prj1_write_req_ack_sel),
    .write_finish       (),
    .write_addr_0       (24'd0),
    .write_addr_1       (24'd0),
    .write_addr_2       (24'd0),
    .write_addr_3       (24'd0),
    .write_addr_index   (2'd0),
    .write_len          (24'd307200),
    .write_en           (prj1_write_en_sel),
    .write_data         (prj1_write_data_sel)
);

// ========================================================================
// UDP/IP 协议栈（两个工程共用）
// ========================================================================
udp_ip_protocol_stack #(
    .DEVICE             (DEVICE),
    .LOCAL_UDP_PORT_NUM (LOCAL_UDP_PORT_NUM),
    .LOCAL_IP_ADDRESS   (LOCAL_IP_ADDRESS),
    .LOCAL_MAC_ADDRESS  (LOCAL_MAC_ADDRESS)
) u_udp_stack (
    .udp_rx_clk         (udp_clk),
    .udp_tx_clk         (udp_clk),
    .reset              (reset),
    .udp2app_tx_ready   (udp_tx_ready),
    .udp2app_tx_ack     (app_tx_ack),
    .app_tx_request     (app_tx_data_request),
    .app_tx_data_valid  (app_tx_data_valid),
    .app_tx_data        (app_tx_data),
    .app_tx_data_length (udp_data_length),
    .app_tx_dst_port    (DST_UDP_PORT_NUM),
    .ip_tx_dst_address  (DST_IP_ADDRESS),
    .input_local_udp_port_num(LOCAL_UDP_PORT_NUM),
    .input_local_udp_port_num_valid(1'b0),
    .input_local_ip_address(LOCAL_IP_ADDRESS),
    .input_local_ip_address_valid(1'b0),
    .app_rx_data_valid  (app_rx_data_valid),
    .app_rx_data        (app_rx_data),
    .app_rx_data_length (app_rx_data_length),
    .app_rx_port_num    (app_rx_port_num),
    .temac_rx_ready     (temac_rx_ready),
    .temac_rx_valid     (!temac_rx_valid),
    .temac_rx_data      (temac_rx_data),
    .temac_rx_sof       (temac_rx_sof),
    .temac_rx_eof       (temac_rx_eof),
    .temac_tx_ready     (temac_tx_ready),
    .temac_tx_valid     (temac_tx_valid),
    .temac_tx_data      (temac_tx_data),
    .temac_tx_sof       (temac_tx_sof),
    .temac_tx_eof       (temac_tx_eof),
    .ip_rx_error        (),
    .arp_request_no_reply_error()
);

// ========================================================================
// TEMAC以太网MAC
// ========================================================================
temac_block #(
    .DEVICE(DEVICE)
) u_temac (
    .reset              (reset),
    .gtx_clk            (temac_clk),
    .gtx_clk_90         (clk_125m_90),
    .rx_clk             (rx_clk_int),
    .rx_clk_en          (rx_clk_en_int),
    .rx_data            (rx_data),
    .rx_data_valid      (rx_valid),
    .rx_correct_frame   (rx_correct_frame),
    .rx_error_frame     (rx_error_frame),
    .rx_status_vector   (),
    .rx_status_vld      (),
    .tx_clk             (tx_clk_int),
    .tx_clk_en          (tx_clk_en_int),
    .tx_data            (tx_data),
    .tx_data_en         (tx_valid),
    .tx_rdy             (tx_rdy),
    .tx_stop            (tx_stop),
    .tx_collision       (tx_collision),
    .tx_retransmit      (tx_retransmit),
    .tx_ifg_val         (tx_ifg_val),
    .tx_status_vector   (),
    .tx_status_vld      (),
    .pause_req          (pause_req),
    .pause_val          (pause_val),
    .pause_source_addr  (pause_source_addr),
    .unicast_address    (unicast_address),
    .mac_cfg_vector     (mac_cfg_vector),
    .rgmii_txd          (phy1_rgmii_tx_data),
    .rgmii_tx_ctl       (phy1_rgmii_tx_ctl),
    .rgmii_txc          (phy1_rgmii_tx_clk),
    .rgmii_rxd          (phy1_rgmii_rx_data),
    .rgmii_rx_ctl       (phy1_rgmii_rx_ctl),
    .rgmii_rxc          (phy1_rgmii_rx_clk),
    .inband_link_status (),
    .inband_clock_speed (),
    .inband_duplex_status()
);

// ========================================================================
// TX/RX FIFO
// ========================================================================
tx_client_fifo #(
    .DEVICE(DEVICE)
) u_tx_fifo (
    .rd_clk             (tx_clk_int),
    .rd_sreset          (reset),
    .rd_enable          (tx_clk_en_int),
    .tx_data            (tx_data),
    .tx_data_valid      (tx_valid),
    .tx_ack             (tx_rdy),
    .tx_collision       (tx_collision),
    .tx_retransmit      (tx_retransmit),
    .overflow           (),
    .wr_clk             (udp_clk),
    .wr_sreset          (reset),
    .wr_data            (temac_tx_data),
    .wr_sof_n           (temac_tx_sof),
    .wr_eof_n           (temac_tx_eof),
    .wr_src_rdy_n       (temac_tx_valid),
    .wr_dst_rdy_n       (temac_tx_ready),
    .wr_fifo_status     ()
);

rx_client_fifo #(
    .DEVICE(DEVICE)
) u_rx_fifo (
    .wr_clk             (rx_clk_int),
    .wr_enable          (rx_clk_en_int),
    .wr_sreset          (reset),
    .rx_data            (rx_data),
    .rx_data_valid      (rx_valid),
    .rx_good_frame      (rx_correct_frame),
    .rx_bad_frame       (rx_error_frame),
    .overflow           (),
    .rd_clk             (udp_clk),
    .rd_sreset          (reset),
    .rd_data_out        (temac_rx_data),
    .rd_sof_n           (temac_rx_sof),
    .rd_eof_n           (temac_rx_eof),
    .rd_src_rdy_n       (temac_rx_valid),
    .rd_dst_rdy_n       (temac_rx_ready),
    .rx_fifo_status     ()
);

endmodule
