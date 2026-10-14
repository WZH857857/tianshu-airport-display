`timescale 1ns / 1ps
//********************************************************************************
// Module: udp_mode_ctrl
// Description: UDP模式控制解析模块 - 解析上位机发送的控制命令
//
// 命令格式：
//   2字节命令（模式切换）：
//     0x0001: LED模式（bp12_2工程，12个LED控制）
//     0x0002: 数码管模式（bp12_2工程，8个数码管+4个单色LED控制）
//     0x0003: SD卡模式+触发读取（q3v3工程，SD卡写SDRAM并UDP发送）
//     0x0004: 左目摄像头 - 原始图像（q3v3工程，摄像头写SDRAM并UDP发送）
//     0x0005: 左目摄像头 - 二值化图像
//     0x0006: 左目摄像头 - 腐蚀图像
//     0x0007: 左目摄像头 - 开运算图像
//     0x000A: 左目摄像头 - 浮雕效果图像
//     0x000B: 左目摄像头 - 拉普拉斯锐化图像
//     0x000C: 左目摄像头 - Sobel边缘检测图像
//     0x0010: 右目摄像头 - 原始图像
//     0x0012: 右目摄像头 - 二值化图像
//     0x0013: 右目摄像头 - 腐蚀图像
//     0x0014: 右目摄像头 - 开运算图像
//     0x0015: 右目摄像头 - 浮雕效果图像
//     0x0016: 右目摄像头 - 拉普拉斯锐化图像
//     0x0017: 右目摄像头 - Sobel边缘检测图像
//     0x0011: 双目摄像头（无图像处理，原始图像）
//
//   4字节命令（参数配置）：
//     0x0008 + [低阈值] + [高阈值]: 设置二值化高低阈值（0-255）
//
//   2字节命令（开关控制）：
//     0x0009: 切换边框检测使能（toggle）
//
//   2字节命令（分辨率控制）：
//     0x0030: 设置为640×480分辨率
//     0x0031: 设置为1280×720分辨率（720p）
//     0x0032: 设置为1920×1080分辨率（1080p）
//
//********************************************************************************

module udp_mode_ctrl(
    input                       udp_rx_clk,
    input                       reset,

    // UDP接收接口
    input                       app_rx_data_valid,
    input       [7:0]           app_rx_data,
    input       [15:0]          app_rx_data_length,

    // 按键接口（新增）
    input                       key2,                   // SD卡触发按键（对应原q3v3的key1）
    input                       key3,                   // 模式切换按键（对应原q3v3的key2）

    // 模式控制输出
    output reg                  mode_project,           // 0: bp12_2, 1: q3v3
    output reg                  mode_q3v3_internal,     // q3v3内部模式: 0: SD卡, 1: 摄像头
    output reg                  mode_bp12_led_seg,      // bp12_2内部模式: 0: LED模式, 1: 数码管模式
    output                      sd_trigger,             // SD卡读取触发（改为wire类型，由按键或UDP控制）
    output reg  [2:0]           image_proc_mode,        // 图像处理模式: 000=原始, 001=二值化, 010=腐蚀, 011=开运算, 100=浮雕, 101=拉普拉斯, 110=Sobel

    // 二值化阈值参数输出（新增）
    output reg  [7:0]           binary_threshold_low,   // 二值化低阈值（0-255）
    output reg  [7:0]           binary_threshold_high,  // 二值化高阈值（0-255）

    // 边框检测使能输出（新增）
    output reg                  border_enable,          // 边框检测使能

    // 双目摄像头控制输出（新增）
    output reg                  q3v3_cam_dual_enable,   // 0: 单目, 1: 双目
    output reg                  q3v3_cam_right_select,  // 0: 左目, 1: 右目（只在单目模式有效）

    // 分辨率控制输出（720p/1080p扩展功能）
    output reg  [1:0]           resolution_sel,         // 分辨率选择: 0=640×480, 1=720p, 2=1080p
    output reg  [20:0]          frame_pixel_count       // 当前帧像素数
);

// 接收状态机
localparam  IDLE        = 2'd0;
localparam  RECV_DATA   = 2'd1;
localparam  PROCESS     = 2'd2;

reg [1:0]   state;
reg [15:0]  byte_count;
reg [7:0]   byte1;
reg [15:0]  cmd_value;
reg [7:0]   param1, param2;      // 参数缓冲区（用于4字节命令）
reg [15:0]  rx_data_length_reg;  // 保存数据包长度

// SD卡触发脉冲生成
reg         sd_trigger_pulse;
reg [21:0]  sd_trigger_cnt;  // 扩展到22位以支持更长的脉冲时间

always @(posedge udp_rx_clk or negedge reset) begin
    if (!reset) begin
        state <= IDLE;
        byte_count <= 16'd0;
        byte1 <= 8'd0;
        cmd_value <= 16'd0;
        param1 <= 8'd0;
        param2 <= 8'd0;
        rx_data_length_reg <= 16'd0;
        mode_project <= 1'b0;           // 默认bp12_2工程
        mode_q3v3_internal <= 1'b1;     // 默认摄像头模式（q3v3工程内部）
        mode_bp12_led_seg <= 1'b0;      // 默认LED模式（bp12_2工程内部）
        sd_trigger_pulse <= 1'b0;
        image_proc_mode <= 3'b000;      // 默认原始图像模式
        binary_threshold_low <= 8'd50;  // 默认低阈值
        binary_threshold_high <= 8'd200;// 默认高阈值
        border_enable <= 1'b0;          // 默认关闭边框检测
        q3v3_cam_dual_enable <= 1'b0;   // 默认单目模式
        q3v3_cam_right_select <= 1'b0;  // 默认左目
        resolution_sel <= 2'd0;         // 默认640×480
        frame_pixel_count <= 21'd307200; // 默认像素数(640×480)
    end
    else begin
        case (state)
            IDLE: begin
                sd_trigger_pulse <= 1'b0;
                byte_count <= 16'd0;

                // 按键控制：key3按下时，在q3v3模式下切换SD卡/摄像头模式
                if (key3_neg && mode_project == 1'b1) begin
                    mode_q3v3_internal <= ~mode_q3v3_internal;
                end

                // 检查数据包长度，处理2字节或4字节的控制命令
                if (app_rx_data_valid && (app_rx_data_length == 16'd2 || app_rx_data_length == 16'd4)) begin
                    byte1 <= app_rx_data;  // 立即保存第一个字节
                    rx_data_length_reg <= app_rx_data_length;  // 保存数据包长度
                    byte_count <= 16'd1;
                    state <= RECV_DATA;
                end
            end

            RECV_DATA: begin
                if (app_rx_data_valid) begin
                    case (byte_count)
                        16'd1: begin
                            cmd_value <= {byte1, app_rx_data};  // 组合成16位命令
                            if (rx_data_length_reg == 16'd2) begin
                                state <= PROCESS;  // 2字节命令，直接处理
                            end else begin
                                byte_count <= byte_count + 1'd1;  // 4字节命令，继续接收参数
                            end
                        end
                        16'd2: begin
                            param1 <= app_rx_data;  // 保存第1个参数
                            byte_count <= byte_count + 1'd1;
                        end
                        16'd3: begin
                            param2 <= app_rx_data;  // 保存第2个参数
                            state <= PROCESS;       // 接收完成，处理命令
                        end
                        default: begin
                            byte_count <= byte_count + 1'd1;
                        end
                    endcase
                end
            end

            PROCESS: begin
                // 根据命令值设置模式
                case (cmd_value)
                    16'h0001: begin  // LED模式（12个LED控制）
                        mode_project <= 1'b0;        // 切换到bp12_2工程
                        mode_bp12_led_seg <= 1'b0;   // LED模式
                        mode_q3v3_internal <= 1'b1;  // （q3v3信号保持默认，在bp12_2下无效）
                        sd_trigger_pulse <= 1'b0;    // 不触发SD卡
                    end

                    16'h0002: begin  // 数码管模式（8个数码管+4个单色LED）
                        mode_project <= 1'b0;        // 切换到bp12_2工程
                        mode_bp12_led_seg <= 1'b1;   // 数码管模式
                        mode_q3v3_internal <= 1'b1;  // （q3v3信号保持默认，在bp12_2下无效）
                        sd_trigger_pulse <= 1'b0;    // 不触发SD卡
                    end

                    16'h0003: begin  // SD卡模式+触发读取
                        mode_project <= 1'b1;        // 切换到q3v3工程
                        mode_q3v3_internal <= 1'b0;  // SD卡模式（SD卡写SDRAM）
                        mode_bp12_led_seg <= 1'b0;   // （bp12_2信号保持默认，在q3v3下无效）
                        sd_trigger_pulse <= 1'b1;    // 自动触发SD卡读取
                    end

                    16'h0004: begin  // 左目摄像头 - 原始图像
                        mode_project <= 1'b1;        // 切换到q3v3工程
                        mode_q3v3_internal <= 1'b1;  // 摄像头模式（摄像头写SDRAM）
                        mode_bp12_led_seg <= 1'b0;   // （bp12_2信号保持默认，在q3v3下无效）
                        sd_trigger_pulse <= 1'b0;    // 不触发SD卡
                        image_proc_mode <= 3'b000;   // 原始图像
                        q3v3_cam_dual_enable <= 1'b0;   // 单目模式
                        q3v3_cam_right_select <= 1'b0;  // 左目
                    end

                    16'h0005: begin  // 左目摄像头 - 二值化图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b001;   // 二值化
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b0;
                    end

                    16'h0006: begin  // 左目摄像头 - 腐蚀图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b010;   // 腐蚀(膨胀显示)
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b0;
                    end

                    16'h0007: begin  // 左目摄像头 - 开运算图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b011;   // 开运算
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b0;
                    end

                    16'h000A: begin  // 左目摄像头 - 浮雕效果图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b100;   // 浮雕效果
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b0;
                    end

                    16'h000B: begin  // 左目摄像头 - 拉普拉斯锐化图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b101;   // 拉普拉斯锐化
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b0;
                    end

                    16'h000C: begin  // 左目摄像头 - Sobel边缘检测图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b110;   // Sobel边缘检测
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b0;
                    end

                    16'h000D: begin  // 左目摄像头 - 帧差法
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b111;   // 帧差法
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b0;
                    end

                    16'h0010: begin  // 右目摄像头 - 原始图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b000;   // 原始图像
                        q3v3_cam_dual_enable <= 1'b0;   // 单目模式
                        q3v3_cam_right_select <= 1'b1;  // 右目
                    end

                    16'h0011: begin  // 双目摄像头（无图像处理）
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b000;   // 强制原始图像
                        q3v3_cam_dual_enable <= 1'b1;   // 双目模式
                        q3v3_cam_right_select <= 1'b0;  // 双目时该信号无效
                    end

                    16'h0012: begin  // 右目摄像头 - 二值化图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b001;   // 二值化
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b1;
                    end

                    16'h0013: begin  // 右目摄像头 - 腐蚀图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b010;   // 腐蚀
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b1;
                    end

                    16'h0014: begin  // 右目摄像头 - 开运算图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b011;   // 开运算
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b1;
                    end

                    16'h0015: begin  // 右目摄像头 - 浮雕效果图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b100;   // 浮雕效果
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b1;
                    end

                    16'h0016: begin  // 右目摄像头 - 拉普拉斯锐化图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b101;   // 拉普拉斯锐化
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b1;
                    end

                    16'h0017: begin  // 右目摄像头 - Sobel边缘检测图像
                        mode_project <= 1'b1;
                        mode_q3v3_internal <= 1'b1;
                        mode_bp12_led_seg <= 1'b0;
                        sd_trigger_pulse <= 1'b0;
                        image_proc_mode <= 3'b110;   // Sobel边缘检测
                        q3v3_cam_dual_enable <= 1'b0;
                        q3v3_cam_right_select <= 1'b1;
                    end

                    16'h0008: begin  // 配置二值化阈值（4字节命令）
                        // 参数验证：确保低阈值 < 高阈值
                        if (param1 < param2) begin
                            binary_threshold_low <= param1;
                            binary_threshold_high <= param2;
                        end
                        // 如果参数不合法，保持原有阈值不变
                    end

                    16'h0009: begin  // 切换边框检测使能
                        border_enable <= ~border_enable;  // toggle边框检测
                    end

                    // 分辨率控制命令（显式设置）
                    16'h0030: begin  // 设置为640×480分辨率
                        resolution_sel <= 2'd0;
                        frame_pixel_count <= 21'd307200;  // 640×480像素数
                    end

                    16'h0031: begin  // 设置为1280×720分辨率（720p）
                        resolution_sel <= 2'd1;
                        frame_pixel_count <= 21'd921600;  // 1280×720像素数
                    end

                    16'h0032: begin  // 设置为1920×1080分辨率（1080p）
                        resolution_sel <= 2'd2;
                        frame_pixel_count <= 21'd2073600; // 1920×1080像素数
                    end

                    default: begin
                        // 未知命令，保持当前状态
                    end
                endcase

                state <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

// SD卡触发信号生成（将脉冲展宽为按键持续时间）
// SD卡的防抖模块需要20ms稳定时间，所以这里产生30ms的低电平脉冲
reg sd_trigger_udp;
always @(posedge udp_rx_clk or negedge reset) begin
    if (!reset) begin
        sd_trigger_udp <= 1'b1;      // 默认高电平（未按下）
        sd_trigger_cnt <= 22'd0;
    end
    else begin
        if (sd_trigger_pulse) begin
            sd_trigger_udp <= 1'b0;  // 模拟按键按下（低电平有效）
            sd_trigger_cnt <= 22'd3750000;  // 维持30ms (125MHz时钟: 30ms * 125MHz = 3,750,000)
        end
        else if (sd_trigger_cnt > 0) begin
            sd_trigger_cnt <= sd_trigger_cnt - 1'd1;
            sd_trigger_udp <= 1'b0;
        end
        else begin
            sd_trigger_udp <= 1'b1;  // 释放按键
        end
    end
end

// ================================================================
// 按键控制逻辑（新增）
// key2: SD卡触发（对应原q3v3的key1）
// key3: 模式切换（对应原q3v3的key2，在q3v3模式下切换SD/摄像头）
// ================================================================

// key3按键防抖和边沿检测（独立的边沿检测逻辑，不驱动输出）
reg key3_d1, key3_d2;
wire key3_neg;

always @(posedge udp_rx_clk or negedge reset) begin
    if (!reset) begin
        key3_d1 <= 1'b1;
        key3_d2 <= 1'b1;
    end else begin
        key3_d1 <= key3;
        key3_d2 <= key3_d1;
    end
end

assign key3_neg = key3_d2 & (~key3_d1);  // 检测下降沿

// SD卡触发信号：key2和UDP触发的组合（逻辑与）
assign sd_trigger = key2 & sd_trigger_udp;

endmodule
