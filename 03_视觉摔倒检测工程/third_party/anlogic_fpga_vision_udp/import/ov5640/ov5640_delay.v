//****************************************Copyright (c)***********************************//
// 模块功能：OV5640 RGB565转RGB888 + 图像处理封装
//
// 数据流：
//   CMOS RGB565 输入 → 2级延迟 → RGB888转换 → 图像处理 → 输出到SDRAM
//
// 图像处理模式（i_proc_mode）：
//   3'b000: 原始RGB图像（bypass）
//   3'b001: 二值化图像
//   3'b010: 膨胀图像（显示为"腐蚀"）
//   3'b011: 开运算（膨胀+腐蚀）
//   3'b100: 浮雕效果图像
//   3'b101: 拉普拉斯锐化图像
//   3'b110: Sobel边缘检测图像
//
// 边框检测（i_border_enable）：
//   当使能时，在原始图像上绘制白色区域的绿色边框
//****************************************************************************************//

module ov5640_delay(
	input 					clk,
    input					rst_n,

	input 					cmos_frame_vsync,
	input 					cmos_frame_href,
	input 					cmos_frame_valid,
	input 		[15:0] 		cmos_wr_data,

	// 图像处理控制输入（新增）
	input 		[2:0]		i_proc_mode,			// 000:原始, 001:二值化, 010:膨胀, 011:开运算, 100:浮雕, 101:拉普拉斯, 110:Sobel
	input 		[7:0]		i_threshold_low,		// 二值化低阈值
	input 		[7:0]		i_threshold_high,		// 二值化高阈值
	input 					i_border_enable,		// 边框检测使能

	// 输出到SDRAM的信号
	output             		cam_write_en,
	output 		[31:0]      cam_write_data,
	output  reg             cam_write_req,
	input               	cam_write_req_ack,

	// 灰度输出（用于外部帧差法等）
	output 		[7:0]       cam_gray_out
);

//=============================================================================
// 第1级延迟：输入信号缓冲
//=============================================================================
reg cmos_frame_href_d0;
reg cmos_frame_vsync_d0;
reg cmos_frame_valid_d0;
reg [15:0] cmos_wr_data_d0;

always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		cmos_frame_href_d0 <= 1'b0;
		cmos_frame_vsync_d0 <= 1'b0;
		cmos_frame_valid_d0 <= 1'b0;
        cmos_wr_data_d0 <= 16'd0;
	end
	else begin
		cmos_frame_href_d0 <= cmos_frame_href;
		cmos_frame_vsync_d0 <= cmos_frame_vsync;
		cmos_frame_valid_d0 <= cmos_frame_valid;
        cmos_wr_data_d0 <= cmos_wr_data;
	end
end

//=============================================================================
// 第2级延迟：RGB565 → RGB888转换
//=============================================================================
reg cmos_frame_href_d1;
reg cmos_frame_vsync_d1;
reg cmos_frame_valid_d1;
reg [15:0] cmos_wr_data_d1;

wire [7:0] cam_R, cam_G, cam_B;
assign cam_R = {cmos_wr_data_d1[15:11], {3{cmos_wr_data_d1[11]}}};
assign cam_G = {cmos_wr_data_d1[10:5],  {2{cmos_wr_data_d1[5]}}};
assign cam_B = {cmos_wr_data_d1[4:0],   {3{cmos_wr_data_d1[0]}}};

wire [31:0] cam_write_data_rgb888 = {cam_R, cam_G, cam_B, 8'd0};

always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		cmos_frame_href_d1 <= 1'b0;
		cmos_frame_vsync_d1 <= 1'b0;
		cmos_frame_valid_d1 <= 1'b0;
		cmos_wr_data_d1 <= 16'd0;
	end
	else begin
		cmos_frame_href_d1 <= cmos_frame_href_d0;
		cmos_frame_vsync_d1 <= cmos_frame_vsync_d0;
		cmos_frame_valid_d1 <= cmos_frame_valid_d0;
		cmos_wr_data_d1 <= cmos_wr_data_d0;
	end
end

//=============================================================================
// 图像处理模块：嵌入式集成
// 输入：RGB888原始数据 + 时序信号（d1级）
// 输出：处理后的RGB888数据 + 时序信号（自动延迟对齐）
//=============================================================================
wire        proc_hsyn;
wire        proc_vsyn;
wire        proc_en;
wire [31:0] proc_data;
wire [7:0]  proc_gray;

image_proc_wrapper u_image_proc_wrapper (
    .i_clk              (clk),
    .i_rst_n            (rst_n),

    // 输入时序和数据（来自RGB888转换后）
    .i_hsyn             (cmos_frame_href_d1),
    .i_vsyn             (cmos_frame_vsync_d1),
    .i_en               (cmos_frame_valid_d1),
    .i_data             (cam_write_data_rgb888),

    // 处理控制
    .i_proc_mode        (i_proc_mode),
    .i_threshold_low    (i_threshold_low),
    .i_threshold_high   (i_threshold_high),
    .i_border_enable    (i_border_enable),

    // 输出时序和数据（自动延迟对齐）
    .o_hsyn             (proc_hsyn),
    .o_vsyn             (proc_vsyn),
    .o_en               (proc_en),
    .o_data             (proc_data),
    .o_gray             (proc_gray)
);

//=============================================================================
// 输出连接：使用图像处理后的信号
//=============================================================================
assign cam_write_data = proc_data;
assign cam_write_en = proc_en;
assign cam_gray_out = proc_gray;  // 输出灰度值

//=============================================================================
// 写请求生成：使用处理后的vsync信号
//=============================================================================
reg proc_vsyn_d1;
always @(posedge clk or negedge rst_n) begin
	if(~rst_n)
		proc_vsyn_d1 <= 1'b0;
	else
		proc_vsyn_d1 <= proc_vsyn;
end

always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		cam_write_req <= 1'b0;
    end
	else if(proc_vsyn_d1 & ~proc_vsyn) begin  // vsync边沿检测（使用处理后的信号）
		cam_write_req <= 1'b1;
    end
    else if(cam_write_req_ack) begin
		cam_write_req <= 1'b0;
    end
end

endmodule
