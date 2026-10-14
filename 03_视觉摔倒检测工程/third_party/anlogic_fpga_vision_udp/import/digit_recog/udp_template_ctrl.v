//****************************************Copyright (c)***********************************//
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取FPGA & STM32资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           udp_template_ctrl
// Last modified Date:  2025/01/05
// Last Version:        V1.1
// Descriptions:        UDP模板匹配控制模块
//----------------------------------------------------------------------------------------
// Modified by:         Claude
// Modified date:       2025/01/05
// Version:             V1.1
// Descriptions:        UDP control module for template matching
//                      - Receive template image (32×32 = 128 bytes)
//                      - Receive test image (32×32 = 128 bytes)
//                      - Set matching threshold
//                      - Trigger matching process
//
// UDP Command Format:
//   0x00F0: Upload template image (128 bytes data)
//   0x00F1: Upload test image (128 bytes data)
//   0x00F2: Set matching threshold (2 bytes data)
//   0x00F3: Start matching (no data)
//   0x00F4: Read match result (returns 1 byte: result + hamming distance)
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module udp_template_ctrl(
    //module clock
    input                  clk                ,    // 时钟信号
    input                  rst_n              ,    // 复位信号（低有效）

    //UDP RX interface (UDP接收接口)
    input                  udp_rx_valid       ,    // UDP接收数据有效
    input      [15:0]      udp_rx_cmd         ,    // UDP命令码
    input      [7:0]       udp_rx_data        ,    // UDP接收数据（按字节）
    input                  udp_rx_byte_cnt_valid,  // 字节计数有效
    input      [15:0]      udp_rx_byte_cnt    ,    // 当前字节计数

    //Template match interface (模板匹配接口)
    output reg             template_we        ,    // 模板写使能
    output reg [9:0]       template_addr      ,    // 模板地址 (0-1023)
    output reg             template_data      ,    // 模板数据

    output reg             test_we            ,    // 测试图像写使能
    output reg [9:0]       test_addr          ,    // 测试图像地址 (0-1023)
    output reg             test_data          ,    // 测试图像数据

    output reg             start_match        ,    // 开始匹配触发
    output reg [9:0]       match_threshold    ,    // 匹配阈值

    //Match result interface (匹配结果接口)
    input                  match_done         ,    // 匹配完成标志
    input                  match_result       ,    // 匹配结果 (1=匹配, 0=不匹配)
    input      [9:0]       hamming_distance   ,    // 汉明距离

    //UDP TX interface (UDP发送接口 - 用于返回结果)
    output reg             udp_tx_req         ,    // UDP发送请求
    output reg [7:0]       udp_tx_data        ,    // UDP发送数据
    input                  udp_tx_done             // UDP发送完成
);

//localparam define
localparam CMD_UPLOAD_TEMPLATE  = 16'h00F0;      // 上传模板图像
localparam CMD_UPLOAD_TEST      = 16'h00F1;      // 上传测试图像
localparam CMD_SET_THRESHOLD    = 16'h00F2;      // 设置阈值
localparam CMD_START_MATCH      = 16'h00F3;      // 开始匹配
localparam CMD_READ_RESULT      = 16'h00F4;      // 读取结果

localparam IMG_BYTES            = 10'd128;       // 32×32 / 8 = 128 bytes
localparam IMG_PIXELS           = 10'd1024;      // 32×32 = 1024 pixels

//reg define
reg [2:0]              bit_cnt            ;      // 位计数器 (0-7)
reg [9:0]              pixel_cnt          ;      // 像素计数器 (0-1023)
reg                    cmd_valid          ;      // 命令有效标志
reg [15:0]             current_cmd        ;      // 当前命令
reg                    match_done_d1      ;      // 匹配完成延迟

//wire define
wire                   match_done_rise    ;      // 匹配完成上升沿

//*****************************************************
//**                    main code
//*****************************************************

assign match_done_rise = match_done & ~match_done_d1;

//延迟信号用于边沿检测
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        match_done_d1 <= 1'b0;
    else
        match_done_d1 <= match_done;
end

//命令识别和保持
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        cmd_valid   <= 1'b0;
        current_cmd <= 16'd0;
    end
    else if(udp_rx_valid && udp_rx_byte_cnt == 16'd0) begin
        // 接收到新命令（第一个字节）
        current_cmd <= udp_rx_cmd;
        cmd_valid   <= 1'b1;
    end
    else if(udp_rx_byte_cnt >= IMG_BYTES + 2) begin
        // 数据接收完成，清除命令
        cmd_valid   <= 1'b0;
    end
end

//位计数器 (用于从字节中提取位)
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        bit_cnt <= 3'd0;
    else if(udp_rx_byte_cnt_valid && cmd_valid) begin
        if(current_cmd == CMD_UPLOAD_TEMPLATE || current_cmd == CMD_UPLOAD_TEST) begin
            if(bit_cnt == 3'd7)
                bit_cnt <= 3'd0;
            else
                bit_cnt <= bit_cnt + 1'b1;
        end
    end
    else
        bit_cnt <= 3'd0;
end

//像素计数器
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        pixel_cnt <= 10'd0;
    else if(udp_rx_byte_cnt_valid && cmd_valid) begin
        if(current_cmd == CMD_UPLOAD_TEMPLATE || current_cmd == CMD_UPLOAD_TEST) begin
            if(pixel_cnt < IMG_PIXELS - 1)
                pixel_cnt <= pixel_cnt + 1'b1;
            else
                pixel_cnt <= 10'd0;
        end
    end
    else if(!cmd_valid)
        pixel_cnt <= 10'd0;
end

//模板图像写入控制
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        template_we   <= 1'b0;
        template_addr <= 10'd0;
        template_data <= 1'b0;
    end
    else if(cmd_valid && current_cmd == CMD_UPLOAD_TEMPLATE && udp_rx_byte_cnt_valid) begin
        if(udp_rx_byte_cnt >= 16'd2 && udp_rx_byte_cnt < (IMG_BYTES + 2)) begin
            template_we   <= 1'b1;
            template_addr <= pixel_cnt;
            // 从字节中提取位 (MSB first: bit 7 -> bit 0)
            template_data <= udp_rx_data[7 - bit_cnt];
        end
        else
            template_we <= 1'b0;
    end
    else
        template_we <= 1'b0;
end

//测试图像写入控制
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        test_we   <= 1'b0;
        test_addr <= 10'd0;
        test_data <= 1'b0;
    end
    else if(cmd_valid && current_cmd == CMD_UPLOAD_TEST && udp_rx_byte_cnt_valid) begin
        if(udp_rx_byte_cnt >= 16'd2 && udp_rx_byte_cnt < (IMG_BYTES + 2)) begin
            test_we   <= 1'b1;
            test_addr <= pixel_cnt;
            // 从字节中提取位 (MSB first)
            test_data <= udp_rx_data[7 - bit_cnt];
        end
        else
            test_we <= 1'b0;
    end
    else
        test_we <= 1'b0;
end

//阈值设置
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        match_threshold <= 10'd50;       // 默认阈值：50个像素差异
    else if(cmd_valid && current_cmd == CMD_SET_THRESHOLD && udp_rx_byte_cnt_valid) begin
        if(udp_rx_byte_cnt == 16'd2)     // 第一个字节（高8位）
            match_threshold[9:8] <= udp_rx_data[1:0];
        else if(udp_rx_byte_cnt == 16'd3) // 第二个字节（低8位）
            match_threshold[7:0] <= udp_rx_data;
    end
end

//开始匹配触发
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        start_match <= 1'b0;
    else if(cmd_valid && current_cmd == CMD_START_MATCH && udp_rx_valid)
        start_match <= 1'b1;
    else
        start_match <= 1'b0;
end

//UDP发送结果
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        udp_tx_req  <= 1'b0;
        udp_tx_data <= 8'd0;
    end
    else if(match_done_rise) begin
        // 匹配完成，自动发送结果
        udp_tx_req  <= 1'b1;
        // 数据格式: [7]=match_result, [6:0]=hamming_distance[6:0]
        udp_tx_data <= {match_result, 1'b0, hamming_distance[9:4]};
    end
    else if(cmd_valid && current_cmd == CMD_READ_RESULT && udp_rx_valid) begin
        // 主动读取结果
        udp_tx_req  <= 1'b1;
        udp_tx_data <= {match_result, 1'b0, hamming_distance[9:4]};
    end
    else if(udp_tx_done)
        udp_tx_req <= 1'b0;
end

endmodule
