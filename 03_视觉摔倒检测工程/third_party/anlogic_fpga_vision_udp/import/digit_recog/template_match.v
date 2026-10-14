//****************************************Copyright (c)***********************************//
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取FPGA & STM32资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           vip
// Last modified Date:  2019/03/22 16:33:40
// Last Version:        V1.0
// Descriptions:        数字图像处理模块封装层
//----------------------------------------------------------------------------------------
// Created by:          正点原子
// Created date:        2019/03/22 16:33:56
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module template_match #(
    parameter H_PIXEL = 32,              // 图像宽度
    parameter V_PIXEL = 32,              // 图像高度
    parameter DEPBIT  = 10               // 地址位宽（1024像素=10位地址）
)(
    //module clock
    input                      clk               ,    // 时钟信号
    input                      rst_n             ,    // 复位信号（低有效）

    //Template image interface (模板图像接口)
    input                      template_we       ,    // 模板写使能
    input      [DEPBIT-1:0]    template_addr     ,    // 模板地址 (0-1023)
    input                      template_data     ,    // 模板数据 (1-bit: 1=白, 0=黑)

    //Test image interface (测试图像接口)
    input                      test_we           ,    // 测试图像写使能
    input      [DEPBIT-1:0]    test_addr         ,    // 测试图像地址 (0-1023)
    input                      test_data         ,    // 测试图像数据 (1-bit)

    //Control interface (控制接口)
    input                      start_match       ,    // 开始匹配触发信号
    input      [DEPBIT-1:0]    match_threshold   ,    // 匹配阈值（不同像素数量阈值）

    //Result interface (结果接口)
    output reg                 match_result      ,    // 匹配结果 (1=匹配, 0=不匹配)
    output reg                 match_done        ,    // 匹配完成标志
    output reg [DEPBIT-1:0]    hamming_distance       // 汉明距离（不同像素的数量）
);

//localparam define
localparam st_idle    = 2'b00;           // 空闲状态
localparam st_compare = 2'b01;           // 比较状态
localparam st_done    = 2'b10;           // 完成状态

//reg define
reg [ 1:0]          cur_state         ;
reg [ 1:0]          nxt_state         ;
reg [DEPBIT-1:0]    compare_addr      ;  // 比较地址计数器
reg [DEPBIT-1:0]    diff_cnt          ;  // 不同像素计数器

//wire define
wire                template_pixel    ;  // 模板像素值
wire                test_pixel        ;  // 测试像素值
wire                pixel_diff        ;  // 像素差异（XOR）

//*****************************************************
//**                    main code
//*****************************************************

assign pixel_diff = template_pixel ^ test_pixel;  // XOR: 相同=0, 不同=1

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
    .raddr(compare_addr),                // 比较时读取
    .dq_i(template_data),
    .dq_o(template_pixel)
);

//测试图像存储 (Test image RAM)
myram #(
    .WIDTH(1  ),                         // 1-bit per pixel
    .DEPTH(H_PIXEL * V_PIXEL),           // 32×32 = 1024 pixels
    .DEPBIT(DEPBIT)
)u_test_ram(
    //module clock
    .clk(clk),
    //ram interface
    .we(test_we),
    .waddr(test_addr),
    .raddr(compare_addr),                // 比较时读取
    .dq_i(test_data),
    .dq_o(test_pixel)
);

//(三段式状态机)状态转移
always @(posedge clk or negedge rst_n) begin
  if(!rst_n)
      cur_state <= st_idle;
  else
      cur_state <= nxt_state;
end

//状态转移条件
always @( * ) begin
    case(cur_state)
        st_idle: begin
            if(start_match)              // 触发匹配
                nxt_state = st_compare;
            else
                nxt_state = st_idle;
        end
        st_compare:begin
            if(compare_addr == (H_PIXEL * V_PIXEL - 1))  // 比较完所有像素
                nxt_state = st_done;
            else
                nxt_state = st_compare;
        end
        st_done:begin
            nxt_state = st_idle;         // 返回空闲状态
        end
        default:
            nxt_state = st_idle;
    endcase
end

//状态任务
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        compare_addr     <= 10'd0;
        diff_cnt         <= 10'd0;
        hamming_distance <= 10'd0;
        match_result     <= 1'b0;
        match_done       <= 1'b0;
    end
    else case(nxt_state)
        st_idle: begin
            compare_addr     <= 10'd0;
            diff_cnt         <= 10'd0;
            match_done       <= 1'b0;
        end
        st_compare:begin
            compare_addr <= compare_addr + 1'b1;
            // 累计不同像素数量
            if(pixel_diff)
                diff_cnt <= diff_cnt + 1'b1;
        end
        st_done:begin
            // 输出汉明距离
            hamming_distance <= diff_cnt;
            // 判断匹配结果：不同像素数 <= 阈值 则匹配成功
            if(diff_cnt <= match_threshold)
                match_result <= 1'b1;    // 匹配
            else
                match_result <= 1'b0;    // 不匹配
            match_done <= 1'b1;          // 匹配完成标志
        end
    endcase
end

endmodule
