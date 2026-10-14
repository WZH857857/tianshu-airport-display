//****************************************Copyright (c)***********************************//
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取FPGA & STM32资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           vip_template_match
// Last modified Date:  2025/01/05
// Last Version:        V2.0
// Descriptions:        实时视频流模板匹配顶层封装模块
//----------------------------------------------------------------------------------------
// Modified by:         Claude
// Modified date:       2025/01/05
// Version:             V2.0
// Descriptions:        Real-time video stream template matching system
//                      - Template uploaded once via UDP (CMD 0x00F0)
//                      - Real-time matching with video stream
//                      - Match result output every frame
//
// System overview:
//   UDP RX → template upload
//   Video Stream → region_extract → template_match_realtime → match_result
//
// Usage:
//   1. Upload template image via UDP (CMD 0x00F0, 128 bytes)
//   2. Real-time matching starts automatically
//   3. Match result updated every frame
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module vip_template_match(
    //module clock
    input                  clk                ,    // 时钟信号（cam_pclk）
    input                  rst_n              ,    // 复位信号（低有效）

    //UDP RX interface (UDP接收接口 - 仅用于上传模板)
    input                  udp_rx_valid       ,    // UDP接收数据有效
    input      [15:0]      udp_rx_cmd         ,    // UDP命令码
    input      [7:0]       udp_rx_data        ,    // UDP接收数据
    input                  udp_rx_byte_cnt_valid,  // 字节计数有效
    input      [15:0]      udp_rx_byte_cnt    ,    // 字节计数

    //Video stream interface (视频流接口 - 来自左目OV5640)
    input                  video_hsyn         ,    // 行同步
    input                  video_vsyn         ,    // 帧同步
    input                  video_en           ,    // 数据有效
    input      [7:0]       video_gray         ,    // 灰度值

    //Match result outputs (匹配结果输出)
    output                 match_result       ,    // 匹配结果 (1=匹配, 0=不匹配)
    output                 match_result_valid ,    // 匹配结果有效标志（每帧1次）
    output     [9:0]       hamming_distance        // 汉明距离
);

//parameter define
parameter H_PIXEL = 32;                           // 图像宽度
parameter V_PIXEL = 32;                           // 图像高度
parameter DEPBIT  = 10;                           // 地址位宽

//wire define
wire                   template_we       ;
wire [DEPBIT-1:0]      template_addr     ;
wire                   template_data     ;
wire [DEPBIT-1:0]      match_threshold   ;

wire                   region_valid      ;
wire [DEPBIT-1:0]      region_addr       ;
wire                   region_data       ;
wire                   frame_vsync       ;

//*****************************************************
//**                    main code
//*****************************************************

//UDP控制模块（简化版，只处理模板上传）
udp_template_ctrl u_udp_template_ctrl(
    //module clock
    .clk                 (clk                ),
    .rst_n               (rst_n              ),
    //UDP RX interface
    .udp_rx_valid        (udp_rx_valid       ),
    .udp_rx_cmd          (udp_rx_cmd         ),
    .udp_rx_data         (udp_rx_data        ),
    .udp_rx_byte_cnt_valid(udp_rx_byte_cnt_valid),
    .udp_rx_byte_cnt     (udp_rx_byte_cnt    ),
    //Template interface (只输出模板上传信号)
    .template_we         (template_we        ),
    .template_addr       (template_addr      ),
    .template_data       (template_data      ),
    .test_we             (),                     // 不使用
    .test_addr           (),                     // 不使用
    .test_data           (),                     // 不使用
    .start_match         (),                     // 不使用
    .match_threshold     (match_threshold    ),
    //Match result interface (悬空，不通过UDP发送)
    .match_done          (1'b0),
    .match_result        (1'b0),
    .hamming_distance    (10'd0),
    //UDP TX interface (悬空，不使用)
    .udp_tx_req          (),
    .udp_tx_data         (),
    .udp_tx_done         (1'b0)
);

//区域提取模块（从视频流提取32×32左上角区域）
template_region_extract u_template_region_extract(
    .i_clk               (clk                ),
    .i_rst_n             (rst_n              ),
    //Video stream input
    .i_hsyn              (video_hsyn         ),
    .i_vsyn              (video_vsyn         ),
    .i_en                (video_en           ),
    .i_gray              (video_gray         ),
    //Region output
    .o_region_valid      (region_valid       ),
    .o_pixel_addr        (region_addr        ),
    .o_pixel_data        (region_data        ),
    .o_frame_vsync       (frame_vsync        )
);

//实时模板匹配模块
template_match_realtime #(
    .H_PIXEL             (H_PIXEL            ),
    .V_PIXEL             (V_PIXEL            ),
    .DEPBIT              (DEPBIT             )
) u_template_match_realtime(
    //module clock
    .clk                 (clk                ),
    .rst_n               (rst_n              ),
    //Template upload interface
    .template_we         (template_we        ),
    .template_addr       (template_addr      ),
    .template_data       (template_data      ),
    //Real-time video region interface
    .region_valid        (region_valid       ),
    .region_addr         (region_addr        ),
    .region_data         (region_data        ),
    .frame_vsync         (frame_vsync        ),
    //Control
    .match_threshold     (match_threshold    ),
    //Result outputs
    .match_result        (match_result       ),
    .match_result_valid  (match_result_valid ),
    .hamming_distance    (hamming_distance   )
);

endmodule
