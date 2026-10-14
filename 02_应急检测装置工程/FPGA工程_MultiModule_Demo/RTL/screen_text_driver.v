//=============================================================================
// 文件名   : screen_text_driver.v
// 功能     : SPI TFT 状态页驱动（整屏只显示"当前状态 + 对应标识"）
//
// 说明     :
//   * 初始化序列、复位时序、CS 事务与 screen_pattern_driver.v 完全一致，
//     该部分已在 1.9 寸 ZJY190S0800TG01 实机验证通过；
//   * 屏幕内容：状态色外框 + 状态色底条、一个 100x100 图标、
//     两行 16x32 状态文字（状态词 + 原因），以及两行文字与底条之间的
//     一行实时时钟 HH:MM:SS（每秒跟随 DS1307 RTC 刷新）。温度/湿度/
//     火焰 AO/蓝牙等具体数值一律不上屏；
//   * 三种状态与标识一一对应：
//       正常 NORMAL  / ALL CLEAR   绿色圆底白色对勾，绿色外框
//       预警 WARNING / 具体原因    黄色三角黑色感叹号，黄色外框
//       火焰 FIRE    / ALARM       红色火苗加黄色火心，红色外框
//   * 点阵取自厂商例程 lcdfont.h 的 ascii_3216 表（经 font16x32_rom.v 转换）。
//=============================================================================
module screen_text_driver #(
    parameter ST7789 = 1'b1,
    parameter ST7735 = 1'b0,
    // 1 时使用 ZJY190S0800TG01 厂商示例的 ST7789V3 初始化表。
    parameter ST7789_19 = 1'b0,
    parameter X_START = 0,
    parameter Y_START = 0,
    parameter X_LEN   = 240,
    parameter Y_LEN   = 320,
    // CS_PULSE_EN = 0：CS 全程拉低（现有位流行为，保持不变）。
    // CS_PULSE_EN = 1：每个字节发完后 CS 拉高一段空闲时间再拉低，
    //                  满足 ST7789VW 手册要求的 CSX 事务边界。
    parameter CS_PULSE_EN = 1'b0,
    parameter CS_GAP      = 16'd50,   // 1 us @ 50 MHz，仍远大于手册 40 ns 最小高电平
    // SPI_DIV 越大 SPI 越慢：SCL = 50 MHz / (2 * SPI_DIV)。
    // 4 → 6.25 MHz；50 → 500 kHz；200 → 125 kHz（对齐 C51 官方例程速度档，
    // 用于排查杜邦线太长/接触不良导致的数据错乱）。
    parameter [15:0] SPI_DIV = 16'd4
) (
    input  wire clk,
    input  wire rst_n,

    // E104 应急检测装置状态输入。报警判定全部在顶层 FPGA 逻辑内完成，
    // 本模块只把结论翻译成一屏状态页。
    // 注意：温度/湿度/AO/蓝牙这些"数值"端口不上屏，
    // 为了不牵动顶层连线而保留；状态页使用下面 5 个判定结果，
    // 时钟行使用 rtc_hour/min/sec_bcd 三个端口。
    input  wire        temp_alarm,    // 温度超限
    input  wire        humi_alarm,    // 湿度超限
    input  wire        flame_alarm,   // 火焰报警（DO 判定结果）
    input  wire        smoke_alarm,   // MQ-2 烟雾 DO 报警
    input  wire        pir_active,    // SR602 检测到人体

    // 以下端口当前不参与显示，保留给后续扩展 / 调试。
    input  wire [7:0]  temp_in,       // DHT11 温度，整数摄氏度
    input  wire [7:0]  humi_in,       // DHT11 湿度，整数百分比
    input  wire        dht_ok,        // DHT11 最近一次读取校验通过
    input  wire [11:0] flame_ao,      // 火焰传感器 AO 原始值 0..4095
    input  wire        rtc_ok,        // RTC 至少成功读过一次
    input  wire [7:0]  rtc_hour_bcd,  // RTC 时 BCD
    input  wire [7:0]  rtc_min_bcd,   // RTC 分 BCD
    input  wire [7:0]  rtc_sec_bcd,   // RTC 秒 BCD
    input  wire        ble_online,    // 蓝牙链路在线

    output reg  tft_cs_n,
    output reg  tft_res_n,
    output reg  tft_dc,
    output wire tft_scl,
    output wire tft_sda
);

    // 面板有效显示区在控制器显存中的起点和尺寸。
    // 1.14 寸 ST7789：X_START=52，Y_START=40，X_LEN=135，Y_LEN=240。
    // 1.8  寸 ST7735S（MADCTL=0xC8）：X_START=1，Y_START=2，X_LEN=160，Y_LEN=128。
    // 1.9  寸 ST7789V3（ZJY190S0800TG01）：X_START=35，Y_START=0，X_LEN=170，Y_LEN=320。
    localparam [8:0]  X_LAST    = X_LEN - 1;
    localparam [8:0]  Y_LAST    = Y_LEN - 1;
    localparam [15:0] X_START_W = X_START;
    localparam [15:0] Y_START_W = Y_START;
    localparam [15:0] X_END_W   = X_START + X_LEN - 1;
    localparam [15:0] Y_END_W   = Y_START + Y_LEN - 1;
    // 厂商 C51 例程复位低电平 30 ms，STM32 例程 100 ms；这里取 100 ms 对齐。
    localparam [22:0] RESET_LOW_CYCLES  = 23'd5_000_000;  // 100 ms @ 50 MHz
    localparam [23:0] INIT_DELAY_CYCLES = 24'd6_000_000;  // 120 ms @ 50 MHz
    localparam [25:0] REFRESH_CYCLES    = 26'd25_000_000; // 0.5 s @ 50 MHz

    // 三种控制器的初始化表长度不同。
    localparam [6:0] INIT_ST_LAST     = 7'd61;
    localparam [6:0] INIT_ST19_LAST   = 7'd60;
    localparam [6:0] INIT_ILI_LAST    = 7'd7;
    localparam [6:0] INIT_ST7735_LAST = 7'd42;
    wire [6:0] init_last = ST7735 ? INIT_ST7735_LAST :
                           (ST7789_19 ? INIT_ST19_LAST :
                           (ST7789 ? INIT_ST_LAST : INIT_ILI_LAST));

    localparam [3:0] ST_IDLE       = 4'd0;
    localparam [3:0] ST_RESET_LOW  = 4'd1;
    localparam [3:0] ST_RESET_HIGH = 4'd2;
    localparam [3:0] ST_INIT       = 4'd3;
    localparam [3:0] ST_INIT_WAIT  = 4'd4;
    localparam [3:0] ST_SET_X      = 4'd5;
    localparam [3:0] ST_SET_Y      = 4'd6;
    localparam [3:0] ST_RAMWR      = 4'd7;
    localparam [3:0] ST_PIXEL_HI   = 4'd8;
    localparam [3:0] ST_PIXEL_LO   = 4'd9;
    localparam [3:0] ST_REFRESH    = 4'd10;

    reg [3:0]  state;
    reg [22:0] delay_cnt;
    reg [23:0] init_wait_cnt;
    reg [25:0] refresh_cnt;
    reg [6:0]  init_idx;
    reg [2:0]  setup_stage;
    reg [8:0]  x_pos;
    reg [8:0]  y_pos;

    reg        spi_start;
    reg        spi_seen;
    reg        spi_busy;
    reg [15:0] spi_div_cnt;
    reg [2:0]  spi_bit_cnt;
    reg [7:0]  spi_shift;
    reg        spi_scl;
    reg        spi_sda;
    reg [7:0]  spi_data_hold;
    reg [8:0]  cur_word;
    reg [15:0] cs_wait_cnt;      // CS 空闲计时
    reg        cs_pulse_pending; // 本字节发完后需要产生 CS 高脉冲
    wire       spi_accept;       // 只有 CS 空闲结束且发送器空闲时才接受新字节

    assign tft_scl  = spi_scl;
    assign tft_sda  = spi_sda;
    assign spi_accept = !spi_busy && !cs_pulse_pending;

    //=========================================================================
    // ST7789 常用初始化序列。idx[8]=1 表示命令，idx[8]=0 表示数据。
    //=========================================================================
    function [8:0] init_word_st;
        input [6:0] idx;
        begin
            case (idx)
                7'd0:  init_word_st = {1'b1, 8'h01}; // SWRESET
                7'd1:  init_word_st = {1'b1, 8'h11}; // SLPOUT
                7'd2:  init_word_st = {1'b1, 8'h3A}; // COLMOD
                7'd3:  init_word_st = {1'b0, 8'h55}; // RGB565
                7'd4:  init_word_st = {1'b1, 8'h36}; // MADCTL
                7'd5:  init_word_st = {1'b0, 8'h00};
                7'd6:  init_word_st = {1'b1, 8'hB2}; // PORCTRL
                7'd7:  init_word_st = {1'b0, 8'h0C};
                7'd8:  init_word_st = {1'b0, 8'h0C};
                7'd9:  init_word_st = {1'b0, 8'h00};
                7'd10: init_word_st = {1'b0, 8'h33};
                7'd11: init_word_st = {1'b0, 8'h33};
                7'd12: init_word_st = {1'b1, 8'hB7}; // GCTRL
                7'd13: init_word_st = {1'b0, 8'h35};
                7'd14: init_word_st = {1'b1, 8'hBB}; // VCOMS
                7'd15: init_word_st = {1'b0, 8'h19};
                7'd16: init_word_st = {1'b1, 8'hC0}; // LCMCTRL
                7'd17: init_word_st = {1'b0, 8'h2C};
                7'd18: init_word_st = {1'b1, 8'hC2}; // VDVVRHEN
                7'd19: init_word_st = {1'b0, 8'h01};
                7'd20: init_word_st = {1'b1, 8'hC3}; // VRHS
                7'd21: init_word_st = {1'b0, 8'h12};
                7'd22: init_word_st = {1'b1, 8'hC4}; // VDVS
                7'd23: init_word_st = {1'b0, 8'h20};
                7'd24: init_word_st = {1'b1, 8'hC6}; // FRCTRL2
                7'd25: init_word_st = {1'b0, 8'h0F};
                7'd26: init_word_st = {1'b1, 8'hD0}; // PWCTRL1
                7'd27: init_word_st = {1'b0, 8'hA4};
                7'd28: init_word_st = {1'b0, 8'hA1};
                7'd29: init_word_st = {1'b1, 8'h21}; // INVON
                7'd30: init_word_st = {1'b1, 8'hE0}; // PVGAMCTRL
                7'd31: init_word_st = {1'b0, 8'hD0};
                7'd32: init_word_st = {1'b0, 8'h04};
                7'd33: init_word_st = {1'b0, 8'h0D};
                7'd34: init_word_st = {1'b0, 8'h11};
                7'd35: init_word_st = {1'b0, 8'h13};
                7'd36: init_word_st = {1'b0, 8'h2B};
                7'd37: init_word_st = {1'b0, 8'h3F};
                7'd38: init_word_st = {1'b0, 8'h54};
                7'd39: init_word_st = {1'b0, 8'h4C};
                7'd40: init_word_st = {1'b0, 8'h18};
                7'd41: init_word_st = {1'b0, 8'h0D};
                7'd42: init_word_st = {1'b0, 8'h0B};
                7'd43: init_word_st = {1'b0, 8'h1F};
                7'd44: init_word_st = {1'b0, 8'h23};
                7'd45: init_word_st = {1'b1, 8'hE1}; // NVGAMCTRL
                7'd46: init_word_st = {1'b0, 8'hD0};
                7'd47: init_word_st = {1'b0, 8'h04};
                7'd48: init_word_st = {1'b0, 8'h0C};
                7'd49: init_word_st = {1'b0, 8'h11};
                7'd50: init_word_st = {1'b0, 8'h13};
                7'd51: init_word_st = {1'b0, 8'h2C};
                7'd52: init_word_st = {1'b0, 8'h3F};
                7'd53: init_word_st = {1'b0, 8'h44};
                7'd54: init_word_st = {1'b0, 8'h51};
                7'd55: init_word_st = {1'b0, 8'h2F};
                7'd56: init_word_st = {1'b0, 8'h1F};
                7'd57: init_word_st = {1'b0, 8'h1F};
                7'd58: init_word_st = {1'b0, 8'h20};
                7'd59: init_word_st = {1'b0, 8'h23};
                7'd60: init_word_st = {1'b1, 8'h11}; // SLPOUT
                7'd61: init_word_st = {1'b1, 8'h29}; // DISPON
                default: init_word_st = {1'b1, 8'h00};
            endcase
        end
    endfunction

    //=========================================================================
    // ZJY190S0800TG01 厂商示例初始化序列。
    // 这是 1.9 寸 170x320 ST7789V3 的完整序列，参数与示例代码一致。
    //=========================================================================
    function [8:0] init_word_st19;
        input [6:0] idx;
        begin
            case (idx)
                7'd0:  init_word_st19 = {1'b1, 8'h11}; // SLPOUT
                7'd1:  init_word_st19 = {1'b1, 8'h36}; // MADCTL
                7'd2:  init_word_st19 = {1'b0, 8'h00}; // 竖屏方向
                7'd3:  init_word_st19 = {1'b1, 8'h3A}; // COLMOD
                7'd4:  init_word_st19 = {1'b0, 8'h05}; // RGB565
                7'd5:  init_word_st19 = {1'b1, 8'hB2}; // PORCTRL
                7'd6:  init_word_st19 = {1'b0, 8'h0C};
                7'd7:  init_word_st19 = {1'b0, 8'h0C};
                7'd8:  init_word_st19 = {1'b0, 8'h00};
                7'd9:  init_word_st19 = {1'b0, 8'h33};
                7'd10: init_word_st19 = {1'b0, 8'h33};
                7'd11: init_word_st19 = {1'b1, 8'hB7}; // GCTRL
                7'd12: init_word_st19 = {1'b0, 8'h35};
                7'd13: init_word_st19 = {1'b1, 8'hBB}; // VCOMS
                7'd14: init_word_st19 = {1'b0, 8'h1A};
                7'd15: init_word_st19 = {1'b1, 8'hC0}; // LCMCTRL
                7'd16: init_word_st19 = {1'b0, 8'h2C};
                7'd17: init_word_st19 = {1'b1, 8'hC2}; // VDVVRHEN
                7'd18: init_word_st19 = {1'b0, 8'h01};
                7'd19: init_word_st19 = {1'b1, 8'hC3}; // VRHS
                7'd20: init_word_st19 = {1'b0, 8'h0B};
                7'd21: init_word_st19 = {1'b1, 8'hC4}; // VDVS
                7'd22: init_word_st19 = {1'b0, 8'h20};
                7'd23: init_word_st19 = {1'b1, 8'hC6}; // FRCTRL2
                7'd24: init_word_st19 = {1'b0, 8'h0F};
                7'd25: init_word_st19 = {1'b1, 8'hD0}; // PWCTRL1
                7'd26: init_word_st19 = {1'b0, 8'hA4};
                7'd27: init_word_st19 = {1'b0, 8'hA1};
                7'd28: init_word_st19 = {1'b1, 8'h21}; // INVON
                7'd29: init_word_st19 = {1'b1, 8'hE0}; // PVGAMCTRL
                7'd30: init_word_st19 = {1'b0, 8'hF0};
                7'd31: init_word_st19 = {1'b0, 8'h00};
                7'd32: init_word_st19 = {1'b0, 8'h04};
                7'd33: init_word_st19 = {1'b0, 8'h04};
                7'd34: init_word_st19 = {1'b0, 8'h04};
                7'd35: init_word_st19 = {1'b0, 8'h05};
                7'd36: init_word_st19 = {1'b0, 8'h29};
                7'd37: init_word_st19 = {1'b0, 8'h33};
                7'd38: init_word_st19 = {1'b0, 8'h3E};
                7'd39: init_word_st19 = {1'b0, 8'h38};
                7'd40: init_word_st19 = {1'b0, 8'h12};
                7'd41: init_word_st19 = {1'b0, 8'h12};
                7'd42: init_word_st19 = {1'b0, 8'h28};
                7'd43: init_word_st19 = {1'b0, 8'h30};
                7'd44: init_word_st19 = {1'b1, 8'hE1}; // NVGAMCTRL
                7'd45: init_word_st19 = {1'b0, 8'hF0};
                7'd46: init_word_st19 = {1'b0, 8'h07};
                7'd47: init_word_st19 = {1'b0, 8'h0A};
                7'd48: init_word_st19 = {1'b0, 8'h0D};
                7'd49: init_word_st19 = {1'b0, 8'h0B};
                7'd50: init_word_st19 = {1'b0, 8'h07};
                7'd51: init_word_st19 = {1'b0, 8'h28};
                7'd52: init_word_st19 = {1'b0, 8'h33};
                7'd53: init_word_st19 = {1'b0, 8'h3E};
                7'd54: init_word_st19 = {1'b0, 8'h36};
                7'd55: init_word_st19 = {1'b0, 8'h14};
                7'd56: init_word_st19 = {1'b0, 8'h14};
                7'd57: init_word_st19 = {1'b0, 8'h29};
                7'd58: init_word_st19 = {1'b0, 8'h32};
                7'd59: init_word_st19 = {1'b1, 8'h11}; // SLPOUT
                7'd60: init_word_st19 = {1'b1, 8'h29}; // DISPON
                default: init_word_st19 = {1'b1, 8'h00};
            endcase
        end
    endfunction

    //=========================================================================
    // ILI9341 最小初始化序列。只保留所有 ILI9341 都会接受的基础命令。
    //=========================================================================
    function [8:0] init_word_ili;
        input [6:0] idx;
        begin
            case (idx)
                7'd0: init_word_ili = {1'b1, 8'h01}; // SWRESET
                7'd1: init_word_ili = {1'b1, 8'h11}; // SLPOUT
                7'd2: init_word_ili = {1'b1, 8'h3A}; // COLMOD
                7'd3: init_word_ili = {1'b0, 8'h55}; // RGB565
                7'd4: init_word_ili = {1'b1, 8'h36}; // MADCTL
                7'd5: init_word_ili = {1'b0, 8'h00};
                7'd6: init_word_ili = {1'b1, 8'h21}; // INVON
                7'd7: init_word_ili = {1'b1, 8'h29}; // DISPON
                default: init_word_ili = {1'b1, 8'h00};
            endcase
        end
    endfunction

    //=========================================================================
    // ST7735S 常用初始化序列，显存范围 128x160。
    //=========================================================================
    function [8:0] init_word_st7735;
        input [6:0] idx;
        begin
            case (idx)
                7'd0:  init_word_st7735 = {1'b1, 8'h01}; // SWRESET
                7'd1:  init_word_st7735 = {1'b1, 8'h11}; // SLPOUT
                7'd2:  init_word_st7735 = {1'b1, 8'hB1}; // FRMCTR1
                7'd3:  init_word_st7735 = {1'b0, 8'h01};
                7'd4:  init_word_st7735 = {1'b0, 8'h2C};
                7'd5:  init_word_st7735 = {1'b0, 8'h2D};
                7'd6:  init_word_st7735 = {1'b1, 8'hB2}; // FRMCTR2
                7'd7:  init_word_st7735 = {1'b0, 8'h01};
                7'd8:  init_word_st7735 = {1'b0, 8'h2C};
                7'd9:  init_word_st7735 = {1'b0, 8'h2D};
                7'd10: init_word_st7735 = {1'b1, 8'hB3}; // FRMCTR3
                7'd11: init_word_st7735 = {1'b0, 8'h01};
                7'd12: init_word_st7735 = {1'b0, 8'h2C};
                7'd13: init_word_st7735 = {1'b0, 8'h2D};
                7'd14: init_word_st7735 = {1'b0, 8'h01};
                7'd15: init_word_st7735 = {1'b0, 8'h2C};
                7'd16: init_word_st7735 = {1'b0, 8'h2D};
                7'd17: init_word_st7735 = {1'b1, 8'hB4}; // INVCTR
                7'd18: init_word_st7735 = {1'b0, 8'h07};
                7'd19: init_word_st7735 = {1'b1, 8'hC0}; // PWCTR1
                7'd20: init_word_st7735 = {1'b0, 8'hA2};
                7'd21: init_word_st7735 = {1'b0, 8'h02};
                7'd22: init_word_st7735 = {1'b0, 8'h84};
                7'd23: init_word_st7735 = {1'b1, 8'hC1}; // PWCTR2
                7'd24: init_word_st7735 = {1'b0, 8'hC5};
                7'd25: init_word_st7735 = {1'b1, 8'hC2}; // PWCTR3
                7'd26: init_word_st7735 = {1'b0, 8'h0A};
                7'd27: init_word_st7735 = {1'b0, 8'h00};
                7'd28: init_word_st7735 = {1'b1, 8'hC3}; // PWCTR4
                7'd29: init_word_st7735 = {1'b0, 8'h8A};
                7'd30: init_word_st7735 = {1'b0, 8'h2A};
                7'd31: init_word_st7735 = {1'b1, 8'hC4}; // PWCTR5
                7'd32: init_word_st7735 = {1'b0, 8'h8A};
                7'd33: init_word_st7735 = {1'b0, 8'hEE};
                7'd34: init_word_st7735 = {1'b1, 8'hC5}; // VMCTR1
                7'd35: init_word_st7735 = {1'b0, 8'h0E};
                7'd36: init_word_st7735 = {1'b1, 8'h21}; // INVON
                7'd37: init_word_st7735 = {1'b1, 8'h36}; // MADCTL
                7'd38: init_word_st7735 = {1'b0, 8'hC8};
                7'd39: init_word_st7735 = {1'b1, 8'h3A}; // COLMOD
                7'd40: init_word_st7735 = {1'b0, 8'h05}; // RGB565
                7'd41: init_word_st7735 = {1'b1, 8'h13}; // NORON
                7'd42: init_word_st7735 = {1'b1, 8'h29}; // DISPON
                default: init_word_st7735 = {1'b1, 8'h00};
            endcase
        end
    endfunction

    wire [8:0] init_word_now = ST7735 ? init_word_st7735(init_idx) :
                               (ST7789_19 ? init_word_st19(init_idx) :
                               (ST7789 ? init_word_st(init_idx) : init_word_ili(init_idx)));

    //=========================================================================
    // SPI 字节发送器：Mode 0，MSB first，SCL 空闲为低。
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_scl     <= 1'b0;
            spi_sda     <= 1'b1;
            spi_busy    <= 1'b0;
            spi_div_cnt <= 16'd0;
            spi_bit_cnt <= 3'd0;
            spi_shift   <= 8'h00;
            tft_cs_n    <= CS_PULSE_EN ? 1'b1 : 1'b0;
            cs_wait_cnt <= 16'd0;
            cs_pulse_pending <= 1'b0;
        end
        else if (!spi_busy) begin
            spi_scl     <= 1'b0;
            spi_div_cnt <= 16'd0;
            spi_bit_cnt <= 3'd0;
            if (cs_pulse_pending) begin
                // 上字节结束后的 CS 空闲期：先拉高，再在空闲结束时拉低。
                if (cs_wait_cnt >= CS_GAP - 1'b1) begin
                    tft_cs_n    <= 1'b0;   // 下降沿：开始下一次传输
                    cs_wait_cnt <= 16'd0;
                    cs_pulse_pending <= 1'b0;
                end
                else begin
                    tft_cs_n    <= 1'b1;   // 保持高电平，串口逻辑被重新初始化
                    cs_wait_cnt <= cs_wait_cnt + 1'b1;
                end
            end
            else begin
                tft_cs_n <= 1'b0;
            end
            if (spi_start && !cs_pulse_pending) begin
                spi_busy  <= 1'b1;
                spi_shift <= spi_data_hold;
                spi_sda   <= spi_data_hold[7];
            end
        end
        else begin
            if (spi_div_cnt == SPI_DIV - 1'b1) begin
                spi_div_cnt <= 16'd0;

                if (!spi_scl) begin
                    spi_scl <= 1'b1;       // 上升沿，屏幕采样 MOSI
                end
                else begin
                    spi_scl <= 1'b0;       // 下降沿，FPGA 更新下一位
                    if (spi_bit_cnt == 3'd7) begin
                        if (CS_PULSE_EN) begin
                            // 字节已完整发完，让 CS 拉高形成标准事务边界。
                            spi_busy         <= 1'b0;
                            cs_wait_cnt      <= 16'd0;
                            cs_pulse_pending <= 1'b1;
                        end
                        else begin
                            spi_busy <= 1'b0;
                        end
                    end
                    else begin
                        spi_bit_cnt <= spi_bit_cnt + 1'b1;
                        spi_shift   <= {spi_shift[6:0], 1'b0};
                        spi_sda     <= spi_shift[6];
                    end
                end
            end
            else begin
                spi_div_cnt <= spi_div_cnt + 1'b1;
            end
        end
    end

    //=========================================================================
    // 状态页图层参数（170x320 竖屏，只显示“当前状态 + 对应标识”）。
    //   外框   : 6 px 状态色描边
    //   图标   : 100 x 100，位于 x 35..134 / y 45..144
    //   状态词 : 16x32 点阵，y 168..199
    //   原因   : 16x32 点阵，y 208..239
    //   时间   : 16x32 点阵，y 240..271，8 个 16 px 槽位 = HH:MM:SS
    //   色条   : y 272..287，整幅宽的一条状态色
    //=========================================================================
    localparam [8:0]  TXT_X_OFF   = 9'd5;
    localparam [9:0]  TXT_X_END   = 10'd165;
    localparam [9:0]  LINE1_Y_OFF = 10'd168;
    localparam [9:0]  LINE1_Y_END = 10'd200;
    localparam [9:0]  LINE2_Y_OFF = 10'd208;
    localparam [9:0]  LINE2_Y_END = 10'd240;
    localparam [8:0]  ICON_X0     = 9'd35;
    localparam [9:0]  ICON_Y0     = 10'd45;
    localparam [8:0]  BORDER_W    = 9'd6;
    localparam [9:0]  BAR_Y_OFF   = 10'd272;
    localparam [9:0]  BAR_Y_END   = 10'd288;
    localparam [15:0] COLOR_GREEN = 16'h07E0;   // 正常
    localparam [15:0] COLOR_YELLOW= 16'hFFE0;   // 预警
    localparam [15:0] COLOR_RED   = 16'hF800;   // 火焰
    localparam [15:0] COLOR_TEXT  = 16'hFFFF;   // 正文：白色
    localparam [15:0] COLOR_BG    = 16'h0000;   // 背景：黑色

    // 状态码：0 = 全部正常，1 = 预警（高温 / 高湿 / 烟雾 / 有人），
    //          2 = 火焰报警。
    // 判定结果全部来自顶层 FPGA 逻辑，本模块只负责显示。
    wire [1:0]  status_code  = flame_alarm ? 2'd2 :
                               (temp_alarm | humi_alarm | smoke_alarm |
                                pir_active) ? 2'd1 : 2'd0;
    wire [15:0] status_color = (status_code == 2'd2) ? COLOR_RED :
                               (status_code == 2'd1) ? COLOR_YELLOW : COLOR_GREEN;

    //=========================================================================
    // 图标几何：100x100 的局部坐标，ic_ax / ic_ay 是到中心 (50,50) 的距离。
    //=========================================================================
    wire        in_icon  = (x_pos >= ICON_X0) && (x_pos < ICON_X0 + 9'd100) &&
                           (y_pos >= ICON_Y0) && (y_pos < ICON_Y0 + 10'd100);
    wire [6:0]  ic_x     = x_pos - ICON_X0;
    wire [6:0]  ic_y     = y_pos - ICON_Y0;
    wire [6:0]  ic_ax    = (ic_x > 7'd50) ? (ic_x - 7'd50) : (7'd50 - ic_x);
    wire [6:0]  ic_ay    = (ic_y > 7'd50) ? (ic_y - 7'd50) : (7'd50 - ic_y);

    // 正常：绿色圆底 + 白色对勾。
    // 圆用半径查表得到：radius_y(ax) = floor(sqrt(45^2 - ax^2))，
    // 判据 ay <= radius_y(ax) 等价于 ax^2 + ay^2 <= 45^2，是真正的圆，
    // 不像菱形近似那样在四个斜角处缺肉。
    reg  [6:0] radius_y;
    always @(*) begin
        case (ic_ax)
            7'd0:    radius_y = 7'd45;
            7'd1:    radius_y = 7'd44;
            7'd2:    radius_y = 7'd44;
            7'd3:    radius_y = 7'd44;
            7'd4:    radius_y = 7'd44;
            7'd5:    radius_y = 7'd44;
            7'd6:    radius_y = 7'd44;
            7'd7:    radius_y = 7'd44;
            7'd8:    radius_y = 7'd44;
            7'd9:    radius_y = 7'd44;
            7'd10:   radius_y = 7'd43;
            7'd11:   radius_y = 7'd43;
            7'd12:   radius_y = 7'd43;
            7'd13:   radius_y = 7'd43;
            7'd14:   radius_y = 7'd42;
            7'd15:   radius_y = 7'd42;
            7'd16:   radius_y = 7'd42;
            7'd17:   radius_y = 7'd41;
            7'd18:   radius_y = 7'd41;
            7'd19:   radius_y = 7'd40;
            7'd20:   radius_y = 7'd40;
            7'd21:   radius_y = 7'd39;
            7'd22:   radius_y = 7'd39;
            7'd23:   radius_y = 7'd38;
            7'd24:   radius_y = 7'd38;
            7'd25:   radius_y = 7'd37;
            7'd26:   radius_y = 7'd36;
            7'd27:   radius_y = 7'd36;
            7'd28:   radius_y = 7'd35;
            7'd29:   radius_y = 7'd34;
            7'd30:   radius_y = 7'd33;
            7'd31:   radius_y = 7'd32;
            7'd32:   radius_y = 7'd31;
            7'd33:   radius_y = 7'd30;
            7'd34:   radius_y = 7'd29;
            7'd35:   radius_y = 7'd28;
            7'd36:   radius_y = 7'd27;
            7'd37:   radius_y = 7'd25;
            7'd38:   radius_y = 7'd24;
            7'd39:   radius_y = 7'd22;
            7'd40:   radius_y = 7'd20;
            7'd41:   radius_y = 7'd18;
            7'd42:   radius_y = 7'd16;
            7'd43:   radius_y = 7'd13;
            7'd44:   radius_y = 7'd9;
            default: radius_y = 7'd0;
        endcase
    end
    wire icon_circle = (ic_ax <= 7'd45) && (ic_ay <= radius_y);

    // 对勾：两段等宽的粗斜线，宽 11 px，交汇在 (46, 76) 附近。
    wire [7:0] chk_dn = ic_x + 8'd30;              // 左低段 y = x + 30
    wire [7:0] chk_up = 8'd168 - ic_x - ic_x;      // 右高段 y = 168 - 2x
    wire icon_seg_a  = (ic_x >= 7'd30) && (ic_x <= 7'd46) &&
                       (ic_y + 7'd5 >= chk_dn) && (ic_y <= chk_dn + 7'd5);
    wire icon_seg_b  = (ic_x >= 7'd46) && (ic_x <= 7'd72) &&
                       (ic_y + 7'd5 >= chk_up) && (ic_y <= chk_up + 7'd5);
    wire icon_check  = icon_seg_a || icon_seg_b;

    // 预警：黄色三角 + 黑色感叹号。
    wire icon_tri    = (ic_y >= 7'd8) && (ic_y <= 7'd92) &&
                       (ic_ax <= ((ic_y - 7'd8) >> 1));
    wire icon_bar    = (ic_x >= 7'd45) && (ic_x <= 7'd55) &&
                       (ic_y >= 7'd32) && (ic_y <= 7'd64);
    wire icon_dot    = (ic_x >= 7'd45) && (ic_x <= 7'd55) &&
                       (ic_y >= 7'd74) && (ic_y <= 7'd84);

    // 火焰：按行取半宽的火焰轮廓，内部再套一层黄色火心。
    // 半宽表是"细火尖 -> 饱满的下半部 -> 底部收圆"的曲线，外轮廓才像火苗；
    // 再叠加一个中线偏移表，让火尖往一侧弯，不至于像一根直筒。
    reg  [6:0] flame_w;
    always @(*) begin
        case (ic_y[6:2])
            5'd0:    flame_w = 7'd0;
            5'd1:    flame_w = 7'd1;
            5'd2:    flame_w = 7'd2;
            5'd3:    flame_w = 7'd3;
            5'd4:    flame_w = 7'd4;
            5'd5:    flame_w = 7'd6;
            5'd6:    flame_w = 7'd7;
            5'd7:    flame_w = 7'd9;
            5'd8:    flame_w = 7'd11;
            5'd9:    flame_w = 7'd13;
            5'd10:   flame_w = 7'd15;
            5'd11:   flame_w = 7'd17;
            5'd12:   flame_w = 7'd19;
            5'd13:   flame_w = 7'd21;
            5'd14:   flame_w = 7'd24;
            5'd15:   flame_w = 7'd26;
            5'd16:   flame_w = 7'd29;
            5'd17:   flame_w = 7'd32;
            5'd18:   flame_w = 7'd35;
            5'd19:   flame_w = 7'd38;
            5'd20:   flame_w = 7'd41;
            5'd21:   flame_w = 7'd44;
            5'd22:   flame_w = 7'd46;
            5'd23:   flame_w = 7'd46;
            default: flame_w = 7'd38;
        endcase
    end

    reg  [6:0] flame_off;
    always @(*) begin
        case (ic_y[6:2])
            5'd0:    flame_off = 7'd5;
            5'd1:    flame_off = 7'd5;
            5'd2:    flame_off = 7'd4;
            5'd3:    flame_off = 7'd4;
            5'd4:    flame_off = 7'd3;
            5'd5:    flame_off = 7'd3;
            5'd6:    flame_off = 7'd2;
            5'd7:    flame_off = 7'd2;
            5'd8:    flame_off = 7'd2;
            5'd9:    flame_off = 7'd1;
            5'd10:   flame_off = 7'd1;
            5'd11:   flame_off = 7'd1;
            default: flame_off = 7'd0;
        endcase
    end

    // 火苗中线随高度偏移（50 + 偏移），轮廓和火心都绕这条线对称。
    wire [6:0] flame_cx = 7'd50 + flame_off;
    wire [6:0] flame_ax = (ic_x > flame_cx) ? (ic_x - flame_cx) : (flame_cx - ic_x);
    wire [6:0] core_w    = (flame_w >> 1) + (flame_w >> 3);
    wire icon_flame_out  = (ic_y >= 7'd4) && (flame_ax <= flame_w);
    wire icon_flame_core = (ic_y >= 7'd36) && (flame_ax <= core_w);

    wire icon_hit = in_icon &&
                    ((status_code == 2'd0) ? (icon_circle | icon_check) :
                     (status_code == 2'd1) ? icon_tri :
                                             icon_flame_out);
    wire [15:0] icon_rgb =
        (status_code == 2'd0) ? (icon_check ? COLOR_TEXT : COLOR_GREEN) :
        (status_code == 2'd1) ? ((icon_bar | icon_dot) ? COLOR_BG : COLOR_YELLOW) :
                                (icon_flame_core ? COLOR_YELLOW : COLOR_RED);

    //=========================================================================
    // 字符生成：只有两行文字，每行 10 列 x 16 x 32 点阵。
    //   第 1 行：NORMAL / WARNING / FIRE     整机状态词
    //   第 2 行：ALL CLEAR / 具体原因 / ALARM
    // 具体数值（温度、湿度、火焰 AO、时钟、蓝牙）不再上屏，屏幕只回答
    // “现在是什么状态”。
    //=========================================================================
    function [6:0] head_char;
        input [3:0] col;
        begin
            head_char = 7'h20;      // 默认空格
            case (status_code)
                2'd0: begin         // NORMAL，居中在第 2..7 列
                    case (col)
                        4'd2: head_char = 7'h4E;   // N
                        4'd3: head_char = 7'h4F;   // O
                        4'd4: head_char = 7'h52;   // R
                        4'd5: head_char = 7'h4D;   // M
                        4'd6: head_char = 7'h41;   // A
                        4'd7: head_char = 7'h4C;   // L
                        default: head_char = 7'h20;
                    endcase
                end
                2'd1: begin         // WARNING，居中在第 1..7 列
                    case (col)
                        4'd1: head_char = 7'h57;   // W
                        4'd2: head_char = 7'h41;   // A
                        4'd3: head_char = 7'h52;   // R
                        4'd4: head_char = 7'h4E;   // N
                        4'd5: head_char = 7'h49;   // I
                        4'd6: head_char = 7'h4E;   // N
                        4'd7: head_char = 7'h47;   // G
                        default: head_char = 7'h20;
                    endcase
                end
                default: begin      // FIRE，居中在第 3..6 列
                    case (col)
                        4'd3: head_char = 7'h46;   // F
                        4'd4: head_char = 7'h49;   // I
                        4'd5: head_char = 7'h52;   // R
                        4'd6: head_char = 7'h45;   // E
                        default: head_char = 7'h20;
                    endcase
                end
            endcase
        end
    endfunction

    function [6:0] detail_char;
        input [3:0] col;
        begin
            detail_char = 7'h20;
            case (status_code)
                2'd0: begin         // ALL CLEAR，第 0..8 列
                    case (col)
                        4'd0: detail_char = 7'h41;   // A
                        4'd1: detail_char = 7'h4C;   // L
                        4'd2: detail_char = 7'h4C;   // L
                        4'd4: detail_char = 7'h43;   // C
                        4'd5: detail_char = 7'h4C;   // L
                        4'd6: detail_char = 7'h45;   // E
                        4'd7: detail_char = 7'h41;   // A
                        4'd8: detail_char = 7'h52;   // R
                        default: detail_char = 7'h20;
                    endcase
                end
                2'd1: begin         // 预警：显示最严重的一个原因
                    if (smoke_alarm) begin
                        case (col)      // SMOKE，居中在第 3..7 列
                            4'd3: detail_char = 7'h53;   // S
                            4'd4: detail_char = 7'h4D;   // M
                            4'd5: detail_char = 7'h4F;   // O
                            4'd6: detail_char = 7'h4B;   // K
                            4'd7: detail_char = 7'h45;   // E
                            default: detail_char = 7'h20;
                        endcase
                    end
                    else if (temp_alarm && humi_alarm) begin
                        case (col)      // TEMP+HUMI，第 0..8 列
                            4'd0: detail_char = 7'h54;   // T
                            4'd1: detail_char = 7'h45;   // E
                            4'd2: detail_char = 7'h4D;   // M
                            4'd3: detail_char = 7'h50;   // P
                            4'd4: detail_char = 7'h2B;   // +
                            4'd5: detail_char = 7'h48;   // H
                            4'd6: detail_char = 7'h55;   // U
                            4'd7: detail_char = 7'h4D;   // M
                            4'd8: detail_char = 7'h49;   // I
                            default: detail_char = 7'h20;
                        endcase
                    end
                    else if (temp_alarm) begin
                        case (col)      // TEMP HIGH，第 0..8 列
                            4'd0: detail_char = 7'h54;   // T
                            4'd1: detail_char = 7'h45;   // E
                            4'd2: detail_char = 7'h4D;   // M
                            4'd3: detail_char = 7'h50;   // P
                            4'd5: detail_char = 7'h48;   // H
                            4'd6: detail_char = 7'h49;   // I
                            4'd7: detail_char = 7'h47;   // G
                            4'd8: detail_char = 7'h48;   // H
                            default: detail_char = 7'h20;
                        endcase
                    end
                    else if (humi_alarm) begin
                        case (col)      // HUMI HIGH，第 0..8 列
                            4'd0: detail_char = 7'h48;   // H
                            4'd1: detail_char = 7'h55;   // U
                            4'd2: detail_char = 7'h4D;   // M
                            4'd3: detail_char = 7'h49;   // I
                            4'd5: detail_char = 7'h48;   // H
                            4'd6: detail_char = 7'h49;   // I
                            4'd7: detail_char = 7'h47;   // G
                            4'd8: detail_char = 7'h48;   // H
                            default: detail_char = 7'h20;
                        endcase
                    end
                    else begin
                        case (col)      // PERSON，居中在第 2..7 列
                            4'd2: detail_char = 7'h50;   // P
                            4'd3: detail_char = 7'h45;   // E
                            4'd4: detail_char = 7'h52;   // R
                            4'd5: detail_char = 7'h53;   // S
                            4'd6: detail_char = 7'h4F;   // O
                            4'd7: detail_char = 7'h4E;   // N
                            default: detail_char = 7'h20;
                        endcase
                    end
                end
                // 火焰报警：第 1 行已经写了 FIRE，第 2 行只补 ALARM。
                // 10 列 16 px 点阵正好 160 px，比内框宽 2 px，
                // 写满 10 列会被外框切掉首尾，所以这里固定 5 个字符。
                default: begin      // ALARM，居中在第 2..6 列
                    case (col)
                        4'd2: detail_char = 7'h41;   // A
                        4'd3: detail_char = 7'h4C;   // L
                        4'd4: detail_char = 7'h41;   // A
                        4'd5: detail_char = 7'h52;   // R
                        4'd6: detail_char = 7'h4D;   // M
                        default: detail_char = 7'h20;
                    endcase
                end
            endcase
        end
    endfunction

    // 当前像素落点：是否落在某一行文字上、第几个字符格、格内坐标。
    wire        in_line1  = (x_pos >= TXT_X_OFF) && (x_pos < TXT_X_END) &&
                            (y_pos >= LINE1_Y_OFF) && (y_pos < LINE1_Y_END);
    wire        in_line2  = (x_pos >= TXT_X_OFF) && (x_pos < TXT_X_END) &&
                            (y_pos >= LINE2_Y_OFF) && (y_pos < LINE2_Y_END);
    wire [9:0]  dx        = {1'b0, x_pos} - TXT_X_OFF;
    wire [3:0]  cell_col  = dx[7:4];      // dx / 16，0..9
    wire [3:0]  cell_px   = dx[3:0];      // 格内横坐标 0..15
    wire [9:0]  dy1       = {1'b0, y_pos} - LINE1_Y_OFF;
    wire [9:0]  dy2       = {1'b0, y_pos} - LINE2_Y_OFF;
    // 两行共用同一个字库 ROM，按当前落点选字模和行号。
    wire [4:0]  cell_row  = in_line2 ? dy2[4:0] : dy1[4:0];
    wire [6:0]  char_code = in_line2 ? detail_char(cell_col) : head_char(cell_col);

    wire [15:0] glyph_bits;
    font16x32_rom u_font16x32 (
        .ch   (char_code),
        .row  (cell_row),
        .bits (glyph_bits)
    );

    // 厂商例程每字节都是 bit0 先画：bit0 = 该行最左像素。
    wire        glyph_px     = glyph_bits[cell_px];

    //=========================================================================
    // 实时时间行：HH:MM:SS，取 DS1307 的时/分/秒 BCD。
    //   8 个等宽槽位（16 px）：数字 6 个 + 冒号 2 个，共 128 px；
    //   放在 x=21..148、y=240..271，夹在原因行与底部色条之间，互不重叠。
    //   槽位 0/1=时十/个，2=冒号，3/4=分十/个，5=冒号，6/7=秒十/个。
    //   整屏每 0.5 s 重绘一次，RTC 每秒更新，秒位会自动跳动。
    //=========================================================================
    localparam [9:0]  TIME_X_OFF = 10'd21;
    localparam [9:0]  TIME_Y_OFF = 10'd240;
    localparam [9:0]  TIME_Y_END = 10'd272;
    localparam [15:0] COLOR_DIM  = 16'h8410;   // 冒号用暗白色，避免抢状态色

    wire       in_time   = (x_pos >= TIME_X_OFF) && (x_pos < TIME_X_OFF + 10'd128) &&
                           (y_pos >= TIME_Y_OFF) && (y_pos < TIME_Y_END);
    wire [9:0] tdx       = {1'b0, x_pos} - TIME_X_OFF;
    wire [2:0] time_char = tdx[7:4];      // 0..7：6 位数字 + 2 个冒号槽
    wire [3:0] time_px   = tdx[3:0];
    wire [4:0] time_row  = {1'b0, y_pos} - TIME_Y_OFF;
    wire       time_is_colon = (time_char == 3'd2) || (time_char == 3'd5);

    // 按槽位取 BCD 数位；冒号槽返回 0，由 time_is_colon 屏蔽掉数字笔画。
    function [3:0] time_digit;
        input [2:0] slot;
        begin
            case (slot)
                3'd0: time_digit = rtc_hour_bcd[7:4];
                3'd1: time_digit = rtc_hour_bcd[3:0];
                3'd3: time_digit = rtc_min_bcd[7:4];
                3'd4: time_digit = rtc_min_bcd[3:0];
                3'd6: time_digit = rtc_sec_bcd[7:4];
                3'd7: time_digit = rtc_sec_bcd[3:0];
                default: time_digit = 4'd0;
            endcase
        end
    endfunction

    // 16x32 数字字模：直接取厂商点阵中 0..9 的 32 行，避免手绘误差。
    function [15:0] digit_mask;
        input [3:0] d;
        input [4:0] row;
        begin
            digit_mask = 16'h0000;
            case (d)
                // 0
                4'd0: case (row)
                    5'd6: digit_mask = 16'h07C0;
                    5'd7: digit_mask = 16'h0C60;
                    5'd8: digit_mask = 16'h1830;
                    5'd9: digit_mask = 16'h3018;
                    5'd10: digit_mask = 16'h3018;
                    5'd11: digit_mask = 16'h2018;
                    5'd12: digit_mask = 16'h600C;
                    5'd13: digit_mask = 16'h600C;
                    5'd14: digit_mask = 16'h600C;
                    5'd15: digit_mask = 16'h600C;
                    5'd16: digit_mask = 16'h600C;
                    5'd17: digit_mask = 16'h600C;
                    5'd18: digit_mask = 16'h600C;
                    5'd19: digit_mask = 16'h600C;
                    5'd20: digit_mask = 16'h600C;
                    5'd21: digit_mask = 16'h2018;
                    5'd22: digit_mask = 16'h3018;
                    5'd23: digit_mask = 16'h3018;
                    5'd24: digit_mask = 16'h1830;
                    5'd25: digit_mask = 16'h0C60;
                    5'd26: digit_mask = 16'h07C0;
                    default: digit_mask = 16'h0000;
                endcase
                // 1
                4'd1: case (row)
                    5'd6: digit_mask = 16'h0100;
                    5'd7: digit_mask = 16'h0180;
                    5'd8: digit_mask = 16'h01F8;
                    5'd9: digit_mask = 16'h0180;
                    5'd10: digit_mask = 16'h0180;
                    5'd11: digit_mask = 16'h0180;
                    5'd12: digit_mask = 16'h0180;
                    5'd13: digit_mask = 16'h0180;
                    5'd14: digit_mask = 16'h0180;
                    5'd15: digit_mask = 16'h0180;
                    5'd16: digit_mask = 16'h0180;
                    5'd17: digit_mask = 16'h0180;
                    5'd18: digit_mask = 16'h0180;
                    5'd19: digit_mask = 16'h0180;
                    5'd20: digit_mask = 16'h0180;
                    5'd21: digit_mask = 16'h0180;
                    5'd22: digit_mask = 16'h0180;
                    5'd23: digit_mask = 16'h0180;
                    5'd24: digit_mask = 16'h0180;
                    5'd25: digit_mask = 16'h03C0;
                    5'd26: digit_mask = 16'h1FF8;
                    default: digit_mask = 16'h0000;
                endcase
                // 2
                4'd2: case (row)
                    5'd6: digit_mask = 16'h07E0;
                    5'd7: digit_mask = 16'h1C10;
                    5'd8: digit_mask = 16'h1808;
                    5'd9: digit_mask = 16'h3004;
                    5'd10: digit_mask = 16'h3004;
                    5'd11: digit_mask = 16'h300C;
                    5'd12: digit_mask = 16'h300C;
                    5'd13: digit_mask = 16'h3000;
                    5'd14: digit_mask = 16'h1800;
                    5'd15: digit_mask = 16'h0800;
                    5'd16: digit_mask = 16'h0400;
                    5'd17: digit_mask = 16'h0200;
                    5'd18: digit_mask = 16'h0100;
                    5'd19: digit_mask = 16'h0080;
                    5'd20: digit_mask = 16'h0040;
                    5'd21: digit_mask = 16'h2020;
                    5'd22: digit_mask = 16'h2010;
                    5'd23: digit_mask = 16'h2008;
                    5'd24: digit_mask = 16'h3004;
                    5'd25: digit_mask = 16'h1FFC;
                    5'd26: digit_mask = 16'h1FFC;
                    default: digit_mask = 16'h0000;
                endcase
                // 3
                4'd3: case (row)
                    5'd6: digit_mask = 16'h03E0;
                    5'd7: digit_mask = 16'h0E18;
                    5'd8: digit_mask = 16'h0C0C;
                    5'd9: digit_mask = 16'h180C;
                    5'd10: digit_mask = 16'h180C;
                    5'd11: digit_mask = 16'h180C;
                    5'd12: digit_mask = 16'h1800;
                    5'd13: digit_mask = 16'h0C00;
                    5'd14: digit_mask = 16'h0600;
                    5'd15: digit_mask = 16'h03C0;
                    5'd16: digit_mask = 16'h0E00;
                    5'd17: digit_mask = 16'h1800;
                    5'd18: digit_mask = 16'h1000;
                    5'd19: digit_mask = 16'h3000;
                    5'd20: digit_mask = 16'h3000;
                    5'd21: digit_mask = 16'h300C;
                    5'd22: digit_mask = 16'h300C;
                    5'd23: digit_mask = 16'h100C;
                    5'd24: digit_mask = 16'h180C;
                    5'd25: digit_mask = 16'h0C18;
                    5'd26: digit_mask = 16'h03E0;
                    default: digit_mask = 16'h0000;
                endcase
                // 4
                4'd4: case (row)
                    5'd6: digit_mask = 16'h0C00;
                    5'd7: digit_mask = 16'h0E00;
                    5'd8: digit_mask = 16'h0E00;
                    5'd9: digit_mask = 16'h0F00;
                    5'd10: digit_mask = 16'h0E80;
                    5'd11: digit_mask = 16'h0E80;
                    5'd12: digit_mask = 16'h0E40;
                    5'd13: digit_mask = 16'h0E60;
                    5'd14: digit_mask = 16'h0E20;
                    5'd15: digit_mask = 16'h0E10;
                    5'd16: digit_mask = 16'h0E10;
                    5'd17: digit_mask = 16'h0E08;
                    5'd18: digit_mask = 16'h0E04;
                    5'd19: digit_mask = 16'h0E04;
                    5'd20: digit_mask = 16'h7FFE;
                    5'd21: digit_mask = 16'h0E00;
                    5'd22: digit_mask = 16'h0E00;
                    5'd23: digit_mask = 16'h0E00;
                    5'd24: digit_mask = 16'h0E00;
                    5'd25: digit_mask = 16'h0E00;
                    5'd26: digit_mask = 16'h0E00;
                    5'd27: digit_mask = 16'h7FC0;
                    default: digit_mask = 16'h0000;
                endcase
                // 5
                4'd5: case (row)
                    5'd6: digit_mask = 16'h3FF0;
                    5'd7: digit_mask = 16'h3FF0;
                    5'd8: digit_mask = 16'h0010;
                    5'd9: digit_mask = 16'h0010;
                    5'd10: digit_mask = 16'h0010;
                    5'd11: digit_mask = 16'h0008;
                    5'd12: digit_mask = 16'h0008;
                    5'd13: digit_mask = 16'h07C8;
                    5'd14: digit_mask = 16'h0C28;
                    5'd15: digit_mask = 16'h1818;
                    5'd16: digit_mask = 16'h1008;
                    5'd17: digit_mask = 16'h3000;
                    5'd18: digit_mask = 16'h3000;
                    5'd19: digit_mask = 16'h3000;
                    5'd20: digit_mask = 16'h3000;
                    5'd21: digit_mask = 16'h300C;
                    5'd22: digit_mask = 16'h300C;
                    5'd23: digit_mask = 16'h1804;
                    5'd24: digit_mask = 16'h1804;
                    5'd25: digit_mask = 16'h0C08;
                    5'd26: digit_mask = 16'h03F0;
                    default: digit_mask = 16'h0000;
                endcase
                // 6
                4'd6: case (row)
                    5'd6: digit_mask = 16'h0F80;
                    5'd7: digit_mask = 16'h10C0;
                    5'd8: digit_mask = 16'h3020;
                    5'd9: digit_mask = 16'h3010;
                    5'd10: digit_mask = 16'h0018;
                    5'd11: digit_mask = 16'h0018;
                    5'd12: digit_mask = 16'h0008;
                    5'd13: digit_mask = 16'h000C;
                    5'd14: digit_mask = 16'h0F8C;
                    5'd15: digit_mask = 16'h186C;
                    5'd16: digit_mask = 16'h303C;
                    5'd17: digit_mask = 16'h601C;
                    5'd18: digit_mask = 16'h600C;
                    5'd19: digit_mask = 16'h600C;
                    5'd20: digit_mask = 16'h600C;
                    5'd21: digit_mask = 16'h600C;
                    5'd22: digit_mask = 16'h6018;
                    5'd23: digit_mask = 16'h2018;
                    5'd24: digit_mask = 16'h3030;
                    5'd25: digit_mask = 16'h1860;
                    5'd26: digit_mask = 16'h07C0;
                    default: digit_mask = 16'h0000;
                endcase
                // 7
                4'd7: case (row)
                    5'd6: digit_mask = 16'h3FF8;
                    5'd7: digit_mask = 16'h3FF8;
                    5'd8: digit_mask = 16'h101C;
                    5'd9: digit_mask = 16'h080C;
                    5'd10: digit_mask = 16'h0804;
                    5'd11: digit_mask = 16'h0404;
                    5'd12: digit_mask = 16'h0400;
                    5'd13: digit_mask = 16'h0200;
                    5'd14: digit_mask = 16'h0200;
                    5'd15: digit_mask = 16'h0100;
                    5'd16: digit_mask = 16'h0100;
                    5'd17: digit_mask = 16'h0100;
                    5'd18: digit_mask = 16'h0080;
                    5'd19: digit_mask = 16'h0080;
                    5'd20: digit_mask = 16'h0080;
                    5'd21: digit_mask = 16'h00C0;
                    5'd22: digit_mask = 16'h00C0;
                    5'd23: digit_mask = 16'h00C0;
                    5'd24: digit_mask = 16'h00C0;
                    5'd25: digit_mask = 16'h00C0;
                    5'd26: digit_mask = 16'h00C0;
                    default: digit_mask = 16'h0000;
                endcase
                // 8
                4'd8: case (row)
                    5'd6: digit_mask = 16'h07E0;
                    5'd7: digit_mask = 16'h0C30;
                    5'd8: digit_mask = 16'h1818;
                    5'd9: digit_mask = 16'h300C;
                    5'd10: digit_mask = 16'h300C;
                    5'd11: digit_mask = 16'h300C;
                    5'd12: digit_mask = 16'h301C;
                    5'd13: digit_mask = 16'h1838;
                    5'd14: digit_mask = 16'h0870;
                    5'd15: digit_mask = 16'h07E0;
                    5'd16: digit_mask = 16'h07B0;
                    5'd17: digit_mask = 16'h0E18;
                    5'd18: digit_mask = 16'h1C0C;
                    5'd19: digit_mask = 16'h3806;
                    5'd20: digit_mask = 16'h3006;
                    5'd21: digit_mask = 16'h3006;
                    5'd22: digit_mask = 16'h3006;
                    5'd23: digit_mask = 16'h3006;
                    5'd24: digit_mask = 16'h180C;
                    5'd25: digit_mask = 16'h0C18;
                    5'd26: digit_mask = 16'h03E0;
                    default: digit_mask = 16'h0000;
                endcase
                // 9
                default: case (row)
                    5'd6: digit_mask = 16'h03E0;
                    5'd7: digit_mask = 16'h0418;
                    5'd8: digit_mask = 16'h080C;
                    5'd9: digit_mask = 16'h180C;
                    5'd10: digit_mask = 16'h1006;
                    5'd11: digit_mask = 16'h3006;
                    5'd12: digit_mask = 16'h3006;
                    5'd13: digit_mask = 16'h3006;
                    5'd14: digit_mask = 16'h3006;
                    5'd15: digit_mask = 16'h3806;
                    5'd16: digit_mask = 16'h3C0C;
                    5'd17: digit_mask = 16'h3618;
                    5'd18: digit_mask = 16'h31F0;
                    5'd19: digit_mask = 16'h3000;
                    5'd20: digit_mask = 16'h1800;
                    5'd21: digit_mask = 16'h1800;
                    5'd22: digit_mask = 16'h1800;
                    5'd23: digit_mask = 16'h0C0C;
                    5'd24: digit_mask = 16'h060C;
                    5'd25: digit_mask = 16'h030C;
                    5'd26: digit_mask = 16'h01F0;
                    default: digit_mask = 16'h0000;
                endcase
            endcase
        end
    endfunction

    // 时间行：数字槽取 16x32 数字字模，冒号槽在槽位中部点两个 4x3 的方点。
    wire [15:0] digit_bits = digit_mask(time_digit(time_char), time_row);
    wire       digit_hit  = in_time && rtc_ok && !time_is_colon && digit_bits[time_px];
    wire       colon_hit  = in_time && rtc_ok &&  time_is_colon &&
                            (((time_row >= 5'd9) && (time_row <= 5'd11)) ||
                             ((time_row >= 5'd20) && (time_row <= 5'd22))) &&
                            (time_px >= 4'd6) && (time_px <= 4'd9);
    // 状态词用状态色，原因行用白色；文字只落在自己那一行里。
    wire        text_hit     = (in_line1 && glyph_px) || (in_line2 && glyph_px);
    wire [15:0] text_rgb     = in_line1 ? status_color : COLOR_TEXT;

    // 外框和底部色条都用状态色，构成一眼可辨的"标识"。
    wire        in_border    = (x_pos < BORDER_W) || (x_pos > X_LAST - BORDER_W) ||
                               (y_pos < BORDER_W) || (y_pos > Y_LAST - BORDER_W);
    wire        in_bar       = (y_pos >= BAR_Y_OFF) && (y_pos < BAR_Y_END);

    wire [15:0] pattern_color = in_border ? status_color :
                                in_bar    ? status_color :
                                icon_hit  ? icon_rgb     :
                                text_hit  ? text_rgb     :
                                digit_hit ? COLOR_TEXT   :
                                colon_hit ? COLOR_DIM    :
                                            COLOR_BG;

    //=========================================================================
    // 显示主状态机。
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            delay_cnt     <= 23'd0;
            init_wait_cnt <= 24'd0;
            refresh_cnt   <= 26'd0;
            init_idx      <= 7'd0;
            setup_stage   <= 3'd0;
            x_pos         <= 9'd0;
            y_pos         <= 9'd0;
            spi_start     <= 1'b0;
            spi_seen      <= 1'b0;
            spi_data_hold <= 8'h00;
            cur_word      <= 9'h100;
            tft_res_n     <= 1'b0;
            tft_dc        <= 1'b0;
        end
        else begin
            spi_start <= 1'b0;

            case (state)
                ST_IDLE: begin
                    delay_cnt <= 23'd0;
                    tft_res_n <= 1'b0;
                    state     <= ST_RESET_LOW;
                end

                // 复位脚低电平至少 10 ms。
                ST_RESET_LOW: begin
                    tft_res_n <= 1'b0;
                    if (delay_cnt == RESET_LOW_CYCLES - 1'b1) begin
                        delay_cnt <= 23'd0;
                        tft_res_n <= 1'b1;
                        state     <= ST_RESET_HIGH;
                    end
                    else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // 复位释放后再等 100 ms，让屏幕内部稳压稳定（与厂商例程一致）。
                ST_RESET_HIGH: begin
                    if (delay_cnt == 23'd5_000_000 - 1'b1) begin
                        delay_cnt <= 23'd0;
                        init_idx  <= 7'd0;
                        spi_seen  <= 1'b0;
                        state     <= ST_INIT;
                    end
                    else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // 逐条发送初始化命令。
                ST_INIT: begin
                    if (spi_accept && !spi_seen) begin
                        if (init_idx > init_last) begin
                            setup_stage <= 3'd0;
                            state       <= ST_SET_X;
                        end
                        else begin
                            cur_word      <= init_word_now;
                            spi_data_hold <= init_word_now[7:0];
                            // 初始化表里 bit8=1 表示命令、bit8=0 表示数据，
                            // 而屏幕的 DC 是 0=命令、1=数据，所以要取反。
                            tft_dc        <= ~init_word_now[8];
                            spi_start     <= 1'b1;
                            spi_seen      <= 1'b1;
                        end
                    end
                    else if (spi_seen && spi_accept) begin
                        spi_seen <= 1'b0;

                        // 复位、退出睡眠和开显示之后需要给屏幕留出处理时间。
                        if (cur_word[8] &&
                            ((cur_word[7:0] == 8'h01) ||
                             (cur_word[7:0] == 8'h11) ||
                             (cur_word[7:0] == 8'h29))) begin
                            init_wait_cnt <= 24'd0;
                            state         <= ST_INIT_WAIT;
                        end
                        else begin
                            init_idx <= init_idx + 1'b1;
                        end
                    end
                end

                ST_INIT_WAIT: begin
                    if (init_wait_cnt == INIT_DELAY_CYCLES - 1'b1) begin
                        init_wait_cnt <= 24'd0;
                        init_idx      <= init_idx + 1'b1;
                        state         <= ST_INIT;
                    end
                    else begin
                        init_wait_cnt <= init_wait_cnt + 1'b1;
                    end
                end

                // 0x2A：列地址窗口，加入面板 X 偏移。
                ST_SET_X: begin
                    if (spi_accept && !spi_seen) begin
                        if (setup_stage == 3'd0) begin
                            tft_dc        <= 1'b0;
                            spi_data_hold <= 8'h2A;
                        end
                        else begin
                            tft_dc <= 1'b1;
                            case (setup_stage)
                                3'd1: spi_data_hold <= X_START_W[15:8];
                                3'd2: spi_data_hold <= X_START_W[7:0];
                                3'd3: spi_data_hold <= X_END_W[15:8];
                                default: spi_data_hold <= X_END_W[7:0];
                            endcase
                        end
                        spi_start <= 1'b1;
                        spi_seen  <= 1'b1;
                    end
                    else if (spi_seen && spi_accept) begin
                        spi_seen <= 1'b0;
                        if (setup_stage == 3'd4) begin
                            setup_stage <= 3'd0;
                            state       <= ST_SET_Y;
                        end
                        else begin
                            setup_stage <= setup_stage + 1'b1;
                        end
                    end
                end

                // 0x2B：页地址窗口，加入面板 Y 偏移。
                ST_SET_Y: begin
                    if (spi_accept && !spi_seen) begin
                        if (setup_stage == 3'd0) begin
                            tft_dc        <= 1'b0;
                            spi_data_hold <= 8'h2B;
                        end
                        else begin
                            tft_dc <= 1'b1;
                            case (setup_stage)
                                3'd1: spi_data_hold <= Y_START_W[15:8];
                                3'd2: spi_data_hold <= Y_START_W[7:0];
                                3'd3: spi_data_hold <= Y_END_W[15:8];
                                default: spi_data_hold <= Y_END_W[7:0];
                            endcase
                        end
                        spi_start <= 1'b1;
                        spi_seen  <= 1'b1;
                    end
                    else if (spi_seen && spi_accept) begin
                        spi_seen <= 1'b0;
                        if (setup_stage == 3'd4) begin
                            setup_stage <= 3'd0;
                            state       <= ST_RAMWR;
                        end
                        else begin
                            setup_stage <= setup_stage + 1'b1;
                        end
                    end
                end

                ST_RAMWR: begin
                    if (spi_accept && !spi_seen) begin
                        tft_dc        <= 1'b0;
                        spi_data_hold <= 8'h2C;
                        spi_start     <= 1'b1;
                        spi_seen      <= 1'b1;
                    end
                    else if (spi_seen && spi_accept) begin
                        spi_seen <= 1'b0;
                        x_pos    <= 9'd0;
                        y_pos    <= 9'd0;
                        state    <= ST_PIXEL_HI;
                    end
                end

                ST_PIXEL_HI: begin
                    if (spi_accept && !spi_seen) begin
                        tft_dc        <= 1'b1;
                        spi_data_hold <= pattern_color[15:8];
                        spi_start     <= 1'b1;
                        spi_seen      <= 1'b1;
                    end
                    else if (spi_seen && spi_accept) begin
                        spi_seen <= 1'b0;
                        state    <= ST_PIXEL_LO;
                    end
                end

                ST_PIXEL_LO: begin
                    if (spi_accept && !spi_seen) begin
                        tft_dc        <= 1'b1;
                        spi_data_hold <= pattern_color[7:0];
                        spi_start     <= 1'b1;
                        spi_seen      <= 1'b1;
                    end
                    else if (spi_seen && spi_accept) begin
                        spi_seen <= 1'b0;
                        if (x_pos == X_LAST) begin
                            x_pos <= 9'd0;
                            if (y_pos == Y_LAST) begin
                                refresh_cnt <= 26'd0;
                                state       <= ST_REFRESH;
                            end
                            else begin
                                y_pos <= y_pos + 1'b1;
                                state <= ST_PIXEL_HI;
                            end
                        end
                        else begin
                            x_pos <= x_pos + 1'b1;
                            state <= ST_PIXEL_HI;
                        end
                    end
                end

                // 色带画完后等待 0.5 s 再重画一次，便于观察屏幕是否响应。
                ST_REFRESH: begin
                    if (refresh_cnt == REFRESH_CYCLES - 1'b1) begin
                        refresh_cnt <= 26'd0;
                        setup_stage <= 3'd0;
                        state       <= ST_SET_X;
                    end
                    else begin
                        refresh_cnt <= refresh_cnt + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
