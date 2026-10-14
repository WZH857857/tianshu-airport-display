//=============================================================================
// 文件名   : ili9341_display.v
// 功能     : ILI9341 240x320 SPI 屏驱动与状态信息显示
// 显示内容 :
//   E104 SENSOR HUB
//   TEMP/HUM : NORMAL / ALARM
//   FLAME    : SAFE / FIRE
//   RTC      : 12:34:56 或 --:--:--
//   BLE      : ONLINE / NO LNK
// 备注     :
//   照片中的屏幕只标注了通用 8 针接口。本代码按最常见的 ILI9341
//   240x320 模块编写。如果实物使用 ST7789/ST7796，需要替换 INIT_ROM
//   的内容并按实际分辨率修改 X_LAST/Y_LAST。
//=============================================================================
module ili9341_display (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       th_alarm,
    input  wire       flame_alarm,
    input  wire       rtc_valid,
    input  wire [7:0] rtc_hour_bcd,
    input  wire [7:0] rtc_min_bcd,
    input  wire [7:0] rtc_sec_bcd,
    input  wire       ble_online,

    output wire       tft_cs_n,
    output reg        tft_res_n,
    output reg        tft_dc,
    output wire       tft_scl,
    output wire       tft_sda
);

    localparam integer X_LAST = 239;
    localparam integer Y_LAST = 319;
    localparam integer PIXELS = (X_LAST + 1) * (Y_LAST + 1);

    // 6.25 MHz SPI：250 ns 半周期。
    localparam [15:0] SPI_DIV = 16'd4;

    localparam [22:0] RESET_LOW_CYCLES  = 23'd500_000;    // 10 ms
    localparam [22:0] INIT_DELAY_CYCLES = 23'd6_000_000;  // 120 ms
    localparam [26:0] REFRESH_CYCLES    = 27'd50_000_000; // 1 s

    localparam [6:0]  INIT_LAST = 7'd87;

    localparam [4:0] ST_IDLE       = 5'd0;
    localparam [4:0] ST_RESET_LOW  = 5'd1;
    localparam [4:0] ST_RESET_HIGH = 5'd2;
    localparam [4:0] ST_INIT       = 5'd3;
    localparam [4:0] ST_INIT_WAIT  = 5'd4;
    localparam [4:0] ST_SET_X      = 5'd5;
    localparam [4:0] ST_SET_Y      = 5'd6;
    localparam [4:0] ST_RAMWR      = 5'd7;
    localparam [4:0] ST_PIXEL_HI   = 5'd8;
    localparam [4:0] ST_PIXEL_LO   = 5'd9;
    localparam [4:0] ST_REFRESH    = 5'd10;
    localparam [4:0] ST_PIXEL_CALC = 5'd11;
    localparam [4:0] ST_PIXEL_TEXT = 5'd12;
    localparam [4:0] ST_PIXEL_MIX  = 5'd13;

    reg [4:0]  state;
    reg [22:0] delay_cnt;
    reg [26:0] refresh_cnt;
    reg [6:0]  init_idx;
    reg [2:0]  setup_stage;
    reg [8:0]  x_pos;
    reg [8:0]  y_pos;

    reg [8:0]  cur_word;
    reg [15:0] pixel_color;
    reg [15:0] base_color_reg;
    reg [14:0] text_prep_reg;
    reg [16:0] text_pixel_reg;
    reg        spi_start;
    reg        spi_seen;
    reg        spi_busy;
    reg [15:0] spi_div_cnt;
    reg [2:0]  spi_bit_cnt;
    reg [7:0]  spi_shift;
    reg        spi_scl;
    reg        spi_sda;
    reg [7:0]  spi_data_hold;

    assign tft_cs_n = 1'b0;          // 整个工作时间保持片选有效
    assign tft_scl  = spi_scl;
    assign tft_sda  = spi_sda;

    //=========================================================================
    // 5x7 ASCII 字库。只实现本屏需要显示的大写字母、数字和符号。
    // 每个字符使用 5 行，每行低 5 位有效，显示在 8x8 字符格中。
    //=========================================================================
    function [7:0] glyph_row;
        input [7:0] ch;
        input [2:0] row;
        begin
            case (ch)
                8'h20: glyph_row = 8'h00; // space
                8'h2D: begin              // -
                    case (row)
                        3'd0: glyph_row = 8'h08;
                        3'd1: glyph_row = 8'h08;
                        3'd2: glyph_row = 8'h08;
                        3'd3: glyph_row = 8'h08;
                        3'd4: glyph_row = 8'h08;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h2F: begin              // /
                    case (row)
                        3'd0: glyph_row = 8'h20;
                        3'd1: glyph_row = 8'h10;
                        3'd2: glyph_row = 8'h08;
                        3'd3: glyph_row = 8'h04;
                        3'd4: glyph_row = 8'h02;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h30: begin              // 0
                    case (row)
                        3'd0: glyph_row = 8'h3E;
                        3'd1: glyph_row = 8'h51;
                        3'd2: glyph_row = 8'h49;
                        3'd3: glyph_row = 8'h45;
                        3'd4: glyph_row = 8'h3E;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h31: begin              // 1
                    case (row)
                        3'd0: glyph_row = 8'h00;
                        3'd1: glyph_row = 8'h42;
                        3'd2: glyph_row = 8'h7F;
                        3'd3: glyph_row = 8'h40;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h32: begin              // 2
                    case (row)
                        3'd0: glyph_row = 8'h42;
                        3'd1: glyph_row = 8'h61;
                        3'd2: glyph_row = 8'h51;
                        3'd3: glyph_row = 8'h49;
                        3'd4: glyph_row = 8'h46;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h33: begin              // 3
                    case (row)
                        3'd0: glyph_row = 8'h21;
                        3'd1: glyph_row = 8'h41;
                        3'd2: glyph_row = 8'h45;
                        3'd3: glyph_row = 8'h4B;
                        3'd4: glyph_row = 8'h31;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h34: begin              // 4
                    case (row)
                        3'd0: glyph_row = 8'h18;
                        3'd1: glyph_row = 8'h14;
                        3'd2: glyph_row = 8'h12;
                        3'd3: glyph_row = 8'h7F;
                        3'd4: glyph_row = 8'h10;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h35: begin              // 5
                    case (row)
                        3'd0: glyph_row = 8'h27;
                        3'd1: glyph_row = 8'h45;
                        3'd2: glyph_row = 8'h45;
                        3'd3: glyph_row = 8'h45;
                        3'd4: glyph_row = 8'h39;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h36: begin              // 6
                    case (row)
                        3'd0: glyph_row = 8'h3C;
                        3'd1: glyph_row = 8'h4A;
                        3'd2: glyph_row = 8'h49;
                        3'd3: glyph_row = 8'h49;
                        3'd4: glyph_row = 8'h30;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h37: begin              // 7
                    case (row)
                        3'd0: glyph_row = 8'h01;
                        3'd1: glyph_row = 8'h71;
                        3'd2: glyph_row = 8'h09;
                        3'd3: glyph_row = 8'h05;
                        3'd4: glyph_row = 8'h03;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h38: begin              // 8
                    case (row)
                        3'd0: glyph_row = 8'h36;
                        3'd1: glyph_row = 8'h49;
                        3'd2: glyph_row = 8'h49;
                        3'd3: glyph_row = 8'h49;
                        3'd4: glyph_row = 8'h36;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h39: begin              // 9
                    case (row)
                        3'd0: glyph_row = 8'h06;
                        3'd1: glyph_row = 8'h49;
                        3'd2: glyph_row = 8'h49;
                        3'd3: glyph_row = 8'h29;
                        3'd4: glyph_row = 8'h1E;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h3A: begin              // :
                    case (row)
                        3'd1: glyph_row = 8'h36;
                        3'd3: glyph_row = 8'h36;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h41: begin              // A
                    case (row)
                        3'd0: glyph_row = 8'h7E;
                        3'd1: glyph_row = 8'h11;
                        3'd2: glyph_row = 8'h11;
                        3'd3: glyph_row = 8'h11;
                        3'd4: glyph_row = 8'h7E;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h42: begin              // B
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h49;
                        3'd2: glyph_row = 8'h49;
                        3'd3: glyph_row = 8'h49;
                        3'd4: glyph_row = 8'h36;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h43: begin              // C
                    case (row)
                        3'd0: glyph_row = 8'h3E;
                        3'd1: glyph_row = 8'h41;
                        3'd2: glyph_row = 8'h41;
                        3'd3: glyph_row = 8'h41;
                        3'd4: glyph_row = 8'h22;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h44: begin              // D
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h41;
                        3'd2: glyph_row = 8'h41;
                        3'd3: glyph_row = 8'h22;
                        3'd4: glyph_row = 8'h1C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h45: begin              // E
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h49;
                        3'd2: glyph_row = 8'h49;
                        3'd3: glyph_row = 8'h49;
                        3'd4: glyph_row = 8'h41;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h46: begin              // F
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h09;
                        3'd2: glyph_row = 8'h09;
                        3'd3: glyph_row = 8'h01;
                        3'd4: glyph_row = 8'h01;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h48: begin              // H
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h08;
                        3'd2: glyph_row = 8'h08;
                        3'd3: glyph_row = 8'h08;
                        3'd4: glyph_row = 8'h7F;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h49: begin              // I
                    case (row)
                        3'd0: glyph_row = 8'h00;
                        3'd1: glyph_row = 8'h41;
                        3'd2: glyph_row = 8'h7F;
                        3'd3: glyph_row = 8'h41;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4B: begin              // K
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h08;
                        3'd2: glyph_row = 8'h14;
                        3'd3: glyph_row = 8'h22;
                        3'd4: glyph_row = 8'h41;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4C: begin              // L
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h40;
                        3'd2: glyph_row = 8'h40;
                        3'd3: glyph_row = 8'h40;
                        3'd4: glyph_row = 8'h40;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4D: begin              // M
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h02;
                        3'd2: glyph_row = 8'h0C;
                        3'd3: glyph_row = 8'h02;
                        3'd4: glyph_row = 8'h7F;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4E: begin              // N
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h04;
                        3'd2: glyph_row = 8'h08;
                        3'd3: glyph_row = 8'h10;
                        3'd4: glyph_row = 8'h7F;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4F: begin              // O
                    case (row)
                        3'd0: glyph_row = 8'h3E;
                        3'd1: glyph_row = 8'h41;
                        3'd2: glyph_row = 8'h41;
                        3'd3: glyph_row = 8'h41;
                        3'd4: glyph_row = 8'h3E;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h50: begin              // P
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h09;
                        3'd2: glyph_row = 8'h09;
                        3'd3: glyph_row = 8'h09;
                        3'd4: glyph_row = 8'h06;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h52: begin              // R
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h09;
                        3'd2: glyph_row = 8'h19;
                        3'd3: glyph_row = 8'h29;
                        3'd4: glyph_row = 8'h46;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h53: begin              // S
                    case (row)
                        3'd0: glyph_row = 8'h46;
                        3'd1: glyph_row = 8'h49;
                        3'd2: glyph_row = 8'h49;
                        3'd3: glyph_row = 8'h49;
                        3'd4: glyph_row = 8'h31;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h54: begin              // T
                    case (row)
                        3'd0: glyph_row = 8'h01;
                        3'd1: glyph_row = 8'h01;
                        3'd2: glyph_row = 8'h7F;
                        3'd3: glyph_row = 8'h01;
                        3'd4: glyph_row = 8'h01;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h55: begin              // U
                    case (row)
                        3'd0: glyph_row = 8'h3F;
                        3'd1: glyph_row = 8'h40;
                        3'd2: glyph_row = 8'h40;
                        3'd3: glyph_row = 8'h40;
                        3'd4: glyph_row = 8'h3F;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h57: begin              // W
                    case (row)
                        3'd0: glyph_row = 8'h7F;
                        3'd1: glyph_row = 8'h20;
                        3'd2: glyph_row = 8'h18;
                        3'd3: glyph_row = 8'h20;
                        3'd4: glyph_row = 8'h7F;
                        default: glyph_row = 8'h00;
                    endcase
                end
                default: glyph_row = 8'h00;
            endcase
        end
    endfunction

    function glyph_on;
        input [7:0] ch;
        input [2:0] row;
        input [2:0] col;
        reg [7:0] map;
        begin
            map = glyph_row(ch, row);
            glyph_on = map[7 - col];
        end
    endfunction

    function [7:0] ascii_digit;
        input [3:0] value;
        begin
            ascii_digit = {4'h3, value};
        end
    endfunction

    //=========================================================================
    // 根据行号和列号返回显示字符。
    //=========================================================================
    function [7:0] screen_char;
        input [2:0] line;
        input [4:0] col;
        begin
            screen_char = 8'h20;
            case (line)
                3'd0: begin // E104 SENSOR HUB
                    case (col)
                        5'd0:  screen_char = 8'h45;
                        5'd1:  screen_char = 8'h31;
                        5'd2:  screen_char = 8'h30;
                        5'd3:  screen_char = 8'h34;
                        5'd5:  screen_char = 8'h53;
                        5'd6:  screen_char = 8'h45;
                        5'd7:  screen_char = 8'h4E;
                        5'd8:  screen_char = 8'h53;
                        5'd9:  screen_char = 8'h4F;
                        5'd10: screen_char = 8'h52;
                        5'd12: screen_char = 8'h48;
                        5'd13: screen_char = 8'h55;
                        5'd14: screen_char = 8'h42;
                        default: screen_char = 8'h20;
                    endcase
                end
                3'd1: begin // TEMP/HUM: NORMAL / ALARM
                    case (col)
                        5'd0:  screen_char = 8'h54;
                        5'd1:  screen_char = 8'h45;
                        5'd2:  screen_char = 8'h4D;
                        5'd3:  screen_char = 8'h50;
                        5'd4:  screen_char = 8'h2F;
                        5'd5:  screen_char = 8'h48;
                        5'd6:  screen_char = 8'h55;
                        5'd7:  screen_char = 8'h4D;
                        5'd8:  screen_char = 8'h3A;
                        5'd10: screen_char = th_alarm ? 8'h41 : 8'h4E;
                        5'd11: screen_char = th_alarm ? 8'h4C : 8'h4F;
                        5'd12: screen_char = th_alarm ? 8'h41 : 8'h52;
                        5'd13: screen_char = th_alarm ? 8'h52 : 8'h4D;
                        5'd14: screen_char = th_alarm ? 8'h4D : 8'h41;
                        5'd15: screen_char = th_alarm ? 8'h20 : 8'h4C;
                        default: screen_char = 8'h20;
                    endcase
                end
                3'd2: begin // FLAME: SAFE / FIRE
                    case (col)
                        5'd0: screen_char = 8'h46;
                        5'd1: screen_char = 8'h4C;
                        5'd2: screen_char = 8'h41;
                        5'd3: screen_char = 8'h4D;
                        5'd4: screen_char = 8'h45;
                        5'd5: screen_char = 8'h3A;
                        5'd7: screen_char = flame_alarm ? 8'h46 : 8'h53;
                        5'd8: screen_char = flame_alarm ? 8'h49 : 8'h41;
                        5'd9: screen_char = flame_alarm ? 8'h52 : 8'h46;
                        5'd10: screen_char = flame_alarm ? 8'h45 : 8'h45;
                        default: screen_char = 8'h20;
                    endcase
                end
                3'd3: begin // RTC: 12:34:56
                    case (col)
                        5'd0: screen_char = 8'h52;
                        5'd1: screen_char = 8'h54;
                        5'd2: screen_char = 8'h43;
                        5'd3: screen_char = 8'h3A;
                        default: begin
                            if (rtc_valid) begin
                                case (col)
                                    5'd5:  screen_char = ascii_digit(rtc_hour_bcd[7:4]);
                                    5'd6:  screen_char = ascii_digit(rtc_hour_bcd[3:0]);
                                    5'd7:  screen_char = 8'h3A;
                                    5'd8:  screen_char = ascii_digit(rtc_min_bcd[7:4]);
                                    5'd9:  screen_char = ascii_digit(rtc_min_bcd[3:0]);
                                    5'd10: screen_char = 8'h3A;
                                    5'd11: screen_char = ascii_digit(rtc_sec_bcd[7:4]);
                                    5'd12: screen_char = ascii_digit(rtc_sec_bcd[3:0]);
                                    default: screen_char = 8'h20;
                                endcase
                            end
                            else begin
                                case (col)
                                    5'd5, 5'd6, 5'd8, 5'd9, 5'd11, 5'd12:
                                        screen_char = 8'h2D;
                                    5'd7, 5'd10:
                                        screen_char = 8'h3A;
                                    default: screen_char = 8'h20;
                                endcase
                            end
                        end
                    endcase
                end
                3'd4: begin // BLE: ONLINE / NO LNK
                    case (col)
                        5'd0: screen_char = 8'h42;
                        5'd1: screen_char = 8'h4C;
                        5'd2: screen_char = 8'h45;
                        5'd3: screen_char = 8'h3A;
                        5'd5: screen_char = ble_online ? 8'h4F : 8'h4E;
                        5'd6: screen_char = ble_online ? 8'h4E : 8'h4F;
                        5'd7: screen_char = ble_online ? 8'h4C : 8'h20;
                        5'd8: screen_char = ble_online ? 8'h49 : 8'h4C;
                        5'd9: screen_char = ble_online ? 8'h4E : 8'h4E;
                        5'd10: screen_char = ble_online ? 8'h45 : 8'h4B;
                        default: screen_char = 8'h20;
                    endcase
                end
                default: screen_char = 8'h20;
            endcase
        end
    endfunction

    //=========================================================================
    // 画面分三级流水生成：
    //   1. base_color 只处理背景、标题栏、分割线和底部状态块。
    //   2. text_prep 只把当前像素映射到字符格，不查询字库。
    //   3. text_pixel 查询字符和字库，返回是否覆盖及覆盖颜色。
    // 这样可以把长组合逻辑切成三段，避免 50 MHz 下的建立时间违例。
    //=========================================================================
    function [15:0] base_color;
        input [8:0] px;
        input [8:0] py;
        begin
            base_color = 16'h0861;          // 深蓝背景

            // 顶部标题栏和分割线。
            if (py < 9'd32) base_color = 16'h0210;
            if ((py == 9'd32) || (py == 9'd48) || (py == 9'd76) ||
                (py == 9'd104) || (py == 9'd132) || (py == 9'd160))
                base_color = 16'h4A69;

            // 底部状态块：绿色正常、黄色温湿度报警、红色火焰报警。
            if ((py >= 9'd232) && (py < 9'd280)) begin
                if ((px >= 9'd12) && (px < 9'd72))
                    base_color = th_alarm ? 16'hFFE0 : 16'h07E0;
                else if ((px >= 9'd84) && (px < 9'd144))
                    base_color = flame_alarm ? 16'hF800 : 16'h07E0;
                else if ((px >= 9'd156) && (px < 9'd228))
                    base_color = ble_online ? 16'h07E0 : 16'h8410;
            end
        end
    endfunction

    // 返回 {hit, line[2:0], col[4:0], row[2:0], bit_col[2:0]}。
    function [14:0] text_prep;
        input [8:0] px;
        input [8:0] py;
        reg [8:0] tx;
        reg       hit;
        reg [2:0] line;
        begin
            tx   = 9'd0;
            hit  = 1'b0;
            line = 3'd0;

            if ((py >= 9'd8) && (py < 9'd16)) begin
                line = 3'd0;
                tx   = px - 9'd8;
                hit  = (px >= 9'd8) && (tx < 9'd120);
            end
            else if ((py >= 9'd56) && (py < 9'd64)) begin
                line = 3'd1;
                tx   = px - 9'd8;
                hit  = (px >= 9'd8) && (tx < 9'd128);
            end
            else if ((py >= 9'd84) && (py < 9'd92)) begin
                line = 3'd2;
                tx   = px - 9'd8;
                hit  = (px >= 9'd8) && (tx < 9'd88);
            end
            else if ((py >= 9'd112) && (py < 9'd120)) begin
                line = 3'd3;
                tx   = px - 9'd8;
                hit  = (px >= 9'd8) && (tx < 9'd104);
            end
            else if ((py >= 9'd140) && (py < 9'd148)) begin
                line = 3'd4;
                tx   = px - 9'd8;
                hit  = (px >= 9'd8) && (tx < 9'd88);
            end

            text_prep = {hit, line, tx[7:3], py[2:0], tx[2:0]};
        end
    endfunction

    // 返回 {cover, color[15:0]}。cover 为 0 表示该像素保持背景色。
    function [16:0] text_pixel;
        input [14:0] prep;
        reg [7:0] ch;
        reg [2:0] line;
        reg [4:0] col;
        begin
            text_pixel = 17'h00000;
            line       = prep[13:11];
            col        = prep[10:6];

            if (prep[14]) begin
                ch = screen_char(line, col);
                if (glyph_on(ch, prep[5:3], prep[2:0])) begin
                    text_pixel[16] = 1'b1;
                    case (line)
                        3'd0: text_pixel[15:0] = 16'h07FF;  // 标题青色
                        3'd1: begin
                            if ((col >= 5'd10) && th_alarm)
                                text_pixel[15:0] = 16'hF800;
                            else if (col >= 5'd10)
                                text_pixel[15:0] = 16'h07E0;
                            else
                                text_pixel[15:0] = 16'hFFFF;
                        end
                        3'd2: begin
                            if ((col >= 5'd7) && flame_alarm)
                                text_pixel[15:0] = 16'hF800;
                            else if (col >= 5'd7)
                                text_pixel[15:0] = 16'h07E0;
                            else
                                text_pixel[15:0] = 16'hFFFF;
                        end
                        3'd3: text_pixel[15:0] = 16'hFFE0;
                        3'd4: begin
                            if ((col >= 5'd5) && ble_online)
                                text_pixel[15:0] = 16'h07E0;
                            else if (col >= 5'd5)
                                text_pixel[15:0] = 16'hF800;
                            else
                                text_pixel[15:0] = 16'hFFFF;
                        end
                        default: text_pixel[15:0] = 16'hFFFF;
                    endcase
                end
            end
        end
    endfunction

    //=========================================================================
    // ILI9341 常用初始化序列。idx 高位为 1 表示命令，为 0 表示数据。
    //=========================================================================
    function [8:0] init_word;
        input [6:0] idx;
        begin
            case (idx)
                6'd0:  init_word = {1'b1, 8'h01}; // SWRESET
                6'd1:  init_word = {1'b1, 8'h11}; // SLPOUT
                6'd2:  init_word = {1'b1, 8'hEF};
                6'd3:  init_word = {1'b0, 8'h03};
                6'd4:  init_word = {1'b0, 8'h80};
                6'd5:  init_word = {1'b0, 8'h02};
                6'd6:  init_word = {1'b1, 8'hCF};
                6'd7:  init_word = {1'b0, 8'h00};
                6'd8:  init_word = {1'b0, 8'hC1};
                6'd9:  init_word = {1'b0, 8'h30};
                6'd10: init_word = {1'b1, 8'hED};
                6'd11: init_word = {1'b0, 8'h64};
                6'd12: init_word = {1'b0, 8'h03};
                6'd13: init_word = {1'b0, 8'h12};
                6'd14: init_word = {1'b0, 8'h81};
                6'd15: init_word = {1'b1, 8'hE8};
                6'd16: init_word = {1'b0, 8'h85};
                6'd17: init_word = {1'b0, 8'h00};
                6'd18: init_word = {1'b0, 8'h78};
                6'd19: init_word = {1'b1, 8'hCB};
                6'd20: init_word = {1'b0, 8'h39};
                6'd21: init_word = {1'b0, 8'h2C};
                6'd22: init_word = {1'b0, 8'h00};
                6'd23: init_word = {1'b0, 8'h34};
                6'd24: init_word = {1'b0, 8'h02};
                6'd25: init_word = {1'b1, 8'hF7};
                6'd26: init_word = {1'b0, 8'h20};
                6'd27: init_word = {1'b1, 8'hEA};
                6'd28: init_word = {1'b0, 8'h00};
                6'd29: init_word = {1'b0, 8'h00};
                6'd30: init_word = {1'b1, 8'hC0};
                6'd31: init_word = {1'b0, 8'h23};
                6'd32: init_word = {1'b1, 8'hC1};
                6'd33: init_word = {1'b0, 8'h10};
                6'd34: init_word = {1'b1, 8'hC5};
                6'd35: init_word = {1'b0, 8'h3E};
                6'd36: init_word = {1'b0, 8'h28};
                6'd37: init_word = {1'b1, 8'hC7};
                6'd38: init_word = {1'b0, 8'h86};
                6'd39: init_word = {1'b1, 8'h36};
                6'd40: init_word = {1'b0, 8'h48};
                6'd41: init_word = {1'b1, 8'h3A};
                6'd42: init_word = {1'b0, 8'h55};
                6'd43: init_word = {1'b1, 8'hB1};
                6'd44: init_word = {1'b0, 8'h00};
                6'd45: init_word = {1'b0, 8'h18};
                6'd46: init_word = {1'b1, 8'hB6};
                6'd47: init_word = {1'b0, 8'h08};
                6'd48: init_word = {1'b0, 8'h82};
                6'd49: init_word = {1'b0, 8'h27};
                6'd50: init_word = {1'b1, 8'hF2};
                6'd51: init_word = {1'b0, 8'h00};
                6'd52: init_word = {1'b1, 8'h26};
                6'd53: init_word = {1'b0, 8'h01};
                6'd54: init_word = {1'b1, 8'hE0};
                6'd55: init_word = {1'b0, 8'h0F};
                6'd56: init_word = {1'b0, 8'h31};
                6'd57: init_word = {1'b0, 8'h2B};
                6'd58: init_word = {1'b0, 8'h0C};
                6'd59: init_word = {1'b0, 8'h0E};
                6'd60: init_word = {1'b0, 8'h08};
                6'd61: init_word = {1'b0, 8'h4E};
                6'd62: init_word = {1'b0, 8'hF1};
                6'd63: init_word = {1'b0, 8'h37};
                // 注意：64 是 6 位最大值，接下来使用默认分支补齐 gamma 数据。
                default: begin
                    // 索引 64~85 的 Gamma 数据，以及最后的 DISPON 命令。
                    if (idx == 7'd64)      init_word = {1'b0, 8'h07};
                    else if (idx == 7'd65) init_word = {1'b0, 8'h10};
                    else if (idx == 7'd66) init_word = {1'b0, 8'h03};
                    else if (idx == 7'd67) init_word = {1'b0, 8'h0E};
                    else if (idx == 7'd68) init_word = {1'b0, 8'h09};
                    else if (idx == 7'd69) init_word = {1'b0, 8'h00};
                    else if (idx == 7'd70) init_word = {1'b1, 8'hE1};
                    else if (idx == 7'd71) init_word = {1'b0, 8'h00};
                    else if (idx == 7'd72) init_word = {1'b0, 8'h0E};
                    else if (idx == 7'd73) init_word = {1'b0, 8'h14};
                    else if (idx == 7'd74) init_word = {1'b0, 8'h03};
                    else if (idx == 7'd75) init_word = {1'b0, 8'h11};
                    else if (idx == 7'd76) init_word = {1'b0, 8'h07};
                    else if (idx == 7'd77) init_word = {1'b0, 8'h31};
                    else if (idx == 7'd78) init_word = {1'b0, 8'hC1};
                    else if (idx == 7'd79) init_word = {1'b0, 8'h48};
                    else if (idx == 7'd80) init_word = {1'b0, 8'h08};
                    else if (idx == 7'd81) init_word = {1'b0, 8'h0F};
                    else if (idx == 7'd82) init_word = {1'b0, 8'h0C};
                    else if (idx == 7'd83) init_word = {1'b0, 8'h31};
                    else if (idx == 7'd84) init_word = {1'b0, 8'h36};
                    else if (idx == 7'd85) init_word = {1'b0, 8'h0F};
                    else if (idx == 7'd86) init_word = {1'b1, 8'h11}; // SLPOUT
                    else                    init_word = {1'b1, 8'h29}; // DISPON
                end
            endcase
        end
    endfunction

    // 先锁存当前 ROM 字，避免对函数调用结果直接做位选。
    wire [8:0] init_word_now = init_word(init_idx);

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
        end
        else if (!spi_busy) begin
            spi_scl     <= 1'b0;
            spi_div_cnt <= 16'd0;
            spi_bit_cnt <= 3'd0;
            if (spi_start) begin
                spi_busy  <= 1'b1;
                spi_shift <= spi_data_hold;
                spi_sda   <= spi_data_hold[7];
            end
        end
        else begin
            if (spi_div_cnt == SPI_DIV - 1'b1) begin
                spi_div_cnt <= 16'd0;

                if (!spi_scl) begin
                    spi_scl <= 1'b1;       // 上升沿，器件采样 MOSI
                end
                else begin
                    spi_scl <= 1'b0;
                    if (spi_bit_cnt == 3'd7) begin
                        spi_busy <= 1'b0;
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
    // 显示主状态机。
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            delay_cnt     <= 23'd0;
            refresh_cnt   <= 27'd0;
            init_idx      <= 7'd0;
            setup_stage   <= 3'd0;
            x_pos         <= 9'd0;
            y_pos         <= 9'd0;
            cur_word      <= 9'h100;
            pixel_color   <= 16'h0000;
            base_color_reg <= 16'h0861;
            text_prep_reg  <= 15'd0;
            text_pixel_reg <= 17'd0;
            spi_start     <= 1'b0;
            spi_seen      <= 1'b0;
            spi_data_hold <= 8'h00;
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

                // 硬件复位至少保持 10 ms。
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

                ST_RESET_HIGH: begin
                    if (delay_cnt == 23'd5000) begin
                        delay_cnt <= 23'd0;
                        init_idx  <= 7'd0;
                        spi_seen  <= 1'b0;
                        state     <= ST_INIT;
                    end
                    else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // 按 ROM 顺序发送初始化命令和数据。
                ST_INIT: begin
                    if (!spi_busy && !spi_seen) begin
                        cur_word      <= init_word_now;
                        spi_data_hold <= init_word_now[7:0];
                        tft_dc        <= init_word_now[8];
                        spi_start     <= 1'b1;
                        spi_seen      <= 1'b1;
                    end
                    else if (spi_seen && !spi_busy) begin
                        spi_seen <= 1'b0;

                        if (cur_word[8] &&
                            ((cur_word[7:0] == 8'h01) ||
                             (cur_word[7:0] == 8'h11))) begin
                            delay_cnt <= 23'd0;
                            state     <= ST_INIT_WAIT;
                        end
                        else if (init_idx == INIT_LAST) begin
                            init_idx    <= 7'd0;
                            setup_stage <= 3'd0;
                            state       <= ST_SET_X;
                        end
                        else begin
                            init_idx <= init_idx + 1'b1;
                        end
                    end
                end

                ST_INIT_WAIT: begin
                    if (delay_cnt == INIT_DELAY_CYCLES - 1'b1) begin
                        delay_cnt <= 23'd0;
                        init_idx  <= init_idx + 1'b1;
                        state     <= ST_INIT;
                    end
                    else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // 0x2A：设置列地址 0~239。
                ST_SET_X: begin
                    if (!spi_busy && !spi_seen) begin
                        if (setup_stage == 3'd0) begin
                            tft_dc        <= 1'b0;
                            spi_data_hold <= 8'h2A;
                        end
                        else begin
                            tft_dc <= 1'b1;
                            case (setup_stage)
                                3'd1: spi_data_hold <= 8'h00;
                                3'd2: spi_data_hold <= 8'h00;
                                3'd3: spi_data_hold <= 8'h00;
                                default: spi_data_hold <= X_LAST[7:0];
                            endcase
                        end
                        spi_start <= 1'b1;
                        spi_seen  <= 1'b1;
                    end
                    else if (spi_seen && !spi_busy) begin
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

                // 0x2B：设置页地址 0~319。
                ST_SET_Y: begin
                    if (!spi_busy && !spi_seen) begin
                        if (setup_stage == 3'd0) begin
                            tft_dc        <= 1'b0;
                            spi_data_hold <= 8'h2B;
                        end
                        else begin
                            tft_dc <= 1'b1;
                            case (setup_stage)
                                3'd1: spi_data_hold <= 8'h00;
                                3'd2: spi_data_hold <= 8'h00;
                                3'd3: spi_data_hold <= 8'h01; // Y_LAST[15:8]
                                default: spi_data_hold <= Y_LAST[7:0];
                            endcase
                        end
                        spi_start <= 1'b1;
                        spi_seen  <= 1'b1;
                    end
                    else if (spi_seen && !spi_busy) begin
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
                    if (!spi_busy && !spi_seen) begin
                        tft_dc        <= 1'b0;
                        spi_data_hold <= 8'h2C;
                        spi_start     <= 1'b1;
                        spi_seen      <= 1'b1;
                    end
                    else if (spi_seen && !spi_busy) begin
                        spi_seen <= 1'b0;
                        x_pos    <= 9'd0;
                        y_pos    <= 9'd0;
                        state    <= ST_PIXEL_CALC;
                    end
                end

                // 第 1 级流水：计算背景色并缓存当前像素对应的字符位置。
                ST_PIXEL_CALC: begin
                    base_color_reg <= base_color(x_pos, y_pos);
                    text_prep_reg  <= text_prep(x_pos, y_pos);
                    state          <= ST_PIXEL_TEXT;
                end

                // 第 2 级流水：查询字符和字库，计算前景色。
                ST_PIXEL_TEXT: begin
                    text_pixel_reg <= text_pixel(text_prep_reg);
                    state          <= ST_PIXEL_MIX;
                end

                // 第 3 级流水：合成最终 RGB565 颜色。
                ST_PIXEL_MIX: begin
                    if (text_pixel_reg[16])
                        pixel_color <= text_pixel_reg[15:0];
                    else
                        pixel_color <= base_color_reg;
                    state <= ST_PIXEL_HI;
                end

                // 每个像素发送 RGB565 高字节和低字节。
                ST_PIXEL_HI: begin
                    if (!spi_busy && !spi_seen) begin
                        tft_dc        <= 1'b1;
                        spi_data_hold <= pixel_color[15:8];
                        spi_start     <= 1'b1;
                        spi_seen      <= 1'b1;
                    end
                    else if (spi_seen && !spi_busy) begin
                        spi_seen <= 1'b0;
                        state    <= ST_PIXEL_LO;
                    end
                end

                ST_PIXEL_LO: begin
                    if (!spi_busy && !spi_seen) begin
                        tft_dc        <= 1'b1;
                        spi_data_hold <= pixel_color[7:0];
                        spi_start     <= 1'b1;
                        spi_seen      <= 1'b1;
                    end
                    else if (spi_seen && !spi_busy) begin
                        spi_seen <= 1'b0;
                        if (x_pos == X_LAST) begin
                            x_pos <= 9'd0;
                            if (y_pos == Y_LAST) begin
                                refresh_cnt <= 27'd0;
                                state       <= ST_REFRESH;
                            end
                            else begin
                                y_pos <= y_pos + 1'b1;
                                state <= ST_PIXEL_CALC;
                            end
                        end
                        else begin
                            x_pos <= x_pos + 1'b1;
                            state <= ST_PIXEL_CALC;
                        end
                    end
                end

                // 周期性重绘，让 RTC 和报警状态保持最新。
                ST_REFRESH: begin
                    if (refresh_cnt == REFRESH_CYCLES - 1'b1) begin
                        refresh_cnt <= 27'd0;
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
