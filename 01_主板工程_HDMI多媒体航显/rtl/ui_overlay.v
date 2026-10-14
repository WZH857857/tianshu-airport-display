`timescale 1ns / 1ps
//=============================================================================
// ui_overlay.v -字幕、OSD 和音频可视化叠加
//
// This block is deliberately placed after the SDRAM frame reader.  It does
// not change the FAT/BMP loader or the cache manager:
//
//   top-left      : ANLU TIS logo + IMG n/m status + AUTO/MANUAL mode
//   top-right     : airport local HH:MM:SS readout
//   lower band    : slow airport-operation ticker (parallax layer)
//   bottom band   : fast flight-information ticker with alpha gradient
//   lower right   : audio waveform and eight band energy bars from the
//                   HDMI I2S sample stream
//
// The text font is an 8x8 bitmap in logic, so no external character ROM or
// processor is needed.  The audio history is a 128-sample dual-port RAM.
//=============================================================================
module ui_overlay (
    input  wire        pixel_clk,
    input  wire        rst,
    input  wire        frame_tick,
    input  wire        de,
    input  wire [9:0]  px,
    input  wire [9:0]  py,
    input  wire [23:0] base_rgb,

    input  wire        ui_enable,
    input  wire [2:0]  img_slot,
    input  wire [7:0]  img_count,
    input  wire [11:0] img_width,
    input  wire [11:0] img_height,
    input  wire        img_size_valid,
    input  wire [2:0]  user_bright,
    input  wire [2:0]  user_contrast,
    input  wire [1:0]  ui_mode,
    input  wire        param_flash,
    input  wire [1:0]  param_sel,       // 0 brightness, 1 contrast, 2 layout
    input  wire        auto_mode,       // 1 = automatic rotation
    input  wire        emg_fire,        // 1 = flame alarm from the host link
    input  wire        emg_pic,         // 1 = a TF card placard is live

    input  wire        logo_enable,
    input  wire        subtitle_enable,
    input  wire        timestamp_enable,
    input  wire        parallax_enable,
    input  wire        osd_enable,
    input  wire        wave_enable,

    // text pushed in over the serial port.  The writer lives in the 50 MHz
    // system clock domain and hands one byte over at a time through the
    // toggle below; this block keeps the pixel clock copy.
    input  wire        host_text_tog,
    input  wire [4:0]  host_text_addr,
    input  wire [7:0]  host_text_data,
    input  wire        host_text_commit_tog,
    input  wire        host_text_clear_tog,
    input  wire        host_link,

    // wall clock from the serial port.  The host sends hh/mm/ss as BCD and
    // the DS1307 block sends the same fields with its own update toggle.  The
    // host value takes over once received; the RTC is only a never-seen-host
    // fallback.
    input  wire [7:0]  host_time_hh,
    input  wire [7:0]  host_time_mm,
    input  wire [7:0]  host_time_ss,
    input  wire        host_time_tog,

    input  wire [7:0]  rtc_time_hh,
    input  wire [7:0]  rtc_time_mm,
    input  wire [7:0]  rtc_time_ss,
    input  wire        rtc_time_tog,

    input  wire        audio_rst,
    input  wire        audio_valid,
    input  wire [23:0] audio_left,

    // flight information board, written by the host over the serial link
    input  wire        board_enable,
    input  wire        board_wr_tog,
    input  wire [7:0]  board_wr_addr,
    input  wire [7:0]  board_wr_data,

    output reg  [23:0] rgb,
    output reg         hit
);

    localparam [9:0] TOP_X0    = 10'd12;
    localparam [9:0] TOP_Y0    = 10'd12;
    localparam [9:0] TOP_W     = 10'd224;
    localparam [9:0] TOP_H     = 10'd48;

    localparam [9:0] PARAM_X0  = 10'd456;
    localparam [9:0] PARAM_Y0  = 10'd12;
    localparam [9:0] PARAM_W   = 10'd168;
    localparam [9:0] PARAM_H   = 10'd32;

    localparam [9:0] TIME_X0   = 10'd456;
    localparam [9:0] TIME_Y0   = 10'd12;
    localparam [9:0] TIME_W    = 10'd168;
    localparam [9:0] TIME_H    = 10'd32;

    localparam [9:0] TICK_Y0   = 10'd432;
    localparam [9:0] TICK_W    = 10'd640;
    localparam [9:0] TICK_H    = 10'd8;

    localparam [9:0] SUB_X0    = 10'd0;
    localparam [9:0] SUB_Y0    = 10'd448;
    localparam [9:0] SUB_W     = 10'd640;
    localparam [9:0] SUB_H     = 10'd32;

    localparam [9:0] WAVE_X0   = 10'd384;
    localparam [9:0] WAVE_X1   = 10'd639;
    localparam [9:0] WAVE_Y0   = 10'd320;
    localparam [9:0] WAVE_Y1   = 10'd447;
    localparam [9:0] WAVE_GRAPH_Y1 = 10'd383;  // upper 64 rows: waveform
    localparam [9:0] SPEC_Y0   = 10'd384;      // lower 64 rows: spectrum

    // The glyphs are drawn at 2x, so one 8x8 character occupies 16x16
    // pixels.  The panels are 32 pixels tall, so the text is centred by
    // starting eight pixels below the panel top.  Without this offset the
    // low three bits of the row address would wrap and every character
    // would be painted twice inside the band.
    localparam [9:0] TOP_TEXT_Y   = TOP_Y0 + 10'd8;
    localparam [9:0] TOP_RES_TEXT_Y = TOP_Y0 + 10'd26;
    localparam [9:0] PARAM_TEXT_Y = PARAM_Y0 + 10'd8;
    localparam [9:0] TIME_TEXT_Y  = TIME_Y0 + 10'd8;
    localparam [9:0] SUB_TEXT_OFF = 10'd8;

    // Two-flop synchronisers for the slow control inputs from sys_clk.
    reg       ui_enable_s1, ui_enable_s2;
    reg [2:0] ui_mode_s1, ui_mode_s2;
    reg [2:0] bright_s1, bright_s2;
    reg [2:0] contrast_s1, contrast_s2;
    reg [7:0] img_count_s1, img_count_s2;
    reg       pflash_s1, pflash_s2;
    reg [1:0] psel_s1, psel_s2;
    reg       auto_s1, auto_s2;

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            ui_enable_s1 <= 1'b0;
            ui_enable_s2 <= 1'b0;
            ui_mode_s1 <= 3'd0;
            ui_mode_s2 <= 3'd0;
            bright_s1  <= 3'd7;
            bright_s2  <= 3'd7;
            contrast_s1 <= 3'd3;
            contrast_s2 <= 3'd3;
            img_count_s1 <= 8'd0;
            img_count_s2 <= 8'd0;
            pflash_s1  <= 1'b0;
            pflash_s2  <= 1'b0;
            psel_s1    <= 2'd0;
            psel_s2    <= 2'd0;
            auto_s1    <= 1'b0;
            auto_s2    <= 1'b0;
        end else begin
            ui_enable_s1 <= ui_enable;
            ui_enable_s2 <= ui_enable_s1;
            ui_mode_s1 <= {1'b0, ui_mode};
            ui_mode_s2 <= ui_mode_s1;
            bright_s1  <= user_bright;
            bright_s2  <= bright_s1;
            contrast_s1 <= user_contrast;
            contrast_s2 <= contrast_s1;
            img_count_s1 <= img_count;
            img_count_s2 <= img_count_s1;
            pflash_s1  <= param_flash;
            pflash_s2  <= pflash_s1;
            psel_s1    <= param_sel;
            psel_s2    <= psel_s1;
            auto_s1    <= auto_mode;
            auto_s2    <= auto_s1;
        end
    end

    //------------------------------------------------------------------
    // Source-resolution readout.  The BMP size arrives as a 12 bit binary
    // value.  It changes at most once per cached slot, so a small repeated-
    // subtraction converter is enough and avoids placing a large divider in
    // the pixel path.  Both dimensions share the same arithmetic unit, and
    // the digits are only converted when a new slot reports its size.
    //------------------------------------------------------------------
    localparam [1:0] RES_IDLE  = 2'd0,
                     RES_DIGIT = 2'd1;

    reg  [1:0]  res_state;
    reg         res_dim;
    reg  [1:0]  res_phase;
    reg  [11:0] res_rem;
    reg  [11:0] res_place;
    reg  [3:0]  res_digit;
    reg  [15:0] res_accum;
    reg  [15:0] res_width_bcd;
    reg  [15:0] res_height_bcd;
    reg         res_bcd_valid;
    reg  [11:0] res_width_d;
    reg  [11:0] res_height_d;
    reg         res_valid_d;

    // The size comparison is written inline inside the state machine instead of
    // through a continuous assignment.  A derived wire can still be settling in
    // the very time step the source reports a new size; the tracker below would
    // then copy the new width into res_width_d while the stale comparator still
    // read "equal", and the BCD converter would never run again for that image.

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            res_state      <= RES_IDLE;
            res_dim        <= 1'b0;
            res_phase      <= 2'd0;
            res_rem        <= 12'd0;
            res_place      <= 12'd1000;
            res_digit      <= 4'd0;
            res_accum      <= 16'd0;
            res_width_bcd  <= 16'd0;
            res_height_bcd <= 16'd0;
            res_bcd_valid  <= 1'b0;
            res_width_d    <= 12'd0;
            res_height_d   <= 12'd0;
            res_valid_d    <= 1'b0;
        end else begin
            case (res_state)
                RES_IDLE: begin
                    if (((img_width != res_width_d) ||
                         (img_height != res_height_d) ||
                         (img_size_valid != res_valid_d)) &&
                        img_size_valid) begin
                        // Freeze the values used for this conversion.  A
                        // slot change during the short conversion is picked
                        // up when the state machine returns here.
                        res_width_d  <= img_width;
                        res_height_d <= img_height;
                        res_valid_d  <= 1'b1;
                        res_rem      <= img_width;
                        res_place    <= 12'd1000;
                        res_digit    <= 4'd0;
                        res_accum    <= 16'd0;
                        res_phase    <= 2'd0;
                        res_dim      <= 1'b0;
                        res_bcd_valid<= 1'b0;
                        res_state    <= RES_DIGIT;
                    end else begin
                        res_width_d  <= img_width;
                        res_height_d <= img_height;
                        res_valid_d  <= img_size_valid;
                    end
                end

                RES_DIGIT: begin
                    if (res_rem >= res_place) begin
                        res_rem   <= res_rem - res_place;
                        res_digit <= res_digit + 4'd1;
                    end else if (res_phase == 2'd3) begin
                        if (!res_dim) begin
                            res_width_bcd <= {res_accum[11:0], res_digit};
                            res_rem       <= img_height;
                            res_place     <= 12'd1000;
                            res_digit     <= 4'd0;
                            res_accum     <= 16'd0;
                            res_phase     <= 2'd0;
                            res_dim       <= 1'b1;
                        end else begin
                            res_height_bcd <= {res_accum[11:0], res_digit};
                            res_bcd_valid  <= 1'b1;
                            res_state      <= RES_IDLE;
                        end
                    end else begin
                        res_accum <= {res_accum[11:0], res_digit};
                        case (res_phase)
                            2'd0: res_place <= 12'd100;
                            2'd1: res_place <= 12'd10;
                            default: res_place <= 12'd1;
                        endcase
                        res_digit <= 4'd0;
                        res_phase <= res_phase + 2'd1;
                    end
                end

                default: res_state <= RES_IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------
    // Host text from the serial link.  The writer runs at 50 MHz and holds
    // the address and data stable for sixteen clocks before it flips the
    // toggle, so synchronising the toggle (and, behind it, the address and
    // data) moves exactly one byte per handshake.  The local copy lives in
    // this clock domain, which is what the pixel path reads.
    //------------------------------------------------------------------
    reg [7:0] host_char [0:31];
    reg [5:0] host_len;
    reg [5:0] host_max;                 // length of the staged message
    reg       htog_s1, htog_s2, htog_s3;
    reg       hcmt_s1, hcmt_s2, hcmt_s3;
    reg       hclr_s1, hclr_s2, hclr_s3;
    reg       hlink_s1, hlink_s2;
    reg [4:0] haddr_s1, haddr_s2;
    reg [7:0] hdata_s1, hdata_s2;

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            htog_s1  <= 1'b0;
            htog_s2  <= 1'b0;
            htog_s3  <= 1'b0;
            hcmt_s1  <= 1'b0;
            hcmt_s2  <= 1'b0;
            hcmt_s3  <= 1'b0;
            hclr_s1  <= 1'b0;
            hclr_s2  <= 1'b0;
            hclr_s3  <= 1'b0;
            hlink_s1 <= 1'b0;
            hlink_s2 <= 1'b0;
            haddr_s1 <= 5'd0;
            haddr_s2 <= 5'd0;
            hdata_s1 <= 8'd0;
            hdata_s2 <= 8'd0;
            host_len <= 6'd0;
            host_max <= 6'd0;
        end else begin
            htog_s1 <= host_text_tog;
            htog_s2 <= htog_s1;
            htog_s3 <= htog_s2;
            hcmt_s1 <= host_text_commit_tog;
            hcmt_s2 <= hcmt_s1;
            hcmt_s3 <= hcmt_s2;
            hclr_s1 <= host_text_clear_tog;
            hclr_s2 <= hclr_s1;
            hclr_s3 <= hclr_s2;
            hlink_s1 <= host_link;
            hlink_s2 <= hlink_s1;
            haddr_s1 <= host_text_addr;
            haddr_s2 <= haddr_s1;
            hdata_s1 <= host_text_data;
            hdata_s2 <= hdata_s1;

            // Bytes stream in ahead of the checksum, so they are only staged.
            // The length that the ticker reads moves on the commit edge, which
            // the writer raises once the whole frame has been verified.
            if (hcmt_s2 ^ hcmt_s3) begin
                host_len <= host_max;
            end else if (hclr_s2 ^ hclr_s3) begin
                host_max <= 6'd0;
                host_len <= 6'd0;
            end

            if (htog_s2 ^ htog_s3) begin
                host_char[haddr_s2] <= hdata_s2;
                if (({1'b0, haddr_s2} + 6'd1) > host_max)
                    host_max <= {1'b0, haddr_s2} + 6'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // 8x8 bitmap font.  Bit seven is the leftmost pixel.
    //------------------------------------------------------------------
    function [7:0] glyph_row;
        input [7:0] ch;
        input [2:0] row;
        reg [7:0] c;
        begin
            // Text pushed in from the PC may be typed in lower case; the
            // font only carries the upper case glyphs, so fold it here.
            if ((ch >= 8'h61) && (ch <= 8'h7A))
                c = ch - 8'h20;
            else
                c = ch;
            glyph_row = 8'h00;
            case (c)
                8'h20: glyph_row = 8'h00;                       // space
                8'h2D: begin                                    // -
                    case (row)
                        3'd4: glyph_row = 8'h18;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h2E: begin                                    // .
                    case (row)
                        3'd6: glyph_row = 8'h18;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h2F: begin                                    // /
                    case (row)
                        3'd0: glyph_row = 8'h06;
                        3'd1: glyph_row = 8'h0C;
                        3'd2: glyph_row = 8'h18;
                        3'd3: glyph_row = 8'h30;
                        3'd4: glyph_row = 8'h60;
                        3'd5: glyph_row = 8'hC0;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h30: begin                                    // 0
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hCE;
                        3'd3: glyph_row = 8'hD6;
                        3'd4: glyph_row = 8'hE6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h31: begin                                    // 1
                    case (row)
                        3'd0: glyph_row = 8'h10;
                        3'd1: glyph_row = 8'h70;
                        3'd2: glyph_row = 8'h10;
                        3'd3: glyph_row = 8'h10;
                        3'd4: glyph_row = 8'h10;
                        3'd5: glyph_row = 8'h10;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h32: begin                                    // 2
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'h06;
                        3'd3: glyph_row = 8'h1C;
                        3'd4: glyph_row = 8'h30;
                        3'd5: glyph_row = 8'h60;
                        3'd6: glyph_row = 8'hFE;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h33: begin                                    // 3
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'h06;
                        3'd3: glyph_row = 8'h3C;
                        3'd4: glyph_row = 8'h06;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h34: begin                                    // 4
                    case (row)
                        3'd0: glyph_row = 8'h0C;
                        3'd1: glyph_row = 8'h1C;
                        3'd2: glyph_row = 8'h3C;
                        3'd3: glyph_row = 8'h6C;
                        3'd4: glyph_row = 8'hFE;
                        3'd5: glyph_row = 8'h0C;
                        3'd6: glyph_row = 8'h0C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h35: begin                                    // 5
                    case (row)
                        3'd0: glyph_row = 8'hFE;
                        3'd1: glyph_row = 8'hC0;
                        3'd2: glyph_row = 8'hFC;
                        3'd3: glyph_row = 8'h06;
                        3'd4: glyph_row = 8'h06;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h36: begin                                    // 6
                    case (row)
                        3'd0: glyph_row = 8'h3C;
                        3'd1: glyph_row = 8'h60;
                        3'd2: glyph_row = 8'hC0;
                        3'd3: glyph_row = 8'hFC;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h37: begin                                    // 7
                    case (row)
                        3'd0: glyph_row = 8'hFE;
                        3'd1: glyph_row = 8'h06;
                        3'd2: glyph_row = 8'h0C;
                        3'd3: glyph_row = 8'h18;
                        3'd4: glyph_row = 8'h30;
                        3'd5: glyph_row = 8'h30;
                        3'd6: glyph_row = 8'h30;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h38: begin                                    // 8
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'h7C;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h39: begin                                    // 9
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'h7E;
                        3'd4: glyph_row = 8'h06;
                        3'd5: glyph_row = 8'h0C;
                        3'd6: glyph_row = 8'h78;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h3A: begin                                    // :
                    case (row)
                        3'd2, 3'd5: glyph_row = 8'h18;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h41: begin                                    // A
                    case (row)
                        3'd0: glyph_row = 8'h38;
                        3'd1: glyph_row = 8'h6C;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hC6;
                        3'd4: glyph_row = 8'hFE;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h42: begin                                    // B
                    case (row)
                        3'd0: glyph_row = 8'hFC;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hFC;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'hFC;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h43: begin                                    // C
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC0;
                        3'd3: glyph_row = 8'hC0;
                        3'd4: glyph_row = 8'hC0;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h44: begin                                    // D
                    case (row)
                        3'd0: glyph_row = 8'hFC;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hC6;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'hFC;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h45: begin                                    // E
                    case (row)
                        3'd0: glyph_row = 8'hFE;
                        3'd1: glyph_row = 8'hC0;
                        3'd2: glyph_row = 8'hC0;
                        3'd3: glyph_row = 8'hFC;
                        3'd4: glyph_row = 8'hC0;
                        3'd5: glyph_row = 8'hC0;
                        3'd6: glyph_row = 8'hFE;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h46: begin                                    // F
                    case (row)
                        3'd0: glyph_row = 8'hFE;
                        3'd1: glyph_row = 8'hC0;
                        3'd2: glyph_row = 8'hC0;
                        3'd3: glyph_row = 8'hFC;
                        3'd4: glyph_row = 8'hC0;
                        3'd5: glyph_row = 8'hC0;
                        3'd6: glyph_row = 8'hC0;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h47: begin                                    // G
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC0;
                        3'd3: glyph_row = 8'hCE;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h48: begin                                    // H
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hFE;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h49: begin                                    // I
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'h10;
                        3'd2: glyph_row = 8'h10;
                        3'd3: glyph_row = 8'h10;
                        3'd4: glyph_row = 8'h10;
                        3'd5: glyph_row = 8'h10;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4B: begin                                    // K
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hCC;
                        3'd2: glyph_row = 8'hD8;
                        3'd3: glyph_row = 8'hF0;
                        3'd4: glyph_row = 8'hD8;
                        3'd5: glyph_row = 8'hCC;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4A: begin                                    // J
                    case (row)
                        3'd0: glyph_row = 8'h3E;
                        3'd1: glyph_row = 8'h08;
                        3'd2: glyph_row = 8'h08;
                        3'd3: glyph_row = 8'h08;
                        3'd4: glyph_row = 8'h08;
                        3'd5: glyph_row = 8'h88;
                        3'd6: glyph_row = 8'h70;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4C: begin                                    // L
                    case (row)
                        3'd0: glyph_row = 8'hC0;
                        3'd1: glyph_row = 8'hC0;
                        3'd2: glyph_row = 8'hC0;
                        3'd3: glyph_row = 8'hC0;
                        3'd4: glyph_row = 8'hC0;
                        3'd5: glyph_row = 8'hC0;
                        3'd6: glyph_row = 8'hFE;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4D: begin                                    // M
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hEE;
                        3'd2: glyph_row = 8'hFE;
                        3'd3: glyph_row = 8'hD6;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4E: begin                                    // N
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hE6;
                        3'd2: glyph_row = 8'hF6;
                        3'd3: glyph_row = 8'hDE;
                        3'd4: glyph_row = 8'hCE;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h4F: begin                                    // O
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hC6;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h50: begin                                    // P
                    case (row)
                        3'd0: glyph_row = 8'hFC;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hFC;
                        3'd4: glyph_row = 8'hC0;
                        3'd5: glyph_row = 8'hC0;
                        3'd6: glyph_row = 8'hC0;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h51: begin                                    // Q
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hC6;
                        3'd4: glyph_row = 8'hDA;
                        3'd5: glyph_row = 8'hCC;
                        3'd6: glyph_row = 8'h76;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h52: begin                                    // R
                    case (row)
                        3'd0: glyph_row = 8'hFC;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hFC;
                        3'd4: glyph_row = 8'hD8;
                        3'd5: glyph_row = 8'hCC;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h53: begin                                    // S
                    case (row)
                        3'd0: glyph_row = 8'h7C;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC0;
                        3'd3: glyph_row = 8'h7C;
                        3'd4: glyph_row = 8'h06;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h54: begin                                    // T
                    case (row)
                        3'd0: glyph_row = 8'hFE;
                        3'd1: glyph_row = 8'h38;
                        3'd2: glyph_row = 8'h38;
                        3'd3: glyph_row = 8'h38;
                        3'd4: glyph_row = 8'h38;
                        3'd5: glyph_row = 8'h38;
                        3'd6: glyph_row = 8'h38;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h55: begin                                    // U
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hC6;
                        3'd4: glyph_row = 8'hC6;
                        3'd5: glyph_row = 8'hC6;
                        3'd6: glyph_row = 8'h7C;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h56: begin                                    // V
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hC6;
                        3'd4: glyph_row = 8'h6C;
                        3'd5: glyph_row = 8'h38;
                        3'd6: glyph_row = 8'h10;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h57: begin                                    // W
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'hC6;
                        3'd3: glyph_row = 8'hD6;
                        3'd4: glyph_row = 8'hFE;
                        3'd5: glyph_row = 8'hEE;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h58: begin                                    // X
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'h6C;
                        3'd2: glyph_row = 8'h38;
                        3'd3: glyph_row = 8'h10;
                        3'd4: glyph_row = 8'h38;
                        3'd5: glyph_row = 8'h6C;
                        3'd6: glyph_row = 8'hC6;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h59: begin                                    // Y
                    case (row)
                        3'd0: glyph_row = 8'hC6;
                        3'd1: glyph_row = 8'hC6;
                        3'd2: glyph_row = 8'h6C;
                        3'd3: glyph_row = 8'h38;
                        3'd4: glyph_row = 8'h10;
                        3'd5: glyph_row = 8'h10;
                        3'd6: glyph_row = 8'h10;
                        default: glyph_row = 8'h00;
                    endcase
                end
                8'h5A: begin                                    // Z
                    case (row)
                        3'd0: glyph_row = 8'hFE;
                        3'd1: glyph_row = 8'h06;
                        3'd2: glyph_row = 8'h0C;
                        3'd3: glyph_row = 8'h18;
                        3'd4: glyph_row = 8'h30;
                        3'd5: glyph_row = 8'h60;
                        3'd6: glyph_row = 8'hFE;
                        default: glyph_row = 8'h00;
                    endcase
                end
                default: glyph_row = 8'h00;
            endcase
        end
    endfunction

    function glyph_pixel;
        input [7:0] ch;
        input [3:0] gx;
        input [3:0] gy;
        reg [7:0] bits;
        begin
            bits = glyph_row(ch, gy[2:0]);
            glyph_pixel = (gx < 4'd8) && bits[3'd7 - gx[2:0]];
        end
    endfunction

    function text_char_hit;
        input [9:0] px_value;
        input [9:0] py_value;
        input [9:0] x0_value;
        input [9:0] y0_value;
        input [2:0] scale_value;
        input [7:0] ch;
        reg [9:0] dx_value;
        reg [9:0] dy_value;
        reg [3:0] gx_value;
        reg [3:0] gy_value;
        begin
            dx_value = px_value - x0_value;
            dy_value = py_value - y0_value;
            text_char_hit = 1'b0;
            if ((px_value >= x0_value) && (py_value >= y0_value) &&
                (dx_value < (8 << scale_value)) &&
                (dy_value < (8 << scale_value))) begin
                gx_value = dx_value >> scale_value;
                gy_value = dy_value >> scale_value;
                text_char_hit = glyph_pixel(ch, gx_value, gy_value);
            end
        end
    endfunction

    function [7:0] ascii_digit;
        input [3:0] d;
        begin
            ascii_digit = 8'h30 + {4'd0, d};
        end
    endfunction

    // Second row of the top-left panel:  RES <width>X<height>
    // Lead zeroes in the thousands position are blanked so common sizes read
    // as 640X480 instead of 0640X0480.
    function [7:0] res_char;
        input [3:0]  slot;
        input [15:0] width_bcd;
        input [15:0] height_bcd;
        input        valid;
        begin
            if (!valid) begin
                res_char = 8'h20;
            end else begin
                case (slot)
                    4'd0: res_char = 8'h52;                // R
                    4'd1: res_char = 8'h45;                // E
                    4'd2: res_char = 8'h53;                // S
                    4'd3: res_char = 8'h20;
                    4'd4: res_char = (width_bcd[15:12] != 4'd0) ?
                                     ascii_digit(width_bcd[15:12]) : 8'h20;
                    4'd5: res_char = ascii_digit(width_bcd[11:8]);
                    4'd6: res_char = ascii_digit(width_bcd[7:4]);
                    4'd7: res_char = ascii_digit(width_bcd[3:0]);
                    4'd8: res_char = 8'h58;                // X
                    4'd9: res_char = (height_bcd[15:12] != 4'd0) ?
                                     ascii_digit(height_bcd[15:12]) : 8'h20;
                    4'd10: res_char = ascii_digit(height_bcd[11:8]);
                    4'd11: res_char = ascii_digit(height_bcd[7:4]);
                    4'd12: res_char = ascii_digit(height_bcd[3:0]);
                    default: res_char = 8'h20;
                endcase
            end
        end
    endfunction

    function [7:0] top_char;
        input [3:0] slot;
        input [7:0] img_num;
        input [7:0] img_total;
        input       auto_mode_value;
        input       link_value;
        begin
            case (slot)
                4'd0: top_char = 8'h41;                     // A
                4'd1: top_char = 8'h4E;                     // N
                4'd2: top_char = 8'h4C;                     // L
                4'd3: top_char = 8'h55;                     // U
                4'd4: top_char = 8'h20;
                4'd5: top_char = 8'h54;                     // T
                4'd6: top_char = 8'h49;                     // I
                4'd7: top_char = 8'h53;                     // S
                4'd8: top_char = 8'h20;
                4'd9: top_char = ascii_digit(img_num[3:0]);
                4'd10: top_char = 8'h2F;                    // /
                4'd11: top_char = ascii_digit(img_total[3:0]);
                // "U" appears once the PC host has talked to the board
                4'd12: top_char = link_value ? 8'h55 : 8'h20;
                4'd13: top_char = auto_mode_value ? 8'h41 : 8'h4D; // A / M
                default: top_char = 8'h20;
            endcase
        end
    endfunction

    // Parameter panel readout.  The panel is 168 pixels wide, which is ten
    // 16 pixel character slots, so "CONTRAST 7" fits and every slot outside
    // the text is explicitly painted as a space instead of repeating the
    // last glyph across the rest of the band.
    function [7:0] param_char;
        input [3:0] slot;
        input [2:0] bright_level;
        input [2:0] contrast_level;
        input [1:0] mode_value;
        input [1:0] sel;
        begin
            if (sel == 2'd0) begin                        // BRIGHT n
                case (slot)
                    4'd0: param_char = 8'h42;                // B
                    4'd1: param_char = 8'h52;                // R
                    4'd2: param_char = 8'h49;                // I
                    4'd3: param_char = 8'h47;                // G
                    4'd4: param_char = 8'h48;                // H
                    4'd5: param_char = 8'h54;                // T
                    4'd6: param_char = 8'h20;
                    4'd7: param_char = ascii_digit({1'b0, bright_level});
                    default: param_char = 8'h20;
                endcase
            end else if (sel == 2'd1) begin               // CONTRAST n
                case (slot)
                    4'd0: param_char = 8'h43;                // C
                    4'd1: param_char = 8'h4F;                // O
                    4'd2: param_char = 8'h4E;                // N
                    4'd3: param_char = 8'h54;                // T
                    4'd4: param_char = 8'h52;                // R
                    4'd5: param_char = 8'h41;                // A
                    4'd6: param_char = 8'h53;                // S
                    4'd7: param_char = 8'h54;                // T
                    4'd8: param_char = 8'h20;
                    4'd9: param_char = ascii_digit({1'b0, contrast_level});
                    default: param_char = 8'h20;
                endcase
            end else begin                                // OSD <layout>
                case (slot)
                    4'd0: param_char = 8'h4F;                // O
                    4'd1: param_char = 8'h53;                // S
                    4'd2: param_char = 8'h44;                // D
                    4'd3: param_char = 8'h20;
                    default: begin
                        case (mode_value)
                            2'd0: begin                      // OSD FULL
                                case (slot)
                                    4'd4: param_char = 8'h46;     // F
                                    4'd5: param_char = 8'h55;     // U
                                    4'd6: param_char = 8'h4C;     // L
                                    4'd7: param_char = 8'h4C;     // L
                                    default: param_char = 8'h20;
                                endcase
                            end
                            2'd1: begin                      // OSD INFO
                                case (slot)
                                    4'd4: param_char = 8'h49;     // I
                                    4'd5: param_char = 8'h4E;     // N
                                    4'd6: param_char = 8'h46;     // F
                                    4'd7: param_char = 8'h4F;     // O
                                    default: param_char = 8'h20;
                                endcase
                            end
                            2'd2: begin                      // OSD ON
                                case (slot)
                                    4'd4: param_char = 8'h4F;     // O
                                    4'd5: param_char = 8'h4E;     // N
                                    default: param_char = 8'h20;
                                endcase
                            end
                            default: begin                   // OSD OFF
                                case (slot)
                                    4'd4: param_char = 8'h4F;     // O
                                    4'd5: param_char = 8'h46;     // F
                                    4'd6: param_char = 8'h46;     // F
                                    default: param_char = 8'h20;
                                endcase
                            end
                        endcase
                    end
                endcase
            end
        end
    endfunction

    function [7:0] subtitle_char;
        input [4:0] slot;
        begin
            case (slot)
                5'd0:  subtitle_char = 8'h46;                // F
                5'd1:  subtitle_char = 8'h4C;                // L
                5'd2:  subtitle_char = 8'h49;                // I
                5'd3:  subtitle_char = 8'h47;                // G
                5'd4:  subtitle_char = 8'h48;                // H
                5'd5:  subtitle_char = 8'h54;                // T
                5'd6:  subtitle_char = 8'h20;
                5'd7:  subtitle_char = 8'h41;                // A
                5'd8:  subtitle_char = 8'h4E;                // N
                5'd9:  subtitle_char = 8'h55;                // U
                5'd10: subtitle_char = 8'h32;                // 2
                5'd11: subtitle_char = 8'h30;                // 0
                5'd12: subtitle_char = 8'h32;                // 2
                5'd13: subtitle_char = 8'h36;                // 6
                5'd14: subtitle_char = 8'h20;
                5'd15: subtitle_char = 8'h47;                // G
                5'd16: subtitle_char = 8'h41;                // A
                5'd17: subtitle_char = 8'h54;                // T
                5'd18: subtitle_char = 8'h45;                // E
                5'd19: subtitle_char = 8'h20;
                5'd20: subtitle_char = 8'h31;                // 1
                5'd21: subtitle_char = 8'h32;                // 2
                5'd22: subtitle_char = 8'h20;
                5'd23: subtitle_char = 8'h42;                // B
                5'd24: subtitle_char = 8'h4F;                // O
                5'd25: subtitle_char = 8'h41;                // A
                5'd26: subtitle_char = 8'h52;                // R
                5'd27: subtitle_char = 8'h44;                // D
                5'd28: subtitle_char = 8'h49;                // I
                5'd29: subtitle_char = 8'h4E;                // N
                5'd30: subtitle_char = 8'h47;                // G
                5'd31: subtitle_char = 8'h20;
                default: subtitle_char = 8'h20;
            endcase
        end
    endfunction

    // Slow airport-operation layer.  The 32-slot circular message is:
    //   EAST HALL OPEN  WEST HALL OPEN
    // It shares the same on-chip glyph ROM as the main flight ticker.
    function [7:0] ticker_char;
        input [4:0] slot;
        begin
            case (slot)
                5'd0:  ticker_char = 8'h45;                // E
                5'd1:  ticker_char = 8'h41;                // A
                5'd2:  ticker_char = 8'h53;                // S
                5'd3:  ticker_char = 8'h54;                // T
                5'd4:  ticker_char = 8'h20;
                5'd5:  ticker_char = 8'h48;                // H
                5'd6:  ticker_char = 8'h41;                // A
                5'd7:  ticker_char = 8'h4C;                // L
                5'd8:  ticker_char = 8'h4C;                // L
                5'd9:  ticker_char = 8'h20;
                5'd10: ticker_char = 8'h4F;                // O
                5'd11: ticker_char = 8'h50;                // P
                5'd12: ticker_char = 8'h45;                // E
                5'd13: ticker_char = 8'h4E;                // N
                5'd14: ticker_char = 8'h20;
                5'd15: ticker_char = 8'h20;
                5'd16: ticker_char = 8'h57;                // W
                5'd17: ticker_char = 8'h45;                // E
                5'd18: ticker_char = 8'h53;                // S
                5'd19: ticker_char = 8'h54;                // T
                5'd20: ticker_char = 8'h20;
                5'd21: ticker_char = 8'h48;                // H
                5'd22: ticker_char = 8'h41;                // A
                5'd23: ticker_char = 8'h4C;                // L
                5'd24: ticker_char = 8'h4C;                // L
                5'd25: ticker_char = 8'h20;
                5'd26: ticker_char = 8'h4F;                // O
                5'd27: ticker_char = 8'h50;                // P
                5'd28: ticker_char = 8'h45;                // E
                5'd29: ticker_char = 8'h4E;                // N
                5'd30: ticker_char = 8'h20;
                5'd31: ticker_char = 8'h20;
                default: ticker_char = 8'h20;
            endcase
        end
    endfunction

    function [7:0] time_char;
        input [3:0] slot;
        input [3:0] hour_hi;
        input [3:0] hour_lo;
        input [3:0] min_hi;
        input [3:0] min_lo;
        input [3:0] sec_hi;
        input [3:0] sec_lo;
        begin
            case (slot)
                4'd0: time_char = ascii_digit(hour_hi);
                4'd1: time_char = ascii_digit(hour_lo);
                4'd2: time_char = 8'h3A;                 // :
                4'd3: time_char = ascii_digit(min_hi);
                4'd4: time_char = ascii_digit(min_lo);
                4'd5: time_char = 8'h3A;                 // :
                4'd6: time_char = ascii_digit(sec_hi);
                4'd7: time_char = ascii_digit(sec_lo);
                default: time_char = 8'h20;
            endcase
        end
    endfunction

    // Fixed-point alpha blend toward black.  alpha is in 1/64 units:
    //   0 = keep the picture, 64 = fully dark.
    // The +32 performs round-to-nearest before the divide by 64.
    function [23:0] blend_black;
        input [23:0] c;
        input [6:0] alpha;
        reg [6:0] inv;
        reg [15:0] r_term, g_term, b_term;
        reg [7:0] r_out, g_out, b_out;
        begin
            inv = 7'd64 - alpha;
            r_term = c[23:16] * inv;
            g_term = c[15:8]  * inv;
            b_term = c[7:0]   * inv;
            r_out = (r_term + 16'd32) >> 6;
            g_out = (g_term + 16'd32) >> 6;
            b_out = (b_term + 16'd32) >> 6;
            blend_black = {r_out, g_out, b_out};
        end
    endfunction

    function [23:0] dim_rgb;
        input [23:0] c;
        begin
            // 50% translucent dark layer for the fixed panels.
            dim_rgb = blend_black(c, 7'd32);
        end
    endfunction

    // Symmetrical vertical alpha ramp for the subtitle band:
    // 12/64 (19%) at the top/bottom edges to 42/64 (66%) at the middle.
    // Rows 15 and 16 share the centre value, so the 32-row band uses 16
    // distinct alpha levels and the scroll itself stays one pixel/frame.
    function [6:0] sub_bg_alpha;
        input [4:0] row;
        reg [4:0] ramp;
        begin
            if (row < 5'd16)
                ramp = row;
            else
                ramp = 5'd31 - row;
            sub_bg_alpha = 7'd12 + {1'b0, ramp, 1'b0};
        end
    endfunction

    function [23:0] wave_bg;
        input [23:0] c;
        begin
            wave_bg = dim_rgb(c) | 24'h000018;
        end
    endfunction

    // Eight band-energy bins are derived from a small one-pole filter bank.
    // The filter states run at the 48 kHz audio sample rate and use shifts
    // only, so no multiplier or FFT RAM is added to the pixel path.  Adjacent
    // signed low-pass outputs form band-pass signals; each signal is then
    // rectified and smoothed into a stable energy envelope.
    function [12:0] abs13;
        input signed [12:0] value;
        begin
            if (value[12])
                abs13 = ~value + 13'd1;
            else
                abs13 = value;
        end
    endfunction

    function [11:0] envelope_energy;
        input [12:0] envelope;
        begin
            // The one-pole filter bank already produces a useful range.
            // Saturate above 4095 instead of wrapping the 12 bit energy.
            if (envelope[12])
                envelope_energy = 12'hFFF;
            else
                envelope_energy = envelope[11:0];
        end
    endfunction

    function [5:0] energy_to_level;
        input [11:0] energy;
        begin
            // Roughly 4 pixels per octave.  The thresholds are powers of
            // two, which makes the display readable across speech and music
            // without needing a logarithm multiplier.
            if (energy < 12'd4)
                energy_to_level = 6'd0;
            else if (energy < 12'd8)
                energy_to_level = 6'd4;
            else if (energy < 12'd16)
                energy_to_level = 6'd8;
            else if (energy < 12'd32)
                energy_to_level = 6'd12;
            else if (energy < 12'd64)
                energy_to_level = 6'd16;
            else if (energy < 12'd128)
                energy_to_level = 6'd20;
            else if (energy < 12'd256)
                energy_to_level = 6'd24;
            else if (energy < 12'd512)
                energy_to_level = 6'd28;
            else if (energy < 12'd1024)
                energy_to_level = 6'd32;
            else if (energy < 12'd2048)
                energy_to_level = 6'd40;
            else if (energy < 12'd4095)
                energy_to_level = 6'd48;
            else
                energy_to_level = 6'd56;
        end
    endfunction

    //------------------------------------------------------------------
    // scrolling phase and text positions
    //------------------------------------------------------------------
    wire [7:0] img_num = {5'b00000, img_slot} + 8'd1;
    wire [7:0] img_total = (img_count_s2 > 8'd6) ? 8'd6 : img_count_s2;

    reg [8:0] scroll_pos;

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            scroll_pos <= 9'd0;
        end else if (frame_tick) begin
            if (scroll_pos == 9'd511)
                scroll_pos <= 9'd0;
            else
                scroll_pos <= scroll_pos + 9'd1;
        end
    end

    // Airport display time.  The host TIME command is authoritative once the
    // upper computer has pushed it; the DS1307 readback is only used while the
    // host has never spoken.  A local divider off the 25.000 MHz pixel clock
    // keeps the seconds counting on their own, so the panel only follows the
    // link while it is alive and then free runs from the last value.
    reg [7:0] rtc_hh, rtc_mm, rtc_ss;
    reg       time_tog_s1, time_tog_s2, time_tog_s3;
    reg       rtc_tog_s1, rtc_tog_s2, rtc_tog_s3;
    reg       host_time_seen;
    reg [24:0] sec_div;

    wire sec_wrap = (sec_div == 25'd24_999_999);
    wire time_bcd_ok = (rtc_ss[7:4] <= 4'd5) && (rtc_ss[3:0] <= 4'd9) &&
                       (rtc_mm[7:4] <= 4'd5) && (rtc_mm[3:0] <= 4'd9) &&
                       (rtc_hh[7:4] <= 4'd2) && (rtc_hh[3:0] <= 4'd9) &&
                       !((rtc_hh[7:4] == 4'd2) && (rtc_hh[3:0] > 4'd3));

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            time_tog_s1 <= 1'b0;
            time_tog_s2 <= 1'b0;
            time_tog_s3 <= 1'b0;
            rtc_tog_s1 <= 1'b0;
            rtc_tog_s2 <= 1'b0;
            rtc_tog_s3 <= 1'b0;
            host_time_seen <= 1'b0;
            sec_div <= 25'd0;
            rtc_hh <= 8'h00;
            rtc_mm <= 8'h00;
            rtc_ss <= 8'h00;
        end else begin
            time_tog_s1 <= host_time_tog;
            time_tog_s2 <= time_tog_s1;
            time_tog_s3 <= time_tog_s2;
            rtc_tog_s1 <= rtc_time_tog;
            rtc_tog_s2 <= rtc_tog_s1;
            rtc_tog_s3 <= rtc_tog_s2;

            if (sec_wrap)
                sec_div <= 25'd0;
            else
                sec_div <= sec_div + 25'd1;

            if (time_tog_s2 ^ time_tog_s3) begin
                // The host holds the BCD bus stable before flipping the
                // toggle, so the synchronised edge marks safe data.
                host_time_seen <= 1'b1;
                rtc_hh <= host_time_hh;
                rtc_mm <= host_time_mm;
                rtc_ss <= host_time_ss;
                sec_div <= 25'd0;
            end else if ((rtc_tog_s2 ^ rtc_tog_s3) && !host_time_seen) begin
                // The DS1307 bus is registered before its toggle, so the
                // synchronised edge marks safe BCD data.  Once the host has
                // taken over the RTC is ignored, which lets the panel keep
                // running on its own after the link drops instead of jumping
                // back to a possibly stale readback.
                rtc_hh <= rtc_time_hh;
                rtc_mm <= rtc_time_mm;
                rtc_ss <= rtc_time_ss;
                sec_div <= 25'd0;
            end else if (sec_wrap && time_bcd_ok) begin
                // One second of free running BCD arithmetic: hh:mm:ss with
                // 00..59 seconds/minutes and a 00..23 hour wrap.
                if (rtc_ss[3:0] != 4'd9) begin
                    rtc_ss[3:0] <= rtc_ss[3:0] + 4'd1;
                end else begin
                    rtc_ss[3:0] <= 4'd0;
                    if (rtc_ss[7:4] != 4'd5) begin
                        rtc_ss[7:4] <= rtc_ss[7:4] + 4'd1;
                    end else begin
                        rtc_ss[7:4] <= 4'd0;
                        if (rtc_mm[3:0] != 4'd9) begin
                            rtc_mm[3:0] <= rtc_mm[3:0] + 4'd1;
                        end else begin
                            rtc_mm[3:0] <= 4'd0;
                            if (rtc_mm[7:4] != 4'd5) begin
                                rtc_mm[7:4] <= rtc_mm[7:4] + 4'd1;
                            end else begin
                                rtc_mm[7:4] <= 4'd0;
                                if ((rtc_hh[7:4] == 4'd2) &&
                                    (rtc_hh[3:0] == 4'd3)) begin
                                    rtc_hh <= 8'h00;
                                end else if (rtc_hh[3:0] != 4'd9) begin
                                    rtc_hh[3:0] <= rtc_hh[3:0] + 4'd1;
                                end else begin
                                    rtc_hh[3:0] <= 4'd0;
                                    rtc_hh[7:4] <= rtc_hh[7:4] + 4'd1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    //------------------------------------------------------------------
    // text and panel hit tests
    //------------------------------------------------------------------
    wire [9:0] top_dx = px - TOP_X0;
    wire [9:0] top_dy = py - TOP_Y0;
    wire       in_top = (px >= TOP_X0) && (px < (TOP_X0 + TOP_W)) &&
                        (py >= TOP_Y0) && (py < (TOP_Y0 + TOP_H));
    wire [3:0] top_slot = top_dx[8:4];
    wire [9:0] top_char_x = TOP_X0 + {2'b00, top_slot, 4'b0000};
    wire [7:0] top_ch = top_char(top_slot, img_num, img_total, auto_s2,
                                 hlink_s2);
    wire [7:0] res_ch = res_char(top_slot, res_width_bcd, res_height_bcd,
                                 res_bcd_valid);
    // Both rows of the panel share one glyph ROM: the row picks the text and
    // the baseline, the tile position picks the character inside it.  A
    // second text_char_hit call would have duplicated the whole font.
    wire       res_row     = (py >= TOP_RES_TEXT_Y);
    wire [9:0] top_text_y  = res_row ? TOP_RES_TEXT_Y : TOP_TEXT_Y;
    wire [7:0] top_line_ch = res_row ? res_ch : top_ch;
    wire       top_glyph = in_top &&
                           text_char_hit(px, py, top_char_x, top_text_y,
                                         1, top_line_ch);

    wire [9:0] param_dx = px - PARAM_X0;
    wire [9:0] param_dy = py - PARAM_Y0;
    wire       in_param = (px >= PARAM_X0) &&
                          (px < (PARAM_X0 + PARAM_W)) &&
                          (py >= PARAM_Y0) &&
                          (py < (PARAM_Y0 + PARAM_H));
    wire [3:0] param_slot = param_dx[8:4];
    wire [9:0] param_char_x = PARAM_X0 + {2'b00, param_slot, 4'b0000};
    wire [7:0] param_ch = param_char(param_slot, bright_s2, contrast_s2,
                                      ui_mode_s2[1:0], psel_s2);
    wire       param_glyph = in_param &&
                             text_char_hit(px, py, param_char_x, PARAM_TEXT_Y,
                                           1, param_ch);

    wire [9:0] time_dx = px - TIME_X0;
    wire [9:0] time_dy = py - TIME_Y0;
    wire       in_time = (px >= TIME_X0) &&
                         (px < (TIME_X0 + TIME_W)) &&
                         (py >= TIME_Y0) &&
                         (py < (TIME_Y0 + TIME_H));
    // Eight 16 pixel glyphs occupy 128 pixels; the remaining 40 pixels of
    // the 168 pixel panel are split as a 20 pixel left margin.
    wire [9:0] time_gx = time_dx - 10'd20;
    wire [3:0] time_slot = time_gx[8:4];
    wire [9:0] time_char_x = TIME_X0 + 10'd20 +
                             {2'b00, time_slot, 4'b0000};
    wire [7:0] time_ch = time_char(time_slot,
                                   rtc_hh[7:4], rtc_hh[3:0],
                                   rtc_mm[7:4], rtc_mm[3:0],
                                   rtc_ss[7:4], rtc_ss[3:0]);
    wire       time_glyph = in_time &&
                            text_char_hit(px, py, time_char_x, TIME_TEXT_Y,
                                          1, time_ch);

    // The slow ticker reuses the low half of scroll_pos, so it advances one
    // screen pixel every two frames while the main ticker advances every
    // frame.  The different speeds create a visible parallax effect.
    wire [8:0]  tick_pos   = {1'b0, scroll_pos[8:1]};
    wire [10:0] tick_sum   = {1'b0, px} + {2'b00, tick_pos};
    wire [8:0]  tick_phase = tick_sum[8:0];
    wire [4:0]  tick_slot  = tick_phase[7:3];
    wire [3:0]  tick_gx    = {1'b0, tick_phase[2:0]};
    wire [9:0]  tick_dy    = py - TICK_Y0;
    wire        in_tick    = (px < TICK_W) && (py >= TICK_Y0) &&
                             (py < (TICK_Y0 + TICK_H));
    wire [7:0]  tick_ch    = ticker_char(tick_slot);
    wire        tick_glyph = in_tick &&
                             glyph_pixel(tick_ch, tick_gx, tick_dy[2:0]);

    wire [10:0] sub_sum = {1'b0, px} + {2'b00, scroll_pos};
    wire [8:0]  sub_phase = sub_sum[8:0];
    wire [4:0]  sub_slot = sub_phase[8:4];
    wire [3:0]  sub_gx = {1'b0, sub_phase[3:1]};
    wire [9:0]  sub_dy = py - SUB_Y0;
    wire        sub_in_y = (sub_dy >= SUB_TEXT_OFF) &&
                           (sub_dy < (SUB_TEXT_OFF + 10'd16));
    wire [3:0]  sub_gy = (sub_dy - SUB_TEXT_OFF) >> 1;
    // Once the host has pushed a message in, the bottom band shows that
    // message instead of the resident airport text.
    wire [7:0]  host_sub_ch = ({1'b0, sub_slot} < host_len) ?
                              host_char[sub_slot] : 8'h20;
    wire        host_text_on = (host_len != 6'd0);
    wire [7:0]  sub_ch = host_text_on ? host_sub_ch : subtitle_char(sub_slot);
    wire       in_sub = (px < SUB_W) && (py >= SUB_Y0) &&
                        (py < (SUB_Y0 + SUB_H));

    //------------------------------------------------------------------
    // flight information board.  It shares the single glyph lookup below,
    // so adding the board does not duplicate the 8x8 font.
    //------------------------------------------------------------------
    wire        bd_in, bd_y_ok, bd_flash, bd_border;
    wire [7:0]  bd_ch;
    wire [3:0]  bd_gx, bd_gy;
    wire [2:0]  bd_row;
    wire [23:0] bd_breath;

    flight_board u_flight_board (
        .pixel_clk  (pixel_clk),
        .rst_n      (~rst),
        .frame_tick (frame_tick),
        .px         (px),
        .py         (py),
        .enable     (board_enable),
        .wr_tog     (board_wr_tog),
        .wr_addr    (board_wr_addr),
        .wr_data    (board_wr_data),
        .in_board   (bd_in),
        .glyph_y_ok (bd_y_ok),
        .ch         (bd_ch),
        .gx         (bd_gx),
        .gy         (bd_gy),
        .row        (bd_row),
        .row_flash  (bd_flash),
        .border_hit (bd_border),
        .breath_rgb (bd_breath)
    );

    wire       bd_text    = bd_in && bd_y_ok;
    wire [7:0] draw_ch    = bd_text ? bd_ch : sub_ch;
    wire [3:0] draw_gx    = bd_text ? bd_gx : sub_gx;
    wire [3:0] draw_gy    = bd_text ? bd_gy : sub_gy;
    wire       draw_glyph = glyph_pixel(draw_ch, draw_gx, draw_gy);

    wire       sub_glyph = in_sub && sub_in_y && draw_glyph;
    wire       bd_glyph  = bd_text && draw_glyph;

    wire [9:0] wave_dx = px - WAVE_X0;
    wire       wave_in_x = (px >= WAVE_X0) && (px <= WAVE_X1);
    wire [7:0] wave_col = wave_in_x ? wave_dx[8:1] : 8'd0;

    //------------------------------------------------------------------
    // audio history RAM.  It is written by the I2S receiver, which is
    // already in the pixel clock domain.
    //------------------------------------------------------------------
    reg [11:0] sample_mem [0:127];
    reg [6:0]  wave_wr_ptr;

    // Frequency-band state.  The four-times gain in band_energy() is applied
    // after the adjacent low-pass subtraction, so the arithmetic stays small
    // enough for the existing pixel-clock timing budget.
    reg signed [11:0] spec_lp [0:6];
    reg signed [12:0] spec_env [0:7];
    reg [5:0]  spec_raw [0:7];
    reg [5:0]  band_level [0:7];

    wire signed [12:0] audio_signed = {audio_left[23], audio_left[23:12]};
    wire signed [12:0] lp_delta0 = audio_signed - spec_lp[0];
    wire signed [12:0] lp_delta1 = audio_signed - spec_lp[1];
    wire signed [12:0] lp_delta2 = audio_signed - spec_lp[2];
    wire signed [12:0] lp_delta3 = audio_signed - spec_lp[3];
    wire signed [12:0] lp_delta4 = audio_signed - spec_lp[4];
    wire signed [12:0] lp_delta5 = audio_signed - spec_lp[5];
    wire signed [12:0] lp_delta6 = audio_signed - spec_lp[6];

    wire signed [12:0] lp_ext0 = $signed({spec_lp[0][11], spec_lp[0]});
    wire signed [12:0] lp_ext1 = $signed({spec_lp[1][11], spec_lp[1]});
    wire signed [12:0] lp_ext2 = $signed({spec_lp[2][11], spec_lp[2]});
    wire signed [12:0] lp_ext3 = $signed({spec_lp[3][11], spec_lp[3]});
    wire signed [12:0] lp_ext4 = $signed({spec_lp[4][11], spec_lp[4]});
    wire signed [12:0] lp_ext5 = $signed({spec_lp[5][11], spec_lp[5]});
    wire signed [12:0] lp_ext6 = $signed({spec_lp[6][11], spec_lp[6]});

    wire signed [12:0] band_sig0 = lp_ext0;
    wire signed [12:0] band_sig1 = lp_ext1 - lp_ext0;
    wire signed [12:0] band_sig2 = lp_ext2 - lp_ext1;
    wire signed [12:0] band_sig3 = lp_ext3 - lp_ext2;
    wire signed [12:0] band_sig4 = lp_ext4 - lp_ext3;
    wire signed [12:0] band_sig5 = lp_ext5 - lp_ext4;
    wire signed [12:0] band_sig6 = lp_ext6 - lp_ext5;
    wire signed [12:0] band_sig7 = audio_signed - lp_ext6;

    wire [12:0] band_mag0 = abs13(band_sig0);
    wire [12:0] band_mag1 = abs13(band_sig1);
    wire [12:0] band_mag2 = abs13(band_sig2);
    wire [12:0] band_mag3 = abs13(band_sig3);
    wire [12:0] band_mag4 = abs13(band_sig4);
    wire [12:0] band_mag5 = abs13(band_sig5);
    wire [12:0] band_mag6 = abs13(band_sig6);
    wire [12:0] band_mag7 = abs13(band_sig7);

    wire signed [13:0] env_delta0 = $signed({1'b0, band_mag0}) -
                                    $signed({1'b0, spec_env[0]});
    wire signed [13:0] env_delta1 = $signed({1'b0, band_mag1}) -
                                    $signed({1'b0, spec_env[1]});
    wire signed [13:0] env_delta2 = $signed({1'b0, band_mag2}) -
                                    $signed({1'b0, spec_env[2]});
    wire signed [13:0] env_delta3 = $signed({1'b0, band_mag3}) -
                                    $signed({1'b0, spec_env[3]});
    wire signed [13:0] env_delta4 = $signed({1'b0, band_mag4}) -
                                    $signed({1'b0, spec_env[4]});
    wire signed [13:0] env_delta5 = $signed({1'b0, band_mag5}) -
                                    $signed({1'b0, spec_env[5]});
    wire signed [13:0] env_delta6 = $signed({1'b0, band_mag6}) -
                                    $signed({1'b0, spec_env[6]});
    wire signed [13:0] env_delta7 = $signed({1'b0, band_mag7}) -
                                    $signed({1'b0, spec_env[7]});

    reg [11:0] wave_raw;
    always @* begin
        if (audio_rst)
            wave_raw = 12'd0;
        else
            // The newest sample is on the right.  Adding the write pointer
            // makes the history move as new samples arrive.
            wave_raw = sample_mem[(wave_col + {1'b0, wave_wr_ptr}) & 8'h7F];
    end

    wire [7:0] wave_level = wave_raw[11:4] ^ 8'h80;
    wire [9:0] wave_y = WAVE_Y0 + {2'b00, wave_level[7:2]};
    wire       in_wave = (px >= WAVE_X0) && (px <= WAVE_X1) &&
                         (py >= WAVE_Y0) && (py <= WAVE_Y1);
    wire       in_wave_graph = in_wave && (py <= WAVE_GRAPH_Y1);
    wire       in_spectrum = in_wave && (py >= SPEC_Y0);
    wire       wave_line = in_wave_graph && ((py == wave_y) ||
                                             (py == (wave_y + 10'd1)));
    wire       wave_axis = in_wave_graph && (py == (WAVE_Y0 + 10'd32));

    wire [9:0] spec_dx = px - WAVE_X0;
    wire [2:0] spec_bin = spec_dx[7:5];
    wire [4:0] spec_slot_x = spec_dx[4:0];
    wire [5:0] spec_level = band_level[spec_bin];
    wire [9:0] spec_top = WAVE_Y1 - {4'b0000, spec_level};
    wire       spec_bar = in_spectrum && (spec_level != 6'd0) &&
                          (spec_slot_x < 5'd28) && (py >= spec_top);
    wire       spec_grid = in_spectrum && !spec_bar &&
                           ((py == WAVE_Y1) ||
                            ((py[2:0] == 3'd0) && (py != SPEC_Y0)) ||
                            (spec_slot_x == 5'd28));

    wire       top_show   = ui_enable_s2 && de && logo_enable &&
                            (ui_mode_s2 != 3'd3);
    wire       param_show = ui_enable_s2 && de && pflash_s2 &&
                            osd_enable;
    wire       time_show  = ui_enable_s2 && de && timestamp_enable &&
                            (ui_mode_s2 != 3'd3) && !(param_show && in_time);
    wire       tick_show  = ui_enable_s2 && de && parallax_enable &&
                            (ui_mode_s2 != 3'd3);
    wire       sub_show   = ui_enable_s2 && de && subtitle_enable &&
                            (ui_mode_s2 != 3'd3);
    wire       wave_show  = ui_enable_s2 && de && wave_enable &&
                            (ui_mode_s2 == 3'd0);

    always @(posedge pixel_clk or posedge audio_rst) begin
        if (audio_rst) begin
            wave_wr_ptr <= 7'd0;
        end else if (audio_valid) begin
            sample_mem[wave_wr_ptr] <= audio_left[23:12];
            wave_wr_ptr <= wave_wr_ptr + 7'd1;
        end
    end

    always @(posedge pixel_clk or posedge audio_rst) begin
        if (audio_rst) begin
            spec_lp[0] <= 12'd0;
            spec_lp[1] <= 12'd0;
            spec_lp[2] <= 12'd0;
            spec_lp[3] <= 12'd0;
            spec_lp[4] <= 12'd0;
            spec_lp[5] <= 12'd0;
            spec_lp[6] <= 12'd0;
            spec_env[0] <= 13'd0;
            spec_env[1] <= 13'd0;
            spec_env[2] <= 13'd0;
            spec_env[3] <= 13'd0;
            spec_env[4] <= 13'd0;
            spec_env[5] <= 13'd0;
            spec_env[6] <= 13'd0;
            spec_env[7] <= 13'd0;
            spec_raw[0] <= 6'd0;
            spec_raw[1] <= 6'd0;
            spec_raw[2] <= 6'd0;
            spec_raw[3] <= 6'd0;
            spec_raw[4] <= 6'd0;
            spec_raw[5] <= 6'd0;
            spec_raw[6] <= 6'd0;
            spec_raw[7] <= 6'd0;
        end else if (audio_valid) begin
            // Cutoffs are approximately 87, 175, 350, 690, 1.39 k, 2.73 k
            // and 5.3 kHz at 48 kHz.  Adjacent differences become the eight
            // displayed bands; the final band is the residual above 5.3 kHz.
            spec_lp[0] <= spec_lp[0] + (lp_delta0 >>> 7);
            spec_lp[1] <= spec_lp[1] + (lp_delta1 >>> 6);
            spec_lp[2] <= spec_lp[2] + (lp_delta2 >>> 5);
            spec_lp[3] <= spec_lp[3] + (lp_delta3 >>> 4);
            spec_lp[4] <= spec_lp[4] + (lp_delta4 >>> 3);
            spec_lp[5] <= spec_lp[5] + (lp_delta5 >>> 2);
            spec_lp[6] <= spec_lp[6] + (lp_delta6 >>> 1);

            // Rectify each band-pass signal and smooth it into an envelope.
            // The shift of four gives a fast enough attack for speech while
            // keeping the bar movement readable.
            spec_env[0] <= spec_env[0] + (env_delta0 >>> 4);
            spec_env[1] <= spec_env[1] + (env_delta1 >>> 4);
            spec_env[2] <= spec_env[2] + (env_delta2 >>> 4);
            spec_env[3] <= spec_env[3] + (env_delta3 >>> 4);
            spec_env[4] <= spec_env[4] + (env_delta4 >>> 4);
            spec_env[5] <= spec_env[5] + (env_delta5 >>> 4);
            spec_env[6] <= spec_env[6] + (env_delta6 >>> 4);
            spec_env[7] <= spec_env[7] + (env_delta7 >>> 4);

            spec_raw[0] <= energy_to_level(envelope_energy(spec_env[0]));
            spec_raw[1] <= energy_to_level(envelope_energy(spec_env[1]));
            spec_raw[2] <= energy_to_level(envelope_energy(spec_env[2]));
            spec_raw[3] <= energy_to_level(envelope_energy(spec_env[3]));
            spec_raw[4] <= energy_to_level(envelope_energy(spec_env[4]));
            spec_raw[5] <= energy_to_level(envelope_energy(spec_env[5]));
            spec_raw[6] <= energy_to_level(envelope_energy(spec_env[6]));
            spec_raw[7] <= energy_to_level(envelope_energy(spec_env[7]));
        end
    end

    always @(posedge pixel_clk or posedge audio_rst) begin
        if (audio_rst) begin
            band_level[0] <= 6'd0;
            band_level[1] <= 6'd0;
            band_level[2] <= 6'd0;
            band_level[3] <= 6'd0;
            band_level[4] <= 6'd0;
            band_level[5] <= 6'd0;
            band_level[6] <= 6'd0;
            band_level[7] <= 6'd0;
        end else if (frame_tick) begin
            // Fast attack, one-step release per video frame.  This keeps a
            // short syllable visible long enough to read on a photograph.
            if (spec_raw[0] > band_level[0])
                band_level[0] <= spec_raw[0];
            else if (band_level[0] != 6'd0)
                band_level[0] <= band_level[0] - 6'd1;

            if (spec_raw[1] > band_level[1])
                band_level[1] <= spec_raw[1];
            else if (band_level[1] != 6'd0)
                band_level[1] <= band_level[1] - 6'd1;

            if (spec_raw[2] > band_level[2])
                band_level[2] <= spec_raw[2];
            else if (band_level[2] != 6'd0)
                band_level[2] <= band_level[2] - 6'd1;

            if (spec_raw[3] > band_level[3])
                band_level[3] <= spec_raw[3];
            else if (band_level[3] != 6'd0)
                band_level[3] <= band_level[3] - 6'd1;

            if (spec_raw[4] > band_level[4])
                band_level[4] <= spec_raw[4];
            else if (band_level[4] != 6'd0)
                band_level[4] <= band_level[4] - 6'd1;

            if (spec_raw[5] > band_level[5])
                band_level[5] <= spec_raw[5];
            else if (band_level[5] != 6'd0)
                band_level[5] <= band_level[5] - 6'd1;

            if (spec_raw[6] > band_level[6])
                band_level[6] <= spec_raw[6];
            else if (band_level[6] != 6'd0)
                band_level[6] <= band_level[6] - 6'd1;

            if (spec_raw[7] > band_level[7])
                band_level[7] <= spec_raw[7];
            else if (band_level[7] != 6'd0)
                band_level[7] <= band_level[7] - 6'd1;
        end
    end

    //------------------------------------------------------------------
    // Fire emergency layer.  The placards themselves are read from the TF
    // card (EMGFIRE.BMP / EMGEXIT.BMP), so this block only has to do two
    // things:
    //
    //   * while a placard owns the screen, all other layers are bypassed.
    //     emg_pic is held by the sequencer after the alarm clears until the
    //     first normal frame has actually reached the display, so the
    //     flight board cannot appear over the old evacuation image
    //   * while the alarm is high but the placard is still being read from
    //     the card, a red field with the usual yellow / black hazard rails
    //     stands in, so a normal slide is never left on screen
    //------------------------------------------------------------------
    reg emg_s1, emg_s2;

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            emg_s1 <= 1'b0;
            emg_s2 <= 1'b0;
        end else begin
            emg_s1 <= emg_fire;
            emg_s2 <= emg_s1;
        end
    end

    // frame_tick paces the standby placard: bit 5 flips the red field about
    // once a second.
    reg [5:0] emg_anim;
    always @(posedge pixel_clk or posedge rst) begin
        if (rst)
            emg_anim <= 6'd0;
        else if (frame_tick)
            emg_anim <= emg_anim + 6'd1;
    end

    wire        emg_on    = emg_s2 && de;
    wire        emg_flash = emg_anim[5];

    // Hazard rails: 45 degree yellow / black stripes in the top and bottom
    // band, the usual marking of an evacuation sign.
    wire [10:0] emg_diag   = {1'b0, px} + {1'b0, py};
    wire        emg_rail   = (py < 10'd16) || (py >= 10'd464);
    wire        emg_stripe = emg_diag[4] ^ emg_diag[3];
    wire [23:0] emg_rail_rgb = emg_stripe ? 24'hFFC61A : 24'h181818;

    wire [23:0] emg_rgb_c = emg_rail ? emg_rail_rgb :
                            (emg_flash ? 24'hB41212 : 24'h600808);

    // One pipeline stage keeps the layer off the pixel-clock critical path
    // and aligns the placard select with the registered pixel word.
    reg        emg_hit_r;
    reg        emg_pic_r;
    reg [23:0] emg_rgb_r;
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            emg_hit_r <= 1'b0;
            emg_pic_r <= 1'b0;
            emg_rgb_r <= 24'd0;
        end else begin
            emg_hit_r <= emg_on && !emg_pic;
            emg_pic_r <= de && emg_pic;
            emg_rgb_r <= emg_rgb_c;
        end
    end

    //------------------------------------------------------------------
    // final layer selection.  Parameter readout has priority over the logo;
    // the overlay regions do not overlap in normal operation.
    //------------------------------------------------------------------
    always @* begin
        rgb = base_rgb;
        hit = 1'b0;

        if (emg_pic_r) begin
            // Fire placard straight off the TF card: it owns the screen, so
            // the logo, the clock, the flight board and the tickers are all
            // bypassed until the alarm clears.
            hit = 1'b0;
            rgb = base_rgb;
        end else if (emg_hit_r) begin
            // Placard still being read from the card: standby warning board.
            hit = 1'b1;
            rgb = emg_rgb_r;
        end else if (param_show && in_param) begin
            hit = 1'b1;
            rgb = dim_rgb(base_rgb);
            if (param_glyph)
                rgb = 24'hFFC000;
        end else if (time_show && in_time) begin
            hit = 1'b1;
            rgb = dim_rgb(base_rgb);
            if (time_glyph)
                rgb = 24'hFFD060;
        end else if (top_show && in_top) begin
            hit = 1'b1;
            rgb = dim_rgb(base_rgb);
            if (top_glyph)
                rgb = 24'h00E0FF;
        end else if (tick_show && in_tick) begin
            hit = 1'b1;
            rgb = blend_black(base_rgb, 7'd20);
            if (tick_glyph)
                rgb = 24'h7FE7FF;
        end else if (bd_in) begin
            // Flight board: dimmed panel, breathing rails, and the row that
            // was just written by the host flashes to show the update.
            hit = 1'b1;
            rgb = blend_black(base_rgb, 7'd44);
            if (bd_border)
                rgb = bd_breath;
            else if (bd_glyph)
                rgb = bd_flash     ? 24'hFFE060 :
                      (bd_row == 3'd0) ? 24'h00E0FF : 24'hFFFFFF;
        end else if (sub_show && in_sub) begin
            hit = 1'b1;
            rgb = blend_black(base_rgb, sub_bg_alpha(sub_dy[4:0]));
            if (sub_glyph)
                rgb = 24'hFFFFFF;
        end else if (wave_show && in_wave) begin
            hit = 1'b1;
            rgb = wave_bg(base_rgb);
            if (in_spectrum) begin
                if (spec_bar) begin
                    if (spec_bin[2])
                        rgb = 24'hFFB020;       // upper bands
                    else if (spec_bin[1])
                        rgb = 24'h00E0FF;       // upper-mid bands
                    else
                        rgb = 24'h00FF80;       // low bands
                end else if (spec_grid) begin
                    rgb = 24'h006080;
                end
            end else begin
                if (wave_line)
                    rgb = 24'h00FFFF;
                else if (wave_axis)
                    rgb = 24'h006070;
            end
        end
    end

endmodule
