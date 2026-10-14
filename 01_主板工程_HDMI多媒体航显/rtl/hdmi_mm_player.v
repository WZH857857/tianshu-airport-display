`timescale 1ns / 1ps
//=============================================================================
// hdmi_mm_player.v - HDMI multimedia player, top level
//
// picture path
//   TF card -> SPI -> sector buffer -> FAT32 directory scan -> BMP parser
//   -> SDRAM frame buffer -> HDMI transmitter
//
//   the SDRAM holds two 640x480x24 frames. A picture is built in the buffer
//   that is not on screen and swapped in at a video frame boundary, so the
//   picture never tears. The built in test pattern is shown until the first
//   picture arrives and whenever the card path fails.
//
// audio path
//   built in airport chime tone -> I2S -> sample capture -> HDMI audio sample
//   packets inside the data islands of the official HDMI 1.4b transmitter
//   core. The tone is generated on chip, no audio file on the card is needed.
//
//   the frame buffer drives the HDMI 1.4b transmitter core:
//   video_rgb_to_axis_640x480 -> hdmi_1_4b_transmitter_core_wrapper ->
//   hdmi_phy_wrapper -> HDMI_A TMDS pairs. The core builds the HDMI framing
//   (control periods, guard bands and the data islands that carry the audio),
//   so unlike the DEMO 31 DVI encoder it can send sound to the display.
//
//   Video and audio resets are deliberately separated.  The HDMI video path
//   only needs the board reset to be released, while the audio path waits for
//   its dedicated PLL to lock.  An audio-PLL problem must not blank video.
//=============================================================================

//-----------------------------------------------------------------------------
// Support modules that live in their own files.
//
// TD rewrites the .al project file whenever it opens the project and drops
// every source entry it did not create itself.  A hand-added entry therefore
// survives exactly one build and then turns the module into a black box, which
// is how the voice stream, the resampler and the picture scaler once vanished
// from a build without any RTL error.  Including them as text keeps them in
// the netlist no matter what the project file happens to hold.
//-----------------------------------------------------------------------------
`include "audio/wav_voice_stream.v"
`include "audio/voice_player_stream.v"
`include "audio/voice_repeat_ctrl.v"
`include "storage/bmp_scaler.v"
`include "i2c_rtc.v"
`include "uart/uart_rx.v"
`include "uart/uart_tx.v"
`include "uart/uart_ctrl.v"

module hdmi_mm_player #(
    // Advanced-demo feature switches.  Each ADVxx wrapper turns on only the
    // feature being verified; the timestamp defaults off for old wrappers.
    parameter integer FEATURE_UI         = 1,
    parameter integer FEATURE_LOGO       = 1,
    parameter integer FEATURE_SUBTITLE   = 1,
    parameter integer FEATURE_TIMESTAMP  = 0,
    parameter integer FEATURE_PARALLAX   = 0,
    parameter integer FEATURE_FADE       = 1,
    parameter integer FEATURE_BRIGHTNESS = 1,
    parameter integer FEATURE_OSD        = 1,
    parameter integer FEATURE_AUDIO_WAVE = 1,
    // 1 = accept any picture size inside the scaler limits and resize it to
    // the display geometry on the way into the frame store.  0 keeps the
    // historical "only 640 x 480 is accepted" parser.
    parameter integer FEATURE_SCALE      = 0,

    // Transition tuning.  FADE_STEP is the gain step taken on every video
    // frame, so a smaller value gives a longer, smoother fade.  With a 5-bit
    // gain the fade lasts 32/FADE_STEP frames each way.  FADE_WAIT_TIMEOUT is
    // the number of frames the fade controller waits for the new picture
    // before it gives up and fades back in anyway (0 disables the watchdog).
    parameter integer FADE_STEP          = 4,
    parameter integer FADE_WAIT_TIMEOUT  = 0,

    // Automatic picture-advance interval in 50 MHz system clock cycles.
    // The generic player keeps the historical 3 s default; the ADV02
    // wrapper selects 6 s explicitly.
    parameter integer AUTO_HOLD_CYCLES   = 150_000_000,

    // Watchdog for one BMP load, in 50 MHz system clock cycles.  A large
    // source picture takes longer to stream from the card than a 640 x 480
    // one, so the scaling wrapper raises this.
    parameter integer LOAD_CYCLES        = 1_000_000_000,

    // Delay between two successful card voice announcements.  The controller
    // counts one-second ticks on sys_clk, so the default is exactly 60 s.
    parameter [25:0] VOICE_REPEAT_GAP_TICK_CYCLES = 26'd50_000_000,
    parameter [7:0]  VOICE_REPEAT_GAP_SECONDS     = 8'd60,

    // Pause between two fire alarm announcements.  The evacuation message
    // keeps repeating for as long as the alarm is latched, with a short gap so
    // the placard reader still has a window to fetch EMGEXIT.BMP.
    parameter [25:0] FIRE_VOICE_GAP_TICK_CYCLES   = 26'd50_000_000,
    parameter [7:0]  FIRE_VOICE_GAP_SECONDS       = 8'd2
) (
    input  wire       sys_clk,          // 50 MHz
    input  wire       sys_rst_n,
    input  wire       key_mode,         // active low, next picture
    input  wire       key_bright,       // active low, brightness
    input  wire       key_ui,           // active low, overlay mode
    input  wire       key_auto,         // SW1: 1 = automatic rotation

    // CH340 USB-UART bridge on the Type-C socket, same cable as the power
    input  wire       uart_rx_pin,      // FPGA RXD  = F12
    output wire       uart_tx_pin,      // FPGA TXD  = D12

    // DS1307 real-time clock on the J1 expansion header
    output wire       rtc_scl,          // J1.25 / FPGA H15
    inout  wire       rtc_sda,          // J1.26 / FPGA H16

    // Fire emergency, mirrored from the host link.  The top level turns it
    // into the board buzzer pattern and the seven segment alarm display.
    output wire       emg_fire,

    output wire [3:0] led,

    output wire       HDMI_CLK_P,
    output wire       HDMI_D2_P,
    output wire       HDMI_D1_P,
    output wire       HDMI_D0_P,

    // HDMI DDC (EDID read), socket A
    output wire       HDMI_DDC_SCL,
    inout  wire       HDMI_DDC_SDA,

    // TF card, SPI mode
    output wire       SD_nCS,
    output wire       SD_DCLK,
    output wire       SD_MOSI,
    input  wire       SD_MISO
);

    wire pixel_clk;         // 25 MHz
    wire pixel_clk_5x;      // 125 MHz
    wire mem_clk;           // 125 MHz, vendor SDRAM controller
    wire mem_clk_sft;       // 125 MHz, 180 degrees, SDRAM read capture
    wire audio_pll_lock;
    wire audio_mclk;

    //------------------------------------------------------------------
    // serial host link.  The controller itself is instantiated at the end of
    // the file, once every status signal it reports is in scope.
    //------------------------------------------------------------------
    wire        uart_jump_req;
    wire [2:0]  uart_jump_slot;
    wire        uart_auto_ovr_en, uart_auto_ovr_mode;
    wire        uart_bright_we, uart_contrast_we, uart_osd_we;
    wire [2:0]  uart_bright_val, uart_contrast_val;
    wire [1:0]  uart_osd_val;
    wire        uart_text_tog, uart_text_clear_tog, uart_text_commit_tog;
    wire        uart_link_seen;
    wire [4:0]  uart_text_addr;
    wire [7:0]  uart_text_data;
    wire        uart_board_tog, uart_board_enable;
    wire [7:0]  uart_board_addr, uart_board_data;
    wire [7:0]  uart_time_hh, uart_time_mm, uart_time_ss;
    wire        uart_time_tog;
    wire        uart_swap_target_en;
    wire [2:0]  uart_swap_target;
    // Frame store placard-slot select, driven by the picture sequencer.
    wire        ctrl_emg_slot;
    wire        ctrl_emg_slot_sel;

    //------------------------------------------------------------------
    // DS1307 wall clock.  A one-second heartbeat asks the I2C block to read
    // hh/mm/ss.  The BCD bus and its toggle then cross into the pixel-clock
    // overlay as a fallback for a board that has never received the host TIME
    // command.  Once host time arrives, the overlay free-runs locally.
    //------------------------------------------------------------------
    wire        rtc_valid;
    wire [7:0]  rtc_sec_bcd;
    wire [7:0]  rtc_min_bcd;
    wire [7:0]  rtc_hour_bcd;
    reg  [25:0] rtc_read_cnt;
    reg         rtc_time_tog;

    wire rtc_read_tick = (rtc_read_cnt == 26'd49_999_999);

    // Reject a halted or malformed DS1307 register before it reaches the OSD.
    wire rtc_bcd_ok =
        (rtc_sec_bcd[7:4] <= 4'd5) && (rtc_sec_bcd[3:0] <= 4'd9) &&
        (rtc_min_bcd[7:4] <= 4'd5) && (rtc_min_bcd[3:0] <= 4'd9) &&
        (rtc_hour_bcd[7:4] <= 4'd2) && (rtc_hour_bcd[3:0] <= 4'd9) &&
        !rtc_sec_bcd[7];

    wire rtc_update = rtc_valid && rtc_bcd_ok;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            rtc_read_cnt <= 26'd0;
        else if (rtc_read_tick)
            rtc_read_cnt <= 26'd0;
        else
            rtc_read_cnt <= rtc_read_cnt + 26'd1;
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            rtc_time_tog <= 1'b0;
        else if (rtc_update)
            rtc_time_tog <= ~rtc_time_tog;
    end

    i2c_rtc u_i2c_rtc (
        .clk        (sys_clk),
        .rst_n      (sys_rst_n),
        .start_read (rtc_read_tick),
        .busy       (),
        .rtc_valid  (rtc_valid),
        .sec_bcd    (rtc_sec_bcd),
        .min_bcd    (rtc_min_bcd),
        .hour_bcd   (rtc_hour_bcd),
        .rtc_scl    (rtc_scl),
        .rtc_sda    (rtc_sda)
    );

    //------------------------------------------------------------------
    // clocks
    //------------------------------------------------------------------
    //  Use the official lab_ex5_i2s video_pll, not DEMO31 clk_wize:
    //  C0 = 25 MHz,  the pixel clock
    //  C1 = 125 MHz, the 5x pixel clock the HDMI 1.4b serialiser needs
    //
    //  The audio transmitter core's hdmi_phy_wrapper and lane_lvds_10_1
    //  serializer expect the phase relationship of this vendor PLL.  The
    //  DEMO31 PLL has the same two frequencies but the opposite output
    //  order/feedback path, which caused the audio-core bitstream to show
    //  "no signal" even though the video-only DEMO31 path worked.
    video_pll u_pixel_pll (
        .refclk  (sys_clk),
        .reset   (1'b0),
        .clk0_out(pixel_clk),           // 25 MHz
        .clk1_out(pixel_clk_5x)         // 125 MHz
    );

    // The on die SDRAM is a plain SDR device.  The vendor lab_ex5_i2s
    // reference drives it from a 125 MHz clock and captures read data with a
    // second 125 MHz output shifted by 180 degrees.  Use the same clock pair:
    // the capture edge then sits in the middle of the SDRAM data eye instead
    // of on top of the transition.
    sys_pll u_mem_pll (
        .refclk   (sys_clk),
        .reset    (1'b0),
        .clk0_out (),               // 100 MHz, unused
        .clk1_out (mem_clk),        // 125 MHz
        .clk2_out (mem_clk_sft)     // 125 MHz, 180 degrees
    );

    // 50 MHz / 2 * 3 = 75 MHz on C0 (the feedback output), 750 MHz / 61 =
    // 12.295 MHz on C2.  The official HDMI audio path uses this PLL and
    // waits for extlock before releasing the HDMI transmitter.
    PLL_HDMI_AUDIO u_audio_pll (
        .refclk  (sys_clk),
        .reset   (1'b0),
        .extlock (audio_pll_lock),
        .clk0_out(),                // 75 MHz, feedback only
        .clk1_out(),                // 375 MHz, unused
        .clk2_out(audio_mclk)       // 12.295 MHz
    );

    // Keep video available even if the audio PLL fails to lock.  The audio
    // data path has its own reset below and remains gated by audio_pll_lock.
    wire rst_video = ~sys_rst_n;

    //------------------------------------------------------------------
    // video timing and the one pixel delay that matches the FIFO latency
    //------------------------------------------------------------------
    wire [9:0] x, y;
    wire [9:0] px, py;
    wire       hsync, vsync, video_active, frame_tick;
    reg        hsync_d, vsync_d, de_d;
    reg [9:0]  px_d, py_d;

    video_timing_640x480 u_video_timing (
        .pixel_clk    (pixel_clk),
        .reset_n      (~rst_video),
        .x            (x),
        .y            (y),
        .px           (px),
        .py           (py),
        .hsync        (hsync),
        .vsync        (vsync),
        .video_active (video_active),
        .frame_tick   (frame_tick)
    );

    always @(posedge pixel_clk or posedge rst_video) begin
        if (rst_video) begin
            hsync_d <= 1'b1;
            vsync_d <= 1'b1;
            de_d    <= 1'b0;
            px_d    <= 10'd0;
            py_d    <= 10'd0;
        end else begin
            hsync_d <= hsync;
            vsync_d <= vsync;
            de_d    <= video_active;
            // the frame buffer hands its pixel out one pixel later than the
            // timing generator reports, so the overlay uses the same delay
            px_d    <= px;
            py_d    <= py;
        end
    end

    //------------------------------------------------------------------
    // mode key: advance to the next picture
    //------------------------------------------------------------------
    wire mode_pressed;
    wire auto_switch_n;

    button_debounce #(
        .DEBOUNCE_CYCLES(500000)        // 10 ms at 50 MHz
    ) u_mode_button (
        .clock        (sys_clk),
        .reset_n      (sys_rst_n),
        .button_n     (key_mode),
        .pressed_tick (mode_pressed),
        .stable_n     ()
    );

    // Board SW1 is a level switch, not a momentary key.  Feed the debouncer
    // its active-low form so the existing 10 ms filter can be reused, then
    // restore the documented polarity: switch up = 1 = automatic rotation,
    // switch down = 0 = manual (default).
    button_debounce #(
        .DEBOUNCE_CYCLES(500000)
    ) u_auto_switch (
        .clock        (sys_clk),
        .reset_n      (sys_rst_n),
        .button_n     (~key_auto),
        .pressed_tick (),
        .stable_n     (auto_switch_n)
    );

    // The serial host may take the mode over from the switch; the switch
    // keeps control until the host says otherwise.
    wire auto_rotate_en = uart_auto_ovr_en ? uart_auto_ovr_mode
                                           : ~auto_switch_n;

    wire bright_pressed, ui_pressed;
    wire ui_button_n;

    button_debounce #(
        .DEBOUNCE_CYCLES(500000)
    ) u_bright_button (
        .clock        (sys_clk),
        .reset_n      (sys_rst_n),
        .button_n     (key_bright),
        .pressed_tick (bright_pressed),
        .stable_n     ()
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(500000)
    ) u_ui_button (
        .clock        (sys_clk),
        .reset_n      (sys_rst_n),
        .button_n     (key_ui),
        .pressed_tick (ui_pressed),
        .stable_n     (ui_button_n)
    );

    //------------------------------------------------------------------
    // On screen parameter control (advanced item 4: real time adjustment
    // with the change shown on the picture)
    //
    //   KEY2 short       adjust the selected parameter and show it for 1 s
    //   KEY4 short       select the parameter KEY2 adjusts
    //                    BRIGHT -> CONTRAST -> BRIGHT
    //   KEY4 long (1 s)  cycle the overlay layout, the earlier behaviour
    //
    // The parameter panel in ui_overlay draws the value that the pixel path
    // is actually using, so the readout can be photographed as evidence.
    //------------------------------------------------------------------
    localparam [26:0] PARAM_FLASH_CYCLES = 27'd50_000_000;   // 1.0 s at 50 MHz
    localparam [26:0] UI_LONG_CYCLES     = 27'd50_000_000;   // 1.0 s hold

    reg [2:0]  bright_level;     // 0 .. 7, level 7 is 100 percent
    reg [2:0]  contrast_level;   // 0 .. 7, level 3 is 1.00
    reg [1:0]  ui_mode;
    reg [26:0] param_timer;
    reg [1:0]  param_sel;        // 0 brightness, 1 contrast, 2 overlay layout

    // A short press is only known once the key is released, so the debounced
    // level is tracked here and a held key fires the layout step instead.
    reg [26:0] ui_hold_cnt;
    reg        ui_button_n_d;
    reg        ui_long_fired;

    wire ui_release   = ui_button_n && !ui_button_n_d;
    wire ui_long_tick = !ui_button_n &&
                        (ui_hold_cnt == (UI_LONG_CYCLES - 27'd1)) &&
                        !ui_long_fired;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            ui_button_n_d <= 1'b1;
            ui_hold_cnt   <= 27'd0;
            ui_long_fired <= 1'b0;
        end else begin
            ui_button_n_d <= ui_button_n;
            if (ui_button_n) begin
                ui_hold_cnt   <= 27'd0;
                ui_long_fired <= 1'b0;
            end else begin
                if (ui_hold_cnt != UI_LONG_CYCLES)
                    ui_hold_cnt <= ui_hold_cnt + 27'd1;
                if (ui_long_tick)
                    ui_long_fired <= 1'b1;
            end
        end
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            bright_level   <= 3'd7;
            contrast_level <= 3'd3;
            ui_mode        <= 2'd0;
            param_timer    <= 27'd0;
            param_sel      <= 2'd0;
        end else if ((FEATURE_BRIGHTNESS == 0) &&
                     (FEATURE_OSD == 0)) begin
            // Keep the pixel path at full gain and the overlay in its base
            // mode when this wrapper does not expose the parameter controls.
            bright_level   <= 3'd7;
            contrast_level <= 3'd3;
            ui_mode        <= 2'd0;
            param_timer    <= 27'd0;
            param_sel      <= 2'd0;
        end else begin
            // The serial host has priority over KEY2, and both paths show
            // the same parameter panel readout.
            if (uart_bright_we && (FEATURE_BRIGHTNESS != 0)) begin
                bright_level <= uart_bright_val;
                param_sel    <= 2'd0;
                param_timer  <= PARAM_FLASH_CYCLES;
            end else if (uart_contrast_we && (FEATURE_BRIGHTNESS != 0)) begin
                contrast_level <= uart_contrast_val;
                param_sel      <= 2'd1;
                param_timer    <= PARAM_FLASH_CYCLES;
            end else if (uart_osd_we && (FEATURE_OSD != 0)) begin
                ui_mode     <= uart_osd_val;
                param_sel   <= 2'd2;
                param_timer <= PARAM_FLASH_CYCLES;
            end else if (bright_pressed && (FEATURE_BRIGHTNESS != 0)) begin
                // KEY2 always changes the parameter KEY4 selected.
                if (param_sel == 2'd1)
                    contrast_level <= (contrast_level == 3'd0) ?
                                      3'd7 : (contrast_level - 3'd1);
                else
                    bright_level   <= (bright_level == 3'd0) ?
                                      3'd7 : (bright_level - 3'd1);
                param_timer <= PARAM_FLASH_CYCLES;
            end else if (ui_long_tick && (FEATURE_OSD != 0)) begin
                ui_mode     <= ui_mode + 2'd1;
                param_sel   <= 2'd2;
                param_timer <= PARAM_FLASH_CYCLES;
            end else if (ui_release && !ui_long_fired &&
                         (FEATURE_OSD != 0)) begin
                param_sel   <= (param_sel == 2'd0) ? 2'd1 : 2'd0;
                param_timer <= PARAM_FLASH_CYCLES;
            end else if (param_timer != 27'd0) begin
                param_timer <= param_timer - 27'd1;
            end
        end
    end

    wire param_flash = (param_timer != 27'd0);

    //------------------------------------------------------------------
    // TF card, sector buffer and FAT32 picture list
    //------------------------------------------------------------------
    wire        sd_init_done, sd_init_error;
    wire [2:0]  sd_init_fail_code;
    wire        sd_sec_read;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;
    wire        sd_sec_read_data_valid, sd_sec_read_end, sd_sec_read_error;
    wire [3:0]  sd_sec_read_error_code;
    wire [4:0]  sd_dbg_state, sd_dbg_fault_state;
    wire        sd_dbg_error_seen;

    sd_card_top #(
        .SPI_LOW_SPEED_DIV  (16'd248),  // about 100 kHz during card init
        .SPI_HIGH_SPEED_DIV (16'd2)     // 8.33 MHz: conservative high-speed read
    ) u_sd_card (
        .clk                    (sys_clk),
        .rst                    (~sys_rst_n),
        .SD_nCS                 (SD_nCS),
        .SD_DCLK                (SD_DCLK),
        .SD_MOSI                (SD_MOSI),
        .SD_MISO                (SD_MISO),
        .sd_init_done           (sd_init_done),
        .sd_init_error          (sd_init_error),
        .init_fail_code         (sd_init_fail_code),
        .sd_sec_read            (sd_sec_read),
        .sd_sec_read_addr       (sd_sec_read_addr),
        .sd_sec_read_data       (sd_sec_read_data),
        .sd_sec_read_data_valid (sd_sec_read_data_valid),
        .sd_sec_read_end        (sd_sec_read_end),
        .sd_sec_read_error      (sd_sec_read_error),
        .sd_sec_read_error_code (sd_sec_read_error_code),
        .dbg_state              (sd_dbg_state),
        .dbg_fault_state        (sd_dbg_fault_state),
        .dbg_error_seen         (sd_dbg_error_seen)
    );

    wire        sec_start, sec_busy, sec_ready, sec_error, sec_release;
    wire [3:0]  sec_error_code;
    wire [31:0] sec_lba;
    wire [8:0]  sec_raddr0, sec_raddr1, sec_raddr2, sec_raddr3;
    wire [7:0]  sec_rdata0, sec_rdata1, sec_rdata2, sec_rdata3;

    sector_reader #(
        .TIMEOUT_CYCLES (24'hFF_FFFF)
    ) u_sector_reader (
        .clk                     (sys_clk),
        .rst_n                   (sys_rst_n),
        .start                   (sec_start),
        .lba                     (sec_lba),
        .busy                    (sec_busy),
        .ready                   (sec_ready),
        .error                   (sec_error),
        .error_code              (sec_error_code),
        .raddr0                  (sec_raddr0),
        .raddr1                  (sec_raddr1),
        .raddr2                  (sec_raddr2),
        .raddr3                  (sec_raddr3),
        .rdata0                  (sec_rdata0),
        .rdata1                  (sec_rdata1),
        .rdata2                  (sec_rdata2),
        .rdata3                  (sec_rdata3),
        .release_req             (sec_release),
        .sd_init_done            (sd_init_done),
        .sd_sec_read             (sd_sec_read),
        .sd_sec_read_addr        (sd_sec_read_addr),
        .sd_sec_read_data        (sd_sec_read_data),
        .sd_sec_read_data_valid  (sd_sec_read_data_valid),
        .sd_sec_read_end         (sd_sec_read_end),
        .sd_sec_read_error       (sd_sec_read_error),
        .sd_sec_read_error_code  (sd_sec_read_error_code)
    );

    wire        scan_busy, scan_done, scan_error;
    wire [3:0]  scan_fault_code;
    wire [7:0]  file_count, file_index;
    wire        ctrl_load_start, ctrl_load_abort;
    wire        fat_load_start, fat_load_abort;
    wire        load_busy, load_done, load_error;
    wire        start_scan, bmp_start;
    wire        img_active, img_reload, frame_abort;
    wire [2:0]  ctrl_state;
    wire        player_cache_ready;
    wire        card_ok, file_error;
    wire [3:0]  fs_fault_code;
    wire [31:0] selected_start_cluster, selected_size;
    wire        voice_found;
    wire [31:0] voice_start_cluster, voice_size;
    wire        fire_voice_found;
    wire        emg_fire_found, emg_exit_found;
    wire [1:0]  ctrl_load_src;
    wire [2:0]  fat_load_src;
    wire        emg_pic;
    wire        emg_imgs_ready;
    wire [7:0]  file_data;
    wire        file_data_valid, fat_data_ready, bmp_data_ready;
    wire [5:0]  fat_state_dbg;

    // voice announcement streaming (card side) --------------------------
    wire        voice_stream_sample_valid, voice_stream_data_ready;
    wire [7:0]  voice_stream_sample_data;
    wire        voice_stream_abort, voice_stream_done, voice_stream_error;
    wire [31:0] voice_stream_rate;
    wire [7:0]  voice_stream_channels, voice_stream_bits;
    wire [31:0] voice_stream_samples;
    wire [3:0]  voice_stream_error_code;
    wire [5:0]  voice_stream_dbg_state;
    reg         voice_stream_started;
    reg         voice_stream_active;
    reg         voice_stream_start_tog, voice_stream_done_tog;
    reg         voice_stream_stop_seen;
    reg         voice_stream_done_latch;
    reg  [31:0] voice_phase_inc;
    reg  [27:0] voice_wdt_cnt;
    reg         voice_wdt_fired;
    wire        voice_start_raw;
    wire        voice_stream_finished;
    wire        voice_cache_ready;
    wire        voice_repeat_ready;
    wire [7:0]  voice_fifo_wr_data, voice_fifo_rd_data;
    wire        voice_fifo_wr_en, voice_fifo_rd_en;
    wire        voice_fifo_full, voice_fifo_empty;
    wire [10:0] voice_fifo_wr_level, voice_fifo_rd_level;
    wire        voice_generating;

    fat32_file_reader #(
        .MAX_FILES (32)
    ) u_fat32 (
        .clk                    (sys_clk),
        .rst_n                  (sys_rst_n),
        .sd_init_done           (sd_init_done),
        .start_scan             (start_scan),
        .scan_busy              (scan_busy),
        .scan_done              (scan_done),
        .scan_error             (scan_error),
        .scan_fault_code        (scan_fault_code),
        .file_count             (file_count),
        .file_index             (file_index),
        .load_src               (fat_load_src),
        .load_start             (fat_load_start),
        .load_abort             (fat_load_abort),
        .load_busy              (load_busy),
        .load_done              (load_done),
        .load_error             (load_error),
        .selected_start_cluster (selected_start_cluster),
        .selected_size          (selected_size),
        .voice_found            (voice_found),
        .voice_start_cluster    (voice_start_cluster),
        .voice_size             (voice_size),
        .fire_voice_found       (fire_voice_found),
        .emg_fire_found         (emg_fire_found),
        .emg_exit_found         (emg_exit_found),
        .file_data              (file_data),
        .file_data_valid        (file_data_valid),
        .file_data_ready        (fat_data_ready),
        .sec_start              (sec_start),
        .sec_lba                (sec_lba),
        .sec_busy               (sec_busy),
        .sec_ready              (sec_ready),
        .sec_error              (sec_error),
        .sec_error_code         (sec_error_code),
        .sec_raddr0             (sec_raddr0),
        .sec_raddr1             (sec_raddr1),
        .sec_raddr2             (sec_raddr2),
        .sec_raddr3             (sec_raddr3),
        .sec_rdata0             (sec_rdata0),
        .sec_rdata1             (sec_rdata1),
        .sec_rdata2             (sec_rdata2),
        .sec_rdata3             (sec_rdata3),
        .sec_release            (sec_release),
        .dbg_state              (fat_state_dbg)
    );

    //------------------------------------------------------------------
    // voice announcement
    //
    // The directory is scanned and the six pictures are cached first.  Once
    // player_ctrl is holding on the resident SDRAM frames, the card reader is
    // free, so VOICE.WAV (the first .WAV entry in the root directory) is read
    // from that point on.  It is compressed to one mu-law byte per source
    // sample and pushed through a small asynchronous FIFO, which lets the
    // audio side run at 12.288 MHz while the card side stays at 50 MHz.
    //
    // A second .WAV entry, FIRE.WAV, carries the fire alarm announcement.  It
    // rides the same parser, FIFO and player, only the source cluster comes
    // from the fire entry (load_src = 4) and the repeat controller runs from a
    // short gap instead of the one-minute interval.  While the alarm is
    // latched the normal announcement stays parked, FIRE.WAV loops, and the
    // built-in chime is silenced.  A card without FIRE.WAV keeps the previous
    // behaviour: alarm screen, buzzer and segment display only.
    //
    // No picture operation uses the card after the cache is full, so changing
    // a picture only changes the displayed SDRAM slot and can neither restart
    // nor interrupt the stream.  Without a .WAV file the built in chime keeps
    // playing.  A successful announcement is repeated after a one-minute gap;
    // a stream error stops the repeat loop and leaves the chime in charge.
    //------------------------------------------------------------------
    // The announcement is optional, so a card stall must not hold the audio
    // path hostage.  The FAT reader has its own short stall timer; this
    // watchdog covers the whole streaming phase and is reset by every
    // compressed sample.
    localparam [27:0] VOICE_WDT_CYCLES = 28'd250_000_000;   // 5 s at 50 MHz

    // Latched flame alarm.  It owns the screen and the board annunciators.
    // The audio path follows: the normal announcement is aborted and, when
    // the card carries FIRE.WAV, the evacuation message takes its place.
    wire emg_lock       = emg_fire;
    // The audio scheduler and the FAT reader have separate busy signals.  A
    // release command can arrive while the fire WAV has stopped at the audio
    // side but the reader is still dropping its final sector.  Treat the card
    // as busy until both blocks are idle so the recovery load pulse is not
    // swallowed by fat32_file_reader.
    wire emg_card_free  = !voice_stream_active && !load_busy && !scan_busy;

    // The card serves one stream at a time, so remember which file the
    // running session carries.  A mode change (alarm latched, alarm cleared)
    // then aborts exactly the session that no longer belongs.
    reg  emg_lock_q;
    reg  voice_stream_is_fire;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            emg_lock_q <= 1'b0;
        else
            emg_lock_q <= emg_lock;
    end

    // The audio-side player keeps running until the FIFO has drained, even
    // after an alarm transition aborts the card-side parser.  Do not let the
    // scheduler start the next file until that drain is complete, otherwise
    // the first bytes of the new announcement are mixed with the old one.
    reg [2:0] voice_generating_sync;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            voice_generating_sync <= 3'd0;
        else
            voice_generating_sync <= {voice_generating_sync[1:0],
                                      voice_generating};
    end

    wire voice_player_idle = ~voice_generating_sync[2];

    wire emg_edge           = emg_lock ^ emg_lock_q;
    // One scheduler drives both announcements.  The live mode decides which
    // file a start pulse asks for: the normal announcement outside the alarm,
    // the evacuation message while the alarm is latched.
    wire voice_start_pulse      = voice_start_raw && !emg_lock;
    wire fire_voice_start_pulse = voice_start_raw &&  emg_lock;
    wire voice_start_any        = voice_start_raw;
    wire voice_mode_changed = voice_stream_active &&
                              (voice_stream_is_fire != emg_lock);

    assign voice_cache_ready = player_cache_ready;
    assign voice_repeat_ready = scan_done && voice_found && voice_cache_ready &&
                                !emg_lock && voice_player_idle;
    // The alarm latch, the alarm release and a parser fault all end the
    // session in progress so the card is released to the other reader.
    assign voice_stream_finished =
        voice_stream_done || voice_stream_error || voice_wdt_fired ||
        voice_mode_changed;

    // The fire announcement only takes the card once the first placard is on
    // screen, both placards are resident (or the second one is unavailable),
    // and no picture load is in flight or queued.  Without the residency
    // check the FIRE.WAV stream wins the reader arbitration on hardware and
    // EMGEXIT.BMP never finishes loading, leaving the alarm stuck on one
    // picture.
    wire fire_voice_ready = emg_lock && emg_pic && emg_imgs_ready &&
                            fire_voice_found && scan_done &&
                            !voice_stream_active && !load_busy &&
                            !img_active && voice_player_idle;

    // Two schedulers would cost a second state machine and counter bank; the
    // device is slice limited, so the gap length is switched instead.  An
    // alarm transition restarts the scheduler together with the parser.
    voice_repeat_ctrl #(
        .GAP_TICK_CYCLES (VOICE_REPEAT_GAP_TICK_CYCLES)
    ) u_voice_repeat (
        .clk          (sys_clk),
        .rst_n        (sys_rst_n && !emg_edge),
        .gap_seconds  (emg_lock ? FIRE_VOICE_GAP_SECONDS
                                : VOICE_REPEAT_GAP_SECONDS),
        .voice_ready  (voice_repeat_ready || fire_voice_ready),
        .stream_done  (voice_stream_done),
        .stream_error (voice_stream_error | voice_wdt_fired),
        .start_pulse  (voice_start_raw),
        .waiting      ()
    );

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            voice_stream_started <= 1'b0;
            voice_stream_active  <= 1'b0;
            voice_stream_is_fire <= 1'b0;
            voice_stream_start_tog <= 1'b0;
            voice_stream_done_tog  <= 1'b0;
            voice_stream_stop_seen <= 1'b0;
            voice_stream_done_latch <= 1'b0;
        end else begin
            if (voice_start_any) begin
                voice_stream_started <= 1'b1;
                voice_stream_active  <= 1'b1;
                voice_stream_is_fire <= fire_voice_start_pulse;
                voice_stream_stop_seen <= 1'b0;
                voice_stream_done_latch <= 1'b0;
                voice_stream_start_tog <= ~voice_stream_start_tog;
            end else if (voice_stream_finished && !voice_stream_stop_seen) begin
                voice_stream_active  <= 1'b0;
                voice_stream_stop_seen <= 1'b1;
                voice_stream_done_tog <= ~voice_stream_done_tog;
            end

            if (voice_stream_done)
                voice_stream_done_latch <= 1'b1;
        end
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            voice_wdt_cnt   <= 28'd0;
            voice_wdt_fired <= 1'b0;
        end else if (voice_start_any) begin
            voice_wdt_cnt   <= 28'd0;
            voice_wdt_fired <= 1'b0;
        end else if (emg_edge) begin
            // A clean slate across an alarm transition: neither the counter
            // nor a stale watchdog flag may block the next announcement.
            voice_wdt_cnt   <= 28'd0;
            voice_wdt_fired <= 1'b0;
        end else if (voice_stream_active) begin
            if (voice_stream_sample_valid)
                voice_wdt_cnt <= 28'd0;
            else if (voice_wdt_cnt != VOICE_WDT_CYCLES)
                voice_wdt_cnt <= voice_wdt_cnt + 28'd1;
            else
                voice_wdt_fired <= 1'b1;
        end else begin
            voice_wdt_cnt <= 28'd0;
        end
    end

    // the byte stream belongs to the voice streamer while it is active
    assign fat_data_ready = voice_stream_active ? voice_stream_data_ready
                                                : bmp_data_ready;
    // FIRE.WAV (source 4) takes precedence over the normal announcement, but
    // both only lift load_start while no picture load owns the card.
    assign fat_load_src   = fire_voice_start_pulse ? 3'd4 :
                            voice_start_pulse      ? 3'd1 : {1'b0, ctrl_load_src};
    assign fat_load_start = fire_voice_start_pulse ? 1'b1 :
                            voice_start_pulse ? 1'b1 :
                            (ctrl_load_start && !voice_stream_active);
    assign fat_load_abort = voice_stream_active ? (voice_stream_abort |
                                                   voice_wdt_fired |
                                                   voice_mode_changed)
                                                : ctrl_load_abort;

    assign voice_fifo_wr_data = voice_stream_sample_data;
    assign voice_fifo_wr_en   = voice_stream_sample_valid && !voice_fifo_full;

    wav_voice_stream u_voice_stream (
        .clk          (sys_clk),
        // Reset on both alarm edges.  That clears the sticky error output, so
        // the restarting repeat controller cannot sample a stale fault, and
        // it drops the parser out of whatever state the aborted session left.
        // Outside those one cycle pulses the parser must stay alive, because
        // the fire announcement streams while the alarm is latched.
        .rst_n        (sys_rst_n && !emg_edge),
        .start        (voice_start_any),
        .stream_done  (load_done),
        .stream_error (load_error | voice_wdt_fired | voice_mode_changed),
        .data         (file_data),
        .data_valid   (file_data_valid),
        .data_ready   (voice_stream_data_ready),
        .abort        (voice_stream_abort),
        .sample_valid (voice_stream_sample_valid),
        .sample_data  (voice_stream_sample_data),
        .sample_ready (~voice_fifo_full),
        .busy         (),
        .done         (voice_stream_done),
        .error        (voice_stream_error),
        .error_code   (voice_stream_error_code),
        .sample_rate  (voice_stream_rate),
        .channels     (voice_stream_channels),
        .bits         (voice_stream_bits),
        .samples_out  (voice_stream_samples),
        .dbg_state    (voice_stream_dbg_state)
    );

    // The playback rate is whatever the file header said.  The increment is
    // 16.16 samples per 48 kHz frame; it is worked out on the system clock and
    // does not change again, so the audio side can simply read it.
    always @* begin
        case (voice_stream_rate)
            32'd8000:  voice_phase_inc = 32'd10923;
            32'd11025: voice_phase_inc = 32'd15053;
            32'd16000: voice_phase_inc = 32'd21845;
            32'd22050: voice_phase_inc = 32'd30106;
            32'd24000: voice_phase_inc = 32'd32768;
            32'd32000: voice_phase_inc = 32'd43691;
            32'd44100: voice_phase_inc = 32'd60211;
            32'd48000: voice_phase_inc = 32'd65536;
            default:   voice_phase_inc = 32'd10923;
        endcase
    end

    //------------------------------------------------------------------
    // BMP file to pixel stream
    //------------------------------------------------------------------
    wire        bmp_busy, bmp_done, bmp_error;
    wire [7:0]  bmp_error_code;
    wire [23:0] bmp_rgb;
    wire        bmp_px_valid, bmp_px_last, bmp_px_ready;
    wire [15:0] bmp_width, bmp_height, bmp_bpp;
    wire [31:0] bmp_data_offset;

    // picture that actually reaches the frame store loader: either the parsed
    // stream itself (FEATURE_SCALE = 0) or the scaler output
    wire        ld_px_valid, ld_px_ready;
    wire [23:0] ld_rgb;
    wire        scale_busy;

    bmp_stream_parser #(
        .IMG_W    (640),
        .IMG_H    (480),
        .SCALE_EN (FEATURE_SCALE),
        .MAX_W    (1920),
        .MAX_H    (2048)
    ) u_bmp (
        .clk             (sys_clk),
        .rst_n           (sys_rst_n),
        .start           (bmp_start),
        .file_size       (selected_size),
        .busy            (bmp_busy),
        .done            (bmp_done),
        .error           (bmp_error),
        .error_code      (bmp_error_code),
        .data            (file_data),
        .data_valid      (file_data_valid),
        .data_ready      (bmp_data_ready),
        .hold_done       (scale_busy),
        .px_valid        (bmp_px_valid),
        .px_rgb          (bmp_rgb),
        .px_last         (bmp_px_last),
        .px_ready        (bmp_px_ready),
        .hdr_width       (bmp_width),
        .hdr_height      (bmp_height),
        .hdr_bpp         (bmp_bpp),
        .hdr_data_offset (bmp_data_offset)
    );

    //------------------------------------------------------------------
    // picture size adaptation
    //
    // FEATURE_SCALE = 0 keeps the historical path: the parser only accepts
    // 640 x 480 and the frame store is fed directly.  With the feature on,
    // the scaler rebuilds every source row into a 640 entry line buffer and
    // replays it as often as the vertical mapping needs, so the frame store
    // still receives exactly 640 x 480 pixels in bottom up order.
    //------------------------------------------------------------------
    generate
        if (FEATURE_SCALE == 0) begin : g_no_scale
            assign ld_px_valid  = bmp_px_valid;
            assign ld_rgb       = bmp_rgb;
            assign bmp_px_ready = ld_px_ready;
            assign scale_busy   = 1'b0;
        end else begin : g_scale
            bmp_scaler #(
                .DST_W (640),
                .DST_H (480),
                .BRAM_PRIM (1)
            ) u_scaler (
                .clk       (sys_clk),
                .rst_n     (sys_rst_n),
                // a new file starts, or the running one is dropped
                .frame_rst (bmp_start | frame_abort),
                .src_w     (bmp_width),
                .src_h     (bmp_height),
                .s_valid   (bmp_px_valid),
                .s_rgb     (bmp_rgb),
                .s_last    (bmp_px_last),
                .s_ready   (bmp_px_ready),
                .m_valid   (ld_px_valid),
                .m_rgb     (ld_rgb),
                .m_ready   (ld_px_ready),
                .busy      (scale_busy),
                .dbg_row   ()
            );
        end
    endgenerate

    //------------------------------------------------------------------
    // sequencer
    //------------------------------------------------------------------
    wire        frame_preload_ready, frame_ctrl_swap_req, frame_swap_done;
    wire        frame_store_swap_req;
    wire        frame_stored_pulse;

    player_ctrl #(
        .HOLD_CYCLES  (AUTO_HOLD_CYCLES),
        .RETRY_CYCLES (50_000_000),
        .LOAD_CYCLES  (LOAD_CYCLES),    // 20 s by default, the scaling
                                        // wrapper asks for more
        .SCAN_CYCLES  (200_000_000),    // 4 s
        .PRELOAD      (0),
        .CACHE_MODE   (1),
        .CACHE_FRAMES (6)
    ) u_ctrl (
        .clk            (sys_clk),
        .rst_n          (sys_rst_n),
        // The flame alarm freezes the slideshow: the key, the automatic
        // rotation and every host jump are ignored until it clears.
        .key_pressed    (mode_pressed && !emg_lock),
        .auto_rotate_en (auto_rotate_en && !emg_lock),
        .jump_req       (uart_jump_req && !emg_lock),
        .jump_slot      (uart_jump_slot),
        .emg_mode       (emg_lock),
        .emg_card_free  (emg_card_free),
        .emg_fire_found (emg_fire_found),
        .emg_exit_found (emg_exit_found),
        .stored_buf     (stored_buf),
        .emg_pic        (emg_pic),
        .emg_imgs_ready (emg_imgs_ready),
        .emg_slot       (ctrl_emg_slot),
        .emg_slot_sel   (ctrl_emg_slot_sel),
        .load_src       (ctrl_load_src),
        .sd_init_done   (sd_init_done),
        .start_scan     (start_scan),
        // The voice stream starts only after the six resident frames have
        // been cached, so the picture sequencer can see the scan result
        // immediately.
        .scan_done      (scan_done),
        .scan_error     (scan_error),
        .scan_fault_code(scan_fault_code),
        .file_count     (file_count),
        .file_index     (file_index),
        .load_start     (ctrl_load_start),
        .load_abort     (ctrl_load_abort),
        .load_busy      (load_busy),
        .load_done      (load_done),
        .load_error     (load_error),
        .bmp_start      (bmp_start),
        .bmp_done       (bmp_done),
        .bmp_error      (bmp_error),
        .img_active     (img_active),
        .frame_abort    (frame_abort),
        .reload         (img_reload),
        .preload_ready  (frame_preload_ready),
        .swap_req       (frame_ctrl_swap_req),
        .swap_done      (frame_swap_done),
        .swap_target_en (uart_swap_target_en),
        .swap_target    (uart_swap_target),
        .stored_pulse   (frame_stored_pulse),
        .state_dbg      (ctrl_state),
        .card_ok        (card_ok),
        .file_error     (file_error),
        .fs_fault_code  (fs_fault_code),
        .cache_ready    (player_cache_ready)
    );

    //------------------------------------------------------------------
    // SDRAM frame buffer
    //------------------------------------------------------------------
    wire [23:0] fifo_rgb;
    wire        sdram_clk;
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire        sdram_cs_n, sdram_cke;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;

    wire        frame_ready, img_frame_ready, frame_loaded;
    wire        init_done, cal_fail, pix_underflow;
    wire [2:0]  ld_state;
    wire [5:0]  cache_ready;
    wire [2:0]  stored_buf, display_buf;
    wire [11:0] display_width, display_height;
    wire        display_size_valid;
    wire [4:0]  fade_level;
    wire        fade_issue_swap;
    reg         pix_frame_tog;

    // one toggle per video frame, used by the frame buffer to keep the image
    // frame counter locked to the video frame boundary
    always @(posedge pixel_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            pix_frame_tog <= 1'b0;
        else if (frame_tick)
            pix_frame_tog <= ~pix_frame_tog;
    end

    frame_store #(
        .CACHE_MODE (1),
        .BUF_COUNT  (6),
        .BUF_STRIDE (21'h050000),
        .DEFER_SWAP (0)
    ) u_frame_store (
        .clk            (sys_clk),
        .mem_clk        (mem_clk),
        .mem_clk_sft    (mem_clk_sft),
        .rst_n          (sys_rst_n),
        .reload         (img_reload),
        .pattern_mode   (2'd0),
        .img_active     (img_active),
        .img_abort      (frame_abort),
        .swap_req       (frame_store_swap_req),
        .emg_slot       (ctrl_emg_slot),
        .emg_slot_sel   (ctrl_emg_slot_sel),
        .swap_target_en (uart_swap_target_en),
        .swap_target    (uart_swap_target),
        .img_valid      (ld_px_valid),
        .img_rgb        (ld_rgb),
        .img_width      (bmp_width[11:0]),
        .img_height     (bmp_height[11:0]),
        .img_ready      (ld_px_ready),
        .frame_ready    (frame_ready),
        .img_frame_ready(img_frame_ready),
        .frame_loaded   (frame_loaded),
        .preload_ready  (frame_preload_ready),
        .swap_done      (frame_swap_done),
        .cache_ready    (cache_ready),
        .stored_buf     (stored_buf),
        .display_buf    (display_buf),
        .display_width  (display_width),
        .display_height (display_height),
        .display_size_valid (display_size_valid),
        .stored_pulse   (frame_stored_pulse),
        .init_done      (init_done),
        .cal_fail       (cal_fail),
        .cal_latency    (),
        .state_dbg      (ld_state),
        .sdram_clk      (sdram_clk),
        .sdram_ras_n    (sdram_ras_n),
        .sdram_cas_n    (sdram_cas_n),
        .sdram_we_n     (sdram_we_n),
        .sdram_addr     (sdram_addr),
        .sdram_ba       (sdram_ba),
        .sdram_cs_n     (sdram_cs_n),
        .sdram_cke      (sdram_cke),
        .sdram_dm       (sdram_dm),
        .sdram_dq       (sdram_dq),
        .pix_clk        (pixel_clk),
        .pix_de         (video_active),
        .pix_x          (px),
        .pix_y          (py),
        .pix_frame_tog  (pix_frame_tog),
        .pix_rgb        (fifo_rgb),
        .pix_underflow  (pix_underflow)
    );

    //------------------------------------------------------------------
    // picture transition
    //
    // player_ctrl produces its swap request in sys_clk.  A toggle is used
    // instead of a one-cycle pulse so the pixel-domain fade controller can
    // never miss it.  After the fade reaches black, the controller sends a
    // request back to the frame store in sys_clk through the same handshake.
    //------------------------------------------------------------------
    reg        swap_req_tog_sys;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            swap_req_tog_sys <= 1'b0;
        else if (frame_ctrl_swap_req)
            swap_req_tog_sys <= ~swap_req_tog_sys;
    end

    reg [2:0] swap_req_sync_pix;

    always @(posedge pixel_clk or posedge rst_video) begin
        if (rst_video)
            swap_req_sync_pix <= 3'd0;
        else
            swap_req_sync_pix <= {swap_req_sync_pix[1:0], swap_req_tog_sys};
    end

    wire fade_request_pulse = swap_req_sync_pix[2] ^ swap_req_sync_pix[1];

    // frame_store's display-side reader runs on pix_clk, so frame_loaded is
    // already a pixel-clock pulse.  fade_ctrl runs on the same pixel clock;
    // keep this path synchronous.  Routing it through a sys_clk source
    // synchroniser can sample the 40 ns pulse twice and cancel the toggle,
    // which leaves player_ctrl waiting for swap_done and makes KEY1 appear
    // unresponsive after the first failed transition.
    wire image_change_pulse = frame_loaded;

    fade_ctrl #(
        .STEP         (FADE_STEP),
        .WAIT_TIMEOUT (FADE_WAIT_TIMEOUT)
    ) u_fade_ctrl (
        .clk                 (pixel_clk),
        .rst                 (rst_video),
        .frame_tick          (frame_tick),
        .request_pulse       (fade_request_pulse),
        .issue_swap          (fade_issue_swap),
        .image_change_pulse  (image_change_pulse),
        .level               (fade_level)
    );

    reg store_swap_tog_pix;

    always @(posedge pixel_clk or posedge rst_video) begin
        if (rst_video)
            store_swap_tog_pix <= 1'b0;
        else if (fade_issue_swap)
            store_swap_tog_pix <= ~store_swap_tog_pix;
    end

    reg [2:0] store_swap_sync_sys;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            store_swap_sync_sys <= 3'd0;
        else
            store_swap_sync_sys <= {store_swap_sync_sys[1:0],
                                    store_swap_tog_pix};
    end

    // This is the delayed request that the frame-store sequencer sees.
    // With the transition feature disabled, the player's frame-boundary
    // request goes straight to the frame store and the fade controller has
    // no effect on the displayed image.
    assign frame_store_swap_req = (FEATURE_FADE != 0) ?
        (store_swap_sync_sys[2] ^ store_swap_sync_sys[1]) :
        frame_ctrl_swap_req;

    // on die SDRAM, the pins are handled by the PHY macro.  The controller
    // hands out the SDRAM clock it was designed to drive, so the macro uses
    // that output instead of a locally generated clock.
    EG_PHY_SDRAM_2M_32 u_sdram (
        .clk   (sdram_clk),
        .ras_n (sdram_ras_n),
        .cas_n (sdram_cas_n),
        .we_n  (sdram_we_n),
        .addr  (sdram_addr),
        .ba    (sdram_ba),
        .dq    (sdram_dq),
        .cs_n  (sdram_cs_n),
        .dm0   (sdram_dm[0]),
        .dm1   (sdram_dm[1]),
        .dm2   (sdram_dm[2]),
        .dm3   (sdram_dm[3]),
        .cke   (sdram_cke)
    );

    //------------------------------------------------------------------
    // HDMI transmitter: the official HDMI 1.4b core, audio included
    //
    // The frame buffer hands out 24 bit pixels aligned with a one pixel
    // delayed data enable, and the video timing generator runs the same
    // 800 x 525 / 96 / 2 blanking the core is configured for, so the pixel
    // stream can be pushed into the core as it is.  The core regenerates
    // hsync / vsync itself, only the start of frame marker and the data
    // enable are needed on this side.
    //------------------------------------------------------------------
    // User brightness.  Level 7 is 100 percent, level 0 is about 13 percent;
    // the same 5-bit gain also carries the fade level from fade_ctrl.
    function [4:0] user_gain;
        input [2:0] level;
        begin
            case (level)
                3'd0: user_gain = 5'd4;
                3'd1: user_gain = 5'd8;
                3'd2: user_gain = 5'd12;
                3'd3: user_gain = 5'd16;
                3'd4: user_gain = 5'd20;
                3'd5: user_gain = 5'd24;
                3'd6: user_gain = 5'd28;
                default: user_gain = 5'd31;
            endcase
        end
    endfunction

    function [7:0] scale_channel;
        input [7:0] c;
        input [4:0] gain;
        reg [12:0] product;
        begin
            product = c * gain;
            scale_channel = (product + 13'd31) >> 5;
        end
    endfunction

    // User contrast (advanced item 4).  The channel is stretched around mid
    // grey: out = (in - 128) * k / 16 + 128, clamped to 0 .. 255.  Level 3 is
    // k = 16 and is bit exact 1.00, level 0 is 0.625 and level 7 is 1.50.
    function [4:0] user_contrast;
        input [2:0] level;
        begin
            case (level)
                3'd0: user_contrast = 5'd10;
                3'd1: user_contrast = 5'd12;
                3'd2: user_contrast = 5'd14;
                3'd3: user_contrast = 5'd16;
                3'd4: user_contrast = 5'd18;
                3'd5: user_contrast = 5'd20;
                3'd6: user_contrast = 5'd22;
                default: user_contrast = 5'd24;
            endcase
        end
    endfunction

    function [7:0] contrast_channel;
        input [7:0] c;
        input [4:0] k;
        reg signed [9:0]  d;
        reg signed [15:0] p;
        integer t;
        begin
            d = $signed({2'b00, c}) - 10'sd128;
            p = d * $signed({1'b0, k});
            t = (p + 16'sd8) >>> 4;
            t = t + 128;
            if (t < 0)
                contrast_channel = 8'd0;
            else if (t > 255)
                contrast_channel = 8'd255;
            else
                contrast_channel = t[7:0];
        end
    endfunction

    // The two control levels are updated in the 50 MHz system-clock domain.
    // Sample both in the pixel domain before they reach the pixel arithmetic.
    reg [2:0] bright_level_s1, bright_level_s2;
    reg [2:0] contrast_level_s1, contrast_level_s2;

    always @(posedge pixel_clk or posedge rst_video) begin
        if (rst_video) begin
            bright_level_s1   <= 3'd7;
            bright_level_s2   <= 3'd7;
            contrast_level_s1 <= 3'd3;
            contrast_level_s2 <= 3'd3;
        end else begin
            bright_level_s1   <= bright_level;
            bright_level_s2   <= bright_level_s1;
            contrast_level_s1 <= contrast_level;
            contrast_level_s2 <= contrast_level_s1;
        end
    end

    wire [4:0] user_gain_now = user_gain(bright_level_s2);
    wire [4:0] user_contrast_now = user_contrast(contrast_level_s2);
    wire [23:0] cont_rgb = {
        contrast_channel(fifo_rgb[23:16], user_contrast_now),
        contrast_channel(fifo_rgb[15:8],  user_contrast_now),
        contrast_channel(fifo_rgb[7:0],   user_contrast_now)
    };
    // A wrapper that does not enable the transition feature bypasses the
    // fade controller and always uses full fade gain.
    wire [4:0] fade_level_eff = (FEATURE_FADE != 0) ? fade_level : 5'd31;
    wire [9:0] gain_product = fade_level_eff * user_gain_now;
    wire [4:0] total_gain = (gain_product + 10'd31) >> 5;
    wire [23:0] faded_rgb = {
        scale_channel(cont_rgb[23:16], total_gain),
        scale_channel(cont_rgb[15:8],  total_gain),
        scale_channel(cont_rgb[7:0],   total_gain)
    };

    wire [23:0] ui_rgb;
    wire        ui_hit;
    wire [23:0] hdmi_rgb;

    // The LED status word remains available for fault diagnosis.  The large
    // on-screen panel is intentionally not built in the final ADV13 bitstream
    // because the flight-board renderer needs the remaining slice budget.
    reg [3:0]  diag_code;
    reg [2:0]  fault_phase;

    //------------------------------------------------------------------
    // audio master clock for the S/PDIF style bit clock of the data island
    //
    // 50 MHz / 2 * 3 = 75 MHz on C0 (which is also the feedback output the
    // vendor IP is generated for), 750 MHz / 61 = 12.295 MHz on C2.  One I2S
    // frame is 64 bit clocks, so the sample rate is 12.295 MHz / 256 =
    // 48.03 kHz, the value the vendor example runs at.
    //------------------------------------------------------------------
    // The audio tone generator waits for the same PLL lock.
    wire audio_rst = ~sys_rst_n | ~audio_pll_lock;

    //------------------------------------------------------------------
    // airport chime loop and the picture change cue
    //
    // The background voice is a continuous airport announcement chime loop:
    // nothing in the video path may restart it, so there is no reset or
    // restart input here at all.  A picture change only fires a short, quiet
    // announcement tick that is mixed over the running chime.
    //
    // frame_swap_done is one sys_clk cycle at the moment the display really
    // accepted the new buffer, for a KEY1 press and for the automatic
    // advance, so the chime lands exactly on the visible change.  It is
    // turned into a toggle and resynchronised instead of being sampled as a
    // pulse in the audio clock domain.
    //------------------------------------------------------------------
    reg cue_tog;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            cue_tog <= 1'b0;
        else if (frame_swap_done)
            cue_tog <= ~cue_tog;
    end

    reg [2:0] cue_sync;

    always @(posedge audio_mclk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            cue_sync <= 3'd0;
        else
            cue_sync <= {cue_sync[1:0], cue_tog};
    end

    wire audio_cue = cue_sync[2] ^ cue_sync[1];

    wire audio_i2s_bclk, audio_i2s_lrck, audio_i2s_dout;

    //------------------------------------------------------------------
    // voice announcement playback
    //
    // The card side writes one compressed source sample at a time into a
    // small asynchronous FIFO.  The audio side interpolates those samples to
    // the 48 kHz frame rate and mutes the built-in chime while the stream owns
    // the output.
    //------------------------------------------------------------------
    wire      voice_frame_tick;
    wire signed [23:0] voice_sample;

    async_fifo #(
        .DW (8),
        .AW (10)                 // 1024 byte mu-law buffer
    ) u_voice_fifo (
        .wr_clk   (sys_clk),
        .wr_rst_n (sys_rst_n),
        .wr_en    (voice_fifo_wr_en),
        .wr_data  (voice_fifo_wr_data),
        .wr_full  (voice_fifo_full),
        .wr_level (voice_fifo_wr_level),

        .rd_clk   (audio_mclk),
        .rd_rst_n (sys_rst_n),
        .rd_en    (voice_fifo_rd_en),
        .rd_data  (voice_fifo_rd_data),
        .rd_empty (voice_fifo_empty),
        .rd_level (voice_fifo_rd_level)
    );

    voice_player_stream #(
        // mu-law tops out at 32124, so a shift of 7 puts the loudest sample
        // at about -6 dBFS, the level the announcement needs to be heard over
        // the panel speaker
        .GAIN_SHIFT (7)
    ) u_voice_player (
        .clk        (audio_mclk),
        .rst        (audio_rst),
        .start_tog  (voice_stream_start_tog),
        .done_tog   (voice_stream_done_tog),
        .frame_tick (voice_frame_tick),
        .phase_inc  (voice_phase_inc),
        .fifo_data  (voice_fifo_rd_data),
        .fifo_empty (voice_fifo_empty),
        .fifo_rd_en (voice_fifo_rd_en),
        .sample     (voice_sample),
        .generating (voice_generating)
    );

    hdmi_audio_tone_i2s_64fs #(
        .BGM_SHIFT   (15),              // triangle peak 64 << 15 = 2.097 M
        .CUE_SHIFT   (14),              // cue 6 dB under the chime loop
        .UNIT_FRAMES (2400),            // 50 ms per unit -> 2.4 s loop
        .CUE_FRAMES  (9600)             // 0.2 s chime
    ) u_audio_tone (
        .I_mclk       (audio_mclk),
        .I_rst        (audio_rst),
        .I_cue        (audio_cue),
        .I_voice_en   (voice_generating),
        .I_voice      (voice_sample),
        // Keep the chime and the normal announcement silent while the alarm
        // is latched, but let the FIRE.WAV evacuation message through: the
        // mute only applies while that stream has nothing to play.
        .I_mute       (emg_lock && !voice_generating),
        .O_sample_tick(voice_frame_tick),
        .O_i2s_BCLK   (audio_i2s_bclk),
        .O_i2s_LRCK   (audio_i2s_lrck),
        .O_i2s_DOUT   (audio_i2s_dout)
    );

    //------------------------------------------------------------------
    // sample capture and ACR packet values
    //
    // The vendor receiver samples the I2S pins with the pixel clock, which is
    // four times the 6.15 MHz bit clock, and hands one 24 bit sample per
    // channel to the transmitter core.  audio_arc_calculate counts the pixels
    // between two ACR packets, that is 48 samples / 48 kHz = 1 ms = 25000
    // pixels at 25 MHz.
    //------------------------------------------------------------------
    wire        audio_valid;
    wire [23:0] audio_left, audio_right;
    wire        acr_valid;
    wire [19:0] acr_cts, acr_n;

    I2S_receiver u_i2s_receiver (
        .I_clk              (pixel_clk),
        .I_rst              (audio_rst),    // not used inside the vendor block
        .I_i2s_BCLK         (audio_i2s_bclk),
        .I_i2s_LRCK         (audio_i2s_lrck),
        .I_i2s_DOUT         (audio_i2s_dout),
        .O_audio_valid      (audio_valid),
        .O_audio_left_data  (audio_left),
        .O_audio_right_data (audio_right)
    );

    //------------------------------------------------------------------
    // advanced visual layer: subtitle, OSD and audio waveform
    //------------------------------------------------------------------
    ui_overlay u_ui_overlay (
        .pixel_clk    (pixel_clk),
        .rst          (rst_video),
        .frame_tick   (frame_tick),
        .de           (de_d),
        .px           (px_d),
        .py           (py_d),
        .base_rgb     (faded_rgb),
        .ui_enable    (img_frame_ready),
        .img_slot     (display_buf),
        .img_count    (file_count),
        .img_width    (display_width),
        .img_height   (display_height),
        .img_size_valid(display_size_valid),
        .user_bright  (bright_level),
        .user_contrast (contrast_level),
        .ui_mode      (ui_mode),
        .param_flash  (param_flash),
        .param_sel    (param_sel),
        .auto_mode    (auto_rotate_en),
        .emg_fire     (emg_fire),
        .emg_pic      (emg_pic),
        .logo_enable  (FEATURE_LOGO != 0),
        .subtitle_enable (FEATURE_SUBTITLE != 0),
        .timestamp_enable (FEATURE_TIMESTAMP != 0),
        .parallax_enable (FEATURE_PARALLAX != 0),
        .osd_enable   (FEATURE_OSD != 0),
        .wave_enable  (FEATURE_AUDIO_WAVE != 0),
        .host_text_tog       (uart_text_tog),
        .host_text_addr      (uart_text_addr),
        .host_text_data      (uart_text_data),
        .host_text_commit_tog(uart_text_commit_tog),
        .host_text_clear_tog (uart_text_clear_tog),
        .host_link           (uart_link_seen),
        .host_time_hh        (uart_time_hh),
        .host_time_mm        (uart_time_mm),
        .host_time_ss        (uart_time_ss),
        .host_time_tog       (uart_time_tog),
        .rtc_time_hh         (rtc_hour_bcd),
        .rtc_time_mm         (rtc_min_bcd),
        .rtc_time_ss         (rtc_sec_bcd),
        .rtc_time_tog        (rtc_time_tog),
        .board_enable        (uart_board_enable),
        .board_wr_tog        (uart_board_tog),
        .board_wr_addr       (uart_board_addr),
        .board_wr_data       (uart_board_data),
        .audio_rst    (audio_rst),
        .audio_valid  (audio_valid),
        .audio_left   (audio_left),
        .rgb          (ui_rgb),
        .hit          (ui_hit)
    );

    assign hdmi_rgb = ((FEATURE_UI != 0) && ui_hit) ? ui_rgb : faded_rgb;

    audio_arc_calculate #(
        .ACR_N (6144)
    ) u_audio_arc (
        .I_clk         (pixel_clk),
        .I_rst         (audio_rst),
        .I_audio_valid (audio_valid),
        .O_acr_valid   (acr_valid),
        .O_acr_cts     (acr_cts),
        .O_acr_n       (acr_n)
    );

    //------------------------------------------------------------------
    // RGB / DE to 24 bit AXIS
    //------------------------------------------------------------------
    wire        axis_s_user, axis_s_valid, axis_s_last, axis_s_ready;
    wire [23:0] axis_s_data;

    video_rgb_to_axis_640x480 u_video_to_axis (
        .I_clk         (pixel_clk),
        .I_rst         (rst_video),
        .I_vs          (vsync_d),
        .I_de          (de_d),
        .I_rgb         (hdmi_rgb),
        .O_video_user  (axis_s_user),
        .O_video_valid (axis_s_valid),
        .O_video_last  (axis_s_last),
        .O_video_data  (axis_s_data)
    );

    //------------------------------------------------------------------
    // HDMI 1.4b transmitter core and its TMDS serialisers
    //------------------------------------------------------------------
    wire        edid_trig, edid_valid;
    wire [7:0]  edid_data;
    wire [9:0]  tmds_ch0, tmds_ch1, tmds_ch2, tmds_clk;

    // The vendor example fires the EDID read once after configuration, so a
    // display that is switched on later is never probed.  Keep asking once a
    // second until the first byte comes back.
    edid_retry #(
        .RETRY_CYCLES (25'd24_999_999)      // 1 s at 25 MHz
    ) u_edid_retry (
        .clk        (pixel_clk),
        .rst        (rst_video),
        .edid_valid (edid_valid),
        .trig       (edid_trig)
    );

    hdmi_1_4b_transmitter_core_wrapper #(
        .DEVICE            ("EG"    ),
        .HTOTAL            (800     ),
        .HSA               (96      ),
        .HFP               (16      ),
        .HBP               (48      ),
        .HACTIVE           (640     ),
        .VTOTAL            (525     ),
        .VSA               (2       ),
        .VFP               (10      ),
        .VBP               (33      ),
        .VACTIVE           (480     ),
        .VIDEO_TPG         ("Disable"),
        .VIDEO_FORMAT      ("RGB"   ),
        .VIDEO_VIC         (1       ),
        .AUDIO_SAMPLE_RATE ("48K"   ),
        .IIC_SCL_DIV       (250     )
    ) u_hdmi_core (
        .I_pixel_clk        (pixel_clk),
        .I_rst              (rst_video),
        .I_edid_read_trig   (edid_trig),
        .O_edid_read_valid  (edid_valid),
        .O_edid_read_data   (edid_data),

        .I_axis_s_user      (axis_s_user),
        .I_axis_s_valid     (axis_s_valid),
        .I_axis_s_last      (axis_s_last),
        .I_axis_s_data      (axis_s_data),
        .O_axis_s_ready     (axis_s_ready),

        .I_audio_valid      (audio_valid),
        .I_audio_left_data  (audio_left),
        .I_audio_right_data (audio_right),
        .I_acr_valid        (acr_valid),
        .I_acr_cts          (acr_cts),
        .I_acr_n            (acr_n),

        .O_video_locked     (),
        .O_ddc_scl          (HDMI_DDC_SCL),
        .IO_ddc_sda         (HDMI_DDC_SDA),

        .O_ch0_tmds_data    (tmds_ch0),
        .O_ch1_tmds_data    (tmds_ch1),
        .O_ch2_tmds_data    (tmds_ch2),
        .O_clk_tmds_data    (tmds_clk)
    );

    hdmi_phy_wrapper #(
        .DEVICE ("EG")
    ) u_hdmi_phy (
        .I_pixel_clk        (pixel_clk),
        .I_serial_clk       (pixel_clk_5x),
        .I_rst              (rst_video),
        .I_tmds_channel_0   (tmds_ch0),
        .I_tmds_channel_1   (tmds_ch1),
        .I_tmds_channel_2   (tmds_ch2),
        .I_tmds_channel_clk (tmds_clk),
        .O_tmds_ch0_p       (HDMI_D0_P),
        .O_tmds_ch1_p       (HDMI_D1_P),
        .O_tmds_ch2_p       (HDMI_D2_P),
        .O_tmds_clk_p       (HDMI_CLK_P)
    );

    //------------------------------------------------------------------
    // status LEDs
    //   LED1 alive, LED2 a frame is in the buffer
    //   LED3 a card or file problem, LED4 SDRAM or pixel underflow
    //------------------------------------------------------------------
    reg [22:0] heartbeat;
    reg [26:0] startup_blink;
    reg [24:0] key_flash;

    // Diagnostic-bitstream signature: one long flash, one dark gap, then a
    // second long flash.  This is deliberately different from the single
    // 0.5 s flash in the previous bitstreams, so an old download cannot be
    // mistaken for the SD-state-diagnostic build.
    localparam [26:0] STARTUP_ON1_END = 27'd25_000_000; // 0.5 s
    localparam [26:0] STARTUP_OFF_END = 27'd50_000_000; // 1.0 s
    localparam [26:0] STARTUP_ON2_END = 27'd75_000_000; // 1.5 s

    always @(posedge pixel_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            heartbeat <= 23'd0;
        else
            heartbeat <= heartbeat + 1'b1;
    end

    // A short all-LED flash makes it obvious which bitstream is running.
    // It lasts 0.5 s at 50 MHz and then releases the LEDs to the status code.
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            startup_blink <= 25'd0;
        else if (startup_blink < STARTUP_ON2_END)
            startup_blink <= startup_blink + 27'd1;
    end

    // A visible acknowledgement for every key press.  It stays on for 0.5 s
    // even when the next BMP is still being read from the TF card, so a slow
    // picture change cannot be mistaken for a missed key press, and the
    // parameter keys below the screen are mirrored on LED1 as well.
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            key_flash <= 25'd0;
        else if (mode_pressed || bright_pressed || ui_pressed)
            key_flash <= 25'd25_000_000;
        else if (key_flash != 25'd0)
            key_flash <= key_flash - 25'd1;
    end

    // Board diagnostic mapping for the current bring-up bit.  LED1 is
    // reused as the low bit once a stable code exists, so the four LEDs
    // display LED4:LED3:LED2:LED1 as a binary status word:
    //   0000 initialization or scanning still in progress (LED1 heartbeat)
    //   0001 SD initialization complete
    //   0010 a real BMP frame has been written into SDRAM
    //   0011 MBR/VBR/FAT32/root-directory structure error
    //   0100 no BMP files found in the root directory
    //   0101 directory scan watchdog timeout
    //   0110 sector-read error or timeout
    //   0111 file data stream or FAT-chain read error
    //   1000 BMP format or header error
    //   1001 file-load watchdog timeout
    //   1010 CMD17 was rejected by the card
    //   1011 CMD17 data-token timeout
    //   1100 CMD0 failed, card did not enter SPI mode
    //        (or sector data length mismatch after SD init)
    //   1101 CMD8 failed, no usable v2 card response; after successful
    //        initialization this code also means CMD17 got no response
    //   1110 CMD55 or ACMD41 failed, card stayed idle
    //   1111 CMD58 or CMD16 failed, OCR/block-length stage failed
    //
    // Debug override: for the generic sector timeout (fs_fault_code=4),
    // cycle through three independent fault codes instead of showing one
    // ambiguous value.  The sequence is:
    //   1) sector error code
    //   2) SD controller state at the first sector error
    //   3) file-system fault code
    // A short all-off gap separates the values so they cannot be merged.
    reg [3:0]  sd_state_at_fault;
    reg [3:0]  sec_code_at_fault;
    reg        sec_error_q;
    reg        sec_busy_q;
    reg [29:0] fault_phase_cnt;

    localparam [29:0] FAULT_CODE_CYCLES  = 30'd100_000_000; // 2.0 s
    localparam [29:0] FAULT_BLANK_CYCLES = 30'd25_000_000;  // 0.5 s
    localparam [29:0] FAULT_LONG_BLANK   = 30'd175_000_000; // 3.5 s
    localparam [29:0] FAULT_ALL_CYCLES   = 30'd525_000_000; // 10.5 s

    wire fault_cycle_active =
        file_error && (fs_fault_code == 4'd4) &&
        (startup_blink >= STARTUP_ON2_END);

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            fault_phase_cnt <= 30'd0;
        else if (!fault_cycle_active)
            fault_phase_cnt <= 30'd0;
        else if (fault_phase_cnt == FAULT_ALL_CYCLES)
            fault_phase_cnt <= 30'd0;
        else
            fault_phase_cnt <= fault_phase_cnt + 30'd1;
    end

    always @* begin
        fault_phase = 3'd0;
        if (fault_phase_cnt < FAULT_CODE_CYCLES)
            fault_phase = 3'd0;
        else if (fault_phase_cnt <
                 (FAULT_CODE_CYCLES + FAULT_BLANK_CYCLES))
            fault_phase = 3'd1;
        else if (fault_phase_cnt <
                 (FAULT_CODE_CYCLES + FAULT_BLANK_CYCLES +
                  FAULT_CODE_CYCLES))
            fault_phase = 3'd2;
        else if (fault_phase_cnt <
                 (FAULT_CODE_CYCLES + FAULT_BLANK_CYCLES +
                  FAULT_CODE_CYCLES + FAULT_BLANK_CYCLES))
            fault_phase = 3'd3;
        else if (fault_phase_cnt <
                 (FAULT_CODE_CYCLES + FAULT_BLANK_CYCLES +
                  FAULT_CODE_CYCLES + FAULT_BLANK_CYCLES +
                  FAULT_CODE_CYCLES))
            fault_phase = 3'd4;
        else
            fault_phase = 3'd5;
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sd_state_at_fault <= 4'd0;
            sec_code_at_fault  <= 4'd0;
            sec_error_q       <= 1'b0;
            sec_busy_q        <= 1'b0;
        end else begin
            sec_error_q <= sec_error;
            sec_busy_q  <= sec_busy;
            if (sec_error && !sec_error_q) begin
                if (sec_busy_q) begin
                    sec_code_at_fault  <= sec_error_code;
                    sd_state_at_fault  <= sd_dbg_error_seen ?
                                          sd_dbg_fault_state[3:0] :
                                          sd_dbg_state[3:0];
                end else begin
                    sec_code_at_fault  <= 4'hF;
                    sd_state_at_fault  <= 4'hF;
                end
            end
        end
    end

    always @* begin
        if (sd_init_error)
            diag_code = 4'd8 + {1'b0, sd_init_fail_code};
        else if (file_error && (fs_fault_code == 4'd4)) begin
            case (fault_phase)
                3'd0:    diag_code = sec_code_at_fault;
                3'd1:    diag_code = 4'd0;
                3'd2:    diag_code = sd_state_at_fault;
                3'd3:    diag_code = 4'd0;
                3'd4:    diag_code = fs_fault_code;
                default: diag_code = 4'd0;
            endcase
        end
        else if (file_error)
            diag_code = 4'd2 + {1'b0, fs_fault_code};
        else if (img_frame_ready)
            diag_code = 4'd2;
        else if (sd_init_done)
            diag_code = 4'd1;
        else
            diag_code = 4'd0;
    end

    wire [3:0] status_led = (diag_code == 4'd0) ?
                            {3'b000, heartbeat[22]} : diag_code;
    wire [3:0] key_ack_led = (key_flash != 25'd0) ? 4'b0001 : 4'b0000;

    // When the streaming announcement cannot be played the built-in chime
    // keeps running.  While that is the case the LEDs alternate between the
    // status word above and a voice fault code, with an all-off page in
    // between so the two words are never read as one:
    //   0001 the root directory holds no .WAV file
    //   0010 the voice phase hit the five second watchdog
    //   0011 the phase ended before the loader reported success
    //   1xxx the loader rejected the file, xxx is its error code
    //        (1 not RIFF, 2 unsupported fmt, 3 no data chunk, 4 empty,
    //         5 card stream failed)
    // page 0 shows the status, page 2 the fault code, pages 1 and 3 are dark
    reg [27:0] voice_page_cnt;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            voice_page_cnt <= 28'd0;
        else
            voice_page_cnt <= voice_page_cnt + 28'd1;
    end

    wire [1:0] voice_page = voice_page_cnt[27:26];   // 1.34 s per page
    wire [3:0] voice_fault_code =
        (scan_done && !voice_found) ? 4'd1 :
        voice_wdt_fired              ? 4'd2 :
        voice_stream_error           ? (4'd8 + {1'b0,
                                                voice_stream_error_code[2:0]}) :
        (voice_stream_started && !voice_stream_active &&
         !voice_stream_done_latch)    ? 4'd3 :
                                        4'd0;
    wire [3:0] voice_fault_led  = (voice_page == 2'd0) ? status_led :
                                  (voice_page == 2'd2) ? voice_fault_code :
                                                         4'b0000;
    wire       voice_fault_active = (voice_fault_code != 4'd0) &&
                                    (startup_blink >= STARTUP_ON2_END);

    // LED4 (MSB) is the sticky display-read-underflow flag exported by the
    // frame store.  A healthy picture shows 0010; after the first underflow
    // the word becomes 1010 and stays there until reset.  That tells an
    // SDRAM read underflow apart from source-picture noise or TMDS damage.
    assign led = (startup_blink < STARTUP_ON1_END) ? 4'b1111 :
                 (startup_blink < STARTUP_OFF_END) ? 4'b0000 :
                 (startup_blink < STARTUP_ON2_END) ? 4'b1111 :
                 voice_fault_active ? voice_fault_led :
                 (status_led | key_ack_led | {cal_fail, 3'b000});

    //------------------------------------------------------------------
    // serial host link
    //
    // The PC sends one framed command and the board answers with one framed
    // reply, so the host always knows what the board really did.  Nothing
    // here touches the picture loader or the audio stream: a slot command
    // only names the cached SDRAM buffer that should be shown next, and the
    // text and parameter commands only reach the overlay and the gain
    // registers.
    //------------------------------------------------------------------
    uart_ctrl #(
        .DEFAULT_DIV (16'd434),     // 115200 baud
        .VERSION     (8'h1B)
    ) u_uart_ctrl (
        .clk             (sys_clk),
        .rst_n           (sys_rst_n),
        .rx_pin          (uart_rx_pin),
        .tx_pin          (uart_tx_pin),
        .text_addr       (uart_text_addr),
        .text_data       (uart_text_data),
        .text_tog        (uart_text_tog),
        .text_clear_tog  (uart_text_clear_tog),
        .text_commit_tog (uart_text_commit_tog),
        .text_len        (),
        .board_tog       (uart_board_tog),
        .board_addr      (uart_board_addr),
        .board_data      (uart_board_data),
        .board_enable    (uart_board_enable),
        .jump_req        (uart_jump_req),
        .jump_slot       (uart_jump_slot),
        .auto_ovr_en     (uart_auto_ovr_en),
        .auto_ovr_mode   (uart_auto_ovr_mode),
        .bright_we       (uart_bright_we),
        .bright_val      (uart_bright_val),
        .contrast_we     (uart_contrast_we),
        .contrast_val    (uart_contrast_val),
        .osd_we          (uart_osd_we),
        .osd_val         (uart_osd_val),
        .baud_div        (),
        .link_seen       (uart_link_seen),
        .time_hh         (uart_time_hh),
        .time_mm         (uart_time_mm),
        .time_ss         (uart_time_ss),
        .time_tog        (uart_time_tog),
        .emg_fire        (emg_fire),
        .st_slot         (display_buf),
        .st_total        (file_count),
        .st_cache        (cache_ready),
        .st_bright       (bright_level),
        .st_contrast     (contrast_level),
        .st_osd          (ui_mode),
        .st_auto         (auto_rotate_en),
        .st_card_ok      (card_ok)
    );

endmodule
