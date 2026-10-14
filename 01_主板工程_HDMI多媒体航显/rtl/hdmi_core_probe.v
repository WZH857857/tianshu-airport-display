`timescale 1ns / 1ps
//=============================================================================
// hdmi_core_probe.v - minimal HDMI 1.4b transmitter bring-up test
//
// This deliberately removes the TF card, FAT32 parser, BMP decoder and SDRAM
// frame buffer. It drives the vendor HDMI 1.4b core from a fixed internal
// video source and feeds it the same built-in 48 kHz I2S tone used by the
// full player.
//
// HDMI reset is intentionally independent of the audio PLL lock here. If the
// audio PLL fails to lock, video can still appear and the LEDs can report the
// audio state independently.
//=============================================================================
module hdmi_core_probe (
    input  wire       sys_clk,      // 50 MHz board clock
    input  wire       sys_rst_n,    // KEY3, active low

    output wire [3:0] led,

    output wire       HDMI_CLK_P,
    output wire       HDMI_D2_P,
    output wire       HDMI_D1_P,
    output wire       HDMI_D0_P,

    output wire       HDMI_DDC_SCL,
    inout  wire       HDMI_DDC_SDA
);

    wire pixel_clk;
    wire pixel_clk_5x;
    wire audio_mclk;
    wire audio_pll_lock;

    video_pll u_pixel_pll (
        .refclk  (sys_clk),
        .reset   (1'b0),
        .clk0_out(pixel_clk),       // 25 MHz
        .clk1_out(pixel_clk_5x)     // 125 MHz
    );

    PLL_HDMI_AUDIO u_audio_pll (
        .refclk  (sys_clk),
        .reset   (1'b0),
        .extlock (audio_pll_lock),
        .clk0_out(),
        .clk1_out(),
        .clk2_out(audio_mclk)       // 12.295 MHz, about 48.03 kHz I2S
    );

    // Keep video independent from the audio PLL while probing.
    wire rst_video = ~sys_rst_n;
    wire rst_audio = ~sys_rst_n | ~audio_pll_lock;

    //--------------------------------------------------------------------
    // fixed 640x480 AXIS video source
    //--------------------------------------------------------------------
    wire        axis_s_user;
    wire        axis_s_valid;
    wire        axis_s_last;
    wire [23:0] axis_s_data;
    wire        axis_s_ready;

    probe_color_bars #(
        .HTOTAL  (800),
        .HACTIVE (640),
        .VTOTAL  (525),
        .VACTIVE (480)
    ) u_color_bars (
        .I_clk         (pixel_clk),
        .I_rst         (rst_video),
        .O_video_user  (axis_s_user),
        .O_video_valid (axis_s_valid),
        .O_video_last  (axis_s_last),
        .O_video_data  (axis_s_data)
    );

    //--------------------------------------------------------------------
    // built-in I2S tone -> audio sample packet inputs
    //--------------------------------------------------------------------
    wire audio_i2s_bclk;
    wire audio_i2s_lrck;
    wire audio_i2s_dout;
    wire audio_valid;
    wire [23:0] audio_left;
    wire [23:0] audio_right;
    wire acr_valid;
    wire [19:0] acr_cts;
    wire [19:0] acr_n;

    hdmi_audio_tone_i2s_64fs #(
        .BGM_SHIFT   (15),
        .CUE_SHIFT   (14),
        .UNIT_FRAMES (2400),
        .CUE_FRAMES  (9600)
    ) u_audio_tone (
        .I_mclk    (audio_mclk),
        .I_rst     (rst_audio),
        .I_cue     (1'b0),
        .O_i2s_BCLK(audio_i2s_bclk),
        .O_i2s_LRCK(audio_i2s_lrck),
        .O_i2s_DOUT(audio_i2s_dout)
    );

    I2S_receiver u_i2s_receiver (
        .I_clk              (pixel_clk),
        .I_rst              (rst_audio),
        .I_i2s_BCLK         (audio_i2s_bclk),
        .I_i2s_LRCK         (audio_i2s_lrck),
        .I_i2s_DOUT         (audio_i2s_dout),
        .O_audio_valid      (audio_valid),
        .O_audio_left_data  (audio_left),
        .O_audio_right_data (audio_right)
    );

    audio_arc_calculate #(
        .ACR_N (6144)
    ) u_audio_arc (
        .I_clk         (pixel_clk),
        .I_rst         (rst_audio),
        .I_audio_valid (audio_valid),
        .O_acr_valid   (acr_valid),
        .O_acr_cts     (acr_cts),
        .O_acr_n       (acr_n)
    );

    //--------------------------------------------------------------------
    // one EDID read pulse, matching the vendor reference
    //--------------------------------------------------------------------
    wire edid_trig;
    wire edid_valid;
    wire [7:0] edid_data;

    startup_pulse #(
        .CNT_MAX (20'd100000)
    ) u_startup_pulse (
        .I_clk   (pixel_clk),
        .I_rst   (rst_video),
        .O_pulse (edid_trig)
    );

    //--------------------------------------------------------------------
    // HDMI 1.4b core and TMDS serialisers
    //--------------------------------------------------------------------
    wire        video_locked;
    wire [9:0]  tmds_ch0;
    wire [9:0]  tmds_ch1;
    wire [9:0]  tmds_ch2;
    wire [9:0]  tmds_clk;

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

        .O_video_locked     (video_locked),
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

    //--------------------------------------------------------------------
    // LED status:
    //   LED1 = live pixel clock (heartbeat)
    //   LED2 = audio PLL lock
    //   LED3 = a non-zero I2S stereo sample reached the HDMI core
    //   LED4 = a valid ACR packet was generated for the HDMI core
    //--------------------------------------------------------------------
    reg [22:0] heartbeat;
    reg        audio_sample_seen;
    reg        acr_seen;

    always @(posedge pixel_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            heartbeat <= 23'd0;
        else
            heartbeat <= heartbeat + 1'b1;
    end

    always @(posedge pixel_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            audio_sample_seen <= 1'b0;
            acr_seen          <= 1'b0;
        end else begin
            if (audio_valid && ((audio_left != 24'd0) ||
                                (audio_right != 24'd0)))
                audio_sample_seen <= 1'b1;

            if (acr_valid && (acr_n == 20'd6144))
                acr_seen <= 1'b1;
        end
    end

    assign led = {
        acr_seen,
        audio_sample_seen,
        audio_pll_lock,
        heartbeat[22]
    };

endmodule

//----------------------------------------------------------------------------
// probe_color_bars.v - 640x480 eight vertical colour bars
//----------------------------------------------------------------------------
module probe_color_bars #(
    parameter integer HTOTAL  = 800,
    parameter integer HACTIVE = 640,
    parameter integer VTOTAL  = 525,
    parameter integer VACTIVE = 480
)(
    input  wire        I_clk,
    input  wire        I_rst,

    output reg         O_video_user,
    output reg         O_video_valid,
    output reg         O_video_last,
    output reg  [23:0] O_video_data
);

    localparam integer HBLANK = HTOTAL - HACTIVE;
    localparam integer VBLANK = VTOTAL - VACTIVE;

    reg [9:0] x;
    reg [9:0] y;

    wire active = (x >= HBLANK) && (x < HTOTAL) &&
                  (y >= VBLANK) && (y < VTOTAL);
    wire [9:0] bx = active ? (x - HBLANK) : 10'd0;

    reg [23:0] bar_rgb;

    always @* begin
        case (bx / 10'd80)
            4'd0: bar_rgb = 24'hFF_FF_FF; // white
            4'd1: bar_rgb = 24'hFF_FF_00; // yellow
            4'd2: bar_rgb = 24'h00_FF_FF; // cyan
            4'd3: bar_rgb = 24'h00_FF_00; // green
            4'd4: bar_rgb = 24'hFF_00_FF; // magenta
            4'd5: bar_rgb = 24'hFF_00_00; // red
            4'd6: bar_rgb = 24'h00_00_FF; // blue
            default: bar_rgb = 24'h00_00_00; // black
        endcase
    end

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            x             <= 10'd0;
            y             <= 10'd0;
            O_video_user  <= 1'b0;
            O_video_valid <= 1'b0;
            O_video_last  <= 1'b0;
            O_video_data  <= 24'd0;
        end else begin
            if (x == HTOTAL - 1) begin
                x <= 10'd0;
                if (y == VTOTAL - 1)
                    y <= 10'd0;
                else
                    y <= y + 1'b1;
            end else begin
                x <= x + 1'b1;
            end

            O_video_user  <= active && (x == HBLANK) && (y == VBLANK);
            O_video_valid <= active;
            O_video_last  <= active && (x == HTOTAL - 1) &&
                             (y == VTOTAL - 1);
            O_video_data  <= active ? bar_rgb : 24'd0;
        end
    end

endmodule
