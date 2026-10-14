`timescale 1ns / 1ps
//=============================================================================
// hdmi_adv01_subtitle.v - ADV01: layer overlay and scrolling subtitle
//
// This wrapper keeps the verified base player and turns on only the overlay
// feature for the first advanced-requirement build:
//
//   top-left : ANLU TIS logo and IMG n/m
//   top-right: local TIME MM:SS
//   lower    : slow airport-operation ticker (parallax layer)
//   bottom   : fast flight-info ticker with alpha gradient
//
// Fade, brightness, parameter OSD and the audio waveform stay disabled so a
// board test of this bit verifies one advanced item at a time.
//=============================================================================
module hdmi_adv01_subtitle (
    input  wire       sys_clk,          // 50 MHz
    input  wire       sys_rst_n,
    input  wire       key_mode,         // active low, next picture
    input  wire       key_bright,
    input  wire       key_ui,
    input  wire       key_auto,         // SW1: 1 = automatic rotation

    output wire [3:0] led,

    output wire       HDMI_CLK_P,
    output wire       HDMI_D2_P,
    output wire       HDMI_D1_P,
    output wire       HDMI_D0_P,

    output wire       HDMI_DDC_SCL,
    inout  wire       HDMI_DDC_SDA,

    output wire       SD_nCS,
    output wire       SD_DCLK,
    output wire       SD_MOSI,
    input  wire       SD_MISO
);

    hdmi_mm_player #(
        .FEATURE_UI         (1),
        .FEATURE_LOGO       (1),
        .FEATURE_SUBTITLE   (1),
        .FEATURE_TIMESTAMP  (1),
        .FEATURE_PARALLAX   (1),
        .FEATURE_FADE       (0),
        .FEATURE_BRIGHTNESS (0),
        .FEATURE_OSD        (0),
        .FEATURE_AUDIO_WAVE (0)
    ) u_player (
        .sys_clk       (sys_clk),
        .sys_rst_n     (sys_rst_n),
        .key_mode      (key_mode),
        .key_bright    (key_bright),
        .key_ui        (key_ui),
        .key_auto      (key_auto),
        .led           (led),
        .HDMI_CLK_P    (HDMI_CLK_P),
        .HDMI_D2_P     (HDMI_D2_P),
        .HDMI_D1_P     (HDMI_D1_P),
        .HDMI_D0_P     (HDMI_D0_P),
        .HDMI_DDC_SCL  (HDMI_DDC_SCL),
        .HDMI_DDC_SDA  (HDMI_DDC_SDA),
        .SD_nCS        (SD_nCS),
        .SD_DCLK       (SD_DCLK),
        .SD_MOSI       (SD_MOSI),
        .SD_MISO       (SD_MISO)
    );

endmodule
