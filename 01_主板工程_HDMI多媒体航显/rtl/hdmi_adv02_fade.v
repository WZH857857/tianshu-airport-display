`timescale 1ns / 1ps
//=============================================================================
// hdmi_adv02_fade.v - ADV02: picture switching with a fade transition
//
// This wrapper keeps everything ADV01 delivers (terminal id, clock, double
// scrolling ticker) and adds the second advanced item: a real transition
// when the picture changes, whether it is triggered by KEY1 or by the
// automatic rotation (6 s per picture).
//
// The transition is a fade through black: the picture gain ramps down over
// 16 video frames, the frame store swaps at black, then the gain ramps back
// up over another 16 frames (~0.53 s in total).  Only the picture layer is
// faded - the overlay text keeps its brightness, so the information layer
// stays continuous while the photo behind it changes.
//
// FADE_STEP 2 means the 5-bit gain walks 31 -> 0 in 16 frames instead of the
// 8 frames the old default produced, which turned the transition into a
// flicker rather than a visible fade.
//=============================================================================
module hdmi_adv02_fade (
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
        .FEATURE_FADE       (1),
        .FEATURE_BRIGHTNESS (0),
        .FEATURE_OSD        (0),
        .FEATURE_AUDIO_WAVE (0),
        .FADE_STEP          (2),
        .FADE_WAIT_TIMEOUT  (180),
        .AUTO_HOLD_CYCLES   (300_000_000)   // 6 s per picture
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
