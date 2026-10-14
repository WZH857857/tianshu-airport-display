`timescale 1ns / 1ps
//=============================================================================
// hdmi_adv03_scale.v - ADV03: picture size adaptation
//
// This wrapper keeps everything ADV02 delivers (the overlay layer, the fade
// transition and the 6 s rotation) and turns on the third advanced item: a
// picture whose size is not the display size is resized on its way into the
// frame memory instead of being rejected.
//
// FEATURE_SCALE = 1 switches the BMP header check to the "anything inside the
// scaler limits" mode and inserts rtl/storage/bmp_scaler.v in front of the
// frame store loader.  The frame store still receives exactly 640 x 480
// pixels, so the display path, the transition and the overlay are untouched.
//
// LOAD_CYCLES is raised because a 1600 x 1200 picture carries 5.8 MB over the
// SPI link where a 640 x 480 one carries 0.9 MB; the 20 s default of the base
// project would time the load out halfway through such a file.
//=============================================================================
module hdmi_adv03_scale (
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
        .FEATURE_SCALE      (1),
        .FADE_STEP          (2),
        .FADE_WAIT_TIMEOUT  (180),
        .AUTO_HOLD_CYCLES   (300_000_000),  // 6 s per picture
        .LOAD_CYCLES        (2_000_000_000) // 40 s, a big picture is allowed
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
