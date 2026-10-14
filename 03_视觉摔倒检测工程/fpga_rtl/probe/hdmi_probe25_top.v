//===========================================================================
// hdmi_probe25_top.v -- colour-bar probe with the exact 25 MHz raster
//
// Same circuit as hdmi_probe_top, only the video PLL differs: it asks for
// 25.000 MHz / 125.000 MHz (the CEA-861 640x480 pair) instead of the
// 24.000 MHz / 120.000 MHz that falls out of the plain oscillator clock.
// Used to tell a "monitor will not lock to 57 Hz" problem apart from a real
// PLL or TMDS problem.
//===========================================================================
`timescale 1ns / 1ps

module hdmi_probe25_top (
    input  wire       clk_24m,       // P13
    input  wire       key_n,         // P51, KEY0, low when pressed

    output wire       HDMI_CLK_P,    // P47
    output wire       HDMI_D2_P,     // P61
    output wire       HDMI_D1_P,     // P55
    output wire       HDMI_D0_P,     // P49

    output wire       led595_clk,    // P59
    output wire       led595_latch,  // P57
    output wire       led595_dout    // P45
);

    hdmi_probe_top #(
        .CLK_25 (1)
    ) u_probe (
        .clk_24m     (clk_24m),
        .key_n       (key_n),
        .HDMI_CLK_P  (HDMI_CLK_P),
        .HDMI_D2_P   (HDMI_D2_P),
        .HDMI_D1_P   (HDMI_D1_P),
        .HDMI_D0_P   (HDMI_D0_P),
        .led595_clk  (led595_clk),
        .led595_latch(led595_latch),
        .led595_dout (led595_dout)
    );

endmodule
