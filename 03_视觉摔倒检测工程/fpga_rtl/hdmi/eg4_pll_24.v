//===========================================================================
// eg4_pll_24.v -- the two HDMI clock trees for the VF-E20NG88 board
//
// The board oscillator on P13 is 24 MHz.  The vendor examples that run HDMI
// on this board derive every other clock from the same oscillator, and the
// loop filter values below are the ones the vendor ships for a 960 MHz VCO in
// the 00_Board_Basic_Test project (FIN 24 MHz, REFCLK_DIV 1, FBCLK_DIV 4,
// CLKC0_DIV 10).  Both PLLs here use that same 960 MHz VCO, so the same loop
// filter applies.
//
//   hdmi_video_pll_24
//        VCO 24 * 1 * 40 = 960 MHz
//        C0 = 960 / 40 =  24.000 MHz  pixel clock  (through BUFG, feedback)
//        C1 = 960 /  8 = 120.000 MHz  5x pixel clock for the TMDS serialiser
//
//   hdmi_mem_pll_24
//        VCO 24 * 5 * 8 = 960 MHz
//        C0 = 960 /  8 = 120.000 MHz  SDRAM controller clock (BUFG, feedback)
//        C2 = 960 /  8 = 120.000 MHz, 180 degrees  read capture clock
//
// 24 MHz pixel clock instead of 25 MHz gives 640x480 at 57.1 Hz on the
// 800x525 raster the vendor 640x480 example uses.  That is the raster this
// board's 01-1_EG4_HDMI_TX_Test_640480 demo runs at, so it is the safe
// choice for this oscillator.
//===========================================================================
`timescale 1ns / 1ps

module hdmi_video_pll_24 (
    input  wire refclk,          // 24 MHz board oscillator
    input  wire reset,           // active high
    output wire clk_pixel,       // 24 MHz
    output wire clk_pixel_5x,    // 120 MHz
    output wire extlock
);

    wire clk0_buf;
    wire unused_c2, unused_c3, unused_c4;
    wire unused_psdone;
    wire [7:0] unused_do;

    EG_LOGIC_BUFG bufg_feedback (
        .i (clk0_buf),
        .o (clk_pixel)
    );

    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"),
        .DYNCFG("DISABLE"),
        .FIN("24.000000"),
        .FEEDBK_MODE("NORMAL"),
        .FEEDBK_PATH("CLKC0_EXT"),
        .STDBY_ENABLE("DISABLE"),
        .PLLRST_ENA("ENABLE"),
        .SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(2),
        .ICP_CURRENT(9),
        .KVCO(2),
        .LPF_CAPACITOR(1),
        .LPF_RESISTOR(8),
        .REFCLK_DIV(1),
        .FBCLK_DIV(1),
        .CLKC0_ENABLE("ENABLE"),
        .CLKC0_DIV(40),
        .CLKC0_CPHASE(39),
        .CLKC0_FPHASE(0),
        .CLKC1_ENABLE("ENABLE"),
        .CLKC1_DIV(8),
        .CLKC1_CPHASE(7),
        .CLKC1_FPHASE(0)
    ) pll_inst (
        .refclk     (refclk),
        .reset      (reset),
        .stdby      (1'b0),
        .extlock    (extlock),
        .load_reg   (1'b0),
        .psclk      (1'b0),
        .psdown     (1'b0),
        .psstep     (1'b0),
        .psclksel   (3'b000),
        .psdone     (unused_psdone),
        .dclk       (1'b0),
        .dcs        (1'b0),
        .dwe        (1'b0),
        .di         (8'b00000000),
        .daddr      (6'b000000),
        .do         (unused_do),
        .fbclk      (clk_pixel),
        .clkc       ({unused_c4, unused_c3, unused_c2, clk_pixel_5x, clk0_buf})
    );

endmodule


//===========================================================================
// hdmi_video_pll_25 -- exact 25.000 / 125.000 MHz video clocks
//
// Same oscillator, different reference ratio: divide by 24, multiply by 25,
// so C0 lands exactly on the CEA-861 640x480 pixel clock (25 MHz) and C1 on
// its 5x serial clock (125 MHz).
//
//   PFD   24 / 24      =    1.000 MHz
//   VCO   1.000 x 25 x 40 = 1000.000 MHz
//   C0    1000 / 40    =   25.000 MHz  pixel clock
//   C1    1000 /  8    =  125.000 MHz  5x pixel clock
//
// The VCO frequency and output dividers are exactly the ones the vendor
// `video_pll` IP uses for a 25 MHz / 125 MHz pair (50 MHz input, divide by
// 2), so the loop filter values carry over.
//===========================================================================
module hdmi_video_pll_25 (
    input  wire refclk,          // 24 MHz board oscillator
    input  wire reset,           // active high
    output wire clk_pixel,       // 25.000 MHz
    output wire clk_pixel_5x,    // 125.000 MHz
    output wire extlock
);

    wire clk0_buf;
    wire unused_c2, unused_c3, unused_c4;
    wire unused_psdone;
    wire [7:0] unused_do;

    EG_LOGIC_BUFG bufg_feedback (
        .i (clk0_buf),
        .o (clk_pixel)
    );

    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"),
        .DYNCFG("DISABLE"),
        .FIN("24.000000"),
        .FEEDBK_MODE("NORMAL"),
        .FEEDBK_PATH("CLKC0_EXT"),
        .STDBY_ENABLE("DISABLE"),
        .PLLRST_ENA("ENABLE"),
        .SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(2),
        .ICP_CURRENT(9),
        .KVCO(2),
        .LPF_CAPACITOR(1),
        .LPF_RESISTOR(8),
        .REFCLK_DIV(24),
        .FBCLK_DIV(25),
        .CLKC0_ENABLE("ENABLE"),
        .CLKC0_DIV(40),
        .CLKC0_CPHASE(39),
        .CLKC0_FPHASE(0),
        .CLKC1_ENABLE("ENABLE"),
        .CLKC1_DIV(8),
        .CLKC1_CPHASE(7),
        .CLKC1_FPHASE(0)
    ) pll_inst (
        .refclk     (refclk),
        .reset      (reset),
        .stdby      (1'b0),
        .extlock    (extlock),
        .load_reg   (1'b0),
        .psclk      (1'b0),
        .psdown     (1'b0),
        .psstep     (1'b0),
        .psclksel   (3'b000),
        .psdone     (unused_psdone),
        .dclk       (1'b0),
        .dcs        (1'b0),
        .dwe        (1'b0),
        .di         (8'b00000000),
        .daddr      (6'b000000),
        .do         (unused_do),
        .fbclk      (clk_pixel),
        .clkc       ({unused_c4, unused_c3, unused_c2, clk_pixel_5x, clk0_buf})
    );

endmodule


module hdmi_mem_pll_24 (
    input  wire refclk,          // 24 MHz board oscillator
    input  wire reset,           // active high
    output wire clk_mem,         // 120 MHz
    output wire clk_mem_sft,     // 120 MHz, 180 degrees
    output wire extlock
);

    wire clk0_buf;
    wire unused_c1, unused_c3, unused_c4;
    wire unused_psdone;
    wire [7:0] unused_do;

    EG_LOGIC_BUFG bufg_feedback (
        .i (clk0_buf),
        .o (clk_mem)
    );

    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"),
        .DYNCFG("DISABLE"),
        .FIN("24.000000"),
        .FEEDBK_MODE("NORMAL"),
        .FEEDBK_PATH("CLKC0_EXT"),
        .STDBY_ENABLE("DISABLE"),
        .PLLRST_ENA("ENABLE"),
        .SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(2),
        .ICP_CURRENT(9),
        .KVCO(2),
        .LPF_CAPACITOR(1),
        .LPF_RESISTOR(8),
        .REFCLK_DIV(1),
        .FBCLK_DIV(5),
        .CLKC0_ENABLE("ENABLE"),
        .CLKC0_DIV(8),
        .CLKC0_CPHASE(7),
        .CLKC0_FPHASE(0),
        .CLKC1_ENABLE("DISABLE"),
        .CLKC2_ENABLE("ENABLE"),
        .CLKC2_DIV(8),
        .CLKC2_CPHASE(3),
        .CLKC2_FPHASE(0)
    ) pll_inst (
        .refclk     (refclk),
        .reset      (reset),
        .stdby      (1'b0),
        .extlock    (extlock),
        .load_reg   (1'b0),
        .psclk      (1'b0),
        .psdown     (1'b0),
        .psstep     (1'b0),
        .psclksel   (3'b000),
        .psdone     (unused_psdone),
        .dclk       (1'b0),
        .dcs        (1'b0),
        .dwe        (1'b0),
        .di         (8'b00000000),
        .daddr      (6'b000000),
        .do         (unused_do),
        .fbclk      (clk_mem),
        .clkc       ({unused_c4, unused_c3, clk_mem_sft, unused_c1, clk0_buf})
    );

endmodule
