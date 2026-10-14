//===========================================================================
// hdmi_probe_top.v -- minimal "is the HDMI path alive" bit for VF-E20NG88
//
// This bit deliberately contains none of the fall-detection design: no
// camera, no SDRAM frame store, no vision core.  It is the smallest circuit
// that can put a picture on the HDMI socket:
//
//   24 MHz P13 oscillator -> hdmi_video_pll_24 (24 MHz / 120 MHz)
//   -> 640x480 raster -> colour bars + moving square -> hdmi_tx -> socket
//
// How to read the result:
//   Colour bars on screen      PLL and TMDS chain are fine, so the fault is
//                              downstream (SDRAM store, camera front end or
//                              the reset chain of the full design).
//   Monitor stays "no signal"  the TMDS clock is not coming out at all.
//   Bars appear then drop      the 24 MHz (57.1 Hz) raster is marginal for
//                              that monitor; the full design needs 25 MHz.
//
// 74HC595 LED chain, LED7..LED0 exactly like the full design:
//   LED0  POR done
//   LED1  video PLL locked      <- must be on, otherwise nothing is clocked
//   LED2  heartbeat (1 Hz, 24 MHz side)
//   LED3  raster running (1 Hz, pixel side)
//   LED7  constant 1 (chain alive)
//===========================================================================
`timescale 1ns / 1ps

module hdmi_probe_top #(
    parameter integer CLK_25 = 0        // 1 = exact 25 MHz CEA raster
) (
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

    // ------------------------------------------------------------ power-on
    // KEY0 held low restarts the counter, which holds the PLL in reset.
    localparam [19:0] POR_END = 20'hFFFFF;      // ~43.7 ms at 24 MHz

    reg [19:0] por_cnt;
    always @(posedge clk_24m or negedge key_n) begin
        if (!key_n)                    por_cnt <= 20'd0;
        else if (por_cnt != POR_END)   por_cnt <= por_cnt + 20'd1;
    end

    wire por_done = (por_cnt == POR_END);

    // ---------------------------------------------------------- video PLL
    wire clk_pixel, clk_pixel_5x, pll_lock;

    generate
        if (CLK_25) begin : g_pll_25
            hdmi_video_pll_25 u_pll (
                .refclk       (clk_24m),
                .reset        (~por_done),
                .clk_pixel    (clk_pixel),
                .clk_pixel_5x (clk_pixel_5x),
                .extlock      (pll_lock)
            );
        end else begin : g_pll_24
            hdmi_video_pll_24 u_pll (
                .refclk       (clk_24m),
                .reset        (~por_done),
                .clk_pixel    (clk_pixel),
                .clk_pixel_5x (clk_pixel_5x),
                .extlock      (pll_lock)
            );
        end
    endgenerate

    wire rst_n = por_done & pll_lock;

    // -------------------------------------------------------------- raster
    wire [9:0] x, y, px, py;
    wire       hsync, vsync, video_active, frame_tick;

    video_timing_640x480 u_timing (
        .pixel_clk    (clk_pixel),
        .reset_n      (rst_n),
        .x            (x),
        .y            (y),
        .px           (px),
        .py           (py),
        .hsync        (hsync),
        .vsync        (vsync),
        .video_active (video_active),
        .frame_tick   (frame_tick)
    );

    // One cycle of delay, the same relationship the full design uses.
    reg        hs_d, vs_d, de_d;
    reg [9:0]  px_d, py_d;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            hs_d <= 1'b1;
            vs_d <= 1'b1;
            de_d <= 1'b0;
            px_d <= 10'd0;
            py_d <= 10'd0;
        end else begin
            hs_d <= hsync;
            vs_d <= vsync;
            de_d <= video_active;
            px_d <= px;
            py_d <= py;
        end
    end

    // ----------------------------------------------------------- picture
    reg [23:0] bar_rgb;
    always @(*) begin
        case (px_d[9:7])                    // 8 stripes, 128 pixels each
            3'd0:    bar_rgb = 24'hFFFFFF;  // white
            3'd1:    bar_rgb = 24'hFFFF00;  // yellow
            3'd2:    bar_rgb = 24'h00FFFF;  // cyan
            3'd3:    bar_rgb = 24'h00FF00;  // green
            3'd4:    bar_rgb = 24'hFF00FF;  // magenta
            3'd5:    bar_rgb = 24'hFF0000;  // red
            3'd6:    bar_rgb = 24'h0000FF;  // blue
            default: bar_rgb = 24'h000000;  // black
        endcase
    end

    // A square that walks left to right, so a frozen picture is obvious.
    reg [9:0] mvx;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n)                             mvx <= 10'd0;
        else if (frame_tick && mvx >= 10'd560)  mvx <= 10'd0;
        else if (frame_tick)                    mvx <= mvx + 10'd8;
    end

    wire moving = (px_d >= mvx) && (px_d < mvx + 10'd80) &&
                  (py_d >= 10'd200) && (py_d < 10'd280);

    wire [23:0] rgb = !de_d   ? 24'h000000 :
                      moving  ? 24'hFFFFFF : bar_rgb;

    // ------------------------------------------------------------- status
    reg [23:0] hb_cnt;
    reg        hb;
    always @(posedge clk_24m or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt <= 24'd0;
            hb     <= 1'b0;
        end else if (hb_cnt == 24'd12000000 - 24'd1) begin
            hb_cnt <= 24'd0;
            hb     <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 24'd1;
        end
    end

    reg [24:0] fa_cnt;
    reg        fa;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            fa_cnt <= 25'd0;
            fa     <= 1'b0;
        end else if (fa_cnt == 25'd12000000 - 25'd1) begin
            fa_cnt <= 25'd0;
            fa     <= ~fa;
        end else begin
            fa_cnt <= fa_cnt + 25'd1;
        end
    end

    wire [7:0] led_data = {1'b1, 1'b0, 1'b0, 1'b0, fa, hb, pll_lock, por_done};

    led595_shift u_led (
        .clk     (clk_24m),
        .rst_n   (por_done),        // the chain must work even with the PLL unlocked
        .data    (led_data),
        .clk_out (led595_clk),
        .latch   (led595_latch),
        .dout    (led595_dout)
    );

    // ------------------------------------------------------------ HDMI out
    // RST_I inside DVITransmitter is active high.
    hdmi_tx #(
        .FAMILY ("EG4")
    ) u_hdmi_tx (
        .PXLCLK_I    (clk_pixel),
        .PXLCLK_5X_I (clk_pixel_5x),
        .RST_N       (~rst_n),
        .VGA_HS      (hs_d),
        .VGA_VS      (vs_d),
        .VGA_DE      (de_d),
        .VGA_RGB     (rgb),
        .HDMI_CLK_P  (HDMI_CLK_P),
        .HDMI_D2_P   (HDMI_D2_P),
        .HDMI_D1_P   (HDMI_D1_P),
        .HDMI_D0_P   (HDMI_D0_P)
    );

endmodule
