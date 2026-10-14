`timescale 1ns / 1ps
//=============================================================================
// video_timing_640x480.v - 640x480 @ 59.3 Hz raster
//
// The counters, the sync pulses and the data enable window are placed exactly
// where the official DEMO 31 vga_disp puts them.  That is the only raster this
// board, cable, monitor and HDMI_A socket combination has been shown to
// display, so there is no reason to move anything:
//
//   hcnt 0 .. 799   (800 clocks per line)
//   vcnt 0 .. 524   (525 lines per frame)  ->  25 MHz / (800 * 525) = 59.52 Hz
//
//   data enable   hcnt 160 .. 799  and vcnt 45 .. 524    (640 x 480 pixels)
//   hsync low     hcnt 656 .. 751  (96 clocks, retained for the debug scope)
//   vsync low     vcnt 490 .. 491  (2 lines, retained for the debug scope)
//
// The active-window position matches the vendor HDMI 1.4b example and the
// working hdmi_core_probe source.  The transmitter core consumes AXIS, so
// matching the vendor's HBLANK/VBLANK placement matters more than matching
// the legacy DEMO31 raster counters.
//
// px / py are the picture coordinates of the current pixel, valid while
// video_active is high; x / y stay the raw raster counters.
//=============================================================================
module video_timing_640x480 (
    input  wire       pixel_clk,
    input  wire       reset_n,
    output reg  [9:0] x,
    output reg  [9:0] y,
    output wire [9:0] px,
    output wire [9:0] py,
    output wire       hsync,
    output wire       vsync,
    output wire       video_active,
    output wire       frame_tick
);

    localparam [9:0] H_LAST = 10'd799;         // last horizontal count
    localparam [9:0] V_LAST = 10'd524;         // last vertical count

    localparam [9:0] H_DE_START   = 10'd160;
    localparam [9:0] H_DE_END     = 10'd800;
    localparam [9:0] H_SYNC_START = 10'd656;
    localparam [9:0] H_SYNC_END   = 10'd752;

    localparam [9:0] V_DE_START   = 10'd45;
    localparam [9:0] V_DE_END     = 10'd525;
    localparam [9:0] V_SYNC_START = 10'd490;
    localparam [9:0] V_SYNC_END   = 10'd492;

    always @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n) begin
            x <= 10'd0;
            y <= 10'd0;
        end else if (x < H_LAST) begin
            x <= x + 1'b1;
        end else begin
            x <= 10'd0;
            if (y < V_LAST)
                y <= y + 1'b1;
            else
                y <= 10'd0;
        end
    end

    assign hsync = ~((x >= H_SYNC_START) && (x < H_SYNC_END));
    assign vsync = ~((y >= V_SYNC_START) && (y < V_SYNC_END));

    assign video_active = (x >= H_DE_START) && (x < H_DE_END) &&
                          (y >= V_DE_START) && (y < V_DE_END);

    assign px = x - H_DE_START;
    assign py = y - V_DE_START;

    assign frame_tick = (x == 10'd0) && (y == 10'd0);

endmodule
