`timescale 1ns / 1ps
//=============================================================================
// diag_overlay.v - on screen diagnostic readout
//
// The four status LEDs are hard to read while they cycle, so the same
// information is drawn on top of the fallback picture: two large hexadecimal
// digits on a dark panel, readable from a photograph of the monitor.
//
//   left  digit : source of the reading
//                 0 = running state code
//                 1 = SD initialization failed
//                 2 = fault cycle group 1 (sector error code)
//                 3 = fault cycle group 2 (SD controller state)
//                 4 = fault cycle group 3 (file system fault code)
//   right digit : the four bit code itself, same value as LED4..LED1
//
// Coordinates are the 640x480 active area coordinates that already travel
// with the delayed data enable, so the panel lines up with the pixel stream.
//=============================================================================
module diag_overlay (
    input  wire [9:0]  px,
    input  wire [9:0]  py,
    input  wire        de,
    input  wire [3:0]  code,
    input  wire [3:0]  source,
    input  wire        enable,
    output reg  [23:0] rgb,
    output reg         hit
);

    localparam [9:0] PANEL_X0 = 10'd120;
    localparam [9:0] PANEL_X1 = 10'd519;
    localparam [9:0] PANEL_Y0 = 10'd120;
    localparam [9:0] PANEL_Y1 = 10'd359;
    localparam [9:0] DIG_Y    = 10'd144;
    localparam [9:0] DIG_A_X  = 10'd160;
    localparam [9:0] DIG_B_X  = 10'd364;
    localparam [9:0] DIG_W    = 10'd96;
    localparam [9:0] DIG_H    = 10'd192;
    localparam [9:0] BORDER   = 10'd4;

    // Segment shapes inside one digit box: horizontal bars are inset by the
    // bar thickness, vertical bars ocupy the left and right columns.
    localparam [9:0] BAR_X0 = 10'd24;   // left edge of a horizontal bar
    localparam [9:0] BAR_X1 = 10'd71;   // right edge of a horizontal bar
    localparam [9:0] BAR_T0 = 10'd23;   // bottom of the top bar
    localparam [9:0] MID_T1 = 10'd83;   // bottom of the upper vertical bars
    localparam [9:0] MID_B0 = 10'd84;   // top of the middle bar
    localparam [9:0] MID_B1 = 10'd107;  // bottom of the middle bar
    localparam [9:0] LOW_T0 = 10'd108;  // top of the lower vertical bars
    localparam [9:0] LOW_T1 = 10'd167;  // bottom of the lower vertical bars
    localparam [9:0] BOT_T0 = 10'd168;  // top of the bottom bar

    reg [9:0] dx_a, dy_a, dx_b, dy_b;
    reg       in_panel, on_border;

    // Bit order of the mask is {g, f, e, d, c, b, a}.
    function seg_on;
        input [9:0] dx;
        input [9:0] dy;
        input [3:0] val;
        reg   [6:0] mask;
        reg   [6:0] shape;
        begin
            case (val)
                4'h0:    mask = 7'b0111111;
                4'h1:    mask = 7'b0000110;
                4'h2:    mask = 7'b1011011;
                4'h3:    mask = 7'b1001111;
                4'h4:    mask = 7'b1100110;
                4'h5:    mask = 7'b1101101;
                4'h6:    mask = 7'b1111101;
                4'h7:    mask = 7'b0000111;
                4'h8:    mask = 7'b1111111;
                4'h9:    mask = 7'b1101111;
                4'hA:    mask = 7'b1110111;
                4'hB:    mask = 7'b1111100;
                4'hC:    mask = 7'b0111001;
                4'hD:    mask = 7'b1011110;
                4'hE:    mask = 7'b1111001;
                default: mask = 7'b1110001;
            endcase

            shape = 7'd0;
            if ((dx >= BAR_X0) && (dx <= BAR_X1)) begin
                if (dy <= BAR_T0)
                    shape[0] = 1'b1;
                if ((dy >= MID_B0) && (dy <= MID_B1))
                    shape[6] = 1'b1;
                if (dy >= BOT_T0)
                    shape[3] = 1'b1;
            end
            if (dx < BAR_X0) begin
                if ((dy > BAR_T0) && (dy <= MID_T1))
                    shape[5] = 1'b1;
                if ((dy >= LOW_T0) && (dy <= LOW_T1))
                    shape[4] = 1'b1;
            end
            if (dx > BAR_X1) begin
                if ((dy > BAR_T0) && (dy <= MID_T1))
                    shape[1] = 1'b1;
                if ((dy >= LOW_T0) && (dy <= LOW_T1))
                    shape[2] = 1'b1;
            end

            seg_on = (dx < DIG_W) && (dy < DIG_H) && (|(shape & mask));
        end
    endfunction

    always @* begin
        dx_a = px - DIG_A_X;
        dy_a = py - DIG_Y;
        dx_b = px - DIG_B_X;
        dy_b = py - DIG_Y;

        in_panel = (px >= PANEL_X0) && (px <= PANEL_X1) &&
                   (py >= PANEL_Y0) && (py <= PANEL_Y1);

        on_border = in_panel &&
                    (((px - PANEL_X0) < BORDER) ||
                     ((PANEL_X1 - px) < BORDER) ||
                     ((py - PANEL_Y0) < BORDER) ||
                     ((PANEL_Y1 - py) < BORDER));

        rgb = 24'h000000;
        hit = 1'b0;

        if (enable && de && in_panel) begin
            hit = 1'b1;
            rgb = 24'h101010;
            if (on_border)
                rgb = 24'hFFFFFF;
            else if (seg_on(dx_a, dy_a, source))
                rgb = 24'hFFC000;               // source digit: amber
            else if (seg_on(dx_b, dy_b, code))
                rgb = 24'h00E000;               // code digit: green
        end
    end

endmodule
