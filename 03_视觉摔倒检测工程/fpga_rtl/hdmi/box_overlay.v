//===========================================================================
// box_overlay.v -- draw the four tracked boxes on the HDMI picture
//
// The boxes come from multi_bbox/track_assoc and live in the vision raster
// (320 x 180).  The HDMI picture is 640 x 480, so the picture coordinate is
// mapped back into vision space instead of scaling every box up:
//
//   vx = px / 2
//   vy = (py * 3) / 8
//
// (py * 3) / 8 is the exact inverse of the 8/3 vertical stretch for the
// integer raster: display row py belongs to vision row floor(3*py/8).  That
// keeps the whole overlay to one small multiply and two shifts, no divider.
//
// Colours: every tracked slot gets its own hue, so the monitor reads like a
// multi-target tracker instead of a single-colour blob view -
//
//   slot 0  green      slot 2  amber
//   slot 1  cyan       slot 3  magenta
//
// Because track_assoc keeps a person on the same slot from frame to frame,
// the colour sticks to the person: you can follow "the cyan one" across the
// picture with your eye.  A slot whose posture machine has confirmed a fall
// overrides all of that with red, and its interior is dimmed so the alarm is
// the brightest thing on the screen.
//
// Every box also carries a name tag drawn just above it (tools/gen_glyphs.py
// renders the Chinese bitmaps).  A tracked person prints `87%` on the slot
// colour - the confidence score is described in rtl/person_score.v; a
// confirmed fall keeps the red `摔倒` alarm tag.
//
// Two tag geometries, because the two labels have very different jobs:
//
//   person   3 x 5 digit / 5 x 5 percent font, one font pixel = one vision
//            pixel, packed into 8-wide tiles: `87%` is 24 x 5 vision pixels,
//            i.e. 48 x 13 picture pixels - the size of a detector tag.
//   fall     the two 24 x 18 Chinese cells stay as they are (48 x 48 picture
//            pixels per character): the alarm is rare and is meant to shout.
//===========================================================================
`timescale 1ns / 1ps

module box_overlay #(
    parameter integer NB = 4
) (
    input  wire            de,
    input  wire [9:0]      px,
    input  wire [9:0]      py,
    input  wire [23:0]     rgb_in,

    input  wire [NB-1:0]   found,
    input  wire [NB-1:0]   alarm,
    input  wire [NB*9-1:0] bx,
    input  wire [NB*9-1:0] by,
    input  wire [NB*9-1:0] bw,
    input  wire [NB*9-1:0] bh,
    input  wire [NB*7-1:0] score,

    output reg  [23:0]     rgb_out
);

    wire [9:0]  vx   = {1'b0, px[9:1]};      // 0 .. 319
    wire [10:0] py3  = {1'b0, py} * 11'd3;   // 0 .. 1572
    wire [9:0]  vy   = py3[10:3];            // 0 .. 196

    wire [NB-1:0] edge_hit;
    wire [NB-1:0] inside_hit;

    genvar gi;
    generate
        for (gi = 0; gi < NB; gi = gi + 1) begin : g_box
            wire [8:0]  x0 = bx[gi*9 +: 9];
            wire [8:0]  y0 = by[gi*9 +: 9];
            wire [8:0]  w  = bw[gi*9 +: 9];
            wire [8:0]  h  = bh[gi*9 +: 9];

            // One wrapping subtraction per axis answers every question: a
            // pixel left of / above the box makes dx or dy wrap past 1k,
            // which no box edge (0 .. 511) can reach, so a single unsigned
            // compare replaces the (vx >= x0) && (vx < x0 + w) pair.
            wire [9:0]  dx = {1'b0, vx} - {1'b0, x0};
            wire [9:0]  dy = {1'b0, vy} - {1'b0, y0};

            wire        in_x   = (dx < {1'b0, w});
            wire        in_y   = (dy < {1'b0, h});
            wire        core_x = in_x && (dx != 10'd0);
            wire        core_y = in_y && (dy != 10'd0);
            // The 3-pixel border band spans x0-1 .. x1, i.e. dx == -1 or dx
            // <= w.
            wire        band_x = (dx <= {1'b0, w}) || (dx == 10'h3FF);
            wire        band_y = (dy <= {1'b0, h}) || (dy == 10'h3FF);

            assign inside_hit[gi] = found[gi] && in_x && in_y;
            assign edge_hit[gi]   = found[gi] && band_x && band_y &&
                                    !(core_x && core_y);
        end
    endgenerate

    wire any_alarm_edge   = |(edge_hit   & alarm);
    wire any_alarm_inside = |(inside_hit & alarm);
    wire any_alarm_box    = |(found & alarm);

    // One hue per slot.  NB is 4 today; a wider bank repeats the palette in
    // order, which stays readable as long as neighbours differ.
    function [23:0] slot_colour;
        input [1:0] s;
        begin
            case (s)
                2'd0:    slot_colour = 24'h00FF60;   // green
                2'd1:    slot_colour = 24'h00C8FF;   // cyan
                2'd2:    slot_colour = 24'hFFC400;   // amber
                default: slot_colour = 24'hFF40FF;   // magenta
            endcase
        end
    endfunction

    integer ci;
    reg [23:0] track_rgb;
    always @(*) begin
        track_rgb = 24'h00FF60;
        for (ci = NB - 1; ci >= 0; ci = ci - 1)
            if (edge_hit[ci]) track_rgb = slot_colour(ci[1:0]);
    end

    // -------------------------------------------------------------- glyphs
    // 24 x 18 one-bit cells, bit 23 = leftmost column, row 0 = top.
    // Rebuild with: python fpga/tools/gen_glyphs.py --verilog
    function [23:0] glyph_row;
        input [1:0] ch;      // 0 = 摔, 1 = 倒
        input [4:0] row;     // 0 .. 17
        reg [23:0] bits;
        begin
            bits = 24'd0;
            case ({ch, row})
                7'd0: bits = 24'h000000;
                7'd1: bits = 24'h181800;
                7'd2: bits = 24'h180C00;
                7'd3: bits = 24'h19FFE0;
                7'd4: bits = 24'h581000;
                7'd5: bits = 24'h7F9840;
                7'd6: bits = 24'h19B6C0;
                7'd7: bits = 24'h183E80;
                7'd8: bits = 24'h1A2C00;
                7'd9: bits = 24'h1EDAC0;
                7'd10: bits = 24'h79BF60;
                7'd11: bits = 24'h7B3F60;
                7'd12: bits = 24'h180800;
                7'd13: bits = 24'h180C00;
                7'd14: bits = 24'h1BFFE0;
                7'd15: bits = 24'h198C60;
                7'd16: bits = 24'h180C00;
                7'd17: bits = 24'h380C00;
                7'd32: bits = 24'h000000;
                7'd33: bits = 24'h0C0060;
                7'd34: bits = 24'h0FF860;
                7'd35: bits = 24'h19F260;
                7'd36: bits = 24'h18C360;
                7'd37: bits = 24'h189360;
                7'd38: bits = 24'h399B60;
                7'd39: bits = 24'h7BFB60;
                7'd40: bits = 24'h7BFB60;
                7'd41: bits = 24'h584360;
                7'd42: bits = 24'h184360;
                7'd43: bits = 24'h1BFB60;
                7'd44: bits = 24'h19FB60;
                7'd45: bits = 24'h184360;
                7'd46: bits = 24'h184060;
                7'd47: bits = 24'h18F860;
                7'd48: bits = 24'h1BF0E0;
                7'd49: bits = 24'h1B01C0;
                default: bits = 24'd0;
            endcase
            glyph_row = bits;
        end
    endfunction

    // The confidence readout uses a compact 3x5 digit font instead of adding
    // eleven more 24x18 font cells.  One font pixel is one vision pixel, so
    // the tag stays the size of a detector label instead of a box.
    function [14:0] digit_pattern;
        input [3:0] d;
        begin
            case (d)
                4'd0: digit_pattern = 15'b111101101101111;
                4'd1: digit_pattern = 15'b010110010010111;
                4'd2: digit_pattern = 15'b111001111100111;
                4'd3: digit_pattern = 15'b111001111001111;
                4'd4: digit_pattern = 15'b101101111001001;
                4'd5: digit_pattern = 15'b111100111001111;
                4'd6: digit_pattern = 15'b111100111101111;
                4'd7: digit_pattern = 15'b111001001001001;
                4'd8: digit_pattern = 15'b111101111101111;
                default: digit_pattern = 15'b111101111001111;
            endcase
        end
    endfunction

    // One font row of a digit: the three pixels of that row, leftmost in
    // bit 2.  A five-way constant mux costs three LUTs; the row/3 divide and
    // the bit-spreading the first version needed cost far more.
    function [2:0] digit_group;
        input [3:0] d;
        input [2:0] row;
        reg [14:0] p;
        begin
            p = digit_pattern(d);
            if      (row == 3'd0) digit_group = p[14:12];
            else if (row == 3'd1) digit_group = p[11:9];
            else if (row == 3'd2) digit_group = p[8:6];
            else if (row == 3'd3) digit_group = p[5:3];
            else if (row == 3'd4) digit_group = p[2:0];
            else                  digit_group = 3'd0;
        end
    endfunction

    // 5x5 percent sign: two dots, a diagonal, and two more dots, again one
    // row per font row with the leftmost pixel in bit 4.
    function [4:0] percent_group;
        input [2:0] row;
        begin
            if      (row == 3'd0) percent_group = 5'b11001;
            else if (row == 3'd1) percent_group = 5'b11010;
            else if (row == 3'd2) percent_group = 5'b00100;
            else if (row == 3'd3) percent_group = 5'b01011;
            else if (row == 3'd4) percent_group = 5'b10011;
            else                  percent_group = 5'd0;
        end
    endfunction

    function [3:0] tens_digit;
        input [6:0] s;
        begin
            if      (s >= 7'd90) tens_digit = 4'd9;
            else if (s >= 7'd80) tens_digit = 4'd8;
            else if (s >= 7'd70) tens_digit = 4'd7;
            else if (s >= 7'd60) tens_digit = 4'd6;
            else if (s >= 7'd50) tens_digit = 4'd5;
            else if (s >= 7'd40) tens_digit = 4'd4;
            else if (s >= 7'd30) tens_digit = 4'd3;
            else if (s >= 7'd20) tens_digit = 4'd2;
            else if (s >= 7'd10) tens_digit = 4'd1;
            else                 tens_digit = 4'd0;
        end
    endfunction

    function [3:0] ones_digit;
        input [6:0] s;
        begin
            if      (s >= 7'd90) ones_digit = s - 7'd90;
            else if (s >= 7'd80) ones_digit = s - 7'd80;
            else if (s >= 7'd70) ones_digit = s - 7'd70;
            else if (s >= 7'd60) ones_digit = s - 7'd60;
            else if (s >= 7'd50) ones_digit = s - 7'd50;
            else if (s >= 7'd40) ones_digit = s - 7'd40;
            else if (s >= 7'd30) ones_digit = s - 7'd30;
            else if (s >= 7'd20) ones_digit = s - 7'd20;
            else if (s >= 7'd10) ones_digit = s - 7'd10;
            else                 ones_digit = s;
        end
    endfunction

    // Tag geometry, in vision pixels, set per slot inside the generate loop:
    // a person tag is 24 x 5 (three 8-wide tiles of 5 font rows), the fall
    // tag is 48 x 18 (two 24x18 Chinese cells).

    wire [NB-1:0]   lbl_hit;
    wire [NB*7-1:0] lbl_acol;        // column inside the tag, 0 .. 95
    wire [NB*5-1:0] lbl_arow;        // row inside the tag, 0 .. 17

    generate
        for (gi = 0; gi < NB; gi = gi + 1) begin : g_label
            wire [8:0]  x0 = bx[gi*9 +: 9];
            wire [8:0]  y0 = by[gi*9 +: 9];
            // `摔倒` for an alarm, `87%` for a tracked person.
            wire [9:0]  lw  = alarm[gi] ? 10'd48 : 10'd24;
            wire [4:0]  lh  = alarm[gi] ? 5'd18  : 5'd5;
            // The tag sits above the box; a box in the top margin keeps its
            // tag inside the top-left corner instead of off-screen.  Both
            // offsets are the same wrapping subtraction as the box itself:
            // rows above the tag wrap past 1k and fail the < lh test.
            wire [9:0]  dy = {1'b0, vy} - {1'b0, y0};
            wire [9:0]  dx = {1'b0, vx} - {1'b0, x0};
            wire [9:0]  ly = (y0 >= (lh + 1)) ? (dy + lh + 1)
                                              : (dy - 10'd1);

            assign lbl_hit[gi] = found[gi] && (dx < lw) && (ly < lh);
            assign lbl_acol[gi*7 +: 7] = dx[6:0];
            assign lbl_arow[gi*5 +: 5] = ly[4:0];
        end
    endgenerate

    reg             lbl_any, lbl_alarm;
    reg [23:0]      lbl_colour;
    reg [6:0]       lbl_col, lbl_cell_col, lbl_score;
    reg [4:0]       lbl_row;
    reg [3:0]       lbl_ch, lbl_digit;
    reg [1:0]       lbl_cell;       // 0/1 = 摔/倒 or tens/ones, 2 = %
    integer li;
    always @(*) begin
        lbl_any      = 1'b0;
        lbl_alarm    = 1'b0;
        lbl_colour   = 24'h00FF60;
        lbl_row      = 5'd0;
        lbl_col      = 7'd0;
        lbl_cell     = 2'd0;
        lbl_cell_col = 7'd0;
        lbl_ch       = 4'd0;
        lbl_digit    = 4'd0;
        lbl_score    = 7'd0;
        // Lowest slot first; the loop runs backwards so slot 0 wins a
        // (vanishingly rare) overlap and the tag colour stays predictable.
        for (li = NB - 1; li >= 0; li = li - 1) begin
            if (lbl_hit[li]) begin
                lbl_any   = 1'b1;
                lbl_alarm = alarm[li];
                lbl_row   = lbl_arow[li*5 +: 5];
                lbl_col   = lbl_acol[li*7 +: 7];
                lbl_score = score[li*7 +: 7];
                lbl_digit = 4'd0;

                // 24-pixel person tags == three 8-wide tiles; the 48-pixel
                // alarm tag == two 24-wide cells.
                lbl_cell_col = lbl_col;
                if (lbl_alarm) begin
                    lbl_cell = (lbl_cell_col >= 7'd24) ? 2'd1 : 2'd0;
                    if (lbl_cell == 2'd1)
                        lbl_cell_col = lbl_cell_col - 7'd24;
                    lbl_ch     = {3'd0, lbl_cell[0]};
                    lbl_colour = 24'hFF0000;
                end else begin
                    lbl_cell     = {1'b0, lbl_cell_col[4:3]};
                    lbl_cell_col = {2'b0, lbl_cell_col[2:0]};
                    if (lbl_cell == 2'd2) begin
                        lbl_ch = 4'd12;                      // %
                    end else begin
                        lbl_digit = (lbl_cell == 2'd0)
                                    ? tens_digit(lbl_score)
                                    : ones_digit(lbl_score);
                        lbl_ch = 4'd2 + lbl_digit;
                    end
                    lbl_colour = slot_colour(li[1:0]);
                end
            end
        end
    end

    // The Chinese cell bitmaps put the leftmost column in bit 23
    // (tools/gen_glyphs.py and the original 人 cell agree), so that pixel
    // test is a right shift by 23 minus the in-cell column.  The small font
    // needs no shift at all: the digit sits in tile columns 2..4, the percent
    // sign in 1..5, and the tile column picks one bit of the row group.
    wire [23:0] lbl_bits = glyph_row(lbl_ch[0], lbl_row);
    wire [2:0]  lbl_dgrp = digit_group(lbl_digit, lbl_row[2:0]);
    wire [4:0]  lbl_pgrp = percent_group(lbl_row[2:0]);
    reg         lbl_on;
    always @(*) begin
        if (lbl_alarm) begin
            lbl_on = (lbl_bits >> (7'd23 - lbl_cell_col)) & 1'b1;
        end else if (lbl_ch == 4'd12) begin
            lbl_on = ((lbl_cell_col >= 7'd1) && (lbl_cell_col <= 7'd5))
                     ? lbl_pgrp[7'd4 - (lbl_cell_col - 7'd1)] : 1'b0;
        end else begin
            lbl_on = ((lbl_cell_col >= 7'd2) && (lbl_cell_col <= 7'd4))
                     ? lbl_dgrp[7'd2 - (lbl_cell_col - 7'd2)] : 1'b0;
        end
    end

    always @(*) begin
        if (!de)                     rgb_out = 24'h000000;
        // The tag hangs on the box's top-left corner, so it has to win over
        // the edge pixel it overlaps - otherwise the first glyph column is
        // painted over by the rectangle colour.
        else if (lbl_any)            rgb_out = lbl_on ? lbl_colour : 24'h000000;
        else if (any_alarm_edge)     rgb_out = 24'hFF0000;   // confirmed fall
        else if (|edge_hit)          rgb_out = track_rgb;    // tracked person
        else if (any_alarm_inside)   rgb_out = {rgb_in[23:17], 1'b0,
                                                rgb_in[15:9],  1'b0,
                                                rgb_in[7:1],   1'b0};
        else if (any_alarm_box)      rgb_out = rgb_in;
        else                         rgb_out = rgb_in;
    end

endmodule
