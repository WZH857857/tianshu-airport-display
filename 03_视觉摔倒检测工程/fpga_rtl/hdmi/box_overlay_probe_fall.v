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
// renders the bitmaps).  A tracked person prints `87%` on the slot colour -
// the confidence score is described in rtl/person_score.v; a confirmed fall
// keeps the red `摔倒` alarm tag.  The glyph cells are 24 x 18 vision pixels,
// which the 640x480 picture stretches back to a 48 x 48 square, so the tag
// is square and legible on the monitor without a scaler.
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

    // The confidence readout uses a compact 3x5 digit font instead of adding
    // eleven more 24x18 font cells.  Each font pixel becomes 5x3 vision
    // pixels, so `87%` stays legible while the extra LUT cost stays small.
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

    // F/A/L for the red FALL alarm tag.  They are the same 3x5 cell as the
    // digits, so the alarm and confidence labels share one renderer.
    function [14:0] letter_pattern;
        input [1:0] l;
        begin
            case (l)
                2'd0: letter_pattern = 15'b111100111100100;   // F
                2'd1: letter_pattern = 15'b111101111101101;   // A
                default: letter_pattern = 15'b100100100100111; // L
            endcase
        end
    endfunction

    // One font row -> cell columns.  The leftmost font pixel lands in bit 23
    // of the cell, so the highest group of the spread value comes from the
    // leftmost font pixel; spreading a bit over five/three columns is wiring,
    // not logic.
    function [14:0] spread3;
        input [2:0] g;                       // g[2] = leftmost font pixel
        begin
            spread3 = { g[2] ? 5'b11111 : 5'b00000,
                        g[1] ? 5'b11111 : 5'b00000,
                        g[0] ? 5'b11111 : 5'b00000 };
        end
    endfunction

    function [14:0] spread5;
        input [4:0] g;                       // g[4] = leftmost font pixel
        begin
            spread5 = { g[4] ? 3'b111 : 3'b000,
                        g[3] ? 3'b111 : 3'b000,
                        g[2] ? 3'b111 : 3'b000,
                        g[1] ? 3'b111 : 3'b000,
                        g[0] ? 3'b111 : 3'b000 };
        end
    endfunction

    // Row slice of the 3x5 digit font.  row/3 picks the font row, the three
    // pattern bits spread over fifteen columns and the block sits four
    // columns in, so the digit is centred in the 24-wide cell.
    function [23:0] small_row;
        input [14:0] pattern;
        input [4:0] row;
        reg [2:0]  g;
        begin
            case (row)
                5'd0,  5'd1,  5'd2:  g = pattern[14:12];
                5'd3,  5'd4,  5'd5:  g = pattern[11:9];
                5'd6,  5'd7,  5'd8:  g = pattern[8:6];
                5'd9,  5'd10, 5'd11: g = pattern[5:3];
                5'd12, 5'd13, 5'd14: g = pattern[2:0];
                default:             g = 3'd0;
            endcase
            small_row = {9'd0, spread3(g)} << 4;
        end
    endfunction

    // 5x5 percent sign: two dots, a diagonal, and two more dots.
    function [23:0] percent_row;
        input [4:0] row;
        reg [4:0] g;
        begin
            case (row)
                5'd0,  5'd1,  5'd2:  g = 5'b11001;
                5'd3,  5'd4,  5'd5:  g = 5'b11010;
                5'd6,  5'd7,  5'd8:  g = 5'b00100;
                5'd9,  5'd10, 5'd11: g = 5'b01011;
                5'd12, 5'd13, 5'd14: g = 5'b10011;
                default:             g = 5'd0;
            endcase
            percent_row = {9'd0, spread5(g)} << 4;
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

    function [23:0] glyph_pick;
        input [3:0] ch;      // 0..9 digits, 10..12 F/A/L, 13 = %
        input [4:0] row;
        begin
            if (ch <= 4'd9)
                glyph_pick = small_row(digit_pattern(ch), row);
            else if (ch <= 4'd12)
                glyph_pick = small_row(letter_pattern(ch - 4'd10), row);
            else
                glyph_pick = percent_row(row);
        end
    endfunction

    localparam integer LBL_W = 24;   // one glyph cell, vision pixels
    localparam integer LBL_H = 18;

    wire [NB-1:0]   lbl_hit;
    wire [NB*7-1:0] lbl_acol;        // column inside the tag, 0 .. 95
    wire [NB*5-1:0] lbl_arow;        // row inside the tag, 0 .. 17

    generate
        for (gi = 0; gi < NB; gi = gi + 1) begin : g_label
            wire [8:0]  x0 = bx[gi*9 +: 9];
            wire [8:0]  y0 = by[gi*9 +: 9];
            // `FALL` for an alarm, `87%` for a tracked person.
            wire [9:0]  lw  = alarm[gi] ? 10'd96 : 10'd72;
            // The tag sits above the box; a box in the top margin keeps its
            // tag inside the top-left corner instead of off-screen.  Both
            // offsets are the same wrapping subtraction as the box itself:
            // rows above the tag wrap past 1k and fail the < LBL_H test.
            wire [9:0]  dy = {1'b0, vy} - {1'b0, y0};
            wire [9:0]  dx = {1'b0, vx} - {1'b0, x0};
            wire [9:0]  ly = (y0 >= LBL_H + 1) ? (dy + (LBL_H + 1))
                                               : (dy - 10'd1);

            assign lbl_hit[gi] = found[gi] && (dx < lw) && (ly < LBL_H);
            assign lbl_acol[gi*7 +: 7] = dx[6:0];
            assign lbl_arow[gi*5 +: 5] = ly[4:0];
        end
    endgenerate

    reg             lbl_any;
    reg [23:0]      lbl_colour;
    reg [6:0]       lbl_col, lbl_cell_col, lbl_score;
    reg [4:0]       lbl_row;
    reg [3:0]       lbl_ch;
    reg [2:0]       lbl_cell;       // cell index inside the tag
    integer li;
    always @(*) begin
        lbl_any      = 1'b0;
        lbl_colour   = 24'h00FF60;
        lbl_row      = 5'd0;
        lbl_col      = 7'd0;
        lbl_cell     = 3'd0;
        lbl_cell_col = 7'd0;
        lbl_ch       = 4'd0;
        lbl_score    = 7'd0;
        // Lowest slot first; the loop runs backwards so slot 0 wins a
        // (vanishingly rare) overlap and the tag colour stays predictable.
        for (li = NB - 1; li >= 0; li = li - 1) begin
            if (lbl_hit[li]) begin
                lbl_any = 1'b1;
                lbl_row = lbl_arow[li*5 +: 5];
                lbl_col = lbl_acol[li*7 +: 7];
                lbl_score = score[li*7 +: 7];

                // Person tags are three cells; alarm tags are four.
                lbl_cell_col = lbl_col;
                lbl_cell = 3'd0;
                if (lbl_cell_col >= 7'd72) begin
                    lbl_cell = 3'd3;
                    lbl_cell_col = lbl_cell_col - 7'd72;
                end else if (lbl_cell_col >= 7'd48) begin
                    lbl_cell = 3'd2;
                    lbl_cell_col = lbl_cell_col - 7'd48;
                end else if (lbl_cell_col >= 7'd24) begin
                    lbl_cell = 3'd1;
                    lbl_cell_col = lbl_cell_col - 7'd24;
                end

                if (alarm[li]) begin
                    case (lbl_cell)
                        3'd0:    lbl_ch = 4'd10;            // F
                        3'd1:    lbl_ch = 4'd11;            // A
                        default: lbl_ch = 4'd12;            // L
                    endcase
                    lbl_colour = 24'hFF0000;
                end else begin
                    case (lbl_cell)
                        3'd0:    lbl_ch = tens_digit(lbl_score);
                        3'd1:    lbl_ch = ones_digit(lbl_score);
                        default: lbl_ch = 4'd13;             // %
                    endcase
                    lbl_colour = slot_colour(li[1:0]);
                end
            end
        end
    end

    // The cell bitmaps put the leftmost column in bit 23 (tools/gen_glyphs.py
    // and the original 人 cell agree), so the pixel test is a right shift by
    // 23 minus the in-cell column.
    wire [23:0] lbl_bits = glyph_pick(lbl_ch, lbl_row);
    wire        lbl_on   = (lbl_bits >> (7'd23 - lbl_cell_col)) & 1'b1;

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
