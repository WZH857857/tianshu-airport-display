//===========================================================================
// tb_box_overlay.v -- does every tracked slot get its own colour?
//
// The overlay is a pure combinational pixel mapper, so the bench drives the
// raster coordinates of one known edge pixel per slot and reads back the
// colour:
//
//   slot 0 green   00 FF 60      slot 2 amber    FF C4 00
//   slot 1 cyan    00 C8 FF      slot 3 magenta  FF 40 FF
//
// and then checks that a confirmed fall overrides the hue with red while a
// pixel away from every box is passed through untouched.
//===========================================================================
`timescale 1ns / 1ps

module tb_box_overlay;

    localparam integer NB = 4;

    reg         de = 1'b1;
    reg  [9:0]  px = 10'd0;
    reg  [9:0]  py = 10'd0;
    reg  [23:0] rgb_in = 24'h123456;
    reg  [NB-1:0] found = 4'd0;
    reg  [NB-1:0] alarm = 4'd0;

    reg  [NB*9-1:0] bx = 0, by = 0, bw = 0, bh = 0;
    reg  [NB*7-1:0] score = 0;
    wire [23:0] rgb_out;

    integer errors = 0;
    integer i;

    box_overlay #(.NB (NB)) dut (
        .de      (de),
        .px      (px),
        .py      (py),
        .rgb_in  (rgb_in),
        .found   (found),
        .alarm   (alarm),
        .bx      (bx),
        .by      (by),
        .bw      (bw),
        .bh      (bh),
        .score   (score),
        .rgb_out (rgb_out)
    );

    task set_box(input integer s, input integer x0, input integer y0,
                 input integer w, input integer h);
        begin
            bx[s*9 +: 9] = x0;
            by[s*9 +: 9] = y0;
            bw[s*9 +: 9] = w;
            bh[s*9 +: 9] = h;
        end
    endtask

    task check_rgb(input [23:0] want, input [255:0] what);
        begin
            if (rgb_out !== want) begin
                errors = errors + 1;
                $display("FAIL %0s: got %06h want %06h at px=%0d py=%0d",
                         what, rgb_out, want, px, py);
            end else begin
                $display("ok   %0s -> %06h", what, rgb_out);
            end
        end
    endtask

    initial begin
        // Four well separated boxes; the top-left corner of each is an edge.
        for (i = 0; i < NB; i = i + 1)
            set_box(i, 20 + i * 40, 10 + i * 20, 30, 20);

        for (i = 0; i < NB; i = i + 1) begin
            found = (4'd1 << i);
            alarm = 4'd0;
            px = 2 * (20 + i * 40);            // vx = x0
            py = ((8 * (10 + i * 20)) + 2) / 3; // vy = y0 (ceil of 8*y0/3)
            #1;
            case (i)
                0: check_rgb(24'h00FF60, "slot0 green  ");
                1: check_rgb(24'h00C8FF, "slot1 cyan   ");
                2: check_rgb(24'hFFC400, "slot2 amber  ");
                3: check_rgb(24'hFF40FF, "slot3 magenta");
            endcase
        end

        // A confirmed fall on slot 2 must beat the palette.
        found = 4'd0; alarm = 4'd0;
        found[2] = 1'b1; alarm[2] = 1'b1;
        px = 2 * 100; py = ((8 * 50) + 2) / 3;
        #1;
        check_rgb(24'hFF0000, "slot2 alarm  ");

        // ---- name tags -------------------------------------------------
        // `87%` hangs on the box on the slot colour.  It is a 24 x 5 vision
        // pixel tag: three 8-wide tiles, a digit in tile columns 2..4 and the
        // percent sign in 1..5, checked column by column in
        // tools/label_font_check.py.  Box 0 sits at y0 = 10, only four vision
        // rows below the top of the raster, so the tag goes below the box top
        // (vision rows 4..8) and the black backing has to win over the box
        // edge it overlaps.
        found = 4'd0; alarm = 4'd0;
        found[0] = 1'b1;
        score[0*7 +: 7] = 7'd87;
        px = 2 * (20 + 2); py = 10'd11;        // tens 8, top bar (vy = 4)
        #1;
        check_rgb(24'h00FF60, "tag 8 pixel  ");
        px = 2 * 20; py = 10'd11;              // left margin of the tag
        #1;
        check_rgb(24'h000000, "tag backing  ");
        px = 2 * (20 + 10); py = 10'd11;       // ones 7, top bar
        #1;
        check_rgb(24'h00FF60, "tag 7 pixel  ");
        px = 2 * (20 + 19); py = 10'd11;       // between the % dots
        #1;
        check_rgb(24'h000000, "tag % gap    ");
        px = 2 * (20 + 17); py = 10'd11;       // % top-left dot
        #1;
        check_rgb(24'h00FF60, "tag % pixel  ");

        // 摔倒 on the alarmed slot: 摔 row 1 is 0x181800 (in-cell columns
        // 3/4 and 11/12), 倒 row 1 is 0x0C0060 (columns 4/5 and 17/18 of the
        // second cell).  Row 1 of a box at y0 = 50 is vision row 32, so
        // py = (8*32 + 2)/3 = 86.
        found = 4'd0; alarm = 4'd0;
        found[2] = 1'b1; alarm[2] = 1'b1;
        score[2*7 +: 7] = 7'd0;
        px = 2 * 103; py = 10'd86;             // 摔, in-cell column 3
        #1;
        check_rgb(24'hFF0000, "tag 摔 pixel ");
        px = 2 * 106; py = 10'd86;             // 摔, inside its right gap
        #1;
        check_rgb(24'h000000, "tag 摔 gap   ");
        px = 2 * 128; py = 10'd86;             // 倒, in-cell column 4
        #1;
        check_rgb(24'hFF0000, "tag 倒 pixel ");

        // Away from every box the picture passes through unchanged.
        found = 4'd0; alarm = 4'd0;
        px = 10'd600; py = 10'd400;
        #1;
        check_rgb(24'h123456, "pass-through ");

        // With no pixels enabled the output is black.
        de = 1'b0; #1;
        check_rgb(24'h000000, "blanking     ");

        if (errors == 0) $display("PASS tb_box_overlay: per-slot colours + alarm override");
        else             $display("FAIL tb_box_overlay: %0d errors", errors);
        $finish;
    end

endmodule
