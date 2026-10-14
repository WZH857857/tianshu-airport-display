`timescale 1ns / 1ps
module tb_probe;
    reg         de = 1'b1;
    reg  [9:0]  px = 10'd0;
    reg  [9:0]  py = 10'd0;
    reg  [23:0] rgb_in = 24'h123456;
    reg  [3:0]  found = 4'd0;
    reg  [3:0]  alarm = 4'd0;
    reg  [35:0] bx = 0, by = 0, bw = 0, bh = 0;
    reg  [27:0] score = 0;
    wire [23:0] rgb_out;
    integer i, lit;

    box_overlay #(.NB(4)) dut (
        .de(de), .px(px), .py(py), .rgb_in(rgb_in),
        .found(found), .alarm(alarm), .bx(bx), .by(by),
        .bw(bw), .bh(bh), .score(score), .rgb_out(rgb_out));

    // Second instance used as a glyph reader: drive one box over a wide
    // label and read the lit columns of each character position.
    wire [23:0] g_rgb;
    box_overlay #(.NB(4)) gdut (
        .de(de), .px(px), .py(py), .rgb_in(24'h000000),
        .found(found), .alarm(alarm), .bx(bx), .by(by),
        .bw(bw), .bh(bh), .score(score), .rgb_out(g_rgb));

    initial begin
        bx[8:0] = 20; by[8:0] = 10;
        bw[8:0] = 30; bh[8:0] = 20;
        found[0] = 1'b1;
        score[6:0] = 7'd87;
        #1;
        for (i = 0; i < 96; i = i + 1) begin
            px = 2 * (20 + i);
            py = 10'd30;
            #1;
            if (dut.lbl_on || (i >= 24 && i <= 36))
                $display("i=%0d lbl_col=%0d cellcol=%0d cell=%0d ch=%0d bits=%06h on=%0d",
                         i, dut.lbl_col, dut.lbl_cell_col, dut.lbl_cell,
                         dut.lbl_ch, dut.lbl_bits, dut.lbl_on);
        end
        $display("total lit = %0d", lit);

        found = 4'd0; alarm = 4'd0;
        found[2] = 1'b1; alarm[2] = 1'b1;
        bx[2*9 +: 9] = 100; by[2*9 +: 9] = 50;
        bw[2*9 +: 9] = 30; bh[2*9 +: 9] = 20;
        lit = 0;
        #1;
        $display("box2: x0=%0d y0=%0d lw=%0d lbl_hit2=%0d found=%b alarm=%b",
                 dut.bx[2*9 +: 9], dut.by[2*9 +: 9],
                 dut.g_label[2].lw, dut.lbl_hit[2], found, alarm);
        for (i = 0; i < 48; i = i + 1) begin
            px = 2 * (100 + i);
            py = 10'd84;
            #1;
            if (dut.lbl_on)
                $display("alarm lit out_col=%0d ch=%0d cellcol=%0d bits=%06h",
                         i, dut.lbl_ch, dut.lbl_cell_col, dut.lbl_bits);
        end
        $display("alarm total lit = %0d", lit);
        $finish;
    end
endmodule
