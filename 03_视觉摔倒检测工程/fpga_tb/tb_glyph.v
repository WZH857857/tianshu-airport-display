`timescale 1ns / 1ps
// Direct probe of box_overlay's internal glyph_pick: which columns of a
// 24-pixel cell light for each character code at row 0?
module tb_glyph;
    integer ch, col;
    reg [23:0] bits;
    initial begin
        for (ch = 0; ch <= 13; ch = ch + 1) begin
            bits = box_overlay.glyph_pick(ch[3:0], 5'd0);
            $write("ch=%0d bits=%06h cols:", ch, bits);
            for (col = 0; col < 24; col = col + 1)
                if ((bits >> (23 - col)) & 1'b1) $write(" %0d", col);
            $write("\n");
        end
        $finish;
    end
endmodule
