`timescale 1ns / 1ps
// Pass-through harness: expose row slices of one glyph cell and of the
// compact digit font so a test can read them without poking inside the DUT.
module cell_probe (
    input  wire [3:0] ch,
    input  wire [4:0] row,
    output wire [23:0] bits
);
    assign bits = box_overlay.glyph_pick(ch, row);
endmodule

module tb_cell;
    reg  [3:0] ch;
    reg  [4:0] row;
    wire [23:0] bits;
    integer col;
    cell_probe dut (.ch(ch), .row(row), .bits(bits));
    initial begin
        for (ch = 0; ch <= 12; ch = ch + 1) begin
            row = 5'd0;
            #1;
            $write("ch=%0d row0 bits=%06h cols:", ch, bits);
            for (col = 0; col < 24; col = col + 1)
                if ((bits >> (23 - col)) & 1'b1) $write(" %0d", col);
            $write("\n");
        end
        // row slices of digit 8
        ch = 4'd10;
        for (row = 0; row < 5; row = row + 1) begin
            #1;
            $display("digit8 row%0d = %06h", row, bits);
        end
        $finish;
    end
endmodule
