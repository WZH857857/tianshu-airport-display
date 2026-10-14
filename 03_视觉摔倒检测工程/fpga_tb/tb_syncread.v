// Minimal sanity check of the synchronous-read idiom used by multi_bbox:
// address registered on one edge, data wire valid on the next cycle.
`timescale 1ns / 1ps

module tb_syncread;
reg clk = 0;
reg [3:0] addr = 0;
reg [7:0] mem [0:15];
reg [3:0] dly = 0;
wire [7:0] q = mem[addr];

always #5 clk = ~clk;

always @(posedge clk) begin
    dly <= addr;
end

integer i;
initial begin
    for (i = 0; i < 16; i = i + 1)
        mem[i] = i * 3;
    repeat (2) @(negedge clk);
    for (i = 0; i < 8; i = i + 1) begin
        @(negedge clk);
        addr = i;
        @(posedge clk); #1;
        $display("cycle %0d dly=%0d q=%0d addr=%0d", i, dly, q, addr);
    end
    $finish;
end
endmodule
