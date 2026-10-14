//===========================================================================
// tb_status_ae.v -- byte format of the exposure telemetry
//
// SHOW_SLOT=1 and SHOW_AE=1 together produce the line the board build prints:
//
//   S<slot> F<frame:4> <found:1> <x:3> <y:3> <w:3> <h:3> <area:5> <state:1>
//   A<mean:2><shutter:3><gain:2><zone:1>
//
// 45 bytes, checked here byte for byte against a hand-written payload so a
// miscounted step in the formatter cannot reach the board.
//===========================================================================
`timescale 1ns / 1ps

module tb_status_ae;

localparam integer DIV    = 208;      // 24 MHz / 115200
localparam integer BIT_NS = 8667;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg trig = 1'b0;

wire tx, ready;

status_uart #(
    .CLK_HZ    (24000000),
    .BAUD      (115200),
    .SHOW_SLOT (1),
    .SHOW_AE   (1)
) u_uart (
    .clk        (clk),
    .rst_n      (rst_n),
    .trig       (trig),
    .slot       (2'd2),
    .frame_cnt  (16'h0123),
    .found      (1'b1),
    .bx         (9'h0A5),
    .by         (9'h05A),
    .bw         (9'h028),
    .bh         (9'h014),
    .area       (17'h1A4C),
    .state      (3'd2),
    .ae_mean    (8'h7D),
    .ae_shutter (16'h0208),
    .ae_gain    (8'h09),
    .ae_zone    (2'd1),
    .tx         (tx),
    .ready      (ready)
);

always #20.8335 clk = ~clk;

integer errors = 0;
integer nbytes = 0;
reg [7:0] rxline [0:63];
integer i, bit;
reg [7:0] rxb;

always begin
    @(negedge tx);
    #(BIT_NS + BIT_NS / 2);
    for (bit = 0; bit < 8; bit = bit + 1) begin
        rxb[bit] = tx;
        #(BIT_NS);
    end
    if (nbytes < 64) begin
        rxline[nbytes] = rxb;
        nbytes = nbytes + 1;
    end
end

reg [8*64-1:0] got;

initial begin
    repeat (8) @(negedge clk);
    rst_n = 1'b1;
    repeat (4) @(negedge clk);

    @(negedge clk); trig = 1'b1;
    @(negedge clk); trig = 1'b0;

    // 45 bytes at 86.8 us = 3.9 ms, plus the tail of the shifter
    #5000000;

    got = 0;
    for (i = 0; i < nbytes && i < 64; i = i + 1) begin
        got = got << 8;
        got = got | rxline[i];
    end

    if ((nbytes != 45) ||
        (got !== "S2 F0123 1 0A5 05A 028 014 01A4C 2 7D208091\015\012")) begin
        errors = errors + 1;
        $display("FAIL: %0d bytes", nbytes);
        $display("     got  |%0s|", got);
        $display("     want |S2 F0123 1 0A5 05A 028 014 01A4C 2 7D208091|");
    end else begin
        $display("line ok (%0d bytes): %0s", nbytes, got);
    end

    if (errors == 0) $display("PASS tb_status_ae: exposure telemetry line matches");
    else             $display("FAIL tb_status_ae: %0d errors", errors);
    $finish;
end

initial begin
    #20000000;
    $display("FAIL tb_status_ae: timeout");
    $finish;
end

endmodule
