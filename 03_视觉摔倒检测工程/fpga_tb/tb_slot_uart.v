//===========================================================================
// tb_slot_uart.v -- slot_scan + status_uart in the crowd configuration
//
// Drives one snap pulse carrying four different slot results, receives the
// four 115200 lines byte by byte, and compares each line against the string
// the documented format calls for:
//
//     S<slot> F<frame:4> <found:1> <x:3> <y:3> <w:3> <h:3> <area:5> <state:1>
//
// It also walks a second burst with a different payload, so a stuck or
// double-firing trigger cannot pass.
//===========================================================================
`timescale 1ns / 1ps

module tb_slot_uart;

localparam integer NB     = 4;
localparam integer DIV    = 208;             // 24 MHz / 115200
localparam integer BIT_NS = 8667;            // 41.667 ns x 208

reg clk = 1'b0;
reg rst_n = 1'b0;

reg         snap = 1'b0;
reg [15:0]  frame = 16'd0;
reg [NB-1:0] found = 4'b0000;
reg [NB*3-1:0] state = 12'd0;
reg [NB*9-1:0] x = 36'd0, y = 36'd0, w = 36'd0, h = 36'd0;
reg [NB*17-1:0] area = 68'd0;

wire        trig;
wire [1:0]  slot;
wire [15:0] o_frame;
wire        o_found;
wire [8:0]  o_x, o_y, o_w, o_h;
wire [16:0] o_area;
wire [2:0]  o_state;
wire        busy;
wire        tx;
wire        ready;

slot_scan #(
    .NSLOT (NB)
) u_scan (
    .clk     (clk),
    .rst_n   (rst_n),
    .snap    (snap),
    .ready   (ready),
    .frame   (frame),
    .found   (found),
    .state   (state),
    .x       (x),
    .y       (y),
    .w       (w),
    .h       (h),
    .area    (area),
    .trig    (trig),
    .slot    (slot),
    .o_frame (o_frame),
    .o_found (o_found),
    .o_x     (o_x),
    .o_y     (o_y),
    .o_w     (o_w),
    .o_h     (o_h),
    .o_area  (o_area),
    .o_state (o_state),
    .busy    (busy)
);

status_uart #(
    .CLK_HZ    (24000000),
    .BAUD      (115200),
    .SHOW_SLOT (1)
) u_uart (
    .clk       (clk),
    .rst_n     (rst_n),
    .trig      (trig),
    .slot      (slot),
    .frame_cnt (o_frame),
    .found     (o_found),
    .bx        (o_x),
    .by        (o_y),
    .bw        (o_w),
    .bh        (o_h),
    .area      (o_area),
    .state     (o_state),
    .tx        (tx),
    .ready     (ready)
);

always #20.8335 clk = ~clk;

// ------------------------------------------------------------- receiver
integer errors = 0;
integer line_no = 0;
integer nbytes = 0;
reg [7:0] rxline [0:63];
integer i;
integer bit;
reg [7:0] rxb;

task expect_line;
    input integer idx;
    input [8*48-1:0] want;
    reg [8*48-1:0] got;
    begin
        got = 0;
        for (i = 0; i < nbytes && i < 48; i = i + 1) begin
            got = got << 8;
            got = got | rxline[i];
        end
        if ((got !== want) || (nbytes != 36)) begin
            errors = errors + 1;
            $display("FAIL line %0d (%0d bytes, want 36)", idx, nbytes);
            $display("     got  |%0s|", got);
            $display("     want |%0s|", want);
        end else begin
            $display("line %0d ok: %0s", idx, got);
        end
    end
endtask

always begin
    @(negedge tx);
    // The falling edge starts the start bit: wait half of that bit plus one
    // whole bit to land in the middle of the LSB.
    #(BIT_NS + BIT_NS / 2);
    for (bit = 0; bit < 8; bit = bit + 1) begin
        rxb[bit] = tx;
        #(BIT_NS);
    end
    if (nbytes < 64) begin
        rxline[nbytes] = rxb;
        nbytes = nbytes + 1;
    end
    if (rxb == 8'h0A) begin
        // CR LF ended the line: check it against the slot's expected string.
        case (line_no)
        0: expect_line(0, "S0 F0012 1 0A5 05A 028 014 01A4C 2\015\012");
        1: expect_line(1, "S1 F0012 0 000 000 000 000 00000 0\015\012");
        2: expect_line(2, "S2 F0012 1 00B 010 020 009 00063 3\015\012");
        3: expect_line(3, "S3 F0012 0 000 000 000 000 00000 1\015\012");
        4: expect_line(4, "S0 F0ABC 1 123 0FE 080 040 1FFFF 1\015\012");
        5: expect_line(5, "S1 F0ABC 1 045 067 012 00A 0ABCD 2\015\012");
        6: expect_line(6, "S2 F0ABC 1 1FF 1FF 1FF 1FF 1FFFF 3\015\012");
        7: expect_line(7, "S3 F0ABC 1 000 001 002 003 00004 0\015\012");
        default: begin
            errors = errors + 1;
            $display("FAIL unexpected line %0d", line_no);
        end
        endcase
        line_no = line_no + 1;
        nbytes = 0;
    end
end

task load_burst_a;
    begin
        frame = 16'h0012;
        found = 4'b0101;                       // slots 0 and 2 hit
        state = {3'd1, 3'd3, 3'd0, 3'd2};      // [3],[2],[1],[0]
        x = {9'h000, 9'h00B, 9'h000, 9'h0A5};
        y = {9'h000, 9'h010, 9'h000, 9'h05A};
        w = {9'h000, 9'h020, 9'h000, 9'h028};
        h = {9'h000, 9'h009, 9'h000, 9'h014};
        area = {17'd0, 17'd99, 17'd0, 17'h1A4C};
    end
endtask

task load_burst_b;
    begin
        frame = 16'h0ABC;
        found = 4'b1111;
        state = {3'd0, 3'd3, 3'd2, 3'd1};      // [3],[2],[1],[0]
        x = {9'h000, 9'h1FF, 9'h045, 9'h123};
        y = {9'h001, 9'h1FF, 9'h067, 9'h0FE};
        w = {9'h002, 9'h1FF, 9'h012, 9'h080};
        h = {9'h003, 9'h1FF, 9'h00A, 9'h040};
        area = {17'd4, 17'h1FFFF, 17'h0ABCD, 17'h1FFFF};
    end
endtask

initial begin
    repeat (8) @(negedge clk);
    rst_n = 1'b1;
    repeat (4) @(negedge clk);

    load_burst_a;
    @(negedge clk);
    snap = 1'b1;
    @(negedge clk);
    snap = 1'b0;

    // Four lines at ~2.9 ms each: wait for the walker to go idle again.
    // The snap pulse is registered on the edge right after it is applied, so
    // the burst can already be running here: only the fall is guaranteed to
    // lie ahead.
    @(negedge busy);
    // The formatter frees `ready` when the last byte is *loaded*, so give the
    // shifter its ten bit times before counting the decoded lines.
    #120000;      // 120 us in 1 ns units

    if (line_no != 4) begin
        errors = errors + 1;
        $display("FAIL after burst A: %0d lines (%0d bytes), want 4",
                 line_no, nbytes);
    end

    // Second burst: different payload, so a stuck trigger or a stale mux
    // cannot pass by accident.
    load_burst_b;
    @(negedge clk);
    snap = 1'b1;
    @(negedge clk);
    snap = 1'b0;
    @(negedge busy);
    #120000;

    if (line_no != 8) begin
        errors = errors + 1;
        $display("FAIL after burst B: %0d lines, want 8", line_no);
    end

    if (errors == 0) $display("PASS tb_slot_uart: 8 slot lines decoded");
    else             $display("FAIL tb_slot_uart: %0d errors", errors);
    $finish;
end

initial begin
    #40000000;    // 40 ms in 1 ns units
    $display("FAIL tb_slot_uart: timeout");
    $finish;
end

endmodule
