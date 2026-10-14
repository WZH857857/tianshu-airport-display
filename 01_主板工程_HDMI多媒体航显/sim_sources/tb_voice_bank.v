`timescale 1ns / 1ps
//=============================================================================
// tb_voice_bank.v - split voice sample memory
//
// The bank is stored as a 32768 word and an 8192 word array so that the
// synthesiser maps it to 40 block RAMs instead of 64.  The two arrays sit
// behind one flat address space, so the bench writes and reads entries on both
// sides of the 32768 boundary, checks the last valid address, and checks that
// the read data still arrives one read clock after the address.
//=============================================================================
module tb_voice_bank;

    localparam integer WORDS = 40960;

    reg         wr_clk;
    reg         wr_en;
    reg  [15:0] wr_addr;
    reg  [7:0]  wr_data;

    reg         rd_clk;
    reg  [15:0] rd_addr;
    wire [7:0]  rd_data;

    integer errors;

    voice_bank #(
        .WORDS (WORDS),
        .AW    (16)
    ) dut (
        .wr_clk  (wr_clk),
        .wr_en   (wr_en),
        .wr_addr (wr_addr),
        .wr_data (wr_data),
        .rd_clk  (rd_clk),
        .rd_addr (rd_addr),
        .rd_data (rd_data)
    );

    always #10 wr_clk = ~wr_clk;   // 50 MHz stream side
    always #40 rd_clk = ~rd_clk;   // 12.5 MHz audio side

    task wr_one(input [15:0] a, input [7:0] d);
        begin
            @(negedge wr_clk);
            wr_addr = a;
            wr_data = d;
            wr_en   = 1'b1;
            @(negedge wr_clk);
            wr_en   = 1'b0;
        end
    endtask

    task rd_check(input [15:0] a, input [7:0] exp, input [127:0] what);
        reg [7:0] got;
        begin
            @(negedge rd_clk);
            rd_addr = a;
            @(posedge rd_clk);       // address registered here
            @(posedge rd_clk);       // data registered here
            #1 got = rd_data;
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL %0s addr=%0d got=%02x exp=%02x", what, a, got, exp);
            end
        end
    endtask

    initial begin
        errors  = 0;
        wr_clk  = 1'b0;
        rd_clk  = 1'b0;
        wr_en   = 1'b0;
        wr_addr = 16'd0;
        wr_data = 8'd0;
        rd_addr = 16'd0;

        // entries that straddle the 32768 boundary and the last valid word
        wr_one(16'd0,     8'h11);
        wr_one(16'd32767, 8'h22);
        wr_one(16'd32768, 8'h33);
        wr_one(16'd32769, 8'h44);
        wr_one(16'd40959, 8'h55);

        rd_check(16'd0,     8'h11, "first");
        rd_check(16'd32767, 8'h22, "last of low array");
        rd_check(16'd32768, 8'h33, "first of high array");
        rd_check(16'd32769, 8'h44, "second of high array");
        rd_check(16'd40959, 8'h55, "last valid");

        if (errors == 0)
            $display("RESULT: PASS tb_voice_bank");
        else
            $display("RESULT: FAIL tb_voice_bank errors=%0d", errors);
        $finish;
    end

endmodule
