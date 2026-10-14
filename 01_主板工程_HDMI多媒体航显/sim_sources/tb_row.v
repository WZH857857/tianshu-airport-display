`timescale 1ns / 1ps
//=============================================================================
// tb_row.v - focused check: fill one row, then read it back as one burst.
// Prints the model memory around the end of the row so a missing write can be
// told apart from a read capture problem.
//=============================================================================
module tb_row;

    reg  clk   = 1'b0;
    reg  rst_n = 1'b0;
    always #4 clk = ~clk;

    reg  [20:0] rd_addr  = 21'd0;
    reg  [10:0] rd_len   = 11'd1;
    reg         rd_start = 1'b0;
    wire        rd_ack, rd_busy, rd_valid, rd_last;
    wire [31:0] rd_data;

    reg  [20:0] wr_addr  = 21'd0;
    reg  [31:0] wr_data  = 32'd0;
    reg         wr_start = 1'b0;
    wire        wr_ack, wr_done;

    wire        init_done, cal_done, cal_fail;
    wire [3:0]  cal_latency;
    wire [15:0] refresh_cnt;

    wire        sdram_clk, sdram_cke, sdram_cs_n;
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;

    integer     i, n;
    integer     errors = 0;
    reg  [31:0] got;

    sdram_ctrl #(
        .PWRUP_CYC (200),
        .CAL_MAX   (6)
    ) u_dut (
        .clk (clk), .rst_n (rst_n),
        .rd_start (rd_start), .rd_addr (rd_addr), .rd_len (rd_len),
        .rd_ack (rd_ack), .rd_busy (rd_busy), .rd_valid (rd_valid),
        .rd_last (rd_last), .rd_data (rd_data),
        .wr_start (wr_start), .wr_addr (wr_addr), .wr_data (wr_data),
        .wr_ack (wr_ack), .wr_done (wr_done),
        .init_done (init_done), .cal_done (cal_done), .cal_fail (cal_fail),
        .cal_latency (cal_latency), .refresh_cnt (refresh_cnt),
        .sdram_clk (sdram_clk), .sdram_cke (sdram_cke), .sdram_cs_n (sdram_cs_n),
        .sdram_ras_n (sdram_ras_n), .sdram_cas_n (sdram_cas_n),
        .sdram_we_n (sdram_we_n), .sdram_addr (sdram_addr), .sdram_ba (sdram_ba),
        .sdram_dm (sdram_dm), .sdram_dq (sdram_dq)
    );

    sdram_model #(.CL(3)) u_sdram (
        .clk (clk), .cke (sdram_cke), .cs_n (sdram_cs_n),
        .ras_n (sdram_ras_n), .cas_n (sdram_cas_n), .we_n (sdram_we_n),
        .addr (sdram_addr), .ba (sdram_ba), .dq (sdram_dq), .dm (sdram_dm)
    );

    function [20:0] mkaddr;
        input integer bank;
        input integer row;
        input integer col;
        reg [1:0] b;
        reg [9:0] r;
        reg [8:0] c;
        begin
            b = bank; r = row; c = col;
            mkaddr = {r, b, c};
        end
    endfunction

    function [31:0] patt;
        input [20:0] a;
        begin
            patt = {11'h000, a} ^ 32'hC0DE_1234;
        end
    endfunction

    task wr_word;
        input [20:0] a;
        input [31:0] d;
        begin
            @(negedge clk);
            wr_start = 1'b1; wr_addr = a; wr_data = d;
            @(posedge clk);
            @(negedge clk);
            while (!wr_ack) begin @(posedge clk); @(negedge clk); end
            wr_start = 1'b0;
            while (!wr_done) begin @(posedge clk); @(negedge clk); end
        end
    endtask

    task rd_burst;
        input [20:0] a;
        input integer len;
        begin
            @(negedge clk);
            rd_start = 1'b1; rd_addr = a; rd_len = len;
            @(posedge clk);
            @(negedge clk);
            while (!rd_ack) begin @(posedge clk); @(negedge clk); end
            rd_start = 1'b0;

            n = 0;
            while (n < len) begin
                @(negedge clk);
                if (rd_valid) begin
                    got = rd_data;
                    if (got !== patt(a + n)) begin
                        errors = errors + 1;
                        $display("  MISMATCH addr=%06x got=%08x exp=%08x (mem=%08x)",
                                 a + n, got, patt(a + n), u_sdram.mem[a + n]);
                    end
                    n = n + 1;
                end
            end
        end
    endtask

    initial begin
        repeat (20) @(negedge clk);
        rst_n = 1'b1;
        while (!cal_done) @(negedge clk);
        $display("cal_done=%0b latency=%0d at %0t", cal_done, cal_latency, $time);

        // fill bank 1 row 10 completely
        for (i = 0; i < 512; i = i + 1)
            wr_word(mkaddr(1, 10, i), patt(mkaddr(1, 10, i)));

        $display("after the write pass:");
        for (i = 508; i < 512; i = i + 1)
            $display("  mem col %0d = %08x", i, u_sdram.mem[mkaddr(1, 10, i)]);

        // single word read of the last column
        rd_burst(mkaddr(1, 10, 511), 1);
        $display("single read col 511 -> got=%08x errors=%0d", got, errors);

        // full row burst
        errors = 0;
        rd_burst(mkaddr(1, 10, 0), 512);
        $display("full row burst errors=%0d", errors);

        // second burst, page must still be open
        errors = 0;
        rd_burst(mkaddr(1, 10, 0), 512);
        $display("second full row burst errors=%0d", errors);

        $display("model stats: ACT=%0d PRE=%0d REF=%0d RD=%0d WR=%0d errors=%0d",
                 u_sdram.n_act, u_sdram.n_pre, u_sdram.n_ref,
                 u_sdram.n_rd, u_sdram.n_wr, u_sdram.n_err);
        $finish;
    end

endmodule
