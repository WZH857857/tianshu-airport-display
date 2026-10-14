`timescale 1ns / 1ps
// small focused debug bench: watch the SDRAM bus around one write and one read
module tb_debug;

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
    wire        wr_ack;

    wire        init_done, cal_done, cal_fail;
    wire [3:0]  cal_latency;
    wire [15:0] refresh_cnt;

    wire        sdram_clk, sdram_cke, sdram_cs_n;
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;

    integer     t0;

    sdram_ctrl #(
        .PWRUP_CYC (200),               // short power up wait for the debug run
        .CAL_MAX   (6)
    ) u_dut (
        .clk (clk), .rst_n (rst_n),
        .rd_start (rd_start), .rd_addr (rd_addr), .rd_len (rd_len),
        .rd_ack (rd_ack), .rd_busy (rd_busy), .rd_valid (rd_valid),
        .rd_last (rd_last), .rd_data (rd_data),
        .wr_start (wr_start), .wr_addr (wr_addr), .wr_data (wr_data),
        .wr_ack (wr_ack),
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

    // watch which clock cycle the model drives data on
    reg [3:0] prev_dv;
    always @(posedge clk) prev_dv <= u_sdram.dv[3:0];

    // trace the calibration phase
    initial begin
        #1;
        while (cal_done !== 1'b1) begin
            @(posedge clk); #1;
            if (init_done === 1'b1)
                $display("%6t CAL st=%0d sel=%0d wc=%0d rc=%0d cap=%0d op=%06x rdv=%0b rdd=%08x dq=%08x",
                         $time, u_dut.state, u_dut.cal_sel, u_dut.cal_wr_cnt,
                         u_dut.cal_rd_cnt, u_dut.cap_cnt, u_dut.op_addr,
                         rd_valid, rd_data, sdram_dq);
        end
    end

    initial begin
        $dumpfile("tb_debug.vcd");
        $dumpvars(0, tb_debug);
        repeat (20) @(negedge clk);
        rst_n = 1'b1;

        while (!cal_done) @(negedge clk);
        $display("");
        $display("cal_done=%0b cal_fail=%0b latency=%0d at %0t",
                 cal_done, cal_fail, cal_latency, $time);

        // one write
        @(negedge clk);
        wr_start = 1; wr_addr = 21'h000123; wr_data = 32'hdead_beef;
        @(posedge clk); @(negedge clk);
        while (!wr_ack) begin @(posedge clk); @(negedge clk); end
        wr_start = 0;
        $display("write accepted at %0t", $time);

        // watch the bus for the next 30 clocks
        $display(" time  state opaddr  cmd(hit)  cmdba cmdadr   dq(for rd)   rdvalid rddata");
        repeat (30) begin
            @(posedge clk); #1;
            $display("%6t  %2d  %06x  %b%b%b     %0d  %04x  %08x   %b  %08x",
                     $time, u_dut.state, u_dut.op_addr,
                     sdram_ras_n, sdram_cas_n, sdram_we_n,
                     sdram_ba, sdram_addr, sdram_dq, rd_valid, rd_data);
        end

        // one read burst of 4
        @(negedge clk);
        rd_start = 1; rd_addr = 21'h000123; rd_len = 4;
        @(posedge clk); @(negedge clk);
        while (!rd_ack) begin @(posedge clk); @(negedge clk); end
        rd_start = 0;
        $display("");
        $display("read accepted at %0t", $time);
        repeat (40) begin
            @(posedge clk); #1;
            $display("%6t  %2d  %06x  %b%b%b     %0d  %04x  %08x   %b  %08x  wait=%0d cap=%0b dv=%b%0b%0b%0b",
                     $time, u_dut.state, u_dut.op_addr,
                     sdram_ras_n, sdram_cas_n, sdram_we_n,
                     sdram_ba, sdram_addr, sdram_dq, rd_valid, rd_data,
                     u_dut.rd_wait_cnt, u_dut.rd_cap_act,
                     u_sdram.dv[3], u_sdram.dv[2], u_sdram.dv[1], u_sdram.dv[0]);
        end
        $finish;
    end

endmodule
