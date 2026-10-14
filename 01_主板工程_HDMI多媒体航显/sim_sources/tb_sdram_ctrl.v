`timescale 1ns / 1ps
//=============================================================================
// tb_sdram_ctrl.v - self checking test bench for sdram_ctrl
//
//   * power up, calibration and protocol checks
//   * single word read/write over bank and row changes
//   * full row burst reads, row hits and repeat accesses
//   * interleaved video style burst reads plus scattered writes
//   * auto refresh activity
//=============================================================================
module tb_sdram_ctrl;

    reg  clk   = 1'b0;
    reg  rst_n = 1'b0;

    always #4 clk = ~clk;               // 125 MHz

    // controller interface
    reg  [20:0] rd_addr;
    reg  [10:0] rd_len;
    reg         rd_start;
    wire        rd_ack, rd_busy, rd_valid, rd_last;
    wire [31:0] rd_data;

    reg  [20:0] wr_addr;
    reg  [31:0] wr_data;
    reg         wr_start;
    wire        wr_ack, wr_done;

    wire        init_done, cal_done, cal_fail;
    wire [3:0]  cal_latency;
    wire [15:0] refresh_cnt;

    // SDRAM pins
    wire        sdram_clk, sdram_cke, sdram_cs_n;
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;

    integer     errors = 0;
    integer     checks = 0;
    integer     i, j, k, n;
    reg  [31:0] exp_word;
    reg  [31:0] got;
    reg  [20:0] offs;
    integer     t0;

    //------------------------------------------------------------------
    sdram_ctrl #(
        .T_RP        (3),
        .T_RCD       (3),
        .T_MRD       (3),
        .T_RFC       (9),
        .T_WR_GAP    (4),
        .T_RD_GAP    (3),
        .REFRESH_CYC (975),
        .PWRUP_CYC   (25000),
        .CAL_MIN     (1),
        .CAL_MAX     (10),
        .CAL_ADDR    (21'h1F_FFFC),
        .MODE_REG    (11'h030)
    ) u_dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .rd_start    (rd_start),
        .rd_addr     (rd_addr),
        .rd_len      (rd_len),
        .rd_ack      (rd_ack),
        .rd_busy     (rd_busy),
        .rd_valid    (rd_valid),
        .rd_last     (rd_last),
        .rd_data     (rd_data),
        .wr_start    (wr_start),
        .wr_addr     (wr_addr),
        .wr_data     (wr_data),
        .wr_ack      (wr_ack),
        .wr_done     (wr_done),
        .init_done   (init_done),
        .cal_done    (cal_done),
        .cal_fail    (cal_fail),
        .cal_latency (cal_latency),
        .refresh_cnt (refresh_cnt),
        .sdram_clk   (sdram_clk),
        .sdram_cke   (sdram_cke),
        .sdram_cs_n  (sdram_cs_n),
        .sdram_ras_n (sdram_ras_n),
        .sdram_cas_n (sdram_cas_n),
        .sdram_we_n  (sdram_we_n),
        .sdram_addr  (sdram_addr),
        .sdram_ba    (sdram_ba),
        .sdram_dm    (sdram_dm),
        .sdram_dq    (sdram_dq)
    );

    sdram_model #(
        .CL   (3),
        .T_RCD(3),
        .T_RP (3),
        .T_RAS(5),
        .T_RFC(8)
    ) u_sdram (
        .clk   (clk),
        .cke   (sdram_cke),
        .cs_n  (sdram_cs_n),
        .ras_n (sdram_ras_n),
        .cas_n (sdram_cas_n),
        .we_n  (sdram_we_n),
        .addr  (sdram_addr),
        .ba    (sdram_ba),
        .dq    (sdram_dq),
        .dm    (sdram_dm)
    );

    //------------------------------------------------------------------
    function [20:0] mkaddr;
        input integer bank;
        input integer row;
        input integer col;
        reg [1:0] b;
        reg [9:0] r;
        reg [8:0] c;
        begin
            b = bank;
            r = row;
            c = col;
            mkaddr = {r, b, c};
        end
    endfunction

    function [31:0] patt;
        input [20:0] a;
        begin
            patt = {11'h000, a} ^ 32'hC0DE_1234;
        end
    endfunction

    //------------------------------------------------------------------
    // background "image loader" used by test 4
    //
    // It walks 1024 sequential words in bank 2, rows 300 / 301, i.e. the
    // worst case for the display port: same bank, different row.
    //------------------------------------------------------------------
    integer     LW_MAX = 1024;
    reg         loader_run    = 1'b0;
    reg         loader_active = 1'b0;
    integer     lw            = 0;      // total words written, never wraps
    reg  [20:0] la;

    function [20:0] ldr_addr;
        input integer n;
        begin
            ldr_addr = mkaddr(2, 300 + (n / 512), n % 512);
        end
    endfunction

    initial begin
        forever begin
            @(posedge clk);
            if (loader_run === 1'b1 && lw < LW_MAX) begin
                loader_active = 1'b1;
                la = ldr_addr(lw);
                @(negedge clk);
                wr_start = 1'b1;
                wr_addr  = la;
                wr_data  = patt(la);
                @(posedge clk);
                @(negedge clk);
                while (!wr_ack) begin
                    @(posedge clk);
                    @(negedge clk);
                end
                wr_start = 1'b0;
                while (!wr_done) begin
                    @(posedge clk);
                    @(negedge clk);
                end
                lw = lw + 1;
                loader_active = 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    task wr_word;
        input [20:0] a;
        input [31:0] d;
        begin
            @(negedge clk);
            wr_start = 1'b1;
            wr_addr  = a;
            wr_data  = d;
            @(posedge clk);
            @(negedge clk);
            while (!wr_ack) begin
                @(posedge clk);
                @(negedge clk);
            end
            wr_start = 1'b0;
            // wr_ack only means the request was latched: wait until the word
            // has really reached the memory, otherwise a following read may
            // be served before the queued write and return stale data
            while (!wr_done) begin
                @(posedge clk);
                @(negedge clk);
            end
        end
    endtask

    // burst read with expected data check
    task rd_burst;
        input [20:0] a;
        input integer len;
        input integer verbose;
        begin
            @(negedge clk);
            rd_start = 1'b1;
            rd_addr  = a;
            rd_len   = len;
            @(posedge clk);
            @(negedge clk);
            while (!rd_ack) begin
                @(posedge clk);
                @(negedge clk);
            end
            rd_start = 1'b0;

            n = 0;
            while (n < len) begin
                @(negedge clk);
                if (rd_valid) begin
                    got      = rd_data;
                    offs     = n;
                    exp_word = patt(a + offs);
                    checks   = checks + 1;
                    if (got !== exp_word) begin
                        errors = errors + 1;
                        if (errors <= 10)
                            $display("[%0t] DATA MISMATCH addr=%06x got=%08x exp=%08x",
                                     $time, a + n, got, exp_word);
                    end
                    n = n + 1;
                end
            end
            if (verbose)
                $display("  burst addr=%06x len=%0d done at %0t", a, len, $time);
        end
    endtask

    //------------------------------------------------------------------
    // waveform dump is off by default: dumping the whole hierarchy would
    // write out the 2M x 32 bit SDRAM model (several GB). Use
    //   vvp tb_sdram_ctrl.vvp +vcd
    // to get a shallow dump of this test bench only.
    //------------------------------------------------------------------
    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_sdram_ctrl.vcd");
            $dumpvars(1, tb_sdram_ctrl);
        end
    end

    // global guard: a hung handshake must not spin forever
    initial begin
        #5000000;                       // 5 ms of simulated time
        $display("");
        $display("GLOBAL TIMEOUT: simulation did not finish in 5 ms");
        $display("  RESULT: FAIL");
        $finish;
    end

    //------------------------------------------------------------------
    initial begin
        rd_start = 0; rd_addr = 0; rd_len = 1;
        wr_start = 0; wr_addr = 0; wr_data = 0;

        rst_n = 1'b0;
        repeat (20) @(negedge clk);
        rst_n = 1'b1;

        //--------------------------------------------------------------
        $display("");
        $display("=== test 1: power up sequence and read latency calibration ===");
        t0 = $time;
        while (!cal_done) begin
            @(negedge clk);
            if ($time - t0 > 600000) begin
                $display("TIMEOUT waiting for cal_done");
                errors = errors + 1;
                $finish;
            end
        end
        $display("  init_done=%0b cal_done=%0b cal_fail=%0b cal_latency=%0d (time %0t)",
                 init_done, cal_done, cal_fail, cal_latency, $time);
        if (!init_done) begin errors = errors + 1; $display("  FAIL: init_done not set"); end
        if (cal_fail)   begin errors = errors + 1; $display("  FAIL: calibration failed"); end
        if (cal_latency != 4'd3) begin
            $display("  WARN: calibrated latency %0d, model expects 3", cal_latency);
        end
        $display("  power up wait   : %0t ns", $time);

        //--------------------------------------------------------------
        $display("");
        $display("=== test 2: single word accesses over banks and rows ===");
        // write a scattered set of words, one per bank/row combination
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 6; j = j + 1) begin
                case (j)
                    0: k = 0;
                    1: k = 1;
                    2: k = 2;
                    3: k = 511;
                    4: k = 512;
                    default: k = 1023;
                endcase
                wr_word(mkaddr(i, k, 0),          patt(mkaddr(i, k, 0)));
                wr_word(mkaddr(i, k, 255),        patt(mkaddr(i, k, 255)));
                wr_word(mkaddr(i, k, 511),        patt(mkaddr(i, k, 511)));
            end
        end
        $display("  wrote 72 scattered words");

        // read them back, bank/row order reversed to force row misses
        for (i = 3; i >= 0; i = i - 1) begin
            for (j = 5; j >= 0; j = j - 1) begin
                case (j)
                    0: k = 0;
                    1: k = 1;
                    2: k = 2;
                    3: k = 511;
                    4: k = 512;
                    default: k = 1023;
                endcase
                rd_burst(mkaddr(i, k, 0),   1, 0);
                rd_burst(mkaddr(i, k, 255), 1, 0);
                rd_burst(mkaddr(i, k, 511), 1, 0);
            end
        end
        $display("  read back 72 scattered words, checks=%0d errors=%0d", checks, errors);

        //--------------------------------------------------------------
        $display("");
        $display("=== test 3: full row burst read (512 words) ===");
        for (i = 0; i < 512; i = i + 1)
            wr_word(mkaddr(1, 10, i), patt(mkaddr(1, 10, i)));
        $display("  row filled at %0t ns", $time);
        rd_burst(mkaddr(1, 10, 0), 512, 1);      // row hit after the writes
        rd_burst(mkaddr(1, 10, 0), 512, 1);      // second hit, page stays open
        $display("  checks=%0d errors=%0d", checks, errors);

        // a burst that starts in the middle of a row
        rd_burst(mkaddr(1, 10, 100), 200, 1);

        //--------------------------------------------------------------
        $display("");
        $display("=== test 4: video style line reads with a concurrent loader ===");
        // a display line is 640 words: 512 in one row plus 128 in the next row
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 512; j = j + 1)
                wr_word(mkaddr(2, 20 + 2*i, j), patt(mkaddr(2, 20 + 2*i, j)));
            for (j = 0; j < 128; j = j + 1)
                wr_word(mkaddr(2, 21 + 2*i, j), patt(mkaddr(2, 21 + 2*i, j)));
        end
        $display("  4 display lines preloaded at %0t ns", $time);

        lw = 0;
        loader_run = 1'b1;
        for (i = 0; i < 4; i = i + 1) begin
            $display("  pass 1, line %0d at %0t ns", i, $time);
            rd_burst(mkaddr(2, 20 + 2*i, 0), 512, 0);
            rd_burst(mkaddr(2, 21 + 2*i, 0), 128, 0);
        end
        // read the lines a second time while the loader keeps hammering bank 2
        for (i = 0; i < 4; i = i + 1) begin
            $display("  pass 2, line %0d at %0t ns", i, $time);
            rd_burst(mkaddr(2, 20 + 2*i, 0), 512, 0);
            rd_burst(mkaddr(2, 21 + 2*i, 0), 128, 0);
        end
        loader_run = 1'b0;
        k = 0;
        while (loader_active === 1'b1 && k < 10000) begin
            @(negedge clk);
            k = k + 1;
        end
        if (loader_active === 1'b1) begin
            errors = errors + 1;
            $display("  FAIL: loader did not stop, write port is starved");
        end
        // both ports have to keep moving: every read burst must have been
        // served and the loader must have been granted bus slots as well
        $display("  display stream done at %0t ns, loader wrote %0d words",
                 $time, lw);
        if (lw < 8) begin
            errors = errors + 1;
            $display("  FAIL: write port starved while the display was reading");
        end else begin
            $display("  PASS: write port kept making progress under read load");
        end

        // the words the loader managed to write must be intact
        n = (lw / 16) * 16;
        for (i = 0; i < n; i = i + 16) begin
            if (i < 512)
                rd_burst(mkaddr(2, 300, i), 16, 0);
            else
                rd_burst(mkaddr(2, 301, i - 512), 16, 0);
        end
        $display("  verified first %0d loader words, checks=%0d errors=%0d",
                 n, checks, errors);

        //--------------------------------------------------------------
        // the loader now gets the bus to itself and finishes the block
        //--------------------------------------------------------------
        loader_run = 1'b1;
        t0 = $time;
        while (lw < LW_MAX && ($time - t0) < 200000)
            @(negedge clk);
        loader_run = 1'b0;
        k = 0;
        while (loader_active === 1'b1 && k < 10000) begin
            @(negedge clk);
            k = k + 1;
        end
        $display("  loader finished %0d/%0d words at %0t ns", lw, LW_MAX, $time);
        if (lw != LW_MAX) begin
            errors = errors + 1;
            $display("  FAIL: loader could not finish the block");
        end

        // whole 1024 word loader block must read back correctly
        for (i = 0; i < 512; i = i + 64)
            rd_burst(mkaddr(2, 300, i), 64, 0);
        for (i = 0; i < 512; i = i + 64)
            rd_burst(mkaddr(2, 301, i), 64, 0);
        $display("  loader block verified, checks=%0d errors=%0d", checks, errors);

        //--------------------------------------------------------------
        $display("");
        $display("=== test 5: auto refresh ===");
        i = refresh_cnt;
        $display("  refresh_cnt before wait = %0d at %0t", i, $time);
        repeat (20000) @(negedge clk);          // 160 us
        $display("  refresh_cnt after  wait = %0d at %0t", refresh_cnt, $time);
        if (refresh_cnt - i < 15) begin
            errors = errors + 1;
            $display("  FAIL: too few refreshes (%0d), expected about 20",
                     refresh_cnt - i);
        end else begin
            $display("  PASS: %0d auto refreshes issued", refresh_cnt - i);
        end

        // data must still be intact after all of the refreshing
        rd_burst(mkaddr(1, 10, 0), 512, 0);
        rd_burst(mkaddr(0, 1023, 511), 1, 0);
        rd_burst(mkaddr(3, 511, 255), 1, 0);

        //--------------------------------------------------------------
        $display("");
        $display("=== summary ===");
        u_sdram.show_stats;
        $display("  data checks=%0d data errors=%0d protocol errors=%0d",
                 checks, errors, u_sdram.n_err);
        if (errors == 0 && u_sdram.n_err == 0)
            $display("  RESULT: PASS");
        else
            $display("  RESULT: FAIL");
        $display("");
        $finish;
    end

endmodule
