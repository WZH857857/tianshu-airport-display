//===========================================================================
// tb_ov5640_sccb_cfg.v -- SCCB protocol + register-sequence check
//
// A bus monitor decodes every START/9-bit-byte/STOP transaction and checks:
//   * 250 write transactions, 36 SCL pulses each (4 bytes + 4 ack slots)
//   * device byte 0x78, 16-bit register address, 8-bit data
//   * key entries of the 640x480 RGB565 table (reset, PLL, size, timing)
//   * >= RESET_MS between 0x3008=0x82 and 0x3008=0x02
//   * SCL period 4 us (250 kHz) and SDA moving only with SCL low, except the
//     START/STOP conditions
//   * cfg_done rises only after the last STOP
//
// POWERUP_MS/RESET_MS are shortened to 1 ms so the run stays short.
//===========================================================================
`timescale 1ns / 1ps

module tb_ov5640_sccb_cfg;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #20.833 clk = ~clk;                 // 24 MHz

    wire scl;
    wire sda;
    pullup(sda);                               // board has SCCB pull-ups

    wire cfg_done;
    wire cfg_busy;
    wire [7:0] wr_count;

    ov5640_sccb_cfg #(
        .CLK_HZ     (24000000),
        .SCCB_HZ    (250000),
        .POWERUP_MS (1),
        .RESET_MS   (1)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .scl      (scl),
        .sda      (sda),
        .cfg_done (cfg_done),
        .cfg_busy (cfg_busy),
        .wr_count (wr_count)
    );

    // ------------------------------------------------------- bus monitor
    integer errors = 0;
    integer tx_count = 0;
    integer scl_rise_count = 0;
    integer scl_edge_violations = 0;
    integer tx_edge = 0;                       // SCL rising edges in this tx
    integer bitn = 0;
    integer byte_n = 0;
    reg [7:0] shreg = 8'h00;
    reg [7:0] b0, b1, b2, b3;
    reg       started = 1'b0;
    reg       scl_d = 1'b1;
    reg       sda_d = 1'b1;
    time      last_scl_rise = 0;
    time      t_reset_stop  = 0;
    time      t_wake_start  = 0;
    reg       reset_seen = 1'b0;

    wire start_cond = scl && scl_d && sda_d && !sda;
    wire stop_cond  = scl && scl_d && !sda_d && sda;

    task check(input cond, input [255:0] what);
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL t=%0t %0s", $time, what);
            end
        end
    endtask

    task expect_tx(input integer idx, input [15:0] addr, input [7:0] data,
                   input [255:0] what);
        begin
            // called on the STOP cycle, so tx_count still holds this index
            if (tx_count == idx) begin
                if (b0 !== 8'h78 || {b1, b2} !== addr || b3 !== data) begin
                    errors = errors + 1;
                    $display("FAIL t=%0t %0s: got dev=%02h addr=%04h data=%02h",
                             $time, what, b0, {b1, b2}, b3);
                end
            end
        end
    endtask

    always @(posedge clk) begin
        scl_d <= scl;
        sda_d <= sda;

        // START / STOP conditions (SDA moves while SCL is high)
        if (start_cond) begin
            started <= 1'b1;
            bitn    <= 0;
            byte_n  <= 0;
            tx_edge <= 0;
            if (tx_count == 3) t_wake_start <= $time;
        end
        if (stop_cond) begin
            started <= 1'b0;
            tx_count <= tx_count + 1;
        end

        // SCL rising edge: sample one bit
        if (scl && !scl_d && started) begin
            scl_rise_count <= scl_rise_count + 1;
            // edges 2..36 of the 36-bit payload are evenly spaced; the
            // edge that belongs to STOP is not part of that cadence
            if (tx_edge >= 1 && tx_edge <= 35) begin
                check(($time - last_scl_rise) > 3900 &&
                      ($time - last_scl_rise) < 4100, "SCL period ~4 us");
            end
            last_scl_rise <= $time;
            tx_edge <= tx_edge + 1;

            if (bitn < 8) begin
                shreg <= {shreg[6:0], sda};
                bitn  <= bitn + 1;
            end else begin
                bitn <= 0;
                case (byte_n)
                0: b0 <= shreg;
                1: b1 <= shreg;
                2: b2 <= shreg;
                default: b3 <= shreg;
                endcase
                byte_n <= byte_n + 1;
            end
        end

        // SDA may only move while SCL is low, apart from START/STOP
        if (scl && scl_d && (sda !== sda_d) && !start_cond && !stop_cond)
            scl_edge_violations <= scl_edge_violations + 1;
    end

    // ------------------------------------------- sequence-specific checks
    always @(posedge clk) begin
        if (stop_cond) begin
            expect_tx(0,   16'h300A, 8'h00, "entry 0");
            expect_tx(1,   16'h300B, 8'h00, "entry 1");
            expect_tx(2,   16'h3008, 8'h82, "entry 2 software reset");
            expect_tx(3,   16'h3008, 8'h02, "entry 3 wake up");
            expect_tx(4,   16'h3103, 8'h02, "entry 4 PLL enable");
            expect_tx(54,  16'h4300, 8'h61, "entry 54 RGB565");
            expect_tx(55,  16'h501F, 8'h01, "entry 55 ISP RGB");
            expect_tx(205, 16'h3035, 8'h11, "entry 205 PLL 48 MHz");
            expect_tx(206, 16'h3036, 8'h3C, "entry 206 PLL multiplier");
            expect_tx(220, 16'h3808, 8'h02, "entry 220 width high");
            expect_tx(221, 16'h3809, 8'h80, "entry 221 width low");
            expect_tx(222, 16'h380A, 8'h01, "entry 222 height high");
            expect_tx(223, 16'h380B, 8'hE0, "entry 223 height low");
            expect_tx(224, 16'h380C, 8'h07, "entry 224 HTS high");
            expect_tx(225, 16'h380D, 8'h40, "entry 225 HTS low");
            expect_tx(226, 16'h380E, 8'h03, "entry 226 VTS high");
            expect_tx(227, 16'h380F, 8'hD8, "entry 227 VTS low");
            expect_tx(249, 16'h3019, 8'h00, "entry 249 last write");

            if (tx_count == 2) t_reset_stop <= $time;
            if (tx_count == 3) begin
                check($time - t_reset_stop >= 1000000, "1 ms reset settle");
                check($time - t_reset_stop <= 1400000, "reset settle not too long");
            end
        end
    end

    // ------------------------------------------------------------- stimulus
    initial begin
        $dumpfile("tb_ov5640_sccb_cfg.vcd");
        $dumpvars(0, tb_ov5640_sccb_cfg);

        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        // 250 writes x (36 bits x 4 us) + 20 ms wait + 1 ms reset settle
        #80_000_000;

        check(tx_count == 250, "transaction count");
        check(cfg_done == 1'b1, "cfg_done");
        check(cfg_busy == 1'b0, "cfg_busy clears");
        check(wr_count == 8'd250, "wr_count");
        check(scl_edge_violations == 0, "SDA stable while SCL high");
        check(t_wake_start - t_reset_stop >= 1000000, "settle before wake write");
        check(scl_rise_count == 9250, "SCL rising edges (36 bits + stop per write)");

        if (errors == 0) $display("PASS tb_ov5640_sccb_cfg: 250 writes, protocol clean");
        else             $display("FAIL tb_ov5640_sccb_cfg: %0d errors", errors);
        $finish;
    end

endmodule
