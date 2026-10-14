//===========================================================================
// tb_mt9m001_cfg.v -- MT9M001 two-wire protocol + boot table check
//
// A bus monitor decodes every START/four-byte/STOP transaction and checks:
//   * 13 boot writes: device byte 0xBA, an 8-bit register address (one byte,
//     not the OV5640's 16-bit address) and a 16-bit big-endian value
//   * the entries that matter: soft reset, stop, window, blanking, skip,
//     gain, shutter, start
//   * >= RESET_MS between 0x0D=0x0001 and 0x0D=0x0000
//   * SCL period 10 us (100 kHz) and SDA stable while SCL is high
//   * the master releases SDA for the ninth slot of every byte, so the slave
//     model can acknowledge: the model pulls SDA low in each ACK slot and the
//     monitor fails the run if the bus reads anything but a clean 0 - that is
//     the contention an off-by-one release would produce
//   * the dynamic port: one toggle in, one bus write plus one ack toggle out,
//     for both the shutter (0x09) and the gain (0x35) register
//
// POWERUP_MS/RESET_MS are shortened to 1 ms so the run stays short.
//===========================================================================
`timescale 1ns / 1ps

module tb_mt9m001_cfg;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #20.833 clk = ~clk;                 // 24 MHz

    wire scl;
    wire sda;
    pullup(sda);                               // module has two-wire pull-ups

    wire cfg_done;
    wire cfg_busy;
    wire [7:0] wr_count;

    reg        dyn_tog  = 1'b0;
    reg [7:0]  dyn_addr = 8'h00;
    reg [15:0] dyn_data = 16'h0000;
    wire       ack_tog;

    mt9m001_cfg #(
        .CLK_HZ     (24000000),
        .SCL_HZ     (100000),
        .POWERUP_MS (1),
        .RESET_MS   (1)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .scl      (scl),
        .sda      (sda),
        .cfg_done (cfg_done),
        .cfg_busy (cfg_busy),
        .wr_count (wr_count),
        .dyn_tog  (dyn_tog),
        .dyn_addr (dyn_addr),
        .dyn_data (dyn_data),
        .ack_tog  (ack_tog)
    );

    // ------------------------------------------------------ slave model
    // Pulls SDA low in the 9th, 18th, 27th and 36th slot of a transaction,
    // i.e. exactly where the part acknowledges.  Because the model drives a
    // hard 0, a master that still drove a data bit in that slot would turn the
    // bus into X and the check below catches it.
    reg ack_drive = 1'b0;
    assign sda = ack_drive ? 1'b0 : 1'bz;

    // ------------------------------------------------------- bus monitor
    // Everything is sampled in the clk domain off registered copies of
    // scl/sda, so no block races with the DUT's own clock edges.
    integer errors = 0;
    integer tx_count = 0;
    integer scl_rise_count = 0;
    integer acks_seen = 0;
    integer bitn = 0;
    integer byte_n = 0;
    integer slot_n = 0;                        // rises seen in this transaction
    reg [7:0] shreg = 8'h00;
    reg [7:0] b0, b1, b2, b3;
    reg       started = 1'b0;
    reg       scl_d = 1'b1;
    reg       sda_d = 1'b1;
    reg       in_ack = 1'b0;
    time      last_scl_rise = 0;
    time      t_reset_stop = 0;
    integer   scl_hi_violations = 0;

    wire start_cond = scl && scl_d && sda_d && !sda;
    wire stop_cond  = scl && scl_d && !sda_d && sda;
    wire scl_rise   = scl && !scl_d;
    wire scl_fall   = !scl && scl_d;
    wire [7:0] shreg_n = {shreg[6:0], sda};

    task check(input cond, input [255:0] what);
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL t=%0t %0s", $time, what);
            end
        end
    endtask

    task expect_tx(input integer idx, input [7:0] addr, input [15:0] data,
                   input [255:0] what);
        begin
            if (tx_count == idx) begin
                if (b0 !== 8'hBA || b1 !== addr || {b2, b3} !== data) begin
                    errors = errors + 1;
                    $display("FAIL t=%0t %0s: got dev=%02h addr=%02h data=%04h",
                             $time, what, b0, b1, {b2, b3});
                end
            end
        end
    endtask

    always @(posedge clk) begin
        scl_d <= scl;
        sda_d <= sda;

        if (start_cond) begin
            started  <= 1'b1;
            bitn     <= 0;
            byte_n   <= 0;
            slot_n   <= 0;
            in_ack   <= 1'b0;
            ack_drive <= 1'b0;
        end

        if (scl_rise && started) begin
            scl_rise_count <= scl_rise_count + 1;

            if (slot_n >= 1 && slot_n <= 35)
                check(($time - last_scl_rise) > 9800 &&
                      ($time - last_scl_rise) < 10200, "SCL period ~10 us");
            last_scl_rise <= $time;

            if ((slot_n % 9) == 8) begin
                // ninth slot of the byte: the slave owns the line
                acks_seen <= acks_seen + 1;
                in_ack    <= 1'b1;
                check(sda === 1'b0, "ACK slot reads a clean 0");
                bitn <= 0;
            end else begin
                in_ack <= 1'b0;
                shreg  <= shreg_n;
                if (bitn == 7) begin
                    bitn <= 0;
                    case (byte_n)
                    0: b0 <= shreg_n;
                    1: b1 <= shreg_n;
                    2: b2 <= shreg_n;
                    default: b3 <= shreg_n;
                    endcase
                    byte_n <= byte_n + 1;
                end else begin
                    bitn <= bitn + 1;
                end
            end

            slot_n <= slot_n + 1;
        end

        // slave drives from the falling edge after the 8th rise until the
        // falling edge after the 9th
        if (scl_fall && started) begin
            if ((slot_n % 9) == 8) ack_drive <= #1 1'b1;
            if (((slot_n % 9) == 0) && (slot_n != 0)) begin
                ack_drive <= #1 1'b0;
                in_ack    <= 1'b0;
            end
        end

        if (stop_cond) begin
            started   <= 1'b0;
            in_ack    <= 1'b0;
            ack_drive <= 1'b0;
            tx_count  <= tx_count + 1;
            if (tx_count == 0) t_reset_stop <= $time;
        end

        // SDA may only move while SCL is low, apart from START/STOP and the
        // acknowledge slots, which the slave model drives.
        if (scl && scl_d && (sda !== sda_d) && !start_cond && !stop_cond &&
            !in_ack && !ack_drive)
            scl_hi_violations <= scl_hi_violations + 1;
    end

    // ------------------------------------------- sequence-specific checks
    always @(posedge clk) begin
        if (stop_cond) begin
            expect_tx(0,  8'h0D, 16'h0001, "entry 0 soft reset");
            expect_tx(1,  8'h0D, 16'h0000, "entry 1 release reset");
            expect_tx(2,  8'h07, 16'h0000, "entry 2 stop readout");
            expect_tx(3,  8'h01, 16'h000C, "entry 3 row start");
            expect_tx(4,  8'h02, 16'h0014, "entry 4 column start");
            expect_tx(5,  8'h03, 16'h03FF, "entry 5 1024 rows");
            expect_tx(6,  8'h04, 16'h04FF, "entry 6 1280 columns");
            expect_tx(7,  8'h05, 16'h01F4, "entry 7 horizontal blanking");
            expect_tx(8,  8'h06, 16'h0019, "entry 8 vertical blanking");
            expect_tx(9,  8'h20, 16'h0018, "entry 9 row+column skip");
            expect_tx(10, 8'h35, 16'h0008, "entry 10 gain 1.0x");
            expect_tx(11, 8'h09, 16'h00C8, "entry 11 shutter 200 rows");
            expect_tx(12, 8'h07, 16'h0002, "entry 12 start streaming");

            if (tx_count == 1) begin
                check($time - t_reset_stop >= 1000000, "1 ms reset settle");
                check($time - t_reset_stop <= 1400000, "reset settle not long");
            end
        end
    end

    // ------------------------------------------- dynamic port handshake
    task dyn_write(input [7:0] addr, input [15:0] data, input [255:0] what);
        reg ack_start;
        begin
            dyn_addr  = addr;
            dyn_data  = data;
            ack_start = ack_tog;
            @(posedge clk);
            dyn_tog = ~dyn_tog;
            fork : wait_ack
                begin
                    wait (ack_tog !== ack_start);
                    disable wait_ack;
                end
                begin
                    #1_000_000;
                    check(1'b0, what);
                    disable wait_ack;
                end
            join
            @(posedge clk);
        end
    endtask

    // ------------------------------------------------------------- stimulus
    initial begin
        $dumpfile("tb_mt9m001_cfg.vcd");
        $dumpvars(0, tb_mt9m001_cfg);

        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        // 13 writes x 36 SCL periods x 10 us + 20 ms wait + reset settle
        #30_000_000;
        check(tx_count == 13, "13 boot transactions");
        check(cfg_done == 1'b1, "cfg_done after the table");
        check(cfg_busy == 1'b0, "cfg_busy clears");
        check(wr_count == 8'd13, "wr_count");
        check(acks_seen == 52, "52 acknowledge slots (13 x 4)");
        check(scl_rise_count == 481, "481 SCL rises (36 bits + stop per write)");
        check(scl_hi_violations == 0, "SDA stable while SCL high");

        // Two dynamic writes: the exposure loop's shutter, then gain.
        dyn_write(8'h09, 16'h00A0, "dynamic shutter ack");
        expect_tx(13, 8'h09, 16'h00A0, "dynamic shutter value");

        dyn_write(8'h35, 16'h0009, "dynamic gain ack");
        expect_tx(14, 8'h35, 16'h0009, "dynamic gain value");

        check(tx_count == 15, "both dynamic writes reached the bus");
        check(cfg_done == 1'b1, "cfg_done stays high");
        check(wr_count == 8'd15, "wr_count counts the dynamic writes");

        if (errors == 0) $display("PASS tb_mt9m001_cfg: 13 boot + 2 dynamic writes, protocol clean");
        else             $display("FAIL tb_mt9m001_cfg: %0d errors", errors);
        $finish;
    end

endmodule
