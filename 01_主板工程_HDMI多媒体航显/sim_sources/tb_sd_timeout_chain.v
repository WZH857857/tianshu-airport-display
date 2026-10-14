`timescale 1ns / 1ps
//=============================================================================
// tb_sd_timeout_chain.v - verify that specific CMD17 failures survive the
// sector-reader wrapper instead of being collapsed into its generic timeout.
//=============================================================================
module tb_sd_timeout_chain;

    reg         clk;
    reg         rst;
    reg         rst_n;

    wire        sd_init_done;
    wire        sd_init_error;
    wire [2:0]  init_fail_code;
    wire        sd_sec_read;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;
    wire        sd_sec_read_data_valid;
    wire        sd_sec_read_end;
    wire        sd_sec_read_error;
    wire [3:0]  sd_sec_read_error_code;

    wire [15:0] spi_clk_div;
    wire        cmd_req;
    reg         cmd_req_ack;
    reg         cmd_req_error;
    wire [47:0] cmd;
    wire [7:0]  cmd_r1;
    wire [15:0] cmd_data_len;

    wire        block_read_req;
    reg         block_read_valid;
    reg  [7:0]  block_read_data;
    wire        block_read_req_ack;

    wire        sec_start;
    wire [31:0] sec_lba;
    wire        sec_busy;
    wire        sec_ready;
    wire        sec_error;
    wire [3:0]  sec_error_code;
    reg         sec_release;

    reg  [31:0] mock_cmd_count;
    reg  [1:0]  mock_delay;
    reg         reject_cmd17;
    reg         hold_cmd17;
    integer     errors;

    localparam [4:0] DUT_S_WAIT_READ = 5'd7;

    always #10 clk = ~clk;

    sd_card_sec_read_write #(
        .SPI_LOW_SPEED_DIV     (16'd8),
        .SPI_HIGH_SPEED_DIV    (16'd1),
        .BLOCK_TIMEOUT_CYCLES  (24'd200),
        .CMD_TIMEOUT_CYCLES    (24'd200),
        .POWER_WAIT_CYCLES     (24'd16)
    ) dut (
        .clk                    (clk),
        .rst                    (rst),
        .sd_init_done           (sd_init_done),
        .sd_init_error          (sd_init_error),
        .init_fail_code         (init_fail_code),
        .sd_sec_read            (sd_sec_read),
        .sd_sec_read_addr       (sd_sec_read_addr),
        .sd_sec_read_data       (sd_sec_read_data),
        .sd_sec_read_data_valid (sd_sec_read_data_valid),
        .sd_sec_read_end        (sd_sec_read_end),
        .sd_sec_read_error      (sd_sec_read_error),
        .sd_sec_read_error_code (sd_sec_read_error_code),
        .spi_clk_div            (spi_clk_div),
        .cmd_req                (cmd_req),
        .cmd_req_ack            (cmd_req_ack),
        .cmd_req_error          (cmd_req_error),
        .cmd                    (cmd),
        .cmd_r1                 (cmd_r1),
        .cmd_data_len           (cmd_data_len),
        .block_read_req         (block_read_req),
        .block_read_valid       (block_read_valid),
        .block_read_data        (block_read_data),
        .block_read_req_ack     (block_read_req_ack)
    );

    sector_reader #(
        .TIMEOUT_CYCLES (24'd100_000)
    ) sec (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .start                   (sec_start),
        .lba                     (sec_lba),
        .busy                    (sec_busy),
        .ready                   (sec_ready),
        .error                   (sec_error),
        .error_code              (sec_error_code),
        .raddr0                  (9'd0),
        .raddr1                  (9'd0),
        .raddr2                  (9'd0),
        .raddr3                  (9'd0),
        .rdata0                  (),
        .rdata1                  (),
        .rdata2                  (),
        .rdata3                  (),
        .release_req             (sec_release),
        .sd_init_done            (sd_init_done),
        .sd_sec_read             (sd_sec_read),
        .sd_sec_read_addr        (sd_sec_read_addr),
        .sd_sec_read_data        (sd_sec_read_data),
        .sd_sec_read_data_valid  (sd_sec_read_data_valid),
        .sd_sec_read_end         (sd_sec_read_end),
        .sd_sec_read_error       (sd_sec_read_error),
        .sd_sec_read_error_code  (sd_sec_read_error_code)
    );

    // The real sector reader owns the start/lba interface.  This test drives
    // it through a small simple source instead of adding a second FSM.
    reg start_toggle;
    assign sec_start = start_toggle;
    assign sec_lba   = 32'h0000_1234;

    // CMD17 mock: accept the command, then never acknowledge the data phase.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cmd_req_ack   <= 1'b0;
            cmd_req_error <= 1'b0;
            mock_delay    <= 2'd0;
            mock_cmd_count<= 32'd0;
        end else begin
            cmd_req_ack   <= 1'b0;
            cmd_req_error <= 1'b0;
            if (cmd_req && !hold_cmd17) begin
                if (mock_delay == 2'd0) begin
                    mock_delay <= 2'd1;
                end else if (mock_delay == 2'd1) begin
                    cmd_req_ack   <= 1'b1;
                    cmd_req_error <= reject_cmd17;
                    mock_delay    <= 2'd2;
                    mock_cmd_count <= mock_cmd_count + 32'd1;
                end
            end else begin
                mock_delay <= 2'd0;
            end
        end
    end

    assign block_read_req_ack = 1'b0;

    initial begin
        clk             = 1'b0;
        rst             = 1'b1;
        rst_n           = 1'b0;
        start_toggle    = 1'b0;
        reject_cmd17    = 1'b0;
        hold_cmd17      = 1'b0;
        block_read_valid= 1'b0;
        block_read_data = 8'h00;
        errors          = 0;

        repeat (8) @(posedge clk);
        rst    = 1'b0;
        rst_n  = 1'b1;
        @(negedge clk);
        dut.state        = DUT_S_WAIT_READ;
        dut.sd_init_done = 1'b1;
        dut.card_is_sdhc = 1'b1;

        // Phase 1: command accepted, data token never arrives -> code 9.
        @(negedge clk);
        start_toggle = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start_toggle = 1'b0;

        wait (sec_error);
        #1;
        if (sec_error_code !== 4'd9) begin
            $display("FAIL: token timeout reported code %0d, expected 9",
                     sec_error_code);
            errors = errors + 1;
        end else begin
            $display("PASS: token timeout propagated as code 9");
        end

        // Release the sticky sector error before the next case.
        @(negedge clk);
        sec_release = 1'b1;
        @(posedge clk);
        @(negedge clk);
        sec_release = 1'b0;
        repeat (4) @(posedge clk);

        // Phase 2: command rejected by the card -> code 8.
        reject_cmd17 = 1'b1;
        @(negedge clk);
        start_toggle = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start_toggle = 1'b0;

        wait (sec_error);
        #1;
        if (sec_error_code !== 4'd8) begin
            $display("FAIL: command rejection reported code %0d, expected 8",
                     sec_error_code);
            errors = errors + 1;
        end else begin
            $display("PASS: CMD17 rejection propagated as code 8");
        end

        // Release before the command-response timeout case.
        @(negedge clk);
        sec_release = 1'b1;
        @(posedge clk);
        @(negedge clk);
        sec_release = 1'b0;
        repeat (4) @(posedge clk);

        // Phase 3: no command acknowledge at all -> code 11.
        reject_cmd17 = 1'b0;
        hold_cmd17   = 1'b1;
        @(negedge clk);
        start_toggle = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start_toggle = 1'b0;

        wait (sec_error);
        #1;
        if (sec_error_code !== 4'd11) begin
            $display("FAIL: command timeout reported code %0d, expected 11",
                     sec_error_code);
            errors = errors + 1;
        end else begin
            $display("PASS: CMD17 response timeout propagated as code 11");
        end

        if (errors == 0)
            $display("RESULT: PASS tb_sd_timeout_chain");
        else
            $display("RESULT: FAIL tb_sd_timeout_chain errors=%0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_sd_timeout_chain timeout");
        $finish;
    end

endmodule
