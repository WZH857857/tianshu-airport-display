`timescale 1ns / 1ps
//=============================================================================
// tb_sd_sec_rw_handshake.v - focused SD CMD17 handshake/OCR regression
//
// This unit test bypasses the long card initialization sequence.  It first
// injects a CMD58 OCR response with CCS=1, then restarts the DUT in
// S_WAIT_READ and checks that one CMD17:
//   * uses block addressing (argument is not shifted left by 9),
//   * is issued only once, and
//   * receives exactly one 512-byte block.
//=============================================================================
module tb_sd_sec_rw_handshake;

    reg         clk;
    reg         rst;
    reg         sd_sec_read;
    reg  [31:0] sd_sec_read_addr;

    wire        sd_init_done;
    wire        sd_init_error;
    wire [2:0]  init_fail_code;
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

    localparam [4:0] DUT_S_CMD58     = 5'd5,
                     DUT_S_WAIT_READ = 5'd7;

    localparam [2:0] M_IDLE       = 3'd0,
                     M_WAIT       = 3'd1,
                     M_ACK        = 3'd2,
                     M_RETURN     = 3'd3,
                     M_CHECK      = 3'd4,
                     M_BLOCK_WAIT = 3'd5,
                     M_BLOCK_DATA = 3'd6,
                     M_BLOCK_ACK  = 3'd7;

    reg [2:0]  mock_state;
    reg [1:0]  phase;              // 0 = OCR injection, 1 = CMD17 test
    reg [7:0]  ocr_bytes [0:3];
    integer    ocr_idx;
    integer    mock_wait;
    integer    block_idx;
    integer    cmd17_count;
    integer    data_count;
    integer    test_errors;
    reg [31:0] cmd17_arg;
    reg        block_read_ack_reg;

    sd_card_sec_read_write #(
        .SPI_LOW_SPEED_DIV  (16'd8),
        .SPI_HIGH_SPEED_DIV (16'd1),
        .POWER_WAIT_CYCLES  (24'd16)
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

    always #10 clk = ~clk;

    initial begin
        clk              = 1'b0;
        rst              = 1'b1;
        sd_sec_read      = 1'b0;
        sd_sec_read_addr = 32'h0000_1234;
        cmd_req_ack      = 1'b0;
        cmd_req_error    = 1'b0;
        mock_state       = M_IDLE;
        phase            = 2'd0;
        ocr_bytes[0]     = 8'h40;       // OCR[31:24], CCS = 1
        ocr_bytes[1]     = 8'h00;
        ocr_bytes[2]     = 8'h00;
        ocr_bytes[3]     = 8'h00;
        ocr_idx          = 0;
        mock_wait        = 0;
        block_idx        = 0;
        cmd17_count      = 0;
        data_count       = 0;
        test_errors      = 0;
        cmd17_arg        = 32'd0;

        repeat (8) @(posedge clk);
        rst = 1'b0;

        // Phase 0: put the command FSM in CMD58 and inject four OCR bytes.
        @(negedge clk);
        dut.state   = DUT_S_CMD58;
        dut.ocr_cnt = 3'd0;
        dut.ocr     = 32'd0;
        ocr_idx     = 0;
        repeat (6) @(posedge clk);

        if ((dut.ocr !== 32'h4000_0000) || (dut.ocr_cnt !== 3'd4)) begin
            $display("FAIL: OCR=%08x cnt=%0d expected 40000000/4",
                     dut.ocr, dut.ocr_cnt);
            test_errors = test_errors + 1;
        end

        // Reset again, then jump directly to the read-wait state.
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(negedge clk);
        dut.state        = DUT_S_WAIT_READ;
        dut.sd_init_done = 1'b1;
        dut.card_is_sdhc = 1'b1;
        dut.ocr          = 32'h4000_0000;
        dut.cmd_req      = 1'b0;
        sd_sec_read      = 1'b1;
        phase            = 2'd1;

        wait (sd_sec_read_end);
        @(posedge clk);
        sd_sec_read = 1'b0;
        repeat (8) @(posedge clk);

        if (cmd17_count != 1) begin
            $display("FAIL: CMD17 issued %0d times", cmd17_count);
            test_errors = test_errors + 1;
        end else if (cmd17_arg != 32'h0000_1234) begin
            $display("FAIL: CMD17 arg=%08x expected=00001234", cmd17_arg);
            test_errors = test_errors + 1;
        end else if (data_count != 512) begin
            $display("FAIL: data bytes=%0d expected=512", data_count);
            test_errors = test_errors + 1;
        end else if (sd_sec_read_error) begin
            $display("FAIL: sd_sec_read_error asserted");
            test_errors = test_errors + 1;
        end

        if (test_errors == 0)
            $display("PASS: OCR CCS=1, CMD17 once at 00001234, 512 data bytes");
        else
            $display("FAIL: %0d test error(s)", test_errors);

        $finish;
    end

    // OCR byte injection.
    always @(posedge clk) begin
        if ((phase == 2'd0) && (ocr_idx < 4))
            ocr_idx <= ocr_idx + 1;
    end

    // Command/block mock.  It returns to an idle-like check state after each
    // response, so a still-high cmd_req is counted as a duplicate request.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mock_state    <= M_IDLE;
            cmd_req_ack   <= 1'b0;
            cmd_req_error <= 1'b0;
            mock_wait     <= 0;
            block_idx     <= 0;
            cmd17_count   <= 0;
            cmd17_arg     <= 32'd0;
        end else begin
            cmd_req_ack   <= 1'b0;
            cmd_req_error <= 1'b0;

            if (phase == 2'd1) begin
                case (mock_state)
                    M_IDLE: begin
                        if (cmd_req) begin
                            cmd17_arg  <= cmd[39:8];
                            mock_wait  <= 0;
                            mock_state <= M_WAIT;
                        end
                    end

                    M_WAIT: begin
                        if (mock_wait == 2)
                            mock_state <= M_ACK;
                        else
                            mock_wait <= mock_wait + 1;
                    end

                    M_ACK: begin
                        cmd_req_ack <= 1'b1;
                        cmd17_count <= cmd17_count + 1;
                        mock_state  <= M_RETURN;
                    end

                    M_RETURN: begin
                        mock_state <= M_CHECK;
                    end

                    M_CHECK: begin
                        if (cmd_req)
                            cmd17_count <= cmd17_count + 1;
                        mock_state <= M_BLOCK_WAIT;
                    end

                    M_BLOCK_WAIT: begin
                        if (block_read_req) begin
                            block_idx  <= 0;
                            mock_state <= M_BLOCK_DATA;
                        end
                    end

                    M_BLOCK_DATA: begin
                        if (block_idx < 512)
                            block_idx <= block_idx + 1;
                        else
                            mock_state <= M_BLOCK_ACK;
                    end

                    M_BLOCK_ACK: begin
                        mock_state <= M_IDLE;
                    end

                    default: mock_state <= M_IDLE;
                endcase
            end
        end
    end

    assign block_read_req_ack = block_read_ack_reg;

    always @(posedge clk or posedge rst) begin
        if (rst)
            block_read_ack_reg <= 1'b0;
        else
            block_read_ack_reg <= (mock_state == M_BLOCK_ACK);
    end

    // Data is combinational so the DUT samples the intended byte on the next
    // clock edge, matching the real registered byte engine timing.
    always @* begin
        block_read_valid = 1'b0;
        block_read_data  = 8'h00;

        if ((phase == 2'd0) && (ocr_idx < 4)) begin
            block_read_valid = 1'b1;
            block_read_data  = ocr_bytes[ocr_idx];
        end else if ((phase == 2'd1) && (mock_state == M_BLOCK_DATA) &&
                     (block_idx < 512)) begin
            block_read_valid = 1'b1;
            block_read_data  = block_idx[7:0];
        end
    end

    always @(posedge clk) begin
        if (sd_sec_read_data_valid)
            data_count <= data_count + 1;
    end

endmodule
