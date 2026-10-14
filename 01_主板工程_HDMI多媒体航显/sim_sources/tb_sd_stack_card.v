`timescale 1ns / 1ps
//=============================================================================
// tb_sd_stack_card.v - full SPI command/read stack against a small SD card
// model.  This catches command framing, CPOL/CPHA, R1 timing and the
// CMD17 data-token handoff that a mocked block interface cannot see.
//=============================================================================
module tb_sd_stack_card;

    reg         clk;
    reg         rst;
    reg         sd_sec_read;
    reg  [31:0] sd_sec_read_addr;

    wire        SD_nCS;
    wire        SD_DCLK;
    wire        SD_MOSI;
    wire        SD_MISO;

    wire        sd_init_done;
    wire        sd_init_error;
    wire [2:0]  init_fail_code;
    wire [7:0]  sd_sec_read_data;
    wire        sd_sec_read_data_valid;
    wire        sd_sec_read_end;
    wire        sd_sec_read_error;
    wire [3:0]  sd_sec_read_error_code;

    integer     data_count;
    integer     data_errors;
    integer     wait_count;
    reg  [31:0] expected_lba;

    always #10 clk = ~clk;

    sd_card_top #(
        .SPI_LOW_SPEED_DIV  (16'd8),
        .SPI_HIGH_SPEED_DIV (16'd1),
        .POWER_WAIT_CYCLES  (24'd16)
    ) dut (
        .clk                    (clk),
        .rst                    (rst),
        .SD_nCS                 (SD_nCS),
        .SD_DCLK                (SD_DCLK),
        .SD_MOSI                (SD_MOSI),
        .SD_MISO                (SD_MISO),
        .sd_init_done           (sd_init_done),
        .sd_init_error          (sd_init_error),
        .init_fail_code         (init_fail_code),
        .sd_sec_read            (sd_sec_read),
        .sd_sec_read_addr       (sd_sec_read_addr),
        .sd_sec_read_data       (sd_sec_read_data),
        .sd_sec_read_data_valid (sd_sec_read_data_valid),
        .sd_sec_read_end        (sd_sec_read_end),
        .sd_sec_read_error      (sd_sec_read_error),
        .sd_sec_read_error_code (sd_sec_read_error_code)
    );

    //------------------------------------------------------------------
    // Minimal card model.  MOSI is sampled on the rising edge and MISO is
    // changed on the falling edge, matching CPOL=1/CPHA=1 SPI mode.
    //------------------------------------------------------------------
    reg [7:0] rx_shift;
    reg [2:0] rx_bit;
    reg [2:0] cmd_idx;
    reg [7:0] cmd_byte [0:5];
    reg       acmd_pending;

    reg [7:0] resp_mem [0:520];
    reg [15:0] resp_len;
    reg [15:0] resp_idx;
    reg [3:0]  tx_bit;
    reg        tx_active;
    reg [7:0]  tx_shift;
    reg        miso_reg;
    integer    fill_i;

    assign SD_MISO = miso_reg;

    task load_simple_response;
        input [7:0] r1;
        begin
            resp_mem[0] = r1;
            resp_len    = 16'd1;
            resp_idx    = 16'd0;
            tx_bit      = 4'd0;
            tx_shift    = r1;
            tx_active   = 1'b1;
        end
    endtask

    task load_r7_response;
        input [7:0] r1;
        input [7:0] b0;
        input [7:0] b1;
        input [7:0] b2;
        input [7:0] b3;
        begin
            resp_mem[0] = r1;
            resp_mem[1] = b0;
            resp_mem[2] = b1;
            resp_mem[3] = b2;
            resp_mem[4] = b3;
            resp_len    = 16'd5;
            resp_idx    = 16'd0;
            tx_bit      = 4'd0;
            tx_shift    = r1;
            tx_active   = 1'b1;
        end
    endtask

    task load_read_response;
        input [31:0] lba;
        begin
            resp_mem[0] = 8'h00;
            resp_mem[1] = 8'hFF;
            resp_mem[2] = 8'hFE;
            for (fill_i = 0; fill_i < 512; fill_i = fill_i + 1)
                resp_mem[3 + fill_i] = (lba[7:0] ^ fill_i[7:0]);
            resp_mem[515] = 8'h00;
            resp_mem[516] = 8'hFF;
            resp_len      = 16'd517;
            resp_idx      = 16'd0;
            tx_bit        = 4'd0;
            tx_shift      = 8'h00;
            tx_active     = 1'b1;
        end
    endtask

    always @(posedge SD_DCLK or posedge rst) begin
        if (rst) begin
            rx_shift     <= 8'h00;
            rx_bit       <= 3'd0;
            cmd_idx      <= 3'd0;
            acmd_pending <= 1'b0;
            tx_active    <= 1'b0;
            tx_bit       <= 4'd0;
            tx_shift     <= 8'hFF;
            miso_reg     <= 1'b1;
            resp_len     <= 16'd0;
            resp_idx     <= 16'd0;
            for (fill_i = 0; fill_i < 6; fill_i = fill_i + 1)
                cmd_byte[fill_i] <= 8'h00;
        end else if (!SD_nCS) begin
            rx_shift <= {rx_shift[6:0], SD_MOSI};
            if (rx_bit == 3'd7) begin
                rx_bit <= 3'd0;
                if (cmd_idx < 3'd6)
                    cmd_byte[cmd_idx] <= {rx_shift[6:0], SD_MOSI};

                if (cmd_idx == 3'd5) begin
                    case (cmd_byte[0])
                        8'h40: load_simple_response(8'h01); // CMD0
                        8'h48: load_r7_response(8'h01, 8'h00, 8'h00, 8'h01, 8'hAA);
                        8'h77: begin
                            load_simple_response(8'h01); // CMD55
                            acmd_pending <= 1'b1;
                        end
                        8'h69: begin
                            load_simple_response(8'h00); // ACMD41
                            acmd_pending <= 1'b0;
                        end
                        8'h7A: load_r7_response(8'h00, 8'h40, 8'h00, 8'h00, 8'h00); // CMD58 OCR CCS=1
                        8'h50: load_simple_response(8'h00); // CMD16
                        8'h51: load_read_response({cmd_byte[1], cmd_byte[2],
                                                   cmd_byte[3], cmd_byte[4]});
                        default: load_simple_response(8'h04); // illegal command
                    endcase
                    cmd_idx <= 3'd7;        // wait for CS to rise
                end else if (cmd_idx < 3'd6) begin
                    cmd_idx <= cmd_idx + 3'd1;
                end
            end else begin
                rx_bit <= rx_bit + 3'd1;
            end
        end else begin
            cmd_idx <= 3'd0;
            rx_bit  <= 3'd0;
        end
    end

    always @(negedge SD_DCLK or posedge rst) begin
        if (rst) begin
            tx_bit    <= 4'd0;
            tx_shift  <= 8'hFF;
            miso_reg  <= 1'b1;
            tx_active <= 1'b0;
            resp_idx  <= 16'd0;
        end else if (!SD_nCS) begin
            if (tx_active) begin
                miso_reg <= tx_shift[7];
                if (tx_bit == 4'd7) begin
                    tx_bit <= 4'd0;
                    if ((resp_idx + 16'd1) < resp_len) begin
                        resp_idx <= resp_idx + 16'd1;
                        tx_shift <= resp_mem[resp_idx + 16'd1];
                    end else begin
                        tx_active <= 1'b0;
                        tx_shift  <= 8'hFF;
                    end
                end else begin
                    tx_bit   <= tx_bit + 4'd1;
                    tx_shift <= {tx_shift[6:0], 1'b1};
                end
            end
            else begin
                miso_reg <= 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // Data collection and result
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (sd_sec_read_data_valid) begin
            if (sd_sec_read_data !== (expected_lba[7:0] ^ data_count[7:0])) begin
                $display("FAIL: data[%0d]=%02x expected %02x",
                         data_count, sd_sec_read_data,
                         expected_lba[7:0] ^ data_count[7:0]);
                data_errors = data_errors + 1;
            end
            data_count = data_count + 1;
        end
    end

    initial begin
        clk              = 1'b0;
        rst              = 1'b1;
        sd_sec_read      = 1'b0;
        sd_sec_read_addr = 32'h0000_0021;
        expected_lba     = 32'h0000_0021;
        data_count       = 0;
        data_errors      = 0;

        repeat (20) @(posedge clk);
        rst = 1'b0;

        wait_count = 0;
        while (!sd_init_done && !sd_init_error && wait_count < 2_000_000) begin
            @(posedge clk);
            wait_count = wait_count + 1;
        end

        if (sd_init_error) begin
            $display("FAIL: SD init error code=%0d", init_fail_code);
            $finish;
        end
        if (!sd_init_done) begin
            $display("FAIL: SD init timeout");
            $finish;
        end

        @(negedge clk);
        sd_sec_read = 1'b1;

        wait_count = 0;
        while (!sd_sec_read_end && !sd_sec_read_error &&
               wait_count < 2_000_000) begin
            @(posedge clk);
            wait_count = wait_count + 1;
        end

        if (sd_sec_read_error) begin
            $display("FAIL: SD read error code=%0d", sd_sec_read_error_code);
            $finish;
        end
        if (!sd_sec_read_end) begin
            $display("FAIL: SD read timeout");
            $finish;
        end

        @(posedge clk);
        sd_sec_read = 1'b0;
        repeat (8) @(posedge clk);

        if ((data_count == 512) && (data_errors == 0))
            $display("RESULT: PASS tb_sd_stack_card bytes=%0d", data_count);
        else
            $display("RESULT: FAIL tb_sd_stack_card bytes=%0d errors=%0d",
                     data_count, data_errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("RESULT: FAIL tb_sd_stack_card global timeout");
        $finish;
    end

endmodule
