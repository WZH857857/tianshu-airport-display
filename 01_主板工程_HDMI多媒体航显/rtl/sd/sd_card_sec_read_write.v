`timescale 1ns / 1ps
//=============================================================================
// sd_card_sec_read_write.v - TF card initialization and single block read
//
// The command sequence follows the vendor's SD SPI reference:
//   CMD0 -> CMD8 -> CMD55/ACMD41 -> CMD58 -> [CMD16] -> CMD17
//
// CMD58 is used only to distinguish SDHC (LBA addressing) from SDSC (byte
// addressing). The read interface is deliberately one sector at a time.
//
// CMD16 is only issued to SDSC cards: SDHC/SDXC have a fixed 512 byte block
// length.  A card that answers CMD16 with an error is still usable, because
// 512 bytes is the block length a card uses after reset, so SET_BLOCKLEN
// never fails the initialization any more.
//=============================================================================
module sd_card_sec_read_write #(
    parameter [15:0] SPI_LOW_SPEED_DIV  = 16'd248,
    parameter [15:0] SPI_HIGH_SPEED_DIV = 16'd4,
    parameter [23:0] BLOCK_TIMEOUT_CYCLES = 24'd12_500_000,
    parameter [23:0] CMD_TIMEOUT_CYCLES   = 24'd1_000_000,
    // Cards need their supply to settle and at least 74 clocks with CS high
    // before CMD0.  100 ms at 50 MHz gives a slow card room to come up.
    parameter [23:0] POWER_WAIT_CYCLES    = 24'd5_000_000,
    parameter [3:0]  CMD0_MAX_TRIES       = 4'd4
)(
    input  wire        clk,
    input  wire        rst,                 // active high

    output reg         sd_init_done,
    output reg         sd_init_error,
    output reg  [2:0]  init_fail_code,
    input  wire        sd_sec_read,
    input  wire [31:0] sd_sec_read_addr,
    output reg  [7:0]  sd_sec_read_data,
    output reg         sd_sec_read_data_valid,
    output wire        sd_sec_read_end,
    output reg         sd_sec_read_error,
    output reg  [3:0]  sd_sec_read_error_code,

    output reg  [15:0] spi_clk_div,
    output reg         cmd_req,
    input  wire        cmd_req_ack,
    input  wire        cmd_req_error,
    output reg  [47:0] cmd,
    output reg  [7:0]  cmd_r1,
    output reg  [15:0] cmd_data_len,

    output reg         block_read_req,
    input  wire        block_read_valid,
    input  wire [7:0]  block_read_data,
    input  wire        block_read_req_ack,

    // Bring-up diagnostics: expose the live state and latch the state that
    // generated a specific CMD17/block error.
    output wire [4:0]  dbg_state,
    output reg  [4:0]  dbg_fault_state,
    output reg         dbg_error_seen
);

    localparam [4:0] S_IDLE       = 5'd0,
                     S_CMD0       = 5'd1,
                     S_CMD8       = 5'd2,
                     S_CMD55      = 5'd3,
                     S_CMD41      = 5'd4,
                     S_CMD58      = 5'd5,
                     S_CMD16      = 5'd6,
                     S_WAIT_READ  = 5'd7,
                     S_CMD17      = 5'd8,
                     S_BLOCK_REQ  = 5'd9,
                     S_READ_END   = 5'd10,
                     S_COOLDOWN   = 5'd11,
                     S_RETRY      = 5'd12,
                     S_INIT_FAIL  = 5'd13;

    reg [4:0]  state;
    reg [31:0] sec_addr;
    reg [7:0]  retry_cnt;
    reg [23:0] retry_wait;
    reg [23:0] acmd41_wait;
    reg [2:0]  ocr_cnt;
    reg [31:0] ocr;
    reg        card_is_sdhc;
    reg [23:0] block_wait_cnt;
    reg [23:0] cmd_wait_cnt;
    reg [23:0] power_wait_cnt;
    reg [3:0]  cmd0_tries;

    wire cmd_ok   = cmd_req_ack && !cmd_req_error;
    wire cmd_fail = cmd_req_ack &&  cmd_req_error;

    assign sd_sec_read_end = (state == S_READ_END);
    assign dbg_state       = state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state                <= S_IDLE;
            sd_init_done         <= 1'b0;
            sd_init_error        <= 1'b0;
            init_fail_code       <= 3'd0;
            sd_sec_read_data     <= 8'd0;
            sd_sec_read_data_valid <= 1'b0;
            sd_sec_read_error    <= 1'b0;
            sd_sec_read_error_code <= 4'd0;
            dbg_fault_state      <= 5'd0;
            dbg_error_seen       <= 1'b0;
            spi_clk_div          <= SPI_LOW_SPEED_DIV;
            cmd_req              <= 1'b0;
            cmd                  <= 48'd0;
            cmd_r1               <= 8'd0;
            cmd_data_len         <= 16'd0;
            block_read_req       <= 1'b0;
            sec_addr             <= 32'd0;
            retry_cnt            <= 8'd0;
            retry_wait           <= 24'd0;
            acmd41_wait          <= 24'd0;
            ocr_cnt              <= 3'd0;
            ocr                  <= 32'd0;
            card_is_sdhc         <= 1'b1;
            block_wait_cnt       <= 24'd0;
            cmd_wait_cnt         <= 24'd0;
            power_wait_cnt       <= 24'd0;
            cmd0_tries           <= 4'd0;
        end else begin
            sd_sec_read_data_valid <= 1'b0;
            sd_sec_read_error      <= 1'b0;

            case (state)
                S_IDLE: begin
                    sd_init_done  <= 1'b0;
                    sd_init_error <= 1'b0;
                    init_fail_code <= 3'd0;
                    spi_clk_div   <= SPI_LOW_SPEED_DIV;
                    retry_cnt     <= 8'd0;
                    cmd_wait_cnt  <= 24'd0;
                    // Let the card supply settle before the first command.
                    // Re-entering this state after a rejected CMD0 also
                    // gives the card a fresh pause before the next attempt.
                    if (power_wait_cnt < POWER_WAIT_CYCLES) begin
                        power_wait_cnt <= power_wait_cnt + 24'd1;
                    end else begin
                        power_wait_cnt <= 24'd0;
                        state          <= S_CMD0;
                    end
                end

                S_CMD0: begin
                    cmd_req      <= 1'b1;
                    cmd_data_len <= 16'd0;
                    cmd_r1       <= 8'h01;
                    cmd          <= {8'd0, 8'h00, 8'h00, 8'h00, 8'h00, 8'h95};
                    if (cmd_ok) begin
                        cmd_wait_cnt <= 24'd0;
                        cmd0_tries   <= 4'd0;
                        state <= S_CMD8;
                    end else if (cmd_fail ||
                                 (cmd_wait_cnt >= CMD_TIMEOUT_CYCLES)) begin
                        // A card that was powered up with the board can miss
                        // the first command; CMD0 is safe to repeat.
                        cmd_req <= 1'b0;
                        if (cmd0_tries < (CMD0_MAX_TRIES - 4'd1)) begin
                            cmd0_tries <= cmd0_tries + 4'd1;
                            state      <= S_IDLE;
                        end else begin
                            init_fail_code <= 3'd4;
                            state          <= S_INIT_FAIL;
                        end
                    end else begin
                        cmd_wait_cnt <= cmd_wait_cnt + 24'd1;
                    end
                end

                S_CMD8: begin
                    cmd_req      <= 1'b1;
                    cmd_data_len <= 16'd4;
                    cmd_r1       <= 8'h01;
                    cmd          <= {8'd8, 8'h00, 8'h00, 8'h01, 8'hAA, 8'h87};
                    if (cmd_ok) begin
                        cmd_wait_cnt <= 24'd0;
                        state <= S_CMD55;
                    end else if (cmd_fail) begin
                        init_fail_code <= 3'd5;
                        state <= S_INIT_FAIL;
                    end else if (cmd_wait_cnt >= CMD_TIMEOUT_CYCLES) begin
                        cmd_req        <= 1'b0;
                        init_fail_code <= 3'd5;
                        state          <= S_INIT_FAIL;
                    end else begin
                        cmd_wait_cnt <= cmd_wait_cnt + 24'd1;
                    end
                end

                S_CMD55: begin
                    cmd_req      <= 1'b1;
                    cmd_data_len <= 16'd0;
                    cmd_r1       <= 8'h01;
                    cmd          <= {8'd55, 8'h00, 8'h00, 8'h00, 8'h00, 8'hFF};
                    if (cmd_ok) begin
                        cmd_wait_cnt <= 24'd0;
                        state <= S_CMD41;
                    end else if (cmd_fail) begin
                        init_fail_code <= 3'd6;
                        state <= S_INIT_FAIL;
                    end else if (cmd_wait_cnt >= CMD_TIMEOUT_CYCLES) begin
                        cmd_req        <= 1'b0;
                        init_fail_code <= 3'd6;
                        state          <= S_INIT_FAIL;
                    end else begin
                        cmd_wait_cnt <= cmd_wait_cnt + 24'd1;
                    end
                end

                S_CMD41: begin
                    cmd_req      <= 1'b1;
                    cmd_data_len <= 16'd0;
                    cmd_r1       <= 8'h00;
                    cmd          <= {8'd41, 8'h40, 8'h00, 8'h00, 8'h00, 8'hFF};
                    if (cmd_ok) begin
                        cmd_wait_cnt <= 24'd0;
                        state <= S_CMD58;
                    end else if (cmd_fail) begin
                        // A busy response is reported as an error by the
                        // command engine; give the card time, then repeat
                        // CMD55/ACMD41.
                        if (acmd41_wait == 24'd1_000_000) begin
                            init_fail_code <= 3'd6;
                            state <= S_INIT_FAIL;
                        end else
                            state <= S_RETRY;
                    end else if (cmd_wait_cnt >= CMD_TIMEOUT_CYCLES) begin
                        cmd_req        <= 1'b0;
                        init_fail_code <= 3'd6;
                        state          <= S_INIT_FAIL;
                    end else begin
                        cmd_wait_cnt <= cmd_wait_cnt + 24'd1;
                    end
                end

                S_RETRY: begin
                    cmd_req <= 1'b0;
                    cmd_wait_cnt <= 24'd0;
                    if (retry_wait == 24'd200_000) begin
                        retry_wait  <= 24'd0;
                        acmd41_wait <= acmd41_wait + 24'd1;
                        state       <= S_CMD55;
                    end else begin
                        retry_wait <= retry_wait + 24'd1;
                    end
                end

                S_CMD58: begin
                    cmd_req      <= 1'b1;
                    cmd_data_len <= 16'd4;
                    cmd_r1       <= 8'h00;
                    cmd          <= {8'd58, 8'h00, 8'h00, 8'h00, 8'h00, 8'hFF};

                    if (block_read_valid) begin
                        case (ocr_cnt)
                            // CMD58 returns the OCR MSB first.  Keep the
                            // byte order intact so OCR[30] really is CCS.
                            3'd0: ocr[31:24] <= block_read_data;
                            3'd1: ocr[23:16] <= block_read_data;
                            3'd2: ocr[15:8]  <= block_read_data;
                            3'd3: ocr[7:0]   <= block_read_data;
                            default: ;
                        endcase
                        ocr_cnt <= ocr_cnt + 3'd1;
                    end

                    if (cmd_ok) begin
                        cmd_wait_cnt <= 24'd0;
                        // SDHC/SDXC cards have a fixed 512 byte block length
                        // and need no SET_BLOCKLEN.  Cards that do not accept
                        // CMD16 answer with an error bit, which used to send
                        // bring-up straight into S_INIT_FAIL.  CCS is OCR[30],
                        // captured above while the four OCR bytes arrived.
                        if (ocr[30]) begin
                            card_is_sdhc <= 1'b1;
                            sd_init_done <= 1'b1;
                            spi_clk_div  <= SPI_HIGH_SPEED_DIV;
                            cmd_req      <= 1'b0;
                            state        <= S_WAIT_READ;
                        end else begin
                            state <= S_CMD16;
                        end
                    end else if (cmd_fail) begin
                        init_fail_code <= 3'd7;
                        state <= S_INIT_FAIL;
                    end else if (cmd_wait_cnt >= CMD_TIMEOUT_CYCLES) begin
                        cmd_req        <= 1'b0;
                        init_fail_code <= 3'd7;
                        state          <= S_INIT_FAIL;
                    end else begin
                        cmd_wait_cnt <= cmd_wait_cnt + 24'd1;
                    end
                end

                S_CMD16: begin
                    cmd_req      <= 1'b1;
                    cmd_data_len <= 16'd0;
                    cmd_r1       <= 8'h00;
                    cmd          <= {8'd16, 8'h00, 8'h00, 8'h02, 8'h00, 8'hFF};
                    if (cmd_ok) begin
                        card_is_sdhc <= ocr[30];
                        sd_init_done <= 1'b1;
                        spi_clk_div  <= SPI_HIGH_SPEED_DIV;
                        cmd_req      <= 1'b0;
                        cmd_wait_cnt <= 24'd0;
                        state        <= S_WAIT_READ;
                    end else if (cmd_fail ||
                                 (cmd_wait_cnt >= CMD_TIMEOUT_CYCLES)) begin
                        // A card that rejects SET_BLOCKLEN still reads 512 byte
                        // blocks: that is the block length it uses after
                        // reset.  Continue into the data path instead of
                        // declaring the initialization dead.
                        cmd_req      <= 1'b0;
                        card_is_sdhc <= 1'b0;
                        sd_init_done <= 1'b1;
                        spi_clk_div  <= SPI_HIGH_SPEED_DIV;
                        state        <= S_WAIT_READ;
                    end else begin
                        cmd_wait_cnt <= cmd_wait_cnt + 24'd1;
                    end
                end

                S_WAIT_READ: begin
                    cmd_req        <= 1'b0;
                    block_read_req <= 1'b0;
                    block_wait_cnt <= 24'd0;
                    cmd_wait_cnt   <= 24'd0;
                    sd_sec_read_error_code <= 4'd0;
                    if (sd_sec_read) begin
                        // SDHC uses the logical block address directly;
                        // SDSC addresses the card in bytes.
                        sec_addr <= card_is_sdhc ? sd_sec_read_addr
                                               : {sd_sec_read_addr[22:0], 9'b0};
                        state    <= S_CMD17;
                    end
                end

                S_CMD17: begin
                    cmd_req      <= 1'b1;
                    cmd_data_len <= 16'd0;
                    cmd_r1       <= 8'h00;
                    cmd          <= {8'd17, sec_addr, 8'hFF};
                    if (cmd_ok) begin
                        // Drop cmd_req before entering the block-data state.
                        // Otherwise sd_card_cmd sees the still-high request
                        // when it returns to S_WAIT and issues a second CMD17.
                        cmd_req <= 1'b0;
                        cmd_wait_cnt <= 24'd0;
                        state <= S_BLOCK_REQ;
                    end else if (cmd_fail) begin
                        cmd_req           <= 1'b0;
                        sd_sec_read_error <= 1'b1;
                        sd_sec_read_error_code <= 4'd8;
                        dbg_fault_state   <= state;
                        dbg_error_seen    <= 1'b1;
                        state             <= S_COOLDOWN;
                    end else if (cmd_wait_cnt >= CMD_TIMEOUT_CYCLES) begin
                        // A card that has already completed initialization
                        // should answer CMD17 within a few tens of
                        // microseconds.  Report a command-response timeout
                        // explicitly instead of leaving the sector wrapper
                        // to expire with its generic code 4.
                        cmd_req           <= 1'b0;
                        cmd_wait_cnt      <= 24'd0;
                        sd_sec_read_error <= 1'b1;
                        sd_sec_read_error_code <= 4'd11;
                        dbg_fault_state   <= state;
                        dbg_error_seen    <= 1'b1;
                        state             <= S_COOLDOWN;
                    end else begin
                        cmd_wait_cnt <= cmd_wait_cnt + 24'd1;
                    end
                end

                S_BLOCK_REQ: begin
                    block_read_req <= 1'b1;
                    if (block_read_req_ack) begin
                        block_read_req <= 1'b0;
                        block_wait_cnt <= 24'd0;
                        state          <= S_READ_END;
                    end else if (block_wait_cnt >= BLOCK_TIMEOUT_CYCLES) begin
                        // About 250 ms at 50 MHz.  Abort the byte engine by
                        // withdrawing block_read_req, then let the sector
                        // reader report a precise token timeout.
                        block_read_req <= 1'b0;
                        block_wait_cnt <= 24'd0;
                        sd_sec_read_error <= 1'b1;
                        sd_sec_read_error_code <= 4'd9;
                        dbg_fault_state   <= state;
                        dbg_error_seen    <= 1'b1;
                        state          <= S_COOLDOWN;
                    end else begin
                        block_wait_cnt <= block_wait_cnt + 24'd1;
                    end
                end

                S_READ_END: begin
                    state <= S_COOLDOWN;
                end

                S_COOLDOWN: begin
                    block_read_req <= 1'b0;
                    if (!sd_sec_read)
                        state <= S_WAIT_READ;
                end

                S_INIT_FAIL: begin
                    cmd_req       <= 1'b0;
                    sd_init_done  <= 1'b0;
                    sd_init_error <= 1'b1;
                    state         <= S_WAIT_READ;
                end

                default: state <= S_IDLE;
            endcase

            // The data valid pulse is qualified here so the consumer only sees
            // it during a block read, not during command argument clocks.
            if ((state == S_BLOCK_REQ) && block_read_valid)
                sd_sec_read_data_valid <= 1'b1;
            if ((state == S_BLOCK_REQ) && block_read_valid)
                sd_sec_read_data <= block_read_data;
        end
    end

endmodule
