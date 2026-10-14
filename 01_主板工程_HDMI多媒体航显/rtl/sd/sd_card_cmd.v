`timescale 1ns / 1ps
//=============================================================================
// sd_card_cmd.v - SD command framing and block transfer engine
//
// Read path used by this project:
//   CMD0, CMD8, CMD55/ACMD41, CMD58, CMD16, CMD17
//
// The write path is retained from the vendor reference so the block interface
// stays compatible, but the multimedia player does not enable it.
//=============================================================================
module sd_card_cmd (
    input  wire        sys_clk,
    input  wire        rst,             // active high

    input  wire [15:0] spi_clk_div,
    input  wire        cmd_req,
    output wire        cmd_req_ack,
    output reg         cmd_req_error,
    input  wire [47:0] cmd,
    input  wire [7:0]  cmd_r1,
    input  wire [15:0] cmd_data_len,

    input  wire        block_read_req,
    output reg         block_read_valid,
    output reg  [7:0]  block_read_data,
    output wire        block_read_req_ack,

    input  wire        block_write_req,
    input  wire [7:0]  block_write_data,
    output wire        block_write_data_rd,
    output wire        block_write_req_ack,

    output wire        nCS_ctrl,
    output reg  [15:0] clk_div,
    output reg         spi_wr_req,
    input  wire        spi_wr_ack,
    output wire [7:0]  spi_data_in,
    input  wire [7:0]  spi_data_out
);

    localparam [4:0] S_IDLE         = 5'd0,
                     S_WAIT         = 5'd1,
                     S_INIT         = 5'd2,
                     S_CMD_PRE      = 5'd3,
                     S_CMD          = 5'd4,
                     S_CMD_DATA     = 5'd5,
                     S_READ_WAIT    = 5'd6,
                     S_READ         = 5'd7,
                     S_READ_ACK     = 5'd8,
                     S_WRITE_TOKEN  = 5'd9,
                     S_WRITE_DATA_0 = 5'd10,
                     S_WRITE_DATA_1 = 5'd11,
                     S_WRITE_CRC    = 5'd12,
                     S_WRITE_SUC    = 5'd13,
                     S_WRITE_BUSY   = 5'd14,
                     S_WRITE_ACK    = 5'd15,
                     S_ERR          = 5'd16,
                     S_END          = 5'd17;

    // In SPI mode a card answers within eight byte times.  The limit only
    // guards against a card that never answers: without it a dead card keeps
    // the command engine busy for 65 535 dummy bytes.
    localparam [15:0] R1_SCAN_LIMIT = 16'd64;

    reg [4:0]  state;
    reg        CS_reg;
    reg [15:0] byte_cnt;
    reg [7:0]  send_data;
    reg [9:0]  wr_data_cnt;

    wire [7:0] data_recv = spi_data_out;

    assign cmd_req_ack           = (state == S_END);
    assign block_read_req_ack    = (state == S_READ_ACK);
    assign block_write_req_ack   = (state == S_WRITE_ACK);
    assign block_write_data_rd   = (state == S_WRITE_DATA_0);
    assign spi_data_in           = send_data;
    assign nCS_ctrl              = CS_reg;

    always @(posedge sys_clk or posedge rst) begin
        if (rst) begin
            state          <= S_IDLE;
            CS_reg         <= 1'b1;
            spi_wr_req     <= 1'b0;
            byte_cnt       <= 16'd0;
            clk_div        <= 16'd0;
            send_data      <= 8'hFF;
            cmd_req_error  <= 1'b0;
            wr_data_cnt    <= 10'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    state     <= S_INIT;
                    clk_div   <= spi_clk_div;
                    CS_reg    <= 1'b1;
                    byte_cnt  <= 16'd0;
                    spi_wr_req<= 1'b0;
                end

                S_INIT: begin
                    // At least 74 clocks with CS high before the first command.
                    if (spi_wr_ack) begin
                        if (byte_cnt >= 16'd10) begin
                            byte_cnt   <= 16'd0;
                            spi_wr_req <= 1'b0;
                            state      <= S_WAIT;
                        end else begin
                            byte_cnt <= byte_cnt + 16'd1;
                        end
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_WAIT: begin
                    cmd_req_error <= 1'b0;
                    wr_data_cnt   <= 10'd0;
                    clk_div       <= spi_clk_div;
                    if (cmd_req)
                        state <= S_CMD_PRE;
                    else if (block_read_req)
                        state <= S_READ_WAIT;
                    else if (block_write_req)
                        state <= S_WRITE_TOKEN;
                end

                S_CMD_PRE: begin
                    if (spi_wr_ack) begin
                        state      <= S_CMD;
                        spi_wr_req <= 1'b0;
                        byte_cnt   <= 16'd0;
                    end else begin
                        spi_wr_req <= 1'b1;
                        CS_reg     <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_CMD: begin
                    if (spi_wr_ack) begin
                        // The card must see all six command bytes before its
                        // R1 response is valid.  MISO can be low while the
                        // command is being shifted out, which used to look
                        // like an immediate R1=0x00 response on CMD16/CMD17.
                        if (byte_cnt < 16'd6) begin
                            byte_cnt <= byte_cnt + 16'd1;
                        end else if (data_recv == cmd_r1) begin
                            spi_wr_req <= 1'b0;
                            if (cmd_data_len != 16'd0) begin
                                state    <= S_CMD_DATA;
                                byte_cnt <= 16'd0;
                            end else begin
                                state    <= S_END;
                                byte_cnt <= 16'd0;
                            end
                        end else if ((data_recv[7] == 1'b0) ||
                                     (byte_cnt >= R1_SCAN_LIMIT)) begin
                            state      <= S_ERR;
                            spi_wr_req <= 1'b0;
                            byte_cnt   <= 16'd0;
                        end else begin
                            byte_cnt <= byte_cnt + 16'd1;
                        end
                    end else begin
                        spi_wr_req <= 1'b1;
                        CS_reg     <= 1'b0;
                        case (byte_cnt)
                            16'd0: send_data <= (cmd[47:40] | 8'h40);
                            16'd1: send_data <= cmd[39:32];
                            16'd2: send_data <= cmd[31:24];
                            16'd3: send_data <= cmd[23:16];
                            16'd4: send_data <= cmd[15:8];
                            16'd5: send_data <= cmd[7:0];
                            default: send_data <= 8'hFF;
                        endcase
                    end
                end

                S_CMD_DATA: begin
                    if (spi_wr_ack) begin
                        if (byte_cnt == (cmd_data_len - 16'd1)) begin
                            state      <= S_END;
                            spi_wr_req <= 1'b0;
                            byte_cnt   <= 16'd0;
                        end else begin
                            byte_cnt <= byte_cnt + 16'd1;
                        end
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_READ_WAIT: begin
                    if (!block_read_req) begin
                        // The sector layer timed out and withdrew the request.
                        spi_wr_req <= 1'b0;
                        CS_reg     <= 1'b1;
                        state      <= S_WAIT;
                    end else if (spi_wr_ack && (data_recv == 8'hFE)) begin
                        spi_wr_req <= 1'b0;
                        state      <= S_READ;
                        byte_cnt   <= 16'd0;
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_READ: begin
                    if (!block_read_req) begin
                        spi_wr_req <= 1'b0;
                        CS_reg     <= 1'b1;
                        state      <= S_WAIT;
                    end else if (spi_wr_ack) begin
                        if (byte_cnt == 16'd513) begin
                            state      <= S_READ_ACK;
                            spi_wr_req <= 1'b0;
                            byte_cnt   <= 16'd0;
                        end else begin
                            byte_cnt <= byte_cnt + 16'd1;
                        end
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_WRITE_TOKEN: begin
                    if (spi_wr_ack) begin
                        state      <= S_WRITE_DATA_0;
                        spi_wr_req <= 1'b0;
                        byte_cnt   <= 16'd0;
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFE;
                    end
                end

                S_WRITE_DATA_0: begin
                    state       <= S_WRITE_DATA_1;
                    wr_data_cnt <= wr_data_cnt + 10'd1;
                end

                S_WRITE_DATA_1: begin
                    if (spi_wr_ack && (wr_data_cnt == 10'd512)) begin
                        state      <= S_WRITE_CRC;
                        spi_wr_req <= 1'b0;
                    end else if (spi_wr_ack) begin
                        state      <= S_WRITE_DATA_0;
                        spi_wr_req <= 1'b0;
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= block_write_data;
                    end
                end

                S_WRITE_CRC: begin
                    if (spi_wr_ack) begin
                        if (byte_cnt == 16'd1) begin
                            state      <= S_WRITE_SUC;
                            spi_wr_req <= 1'b0;
                            byte_cnt   <= 16'd0;
                        end else begin
                            byte_cnt <= byte_cnt + 16'd1;
                        end
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_WRITE_SUC: begin
                    if (spi_wr_ack) begin
                        if (data_recv[4:0] == 5'b00101) begin
                            state      <= S_WRITE_BUSY;
                            spi_wr_req <= 1'b0;
                        end
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_WRITE_BUSY: begin
                    if (spi_wr_ack) begin
                        if (data_recv == 8'hFF) begin
                            state      <= S_WRITE_ACK;
                            spi_wr_req <= 1'b0;
                        end
                    end else begin
                        spi_wr_req <= 1'b1;
                        send_data  <= 8'hFF;
                    end
                end

                S_ERR: begin
                    state         <= S_END;
                    cmd_req_error <= 1'b1;
                end

                S_READ_ACK,
                S_WRITE_ACK,
                S_END: begin
                    state <= S_WAIT;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            block_read_valid <= 1'b0;
        else if (((state == S_READ) && (byte_cnt < 16'd512)) ||
                 (state == S_CMD_DATA))
            block_read_valid <= spi_wr_ack;
        else
            block_read_valid <= 1'b0;
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            block_read_data <= 8'd0;
        else if (((state == S_READ) && spi_wr_ack) ||
                 (state == S_CMD_DATA))
            block_read_data <= data_recv;
    end

endmodule
