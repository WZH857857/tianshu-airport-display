`timescale 1ns / 1ps
//=============================================================================
// uart_rx.v - 8N1 UART receiver
//
// The receiver samples the line in the middle of every bit with a counter
// that is loaded from "div" (system clocks per bit).  "div" is a port rather
// than a parameter so the host can raise the baud rate at run time, which is
// what the picture upload path needs.  The value is latched on the falling
// edge of the start bit, so a baud change command can never tear a byte
// apart: the byte that carries the command still finishes at the old rate.
//
// The input pin comes straight from the CH340 and is asynchronous to the
// 50 MHz system clock, so it passes through a two flop synchroniser.  The
// third flop is only used for edge detection.
//=============================================================================
module uart_rx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] div,         // system clocks per bit
    input  wire        rx,          // asynchronous serial input
    output reg  [7:0]  data,
    output reg         data_valid   // one clock pulse per received byte
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg        rx_s1, rx_s2, rx_s3;
    reg [1:0]  state;
    reg [15:0] cnt;
    reg [15:0] div_q;
    reg [2:0]  bit_idx;
    reg [7:0]  sh;

    wire [15:0] div_use = (div == 16'd0) ? 16'd434 : div;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s1      <= 1'b1;
            rx_s2      <= 1'b1;
            rx_s3      <= 1'b1;
            state      <= S_IDLE;
            cnt        <= 16'd0;
            div_q      <= 16'd434;
            bit_idx    <= 3'd0;
            sh         <= 8'd0;
            data       <= 8'd0;
            data_valid <= 1'b0;
        end else begin
            rx_s1 <= rx;
            rx_s2 <= rx_s1;
            rx_s3 <= rx_s2;

            data_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    // falling edge of a start bit
                    if (!rx_s2 && rx_s3) begin
                        div_q   <= div_use;
                        cnt     <= {1'b0, div_use[15:1]};
                        bit_idx <= 3'd0;
                        state   <= S_START;
                    end
                end

                S_START: begin
                    // one more look half a bit later rejects a glitch
                    if (cnt == 16'd0) begin
                        if (!rx_s2) begin
                            cnt   <= div_q - 16'd1;
                            state <= S_DATA;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                S_DATA: begin
                    if (cnt == 16'd0) begin
                        sh <= {rx_s2, sh[7:1]};     // LSB first
                        cnt <= div_q - 16'd1;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                default: begin
                    // stop bit: report the byte, a framing error is ignored
                    if (cnt == 16'd0) begin
                        data       <= sh;
                        data_valid <= 1'b1;
                        state      <= S_IDLE;
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end
            endcase
        end
    end

endmodule
