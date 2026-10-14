`timescale 1ns / 1ps
//=============================================================================
// uart_tx.v - 8N1 UART transmitter
//
// "div" is latched when a byte is accepted, so the divider can be changed
// with the same command whose acknowledgement is still being shifted out.
// "busy" is high from the accepted send pulse until the stop bit ends, and
// "done" is a one clock pulse at that moment.
//=============================================================================
module uart_tx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] div,         // system clocks per bit
    input  wire        send,        // one clock pulse
    input  wire [7:0]  data,
    output reg         tx,
    output reg         busy,
    output reg         done
);

    reg [15:0] cnt;
    reg [15:0] div_q;
    reg [3:0]  bit_idx;     // 0 = start, 1..8 = data, 9 = stop
    reg [7:0]  sh;

    wire [15:0] div_use = (div == 16'd0) ? 16'd434 : div;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx      <= 1'b1;
            busy    <= 1'b0;
            done    <= 1'b0;
            cnt     <= 16'd0;
            div_q   <= 16'd434;
            bit_idx <= 4'd0;
            sh      <= 8'd0;
        end else begin
            done <= 1'b0;

            if (!busy) begin
                tx <= 1'b1;
                if (send) begin
                    div_q   <= div_use;
                    sh      <= data;
                    bit_idx <= 4'd0;
                    cnt     <= div_use - 16'd1;
                    tx      <= 1'b0;            // start bit
                    busy    <= 1'b1;
                end
            end else if (cnt != 16'd0) begin
                cnt <= cnt - 16'd1;
            end else begin
                cnt <= div_q - 16'd1;
                if (bit_idx == 4'd9) begin
                    tx      <= 1'b1;            // stop bit is already on the line
                    busy    <= 1'b0;
                    done    <= 1'b1;
                end else begin
                    bit_idx <= bit_idx + 4'd1;
                    if (bit_idx == 4'd8)
                        tx <= 1'b1;             // stop bit
                    else
                        tx <= sh[0];
                    sh <= {1'b0, sh[7:1]};
                end
            end
        end
    end

endmodule
