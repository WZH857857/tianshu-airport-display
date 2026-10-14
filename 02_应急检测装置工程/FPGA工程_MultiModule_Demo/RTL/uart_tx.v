//=============================================================================
// 文件名   : uart_tx.v
// 功能     : 简单 UART 发送器，默认 115200-8-N-1
// 适用工程 : E104 / EG4X20BG256
// 说明     :
//   1. start 拉高一个时钟周期后，开始发送 data 中的 8 位数据。
//   2. tx_busy 为 1 时不要再次给出 start，否则本次发送可能被忽略。
//   3. tx_done 在停止位发送结束时给出一个时钟周期的高脉冲。
//=============================================================================
module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx,
    output reg        tx_busy,
    output reg        tx_done
);

    localparam integer DIV = CLK_HZ / BAUD;
    localparam [9:0]   IDLE_BITS = 10'b1111111111;

    reg [15:0] div_cnt;
    reg [3:0]  bit_cnt;
    reg [9:0]  shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 16'd0;
            bit_cnt <= 4'd0;
            shift   <= IDLE_BITS;
            tx      <= 1'b1;
            tx_busy <= 1'b0;
            tx_done <= 1'b0;
        end
        else begin
            tx_done <= 1'b0;

            if (!tx_busy) begin
                tx <= 1'b1;
                if (start) begin
                    // 位顺序：起始位、D0~D7、停止位。
                    shift   <= {1'b1, data, 1'b0};
                    bit_cnt <= 4'd0;
                    div_cnt <= 16'd0;
                    tx      <= 1'b0;
                    tx_busy <= 1'b1;
                end
            end
            else begin
                if (div_cnt == DIV - 1) begin
                    div_cnt <= 16'd0;
                    tx      <= shift[1];          // 输出下一位
                    shift   <= {1'b1, shift[9:1]};

                    if (bit_cnt == 4'd9) begin
                        tx_busy <= 1'b0;
                        tx_done <= 1'b1;
                        tx      <= 1'b1;
                    end
                    else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
                else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end

endmodule
