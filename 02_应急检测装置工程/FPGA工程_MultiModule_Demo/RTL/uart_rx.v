//=============================================================================
// 文件名   : uart_rx.v
// 功能     : 简单 UART 接收器，默认 115200-8-N-1
// 说明     :
//   每收到一个完整字节，rx_valid 输出一个时钟周期高脉冲。
//   本工程只使用它判断蓝牙端是否曾经发来过数据，不解析具体命令。
//=============================================================================
module uart_rx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);

    localparam integer DIV = CLK_HZ / BAUD;

    reg [1:0]  rx_sync;
    reg [15:0] div_cnt;
    reg [3:0]  bit_idx;
    reg [7:0]  shift;
    reg        busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync  <= 2'b11;
            div_cnt  <= 16'd0;
            bit_idx  <= 4'd0;
            shift    <= 8'h00;
            busy     <= 1'b0;
            rx_data  <= 8'h00;
            rx_valid <= 1'b0;
        end
        else begin
            rx_sync  <= {rx_sync[0], rx};
            rx_valid <= 1'b0;

            if (!busy) begin
                div_cnt <= 16'd0;
                bit_idx <= 4'd0;

                // 连续采样到低电平后，再等待半个位周期进入数据位中心。
                if (rx_sync[1] == 1'b0) begin
                    busy    <= 1'b1;
                    div_cnt <= DIV / 2;
                end
            end
            else begin
                if (div_cnt == DIV - 1) begin
                    div_cnt <= 16'd0;

                    if (bit_idx == 3'd0) begin
                        // 第一次采样理论上位于起始位中心。
                        bit_idx <= 4'd1;
                    end
                    else if (bit_idx <= 4'd8) begin
                        shift   <= {rx_sync[1], shift[7:1]};
                        bit_idx <= bit_idx + 1'b1;
                    end
                    else begin
                        // bit_idx == 9，采样停止位后输出数据。
                        busy     <= 1'b0;
                        rx_data  <= shift;
                        rx_valid <= 1'b1;
                    end
                end
                else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end

endmodule
