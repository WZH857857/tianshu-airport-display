//=============================================================================
// 文件名   : uart_status_streamer.v
// 功能     : 每秒通过蓝牙串口发送一行板卡状态
// 默认格式 :
//   E104 STATUS T=NORMAL F=SAFE  R=12:34:56 B=ONLINE\r\n
//   E104 STATUS T=ALARM  F=FIRE  R=12:34:56 B=NO LNK\r\n
// 端口方向 :
//   模块 TXD -> FPGA K16，对应本工程 uart_rx
//   模块 RXD <- FPGA K15，对应本工程 uart_tx
//=============================================================================
module uart_status_streamer #(
    parameter integer CLK_HZ = 50_000_000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       th_alarm,
    input  wire       flame_alarm,
    input  wire       rtc_valid,
    input  wire [7:0] rtc_hour_bcd,
    input  wire [7:0] rtc_min_bcd,
    input  wire [7:0] rtc_sec_bcd,
    input  wire       ble_online,
    input  wire       tx_busy,
    output reg        tx_start,
    output reg  [7:0] tx_data
);

    localparam integer PERIOD_CYCLES = CLK_HZ;
    localparam [5:0] PACKET_LEN = 6'd49;

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_LOAD = 2'd1;
    localparam [1:0] S_WAIT = 2'd2;
    localparam [1:0] S_SEND = 2'd3;

    reg [1:0]  state;
    reg [26:0] period_cnt;
    reg [5:0]  char_idx;

    function [7:0] ascii_digit;
        input [3:0] value;
        begin
            ascii_digit = {4'h3, value};
        end
    endfunction

    // 固定 49 字节，方便蓝牙串口和逻辑分析仪查看。
    function [7:0] packet_char;
        input [5:0] idx;
        begin
            packet_char = 8'h20;
            case (idx)
                6'd0:  packet_char = 8'h45; // E
                6'd1:  packet_char = 8'h31; // 1
                6'd2:  packet_char = 8'h30; // 0
                6'd3:  packet_char = 8'h34; // 4
                6'd5:  packet_char = 8'h53; // S
                6'd6:  packet_char = 8'h54; // T
                6'd7:  packet_char = 8'h41; // A
                6'd8:  packet_char = 8'h54; // T
                6'd9:  packet_char = 8'h55; // U
                6'd10: packet_char = 8'h53; // S
                6'd12: packet_char = 8'h54; // T
                6'd13: packet_char = 8'h3D; // =
                6'd14: packet_char = th_alarm ? 8'h41 : 8'h4E;
                6'd15: packet_char = th_alarm ? 8'h4C : 8'h4F;
                6'd16: packet_char = th_alarm ? 8'h41 : 8'h52;
                6'd17: packet_char = th_alarm ? 8'h52 : 8'h4D;
                6'd18: packet_char = th_alarm ? 8'h4D : 8'h41;
                6'd19: packet_char = th_alarm ? 8'h20 : 8'h4C;
                6'd21: packet_char = 8'h46; // F
                6'd22: packet_char = 8'h3D; // =
                6'd23: packet_char = flame_alarm ? 8'h46 : 8'h53;
                6'd24: packet_char = flame_alarm ? 8'h49 : 8'h41;
                6'd25: packet_char = flame_alarm ? 8'h52 : 8'h46;
                6'd26: packet_char = flame_alarm ? 8'h45 : 8'h45;
                6'd28: packet_char = 8'h52; // R
                6'd29: packet_char = 8'h3D; // =
                6'd30: packet_char = rtc_valid ? ascii_digit(rtc_hour_bcd[7:4]) : 8'h2D;
                6'd31: packet_char = rtc_valid ? ascii_digit(rtc_hour_bcd[3:0]) : 8'h2D;
                6'd32: packet_char = 8'h3A;
                6'd33: packet_char = rtc_valid ? ascii_digit(rtc_min_bcd[7:4]) : 8'h2D;
                6'd34: packet_char = rtc_valid ? ascii_digit(rtc_min_bcd[3:0]) : 8'h2D;
                6'd35: packet_char = 8'h3A;
                6'd36: packet_char = rtc_valid ? ascii_digit(rtc_sec_bcd[7:4]) : 8'h2D;
                6'd37: packet_char = rtc_valid ? ascii_digit(rtc_sec_bcd[3:0]) : 8'h2D;
                6'd39: packet_char = 8'h42; // B
                6'd40: packet_char = 8'h3D; // =
                6'd41: packet_char = ble_online ? 8'h4F : 8'h4E;
                6'd42: packet_char = ble_online ? 8'h4E : 8'h4F;
                6'd43: packet_char = ble_online ? 8'h4C : 8'h20;
                6'd44: packet_char = ble_online ? 8'h49 : 8'h4C;
                6'd45: packet_char = ble_online ? 8'h4E : 8'h4E;
                6'd46: packet_char = ble_online ? 8'h45 : 8'h4B;
                6'd47: packet_char = 8'h0D; // CR
                6'd48: packet_char = 8'h0A; // LF
                default: packet_char = 8'h20;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            period_cnt <= 27'd0;
            char_idx   <= 6'd0;
            tx_start   <= 1'b0;
            tx_data    <= 8'h00;
        end
        else begin
            tx_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (period_cnt == PERIOD_CYCLES - 1) begin
                        period_cnt <= 27'd0;
                        char_idx   <= 6'd0;
                        state      <= S_LOAD;
                    end
                    else begin
                        period_cnt <= period_cnt + 1'b1;
                    end
                end

                S_LOAD: begin
                    tx_data  <= packet_char(char_idx);
                    tx_start <= 1'b1;
                    state    <= S_WAIT;
                end

                S_WAIT: begin
                    // 等待 uart_tx 接收 start 并拉高 tx_busy。
                    if (tx_busy) begin
                        state <= S_SEND;
                    end
                end

                S_SEND: begin
                    if (!tx_busy) begin
                        if (char_idx == PACKET_LEN - 1'b1) begin
                            state <= S_IDLE;
                        end
                        else begin
                            char_idx <= char_idx + 1'b1;
                            state    <= S_LOAD;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
