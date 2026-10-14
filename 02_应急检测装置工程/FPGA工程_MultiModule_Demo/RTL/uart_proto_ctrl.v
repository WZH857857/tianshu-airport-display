//=============================================================================
// 文件名   : uart_proto_ctrl.v
// 功能     : 解析“天枢空港智显上位机”协议帧，作为板卡应答状态轮询
// 帧格式   : A5 5A CMD LEN P0..P(LEN-1) XOR
//            校验 XOR = CMD ^ LEN ^ P0 ^ ... ^ P(LEN-1)，不含帧头
// 支持命令 :
//   CMD_STATUS = 0x10 -> R_STATUS = 0x90 + 25 字节状态（上位机每 0.5 s 轮询）
//   CMD_CLEAR  = 0x11 -> R_CLEAR  = 0x91（清除温湿度/火焰锁存报警）
//   CMD_PING   = 0x01 -> R_PING   = 0x81
//   其它命令         -> R_ERROR  = 0xE0 + [命令字, 错误码 1]
// R_STATUS 载荷与上位机固定状态栏的对应关系 :
//   [0]  版本标识 0xE5
//   [1]  槽位 0
//   [2]  模块总数 9（DHT11/火焰/PIR/RTC/LCD/蓝牙/MQ-2/水位/土壤）
//   [3]  状态位：bit0 整机报警 bit1 温度 bit2 湿度 bit3 火焰 bit4 人体
//        bit5 MQ-2 烟雾预警
//   [4]  温度报警级别（报警 7 / 正常 0）
//   [5]  火焰报警级别（报警 7 / 正常 0）
//   [6]  bit0 温度报警 bit1 湿度报警
//   [7]  在线位图：bit0 蓝牙 bit1 RTC bit2 湿度报警 bit3 火焰 bit4 人体
//        bit5 DHT11 数据有效
//   [8]  RTC 秒（十进制 0-59）
//   [9]  bit0 已收到轮询 bit1 板卡正常
//   [10] RTC 小时 (BCD)
//   [11] RTC 分钟 (BCD)
//   [12] RTC 秒   (BCD)
//   [13] 温度数值（整数摄氏度）
//   [14] 湿度数值（整数百分比）
//   [15] 火焰 AO 低 8 位
//   [16] 火焰 AO 高 4 位（bit3:0）
//   [17] 功能位图 0x0F：TFT/DHT11/ADC/PIR 全部启用
//   [18] 扩展状态：bit0 MQ-2 DO 报警 bit1 土壤 DO 湿润
//   [19] MQ-2 烟雾相对浓度 0~100 %
//   [20] 水位 0~100 %
//   [21] 土壤湿度 0~100 %
//   [22] MQ-2 AO 低 8 位
//   [23] MQ-2 AO 高 4 位（bit3:0）
//   [24] 功能位图 2：bit0 MQ-2 bit1 水位 bit2 土壤 bit3 土壤 DO
// 端口方向 :
//   蓝牙模块 TXD -> FPGA K16（uart_rx）
//   蓝牙模块 RXD <- FPGA K15（uart_tx）
//=============================================================================
module uart_proto_ctrl (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_valid,
    input  wire [7:0] rx_data,
    input  wire [7:0] temp_value,
    input  wire [7:0] humi_value,
    input  wire       dht_ok,
    input  wire       temp_alarm,
    input  wire       humi_alarm,
    input  wire       flame_alarm,
    input  wire [11:0] flame_ao,
    input  wire       smoke_alarm,
    input  wire [11:0] smoke_ao,
    input  wire [7:0] smoke_value,
    input  wire [7:0] water_value,
    input  wire [7:0] soil_value,
    input  wire       soil_wet,
    input  wire       pir_active,
    input  wire       rtc_valid,
    input  wire [7:0] rtc_sec_bcd,
    input  wire [7:0] rtc_min_bcd,
    input  wire [7:0] rtc_hour_bcd,
    input  wire       ble_online,
    output reg        clear_pulse,
    output reg        tx_start,
    output reg  [7:0] tx_data,
    input  wire       tx_busy
);

    //------------------------------------------------------------------
    // 协议常量
    //------------------------------------------------------------------
    localparam [7:0] HDR1       = 8'hA5;
    localparam [7:0] HDR2       = 8'h5A;
    localparam [7:0] CMD_PING   = 8'h01;
    localparam [7:0] CMD_STATUS = 8'h10;
    localparam [7:0] CMD_CLEAR  = 8'h11;
    localparam [7:0] R_PING     = 8'h81;
    localparam [7:0] R_STATUS   = 8'h90;
    localparam [7:0] R_CLEAR    = 8'h91;
    localparam [7:0] R_ERROR    = 8'hE0;

    // 应答种类
    localparam [2:0] K_NONE   = 3'd0;
    localparam [2:0] K_PING   = 3'd1;
    localparam [2:0] K_STATUS = 3'd2;
    localparam [2:0] K_CLEAR  = 3'd3;
    localparam [2:0] K_ERROR  = 3'd4;

    // 接收 / 发送状态
    localparam [3:0] R_H1   = 4'd0;
    localparam [3:0] R_H2   = 4'd1;
    localparam [3:0] R_CMD  = 4'd2;
    localparam [3:0] R_LEN  = 4'd3;
    localparam [3:0] R_PAY  = 4'd4;
    localparam [3:0] R_CHK  = 4'd5;
    localparam [3:0] T_LOAD = 4'd6;
    localparam [3:0] T_WAIT = 4'd7;
    localparam [3:0] T_SEND = 4'd8;

    // 字节间超时：约 3 ms @50 MHz，防止半帧卡死状态机
    localparam [17:0] RX_TIMEOUT = 18'd150_000;

    //------------------------------------------------------------------
    // R_STATUS 载荷（组合生成，实时反映板卡状态）
    //------------------------------------------------------------------
    wire [3:0] rtc_sec_hi  = rtc_sec_bcd[7:4];
    wire [3:0] rtc_sec_lo  = rtc_sec_bcd[3:0];
    wire [7:0] rtc_sec_dec = ({4'b0, rtc_sec_hi} << 3)
                           + ({4'b0, rtc_sec_hi} << 1)
                           + {4'b0, rtc_sec_lo};

    wire       alarm_any = temp_alarm | humi_alarm | flame_alarm | smoke_alarm;

    wire [7:0] st_p0  = 8'hE5;                                  // 版本标识
    wire [7:0] st_p1  = 8'h00;                                  // 槽位
    wire [7:0] st_p2  = 8'd9;                                   // 模块总数
    wire [7:0] st_p3  = {2'b00, smoke_alarm, pir_active, flame_alarm,
                         humi_alarm, temp_alarm, alarm_any};    // 报警状态位
    wire [7:0] st_p4  = temp_alarm ? 8'd7 : 8'd0;               // 温度报警级别
    wire [7:0] st_p5  = flame_alarm ? 8'd7 : 8'd0;              // 火焰报警级别
    wire [7:0] st_p6  = {6'b0, humi_alarm, temp_alarm};         // 温湿度报警位
    wire [7:0] st_p7  = {2'b00, dht_ok, pir_active, flame_alarm,
                         humi_alarm, rtc_valid, ble_online};    // 在线位图
    wire [7:0] st_p8  = (rtc_valid && (rtc_sec_dec < 8'd60))
                      ? rtc_sec_dec : 8'h00;                    // RTC 秒
    wire [7:0] st_p9  = {6'b0, 1'b1, 1'b1};                     // 链路/板卡正常
    wire [7:0] st_p10 = rtc_valid ? rtc_hour_bcd : 8'h00;       // RTC 时 (BCD)
    wire [7:0] st_p11 = rtc_valid ? rtc_min_bcd  : 8'h00;       // RTC 分 (BCD)
    wire [7:0] st_p12 = rtc_valid ? rtc_sec_bcd  : 8'h00;       // RTC 秒 (BCD)
    wire [7:0] st_p13 = temp_value;                             // 温度数值
    wire [7:0] st_p14 = humi_value;                             // 湿度数值
    wire [7:0] st_p15 = flame_ao[7:0];                          // AO 低 8 位
    wire [7:0] st_p16 = {4'b0000, flame_ao[11:8]};              // AO 高 4 位
    wire [7:0] st_p17 = 8'h0F;                                  // 功能位图
    wire [7:0] st_p18 = {6'b000000, soil_wet, smoke_alarm};     // 扩展状态位
    wire [7:0] st_p19 = smoke_value;                            // MQ-2 浓度 %
    wire [7:0] st_p20 = water_value;                            // 水位 %
    wire [7:0] st_p21 = soil_value;                             // 土壤湿度 %
    wire [7:0] st_p22 = smoke_ao[7:0];                          // MQ-2 AO 低 8 位
    wire [7:0] st_p23 = {4'b0000, smoke_ao[11:8]};              // MQ-2 AO 高 4 位
    wire [7:0] st_p24 = 8'h0F;                                  // 功能位图 2

    wire [7:0] st_xor = R_STATUS ^ 8'h19
                      ^ st_p0 ^ st_p1 ^ st_p2 ^ st_p3 ^ st_p4 ^ st_p5
                      ^ st_p6 ^ st_p7 ^ st_p8 ^ st_p9 ^ st_p10 ^ st_p11
                      ^ st_p12 ^ st_p13 ^ st_p14 ^ st_p15 ^ st_p16 ^ st_p17
                      ^ st_p18 ^ st_p19 ^ st_p20 ^ st_p21 ^ st_p22 ^ st_p23
                      ^ st_p24;

    //------------------------------------------------------------------
    // 寄存器
    //------------------------------------------------------------------
    reg [3:0]  state;
    reg [7:0]  cmd_reg;
    reg [7:0]  len_reg;
    reg [7:0]  byte_cnt;
    reg [7:0]  xor_acc;
    reg [2:0]  resp_kind;
    reg [4:0]  resp_len;
    reg [4:0]  tx_idx;
    reg [17:0] rx_idle_cnt;

    // 应答开始时锁存整帧内容，避免发送途中状态变化破坏帧内一致性
    reg [7:0] lat_p0, lat_p1, lat_p2, lat_p3, lat_p4;
    reg [7:0] lat_p5, lat_p6, lat_p7, lat_p8, lat_p9;
    reg [7:0] lat_p10, lat_p11, lat_p12, lat_p13, lat_p14;
    reg [7:0] lat_p15, lat_p16, lat_p17;
    reg [7:0] lat_p18, lat_p19, lat_p20, lat_p21;
    reg [7:0] lat_p22, lat_p23, lat_p24, lat_xor;

    wire is_rx_state = (state <= R_CHK);

    //------------------------------------------------------------------
    // 应答字节查询：按索引返回完整发送帧（含帧头与校验）
    //------------------------------------------------------------------
    function [7:0] resp_byte;
        input [4:0] idx;
        begin
            resp_byte = 8'h00;
            case (resp_kind)
                K_PING: begin
                    case (idx)
                        5'd0: resp_byte = HDR1;
                        5'd1: resp_byte = HDR2;
                        5'd2: resp_byte = R_PING;
                        5'd3: resp_byte = 8'h00;
                        5'd4: resp_byte = R_PING ^ 8'h00;
                        default: resp_byte = 8'h00;
                    endcase
                end
                K_STATUS: begin
                    case (idx)
                        5'd0:  resp_byte = HDR1;
                        5'd1:  resp_byte = HDR2;
                        5'd2:  resp_byte = R_STATUS;
                        5'd3:  resp_byte = 8'h19;
                        5'd4:  resp_byte = lat_p0;
                        5'd5:  resp_byte = lat_p1;
                        5'd6:  resp_byte = lat_p2;
                        5'd7:  resp_byte = lat_p3;
                        5'd8:  resp_byte = lat_p4;
                        5'd9:  resp_byte = lat_p5;
                        5'd10: resp_byte = lat_p6;
                        5'd11: resp_byte = lat_p7;
                        5'd12: resp_byte = lat_p8;
                        5'd13: resp_byte = lat_p9;
                        5'd14: resp_byte = lat_p10;
                        5'd15: resp_byte = lat_p11;
                        5'd16: resp_byte = lat_p12;
                        5'd17: resp_byte = lat_p13;
                        5'd18: resp_byte = lat_p14;
                        5'd19: resp_byte = lat_p15;
                        5'd20: resp_byte = lat_p16;
                        5'd21: resp_byte = lat_p17;
                        5'd22: resp_byte = lat_p18;
                        5'd23: resp_byte = lat_p19;
                        5'd24: resp_byte = lat_p20;
                        5'd25: resp_byte = lat_p21;
                        5'd26: resp_byte = lat_p22;
                        5'd27: resp_byte = lat_p23;
                        5'd28: resp_byte = lat_p24;
                        5'd29: resp_byte = lat_xor;
                        default: resp_byte = 8'h00;
                    endcase
                end
                K_CLEAR: begin
                    case (idx)
                        5'd0: resp_byte = HDR1;
                        5'd1: resp_byte = HDR2;
                        5'd2: resp_byte = R_CLEAR;
                        5'd3: resp_byte = 8'h00;
                        5'd4: resp_byte = R_CLEAR ^ 8'h00;
                        default: resp_byte = 8'h00;
                    endcase
                end
                K_ERROR: begin
                    case (idx)
                        5'd0: resp_byte = HDR1;
                        5'd1: resp_byte = HDR2;
                        5'd2: resp_byte = R_ERROR;
                        5'd3: resp_byte = 8'h02;
                        5'd4: resp_byte = cmd_reg;              // 被拒绝的命令
                        5'd5: resp_byte = 8'h01;                // 错误码 1：不支持
                        5'd6: resp_byte = R_ERROR ^ 8'h02
                                        ^ cmd_reg ^ 8'h01;
                        default: resp_byte = 8'h00;
                    endcase
                end
                default: resp_byte = 8'h00;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // 接收帧 + 发送应答
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= R_H1;
            cmd_reg     <= 8'h00;
            len_reg     <= 8'h00;
            byte_cnt    <= 8'h00;
            xor_acc     <= 8'h00;
            resp_kind   <= K_NONE;
            resp_len    <= 5'd0;
            tx_idx      <= 5'd0;
            rx_idle_cnt <= 18'd0;
            clear_pulse <= 1'b0;
            tx_start    <= 1'b0;
            tx_data     <= 8'h00;
            lat_p0      <= 8'h00;
            lat_p1      <= 8'h00;
            lat_p2      <= 8'h00;
            lat_p3      <= 8'h00;
            lat_p4      <= 8'h00;
            lat_p5      <= 8'h00;
            lat_p6      <= 8'h00;
            lat_p7      <= 8'h00;
            lat_p8      <= 8'h00;
            lat_p9      <= 8'h00;
            lat_p10     <= 8'h00;
            lat_p11     <= 8'h00;
            lat_p12     <= 8'h00;
            lat_p13     <= 8'h00;
            lat_p14     <= 8'h00;
            lat_p15     <= 8'h00;
            lat_p16     <= 8'h00;
            lat_p17     <= 8'h00;
            lat_p18     <= 8'h00;
            lat_p19     <= 8'h00;
            lat_p20     <= 8'h00;
            lat_p21     <= 8'h00;
            lat_p22     <= 8'h00;
            lat_p23     <= 8'h00;
            lat_p24     <= 8'h00;
            lat_xor     <= 8'h00;
        end
        else begin
            clear_pulse <= 1'b0;
            tx_start    <= 1'b0;

            // 接收态字节间超时计数
            if (!is_rx_state || rx_valid || (state == R_H1))
                rx_idle_cnt <= 18'd0;
            else if (rx_idle_cnt == RX_TIMEOUT)
                rx_idle_cnt <= 18'd0;
            else
                rx_idle_cnt <= rx_idle_cnt + 1'b1;

            if (is_rx_state && (state != R_H1) &&
                (rx_idle_cnt == RX_TIMEOUT)) begin
                // 半帧断流：丢弃并重新找帧头
                state <= R_H1;
            end
            else begin
                case (state)
                    R_H1: begin
                        if (rx_valid && (rx_data == HDR1))
                            state <= R_H2;
                    end

                    R_H2: begin
                        if (rx_valid) begin
                            if (rx_data == HDR2)
                                state <= R_CMD;
                            else if (rx_data == HDR1)
                                state <= R_H2;
                            else
                                state <= R_H1;
                        end
                    end

                    R_CMD: begin
                        if (rx_valid) begin
                            cmd_reg <= rx_data;
                            xor_acc <= rx_data;
                            state   <= R_LEN;
                        end
                    end

                    R_LEN: begin
                        if (rx_valid) begin
                            len_reg  <= rx_data;
                            xor_acc  <= xor_acc ^ rx_data;
                            byte_cnt <= 8'd0;
                            state    <= (rx_data == 8'h00) ? R_CHK : R_PAY;
                        end
                    end

                    R_PAY: begin
                        if (rx_valid) begin
                            xor_acc <= xor_acc ^ rx_data;
                            if ((byte_cnt + 8'd1) == len_reg)
                                state <= R_CHK;
                            else
                                byte_cnt <= byte_cnt + 8'd1;
                        end
                    end

                    R_CHK: begin
                        if (rx_valid) begin
                            if (rx_data == xor_acc) begin
                                case (cmd_reg)
                                    CMD_PING: begin
                                        resp_kind <= K_PING;
                                        resp_len  <= 5'd5;
                                    end
                                    CMD_STATUS: begin
                                        resp_kind <= K_STATUS;
                                        resp_len  <= 5'd30;
                                        // 锁存整帧，发送期间状态变化不影响校验
                                        lat_p0 <= st_p0;
                                        lat_p1 <= st_p1;
                                        lat_p2 <= st_p2;
                                        lat_p3 <= st_p3;
                                        lat_p4 <= st_p4;
                                        lat_p5 <= st_p5;
                                        lat_p6 <= st_p6;
                                        lat_p7 <= st_p7;
                                        lat_p8 <= st_p8;
                                        lat_p9 <= st_p9;
                                        lat_p10 <= st_p10;
                                        lat_p11 <= st_p11;
                                        lat_p12 <= st_p12;
                                        lat_p13 <= st_p13;
                                        lat_p14 <= st_p14;
                                        lat_p15 <= st_p15;
                                        lat_p16 <= st_p16;
                                        lat_p17 <= st_p17;
                                        lat_p18 <= st_p18;
                                        lat_p19 <= st_p19;
                                        lat_p20 <= st_p20;
                                        lat_p21 <= st_p21;
                                        lat_p22 <= st_p22;
                                        lat_p23 <= st_p23;
                                        lat_p24 <= st_p24;
                                        lat_xor <= st_xor;
                                    end
                                    CMD_CLEAR: begin
                                        // 单周期清警脉冲：由顶层清除温湿度/火焰锁存报警。
                                        // 人体通道不受影响，人离开后屏幕自动回安全页。
                                        clear_pulse <= 1'b1;
                                        resp_kind   <= K_CLEAR;
                                        resp_len    <= 5'd5;
                                    end
                                    default: begin
                                        resp_kind <= K_ERROR;
                                        resp_len  <= 5'd7;
                                    end
                                endcase
                                tx_idx <= 5'd0;
                                state  <= T_LOAD;
                            end
                            else begin
                                // 校验错误：静默丢弃
                                state <= R_H1;
                            end
                        end
                    end

                    T_LOAD: begin
                        tx_data  <= resp_byte(tx_idx);
                        tx_start <= 1'b1;
                        state    <= T_WAIT;
                    end

                    T_WAIT: begin
                        // 等待 uart_tx 接收 start 并拉高 tx_busy
                        if (tx_busy)
                            state <= T_SEND;
                    end

                    T_SEND: begin
                        if (!tx_busy) begin
                            if (tx_idx == resp_len - 5'd1)
                                state <= R_H1;
                            else begin
                                tx_idx <= tx_idx + 5'd1;
                                state  <= T_LOAD;
                            end
                        end
                    end

                    default: state <= R_H1;
                endcase
            end
        end
    end

endmodule
