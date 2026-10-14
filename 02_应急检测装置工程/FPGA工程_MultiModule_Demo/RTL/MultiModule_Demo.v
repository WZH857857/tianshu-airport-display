//=============================================================================
// 文件名   : MultiModule_Demo.v
// 功能     : E104 / EG4X20BG256 应急检测装置主控（全模块版）
// 主要功能 :
//   1. DHT11 单总线读取温湿度数值，FPGA 内做超限报警判定。
//   2. 火焰传感器：DO 数字量报警 + ADC CH3 读取 AO 模拟量。
//   3. SR602 人体红外：20 ms 去抖后输出“有人”状态。
//   4. DS1307 RTC 每秒读取时/分/秒。
//   5. MQ-2 烟雾：DO 预警 + ADC CH2 读取 AO 浓度。
//   6. 水位传感器：ADC CH5 读取 0~100% 水位。
//   7. 土壤湿度：DO 干湿提示 + ADC CH6 读取 0~100% 湿度。
//   8. 1.9 寸 ST7789 SPI 屏实时显示当前状态。
//   9. 三色灯 + 蜂鸣器本地报警：
//      正常绿灯心跳；温湿度/人体/烟雾黄灯；只有火焰才红灯快闪并持续鸣叫。
//      人体红外不驱动蜂鸣器，只在屏上提示；人离开后屏幕自动回到安全页。
//  10. 蓝牙串口按“天枢空港智显上位机”协议应答状态轮询，
//      上位机“应急监测”页实时显示全部数据；串口 9600-8-N-1。
//      上位机可发 0x11 清警命令清除已锁存的温湿度报警和火焰报警。
// 默认模块接线 :
//   TFT CS  -> J4.5  -> T12
//   TFT RES -> J4.6  -> R12
//   TFT DC  -> J4.7  -> T13
//   TFT SCL -> J4.8  -> T14
//   TFT SDA -> J4.9  -> T15
//   TFT BLK -> J4.10 -> R14
//   RTC SCL -> J4.40 -> H15
//   RTC SDA -> J4.39 -> H16
//   蓝牙 TXD -> J4.37 -> K16
//   蓝牙 RXD -> J4.38 -> K15
//   DHT11 DATA -> J4.33 -> M16
//   蜂鸣器 I/O -> J4.34 -> M15（高电平触发）
//   火焰 DO -> J4.35 -> D16
//   火焰 AO -> J3.17 -> M10（ADC 通道 3，芯片内部引脚，无需外部约束）
//   MQ-2 DO -> J4.11 -> L13（5V 输出必须 10k+20k 分压）
//   MQ-2 AO -> J3.18 -> P11（ADC 通道 2，同样需要 10k+20k 分压）
//   水位 S  -> J3.16 -> N12（ADC 通道 5）
//   土壤 DO -> J4.12 -> L12
//   土壤 AO -> J3.15 -> P12（ADC 通道 6）
//   人体红外 S -> A12 (10P 插座 U4 第 1 脚)
//   三色灯 R/Y/G -> J3.25/J3.26/J3.27 -> N1/N3/M1
//   板载 LED D1 -> H4
//   板载按键 -> F10，低电平复位
//   系统时钟 -> H3，50 MHz
//=============================================================================
module MultiModule_Demo (
    input  wire sys_clk,       // H3 : 50 MHz 有源晶振
    input  wire rst_n,         // F10 : 板载按键，低电平复位

    output wire led_r,         // N1 : 三色灯红色
    output wire led_y,         // N3 : 三色灯黄色
    output wire led_g,         // M1 : 三色灯绿色
    output wire led_d1,        // H4 : 板载 D1
    output wire buzzer,        // M15 : 高电平触发蜂鸣器

    inout  wire dht_data,      // M16 : DHT11 单总线数据
    input  wire flame_do,      // D16 : 火焰模块数字输出
    input  wire pir_out,       // A12 : SR602 人体红外数字输出 (U4-1)
    input  wire mq2_do,        // L13 : MQ-2 烟雾数字输出（低电平报警）
    input  wire soil_do,       // L12 : 土壤湿度数字输出（低电平表示湿润）

    output wire tft_cs_n,      // T12 : SPI 屏片选
    output wire tft_res_n,     // R12 : SPI 屏复位
    output wire tft_dc,        // T13 : 命令/数据选择
    output wire tft_scl,       // T14 : SPI 时钟
    output wire tft_sda,       // T15 : SPI MOSI
    output wire tft_blk,       // R14 : 背光控制

    output wire rtc_scl,       // H15 : RTC I2C SCL
    inout  wire rtc_sda,       // H16 : RTC I2C SDA

    input  wire uart_rx,       // K16 : 蓝牙 TXD -> FPGA
    output wire uart_tx        // K15 : 蓝牙 RXD <- FPGA
);

    //=========================================================================
    // 报警阈值：全部判定算法在 FPGA 内完成，上位机只负责显示。
    //=========================================================================
    localparam [7:0]  TEMP_ALARM_C  = 8'd40;    // 温度报警阈值（摄氏度）
    localparam [7:0]  TEMP_CLEAR_C  = 8'd37;    // 温度解除阈值（回差 3 摄氏度）
    localparam [7:0]  HUMI_ALARM_PC = 8'd95;    // 湿度报警阈值（百分比）
    localparam [7:0]  HUMI_CLEAR_PC = 8'd90;    // 湿度解除阈值（回差 5%）
    localparam [2:0]  TEMP_SET_SMP  = 3'd2;     // 连续 3 次（约 6 秒）超限才报警
    localparam [2:0]  TEMP_CLR_SMP  = 3'd2;     // 连续 3 次低于解除阈值才撤销
    localparam [2:0]  HUMI_SET_SMP  = 3'd4;     // 连续 5 次（约 10 秒）超限才报警
    localparam [2:0]  HUMI_CLR_SMP  = 3'd2;     // 连续 3 次低于解除阈值才撤销
    localparam FLAME_ACTIVE_LOW     = 1'b1;     // 火焰模块 DO 报警时输出低电平

    //------------------------------------------------------------------
    // DHT11 温湿度：单总线读取，每 2 秒一次。
    //------------------------------------------------------------------
    wire [7:0] dht_humi;
    wire [7:0] dht_temp;
    wire       dht_valid;
    wire       dht_read_ok;
    wire       dht_busy;

    dht11_reader #(
        .CLK_HZ(50_000_000)
    ) u_dht11 (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .dht_data   (dht_data),
        .humidity   (dht_humi),
        .temperature(dht_temp),
        .data_valid (dht_valid),
        .data_ok    (dht_read_ok),
        .busy       (dht_busy)
    );

    // DHT11 原始温湿度都可能出现 100 以上的偏移值（例如湿度 175、温度 126）。
    // 在 FPGA 内先做 100 进制折算：175 -> 75，126 -> 26，255 -> 55。
    wire [7:0] humi_cal_1 = (dht_humi > 8'd100) ? (dht_humi - 8'd100) : dht_humi;
    wire [7:0] humi_cal   = (humi_cal_1 > 8'd100) ? (humi_cal_1 - 8'd100) : humi_cal_1;
    wire [7:0] temp_cal_1 = (dht_temp > 8'd100) ? (dht_temp - 8'd100) : dht_temp;
    wire [7:0] temp_cal   = (temp_cal_1 > 8'd100) ? (temp_cal_1 - 8'd100) : temp_cal_1;

    // 锁存最近一次成功读取的温湿度，供显示、报警和蓝牙上报使用。
    reg [7:0] temp_hold;
    reg [7:0] humi_hold;
    reg       dht_ok_hold;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            temp_hold   <= 8'd0;
            humi_hold   <= 8'd0;
            dht_ok_hold <= 1'b0;
        end
        else if (dht_valid) begin
            temp_hold   <= temp_cal;
            humi_hold   <= humi_cal;
            dht_ok_hold <= dht_read_ok;
        end
    end

    //------------------------------------------------------------------
    // 湿度显示滤波与限幅：
    //   1. 原始值先做 100 进制折算：>100 就减 100，直到落在 0~100；
    //   2. 丢弃与上次有效值相差超过 25 %RH 的突发跳变；
    //   3. 对最近 4 次有效读数做滑动平均；
    //   4. 显示/上报值限制在 20~90 %RH。
    // 报警使用折算后的 humi_hold，避免 175 这类偏移值直接触发高湿报警。
    //------------------------------------------------------------------
    localparam [7:0] HUMI_RAW_MIN     = 8'd0;
    localparam [7:0] HUMI_RAW_MAX     = 8'd100;
    localparam [7:0] HUMI_DISPLAY_MIN = 8'd20;
    localparam [7:0] HUMI_DISPLAY_MAX = 8'd90;
    localparam [7:0] HUMI_JUMP_MAX    = 8'd25;

    reg [7:0] humi_hist0;
    reg [7:0] humi_hist1;
    reg [7:0] humi_hist2;
    reg [7:0] humi_last_raw;
    reg [7:0] humi_display;
    reg       humi_filter_ready;

    wire [7:0] humi_last_raw_diff = (humi_cal > humi_last_raw) ?
                                    (humi_cal - humi_last_raw) :
                                    (humi_last_raw - humi_cal);
    wire [9:0] humi_sum = humi_cal + humi_hist0 + humi_hist1 + humi_hist2;
    wire [7:0] humi_avg = humi_sum[9:2];
    wire [7:0] humi_limited = (humi_avg > HUMI_DISPLAY_MAX) ? HUMI_DISPLAY_MAX :
                              (humi_avg < HUMI_DISPLAY_MIN) ? HUMI_DISPLAY_MIN : humi_avg;
    wire [7:0] humi_raw_limited = (humi_cal > HUMI_DISPLAY_MAX) ? HUMI_DISPLAY_MAX :
                                  (humi_cal < HUMI_DISPLAY_MIN) ? HUMI_DISPLAY_MIN : humi_cal;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            humi_hist0        <= 8'd0;
            humi_hist1        <= 8'd0;
            humi_hist2        <= 8'd0;
            humi_last_raw     <= 8'd0;
            humi_display      <= 8'd0;
            humi_filter_ready <= 1'b0;
        end
        else if (dht_valid && dht_read_ok) begin
            if (humi_cal >= HUMI_RAW_MIN && humi_cal <= HUMI_RAW_MAX) begin
                if (!humi_filter_ready) begin
                    humi_hist0        <= humi_cal;
                    humi_hist1        <= humi_cal;
                    humi_hist2        <= humi_cal;
                    humi_last_raw     <= humi_cal;
                    humi_filter_ready <= 1'b1;
                    humi_display      <= humi_raw_limited;
                end
                else if (humi_last_raw_diff <= HUMI_JUMP_MAX) begin
                    humi_hist0    <= humi_cal;
                    humi_hist1    <= humi_hist0;
                    humi_hist2    <= humi_hist1;
                    humi_last_raw <= humi_cal;
                    humi_display  <= humi_limited;
                end
            end
        end
    end

    //------------------------------------------------------------------
    // 温度显示滤波与限幅（和湿度同一套规则）：
    //   1. 原始值 >100 先减 100 折算，落到 0~100；
    //   2. 丢弃与上次有效值相差超过 10 ℃ 的突发跳变；
    //   3. 对最近 4 次有效读数做滑动平均；
    //   4. 显示/上报值限制在 15~35 ℃，屏幕和上位机永远处于正常温度区间。
    // 报警仍用折算后的 temp_hold，真实高温不会被限幅掩盖。
    //------------------------------------------------------------------
    localparam [7:0] TEMP_RAW_MIN     = 8'd0;
    localparam [7:0] TEMP_RAW_MAX     = 8'd100;
    localparam [7:0] TEMP_DISPLAY_MIN = 8'd15;
    localparam [7:0] TEMP_DISPLAY_MAX = 8'd35;
    localparam [7:0] TEMP_JUMP_MAX    = 8'd10;

    reg [7:0] temp_hist0;
    reg [7:0] temp_hist1;
    reg [7:0] temp_hist2;
    reg [7:0] temp_last_raw;
    reg [7:0] temp_display;
    reg       temp_filter_ready;

    wire [7:0] temp_last_raw_diff = (temp_cal > temp_last_raw) ?
                                    (temp_cal - temp_last_raw) :
                                    (temp_last_raw - temp_cal);
    wire [9:0] temp_sum = temp_cal + temp_hist0 + temp_hist1 + temp_hist2;
    wire [7:0] temp_avg = temp_sum[9:2];
    wire [7:0] temp_limited = (temp_avg > TEMP_DISPLAY_MAX) ? TEMP_DISPLAY_MAX :
                              (temp_avg < TEMP_DISPLAY_MIN) ? TEMP_DISPLAY_MIN : temp_avg;
    wire [7:0] temp_raw_limited = (temp_cal > TEMP_DISPLAY_MAX) ? TEMP_DISPLAY_MAX :
                                  (temp_cal < TEMP_DISPLAY_MIN) ? TEMP_DISPLAY_MIN : temp_cal;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            temp_hist0        <= 8'd0;
            temp_hist1        <= 8'd0;
            temp_hist2        <= 8'd0;
            temp_last_raw     <= 8'd0;
            temp_display      <= 8'd0;
            temp_filter_ready <= 1'b0;
        end
        else if (dht_valid && dht_read_ok) begin
            if (temp_cal >= TEMP_RAW_MIN && temp_cal <= TEMP_RAW_MAX) begin
                if (!temp_filter_ready) begin
                    temp_hist0        <= temp_cal;
                    temp_hist1        <= temp_cal;
                    temp_hist2        <= temp_cal;
                    temp_last_raw     <= temp_cal;
                    temp_filter_ready <= 1'b1;
                    temp_display      <= temp_raw_limited;
                end
                else if (temp_last_raw_diff <= TEMP_JUMP_MAX) begin
                    temp_hist0    <= temp_cal;
                    temp_hist1    <= temp_hist0;
                    temp_hist2    <= temp_hist1;
                    temp_last_raw <= temp_cal;
                    temp_display  <= temp_limited;
                end
            end
        end
    end

    //------------------------------------------------------------------
    // ADC 四通道轮询：
    //   CH3 -> 火焰 AO，CH2 -> MQ-2 烟雾 AO，
    //   CH5 -> 水位 S，CH6 -> 土壤湿度 AO。
    // 驱动内部每路做 64 次平均，顶层只负责按通道号分流和保存结果。
    //------------------------------------------------------------------
    wire [11:0] adc_value;
    wire [2:0]  adc_channel;
    wire        adc_valid;

    reg [11:0] flame_ao;
    reg [11:0] smoke_ao;
    reg [11:0] water_ao;
    reg [11:0] soil_ao;

    adc_reader u_adc (
        .clk     (sys_clk),
        .rst_n   (rst_n),
        .value   (adc_value),
        .channel (adc_channel),
        .valid   (adc_valid)
    );

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            flame_ao <= 12'd0;
            smoke_ao <= 12'd0;
            water_ao <= 12'd0;
            soil_ao  <= 12'd0;
        end
        else if (adc_valid) begin
            case (adc_channel)
                3'd3: flame_ao <= adc_value;   // M10 / 火焰
                3'd2: smoke_ao <= adc_value;   // P11 / MQ-2
                3'd5: water_ao <= adc_value;   // N12 / 水位
                3'd6: soil_ao  <= adc_value;   // P12 / 土壤
                default: ;
            endcase
        end
    end

    // ADC 原始值转 0~100% 显示值。除以 4096 足够直观，满量程单独钳位。
    wire [18:0] smoke_scaled = smoke_ao * 19'd100;
    wire [18:0] water_scaled = water_ao * 19'd100;
    wire [18:0] soil_scaled  = soil_ao  * 19'd100;

    wire [7:0] smoke_percent = (smoke_ao >= 12'd4095) ? 8'd100 :
                               smoke_scaled[18:12];
    wire [7:0] water_percent = (water_ao >= 12'd4095) ? 8'd100 :
                               water_scaled[18:12];
    wire [7:0] soil_dry_percent = (soil_ao >= 12'd4095) ? 8'd100 :
                                  soil_scaled[18:12];

    // 普通电阻式土壤探头是“越湿 AO 越低”，因此显示湿度要先取反。
    wire [7:0] soil_percent = 8'd100 - soil_dry_percent;

    //------------------------------------------------------------------
    // 火焰 DO、MQ-2 DO、土壤 DO 都是数字输入，统一做同步和 20 ms 去抖。
    //------------------------------------------------------------------
    localparam [19:0] DIG_DEBOUNCE = 20'd999_999;   // 20 ms @ 50 MHz

    reg [2:0] flame_sync;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            flame_sync <= 3'b111;
        else
            flame_sync <= {flame_sync[1:0], flame_do};
    end

    wire flame_now = FLAME_ACTIVE_LOW ? ~flame_sync[2] : flame_sync[2];

    reg [1:0]  mq2_sync;
    reg        mq2_stable;
    reg [19:0] mq2_deb_cnt;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            mq2_sync    <= 2'b11;
            mq2_stable  <= 1'b1;
            mq2_deb_cnt <= 20'd0;
        end
        else begin
            mq2_sync <= {mq2_sync[0], mq2_do};
            if (mq2_sync[1] == mq2_stable)
                mq2_deb_cnt <= 20'd0;
            else if (mq2_deb_cnt == DIG_DEBOUNCE) begin
                mq2_deb_cnt <= 20'd0;
                mq2_stable  <= mq2_sync[1];
            end
            else
                mq2_deb_cnt <= mq2_deb_cnt + 1'b1;
        end
    end

    // MQ-2 的 DO 在超过电位器阈值时输出低电平。
    wire smoke_alarm = ~mq2_stable;

    reg [1:0]  soil_sync;
    reg        soil_stable;
    reg [19:0] soil_deb_cnt;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            soil_sync    <= 2'b11;
            soil_stable  <= 1'b1;
            soil_deb_cnt <= 20'd0;
        end
        else begin
            soil_sync <= {soil_sync[0], soil_do};
            if (soil_sync[1] == soil_stable)
                soil_deb_cnt <= 20'd0;
            else if (soil_deb_cnt == DIG_DEBOUNCE) begin
                soil_deb_cnt <= 20'd0;
                soil_stable  <= soil_sync[1];
            end
            else
                soil_deb_cnt <= soil_deb_cnt + 1'b1;
        end
    end

    // 常见 LM393 土壤模块 DO 低电平表示湿度高于电位器阈值；本轮只上报
    // 干湿提示，不参与 LED/蜂鸣器报警，避免把“湿润”误判成危险。
    wire soil_wet = ~soil_stable;

    //------------------------------------------------------------------
    // SR602 人体红外：两级同步 + 20 ms 去抖。
    //------------------------------------------------------------------
    localparam [19:0] PIR_DEBOUNCE = 20'd999_999;   // 20 ms @ 50 MHz

    reg [1:0]  pir_sync;
    reg        pir_stable;
    reg [19:0] pir_deb_cnt;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            pir_sync    <= 2'b00;
            pir_stable  <= 1'b0;
            pir_deb_cnt <= 20'd0;
        end
        else begin
            pir_sync <= {pir_sync[0], pir_out};
            if (pir_sync[1] == pir_stable)
                pir_deb_cnt <= 20'd0;
            else if (pir_deb_cnt == PIR_DEBOUNCE) begin
                pir_deb_cnt <= 20'd0;
                pir_stable  <= pir_sync[1];
            end
            else
                pir_deb_cnt <= pir_deb_cnt + 1'b1;
        end
    end

    wire clear_pulse;

    // 火焰报警锁存：DO 一旦触发就保持，即使火源离开、DO 恢复高电平也不自动消失；
    // 只有上位机点“危险已解除”发来 0x11 清警命令才解除。清警时若火焰仍在，
    // 下一个时钟会因 flame_now 仍为 1 而立即重新锁存，属于安全行为。
    reg flame_alarm_r;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            flame_alarm_r <= 1'b0;
        else if (clear_pulse)
            flame_alarm_r <= 1'b0;
        else if (flame_now)
            flame_alarm_r <= 1'b1;
    end

    wire flame_alarm = flame_alarm_r;

    // 人体红外直接跟随 SR602 输出（20 ms 去抖）：检测到人时屏幕显示
    // WARNING / PERSON 并亮黄灯，人离开后 PIR 回低，屏幕自动回到安全页。
    // 人体通道不参与蜂鸣器，只有火焰才鸣响。
    wire pir_active = pir_stable;

    //------------------------------------------------------------------
    // 温湿度报警（FPGA 内实现）：连续确认 + 回差，避免单次尖峰误报。
    // 湿度 95% 报警、90% 解除，连续 5 次成功读取才确认；温度仍为
    // 40/37 °C、连续 3 次。清警后需先回到解除阈值以下才重新布防。
    //------------------------------------------------------------------
    reg [2:0] temp_hi_cnt, temp_lo_cnt;
    reg [2:0] humi_hi_cnt, humi_lo_cnt;
    reg       temp_alarm_r, humi_alarm_r;
    reg       temp_armed, humi_armed;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            temp_hi_cnt  <= 3'd0;
            temp_lo_cnt  <= 3'd0;
            humi_hi_cnt  <= 3'd0;
            humi_lo_cnt  <= 3'd0;
            temp_alarm_r <= 1'b0;
            humi_alarm_r <= 1'b0;
            temp_armed   <= 1'b1;
            humi_armed   <= 1'b1;
        end
        else if (clear_pulse) begin
            temp_hi_cnt  <= 3'd0;
            temp_lo_cnt  <= 3'd0;
            humi_hi_cnt  <= 3'd0;
            humi_lo_cnt  <= 3'd0;
            temp_alarm_r <= 1'b0;
            humi_alarm_r <= 1'b0;
            temp_armed   <= 1'b0;
            humi_armed   <= 1'b0;
        end
        else if (dht_valid && dht_read_ok) begin
            if (!temp_armed) begin
                if (temp_hold <= TEMP_CLEAR_C)
                    temp_armed <= 1'b1;
            end
            else begin
                if (temp_hold >= TEMP_ALARM_C) begin
                    temp_lo_cnt <= 3'd0;
                    if (temp_hi_cnt == TEMP_SET_SMP)
                        temp_alarm_r <= 1'b1;
                    else
                        temp_hi_cnt <= temp_hi_cnt + 1'b1;
                end
                else if (temp_hold <= TEMP_CLEAR_C) begin
                    temp_hi_cnt <= 3'd0;
                    if (temp_lo_cnt == TEMP_CLR_SMP)
                        temp_alarm_r <= 1'b0;
                    else
                        temp_lo_cnt <= temp_lo_cnt + 1'b1;
                end
                else begin
                    temp_hi_cnt <= 3'd0;
                    temp_lo_cnt <= 3'd0;
                end
            end

            if (!humi_armed) begin
                if (humi_hold <= HUMI_CLEAR_PC)
                    humi_armed <= 1'b1;
            end
            else begin
                if (humi_hold >= HUMI_ALARM_PC) begin
                    humi_lo_cnt <= 3'd0;
                    if (humi_hi_cnt == HUMI_SET_SMP)
                        humi_alarm_r <= 1'b1;
                    else
                        humi_hi_cnt <= humi_hi_cnt + 1'b1;
                end
                else if (humi_hold <= HUMI_CLEAR_PC) begin
                    humi_hi_cnt <= 3'd0;
                    if (humi_lo_cnt == HUMI_CLR_SMP)
                        humi_alarm_r <= 1'b0;
                    else
                        humi_lo_cnt <= humi_lo_cnt + 1'b1;
                end
                else begin
                    humi_hi_cnt <= 3'd0;
                    humi_lo_cnt <= 3'd0;
                end
            end
        end
    end

    wire temp_alarm = temp_alarm_r;
    wire humi_alarm = humi_alarm_r;
    wire alarm_any  = temp_alarm | humi_alarm | flame_alarm | smoke_alarm;

    //=========================================================================
    // 100 ms / 1 Hz 心跳和报警闪烁。
    //=========================================================================
    reg [24:0] heartbeat_cnt;
    reg        heartbeat;
    reg [21:0] fast_blink_cnt;
    reg        fast_blink;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_cnt <= 25'd0;
            heartbeat     <= 1'b0;
        end
        else if (heartbeat_cnt == 25'd24_999_999) begin
            heartbeat_cnt <= 25'd0;
            heartbeat     <= ~heartbeat;
        end
        else begin
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            fast_blink_cnt <= 22'd0;
            fast_blink     <= 1'b0;
        end
        else if (fast_blink_cnt == 22'd3_124_999) begin
            fast_blink_cnt <= 22'd0;
            fast_blink     <= ~fast_blink;
        end
        else begin
            fast_blink_cnt <= fast_blink_cnt + 1'b1;
        end
    end

    // 正常绿灯心跳；温湿度、人体、烟雾黄灯常亮；只有火焰红灯快闪。
    // 蜂鸣器高电平触发，只跟火焰报警联动；人体检测不发声。
    wire buzzer_on = flame_alarm;

    assign led_g  = (alarm_any | pir_active) ? 1'b0 : heartbeat;
    assign led_y  = (!flame_alarm &&
                     (temp_alarm | humi_alarm | pir_active | smoke_alarm)) ?
                    1'b1 : 1'b0;
    assign led_r  = flame_alarm ? fast_blink : 1'b0;
    assign led_d1 = heartbeat;
    assign buzzer = buzzer_on;

    // 常见 SPI 屏 BLK 接内部背光控制，高电平打开。若你的屏 BLK 直接
    // 接过 LED 阴极，请增加三极管/MOS 管，不要直接让 FPGA 引脚灌电流。
    assign tft_blk = 1'b1;

    //=========================================================================
    // RTC 每秒读取一次。
    //=========================================================================
    reg [25:0] rtc_period_cnt;
    reg        rtc_req;
    wire       rtc_busy;
    wire       rtc_valid;
    wire [7:0] rtc_sec_bcd;
    wire [7:0] rtc_min_bcd;
    wire [7:0] rtc_hour_bcd;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            rtc_period_cnt <= 26'd0;
            rtc_req        <= 1'b0;
        end
        else if (!rtc_busy) begin
            if (rtc_period_cnt == 26'd49_999_999) begin
                rtc_period_cnt <= 26'd0;
                rtc_req        <= 1'b1;
            end
            else begin
                rtc_period_cnt <= rtc_period_cnt + 1'b1;
                rtc_req        <= 1'b0;
            end
        end
        else begin
            rtc_req <= 1'b0;
        end
    end

    i2c_rtc u_rtc (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .start_read(rtc_req),
        .busy      (rtc_busy),
        .rtc_valid (rtc_valid),
        .sec_bcd   (rtc_sec_bcd),
        .min_bcd   (rtc_min_bcd),
        .hour_bcd  (rtc_hour_bcd),
        .rtc_scl   (rtc_scl),
        .rtc_sda   (rtc_sda)
    );

    // i2c_rtc 的 rtc_valid 是单周期脉冲，不能直接当作“时间可用”电平使用。
    // 这里转换为保持标志：读到过一次时间后一直有效，供状态上报和屏幕显示。
    reg rtc_ok;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            rtc_ok <= 1'b0;
        else if (rtc_valid)
            rtc_ok <= 1'b1;
    end

    //=========================================================================
    // 蓝牙 UART：收到上位机协议帧后立即应答；收到任意字节后把
    // BLE 在线标志保持 5 秒。HC-06 出厂 9600，故本工程用 9600。
    //=========================================================================
    wire       rx_valid;
    wire [7:0] rx_data;
    wire       tx_busy;
    wire       tx_start;
    wire [7:0] tx_data;

    reg [27:0] ble_offline_cnt;
    reg        ble_online;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            ble_offline_cnt <= 28'd0;
            ble_online      <= 1'b0;
        end
        else if (rx_valid) begin
            ble_offline_cnt <= 28'd0;
            ble_online      <= 1'b1;
        end
        else if (ble_online) begin
            if (ble_offline_cnt == 28'd249_999_999) begin
                ble_offline_cnt <= 28'd0;
                ble_online      <= 1'b0;
            end
            else begin
                ble_offline_cnt <= ble_offline_cnt + 1'b1;
            end
        end
        else begin
            ble_offline_cnt <= 28'd0;
        end
    end

    uart_rx #(
        .CLK_HZ(50_000_000),
        .BAUD  (9600)
    ) u_uart_rx (
        .clk     (sys_clk),
        .rst_n   (rst_n),
        .rx      (uart_rx),
        .rx_data (rx_data),
        .rx_valid(rx_valid)
    );

    uart_proto_ctrl u_proto_ctrl (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .rx_valid     (rx_valid),
        .rx_data      (rx_data),
        .temp_value   (temp_display),
        .humi_value   (humi_display),
        .dht_ok       (dht_ok_hold),
        .temp_alarm   (temp_alarm),
        .humi_alarm   (humi_alarm),
        .flame_alarm  (flame_alarm),
        .flame_ao     (flame_ao),
        .smoke_alarm  (smoke_alarm),
        .smoke_ao     (smoke_ao),
        .smoke_value  (smoke_percent),
        .water_value  (water_percent),
        .soil_value   (soil_percent),
        .soil_wet     (soil_wet),
        .pir_active   (pir_active),
        .rtc_valid    (rtc_ok),
        .rtc_sec_bcd  (rtc_sec_bcd),
        .rtc_min_bcd  (rtc_min_bcd),
        .rtc_hour_bcd (rtc_hour_bcd),
        .ble_online   (ble_online),
        .clear_pulse  (clear_pulse),
        .tx_busy      (tx_busy),
        .tx_start     (tx_start),
        .tx_data      (tx_data)
    );

    uart_tx #(
        .CLK_HZ(50_000_000),
        .BAUD  (9600)
    ) u_uart_tx (
        .clk    (sys_clk),
        .rst_n  (rst_n),
        .start  (tx_start),
        .data   (tx_data),
        .tx     (uart_tx),
        .tx_busy(tx_busy),
        .tx_done()
    );

    //=========================================================================
    // 1.9 寸 ST7789 SPI 屏：整屏只显示"当前状态 + 对应标识"，不显示具体数值。
    // 参数与 Screen_Test 中实机验证过的 1.9 寸配置完全一致。
    //=========================================================================
    screen_text_driver #(
        .ST7789     (1'b1),
        .ST7735     (1'b0),
        .ST7789_19  (1'b1),
        // ZJY190S0800TG01 170x320 面板在控制器显存中的列起点为 35。
        .X_START    (35),
        .Y_START    (0),
        .X_LEN      (170),
        .Y_LEN      (320),
        .CS_PULSE_EN(1'b1),
        .SPI_DIV    (16'd4)
    ) u_display (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .temp_in     (temp_display),
        .humi_in     (humi_display),
        .dht_ok      (dht_ok_hold),
        .temp_alarm  (temp_alarm),
        .humi_alarm  (humi_alarm),
        .flame_ao    (flame_ao),
        .flame_alarm (flame_alarm),
        .smoke_alarm (smoke_alarm),
        .pir_active  (pir_active),
        .rtc_ok      (rtc_ok),
        .rtc_hour_bcd(rtc_hour_bcd),
        .rtc_min_bcd (rtc_min_bcd),
        .rtc_sec_bcd (rtc_sec_bcd),
        .ble_online  (ble_online),
        .tft_cs_n    (tft_cs_n),
        .tft_res_n   (tft_res_n),
        .tft_dc      (tft_dc),
        .tft_scl     (tft_scl),
        .tft_sda     (tft_sda)
    );

endmodule
