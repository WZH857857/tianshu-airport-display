//=============================================================================
// 文件名   : adc_reader.v
// 功能     : 用 EG4X20B 内嵌 SAR ADC 轮询四路模拟输入
//
// 硬件通道：
//   ADC 通道 3 = M10 = J3.17 -> 火焰 AO
//   ADC 通道 2 = P11 = J3.18 -> MQ-2 烟雾 AO
//   ADC 通道 5 = N12 = J3.16 -> 水位传感器 S
//   ADC 通道 6 = P12 = J3.15 -> 土壤湿度 AO
//
//   EG_PHY_ADC 原语直接占用芯片模拟脚，不需要（也不能）在 UCF 里写
//   这些模拟脚的 LOCATION。板上 ADC_VDDA / ADC_VREF 都是 3.3V，因此
//   满量程按 3.3V 理解；手册有效区间约 0.33V ~ 2.97V，超出会削顶。
//
// 采样时序（依据 TN304 和 TD 自带仿真模型 eg_phy_adc.v）：
//   * ADC 工作时钟最高 16MHz，这里用 50MHz / 4 = 12.5MHz；
//   * 一次转换 16 个 ADC 时钟（1.28us），EOC 在转换结束时高一个周期；
//   * 手册要求 SOC 间隔小于 10us，这里约 1.5us 启动一次；
//   * 切换通道后先丢弃 SETTLE_CONV 次转换，避免通道间串扰。
//
// 数据通路：
//   EOC 经两级同步后在 50MHz 域检上升沿。每路连续累加 64 次后取平均，
//   输出一个 valid 脉冲，同时给出 channel 编号和 value 原始值。四路轮询
//   一轮约 400us，足够温湿度之外的所有模拟量实时刷新。
//=============================================================================
module adc_reader #(
    parameter [2:0]  CH_FLAME     = 3'd3,  // M10 / J3.17
    parameter [2:0]  CH_SMOKE     = 3'd2,  // P11 / J3.18
    parameter [2:0]  CH_WATER     = 3'd5,  // N12 / J3.16
    parameter [2:0]  CH_SOIL      = 3'd6,  // P12 / J3.15
    parameter [2:0]  SETTLE_CONV  = 3'd3,  // 切通道后丢弃的转换次数
    parameter [15:0] SOC_HOLD     = 16'd8, // SOC 高电平保持 8 * 20ns = 160ns
    parameter [15:0] EOC_TIMEOUT  = 16'd600 // 约 12us 等不到 EOC 就重发 SOC
) (
    input  wire        clk,       // 50MHz 系统时钟（H3）
    input  wire        rst_n,     // 低电平复位（F10）
    output reg  [11:0] value,     // 64 次平均后的 ADC 原始值 0..4095
    output reg  [2:0]  channel,   // 本次 value 对应的 ADC 通道号
    output reg         valid      // 每完成一路平均后输出一个 50MHz 周期高脉冲
);

    //-------------------------------------------------------------------------
    // 12.5MHz ADC 工作时钟：clk_div[1] 每 2 个系统时钟翻转一次，
    // 占空比 50%，高/低各 40ns，低于手册 16MHz 上限。
    //-------------------------------------------------------------------------
    reg [1:0] clk_div;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            clk_div <= 2'd0;
        else
            clk_div <= clk_div + 1'b1;
    end

    wire adc_clk = clk_div[1];

    //-------------------------------------------------------------------------
    // EG_PHY_ADC 原语：启用本次用到的四路，其余关闭。
    // VREF 使用板上 3.3V 参考。
    //-------------------------------------------------------------------------
    reg         adc_soc;
    reg  [2:0]  adc_s;
    wire        adc_eoc;
    wire [11:0] adc_dout;

    EG_PHY_ADC #(
        .CH0 ("DISABLE"),
        .CH1 ("DISABLE"),
        .CH2 ("ENABLE"),
        .CH3 ("ENABLE"),
        .CH4 ("DISABLE"),
        .CH5 ("ENABLE"),
        .CH6 ("ENABLE"),
        .CH7 ("DISABLE"),
        .VREF("ENABLE")
    ) u_eg_phy_adc (
        .clk (adc_clk),
        .pd  (1'b0),
        .s   (adc_s),
        .soc (adc_soc),
        .eoc (adc_eoc),
        .dout(adc_dout)
    );

    // EOC 相对 50MHz 域是异步信号，先过两级同步器再检上升沿。
    reg [1:0] eoc_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            eoc_sync <= 2'b00;
        else
            eoc_sync <= {eoc_sync[0], adc_eoc};
    end

    wire eoc_rise = eoc_sync[0] & ~eoc_sync[1];

    //-------------------------------------------------------------------------
    // 采样状态机（50MHz 域）：
    //   S_SOC  : SOC 拉高保持 SOC_HOLD 拍；
    //   S_WAIT : 等 EOC 上升沿，取数累加；超时就重发一次。
    // 每路 64 个样本取平均，然后切到下一路。
    //-------------------------------------------------------------------------
    localparam S_SOC  = 1'b0;
    localparam S_WAIT = 1'b1;

    reg        fsm;
    reg [15:0] delay_cnt;
    reg [17:0] acc;          // 64 * 4095 = 262080，需要 18 bit
    reg [5:0]  avg_cnt;
    reg [1:0]  seq;          // 0=火焰 1=烟雾 2=水位 3=土壤
    reg [2:0]  settle_left;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm         <= S_SOC;
            adc_soc     <= 1'b1;
            adc_s       <= CH_FLAME;
            delay_cnt   <= 16'd0;
            acc         <= 18'd0;
            avg_cnt     <= 6'd0;
            seq         <= 2'd0;
            settle_left <= SETTLE_CONV;
            value       <= 12'd0;
            channel     <= CH_FLAME;
            valid       <= 1'b0;
        end
        else begin
            valid <= 1'b0;

            case (fsm)
                S_SOC: begin
                    adc_soc <= 1'b1;
                    if (delay_cnt == SOC_HOLD - 1'b1) begin
                        delay_cnt <= 16'd0;
                        adc_soc   <= 1'b0;
                        fsm       <= S_WAIT;
                    end
                    else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                S_WAIT: begin
                    adc_soc <= 1'b0;
                    if (eoc_rise) begin
                        if (settle_left != 3'd0) begin
                            // 通道刚切换：丢弃前几次转换，等待采样保持稳定。
                            settle_left <= settle_left - 1'b1;
                            acc         <= 18'd0;
                            avg_cnt     <= 6'd0;
                        end
                        else if (avg_cnt == 6'd63) begin
                            // 第 64 个样本和前面 63 个一起平均后输出。
                            value   <= (acc + adc_dout) >> 6;
                            channel <= adc_s;
                            valid   <= 1'b1;
                            acc     <= 18'd0;
                            avg_cnt <= 6'd0;
                            settle_left <= SETTLE_CONV;

                            // 下一轮换通道。
                            case (seq)
                                2'd0: begin adc_s <= CH_SMOKE; seq <= 2'd1; end
                                2'd1: begin adc_s <= CH_WATER; seq <= 2'd2; end
                                2'd2: begin adc_s <= CH_SOIL;  seq <= 2'd3; end
                                default: begin adc_s <= CH_FLAME; seq <= 2'd0; end
                            endcase
                        end
                        else begin
                            acc     <= acc + adc_dout;
                            avg_cnt <= avg_cnt + 1'b1;
                        end

                        delay_cnt <= 16'd0;
                        adc_soc   <= 1'b1;
                        fsm       <= S_SOC;
                    end
                    else if (delay_cnt == EOC_TIMEOUT) begin
                        // ADC 没回应：重新拉一次 SOC，避免卡死。
                        delay_cnt <= 16'd0;
                        adc_soc   <= 1'b1;
                        fsm       <= S_SOC;
                    end
                    else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                default: begin
                    fsm     <= S_SOC;
                    adc_soc <= 1'b1;
                end
            endcase
        end
    end

endmodule
