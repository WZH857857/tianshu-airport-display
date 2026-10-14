//=============================================================================
// dht11_reader.v - DHT11 单总线温湿度读取器
//
// 时序（50 MHz 时钟）：
//   1. 主机把 DATA 拉低 18 ms，然后释放；
//   2. DHT11 回应 80 us 低 + 80 us 高；
//   3. 每个数据位先低 50 us，再高 26~28 us（0）或 70 us（1）；
//   4. 40 位数据 = 湿度整数、湿度小数、温度整数、温度小数、校验和。
//
// DATA 是双向单总线，主机只在起始阶段拉低，其余时间释放为高阻，
// 由外部上拉电阻把总线拉高。DHT11 小数位在 DHT11 上固定为 0。
// 每 2 秒读取一次，读取失败或校验失败时 data_ok = 0。
//=============================================================================
module dht11_reader #(
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer START_LOW_US = 18_000,
    parameter integer PERIOD_US    = 2_000_000
) (
    input  wire       clk,
    input  wire       rst_n,
    inout  wire       dht_data,

    output reg  [7:0] humidity,
    output reg  [7:0] temperature,
    output reg        data_valid,   // 一次读取结束时的 1 拍脉冲
    output reg        data_ok,      // 最近一次读取校验是否成功
    output reg        busy          // 正在执行单总线时序
);

    localparam integer CYC_PER_US = CLK_HZ / 1_000_000;
    localparam [31:0] START_LOW_CYC = START_LOW_US * CYC_PER_US;
    localparam [31:0] PERIOD_CYC    = PERIOD_US * CYC_PER_US;
    localparam [31:0] TIMEOUT_CYC   = 200 * CYC_PER_US; // 单个阶段超时 200 us
    localparam [31:0] SAMPLE_CYC    = 40 * CYC_PER_US;  // 位高电平中间采样

    localparam [3:0] S_IDLE       = 4'd0;
    localparam [3:0] S_START_LOW  = 4'd1;
    localparam [3:0] S_RELEASE    = 4'd2;
    localparam [3:0] S_RESP_LOW   = 4'd3;
    localparam [3:0] S_RESP_HIGH  = 4'd4;
    localparam [3:0] S_BIT_LOW    = 4'd5;
    localparam [3:0] S_SAMPLE     = 4'd6;
    localparam [3:0] S_WAIT_FALL  = 4'd7;
    localparam [3:0] S_FINISH     = 4'd8;
    localparam [3:0] S_ERROR      = 4'd9;

    reg [3:0]  state;
    reg [31:0] timer;
    reg [5:0]  bit_idx;
    reg [39:0] shifter;
    reg        drive_low;

    // DATA 单总线：只有主机主动拉低时才驱动 0，其余时间释放为高阻。
    assign dht_data = drive_low ? 1'b0 : 1'bz;

    // 输入同步，避免异步单总线信号直接进入状态机。
    reg [2:0] dht_sync;
    reg       dht_in_d;
    wire      dht_in   = dht_sync[2];
    wire      dht_rise = dht_in & ~dht_in_d;
    wire      dht_fall = ~dht_in & dht_in_d;

    always @(posedge clk) begin
        dht_sync <= {dht_sync[1:0], dht_data};
        dht_in_d <= dht_in;
    end

    wire [9:0] checksum_sum = shifter[39:32] + shifter[31:24] +
                              shifter[23:16] + shifter[15:8];
    wire checksum_ok = (checksum_sum[7:0] == shifter[7:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            timer       <= 32'd0;
            bit_idx     <= 6'd0;
            shifter     <= 40'd0;
            drive_low   <= 1'b0;
            humidity    <= 8'd0;
            temperature <= 8'd0;
            data_valid  <= 1'b0;
            data_ok     <= 1'b0;
            busy        <= 1'b0;
        end
        else begin
            data_valid <= 1'b0;

            case (state)
                // 上电和每次读取后等待 2 秒，让 DHT11 完成一次采样。
                S_IDLE: begin
                    drive_low <= 1'b0;
                    busy      <= 1'b0;
                    if (timer >= PERIOD_CYC - 1'b1) begin
                        timer     <= 32'd0;
                        shifter   <= 40'd0;
                        bit_idx   <= 6'd0;
                        drive_low <= 1'b1;
                        busy      <= 1'b1;
                        state     <= S_START_LOW;
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 主机拉低 18 ms。
                S_START_LOW: begin
                    if (timer >= START_LOW_CYC - 1'b1) begin
                        timer     <= 32'd0;
                        drive_low <= 1'b0;
                        state     <= S_RELEASE;
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 释放后等待 DHT11 把总线拉低。
                S_RELEASE: begin
                    if (dht_in == 1'b0) begin
                        timer <= 32'd0;
                        state <= S_RESP_LOW;
                    end
                    else if (timer >= TIMEOUT_CYC) begin
                        state <= S_ERROR;
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 响应低电平 80 us，等待回到高。
                S_RESP_LOW: begin
                    if (dht_in == 1'b1) begin
                        timer <= 32'd0;
                        state <= S_RESP_HIGH;
                    end
                    else if (timer >= TIMEOUT_CYC) begin
                        state <= S_ERROR;
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 响应高电平 80 us，等待变低。
                S_RESP_HIGH: begin
                    if (dht_in == 1'b0) begin
                        timer <= 32'd0;
                        state <= S_BIT_LOW;
                    end
                    else if (timer >= TIMEOUT_CYC) begin
                        state <= S_ERROR;
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 每个数据位先有 50 us 低电平，等上升沿。
                S_BIT_LOW: begin
                    if (dht_rise) begin
                        timer <= 32'd0;
                        state <= S_SAMPLE;
                    end
                    else if (timer >= TIMEOUT_CYC) begin
                        state <= S_ERROR;
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 上升沿后 40 us 采样：仍为高是 1，已经变低是 0。
                S_SAMPLE: begin
                    if (timer >= SAMPLE_CYC - 1'b1) begin
                        timer   <= 32'd0;
                        shifter <= {shifter[38:0], dht_in};
                        if (bit_idx == 6'd39) begin
                            state <= S_FINISH;
                        end
                        // 位为 1 时总线仍是高，要等高电平结束；位为 0 时
                        // 总线早在采样点之前就回到低，直接等下一个上升沿。
                        else if (dht_in == 1'b1) begin
                            bit_idx <= bit_idx + 1'b1;
                            state   <= S_WAIT_FALL;
                        end
                        else begin
                            bit_idx <= bit_idx + 1'b1;
                            state   <= S_BIT_LOW;
                        end
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 等当前位的高电平结束，再读下一位。
                S_WAIT_FALL: begin
                    if (dht_fall) begin
                        timer <= 32'd0;
                        state <= S_BIT_LOW;
                    end
                    else if (timer >= TIMEOUT_CYC) begin
                        state <= S_ERROR;
                    end
                    else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 40 位读完，校验并锁存温湿度。
                S_FINISH: begin
                    busy       <= 1'b0;
                    data_valid <= 1'b1;
                    timer      <= 32'd0;
                    if (checksum_ok) begin
                        humidity    <= shifter[39:32];
                        temperature <= shifter[23:16];
                        data_ok     <= 1'b1;
                    end
                    else begin
                        data_ok <= 1'b0;
                    end
                    state <= S_IDLE;
                end

                // 超时或响应错误，保持上一次数据，标记本次失败。
                S_ERROR: begin
                    busy       <= 1'b0;
                    data_valid <= 1'b1;
                    data_ok    <= 1'b0;
                    timer      <= 32'd0;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
