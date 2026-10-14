module dual_cam_send(
    input                 clk,
    input                 rst_n,

    // ===== SDRAM 读取接口 =====
    output reg            read_req,
    input                 read_req_ack,
    output reg            read_en,
    input      [31:0]     read_data,

    // ===== 模板匹配结果输入 (实时嵌入到packet header) =====
    input                 match_result,        // 匹配结果 (1=匹配, 0=不匹配)
    input      [9:0]      hamming_distance,    // 汉明距离 (0-1023)

    // ===== 双目测距输入 (新增) =====
    input                 stereo_ranging_enable,  // 测距模式使能
    input      [7:0]      stereo_distance,        // 测距距离 (0-255cm)

    // ===== UDP 发送接口 =====
    input                 udp_tx_ready,
    input                 app_tx_ack,
    output reg            app_tx_data_request,
    output reg            app_tx_data_valid,
    output reg [7:0]      app_tx_data,
    output reg [15:0]     udp_data_length
);

// ============================================================================
// 参数定义（双模式支持：RGB / 测距）
// ============================================================================

localparam  IMG_HEADER      = 32'hAA0055FF;
localparam  IMG_WIDTH       = 32'd640;
localparam  IMG_HEIGHT      = 32'd480;

// 动态参数（根据模式切换）
wire [31:0] IMG_TOTAL       = stereo_ranging_enable ? 32'd614400 : 32'd1228800;  // 测距模式减半
wire [31:0] IMG_FRAMSIZE    = stereo_ranging_enable ? 32'd424 : 32'd848;         // 每包字节数
localparam  IMG_FRAMTOTAL   = 32'd1450;                                           // 总包数保持1450
localparam  IMG_HEADER_LEN  = 256;                                                // 32字节header

// ============================================================================
// 状态机定义
// ============================================================================

localparam  START_UDP       = 3'd0;
localparam  WAIT_FIFO_RDY   = 3'd1;
localparam  WAIT_UDP_DATA   = 3'd2;
localparam  WAIT_ACK        = 3'd3;
localparam  SEND_UDP_HEADER = 3'd4;
localparam  SEND_UDP_DATA   = 3'd5;
localparam  DELAY           = 3'd6;

reg [2:0]   STATE;

// ============================================================================
// 寄存器定义
// ============================================================================

reg [31:0]  IMG_FRAMSEQ;
reg [31:0]  IMG_PICSEQ;
reg [31:0]  IMG_OFFSET;

reg [8:0]   app_tx_header_cnt;
reg [11:0]  fifo_read_data_cnt; // 需要能计数到 848，11:0 (0-4095) 足够
reg [21:0]  delay_cnt;

// ============================================================================
// UDP 包头
// ============================================================================

// 匹配结果字段 (header[7]): bit 31=match_result, bits 9:0=hamming_distance
wire [31:0] MATCH_RESULT_HEADER = {match_result, 21'd0, hamming_distance};

// IMG_OFFSET字段 (header[4]): bits[31:24]=stereo_distance, bits[23:0]=IMG_OFFSET
wire [31:0] IMG_OFFSET_WITH_DISTANCE = {stereo_distance, IMG_OFFSET[23:0]};

wire [255:0] UDP_HEADER_32 = {
    MATCH_RESULT_HEADER,         // header[7] - 模板匹配结果
    IMG_FRAMSEQ,                 // header[6] - 包序号
    IMG_PICSEQ,                  // header[5] - 帧序号
    IMG_OFFSET_WITH_DISTANCE,    // header[4] - 双目测距[31:24] + 偏移[23:0]
    IMG_TOTAL,                   // header[3] - 总字节数（动态）
    IMG_HEIGHT,                  // header[2] - 图像高度
    IMG_WIDTH,                   // header[1] - 图像宽度
    IMG_HEADER                   // header[0] - Magic number 0xAA0055FF
};

// ============================================================================
// 像素数据处理
// ============================================================================

reg [31:0] read_data_buf;
reg [1:0]  byte_select_cnt; // 用于选择字节的计数器 (0, 1, 2, 3)
reg        read_en_d1;
reg        read_en_d2;

// ============================================================================
// 主状态机
// ============================================================================

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        STATE               <= START_UDP;
        app_tx_data_request <= 1'b0;
        app_tx_data_valid   <= 1'b0;
        app_tx_data         <= 8'd0;
        udp_data_length     <= 16'd880; // 默认RGB模式：包头32字节 + 数据848字节 = 880字节
        IMG_FRAMSEQ         <= 32'd0;
        IMG_PICSEQ          <= 32'd0;
        IMG_OFFSET          <= 32'd0;
        app_tx_header_cnt   <= 9'd0;
        fifo_read_data_cnt  <= 12'd0;
        delay_cnt           <= 22'd0;
        read_req            <= 1'b0;
        read_en             <= 1'b0;
        read_en_d1          <= 1'b0;
        read_en_d2          <= 1'b0;
        byte_select_cnt     <= 2'd0;
        read_data_buf       <= 32'd0;
    end
    else begin
        // 动态更新UDP包长度（根据模式）
        udp_data_length <= stereo_ranging_enable ? 16'd456 : 16'd880;  // 测距:32+424, RGB:32+848

        case(STATE)
            // ========== 状态 0：开始新帧 ==========
            START_UDP: begin
                app_tx_data_request <= 1'b0;
                app_tx_data_valid   <= 1'b0;
                fifo_read_data_cnt  <= 12'd0;
                IMG_FRAMSEQ         <= 32'd0;
                IMG_OFFSET          <= 32'd0;
                read_req            <= 1'b0;
                read_en             <= 1'b0;
                IMG_PICSEQ          <= IMG_PICSEQ + 1'd1;
                delay_cnt           <= 22'd0;
                byte_select_cnt     <= 2'd0;
                read_en_d1          <= 1'b0;
                read_en_d2          <= 1'b0;
                STATE               <= WAIT_FIFO_RDY;
            end

            // ========== 状态 1：等待 FIFO 准备好 ==========
            WAIT_FIFO_RDY: begin
                if(delay_cnt >= 2000) begin
                    delay_cnt <= 22'd0;
                    STATE     <= WAIT_UDP_DATA;
                end
                else begin
                    delay_cnt <= delay_cnt + 1'd1;
                    STATE     <= WAIT_FIFO_RDY;
                end

                if(delay_cnt == 10)
                    read_req <= 1'b1;
                else if(read_req_ack)
                    read_req <= 1'b0;
            end

            // ========== 状态 2：等待 UDP 准备好 ==========
            WAIT_UDP_DATA: begin
                if(udp_tx_ready) begin
                    app_tx_data_request <= 1'b1;
                    STATE               <= WAIT_ACK;
                end
                else begin
                    app_tx_data_request <= 1'b0;
                    STATE               <= WAIT_UDP_DATA;
                end
            end

            // ========== 状态 3：等待 UDP 应答 ==========
            WAIT_ACK: begin
                if(app_tx_ack) begin
                    app_tx_data_request <= 1'b0;
                    app_tx_header_cnt   <= 9'd8;
                    app_tx_data_valid   <= 1'b1;
                    app_tx_data         <= UDP_HEADER_32[7:0];
                    STATE               <= SEND_UDP_HEADER;
                end
                else begin
                    app_tx_data_request <= 1'b1;
                    STATE               <= WAIT_ACK;
                end
            end

            // ========== 状态 4：发送 UDP 包头 ==========
            SEND_UDP_HEADER: begin
                // read_en延迟打拍（每个周期都执行）
                read_en_d2 <= read_en_d1;
                read_en_d1 <= read_en;

                // 当read_en_d2有效时，锁存SDRAM数据
                if(read_en_d2) begin
                    read_data_buf <= read_data;
                end

                // 提前开始读取（在header发送期间预读第一个数据）
                // 确保在进入SEND_UDP_DATA前，read_data_buf已有有效数据
                if(app_tx_header_cnt == 9'd224) begin  // 提前8个周期开始读
                    read_en <= 1'b1;
                end
                else if(app_tx_header_cnt == 9'd232) begin
                    read_en <= 1'b0;
                end

                if(app_tx_header_cnt >= IMG_HEADER_LEN) begin
                    STATE              <= SEND_UDP_DATA;
                    app_tx_data_valid  <= 1'b1;
                    app_tx_data        <= UDP_HEADER_32[app_tx_header_cnt +: 8];
                    app_tx_header_cnt  <= 9'd0;
                    fifo_read_data_cnt <= 12'd0;
                    byte_select_cnt    <= 2'd0;
                end
                else begin
                    STATE             <= SEND_UDP_HEADER;
                    app_tx_data_valid <= 1'b1;
                    app_tx_data       <= UDP_HEADER_32[app_tx_header_cnt +: 8];
                    app_tx_header_cnt <= app_tx_header_cnt + 8;
                end
            end

            // ========== 状态 5：发送图像数据（根据模式选择格式）==========
            SEND_UDP_DATA: begin
                // read_en 延迟打拍
                read_en_d2 <= read_en_d1;
                read_en_d1 <= read_en;

                // 当read_en_d2有效时，锁存SDRAM数据
                if(read_en_d2) begin
                    read_data_buf <= read_data;
                end

                // 读使能控制（根据模式决定何时读取下一个32位数据）
                if (stereo_ranging_enable) begin
                    // 测距模式：2字节/像素，第2个字节时读下一个数据
                    if(byte_select_cnt == 2'd1) begin
                        read_en <= 1'b1;
                    end else begin
                        read_en <= 1'b0;
                    end
                end else begin
                    // RGB模式：4字节/像素，第4个字节时读下一个数据
                    if(byte_select_cnt == 2'd3) begin
                        read_en <= 1'b1;
                    end else begin
                        read_en <= 1'b0;
                    end
                end

                // 数据输出（根据模式选择）
                if (stereo_ranging_enable) begin
                    // 测距模式：2字节/像素（左二值化 + 右二值化）
                    // read_data_buf = {left[31:24], left[23:16], right[15:8], right[7:0]}
                    case(byte_select_cnt)
                        2'd0:    app_tx_data <= read_data_buf[31:24];  // 左目二值化
                        2'd1:    app_tx_data <= read_data_buf[15:8];   // 右目二值化
                        default: app_tx_data <= 8'h00;
                    endcase
                end else begin
                    // RGB模式：4字节/像素（左RGB565 + 右RGB565）
                    case(byte_select_cnt)
                        2'd0:    app_tx_data <= read_data_buf[31:24];  // 左目RGB565 [15:8]
                        2'd1:    app_tx_data <= read_data_buf[23:16];  // 左目RGB565 [7:0]
                        2'd2:    app_tx_data <= read_data_buf[15:8];   // 右目RGB565 [15:8]
                        2'd3:    app_tx_data <= read_data_buf[7:0];    // 右目RGB565 [7:0]
                        default: app_tx_data <= 8'h00;
                    endcase
                end

                // 包发送控制
                if(fifo_read_data_cnt >= (IMG_FRAMSIZE - 1)) begin
                    fifo_read_data_cnt <= 12'd0;
                    app_tx_data_valid  <= 1'b0;
                    read_en            <= 1'b0;
                    read_en_d1         <= 1'b0;
                    read_en_d2         <= 1'b0;
                    byte_select_cnt    <= 2'd0;
                    STATE              <= DELAY;
                end
                else begin
                    fifo_read_data_cnt <= fifo_read_data_cnt + 1'd1;
                    app_tx_data_valid  <= 1'b1;
                    STATE              <= SEND_UDP_DATA;

                    // 字节计数控制（根据模式）
                    if (stereo_ranging_enable) begin
                        if(byte_select_cnt >= 2'd1)
                            byte_select_cnt <= 2'd0;
                        else
                            byte_select_cnt <= byte_select_cnt + 1;
                    end else begin
                        if(byte_select_cnt >= 2'd3)
                            byte_select_cnt <= 2'd0;
                        else
                            byte_select_cnt <= byte_select_cnt + 1;
                    end
                end
            end

            // ========== 状态 6：延迟 ==========
            DELAY: begin
                if(delay_cnt >= 1500) begin
                    delay_cnt   <= 22'd0;
                    IMG_FRAMSEQ <= IMG_FRAMSEQ + 1'd1;
                    IMG_OFFSET  <= IMG_OFFSET + IMG_FRAMSIZE;

                    if(IMG_FRAMSEQ >= (IMG_FRAMTOTAL - 1)) // 检查是否已发送1450个包 (0-1449)
                        STATE <= START_UDP;
                    else
                        STATE <= WAIT_UDP_DATA;
                end
                else begin
                    delay_cnt <= delay_cnt + 1'd1;
                    STATE     <= DELAY;
                end
            end

            default: STATE <= START_UDP;
        endcase
    end
end

endmodule