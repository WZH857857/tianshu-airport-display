
  module udp_ov5640_send(
        input               clk,
        input               rst_n,

        // ===== SDRAM 读取接口 =====
        output reg          read_req,
        input               read_req_ack,
        output reg          read_en,
        input [31:0]        read_data,

        // ===== UDP 发送接口 =====
        input               udp_tx_ready,
        input               app_tx_ack,
        output reg          app_tx_data_request,
        output reg          app_tx_data_valid,
        output reg [7:0]    app_tx_data,
        output reg [15:0]   udp_data_length
  );

  // ============================================================================
  // 参数定义
  // ============================================================================

  localparam  IMG_HEADER      = 32'hAA0055FF;
  localparam  IMG_WIDTH       = 32'd640;
  localparam  IMG_HEIGHT      = 32'd480;
  localparam  IMG_TOTAL       = IMG_WIDTH * IMG_HEIGHT * 3;
  localparam  IMG_FRAMSIZE    = 32'd636;
  localparam  IMG_FRAMTOTAL   = 32'd1450;
  localparam  IMG_HEADER_LEN  = 256;

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
  reg [11:0]  fifo_read_data_cnt;
  reg [21:0]  delay_cnt;

  // ============================================================================
  // UDP 包头
  // ============================================================================

  wire [255:0] UDP_HEADER_32 = {
        IMG_FRAMSIZE,
        IMG_FRAMSEQ,
        IMG_PICSEQ,
        IMG_OFFSET,
        IMG_TOTAL,
        IMG_HEIGHT,
        IMG_WIDTH,
        IMG_HEADER
  };

  // ============================================================================
  // 像素数据处理
  // ============================================================================

  reg [31:0] read_data_buf;
  reg [1:0]  byte_select_cnt; // 修改：用于选择R/G/B字节的计数器 (0, 1, 2)
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
                udp_data_length     <= 16'd668;
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
                byte_select_cnt     <= 2'd0; // 修改
                read_data_buf       <= 32'd0;
        end
        else begin
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
                                byte_select_cnt     <= 2'd0; // 修改
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
                                // 提前3个周期开始读取!!!
                                if(app_tx_header_cnt == 9'd232) begin
                                        read_en <= 1'b1;
                                end
                                else if(app_tx_header_cnt == 9'd240) begin
                                        read_en <= 1'b0;
                                        read_en_d1 <= 1'b1;
                                end
                                else if(app_tx_header_cnt == 9'd248) begin
                                        read_en_d1 <= 1'b0;
                                        read_en_d2 <= 1'b1;  // 标记数据准备就绪
                                end

                                if(app_tx_header_cnt >= IMG_HEADER_LEN) begin
                                        STATE              <= SEND_UDP_DATA;
                                        app_tx_data_valid  <= 1'b1;
                                        app_tx_data        <= UDP_HEADER_32[app_tx_header_cnt +: 8];
                                        app_tx_header_cnt  <= 9'd0;
                                        fifo_read_data_cnt <= 12'd0;
                                        byte_select_cnt    <= 2'd0; //不要改操
                                end
                                else begin
                                        STATE             <= SEND_UDP_HEADER;
                                        app_tx_data_valid <= 1'b1;
                                        app_tx_data       <= UDP_HEADER_32[app_tx_header_cnt +: 8];
                                        app_tx_header_cnt <= app_tx_header_cnt + 8;
                                end
                        end

                        // ========== 状态 5：发送图像数据 (3字节/像素) ==========
                        SEND_UDP_DATA: begin
                                // read_en 延迟打拍 (保持和原设计一致的流水线)
                                read_en_d2 <= read_en_d1;
                                read_en_d1 <= read_en;

                                if(read_en_d2) begin
                                        read_data_buf <= read_data;
                                end

                                // 读使能控制 
                                if(byte_select_cnt == 2'd2) begin
                                        read_en <= 1'b1;
                                end
                                else begin
                                        read_en <= 1'b0;
                                end

                                // 数据输出 
                                case(byte_select_cnt)
                                        2'd0: app_tx_data <= read_data_buf[31:24];  // R
                                        2'd1: app_tx_data <= read_data_buf[23:16];  // G
                                        2'd2: app_tx_data <= read_data_buf[15:8];   // B
                                        default: app_tx_data <= 8'h00;
                                endcase

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

                                        if(byte_select_cnt >= 2'd2)
                                                byte_select_cnt <= 2'd0;
                                        else
                                                byte_select_cnt <= byte_select_cnt + 1;
                                end
                        end

                        // ========== 状态 6：延迟 ==========
                        DELAY: begin
                                if(delay_cnt >= 1500) begin
                                        delay_cnt  <= 22'd0;
                                        IMG_FRAMSEQ <= IMG_FRAMSEQ + 1'd1;
                                        IMG_OFFSET  <= IMG_OFFSET + IMG_FRAMSIZE;

                                        if(IMG_FRAMSEQ >= (IMG_FRAMTOTAL - 1))
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
