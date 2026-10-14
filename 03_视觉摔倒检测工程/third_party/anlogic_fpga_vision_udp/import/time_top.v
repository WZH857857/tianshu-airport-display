module time_top (
    input           clk_50m,        // 50MHz输入时钟
    input           rst_n,          // 异步复位，低电平有效
    input [1:0]     resolution_sel, // 分辨率选择: 0=640×480, 1=720p, 2=1080p
    output          clk_125m_0_1,   // 125MHz, 0度
    output          clk_125m_0_2,   // 125MHz, 0度（第二个）
    output          clk_125m_90,    // 125MHz, 90度
    output          clk_25m_0,      // 25MHz, 0度 (640×480像素时钟)
    output          clk_12_5m_0,    // 12.5MHz, 0度
    output          clk_50m_0,      // 50MHz, 0度
    output          clk_50m_180,    // 50MHz, 180度
    output          clk_1_25m_0,
    output          pll_locked,     // PLL锁定信号
    output          video_clk_out,  // 选择后的像素时钟
    output          video_clk_5x_out, // 选择后的5x时钟
    output          video_pll_locked  // video_pll锁定信号
);

// PLL实例化
pll_core pll_core (
    .refclk    (clk_50m),      // 输入时钟50MHz
    .reset      (~rst_n),       // PLL复位（高电平有效）
    .clk0_out   (clk_125m_0_1), // 125MHz, 0度
    .clk1_out   (clk_125m_90),  // 125MHz, 90度
    .clk2_out   (clk_50m_0),    // 50MHz, 0度
    .clk3_out   (clk_50m_180),  // 50MHz, 180度
    .clk4_out   (clk_25m_0),    // 25MHz, 0度
    .extlock     (pll_locked)    // PLL锁定信号
);

// 将第一个125MHz 0度时钟连接到第二个输出
assign clk_125m_0_2 = clk_125m_0_1;

// ============================================================
// video_pll: 生成720p和1080p的像素时钟和5x时钟
// ============================================================
wire clk_720p_pixel;   // 74.25MHz
wire clk_720p_5x;      // 371.25MHz
wire clk_1080p_pixel;  // 148.5MHz
wire clk_1080p_5x;     // 742.5MHz

video_pll video_pll_inst (
    .refclk(clk_50m),
    .reset(~rst_n),
    .clk0_out(clk_720p_pixel),
    .clk1_out(clk_720p_5x),
    .clk2_out(clk_1080p_pixel),
    .clk3_out(clk_1080p_5x)
);

// video_pll没有独立的extlock，使用pll_core的locked信号
assign video_pll_locked = pll_locked;

// ============================================================
// 时钟选择逻辑：根据分辨率选择对应的像素时钟和5x时钟
// ============================================================
reg [1:0] resolution_sel_sync;
reg video_clk_mux;
reg video_clk_5x_mux;

// 同步resolution_sel到时钟域（防止亚稳态）
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        resolution_sel_sync <= 2'd0;
    else
        resolution_sel_sync <= resolution_sel;
end

// 时钟选择（组合逻辑）
always @(*) begin
    case (resolution_sel_sync)
        2'd0: begin  // 640×480
            video_clk_mux = clk_25m_0;
            video_clk_5x_mux = clk_125m_0_1;
        end
        2'd1: begin  // 1280×720
            video_clk_mux = clk_720p_pixel;
            video_clk_5x_mux = clk_720p_5x;
        end
        2'd2: begin  // 1920×1080
            video_clk_mux = clk_1080p_pixel;
            video_clk_5x_mux = clk_1080p_5x;
        end
        default: begin  // 默认640×480
            video_clk_mux = clk_25m_0;
            video_clk_5x_mux = clk_125m_0_1;
        end
    endcase
end

assign video_clk_out = video_clk_mux;
assign video_clk_5x_out = video_clk_5x_mux;

// ============================================================
// 12.5MHz分频器（从25MHz分频得到）
// ============================================================
reg clk_12_5m_reg;
reg div_cnt;

always @(posedge clk_25m_0 or negedge rst_n) begin
    if (!rst_n) begin
        clk_12_5m_reg <= 1'b0;
        div_cnt <= 1'b0;
    end 
    else if (pll_locked) begin
        if (div_cnt == 1'b1) begin
            clk_12_5m_reg <= ~clk_12_5m_reg;
            div_cnt <= 1'b0;
        end 
        else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end
end

assign clk_12_5m_0 = clk_12_5m_reg;




// 1.25MHz分频器（从12.5MHz分频得到）
reg clk_1_25m_reg;
reg [2:0] div_cnt_1_25;

always @(posedge clk_12_5m_0 or negedge rst_n) begin
    if (!rst_n) begin
        clk_1_25m_reg <= 1'b0;
        div_cnt_1_25 <= 3'b0;
    end else if (pll_locked) begin
        if (div_cnt_1_25 == 3'd4) begin  // 5分频（0-4计数）
            clk_1_25m_reg <= ~clk_1_25m_reg;
            div_cnt_1_25 <= 3'b0;
        end else begin
            div_cnt_1_25 <= div_cnt_1_25 + 3'b1;
        end
    end
end

assign clk_1_25m_0 = clk_1_25m_reg;

endmodule