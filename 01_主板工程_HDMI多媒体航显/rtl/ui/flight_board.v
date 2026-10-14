`timescale 1ns / 1ps
//=============================================================================
// flight_board.v - 航班信息看板叠加层
//
// 6 行 x 40 字符的实时看板，固定在 y = 96..287，横跨整个 640 像素宽。
// 字符按 2 倍放大绘制，所以一格里是 16x16 像素、行距 32 像素。
// 第一行是 FLIGHT/DEST/SCHED/GATE/STATUS 表头，下面五行放具体航班。
//
// The character buffer is distributed RAM and the glyphs are still drawn by
// the single 8x8 font that lives in ui_overlay.v.  Sharing that font instead
// of adding a second copy is what makes the board affordable at 96% slice
// usage: the whole feature adds one RAM array, the address arithmetic and a
// small amount of colour logic.
//
// The host writes one byte per UART handshake; the address is row * 40 + col.
// A write makes that row flash for about half a second, which is the visible
// "the data just changed" effect on screen.
//=============================================================================
module flight_board (
    input  wire        pixel_clk,
    input  wire        rst_n,
    input  wire        frame_tick,

    input  wire [9:0]  px,
    input  wire [9:0]  py,

    input  wire        enable,        // board on/off, sys clock domain

    input  wire        wr_tog,        // one toggle per written byte
    input  wire [7:0]  wr_addr,       // 0..239
    input  wire [7:0]  wr_data,

    output wire        in_board,      // pixel is inside the board band
    output wire        glyph_y_ok,    // pixel is inside the text rows
    output wire [7:0]  ch,
    output wire [3:0]  gx,
    output wire [3:0]  gy,
    output wire [2:0]  row,
    output wire        row_flash,     // this row was just updated
    output wire        border_hit,    // 2-pixel top/bottom rail
    output wire [23:0] breath_rgb     // slowly cycling rail colour
);

    localparam integer ROWS     = 6;
    localparam integer COLS     = 40;
    localparam integer CHARS    = ROWS * COLS;   // 240

    localparam [9:0] BOARD_Y0 = 10'd96;
    localparam [9:0] BOARD_H  = 10'd192;
    localparam [4:0] TEXT_OFF = 5'd8;            // centre 16 px glyph in 32

    //------------------------------------------------------------------
    // host write port, synchronised into the pixel clock domain.
    // The writer holds address and data stable while it flips the toggle,
    // so sampling them two stages behind the toggle is safe.
    //------------------------------------------------------------------
    reg        wtog_s1, wtog_s2, wtog_s3;
    reg [7:0]  waddr_s1, waddr_s2;
    reg [7:0]  wdata_s1, wdata_s2;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            wtog_s1  <= 1'b0;
            wtog_s2  <= 1'b0;
            wtog_s3  <= 1'b0;
            waddr_s1 <= 8'd0;
            waddr_s2 <= 8'd0;
            wdata_s1 <= 8'd0;
            wdata_s2 <= 8'd0;
        end else begin
            wtog_s1  <= wr_tog;
            wtog_s2  <= wtog_s1;
            wtog_s3  <= wtog_s2;
            waddr_s1 <= wr_addr;
            waddr_s2 <= waddr_s1;
            wdata_s1 <= wr_data;
            wdata_s2 <= wdata_s1;
        end
    end

    wire       wr_hit = wtog_s2 ^ wtog_s3;
    wire [7:0] wr_a   = waddr_s2;
    wire [7:0] wr_d   = wdata_s2;
    wire       wr_ok  = (wr_a < 8'd240);

    //------------------------------------------------------------------
    // 6 x 40 character buffer.  The design is slice-limited, so this small
    // array is forced into one of the unused 9K block RAMs.  The extra 80
    // bytes still fit in the same block, so the map stays compact.
    //------------------------------------------------------------------
    reg [7:0] ch_mem [0:CHARS-1]; /* fehdl force_ram=1, ram_style="bram" */
    reg [7:0] rd_ch;
    reg [5:0] flash_cnt;
    reg [2:0] flash_row;

    //------------------------------------------------------------------
    // pixel address decode
    //------------------------------------------------------------------
    wire [9:0] dy      = py - BOARD_Y0;
    wire       in_y    = (py >= BOARD_Y0) && (py < (BOARD_Y0 + BOARD_H));

    wire [2:0] row_i   = dy[7:5];
    wire [4:0] y_row   = dy[4:0];
    wire [5:0] col_i   = px[9:4];
    wire [2:0] gx_i    = px[3:1];

    // row * 40 = row * 32 + row * 8
    wire [7:0] row_x32 = {row_i, 5'b00000};
    wire [7:0] row_x8  = {2'b00, row_i, 3'b000};
    wire [7:0] row_base= row_x32 + row_x8;
    wire [7:0] rd_addr = row_base + {2'b00, col_i};

    wire       y_ok    = (y_row >= TEXT_OFF) && (y_row < (TEXT_OFF + 5'd16));
    wire [4:0] y_off   = y_row - TEXT_OFF;
    wire [2:0] gy_i    = y_off[3:1];

    reg        en_s1, en_s2;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            en_s1 <= 1'b0;
            en_s2 <= 1'b0;
        end else begin
            en_s1 <= enable;
            en_s2 <= en_s1;
        end
    end

    // Registered pixel-side outputs.  These use the same pipeline stage as
    // the BRAM read so a character and its coordinates stay together.
    reg        in_board_r;
    reg        glyph_y_ok_r;
    reg [3:0]  gx_r, gy_r;
    reg [2:0]  row_r;
    reg        row_flash_r;
    reg        border_hit_r;

    wire [2:0] wr_row = (wr_a < 8'd40)  ? 3'd0 :
                        (wr_a < 8'd80)  ? 3'd1 :
                        (wr_a < 8'd120) ? 3'd2 :
                        (wr_a < 8'd160) ? 3'd3 :
                        (wr_a < 8'd200) ? 3'd4 : 3'd5;
    wire       row_flash_now = (flash_cnt != 6'd0) && (row_i == flash_row);
    wire       border_hit_now = in_y &&
                                ((dy < 10'd2) || (dy >= (BOARD_H - 10'd2)));

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ch        <= 8'd0;
            in_board_r   <= 1'b0;
            glyph_y_ok_r <= 1'b0;
            gx_r         <= 4'd0;
            gy_r         <= 4'd0;
            row_r        <= 3'd0;
            row_flash_r  <= 1'b0;
            border_hit_r <= 1'b0;
        end else begin
            if (wr_hit && wr_ok)
                ch_mem[wr_a] <= wr_d;
            rd_ch        <= ch_mem[rd_addr];
            in_board_r   <= en_s2 && in_y;
            glyph_y_ok_r <= in_y && y_ok;
            gx_r         <= {1'b0, gx_i};
            gy_r         <= {1'b0, gy_i};
            row_r        <= row_i;
            row_flash_r  <= row_flash_now;
            border_hit_r <= border_hit_now;
        end
    end

    assign in_board   = in_board_r;
    assign glyph_y_ok = glyph_y_ok_r;
    assign ch         = rd_ch;
    assign gx         = gx_r;
    assign gy         = gy_r;
    assign row        = row_r;

    //------------------------------------------------------------------
    // update flash: the row that was just written blinks for ~0.5 s.
    //------------------------------------------------------------------
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            flash_cnt <= 6'd0;
            flash_row <= 3'd0;
        end else if (wr_hit && wr_ok) begin
            flash_cnt <= 6'd30;
            flash_row <= wr_row;
        end else if (frame_tick && (flash_cnt != 6'd0)) begin
            flash_cnt <= flash_cnt - 6'd1;
        end
    end

    assign row_flash = row_flash_r;

    //------------------------------------------------------------------
    // decorative rails and their slow colour cycle
    //------------------------------------------------------------------
    assign border_hit   = border_hit_r;

    reg [7:0] breath_cnt;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n)
            breath_cnt <= 8'd0;
        else if (frame_tick)
            breath_cnt <= breath_cnt + 8'd1;
    end

    assign breath_rgb = (breath_cnt[7:6] == 2'd0) ? 24'h0080B0 :
                        (breath_cnt[7:6] == 2'd1) ? 24'h00C8F0 :
                        (breath_cnt[7:6] == 2'd2) ? 24'h7FE8FF :
                                                    24'h0090C8;

endmodule
