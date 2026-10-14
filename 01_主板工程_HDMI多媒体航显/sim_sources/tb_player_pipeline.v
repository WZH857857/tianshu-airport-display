`timescale 1ns / 1ps
//=============================================================================
// tb_player_pipeline.v - the whole picture path, without the HDMI PHY
//
//   player_ctrl -> fat32_file_reader -> sector_reader -> BMP parser
//               -> frame_store -> SDRAM -> memory contents
//
// The SD card controller is replaced by a small model that answers the
// sector read interface from a synthetic FAT32 image, so everything above the
// SPI byte layer is exercised: mounting, the directory scan, the file stream,
// the BMP decode and the frame buffer write.
//
// The picture is 640 x 4 to keep the run short; the row stride and the row
// order logic are the same as in the 640 x 480 build.
//=============================================================================
module tb_player_pipeline;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer ROW_BYTES   = 640 * 3;
    localparam integer DATA_OFFSET = 54;
    localparam integer FILE_SIZE   = DATA_OFFSET + ROW_BYTES * LINES;
    localparam integer SPC         = 16;        // sectors per cluster
    localparam integer DATA_START  = 3;         // LBA of cluster 2
    localparam integer FILE_CLUSTER= 3;
    localparam integer FILE_LBA    = DATA_START + (FILE_CLUSTER - 2) * SPC;
    localparam integer CARD_SECTORS= FILE_LBA + SPC;
    localparam [20:0]  BUF0_BASE   = 21'h000000;
    localparam [20:0]  BUF1_BASE   = 21'h001000;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    wire sd_init_done = 1'b1;

    always #10 clk = ~clk;              // 50 MHz

    //------------------------------------------------------------------
    // synthetic FAT32 card image
    //------------------------------------------------------------------
    reg [7:0] card [0:CARD_SECTORS*512-1];
    integer   i, n;
    integer   y, x, r;
    integer   errors;

    function [7:0] pixel_byte;
        input integer k;                // byte offset inside the pixel data
        integer rr, rem, xx, cc;
        begin
            rr  = k / ROW_BYTES;
            rem = k % ROW_BYTES;
            xx  = rem / 3;
            cc  = rem % 3;
            if      (cc == 0) pixel_byte = 8'h5A;       // blue
            else if (cc == 1) pixel_byte = xx[7:0];     // green
            else              pixel_byte = rr[7:0];     // red
        end
    endfunction

    task card_zero;
        input integer sector;
        begin
            for (i = 0; i < 512; i = i + 1)
                card[sector*512 + i] = 8'h00;
        end
    endtask

    task card_put;
        input integer sector;
        input integer offset;
        input [7:0]   value;
        begin
            card[sector*512 + offset] = value;
        end
    endtask

    task card_put32;
        input integer sector;
        input integer offset;
        input [31:0]  value;
        begin
            card[sector*512 + offset + 0] = value[7:0];
            card[sector*512 + offset + 1] = value[15:8];
            card[sector*512 + offset + 2] = value[23:16];
            card[sector*512 + offset + 3] = value[31:24];
        end
    endtask

    //------------------------------------------------------------------
    // modelled SD controller: streams a sector, four cycles after the
    // request, one byte per cycle
    //------------------------------------------------------------------
    wire        sd_sec_read;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;
    wire        sd_sec_read_data_valid;
    wire        sd_sec_read_end;
    wire        sd_sec_read_error;
    wire [3:0]  sd_sec_read_error_code;

    reg  [7:0]  m_data;
    reg         m_valid, m_end;
    reg  [31:0] m_lba;
    reg  [9:0]  m_cnt;
    reg  [2:0]  m_state;

    assign sd_sec_read_data       = m_data;
    assign sd_sec_read_data_valid = m_valid;
    assign sd_sec_read_end        = m_end;
    assign sd_sec_read_error      = 1'b0;
    assign sd_sec_read_error_code = 4'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_state <= 3'd0;
            m_lba   <= 32'd0;
            m_cnt   <= 10'd0;
            m_data  <= 8'd0;
            m_valid <= 1'b0;
            m_end   <= 1'b0;
        end else begin
            m_valid <= 1'b0;
            m_end   <= 1'b0;

            case (m_state)
                3'd0: if (sd_sec_read) begin
                          m_lba   <= sd_sec_read_addr;
                          m_cnt   <= 10'd0;
                          m_state <= 3'd1;
                      end
                3'd1: m_state <= 3'd2;                  // read latency
                3'd2: begin
                          m_data  <= card[(m_lba << 9) + m_cnt];
                          m_valid <= 1'b1;
                          if (m_cnt == 10'd511)
                              m_state <= 3'd3;
                          else
                              m_cnt <= m_cnt + 10'd1;
                      end
                3'd3: begin
                          m_end   <= 1'b1;
                          m_state <= 3'd4;
                      end
                3'd4: if (!sd_sec_read)
                          m_state <= 3'd0;
                default: m_state <= 3'd0;
            endcase
        end
    end

    //------------------------------------------------------------------
    // sector buffer, FAT32 reader and BMP parser
    //------------------------------------------------------------------
    wire        sec_start, sec_busy, sec_ready, sec_error, sec_release;
    wire [3:0]  sec_error_code;
    wire [31:0] sec_lba;
    wire [8:0]  sec_raddr0, sec_raddr1, sec_raddr2, sec_raddr3;
    wire [7:0]  sec_rdata0, sec_rdata1, sec_rdata2, sec_rdata3;

    sector_reader #(
        .TIMEOUT_CYCLES (24'hFF_FFFF)
    ) u_sector (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .start                   (sec_start),
        .lba                     (sec_lba),
        .busy                    (sec_busy),
        .ready                   (sec_ready),
        .error                   (sec_error),
        .error_code              (sec_error_code),
        .raddr0                  (sec_raddr0),
        .raddr1                  (sec_raddr1),
        .raddr2                  (sec_raddr2),
        .raddr3                  (sec_raddr3),
        .rdata0                  (sec_rdata0),
        .rdata1                  (sec_rdata1),
        .rdata2                  (sec_rdata2),
        .rdata3                  (sec_rdata3),
        .release_req             (sec_release),
        .sd_init_done            (sd_init_done),
        .sd_sec_read             (sd_sec_read),
        .sd_sec_read_addr        (sd_sec_read_addr),
        .sd_sec_read_data        (sd_sec_read_data),
        .sd_sec_read_data_valid  (sd_sec_read_data_valid),
        .sd_sec_read_end         (sd_sec_read_end),
        .sd_sec_read_error       (sd_sec_read_error),
        .sd_sec_read_error_code  (sd_sec_read_error_code)
    );

    wire        start_scan, scan_busy, scan_done, scan_error;
    wire [3:0]  scan_fault_code;
    wire [7:0]  file_count, file_index;
    wire        load_start, load_abort, load_busy, load_done, load_error;
    wire [31:0] selected_start_cluster, selected_size;
    wire [7:0]  file_data;
    wire        file_data_valid, file_data_ready;
    wire [5:0]  fat_state_dbg;

    fat32_file_reader #(
        .MAX_FILES (32)
    ) u_fat32 (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .sd_init_done           (sd_init_done),
        .start_scan             (start_scan),
        .scan_busy              (scan_busy),
        .scan_done              (scan_done),
        .scan_error             (scan_error),
        .scan_fault_code        (scan_fault_code),
        .file_count             (file_count),
        .file_index             (file_index),
        .load_src               (2'd0),
        .load_start             (load_start),
        .load_abort             (load_abort),
        .load_busy              (load_busy),
        .load_done              (load_done),
        .load_error             (load_error),
        .selected_start_cluster (selected_start_cluster),
        .selected_size          (selected_size),
        .voice_found            (),
        .voice_start_cluster    (),
        .voice_size             (),
        .file_data              (file_data),
        .file_data_valid        (file_data_valid),
        .file_data_ready        (file_data_ready),
        .sec_start              (sec_start),
        .sec_lba                (sec_lba),
        .sec_busy               (sec_busy),
        .sec_ready              (sec_ready),
        .sec_error              (sec_error),
        .sec_error_code         (sec_error_code),
        .sec_raddr0             (sec_raddr0),
        .sec_raddr1             (sec_raddr1),
        .sec_raddr2             (sec_raddr2),
        .sec_raddr3             (sec_raddr3),
        .sec_rdata0             (sec_rdata0),
        .sec_rdata1             (sec_rdata1),
        .sec_rdata2             (sec_rdata2),
        .sec_rdata3             (sec_rdata3),
        .sec_release            (sec_release),
        .dbg_state              (fat_state_dbg)
    );

    wire        bmp_busy, bmp_done, bmp_error;
    wire [7:0]  bmp_error_code;
    wire [23:0] bmp_rgb;
    wire        bmp_px_valid, bmp_px_last, bmp_px_ready;
    wire [15:0] bmp_width, bmp_height, bmp_bpp;
    wire [31:0] bmp_data_offset;
    wire        bmp_start;
    wire        img_active, img_reload;

    bmp_stream_parser #(
        .IMG_W (640),
        .IMG_H (LINES)
    ) u_bmp (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (bmp_start),
        .file_size       (selected_size),
        .busy            (bmp_busy),
        .done            (bmp_done),
        .error           (bmp_error),
        .error_code      (bmp_error_code),
        .data            (file_data),
        .data_valid      (file_data_valid),
        .data_ready      (file_data_ready),
        .hold_done       (1'b0),
        .px_valid        (bmp_px_valid),
        .px_rgb          (bmp_rgb),
        .px_last         (bmp_px_last),
        .px_ready        (bmp_px_ready),
        .hdr_width       (bmp_width),
        .hdr_height      (bmp_height),
        .hdr_bpp         (bmp_bpp),
        .hdr_data_offset (bmp_data_offset)
    );

    //------------------------------------------------------------------
    // the real sequencer, with short timers so the test does not crawl
    //------------------------------------------------------------------
    wire [2:0] ctrl_state;
    wire       card_ok, file_error;

    player_ctrl #(
        .HOLD_CYCLES  (100000),
        .RETRY_CYCLES (50000),
        .LOAD_CYCLES  (2000000),
        .SCAN_CYCLES  (2000000),
        .PRELOAD      (0)
    ) u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .key_pressed    (1'b0),
        .auto_rotate_en (1'b1),
        .sd_init_done   (sd_init_done),
        .start_scan     (start_scan),
        .scan_done      (scan_done),
        .scan_error     (scan_error),
        .file_count     (file_count),
        .file_index     (file_index),
        .load_start     (load_start),
        .load_abort     (load_abort),
        .load_busy      (load_busy),
        .load_done      (load_done),
        .load_error     (load_error),
        .bmp_start      (bmp_start),
        .bmp_done       (bmp_done),
        .bmp_error      (bmp_error),
        .img_active     (img_active),
        .reload         (img_reload),
        .preload_ready  (1'b0),
        .swap_req       (),
        .swap_done      (1'b0),
        .state_dbg      (ctrl_state),
        .card_ok        (card_ok),
        .file_error     (file_error)
    );

    //------------------------------------------------------------------
    // frame buffer and SDRAM model
    //------------------------------------------------------------------
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire        sdram_cs_n, sdram_cke;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;
    wire [23:0] pix_rgb;
    wire        pix_uf;
    wire        frame_ready, frame_loaded, init_done, cal_fail;
    wire [2:0]  ld_state;
    reg         pix_de, pix_frame_tog;
    reg  [10:0] hcount;
    reg  [5:0]  vcount;

    // small video timing model, so the reader can finish frames and swap
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hcount        <= 11'd0;
            vcount        <= 6'd0;
            pix_de        <= 1'b0;
            pix_frame_tog <= 1'b0;
        end else begin
            pix_de <= (hcount < 11'd640) && (vcount < LINES);
            if (hcount == 11'd799) begin
                hcount <= 11'd0;
                if (vcount == (LINES + 20 - 1)) begin
                    vcount        <= 6'd0;
                    pix_frame_tog <= ~pix_frame_tog;
                end else
                    vcount <= vcount + 6'd1;
            end else
                hcount <= hcount + 11'd1;
        end
    end

    frame_store #(
        .FRAME_LINES (LINES),
        .FRAME_WORDS (FRAME_WORDS),
        .BUF0_BASE   (BUF0_BASE),
        .BUF1_BASE   (BUF1_BASE),
        .FIFO_AW     (8)
    ) u_store (
        .clk           (clk),
        .mem_clk       (clk),
        .mem_clk_sft   (clk),
        .rst_n         (rst_n),
        .reload        (img_reload),
        .pattern_mode  (2'd0),
        .img_active    (img_active),
        .img_abort     (1'b0),
        .swap_req      (1'b0),
        .img_valid     (bmp_px_valid),
        .img_rgb       (bmp_rgb),
        .img_ready     (bmp_px_ready),
        .frame_ready   (frame_ready),
        .frame_loaded  (frame_loaded),
        .img_frame_ready (),
        .preload_ready (),
        .swap_done     (),
        .init_done     (init_done),
        .cal_fail      (cal_fail),
        .cal_latency   (),
        .state_dbg     (ld_state),
        .sdram_ras_n   (sdram_ras_n),
        .sdram_cas_n   (sdram_cas_n),
        .sdram_we_n    (sdram_we_n),
        .sdram_addr    (sdram_addr),
        .sdram_ba      (sdram_ba),
        .sdram_cs_n    (sdram_cs_n),
        .sdram_cke     (sdram_cke),
        .sdram_dm      (sdram_dm),
        .sdram_dq      (sdram_dq),
        .pix_clk       (clk),
        .pix_de        (pix_de),
        .pix_frame_tog (pix_frame_tog),
        .pix_rgb       (pix_rgb),
        .pix_underflow (pix_uf)
    );

    sdram_model #(
        .CL    (3),
        .T_RCD (3),
        .T_RP  (3),
        .T_RAS (5),
        .T_RFC (8)
    ) g_model (
        .clk   (clk),
        .cke   (sdram_cke),
        .cs_n  (sdram_cs_n),
        .ras_n (sdram_ras_n),
        .cas_n (sdram_cas_n),
        .we_n  (sdram_we_n),
        .addr  (sdram_addr),
        .ba    (sdram_ba),
        .dq    (sdram_dq),
        .dm    (sdram_dm)
    );

    //------------------------------------------------------------------
    // run
    //------------------------------------------------------------------
    reg [20:0] base;
    reg [23:0] want, got;
    integer    fl_count;

    always @(posedge clk)
        if (rst_n && frame_loaded) begin
            fl_count = fl_count + 1;
            $display("FRAME %0d t=%0t wbuf=%0d dbuf=%0d img_active=%b word0=%08h",
                     fl_count, $time, u_store.wr_buf, u_store.disp_buf_pix,
                     img_active,
                     u_store.u_sdram_ctrl.mem[
                         (u_store.wr_buf ? BUF1_BASE : BUF0_BASE)]);
        end

    integer take_cnt;
    reg [2:0] prev_wstate;
    reg       bmp_err_q, load_err_q, bmp_done_q, load_done_q, scan_err_q;

    always @(posedge clk) begin
        if (rst_n) begin
            bmp_err_q   <= bmp_error;
            load_err_q  <= load_error;
            bmp_done_q  <= bmp_done;
            load_done_q <= load_done;
            scan_err_q  <= scan_error;
            if (bmp_error && !bmp_err_q)
                $display("EVENT bmp_error t=%0t code=%0d", $time, bmp_error_code);
            if (load_error && !load_err_q)
                $display("EVENT load_error t=%0t", $time);
            if (bmp_done && !bmp_done_q)
                $display("EVENT bmp_done t=%0t", $time);
            if (load_done && !load_done_q)
                $display("EVENT load_done t=%0t", $time);
            if (scan_error && !scan_err_q)
                $display("EVENT scan_error t=%0t", $time);
        end
        if (rst_n && bmp_px_valid && bmp_px_ready) begin
            take_cnt = take_cnt + 1;
            if (take_cnt <= 6)
                $display("TAKE %0d t=%0t rgb=%06h wstate=%0d count=%0d",
                         take_cnt, $time, bmp_rgb, u_store.wstate,
                         u_store.wr_count);
        end
        if (rst_n && (u_store.wstate !== prev_wstate)) begin
            prev_wstate <= u_store.wstate;
            if (take_cnt <= 6 || u_store.wstate == 3'd0)
                $display("WSTATE t=%0t %0d -> %0d img_active=%b",
                         $time, prev_wstate, u_store.wstate, img_active);
        end
    end

    initial begin
        // blank image
        for (i = 0; i < CARD_SECTORS*512; i = i + 1)
            card[i] = 8'h00;

        // MBR: one FAT32 partition starting at LBA 1
        card_put(0, 446 + 4, 8'h0C);
        card_put32(0, 446 + 8, 32'd1);
        card_put(0, 510, 8'h55);
        card_put(0, 511, 8'hAA);

        // volume boot record
        card_put(1, 11, 8'h00);
        card_put(1, 12, 8'h02);         // 512 bytes per sector
        card_put(1, 13, 8'd16);         // sectors per cluster
        card_put(1, 14, 8'h01);         // reserved sectors
        card_put(1, 16, 8'h01);         // number of FATs
        card_put32(1, 36, 32'd1);       // FAT32 size in sectors
        card_put32(1, 44, 32'd2);       // root directory cluster
        card_put(1, 510, 8'h55);
        card_put(1, 511, 8'hAA);

        // FAT: entry 3 ends the file
        card_put32(2, 12, 32'h0FFF_FFFF);

        // root directory entry: IMG0.BMP, cluster 3, FILE_SIZE bytes
        card_put(3, 0, "I");
        card_put(3, 1, "M");
        card_put(3, 2, "G");
        card_put(3, 3, "0");
        card_put(3, 8, "B");
        card_put(3, 9, "M");
        card_put(3, 10, "P");
        card_put(3, 11, 8'h20);         // attribute: archive
        card_put(3, 20, 8'h00);
        card_put(3, 21, 8'h00);
        card_put(3, 26, 8'd3);
        card_put(3, 27, 8'h00);
        card_put32(3, 28, FILE_SIZE);
        card_put(3, 32, 8'h00);         // end of directory

        // file contents: BMP header then the pixels, bottom row first
        card_put(FILE_LBA, 0, 8'h42);
        card_put(FILE_LBA, 1, 8'h4D);
        card_put(FILE_LBA, 10, 8'd54);
        card_put(FILE_LBA, 14, 8'd40);
        card_put(FILE_LBA, 18, 8'd128); // 640
        card_put(FILE_LBA, 19, 8'd2);
        card_put(FILE_LBA, 22, LINES);
        card_put(FILE_LBA, 26, 8'h01);
        card_put(FILE_LBA, 28, 8'd24);
        for (n = 0; n < ROW_BYTES*LINES; n = n + 1)
            card[(FILE_LBA << 9) + DATA_OFFSET + n] = pixel_byte(n);

        errors = 0;
        fl_count = 0;
        take_cnt = 0;
        prev_wstate = 3'd7;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // wait for the BMP frame to reach the frame buffer
        wait (bmp_done === 1'b1);
        wait (frame_loaded === 1'b1);
        repeat (5) @(posedge clk);

        if (scan_error) begin
            $display("FAIL: scan_error");
            errors = errors + 1;
        end
        if (file_count != 8'd1) begin
            $display("FAIL: file_count=%0d expected 1", file_count);
            errors = errors + 1;
        end
        if (bmp_error) begin
            $display("FAIL: bmp_error=%0d", bmp_error_code);
            errors = errors + 1;
        end
        if (selected_size != FILE_SIZE) begin
            $display("FAIL: selected_size=%0d expected %0d",
                     selected_size, FILE_SIZE);
            errors = errors + 1;
        end
        if (bmp_width != 16'd640 || bmp_height != LINES || bmp_bpp != 16'd24)
            begin
                $display("FAIL: header w=%0d h=%0d bpp=%0d",
                         bmp_width, bmp_height, bmp_bpp);
                errors = errors + 1;
            end

        // picture row y comes from file row LINES-1-y
        base = u_store.wr_buf ? BUF1_BASE : BUF0_BASE;
        for (y = 0; y < LINES; y = y + 1) begin
            r = LINES - 1 - y;
            for (x = 0; x < 640; x = x + 1) begin
                want = {r[7:0], x[7:0], 8'h5A};
                got  = u_store.u_sdram_ctrl.mem[base + y * 640 + x][23:0];
                if (got !== want) begin
                    errors = errors + 1;
                    if (errors <= 8)
                        $display("MISMATCH y=%0d x=%0d got=%06h want=%06h",
                                 y, x, got, want);
                end
            end
        end

        $display("--------------------------------------------------");
        $display("tb_player_pipeline: files=%0d size=%0d bmp_err=%0d",
                 file_count, selected_size, bmp_error_code);
        $display("tb_player_pipeline: ctrl_state=%0d card_ok=%b file_error=%b",
                 ctrl_state, card_ok, file_error);
        $display("tb_player_pipeline: sdram reads=%0d writes=%0d errors=%0d",
                 g_model.n_rd, g_model.n_wr, g_model.n_err);
        $display("tb_player_pipeline: mismatches=%0d", errors);

        if ((errors == 0) && (g_model.n_err == 0))
            $display("RESULT: PASS tb_player_pipeline");
        else
            $display("RESULT: FAIL tb_player_pipeline");
        $finish;
    end

    initial begin
        #50_000_000;
        $display("RESULT: FAIL tb_player_pipeline timeout state=%0d fat=%0d bmp=%b frame=%b",
                 ctrl_state, fat_state_dbg, bmp_busy, frame_loaded);
        $finish;
    end

endmodule
