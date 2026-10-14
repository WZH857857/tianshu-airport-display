`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store_bmp.v - BMP stream into the SDRAM frame buffer
//
// A small picture (640 x 4) keeps the run short while using exactly the same
// loader, SDRAM controller and address arithmetic as the 480 line build.
//
// The generated file stores the rows bottom up, the way a BMP does, and every
// pixel encodes its own place: red carries the file row, green carries the
// column. After the frame is written the SDRAM contents are read back and
// compared against the expected top down picture, which fails if the loader
// gets the row order or the stride wrong.
//=============================================================================
module tb_frame_store_bmp;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer ROW_BYTES   = 640 * 3;          // no BMP padding
    localparam integer DATA_OFFSET = 54;
    localparam integer FILE_SIZE   = DATA_OFFSET + ROW_BYTES * LINES;
    localparam [20:0]  BUF0_BASE   = 21'h000000;
    localparam [20:0]  BUF1_BASE   = 21'h001000;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    always #10 clk = ~clk;          // 50 MHz

    //------------------------------------------------------------------
    // byte source: a synthetic BMP file
    //------------------------------------------------------------------
    reg  [7:0] hdr [0:DATA_OFFSET-1];
    reg  [7:0] data;
    reg        data_valid;
    wire       data_ready;
    integer    pidx;
    reg        bmp_start;
    reg  [31:0] file_size;
    wire       bmp_busy, bmp_done, bmp_error;
    wire [7:0] bmp_error_code;
    wire       px_valid;
    wire [23:0] px_rgb;
    wire       px_last;
    wire       px_ready;

    // file byte n: header first, then bottom up rows, each pixel blue,
    // green, red
    function [7:0] file_byte;
        input integer n;
        integer k, r, rem, x, c;
        begin
            if (n < DATA_OFFSET)
                file_byte = hdr[n];
            else begin
                k   = n - DATA_OFFSET;
                r   = k / ROW_BYTES;
                rem = k % ROW_BYTES;
                x   = rem / 3;
                c   = rem % 3;
                // red = file row, green = column, blue = 0x5A
                if      (c == 0) file_byte = 8'h5A;
                else if (c == 1) file_byte = x[7:0];
                else             file_byte = r[7:0];
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data       <= 8'd0;
            data_valid <= 1'b0;
            pidx       <= 0;
        end else begin
            if (data_valid) begin
                if (data_ready) begin
                    if (pidx < FILE_SIZE) begin
                        data <= file_byte(pidx);
                        pidx <= pidx + 1;
                    end else
                        data_valid <= 1'b0;
                end
            end else if (pidx < FILE_SIZE) begin
                data       <= file_byte(pidx);
                data_valid <= 1'b1;
                pidx       <= pidx + 1;
            end
        end
    end

    bmp_stream_parser #(
        .IMG_W (640),
        .IMG_H (LINES)
    ) u_bmp (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (bmp_start),
        .file_size       (file_size),
        .busy            (bmp_busy),
        .done            (bmp_done),
        .error           (bmp_error),
        .error_code      (bmp_error_code),
        .data            (data),
        .data_valid      (data_valid),
        .data_ready      (data_ready),
        .hold_done       (1'b0),
        .px_valid        (px_valid),
        .px_rgb          (px_rgb),
        .px_last         (px_last),
        .px_ready        (px_ready),
        .hdr_width       (),
        .hdr_height      (),
        .hdr_bpp         (),
        .hdr_data_offset ()
    );

    //------------------------------------------------------------------
    // frame buffer
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
    reg         reload;
    reg         img_active;
    reg         pix_de;
    reg         pix_frame_tog;
    reg  [10:0] hcount;
    reg  [5:0]  vcount;

    //------------------------------------------------------------------
    // small video timing model: 640 active of 800 pixels, LINES of
    // (LINES + 20) lines. It only exists so the display reader can finish
    // frames and apply its buffer swaps.
    //------------------------------------------------------------------
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
    ) dut (
        .clk           (clk),
        .mem_clk       (clk),
        .mem_clk_sft   (clk),
        .rst_n         (rst_n),
        .reload        (reload),
        .pattern_mode  (2'd0),
        .img_active    (img_active),
        .img_abort     (1'b0),
        .swap_req      (1'b0),
        .img_valid     (px_valid),
        .img_rgb       (px_rgb),
        .img_ready     (px_ready),
        .frame_ready   (frame_ready),
        .frame_loaded  (frame_loaded),
        .preload_ready (),
        .swap_done     (),
        .init_done     (init_done),
        .cal_fail      (cal_fail),
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
    // checks
    //------------------------------------------------------------------
    integer errors;
    integer checked;
    integer y, x;
    integer r;
    reg [23:0] want;
    reg [23:0] got;
    integer    pixels_in;
    reg [20:0] base;

    always @(negedge clk)
        if (rst_n && px_valid && px_ready)
            pixels_in = pixels_in + 1;

    initial begin
        // header
        for (x = 0; x < DATA_OFFSET; x = x + 1)
            hdr[x] = 8'h00;
        hdr[0]  = 8'h42;
        hdr[1]  = 8'h4D;
        hdr[10] = 8'd54;
        hdr[14] = 8'd40;
        hdr[18] = 8'd128;
        hdr[19] = 8'd2;
        hdr[22] = LINES;
        hdr[26] = 8'h01;
        hdr[28] = 8'd24;

        errors     = 0;
        checked    = 0;
        pixels_in  = 0;
        pidx       = 0;
        data_valid = 1'b0;
        bmp_start  = 1'b0;
        reload     = 1'b0;
        img_active = 1'b0;
        file_size  = FILE_SIZE;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        wait (init_done === 1'b1);
        repeat (20) @(posedge clk);

        @(negedge clk);
        img_active = 1'b1;
        reload     = 1'b1;
        bmp_start  = 1'b1;
        @(negedge clk);
        reload    = 1'b0;
        bmp_start = 1'b0;

        wait (bmp_done === 1'b1);
        wait (frame_loaded === 1'b1);
        repeat (5) @(posedge clk);

        // picture row y is file row LINES-1-y
        base = dut.wr_buf ? BUF1_BASE : BUF0_BASE;
        for (y = 0; y < LINES; y = y + 1) begin
            r = LINES - 1 - y;
            for (x = 0; x < 640; x = x + 1) begin
                want = {r[7:0], x[7:0], 8'h5A};
                got  = dut.u_sdram_ctrl.mem[base + y * 640 + x][23:0];
                checked = checked + 1;
                if (got !== want) begin
                    errors = errors + 1;
                    if (errors <= 12)
                        $display("MISMATCH y=%0d x=%0d got=%06h want=%06h",
                                 y, x, got, want);
                end
            end
        end

        $display("--------------------------------------------------");
        $display("tb_frame_store_bmp: checked=%0d errors=%0d", checked, errors);
        $display("tb_frame_store_bmp: pixels_in=%0d of %0d",
                 pixels_in, FRAME_WORDS);
        $display("tb_frame_store_bmp: bmp_error=%b code=%0d sdram_err=%0d",
                 bmp_error, bmp_error_code, g_model.n_err);

        if ((errors == 0) && (pixels_in == FRAME_WORDS) &&
            !bmp_error && (g_model.n_err == 0))
            $display("RESULT: PASS tb_frame_store_bmp");
        else
            $display("RESULT: FAIL tb_frame_store_bmp state=%0d", ld_state);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL tb_frame_store_bmp timeout state=%0d frame_loaded=%b bmp_busy=%b",
                 ld_state, frame_loaded, bmp_busy);
        $display("  img_active=%b data_valid=%b data_ready=%b pidx=%0d px_valid=%b px_ready=%b",
                 img_active, data_valid, data_ready, pidx, px_valid, px_ready);
        $display("  bmp busy=%b error=%b code=%0d parser_state=%0d skip_left=%0d consumed=%0d",
                 bmp_busy, bmp_error, bmp_error_code, u_bmp.state,
                 u_bmp.skip_left, u_bmp.consumed);
        $display("  bmp magic=%02h%02h hdr=%02h%02h last_data=%02h",
                 u_bmp.magic0, u_bmp.magic1, hdr[0], hdr[1], data);
        $display("  pending_load=%b write_pending=%b write_req=%b write_ack=%b write_finish=%b",
                 dut.pending_load, dut.write_pending, dut.write_req, dut.write_req_ack,
                 dut.write_finish);
        $finish;
    end

endmodule
