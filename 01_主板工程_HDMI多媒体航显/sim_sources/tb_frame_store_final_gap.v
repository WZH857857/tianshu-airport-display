`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store_final_gap.v - regression for the last-pixel wait
//
// The byte stream feeding a BMP parser is not continuous: after pixel
// FRAME_LAST-1 is accepted, img_valid drops while the parser consumes the
// final three bytes.  The loader must wait, not treat that gap as an early
// end of the file.  This test sends a 640 x 4 frame, intentionally leaves a
// long gap before the final pixel, and requires the frame to be committed.
//=============================================================================
module tb_frame_store_final_gap;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam [20:0]  BUF0_BASE   = 21'h000000;
    localparam [20:0]  BUF1_BASE   = 21'h001000;

    reg clk      = 1'b0;
    reg mem_clk  = 1'b0;
    reg rst_n    = 1'b0;

    always #10 clk     = ~clk;       // 50 MHz
    always #4  mem_clk = ~mem_clk;   // 125 MHz

    reg         reload;
    reg         img_active;
    reg         img_valid;
    reg  [23:0] img_rgb;
    wire        img_ready;
    wire        frame_ready;
    wire        img_frame_ready;
    wire        frame_loaded;
    wire        init_done;
    wire        cal_fail;
    wire [2:0]  ld_state;

    reg         pix_de;
    reg  [9:0]  pix_x;
    reg  [9:0]  pix_y;
    reg         pix_frame_tog;
    wire [23:0] pix_rgb;
    wire        pix_uf;

    wire [31:0] sdram_dq;

    integer p;
    integer errors;
    reg [23:0] got;
    reg [20:0] test_base;

    frame_store #(
        .FRAME_LINES (LINES),
        .FRAME_WORDS (FRAME_WORDS),
        .BUF0_BASE   (BUF0_BASE),
        .BUF1_BASE   (BUF1_BASE),
        .FIFO_AW     (8),
        .CMD_AW      (7)
    ) dut (
        .clk             (clk),
        .mem_clk         (mem_clk),
        .mem_clk_sft     (mem_clk),
        .rst_n           (rst_n),
        .reload          (reload),
        .pattern_mode    (2'd0),
        .img_active      (img_active),
        .img_abort       (1'b0),
        .swap_req        (1'b0),
        .img_valid       (img_valid),
        .img_rgb         (img_rgb),
        .img_ready       (img_ready),
        .frame_ready     (frame_ready),
        .img_frame_ready (img_frame_ready),
        .frame_loaded    (frame_loaded),
        .preload_ready   (),
        .swap_done       (),
        .init_done       (init_done),
        .cal_fail        (cal_fail),
        .cal_latency     (),
        .state_dbg       (ld_state),
        .sdram_clk       (),
        .sdram_ras_n     (),
        .sdram_cas_n     (),
        .sdram_we_n      (),
        .sdram_addr      (),
        .sdram_ba        (),
        .sdram_cs_n      (),
        .sdram_cke       (),
        .sdram_dm        (),
        .sdram_dq        (sdram_dq),
        .pix_clk         (mem_clk),
        .pix_de          (pix_de),
        .pix_x           (pix_x),
        .pix_y           (pix_y),
        .pix_frame_tog   (pix_frame_tog),
        .pix_rgb         (pix_rgb),
        .pix_underflow   (pix_uf)
    );

    // Keep the display FIFO draining.  The test only needs the swap to
    // complete, so the exact video timing of the active pixels is irrelevant.
    always @(posedge mem_clk or negedge rst_n) begin
        if (!rst_n) begin
            pix_de        <= 1'b0;
            pix_x         <= 10'd0;
            pix_y         <= 10'd0;
            pix_frame_tog <= 1'b0;
        end else begin
            pix_de <= 1'b1;
            if (pix_x == 10'd799) begin
                pix_x <= 10'd0;
                if (pix_y == 10'd524) begin
                    pix_y         <= 10'd0;
                    pix_frame_tog <= ~pix_frame_tog;
                end else
                    pix_y <= pix_y + 10'd1;
            end else
                pix_x <= pix_x + 10'd1;
        end
    end

    task send_pixel;
        input [23:0] rgb;
        begin
            @(negedge clk);
            img_rgb   = rgb;
            img_valid = 1'b1;
            wait (img_ready === 1'b1);
            @(posedge clk);
            @(negedge clk);
            img_valid = 1'b0;
        end
    endtask

    initial begin
        reload      = 1'b0;
        img_active  = 1'b0;
        img_valid   = 1'b0;
        img_rgb     = 24'd0;
        errors      = 0;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        wait (init_done === 1'b1);
        repeat (20) @(posedge clk);

        // Send everything except the final pixel.  After pixel
        // FRAME_WORDS-2 the loader is waiting with wcnt == FRAME_LAST.
        @(negedge clk);
        img_active = 1'b1;
        reload     = 1'b1;
        @(negedge clk);
        reload     = 1'b0;

        for (p = 0; p < (FRAME_WORDS - 1); p = p + 1)
            send_pixel(p[23:0]);

        // This gap used to make the old loader rebuild one pixel short.
        repeat (80) @(posedge clk);
        if (dut.wstate !== 2'd2 || dut.wr_count !== (FRAME_WORDS - 1)) begin
            errors = errors + 1;
            $display("ERROR: loader left the final-pixel wait: state=%0d wcnt=%0d",
                     dut.wstate, dut.wr_count);
        end

        send_pixel((FRAME_WORDS - 1));

        wait (img_frame_ready === 1'b1);
        wait (dut.have_img_p === 1'b1);
        repeat (50) @(posedge clk);

        // The BMP rows arrive bottom up: file row 0 is memory row 3.
        test_base = dut.wr_buf ? BUF1_BASE : BUF0_BASE;
        got = dut.u_sdram_ctrl.mem[test_base + 3 * 640 + 0][23:0];
        if (got !== 24'd0) begin
            errors = errors + 1;
            $display("ERROR: first pixel = %06h", got);
        end

        got = dut.u_sdram_ctrl.mem[test_base + 3 * 640 + 639][23:0];
        if (got !== 24'd639) begin
            errors = errors + 1;
            $display("ERROR: bottom row end = %06h", got);
        end

        got = dut.u_sdram_ctrl.mem[test_base + 0 * 640 + 639][23:0];
        if (got !== (FRAME_WORDS - 1)) begin
            errors = errors + 1;
            $display("ERROR: final pixel = %06h want=%06h", got,
                     (FRAME_WORDS - 1));
        end

        if (frame_ready !== 1'b1) begin
            errors = errors + 1;
            $display("ERROR: frame_ready did not assert");
        end

        if (dut.have_img_p !== 1'b1) begin
            errors = errors + 1;
            $display("ERROR: display reader did not arm");
        end

        $display("--------------------------------------------------");
        $display("tb_frame_store_final_gap: img_frame_ready=%b frame_loaded=%b have_img=%b",
                 img_frame_ready, frame_loaded, dut.have_img_p);
        $display("tb_frame_store_final_gap: final state=%0d wcnt=%0d",
                 dut.wstate, dut.wr_count);
        if (errors == 0)
            $display("RESULT: PASS tb_frame_store_final_gap");
        else
            $display("RESULT: FAIL tb_frame_store_final_gap errors=%0d", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("RESULT: FAIL tb_frame_store_final_gap timeout wstate=%0d wcnt=%0d img_ready=%b",
                 dut.wstate, dut.wr_count, img_frame_ready);
        $finish;
    end

endmodule
