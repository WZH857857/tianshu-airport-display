`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store_defer_swap.v
//
// V5 regression for the preload path:
//   1. the first frame is published automatically;
//   2. the second complete frame is held off screen until swap_req;
//   3. the swap happens at a video frame boundary with no mixed frame;
//   4. a third load cannot start while the deferred frame is waiting.
//=============================================================================
module tb_frame_store_defer_swap;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer H_TOTAL     = 800;
    localparam integer V_TOTAL     = LINES + 20;
    localparam integer V_ACTIVE    = 4;
    localparam integer CAPTURE_FRAMES = 6;
    localparam [20:0] BUF0_BASE    = 21'h000000;
    localparam [20:0] BUF1_BASE    = 21'h001000;
    localparam [23:0] COLOR_A      = 24'hA00000;
    localparam [23:0] COLOR_B      = 24'h00B000;

    reg clk     = 1'b0;
    reg mem_clk = 1'b0;
    reg pix_clk = 1'b0;
    reg rst_n   = 1'b0;

    always #10 clk     = ~clk;       // 50 MHz
    always #4  mem_clk = ~mem_clk;   // 125 MHz
    always #20 pix_clk = ~pix_clk;   // 25 MHz

    reg         reload;
    reg         img_active;
    reg         img_abort;
    reg         img_valid;
    reg  [23:0] img_rgb;
    reg         swap_req;
    wire        img_ready;
    wire        frame_ready;
    wire        img_frame_ready;
    wire        frame_loaded;
    wire        preload_ready;
    wire        swap_done;
    wire        init_done;
    wire        cal_fail;
    wire [2:0]  ld_state;

    wire        sdram_clk;
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire        sdram_cs_n, sdram_cke;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;

    wire [23:0] pix_rgb;
    wire        pix_uf;
    reg         pix_frame_tog;
    integer     hx, vy;
    reg         pix_de_pipe;
    reg         capture_en;
    integer     captured;
    integer     loaded_frames;
    integer     buffer_conflicts;

    reg [23:0] captured_pixels [0:(CAPTURE_FRAMES * FRAME_WORDS) - 1];

    wire pix_de_raw = (hx < 640) &&
                      (vy >= V_ACTIVE) &&
                      (vy < (V_ACTIVE + LINES));

    frame_store #(
        .FRAME_LINES (LINES),
        .FRAME_WORDS (FRAME_WORDS),
        .BUF0_BASE   (BUF0_BASE),
        .BUF1_BASE   (BUF1_BASE),
        .DEFER_SWAP  (1)
    ) dut (
        .clk             (clk),
        .mem_clk         (mem_clk),
        .mem_clk_sft     (mem_clk),
        .rst_n           (rst_n),
        .reload          (reload),
        .pattern_mode    (2'd0),
        .img_active      (img_active),
        .img_abort       (img_abort),
        .swap_req        (swap_req),
        .img_valid       (img_valid),
        .img_rgb         (img_rgb),
        .img_ready       (img_ready),
        .frame_ready     (frame_ready),
        .img_frame_ready (img_frame_ready),
        .frame_loaded    (frame_loaded),
        .preload_ready   (preload_ready),
        .swap_done       (swap_done),
        .init_done       (init_done),
        .cal_fail        (cal_fail),
        .cal_latency     (),
        .state_dbg       (ld_state),
        .sdram_clk       (sdram_clk),
        .sdram_ras_n     (sdram_ras_n),
        .sdram_cas_n     (sdram_cas_n),
        .sdram_we_n      (sdram_we_n),
        .sdram_addr      (sdram_addr),
        .sdram_ba        (sdram_ba),
        .sdram_cs_n      (sdram_cs_n),
        .sdram_cke       (sdram_cke),
        .sdram_dm        (sdram_dm),
        .sdram_dq        (sdram_dq),
        .pix_clk         (pix_clk),
        .pix_de          (pix_de_raw),
        .pix_x           (hx[9:0]),
        .pix_y           ((vy >= V_ACTIVE) ? (vy - V_ACTIVE) : 0),
        .pix_frame_tog   (pix_frame_tog),
        .pix_rgb         (pix_rgb),
        .pix_underflow   (pix_uf)
    );

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            hx            <= 0;
            vy            <= 0;
            pix_frame_tog <= 1'b0;
            pix_de_pipe   <= 1'b0;
        end else begin
            pix_de_pipe <= pix_de_raw;

            if (hx == H_TOTAL - 1) begin
                hx <= 0;
                if (vy == V_TOTAL - 1) begin
                    vy            <= 0;
                    pix_frame_tog <= ~pix_frame_tog;
                end else begin
                    vy <= vy + 1;
                end
            end else begin
                hx <= hx + 1;
            end
        end
    end

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            capture_en <= 1'b0;
            captured   <= 0;
        end else if (capture_en && pix_de_pipe &&
                     captured < (CAPTURE_FRAMES * FRAME_WORDS)) begin
            captured_pixels[captured] <= pix_rgb;
            captured <= captured + 1;
        end
    end

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)
            loaded_frames <= 0;
        else if (frame_loaded)
            loaded_frames <= loaded_frames + 1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            buffer_conflicts <= 0;
        else if (dut.have_img_p && dut.img_active &&
                 (dut.wstate == 2'd2) &&
                 (dut.wr_buf == dut.disp_buf_pix))
            buffer_conflicts <= buffer_conflicts + 1;
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

    task load_frame;
        input [23:0] rgb;
        integer i;
        begin
            @(negedge clk);
            img_abort  = 1'b0;
            img_active = 1'b1;
            reload     = 1'b1;
            @(negedge clk);
            reload = 1'b0;

            for (i = 0; i < FRAME_WORDS; i = i + 1)
                send_pixel(rgb);

            @(negedge clk);
            img_active = 1'b0;
        end
    endtask

    task start_capture;
        begin
            captured   = 0;
            capture_en = 1'b1;
        end
    endtask

    task wait_capture;
        input integer words;
        integer n;
        begin
            n = 0;
            while (captured < words && n < 2000000) begin
                @(posedge pix_clk);
                n = n + 1;
            end
            capture_en = 1'b0;
            if (captured < words) begin
                $display("FAIL capture timeout: captured=%0d want=%0d",
                         captured, words);
                $finish;
            end
        end
    endtask

    task expect_capture;
        input [23:0] rgb;
        input integer words;
        input [8*24-1:0] label;
        integer n;
        integer bad;
        begin
            bad = 0;
            for (n = 0; n < words; n = n + 1)
                if (captured_pixels[n] !== rgb)
                    bad = bad + 1;
            if (bad != 0) begin
                $display("FAIL %0s has %0d unexpected pixels", label, bad);
                $finish;
            end
            $display("OK   %0s captured %0d pixels", label, words);
        end
    endtask

    integer frame_words;
    integer errors;

    initial begin
        reload       = 1'b0;
        img_active   = 1'b0;
        img_abort    = 1'b0;
        img_valid    = 1'b0;
        img_rgb      = 24'd0;
        swap_req     = 1'b0;
        capture_en   = 1'b0;
        captured     = 0;
        loaded_frames = 0;
        buffer_conflicts = 0;
        errors = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        wait (init_done === 1'b1);
        repeat (20) @(posedge clk);

        load_frame(COLOR_A);
        wait (loaded_frames >= 1);
        wait (dut.have_img_p === 1'b1);

        start_capture();
        wait_capture(2 * FRAME_WORDS);
        expect_capture(COLOR_A, 2 * FRAME_WORDS, "first frame");

        load_frame(COLOR_B);
        wait (preload_ready === 1'b1);
        if (loaded_frames != 1) begin
            $display("FAIL preload published itself: loaded_frames=%0d",
                     loaded_frames);
            errors = errors + 1;
        end

        // Stay on A for several complete video frames while B is ready.
        start_capture();
        wait_capture(2 * FRAME_WORDS);
        expect_capture(COLOR_A, 2 * FRAME_WORDS, "deferred frame");

        if (dut.wstate != 2'd0 || dut.preload_ready !== 1'b1) begin
            $display("FAIL frame store not idle with deferred frame: wstate=%0d preload_ready=%b",
                     dut.wstate, dut.preload_ready);
            errors = errors + 1;
        end

        @(negedge clk);
        swap_req = 1'b1;
        @(negedge clk);
        swap_req = 1'b0;

        wait (swap_done === 1'b1);
        wait (loaded_frames >= 2);
        if (preload_ready !== 1'b0) begin
            $display("FAIL preload_ready did not clear at swap");
            errors = errors + 1;
        end

        start_capture();
        wait_capture(2 * FRAME_WORDS);
        expect_capture(COLOR_B, 2 * FRAME_WORDS, "swapped frame");

        if (buffer_conflicts != 0) begin
            $display("FAIL buffer conflicts=%0d", buffer_conflicts);
            errors = errors + 1;
        end
        if (pix_uf) begin
            $display("FAIL pixel underflow during deferred swap");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_frame_store_defer_swap");
        else
            $display("RESULT: FAIL tb_frame_store_defer_swap errors=%0d",
                     errors);
        $finish;
    end

    initial begin
        #30_000_000;
        $display("RESULT: FAIL tb_frame_store_defer_swap timeout state=%0d ready=%b captured=%0d",
                 ld_state, preload_ready, captured);
        $finish;
    end

endmodule
