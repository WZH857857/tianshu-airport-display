`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store_cache.v
//
// Three complete frames are written into three resident SDRAM slots.  The
// first frame publishes automatically; later frames remain off screen until
// swap_req selects the next ready slot.
//=============================================================================
module tb_frame_store_cache;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer H_TOTAL     = 800;
    localparam integer V_TOTAL     = LINES + 4;
    localparam integer V_ACTIVE    = 4;
    localparam integer BUF_COUNT   = 3;
    localparam [20:0] BUF_STRIDE   = 21'h001000;

    localparam [23:0] COLOR_A = 24'hA00000;
    localparam [23:0] COLOR_B = 24'h00B000;
    localparam [23:0] COLOR_C = 24'h0000C0;

    reg clk     = 1'b0;
    reg mem_clk = 1'b0;
    reg pix_clk = 1'b0;
    reg rst_n   = 1'b0;

    always #10 clk     = ~clk;
    always #4  mem_clk = ~mem_clk;
    always #20 pix_clk = ~pix_clk;

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
    wire        init_done;
    wire        cal_fail;
    wire [2:0]  ld_state;
    wire [BUF_COUNT-1:0] cache_ready;
    wire [2:0]  stored_buf;
    wire        stored_pulse;

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
    integer     errors;

    reg [23:0] captured_pixels [0:FRAME_WORDS-1];

    wire pix_de_raw = (hx < 640) &&
                      (vy >= V_ACTIVE) &&
                      (vy < (V_ACTIVE + LINES));

    frame_store #(
        .FRAME_LINES (LINES),
        .FRAME_WORDS (FRAME_WORDS),
        .BUF0_BASE   (21'h000000),
        .BUF1_BASE   (21'h001000),
        .CACHE_MODE  (1),
        .BUF_COUNT   (BUF_COUNT),
        .BUF_STRIDE  (BUF_STRIDE),
        .DEFER_SWAP  (0)
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
        .emg_slot        (1'b0),
        .emg_slot_sel    (1'b0),
        .img_valid       (img_valid),
        .img_rgb         (img_rgb),
        .img_ready       (img_ready),
        .frame_ready     (frame_ready),
        .img_frame_ready (img_frame_ready),
        .frame_loaded    (frame_loaded),
        .preload_ready   (),
        .swap_done       (),
        .cache_ready     (cache_ready),
        .stored_buf      (stored_buf),
        .stored_pulse    (stored_pulse),
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
            capture_en    <= 1'b0;
            captured      <= 0;
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

            if (capture_en && pix_de_pipe && captured < FRAME_WORDS) begin
                captured_pixels[captured] <= pix_rgb;
                captured <= captured + 1;
            end
        end
    end

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)
            loaded_frames <= 0;
        else if (frame_loaded)
            loaded_frames <= loaded_frames + 1;
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

    task wait_stored;
        integer n;
        begin
            n = 0;
            while (!stored_pulse && n < 100000) begin
                @(posedge clk);
                n = n + 1;
            end
            if (!stored_pulse) begin
                $display("FAIL stored_pulse timeout");
                errors = errors + 1;
            end
        end
    endtask

    task wait_frame_start;
        begin
            @(posedge pix_clk);
            while (!(hx == 0 && vy == 0))
                @(posedge pix_clk);
        end
    endtask

    task capture_frame;
        input [23:0] expected;
        input [8*16-1:0] label;
        integer i;
        integer bad;
        integer count_a;
        integer count_b;
        integer count_c;
        integer count_other;
        begin
            wait_frame_start();
            captured   = 0;
            capture_en = 1'b1;
            while (captured < FRAME_WORDS)
                @(posedge pix_clk);
            capture_en = 1'b0;

            bad = 0;
            count_a = 0;
            count_b = 0;
            count_c = 0;
            count_other = 0;
            for (i = 0; i < FRAME_WORDS; i = i + 1) begin
                if (captured_pixels[i] !== expected)
                    bad = bad + 1;
                if (captured_pixels[i] === COLOR_A)
                    count_a = count_a + 1;
                else if (captured_pixels[i] === COLOR_B)
                    count_b = count_b + 1;
                else if (captured_pixels[i] === COLOR_C)
                    count_c = count_c + 1;
                else
                    count_other = count_other + 1;
            end
            if (bad != 0) begin
                $display("FAIL %0s has %0d wrong pixels first=%h counts A/B/C/other=%0d/%0d/%0d/%0d",
                         label, bad, captured_pixels[0],
                         count_a, count_b, count_c, count_other);
                errors = errors + 1;
            end else begin
                $display("OK   %0s captured correctly", label);
            end
        end
    endtask

    task request_swap;
        input integer expected_loaded;
        integer n;
        begin
            @(negedge clk);
            swap_req = 1'b1;
            @(negedge clk);
            swap_req = 1'b0;

            n = 0;
            while (loaded_frames < expected_loaded && n < 100000) begin
                @(posedge pix_clk);
                n = n + 1;
            end
            if (loaded_frames < expected_loaded) begin
                $display("FAIL swap timeout loaded=%0d expected=%0d",
                         loaded_frames, expected_loaded);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        reload     = 1'b0;
        img_active = 1'b0;
        img_abort  = 1'b0;
        img_valid  = 1'b0;
        img_rgb    = 24'd0;
        swap_req   = 1'b0;
        errors     = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        wait (init_done === 1'b1);
        repeat (20) @(posedge clk);

        load_frame(COLOR_A);
        wait_stored();
        wait (loaded_frames >= 1);
        repeat (5) @(posedge clk);
        if (cache_ready !== 3'b001 || stored_buf !== 3'd0) begin
            $display("FAIL first cache slot ready=%b stored=%0d",
                     cache_ready, stored_buf);
            errors = errors + 1;
        end
        capture_frame(COLOR_A, "first frame");

        load_frame(COLOR_B);
        wait_stored();
        repeat (5) @(posedge clk);
        if (cache_ready !== 3'b011 || stored_buf !== 3'd1) begin
            $display("FAIL second cache slot ready=%b stored=%0d",
                     cache_ready, stored_buf);
            errors = errors + 1;
        end
        capture_frame(COLOR_A, "second cache stays off screen");

        load_frame(COLOR_C);
        wait_stored();
        repeat (5) @(posedge clk);
        if (cache_ready !== 3'b111 || stored_buf !== 3'd2) begin
            $display("FAIL third cache slot ready=%b stored=%0d",
                     cache_ready, stored_buf);
            errors = errors + 1;
        end
        capture_frame(COLOR_A, "third cache stays off screen");

        request_swap(2);
        capture_frame(COLOR_B, "swap to slot 1");

        request_swap(3);
        capture_frame(COLOR_C, "swap to slot 2");

        request_swap(4);
        capture_frame(COLOR_A, "swap wraps to slot 0");

        if (pix_uf) begin
            $display("FAIL pixel underflow during cache test");
            errors = errors + 1;
        end
        if (frame_ready !== 1'b1 || img_frame_ready !== 1'b1) begin
            $display("FAIL cache status frame_ready=%b image_ready=%b",
                     frame_ready, img_frame_ready);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_frame_store_cache");
        else
            $display("RESULT: FAIL tb_frame_store_cache errors=%0d", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("RESULT: FAIL tb_frame_store_cache timeout state=%0d loaded=%0d",
                 ld_state, loaded_frames);
        $finish;
    end

endmodule
