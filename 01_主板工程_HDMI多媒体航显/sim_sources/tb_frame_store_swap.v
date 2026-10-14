`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store_swap.v
//
// Load two different full frames into the two SDRAM buffers and check that the
// display output changes from one complete frame to the other at a video frame
// boundary.  A frame that contains both colours would mean that the display is
// looking at the buffer while it is still being written.
//=============================================================================
module tb_frame_store_swap;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer H_TOTAL     = 800;
    localparam integer V_TOTAL     = LINES + 20;
    localparam integer V_ACTIVE    = 4;
    localparam integer CAPTURE_FRAMES = 7;
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
    wire        img_ready;
    wire        frame_ready;
    wire        img_frame_ready;
    wire        frame_loaded;
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
    integer     buffer_conflicts;
    integer     second_buffer_writes;
    integer     abort_ok;
    reg [23:0]  captured_pixels [0:(CAPTURE_FRAMES * FRAME_WORDS) - 1];

    wire pix_de_raw = (hx < 640) &&
                      (vy >= V_ACTIVE) &&
                      (vy < (V_ACTIVE + LINES));

    frame_store #(
        .FRAME_LINES (LINES),
        .FRAME_WORDS (FRAME_WORDS),
        .BUF0_BASE   (BUF0_BASE),
        .BUF1_BASE   (BUF1_BASE)
    ) dut (
        .clk             (clk),
        .mem_clk         (mem_clk),
        .mem_clk_sft     (mem_clk),
        .rst_n           (rst_n),
        .reload          (reload),
        .pattern_mode    (2'd0),
        .img_active      (img_active),
        .img_abort       (img_abort),
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
        .pix_y           (vy >= V_ACTIVE ? (vy - V_ACTIVE) : 0),
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

    task load_partial_and_abort;
        input [23:0] rgb;
        input integer pixels;
        integer i;
        begin
            @(negedge clk);
            img_abort  = 1'b0;
            img_active = 1'b1;
            reload     = 1'b1;
            @(negedge clk);
            reload = 1'b0;

            for (i = 0; i < pixels; i = i + 1)
                send_pixel(rgb);

            @(negedge clk);
            img_abort = 1'b1;
            @(negedge clk);
            img_abort  = 1'b0;
            img_active = 1'b0;
        end
    endtask

    task load_full_and_abort_at_finish;
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

            // The last pixel has now been accepted and wr_done is set, but
            // the SDRAM scheduler may still be finishing the FIFO.  Abort
            // here to verify that the pending commit is discarded.
            @(negedge clk);
            img_abort = 1'b1;
            @(negedge clk);
            img_abort  = 1'b0;
            img_active = 1'b0;
        end
    endtask

    integer frame;
    integer i;
    integer non_a;
    integer non_b;
    integer mixed_frames;
    integer loaded_frames;
    integer loaded_frames_before_abort;

    // frame_loaded is generated in the 25 MHz pixel clock domain.  Sampling
    // it with the 50 MHz loader clock would count the same pulse twice.
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)
            loaded_frames <= 0;
        else if (frame_loaded)
            loaded_frames <= loaded_frames + 1;
    end

    always @(posedge pix_clk) begin
        if (frame_loaded)
            $display("DBG frame_loaded loaded=%0d wstate=%0d wr_count=%0d img_active=%b abort=%b ready_tog=%b",
                     loaded_frames, dut.wstate, dut.wr_count, img_active,
                     img_abort, dut.ready_tog);
    end

    // The real 640x480 transfer lasts about 2.6 s, so the display scans the
    // old frame many times while the new frame is still being written.  This
    // short functional test inspects the selected buffer index directly and
    // catches a write into the live buffer even before pixel mixing occurs.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buffer_conflicts     <= 0;
            second_buffer_writes <= 0;
        end else begin
            if (dut.have_img_p && dut.img_active &&
                (dut.wstate == 2'd2) &&
                (dut.wr_buf == dut.disp_buf_pix))
                buffer_conflicts <= buffer_conflicts + 1;

            if (dut.img_active && !dut.wr_done &&
                (dut.wstate == 2'd2) && (dut.wr_buf == 2'd1))
                second_buffer_writes <= second_buffer_writes + 1;
        end
    end

    initial begin
        reload     = 1'b0;
        img_active = 1'b0;
        img_abort  = 1'b0;
        img_valid  = 1'b0;
        img_rgb    = 24'd0;
        capture_en = 1'b0;
        captured   = 0;
        loaded_frames = 0;
        buffer_conflicts = 0;
        second_buffer_writes = 0;
        abort_ok = 1;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;

        wait (init_done === 1'b1);
        repeat (20) @(posedge clk);

        load_frame(COLOR_A);
        wait (loaded_frames >= 1);
        capture_en = 1'b1;

        wait (captured >= (3 * FRAME_WORDS));

        // Exercise the end-of-transfer race: a skip is asserted after the
        // last pixel is handed to the frame store but before write_finish.
        loaded_frames_before_abort = loaded_frames;
        load_full_and_abort_at_finish(24'h654321);
        repeat (20) @(posedge clk);
        if ((loaded_frames != loaded_frames_before_abort) ||
            (dut.wstate != 2'd0) || (dut.wr_count != 19'd0)) begin
            $display("FAIL end-of-frame abort: loaded_frames=%0d before=%0d wstate=%0d wr_count=%0d",
                     loaded_frames, loaded_frames_before_abort,
                     dut.wstate, dut.wr_count);
            abort_ok = 0;
        end else begin
            $display("OK   end-of-frame abort discarded pending commit");
        end

        // Abort a half-loaded frame and verify that the write side returns to
        // a clean idle state without committing or mixing the partial frame.
        loaded_frames_before_abort = loaded_frames;
        load_partial_and_abort(24'h123456, FRAME_WORDS / 2);
        repeat (20) @(posedge clk);
        if ((loaded_frames != loaded_frames_before_abort) ||
            (dut.wstate != 2'd0) ||
            (dut.wr_count != 19'd0)) begin
            $display("FAIL abort cleanup: loaded_frames=%0d before=%0d wstate=%0d wr_count=%0d",
                     loaded_frames, loaded_frames_before_abort,
                     dut.wstate, dut.wr_count);
            abort_ok = 0;
        end else begin
            $display("OK   partial frame aborted; write side is idle");
        end

        load_frame(COLOR_B);
        wait (loaded_frames >= (loaded_frames_before_abort + 1));
        wait (captured >= (CAPTURE_FRAMES * FRAME_WORDS));

        non_a       = 0;
        non_b       = 0;
        mixed_frames = 0;

        for (frame = 0; frame < CAPTURE_FRAMES; frame = frame + 1) begin
            integer has_a;
            integer has_b;
            integer has_other;
            has_a     = 0;
            has_b     = 0;
            has_other = 0;

            for (i = 0; i < FRAME_WORDS; i = i + 1) begin
                if (captured_pixels[frame * FRAME_WORDS + i] === COLOR_A)
                    has_a = 1;
                else if (captured_pixels[frame * FRAME_WORDS + i] === COLOR_B)
                    has_b = 1;
                else
                    has_other = 1;
            end

            if (has_a && has_b) begin
                mixed_frames = mixed_frames + 1;
                $display("MIXED frame=%0d", frame);
            end
            if (!has_a && !has_b)
                non_a = non_a + 1;
            if (has_other)
                non_b = non_b + 1;
        end

        $display("--------------------------------------------------");
        $display("tb_frame_store_swap: frame_ready=%b img_frame_ready=%b underflow=%b",
                 frame_ready, img_frame_ready, pix_uf);
        $display("tb_frame_store_swap: capture_frames=%0d mixed=%0d other=%0d no_colour=%0d",
                 CAPTURE_FRAMES, mixed_frames, non_b, non_a);
        $display("tb_frame_store_swap: buffer_conflicts=%0d second_buffer_writes=%0d",
                 buffer_conflicts, second_buffer_writes);

        if ((mixed_frames == 0) && (non_b == 0) &&
            (buffer_conflicts == 0) && (second_buffer_writes != 0) &&
            (abort_ok != 0))
            $display("RESULT: PASS tb_frame_store_swap");
        else
            $display("RESULT: FAIL tb_frame_store_swap");
        $finish;
    end

    initial begin
        #20_000_000;
        $display("RESULT: FAIL tb_frame_store_swap timeout state=%0d ready=%b captured=%0d",
                 ld_state, img_frame_ready, captured);
        $finish;
    end

endmodule
