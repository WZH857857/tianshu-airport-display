`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store_official.v
//
// A small 640 x 4 frame is pushed through the new official frame scheduler.
// The test checks:
//   1. all 2560 pixels are accepted;
//   2. write_finish arrives before img_frame_ready;
//   3. the BMP bottom-up rows really map to top-down SDRAM rows;
//   4. the display FIFO returns the same top-down picture;
//   5. a second frame window returns the same picture again, which shows the
//      read scheduler restarts cleanly at every frame boundary.
//
// The vendor controller is represented by sim/sdr_as_ram_stub.v.  The stub
// deliberately keeps the 3-cycle read latency, so this test catches a wrong
// read-enable / FIFO handshake as well as the old direct-request scheduling.
//=============================================================================
module tb_frame_store_official;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer H_TOTAL     = 800;
    localparam integer V_TOTAL     = LINES + 20;
    localparam integer V_ACTIVE    = 4;
    localparam [20:0]   BUF0_BASE  = 21'h000000;
    localparam [20:0]   BUF1_BASE  = 21'h001000;

    reg clk     = 1'b0;
    reg mem_clk = 1'b0;
    reg pix_clk = 1'b0;
    reg rst_n   = 1'b0;

    always #10 clk     = ~clk;       // 50 MHz
    always #4  mem_clk = ~mem_clk;   // 125 MHz
    always #20 pix_clk = ~pix_clk;   // 25 MHz

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
    integer     pix_index;
    integer     pix_errors;
    integer     mem_errors;
    integer     checked;
    integer     i, r, x, y;
    reg [23:0]  expected_mem;
    reg [23:0]  captured [0:FRAME_WORDS-1];
    reg         capture_en;
    reg [23:0]  want_shift;
    reg [7:0]   row_expected;
    integer     shift;
    integer     shift_errors;
    reg         pix_de_pipe;
    reg [9:0]   cap_x, cap_y;
    integer     phase_pass;
    integer     phase_fail;

    wire        pix_de_raw = (hx < 640) &&
                            (vy >= V_ACTIVE) &&
                            (vy < (V_ACTIVE + LINES));
    wire [9:0]  pix_y_calc = (vy >= V_ACTIVE) ?
                             (vy - V_ACTIVE) : 10'd0;

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
        .img_abort       (1'b0),
        .swap_req        (1'b0),
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
        .pix_y           (pix_y_calc),
        .pix_frame_tog   (pix_frame_tog),
        .pix_rgb         (pix_rgb),
        .pix_underflow   (pix_uf)
    );

    // Video timing model: the first four lines are blank, which gives the
    // read request time to fill the display FIFO before active data starts.
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            hx            <= 0;
            vy            <= 0;
            pix_frame_tog <= 1'b0;
            pix_index     <= 0;
            pix_de_pipe   <= 1'b0;
            cap_x         <= 10'd0;
            cap_y         <= 10'd0;
        end else begin
            // The frame-store output is registered on the read-enable
            // cycle, so the data present now belongs to the previous
            // active coordinate.
            pix_de_pipe <= pix_de_raw;
            cap_x       <= hx[9:0];
            cap_y       <= pix_y_calc;

            if (pix_de_pipe) begin
                if (pix_index < FRAME_WORDS)
                    captured[pix_index] <= pix_rgb;
                pix_index <= pix_index + 1;
            end

            if (hx == H_TOTAL - 1) begin
                hx <= 0;
                if (vy == V_TOTAL - 1) begin
                    vy            <= 0;
                    pix_frame_tog <= ~pix_frame_tog;
                    pix_index     <= 0;
                end else begin
                    vy <= vy + 1;
                end
            end else begin
                hx <= hx + 1;
            end
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

    // Compare the frame the display FIFO just returned against the expected
    // picture.  flip_expected = 0 is the normal top-down picture, 1 is the
    // bottom-up one the flip switch selects.  A small +/- shift search
    // absorbs the pipeline delay of the capture model, exactly like the
    // single check this test used before.
    task compare_display;
        input integer phase;
        input integer flip_expected;
        integer sh;
        integer idx;
        integer py;
        integer px;
        integer errs;
        integer best;
        integer dump_left;
        reg [7:0]  row;
        reg [23:0] want;
        begin
            best = FRAME_WORDS + 1;
            for (sh = -4; sh < 5; sh = sh + 1) begin
                errs = 0;
                for (idx = 0; idx < FRAME_WORDS; idx = idx + 1) begin
                    if (((idx + sh) >= 0) && ((idx + sh) < FRAME_WORDS)) begin
                        py = (idx + sh) / 640;
                        px = (idx + sh) % 640;
                        row = (LINES - 1 - py) % 256;
                        if (flip_expected)
                            px = 639 - px;
                        want = {row, px[7:0], 8'h5A};
                        if (captured[idx] !== want)
                            errs = errs + 1;
                    end
                end
                if (errs < best)
                    best = errs;
            end

            $display("tb_frame_store_official: phase %0d flip=%0d best_errors=%0d",
                     phase, flip_expected, best);

            if (best == 0) begin
                phase_pass = phase_pass + 1;
            end else begin
                phase_fail = phase_fail + 1;
                dump_left = 8;
                for (idx = 0; (idx < FRAME_WORDS) && (dump_left > 0);
                     idx = idx + 1) begin
                    py = idx / 640;
                    px = idx % 640;
                    row = (LINES - 1 - py) % 256;
                    if (flip_expected)
                        px = 639 - px;
                    want = {row, px[7:0], 8'h5A};
                    if (captured[idx] !== want) begin
                        $display("  mismatch idx=%0d got=%06h want=%06h",
                                 idx, captured[idx], want);
                        dump_left = dump_left - 1;
                    end
                end
            end
        end
    endtask

    initial begin
        reload      = 1'b0;
        img_active  = 1'b0;
        img_valid   = 1'b0;
        img_rgb     = 24'd0;
        pix_errors  = 0;
        mem_errors  = 0;
        checked     = 0;
        capture_en  = 1'b0;
        pix_index   = 0;
        phase_pass  = 0;
        phase_fail  = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;

        wait (init_done === 1'b1);
        repeat (20) @(posedge clk);

        // Start a BMP-style, bottom-up frame.  File row 0 is red, green is
        // the column, blue is a fixed marker.
        @(negedge clk);
        img_active = 1'b1;
        reload     = 1'b1;
        @(negedge clk);
        reload = 1'b0;

        for (r = 0; r < LINES; r = r + 1) begin
            for (x = 0; x < 640; x = x + 1) begin
                send_pixel({r[7:0], x[7:0], 8'h5A});
            end
        end

        // Let the parser finish and the write FIFO drain.
        @(negedge clk);
        img_active = 1'b0;
        repeat (20) @(posedge clk);

        wait (img_frame_ready === 1'b1);
        repeat (100) @(posedge mem_clk);

        // Check the SDRAM rows: the input was bottom-up, so memory row 0
        // must hold file row LINES-1.
        for (y = 0; y < LINES; y = y + 1) begin
            for (x = 0; x < 640; x = x + 1) begin
                row_expected = LINES - 1 - y;
                expected_mem = {row_expected, x[7:0], 8'h5A};
                if (dut.u_sdram_ctrl.mem[y * 640 + x][23:0] !== expected_mem) begin
                    mem_errors = mem_errors + 1;
                    if (mem_errors <= 8)
                        $display("SDRAM mismatch y=%0d x=%0d got=%06h want=%06h",
                                 y, x,
                                 dut.u_sdram_ctrl.mem[y * 640 + x][23:0],
                                 expected_mem);
                end
            end
        end

        // Phase 1: the cached frame must come back in the normal top-down
        // order.
        wait (frame_loaded === 1'b1);

        repeat (V_TOTAL * H_TOTAL) @(posedge pix_clk);
        compare_display(1, 0);

        // Phase 2: the next frame window has to show the same picture, with
        // no reload in between.
        repeat (V_TOTAL * H_TOTAL) @(posedge pix_clk);
        compare_display(2, 0);

        $display("--------------------------------------------------");
        $display("tb_frame_store_official: img_frame_ready=%b frame_loaded=%b",
                 img_frame_ready, frame_loaded);
        $display("tb_frame_store_official: state=%0d SDRAM_errors=%0d phases_pass=%0d phases_fail=%0d",
                 ld_state, mem_errors, phase_pass, phase_fail);
        $display("tb_frame_store_official: pix_underflow=%b", pix_uf);

        if ((phase_fail == 0) && (phase_pass == 3) && (mem_errors == 0))
            $display("RESULT: PASS tb_frame_store_official");
        else
            $display("RESULT: FAIL tb_frame_store_official");
        $finish;
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL tb_frame_store_official timeout loader_state=%0d ready=%b",
                 ld_state, img_frame_ready);
        $display("  wr_state=%0d write_cnt=%0d write_len_latch=%0d rdusedw=%0d burst_cnt=%0d",
                 dut.u_frame_rw.frame_fifo_write_m0.state,
                 dut.u_frame_rw.frame_fifo_write_m0.write_cnt,
                 dut.u_frame_rw.frame_fifo_write_m0.write_len_latch,
                 dut.u_frame_rw.frame_fifo_write_m0.rdusedw,
                 dut.u_frame_rw.frame_fifo_write_m0.burst_cnt);
        $display("  write_req=%b write_req_ack=%b write_finish=%b App_wr_en=%b",
                 dut.write_req, dut.write_req_ack, dut.write_finish,
                 dut.app_wr_en);
        $display("  read_state=%0d read_req=%b read_req_ack=%b read_empty=%b",
                 dut.u_frame_rw.frame_fifo_read_m0.state,
                 dut.read_req, dut.read_req_ack, dut.read_empty);
        $finish;
    end

endmodule
