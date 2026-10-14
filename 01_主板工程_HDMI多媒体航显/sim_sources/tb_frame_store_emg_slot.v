`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store_emg_slot.v
//
// Reproduces the production slot layout around emergency placards without
// leaving the 2M x 32 word SDRAM address space:
//
//   1. six normal pictures fill slots 0..5 (the ring wraps back to 0)
//   2. the fire placard borrows a ring slot that is not on screen
//   3. the exit preload borrows the slot after the fire placard even if the
//      fire swap has not reached the display yet
//   4. the alarm clears and the first normal load restarts the ring one slot
//      past the placard on screen, invalidating every old slot
//   5. after the rebuild every slot holds a normal picture and the first
//      rebuilt picture is the one that reaches the display first
//
// After the resume swap the displayed frame must be a normal picture again
// and every slot must hold the latest picture written into it.
//=============================================================================
module tb_frame_store_emg_slot;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer H_TOTAL     = 800;
    localparam integer V_TOTAL     = LINES + 4;
    localparam integer V_ACTIVE    = 4;
    localparam integer BUF_COUNT   = 6;
    // Production spacing: six slots at 0x50000 words each.  Slots 6/7 would
    // start past 0x200000, which is exactly the overflow this bench guards
    // against by keeping the placards inside the ring.
    localparam [20:0]  BUF_STRIDE  = 21'h050000;

    localparam [23:0] PIC_0 = 24'h101010;
    localparam [23:0] PIC_1 = 24'h202020;
    localparam [23:0] PIC_2 = 24'h303030;
    localparam [23:0] PIC_3 = 24'h404040;
    localparam [23:0] PIC_4 = 24'h505050;
    localparam [23:0] PIC_5 = 24'h606060;
    localparam [23:0] PLACARD_F = 24'h00FF00;
    localparam [23:0] PLACARD_E = 24'h0000FF;
    localparam [23:0] NEW_0 = 24'hA10000;
    localparam [23:0] NEW_1 = 24'h00A100;
    localparam [23:0] NEW_2 = 24'h0000A1;
    localparam [23:0] NEW_3 = 24'hA1A100;
    localparam [23:0] NEW_4 = 24'h00A1A1;
    localparam [23:0] NEW_5 = 24'hA100A1;

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
    reg         swap_target_en;
    reg  [2:0]  swap_target;
    reg         emg_slot;
    reg         emg_slot_sel;

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
    wire [2:0]  display_buf;

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
    integer     k;
    reg  [2:0]  fire_slot;
    reg  [2:0]  exit_slot;

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
        .emg_slot        (emg_slot),
        .emg_slot_sel    (emg_slot_sel),
        .swap_target_en  (swap_target_en),
        .swap_target     (swap_target),
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
        .display_buf     (display_buf),
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

    // One picture load.  emg = 1 aims it at a dedicated placard slot; the
    // destination is held for the whole transfer, matching player_ctrl.
    task load_frame;
        input [23:0] rgb;
        input        emg;
        input        sel;
        integer i;
        begin
            @(negedge clk);
            emg_slot     = emg;
            emg_slot_sel = sel;
            img_abort    = 1'b0;
            img_active   = 1'b1;
            reload       = 1'b1;
            @(negedge clk);
            reload = 1'b0;

            for (i = 0; i < FRAME_WORDS; i = i + 1)
                send_pixel(rgb);

            @(negedge clk);
            img_active = 1'b0;
            @(posedge clk);
            @(negedge clk);
            emg_slot = 1'b0;
        end
    endtask

    task wait_stored;
        integer n;
        begin
            n = 0;
            while (!stored_pulse && n < 200000) begin
                @(posedge clk);
                n = n + 1;
            end
            if (!stored_pulse) begin
                $display("FAIL stored_pulse timeout");
                errors = errors + 1;
            end
            @(posedge clk);
        end
    endtask

    task wait_frame_end;
        begin
            @(posedge pix_clk);
            while (!(hx == 0 && vy == 0))
                @(posedge pix_clk);
        end
    endtask

    task capture_and_report;
        input [8*24-1:0] label;
        input [23:0]     expected;
        integer i;
        integer bad;
        begin
            wait_frame_end();
            captured   = 0;
            capture_en = 1'b1;
            while (captured < FRAME_WORDS)
                @(posedge pix_clk);
            capture_en = 1'b0;

            bad = 0;
            for (i = 0; i < FRAME_WORDS; i = i + 1)
                if (captured_pixels[i] !== expected)
                    bad = bad + 1;

            $display("DISPLAY %0s slot=%0d first=%06h expected=%06h bad=%0d",
                     label, display_buf, captured_pixels[0], expected, bad);
            if (bad != 0)
                errors = errors + 1;
        end
    endtask

    // Publish one named slot, the way the host jump / placard path does.
    task publish_slot;
        input [2:0] slot;
        integer base_n;
        begin
            base_n = loaded_frames;
            @(negedge clk);
            swap_target    = slot;
            swap_target_en = 1'b1;
            swap_req       = 1'b1;
            @(negedge clk);
            swap_req       = 1'b0;
            while (loaded_frames == base_n)
                @(posedge pix_clk);
            @(negedge clk);
            swap_target_en = 1'b0;
        end
    endtask

    task check_slot;
        input [2:0]  slot;
        input [23:0] want;
        reg   [23:0] got;
        begin
            got = dut.u_sdram_ctrl.mem[slot * BUF_STRIDE][23:0];
            $display("SLOT %0d first=%06h expected=%06h ready=%b",
                     slot, got, want, cache_ready[slot]);
            if (got !== want) begin
                $display("FAIL slot %0d holds %06h, expected %06h",
                         slot, got, want);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        reload         = 1'b0;
        img_active     = 1'b0;
        img_abort      = 1'b0;
        img_valid      = 1'b0;
        img_rgb        = 24'd0;
        swap_req       = 1'b0;
        swap_target_en = 1'b0;
        swap_target    = 3'd0;
        emg_slot       = 1'b0;
        emg_slot_sel   = 1'b0;
        errors         = 0;
        captured       = 0;
        capture_en     = 1'b0;
        loaded_frames  = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        wait (init_done === 1'b1);
        repeat (20) @(posedge clk);

        // The six ring slots must fit in the on-die 2M x 32 SDRAM.  This is
        // the check that the old dedicated slot 6/7 layout failed.
        if (((BUF_COUNT-1) * BUF_STRIDE + FRAME_WORDS) > 22'h200000) begin
            $display("FAIL slot layout exceeds the 2M x 32 SDRAM");
            errors = errors + 1;
        end
        $display("INFO last slot end = %06h, SDRAM words = 200000",
                 (BUF_COUNT-1) * BUF_STRIDE + FRAME_WORDS);

        // 1. boot cache: six normal pictures fill the ring.
        load_frame(PIC_0, 1'b0, 1'b0); wait_stored();
        load_frame(PIC_1, 1'b0, 1'b0); wait_stored();
        load_frame(PIC_2, 1'b0, 1'b0); wait_stored();
        load_frame(PIC_3, 1'b0, 1'b0); wait_stored();
        load_frame(PIC_4, 1'b0, 1'b0); wait_stored();
        load_frame(PIC_5, 1'b0, 1'b0); wait_stored();
        $display("INFO after boot cache_ready=%b stored=%0d", cache_ready,
                 stored_buf);
        if (cache_ready !== 6'b111111) begin
            $display("FAIL boot cache_ready=%b, expected 111111", cache_ready);
            errors = errors + 1;
        end

        // Park the display on slot 2 so the borrow choice is observable.
        publish_slot(3'd2);
        capture_and_report("third picture live", PIC_2);
        begin : wait_for_live_slot
            integer n;
            n = 0;
            while ((dut.disp_buf_s2 !== 3'd2) && (n < 20000)) begin
                @(posedge clk);
                n = n + 1;
            end
            if (dut.disp_buf_s2 !== 3'd2) begin
                $display("FAIL disp_buf_s2 did not settle on slot 2");
                errors = errors + 1;
            end
        end

        // 2. The fire placard borrows slot 3; the live slot 2 keeps its
        //    picture and leaves the rotate set only for the borrowed slot.
        load_frame(PLACARD_F, 1'b1, 1'b0);
        wait_stored();
        fire_slot = stored_buf;
        $display("INFO fire placard borrowed slot %0d cache_ready=%b",
                 fire_slot, cache_ready);
        if (fire_slot != 3'd3) begin
            $display("FAIL fire placard went to slot %0d, expected 3",
                     fire_slot);
            errors = errors + 1;
        end
        if (cache_ready[fire_slot] !== 1'b0) begin
            $display("FAIL borrowed fire slot %0d stayed in the rotate set",
                     fire_slot);
            errors = errors + 1;
        end
        check_slot(3'd0, PIC_0);
        check_slot(3'd2, PIC_2);
        check_slot(3'd3, PLACARD_F);

        // 3. The exit preload starts before the fire swap reaches the display
        //    (matching player_ctrl).  It must take the slot after the fire
        //    placard, never the fire slot itself.
        load_frame(PLACARD_E, 1'b1, 1'b1);
        wait_stored();
        exit_slot = stored_buf;
        $display("INFO exit placard borrowed slot %0d cache_ready=%b",
                 exit_slot, cache_ready);
        if (exit_slot != 3'd4) begin
            $display("FAIL exit placard went to slot %0d, expected 4",
                     exit_slot);
            errors = errors + 1;
        end
        if (exit_slot == fire_slot) begin
            $display("FAIL exit placard overwrote the fire slot");
            errors = errors + 1;
        end
        if (cache_ready[exit_slot] !== 1'b0) begin
            $display("FAIL borrowed exit slot %0d stayed in the rotate set",
                     exit_slot);
            errors = errors + 1;
        end
        // The early preload must not have disturbed the fire poster.
        check_slot(fire_slot, PLACARD_F);
        check_slot(exit_slot, PLACARD_E);

        // 4. Both placards publish from their borrowed ring slots.
        publish_slot(fire_slot);
        capture_and_report("fire placard live", PLACARD_F);
        publish_slot(exit_slot);
        capture_and_report("exit placard live", PLACARD_E);

        // 5. Alarm clears: the first normal load restarts one slot past the
        //    placard on screen (slot 4 -> slot 5) and invalidates the old ring.
        load_frame(NEW_0, 1'b0, 1'b0); wait_stored();
        if (stored_buf != 3'd5) begin
            $display("FAIL first rebuilt picture went to slot %0d, expected 5",
                     stored_buf);
            errors = errors + 1;
        end
        if (cache_ready !== 6'b100000) begin
            $display("FAIL rebuild did not start from an empty rotate set: %b",
                     cache_ready);
            errors = errors + 1;
        end
        publish_slot(stored_buf);
        capture_and_report("first rebuilt picture", NEW_0);

        load_frame(NEW_1, 1'b0, 1'b0); wait_stored();
        load_frame(NEW_2, 1'b0, 1'b0); wait_stored();
        load_frame(NEW_3, 1'b0, 1'b0); wait_stored();
        load_frame(NEW_4, 1'b0, 1'b0); wait_stored();
        load_frame(NEW_5, 1'b0, 1'b0); wait_stored();

        // The rebuild wrapped 5,0,1,2,3,4; every slot must now hold a normal
        // picture and the placards must be gone from the ring for good.
        check_slot(3'd5, NEW_0);
        check_slot(3'd0, NEW_1);
        check_slot(3'd1, NEW_2);
        check_slot(3'd2, NEW_3);
        check_slot(3'd3, NEW_4);
        check_slot(3'd4, NEW_5);
        if (cache_ready !== 6'b111111) begin
            $display("FAIL rebuilt cache_ready=%b, expected 111111",
                     cache_ready);
            errors = errors + 1;
        end
        for (k = 0; k < BUF_COUNT; k = k + 1) begin
            if ((dut.u_sdram_ctrl.mem[k * BUF_STRIDE][23:0] === PLACARD_F) ||
                (dut.u_sdram_ctrl.mem[k * BUF_STRIDE][23:0] === PLACARD_E)) begin
                $display("FAIL ring slot %0d still holds a placard", k);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("RESULT: PASS tb_frame_store_emg_slot");
        else
            $display("RESULT: FAIL tb_frame_store_emg_slot errors=%0d", errors);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("RESULT: FAIL tb_frame_store_emg_slot timeout state=%0d",
                 ld_state);
        $finish;
    end

endmodule

