`timescale 1ns / 1ps
//=============================================================================
// tb_player_manual_auto.v
//
// Verifies the switch-controlled picture-advance mode:
//   auto_rotate_en = 0  manual, the hold timer must never advance a picture
//   auto_rotate_en = 1  automatic, the timer requests a slot swap at HOLD
//   KEY1                still advances immediately in either mode
//
// It also checks the small overlay mode indicator used by the top-left panel:
// slot 13 is M in manual mode and A in automatic mode.
//=============================================================================
module tb_player_manual_auto;

    localparam [2:0] C_WAIT_SD  = 3'd0,
                     C_SCAN     = 3'd1,
                     C_LOAD_REQ = 3'd2,
                     C_LOADING  = 3'd3,
                     C_HOLD     = 3'd4,
                     C_RETRY    = 3'd5,
                     C_FALLBACK = 3'd6,
                     C_RESTART  = 3'd7;

    localparam integer HOLD_CYCLES  = 200;
    localparam integer RETRY_CYCLES = 100;
    localparam integer LOAD_CYCLES  = 1000;
    localparam integer SCAN_CYCLES  = 500;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg rst_n = 1'b0;
    reg key_pressed = 1'b0;
    reg auto_rotate_en = 1'b0;
    reg sd_init_done = 1'b0;
    reg scan_done = 1'b0;
    reg scan_error = 1'b0;
    reg [3:0] scan_fault_code = 4'd0;
    reg [7:0] file_count = 8'd3;
    reg bmp_done = 1'b0;
    reg bmp_error = 1'b0;
    reg load_error = 1'b0;
    reg stored_pulse = 1'b0;
    reg swap_done = 1'b0;

    wire start_scan, load_start, load_abort, bmp_start;
    wire img_active, frame_abort, reload, card_ok, file_error;
    wire swap_req;
    wire [7:0] file_index;
    wire [2:0] state_dbg;
    wire [3:0] fs_fault_code;

    integer errors = 0;
    integer cycles = 0;
    integer i;
    integer mark_cycle;
    integer gap;

    always @(posedge clk) cycles = cycles + 1;

    player_ctrl #(
        .HOLD_CYCLES  (HOLD_CYCLES),
        .RETRY_CYCLES (RETRY_CYCLES),
        .LOAD_CYCLES  (LOAD_CYCLES),
        .SCAN_CYCLES  (SCAN_CYCLES),
        .PRELOAD      (0),
        .CACHE_MODE   (1),
        .CACHE_FRAMES (6)
    ) u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .key_pressed    (key_pressed),
        .auto_rotate_en (auto_rotate_en),
        .sd_init_done   (sd_init_done),
        .start_scan     (start_scan),
        .scan_done      (scan_done),
        .scan_error     (scan_error),
        .scan_fault_code(scan_fault_code),
        .file_count     (file_count),
        .file_index     (file_index),
        .load_start     (load_start),
        .load_abort     (load_abort),
        .load_busy      (1'b1),
        .load_done      (1'b0),
        .load_error     (load_error),
        .bmp_start      (bmp_start),
        .bmp_done       (bmp_done),
        .bmp_error      (bmp_error),
        .img_active     (img_active),
        .frame_abort    (frame_abort),
        .reload         (reload),
        .preload_ready  (1'b0),
        .swap_req       (swap_req),
        .swap_done      (swap_done),
        .stored_pulse   (stored_pulse),
        .state_dbg      (state_dbg),
        .card_ok        (card_ok),
        .file_error     (file_error),
        .fs_fault_code  (fs_fault_code)
    );

    // Small overlay copy, enough to check the AUTO/MANUAL mode glyph.
    reg [9:0] px = 10'd220;             // slot 13 of the top panel
    reg [9:0] py = 10'd20;
    reg       de = 1'b1;
    wire [23:0] ui_rgb;
    wire        ui_hit;

    ui_overlay u_overlay (
        .pixel_clk        (clk),
        .rst              (~rst_n),
        .frame_tick       (1'b0),
        .de               (de),
        .px               (px),
        .py               (py),
        .base_rgb         (24'hFFFFFF),
        .ui_enable        (1'b1),
        .img_slot         (3'd0),
        .img_count        (8'd3),
        .user_bright      (3'd7),
        .user_contrast    (3'd3),
        .ui_mode          (2'd0),
        .param_flash      (1'b0),
        .param_sel        (2'd0),
        .auto_mode        (auto_rotate_en),
        .logo_enable      (1'b1),
        .subtitle_enable  (1'b0),
        .timestamp_enable (1'b0),
        .parallax_enable  (1'b0),
        .osd_enable       (1'b0),
        .wave_enable      (1'b0),
        .audio_rst        (1'b0),
        .audio_valid      (1'b0),
        .audio_left       (24'd0),
        .rgb              (ui_rgb),
        .hit              (ui_hit)
    );

    task key_pulse;
        begin
            @(negedge clk);
            key_pressed = 1'b1;
            @(negedge clk);
            key_pressed = 1'b0;
        end
    endtask

    task pulse_scan_done;
        begin
            @(negedge clk);
            scan_done = 1'b1;
            @(negedge clk);
            scan_done = 1'b0;
        end
    endtask

    task pulse_bmp_done;
        begin
            @(negedge clk);
            bmp_done = 1'b1;
            @(negedge clk);
            bmp_done = 1'b0;
        end
    endtask

    task pulse_stored;
        begin
            @(negedge clk);
            stored_pulse = 1'b1;
            @(negedge clk);
            stored_pulse = 1'b0;
        end
    endtask

    task pulse_swap_done;
        begin
            @(negedge clk);
            swap_done = 1'b1;
            @(negedge clk);
            swap_done = 1'b0;
        end
    endtask

    task wait_state;
        input [2:0] want;
        input integer limit;
        integer n;
        begin
            n = 0;
            while (state_dbg !== want && n < limit) begin
                @(posedge clk);
                n = n + 1;
            end
            if (state_dbg !== want) begin
                errors = errors + 1;
                $display("FAIL timed out waiting for state %0d, got %0d",
                         want, state_dbg);
            end
        end
    endtask

    task wait_swap_req;
        input integer limit;
        input integer from_cycle;
        integer n;
        begin
            n = 0;
            while (swap_req !== 1'b1 && n < limit) begin
                @(posedge clk);
                n = n + 1;
            end
            if (swap_req !== 1'b1) begin
                errors = errors + 1;
                gap = -1;
                $display("FAIL swap_req did not assert within %0d cycles", limit);
            end else begin
                gap = cycles - from_cycle;
            end
        end
    endtask

    task expect_no_swap;
        input integer limit;
        integer n;
        begin
            n = 0;
            while ((swap_req !== 1'b1) && (n < limit)) begin
                @(posedge clk);
                n = n + 1;
            end
            if (swap_req === 1'b1) begin
                errors = errors + 1;
                $display("FAIL unexpected automatic swap in manual mode");
            end
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        sd_init_done = 1'b1;

        wait_state(C_SCAN, 50);
        pulse_scan_done();
        wait_state(C_LOAD_REQ, 50);

        // Fill the three resident slots.
        for (i = 0; i < 3; i = i + 1) begin
            wait_state(C_LOADING, 50);
            repeat (5) @(posedge clk);
            pulse_bmp_done();
            wait_state(C_HOLD, 50);
            repeat (5) @(posedge clk);
            pulse_stored();
            if (i < 2)
                wait_state(C_LOAD_REQ, 50);
        end

        wait_state(C_HOLD, 50);
        repeat (5) @(posedge clk);

        // 1. Default manual mode: the 200 cycle timer must not swap.
        auto_rotate_en = 1'b0;
        expect_no_swap(HOLD_CYCLES + 120);

        // 2. KEY1 still advances immediately in manual mode.
        mark_cycle = cycles;
        key_pulse();
        wait_swap_req(20, mark_cycle);
        if (gap > 8) begin
            errors = errors + 1;
            $display("FAIL manual KEY1 swap took %0d cycles", gap);
        end
        pulse_swap_done();
        repeat (4) @(posedge clk);

        // 3. Switch to AUTO: the next swap is one full hold interval away.
        auto_rotate_en = 1'b1;
        mark_cycle = cycles;
        wait_swap_req(HOLD_CYCLES + 80, mark_cycle);
        if (gap < (HOLD_CYCLES - 10)) begin
            errors = errors + 1;
            $display("FAIL automatic swap came too early: %0d cycles", gap);
        end
        if (gap > (HOLD_CYCLES + 80)) begin
            errors = errors + 1;
            $display("FAIL automatic swap came too late: %0d cycles", gap);
        end
        pulse_swap_done();
        repeat (4) @(posedge clk);

        // 4. Turning AUTO off parks the timer and prevents another swap.
        auto_rotate_en = 1'b0;
        expect_no_swap(HOLD_CYCLES + 120);

        // 5. Turn AUTO back on: the interval restarts from zero, not from the
        //    point where manual mode interrupted the count.
        auto_rotate_en = 1'b1;
        mark_cycle = cycles;
        wait_swap_req(HOLD_CYCLES + 80, mark_cycle);
        if (gap < (HOLD_CYCLES - 10)) begin
            errors = errors + 1;
            $display("FAIL restarted automatic interval was too short: %0d cycles",
                     gap);
        end
        pulse_swap_done();

        // 6. Mode glyph: manual M, automatic A in the top-left panel.
        auto_rotate_en = 1'b0;
        repeat (4) @(posedge clk);
        if (u_overlay.top_ch !== 8'h4D) begin
            errors = errors + 1;
            $display("FAIL manual mode glyph = %02x, expected M", u_overlay.top_ch);
        end
        auto_rotate_en = 1'b1;
        repeat (4) @(posedge clk);
        if (u_overlay.top_ch !== 8'h41) begin
            errors = errors + 1;
            $display("FAIL automatic mode glyph = %02x, expected A", u_overlay.top_ch);
        end

        if (errors == 0)
            $display("RESULT: PASS tb_player_manual_auto");
        else
            $display("RESULT: FAIL tb_player_manual_auto errors=%0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_player_manual_auto timeout state=%0d", state_dbg);
        $finish;
    end

endmodule
