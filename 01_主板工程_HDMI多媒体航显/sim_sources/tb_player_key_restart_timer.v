`timescale 1ns / 1ps
//=============================================================================
// tb_player_key_restart_timer.v
//
// The 3 second auto advance must restart every time the viewer presses the
// key, otherwise a press just before the timer expires is followed by an
// automatic change almost immediately and the two look unrelated.
//
// The cache-mode hold counter is driven from two places in player_ctrl: the
// swap handshake clears it, and the CC_HOLD state increments it.  These are
// non blocking assignments to the same register in the same always block, so
// whichever statement runs last wins.  This testbench presses the key late in
// the hold window and measures how long the next automatic change takes.
//=============================================================================
module tb_player_key_restart_timer;

    localparam integer HOLD_CYCLES  = 200;
    localparam integer RETRY_CYCLES = 100;
    localparam integer LOAD_CYCLES  = 1000;
    localparam integer SCAN_CYCLES  = 500;

    localparam [2:0] C_WAIT_SD  = 3'd0,
                     C_SCAN     = 3'd1,
                     C_LOAD_REQ = 3'd2,
                     C_LOADING  = 3'd3,
                     C_HOLD     = 3'd4,
                     C_RETRY    = 3'd5,
                     C_FALLBACK = 3'd6,
                     C_RESTART  = 3'd7;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg         rst_n = 1'b0;
    reg         key_pressed = 1'b0;
    reg         sd_init_done = 1'b0;
    reg         scan_done = 1'b0;
    reg         scan_error = 1'b0;
    reg [3:0]   scan_fault_code = 4'd0;
    // Three files only: the cache stops loading once every directory entry is
    // resident, which is what puts the sequencer into the real hold state.
    reg [7:0]   file_count = 8'd3;
    reg         bmp_done = 1'b0;
    reg         bmp_error = 1'b0;
    reg         load_error = 1'b0;
    reg         stored_pulse = 1'b0;
    reg         swap_done = 1'b0;

    wire        start_scan, load_start, load_abort, bmp_start;
    wire        img_active, frame_abort, reload, card_ok, file_error;
    wire        swap_req;
    wire [7:0]  file_index;
    wire [2:0]  state_dbg;
    wire [3:0]  fs_fault_code;

    integer errors = 0;
    integer cycles = 0;
    integer i;
    integer mark_press, mark_swap;
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
        .auto_rotate_en (1'b1),
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
                $display("FAIL state %0d not reached within %0d cycles (state=%0d)",
                         want, limit, state_dbg);
                errors = errors + 1;
            end
        end
    endtask

    // Wait for a swap_req, then count the cycles it took.
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
                $display("FAIL swap_req did not assert within %0d cycles", limit);
                errors = errors + 1;
                gap = -1;
            end else begin
                gap = cycles - from_cycle;
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

        // Fill three cache slots, which is enough for the display to cycle.
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

        // ---- 1. press the key late in the hold window --------------------
        // The automatic change is due in about 50 cycles; a correct design
        // pushes it a full HOLD_CYCLES into the future.
        repeat (HOLD_CYCLES - 55) @(posedge clk);
        mark_press = cycles;
        key_pulse();
        wait_swap_req(HOLD_CYCLES, mark_press);
        $display("NOTE key press to swap_req        : %0d cycles", gap);
        if (gap > 10) begin
            $display("FAIL the key did not swap straight away");
            errors = errors + 1;
        end
        pulse_swap_done();
        repeat (4) @(posedge clk);

        mark_press = cycles;
        wait_swap_req(HOLD_CYCLES + 80, mark_press);
        $display("NOTE key press to next auto change: %0d cycles (want ~%0d)",
                 gap, HOLD_CYCLES);
        if (gap < (HOLD_CYCLES - 10)) begin
            $display("FAIL the 3 s counter did not restart after the key press");
            errors = errors + 1;
        end
        if (gap > (HOLD_CYCLES + 80)) begin
            $display("FAIL the restarted counter ran too long");
            errors = errors + 1;
        end
        pulse_swap_done();
        repeat (4) @(posedge clk);

        // ---- 2. two presses in a row each restart the window -------------
        repeat (20) @(posedge clk);
        mark_press = cycles;
        key_pulse();
        wait_swap_req(HOLD_CYCLES, mark_press);
        pulse_swap_done();
        repeat (4) @(posedge clk);

        mark_press = cycles;
        key_pulse();
        wait_swap_req(HOLD_CYCLES, mark_press);
        pulse_swap_done();
        repeat (4) @(posedge clk);

        mark_press = cycles;
        wait_swap_req(HOLD_CYCLES + 80, mark_press);
        $display("NOTE second press to auto change : %0d cycles (want ~%0d)",
                 gap, HOLD_CYCLES);
        if (gap < (HOLD_CYCLES - 10)) begin
            $display("FAIL the counter did not restart after the second press");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_player_key_restart_timer");
        else
            $display("RESULT: FAIL tb_player_key_restart_timer errors=%0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_player_key_restart_timer timeout state=%0d", state_dbg);
        $finish;
    end

endmodule
