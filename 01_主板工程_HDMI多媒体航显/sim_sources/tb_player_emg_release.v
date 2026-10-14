`timescale 1ns / 1ps
//=============================================================================
// tb_player_emg_release.v
//
// Regression for the fire-alarm release sequence.  The emergency picture
// must remain the live picture until the first normal picture has completed
// its swap.  This prevents normal overlays from being drawn on top of the
// evacuation placard during the cache rebuild.
//=============================================================================
module tb_player_emg_release;

    localparam [2:0] C_SCAN     = 3'd1,
                     C_LOAD_REQ = 3'd2,
                     C_LOADING  = 3'd3,
                     C_HOLD     = 3'd4;

    localparam integer HOLD_CYCLES  = 100;
    localparam integer SWAP_TIMEOUT = 200;
    localparam integer LOAD_CYCLES  = 300;
    localparam integer SCAN_CYCLES  = 500;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg        rst_n = 1'b0;
    reg        key_pressed = 1'b0;
    reg        auto_rotate_en = 1'b0;
    reg        jump_req = 1'b0;
    reg [2:0]  jump_slot = 3'd0;

    reg        emg_mode = 1'b0;
    reg        emg_card_free = 1'b1;
    reg        emg_fire_found = 1'b1;
    reg        emg_exit_found = 1'b1;
    reg [2:0]  stored_buf = 3'd0;

    reg        sd_init_done = 1'b0;
    reg        scan_done = 1'b0;
    reg        scan_error = 1'b0;
    reg [3:0]  scan_fault_code = 4'd0;
    reg [7:0]  file_count = 8'd2;
    reg        bmp_done = 1'b0;
    reg        bmp_error = 1'b0;
    reg        load_error = 1'b0;
    reg        stored_pulse = 1'b0;
    reg        swap_done = 1'b0;

    wire       start_scan;
    wire       load_start;
    wire       load_abort;
    wire       bmp_start;
    wire       img_active;
    wire       frame_abort;
    wire       reload;
    wire       swap_req;
    wire       swap_target_en;
    wire [2:0] swap_target;
    wire [1:0] load_src;
    wire       emg_pic;
    wire       emg_imgs_ready;
    wire       emg_slot;
    wire       emg_slot_sel;
    wire [7:0] file_index;
    wire [2:0] state_dbg;

    integer errors = 0;
    reg     saw_exit_preload = 1'b0;

    always @(posedge clk) begin
        if ((state_dbg == C_LOAD_REQ) && (load_src == 2'd3))
            saw_exit_preload <= 1'b1;
    end

    player_ctrl #(
        .HOLD_CYCLES        (HOLD_CYCLES),
        .SWAP_TIMEOUT_CYCLES(SWAP_TIMEOUT),
        .RETRY_CYCLES       (100),
        .LOAD_CYCLES        (LOAD_CYCLES),
        .SCAN_CYCLES        (SCAN_CYCLES),
        .PRELOAD            (0),
        .CACHE_MODE         (1),
        .CACHE_FRAMES       (2),
        .EMG_HOLD_CYCLES    (100)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .key_pressed    (key_pressed),
        .auto_rotate_en (auto_rotate_en),
        .jump_req       (jump_req),
        .jump_slot      (jump_slot),
        .emg_mode       (emg_mode),
        .emg_card_free  (emg_card_free),
        .emg_fire_found (emg_fire_found),
        .emg_exit_found (emg_exit_found),
        .stored_buf     (stored_buf),
        .emg_pic        (emg_pic),
        .emg_imgs_ready (emg_imgs_ready),
        .emg_slot       (emg_slot),
        .emg_slot_sel   (emg_slot_sel),
        .load_src       (load_src),
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
        .swap_target_en (swap_target_en),
        .swap_target    (swap_target),
        .stored_pulse   (stored_pulse),
        .state_dbg      (state_dbg),
        .card_ok        (),
        .file_error     (),
        .fs_fault_code  (),
        .cache_ready    ()
    );

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
        input [2:0] slot;
        begin
            @(negedge clk);
            stored_buf = slot;
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
            while ((state_dbg !== want) && (n < limit)) begin
                @(posedge clk);
                n = n + 1;
            end
            if (state_dbg !== want) begin
                $display("FAIL timeout waiting for state %0d, got %0d",
                         want, state_dbg);
                errors = errors + 1;
            end
        end
    endtask

    task wait_load_src;
        input [1:0] want;
        input integer limit;
        integer n;
        begin
            n = 0;
            while ((load_src !== want || state_dbg !== C_LOAD_REQ) &&
                   (n < limit)) begin
                @(posedge clk);
                n = n + 1;
            end
            if ((load_src !== want) || (state_dbg !== C_LOAD_REQ)) begin
                $display("FAIL timeout waiting for load_src=%0d state=%0d",
                         want, state_dbg);
                errors = errors + 1;
            end
        end
    endtask

    task wait_swap_req;
        input [8*32-1:0] label;
        input integer limit;
        integer n;
        begin
            n = 0;
            while ((swap_req !== 1'b1) && (n < limit)) begin
                // swap_req is a single cycle pulse, and it is raised on the
                // clock edge after swap_wanted/set.  Sample on the falling
                // edge so the pulse can never be missed to a scheduler race
                // with the DUT's non blocking assignment.
                @(negedge clk);
                n = n + 1;
            end
            if (swap_req !== 1'b1) begin
                $display("FAIL swap_req timeout: %0s", label);
                errors = errors + 1;
            end
        end
    endtask

    task load_one_picture;
        input [2:0] slot;
        begin
            wait_state(C_LOADING, 80);
            repeat (3) @(posedge clk);
            pulse_bmp_done();
            wait_state(C_HOLD, 80);
            repeat (3) @(posedge clk);
            pulse_stored(slot);
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        sd_init_done = 1'b1;

        // Build the normal two-picture cache.
        wait_state(C_SCAN, 30);
        pulse_scan_done();
        wait_state(C_LOAD_REQ, 30);
        load_one_picture(3'd0);
        wait_state(C_LOAD_REQ, 30);
        load_one_picture(3'd1);
        repeat (5) @(posedge clk);

        // Raise the alarm and make the fire placard live.
        emg_mode = 1'b1;
        wait_load_src(2'd2, 80);
        wait_state(C_LOADING, 80);
        repeat (3) @(posedge clk);
        pulse_bmp_done();
        wait_state(C_HOLD, 80);
        repeat (3) @(posedge clk);
        pulse_stored(3'd2);
        if (emg_pic !== 1'b1) begin
            $display("FAIL fire placard did not become live");
            errors = errors + 1;
        end
        if (emg_imgs_ready !== 1'b0) begin
            $display("FAIL voice released before the exit placard was cached");
            errors = errors + 1;
        end
        // The request is a one cycle pulse right after the store.  Look for
        // it before any settle delay, otherwise it is already gone.
        wait_swap_req("fire placard", 40);
        repeat (3) @(posedge clk);
        pulse_swap_done();
        repeat (3) @(posedge clk);

        // The second placard is cached before the alarm is released, so the
        // fire and exit posters can alternate without touching the TF card.
        if (saw_exit_preload !== 1'b1) begin
            $display("FAIL exit placard was not preloaded");
            errors = errors + 1;
        end
        wait_state(C_LOADING, 80);
        repeat (3) @(posedge clk);
        pulse_bmp_done();
        wait_state(C_HOLD, 80);
        repeat (3) @(posedge clk);
        pulse_stored(3'd3);
        repeat (3) @(posedge clk);
        if (emg_pic !== 1'b1) begin
            $display("FAIL exit preload replaced the live placard");
            errors = errors + 1;
        end
        if (emg_imgs_ready !== 1'b1) begin
            $display("FAIL both placards cached but voice still gated");
            errors = errors + 1;
        end
        wait_swap_req("exit placard", 220);
        if (swap_target !== 3'd3) begin
            $display("FAIL exit placard swap targeted slot %0d", swap_target);
            errors = errors + 1;
        end
        repeat (3) @(posedge clk);
        pulse_swap_done();
        repeat (3) @(posedge clk);

        wait_swap_req("fire placard repeat", 220);
        if (swap_target !== 3'd2) begin
            $display("FAIL repeat fire swap targeted slot %0d", swap_target);
            errors = errors + 1;
        end
        repeat (3) @(posedge clk);
        pulse_swap_done();
        repeat (3) @(posedge clk);

        // Clear the alarm while the card is still busy with the fire
        // announcement.  The placard must stay live and the sequencer must
        // wait for the card instead of issuing a scan pulse that the reader
        // would drop.
        emg_mode = 1'b0;
        emg_card_free = 1'b0;
        repeat (5) @(posedge clk);
        if (emg_pic !== 1'b1) begin
            $display("FAIL placard released before the normal swap");
            errors = errors + 1;
        end

        // Give the old failure path time to dead-end in C_SCAN/FALLBACK.
        repeat (30) @(posedge clk);
        if (state_dbg !== C_HOLD) begin
            $display("FAIL release rebuild started while the card was busy: state=%0d",
                     state_dbg);
            errors = errors + 1;
        end

        // The card becomes free.  The rebuild must start from the cached
        // directory and load the first normal picture without a rescan.
        emg_card_free = 1'b1;
        wait_state(C_LOAD_REQ, 30);
        load_one_picture(3'd0);
        wait_swap_req("resume picture", 50);
        repeat (3) @(posedge clk);
        if (emg_pic !== 1'b1) begin
            $display("FAIL placard released before swap_done");
            errors = errors + 1;
        end
        pulse_swap_done();
        repeat (5) @(posedge clk);
        if (emg_pic !== 1'b0) begin
            $display("FAIL placard did not release after normal swap_done");
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("TB_PLAYER_EMG_RELEASE: PASS");
        end else begin
            $display("TB_PLAYER_EMG_RELEASE: FAIL (%0d errors)", errors);
            $fatal(1);
        end
        $finish;
    end

    initial begin
        #500000;
        $display("TB_PLAYER_EMG_RELEASE: TIMEOUT state=%0d emg_pic=%b",
                 state_dbg, emg_pic);
        $fatal(1);
    end

endmodule
