`timescale 1ns / 1ps
//=============================================================================
// tb_player_key.v
//
// V5 key-path regression.  A key before the first picture keeps the old
// hard-skip behavior.  After a picture is live, a key is latched until the
// preloaded buffer is ready and must not abort the preload.
//=============================================================================
module tb_player_key;

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
    reg sd_init_done = 1'b0;
    reg scan_done = 1'b0;
    reg scan_error = 1'b0;
    reg [3:0] scan_fault_code = 4'd0;
    reg [7:0] file_count = 8'd3;
    reg bmp_done = 1'b0;
    reg bmp_error = 1'b0;
    reg load_error = 1'b0;
    reg preload_ready = 1'b0;
    reg swap_done = 1'b0;

    wire start_scan, load_start, load_abort, bmp_start;
    wire img_active, frame_abort, reload, card_ok, file_error;
    wire swap_req;
    wire [7:0] file_index;
    wire [2:0] state_dbg;
    wire [3:0] fs_fault_code;

    player_ctrl #(
        .HOLD_CYCLES  (HOLD_CYCLES),
        .RETRY_CYCLES (RETRY_CYCLES),
        .LOAD_CYCLES  (LOAD_CYCLES),
        .SCAN_CYCLES  (SCAN_CYCLES),
        .PRELOAD      (1)
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
        .preload_ready  (preload_ready),
        .swap_req       (swap_req),
        .swap_done      (swap_done),
        .state_dbg      (state_dbg),
        .card_ok        (card_ok),
        .file_error     (file_error),
        .fs_fault_code  (fs_fault_code)
    );

    integer errors = 0;
    integer abort_pulses = 0;

    always @(posedge clk)
        if (frame_abort)
            abort_pulses <= abort_pulses + 1;

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
                $display("FAIL timeout waiting for state %0d, got %0d",
                         want, state_dbg);
                errors = errors + 1;
            end
        end
    endtask

    task wait_swap_req;
        input integer limit;
        integer n;
        begin
            n = 0;
            while (swap_req !== 1'b1 && n < limit) begin
                @(posedge clk);
                n = n + 1;
            end
            if (swap_req !== 1'b1) begin
                $display("FAIL swap_req did not assert");
                errors = errors + 1;
            end
        end
    endtask

    integer n;

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        sd_init_done = 1'b1;
        wait_state(C_SCAN, 50);
        pulse_scan_done();
        wait_state(C_LOAD_REQ, 50);

        // First frame is loading.  This is still the old hard-skip path.
        wait_state(C_LOADING, 50);
        key_pulse();
        wait_state(C_RESTART, 20);
        wait_state(C_LOAD_REQ, 20);
        if (file_index !== 8'd1) begin
            $display("FAIL first-load key did not skip to file 1, got %0d",
                     file_index);
            errors = errors + 1;
        end

        // Finish the first live frame.  The sequencer must start file 1 as
        // an off-screen preload.
        wait_state(C_LOADING, 50);
        repeat (10) @(posedge clk);
        pulse_bmp_done();
        wait_state(C_LOAD_REQ, 50);
        if (file_index !== 8'd2) begin
            $display("FAIL first completion did not preload file 2, got %0d",
                     file_index);
            errors = errors + 1;
        end

        // A key during this preload is remembered, not destructive.
        wait_state(C_LOADING, 50);
        key_pulse();
        repeat (10) @(posedge clk);
        if (state_dbg !== C_LOADING || swap_req !== 1'b0) begin
            $display("FAIL key during preload changed state or requested early");
            errors = errors + 1;
        end
        pulse_bmp_done();
        wait_state(C_HOLD, 50);

        preload_ready = 1'b1;
        wait_swap_req(20);
        @(posedge clk);
        preload_ready = 1'b0;
        @(negedge clk);
        swap_done = 1'b1;
        @(negedge clk);
        swap_done = 1'b0;
        wait_state(C_LOAD_REQ, 20);
        if (file_index !== 8'd0) begin
            $display("FAIL swap did not wrap to file 0, got %0d", file_index);
            errors = errors + 1;
        end

        if (abort_pulses != 1) begin
            // The first-load hard skip above should produce exactly one.
            $display("FAIL unexpected frame_abort count=%0d", abort_pulses);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_player_key");
        else
            $display("RESULT: FAIL tb_player_key errors=%0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_player_key timeout state=%0d", state_dbg);
        $finish;
    end

endmodule
