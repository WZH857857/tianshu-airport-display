`timescale 1ns / 1ps
//=============================================================================
// tb_player_cache.v
//
// Focused test for resident-frame cache mode.  All three files are loaded
// once, then KEY1 only requests an SDRAM slot swap: no new file or BMP load
// may start.
//=============================================================================
module tb_player_cache;

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
    reg stored_pulse = 1'b0;
    reg swap_done = 1'b0;

    wire start_scan, load_start, load_abort, bmp_start;
    wire img_active, frame_abort, reload, card_ok, file_error;
    wire swap_req;
    wire [7:0] file_index;
    wire [2:0] state_dbg;
    wire [3:0] fs_fault_code;

    integer errors = 0;
    integer load_starts = 0;
    integer bmp_starts = 0;
    integer abort_pulses = 0;

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

    always @(posedge clk) begin
        if (load_start)
            load_starts <= load_starts + 1;
        if (bmp_start)
            bmp_starts <= bmp_starts + 1;
        if (frame_abort)
            abort_pulses <= abort_pulses + 1;
    end

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
                $display("FAIL timed out waiting for state %0d, got %0d",
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
                $display("FAIL swap_req did not assert within %0d cycles", limit);
                errors = errors + 1;
            end else begin
                $display("OK   swap_req asserted after %0d cycles", n);
            end
        end
    endtask

    integer i;

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        sd_init_done = 1'b1;
        wait_state(C_SCAN, 50);
        pulse_scan_done();
        wait_state(C_LOAD_REQ, 50);

        for (i = 0; i < 3; i = i + 1) begin
            wait_state(C_LOADING, 50);
            repeat (5) @(posedge clk);
            pulse_bmp_done();
            wait_state(C_HOLD, 50);
            repeat (5) @(posedge clk);
            pulse_stored();
            if (i < 2) begin
                wait_state(C_LOAD_REQ, 50);
                if (file_index !== (i + 1)) begin
                    $display("FAIL cache requested file %0d after image %0d",
                             file_index, i);
                    errors = errors + 1;
                end
            end
        end

        wait_state(C_HOLD, 50);
        repeat (5) @(posedge clk);
        if (load_starts != 3 || bmp_starts != 3) begin
            $display("FAIL initial cache loads load=%0d bmp=%0d, expected 3/3",
                     load_starts, bmp_starts);
            errors = errors + 1;
        end

        // The cached picture change must not start another file load.
        key_pulse();
        wait_swap_req(10);
        repeat (5) @(posedge clk);
        pulse_swap_done();
        repeat (10) @(posedge clk);
        if (load_starts != 3 || bmp_starts != 3) begin
            $display("FAIL key swap started a new load: load=%0d bmp=%0d",
                     load_starts, bmp_starts);
            errors = errors + 1;
        end

        // The automatic timer must use the same slot-swap path.
        wait_swap_req(HOLD_CYCLES + 30);
        pulse_swap_done();
        repeat (10) @(posedge clk);
        if (load_starts != 3 || bmp_starts != 3) begin
            $display("FAIL automatic swap started a new load: load=%0d bmp=%0d",
                     load_starts, bmp_starts);
            errors = errors + 1;
        end

        if (abort_pulses != 0) begin
            $display("FAIL cache mode unexpectedly aborted a frame");
            errors = errors + 1;
        end
        if (card_ok !== 1'b1 || file_error !== 1'b0) begin
            $display("FAIL cache health card_ok=%b file_error=%b",
                     card_ok, file_error);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_player_cache");
        else
            $display("RESULT: FAIL tb_player_cache errors=%0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_player_cache timeout state=%0d index=%0d",
                 state_dbg, file_index);
        $finish;
    end

endmodule
