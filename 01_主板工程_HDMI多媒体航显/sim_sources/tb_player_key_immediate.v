`timescale 1ns / 1ps
//=============================================================================
// tb_player_key_immediate.v
//
// Regression for the hard-skip KEY1 behavior.  A press during a BMP load
// aborts that load and starts the next file immediately, instead of waiting
// for the current several-second SPI transfer to finish.
//=============================================================================

module tb_player_key_immediate;

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

    wire        start_scan, load_start, load_abort, bmp_start;
    wire        img_active, frame_abort, reload, card_ok, file_error;
    wire [7:0]  file_index;
    wire [2:0]  state_dbg;
    wire [3:0]  fs_fault_code;

    player_ctrl #(
        .HOLD_CYCLES  (HOLD_CYCLES),
        .RETRY_CYCLES (RETRY_CYCLES),
        .LOAD_CYCLES  (LOAD_CYCLES),
        .SCAN_CYCLES  (SCAN_CYCLES),
        .PRELOAD      (0)
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
        .swap_req       (),
        .swap_done      (1'b0),
        .state_dbg      (state_dbg),
        .card_ok        (card_ok),
        .file_error     (file_error),
        .fs_fault_code  (fs_fault_code)
    );

    integer errors = 0;
    integer hold_cycles;
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
                $display("FAIL timed out waiting for state %0d, state=%0d",
                         want, state_dbg);
                errors = errors + 1;
            end
        end
    endtask

    task expect_key_skip;
        input [7:0] want_index;
        input integer limit;
        integer n;
        begin
            key_pulse();
            n = 0;
            while ((state_dbg !== C_LOAD_REQ || file_index !== want_index) &&
                   n < limit) begin
                @(posedge clk);
                n = n + 1;
            end
            if (state_dbg !== C_LOAD_REQ || file_index !== want_index) begin
                $display("FAIL hard skip: state=%0d index=%0d expected index=%0d",
                         state_dbg, file_index, want_index);
                errors = errors + 1;
            end else begin
                $display("OK   hard skip reached file %0d after %0d cycles",
                         file_index, n);
            end
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        sd_init_done = 1'b1;
        wait_state(C_SCAN, 50);
        repeat (4) @(posedge clk);
        pulse_scan_done();
        wait_state(C_LOAD_REQ, 50);
        if (file_index !== 8'd0) begin
            $display("FAIL first file index is %0d, expected 0", file_index);
            errors = errors + 1;
        end

        // Three presses in C_LOADING must hard-skip 0 -> 1 -> 2 -> 0.
        wait_state(C_LOADING, 50);
        expect_key_skip(8'd1, 20);
        wait_state(C_LOADING, 50);
        expect_key_skip(8'd2, 20);
        wait_state(C_LOADING, 50);
        expect_key_skip(8'd0, 20);

        // A press in the short request window must also skip immediately.
        // The current state is already C_LOAD_REQ here.
        expect_key_skip(8'd1, 20);

        // Complete the next load and let the automatic hold timer advance.
        wait_state(C_LOADING, 50);
        repeat (10) @(posedge clk);
        pulse_bmp_done();
        wait_state(C_HOLD, 50);

        hold_cycles = 0;
        while (state_dbg === C_HOLD && hold_cycles < 1000) begin
            @(posedge clk);
            hold_cycles = hold_cycles + 1;
        end
        if (hold_cycles < HOLD_CYCLES - 5 || hold_cycles > HOLD_CYCLES + 5) begin
            $display("FAIL hold timer fired after %0d cycles, expected %0d",
                     hold_cycles, HOLD_CYCLES);
            errors = errors + 1;
        end else begin
            $display("OK   automatic hold timer fired after %0d cycles",
                     hold_cycles);
        end
        wait_state(C_LOAD_REQ, 20);
        if (file_index !== 8'd2) begin
            $display("FAIL automatic advance reached index %0d, expected 2",
                     file_index);
            errors = errors + 1;
        end

        // A press while the picture is held must still advance immediately.
        wait_state(C_LOADING, 50);
        repeat (10) @(posedge clk);
        pulse_bmp_done();
        wait_state(C_HOLD, 50);
        expect_key_skip(8'd0, 20);

        if (abort_pulses == 0) begin
            $display("FAIL hard skip never asserted frame_abort");
            errors = errors + 1;
        end
        if (errors == 0)
            $display("RESULT: PASS tb_player_key_immediate");
        else
            $display("RESULT: FAIL tb_player_key_immediate errors=%0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_player_key_immediate timeout");
        $finish;
    end

endmodule
