`timescale 1ns / 1ps
//=============================================================================
// tb_player_swap_timeout.v
//
// If the frame store never returns swap_done, player_ctrl must not latch
// swap_pending forever.  A second KEY1 request has to be accepted after the
// sequencer-side timeout expires, otherwise the board appears to lose KEY1
// control after one failed transition.
//=============================================================================
module tb_player_swap_timeout;

    localparam integer HOLD_CYCLES         = 1000;
    localparam integer SWAP_TIMEOUT_CYCLES = 40;
    localparam integer RETRY_CYCLES        = 100;
    localparam integer LOAD_CYCLES         = 1000;
    localparam integer SCAN_CYCLES         = 500;

    localparam [2:0] C_LOADING = 3'd3,
                     C_HOLD    = 3'd4;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg         rst_n = 1'b0;
    reg         key_pressed = 1'b0;
    reg         sd_init_done = 1'b0;
    reg         scan_done = 1'b0;
    reg         scan_error = 1'b0;
    reg [3:0]   scan_fault_code = 4'd0;
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
    integer mark;
    integer gap;

    always @(posedge clk) cycles = cycles + 1;

    player_ctrl #(
        .HOLD_CYCLES         (HOLD_CYCLES),
        .SWAP_TIMEOUT_CYCLES (SWAP_TIMEOUT_CYCLES),
        .RETRY_CYCLES        (RETRY_CYCLES),
        .LOAD_CYCLES         (LOAD_CYCLES),
        .SCAN_CYCLES         (SCAN_CYCLES),
        .PRELOAD             (0),
        .CACHE_MODE          (1),
        .CACHE_FRAMES        (6)
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
        .file_index      (file_index),
        .load_start      (load_start),
        .load_abort      (load_abort),
        .load_busy       (1'b1),
        .load_done       (1'b0),
        .load_error      (load_error),
        .bmp_start       (bmp_start),
        .bmp_done        (bmp_done),
        .bmp_error       (bmp_error),
        .img_active      (img_active),
        .frame_abort     (frame_abort),
        .reload          (reload),
        .preload_ready   (1'b0),
        .swap_req        (swap_req),
        .swap_done       (swap_done),
        .stored_pulse    (stored_pulse),
        .state_dbg       (state_dbg),
        .card_ok         (card_ok),
        .file_error      (file_error),
        .fs_fault_code   (fs_fault_code)
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
                $display("FAIL state %0d not reached within %0d cycles", want, limit);
                errors = errors + 1;
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

        wait_state(3'd1, 50);
        pulse_scan_done();
        wait_state(3'd2, 50);

        // Fill three resident slots so the cache path reaches its hold state.
        for (i = 0; i < 3; i = i + 1) begin
            wait_state(C_LOADING, 50);
            repeat (5) @(posedge clk);
            pulse_bmp_done();
            wait_state(C_HOLD, 50);
            repeat (5) @(posedge clk);
            pulse_stored();
            if (i < 2)
                wait_state(3'd2, 50);
        end

        wait_state(C_HOLD, 50);
        repeat (5) @(posedge clk);

        // First request deliberately receives no swap_done.
        mark = cycles;
        key_pulse();
        wait_swap_req(20, mark);
        if (gap <= 0) begin
            $display("FAIL first KEY1 did not issue a swap request");
            errors = errors + 1;
        end

        // Wait past the watchdog, then press KEY1 again without ever
        // asserting swap_done for the first request.
        repeat (SWAP_TIMEOUT_CYCLES + 20) @(posedge clk);
        mark = cycles;
        key_pulse();
        wait_swap_req(20, mark);
        if (gap <= 0) begin
            $display("FAIL KEY1 stayed blocked after the swap timeout");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_player_swap_timeout");
        else
            $display("RESULT: FAIL tb_player_swap_timeout errors=%0d", errors);
        $finish;
    end

    initial begin
        #200000;
        $display("RESULT: FAIL tb_player_swap_timeout timeout state=%0d", state_dbg);
        $finish;
    end

endmodule
