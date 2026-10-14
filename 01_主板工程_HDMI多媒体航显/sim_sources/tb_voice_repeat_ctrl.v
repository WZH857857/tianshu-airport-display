`timescale 1ns / 1ps
//=============================================================================
// tb_voice_repeat_ctrl.v - repeated announcement scheduler check
//
// A shortened five-cycle second and two-second gap exercises the same state
// machine that uses 50,000,000 cycles and 60 seconds on the board.
//=============================================================================
module tb_voice_repeat_ctrl;

    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    // Free running system clock counter.  The gap measurement below is
    // derived from it, so it does not depend on how many simulation deltas a
    // particular event happens to be displayed in.
    integer sys_cycle = 0;
    always @(posedge clk)
        sys_cycle = sys_cycle + 1;

    reg        rst_n = 1'b0;
    reg        voice_ready = 1'b0;
    reg        stream_done = 1'b0;
    reg        stream_error = 1'b0;
    wire       start_pulse;
    wire       waiting;

    integer errors = 0;
    integer i;
    integer start_cycle;
    integer gap_ticks;

    voice_repeat_ctrl #(
        .GAP_TICK_CYCLES (26'd5)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .gap_seconds  (8'd2),
        .voice_ready  (voice_ready),
        .stream_done  (stream_done),
        .stream_error (stream_error),
        .start_pulse  (start_pulse),
        .waiting      (waiting)
    );

    task fail_now;
        input [8*48-1:0] text;
        begin
            errors = errors + 1;
            $display("FAIL: %0s", text);
        end
    endtask

    task expect_first_start;
        begin
            @(posedge clk);
            #6;
            if (!start_pulse)
                fail_now("first start pulse missing");
            @(posedge clk);
            #6;
            if (start_pulse)
                fail_now("start pulse lasted more than one cycle");
        end
    endtask

    task finish_and_wait_gap;
        begin
            @(negedge clk);
            stream_done = 1'b1;
            @(posedge clk);
            @(negedge clk);
            stream_done = 1'b0;          // one sys_clk pulse, as load_done is

            // Wait for the gap to be entered.  The flag is registered from
            // the state, so it appears one clock after the state change.
            i = 0;
            while (!waiting && (i < 8)) begin
                @(negedge clk);
                i = i + 1;
            end
            if (!waiting)
                fail_now("gap state did not start after stream_done");

            // Two seconds at five cycles per second is 10 system clocks.
            // Measure that from the free running counter, then check the
            // start pulse on the positive edge that follows the last gap
            // cycle.  This is a level sample, not an edge race.
            start_cycle = sys_cycle;
            while (!(!waiting && start_pulse)) begin
                @(negedge clk);
                if ((sys_cycle - start_cycle) > 20) begin
                    fail_now("repeat start pulse never appeared");
                    i = 1;
                    disable finish_and_wait_gap;
                end
            end
            gap_ticks = sys_cycle - start_cycle;
            if (gap_ticks != 10)
                fail_now("gap length was not 10 system clocks");

            // Leave the line low for the next round.
            @(negedge clk);
        end
    endtask

    initial begin
        if ($test$plusargs("mon")) begin
            $display("time cyc state tick sec ready done err start wait");
            $monitor("%0t %0d %0d %0d %0d %b %b %b %b %b",
                     $time, sys_cycle, dut.state, dut.tick_cnt, dut.second_cnt,
                     voice_ready, stream_done, stream_error,
                     start_pulse, waiting);
        end
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        // No start before the WAV file and picture cache are ready.
        repeat (5) @(posedge clk);
        #6;
        if (start_pulse)
            fail_now("start occurred before voice_ready");

        // First announcement starts as soon as the ready flag is high.
        voice_ready = 1'b1;
        expect_first_start;

        // First successful announcement schedules the second one.
        finish_and_wait_gap;

        // The second successful announcement schedules the third one too.
        finish_and_wait_gap;

        // A parser error stops the repeat loop.
        @(negedge clk);
        stream_error = 1'b1;
        @(posedge clk);
        #6;
        stream_error = 1'b0;
        repeat (40) @(posedge clk);
        #6;
        if (start_pulse)
            fail_now("start occurred after a stream error");

        if (errors == 0)
            $display("VOICE REPEAT TB: PASS");
        else
            $display("VOICE REPEAT TB: FAIL with %0d error(s)", errors);
        $finish;
    end

endmodule
