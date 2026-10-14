`timescale 1ns / 1ps
//=============================================================================
// tb_fade_ctrl.v - focused regression for the frame-boundary fade controller
//
// Checks:
//   1. the first real picture fades in from black
//   2. a KEY1 request fades out, issues exactly one swap and fades back in
//   3. a second request arriving during fade-in is queued, not lost
//   4. the queued request starts a second fade after the first fade-in ends
//=============================================================================
module tb_fade_ctrl;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg frame_tick = 1'b0;
    reg request_pulse = 1'b0;
    reg image_change_pulse = 1'b0;

    wire       issue_swap;
    wire [4:0] level;

    integer swap_pulses = 0;

    fade_ctrl #(
        .STEP (4)
    ) dut (
        .clk                (clk),
        .rst                (rst),
        .frame_tick         (frame_tick),
        .request_pulse      (request_pulse),
        .issue_swap         (issue_swap),
        .image_change_pulse (image_change_pulse),
        .level              (level)
    );

    always #5 clk = ~clk;

    always @(negedge clk) begin
        if (issue_swap)
            swap_pulses = swap_pulses + 1;
    end

    task do_frame_tick;
        begin
            @(negedge clk);
            frame_tick = 1'b1;
            @(negedge clk);
            frame_tick = 1'b0;
        end
    endtask

    task pulse_request;
        begin
            @(negedge clk);
            request_pulse = 1'b1;
            @(negedge clk);
            request_pulse = 1'b0;
        end
    endtask

    task pulse_image_change;
        begin
            @(negedge clk);
            image_change_pulse = 1'b1;
            @(negedge clk);
            image_change_pulse = 1'b0;
        end
    endtask

    task check_level;
        input [4:0] expected;
        input [8*40-1:0] label;
        begin
            if (level !== expected) begin
                $display("FAIL: %0s level=%0d expected=%0d", label,
                         level, expected);
                $fatal;
            end
        end
    endtask

    task check_swaps;
        input integer expected;
        input [8*40-1:0] label;
        begin
            if (swap_pulses !== expected) begin
                $display("FAIL: %0s swaps=%0d expected=%0d", label,
                         swap_pulses, expected);
                $fatal;
            end
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst = 1'b0;
        repeat (2) @(negedge clk);
        check_level(5'd31, "reset");

        // First real picture: start at black and fade up over eight frames.
        pulse_image_change();
        check_level(5'd0, "first image starts black");
        repeat (8) do_frame_tick();
        check_level(5'd31, "first image fully visible");
        check_swaps(0, "first image has no swap");

        // A normal request must darken, issue one swap and wait for image.
        pulse_request();
        repeat (8) do_frame_tick();
        check_level(5'd0, "first request reaches black");
        check_swaps(1, "first request issues one swap");
        pulse_image_change();
        repeat (8) do_frame_tick();
        check_level(5'd31, "first swap fades in");
        check_swaps(1, "first swap count stays one");

        // Request another swap, then queue a third request during its fade-in.
        pulse_request();
        repeat (8) do_frame_tick();
        check_swaps(2, "second request issues one swap");
        pulse_image_change();

        repeat (2) do_frame_tick();
        check_level(5'd8, "second swap is partially faded in");
        pulse_request();

        // The queued request must not interrupt fade-in.  The level should
        // reach full brightness without issuing another swap yet.
        repeat (6) do_frame_tick();
        check_level(5'd31, "queued request leaves fade-in stable");
        check_swaps(2, "queued request waits for fade-in");

        // The queue then starts a second fade-out and issues one swap.
        repeat (8) do_frame_tick();
        check_level(5'd0, "queued request reaches black");
        check_swaps(3, "queued request issues one swap");

        $display("RESULT: PASS");
        $finish;
    end

endmodule
