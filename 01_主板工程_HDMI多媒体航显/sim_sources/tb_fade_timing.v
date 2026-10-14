`timescale 1ns / 1ps
//=============================================================================
// tb_fade_timing.v - ADV02 transition timing with the values the wrapper uses
//
// The ADV02 wrapper instantiates fade_ctrl with STEP = 2 and a 180-frame
// watchdog.  This test measures what that means in video frames:
//
//   1. the first picture fades in from black
//   2. a swap request fades out, swaps exactly once at black, fades back in
//   3. if the new picture never arrives the watchdog fades back in anyway,
//      so the screen cannot stay black forever
//=============================================================================
module tb_fade_timing;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg frame_tick = 1'b0;
    reg request_pulse = 1'b0;
    reg image_change_pulse = 1'b0;

    wire       issue_swap;
    wire [4:0] level;

    integer ticks;
    integer swaps = 0;
    integer errors = 0;

    fade_ctrl #(
        .STEP         (2),
        .WAIT_TIMEOUT (180)
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

    always @(negedge clk)
        if (issue_swap)
            swaps = swaps + 1;

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

    task wait_bright;
        begin
            ticks = 0;
            while ((level != 5'd31) && (ticks < 400)) begin
                do_frame_tick();
                ticks = ticks + 1;
            end
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst = 1'b0;
        repeat (2) @(negedge clk);

        // 1. first picture fades up from black
        pulse_image_change();
        if (level !== 5'd0) begin
            $display("FAIL: first picture did not start from black (level=%0d)",
                     level);
            errors = errors + 1;
        end
        wait_bright();
        $display("ADV02 TB: first fade-in = %0d frames (expect 16)", ticks);
        if (ticks != 16) errors = errors + 1;

        // 2. normal swap: fade out, one swap at black, fade in
        pulse_request();
        ticks = 0;
        while ((swaps == 0) && (ticks < 200)) begin
            do_frame_tick();
            ticks = ticks + 1;
        end
        $display("ADV02 TB: fade-out = %0d frames, swaps = %0d, level at swap = %0d",
                 ticks, swaps, level);
        if (ticks != 16) errors = errors + 1;
        if (swaps != 1) errors = errors + 1;
        if (level !== 5'd0) errors = errors + 1;

        pulse_image_change();
        wait_bright();
        $display("ADV02 TB: fade-in = %0d frames (expect 16)", ticks);
        if (ticks != 16) errors = errors + 1;
        if (swaps != 1) errors = errors + 1;

        // 3. watchdog: the new picture never arrives
        pulse_request();
        ticks = 0;
        while ((swaps == 1) && (ticks < 300)) begin
            do_frame_tick();
            ticks = ticks + 1;
        end
        if (ticks != 16) begin
            $display("FAIL: second fade-out took %0d frames", ticks);
            errors = errors + 1;
        end
        wait_bright();
        $display("ADV02 TB: watchdog returned to full brightness in %0d frames (expect 196)",
                 ticks);
        if ((ticks < 190) || (ticks > 200)) errors = errors + 1;
        if (swaps != 2) errors = errors + 1;

        if (errors == 0)
            $display("RESULT: PASS tb_fade_timing");
        else
            $display("RESULT: FAIL tb_fade_timing errors=%0d", errors);

        $finish;
    end

endmodule
