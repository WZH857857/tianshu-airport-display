//===========================================================================
// tb_cmos_capture.v -- DVP RGB565 byte stream -> pixel stream
//
// A behavioural OV5640 drives VSYNC (active high pulse), HREF (high during
// a line) and two bytes per pixel.  Every pixel carries its own index, so a
// byte-pair slip shows up as a wrong value, not just a wrong count.
//
// Defaults run a tiny frame; the same bench is re-run at 640x480 with
// parameter overrides:
//   -Ptb_cmos_capture.H_ACTIVE=640 -P...V_ACTIVE=480
//   -P...WAIT_FRAMES=1 -P...TOTAL_FRAMES=2
//
// Checks: de only after arming, one de per pixel, assembled {first,second}
// order, exactly one frame_start per armed frame and always in blanking,
// 640x480 counters, no hw_err on a well-formed stream, and hw_err[0] on a
// deliberately short line.
//===========================================================================
`timescale 1ns / 1ps

module tb_cmos_capture;

    parameter integer H_ACTIVE     = 8;
    parameter integer V_ACTIVE     = 4;
    parameter integer WAIT_FRAMES  = 2;
    parameter integer TOTAL_FRAMES = 5;

    reg pclk = 1'b0;
    reg rst_n = 1'b0;
    reg arm = 1'b0;
    always #10.4 pclk = ~pclk;                 // 48 MHz

    reg        vsync = 1'b0;
    reg        href  = 1'b0;
    reg [7:0]  data  = 8'h00;

    wire        de;
    wire [15:0] pix;
    wire        frame_start;
    wire        armed;
    wire [15:0] frame_cnt;
    wire [9:0]  line_cnt;
    wire [10:0] pix_cnt;
    wire [7:0]  hw_err;

    cmos_capture_rgb565 #(
        .H_ACTIVE    (H_ACTIVE),
        .V_ACTIVE    (V_ACTIVE),
        .WAIT_FRAMES (WAIT_FRAMES)
    ) dut (
        .pclk        (pclk),
        .rst_n       (rst_n),
        .arm         (arm),
        .vsync       (vsync),
        .href        (href),
        .data        (data),
        .de          (de),
        .pix         (pix),
        .frame_start (frame_start),
        .armed       (armed),
        .frame_cnt   (frame_cnt),
        .line_cnt    (line_cnt),
        .pix_cnt     (pix_cnt),
        .hw_err      (hw_err)
    );

    integer errors = 0;
    integer de_total = 0;
    integer fs_total = 0;
    integer exp_idx = 0;
    reg     check_pixels = 1'b1;

    task check(input cond, input [255:0] what);
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL t=%0t %0s", $time, what);
            end
        end
    endtask

    // Pixel index carried by the byte pair: high byte first.
    always @(posedge pclk) begin
        if (de) begin
            de_total <= de_total + 1;
            if (check_pixels && (pix !== exp_idx[15:0])) begin
                errors = errors + 1;
                if (errors < 12)
                    $display("FAIL t=%0t pixel %0d got %04h want %04h",
                             $time, de_total, pix, exp_idx[15:0]);
            end
            exp_idx <= exp_idx + 1;
        end
        if (frame_start) begin
            fs_total <= fs_total + 1;
            exp_idx  <= 0;
            if (href) begin
                errors = errors + 1;
                $display("FAIL t=%0t frame_start while HREF active", $time);
            end
        end
    end

    // ------------------------------------------------------- camera model
    integer x, y, i;
    integer k;
    reg [15:0] sidx;

    // One line: HREF rises together with the first data byte, exactly like
    // the sensor drives it (both change on the PCLK falling edge).
    task send_line(input integer npix, input integer skip_last);
        begin
            for (k = 0; k < npix; k = k + 1) begin
                if (!(skip_last && (k == npix - 1))) begin
                    @(negedge pclk); href = 1'b1; data = sidx[15:8];
                    @(negedge pclk);              data = sidx[7:0];
                    sidx = sidx + 16'd1;
                end
            end
            @(negedge pclk); href = 1'b0; data = 8'h00;
        end
    endtask

    task send_frame(input integer npix, input integer nlines, input integer shorten);
        begin
            // VSYNC leading edge: frame boundary, well inside blanking
            @(negedge pclk); vsync = 1'b1; href = 1'b0; data = 8'h00;
            repeat (20) @(negedge pclk);
            vsync = 1'b0;
            repeat (20) @(negedge pclk);

            sidx = 16'd0;
            for (y = 0; y < nlines; y = y + 1) begin
                send_line(npix, (shorten && (y == 1)));
                repeat (20) @(negedge pclk);
            end
            repeat (40) @(negedge pclk);
        end
    endtask

    initial begin
        if ($test$plusargs("dump")) begin
            $dumpfile("tb_cmos_capture.vcd");
            $dumpvars(0, tb_cmos_capture);
        end

        repeat (10) @(negedge pclk);
        rst_n = 1'b1;
        repeat (10) @(negedge pclk);

        // unarmed frames: de must stay quiet
        send_frame(H_ACTIVE, V_ACTIVE, 0);
        check(de_total == 0, "no de before armed");

        arm = 1'b1;

        // remaining well-formed frames
        for (i = 1; i < TOTAL_FRAMES; i = i + 1)
            send_frame(H_ACTIVE, V_ACTIVE, 0);

        repeat (20) @(negedge pclk);

        check(armed == 1'b1, "armed after WAIT_FRAMES");
        check(hw_err == 8'h00, "no hw_err on clean stream");
        check(fs_total == TOTAL_FRAMES - WAIT_FRAMES, "one frame_start per armed frame");
        check(frame_cnt == (TOTAL_FRAMES - WAIT_FRAMES), "frame_cnt");
        check(de_total == H_ACTIVE * V_ACTIVE * (TOTAL_FRAMES - WAIT_FRAMES),
              "de count = H_ACTIVE*V_ACTIVE per armed frame");
        check(pix_cnt == H_ACTIVE, "last line pixel counter");

        // ------------------------------------------------ malformed frame
        if (H_ACTIVE > 2 && V_ACTIVE > 1) begin
            check_pixels = 1'b0;
            send_frame(H_ACTIVE, V_ACTIVE, 1);      // one pixel missing
            repeat (20) @(negedge pclk);
            check_pixels = 1'b1;
            check(hw_err[0] == 1'b1, "short line flagged");
        end

        if (errors == 0)
            $display("PASS tb_cmos_capture: %0dx%0d, %0d armed frames, %0d pixels",
                     H_ACTIVE, V_ACTIVE, TOTAL_FRAMES - WAIT_FRAMES, de_total);
        else
            $display("FAIL tb_cmos_capture: %0d errors", errors);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("FAIL tb_cmos_capture: timeout");
        $finish;
    end

endmodule
