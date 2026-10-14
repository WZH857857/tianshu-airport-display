//===========================================================================
// tb_board_top.v -- board-level integration test for fall_camera_top
//
// A behavioural OV5640 drives the real DVP byte stream (VSYNC pulse, HREF
// high for 1280 byte clocks per line, two bytes per RGB565 pixel) into the
// full board wrapper.  The bench then checks every observable the board
// gives us before HDMI exists:
//
//   SCCB 250-write sequence -> cfg_done
//   capture arming          -> armed, one frame_start per armed frame
//   640x480 RGB565 stream   -> exactly 307,200 de pulses per armed frame,
//                              hw_err stays 0
//   vision core             -> one bbox_valid pulse per armed frame
//   24 MHz status snapshot  -> 115200 UART line, decoded and printed
//   74HC595 LEDs            -> shift protocol decoded, cfg_done bit set
//
// The scene is a moving bright rectangle on a dark background, so the second
// armed frame already carries real motion.  +LONG replays a standing blob
// that then collapses (falls) and additionally expects the FSM to reach
// PENDING and then FALL with an alarm pulse.
//
// plusargs:  +LONG  +VCD
//
// Runtime is dominated by pixels: 307,200 pclk per frame.  The SCCB rate is
// raised to 2 MHz through the SCCB_HZ parameter so the 250 configuration
// writes cost ~4 ms of 24 MHz time instead of 36 ms (board default is
// unchanged at 250 kHz).
//===========================================================================
`timescale 1ns / 1ps

module tb_board_top;

    localparam integer SW         = 640;
    localparam integer SH         = 480;
    localparam integer ARMED      = 3;          // armed frames, short mode
    localparam integer ARMED_LONG = 8;          // armed frames, +LONG mode
    localparam integer BLANK      = 4000;       // pclk after the last line
    localparam integer UDIV       = 208;        // 24 MHz / 115200

    reg clk_24m = 1'b0;
    reg cmos_pclk = 1'b0;
    always #20.8333 clk_24m  = ~clk_24m;
    always #10.4167 cmos_pclk = ~cmos_pclk;

    reg        key_n       = 1'b1;
    wire       cmos_xclk;
    wire       cmos_sclk;
    wire       cmos_sdat;
    reg  [7:0] cmos_data   = 8'h00;
    reg        cmos_href   = 1'b0;
    reg        cmos_vsync  = 1'b0;
    wire       led595_clk;
    wire       led595_latch;
    wire       led595_dout;
    wire       uart_txd;

    // The camera module has its own SCCB pull-ups on the board.
    pullup pu_sda (cmos_sdat);

    fall_camera_top #(
        .POWERUP_MS  (1),
        .RESET_MS    (1),
        .WAIT_FRAMES (1),
        .SCCB_HZ     (2000000),
        .BAUD        (115200),
        .POR_CYCLES  (20000)
    ) u_dut (
        .clk_24m      (clk_24m),
        .key_n        (key_n),
        .cmos_xclk    (cmos_xclk),
        .cmos_sclk    (cmos_sclk),
        .cmos_sdat    (cmos_sdat),
        .cmos_data    (cmos_data),
        .cmos_pclk    (cmos_pclk),
        .cmos_href    (cmos_href),
        .cmos_vsync   (cmos_vsync),
        .led595_clk   (led595_clk),
        .led595_latch (led595_latch),
        .led595_dout  (led595_dout),
        .uart_txd     (uart_txd)
    );

    integer errors = 0;
    task check(input cond, input [255:0] what);
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL t=%0t %0s", $time, what);
                $fflush;
            end
        end
    endtask

    // ------------------------------------------------- pixel / frame monitors
    integer de_total    = 0;
    integer fs_total    = 0;
    integer bbox_total  = 0;
    integer alarm_total = 0;

    always @(posedge cmos_pclk) begin
        if (u_dut.de_565) de_total <= de_total + 1;
        if (u_dut.frame_start_565) begin
            fs_total <= fs_total + 1;
            if (cmos_href)
                $display("FAIL t=%0t frame_start while HREF active", $time);
        end
        if (u_dut.bbox_valid) begin
            bbox_total <= bbox_total + 1;
            $display("BBOX    t=%0t frame=%0d found=%b x=%0d y=%0d w=%0d h=%0d area=%0d state=%0d",
                     $time, u_dut.frame_cnt, u_dut.bbox_found, u_dut.bbox_x,
                     u_dut.bbox_y, u_dut.bbox_w, u_dut.bbox_h,
                     u_dut.bbox_area, u_dut.posture_state);
        end
        if (u_dut.alarm) begin
            alarm_total <= alarm_total + 1;
            $display("ALARM   t=%0t frame=%0d", $time, u_dut.frame_cnt);
        end
    end

    // ------------------------------------------------------------ UART decode
    reg        rx_busy = 1'b0;
    reg        rx_first = 1'b0;
    reg [15:0] rx_cnt = 16'd0;
    reg [3:0]  rx_bit = 4'd0;
    reg [7:0]  rx_byte = 8'd0;
    reg [7:0]  line_buf [0:63];
    integer    line_len = 0;
    integer    line_count = 0;
    integer    line_bad = 0;

    task line_flush;
        integer i;
        begin
            if (line_len > 1) begin
                line_count = line_count + 1;
                $write("UART#%0d: ", line_count);
                for (i = 0; i < line_len; i = i + 1)
                    $write("%c", line_buf[i]);
                $write("\n");
                if (line_buf[0] !== 8'h46) begin   // 'F'
                    line_bad = line_bad + 1;
                    $display("FAIL malformed UART line (no leading F)");
                end
                if (line_len !== 31) begin
                    line_bad = line_bad + 1;
                    $display("FAIL UART line length %0d, want 31 chars", line_len);
                end
            end
            line_len = 0;
        end
    endtask

    always @(posedge clk_24m) begin
        if (!rx_busy) begin
            if (!uart_txd) begin
                rx_busy  <= 1'b1;
                rx_first <= 1'b1;
                rx_cnt   <= 16'd0;
                rx_bit   <= 4'd0;
            end
        end else if (rx_first) begin
            // stop+half a bit: land in the middle of data bit 0
            if (rx_cnt == (UDIV + UDIV/2 - 1)) begin
                rx_first  <= 1'b0;
                rx_cnt    <= 16'd0;
                rx_byte[0] <= uart_txd;
                rx_bit    <= 4'd1;
            end else begin
                rx_cnt <= rx_cnt + 16'd1;
            end
        end else if (rx_bit < 4'd8) begin
            if (rx_cnt == UDIV - 1) begin
                rx_cnt    <= 16'd0;
                rx_byte[rx_bit] <= uart_txd;
                rx_bit    <= rx_bit + 4'd1;
            end else begin
                rx_cnt <= rx_cnt + 16'd1;
            end
        end else begin
            if (rx_cnt == UDIV/2 - 1) begin
                rx_busy <= 1'b0;
                if (rx_byte == 8'h0A)      line_flush;
                else if (rx_byte == 8'h0D) ;          // CR: not part of the text
                else if (line_len < 63) begin
                    line_buf[line_len] <= rx_byte;
                    line_len = line_len + 1;
                end
            end else begin
                rx_cnt <= rx_cnt + 16'd1;
            end
        end
    end

    // ------------------------------------------------------- 74HC595 decode
    reg [7:0] led_shift = 8'h00;
    reg [7:0] led_shown = 8'h00;
    always @(posedge led595_clk) led_shift <= {led_shift[6:0], led595_dout};
    always @(posedge led595_latch) begin
        led_shown <= led_shift;
        $display("LED595  t=%0t latched %02h (dut %02h)", $time, led_shift,
                 u_dut.led_data);
        $fflush;
    end

    // ---------------------------------------------------------- camera model
    //
    // Short scene: one bright rectangle that walks 16 source pixels per
    // frame, so frame_diff sees a leading and a trailing strip (this is what
    // the real detector keys on) and the bounding box of one strip is what
    // gets reported.
    //
    // +LONG scene: a standing person (80x160 source -> 40x80 working) that
    // then collapses into a wide flat blob (160x40 -> 80x20) lower in the
    // frame.  The person blinks between luma 255 and luma 14 every frame, so
    // the *whole* body lands in the motion mask instead of just the moving
    // edge -- without that, posture_fsm never sees a person-sized reference
    // and the collapse can never be confirmed.
    integer f, y, x0, x1;
    integer s_rx, s_ry, s_rw, s_rh;
    reg [15:0] s_fg;
    reg long_mode;

    task scene_for(input integer fi);
        begin
            if (!long_mode) begin
                s_rx = 100 + 16*fi; s_ry = 60;  s_rw = 96;  s_rh = 200;
                s_fg = 16'hFFFF;
            end else if (fi <= 2) begin
                s_rx = 140;         s_ry = 80;  s_rw = 80;  s_rh = 160;
                s_fg = fi[0] ? 16'hFFFF : 16'h0010;
            end else begin
                s_rx = 100;         s_ry = 220; s_rw = 160; s_rh = 40;
                s_fg = fi[0] ? 16'hFFFF : 16'h0010;
            end
        end
    endtask

    // One pixel = two bytes, high byte first, driven on the PCLK falling edge
    // exactly like the sensor (HREF rises with the first byte).
    task send_seg(input integer n, input [15:0] v);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(negedge cmos_pclk); cmos_href = 1'b1; cmos_data = v[15:8];
                @(negedge cmos_pclk);                     cmos_data = v[7:0];
            end
        end
    endtask

    task send_row;
        input integer yy;
        begin
            x0 = (yy >= s_ry && yy < s_ry + s_rh) ? s_rx : 0;
            x1 = (yy >= s_ry && yy < s_ry + s_rh) ? s_rx + s_rw : 0;
            send_seg(x0,        16'h0010);      // dark background
            send_seg(x1 - x0,   s_fg);          // person
            send_seg(SW - x1,   16'h0010);
            @(negedge cmos_pclk); cmos_href = 1'b0; cmos_data = 8'h00;
        end
    endtask

    task send_frame;
        begin
            scene_for(f);
            $display("INFO t=%0t sending frame f=%0d (%0s)", $time, f,
                     long_mode ? "stand/fall" : "moving blob");
            $fflush;
            @(negedge cmos_pclk);
            cmos_vsync = 1'b1; cmos_href = 1'b0; cmos_data = 8'h00;
            repeat (20) @(negedge cmos_pclk);
            cmos_vsync = 1'b0;
            repeat (20) @(negedge cmos_pclk);
            for (y = 0; y < SH; y = y + 1) begin
                send_row(y);
                repeat (8) @(negedge cmos_pclk);
            end
            repeat (BLANK) @(negedge cmos_pclk);
        end
    endtask

    integer armed_need;
    integer wait_loops;

    initial begin
        long_mode = 0;
        if ($test$plusargs("LONG"))  long_mode = 1;
        if ($test$plusargs("VCD")) begin
            $dumpfile("tb_board_top.vcd");
            $dumpvars(0, tb_board_top);
        end

        armed_need = long_mode ? ARMED_LONG : ARMED;

        @(posedge u_dut.rst_n);
        $display("INFO t=%0t reset released, waiting for SCCB configuration", $time);
        $fflush;

        wait_loops = 0;
        while (u_dut.cfg_done !== 1'b1 && wait_loops < 400000) begin
            @(negedge clk_24m);
            wait_loops = wait_loops + 1;
        end
        $display("INFO t=%0t cfg_done=%b wr_count=%0d after %0d clocks",
                 $time, u_dut.cfg_done, u_dut.wr_count, wait_loops);
        $fflush;
        check(u_dut.cfg_done === 1'b1, "SCCB configuration completes");
        check(u_dut.wr_count == 8'd250, "250 SCCB writes completed");

        // One dummy frame is swallowed while the capture arms.
        f = 0;
        send_frame;
        $display("INFO t=%0t dummy frame done, armed=%b", $time, u_dut.armed);
        $fflush;
        for (f = 1; f <= armed_need; f = f + 1) begin
            send_frame;
            $display("INFO t=%0t frame %0d done, frame_cnt=%0d de=%0d bbox=%0d",
                     $time, f, u_dut.frame_cnt, de_total, bbox_total);
            $fflush;
        end

        // Let the last bbox pass, the UART line and the LED write finish.
        wait_loops = 0;
        while (!(line_count == bbox_total && bbox_total >= armed_need) &&
               wait_loops < 1000000) begin
            @(negedge clk_24m);
            wait_loops = wait_loops + 1;
        end
        repeat (2000) @(negedge clk_24m);
        $display("INFO t=%0t drain done: bbox=%0d uart=%0d", $time, bbox_total, line_count);
        $fflush;

        check(u_dut.cfg_done == 1'b1, "cfg_done stays high");
        check(u_dut.armed    == 1'b1, "capture armed");
        check(u_dut.hw_err   == 8'h00, "no capture geometry error");
        check(fs_total == armed_need, "one frame_start per armed frame");
        check(de_total == SW * SH * armed_need, "307200 de pulses per armed frame");
        check(bbox_total == armed_need, "one bbox_valid pulse per armed frame");
        check(line_count >= armed_need, "one UART line per bbox observation");
        check(line_bad == 0, "UART line format");
        check(led_shown[0] == 1'b1, "LED0 reports cfg_done");

        if (long_mode) begin
            check(u_dut.posture_state == 3'd2 || u_dut.posture_state == 3'd3,
                  "posture machine left SEEK after the collapse");
            check(alarm_total >= 1, "fall alarm pulse");
        end

        if (errors == 0)
            $display("PASS tb_board_top: cfg+%0d frames, de=%0d, bbox=%0d, uart=%0d, hw_err=%02h",
                     armed_need, de_total, bbox_total, line_count, u_dut.hw_err);
        else
            $display("FAIL tb_board_top: %0d errors", errors);
        $finish;
    end

    initial begin
        #1_000_000_000;                       // 1 s of simulated time
        $display("FAIL tb_board_top: timeout at t=%0t", $time);
        $finish;
    end

endmodule
