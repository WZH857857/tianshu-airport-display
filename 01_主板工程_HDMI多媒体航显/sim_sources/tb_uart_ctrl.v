`timescale 1ns / 1ps
//=============================================================================
// tb_uart_ctrl.v
//
// Drives the serial command handler with real 8N1 waveforms and checks both
// directions:
//
//   * PING, TEXT, SLOT, MODE, PARAM, OSD and STATUS replies carry the right
//     header, length, payload and checksum
//   * a corrupted frame changes nothing and produces no reply
//   * the text replay handshake hands over exactly the bytes that were sent
//   * an uncached slot is refused instead of changing the picture
//
// The divider is shrunk to eight clocks per bit so the test stays short; the
// protocol itself is rate independent.
//=============================================================================
module tb_uart_ctrl;

    localparam integer BIT_CLKS = 8;

    reg  clk = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    reg  rst_n  = 1'b0;
    reg  rx_pin = 1'b1;
    wire tx_pin;

    wire [4:0] text_addr;
    wire [7:0] text_data;
    wire       text_tog, text_clear_tog, text_commit_tog;
    wire [5:0] text_len;
    wire       jump_req;
    wire [2:0] jump_slot;
    wire       auto_ovr_en, auto_ovr_mode;
    wire       bright_we, contrast_we, osd_we;
    wire [2:0] bright_val, contrast_val;
    wire [1:0] osd_val;
    wire       link_seen;
    wire       board_tog;
    wire [7:0] board_addr, board_data;
    wire       board_enable;
    wire [7:0] time_hh, time_mm, time_ss;
    wire       time_tog;

    reg [2:0] st_slot     = 3'd1;
    reg [7:0] st_total    = 8'd6;
    reg [5:0] st_cache    = 6'b101010;
    reg [2:0] st_bright   = 3'd7;
    reg [2:0] st_contrast = 3'd3;
    reg [1:0] st_osd      = 2'd0;
    reg       st_auto     = 1'b0;
    reg       st_card_ok  = 1'b1;

    integer errors = 0;

    uart_ctrl #(
        .DEFAULT_DIV (BIT_CLKS),
        .VERSION     (8'h11)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .rx_pin          (rx_pin),
        .tx_pin          (tx_pin),
        .text_addr       (text_addr),
        .text_data       (text_data),
        .text_tog        (text_tog),
        .text_clear_tog  (text_clear_tog),
        .text_commit_tog (text_commit_tog),
        .text_len        (text_len),
        .jump_req        (jump_req),
        .jump_slot       (jump_slot),
        .auto_ovr_en     (auto_ovr_en),
        .auto_ovr_mode   (auto_ovr_mode),
        .bright_we       (bright_we),
        .bright_val      (bright_val),
        .contrast_we     (contrast_we),
        .contrast_val    (contrast_val),
        .osd_we          (osd_we),
        .osd_val         (osd_val),
        .board_tog       (board_tog),
        .board_addr      (board_addr),
        .board_data      (board_data),
        .board_enable    (board_enable),
        .time_hh         (time_hh),
        .time_mm         (time_mm),
        .time_ss         (time_ss),
        .time_tog        (time_tog),
        .baud_div        (),
        .link_seen       (link_seen),
        .st_slot         (st_slot),
        .st_total        (st_total),
        .st_cache        (st_cache),
        .st_bright       (st_bright),
        .st_contrast     (st_contrast),
        .st_osd          (st_osd),
        .st_auto         (st_auto),
        .st_card_ok      (st_card_ok)
    );

    //------------------------------------------------------------------
    // host side helpers, eight clocks per bit
    //------------------------------------------------------------------
    task host_bit;
        input b;
        begin
            rx_pin = b;
            repeat (BIT_CLKS) @(posedge clk);
        end
    endtask

    task host_byte;
        input [7:0] d;
        integer i;
        begin
            host_bit(1'b0);                     // start
            for (i = 0; i < 8; i = i + 1)
                host_bit(d[i]);
            host_bit(1'b1);                     // stop
        end
    endtask

    // host_send2 drives a frame whose payload is at most two bytes
    task host_send2;
        input [7:0] cmd;
        input [7:0] len;
        input [7:0] p0;
        input [7:0] p1;
        reg [7:0] sum;
        begin
            sum = cmd ^ len;
            host_byte(8'hA5);
            host_byte(8'h5A);
            host_byte(cmd);
            host_byte(len);
            if (len > 8'd0) begin
                host_byte(p0);
                sum = sum ^ p0;
            end
            if (len > 8'd1) begin
                host_byte(p1);
                sum = sum ^ p1;
            end
            host_byte(sum);
        end
    endtask

    reg [7:0] text_buf [0:31];
    reg [7:0] board_buf [0:39];

    task host_send_text;
        input [7:0] n;
        integer i;
        reg [7:0] sum;
        begin
            sum = 8'h02 ^ n;
            host_byte(8'hA5);
            host_byte(8'h5A);
            host_byte(8'h02);
            host_byte(n);
            for (i = 0; i < n; i = i + 1) begin
                host_byte(text_buf[i]);
                sum = sum ^ text_buf[i];
            end
            host_byte(sum);
        end
    endtask

    // BOARD carries one row selector and forty ASCII glyphs.
    task host_send_board;
        input [7:0] row;
        integer i;
        reg [7:0] sum;
        begin
            sum = 8'h20 ^ 8'd41 ^ row;
            host_byte(8'hA5);
            host_byte(8'h5A);
            host_byte(8'h20);
            host_byte(8'd41);
            host_byte(row);
            for (i = 0; i < 40; i = i + 1) begin
                host_byte(board_buf[i]);
                sum = sum ^ board_buf[i];
            end
            host_byte(sum);
        end
    endtask

    // TIME carries three BCD bytes: hour, minute and second.
    task host_send_time;
        input [7:0] hh;
        input [7:0] mm;
        input [7:0] ss;
        reg [7:0] sum;
        begin
            sum = 8'h22 ^ 8'd3 ^ hh ^ mm ^ ss;
            host_byte(8'hA5);
            host_byte(8'h5A);
            host_byte(8'h22);
            host_byte(8'd3);
            host_byte(hh);
            host_byte(mm);
            host_byte(ss);
            host_byte(sum);
        end
    endtask

    //------------------------------------------------------------------
    // capture what the board sends back
    //------------------------------------------------------------------
    reg [7:0] reply [0:31];
    integer   reply_n = 0;

    // A receiver on the same clock as the design.  It re-arms on the falling
    // edge of every start bit and samples the middle of each data bit, so it
    // stays locked even when the board sends bytes back to back.
    reg        recv      = 1'b0;
    reg        wait_idle = 1'b0;
    integer    cnt       = 0;
    integer    bitn      = 0;
    reg [7:0]  vv        = 8'd0;

    always @(posedge clk) begin
        if (!recv) begin
            wait_idle = 1'b0;
            cnt       = 0;
            bitn      = 0;
            if (tx_pin === 1'b0)
                recv = 1'b1;
        end else if (wait_idle) begin
            if (tx_pin === 1'b1)        // stop bit reached, re-arm
                recv = 1'b0;
        end else begin
            // +1 because bitn 0 is the first data bit, which starts one whole
            // bit time after the start-bit edge that armed this receiver
            if (cnt == (BIT_CLKS / 2 + (bitn + 1) * BIT_CLKS)) begin
                vv[bitn] = tx_pin;
                if (bitn == 7) begin
                    reply[reply_n] = vv;
                    reply_n        = reply_n + 1;
                    wait_idle      = 1'b1;
                end else begin
                    bitn = bitn + 1;
                end
            end
            cnt = cnt + 1;
        end
    end

    //------------------------------------------------------------------
    // one clock strobes are easy to miss after waiting for a reply, so they
    // are latched here and checked afterwards
    //------------------------------------------------------------------
    reg       seen_bright, seen_contrast, seen_osd, seen_jump;
    reg [2:0] seen_bright_v, seen_contrast_v, seen_jump_v;
    reg [1:0] seen_osd_v;

    task clear_seen;
        begin
            seen_bright   = 1'b0;
            seen_contrast = 1'b0;
            seen_osd      = 1'b0;
            seen_jump     = 1'b0;
        end
    endtask

    always @(posedge clk) begin
        if (bright_we) begin
            seen_bright   = 1'b1;
            seen_bright_v = bright_val;
        end
        if (contrast_we) begin
            seen_contrast   = 1'b1;
            seen_contrast_v = contrast_val;
        end
        if (osd_we) begin
            seen_osd   = 1'b1;
            seen_osd_v = osd_val;
        end
        if (jump_req) begin
            seen_jump   = 1'b1;
            seen_jump_v = jump_slot;
        end
    end

    task wait_reply;
        input integer n;
        integer guard;
        begin
            guard = 0;
            while ((reply_n < n) && (guard < 60000)) begin
                @(posedge clk);
                guard = guard + 1;
            end
            if (reply_n < n) begin
                $display("FAIL: reply timeout, expected %0d bytes, saw %0d",
                         n, reply_n);
                errors = errors + 1;
            end
        end
    endtask

    // check_frame waits for "n" bytes and verifies the header, the length
    // field, the checksum and the first "want_n" payload bytes.
    task check_frame;
        input integer n;
        input [7:0] want_cmd;
        input [7:0] want_len;
        input [7:0] want0;
        input [7:0] want1;
        input integer want_n;
        integer i;
        reg [7:0] sum;
        begin
            wait_reply(n);
            if (reply_n < n) begin
                reply_n = 0;
            end else begin
                if (reply[0] !== 8'hA5 || reply[1] !== 8'h5A) begin
                    $display("FAIL: reply header is %02x %02x", reply[0], reply[1]);
                    errors = errors + 1;
                end
                if (reply[2] !== want_cmd) begin
                    $display("FAIL: reply cmd %02x want %02x", reply[2], want_cmd);
                    errors = errors + 1;
                end
                if (reply[3] !== want_len) begin
                    $display("FAIL: reply len %02x want %02x", reply[3], want_len);
                    errors = errors + 1;
                end
                if ((reply[3] + 5) != n) begin
                    $display("FAIL: length byte %0d does not match frame %0d",
                             reply[3], n);
                    errors = errors + 1;
                end
                sum = 8'h00;
                for (i = 2; i < n - 1; i = i + 1)
                    sum = sum ^ reply[i];
                if (sum !== reply[n - 1]) begin
                    $display("FAIL: reply checksum %02x want %02x",
                             reply[n - 1], sum);
                    errors = errors + 1;
                end
                if (want_n > 0 && reply[4] !== want0) begin
                    $display("FAIL: reply payload0 %02x want %02x",
                             reply[4], want0);
                    errors = errors + 1;
                end
                if (want_n > 1 && reply[5] !== want1) begin
                    $display("FAIL: reply payload1 %02x want %02x",
                             reply[5], want1);
                    errors = errors + 1;
                end
                for (i = 0; i < 16; i = i + 1) reply[i] = 8'h00;
                reply_n = 0;
            end
        end
    endtask

    //------------------------------------------------------------------
    // text replay monitor.  It mirrors ui_overlay: bytes are staged as they
    // stream in, and the length the ticker reads only moves on the commit
    // edge that the controller raises after the checksum has passed.
    //------------------------------------------------------------------
    reg [7:0] got_char [0:31];
    integer   got_n      = 0;
    reg [5:0] ov_max     = 6'd0;
    reg [5:0] ov_len     = 6'd0;
    reg       tog_d, cmt_d, clr_d;
    integer   clr_events = 0;
    integer   cmt_events = 0;
    integer   board_writes = 0;
    reg [7:0] board_last_addr = 8'h00;
    reg [7:0] board_last_data = 8'h00;
    reg       board_tog_d = 1'b0;

    always @(posedge clk) begin
        if (!rst_n) begin
            tog_d      <= 1'b0;
            cmt_d      <= 1'b0;
            clr_d      <= 1'b0;
            got_n      <= 0;
            clr_events <= 0;
            cmt_events <= 0;
        end else begin
            tog_d <= text_tog;
            cmt_d <= text_commit_tog;
            clr_d <= text_clear_tog;
            if (text_tog ^ tog_d) begin
                got_char[text_addr] = text_data;
                got_n = got_n + 1;
                if ((text_addr + 6'd1) > ov_max)
                    ov_max = text_addr + 6'd1;
            end
            if (text_commit_tog ^ cmt_d) begin
                ov_len     = ov_max;
                cmt_events = cmt_events + 1;
            end
            if (text_clear_tog ^ clr_d) begin
                ov_len     = 6'd0;
                ov_max     = 6'd0;
                clr_events = clr_events + 1;
            end
            board_tog_d <= board_tog;
            if (board_tog ^ board_tog_d) begin
                board_writes   = board_writes + 1;
                board_last_addr = board_addr;
                board_last_data = board_data;
            end
        end
    end

    //------------------------------------------------------------------
    // stimulus
    //------------------------------------------------------------------
    integer i;

    initial begin
        $dumpfile("tb_uart_ctrl.vcd");
        $dumpvars(0, tb_uart_ctrl);

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // ---- PING: A5 5A 81 01 11 91 ------------------------------
        host_send2(8'h01, 8'd0, 8'd0, 8'd0);
        check_frame(6, 8'h81, 8'd1, 8'h11, 8'h00, 1);
        if (!link_seen) begin
            $display("FAIL: link_seen did not latch after a valid frame");
            errors = errors + 1;
        end

        // ---- TEXT: "GATE 3" ---------------------------------------
        text_buf[0] = 8'h47;    // G
        text_buf[1] = 8'h41;    // A
        text_buf[2] = 8'h54;    // T
        text_buf[3] = 8'h45;    // E
        text_buf[4] = 8'h20;    // space
        text_buf[5] = 8'h33;    // 3
        got_n      = 0;
        clr_events = 0;
        cmt_events = 0;
        host_send_text(8'd6);
        check_frame(7, 8'h82, 8'd2, 8'd6, 8'd6, 2);
        if (got_n != 6) begin
            $display("FAIL: text replay handed over %0d bytes, want 6", got_n);
            errors = errors + 1;
        end else if ((got_char[0] !== 8'h47) || (got_char[1] !== 8'h41) ||
                     (got_char[2] !== 8'h54) || (got_char[3] !== 8'h45) ||
                     (got_char[4] !== 8'h20) || (got_char[5] !== 8'h33)) begin
            $display("FAIL: text replay bytes are wrong");
            errors = errors + 1;
        end
        if (cmt_events != 1) begin
            $display("FAIL: text commit toggled %0d times, want 1", cmt_events);
            errors = errors + 1;
        end
        if (clr_events != 0) begin
            $display("FAIL: a good text frame blanked the ticker %0d times",
                     clr_events);
            errors = errors + 1;
        end
        if (ov_len != 6'd6) begin
            $display("FAIL: committed text length is %0d, want 6", ov_len);
            errors = errors + 1;
        end
        if (text_len != 6'd6) begin
            $display("FAIL: text_len is %0d, want 6", text_len);
            errors = errors + 1;
        end

        // ---- SLOT: jump to the cached slot 3 ----------------------
        clear_seen;
        host_send2(8'h03, 8'd1, 8'd3, 8'd0);
        check_frame(6, 8'h83, 8'd1, 8'd3, 8'h00, 1);
        if (!seen_jump || (seen_jump_v !== 3'd3)) begin
            $display("FAIL: slot jump was not requested");
            errors = errors + 1;
        end

        // ---- SLOT: a slot that is not cached must be refused -------
        clear_seen;
        host_send2(8'h03, 8'd1, 8'd0, 8'd0);       // cache bit 0 is 0
        check_frame(7, 8'hE0, 8'd2, 8'h03, 8'd3, 2);
        if (seen_jump) begin
            $display("FAIL: an uncached slot still asked for a jump");
            errors = errors + 1;
        end

        // ---- MODE: 1 = automatic ----------------------------------
        host_send2(8'h04, 8'd1, 8'd1, 8'd0);
        check_frame(6, 8'h84, 8'd1, 8'd1, 8'h00, 1);
        if (!auto_ovr_en || !auto_ovr_mode) begin
            $display("FAIL: automatic mode override did not latch");
            errors = errors + 1;
        end

        // ---- MODE: 2 hands control back to SW1 --------------------
        host_send2(8'h04, 8'd1, 8'd2, 8'd0);
        check_frame(6, 8'h84, 8'd1, 8'd2, 8'h00, 1);
        if (auto_ovr_en) begin
            $display("FAIL: mode override did not release the switch");
            errors = errors + 1;
        end

        // ---- PARAM: brightness 5 ----------------------------------
        clear_seen;
        host_send2(8'h05, 8'd2, 8'd0, 8'd5);
        check_frame(7, 8'h85, 8'd2, 8'd0, 8'd5, 2);
        if (!seen_bright || (seen_bright_v !== 3'd5)) begin
            $display("FAIL: brightness write did not use value 5");
            errors = errors + 1;
        end

        // ---- PARAM: contrast 1 ------------------------------------
        clear_seen;
        host_send2(8'h05, 8'd2, 8'd1, 8'd1);
        check_frame(7, 8'h85, 8'd2, 8'd1, 8'd1, 2);
        if (!seen_contrast || (seen_contrast_v !== 3'd1)) begin
            $display("FAIL: contrast write did not use value 1");
            errors = errors + 1;
        end

        // ---- OSD: layout 3 ----------------------------------------
        clear_seen;
        host_send2(8'h06, 8'd1, 8'd3, 8'd0);
        check_frame(6, 8'h86, 8'd1, 8'd3, 8'h00, 1);
        if (!seen_osd || (seen_osd_v !== 2'd3)) begin
            $display("FAIL: OSD layout write is wrong");
            errors = errors + 1;
        end

        // ---- BOARD: row 2, 40 bytes, then visibility control -------
        for (i = 0; i < 40; i = i + 1)
            board_buf[i] = (i < 8) ? 8'h20 : (8'h40 + i[5:0]);
        board_buf[0] = 8'h43;                       // C
        board_buf[1] = 8'h41;                       // A
        board_buf[2] = 8'h31;                       // 1
        board_buf[3] = 8'h35;                       // 5
        board_writes = 0;
        host_send_board(8'd2);
        check_frame(6, 8'hA0, 8'd1, 8'd2, 8'h00, 1);
        if (board_writes != 40) begin
            $display("FAIL: BOARD handed over %0d bytes, want 40", board_writes);
            errors = errors + 1;
        end
        if ((board_last_addr !== 8'd119) || (board_last_data !== board_buf[39])) begin
            $display("FAIL: BOARD last write is addr %0d data %02x",
                     board_last_addr, board_last_data);
            errors = errors + 1;
        end
        if (!board_enable) begin
            $display("FAIL: first accepted BOARD row did not enable the panel");
            errors = errors + 1;
        end

        // ---- BOARD: row 5, the added bottom row, must land at 239 --------
        board_writes = 0;
        host_send_board(8'd5);
        check_frame(6, 8'hA0, 8'd1, 8'd5, 8'h00, 1);
        if (board_writes != 40) begin
            $display("FAIL: BOARD row5 handed over %0d bytes, want 40", board_writes);
            errors = errors + 1;
        end
        if ((board_last_addr !== 8'd239) || (board_last_data !== board_buf[39])) begin
            $display("FAIL: BOARD row5 last write is addr %0d data %02x",
                     board_last_addr, board_last_data);
            errors = errors + 1;
        end

        host_send2(8'h21, 8'd1, 8'd0, 8'd0);
        check_frame(6, 8'hA1, 8'd1, 8'd0, 8'h00, 1);
        if (board_enable) begin
            $display("FAIL: BVIS 0 did not hide the panel");
            errors = errors + 1;
        end
        host_send2(8'h21, 8'd1, 8'd1, 8'd0);
        check_frame(6, 8'hA1, 8'd1, 8'd1, 8'h00, 1);
        if (!board_enable) begin
            $display("FAIL: BVIS 1 did not show the panel");
            errors = errors + 1;
        end

        // ---- TIME: load the real-time clock ------------------------
        host_send_time(8'h12, 8'h34, 8'h56);
        wait_reply(8);
        if ((reply[2] !== 8'hA2) || (reply[3] !== 8'd3) ||
            (reply[4] !== 8'h12) || (reply[5] !== 8'h34) ||
            (reply[6] !== 8'h56)) begin
            $display("FAIL: TIME reply is wrong");
            errors = errors + 1;
        end
        reply_n = 0;
        if ((time_hh !== 8'h12) || (time_mm !== 8'h34) ||
            (time_ss !== 8'h56) || !time_tog) begin
            $display("FAIL: TIME payload did not reach the overlay outputs");
            errors = errors + 1;
        end

        // ---- TIME: an out-of-range hour must be refused -------------
        host_send_time(8'h24, 8'h00, 8'h00);
        check_frame(7, 8'hE0, 8'd2, 8'h22, 8'd5, 2);
        if ((time_hh !== 8'h12) || (time_mm !== 8'h34) ||
            (time_ss !== 8'h56)) begin
            $display("FAIL: bad TIME overwrote the accepted clock");
            errors = errors + 1;
        end

        // ---- STATUS ------------------------------------------------
        host_send2(8'h10, 8'd0, 8'd0, 8'd0);
        wait_reply(15);
        if ((reply[2] !== 8'h90) || (reply[3] !== 8'd10)) begin
            $display("FAIL: STATUS reply header is wrong");
            errors = errors + 1;
        end else begin
            if (reply[4] !== 8'h11) begin         // version
                $display("FAIL: STATUS version is %02x", reply[4]);
                errors = errors + 1;
            end
            if (reply[5] !== 8'h01) begin         // slot
                $display("FAIL: STATUS slot is %02x", reply[5]);
                errors = errors + 1;
            end
            if (reply[6] !== 8'd6) begin          // file count
                $display("FAIL: STATUS file count is %02x", reply[6]);
                errors = errors + 1;
            end
            if (reply[8] !== 8'd7) begin          // brightness
                $display("FAIL: STATUS brightness is %02x", reply[8]);
                errors = errors + 1;
            end
            if (reply[11] !== 8'h2A) begin        // cache 101010
                $display("FAIL: STATUS cache is %02x", reply[11]);
                errors = errors + 1;
            end
            if (reply[12] !== 8'd6) begin         // text length
                $display("FAIL: STATUS text length is %02x", reply[12]);
                errors = errors + 1;
            end
        end
        reply_n = 0;

        // ---- a torn TEXT frame must never reach the screen ----------
        got_n      = 0;
        clr_events = 0;
        cmt_events = 0;
        host_byte(8'hA5);
        host_byte(8'h5A);
        host_byte(8'h02);
        host_byte(8'd2);
        host_byte(8'h42);                         // "BA" arrives intact,
        host_byte(8'h41);                         // the checksum does not
        host_byte(8'h00);
        repeat (1200) @(posedge clk);
        if (cmt_events != 0) begin
            $display("FAIL: a bad text checksum committed the message");
            errors = errors + 1;
        end
        if (clr_events != 1) begin
            $display("FAIL: a torn text frame blanked %0d times, want 1",
                     clr_events);
            errors = errors + 1;
        end
        if (ov_len != 6'd0) begin
            $display("FAIL: a torn text frame left %0d characters on screen",
                     ov_len);
            errors = errors + 1;
        end
        if (text_len != 6'd0) begin
            $display("FAIL: a torn text frame left text_len at %0d", text_len);
            errors = errors + 1;
        end
        if (reply_n != 0) begin
            $display("FAIL: a torn text frame produced a reply");
            errors = errors + 1;
        end

        // ---- corrupted frame must change nothing -------------------
        clear_seen;
        host_byte(8'hA5);
        host_byte(8'h5A);
        host_byte(8'h06);
        host_byte(8'd1);
        host_byte(8'd1);
        host_byte(8'h00);                         // wrong checksum
        repeat (1200) @(posedge clk);
        if (reply_n != 0) begin
            $display("FAIL: a bad checksum produced a reply");
            errors = errors + 1;
        end
        if (seen_osd) begin
            $display("FAIL: a bad checksum reached the OSD control");
            errors = errors + 1;
        end

        // ---- empty TEXT restores the built in message -------------
        // the reply is A5 5A 82 02 <eff_len> <text_len> <sum>, seven bytes
        clr_events = 0;
        host_send_text(8'd0);
        check_frame(7, 8'h82, 8'd2, 8'd0, 8'd0, 2);
        if (text_len != 6'd0) begin
            $display("FAIL: text_len did not return to zero");
            errors = errors + 1;
        end
        if ((clr_events != 1) || (ov_len != 6'd0)) begin
            $display("FAIL: empty TEXT did not hand back to the built in text");
            errors = errors + 1;
        end

        repeat (200) @(posedge clk);
        if (errors == 0)
            $display("PASS: tb_uart_ctrl");
        else
            $display("FAIL: tb_uart_ctrl with %0d errors", errors);
        $finish;
    end

endmodule
