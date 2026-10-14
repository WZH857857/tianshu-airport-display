`timescale 1ns / 1ps
//=============================================================================
// uart_ctrl.v - serial command handler for the HDMI player
//
// The board brings the CH340 USB-UART bridge out on the same Type-C socket
// that powers the board, so the PC host talks to this block directly.  The
// block owns the byte level protocol and the strobes that drive the existing
// picture, parameter and overlay controls.  It never touches the video or
// audio path, so a serial burst cannot disturb the picture or restart the
// announcement.
//
// frame layout, both directions:
//
//     A5 5A | cmd | len | payload[len] | xor
//
// The checksum is the exclusive or of cmd, len and every payload byte; the two
// header bytes are not part of it.
//
// commands from the host                     reply
//   01 PING        -                         81 ver
//   02 TEXT   ascii, 0..32 bytes             82 len text_len   (len 0 = default)
//   03 SLOT   slot 0..5                      83 slot
//   04 MODE   0 manual, 1 auto, 2 follow SW1 84 mode
//   05 PARAM  sel 0 bright / 1 contrast, val 85 sel val
//   06 OSD    layout 0..3                    86 layout
//   07 BAUD   rate index 0..4                87 index  (sent at the new rate)
//   10 STATUS -                              90 version slot total mode bright
//                                                contrast osd cache textlen flags
//   20 BOARD  row 0..5 + 40 ascii bytes      A0 row
//   21 BVIS   0 board off / 1 board on       A1 enable
//   22 TIME   hh mm ss, BCD 0x00..0x23 and   A2 hh mm ss
//             0x00..0x59 for mm/ss
//   23 EMG    0 normal, 1 fire emergency      A3 emg
//   anything else                            E0 cmd code
//
// error codes: 1 bad length, 2 bad slot, 3 slot not cached, 4 unknown
//              command, 5 bad value
//
// Area notes.  The design already sits at 97% of the slices, so this block
// deliberately owns no big arrays:
//
//   * The host text is NOT staged here.  Each payload byte is handed to the
//     pixel clock overlay as it arrives, together with a commit strobe, and
//     the overlay only latches the new length when the checksum has passed.
//     A torn frame therefore never reaches the screen, and the 32 byte buffer
//     exists exactly once, in ui_overlay.
//   * The reply frame is not staged either.  A combinational byte generator
//     feeds both the checksum pass and the shift out, so there is no 16 byte
//     reply array and no indexed read of one.
//
// A frame whose checksum fails while its text was already streaming blanks
// the ticker instead of leaving half of the new message on screen, so the
// fallback is always the built in airport subtitle rather than garbage.
//=============================================================================
module uart_ctrl #(
    parameter [15:0] DEFAULT_DIV = 16'd434,     // 115200 baud at 50 MHz
    parameter [7:0]  VERSION     = 8'h11
)(
    input  wire        clk,             // 50 MHz system clock
    input  wire        rst_n,

    input  wire        rx_pin,          // CH340 TXD -> FPGA RXD
    output wire        tx_pin,          // FPGA TXD -> CH340 RXD

    // host text, handed to the pixel clock overlay
    output reg  [4:0]  text_addr,
    output reg  [7:0]  text_data,
    output reg         text_tog,        // one toggle per byte
    output reg         text_clear_tog,  // drop back to the built in subtitle
    output reg         text_commit_tog, // one toggle per accepted message,
                                        // latches the staged length
    output reg  [5:0]  text_len,

    // flight information board.  One byte per handshake, address is
    // row * 40 + column, and the host also controls the display enable.
    output reg         board_tog,
    output reg  [7:0]  board_addr,
    output reg  [7:0]  board_data,
    output reg         board_enable,

    // picture jump
    output reg         jump_req,
    output reg  [2:0]  jump_slot,

    // mode and display parameters
    output reg         auto_ovr_en,
    output reg         auto_ovr_mode,
    output reg         bright_we,
    output reg  [2:0]  bright_val,
    output reg         contrast_we,
    output reg  [2:0]  contrast_val,
    output reg         osd_we,
    output reg  [1:0]  osd_val,

    // run time baud rate
    output reg  [15:0] baud_div,

    // the overlay draws a link marker once the host has spoken
    output reg         link_seen,

    // wall clock pushed by the host.  hh, mm and ss are BCD so the overlay
    // can load its decimal counters without a binary to BCD conversion.
    output reg  [7:0]  time_hh,
    output reg  [7:0]  time_mm,
    output reg  [7:0]  time_ss,
    output reg         time_tog,        // one toggle per accepted TIME message

    // fire emergency state, latched here and consumed by the overlay and the
    // board level alarm annunciator (buzzer + seven segment digits).
    output reg         emg_fire,

    // live status, all in this clock domain
    input  wire [2:0]  st_slot,
    input  wire [7:0]  st_total,
    input  wire [5:0]  st_cache,
    input  wire [2:0]  st_bright,
    input  wire [2:0]  st_contrast,
    input  wire [1:0]  st_osd,
    input  wire        st_auto,
    input  wire        st_card_ok
);

    //------------------------------------------------------------------
    // byte level interfaces
    //------------------------------------------------------------------
    wire [7:0] rx_data;
    wire       rx_valid;

    reg  [7:0] tx_byte;
    reg        tx_start;
    wire       tx_busy;
    wire       tx_done;

    uart_rx u_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .div        (baud_div),
        .rx         (rx_pin),
        .data       (rx_data),
        .data_valid (rx_valid)
    );

    uart_tx u_tx (
        .clk   (clk),
        .rst_n (rst_n),
        .div   (baud_div),
        .send  (tx_start),
        .data  (tx_byte),
        .tx    (tx_pin),
        .busy  (tx_busy),
        .done  (tx_done)
    );

    //------------------------------------------------------------------
    // states
    //------------------------------------------------------------------
    localparam [2:0] P_HDR1 = 3'd0,
                     P_HDR2 = 3'd1,
                     P_CMD  = 3'd2,
                     P_LEN  = 3'd3,
                     P_PAY  = 3'd4,
                     P_SUM  = 3'd5;

    localparam [1:0] S_HUNT = 2'd0,
                     S_EXEC = 2'd1,
                     S_RCHK = 2'd2,
                     S_TX   = 2'd3;

    reg [2:0]  pstate;
    reg [7:0]  cmd_r;
    reg [7:0]  len_r;
    reg [7:0]  pay_idx;
    reg [7:0]  sum_r;
    reg [7:0]  pay_small [0:1];     // the first two payload bytes
    reg [7:0]  board_base;          // row * 40, latched from payload byte 0
    reg [7:0]  t_hh_r, t_mm_r, t_ss_r;  // staged TIME payload

    reg [1:0]  sstate;
    reg [7:0]  rep_chk;
    reg [3:0]  bidx;                // byte index, shared by both reply phases
    reg        tx_issued;

    wire rx_accept = (sstate == S_HUNT) && !tx_busy;

    reg [7:0]  q0;
    reg [7:0]  q1;
    always @(*) begin
        q0 = pay_small[0];
        q1 = pay_small[1];
    end

    // validity of the last payload, shared by the effect logic and the reply
    wire slot_bad  = (q0 >= 8'd6);
    wire slot_miss = !slot_bad && !st_cache[q0[2:0]];
    wire mode_bad  = (q0 > 8'd2);
    wire parm_bad  = (q0 > 8'd1) || (q1 > 8'd7);
    wire osd_bad   = (q0 > 8'd3);
    wire time_bad  = (t_hh_r > 8'h23) || (t_mm_r > 8'h59) ||
                     (t_ss_r > 8'h59);

    //------------------------------------------------------------------
    // reply description, combinational
    //------------------------------------------------------------------
    reg [7:0] rcmd;
    reg [7:0] rlen;
    reg [3:0] rcode;

    always @(*) begin
        rcmd  = 8'hE0;
        rlen  = 8'd2;
        rcode = 4'd4;                       // unknown command
        case (cmd_r)
            8'h01: begin rcmd = 8'h81; rlen = 8'd1;  end
            8'h02: begin rcmd = 8'h82; rlen = 8'd2;  end
            8'h03: begin
                if (slot_bad)       begin rlen = 8'd2; rcode = 4'd2; end
                else if (slot_miss) begin rlen = 8'd2; rcode = 4'd3; end
                else                begin rcmd = 8'h83; rlen = 8'd1; end
            end
            8'h04: begin
                if (mode_bad) begin rlen = 8'd2; rcode = 4'd5; end
                else          begin rcmd = 8'h84; rlen = 8'd1; end
            end
            8'h05: begin
                if (parm_bad) begin rlen = 8'd2; rcode = 4'd5; end
                else          begin rcmd = 8'h85; rlen = 8'd2; end
            end
            8'h06: begin
                if (osd_bad) begin rlen = 8'd2; rcode = 4'd5; end
                else         begin rcmd = 8'h86; rlen = 8'd1; end
            end
            8'h07: begin rcmd = 8'h87; rlen = 8'd1;  end
            8'h10: begin rcmd = 8'h90; rlen = 8'd10; end
            8'h20: begin rcmd = 8'hA0; rlen = 8'd1;  end
            8'h21: begin rcmd = 8'hA1; rlen = 8'd1;  end
            8'h22: begin
                if (time_bad) begin rlen = 8'd2; rcode = 4'd5; end
                else          begin rcmd = 8'hA2; rlen = 8'd3; end
            end
            8'h23: begin rcmd = 8'hA3; rlen = 8'd1; end
            default: begin end
        endcase
    end

    // Payload byte <pi> of the pending reply.  A function rather than a
    // registered array keeps the block free of the sixteen to one read mux
    // that a staged reply frame would need.
    function [7:0] pay_byte;
        input [3:0] pi;
        begin
            pay_byte = 8'h00;
            if (rcmd == 8'hE0) begin
                case (pi)
                    4'd0: pay_byte = cmd_r;
                    4'd1: pay_byte = {4'd0, rcode};
                    default: pay_byte = 8'h00;
                endcase
            end else begin
                case (cmd_r)
                    8'h01: pay_byte = VERSION;
                    8'h02: pay_byte = (pi == 4'd0) ? len_r : text_len;
                    8'h03: pay_byte = q0;
                    8'h04: pay_byte = q0;
                    8'h05: pay_byte = (pi == 4'd0) ? q0 : q1;
                    8'h06: pay_byte = q0;
                    8'h07: pay_byte = q0;
                    8'h20: pay_byte = q0;
                    8'h21: pay_byte = {7'd0, board_enable};
                    8'h22: case (pi)
                        4'd0: pay_byte = t_hh_r;
                        4'd1: pay_byte = t_mm_r;
                        default: pay_byte = t_ss_r;
                    endcase
                    8'h23: pay_byte = {7'd0, emg_fire};
                    8'h10: case (pi)
                        4'd0: pay_byte = VERSION;
                        4'd1: pay_byte = {5'd0, st_slot};
                        4'd2: pay_byte = st_total;
                        4'd3: pay_byte = {6'd0, auto_ovr_en,
                                          auto_ovr_en ? auto_ovr_mode : st_auto};
                        4'd4: pay_byte = {5'd0, st_bright};
                        4'd5: pay_byte = {5'd0, st_contrast};
                        4'd6: pay_byte = {6'd0, st_osd};
                        4'd7: pay_byte = {2'd0, st_cache};
                        4'd8: pay_byte = text_len;
                        default: pay_byte = {5'd0, st_card_ok, link_seen};
                    endcase
                    default: pay_byte = 8'h00;
                endcase
            end
        end
    endfunction

    // One byte of the reply frame at index <idx>.  The checksum pass and the
    // shift out walk the same generator; rep_nxt lets the transmitter fetch
    // the byte after the one it just handed to uart_tx.
    function [7:0] rep_byte;
        input [3:0] idx;
        reg   [3:0] pi;
        begin
            pi = idx - 4'd4;
            case (idx)
                4'd0: rep_byte = 8'hA5;
                4'd1: rep_byte = 8'h5A;
                4'd2: rep_byte = rcmd;
                4'd3: rep_byte = rlen;
                default: rep_byte = ({1'b0, pi} < {1'b0, rlen}) ?
                                    pay_byte(pi) : rep_chk;
            endcase
        end
    endfunction

    wire [7:0] rep_b   = rep_byte(bidx);
    wire [7:0] rep_nxt = rep_byte(bidx + 4'd1);

    // cmd sits at bidx 2, len at 3 and the payload at 4..rlen+3, so the
    // checksum pass runs 2..rlen+3 and the frame is rlen+5 bytes long
    wire [4:0] chk_last = {1'b0, rlen} + 5'd3;
    wire [4:0] tx_last  = {1'b0, rlen} + 5'd5;

    //------------------------------------------------------------------
    // main state machine
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pstate         <= P_HDR1;
            cmd_r          <= 8'd0;
            len_r          <= 8'd0;
            pay_idx        <= 8'd0;
            sum_r          <= 8'd0;
            sstate         <= S_HUNT;
            rep_chk        <= 8'd0;
            bidx           <= 4'd0;
            tx_issued      <= 1'b0;
            tx_byte        <= 8'd0;
            tx_start       <= 1'b0;
            text_addr      <= 5'd0;
            text_data      <= 8'd0;
            text_tog       <= 1'b0;
            text_clear_tog <= 1'b0;
            text_commit_tog<= 1'b0;
            text_len       <= 6'd0;
            board_tog      <= 1'b0;
            board_addr     <= 8'd0;
            board_data     <= 8'd0;
            board_base     <= 8'd0;
            board_enable   <= 1'b0;
            jump_req       <= 1'b0;
            jump_slot      <= 3'd0;
            auto_ovr_en    <= 1'b0;
            auto_ovr_mode  <= 1'b0;
            bright_we      <= 1'b0;
            bright_val     <= 3'd7;
            contrast_we    <= 1'b0;
            contrast_val   <= 3'd3;
            osd_we         <= 1'b0;
            osd_val        <= 2'd0;
            baud_div       <= DEFAULT_DIV;
            link_seen      <= 1'b0;
            time_hh        <= 8'h00;
            time_mm        <= 8'h00;
            time_ss        <= 8'h00;
            time_tog       <= 1'b0;
            emg_fire       <= 1'b0;
            t_hh_r         <= 8'h00;
            t_mm_r         <= 8'h00;
            t_ss_r         <= 8'h00;
        end else begin
            // one clock strobes
            tx_start       <= 1'b0;
            bright_we      <= 1'b0;
            contrast_we    <= 1'b0;
            osd_we         <= 1'b0;
            jump_req       <= 1'b0;

            //----------------------------------------------------------
            // receive path.  Once a reply is being prepared or shifted
            // out the parser is parked, so a byte that arrives in that
            // window can never corrupt the frame state.
            //----------------------------------------------------------
            if (rx_accept) begin
                if (rx_valid) begin
                    case (pstate)
                        P_HDR1: begin
                            if (rx_data == 8'hA5)
                                pstate <= P_HDR2;
                        end
                        P_HDR2: begin
                            pstate <= (rx_data == 8'h5A) ? P_CMD : P_HDR1;
                        end
                        P_CMD: begin
                            cmd_r  <= rx_data;
                            sum_r  <= rx_data;
                            pstate <= P_LEN;
                        end
                        P_LEN: begin
                            len_r   <= rx_data;
                            sum_r   <= sum_r ^ rx_data;
                            pay_idx <= 8'd0;
                            if (rx_data == 8'd0)
                                pstate <= P_SUM;
                            else if ((cmd_r == 8'h22) && (rx_data != 8'd3))
                                pstate <= P_HDR1;   // TIME carries exactly 3
                            else if (rx_data > ((cmd_r == 8'h20) ? 8'd41 : 8'd32))
                                pstate <= P_HDR1;   // cannot be a real frame
                            else
                                pstate <= P_PAY;
                        end
                        P_PAY: begin
                            sum_r <= sum_r ^ rx_data;
                            if (pay_idx < 8'd2)
                                pay_small[pay_idx[0]] <= rx_data;
                            if (cmd_r == 8'h02) begin
                                // straight through to the pixel domain; the
                                // overlay keeps it off screen until commit
                                text_addr <= pay_idx[4:0];
                                text_data <= rx_data;
                                text_tog  <= ~text_tog;
                            end else if (cmd_r == 8'h20) begin
                                // byte 0 is the row, bytes 1..40 are the text
                                if (pay_idx == 8'd0) begin
                                    board_base <= {rx_data[2:0], 5'b00000} +
                                                  {2'b00, rx_data[2:0], 3'b000};
                                end else begin
                                    board_addr <= board_base +
                                                  (pay_idx - 8'd1);
                                    board_data <= rx_data;
                                    board_tog  <= ~board_tog;
                                end
                            end else if (cmd_r == 8'h22) begin
                                if (pay_idx == 8'd0)
                                    t_hh_r <= rx_data;
                                else if (pay_idx == 8'd1)
                                    t_mm_r <= rx_data;
                                else
                                    t_ss_r <= rx_data;
                            end
                            pay_idx <= pay_idx + 8'd1;
                            if (pay_idx == (len_r - 8'd1))
                                pstate <= P_SUM;
                        end
                        default: begin       // P_SUM
                            pstate <= P_HDR1;
                            if (rx_data == sum_r) begin
                                link_seen <= 1'b1;
                                if (cmd_r == 8'h02) begin
                                    if (len_r == 8'd0) begin
                                        // empty text restores the built in
                                        // airport message
                                        text_clear_tog <= ~text_clear_tog;
                                        text_len       <= 6'd0;
                                    end else begin
                                        text_len <= len_r[5:0];
                                        text_commit_tog <= ~text_commit_tog;
                                    end
                                end
                                sstate <= S_EXEC;
                            end else if ((cmd_r == 8'h02) && (len_r != 8'd0)) begin
                                // a torn text frame must not leave half of the
                                // new message on screen
                                text_clear_tog <= ~text_clear_tog;
                                text_len       <= 6'd0;
                            end
                        end
                    endcase
                end
            end

            //----------------------------------------------------------
            // side effects, one shot per accepted frame
            //----------------------------------------------------------
            if (sstate == S_EXEC) begin
                if (cmd_r == 8'h03 && !slot_bad && !slot_miss) begin
                    jump_slot <= q0[2:0];
                    jump_req  <= 1'b1;
                end else if (cmd_r == 8'h04 && !mode_bad) begin
                    if (q0 == 8'd2)
                        auto_ovr_en <= 1'b0;     // back to the switch
                    else begin
                        auto_ovr_en   <= 1'b1;
                        auto_ovr_mode <= q0[0];
                    end
                end else if (cmd_r == 8'h05 && !parm_bad) begin
                    if (q0 == 8'd1) begin
                        contrast_we  <= 1'b1;
                        contrast_val <= q1[2:0];
                    end else begin
                        bright_we  <= 1'b1;
                        bright_val <= q1[2:0];
                    end
                end else if (cmd_r == 8'h06 && !osd_bad) begin
                    osd_we  <= 1'b1;
                    osd_val <= q0[1:0];
                end else if (cmd_r == 8'h07) begin
                    case (q0)
                        8'd0: baud_div <= 16'd434;      // 115200
                        8'd1: baud_div <= 16'd217;      // 230400
                        8'd2: baud_div <= 16'd108;      // 460800
                        8'd3: baud_div <= 16'd54;       // 921600
                        8'd4: baud_div <= 16'd50;       // 1000000
                        default: baud_div <= DEFAULT_DIV;
                    endcase
                end else if (cmd_r == 8'h20) begin
                    // the first accepted line turns the board on
                    board_enable <= 1'b1;
                end else if (cmd_r == 8'h21) begin
                    board_enable <= q0[0];
                end else if (cmd_r == 8'h22 && !time_bad) begin
                    time_hh  <= t_hh_r;
                    time_mm  <= t_mm_r;
                    time_ss  <= t_ss_r;
                    time_tog <= ~time_tog;
                end else if (cmd_r == 8'h23) begin
                    // The E104 board latches the flame alarm; this level
                    // follows it until the console sends the clear command
                    // and the detector reports a quiet input again.
                    emg_fire <= q0[0];
                end
                sstate <= S_RCHK;
                bidx   <= 4'd0;
                rep_chk<= 8'd0;
            end

            //----------------------------------------------------------
            // reply path
            //----------------------------------------------------------
            case (sstate)
                S_HUNT: begin
                    bidx      <= 4'd0;
                    tx_issued <= 1'b0;
                end

                S_RCHK: begin
                    if (bidx == 4'd0) begin
                        bidx <= 4'd2;           // first checksummed byte
                    end else if ({1'b0, bidx} <= chk_last) begin
                        rep_chk <= (bidx == 4'd2) ? rep_b : (rep_chk ^ rep_b);
                        bidx    <= bidx + 4'd1;
                    end else begin
                        bidx    <= 4'd0;
                        tx_issued <= 1'b0;
                        sstate  <= S_TX;
                    end
                end

                S_EXEC: ;               // the effect block above owns this state

                S_TX: begin
                    if (!tx_issued) begin
                        tx_byte   <= rep_b;     // bidx points at the byte to send
                        tx_start  <= 1'b1;
                        tx_issued <= 1'b1;
                    end else if (tx_done) begin
                        if ({1'b0, bidx} + 5'd1 < tx_last) begin
                            bidx      <= bidx + 4'd1;
                            tx_byte   <= rep_nxt;
                            tx_start  <= 1'b1;
                            tx_issued <= 1'b1;
                        end else begin
                            tx_issued <= 1'b0;
                            sstate    <= S_HUNT;
                        end
                    end
                end

                default: sstate <= S_HUNT;
            endcase
        end
    end

endmodule
