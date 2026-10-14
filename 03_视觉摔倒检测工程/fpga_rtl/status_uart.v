//===========================================================================
// status_uart.v -- 115200 8N1 ASCII status line for board bring-up
//
// Once per bounding-box observation the FPGA prints one line to the USB-UART
// bridge (fpga_txd = P3).  Everything is hex, so the formatter is a fixed
// sequence of nibbles with no division:
//
//   F<frame:4> <found:1> <x:3> <y:3> <w:3> <h:3> <area:5> <state:1>\r\n
//
// e.g.  "F00A7 1 0C8 05A 028 014 01A4C 2"  -> frame 167, lone blob at
// (200,90) sized 40x20 (area 6732), posture machine in state 2 (FALL).
//
// The crowd build prints one line per slot, and those four lines need to be
// told apart, so SHOW_SLOT=1 prepends "<slot> " and adds one character:
//
//   S<slot:1> F<frame:4> ... <state:1>\r\n
//
// SHOW_SLOT=0 reproduces the solo line byte for byte, which is what the
// committed board bitstream and tb_board_top expect.
//
// SHOW_AE=1 appends the exposure loop's own view of the picture, because the
// only complaint that cannot be answered from the bbox fields is "why is the
// screen so dark":
//
//   ... <state:1> <mean:2><shutter:3><gain:2><zone:1>\r\n
//
// mean is the 0..255 frame mean the loop measured, shutter its integration
// rows (0x09), gain the raw 0x35 register code (0x08 = 1.0x, 0x20 = 4.0x,
// 0x51 = 4.25x, 0x60 = 8.0x, 0x61..0x67 = 9x..15x) and zone the dead
// band it placed the frame in (0 dark, 1 in band, 2 bright).  Nine extra
// bytes take the line to 42, i.e. 3.6 ms, so four slots still fit inside one
// 30 ms frame with room to spare.
//
// The bit clock is CLK_HZ/BAUD with integer division (24 MHz/115200 = 208,
// -0.16% error).  A line is 33 bytes = 2.9 ms, so even at 30 fps the link
// stays under 10% utilised.
//
// The formatter prints whatever slot_scan hands it, so the crowd build can
// point the same byte format at the motion_hist publish instead of the
// tracker box (slot_scan's USE_PBOX).  Nothing here changes.
//===========================================================================
`timescale 1ns / 1ps

module status_uart #(
    parameter integer CLK_HZ    = 24000000,
    parameter integer BAUD      = 115200,
    parameter integer SHOW_SLOT = 0,
    parameter integer SHOW_AE   = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        trig,          // one clk pulse

    input  wire [1:0]  slot,          // prefix, only when SHOW_SLOT=1
    input  wire [15:0] frame_cnt,
    input  wire        found,
    input  wire [8:0]  bx,
    input  wire [8:0]  by,
    input  wire [8:0]  bw,
    input  wire [8:0]  bh,
    input  wire [16:0] area,
    input  wire [2:0]  state,

    input  wire [7:0]  ae_mean,
    input  wire [15:0] ae_shutter,
    input  wire [7:0]  ae_gain,
    input  wire [1:0]  ae_zone,

    output reg         tx,
    output wire        ready          // no message in flight, next trig lands
);

    localparam integer DIV = CLK_HZ / BAUD;
    localparam [5:0]   PREF6 = SHOW_SLOT ? 6'd3 : 6'd0;
    // Body lengths measured from step_idx 0 to the final LF nibble.
    localparam integer BODY = 33 + (SHOW_AE ? 9 : 0);
    localparam [5:0]   LAST_STEP = BODY - 1 + PREF6;

    // ---------------------------------------------------- message payload
    reg [15:0] r_frame;
    reg        r_found;
    reg [8:0]  r_x, r_y, r_w, r_h;
    reg [16:0] r_area;
    reg [2:0]  r_state;
    reg [1:0]  r_slot;
    reg [7:0]  r_ae_mean;
    reg [15:0] r_ae_shut;
    reg [7:0]  r_ae_gain;
    reg [1:0]  r_ae_zone;

    function [7:0] hx;
        input [3:0] v;
        begin
            hx = (v < 4'd10) ? (8'h30 + {4'b0, v}) : (8'h37 + {4'b0, v});
        end
    endfunction

    reg [5:0] step;
    reg       is_lit;
    reg [7:0] lit;
    reg [3:0] nib;

    wire [5:0] step_idx = step - PREF6;
    wire       prefix_step = SHOW_SLOT && (step < 6'd3);

    always @(*) begin
        is_lit = 1'b0;
        lit    = 8'h20;
        nib    = 4'h0;
        if (prefix_step) begin
            if (step == 6'd0) begin is_lit = 1'b1; lit = 8'h53; end  // 'S'
            else if (step == 6'd1)   nib = {2'b00, r_slot};
            else begin is_lit = 1'b1; lit = 8'h20; end               // ' '
        end else begin
        case (step_idx)
        6'd0 : begin is_lit = 1'b1; lit = 8'h46; end          // 'F'
        6'd1 : nib = r_frame[15:12];
        6'd2 : nib = r_frame[11:8];
        6'd3 : nib = r_frame[7:4];
        6'd4 : nib = r_frame[3:0];
        6'd5 : begin is_lit = 1'b1; lit = 8'h20; end
        6'd6 : nib = {3'b000, r_found};
        6'd7 : begin is_lit = 1'b1; lit = 8'h20; end
        6'd8 : nib = {3'b000, r_x[8]};
        6'd9 : nib = r_x[7:4];
        6'd10: nib = r_x[3:0];
        6'd11: begin is_lit = 1'b1; lit = 8'h20; end
        6'd12: nib = {3'b000, r_y[8]};
        6'd13: nib = r_y[7:4];
        6'd14: nib = r_y[3:0];
        6'd15: begin is_lit = 1'b1; lit = 8'h20; end
        6'd16: nib = {3'b000, r_w[8]};
        6'd17: nib = r_w[7:4];
        6'd18: nib = r_w[3:0];
        6'd19: begin is_lit = 1'b1; lit = 8'h20; end
        6'd20: nib = {3'b000, r_h[8]};
        6'd21: nib = r_h[7:4];
        6'd22: nib = r_h[3:0];
        6'd23: begin is_lit = 1'b1; lit = 8'h20; end
        6'd24: nib = {3'b000, r_area[16]};
        6'd25: nib = r_area[15:12];
        6'd26: nib = r_area[11:8];
        6'd27: nib = r_area[7:4];
        6'd28: nib = r_area[3:0];
        6'd29: begin is_lit = 1'b1; lit = 8'h20; end
        6'd30: nib = {1'b0, r_state};
        // The exposure block goes in front of CR/LF, so closing the message
        // stays the last thing the formatter does.  With SHOW_AE=0 the line
        // still ends at step 32 and the bytes are the original 0..32.
        6'd31: if (SHOW_AE) begin is_lit = 1'b1; lit = 8'h20; end
               else         begin is_lit = 1'b1; lit = 8'h0D; end   // ' '/CR
        6'd32: if (SHOW_AE) nib = r_ae_mean[7:4];
               else         begin is_lit = 1'b1; lit = 8'h0A; end   // LF
        6'd33: nib = r_ae_mean[3:0];
        6'd34: nib = r_ae_shut[11:8];
        6'd35: nib = r_ae_shut[7:4];
        6'd36: nib = r_ae_shut[3:0];
        6'd37: nib = r_ae_gain[7:4];
        6'd38: nib = r_ae_gain[3:0];
        6'd39: nib = {2'b00, r_ae_zone};
        6'd40: begin is_lit = 1'b1; lit = 8'h0D; end          // CR
        6'd41: begin is_lit = 1'b1; lit = 8'h0A; end          // LF
        default: begin is_lit = 1'b1; lit = 8'h20; end
        endcase
        end
    end

    wire [7:0] byte_out = is_lit ? lit : hx(nib);

    // ------------------------------------------------------ UART shifter
    localparam [1:0] U_IDLE = 2'd0, U_SEND = 2'd1;

    reg [1:0]  ustate;
    reg [9:0]  sh;
    reg [3:0]  ubit;
    reg [15:0] ucnt;
    reg        msg_active;
    reg        pending;

    wire load = msg_active && (ustate == U_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ustate     <= U_IDLE;
            sh         <= 10'h3FF;
            ubit       <= 4'd0;
            ucnt       <= 16'd0;
            tx         <= 1'b1;
            step       <= 6'd0;
            msg_active <= 1'b0;
            pending    <= 1'b0;
            r_frame    <= 16'd0;
            r_found    <= 1'b0;
            r_x        <= 9'd0;
            r_y        <= 9'd0;
            r_w        <= 9'd0;
            r_h        <= 9'd0;
            r_area     <= 17'd0;
            r_state    <= 3'd0;
            r_slot     <= 2'd0;
            r_ae_mean  <= 8'd0;
            r_ae_shut  <= 16'd0;
            r_ae_gain  <= 8'd0;
            r_ae_zone  <= 2'd0;
        end else begin
            // ------------------------------------------------ trigger
            if (trig) begin
                r_slot  <= slot;
                r_frame <= frame_cnt;
                r_found <= found;
                r_x     <= bx;
                r_y     <= by;
                r_w     <= bw;
                r_h     <= bh;
                r_area  <= area;
                r_state <= state;
                r_ae_mean <= ae_mean;
                r_ae_shut <= ae_shutter;
                r_ae_gain <= ae_gain;
                r_ae_zone <= ae_zone;
                pending <= 1'b1;
            end

            if (!msg_active) begin
                if (pending) begin
                    pending    <= 1'b0;
                    msg_active <= 1'b1;
                    step       <= 6'd0;
                end
            end else if (load) begin
                if (step == LAST_STEP) msg_active <= 1'b0;
                else                   step <= step + 6'd1;
            end

            // ------------------------------------------------ shifter
            case (ustate)
            U_IDLE: begin
                tx   <= 1'b1;
                ucnt <= 16'd0;
                if (load) begin
                    sh     <= {1'b1, byte_out, 1'b0};   // stop, data, start
                    ubit   <= 4'd0;
                    ustate <= U_SEND;
                end
            end
            U_SEND: begin
                tx <= sh[ubit];
                if (ucnt == DIV - 1) begin
                    ucnt <= 16'd0;
                    if (ubit == 4'd9) ustate <= U_IDLE;
                    else              ubit   <= ubit + 4'd1;
                end else begin
                    ucnt <= ucnt + 16'd1;
                end
            end
            default: ustate <= U_IDLE;
            endcase
        end
    end

    assign ready = !msg_active && !pending;

endmodule
