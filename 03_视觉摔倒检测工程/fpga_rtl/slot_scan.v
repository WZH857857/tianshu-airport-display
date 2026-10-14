//===========================================================================
// slot_scan.v -- walk the crowd detector's slots into one status formatter
//
// The crowd top publishes all NSLOT boxes on the same frame pulse, but the
// 115200 line format reports one box at a time.  This module takes the
// snapshot registers and drives a single status_uart with one trigger per
// slot, in slot order, so the wire format stays exactly the solo format plus
// the slot prefix.
//
// Flow per slot:  trig one cycle -> wait for the formatter to pick it up
// (ready low) -> wait for the line to finish (ready high) -> next slot.
// A burst is 4 x 2.9 ms = 11.6 ms against a 33 ms frame, so it always fits;
// a snap that arrives while a burst is still running is dropped rather than
// queued (the FPGA has no room for a report queue, and the next frame's
// snapshot is 33 ms away).
//
// The timeout is a stuck-formatter guard only: if ready never comes back the
// scan aborts instead of wedging every later frame.
//
// USE_PBOX=1 points the same byte format at motion_hist's whole-person
// publish instead of the tracker box, so a board session can answer "is the
// figure the overlay draws actually being produced" with the serial port
// alone.  The fields keep their slots but change meaning:
//
//   found  <- pbox_found      x/y/w/h <- pbox_x/y/w/h
//   area   <- pbox_cells      (the sweep's trail cell count, 13 bits)
//   state  <- {published, pass_hit, sweep_hit}
//
// The three state bits are sticky "has this ever happened" flags from
// motion_hist: a publish reached the output, the end-of-frame pass saw at
// least one motion cell inside a window, and the sweep read at least one
// live trail cell.  They separate "the memory never fills" from "the memory
// fills but the sweep cannot read it" without a second build.
//===========================================================================
`timescale 1ns / 1ps

module slot_scan #(
    parameter integer NSLOT   = 4,
    parameter integer TIMEOUT = 200000,     // 24 MHz -> ~8 ms
    parameter integer USE_PBOX = 0          // report the hist publish instead
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        snap,        // one clk pulse, starts a burst
    input  wire        ready,       // formatter idle, next trig will land

    input  wire [15:0] frame,
    input  wire [NSLOT-1:0]    found,
    input  wire [NSLOT*3-1:0]  state,
    input  wire [NSLOT*9-1:0]  x,
    input  wire [NSLOT*9-1:0]  y,
    input  wire [NSLOT*9-1:0]  w,
    input  wire [NSLOT*9-1:0]  h,
    input  wire [NSLOT*17-1:0] area,

    input  wire [NSLOT-1:0]    pb_found,
    input  wire [NSLOT*9-1:0]  pb_x,
    input  wire [NSLOT*9-1:0]  pb_y,
    input  wire [NSLOT*9-1:0]  pb_w,
    input  wire [NSLOT*9-1:0]  pb_h,
    input  wire [NSLOT*13-1:0] pb_cells,
    input  wire [2:0]          pb_flags,    // {published, pass_hit, sweep_hit}

    output reg         trig,
    output wire [1:0]  slot,
    output wire [15:0] o_frame,
    output wire        o_found,
    output wire [8:0]  o_x,
    output wire [8:0]  o_y,
    output wire [8:0]  o_w,
    output wire [8:0]  o_h,
    output wire [16:0] o_area,
    output wire [2:0]  o_state,
    output wire        busy
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_TRIG  = 2'd1,
                     S_BUSY  = 2'd2,
                     S_DRAIN = 2'd3;

    reg [1:0]  state_r;
    reg [1:0]  sel;
    reg [17:0] tmo;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r <= S_IDLE;
            sel     <= 2'd0;
            trig    <= 1'b0;
            tmo     <= 18'd0;
        end else begin
            trig <= 1'b0;
            case (state_r)
            S_IDLE: begin
                tmo <= 18'd0;
                if (snap) begin
                    sel     <= 2'd0;
                    state_r <= S_TRIG;
                end
            end
            S_TRIG: begin
                // One cycle wide, so the formatter latches this slot once.
                trig    <= 1'b1;
                tmo     <= 18'd0;
                state_r <= S_BUSY;
            end
            S_BUSY: begin
                if (tmo == TIMEOUT[17:0]) begin
                    state_r <= S_IDLE;
                end else if (!ready) begin
                    // The formatter accepted the trigger; wait it out.
                    tmo     <= 18'd0;
                    state_r <= S_DRAIN;
                end else begin
                    tmo <= tmo + 18'd1;
                end
            end
            S_DRAIN: begin
                if (ready) begin
                    tmo <= 18'd0;
                    if (sel == NSLOT - 1) begin
                        state_r <= S_IDLE;
                    end else begin
                        sel     <= sel + 2'd1;
                        state_r <= S_TRIG;
                    end
                end else if (tmo == TIMEOUT[17:0]) begin
                    state_r <= S_IDLE;
                end else begin
                    tmo <= tmo + 18'd1;
                end
            end
            default: state_r <= S_IDLE;
            endcase
        end
    end

    // 4:1 muxes off the walking index; the formatter samples them on the
    // cycle trig is high, one full cycle after sel settles.
    assign slot    = sel;
    assign o_frame = frame;
    assign o_found = USE_PBOX ? pb_found[sel] : found[sel];
    assign o_state = USE_PBOX ? pb_flags : state[sel*3 +: 3];
    assign o_x     = USE_PBOX ? pb_x[sel*9 +: 9] : x[sel*9 +: 9];
    assign o_y     = USE_PBOX ? pb_y[sel*9 +: 9] : y[sel*9 +: 9];
    assign o_w     = USE_PBOX ? pb_w[sel*9 +: 9] : w[sel*9 +: 9];
    assign o_h     = USE_PBOX ? pb_h[sel*9 +: 9] : h[sel*9 +: 9];
    assign o_area  = USE_PBOX ? {4'b0000, pb_cells[sel*13 +: 13]}
                              : area[sel*17 +: 17];
    assign busy    = (state_r != S_IDLE);

endmodule
