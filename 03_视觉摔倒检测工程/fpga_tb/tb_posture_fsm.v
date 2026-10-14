//===========================================================================
// tb_posture_fsm.v -- replay one per-track observation stream through the
// RTL posture machine and print the result the driver compares against the
// Python trace.
//
// Vector row (one observation per row, whitespace separated):
//     frame valid x y w h area confirm_required
//
// `en` is a one-clock strobe exactly like bbox_valid on the board: the
// observation is loaded on the falling edge and the FSM's input stage
// captures it on the next rising edge.  Only that edge carries information:
// the state machine works entirely from the captured copy, so the vector may
// change as soon as `en` drops.  The bench waits an extra disabled clock
// after sampling before loading the next row.
//
// +HOSTILE=1 makes the bench do what the real chain does: the frame counter
// moves on the moment `en` is deasserted, one edge *before* the state machine
// consumes the observation.  posture_fsm must read its captured registers in
// that cycle -- reading the input ports instead shifts pending_frame by one
// frame and delays every alarm by a frame (the bug this option exists to
// catch).
//
// plusargs:
//     +VEC=<path>   vector file produced by fpga/tools/rtl_check.py
//     +HOSTILE=<0|1> see above
//
// Output lines parsed by the driver:
//     ALARM <frame>          on every RTL alarm pulse
//     ROW <frame> <valid> <state>   after every row
//     DONE
//===========================================================================
`timescale 1ns / 1ps
`include "params.vh"

module tb_posture_fsm;

reg         clk = 1'b0;
reg         rst_n = 1'b0;
reg         valid = 1'b0;
reg         en = 1'b0;
reg  [31:0] frame = 32'd0;
reg  [9:0]  x = 10'd0, y = 10'd0, w = 10'd0, h = 10'd0;
reg  [23:0] area = 24'd0;
reg  [2:0]  confirm_required = 3'd2;

wire        alarm;
wire [2:0]  state;

posture_fsm #(
    .FRAME_W         (320),
    .FRAME_H         (`FRAME_H),
    .MIN_REF_AREA    (`MIN_REF_AREA),
    .MIN_REF_HEIGHT  (`MIN_REF_HEIGHT),
    .MAX_REF_W       (`MAX_REF_W),
    .MAX_REF_H       (`MAX_REF_H),
    .MAX_REF_AREA    (`MAX_REF_AREA),
    .SUNK_CY2        (`SUNK_CY2),
    .REF_TTL         (`REF_TTL),
    .COLLAPSE_FRAMES (`COLLAPSE_FRAMES),
    .RECOVERY_WINDOW (`RECOVERY_WINDOW),
    .COOLDOWN_FRAMES (`COOLDOWN_FRAMES)
) dut (
    .clk              (clk),
    .rst_n            (rst_n),
    .en               (en),
    .valid            (valid),
    .frame            (frame),
    .x                (x),
    .y                (y),
    .w                (w),
    .h                (h),
    .area             (area),
    .confirm_required (confirm_required),
    .alarm            (alarm),
    .state            (state),

    // This bench replays one track through the engine directly; posture_bank
    // (the crowd path) is what drives the context ports.
    .ctx_in           ({242{1'b0}}),
    .ctx_load         (1'b0),
    .ctx_out          ()
);

always #5 clk = ~clk;

reg [1023:0] vecpath;
integer fd;
integer rc;
integer hostile;
reg [31:0] frame_disp;

initial begin
    if (!$value$plusargs("VEC=%s", vecpath)) begin
        $display("ERROR no +VEC=<path> plusarg");
        $finish;
    end
    if (!$value$plusargs("HOSTILE=%d", hostile)) hostile = 0;
    fd = $fopen(vecpath, "r");
    if (fd == 0) begin
        $display("ERROR cannot open vector file");
        $finish;
    end

    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    while (!$feof(fd)) begin
        rc = $fscanf(fd, "%d %d %d %d %d %d %d %d\n",
                     frame, valid, x, y, w, h, area, confirm_required);
        if (rc == 8) begin
            // Load on the falling edge, then strobe `en` for exactly one
            // clock.  posture_fsm registers the observation (products and
            // ratio comparisons) at that rising edge; the state machine
            // consumes it at the following rising edge, which is where the
            // result is sampled.  The vector stays loaded through that
            // second falling edge, so both pipeline edges see one coherent
            // observation.
            @(negedge clk);
            en = 1'b1;
            @(posedge clk);
            @(negedge clk);
            en = 1'b0;
            // The capture is done; a real caller may already be counting the
            // next frame.  Only `en`-edge values are part of the contract.
            frame_disp = frame;
            if (hostile) frame = frame + 32'd1;
            @(posedge clk); #1;
            if (alarm) $display("ALARM %0d", frame_disp);
            $display("ROW %0d %0d %0d", frame_disp, valid, state);
            @(negedge clk);
        end else if (rc != -1) begin
            $display("PARSE-ERROR rc=%0d", rc);
        end
    end

    $display("DONE");
    $finish;
end

endmodule
