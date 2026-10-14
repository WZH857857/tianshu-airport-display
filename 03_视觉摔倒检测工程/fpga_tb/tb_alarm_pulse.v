//===========================================================================
// tb_alarm_pulse.v -- alarm must be exactly one clock wide in both wiring
// styles the design uses.
//
// The standalone core (fall_core_top) ties `en` high and feeds one row per
// clock, so a level that lingers is invisible there.  The board top instead
// drives `en` from the proj_bbox result pulse: one enabled clock per frame,
// with the enable low for the rest of the frame.  When alarm was cleared
// only inside the `en` branch, that configuration held the alarm level from
// the firing frame until the next frame's enable, which is what flooded
// tb_board_top +LONG with ALARM lines.
//
// This bench replays the same observation stream through the machine twice:
// once with a per-frame strobe, once with `en` tied high, and measures the
// longest run of consecutive clocks with alarm asserted.
//
// Scenario (board constants, working resolution 320x180):
//   frames 0..2  standing person   x=140 y= 30 w=40 h=120 area=4000
//   frames 3..4  collapsed blob    x=110 y=110 w=90 h= 24 area=1900
//   frames 5..7  no motion blob    (valid=0)
//
//   frame 0 latches the standing reference, frames 3/4 build two
//   consecutive flat observations so detect_fire latches the pending
//   candidate at frame 4, and the silent confirm window elapses at frame 6.
//
// PASS requires: one alarm in each phase, longest run == 1 clock in each.
//===========================================================================
`timescale 1ns / 1ps

module tb_alarm_pulse;

// Board parameters from td/fall_core_top.v
localparam integer FRAME_W         = 320;
localparam integer FRAME_H         = 180;
localparam integer MIN_REF_AREA    = 1600;
localparam integer MIN_REF_HEIGHT  = 25;
localparam integer MAX_REF_W       = 176;
localparam integer MAX_REF_H       = 171;
localparam integer MAX_REF_AREA    = 11520;
localparam integer SUNK_CY2        = 15;
localparam integer REF_TTL         = 300;
localparam integer COLLAPSE_FRAMES = 36;
localparam integer RECOVERY_WINDOW = 2;
localparam integer COOLDOWN_FRAMES = 60;

localparam integer N_FRAME = 8;

reg         clk = 1'b0;
reg         rst_n = 1'b0;
reg         en_drive = 1'b0;
reg         en_free = 1'b0;      // 0 = per-frame strobe, 1 = tied high
reg         valid = 1'b0;
reg  [31:0] frame = 32'd0;
reg  [9:0]  x = 10'd0, y = 10'd0, w = 10'd0, h = 10'd0;
reg  [23:0] area = 24'd0;
reg  [2:0]  confirm_required = 3'd2;

wire        en = en_free ? 1'b1 : en_drive;

wire        alarm;
wire [2:0]  state;

posture_fsm #(
    .FRAME_W         (FRAME_W),
    .FRAME_H         (FRAME_H),
    .MIN_REF_AREA    (MIN_REF_AREA),
    .MIN_REF_HEIGHT  (MIN_REF_HEIGHT),
    .MAX_REF_W       (MAX_REF_W),
    .MAX_REF_H       (MAX_REF_H),
    .MAX_REF_AREA    (MAX_REF_AREA),
    .SUNK_CY2        (SUNK_CY2),
    .REF_TTL         (REF_TTL),
    .COLLAPSE_FRAMES (COLLAPSE_FRAMES),
    .RECOVERY_WINDOW (RECOVERY_WINDOW),
    .COOLDOWN_FRAMES (COOLDOWN_FRAMES)
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

    // This bench owns the engine, so the time-shared context ports stay off.
    .ctx_in           ({242{1'b0}}),
    .ctx_load         (1'b0),
    .ctx_out          ()
);

always #5 clk = ~clk;

// --------------------------------------------------------------- run monitor
integer run_len  = 0;
integer max_run  = 0;
integer alarms   = 0;
reg     alarm_d  = 1'b0;

always @(negedge clk) begin
    if (alarm) begin
        run_len = run_len + 1;
        if (run_len > max_run) max_run = run_len;
    end else begin
        run_len = 0;
    end
    if (alarm && !alarm_d) alarms = alarms + 1;
    alarm_d = alarm;
end

task reset_counters;
begin
    run_len = 0;
    max_run = 0;
    alarms  = 0;
    alarm_d = 1'b0;
end
endtask

task set_obs(input integer f);
begin
    frame = f[31:0];
    case (f)
        0, 1, 2: begin
            valid = 1'b1;
            x = 10'd140; y = 10'd30; w = 10'd40; h = 10'd120; area = 24'd4000;
        end
        3, 4: begin
            valid = 1'b1;
            x = 10'd110; y = 10'd110; w = 10'd90; h = 10'd24; area = 24'd1900;
        end
        default: begin
            valid = 1'b0;
            x = 10'd0; y = 10'd0; w = 10'd0; h = 10'd0; area = 24'd0;
        end
    endcase
end
endtask

task do_reset;
begin
    rst_n    = 1'b0;
    en_drive = 1'b0;
    valid    = 1'b0;
    frame    = 32'd0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
end
endtask

// ------------------------------------------------------------------ stimulus
integer ph;
integer f;
integer alarm_a, max_a;
integer alarm_b, max_b;
integer errors = 0;

initial begin
    if ($test$plusargs("VCD")) begin
        $dumpfile("tb_alarm_pulse.vcd");
        $dumpvars(0, tb_alarm_pulse);
    end

    // ---- phase A: en strobed once per observation (board wiring) ----------
    do_reset;
    en_free = 1'b0;
    reset_counters;
    for (f = 0; f < N_FRAME; f = f + 1) begin
        // observation is presented while the strobe is low, then latched by
        // one enabled clock; the following clocks are a normal frame gap
        set_obs(f);
        @(negedge clk);
        en_drive = 1'b1;
        @(negedge clk);
        en_drive = 1'b0;
        repeat (3) @(negedge clk);
    end
    repeat (2) @(negedge clk);
    alarm_a = alarms;
    max_a   = max_run;

    // a second, clean gap phase: alarm must not re-assert on its own
    repeat (4) @(negedge clk);
    if (alarm !== 1'b0) begin
        $display("FAIL alarm stuck high outside an enabled clock");
        errors = errors + 1;
    end

    // ---- phase B: en tied high (core wiring) ------------------------------
    //
    // The core feeds one observation per enabled clock, but posture_fsm
    // registers each observation one clock before the state machine sees it.
    // Hold every observation for two clocks so the pipelined machine gets
    // exactly one clean beat per frame, then drain a blank window so the
    // silent-confirm path can fire.
    en_free = 1'b1;
    do_reset;
    reset_counters;
    for (f = 0; f < N_FRAME; f = f + 1) begin
        set_obs(f);
        @(negedge clk);
        @(negedge clk);
    end
    repeat (4) @(negedge clk);
    alarm_b = alarms;
    max_b   = max_run;

    $display("PHASE-A strobed  alarms=%0d max_run=%0d clocks", alarm_a, max_a);
    $display("PHASE-B tied-hi alarms=%0d max_run=%0d clocks", alarm_b, max_b);
    if (alarm_a != 1) begin
        $display("FAIL strobed wiring produced %0d alarms, expected 1", alarm_a);
        errors = errors + 1;
    end
    if (max_a != 1) begin
        $display("FAIL strobed wiring held alarm for %0d clocks, expected 1", max_a);
        errors = errors + 1;
    end
    if (alarm_b != 1) begin
        $display("FAIL free-running wiring produced %0d alarms, expected 1", alarm_b);
        errors = errors + 1;
    end
    if (max_b != 1) begin
        $display("FAIL free-running wiring held alarm for %0d clocks, expected 1", max_b);
        errors = errors + 1;
    end

    if (errors == 0) $display("PASS tb_alarm_pulse");
    else             $display("FAIL tb_alarm_pulse errors=%0d", errors);
    $finish;
end

endmodule

