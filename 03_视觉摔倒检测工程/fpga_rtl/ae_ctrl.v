//===========================================================================
// ae_ctrl.v -- on-chip auto exposure for the MT9M001 front end
//
// The ID-003A module ships without a lens-side AGC/AEC loop and its power-on
// shutter is "as long as a whole frame", which saturates the 10-bit ADC in
// any normally lit room (the part is specified at 2.1 V/lux-s).  This block
// closes a slow exposure loop in the fabric instead of asking the operator to
// hand-tune a register.
//
// Every frame it sums every SAMPLE_DIV'th pixel of the 640x480 gray stream,
// so a frame contributes SAMPLES values in 0..255.  Comparing that sum
// against SAMPLES*LOW_MEAN / SAMPLES*HIGH_MEAN needs no divider at all: the
// thresholds are constants.  When the frame is outside the dead band the
// shutter register (0x09, integration rows) is stepped by 1/32 of its current
// value; only when the shutter is already at its limit does the loop fall
// through to the global gain register (0x35).
//
// The gain register is not one linear ladder.  Table 6 of the MT9M001
// datasheet gives three recommended ranges:
//
//   1.00x .. 4.00x   0.125 steps   codes 0x08 .. 0x20
//   4.25x .. 8.00x   0.250 steps   codes 0x51 .. 0x60   (bit6 multiplicative)
//   9x    .. 15x     1.0   steps   codes 0x61 .. 0x67
//
// The loop walks an index across all three ranges, so a dim room can be
// answered with analogue gain - which is applied before the ADC and therefore
// also buys back quantisation headroom - instead of dropping the frame rate
// with an exposure longer than the frame.  The 9x..15x range is the last
// resort: a step there is a whole 1x (12.5% at the bottom of the range), but
// it can only be *entered* while the picture is still below LOW_MEAN at 8.0x,
// so the first step lands at 8/9*LOW_MEAN..(8/9)*HIGH_MEAN < HIGH_MEAN - the
// coarse ladder cannot overshoot the dead band on its own.
//
// The dead band is the picture the operator actually sees, so it is set for a
// *viewable* exposure rather than for "not clipped".  The first board build
// used 70..150: a dim room walks the loop up until the mean just crosses 70
// and it stops there, which is a 27% grey picture - visibly dark on a monitor
// even though the loop reported "in band".  The band is now 115..165, i.e. the
// loop keeps brightening until the frame is at least 45% grey.  A scene the
// loop walks *down* into the band parks just under HIGH_MEAN, which is the
// 64% grey it should be for a normally lit room.
//
// mean_dbg exposes the frame mean the loop measured (0..255) so the status
// UART can print it: (acc_last * 27962) >> 30 is acc_last / 38400 with a
// 0.002% error, which is one DSP multiply and no divider.
//
// Why this matters on the board: with the loop pinned at shutter 520 and gain
// 4.0x the measured mean in a night-lit room is 38/255, i.e. the part has no
// exposure left.  One more stop of analogue gain costs nothing at 33 fps and
// is the only lever left that does not stretch the frame.
//
// Three rules keep the loop from disturbing the detector:
//
//   * The write is issued on a frame_start pulse, i.e. during FRAME_VALID
//     blanking, so a register the sensor applies immediately can only touch
//     the frame that is about to be frozen - never the tail of a frame the
//     vision core is already watching.
//   * freeze is raised in that same cycle and covers FREEZE_FRAMES whole
//     frames, de and frame_start together.  The top hands the vision core a
//     stream with those frames removed rather than a partial frame: the
//     multi_bbox pass counter is a plain 640x480 pixel census, so one short
//     frame would shift every following row for good.
//   * freeze is released on frame_end - FRAME_VALID's trailing edge, one
//     vertical blank before the next frame_start - so the first frame the
//     core sees again opens with its own frame_start pulse and is complete.
//
// The step is deliberately small (1/32, about 3%): the difference between the
// last frame before a change and the first frame after it is what frame_diff
// sees, and DIFF_THRESH is 10 counts out of 255, so a step that moves a bright
// pixel by more than 10 would hand the detector a screenful of false motion.
// With the dead band plus an 8-frame cadence it takes a few seconds to walk a
// badly exposed scene into range, and then the loop stops writing altogether.
//
// The register model kept here starts at the same values the boot table in
// mt9m001_cfg.v writes, so the first adjustment is a step from the real
// register value rather than from an arbitrary guess.
//===========================================================================
`timescale 1ns / 1ps

module ae_ctrl #(
    parameter integer ENABLE        = 1,
    parameter integer SHUT_INIT     = 200,
    parameter integer SHUT_MIN      = 16,
    parameter integer SHUT_MAX      = 520,
    parameter integer GAIN_INIT_IDX = 0,     // 1.0x
    parameter integer GAIN_MIN_IDX  = 0,     // 1.0x, code 0x08
    parameter integer GAIN_MAX_IDX  = 47,    // 15x,  code 0x67
    parameter integer SAMPLE_DIV    = 8,
    parameter integer SAMPLES       = 38400,   // 640*480/SAMPLE_DIV
    parameter integer LOW_MEAN      = 115,     // brighten below this frame mean
    parameter integer HIGH_MEAN     = 165,     // darken above this frame mean
    parameter integer PERIOD_FRAMES = 4,
    parameter integer FREEZE_FRAMES = 2
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        de,           // 640x480 gray stream, one pixel/clock
    input  wire [7:0]  gray,
    input  wire        frame_start,  // one pulse per frame, in blanking
    input  wire        frame_end,    // one pulse per frame, in blanking

    output reg         req_tog,      // toggle to mt9m001_cfg's dynamic port
    output reg  [7:0]  req_addr,
    output reg  [15:0] req_data,
    input  wire        ack_tog,

    output wire        freeze,
    output reg  [1:0]  zone,         // 0 dark, 1 in band, 2 bright
    output reg  [15:0] shutter_dbg,
    output reg  [7:0]  gain_dbg,
    output wire [7:0]  mean_dbg      // last frame's mean, 0..255
);

    localparam [23:0] LOW_ACC  = SAMPLES * LOW_MEAN;
    localparam [23:0] HIGH_ACC = SAMPLES * HIGH_MEAN;

    reg [7:0]  sample_cnt;
    reg [23:0] acc;
    reg [23:0] acc_last;
    reg        acc_valid;
    // Cadence slot counter: counts 0..PERIOD_FRAMES-1 and wraps.  This used
    // to be an 8-bit frame counter tested with %, which saturates at 0xFF
    // after 255 frames and then never matches the modulo again - the loop
    // adjusted the picture for eight seconds and then went deaf forever.
    reg [7:0]  cadence_cnt;

    reg [15:0] shutter;
    reg [7:0]  gain;
    reg [1:0]  ack_sync;
    reg        pending;
    reg [2:0]  freeze_left;

    // acc_last / SAMPLES as an 8-bit mean, without a divider: round(2^30 /
    // SAMPLES) is a compile-time constant, and for the production geometry it
    // is 27962, i.e. 0.002% off a true divide.
    localparam integer MEAN_MUL = ((1 << 30) + (SAMPLES / 2)) / SAMPLES;
    wire [47:0] mean_prod = acc_last * MEAN_MUL;
    assign mean_dbg = mean_prod[37:30];

    localparam [15:0] SHUT_INIT_W = SHUT_INIT;
    localparam [15:0] SHUT_MAX_W  = SHUT_MAX;
    localparam [15:0] SHUT_MIN_W  = SHUT_MIN;
    localparam [7:0]  GAIN_INIT_W = GAIN_INIT_IDX;
    localparam [7:0]  GAIN_MIN_W  = GAIN_MIN_IDX;
    localparam [7:0]  GAIN_MAX_W  = GAIN_MAX_IDX;

    // Index -> 0x35 code across the three recommended ranges.
    function [7:0] gain_code;
        input [7:0] idx;
        begin
            if (idx <= 8'd24)
                gain_code = 8'd8 + idx;                       // 0x08..0x20
            else if (idx <= 8'd40)
                gain_code = 8'h51 + idx - 8'd25;              // 0x51..0x60
            else
                gain_code = 8'h61 + idx - 8'd41;              // 0x61..0x67
        end
    endfunction

    wire ack_edge = ack_sync[1] ^ ack_sync[0];

    // Combinational next values: the step and the new register value must be
    // resolved before the request is latched, so they cannot be registers
    // that are updated in the same cycle they are read.  >>5 is the 3% step
    // discussed in the header; the gain register only has 1/8 steps.
    wire [15:0] step_now = (shutter >> 5) ? (shutter >> 5) : 16'd1;
    wire [15:0] shut_up  = ((shutter + step_now) > SHUT_MAX_W)
                           ? SHUT_MAX_W : (shutter + step_now);
    wire [15:0] shut_dn  = ((shutter > (step_now + SHUT_MIN_W)) &&
                            ((shutter - step_now) >= SHUT_MIN_W))
                           ? (shutter - step_now) : SHUT_MIN_W;

    wire too_bright = (acc > HIGH_ACC);
    wire too_dark   = (acc < LOW_ACC);
    wire cadence    = (cadence_cnt == 8'd0);

    wire can_shut_dn = (shutter > SHUT_MIN_W);
    wire can_shut_up = (shutter < SHUT_MAX_W);
    wire can_gain_dn = (gain > GAIN_MIN_W);
    wire can_gain_up = (gain < GAIN_MAX_W);

    wire want_write = (ENABLE != 0) && acc_valid && cadence &&
                      (freeze_left == 3'd0) && !pending &&
                      ((too_bright && (can_shut_dn || can_gain_dn)) ||
                       (too_dark   && (can_shut_up || can_gain_up)));

    // frame_start arrives in blanking, so this pulse is the natural moment to
    // post the write and open the freeze window at the same time.
    wire issue_now = frame_start && want_write;

    // High for the issued frame and for every frame the counter still owes, so
    // the core loses exactly FREEZE_FRAMES complete frames.
    assign freeze = issue_now || (freeze_left != 3'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_cnt  <= 8'd0;
            acc         <= 24'd0;
            acc_last    <= 24'd0;
            acc_valid   <= 1'b0;
            cadence_cnt <= 8'd0;
            shutter     <= SHUT_INIT_W;
            gain        <= GAIN_INIT_W;
            ack_sync    <= 2'b00;
            pending     <= 1'b0;
            freeze_left <= 3'd0;
            req_tog     <= 1'b0;
            req_addr    <= 8'h09;
            req_data    <= SHUT_INIT_W;
            zone        <= 2'd1;
            shutter_dbg <= SHUT_INIT_W;
            gain_dbg    <= gain_code(GAIN_INIT_W);
        end else begin
            ack_sync <= {ack_sync[0], ack_tog};

            if (ack_edge && pending) pending <= 1'b0;

            // ------------------------------------------------- accumulate
            if (de) begin
                if (sample_cnt == SAMPLE_DIV - 1) begin
                    sample_cnt <= 8'd0;
                    acc <= acc + {16'd0, gray};
                end else begin
                    sample_cnt <= sample_cnt + 8'd1;
                end
            end

            // ------------------------------------------------- frame tick
            if (frame_start) begin
                acc_last  <= acc;
                acc       <= 24'd0;
                acc_valid <= 1'b1;
                cadence_cnt <= (cadence_cnt >= (PERIOD_FRAMES - 1))
                               ? 8'd0 : (cadence_cnt + 8'd1);

                if (acc_valid) begin
                    if (acc > HIGH_ACC)      zone <= 2'd2;
                    else if (acc < LOW_ACC)  zone <= 2'd0;
                    else                     zone <= 2'd1;
                end
            end

            // ------------------------------------------ exposure adjust
            if (issue_now) begin
                pending     <= 1'b1;
                freeze_left <= FREEZE_FRAMES;
                req_tog     <= ~req_tog;

                if (too_bright) begin
                    if (can_shut_dn) begin
                        shutter     <= shut_dn;
                        req_addr    <= 8'h09;
                        req_data    <= shut_dn;
                        shutter_dbg <= shut_dn;
                    end else begin
                        gain     <= gain - 8'd1;
                        req_addr <= 8'h35;
                    req_data <= {8'd0, gain_code(gain - 8'd1)};
                    gain_dbg <= gain_code(gain - 8'd1);
                    end
                end else begin
                    if (can_shut_up) begin
                        shutter     <= shut_up;
                        req_addr    <= 8'h09;
                        req_data    <= shut_up;
                        shutter_dbg <= shut_up;
                    end else begin
                        gain     <= gain + 8'd1;
                        req_addr <= 8'h35;
                    req_data <= {8'd0, gain_code(gain + 8'd1)};
                    gain_dbg <= gain_code(gain + 8'd1);
                    end
                end
            end

            // -------------------------------------------- freeze release
            // frame_end is FRAME_VALID's trailing edge, a whole vertical
            // blank before the next frame_start, so dropping freeze here
            // still lets the next frame_start pulse through to the core.
            if (frame_end && (freeze_left != 3'd0))
                freeze_left <= freeze_left - 3'd1;
        end
    end

endmodule
