//===========================================================================
// tb_ae_ctrl.v -- does the exposure loop walk the picture into the new band?
//
// The symptom this pins down is "the HDMI picture is too dark even though the
// exposure loop reports in band".  The loop is closed around a small sensor
// model, pixel = scene * shutter * gain_x8 / (200 * 8) clamped to 255, so
// shutter 200 with gain 1.0x is exactly the boot table's exposure.  The model
// converts the 0x35 *code* to eighths of a stop the way Table 6 of the
// MT9M001 datasheet defines it (0x08..0x20 are 1/8 steps, 0x51..0x60 are
// 1/4 steps with bit 6 as a multiplier, 0x61..0x67 are the 1x night ladder),
// so a scene that needs more light really does start dark and each ladder is
// reached only when the one before it is exhausted.
//
//   phase 1  scene   6  below LOW_MEAN  -> shutter must run up, then gain
//   phase 2  scene 220  above HIGH_MEAN -> shutter must run down
//   phase 3  scene   4  dimmer than the 8x ladder can fix -> the 9x..15x
//                       ladder must finish the job and still park in band
//
// In both phases the loop has to park the mean inside 115..165, stop writing
// once it is there, and never issue a write less than PERIOD_FRAMES apart.
//===========================================================================
`timescale 1ns / 1ps

module tb_ae_ctrl;

// 80x60 grey, one sample every 8 pixels: 600 samples per frame, which keeps a
// 200-frame convergence run down to ~800k clocks.
localparam integer W       = 80;
localparam integer H       = 60;
localparam integer DIV     = 8;
localparam integer SAMPLES = (W * H) / DIV;

reg clk = 1'b0;
always #20.8335 clk = ~clk;          // 24 MHz

reg        rst_n       = 1'b0;
reg        de          = 1'b0;
reg [7:0]  gray        = 8'd0;
reg        frame_start = 1'b0;
reg        frame_end   = 1'b0;
reg        ack_tog     = 1'b0;

wire        req_tog;
wire [7:0]  req_addr;
wire [15:0] req_data;
wire        freeze;
wire [1:0]  zone;
wire [15:0] shutter_dbg;
wire [7:0]  gain_dbg, mean_dbg;

ae_ctrl #(
    .SHUT_INIT     (200),
    .SHUT_MIN      (16),
    .SHUT_MAX      (520),
    .GAIN_INIT_IDX (0),
    .GAIN_MIN_IDX  (0),
    .GAIN_MAX_IDX  (47),
    .SAMPLE_DIV    (DIV),
    .SAMPLES       (SAMPLES),
    .LOW_MEAN      (115),
    .HIGH_MEAN     (165),
    .PERIOD_FRAMES (4),
    .FREEZE_FRAMES (2)
) dut (
    .clk         (clk),
    .rst_n       (rst_n),
    .de          (de),
    .gray        (gray),
    .frame_start (frame_start),
    .frame_end   (frame_end),
    .req_tog     (req_tog),
    .req_addr    (req_addr),
    .req_data    (req_data),
    .ack_tog     (ack_tog),
    .freeze      (freeze),
    .zone        (zone),
    .shutter_dbg (shutter_dbg),
    .gain_dbg    (gain_dbg),
    .mean_dbg    (mean_dbg)
);

// ------------------------------------------------------------------ wire model
// The board's write engine answers a request about 32 us later (2.4 kHz of
// SCL at 100 kHz); 100 clocks is close enough to make the handshake matter.
reg       req_d    = 1'b0;
reg [7:0] ack_wait = 8'd0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_d    <= 1'b0;
        ack_wait <= 8'd0;
        ack_tog  <= 1'b0;
    end else begin
        req_d <= req_tog;
        if (req_d ^ req_tog) begin
            ack_wait <= 8'd100;
        end else if (ack_wait != 8'd0) begin
            ack_wait <= ack_wait - 8'd1;
            if (ack_wait == 8'd1) ack_tog <= ~ack_tog;
        end
    end
end

// ------------------------------------------------------------------ bookkeeping
integer errors = 0;
integer frames = 0;
integer writes = 0;

reg req_p = 1'b0;
integer frames_at_last_write = 0;
integer min_write_gap = 1000000;
integer trace_writes = 0;

always @(posedge clk) begin
    if (rst_n) begin
        req_p <= req_tog;
        if (req_p ^ req_tog) begin
            writes = writes + 1;
            if ((frames - frames_at_last_write) < min_write_gap)
                min_write_gap = frames - frames_at_last_write;
            frames_at_last_write = frames;
            if (trace_writes)
                $display("  WRITE %0d: frame %0d addr %02h data %04h",
                         writes, frames, req_addr, req_data);
        end
    end
end

task one_frame;
    input integer scene;
    integer r, c, v;
    integer eff_shut, eff_gain, gx8;
    begin
        @(negedge clk); frame_start = 1'b1;
        @(negedge clk); frame_start = 1'b0;
        // The write this frame_start triggered lands in the sensor before the
        // first row is read out, which is what the real part does.
        @(negedge clk); eff_shut = shutter_dbg; eff_gain = gain_dbg;
        // 0x35 code -> eighths of a stop: 0x08..0x20 pass through as 1..4x,
        // 0x51..0x60 are 4.25x..8x in 1/4 steps, 0x61..0x67 are 9x..15x.
        if (eff_gain <= 32)      gx8 = eff_gain;
        else if (eff_gain <= 96) gx8 = 2 * eff_gain - 128;
        else                     gx8 = 8 * (eff_gain - 88);

        for (r = 0; r < H; r = r + 1) begin
            for (c = 0; c < W; c = c + 1) begin
                v = (scene * eff_shut * gx8) / 1600;   // 200 rows x 1.0x
                if (v > 255) v = 255;
                de   <= 1'b1;
                gray <= v[7:0];
                @(negedge clk);
            end
        end

        de <= 1'b0;
        @(negedge clk); frame_end = 1'b1;
        @(negedge clk); frame_end = 1'b0;
        frames = frames + 1;
    end
endtask

task run_scene;
    input integer scene;
    input integer nframes;
    input [8*24-1:0] tag;
    integer i;
    integer writes_before;
    integer frame_before;
    begin
        writes_before = writes;
        frame_before  = frames;
        for (i = 0; i < nframes; i = i + 1) one_frame(scene);

        $display("%0s: %0d frames, %0d writes, mean %0d shutter %0d gain %0d zone %0d",
                 tag, frames - frame_before, writes - writes_before,
                 mean_dbg, shutter_dbg, gain_dbg, zone);

        // 1. the picture must have landed inside the dead band
        if (zone !== 2'd1) begin
            errors = errors + 1;
            $display("FAIL %0s: zone %0d, want 1 (in band)", tag, zone);
        end
        if (mean_dbg < 8'd112 || mean_dbg > 8'd168) begin
            errors = errors + 1;
            $display("FAIL %0s: mean %0d outside 112..168", tag, mean_dbg);
        end

        // 2. a settled loop must stop writing: no request in the last 40
        //    frames, which is ten cadence slots
        if ((frames - frames_at_last_write) < 40) begin
            errors = errors + 1;
            $display("FAIL %0s: still writing %0d frames after the last write",
                     tag, frames - frames_at_last_write);
        end
    end
endtask

task reset_dut;
    begin
        rst_n = 1'b0;
        repeat (8) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);
    end
endtask

initial begin
    trace_writes = $test$plusargs("writes");
    reset_dut;

    // 400 frames is ~100 cadence slots: enough for shutter 200 -> 520 (32
    // writes) plus the 38 gain steps a very dim scene needs to reach 8.0x's
    // neighbourhood (code 0x5E is where this model lands in band).
    run_scene(6, 400, "dark ");

    if (shutter_dbg != 16'd520) begin
        errors = errors + 1;
        $display("FAIL dark: shutter %0d, want the 520 limit", shutter_dbg);
    end
    if (gain_dbg < 8'h51) begin
        errors = errors + 1;
        $display("FAIL dark: gain code %02h, want the 4.25x..8x ladder", gain_dbg);
    end

    // Back to the boot exposure, so the bright scene walks the shutter down
    // from 200 to ~146 in ten writes instead of unwinding the dark run first.
    reset_dut;
    run_scene(220, 120, "bright");

    if (shutter_dbg >= 16'd520) begin
        errors = errors + 1;
        $display("FAIL bright: shutter %0d, want it back below the limit",
                 shutter_dbg);
    end

    // A room dimmer than 8.0x can fix: the loop must walk into the 9x..15x
    // ladder, land in band there, and leave the shutter at its limit.
    reset_dut;
    run_scene(4, 480, "night");

    if (gain_dbg < 8'h61) begin
        errors = errors + 1;
        $display("FAIL night: gain code %02h, want the 9x..15x ladder", gain_dbg);
    end
    if (shutter_dbg != 16'd520) begin
        errors = errors + 1;
        $display("FAIL night: shutter %0d, want the 520 limit", shutter_dbg);
    end

    // 3. the loop only ever writes on its 4-frame cadence
    if (min_write_gap < 4) begin
        errors = errors + 1;
        $display("FAIL: writes %0d frames apart, want >= 4", min_write_gap);
    end

    if (errors == 0)
        $display("PASS tb_ae_ctrl: dark, bright and night scenes all park in 115..165");
    else
        $display("FAIL tb_ae_ctrl: %0d errors", errors);
    $finish;
end

initial begin
    #600000000;   // 600 ms in 1 ns units: ~200 us per modelled frame
    $display("FAIL tb_ae_ctrl: timeout");
    $finish;
end

endmodule
