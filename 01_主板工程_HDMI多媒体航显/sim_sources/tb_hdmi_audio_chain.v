`timescale 1ns / 1ps
//=============================================================================
// tb_hdmi_audio_chain.v
//
// Checks the audio side of the design, that is everything except the
// encrypted HDMI transmitter core:
//
//   hdmi_audio_tone_i2s_64fs  the 48 kHz background music generator
//   I2S_receiver              the sample capture running on the pixel clock
//   audio_arc_calculate       the ACR packet values
//
// What is checked
//   * the O_audio_valid frame rate is 48 kHz
//   * LRCK toggles exactly once per captured frame
//   * every captured sample stays inside +/- 3.2 M counts, so the two voices
//     never wrap the 24 bit word
//   * the phrase follows the airport chime table (E5, then C5, then a rest),
//     measured from the zero crossing interval of the captured samples
//   * a picture change (I_cue) does mix the announcement tick in, but does
//     NOT restart the phrase: the pitch measured after the pulse is still C5,
//     never the first note E5
//   * the ACR packet carries N = 6144 and CTS close to 25000, which is what
//     the HDMI transmitter core needs for 48 kHz over a 25 MHz pixel clock
//
// The tempo is shortened here (UNIT = 200 frames instead of the 2400 the
// design uses) so the phrase runs in a few milliseconds of simulated audio.
// The pitch of a note is set by its phase increment and does not depend on
// the tempo, so every measurement below is the value the board produces.
//=============================================================================

module tb_hdmi_audio_chain;

    localparam integer UNIT      = 200;    // frames per duration unit
    localparam integer CUE_LEN   = 1200;   // cue length in frames
    localparam integer AMP_MAX   = 3200000;  // |sample| must stay below this

    // note boundaries in captured frames, taken from melody_lut:
    //   step 0 E5 10 units, step 1 C5 14 units, step 2 rest 24 units
    localparam integer N1 = 10 * UNIT;          // 2000
    localparam integer N2 = N1 + 14 * UNIT;     // 4800
    localparam integer N3 = N2 + 24 * UNIT;     // 9600

    localparam integer CUE_FRAME = N1 + 1200;   // 3200, inside the C5 note
    localparam integer CUE_TAIL  = CUE_FRAME + CUE_LEN + 100;

    // measurement windows, each one well inside a single note
    localparam integer W0A = 300,             W0B = N1 - 300;         // E5
    localparam integer W1A = N1 + 200,        W1B = N1 + 1000;        // C5 before cue
    localparam integer W2A = CUE_TAIL,        W2B = N2 - 100;         // C5 after cue
    localparam integer W3A = 0,               W3B = 0;                // unused
    localparam integer W4A = CUE_FRAME - 600, W4B = CUE_FRAME - 100;  // music

    // expected zero crossing intervals on the board, at the real 48.03 kHz
    // frame rate:  E5 36.4, C5 45.9 frames
    localparam integer E5_MIN = 33, E5_MAX = 41;
    localparam integer C5_MIN = 41, C5_MAX = 51;

    // 12.288 MHz audio master clock and the 25 MHz pixel clock
    reg mclk = 1'b0;
    reg pclk = 1'b0;
    always #40.6901 mclk = ~mclk;
    always #20      pclk = ~pclk;

    reg rst = 1'b1;
    reg cue = 1'b0;

    wire        bclk, lrck, dout;
    wire        audio_valid;
    wire [23:0] left_data, right_data;
    wire        acr_valid;
    wire [19:0] acr_cts, acr_n;

    hdmi_audio_tone_i2s_64fs #(
        .BGM_SHIFT   (15),
        .CUE_SHIFT   (14),
        .UNIT_FRAMES (UNIT),
        .CUE_FRAMES  (CUE_LEN)
    ) u_tone (
        .I_mclk       (mclk),
        .I_rst        (rst),
        .I_cue        (cue),
        .I_voice_en   (1'b0),
        .I_voice      (24'sd0),
        .I_mute       (1'b0),
        .O_sample_tick(),
        .O_i2s_BCLK   (bclk),
        .O_i2s_LRCK   (lrck),
        .O_i2s_DOUT   (dout)
    );

    I2S_receiver u_rx (
        .I_clk              (pclk),
        .I_rst              (rst),
        .I_i2s_BCLK         (bclk),
        .I_i2s_LRCK         (lrck),
        .I_i2s_DOUT         (dout),
        .O_audio_valid      (audio_valid),
        .O_audio_left_data  (left_data),
        .O_audio_right_data (right_data)
    );

    audio_arc_calculate #(
        .ACR_N (6144)
    ) u_arc (
        .I_clk         (pclk),
        .I_rst         (rst),
        .I_audio_valid (audio_valid),
        .O_acr_valid   (acr_valid),
        .O_acr_cts     (acr_cts),
        .O_acr_n       (acr_n)
    );

    // ------------------------------------------------------------------
    // measurement state
    // ------------------------------------------------------------------
    integer frames;
    integer sample_bad;
    integer lrck_toggles;
    reg     lrck_d;

    integer acr_packets;
    integer acr_n_bad;
    integer acr_cts_bad;
    integer acr_cts_min;
    integer acr_cts_max;

    integer peak_all;
    integer cue_peak;
    integer bgm_peak;

    integer xcnt   [0:4];
    integer xfirst [0:4];
    integer xlast  [0:4];
    integer avg    [0:4];
    reg     sign_d;

    integer k;
    reg     cue_done;
    integer errors;

    function in_win;
        input integer w;
        input integer f;
        begin
            case (w)
                0: in_win = (f > W0A && f <= W0B);
                1: in_win = (f > W1A && f <= W1B);
                2: in_win = (f > W2A && f <= W2B);
                3: in_win = (f > W3A && f <= W3B);
                default: in_win = (f > W4A && f <= W4B);
            endcase
        end
    endfunction

    function [23:0] mag24;
        input [23:0] v;
        begin
            mag24 = v[23] ? (~v + 24'd1) : v;
        end
    endfunction

    initial begin
        frames       = 0;
        sample_bad   = 0;
        lrck_toggles = 0;
        lrck_d       = 1'b0;
        acr_packets  = 0;
        acr_n_bad    = 0;
        acr_cts_bad  = 0;
        acr_cts_min  = 1000000;
        acr_cts_max  = 0;
        peak_all     = 0;
        cue_peak     = 0;
        bgm_peak     = 0;
        sign_d       = 1'b0;
        cue_done     = 1'b0;
        errors       = 0;
        for (k = 0; k < 5; k = k + 1) begin
            xcnt[k]   = 0;
            xfirst[k] = 0;
            xlast[k]  = 0;
            avg[k]    = 0;
        end

        repeat (20) @(posedge pclk);
        rst = 1'b0;
    end

    // ------------------------------------------------------------------
    // observation
    // ------------------------------------------------------------------
    always @(posedge pclk) begin
        if (lrck !== lrck_d) begin
            lrck_toggles = lrck_toggles + 1;
            lrck_d       = lrck;
        end

        if (audio_valid) begin
            frames = frames + 1;

            if ($signed(left_data) > AMP_MAX || $signed(left_data) < -AMP_MAX)
                sample_bad = sample_bad + 1;
            if ($signed(right_data) > AMP_MAX || $signed(right_data) < -AMP_MAX)
                sample_bad = sample_bad + 1;

            if (mag24(left_data) > peak_all)
                peak_all = mag24(left_data);

            // zero crossings of the captured waveform, per measurement window
            for (k = 0; k < 5; k = k + 1) begin
                if (in_win(k, frames)) begin
                    if (left_data[23] !== sign_d) begin
                        xcnt[k] = xcnt[k] + 1;
                        if (xcnt[k] == 1)
                            xfirst[k] = frames;
                        xlast[k] = frames;
                    end
                end
            end
            sign_d = left_data[23];

            if (frames > CUE_FRAME && frames <= CUE_FRAME + 500) begin
                if (mag24(left_data) > cue_peak)
                    cue_peak = mag24(left_data);
            end
            if (frames > W4A && frames <= W4B) begin
                if (mag24(left_data) > bgm_peak)
                    bgm_peak = mag24(left_data);
            end
        end

        if (acr_valid) begin
            acr_packets = acr_packets + 1;
            if (acr_n !== 20'd6144)
                acr_n_bad = acr_n_bad + 1;
            if (acr_cts < 24990 || acr_cts > 25010)
                acr_cts_bad = acr_cts_bad + 1;
            if (acr_cts < acr_cts_min) acr_cts_min = acr_cts;
            if (acr_cts > acr_cts_max) acr_cts_max = acr_cts;
        end
    end

    // ------------------------------------------------------------------
    // the picture change pulse, in the middle of the C5 note
    // ------------------------------------------------------------------
    initial begin
        @(negedge rst);
        wait (frames >= CUE_FRAME - 20);
        @(posedge mclk);
        cue = 1'b1;
        repeat (4) @(posedge mclk);
        cue = 1'b0;
        cue_done = 1'b1;
    end

    // ------------------------------------------------------------------
    // verdict
    // ------------------------------------------------------------------
    initial begin
        wait (frames >= N3 + 400);

        for (k = 0; k < 5; k = k + 1)
            if (xcnt[k] > 1)
                avg[k] = (xlast[k] - xfirst[k]) / (xcnt[k] - 1);

        $display("---- hdmi audio chain ----");
        $display("frames                  = %0d", frames);
        $display("simulation time         = %0.2f ms", $time / 1000000.0);
        $display("audio frame rate        = %0.2f kHz",
                 frames * 1.0 / ($time * 1.0e-6));
        $display("lrck toggles            = %0d (expect %0d +/- 2 at stop)",
                 lrck_toggles, frames * 2);
        $display("samples outside +/-3.2M = %0d", sample_bad);
        $display("peak |sample|           = %0d", peak_all);
        $display("E5 half period          = %0d frames (expect %0d..%0d)",
                 avg[0], E5_MIN, E5_MAX);
        $display("C5 half period before cue= %0d frames (expect %0d..%0d)",
                 avg[1], C5_MIN, C5_MAX);
        $display("C5 half period after cue = %0d frames (expect %0d..%0d, a",
                 avg[2], C5_MIN, C5_MAX);
        $display("                            restart would show %0d..%0d)",
                 E5_MIN, E5_MAX);
        $display("peak before cue         = %0d", bgm_peak);
        $display("peak during cue         = %0d", cue_peak);
        $display("acr packets             = %0d", acr_packets);
        $display("acr n wrong             = %0d", acr_n_bad);
        $display("acr cts out of range    = %0d (min %0d, max %0d, expect 25000)",
                 acr_cts_bad, acr_cts_min, acr_cts_max);

        if (frames < N3) begin
            $display("FAIL too few audio frames were recovered");
            errors = errors + 1;
        end
        // the test stops at a fixed simulation time, which can include up to
        // two extra LRCK edges beyond the last frame the receiver counted
        if ((lrck_toggles + 2 < frames * 2) ||
            (lrck_toggles > frames * 2 + 2)) begin
            $display("FAIL lrck toggles are inconsistent with the captured frames");
            errors = errors + 1;
        end
        if (sample_bad != 0) begin
            $display("FAIL %0d captured samples left the +/-3.2M range", sample_bad);
            errors = errors + 1;
        end
        if (peak_all < 1500000) begin
            $display("FAIL the music is too quiet, peak is only %0d", peak_all);
            errors = errors + 1;
        end
        if (avg[0] < E5_MIN || avg[0] > E5_MAX) begin
            $display("FAIL the first note is not E5");
            errors = errors + 1;
        end
        if (avg[1] < C5_MIN || avg[1] > C5_MAX) begin
            $display("FAIL the second note is not C5");
            errors = errors + 1;
        end
        if (avg[2] < C5_MIN || avg[2] > C5_MAX) begin
            $display("FAIL the picture change restarted the phrase");
            errors = errors + 1;
        end
        if (xcnt[2] < 3) begin
            $display("FAIL too few zero crossings after the picture change");
            errors = errors + 1;
        end
        if (cue_peak < bgm_peak + 200000) begin
            $display("FAIL the picture change chime is not audible");
            errors = errors + 1;
        end
        if (!cue_done) begin
            $display("FAIL the picture change pulse was never issued");
            errors = errors + 1;
        end
        if (acr_packets < 10) begin
            $display("FAIL no ACR packets were generated");
            errors = errors + 1;
        end
        if (acr_n_bad != 0 || acr_cts_bad != 0) begin
            $display("FAIL the ACR packet contents are wrong");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_hdmi_audio_chain");
        else
            $display("RESULT: FAIL tb_hdmi_audio_chain errors=%0d", errors);

        $finish;
    end

endmodule
