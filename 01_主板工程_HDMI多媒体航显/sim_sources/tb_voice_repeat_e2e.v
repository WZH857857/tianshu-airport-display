`timescale 1ps / 1ps
//=============================================================================
// tb_voice_repeat_e2e.v - two complete announcement rounds
//
// The repeat controller, the card WAV parser, the asynchronous FIFO and the
// 48 kHz player are wired exactly as the top level wires them.  A short
// 8 kHz file is pushed once per round and the test checks the properties the
// board relies on:
//
//   * the first round starts as soon as the cache is ready
//   * the first round drains completely before the repeat
//   * the repeat waits for the configured gap
//   * the second round restarts the parser from sample 0 and produces audio
//
// The gap is shortened to twelve 10 ms ticks so the simulation stays short;
// the board default is sixty one-second ticks.
//=============================================================================
module tb_voice_repeat_e2e;

    localparam integer SYS_HALF = 10_000;      // 50 MHz
    localparam integer AUD_HALF = 40_690;      // 12.288 MHz
    localparam integer FRAME_DIV = 256;        // 12.288 MHz / 48 kHz
    localparam [31:0] RATE_8K_INC = 32'd10923; // 16.16 increment for 8 kHz

    localparam [25:0] GAP_TICK = 26'd500_000;  // 10 ms at 50 MHz
    localparam [7:0]  GAP_SECS = 8'd12;        // twelve ticks -> 120 ms
    localparam integer EXPECT_GAP_CYCLES = 6_000_000;
    localparam integer GAP_SLACK_CYCLES = 50_000;
    localparam integer DRAIN_LIMIT_CYCLES = 4_000_000;

    reg sys_clk = 1'b0;
    reg aud_clk = 1'b0;
    always #SYS_HALF sys_clk = ~sys_clk;
    always #AUD_HALF aud_clk = ~aud_clk;

    integer errors = 0;
    // $time is in ps here, so one 50 MHz cycle is 20,000 ps.
    integer pulse_cycle [0:1];
    integer round_bytes [0:1];

    task fail_now;
        input [8*60-1:0] text;
        begin
            errors = errors + 1;
            $display("FAIL: %0s", text);
        end
    endtask

    //------------------------------------------------------------------
    // repeat scheduler
    //------------------------------------------------------------------
    reg  sys_rst_n = 1'b0;
    reg  voice_ready = 1'b0;
    wire voice_start_pulse;
    wire voice_waiting;

    voice_repeat_ctrl #(
        .GAP_TICK_CYCLES (GAP_TICK)
    ) u_repeat (
        .clk          (sys_clk),
        .rst_n        (sys_rst_n),
        .gap_seconds  (GAP_SECS),
        .voice_ready  (voice_ready),
        .stream_done  (parser_done),
        .stream_error (parser_error),
        .start_pulse  (voice_start_pulse),
        .waiting      (voice_waiting)
    );

    //------------------------------------------------------------------
    // card side: parser, byte handshake and compressed sample stream
    //------------------------------------------------------------------
    reg  [7:0]  card_data = 8'd0;
    reg         card_valid = 1'b0;
    wire        card_ready;
    wire        parser_abort;
    wire        sample_valid;
    wire [7:0]  sample_data;
    wire        sample_ready;
    wire        parser_busy, parser_done, parser_error;
    wire [3:0]  parser_error_code;
    wire [31:0] parser_rate, parser_samples;
    wire [7:0]  parser_channels, parser_bits;
    wire [5:0]  parser_dbg;

    wav_voice_stream u_parser (
        .clk          (sys_clk),
        .rst_n        (sys_rst_n),
        .start        (voice_start_pulse),
        .stream_done  (1'b0),
        .stream_error (1'b0),
        .data         (card_data),
        .data_valid   (card_valid),
        .data_ready   (card_ready),
        .abort        (parser_abort),
        .sample_valid (sample_valid),
        .sample_data  (sample_data),
        .sample_ready (sample_ready),
        .busy         (parser_busy),
        .done         (parser_done),
        .error        (parser_error),
        .error_code   (parser_error_code),
        .sample_rate  (parser_rate),
        .channels     (parser_channels),
        .bits         (parser_bits),
        .samples_out  (parser_samples),
        .dbg_state    (parser_dbg)
    );

    wire        fifo_full, fifo_empty;
    wire [10:0] fifo_wr_level, fifo_rd_level;
    wire [7:0]  fifo_rd_data;
    wire [7:0]  fifo_wr_data;
    wire        fifo_wr_en;
    wire        fifo_rd_en;

    assign sample_ready = ~fifo_full;
    assign fifo_wr_data = sample_data;
    assign fifo_wr_en   = sample_valid && ~fifo_full;

    async_fifo #(
        .DW (8),
        .AW (10)
    ) u_fifo (
        .wr_clk   (sys_clk),
        .wr_rst_n (sys_rst_n),
        .wr_en    (fifo_wr_en),
        .wr_data  (fifo_wr_data),
        .wr_full  (fifo_full),
        .wr_level (fifo_wr_level),
        .rd_clk   (aud_clk),
        .rd_rst_n (sys_rst_n),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_rd_data),
        .rd_empty (fifo_empty),
        .rd_level (fifo_rd_level)
    );

    //------------------------------------------------------------------
    // audio side: exactly the top level wiring
    //------------------------------------------------------------------
    reg  [7:0] frame_cnt = 8'd0;
    reg        frame_tick = 1'b0;
    reg        frame_tick_q = 1'b0;
    reg        start_tog = 1'b0;
    reg        done_tog = 1'b0;
    wire signed [23:0] voice_sample;
    wire        voice_generating;

    integer start_events = 0;
    integer audio_frames = 0;
    reg     was_generating = 1'b0;

    always @(posedge aud_clk) begin
        frame_tick <= (frame_cnt == FRAME_DIV - 1);
        if (frame_cnt == FRAME_DIV - 1)
            frame_cnt <= 8'd0;
        else
            frame_cnt <= frame_cnt + 8'd1;

        frame_tick_q <= frame_tick;
        was_generating <= voice_generating;
        if (voice_generating && !was_generating)
            start_events = start_events + 1;
        if (frame_tick_q && voice_generating && (voice_sample != 24'sd0))
            audio_frames = audio_frames + 1;
    end

    voice_player_stream #(
        .GAIN_SHIFT (7),
        .DRAIN_HOLD (3)
    ) u_player (
        .clk        (aud_clk),
        .rst        (~sys_rst_n),
        .start_tog  (start_tog),
        .done_tog   (done_tog),
        .frame_tick (frame_tick),
        .phase_inc  (RATE_8K_INC),
        .fifo_data  (fifo_rd_data),
        .fifo_empty (fifo_empty),
        .fifo_rd_en (fifo_rd_en),
        .sample     (voice_sample),
        .generating (voice_generating)
    );

    // The top level toggles start_tog and done_tog from the system clock
    // domain; keep the same two flip-flops here.
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            start_tog <= 1'b0;
            done_tog  <= 1'b0;
        end else begin
            if (voice_start_pulse)
                start_tog <= ~start_tog;
            if (parser_done)
                done_tog <= ~done_tog;
        end
    end

    //------------------------------------------------------------------
    // stimulus helpers
    //------------------------------------------------------------------
    integer in_fd;
    integer got;
    integer byte_count;
    reg [8*160-1:0] wav_path;

    // Progress trace.  A behavioural two-round simulation is long enough that
    // a silent hang is hard to tell apart from ordinary run time.
    reg trace_on = 1'b0;
    integer trace_window = 0;
    always @(posedge sys_clk) begin
        if (trace_on) begin
            trace_window = trace_window + 1;
            if (trace_window == 200_000) begin
                trace_window = 0;
                $display("t=%0t cycles=%0d wait=%b gen=%b fifo_empty=%b parser=%0d samples=%0d",
                         $time, $time / 20_000, voice_waiting, voice_generating,
                         fifo_empty, parser_dbg, parser_samples);
                $fflush;
            end
        end
    end

    task open_round;
        output ok;
        begin
            in_fd = $fopen(wav_path, "rb");
            if (in_fd == 0) begin
                fail_now("cannot open the short WAV file");
                ok = 1'b0;
            end else begin
                ok = 1'b1;
            end
            byte_count = 0;
        end
    endtask

    // Push one whole file through the same byte handshake the FAT32 reader
    // uses.  card_valid is held until the parser has taken the byte, so the
    // FIFO full back pressure is exercised without the byte being lost.
    task push_round;
        begin
            got = $fgetc(in_fd);
            while (got != -1) begin
                @(negedge sys_clk);
                card_data  = got[7:0];
                card_valid = 1'b1;
                @(posedge sys_clk);
                while (!card_ready)
                    @(posedge sys_clk);
                @(negedge sys_clk);
                card_valid = 1'b0;
                byte_count = byte_count + 1;
                got = $fgetc(in_fd);
            end
            $fclose(in_fd);
        end
    endtask

    // Wait for the next start pulse and remember the system cycle it landed
    // on.  This must be called before the pulse is issued, so the level is
    // sampled first and the wait is a true edge wait.
    task wait_start_pulse;
        input integer slot;
        begin
            if (voice_start_pulse)
                fail_now("start pulse was already high when it was awaited");
            @(posedge voice_start_pulse);
            pulse_cycle[slot] = $time / 20_000;
        end
    endtask

    // Wait until the player has drained the FIFO and released the chime.
    // The shortened gap has to be long enough for this to complete before the
    // next round starts.
    time idle_time [0:1];

    task wait_player_idle;
        input integer slot;
        input integer limit_cycles;
        time start_time;
        begin
            idle_time[slot] = 0;
            start_time = $time;
            if (!voice_generating)
                fail_now("player was already idle when the drain was awaited");
            while (voice_generating) begin
                @(posedge sys_clk);
                if ((($time - start_time) / 20_000) > limit_cycles) begin
                    fail_now("player did not drain before the repeat");
                    disable wait_player_idle;
                end
            end
            idle_time[slot] = $time;
        end
    endtask

    //------------------------------------------------------------------
    // test sequence
    //------------------------------------------------------------------
    integer gap_cycles;
    reg     wav_ok;

    initial begin
        wav_path = "sim/adv09_short.wav";
        idle_time[0] = 0;
        idle_time[1] = 0;
        if ($test$plusargs("trace"))
            trace_on = 1'b1;

        repeat (8) @(posedge sys_clk);
        sys_rst_n = 1'b1;
        if (trace_on)
            $display("CKPT reset released cycles=%0d", $time / 20_000);

        // Arm the scheduler only after the first round has been awaited, so
        // the opening pulse cannot slip past the test.
        fork
            wait_start_pulse(0);
            begin
                @(negedge sys_clk);
                voice_ready = 1'b1;
            end
        join
        if (trace_on)
            $display("CKPT first start cycles=%0d", pulse_cycle[0]);

        open_round(wav_ok);
        if (!wav_ok) begin
            $display("VOICE REPEAT E2E: FAIL, missing test WAV");
            $finish;
        end
        push_round;
        round_bytes[0] = byte_count;
        $display("round 0 pushed %0d bytes, start cycle %0d, done cycle %0d",
                 byte_count, pulse_cycle[0], $time / 20_000);

        // The audio side needs a few frames to pick the start toggle up and
        // to fill the interpolation queue before the drain can be observed.
        repeat (4000) @(posedge aud_clk);
        if (!voice_generating)
            fail_now("the player never started on the first round");
        wait_player_idle(0, DRAIN_LIMIT_CYCLES);
        $display("round 0 drained at cycle %0d", idle_time[0] / 20_000);

        // The second round is pushed through the same handshake as the first.
        wait_start_pulse(1);
        if (trace_on)
            $display("CKPT second start cycles=%0d", pulse_cycle[1]);

        open_round(wav_ok);
        if (!wav_ok) begin
            $display("VOICE REPEAT E2E: FAIL, missing test WAV");
            $finish;
        end
        push_round;
        round_bytes[1] = byte_count;
        $display("round 1 pushed %0d bytes, start cycle %0d, done cycle %0d",
                 byte_count, pulse_cycle[1], $time / 20_000);

        repeat (4000) @(posedge aud_clk);
        if (!voice_generating)
            fail_now("the player never started on the second round");
        wait_player_idle(1, DRAIN_LIMIT_CYCLES);
        $display("round 1 drained at cycle %0d", idle_time[1] / 20_000);

        // The two rounds are separated by the parse time plus the configured
        // gap, so only a lower bound and a small slack window are meaningful.
        gap_cycles = pulse_cycle[1] - pulse_cycle[0];
        if (gap_cycles < EXPECT_GAP_CYCLES)
            fail_now("the repeat started before the configured gap");
        if (gap_cycles > (EXPECT_GAP_CYCLES + GAP_SLACK_CYCLES))
            fail_now("the repeat gap was longer than the configured window");
        if ((idle_time[0] / 20_000) >= pulse_cycle[1])
            fail_now("the first round did not drain before the repeat");

        // Two complete parses and no parser error.
        if (parser_error)
            fail_now("parser reported an error");
        if (parser_error_code != 4'd0)
            fail_now("parser error code became non zero");
        if (parser_samples != 32'd200)
            fail_now("second round did not restart at sample 0");
        if (round_bytes[0] != 444 || round_bytes[1] != 444)
            fail_now("a round did not push the whole file");

        // Each round has to produce its own audio, not just one.
        if (start_events != 2)
            fail_now("the player did not start exactly twice");
        if (audio_frames < 100)
            fail_now("the player produced almost no audio frames");

        if (errors == 0)
            $display("VOICE REPEAT E2E: PASS");
        else
            $display("VOICE REPEAT E2E: FAIL with %0d error(s)", errors);
        $display("rounds: %0d bytes, %0d bytes; gap %0d cycles; %0d audio frames",
                 round_bytes[0], round_bytes[1], gap_cycles, audio_frames);
        $finish;
    end

endmodule
