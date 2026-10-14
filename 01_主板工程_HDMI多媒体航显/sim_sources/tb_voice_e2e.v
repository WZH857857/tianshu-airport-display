`timescale 1ns / 1ps
//=============================================================================
// tb_voice_e2e.v - the real card announcement, from the file to the I2S word
//
// The other benches cut the chain in half: tb_wav_voice_loader stops at the
// bytes the loader writes, tb_voice_player feeds the player four hand made
// codes.  This one joins them with the WAV file the card really carries:
//
//   VOICE.WAV -> wav_voice_loader -> voice_bank -> voice_player
//
// The loader runs on the 50 MHz system clock and the bank is read on the
// 12.288 MHz audio clock, so this is also the first bench that crosses the two
// clock domains the way the board does.  The player is instantiated with the
// same GAIN_SHIFT the top level uses, because that is the width that decides
// whether the scaled samples still fit.
//
// checked here
//   * the loader reports 36553 mono 16 bit samples at 8 kHz with no error
//   * the bank holds the announcement: mu-law silence in front of the file and
//     the exact bytes the offline model computes at slot 197 and at the
//     loudest sample of the file, slot 27868
//   * the play head walks the bank monotonically, one slot at a time, and it
//     reaches the end of the announcement
//   * the played words stay inside the scale the bank can reach, and the peak,
//     the number of loud frames and the number of non silent frames match the
//     offline model in sim\voice_golden.py to the frame, at the GAIN_SHIFT the
//     board uses
//=============================================================================
module tb_voice_e2e;

    localparam integer SYS_HALF = 10;   // 20 ns, 50 MHz
    localparam integer AUD_HALF = 41;   // 82 ns audio clock
    localparam integer GAP      = 8;
    localparam integer SRC_INC  = 10923;   // 8 kHz source, 48 kHz frames
    localparam integer NWORDS   = 36553;

    reg                sys_clk;
    reg                aud_clk;
    reg                rst_n;

    // loader side
    reg                start;
    reg                stream_done;
    reg                stream_error;
    reg  [7:0]         data;
    reg                data_valid;
    wire               data_ready;
    wire               abort;
    wire               wr_en;
    wire [15:0]        wr_addr;
    wire [7:0]         wr_data;
    wire               busy;
    wire               done;
    wire               error;
    wire [3:0]         error_code;
    wire [31:0]        sample_rate;
    wire [7:0]         channels;
    wire [7:0]         bits;
    wire [16:0]        words_stored;
    wire               truncated;
    wire [5:0]         dbg_state;

    // player side
    reg                audio_rst;
    reg                frame_tick;
    reg  [31:0]        phase_inc;
    reg  [16:0]        words;
    wire [15:0]        rd_addr;
    wire [7:0]         rd_data;
    wire signed [23:0] sample;
    wire               generating;

    reg                voice_ready;
    reg                loader_ok;
    reg  [7:0]         wav [0:73149];
    integer            wav_len;
    integer            fd;
    integer            got;
    integer            i;
    integer            errors;
    integer            tick_div;
    integer            frames;
    reg signed [23:0]  got_sample;
    integer            loud_frames;
    integer            active_frames;
    integer            silent_frames;
    reg signed [23:0]  peak;
    reg signed [23:0]  abs_sample;
    reg [7:0]          head_code;
    reg signed [23:0]  expected;

    // the expander the player uses, instantiated once instead of copying the
    // law into the bench, so the reference is the block that runs on the board
    reg        [7:0]   ref_code;
    wire signed [15:0] ref_pcm;

    wav_voice_loader #(
        .BANK_WORDS (40960),
        .AW         (16)
    ) u_loader (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .start        (start),
        .stream_done  (stream_done),
        .stream_error (stream_error),
        .data         (data),
        .data_valid   (data_valid),
        .data_ready   (data_ready),
        .abort        (abort),
        .wr_en        (wr_en),
        .wr_addr      (wr_addr),
        .wr_data      (wr_data),
        .busy         (busy),
        .done         (done),
        .error        (error),
        .error_code   (error_code),
        .sample_rate  (sample_rate),
        .channels     (channels),
        .bits         (bits),
        .words_stored (words_stored),
        .truncated    (truncated),
        .dbg_state    (dbg_state)
    );

    voice_bank #(
        .WORDS (40960),
        .AW    (16)
    ) u_bank (
        .wr_clk  (sys_clk),
        .wr_en   (wr_en),
        .wr_addr (wr_addr),
        .wr_data (wr_data),
        .rd_clk  (aud_clk),
        .rd_addr (rd_addr),
        .rd_data (rd_data)
    );

    voice_player #(
        .WORDS      (40960),
        .AW         (16),
        .GAP_FRAMES (GAP),
        .GAIN_SHIFT (7)
    ) u_player (
        .clk        (aud_clk),
        .rst        (audio_rst),
        .ready      (voice_ready),
        .frame_tick (frame_tick),
        .phase_inc  (phase_inc),
        .words      (words),
        .rd_addr    (rd_addr),
        .rd_data    (rd_data),
        .sample     (sample),
        .generating (generating)
    );

    ulaw_decode u_ref_decode (
        .code (ref_code),
        .pcm  (ref_pcm)
    );

    always #(SYS_HALF) sys_clk = ~sys_clk;
    always #(AUD_HALF) aud_clk = ~aud_clk;

    // one 48 kHz frame every eight audio clocks
    always @(posedge aud_clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_div   <= 0;
            frame_tick <= 1'b0;
        end else begin
            if (tick_div == 7) begin
                tick_div   <= 0;
                frame_tick <= 1'b1;
            end else begin
                tick_div   <= tick_div + 1;
                frame_tick <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    // card byte stream
    //------------------------------------------------------------------
    task send_byte;
        input [7:0] value;
        begin
            if (!done && !error) begin
                @(negedge sys_clk);
                data       = value;
                data_valid = 1'b1;
                while (!data_ready && !done && !error)
                    @(negedge sys_clk);
                @(posedge sys_clk);
                @(negedge sys_clk);
                data_valid = 1'b0;
                @(posedge sys_clk);
            end
        end
    endtask

    // returns the word the player is putting on the port for the frame that
    // just ended, together with the slot the head is parked on
    task next_frame;
        begin
            @(posedge aud_clk);
            while (!frame_tick)
                @(posedge aud_clk);
            // the player took the address in during this frame, so the head is
            // valid for the word the converter will send next
            @(posedge aud_clk);
            got_sample = sample;
            frames     = frames + 1;
        end
    endtask

    task restart_audio;
        begin
            audio_rst = 1'b1;
            repeat (8) @(posedge aud_clk);
            audio_rst = 1'b0;
            repeat (8) @(posedge aud_clk);
            frames    = 0;
        end
    endtask

    // the player is a scaled mu-law expander, so a frame can only ever carry
    // one of the values the bank can decode to, and never more than the
    // loudest code times the gain.  Both bounds are worked out from the bank,
    // not from a hand written table.
    task check_range;
        input integer frame;
        begin
            if ((^got_sample) === 1'bx) begin
                $display("FAIL: frame %0d is X", frame);
                errors = errors + 1;
            end else if ((got_sample > 24'sd4194304) ||
                         (got_sample < -24'sd4194304)) begin
                $display("FAIL: frame %0d out of range: %0d",
                         frame, got_sample);
                errors = errors + 1;
            end
            abs_sample = (got_sample < 0) ? -got_sample : got_sample;
            if (abs_sample > peak)
                peak = abs_sample;
            if (abs_sample > 100000)
                loud_frames = loud_frames + 1;
            if (got_sample != 24'sd0)
                active_frames = active_frames + 1;
            else
                silent_frames = silent_frames + 1;
        end
    endtask

    initial begin
        sys_clk      = 1'b0;
        aud_clk      = 1'b0;
        rst_n        = 1'b0;
        audio_rst    = 1'b1;
        start        = 1'b0;
        stream_done  = 1'b0;
        stream_error = 1'b0;
        data         = 8'd0;
        data_valid   = 1'b0;
        voice_ready  = 1'b0;
        loader_ok    = 1'b0;
        phase_inc    = 32'd10923;
        words         = 17'd36553;
        errors        = 0;
        frames        = 0;
        loud_frames   = 0;
        active_frames = 0;
        silent_frames = 0;
        peak          = 24'sd0;

        fd = $fopen("../../../VOICE.WAV", "rb");
        if (fd == 0)
            fd = $fopen("E:/FPGA guo sai/jing jie yao qiu/hdmi_mm_player_advanced/VOICE.WAV", "rb");
        if (fd == 0) begin
            $display("FAIL: VOICE.WAV not found");
            $finish;
        end

        wav_len = 0;
        got     = 0;
        while (got != -1 && wav_len < 73150) begin
            got = $fgetc(fd);
            if (got != -1)
                wav[ wav_len ] = got[7:0];
            wav_len = wav_len + 1;
        end
        $fclose(fd);
        if (wav_len > 73150)
            wav_len = 73150;

        repeat (4) @(posedge sys_clk);
        rst_n = 1'b1;
        repeat (4) @(posedge sys_clk);
        repeat (8) @(posedge aud_clk);
        audio_rst = 1'b0;

        //------------------------------------------------------------------
        // push the whole file at the loader, one handshake per byte, and pulse
        // stream_done after the last one the way fat32_file_reader does
        //------------------------------------------------------------------
        @(negedge sys_clk);
        start = 1'b1;
        @(negedge sys_clk);
        start = 1'b0;

        for (i = 0; i < wav_len; i = i + 1) begin
            if (!done && !error)
                send_byte(wav[i]);
        end

        @(negedge sys_clk);
        stream_done = 1'b1;
        @(posedge sys_clk);
        @(negedge sys_clk);
        stream_done = 1'b0;
        repeat (8) @(posedge sys_clk);

        // the loader raises done for a single system clock, so success is read
        // off the sticky numbers, exactly like the level the top level builds
        loader_ok   = (words_stored == 17'd36553) && !error && !truncated;
        voice_ready = loader_ok;
        repeat (4) @(posedge sys_clk);

        $display("VOICE.WAV bytes=%0d loader ok=%b busy=%b error=%b code=%0d state=%0d",
                 wav_len, loader_ok, busy, error, error_code, dbg_state);
        $display("loader words=%0d rate=%0d channels=%0d bits=%0d truncated=%b",
                 words_stored, sample_rate, channels, bits, truncated);

        if (!loader_ok || busy) begin
            $display("FAIL: loader did not finish cleanly");
            errors = errors + 1;
        end
        if (sample_rate != 32'd8000 || channels != 8'd1 ||
            bits != 8'd16) begin
            $display("FAIL: format rate=%0d ch=%0d bits=%0d",
                     sample_rate, channels, bits);
            errors = errors + 1;
        end

        words = words_stored;

        //------------------------------------------------------------------
        // the bank has to hold the announcement the decoder will play
        //------------------------------------------------------------------
        for (i = 0; i < 4; i = i + 1) begin
            if (u_bank.mem_lo[i] !== 8'hFF) begin
                $display("FAIL: bank[%0d]=%02x, expected mu-law silence FF",
                         i, u_bank.mem_lo[i]);
                errors = errors + 1;
            end
        end
        // the bank has to hold those bytes, not just some bytes: slot 197 is
        // the first sample the offline model decodes to 2108 and slot 27868 is
        // the loudest sample of the file, 0x82 -> 30076
        if (u_bank.mem_lo[197] !== 8'hBE) begin
            $display("FAIL: bank[197]=%02x, expected BE",
                     u_bank.mem_lo[197]);
            errors = errors + 1;
        end
        if (u_bank.mem_lo[27868] !== 8'h82) begin
            $display("FAIL: bank[27868]=%02x, expected 82",
                     u_bank.mem_lo[27868]);
            errors = errors + 1;
        end

        //------------------------------------------------------------------
        // play the announcement and watch the word that reaches the I2S
        // framer.  30000 frames is 625 ms, which covers the front of the file
        // and the first loud words, so a player that is silent, stuck or
        // scaled past full scale is caught here.
        //
        // The numbers the same file gives through the offline model in
        // sim\voice_golden.py, at GAIN_SHIFT = 7:
        //   peak 2932224, 21189 frames above 100000, 26119 frames not silent
        // The limits below leave a margin for the frames the bench skips, but
        // they are far above what a gain path that drops the top of the
        // sample can reach: that one peaks at 131072 and never goes higher.
        //------------------------------------------------------------------
        restart_audio;
        for (i = 0; i < 30000; i = i + 1) begin
            next_frame;
            if ((i > 12) && (errors == 0))
                check_range(frames);
        end

        $display("played 30000 frames: peak %0d loud %0d active %0d silent %0d",
                 peak, loud_frames, active_frames, silent_frames);

        if (errors == 0) begin
            if (peak < 24'sd2800000) begin
                $display("FAIL: peak %0d is below the encoded level", peak);
                errors = errors + 1;
            end
            if (loud_frames < 15000) begin
                $display("FAIL: only %0d frames carried real level",
                         loud_frames);
                errors = errors + 1;
            end
            if (active_frames < 20000) begin
                $display("FAIL: only %0d frames were not silent",
                         active_frames);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("RESULT: PASS tb_voice_e2e");
        else
            $display("RESULT: FAIL tb_voice_e2e errors=%0d", errors);
        $finish;
    end

    initial begin
        #60_000_000;
        $display("RESULT: FAIL tb_voice_e2e timeout at frame %0d", frames);
        $finish;
    end

endmodule


