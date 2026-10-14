`timescale 1ps / 1ps
//=============================================================================
// tb_voice_stream_e2e.v - parser + asynchronous FIFO + streaming player
//
// The three blocks that make up the card voice path are wired together
// exactly as the top level wires them, the real VOICE.WAV is pushed through
// the card-side handshake, and every 48 kHz frame the player produces is
// written to a text file.  That file is what the HDMI audio island would send
// to the monitor, so it can be turned back into a WAV and listened to.
//
// Usage: vvp build_voice_stream_e2e.vvp +frames=150000 +out=voice_out.txt
//=============================================================================
module tb_voice_stream_e2e;

    localparam integer SYS_HALF = 10_000;      // 50 MHz
    localparam integer AUD_HALF = 40_690;      // 12.288 MHz
    localparam integer FRAME_DIV = 256;        // 12.288 MHz / 48 kHz
    localparam [31:0] RATE_8K_INC = 32'd10923; // 16.16 increment for 8 kHz

    reg sys_clk = 1'b0;
    reg aud_clk = 1'b0;
    always #SYS_HALF sys_clk = ~sys_clk;
    always #AUD_HALF aud_clk = ~aud_clk;

    //------------------------------------------------------------------
    // card side
    //------------------------------------------------------------------
    reg         sys_rst_n = 1'b0;
    reg         start = 1'b0;
    reg         stream_done = 1'b0;
    reg         stream_error = 1'b0;
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
        .start        (start),
        .stream_done  (stream_done),
        .stream_error (stream_error),
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
    // audio side
    //------------------------------------------------------------------
    reg  [7:0]  frame_cnt = 8'd0;
    reg         frame_tick = 1'b0;
    reg         frame_tick_q = 1'b0;
    reg         start_tog = 1'b0;
    reg         done_tog = 1'b0;
    reg  [2:0]  done_sync = 3'd0;
    wire signed [23:0] voice_sample;
    wire        voice_generating;

    always @(posedge aud_clk) begin
        frame_tick <= (frame_cnt == FRAME_DIV - 1);
        if (frame_cnt == FRAME_DIV - 1)
            frame_cnt <= 8'd0;
        else
            frame_cnt <= frame_cnt + 8'd1;
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

    //------------------------------------------------------------------
    // capture and stimulus
    //------------------------------------------------------------------
    integer out_fd;
    integer in_fd;
    integer got;
    integer frames;
    integer frame_limit;
    integer byte_count;
    integer underruns;
    reg     generating_q;
    reg     underrun_q;

    initial begin
        if (!$value$plusargs("frames=%d", frame_limit))
            frame_limit = 150000;
        out_fd = $fopen("build/voice_stream_out.txt", "w");
        if (out_fd == 0) begin
            $display("FAIL: cannot open output file");
            $finish;
        end

        frames    = 0;
        underruns = 0;
        byte_count = 0;

        in_fd = $fopen("E:/FPGA guo sai/jing jie yao qiu/hdmi_mm_player_advanced/VOICE.WAV", "rb");
        if (in_fd == 0) begin
            $display("FAIL: VOICE.WAV not found");
            $finish;
        end

        repeat (8) @(posedge sys_clk);
        sys_rst_n = 1'b1;
        repeat (8) @(posedge sys_clk);

        @(negedge sys_clk);
        start     = 1'b1;
        start_tog = 1'b1;
        @(negedge sys_clk);
        start     = 1'b0;

        // Push the real card WAV through the same handshake the FAT32 reader
        // uses.  The parser back-pressures through sample_ready, so this also
        // exercises the FIFO full path.
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

        @(negedge sys_clk);
        stream_done = 1'b1;
        @(negedge sys_clk);
        stream_done = 1'b0;

        $display("pushed %0d bytes through the card handshake", byte_count);
    end

    // Record the frame the player hands to the audio island.  sample is
    // registered, so it is captured on the frame after the tick.
    always @(posedge aud_clk) begin
        frame_tick_q  <= frame_tick;
        generating_q  <= voice_generating;
        underrun_q    <= voice_generating && (fifo_rd_level == 11'd0) &&
                         (fifo_empty);
        done_sync     <= {done_sync[1:0], done_tog};

        if (frame_tick_q) begin
            $fwrite(out_fd, "%0d %0d\n", voice_generating, voice_sample);
            frames = frames + 1;
            if (voice_generating && fifo_empty && (fifo_rd_level == 11'd0))
                underruns = underruns + 1;
            if (frames >= frame_limit) begin
                $display("captured %0d frames, %0d frames with an empty FIFO",
                         frames, underruns);
                $display("parser: samples=%0d rate=%0d bits=%0d ch=%0d done=%b err=%b code=%0d",
                         parser_samples, parser_rate, parser_bits,
                         parser_channels, parser_done, parser_error,
                         parser_error_code);
                $fclose(out_fd);
                $display("RESULT: DONE tb_voice_stream_e2e");
                $finish;
            end
        end
    end

    // done_tog is raised once the parser reports the end of the data chunk.
    always @(posedge sys_clk) begin
        if (!sys_rst_n)
            done_tog <= 1'b0;
        else if (parser_done)
            done_tog <= 1'b1;
    end

endmodule
