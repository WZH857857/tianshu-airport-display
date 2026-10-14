`timescale 1ns / 1ps
//=============================================================================
// tb_voice_stream_phase.v - exact phase-accumulator regression
//
// This bench preloads a known mu-law stream, then drives exactly one
// frame_tick per 48 kHz output frame.  The expected sample is calculated from
// the mathematical source position, not from a copy of the RTL branch
// structure:
//
//     position = frame * phase_inc
//     index    = position >> 16
//     frac     = (position >> 8) & 0xff
//
// It catches the off-by-one phase error that made every wrap repeat a source
// sample and turned speech into a periodic buzzing sound.
//=============================================================================
module tb_voice_stream_phase;

    localparam integer SYS_HALF = 10;          // 50 MHz
    localparam integer AUD_HALF = 41;          // 12.288 MHz
    localparam integer N        = 1300;        // source codes preloaded

    reg         sys_clk = 1'b0;
    reg         aud_clk = 1'b0;
    reg         rst_n = 1'b0;
    reg         audio_rst = 1'b1;
    reg         wr_en = 1'b0;
    reg  [7:0]  wr_data = 8'd0;
    reg  [7:0]  codes [0:N-1];
    reg         start_tog = 1'b0;
    reg         done_tog = 1'b0;
    reg         frame_tick = 1'b0;
    reg         frame_enable = 1'b0;
    reg  [7:0]  frame_div = 8'd0;
    reg  [31:0] phase_inc = 32'd10923;

    wire        wr_full;
    wire [11:0] wr_level;
    wire        rd_en;
    wire [7:0]  rd_data;
    wire        rd_empty;
    wire [11:0] rd_level;
    wire signed [23:0] sample;
    wire        generating;

    integer     i;
    integer     frame_count;
    integer     frame_limit;
    integer     errors;
    integer     position;
    integer     src_index;
    integer     frac;
    integer     prev_dec;
    integer     next_dec;
    integer     expected;
    integer     got;

    async_fifo #(
        .DW (8),
        .AW (11)
    ) u_fifo (
        .wr_clk   (sys_clk),
        .wr_rst_n (rst_n),
        .wr_en    (wr_en),
        .wr_data  (wr_data),
        .wr_full  (wr_full),
        .wr_level (wr_level),
        .rd_clk   (aud_clk),
        .rd_rst_n (rst_n),
        .rd_en    (rd_en),
        .rd_data  (rd_data),
        .rd_empty (rd_empty),
        .rd_level (rd_level)
    );

    voice_player_stream #(
        .GAIN_SHIFT (7)
    ) u_player (
        .clk        (aud_clk),
        .rst        (audio_rst),
        .start_tog  (start_tog),
        .done_tog   (done_tog),
        .frame_tick (frame_tick),
        .phase_inc  (phase_inc),
        .fifo_data  (rd_data),
        .fifo_empty (rd_empty),
        .fifo_rd_en (rd_en),
        .sample     (sample),
        .generating (generating)
    );

    function integer decode_ulaw;
        input [7:0] code;
        integer u;
        integer sign;
        integer exp;
        integer mant;
        integer mag;
        begin
            u    = (~code) & 8'hFF;
            sign = (u >> 7) & 1;
            exp  = (u >> 4) & 7;
            mant = u & 15;
            mag  = (((mant << 3) + 132) << exp) - 132;
            decode_ulaw = sign ? -mag : mag;
        end
    endfunction

    always #SYS_HALF sys_clk = ~sys_clk;
    always #AUD_HALF aud_clk = ~aud_clk;

    // One tick every 256 audio clocks, the same spacing as the real
    // 12.288 MHz / 48 kHz HDMI audio frame.
    always @(posedge aud_clk) begin
        if (!frame_enable) begin
            frame_div  <= 8'd0;
            frame_tick <= 1'b0;
        end else begin
            frame_tick <= (frame_div == 8'd255);
            frame_div  <= (frame_div == 8'd255) ? 8'd0 : frame_div + 8'd1;
        end
    end

    task next_frame;
        begin
            @(negedge aud_clk);
            while (!frame_tick)
                @(negedge aud_clk);
            @(posedge aud_clk);
            #1;
            got = sample;
            frame_count = frame_count + 1;
        end
    endtask

    initial begin
        if (!$value$plusargs("frames=%d", frame_limit))
            frame_limit = 2000;
        if (!$value$plusargs("inc=%d", phase_inc))
            phase_inc = 32'd10923;

        errors = 0;
        frame_count = 0;

        // Include positive, negative, zero and full-scale values so a sign or
        // width error cannot hide in a quiet test pattern.
        for (i = 0; i < N; i = i + 1)
            codes[i] = (i * 37 + 19) & 8'hFF;
        codes[0] = 8'hCE;
        codes[1] = 8'hA0;
        codes[2] = 8'h80;
        codes[3] = 8'h00;
        codes[4] = 8'hFF;
        codes[5] = 8'h7F;

        repeat (4) @(posedge sys_clk);
        rst_n = 1'b1;
        repeat (4) @(posedge sys_clk);

        // Preload the compressed source sequence.  FIFO depth is larger than
        // the whole sequence, so the test measures interpolation only.
        for (i = 0; i < N; i = i + 1) begin
            @(negedge sys_clk);
            wr_data = codes[i];
            wr_en   = 1'b1;
            @(posedge sys_clk);
            @(negedge sys_clk);
            wr_en   = 1'b0;
        end

        // Start the audio side with the FIFO already non-empty.
        @(negedge sys_clk);
        audio_rst = 1'b0;
        repeat (8) @(posedge aud_clk);
        @(negedge sys_clk);
        start_tog = ~start_tog;

        while (!generating)
            @(posedge aud_clk);
        while (u_player.q_count != 2'd2)
            @(posedge aud_clk);
        @(negedge aud_clk);
        frame_enable = 1'b1;

        // The first frame is source sample zero.  Later frames use the exact
        // mathematical position of the phase accumulator.
        for (i = 0; i < frame_limit; i = i + 1) begin
            next_frame;

            position  = i * phase_inc;
            src_index = position >> 16;
            frac      = (position >> 8) & 255;

            if (src_index >= N) begin
                $display("FAIL: source index %0d beyond preload", src_index);
                errors = errors + 1;
            end else begin
                prev_dec = decode_ulaw(codes[src_index]);
                if (frac == 0)
                    expected = prev_dec;
                else if (src_index + 1 < N) begin
                    next_dec = decode_ulaw(codes[src_index + 1]);
                    expected = prev_dec +
                               (((next_dec - prev_dec) * frac) >>> 8);
                end else
                    expected = prev_dec;
                expected = expected * 128;

                if (got !== expected) begin
                    $display("FAIL frame %0d: got=%0d expected=%0d idx=%0d frac=%0d",
                             i, got, expected, src_index, frac);
                    errors = errors + 1;
                end
            end
        end

        if (errors == 0)
            $display("RESULT: PASS tb_voice_stream_phase inc=%0d frames=%0d",
                     phase_inc, frame_limit);
        else
            $display("RESULT: FAIL tb_voice_stream_phase inc=%0d errors=%0d",
                     phase_inc, errors);
        $finish;
    end

    initial begin
        #500_000_000;
        $display("RESULT: FAIL tb_voice_stream_phase timeout gen=%b q=%0d empty=%b rd_en=%b pending=%b capture=%b wr=%0d",
                 generating, u_player.q_count, rd_empty, rd_en,
                 u_player.rd_pending, u_player.rd_capture, wr_level);
        $finish;
    end

endmodule
