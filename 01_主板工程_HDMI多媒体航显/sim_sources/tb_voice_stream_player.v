`timescale 1ns / 1ps
//=============================================================================
// tb_voice_stream_player.v - registered-FIFO read path of the stream player
//
// A known mu-law sequence is preloaded into the asynchronous FIFO, then the
// player is run at one source sample per 48 kHz frame.  The bench checks the
// complete decoded sequence, which catches the classic async_fifo mistake of
// using rd_data in the same cycle as rd_en (the first sample is then lost or
// the sequence is shifted by one).
//=============================================================================
module tb_voice_stream_player;

    localparam integer N = 12;
    localparam integer SYS_HALF = 10;
    localparam integer AUD_HALF = 41;

    reg         sys_clk;
    reg         aud_clk;
    reg         rst_n;
    reg         audio_rst;
    reg         wr_en;
    reg  [7:0]  wr_data;
    reg  [7:0]  codes [0:N-1];
    reg         start_tog;
    reg         done_tog;
    reg         frame_tick;
    reg  [31:0] phase_inc;

    wire        wr_full;
    wire [8:0]  wr_level;
    wire        rd_en;
    wire [7:0]  rd_data;
    wire        rd_empty;
    wire [8:0]  rd_level;
    wire signed [23:0] sample;
    wire        generating;

    reg  [7:0]  ref_code;
    wire signed [15:0] ref_pcm;
    reg signed [23:0] expected;
    integer     errors;
    integer     i;
    integer     frame_count;
    integer     got_count;
    integer     tick_div;
    reg         seen_first;
    reg signed [23:0] got;

    async_fifo #(
        .DW (8),
        .AW (8)
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

    ulaw_decode u_ref_decode (
        .code (ref_code),
        .pcm  (ref_pcm)
    );

    always #(SYS_HALF) sys_clk = ~sys_clk;
    always #(AUD_HALF) aud_clk = ~aud_clk;

    // One 48 kHz frame every eight audio clocks in this bench.
    always @(posedge aud_clk or posedge audio_rst) begin
        if (audio_rst) begin
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

    task next_frame;
        begin
            @(posedge aud_clk);
            while (!frame_tick)
                @(posedge aud_clk);
            @(posedge aud_clk);
            got = sample;
            frame_count = frame_count + 1;
        end
    endtask

    initial begin
        sys_clk    = 1'b0;
        aud_clk    = 1'b0;
        rst_n      = 1'b0;
        audio_rst  = 1'b1;
        wr_en      = 1'b0;
        wr_data    = 8'd0;
        start_tog  = 1'b0;
        done_tog   = 1'b0;
        frame_tick = 1'b0;
        phase_inc  = 32'd65536;   // one source sample per 48 kHz frame
        errors     = 0;
        frame_count = 0;
        got_count   = 0;
        seen_first  = 1'b0;

        // Include positive, negative and zero values.  0x80/0x00 are the
        // loudest positive/negative codes and exercise the full gain width.
        codes[0]  = 8'hCE;
        codes[1]  = 8'hA0;
        codes[2]  = 8'h80;
        codes[3]  = 8'h00;
        codes[4]  = 8'hFF;
        codes[5]  = 8'h7F;
        codes[6]  = 8'hC0;
        codes[7]  = 8'h40;
        codes[8]  = 8'h90;
        codes[9]  = 8'h70;
        codes[10] = 8'hE0;
        codes[11] = 8'h20;

        repeat (4) @(posedge sys_clk);
        rst_n = 1'b1;
        repeat (4) @(posedge sys_clk);

        // preload the whole sequence before the audio side starts
        for (i = 0; i < N; i = i + 1) begin
            @(negedge sys_clk);
            wr_data = codes[i];
            wr_en   = 1'b1;
            @(posedge sys_clk);
            @(negedge sys_clk);
            wr_en   = 1'b0;
        end

        repeat (4) @(posedge sys_clk);
        @(negedge sys_clk);
        start_tog = ~start_tog;
        @(posedge sys_clk);

        repeat (4) @(posedge aud_clk);
        audio_rst = 1'b0;
        repeat (4) @(posedge aud_clk);

        // Make sure the start event crosses first, then tell the player that
        // the producer has finished.  The FIFO still contains the sequence.
        while (!generating)
            @(posedge aud_clk);
        @(negedge sys_clk);
        done_tog = ~done_tog;

        while ((got_count < N) && (frame_count < (N * 8))) begin
            next_frame;
            if (generating && !seen_first && (got != 24'sd0))
                seen_first = 1'b1;

            if (seen_first) begin
                ref_code = codes[got_count];
                #1;
                expected = ref_pcm * 128;
                if (got !== expected) begin
                    $display("FAIL: sample %0d got %0d expected %0d",
                             got_count, got, expected);
                    errors = errors + 1;
                end
                got_count = got_count + 1;
            end
        end

        if (got_count != N) begin
            $display("FAIL: only %0d samples observed", got_count);
            errors = errors + 1;
        end

        // After the FIFO drains, the player must release the chime path.
        for (i = 0; i < 8; i = i + 1)
            next_frame;
        if (generating !== 1'b0) begin
            $display("FAIL: generating did not fall after the stream drained");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_voice_stream_player");
        else
            $display("RESULT: FAIL tb_voice_stream_player errors=%0d", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL tb_voice_stream_player timeout");
        $finish;
    end

endmodule
