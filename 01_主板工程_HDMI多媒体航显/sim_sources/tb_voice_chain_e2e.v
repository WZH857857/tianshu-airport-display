`timescale 1ps / 1ps
//=============================================================================
// tb_voice_chain_e2e.v - fat32_file_reader -> wav_voice_stream -> async FIFO
//                        -> voice_player_stream
//
// The three blocks that make up the card voice path are wired exactly the way
// hdmi_mm_player wires them, but the picture cache is replaced by a simple
// "the cache is ready now" flag so the bench stays short.  A synthetic FAT32
// card carries one BMP and one small 8 kHz / 16 bit mono WAV; the WAV data is
// a sawtooth with a healthy amplitude so a silent result is unambiguous.
//
// Checks:
//   1. the scan still lists the BMP and remembers the WAV entry
//   2. a single cycle load_start (what voice_start_pulse produces) is enough
//      to make the reader stream the WAV
//   3. the parser reports the expected sample count and the header format
//   4. the audio side really hands non-zero samples to the chime mixer and
//      releases the chime again when the stream ends
//=============================================================================
module tb_voice_chain_e2e;

    localparam integer SYS_HALF = 10_000;      // 50 MHz
    localparam integer AUD_HALF = 40_690;      // 12.288 MHz
    localparam integer FRAME_DIV = 256;        // 12.288 MHz / 48 kHz
    localparam [31:0] RATE_8K_INC = 32'd10923; // 16.16 increment for 8 kHz

    // 256 samples at 8 kHz is 32 ms of audio.  That is long enough for the
    // player to produce well over the 200 frames the checks below ask for,
    // and short enough that the bench finishes in a couple of minutes; the
    // byte path, the FIFO back pressure and the chime hand-back are the same
    // at any length.
    localparam integer WAV_SAMPLES = 256;
    localparam integer WAV_BYTES   = 44 + 2 * WAV_SAMPLES;

    reg sys_clk = 1'b0;
    reg aud_clk = 1'b0;
    always #SYS_HALF sys_clk = ~sys_clk;
    always #AUD_HALF aud_clk = ~aud_clk;

    reg sys_rst_n = 1'b0;

    //------------------------------------------------------------------
    // synthetic FAT32 card
    //------------------------------------------------------------------
    reg  [7:0] card [0:8191];              // 16 sectors
    reg  [31:0] mock_lba;
    reg  [3:0]  mock_delay;
    reg         mock_busy;
    reg         sec_ready;
    reg         sec_error;
    integer     i;
    integer     z;
    integer     errors;

    wire [31:0] sec_lba;
    wire        sec_start;
    wire        sec_release;
    wire [8:0]  sec_raddr0, sec_raddr1, sec_raddr2, sec_raddr3;
    wire [7:0]  sec_rdata0, sec_rdata1, sec_rdata2, sec_rdata3;

    assign sec_rdata0 = card[(mock_lba << 9) + sec_raddr0];
    assign sec_rdata1 = card[(mock_lba << 9) + sec_raddr1];
    assign sec_rdata2 = card[(mock_lba << 9) + sec_raddr2];
    assign sec_rdata3 = card[(mock_lba << 9) + sec_raddr3];

    task card_zero;
        input integer sector;
        begin
            for (z = 0; z < 512; z = z + 1)
                card[(sector << 9) + z] = 8'h00;
        end
    endtask

    task card_put;
        input integer sector;
        input integer offset;
        input [7:0]   value;
        begin
            card[(sector << 9) + offset] = value;
        end
    endtask

    task card_put32;
        input integer sector;
        input integer offset;
        input [31:0]  value;
        begin
            card[(sector << 9) + offset + 0] = value[7:0];
            card[(sector << 9) + offset + 1] = value[15:8];
            card[(sector << 9) + offset + 2] = value[23:16];
            card[(sector << 9) + offset + 3] = value[31:24];
        end
    endtask

    // The WAV starts at cluster 4, that is LBA 5, and one cluster is one
    // 512 byte sector, so the byte address maps straight onto card sectors.
    task wav_put;
        input integer offset;
        input [7:0]   value;
        begin
            card[((5 + (offset / 512)) << 9) + (offset % 512)] = value;
        end
    endtask

    task wav_put16;
        input integer offset;
        input [15:0]  value;
        begin
            wav_put(offset + 0, value[7:0]);
            wav_put(offset + 1, value[15:8]);
        end
    endtask

    task wav_put32;
        input integer offset;
        input [31:0]  value;
        begin
            wav_put(offset + 0, value[7:0]);
            wav_put(offset + 1, value[15:8]);
            wav_put(offset + 2, value[23:16]);
            wav_put(offset + 3, value[31:24]);
        end
    endtask

    //------------------------------------------------------------------
    // card reader
    //------------------------------------------------------------------
    reg         start_scan;
    wire        scan_busy, scan_done, scan_error;
    wire [3:0]  scan_fault_code;
    wire [7:0]  file_count, file_index;
    wire        load_busy, load_done, load_error;
    wire [31:0] selected_start_cluster, selected_size;
    wire        voice_found;
    wire [31:0] voice_start_cluster, voice_size;
    wire [7:0]  file_data;
    wire        file_data_valid;
    wire        fat_data_ready;
    wire [5:0]  fat_dbg_state;

    reg         voice_stream_started;
    wire        voice_start_pulse;
    reg         cache_ready;               // player_ctrl stand-in
    reg         voice_stream_active;

    wire        parser_abort;
    wire        parser_ready;
    wire        parser_sample_valid;
    wire [7:0]  parser_sample_data;
    wire        parser_busy, parser_done, parser_error;
    wire [3:0]  parser_error_code;
    wire [31:0] parser_rate, parser_samples;
    wire [7:0]  parser_channels, parser_bits;
    wire [5:0]  parser_dbg;

    wire        fifo_full, fifo_empty;
    wire [10:0] fifo_wr_level, fifo_rd_level;
    wire        fifo_rd_en;
    wire [7:0]  fifo_rd_data;
    wire        fifo_wr_en;

    wire signed [23:0] voice_sample;
    wire        voice_generating;
    wire        voice_stream_done;

    assign voice_start_pulse = scan_done && voice_found && cache_ready &&
                               !voice_stream_started;
    assign fat_data_ready    = voice_stream_active ? parser_ready : 1'b0;
    assign fifo_wr_en        = parser_sample_valid && !fifo_full;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            voice_stream_started <= 1'b0;
            voice_stream_active  <= 1'b0;
        end else if (voice_start_pulse) begin
            voice_stream_started <= 1'b1;
            voice_stream_active  <= 1'b1;
        end else if (voice_stream_done) begin
            voice_stream_active  <= 1'b0;
        end
    end

    fat32_file_reader #(
        .MAX_FILES (8)
    ) u_fat (
        .clk                    (sys_clk),
        .rst_n                  (sys_rst_n),
        .sd_init_done           (1'b1),
        .start_scan             (start_scan),
        .scan_busy              (scan_busy),
        .scan_done              (scan_done),
        .scan_error             (scan_error),
        .scan_fault_code        (scan_fault_code),
        .file_count             (file_count),
        .file_index             (file_index),
        .load_src               (voice_stream_active ? 2'd1 : 2'd0),
        .load_start             (voice_start_pulse),
        .load_abort             (voice_stream_active ? parser_abort : 1'b0),
        .load_busy              (load_busy),
        .load_done              (load_done),
        .load_error             (load_error),
        .selected_start_cluster (selected_start_cluster),
        .selected_size          (selected_size),
        .voice_found            (voice_found),
        .voice_start_cluster    (voice_start_cluster),
        .voice_size             (voice_size),
        .file_data              (file_data),
        .file_data_valid        (file_data_valid),
        .file_data_ready        (fat_data_ready),
        .sec_start              (sec_start),
        .sec_lba                (sec_lba),
        .sec_busy               (mock_busy),
        .sec_ready              (sec_ready),
        .sec_error              (sec_error),
        .sec_error_code         (),
        .sec_raddr0             (sec_raddr0),
        .sec_raddr1             (sec_raddr1),
        .sec_raddr2             (sec_raddr2),
        .sec_raddr3             (sec_raddr3),
        .sec_rdata0             (sec_rdata0),
        .sec_rdata1             (sec_rdata1),
        .sec_rdata2             (sec_rdata2),
        .sec_rdata3             (sec_rdata3),
        .sec_release            (sec_release),
        .dbg_state              (fat_dbg_state)
    );

    wav_voice_stream u_parser (
        .clk          (sys_clk),
        .rst_n        (sys_rst_n),
        .start        (voice_start_pulse),
        .stream_done  (load_done),
        .stream_error (load_error),
        .data         (file_data),
        .data_valid   (file_data_valid),
        .data_ready   (parser_ready),
        .abort        (parser_abort),
        .sample_valid (parser_sample_valid),
        .sample_data  (parser_sample_data),
        .sample_ready (~fifo_full),
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

    async_fifo #(
        .DW (8),
        .AW (10)
    ) u_fifo (
        .wr_clk   (sys_clk),
        .wr_rst_n (sys_rst_n),
        .wr_en    (fifo_wr_en),
        .wr_data  (parser_sample_data),
        .wr_full  (fifo_full),
        .wr_level (fifo_wr_level),
        .rd_clk   (aud_clk),
        .rd_rst_n (sys_rst_n),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_rd_data),
        .rd_empty (fifo_empty),
        .rd_level (fifo_rd_level)
    );

    reg [7:0] frame_cnt = 8'd0;
    reg       frame_tick = 1'b0;
    reg       start_tog = 1'b0;
    reg       done_tog = 1'b0;

    always @(posedge aud_clk) begin
        frame_tick <= (frame_cnt == FRAME_DIV - 1);
        if (frame_cnt == FRAME_DIV - 1)
            frame_cnt <= 8'd0;
        else
            frame_cnt <= frame_cnt + 8'd1;
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            start_tog <= 1'b0;
        else if (voice_start_pulse)
            start_tog <= ~start_tog;
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            done_tog <= 1'b0;
        else if (parser_done)
            done_tog <= ~done_tog;
    end

    assign voice_stream_done = parser_done;

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
    // audio side observation
    //------------------------------------------------------------------
    integer loud_frames;
    integer voice_frames;
    reg     generating_q;

    always @(posedge aud_clk) begin
        generating_q <= voice_generating;
        if (frame_tick) begin
            if (voice_generating) begin
                voice_frames <= voice_frames + 1;
                if (voice_sample != 24'sd0)
                    loud_frames <= loud_frames + 1;
            end
        end
    end

    //------------------------------------------------------------------
    // mock SD sector source
    //------------------------------------------------------------------
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            mock_lba   <= 32'd0;
            mock_delay <= 4'd0;
            mock_busy  <= 1'b0;
            sec_ready  <= 1'b0;
            sec_error  <= 1'b0;
        end else begin
            if (sec_release) begin
                sec_ready <= 1'b0;
                mock_busy <= 1'b0;
            end
            if (!mock_busy && !sec_ready && sec_start) begin
                mock_lba   <= sec_lba;
                mock_delay <= 4'd3;
                mock_busy  <= 1'b1;
                sec_error  <= 1'b0;
            end else if (mock_busy) begin
                if (mock_delay == 4'd0) begin
                    mock_busy <= 1'b0;
                    sec_ready <= 1'b1;
                end else
                    mock_delay <= mock_delay - 4'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // stimulus
    //------------------------------------------------------------------
    integer sample_index;
    integer sample_value;
    integer data_bytes;
    integer wait_cycles;

    initial begin
        for (i = 0; i < 16; i = i + 1)
            card_zero(i);

        // MBR: partition 0, type 0x0C, start LBA 1.
        card_put(0, 450, 8'h0C);
        card_put32(0, 454, 32'd1);
        card_put(0, 510, 8'h55);
        card_put(0, 511, 8'hAA);

        // VBR at LBA 1: 512 byte sectors, 1 sector per cluster, one FAT,
        // FAT32 size 1 sector, root cluster 2.
        card_put(1, 0, 8'hEB);
        card_put(1, 1, 8'h3C);
        card_put(1, 2, 8'h90);
        card_put(1, 11, 8'h00);
        card_put(1, 12, 8'h02);
        card_put(1, 13, 8'h01);
        card_put(1, 14, 8'h01);
        card_put(1, 15, 8'h00);
        card_put(1, 16, 8'h01);
        card_put(1, 17, 8'h00);
        card_put(1, 18, 8'h00);
        card_put32(1, 32, 32'd4096);
        card_put32(1, 36, 32'd1);
        card_put32(1, 44, 32'd2);
        card_put(1, 82, "F");
        card_put(1, 83, "A");
        card_put(1, 84, "T");
        card_put(1, 85, "3");
        card_put(1, 86, "2");
        card_put(1, 510, 8'h55);
        card_put(1, 511, 8'hAA);

        // FAT at LBA 2.
        card_put32(2, 0,  32'h0FFF_FFF8);
        card_put32(2, 4,  32'h0FFF_FFFF);
        card_put32(2, 8,  32'h0FFF_FFFF);
        card_put32(2, 12, 32'h0FFF_FFFF);
        card_put32(2, 16, 32'd5);           // cluster 4 -> 5
        card_put32(2, 20, 32'd6);           // cluster 5 -> 6
        card_put32(2, 24, 32'd7);           // cluster 6 -> 7
        card_put32(2, 28, 32'd8);           // cluster 7 -> 8
        card_put32(2, 32, 32'h0FFF_FFFF);   // cluster 8: end

        // Root directory at LBA 3: IMG0.BMP at cluster 3, VOICE.WAV at
        // cluster 4 with WAV_BYTES bytes, then end of directory.
        card_put(3, 0, "I");
        card_put(3, 1, "M");
        card_put(3, 2, "G");
        card_put(3, 3, "0");
        card_put(3, 8, "B");
        card_put(3, 9, "M");
        card_put(3, 10, "P");
        card_put(3, 11, 8'h20);
        card_put(3, 26, 8'h03);
        card_put32(3, 28, 32'd512);

        card_put(3, 32, "V");
        card_put(3, 33, "O");
        card_put(3, 34, "I");
        card_put(3, 35, "C");
        card_put(3, 36, "E");
        card_put(3, 37, 8'h20);
        card_put(3, 38, 8'h20);
        card_put(3, 39, 8'h20);
        card_put(3, 40, "W");
        card_put(3, 41, "A");
        card_put(3, 42, "V");
        card_put(3, 43, 8'h20);
        card_put(3, 58, 8'h04);            // first cluster low = 4
        card_put(3, 59, 8'h00);
        card_put32(3, 60, WAV_BYTES);
        card_put(3, 64, 8'h00);            // end of directory

        // BMP payload (never parsed here).
        for (i = 0; i < 512; i = i + 1)
            card_put(4, i, i[7:0] ^ 8'h5A);

        // WAV payload: 8 kHz / 16 bit mono, 1024 samples of sawtooth.
        data_bytes = 2 * WAV_SAMPLES;
        wav_put(0, "R");
        wav_put(1, "I");
        wav_put(2, "F");
        wav_put(3, "F");
        wav_put32(4, 36 + data_bytes);
        wav_put(8, "W");
        wav_put(9, "A");
        wav_put(10, "V");
        wav_put(11, "E");
        wav_put(12, "f");
        wav_put(13, "m");
        wav_put(14, "t");
        wav_put(15, 8'h20);
        wav_put32(16, 16);
        wav_put16(20, 16'd1);              // PCM
        wav_put16(22, 16'd1);              // mono
        wav_put32(24, 32'd8000);           // sample rate
        wav_put32(28, 32'd16000);          // byte rate
        wav_put16(32, 16'd2);              // block align
        wav_put16(34, 16'd16);             // bits per sample
        wav_put(36, "d");
        wav_put(37, "a");
        wav_put(38, "t");
        wav_put(39, "a");
        wav_put32(40, data_bytes);

        for (sample_index = 0; sample_index < WAV_SAMPLES;
             sample_index = sample_index + 1) begin
            sample_value = ((sample_index % 128) - 64) * 256;
            wav_put16(44 + 2 * sample_index, sample_value[15:0]);
        end

        //------------------------------------------------------------------
        start_scan         = 1'b0;
        cache_ready        = 1'b0;
        errors             = 0;
        loud_frames        = 0;
        voice_frames       = 0;

        repeat (5) @(posedge sys_clk);
        sys_rst_n = 1'b1;
        repeat (5) @(posedge sys_clk);

        @(negedge sys_clk);
        start_scan = 1'b1;
        @(negedge sys_clk);
        start_scan = 1'b0;

        wait_cycles = 0;
        while (!scan_done && wait_cycles < 20000) begin
            @(posedge sys_clk);
            wait_cycles = wait_cycles + 1;
        end
        if (!scan_done) begin
            $display("FAIL: the scan never finished (state=%0d)",
                     fat_dbg_state);
            errors = errors + 1;
        end
        $display("scan: done=%b error=%b bmp_files=%0d voice_found=%b cluster=%0d size=%0d",
                 scan_done, scan_error, file_count, voice_found,
                 voice_start_cluster, voice_size);
        $display("DBG cl_hi=%02x%02x cl_lo=%02x%02x voice_r=%0d r=%0d data_start=%0d dir_lba=%0d",
                 u_fat.dir_cl_hi1, u_fat.dir_cl_hi0,
                 u_fat.dir_cl_lo1, u_fat.dir_cl_lo0,
                 u_fat.voice_start_cluster_r, u_fat.voice_size_r,
                 u_fat.data_start, u_fat.dir_lba);
        $display("DBG card[52]=%02x card[53]=%02x card[58]=%02x card[59]=%02x",
                 card[3*512+52], card[3*512+53],
                 card[3*512+58], card[3*512+59]);
        if (scan_error || (file_count != 8'd1)) begin
            $display("FAIL: unexpected scan result");
            errors = errors + 1;
        end
        if (voice_found !== 1'b1) begin
            $display("FAIL: the WAV entry was not remembered");
            errors = errors + 1;
        end
        if (voice_start_cluster !== 32'd4) begin
            $display("FAIL: voice cluster=%0d expected 4",
                     voice_start_cluster);
            errors = errors + 1;
        end
        if (voice_size !== WAV_BYTES[31:0]) begin
            $display("FAIL: voice size=%0d expected %0d",
                     voice_size, WAV_BYTES);
            errors = errors + 1;
        end

        // The picture cache is declared ready; the single cycle start pulse
        // is produced by the same equation the top level uses.
        @(negedge sys_clk);
        cache_ready = 1'b1;

        wait_cycles = 0;
        while (!parser_done && wait_cycles < 400000) begin
            @(posedge sys_clk);
            wait_cycles = wait_cycles + 1;
        end
        if (!parser_done) begin
            $display("FAIL: the parser never finished (state=%0d fat=%0d samples=%0d active=%b)",
                     parser_dbg, fat_dbg_state, parser_samples,
                     voice_stream_active);
            errors = errors + 1;
        end

        $display("parser: done=%b error=%b code=%0d samples=%0d rate=%0d ch=%0d bits=%0d",
                 parser_done, parser_error, parser_error_code, parser_samples,
                 parser_rate, parser_channels, parser_bits);
        if (parser_error) begin
            $display("FAIL: parser error code %0d", parser_error_code);
            errors = errors + 1;
        end
        if (parser_samples != WAV_SAMPLES) begin
            $display("FAIL: parser samples=%0d expected %0d",
                     parser_samples, WAV_SAMPLES);
            errors = errors + 1;
        end
        if ((parser_rate != 32'd8000) || (parser_channels != 8'd1) ||
            (parser_bits != 8'd16)) begin
            $display("FAIL: unexpected WAV format");
            errors = errors + 1;
        end

        // Let the audio side drain the FIFO and release the chime again.
        // The synthetic file is 1024 samples at 8 kHz, so the player needs
        // roughly 128 ms of audio time to reach the end of the data chunk.
        // The drain is awaited instead of being counted out, because a fixed
        // window shorter than the file would report a false failure.
        wait_cycles = 0;
        while (voice_generating && (wait_cycles < 3000000)) begin
            @(posedge aud_clk);
            wait_cycles = wait_cycles + 1;
        end

        $display("audio: frames=%0d non-zero=%0d generating=%b",
                 voice_frames, loud_frames, voice_generating);
        if (voice_frames < 200) begin
            $display("FAIL: the audio side produced almost no frames (%0d)",
                     voice_frames);
            errors = errors + 1;
        end
        if (loud_frames < voice_frames / 2) begin
            $display("FAIL: most audio frames were silent (%0d of %0d)",
                     loud_frames, voice_frames);
            errors = errors + 1;
        end
        if (voice_generating) begin
            $display("FAIL: the announcement never gave the chime back");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_voice_chain_e2e");
        else
            $display("RESULT: FAIL tb_voice_chain_e2e errors=%0d", errors);
        $finish;
    end

    initial begin
        #80_000_000_000;
        $display("RESULT: FAIL tb_voice_chain_e2e timeout parser=%0d fat=%0d samples=%0d active=%b",
                 parser_dbg, fat_dbg_state, parser_samples,
                 voice_stream_active);
        $finish;
    end

endmodule
