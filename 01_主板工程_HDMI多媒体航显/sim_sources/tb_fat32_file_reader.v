`timescale 1ns / 1ps
//=============================================================================
// tb_fat32_file_reader.v - synthetic FAT32 card test
//
// The card image contains one 0x0C partition at LBA 1, a FAT32 BPB, a root
// directory with one 8.3 BMP entry, FIRE.WAV ahead of VOICE.WAV, plus the
// data clusters for all three streams.
//
// Checks:
//   1. the scan still lists exactly one BMP and remembers the WAV entry
//      (FIRE.WAV must not steal the normal announcement slot)
//   2. load_src = 0 streams the BMP at cluster 3, 512 bytes
//   3. load_src = 1 streams the WAV at cluster 5, 1024 bytes over the
//      cluster chain 5 -> 6 -> EOC
//   4. load_src = 4 streams FIRE.WAV at cluster 7, 512 bytes
//=============================================================================
module tb_fat32_file_reader;

    reg         clk;
    reg         rst_n;
    reg         sd_init_done;
    reg         start_scan;
    wire        scan_busy;
    wire        scan_done;
    wire        scan_error;
    wire [7:0]  file_count;
    reg  [7:0]  file_index;
    reg  [2:0]  load_src;
    reg         load_start;
    wire        load_busy;
    wire        load_done;
    wire        load_error;
    wire [31:0] selected_start_cluster;
    wire [31:0] selected_size;
    wire        voice_found;
    wire [31:0] voice_start_cluster;
    wire [31:0] voice_size;
    wire        fire_voice_found;
    wire [7:0]  file_data;
    wire        file_data_valid;

    reg         sec_ready;
    reg         sec_error;
    wire [3:0]  sec_error_code;
    wire [8:0]  sec_raddr1;
    wire [8:0]  sec_raddr2;
    wire [8:0]  sec_raddr3;
    wire [7:0]  sec_rdata0;
    wire [7:0]  sec_rdata1;
    wire [7:0]  sec_rdata2;
    wire [7:0]  sec_rdata3;
    wire        sec_start;
    wire [31:0] sec_lba;
    wire        sec_release;
    wire [8:0]  sec_raddr0;
    wire [5:0]  dbg_state;

    reg  [7:0]  card [0:8191];      // 16 sectors
    reg  [31:0] mock_lba;
    reg  [3:0]  mock_delay;
    reg         mock_busy;
    reg  [5:0]  mon_prev_state;
    integer     i;
    integer     errors;
    integer     got_count;
    integer     wav_count;
    integer     fire_count;
    reg  [7:0]  expected_wav;

    assign sec_rdata0 = card[(mock_lba << 9) + sec_raddr0];
    assign sec_rdata1 = card[(mock_lba << 9) + sec_raddr1];
    assign sec_rdata2 = card[(mock_lba << 9) + sec_raddr2];
    assign sec_rdata3 = card[(mock_lba << 9) + sec_raddr3];

    fat32_file_reader #(
        .MAX_FILES(16)
    ) dut (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .sd_init_done          (sd_init_done),
        .start_scan            (start_scan),
        .scan_busy             (scan_busy),
        .scan_done             (scan_done),
        .scan_error            (scan_error),
        .file_count            (file_count),
        .file_index            (file_index),
        .load_src              (load_src),
        .load_start            (load_start),
        .load_abort            (1'b0),
        .load_busy             (load_busy),
        .load_done             (load_done),
        .load_error            (load_error),
        .selected_start_cluster(selected_start_cluster),
        .selected_size         (selected_size),
        .voice_found           (voice_found),
        .voice_start_cluster   (voice_start_cluster),
        .voice_size            (voice_size),
        .fire_voice_found      (fire_voice_found),
        .file_data             (file_data),
        .file_data_valid       (file_data_valid),
        .file_data_ready       (1'b1),
        .sec_start             (sec_start),
        .sec_lba               (sec_lba),
        .sec_busy              (mock_busy),
        .sec_ready             (sec_ready),
        .sec_error             (sec_error),
        .sec_error_code        (sec_error_code),
        .scan_fault_code       (),
        .sec_raddr0            (sec_raddr0),
        .sec_raddr1            (sec_raddr1),
        .sec_raddr2            (sec_raddr2),
        .sec_raddr3            (sec_raddr3),
        .sec_rdata0            (sec_rdata0),
        .sec_rdata1            (sec_rdata1),
        .sec_rdata2            (sec_rdata2),
        .sec_rdata3            (sec_rdata3),
        .sec_release           (sec_release),
        .dbg_state             (dbg_state)
    );

    always #10 clk = ~clk;

    task card_set_sector;
        input integer sector;
        begin
            for (i = 0; i < 512; i = i + 1)
                card[(sector << 9) + i] = 8'h00;
        end
    endtask

    task card_put;
        input integer sector;
        input integer offset;
        input [7:0] value;
        begin
            card[(sector << 9) + offset] = value;
        end
    endtask

    task card_put32;
        input integer sector;
        input integer offset;
        input [31:0] value;
        begin
            card[(sector << 9) + offset + 0] = value[7:0];
            card[(sector << 9) + offset + 1] = value[15:8];
            card[(sector << 9) + offset + 2] = value[23:16];
            card[(sector << 9) + offset + 3] = value[31:24];
        end
    endtask

    initial begin
        //------------------------------------------------------------------
        // synthetic card image
        //------------------------------------------------------------------
        for (i = 0; i < 8192; i = i + 1)
            card[i] = 8'h00;

        // MBR: partition 0, type 0x0C, start LBA 1.
        card_put(0, 446 + 4, 8'h0C);
        card_put32(0, 446 + 8, 32'd1);
        card_put(0, 510, 8'h55);
        card_put(0, 511, 8'hAA);

        // VBR at LBA 1.
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
        card_put32(2, 16, 32'h0FFF_FFFF);   // cluster 4
        card_put32(2, 20, 32'd6);           // cluster 5 -> 6
        card_put32(2, 24, 32'h0FFF_FFFF);   // cluster 6
        card_put32(2, 28, 32'h0FFF_FFFF);   // cluster 7

        // Root directory at LBA 3: IMG0.BMP, cluster 3, size 512.
        card_put(3, 0, "I");
        card_put(3, 1, "M");
        card_put(3, 2, "G");
        card_put(3, 3, "0");
        card_put(3, 8, "B");
        card_put(3, 9, "M");
        card_put(3, 10, "P");
        card_put(3, 11, 8'h20);
        card_put(3, 20, 8'h00);
        card_put(3, 21, 8'h00);
        card_put(3, 26, 8'h03);
        card_put(3, 27, 8'h00);
        card_put32(3, 28, 32'd512);

        // FIRE.WAV in the second slot: cluster 7, 512 bytes.  It sits ahead
        // of VOICE.WAV on purpose, to prove the normal announcement keeps its
        // own entry.
        card_put(3, 32, "F");
        card_put(3, 33, "I");
        card_put(3, 34, "R");
        card_put(3, 35, "E");
        card_put(3, 36, 8'h20);
        card_put(3, 37, 8'h20);
        card_put(3, 38, 8'h20);
        card_put(3, 39, 8'h20);
        card_put(3, 40, "W");
        card_put(3, 41, "A");
        card_put(3, 42, "V");
        card_put(3, 43, 8'h20);       // attribute: archive
        card_put(3, 52, 8'h00);       // cluster high
        card_put(3, 53, 8'h00);
        card_put(3, 58, 8'h07);       // cluster low = 7
        card_put(3, 59, 8'h00);
        card_put32(3, 60, 32'd512);

        // VOICE.WAV in the third slot: cluster 5, 1024 bytes, chain 5 -> 6.
        card_put(3, 64, "V");
        card_put(3, 65, "O");
        card_put(3, 66, "I");
        card_put(3, 67, "C");
        card_put(3, 68, "E");
        card_put(3, 69, 8'h20);
        card_put(3, 70, 8'h20);
        card_put(3, 71, 8'h20);
        card_put(3, 72, "W");
        card_put(3, 73, "A");
        card_put(3, 74, "V");
        card_put(3, 75, 8'h20);       // attribute: archive
        card_put(3, 84, 8'h00);       // cluster high
        card_put(3, 85, 8'h00);
        card_put(3, 90, 8'h05);       // cluster low = 5
        card_put(3, 91, 8'h00);
        card_put32(3, 92, 32'd1024);
        card_put(3, 96, 8'h00);       // end of directory

        // File cluster at LBA 4.
        for (i = 0; i < 512; i = i + 1)
            card_put(4, i, i[7:0] ^ 8'h5A);

        // WAV data clusters at LBA 6 (cluster 5) and LBA 7 (cluster 6).
        for (i = 0; i < 512; i = i + 1)
            card_put(6, i, i[7:0] ^ 8'hA5);
        for (i = 0; i < 512; i = i + 1)
            card_put(7, i, i[7:0] ^ 8'h3C);

        // FIRE.WAV data cluster at LBA 8 (cluster 7).
        for (i = 0; i < 512; i = i + 1)
            card_put(8, i, i[7:0] ^ 8'hC3);

        //------------------------------------------------------------------
        // reset and scan
        //------------------------------------------------------------------
        clk          = 1'b0;
        rst_n        = 1'b0;
        sd_init_done = 1'b1;
        start_scan   = 1'b0;
        file_index   = 8'd0;
        load_src     = 2'd0;
        load_start   = 1'b0;
        sec_ready    = 1'b0;
        sec_error    = 1'b0;
        mock_lba     = 32'd0;
        mock_delay   = 4'd0;
        mock_busy    = 1'b0;
        errors       = 0;
        got_count    = 0;
        wav_count    = 0;
        fire_count   = 0;
        expected_wav = 8'h00;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        @(negedge clk);
        start_scan = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start_scan = 1'b0;

        wait (scan_done == 1'b1);
        $display("DBG mount_lba=%0d fat_start=%0d data_start=%0d dir_lba=%0d list_count=%0d",
                 dut.mount_lba, dut.fat_start, dut.data_start,
                 dut.dir_lba, dut.list_count);
        $display("DBG entry first=%02x attr=%02x ext=%02x%02x%02x cl=%02x%02x%02x%02x size=%02x%02x%02x%02x",
                 dut.dir_first_byte, dut.dir_attr,
                 dut.dir_ext0, dut.dir_ext1, dut.dir_ext2,
                 dut.dir_cl_hi1, dut.dir_cl_hi0,
                 dut.dir_cl_lo1, dut.dir_cl_lo0,
                 dut.dir_size3, dut.dir_size2,
                 dut.dir_size1, dut.dir_size0);
        if (scan_error) begin
            $display("FAIL: scan_error set");
            errors = errors + 1;
        end
        if (file_count != 8'd1) begin
            $display("FAIL: file_count=%0d expected 1", file_count);
            errors = errors + 1;
        end
        if (voice_found !== 1'b1) begin
            $display("FAIL: voice_found=%0b expected 1", voice_found);
            errors = errors + 1;
        end
        if (voice_start_cluster !== 32'd5) begin
            $display("FAIL: voice cluster=%0d expected 5",
                     voice_start_cluster);
            errors = errors + 1;
        end
        if (voice_size !== 32'd1024) begin
            $display("FAIL: voice size=%0d expected 1024", voice_size);
            errors = errors + 1;
        end
        if (fire_voice_found !== 1'b1) begin
            $display("FAIL: fire_voice_found=%0b expected 1", fire_voice_found);
            errors = errors + 1;
        end

        @(negedge clk);
        load_start = 1'b1;
        @(posedge clk);
        @(negedge clk);
        load_start = 1'b0;

        while (got_count < 512) begin
            @(negedge clk);
            if (file_data_valid) begin
                if (file_data !== (got_count[7:0] ^ 8'h5A)) begin
                    $display("FAIL: byte %0d got %02x expected %02x",
                             got_count, file_data, got_count[7:0] ^ 8'h5A);
                    errors = errors + 1;
                    got_count = 512;
                end else begin
                    got_count = got_count + 1;
                end
            end
        end

        wait (load_done == 1'b1);
        if (load_error) begin
            $display("FAIL: load_error set");
            errors = errors + 1;
        end
        if (selected_start_cluster !== 32'd3) begin
            $display("FAIL: selected cluster=%0d expected 3",
                     selected_start_cluster);
            errors = errors + 1;
        end
        if (selected_size !== 32'd512) begin
            $display("FAIL: selected size=%0d expected 512", selected_size);
            errors = errors + 1;
        end

        //------------------------------------------------------------------
        // second stream: the WAV file, ignoring the picture list entirely
        //------------------------------------------------------------------
        @(posedge clk);
        wait (load_done == 1'b0);
        @(negedge clk);
        load_src = 2'd1;
        load_start = 1'b1;
        @(posedge clk);
        @(negedge clk);
        load_start = 1'b0;

        while (wav_count < 1024) begin
            @(negedge clk);
            if (file_data_valid) begin
                if (wav_count < 512)
                    expected_wav = wav_count[7:0] ^ 8'hA5;
                else
                    expected_wav = wav_count[7:0] ^ 8'h3C;
                if (file_data !== expected_wav) begin
                    $display("FAIL: wav byte %0d got %02x expected %02x",
                             wav_count, file_data, expected_wav);
                    errors = errors + 1;
                    wav_count = 1024;
                end else begin
                    wav_count = wav_count + 1;
                end
            end
        end

        wait (load_done == 1'b1);
        if (load_error) begin
            $display("FAIL: voice load_error set");
            errors = errors + 1;
        end
        if (selected_start_cluster !== 32'd5) begin
            $display("FAIL: voice selected cluster=%0d expected 5",
                     selected_start_cluster);
            errors = errors + 1;
        end
        if (selected_size !== 32'd1024) begin
            $display("FAIL: voice selected size=%0d expected 1024",
                     selected_size);
            errors = errors + 1;
        end
        load_src = 3'd0;

        //------------------------------------------------------------------
        // third stream: FIRE.WAV (load_src = 4)
        //------------------------------------------------------------------
        @(posedge clk);
        wait (load_done == 1'b0);
        @(negedge clk);
        load_src = 3'd4;
        load_start = 1'b1;
        @(posedge clk);
        @(negedge clk);
        load_start = 1'b0;

        while (fire_count < 512) begin
            @(negedge clk);
            if (file_data_valid) begin
                if (file_data !== (fire_count[7:0] ^ 8'hC3)) begin
                    $display("FAIL: fire wav byte %0d got %02x expected %02x",
                             fire_count, file_data, fire_count[7:0] ^ 8'hC3);
                    errors = errors + 1;
                    fire_count = 512;
                end else begin
                    fire_count = fire_count + 1;
                end
            end
        end

        wait (load_done == 1'b1);
        if (load_error) begin
            $display("FAIL: fire wav load_error set");
            errors = errors + 1;
        end
        if (selected_start_cluster !== 32'd7) begin
            $display("FAIL: fire wav selected cluster=%0d expected 7",
                     selected_start_cluster);
            errors = errors + 1;
        end
        if (selected_size !== 32'd512) begin
            $display("FAIL: fire wav selected size=%0d expected 512",
                     selected_size);
            errors = errors + 1;
        end
        load_src = 3'd0;

        if (errors == 0)
            $display("RESULT: PASS tb_fat32_file_reader bmp_bytes=%0d wav_bytes=%0d fire_bytes=%0d",
                     got_count, wav_count, fire_count);
        else
            $display("RESULT: FAIL tb_fat32_file_reader errors=%0d state=%0d",
                     errors, dbg_state);
        $finish;
    end

    // Mock SD sector source: four cycles after start, the sector is ready.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
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

            if (sec_start && !mock_busy && !sec_ready) begin
                mock_lba   <= sec_lba;
                mock_delay <= 4'd4;
                mock_busy  <= 1'b1;
            end

            if (mock_busy) begin
                if (mock_delay == 4'd0) begin
                    mock_busy <= 1'b0;
                    sec_ready <= 1'b1;
                end else
                    mock_delay <= mock_delay - 4'd1;
            end
        end
    end

    initial begin
        #20_000_000;
        $display("RESULT: FAIL tb_fat32_file_reader timeout state=%0d ready=%b",
                 dbg_state, sec_ready);
        $finish;
    end

endmodule
