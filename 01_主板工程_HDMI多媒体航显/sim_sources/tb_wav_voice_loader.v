`timescale 1ns / 1ps
//=============================================================================
// tb_wav_voice_loader.v - streaming WAV parser and bank packer
//
// Three card files are pushed through the loader byte by byte:
//   1. 16 bit PCM mono 8 kHz, with an odd sized LIST chunk in front of the fmt
//      chunk and a trailing chunk after data
//   2. 8 bit mu-law stereo in a WAVE_FORMAT_EXTENSIBLE fmt chunk, whose data
//      chunk claims more bytes than the file really holds
//   3. a file that is not RIFF/WAVE at all
//
// The mu-law codes the loader is expected to write come from the G.711
// reference table checked in tb_ulaw.
//=============================================================================
module tb_wav_voice_loader;

    reg         clk;
    reg         rst_n;
    reg         start;
    reg         stream_done;
    reg         stream_error;
    reg  [7:0]  data;
    reg         data_valid;
    wire        data_ready;
    wire        abort;
    wire        wr_en;
    wire [15:0] wr_addr;
    wire [7:0]  wr_data;
    wire        busy;
    wire        done;
    wire        error;
    wire [3:0]  error_code;
    wire [31:0] sample_rate;
    wire [7:0]  channels;
    wire [7:0]  bits;
    wire [16:0] words_stored;
    wire        truncated;
    wire [5:0]  dbg_state;

    reg  [7:0]  file_bytes [0:255];
    reg  [7:0]  bank_got   [0:63];
    reg  [7:0]  bank_ref   [0:63];
    integer     errors;
    integer     i;
    integer     file_len;
    reg         saw_abort;
    reg         saw_done;

    wav_voice_loader #(
        .BANK_WORDS (65536),
        .AW         (16)
    ) dut (
        .clk          (clk),
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

    always #10 clk = ~clk;

    // capture what the loader writes into the bank
    always @(posedge clk) begin
        if (wr_en)
            bank_got[wr_addr[5:0]] <= wr_data;
        if (abort)
            saw_abort <= 1'b1;
        if (done)
            saw_done <= 1'b1;
    end

    //------------------------------------------------------------------
    // byte stream helpers
    //------------------------------------------------------------------
    task send_byte;
        input [7:0] value;
        begin
            @(negedge clk);
            data       = value;
            data_valid = 1'b1;
            // the byte is held on the wire while the loader is between fields
            while (busy && !data_ready)
                @(negedge clk);
            @(posedge clk);
            @(negedge clk);
            data_valid = 1'b0;
            @(posedge clk);
        end
    endtask

    // The loader hands the card back as soon as the last byte it cares about
    // has arrived, so the tail of the file is never pushed at it.
    task send_range;
        input integer first;
        input integer count;
        integer k;
        begin
            for (k = 0; k < count; k = k + 1) begin
                if (busy)
                    send_byte(file_bytes[first + k]);
                else
                    k = count;
            end
        end
    endtask

    task begin_file;
        begin
            @(negedge clk);
            start      = 1'b1;
            saw_abort  = 1'b0;
            saw_done   = 1'b0;
            @(negedge clk);
            start      = 1'b0;
            @(posedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // file builders
    //------------------------------------------------------------------
    task put;
        input integer at;
        input [7:0]   value;
        begin
            file_bytes[at] = value;
        end
    endtask

    task put16;
        input integer at;
        input [15:0]  value;
        begin
            file_bytes[at]     = value[7:0];
            file_bytes[at + 1] = value[15:8];
        end
    endtask

    task put32;
        input integer at;
        input [31:0]  value;
        begin
            file_bytes[at]     = value[7:0];
            file_bytes[at + 1] = value[15:8];
            file_bytes[at + 2] = value[23:16];
            file_bytes[at + 3] = value[31:24];
        end
    endtask

    task put_tag;
        input integer at;
        input [7:0] c0, c1, c2, c3;
        begin
            file_bytes[at]     = c0;
            file_bytes[at + 1] = c1;
            file_bytes[at + 2] = c2;
            file_bytes[at + 3] = c3;
        end
    endtask

    initial begin
        clk          = 1'b0;
        rst_n        = 1'b0;
        start        = 1'b0;
        stream_done  = 1'b0;
        stream_error = 1'b0;
        data         = 8'd0;
        data_valid   = 1'b0;
        errors       = 0;
        saw_abort    = 1'b0;
        saw_done     = 1'b0;
        for (i = 0; i < 256; i = i + 1)
            file_bytes[i] = 8'h00;
        for (i = 0; i < 64; i = i + 1) begin
            bank_got[i] = 8'h00;
            bank_ref[i] = 8'h00;
        end

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        //------------------------------------------------------------------
        // case 1: PCM 16 bit mono, LIST chunk (odd size) before fmt,
        //         trailing chunk after the data chunk
        //------------------------------------------------------------------
        put_tag(0, "R", "I", "F", "F");
        put32(4, 32'd82);
        put_tag(8, "W", "A", "V", "E");

        put_tag(12, "L", "I", "S", "T");
        put32(16, 32'd5);                 // odd size: 5 bytes plus a pad byte
        put(20, 8'h4C);
        put(21, 8'h49);
        put(22, 8'h53);
        put(23, 8'h54);
        put(24, 8'h00);
        put(25, 8'h00);                   // pad

        put_tag(26, "f", "m", "t", " ");
        put32(30, 32'd16);
        put16(34, 16'd1);                 // WAVE_FORMAT_PCM
        put16(36, 16'd1);                 // mono
        put32(38, 32'd8000);
        put32(42, 32'd16000);
        put16(46, 16'd2);
        put16(48, 16'd16);

        put_tag(50, "d", "a", "t", "a");
        put32(54, 32'd16);                // eight samples
        // 0, 1000, 8000, -1000, 32767, -32768, 100, -100
        put16(58,  16'd0);
        put16(60,  16'd1000);
        put16(62,  16'd8000);
        put16(64,  -16'd1000);
        put16(66,  16'd32767);
        put16(68,  -16'd32768);
        put16(70,  16'd100);
        put16(72,  -16'd100);

        put_tag(74, "f", "a", "c", "t");  // a trailing chunk the loader must
        put32(78, 32'd4);                 // never have to read
        file_len = 82;

        bank_ref[0] = 8'hFF;              // 0
        bank_ref[1] = 8'hCE;              // 1000
        bank_ref[2] = 8'hA0;              // 8000
        bank_ref[3] = 8'h4E;              // -1000
        bank_ref[4] = 8'h80;              // 32767
        bank_ref[5] = 8'h00;              // -32768
        bank_ref[6] = 8'hF2;              // 100
        bank_ref[7] = 8'h72;              // -100

        begin_file;
        send_range(0, file_len);
        repeat (8) @(posedge clk);

        if (!saw_done || busy || error) begin
            $display("FAIL: case1 done=%b busy=%b error=%b code=%0d state=%0d",
                     saw_done, busy, error, error_code, dbg_state);
            errors = errors + 1;
        end
        if (words_stored != 17'd8) begin
            $display("FAIL: case1 samples=%0d expected 8", words_stored);
            errors = errors + 1;
        end
        if (sample_rate != 32'd8000) begin
            $display("FAIL: case1 rate=%0d expected 8000", sample_rate);
            errors = errors + 1;
        end
        if (channels != 8'd1 || bits != 8'd16) begin
            $display("FAIL: case1 channels=%0d bits=%0d", channels, bits);
            errors = errors + 1;
        end
        if (truncated) begin
            $display("FAIL: case1 reported truncation");
            errors = errors + 1;
        end
        if (!saw_abort) begin
            $display("FAIL: case1 did not release the card early");
            errors = errors + 1;
        end
        for (i = 0; i < 8; i = i + 1)
            if (bank_got[i] !== bank_ref[i]) begin
                $display("FAIL: case1 bank[%0d]=%02x expected %02x",
                         i, bank_got[i], bank_ref[i]);
                errors = errors + 1;
            end

        //------------------------------------------------------------------
        // case 2: WAVE_FORMAT_EXTENSIBLE, 8 bit mu-law stereo 8 kHz, the
        //         data chunk is longer than the file
        //------------------------------------------------------------------
        for (i = 0; i < 256; i = i + 1)
            file_bytes[i] = 8'h00;
        for (i = 0; i < 64; i = i + 1)
            bank_got[i] = 8'h00;

        put_tag(0, "R", "I", "F", "F");
        put32(4, 32'd100);
        put_tag(8, "W", "A", "V", "E");

        put_tag(12, "f", "m", "t", " ");
        put32(16, 32'd40);                // extensible fmt chunk
        put16(20, 16'hFFFE);
        put16(22, 16'd2);                 // stereo
        put32(24, 32'd8000);
        put32(28, 32'd16000);
        put16(32, 16'd2);
        put16(34, 16'd8);                 // 8 bit mu-law
        put16(36, 16'd22);
        put16(38, 16'd8);                 // valid bits
        put32(40, 32'h0000_0003);
        put16(44, 16'h0007);              // sub format: mu-law
        put16(46, 16'h0000);
        // remaining GUID bytes stay zero

        // the fmt body runs from 20 to 59, so the data chunk starts at 60
        put_tag(60, "d", "a", "t", "a");
        put32(64, 32'd100);               // claims far more than the file holds
        file_bytes[68] = 8'hFF;           // left 1
        file_bytes[69] = 8'h00;           // right, dropped
        file_bytes[70] = 8'h80;           // left 2
        file_bytes[71] = 8'h7F;
        file_bytes[72] = 8'hCE;           // left 3
        file_bytes[73] = 8'h31;
        file_bytes[74] = 8'h00;           // left 4
        file_bytes[75] = 8'hFF;
        file_len = 76;

        bank_ref[0] = 8'hFF;
        bank_ref[1] = 8'h80;
        bank_ref[2] = 8'hCE;
        bank_ref[3] = 8'h00;

        begin_file;
        send_range(0, file_len);

        // the file ends here: fat32_file_reader pulses load_done
        @(negedge clk);
        stream_done = 1'b1;
        @(posedge clk);
        @(negedge clk);
        stream_done = 1'b0;
        repeat (4) @(posedge clk);

        if (!saw_done || busy || error) begin
            $display("FAIL: case2 done=%b busy=%b error=%b code=%0d state=%0d",
                     saw_done, busy, error, error_code, dbg_state);
            errors = errors + 1;
        end
        if (words_stored != 17'd4) begin
            $display("FAIL: case2 samples=%0d expected 4", words_stored);
            errors = errors + 1;
        end
        if (channels != 8'd2 || bits != 8'd8) begin
            $display("FAIL: case2 channels=%0d bits=%0d", channels, bits);
            errors = errors + 1;
        end
        for (i = 0; i < 4; i = i + 1)
            if (bank_got[i] !== bank_ref[i]) begin
                $display("FAIL: case2 bank[%0d]=%02x expected %02x",
                         i, bank_got[i], bank_ref[i]);
                errors = errors + 1;
            end

        //------------------------------------------------------------------
        // case 3: not a RIFF file
        //------------------------------------------------------------------
        put_tag(0, "J", "U", "N", "K");
        put32(4, 32'd16);
        put_tag(8, "N", "O", "P", "E");
        file_len = 12;

        begin_file;
        send_range(0, file_len);
        repeat (4) @(posedge clk);

        if (!error || (error_code != 4'd1)) begin
            $display("FAIL: case3 error=%b code=%0d expected 1",
                     error, error_code);
            errors = errors + 1;
        end
        if (words_stored != 17'd0) begin
            $display("FAIL: case3 stored %0d samples", words_stored);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_wav_voice_loader");
        else
            $display("RESULT: FAIL tb_wav_voice_loader errors=%0d state=%0d",
                     errors, dbg_state);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL tb_wav_voice_loader timeout state=%0d busy=%b",
                 dbg_state, busy);
        $finish;
    end

endmodule
