`timescale 1ns / 1ps
//=============================================================================
// tb_fire_wav_hq.v - stream the delivered 8 kHz FIRE.WAV through the parser
//=============================================================================
module tb_fire_wav_hq;

    reg         clk;
    reg         rst_n;
    reg         start;
    reg         stream_done;
    reg         stream_error;
    reg  [7:0]  data;
    reg         data_valid;
    wire        data_ready;
    wire        abort;
    wire        sample_valid;
    wire [7:0]  sample_data;
    wire        sample_ready;
    wire        busy;
    wire        done;
    wire        error;
    wire [3:0]  error_code;
    wire [31:0] sample_rate;
    wire [7:0]  channels;
    wire [7:0]  bits;
    wire [31:0] samples_out;
    wire [5:0]  dbg_state;

    integer     fd;
    integer     got;
    integer     byte_count;
    integer     expected_samples;
    integer     errors;
    reg         saw_done;
    reg         saw_error;

    wav_voice_stream dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .stream_done  (stream_done),
        .stream_error (stream_error),
        .data         (data),
        .data_valid   (data_valid),
        .data_ready   (data_ready),
        .abort        (abort),
        .sample_valid (sample_valid),
        .sample_data  (sample_data),
        .sample_ready (sample_ready),
        .busy         (busy),
        .done         (done),
        .error        (error),
        .error_code   (error_code),
        .sample_rate  (sample_rate),
        .channels     (channels),
        .bits         (bits),
        .samples_out  (samples_out),
        .dbg_state    (dbg_state)
    );

    assign sample_ready = 1'b1;

    always #10 clk = ~clk;

    always @(posedge clk) begin
        if (done)
            saw_done <= 1'b1;
        if (error)
            saw_error <= 1'b1;
    end

    task send_byte;
        input [7:0] value;
        begin
            @(negedge clk);
            data       = value;
            data_valid = 1'b1;
            @(posedge clk);
            while (!data_ready) begin
                @(posedge clk);
            end
            @(negedge clk);
            data_valid = 1'b0;
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
        saw_done     = 1'b0;
        saw_error    = 1'b0;
        byte_count   = 0;

        fd = $fopen("E:/FPGA guo sai/jing jie yao qiu/hdmi_mm_player_advanced/sdcard/FIRE.WAV", "rb");
        if (fd == 0) begin
            $display("FAIL: FIRE.WAV not found");
            $finish;
        end

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        got = $fgetc(fd);
        while (got != -1) begin
            send_byte(got[7:0]);
            byte_count = byte_count + 1;
            got = $fgetc(fd);
        end
        $fclose(fd);

        @(negedge clk);
        stream_done = 1'b1;
        @(posedge clk);
        @(negedge clk);
        stream_done = 1'b0;

        repeat (8) @(posedge clk);

        expected_samples = (byte_count - 44) / 2;
        $display("FIRE.WAV bytes=%0d expected samples=%0d parser samples=%0d",
                 byte_count, expected_samples, samples_out);
        $display("rate=%0d channels=%0d bits=%0d done=%b error=%b code=%0d",
                 sample_rate, channels, bits, saw_done, error, error_code);

        if (!saw_done || saw_error || error || busy) begin
            $display("FAIL: parser did not finish cleanly");
            errors = errors + 1;
        end
        if ((sample_rate != 32'd8000) || (channels != 8'd1) ||
            (bits != 8'd16)) begin
            $display("FAIL: unexpected WAV format");
            errors = errors + 1;
        end
        if (samples_out != expected_samples) begin
            $display("FAIL: samples_out=%0d expected %0d",
                     samples_out, expected_samples);
            errors = errors + 1;
        end
        if (samples_out != 32'd205421) begin
            $display("FAIL: unexpected sample count %0d", samples_out);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_fire_wav_hq");
        else
            $display("RESULT: FAIL tb_fire_wav_hq errors=%0d", errors);
        $finish;
    end

    initial begin
        #120_000_000;
        $display("RESULT: FAIL tb_fire_wav_hq timeout state=%0d", dbg_state);
        $finish;
    end

endmodule
