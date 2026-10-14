`timescale 1ns / 1ps
//=============================================================================
// wav_voice_stream.v - stream a RIFF/WAVE file as one mu-law byte per sample
//
// This is the streaming companion of wav_voice_loader.  It consumes the same
// byte handshake from fat32_file_reader, parses the RIFF header, and exposes
// one compressed sample at a time.  The downstream FIFO is allowed to back
// pressure the card reader, so a one minute announcement does not need to fit
// in the FPGA block RAM.
//
// The incoming byte is registered before the parser and mu-law encoder.  That
// one stage is intentional: without it the card-side state decode, sector RAM
// read, RIFF parser and encoder form one 22 ns path on the 50 MHz clock.
//
// Accepted input:
//   * PCM16 mono/stereo, PCM8 mono, or mu-law 8 bit mono
//   * any stream rate reported in the fmt chunk
//
// The module stops as soon as the declared data chunk has been consumed; a
// trailer after the audio data is not needed by the player.
//=============================================================================
module wav_voice_stream (
    input  wire        clk,
    input  wire        rst_n,

    // control
    input  wire        start,
    input  wire        stream_done,
    input  wire        stream_error,

    // byte stream from the card
    input  wire [7:0]  data,
    input  wire        data_valid,
    output wire        data_ready,
    output reg         abort,

    // compressed sample stream
    output wire        sample_valid,
    output wire [7:0]  sample_data,
    input  wire        sample_ready,

    // status
    output reg         busy,
    output reg         done,
    output reg         error,
    output reg  [3:0]  error_code,
    output reg  [31:0] sample_rate,
    output reg  [7:0]  channels,
    output reg  [7:0]  bits,
    output reg  [31:0] samples_out,
    output reg  [5:0]  dbg_state
);

    localparam [5:0] S_IDLE   = 6'd0,
                     S_RIFF   = 6'd1,
                     S_RIFFSZ = 6'd2,
                     S_WAVE   = 6'd3,
                     S_CHUNK  = 6'd4,
                     S_CSIZE  = 6'd5,
                     S_FMT    = 6'd6,
                     S_SKIP   = 6'd7,
                     S_DATA   = 6'd8,
                     S_FINISH = 6'd9,
                     S_FAIL   = 6'd10,
                     S_PAUSE  = 6'd11;

    localparam [3:0] E_RIFF   = 4'd1,
                     E_FMT    = 4'd2,
                     E_NODATA = 4'd3,
                     E_EMPTY  = 4'd4,
                     E_STREAM = 4'd5;

    localparam [31:0] ID_RIFF = 32'h5249_4646,
                      ID_WAVE = 32'h5741_5645,
                      ID_FMT  = 32'h666D_7420,
                      ID_DATA = 32'h6461_7461;

    reg [5:0]   st;
    reg [1:0]   bc;
    reg [31:0]  sh;
    reg [31:0]  chunk_id;
    reg [31:0]  chunk_size;
    reg [31:0]  skip_left;
    reg [31:0]  data_left;
    reg [4:0]   fmt_cnt;
    reg [4:0]   fmt_target;
    reg [207:0] fmt_sh;
    reg [2:0]   bcnt;
    reg [7:0]   b0, b1, b2;

    reg [15:0]  fmt_code;
    wire        fmt_is_ulaw = (fmt_code == 16'h0007);

    reg [7:0]   out_byte;
    reg         out_pending;
    reg [7:0]   in_data;
    reg         in_valid;

    // Values assembled in the same way as the byte-by-byte loader.
    wire [31:0] tag_val   = {sh[23:0], in_data};
    wire [31:0] chunk_val = {in_data, sh[7:0], sh[15:8], sh[23:16]};

    wire signed [15:0] pcm_from_u8 =
        ($signed({1'b0, in_data}) - 16'sd128) <<< 8;
    wire signed [15:0] pcm_mono = $signed({in_data, b0});
    wire signed [15:0] ch_left  = $signed({b1, b0});
    wire signed [15:0] ch_right = $signed({in_data, b2});
    wire signed [15:0] pcm_stereo = (ch_left + ch_right) >>> 1;
    wire signed [15:0] pcm_in = (bits == 8'd8) ? pcm_from_u8 :
                                ((channels == 8'd2) ? pcm_stereo : pcm_mono);

    wire [7:0] pcm_code;
    ulaw_encode u_encode (
        .pcm  (pcm_in),
        .code (pcm_code)
    );

    wire        take_sample = (bits == 8'd8) ?
                              ((channels == 8'd2) ? (bcnt == 3'd0) : 1'b1) :
                              ((channels == 8'd2) ? (bcnt == 3'd3) : (bcnt == 3'd1));
    wire [2:0] frame_bytes = (bits == 8'd8) ?
                             ((channels == 8'd2) ? 3'd2 : 3'd1) :
                             ((channels == 8'd2) ? 3'd4 : 3'd2);
    wire [2:0] bcnt_plus   = {1'b0, bcnt} + 3'd1;
    wire [2:0] bcnt_next   = (bcnt_plus >= frame_bytes) ? 3'd0 : bcnt_plus;

    wire [127:0] fmt_val_16  = {fmt_sh[119:0], in_data};
    wire [207:0] fmt_val_26  = {fmt_sh[199:0], in_data};
    wire [207:0] fmt_val     = (fmt_target == 5'd16) ? {fmt_val_16, 80'd0}
                                                     : fmt_val_26;
    wire [15:0]  fmt_code_in = {fmt_val[199:192], fmt_val[207:200]};
    wire [15:0]  fmt_chan_in = {fmt_val[183:176], fmt_val[191:184]};
    wire [31:0]  fmt_rate_in = {fmt_val[151:144], fmt_val[159:152],
                                fmt_val[167:160], fmt_val[175:168]};
    wire [15:0]  fmt_bits_in = {fmt_val[87:80],  fmt_val[95:88]};
    wire [15:0]  fmt_sub_in  = {fmt_val[7:0],    fmt_val[15:8]};
    wire [15:0]  fmt_use_in  = (fmt_code_in == 16'hFFFE) ? fmt_sub_in
                                                         : fmt_code_in;
    wire         fmt_bits_ok =
        ((fmt_use_in == 16'h0001) &&
         ((fmt_bits_in == 16'd16) || (fmt_bits_in == 16'd8))) ||
        ((fmt_use_in == 16'h0007) && (fmt_bits_in == 16'd8));
    wire         fmt_channels_ok =
        ((fmt_chan_in == 16'd1) ||
         ((fmt_chan_in == 16'd2) && (fmt_use_in != 16'h0007)));

    assign sample_valid = out_pending;
    assign sample_data  = out_byte;
    wire   parser_accept = busy && (st <= S_DATA) && !out_pending;
    assign data_ready    = parser_accept && !in_valid;
    wire   byte_take     = in_valid && (st <= S_DATA) && !out_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            bc          <= 2'd0;
            sh          <= 32'd0;
            chunk_id    <= 32'd0;
            chunk_size  <= 32'd0;
            skip_left   <= 32'd0;
            data_left   <= 32'd0;
            fmt_cnt     <= 5'd0;
            fmt_target  <= 5'd16;
            fmt_sh      <= 208'd0;
            bcnt        <= 3'd0;
            b0          <= 8'd0;
            b1          <= 8'd0;
            b2          <= 8'd0;
            fmt_code    <= 16'd0;
            out_byte    <= 8'd0;
            out_pending <= 1'b0;
            in_data     <= 8'd0;
            in_valid    <= 1'b0;
            abort       <= 1'b0;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            error_code  <= 4'd0;
            sample_rate <= 32'd0;
            channels    <= 8'd0;
            bits        <= 8'd0;
            samples_out <= 32'd0;
            dbg_state   <= S_IDLE;
        end else begin
            abort     <= 1'b0;
            done      <= 1'b0;
            dbg_state <= st;

            if (out_pending && sample_ready) begin
                out_pending <= 1'b0;
                samples_out <= samples_out + 32'd1;
            end

            if (data_valid && data_ready) begin
                in_data  <= data;
                in_valid <= 1'b1;
            end else if (byte_take) begin
                in_valid <= 1'b0;
            end

            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy        <= 1'b1;
                        error       <= 1'b0;
                        error_code  <= 4'd0;
                        samples_out <= 32'd0;
                        bc          <= 2'd3;
                        sh          <= 32'd0;
                        fmt_code    <= 16'd0;
                        fmt_cnt     <= 5'd0;
                        bcnt        <= 3'd0;
                        out_pending <= 1'b0;
                        in_valid    <= 1'b0;
                        st          <= S_RIFF;
                    end
                end

                S_RIFF: if (byte_take) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], in_data};
                        bc <= bc - 2'd1;
                    end else if (tag_val != ID_RIFF) begin
                        error_code <= E_RIFF;
                        st         <= S_FAIL;
                    end else begin
                        bc <= 2'd3;
                        st <= S_RIFFSZ;
                    end
                end

                S_RIFFSZ: if (byte_take) begin
                    if (bc != 2'd0)
                        bc <= bc - 2'd1;
                    else begin
                        bc <= 2'd3;
                        st <= S_WAVE;
                    end
                end

                S_WAVE: if (byte_take) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], in_data};
                        bc <= bc - 2'd1;
                    end else if (tag_val != ID_WAVE) begin
                        error_code <= E_RIFF;
                        st         <= S_FAIL;
                    end else begin
                        bc <= 2'd3;
                        st <= S_CHUNK;
                    end
                end

                S_CHUNK: if (byte_take) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], in_data};
                        bc <= bc - 2'd1;
                    end else begin
                        chunk_id <= tag_val;
                        bc       <= 2'd3;
                        st       <= S_CSIZE;
                    end
                end

                S_CSIZE: if (byte_take) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], in_data};
                        bc <= bc - 2'd1;
                    end else begin
                        chunk_size <= chunk_val;
                        bc         <= 2'd3;
                        if (chunk_id == ID_FMT) begin
                            fmt_cnt    <= 5'd0;
                            fmt_target <= (chunk_val >= 32'd26) ? 5'd26 : 5'd16;
                            fmt_sh     <= 208'd0;
                            if (chunk_val < 32'd16) begin
                                error_code <= E_FMT;
                                st         <= S_FAIL;
                            end else
                                st <= S_FMT;
                        end else if (chunk_id == ID_DATA) begin
                            data_left <= chunk_val;
                            bcnt      <= 3'd0;
                            if (fmt_code == 16'd0)
                                begin
                                    error_code <= E_FMT;
                                    st         <= S_FAIL;
                                end
                            else if (chunk_val == 32'd0) begin
                                error_code <= E_EMPTY;
                                st         <= S_FAIL;
                            end else
                                st <= S_DATA;
                        end else begin
                            skip_left <= chunk_val + {31'd0, chunk_val[0]};
                            st        <= (chunk_val != 32'd0) ? S_SKIP : S_PAUSE;
                        end
                    end
                end

                S_FMT: if (byte_take) begin
                    if (fmt_cnt < fmt_target)
                        fmt_sh <= {fmt_sh[199:0], in_data};

                    if (fmt_cnt == (fmt_target - 5'd1)) begin
                        fmt_code <= fmt_use_in;
                        if (!fmt_bits_ok || !fmt_channels_ok) begin
                            error_code <= E_FMT;
                            st         <= S_FAIL;
                        end else begin
                            sample_rate <= fmt_rate_in;
                            channels    <= fmt_chan_in[7:0];
                            bits        <= fmt_bits_in[7:0];
                            skip_left   <= (chunk_size > fmt_target) ?
                                           ((chunk_size - fmt_target) +
                                            {31'd0, chunk_size[0]}) : 32'd0;
                            st          <= (chunk_size > fmt_target) ? S_SKIP : S_PAUSE;
                        end
                    end else begin
                        fmt_cnt <= fmt_cnt + 5'd1;
                    end
                end

                S_SKIP: if (byte_take) begin
                    skip_left <= skip_left - 32'd1;
                    if (skip_left == 32'd1)
                        st <= S_PAUSE;
                end

                S_PAUSE: st <= S_CHUNK;

                S_DATA: if (byte_take) begin
                    bcnt <= bcnt_next;
                    b0   <= (bcnt == 3'd0) ? in_data : b0;
                    b1   <= (bcnt == 3'd1) ? in_data : b1;
                    b2   <= (bcnt == 3'd2) ? in_data : b2;

                    if (take_sample) begin
                        out_pending <= 1'b1;
                        out_byte    <= fmt_is_ulaw ? in_data : pcm_code;
                    end

                    if (data_left <= 32'd1) begin
                        data_left <= 32'd0;
                        abort     <= 1'b1;
                        st        <= S_FINISH;
                    end else begin
                        data_left <= data_left - 32'd1;
                    end
                end

                S_FINISH: begin
                    if (!out_pending) begin
                        if (samples_out == 32'd0) begin
                            error_code <= E_EMPTY;
                            st         <= S_FAIL;
                        end else begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            in_valid <= 1'b0;
                            st   <= S_IDLE;
                        end
                    end
                end

                S_FAIL: begin
                    busy  <= 1'b0;
                    error <= 1'b1;
                    abort <= 1'b1;
                    in_valid <= 1'b0;
                    st    <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase

            if (stream_error && (st != S_IDLE) && (st != S_FINISH)) begin
                busy       <= 1'b0;
                error      <= 1'b1;
                error_code <= E_STREAM;
                abort      <= 1'b1;
                in_valid   <= 1'b0;
                st         <= S_IDLE;
            end else if (stream_done && (st != S_IDLE) && (st != S_FINISH)) begin
                if ((data_left > 32'd1) ||
                    ((data_left == 32'd1) && !in_valid)) begin
                    busy       <= 1'b0;
                    error      <= 1'b1;
                    error_code <= E_NODATA;
                    abort      <= 1'b1;
                    in_valid   <= 1'b0;
                    st         <= S_IDLE;
                end
            end
        end
    end

endmodule
