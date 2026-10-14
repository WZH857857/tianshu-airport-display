`timescale 1ns / 1ps
//=============================================================================
// wav_voice_loader.v - parse a RIFF/WAVE file and pack its samples into the
// voice bank
//
// The byte stream comes straight from fat32_file_reader: one byte per
// handshake, already limited to the selected file.
//
// accepted input
//   * RIFF/WAVE, little endian, chunk sizes honoured (fact, LIST and any other
//     chunk before or after the data chunk is skipped, odd sized chunks
//     include their pad byte)
//   * fmt chunk: WAVE_FORMAT_PCM (1), WAVE_FORMAT_MULAW (7) or
//     WAVE_FORMAT_EXTENSIBLE (0xFFFE) whose sub-format is one of the two
//   * 16 bit PCM or 8 bit (PCM or mu-law), mono or stereo.  Stereo is folded
//     down to mono, 16 bit samples are compressed to mu-law.
//
// The bank holds one byte per sample, so a file that does not fit is cut off
// at the end of the bank and truncated is raised instead of failing.
//=============================================================================
module wav_voice_loader #(
    parameter integer BANK_WORDS = 65536,
    parameter integer AW         = 16
)(
    input  wire          clk,
    input  wire          rst_n,

    // control ---------------------------------------------------------
    input  wire          start,          // one cycle: load the current file
    input  wire          stream_done,    // file stream reached the last byte
    input  wire          stream_error,   // file stream failed

    // byte stream from the card --------------------------------------
    input  wire [7:0]    data,
    input  wire          data_valid,
    output wire          data_ready,
    output reg           abort,          // stop the file stream early

    // voice bank write port -------------------------------------------
    output wire          wr_en,
    output wire [AW-1:0] wr_addr,
    output wire [7:0]    wr_data,

    // status -----------------------------------------------------------
    output reg           busy,
    output reg           done,
    output reg           error,
    output reg  [3:0]    error_code,
    output reg  [31:0]   sample_rate,
    output reg  [7:0]    channels,
    output reg  [7:0]    bits,
    output reg  [AW:0]   words_stored,
    output reg           truncated,
    output reg  [5:0]    dbg_state
);

    //------------------------------------------------------------------
    // states
    //------------------------------------------------------------------
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
                     // one cycle that does not take a byte, used at the end of
                     // a skipped chunk so the next chunk id is not eaten
                     S_PAUSE  = 6'd11;

    // error codes
    localparam [3:0] E_RIFF   = 4'd1,    // not a RIFF/WAVE file
                     E_FMT    = 4'd2,    // fmt chunk missing or unsupported
                     E_NODATA = 4'd3,    // data chunk missing
                     E_EMPTY  = 4'd4,    // data chunk held no usable samples
                     E_STREAM = 4'd5;    // card stream failed

    localparam [31:0] ID_RIFF = 32'h5249_4646,   // 'RIFF'
                      ID_WAVE = 32'h5741_5645,   // 'WAVE'
                      ID_FMT  = 32'h666D_7420,   // 'fmt '
                      ID_DATA = 32'h6461_7461;   // 'data'

    reg [5:0]   st;
    reg [1:0]   bc;              // byte index inside a four byte field
    reg [31:0]  sh;              // four byte field being assembled
    reg [31:0]  chunk_id;
    reg [31:0]  chunk_size;
    reg [31:0]  skip_left;
    reg [31:0]  data_left;
    reg [4:0]   fmt_cnt;
    reg [4:0]   fmt_target;
    reg [207:0] fmt_sh;
    reg [2:0]   bcnt;            // byte index inside one sample
    reg [7:0]   b0, b1, b2;      // sample bytes seen so far

    reg [15:0]  fmt_code;        // effective WAVE format code
    wire        fmt_is_ulaw = (fmt_code == 16'h0007);

    wire [AW:0] stored_full = BANK_WORDS;
    wire        bank_full   = (words_stored >= stored_full);

    // The four byte value that is arriving right now, used to pick the chunk
    // size and its pad byte in the same cycle the last byte lands.  The id
    // fields are kept in the order they arrive on the wire, so a little endian
    // number has to be put back together low byte first.
    wire [31:0] tag_val     = {sh[23:0], data};
    wire [31:0] chunk_val   = {data, sh[7:0], sh[15:8], sh[23:16]};

    //------------------------------------------------------------------
    // sample assembly
    //------------------------------------------------------------------
    // 8 bit PCM is unsigned, 8 bit mu-law is already in the stored form
    wire signed [15:0] pcm_from_u8 =
        ($signed({1'b0, data}) - 16'sd128) <<< 8;

    // little endian 16 bit: b0 is the low byte of the left channel
    wire signed [15:0] pcm_mono = $signed({data, b0});
    wire signed [15:0] ch_left  = $signed({b1, b0});
    wire signed [15:0] ch_right = $signed({data, b2});
    wire signed [15:0] pcm_stereo = (ch_left + ch_right) >>> 1;

    wire signed [15:0] pcm_in = (bits == 8'd8) ? pcm_from_u8 :
                                ((channels == 8'd2) ? pcm_stereo : pcm_mono);

    // the sample is complete on the last byte of the frame
    wire        take_sample = (bits == 8'd8) ?
                              ((channels == 8'd2) ? (bcnt == 3'd0) : 1'b1) :
                              ((channels == 8'd2) ? (bcnt == 3'd3) : (bcnt == 3'd1));

    // one frame is 1, 2 or 4 bytes, so the counter needs three bits
    wire [2:0]  frame_bytes = (bits == 8'd8) ?
                              ((channels == 8'd2) ? 3'd2 : 3'd1) :
                              ((channels == 8'd2) ? 3'd4 : 3'd2);
    wire [2:0]  bcnt_plus   = {1'b0, bcnt} + 3'd1;
    wire [2:0]  bcnt_next   = (bcnt_plus >= frame_bytes) ? 3'd0 : bcnt_plus;

    //------------------------------------------------------------------
    // write pipeline
    //
    // The sector read path off the card and the mu-law compressor both sit on
    // the 50 MHz system clock, and putting them in one clock period made a 25
    // level path from the FAT32 state register into the block RAM data pins.
    // The assembled sample is registered here instead and written on the
    // following cycle, which splits that path in two without changing the
    // byte handshake or the write throughput.
    //------------------------------------------------------------------
    reg signed [15:0] wr_pcm;      // linear sample waiting to be encoded
    reg        [7:0]  wr_raw;      // byte of a file that is already mu-law
    reg               wr_is_ulaw;
    reg        [AW-1:0] wr_addr_q;
    reg               wr_pending;  // the write stage holds a sample right now

    wire [7:0] enc_code;

    ulaw_encode u_encode (
        .pcm  (wr_pcm),
        .code (enc_code)
    );

    // a sample is accepted on the last byte of its frame, and only while the
    // bank still has room for it
    wire wr_take = busy && (st == S_DATA) && data_valid && take_sample &&
                   !bank_full && !error;

    //------------------------------------------------------------------
    // fmt chunk fields, seen as one 26 byte value.  The byte that arrives
    // while fmt_cnt counts down to zero is the last one of the header, so it
    // is shifted in front of the register before the fields are sliced out.
    // For a plain 16 byte fmt chunk the extension bytes read back as zero,
    // which is harmless because the sub format is only looked at when the
    // format code says WAVE_FORMAT_EXTENSIBLE.
    //------------------------------------------------------------------
    // The core header is 16 bytes and the extensible one is 26, so the bytes
    // that have arrived are left justified before the fields are sliced out:
    // a plain fmt chunk then reads its extension as zero.
    wire [127:0] fmt_val_16  = {fmt_sh[119:0], data};
    wire [207:0] fmt_val_26  = {fmt_sh[199:0], data};
    wire [207:0] fmt_val     = (fmt_target == 5'd16) ? {fmt_val_16, 80'd0}
                                                     : fmt_val_26;
    // the header fields are little endian too, so every multi byte field is
    // put back together low byte first
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

    assign wr_en    = wr_pending;
    assign wr_addr  = wr_addr_q;
    assign wr_data  = wr_is_ulaw ? wr_raw : enc_code;
    assign data_ready = busy && (st != S_PAUSE);

    //------------------------------------------------------------------
    // main sequencer
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st           <= S_IDLE;
            bc           <= 2'd0;
            sh           <= 32'd0;
            chunk_id     <= 32'd0;
            chunk_size   <= 32'd0;
            skip_left    <= 32'd0;
            data_left    <= 32'd0;
            fmt_cnt      <= 5'd0;
            fmt_target   <= 5'd16;
            fmt_sh       <= 208'd0;
            bcnt         <= 3'd0;
            b0           <= 8'd0;
            b1           <= 8'd0;
            b2           <= 8'd0;
            fmt_code     <= 16'd0;
            abort        <= 1'b0;
            busy         <= 1'b0;
            done         <= 1'b0;
            error        <= 1'b0;
            error_code   <= 4'd0;
            sample_rate  <= 32'd0;
            channels     <= 8'd0;
            bits         <= 8'd0;
            words_stored <= {(AW+1){1'b0}};
            truncated    <= 1'b0;
            dbg_state    <= S_IDLE;
            wr_pcm       <= 16'sd0;
            wr_raw       <= 8'd0;
            wr_is_ulaw   <= 1'b0;
            wr_addr_q    <= {AW{1'b0}};
            wr_pending   <= 1'b0;
        end else begin
            abort     <= 1'b0;
            done      <= 1'b0;
            dbg_state <= st;

            // the write stage never stalls: whatever it holds is committed to
            // the bank at the end of this cycle
            wr_pending <= 1'b0;

            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy         <= 1'b1;
                        error        <= 1'b0;
                        error_code   <= 4'd0;
                        truncated    <= 1'b0;
                        words_stored <= {(AW+1){1'b0}};
                        bc           <= 2'd3;
                        sh           <= 32'd0;
                        fmt_code     <= 16'd0;
                        fmt_cnt      <= 5'd0;
                        bcnt         <= 3'd0;
                        wr_pending   <= 1'b0;
                        st           <= S_RIFF;
                    end
                end

                // 'RIFF' <size> 'WAVE'
                S_RIFF: if (data_valid) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], data};
                        bc <= bc - 2'd1;
                    end else begin
                        if (tag_val != ID_RIFF) begin
                            error_code <= E_RIFF;
                            st         <= S_FAIL;
                        end else begin
                            bc <= 2'd3;
                            st <= S_RIFFSZ;
                        end
                    end
                end

                S_RIFFSZ: if (data_valid) begin
                    if (bc != 2'd0)
                        bc <= bc - 2'd1;
                    else begin
                        bc <= 2'd3;
                        st <= S_WAVE;
                    end
                end

                S_WAVE: if (data_valid) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], data};
                        bc <= bc - 2'd1;
                    end else begin
                        if (tag_val != ID_WAVE) begin
                            error_code <= E_RIFF;
                            st         <= S_FAIL;
                        end else begin
                            bc <= 2'd3;
                            st <= S_CHUNK;
                        end
                    end
                end

                // <chunk id> <chunk size>
                S_CHUNK: if (data_valid) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], data};
                        bc <= bc - 2'd1;
                    end else begin
                        chunk_id <= tag_val;
                        bc       <= 2'd3;
                        st       <= S_CSIZE;
                    end
                end

                S_CSIZE: if (data_valid) begin
                    if (bc != 2'd0) begin
                        sh <= {sh[23:0], data};
                        bc <= bc - 2'd1;
                    end else begin
                        chunk_size <= chunk_val;
                        bc         <= 2'd3;
                        if (chunk_id == ID_FMT) begin
                            // 16 byte core header, plus the 10 byte extension
                            // of WAVE_FORMAT_EXTENSIBLE when it is present
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
                            if (fmt_code == 16'd0) begin
                                error_code <= E_FMT;
                                st         <= S_FAIL;
                            end else
                                st <= S_DATA;
                        end else begin
                            // the body plus the pad byte an odd size owes;
                            // nothing to skip goes straight to the pause
                            skip_left <= chunk_val + {31'd0, chunk_val[0]};
                            st        <= (chunk_val != 32'd0) ? S_SKIP :
                                                                 S_PAUSE;
                        end
                    end
                end

                // fmt chunk body
                S_FMT: if (data_valid) begin
                    if (fmt_cnt < fmt_target)
                        fmt_sh <= {fmt_sh[199:0], data};

                    if (fmt_cnt == (fmt_target - 5'd1)) begin
                        // fields: format[207:192] channels[191:176]
                        //         rate[175:144] bits[95:80] subfmt[15:0]
                        fmt_code <= fmt_use_in;
                        if (!fmt_bits_ok) begin
                            error_code <= E_FMT;
                            st         <= S_FAIL;
                        end else if ((fmt_chan_in != 16'd1) &&
                                     (fmt_chan_in != 16'd2)) begin
                            error_code <= E_FMT;
                            st         <= S_FAIL;
                        end else begin
                            sample_rate <= fmt_rate_in;
                            channels    <= fmt_chan_in[7:0];
                            bits        <= fmt_bits_in[7:0];
                            // a fmt chunk may carry more than the core header
                            skip_left   <= (chunk_size > fmt_target) ?
                                           ((chunk_size - fmt_target) +
                                            {31'd0, chunk_size[0]}) : 32'd0;
                            st          <= (chunk_size > fmt_target) ? S_SKIP :
                                                                       S_PAUSE;
                        end
                    end else begin
                        fmt_cnt <= fmt_cnt + 5'd1;
                    end
                end

                // discard a chunk we do not use; the count already includes the
                // pad byte an odd size owes
                S_SKIP: if (data_valid) begin
                    skip_left <= skip_left - 32'd1;
                    if (skip_left == 32'd1)
                        st <= S_PAUSE;
                end

                // the byte on the wire now belongs to the next field, so this
                // cycle does not take one
                S_PAUSE: st <= S_CHUNK;

                // data chunk body
                S_DATA: if (data_valid) begin
                    if (wr_take) begin
                        wr_pending   <= 1'b1;
                        wr_is_ulaw   <= fmt_is_ulaw;
                        wr_raw       <= data;
                        wr_pcm       <= pcm_in;
                        wr_addr_q    <= words_stored[AW-1:0];
                        words_stored <= words_stored + {{AW{1'b0}}, 1'b1};
                    end

                    bcnt <= bcnt_next;
                    b0   <= (bcnt == 3'd0) ? data : b0;
                    b1   <= (bcnt == 3'd1) ? data : b1;
                    b2   <= (bcnt == 3'd2) ? data : b2;

                    if (data_left <= 32'd1) begin
                        // last byte of the declared data chunk
                        data_left <= 32'd0;
                        abort     <= 1'b1;
                        st        <= S_FINISH;
                    end else begin
                        data_left <= data_left - 32'd1;
                        if (take_sample && bank_full) begin
                            // the bank is full: keep what we have
                            truncated <= 1'b1;
                            abort     <= 1'b1;
                            st        <= S_FINISH;
                        end
                    end
                end

                S_FINISH: begin
                    if (wr_pending) begin
                        // the last sample still has to reach the bank
                    end else if (words_stored == {(AW+1){1'b0}}) begin
                        error_code <= E_EMPTY;
                        st         <= S_FAIL;
                    end else begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        st   <= S_IDLE;
                    end
                end

                S_FAIL: begin
                    busy       <= 1'b0;
                    error      <= 1'b1;
                    abort      <= 1'b1;
                    st         <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase

            // the card stream can also end on its own, either because the file
            // is shorter than its headers claim or because the directory entry
            // was the whole file
            if (stream_error && (st != S_IDLE) && (st != S_FINISH)) begin
                busy       <= 1'b0;
                st         <= S_IDLE;
                if (words_stored == {(AW+1){1'b0}}) begin
                    error      <= 1'b1;
                    error_code <= E_STREAM;
                end else begin
                    done <= 1'b1;
                end
            end else if (stream_done && (st != S_IDLE)) begin
                if ((words_stored != {(AW+1){1'b0}}) && (st == S_DATA)) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= S_IDLE;
                end else if (st == S_FINISH) begin
                    // the data chunk ended exactly with the file, the normal
                    // finish path already did the accounting
                end else begin
                    busy       <= 1'b0;
                    error      <= 1'b1;
                    error_code <= E_NODATA;
                    st         <= S_IDLE;
                end
            end
        end
    end

endmodule
