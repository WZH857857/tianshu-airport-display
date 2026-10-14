`timescale 1ns / 1ps
//=============================================================================
// bmp_stream_parser.v - turn a BMP byte stream into a 24 bit pixel stream
//
// Accepted format (the format the player advertises for the picture folder):
//   * BITMAPFILEHEADER + BITMAPINFOHEADER, magic "BM"
//   * IMG_W x IMG_H pixels, 24 bits per pixel, BI_RGB (no compression)
//   * bottom up row order, which is the BMP default (positive height)
//
// The parser consumes one byte per handshake from the FAT reader, drops the
// header and the per row padding, and presents blue-green-red triples as
// red-green-blue pixels. Rows are emitted in file order, that is the bottom
// line of the picture first; the frame store maps them back to the right
// memory rows.
//
// SCALE_EN = 1 switches the size check to "anything inside the limits".  The
// picture keeps its own width and height, the row padding is derived from that
// width, and a scaler downstream brings the stream back to the display
// geometry.  hold_done lets that scaler delay the done pulse: it stays high
// while the scaler is still emitting output rows, so the frame store is not
// disabled while the last rows are on their way.
//
// error_code
//   1 magic is not BM        5 height is not IMG_H (or out of range)
//   2 bits per pixel is not 24  6 pixel data offset is too small
//   3 compression is not BI_RGB 7 file ended before the last pixel
//   4 width is not IMG_W (or out of range)  8 the card stopped delivering bytes
//                             9 height is negative (top down BMP)
//=============================================================================
module bmp_stream_parser #(
    parameter integer IMG_W = 640,
    parameter integer IMG_H = 480,
    // 0 = only IMG_W x IMG_H is accepted (the historical behaviour)
    // 1 = any size inside MAX_W x MAX_H is accepted, a scaler follows
    parameter integer SCALE_EN = 0,
    parameter integer MAX_W    = 1920,
    parameter integer MAX_H    = 2048
)(
    input  wire        clk,
    input  wire        rst_n,

    // file to parse: level of busy, start pulse, size in bytes
    input  wire        start,
    input  wire [31:0] file_size,
    output reg         busy,
    output reg         done,
    output reg         error,
    output reg  [7:0]  error_code,

    // byte stream from the FAT reader
    input  wire [7:0]  data,
    input  wire        data_valid,
    output wire        data_ready,

    // 1 = hold the done pulse: a downstream scaler is still emitting the
    //     output rows of this picture
    input  wire        hold_done,

    // pixel stream to the frame store loader
    output reg         px_valid,
    output reg  [23:0] px_rgb,
    output reg         px_last,
    input  wire        px_ready,

    // decoded header, kept for diagnostics and for the on screen status
    output reg  [15:0] hdr_width,
    output reg  [15:0] hdr_height,
    output reg  [15:0] hdr_bpp,
    output reg  [31:0] hdr_data_offset
);

    localparam [3:0] S_IDLE  = 4'd0,
                     S_HDR   = 4'd1,
                     S_CHECK = 4'd2,
                     S_SKIP  = 4'd3,
                     S_PIX   = 4'd4,
                     S_DONE  = 4'd5,
                     S_FAIL  = 4'd6,
                     S_HOLD  = 4'd7;

    localparam integer ROW_PIX_BYTES = 3 * IMG_W;
    localparam integer ROW_PAD       = (4 - (ROW_PIX_BYTES % 4)) % 4;
    localparam integer ROW_BYTES     = ROW_PIX_BYTES + ROW_PAD;

    localparam [31:0] IMG_W32     = IMG_W;
    localparam [31:0] IMG_H32     = IMG_H;
    localparam [31:0] MAX_W32     = MAX_W;
    localparam [31:0] MAX_H32     = MAX_H;
    localparam [1:0]  ROW_PAD_VAL = ROW_PAD;

    localparam [23:0] STALL_LIMIT = 24'hFF_FFFF;

    reg [3:0]  state;
    reg [7:0]  byte_idx;        // header byte index, 0..33
    reg [31:0] skip_left;
    reg [15:0] row_idx;
    reg [11:0] col_idx;
    reg [1:0]  pix_byte;        // 0 = blue, 1 = green, 2 = red
    reg [1:0]  pad_idx;
    reg [31:0] consumed;
    reg [23:0] stall_cnt;

    reg [7:0]  magic0, magic1;
    reg [7:0]  b0, b1, b2, b3;
    reg [31:0] data_offset;
    reg [31:0] dib_size;
    reg [31:0] width;
    reg [31:0] height;
    reg [15:0] planes;
    reg [15:0] bpp;
    reg [31:0] compression;
    reg [15:0] src_w;           // geometry of the file being parsed
    reg [15:0] src_h;
    reg [1:0]  row_pad;         // (3 * src_w) mod 4 == src_w mod 4

    wire [7:0] pixel_byte = (pix_byte == 2'd0) ? b0 :
                            (pix_byte == 2'd1) ? b1 : b2;

    // the reader promises exactly file_size bytes, so running out of bytes
    // before the last pixel means the file is shorter than the header claims
    wire file_exhausted = (file_size != 32'd0) && (consumed >= file_size);

    // data_ready is only asserted in a state that really consumes a byte,
    // otherwise the FAT reader would advance and the byte would be lost
    wire in_skip = (state == S_SKIP) && (skip_left != 32'd0);

    assign data_ready = busy && !px_valid &&
                        ((state == S_HDR) || (state == S_PIX) || in_skip);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            error        <= 1'b0;
            error_code   <= 8'd0;
            byte_idx     <= 8'd0;
            skip_left    <= 32'd0;
            row_idx      <= 16'd0;
            col_idx      <= 12'd0;
            pix_byte     <= 2'd0;
            pad_idx      <= 2'd0;
            consumed     <= 32'd0;
            stall_cnt    <= 24'd0;
            px_valid     <= 1'b0;
            px_rgb       <= 24'd0;
            px_last      <= 1'b0;
            magic0       <= 8'd0;
            magic1       <= 8'd0;
            b0           <= 8'd0;
            b1           <= 8'd0;
            b2           <= 8'd0;
            b3           <= 8'd0;
            data_offset  <= 32'd0;
            dib_size     <= 32'd0;
            width        <= 32'd0;
            height       <= 32'd0;
            planes       <= 16'd0;
            bpp          <= 16'd0;
            compression  <= 32'd0;
            src_w        <= 16'd0;
            src_h        <= 16'd0;
            row_pad      <= 2'd0;
            hdr_width    <= 16'd0;
            hdr_height   <= 16'd0;
            hdr_bpp      <= 16'd0;
            hdr_data_offset <= 32'd0;
        end else begin
            done <= 1'b0;

            // the pixel handshake has priority: the byte stream is stalled
            // until the frame store has taken the pixel
            if (px_valid && px_ready)
                px_valid <= 1'b0;

            // A hard KEY1 skip can restart the file stream while this parser
            // is still in the middle of the previous BMP.  A start pulse must
            // therefore restart the parser from a clean header state even
            // when busy, otherwise the next file is decoded from stale row,
            // column and byte counters.
            if (start) begin
                busy       <= 1'b1;
                error      <= 1'b0;
                error_code <= 8'd0;
                byte_idx   <= 8'd0;
                skip_left  <= 32'd0;
                consumed   <= 32'd0;
                row_idx    <= 16'd0;
                col_idx    <= 12'd0;
                pix_byte   <= 2'd0;
                pad_idx    <= 2'd0;
                stall_cnt  <= 24'd0;
                px_valid   <= 1'b0;
                px_last    <= 1'b0;
                src_w      <= 16'd0;
                src_h      <= 16'd0;
                row_pad    <= 2'd0;
                state      <= S_HDR;
            end else begin
                case (state)
                    S_IDLE: begin
                        stall_cnt <= 24'd0;
                    end

                S_HDR: begin
                    if (data_valid && data_ready) begin
                        // little endian fields, low byte first
                        case (byte_idx)
                            8'd0: magic0 <= data;
                            8'd1: magic1 <= data;
                            8'd10: data_offset[7:0]   <= data;
                            8'd11: data_offset[15:8]  <= data;
                            8'd12: data_offset[23:16] <= data;
                            8'd13: data_offset[31:24] <= data;
                            8'd14: dib_size[7:0]      <= data;
                            8'd15: dib_size[15:8]     <= data;
                            8'd16: dib_size[23:16]    <= data;
                            8'd17: dib_size[31:24]    <= data;
                            8'd18: width[7:0]         <= data;
                            8'd19: width[15:8]        <= data;
                            8'd20: width[23:16]       <= data;
                            8'd21: width[31:24]       <= data;
                            8'd22: height[7:0]        <= data;
                            8'd23: height[15:8]       <= data;
                            8'd24: height[23:16]      <= data;
                            8'd25: height[31:24]      <= data;
                            8'd26: planes[7:0]        <= data;
                            8'd27: planes[15:8]       <= data;
                            8'd28: bpp[7:0]           <= data;
                            8'd29: bpp[15:8]          <= data;
                            8'd30: compression[7:0]   <= data;
                            8'd31: compression[15:8]  <= data;
                            8'd32: compression[23:16] <= data;
                            8'd33: compression[31:24] <= data;
                            default: ;
                        endcase

                        consumed <= consumed + 32'd1;
                        if (byte_idx == 8'd33) begin
                            state           <= S_CHECK;
                        end else begin
                            byte_idx <= byte_idx + 8'd1;
                        end
                    end else if (!data_valid) begin
                        if (stall_cnt == STALL_LIMIT) begin
                            error_code <= 8'd8;
                            state      <= S_FAIL;
                        end else
                            stall_cnt <= stall_cnt + 24'd1;
                    end
                end

                S_CHECK: begin
                    // the header registers settled with the last byte, so the
                    // decoded copy is taken here, one cycle later
                    hdr_width       <= width[15:0];
                    hdr_height      <= height[15:0];
                    hdr_bpp         <= bpp;
                    hdr_data_offset <= data_offset;

                    if ((magic0 != 8'h42) || (magic1 != 8'h4D)) begin
                        error_code <= 8'd1;
                        state      <= S_FAIL;
                    end else if (bpp != 16'd24) begin
                        error_code <= 8'd2;
                        state      <= S_FAIL;
                    end else if (compression != 32'd0) begin
                        error_code <= 8'd3;
                        state      <= S_FAIL;
                    end else if ((SCALE_EN == 0) && (width != IMG_W32)) begin
                        error_code <= 8'd4;
                        state      <= S_FAIL;
                    end else if ((SCALE_EN != 0) &&
                                 ((width == 32'd0) || (width > MAX_W32))) begin
                        error_code <= 8'd4;
                        state      <= S_FAIL;
                    end else if (height[31] == 1'b1) begin
                        // top down BMP: not supported, the row order would
                        // have to be reversed
                        error_code <= 8'd9;
                        state      <= S_FAIL;
                    end else if ((SCALE_EN == 0) && (height != IMG_H32)) begin
                        error_code <= 8'd5;
                        state      <= S_FAIL;
                    end else if ((SCALE_EN != 0) &&
                                 ((height == 32'd0) || (height > MAX_H32))) begin
                        error_code <= 8'd5;
                        state      <= S_FAIL;
                    end else if (data_offset < 32'd34) begin
                        error_code <= 8'd6;
                        state      <= S_FAIL;
                    end else begin
                        // bottom up and inside the limits: this is the picture
                        // that will be parsed, padding included
                        src_w     <= width[15:0];
                        src_h     <= height[15:0];
                        row_pad   <= width[1:0];
                        skip_left <= data_offset - 32'd34;
                        state     <= S_SKIP;
                    end
                end

                S_SKIP: begin
                    if (skip_left == 32'd0) begin
                        state <= S_PIX;
                    end else if (data_valid && data_ready) begin
                        consumed  <= consumed + 32'd1;
                        skip_left <= skip_left - 32'd1;
                    end else if (!data_valid) begin
                        if (file_exhausted) begin
                            error_code <= 8'd7;
                            state      <= S_FAIL;
                        end else if (stall_cnt == STALL_LIMIT) begin
                            error_code <= 8'd8;
                            state      <= S_FAIL;
                        end else
                            stall_cnt <= stall_cnt + 24'd1;
                    end
                end

                S_PIX: begin
                    if (pad_idx != 2'd0) begin
                        // row padding, one byte at a time
                        if (data_valid && data_ready) begin
                            consumed <= consumed + 32'd1;
                            pad_idx  <= pad_idx - 2'd1;
                        end else if (!data_valid) begin
                            if (file_exhausted) begin
                                error_code <= 8'd7;
                                state      <= S_FAIL;
                            end else if (stall_cnt == STALL_LIMIT) begin
                                error_code <= 8'd8;
                                state      <= S_FAIL;
                            end else
                                stall_cnt <= stall_cnt + 24'd1;
                        end
                    end else if (px_valid) begin
                        stall_cnt <= 24'd0;
                        if (px_ready) begin
                            if ((row_idx == (src_h - 16'd1)) &&
                                (col_idx == (src_w - 16'd1))) begin
                                // last pixel of the picture: a downstream
                                // scaler may still have output rows to emit
                                state <= hold_done ? S_HOLD : S_DONE;
                            end else if (col_idx == (src_w - 16'd1)) begin
                                col_idx   <= 12'd0;
                                row_idx   <= row_idx + 16'd1;
                                pix_byte  <= 2'd0;
                                pad_idx   <= (SCALE_EN == 0) ? ROW_PAD_VAL
                                                             : row_pad;
                            end else begin
                                col_idx  <= col_idx + 12'd1;
                                pix_byte <= 2'd0;
                            end
                        end
                    end else if (data_valid && data_ready) begin
                        consumed <= consumed + 32'd1;
                        stall_cnt <= 24'd0;
                        case (pix_byte)
                            2'd0: b0 <= data;
                            2'd1: b1 <= data;
                            default: b2 <= data;
                        endcase
                        if (pix_byte == 2'd2) begin
                            px_rgb   <= {data, b1, b0};
                            px_last  <= (row_idx == (src_h - 16'd1)) &&
                                        (col_idx == (src_w - 16'd1));
                            px_valid <= 1'b1;
                        end else
                            pix_byte <= pix_byte + 2'd1;
                    end else if (!data_valid) begin
                        if (file_exhausted) begin
                            error_code <= 8'd7;
                            state      <= S_FAIL;
                        end else if (stall_cnt == STALL_LIMIT) begin
                            error_code <= 8'd8;
                            state      <= S_FAIL;
                        end else
                            stall_cnt <= stall_cnt + 24'd1;
                    end
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                S_HOLD: begin
                    // The picture has been fully parsed but the scaler is
                    // still emitting the last output rows.  Nothing may be
                    // consumed here: data_ready stays low, so the trailing row
                    // padding and the next file are left alone.
                    if (!hold_done)
                        state <= S_DONE;
                end

                S_FAIL: begin
                    busy       <= 1'b0;
                    error      <= 1'b1;
                    px_valid   <= 1'b0;
                    state      <= S_IDLE;
                end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
