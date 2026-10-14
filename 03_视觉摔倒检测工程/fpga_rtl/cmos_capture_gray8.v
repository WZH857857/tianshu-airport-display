//===========================================================================
// cmos_capture_gray8.v -- MT9M001 parallel gray -> 640x480 "RGB565" stream
//
// The MT9M001 (ID-003A module) drives an 8-bit gray DVP bus: LINE_VALID high
// for the pixels of a row, FRAME_VALID high for the rows of a frame, and data
// that is valid on the falling edge of PIXCLK (datasheet Table 3).  Because
// the board drives XCLK from the 24 MHz oscillator, PIXCLK is 24 MHz, so this
// module runs in the falling-edge domain and hands the pixel stream to the
// vision core, which is clocked on the rising edge of the same clock.
//
// Pixel format handed downstream: the vision core and the display both take
// {R[4:0], G[5:0], B[4:0]}, so the gray byte is replicated into the three
// channels as {g[7:3], g[7:2], g[7:3]}.  gray_front.v recovers 8-bit gray
// with (77*R8 + 150*G8 + 29*B8) >> 8 and 77+150+29 = 256, so replicating the
// byte that way makes gray_front an exact identity -- the whole front end
// behind this point stays bit-identical to the RGB565 path that every
// existing check was written against, and no separate gray front end has to
// be re-verified.
//
// The sensor's skip configuration (reg 0x20 = 0x0018) makes it emit 640x512:
// half of the 1280 columns and half of the 1024 rows, i.e. the whole field of
// view at half resolution rather than a cropped corner.  480 of the 512 rows
// are kept (16 dropped top and bottom) so the core sees exactly 640x480.
//
// If the two-wire configuration did not take effect -- wrong module, an
// unpowered bus, a sensor that kept its factory window -- the part still
// streams 1280x1024 and the same module keeps working: it measures the lines
// and rows of the frames before `arm`, latches `wide_mode`, and from then on
// decimates 2:1 in both directions (1280/2 = 640 columns, rows 32..990 step 2
// = 480).  The demo then runs at ~15 fps instead of ~33 fps, but it runs.
//
// Contract downstream (identical to cmos_capture_rgb565.v):
//   de      : one pclk cycle per assembled pixel, valid in that same cycle
//   pix     : {R[4:0], G[5:0], B[4:0]}, row order top->bottom
//   frame_start : single pclk pulse on the FRAME_VALID leading edge
//   frame_end   : single pclk pulse on the FRAME_VALID trailing edge, i.e. in
//                 the blanking after the last row.  ae_ctrl releases a freeze
//                 on this edge so the next frame_start pulse still reaches
//                 the vision core.
//
// hw_err is sticky and decoded as:
//   [0] a streamed line did not contain the expected number of pixels
//   [1] a streamed frame did not contain the expected number of rows
//   [2] FRAME_VALID moved while LINE_VALID was active (protocol violation)
//   [3] a line arrived with a width that is neither 640 nor 1280
//===========================================================================
`timescale 1ns / 1ps

module cmos_capture_gray8 #(
    parameter integer NARROW_W    = 640,     // pixels per line with skip on
    parameter integer NARROW_H    = 512,     // rows per frame with skip on
    parameter integer WIDE_W      = 1280,    // pixels per line, factory window
    parameter integer WIDE_H      = 1024,    // rows per frame, factory window
    parameter integer OUT_W       = 640,     // pixels handed to the core
    parameter integer OUT_H       = 480,     // rows handed to the core
    parameter integer WAIT_FRAMES = 10
) (
    input  wire       pclk,
    input  wire       rst_n,        // async, level, valid in both domains
    input  wire       arm,          // cfg_done synchronised into pclk

    input  wire       vsync,        // FRAME_VALID, active high
    input  wire       href,         // LINE_VALID,  active high
    input  wire [7:0] data,

    output reg        de,
    output reg  [15:0] pix,
    output reg  [7:0] gray_out,     // same pixel as the raw 8-bit sensor byte
    output reg        frame_start,
    output reg        frame_end,
    output reg        armed,
    output reg        wide_mode,    // 1 = sensor ignored the skip config

    output reg  [15:0] frame_cnt,
    output reg  [10:0] line_cnt,
    output reg  [11:0] pix_cnt,
    output reg  [7:0]  hw_err
);

    // Row window in the narrow (skip) stream: 16..495 of 512 -> 480 rows.
    localparam integer N_TOP = (NARROW_H - OUT_H) / 2;          // 16
    // Row window in the wide (factory) stream, centred like the narrow one:
    // with 1280x1024 that is 32..990 step 2 -> 480 rows.
    localparam integer W_TOP = (WIDE_H - 2 * OUT_H) / 2;        // 32
    localparam integer W_END = W_TOP + 2 * (OUT_H - 1);         // 990

    reg vsync_d0, vsync_d1;
    reg href_d0,  href_fall_d;
    reg [10:0] col_in;      // pixel index inside the current line
    reg [10:0] row_in;      // row index inside the current frame
    reg [5:0]  wait_cnt;
    reg [10:0] meas_pix;    // pixels seen in the line before `arm`
    reg [10:0] meas_rows;   // rows seen in the frame before `arm`
    reg [9:0]  out_col;     // kept pixels so far this line
    reg [9:0]  out_row;     // kept rows so far this frame

    wire vsync_rise =  vsync_d0 & ~vsync_d1;
    wire vsync_fall = ~vsync_d0 &  vsync_d1;
    wire href_rise  =  href & ~href_d0;
    wire href_fall  = ~href &  href_d0;
    wire will_arm   =  armed || (wait_cnt == WAIT_FRAMES);

    // Row and column keep-rules.  Both branches end up with OUT_H rows and
    // OUT_W columns per frame, which is what the vision core expects.
    wire row_keep = wide_mode
                  ? (row_in >= W_TOP) && (row_in <= W_END) && ~row_in[0]
                  : (row_in >= N_TOP) && (row_in < N_TOP + OUT_H);
    wire col_keep = wide_mode ? ~col_in[0] : 1'b1;

    // Belt and braces: a line that is longer than expected must not push
    // extra pixels at the core, which counts pixels to find its row edge.
    wire emit = href && row_keep && col_keep && (out_col < OUT_W);

    // {R[4:0], G[5:0], B[4:0]} with the gray byte in every channel.
    wire [15:0] gray_565 = {data[7:3], data[7:2], data[7:3]};

    always @(negedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d0    <= 1'b0;
            vsync_d1    <= 1'b0;
            href_d0     <= 1'b0;
            href_fall_d <= 1'b0;
            col_in      <= 11'd0;
            row_in      <= 11'd0;
            wait_cnt    <= 6'd0;
            meas_pix    <= 11'd0;
            meas_rows   <= 11'd0;
            out_col     <= 10'd0;
            out_row     <= 10'd0;
            de          <= 1'b0;
            gray_out    <= 8'd0;
            pix         <= 16'd0;
            frame_start <= 1'b0;
            frame_end   <= 1'b0;
            armed       <= 1'b0;
            wide_mode   <= 1'b0;
            frame_cnt   <= 16'd0;
            line_cnt    <= 11'd0;
            pix_cnt     <= 12'd0;
            hw_err      <= 8'd0;
        end else begin
            vsync_d0 <= vsync;
            vsync_d1 <= vsync_d0;
            href_d0  <= href;

            de          <= 1'b0;
            frame_start <= 1'b0;
            frame_end   <= 1'b0;
            href_fall_d <= href_fall;

            // ------------------------------------------- pixel beats
            if (href) begin
                col_in <= col_in + 11'd1;
                // Line census: one beat per PIXCLK on which LINE_VALID is
                // high, i.e. R4eff + 1 = 640 with column skip on and 1280
                // with the factory window.  Read and cleared on href_fall.
                meas_pix <= meas_pix + 11'd1;
                if (armed) begin
                    if (emit) begin
                        gray_out <= data;
                        pix     <= gray_565;
                        de      <= 1'b1;
                        out_col <= out_col + 10'd1;
                    end
                end
            end

            // ------------------------------------------- line and frame ends
            // The column counters (and the line's pixel census) restart at
            // the line boundary, not at the leading edge: the leading edge is
            // itself the first pixel beat, so clearing there would emit
            // column 0 twice.
            if (href_fall) begin
                pix_cnt  <= {1'b0, meas_pix};
                line_cnt <= line_cnt + 11'd1;
                meas_rows <= meas_rows + 11'd1;
                row_in   <= row_in + 11'd1;
                col_in   <= 11'd0;
                meas_pix <= 11'd0;
                out_col  <= 10'd0;

                if (meas_pix != NARROW_W && meas_pix != WIDE_W)
                    hw_err[3] <= 1'b1;

                if (armed) begin
                    if (meas_pix != (wide_mode ? WIDE_W : NARROW_W))
                        hw_err[0] <= 1'b1;
                    if (row_keep) out_row <= out_row + 10'd1;
                end
            end

            // ------------------------------------------- frame boundary
            if (vsync_rise) begin
                frame_start <= will_arm;
                line_cnt    <= 11'd0;
                pix_cnt     <= 12'd0;
                col_in      <= 11'd0;
                row_in      <= 11'd0;
                out_col     <= 10'd0;
                out_row     <= 10'd0;

                if (armed && (href || href_d0)) hw_err[2] <= 1'b1;

                if (!armed) begin
                    // Every unarmed frame is a measurement frame; the last
                    // one before `arm` decides the decimation mode.
                    wide_mode <= (meas_pix > (NARROW_W + WIDE_W) / 2);
                    if (wait_cnt == WAIT_FRAMES) begin
                        armed     <= 1'b1;
                        frame_cnt <= frame_cnt + 16'd1;
                    end else begin
                        wait_cnt <= wait_cnt + 6'd1;
                    end
                end else begin
                    if (meas_rows != (wide_mode ? WIDE_H : NARROW_H))
                        hw_err[1] <= 1'b1;
                    frame_cnt <= frame_cnt + 16'd1;
                end

                meas_pix  <= 11'd0;
                meas_rows <= 11'd0;
            end

            // ------------------------------------------- frame boundary end
            // FRAME_VALID trails the last LINE_VALID by (1 + Reg0x05 - 19)
            // pixel clocks, so this edge is always in blanking, a whole
            // vertical blank ahead of the next frame_start pulse.
            if (vsync_fall) frame_end <= 1'b1;

        end
    end

endmodule
