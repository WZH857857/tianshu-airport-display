//===========================================================================
// cmos_capture_rgb565.v -- OV5640 DVP byte stream -> RGB565 pixel stream
//
// The OV5640 in RGB565 mode sends two bytes per pixel on the 8-bit DVP bus
// while HREF is high.  The first byte is {R[4:0], G[5:3]}, the second is
// {G[2:0], B[4:0]}, so one pixel is {first, second} -- exactly the
// {R[4:0], G[5:0], B[4:0]} word gray_front expects.
//
// Contract produced here (see docs/板级接入.md):
//   de      : one pclk cycle per assembled pixel, data valid in the same
//             cycle it is high (the core samples de/pix on the same edge)
//   pix     : RGB565, row order top->bottom
//   frame_start : single pclk pulse on the VSYNC leading edge, i.e. inside
//             vertical blanking before the first HREF of the frame
//
// The camera only becomes the timing master after SCCB configuration, so the
// module waits WAIT_FRAMES complete VSYNC strokes after `arm` before it lets
// any pixel through; the first armed frame also gets its frame_start.
//
// hw_err is sticky and decoded as:
//   [0] a line did not contain exactly H_ACTIVE pixels
//   [1] a frame did not contain exactly V_ACTIVE lines
//   [2] VSYNC moved while HREF was active (protocol violation)
//   [3] a byte pair was cut by HREF going low (odd byte count)
//===========================================================================
`timescale 1ns / 1ps

module cmos_capture_rgb565 #(
    parameter integer H_ACTIVE    = 640,
    parameter integer V_ACTIVE    = 480,
    parameter integer WAIT_FRAMES = 10
) (
    input  wire        pclk,
    input  wire        rst_n,        // async, level, valid in both domains
    input  wire        arm,          // cfg_done synchronised into pclk

    input  wire        vsync,
    input  wire        href,
    input  wire [7:0]  data,

    output reg         de,
    output reg  [15:0] pix,
    output reg         frame_start,
    output reg         armed,

    output reg  [15:0] frame_cnt,
    output reg  [9:0]  line_cnt,
    output reg  [10:0] pix_cnt,
    output reg  [7:0]  hw_err
);

    reg vsync_d0, vsync_d1;
    reg href_d0,  href_fall_d;
    reg byte_sel;
    reg [7:0] byte0;
    reg [4:0] wait_cnt;

    wire vsync_rise =  vsync_d0 & ~vsync_d1;
    wire href_rise  =  href & ~href_d0;
    wire href_fall  = ~href &  href_d0;
    // frame_start marks the first frame that actually carries pixels, so the
    // core never sees a boundary for a frame the capture drops.
    wire will_arm   =  armed || (wait_cnt == WAIT_FRAMES);

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d0    <= 1'b0;
            vsync_d1    <= 1'b0;
            href_d0     <= 1'b0;
            href_fall_d <= 1'b0;
            byte_sel    <= 1'b0;
            byte0       <= 8'd0;
            de          <= 1'b0;
            pix         <= 16'd0;
            frame_start <= 1'b0;
            armed       <= 1'b0;
            wait_cnt    <= 5'd0;
            frame_cnt   <= 16'd0;
            line_cnt    <= 10'd0;
            pix_cnt     <= 11'd0;
            hw_err      <= 8'd0;
        end else begin
            vsync_d0 <= vsync;
            vsync_d1 <= vsync_d0;
            href_d0  <= href;

            de          <= 1'b0;
            frame_start <= 1'b0;
            href_fall_d <= href_fall;

            // ------------------------------------------------ byte pairing
            if (armed && href) begin
                byte_sel <= ~byte_sel;
                if (byte_sel) begin
                    pix <= {byte0, data};
                    de  <= 1'b1;
                end else begin
                    byte0 <= data;
                end
            end else begin
                // HREF low: the next byte is the first of a new pair
                if (armed && byte_sel) hw_err[3] <= 1'b1;
                byte_sel <= 1'b0;
            end

            // ------------------------------------------------ pixel count
            if (href_rise)         pix_cnt <= 11'd0;
            else if (de)           pix_cnt <= pix_cnt + 11'd1;

            // ---------------------------------- end of line (one cycle late,
            // so the final de pulse of the row is already counted)
            if (href_fall_d) begin
                if (armed) begin
                    line_cnt <= line_cnt + 10'd1;
                    if (pix_cnt != H_ACTIVE) hw_err[0] <= 1'b1;
                end
            end

            // ------------------------------------------------ frame events
            if (vsync_rise) begin
                frame_start <= will_arm;
                line_cnt    <= 10'd0;
                pix_cnt     <= 11'd0;

                if (armed && (href || href_d0)) hw_err[2] <= 1'b1;

                if (!armed) begin
                    if (wait_cnt == WAIT_FRAMES) begin
                        armed     <= 1'b1;
                        frame_cnt <= frame_cnt + 16'd1;
                    end else begin
                        wait_cnt <= wait_cnt + 5'd1;
                    end
                end else begin
                    if (line_cnt != V_ACTIVE) hw_err[1] <= 1'b1;
                    frame_cnt <= frame_cnt + 16'd1;
                end
            end
        end
    end

endmodule
