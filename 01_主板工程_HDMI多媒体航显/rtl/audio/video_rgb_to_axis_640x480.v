`timescale 1ns / 1ps
//=============================================================================
// video_rgb_to_axis_640x480.v
//
// Local copy of the vendor RGB/DE to 24-bit AXIS bridge.
//
// The vendor bridge starts S_x_cnt at zero on the first active pixel and
// increments it on every following active pixel.  With that timing the
// compare must be 638, not 639: on the 640th active pixel S_x_cnt is still
// 638, and comparing against 639 would place O_video_last one pixel after the
// active region, where O_video_valid is already low.
//=============================================================================

module video_rgb_to_axis_640x480 (
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_vs,
    input  wire        I_de,
    input  wire [23:0] I_rgb,

    output reg         O_video_user,
    output reg         O_video_valid,
    output reg         O_video_last,
    output reg  [23:0] O_video_data
);

    reg        S_vs_1d;
    reg        S_de_1d;
    reg        S_frame_arm;
    reg [10:0] S_x_cnt;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_vs_1d       <= 1'b0;
            S_de_1d       <= 1'b0;
            S_frame_arm   <= 1'b1;
            S_x_cnt       <= 11'd0;
            O_video_user  <= 1'b0;
            O_video_valid <= 1'b0;
            O_video_last  <= 1'b0;
            O_video_data  <= 24'd0;
        end
        else begin
            S_vs_1d <= I_vs;
            S_de_1d <= I_de;

            O_video_user  <= 1'b0;
            O_video_valid <= I_de;
            O_video_last  <= 1'b0;
            O_video_data  <= I_rgb;

            // Arm start-of-frame on either VS edge. The first active pixel
            // after that edge is the packet start.
            if (S_vs_1d != I_vs)
                S_frame_arm <= 1'b1;

            if (I_de) begin
                if (!S_de_1d) begin
                    S_x_cnt <= 11'd0;
                    if (S_frame_arm) begin
                        O_video_user <= 1'b1;
                        S_frame_arm  <= 1'b0;
                    end
                end
                else begin
                    S_x_cnt <= S_x_cnt + 11'd1;
                end

                // Pixel 640 still sees S_x_cnt = 638.
                if (S_x_cnt == 11'd638)
                    O_video_last <= 1'b1;
            end
            else begin
                S_x_cnt <= 11'd0;
            end
        end
    end

endmodule
