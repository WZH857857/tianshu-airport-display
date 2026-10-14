`timescale 1ns / 1ps

module test_pattern (
    input  wire        pixel_clk,
    input  wire        reset_n,
    input  wire        video_active,
    input  wire        frame_tick,
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire [1:0]  mode,
    output reg  [23:0] rgb
);

    reg [9:0] motion_x;
    reg [2:0] bar_index;

    always @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n)
            motion_x <= 10'd0;
        else if (frame_tick) begin
            if (motion_x > 10'd540)
                motion_x <= 10'd0;
            else
                motion_x <= motion_x + 10'd4;
        end
    end

    always @* begin
        rgb = 24'h000000;
        bar_index = 3'd0;

        if (x < 10'd80)
            bar_index = 3'd0;
        else if (x < 10'd160)
            bar_index = 3'd1;
        else if (x < 10'd240)
            bar_index = 3'd2;
        else if (x < 10'd320)
            bar_index = 3'd3;
        else if (x < 10'd400)
            bar_index = 3'd4;
        else if (x < 10'd480)
            bar_index = 3'd5;
        else if (x < 10'd560)
            bar_index = 3'd6;
        else
            bar_index = 3'd7;

        case (mode)
            2'd0: begin
                case (bar_index)
                    3'd0: rgb = 24'hFFFFFF;
                    3'd1: rgb = 24'hFFFF00;
                    3'd2: rgb = 24'h00FFFF;
                    3'd3: rgb = 24'h00FF00;
                    3'd4: rgb = 24'hFF00FF;
                    3'd5: rgb = 24'hFF0000;
                    3'd6: rgb = 24'h0000FF;
                    default: rgb = 24'h000000;
                endcase

                if ((x >= motion_x) && (x < motion_x + 10'd4))
                    rgb = 24'hFFFFFF;
            end

            2'd1: begin
                if (x[6] ^ y[6])
                    rgb = 24'h203850;
                else
                    rgb = 24'hD0E0F0;

                if ((x > 10'd316) && (x < 10'd324))
                    rgb = 24'hFF2020;
                if ((y > 10'd236) && (y < 10'd244))
                    rgb = 24'h20FF20;
            end

            2'd2: begin
                rgb = {x[9:2], y[9:2], x[9:2] ^ y[9:2]};

                if ((x >= motion_x) && (x < motion_x + 10'd80) &&
                    (y >= 10'd200) && (y < 10'd280))
                    rgb = 24'hFFFFFF;
            end

            default: begin
                case ({x[8], y[8]})
                    2'b00: rgb = 24'h402020;
                    2'b01: rgb = 24'h204020;
                    2'b10: rgb = 24'h202040;
                    default: rgb = 24'h505020;
                endcase

                if ((x[5:0] < 6'd2) || (y[5:0] < 6'd2))
                    rgb = 24'hFFFFFF;
            end
        endcase

        if ((x < 10'd4) || (x > 10'd635) || (y < 10'd4) || (y > 10'd475))
            rgb = 24'hFFFFFF;

        if (!video_active)
            rgb = 24'h000000;
    end

endmodule
