//===========================================================================
// gray_front.v -- RGB565 -> 8-bit luma -> 2x2 box -> 3-of-4 row decimation
//
// Camera path of the fall detector: the OV5640 feeds RGB565 at 640x480 and
// the algorithm works at 320x180.  The integer luma weights are the same as
// tools/fall_proto.py (77/150/29, >>8) so the RTL and the offline prototype
// stay comparable.
//
//   RGB565 -> 888 by bit replication (R5->R8, G6->G8, B5->B8)
//   gray   = (77*R + 150*G + 29*B) >> 8
//   2x2 box: gray_out = (a + b + c + d) >> 2      (truncating, like numpy)
//
// Vertical decimation: the 2x2 box turns 480 camera rows into 240, then the
// mod-4 row gate keeps 3 of every 4 (0,1,2,4,5,6,8,...) for 180 working
// rows: row 3,7,11,... is the one dropped.  320x240 would need a 76,800-byte
// frame buffer plus a 14,400-entry motion mask against a 76,800-entry ERAM
// budget, so the frame does not fit.  Over the 32-clip corpus
// tools/res_eval.py scores the two candidates as
//
//   320x240   1989 detected frames, 13 alarms
//   320x160   1903 detected frames, 14 alarms   (2-of-3 gate, loses S3/Fall/01)
//   320x180   1946 detected frames, 15 alarms   (3-of-4 gate, keeps every
//                                                true alarm of the 240 line)
//
// so the shallowest gate wins.
//
// The pixel stream has no blanking requirement: drive de_in for exactly
// 640 pixels per row / 480 rows, with a one-cycle frame_start pulse before
// the first pixel of every frame.
//===========================================================================
`timescale 1ns / 1ps

module gray_front (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        de_in,
    input  wire [15:0] pix_in,      // {R[4:0], G[5:0], B[4:0]}
    output reg         de_out,      // 320x180 stream
    output reg  [7:0]  gray_out,
    output reg  [9:0]  x_out,
    output reg  [8:0]  y_out
);

// ------------------------------------------------------- RGB565 -> luma
wire [4:0] r5 = pix_in[15:11];
wire [5:0] g6 = pix_in[10:5];
wire [4:0] b5 = pix_in[4:0];

wire [7:0] r8 = {r5, r5[4:2]};
wire [7:0] g8 = {g6, g6[5:4]};
wire [7:0] b8 = {b5, b5[4:2]};

wire [15:0] luma_sum = 16'd77  * r8 + 16'd150 * g8 + 16'd29 * b8;
wire [7:0]  luma     = luma_sum[15:8];

// ------------------------------------------------------------- 2x2 box
reg [9:0] row_sum [0:319];      // sum of the two pixels of the even row
reg [9:0] pair_even;            // first pixel of the current pair
reg [9:0] x_cnt;                // 0..639 source column
reg [9:0] y_cnt;                // 0..479 source row
reg [1:0] row_phase;            // mod-4 phase of the 2x2-boxed row index
reg [8:0] out_row_cnt;          // 0..179, emitted-row index (y_out)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        de_out  <= 1'b0;
        gray_out<= 8'd0;
        x_out   <= 10'd0;
        y_out   <= 9'd0;
        pair_even <= 10'd0;
        x_cnt   <= 10'd0;
        y_cnt   <= 10'd0;
        row_phase    <= 2'd0;
        out_row_cnt  <= 9'd0;
    end else begin
        de_out <= 1'b0;
        if (frame_start) begin
            x_cnt <= 10'd0;
            y_cnt <= 10'd0;
            row_phase    <= 2'd0;
            out_row_cnt  <= 9'd0;
        end else if (de_in) begin
            if (!x_cnt[0]) begin
                pair_even <= {2'b00, luma};        // first of the pair
            end else if (!y_cnt[0]) begin          // second pixel of an even
                                                   // source row: row_sum done
                row_sum[x_cnt[9:1]] <= pair_even + {2'b00, luma};
            end else if (row_phase != 2'd3) begin     // second pixel of an odd
                                                      // source row: emit unless
                                                      // this is the dropped row
                gray_out <= (row_sum[x_cnt[9:1]] + pair_even
                             + {2'b00, luma}) >> 2;
                x_out    <= x_cnt[9:1];
                y_out    <= out_row_cnt;
                de_out   <= 1'b1;
            end
            if (x_cnt == 10'd639) begin
                x_cnt <= 10'd0;
                y_cnt <= (y_cnt == 10'd479) ? 10'd0 : (y_cnt + 10'd1);
                // The 2x2 box completes on the odd source row, so advance
                // the boxed-row phase there: 0,1,2 kept, 3 dropped.
                if (y_cnt[0]) begin
                    row_phase <= (row_phase == 2'd3) ? 2'd0
                                                     : (row_phase + 2'd1);
                    if (row_phase != 2'd3)
                        out_row_cnt <= out_row_cnt + 9'd1;
                end
            end else begin
                x_cnt <= x_cnt + 10'd1;
            end
        end
    end
end

endmodule
