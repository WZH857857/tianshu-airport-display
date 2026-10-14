// Temporary bring-up bench for multi_bbox: tiny raster, state trace.
`timescale 1ns / 1ps

module tb_dbg_multi;

localparam integer WIDTH     = 32;
localparam integer HEIGHT    = 8;
localparam integer NUM_BLOBS = 4;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg de_in = 1'b0;
reg mask_in = 1'b0;

wire busy;
wire out_valid;
wire [NUM_BLOBS-1:0]    out_found;
wire [NUM_BLOBS*9-1:0]  out_x, out_y, out_w, out_h;
wire [NUM_BLOBS*17-1:0] out_area;

multi_bbox #(
    .WIDTH     (WIDTH),
    .HEIGHT    (HEIGHT),
    .MIN_PIX   (1),
    .MIN_AREA  (1),
    .NUM_BLOBS (NUM_BLOBS)
) uut (
    .clk (clk), .rst_n (rst_n), .de_in (de_in), .mask_in (mask_in),
    .busy (busy), .out_valid (out_valid), .out_found (out_found),
    .out_x (out_x), .out_y (out_y), .out_w (out_w), .out_h (out_h),
    .out_area (out_area)
);

always #5 clk = ~clk;

integer cyc = 0;
integer last_state = -1;
always @(posedge clk) begin
    cyc = cyc + 1;
    if (uut.state != last_state) begin
        $display("t=%0t cyc=%0d state %0d -> %0d row=%0d col=%0d ven=%0d vd=%0d/%0d vcnt=%0d",
                 $time, cyc, last_state, uut.state, uut.acc_row, uut.acc_col,
                 uut.vrun_end, uut.vdrain, uut.vdrain_cnt, uut.vcnt);
        if (uut.state == 4)
            $display("   enter vspan: res_start=%0d res_len=%0d rbest=%0d",
                     uut.res_start, uut.res_len, uut.r_best_len);
        last_state = uut.state;
    end
    if (out_valid)
        $display("t=%0t OUT found=%b x=%0d y=%0d w=%0d h=%0d area=%0d",
                 $time, out_found, out_x[8:0], out_y[8:0], out_w[8:0],
                 out_h[8:0], out_area[16:0]);
    if (cyc > 40000) begin
        $display("TIMEOUT state=%0d row=%0d col=%0d vd=%0d/%0d",
                 uut.state, uut.acc_row, uut.acc_col, uut.vdrain, uut.vdrain_cnt);
        $display("  write side: sx=%0d sy=%0d wbit=%0d waddr=%0d fdone=%0d dstart=%0d dlen=%0d bank=%0d",
                 uut.sx, uut.sy, uut.wbit, uut.waddr, uut.frame_done,
                 uut.done_start, uut.done_len, uut.done_bank);
        $display("  out_valid=%0d busy=%0d pass=%0d/%0d",
                 out_valid, busy, uut.pass_idx, uut.pass_count);
        $finish;
    end
end

integer x, y;
initial begin
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    for (y = 0; y < HEIGHT; y = y + 1) begin
        for (x = 0; x < WIDTH; x = x + 1) begin
            @(negedge clk);
            de_in = 1'b1;
            // Solid frame: the column run must be exactly WIDTH long.
            mask_in = 1'b1;
        end
    end
    @(negedge clk);
    de_in = 1'b0;
    mask_in = 1'b0;
    $display("streamed");
end

endmodule
