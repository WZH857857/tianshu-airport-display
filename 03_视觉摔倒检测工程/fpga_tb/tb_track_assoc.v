//===========================================================================
// tb_track_assoc.v -- vector replay for track_assoc.v
//
// Input text: one frame = four groups of six integers
//             found x y w h area
// Output text: the same six integers plus the 7-bit score per slot.
//
// plusargs: +IN=<file> +OUT=<file> +FRAMES=<n>
//===========================================================================
`timescale 1ns / 1ps

module tb_track_assoc;

localparam integer NB    = 4;
localparam integer XW    = 9;
localparam integer AREAW = 17;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg in_valid = 1'b0;
reg [NB-1:0] in_found;
reg [NB*XW-1:0] in_x, in_y, in_w, in_h;
reg [NB*AREAW-1:0] in_area;

wire out_valid;
wire [NB-1:0] out_found;
wire [NB*XW-1:0] out_x, out_y, out_w, out_h;
wire [NB*AREAW-1:0] out_area;
wire [NB*7-1:0] out_score;

track_assoc #(
    .NUM_BLOBS (NB),
    .XW        (XW),
    .AREAW     (AREAW)
) dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .in_valid  (in_valid),
    .in_found  (in_found),
    .in_x      (in_x),
    .in_y      (in_y),
    .in_w      (in_w),
    .in_h      (in_h),
    .in_area   (in_area),
    .out_valid (out_valid),
    .out_found (out_found),
    .out_x     (out_x),
    .out_y     (out_y),
    .out_w     (out_w),
    .out_h     (out_h),
    .out_area  (out_area),
    .out_score (out_score)
);

always #5 clk = ~clk;

reg [1023:0] in_path, out_path;
integer fd_in, fd_out;
integer nframes, f, k, got, wait_cnt;
integer v_found, v_x, v_y, v_w, v_h, v_area;

initial begin
    if (!$value$plusargs("IN=%s", in_path) ||
        !$value$plusargs("OUT=%s", out_path) ||
        !$value$plusargs("FRAMES=%d", nframes)) begin
        $display("ERROR missing plusarg");
        $finish;
    end
    fd_in = $fopen(in_path, "r");
    fd_out = $fopen(out_path, "w");
    if (fd_in == 0 || fd_out == 0) begin
        $display("ERROR cannot open vector file");
        $finish;
    end

    in_found = {NB{1'b0}};
    in_x = {(NB*XW){1'b0}};
    in_y = {(NB*XW){1'b0}};
    in_w = {(NB*XW){1'b0}};
    in_h = {(NB*XW){1'b0}};
    in_area = {(NB*AREAW){1'b0}};
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    for (f = 0; f < nframes; f = f + 1) begin
        for (k = 0; k < NB; k = k + 1) begin
            got = $fscanf(fd_in, "%d %d %d %d %d %d",
                          v_found, v_x, v_y, v_w, v_h, v_area);
            if (got != 6) begin
                $display("ERROR short vector at frame %0d slot %0d", f, k);
                $finish;
            end
            in_found[k] = v_found[0];
            in_x[k*XW +: XW] = v_x[XW-1:0];
            in_y[k*XW +: XW] = v_y[XW-1:0];
            in_w[k*XW +: XW] = v_w[XW-1:0];
            in_h[k*XW +: XW] = v_h[XW-1:0];
            in_area[k*AREAW +: AREAW] = v_area[AREAW-1:0];
        end

        @(negedge clk);
        in_valid = 1'b1;
        @(posedge clk);
        #1;
        @(negedge clk);
        in_valid = 1'b0;

        wait_cnt = 0;
        while (!out_valid && wait_cnt < 2000) begin
            @(posedge clk);
            wait_cnt = wait_cnt + 1;
        end
        if (!out_valid) begin
            $display("ERROR timeout at frame %0d", f);
            $finish;
        end
        #1;
        for (k = 0; k < NB; k = k + 1) begin
            $fdisplay(fd_out, "%0d %0d %0d %0d %0d %0d %0d",
                      out_found[k],
                      out_x[k*XW +: XW], out_y[k*XW +: XW],
                      out_w[k*XW +: XW], out_h[k*XW +: XW],
                      out_area[k*AREAW +: AREAW],
                      out_score[k*7 +: 7]);
        end

        // Let the pulse fall and the state machine return to S_IDLE.
        @(posedge clk);
        #1;
        @(posedge clk);
        #1;
    end

    $fclose(fd_in);
    $fclose(fd_out);
    $display("DONE frames=%0d", nframes);
    $finish;
end

endmodule
