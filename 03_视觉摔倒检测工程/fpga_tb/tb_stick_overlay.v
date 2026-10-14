//===========================================================================
// tb_stick_overlay.v -- replay box sets through rtl/hdmi/stick_overlay.v
//
// The overlay is combinational over one 640x480 raster, so the bench walks
// the whole frame once per case and writes the RGB it produced.  The cases
// come from a text file, four lines per case:
//
//   found dir alarm x y w h
//
// (+CFG=<file> +OUT=<file>, one 640*480*3 byte dump per case)
// stick_overlay_check.py generates the cases, models the drawing in Python
// and compares the two pixel for pixel.
//===========================================================================
`timescale 1ns / 1ps

module tb_stick_overlay;

localparam integer NB = 4;
localparam integer W  = 640;
localparam integer H  = 480;

reg clk = 1'b0;
reg de = 1'b0;
reg [9:0] px = 10'd0;
reg [9:0] py = 10'd0;
reg [23:0] rgb_in;

reg [NB-1:0]   found, alarm, dir;
reg [NB*9-1:0] bx, by, bw, bh;

wire [23:0] rgb_out;

stick_overlay #(.NB(NB)) u_dut (
    .de         (de),
    .px         (px),
    .py         (py),
    .rgb_in     (rgb_in),
    .found      (found),
    .alarm      (alarm),
    .head_right (dir),
    .bx         (bx),
    .by         (by),
    .bw         (bw),
    .bh         (bh),
    .rgb_out    (rgb_out)
);

always #5 clk = ~clk;

reg [1023:0] cfg_path, out_path;
integer fd_cfg, fd_out, got, cs, sl, i, ncase;
integer f0, d0, a0, x0, y0, w0, h0;
reg [23:0] pat;
reg        done = 1'b0;

initial begin
    if (!$value$plusargs("CFG=%s", cfg_path) ||
        !$value$plusargs("OUT=%s", out_path)) begin
        $display("ERROR need CFG and OUT");
        $finish;
    end
    fd_cfg = $fopen(cfg_path, "r");
    fd_out = $fopen(out_path, "wb");
    if (fd_cfg == 0 || fd_out == 0) begin
        $display("ERROR cannot open files");
        $finish;
    end

    ncase = 0;
    while (!done) begin
        // Four lines per case; a short read means the file ended, and $feof
        // is only reliable after a read, so the read is the loop test.
        got = $fscanf(fd_cfg, "%d %d %d %d %d %d %d\n",
                      f0, d0, a0, x0, y0, w0, h0);
        if (got != 7) done = 1'b1;
        else begin
        found = {NB{1'b0}}; alarm = {NB{1'b0}}; dir = {NB{1'b0}};
        bx = {(NB*9){1'b0}}; by = {(NB*9){1'b0}};
        bw = {(NB*9){1'b0}}; bh = {(NB*9){1'b0}};
        found[0] = f0[0]; dir[0] = d0[0]; alarm[0] = a0[0];
        bx[8:0] = x0[8:0]; by[8:0] = y0[8:0];
        bw[8:0] = w0[8:0]; bh[8:0] = h0[8:0];
        for (sl = 1; sl < NB; sl = sl + 1) begin
            got = $fscanf(fd_cfg, "%d %d %d %d %d %d %d\n",
                          f0, d0, a0, x0, y0, w0, h0);
            if (got != 7) begin
                $display("ERROR short config at case %0d slot %0d", ncase, sl);
                $finish;
            end
            found[sl] = f0[0];
            dir[sl]   = d0[0];
            alarm[sl] = a0[0];
            bx[sl*9 +: 9] = x0[8:0];
            by[sl*9 +: 9] = y0[8:0];
            bw[sl*9 +: 9] = w0[8:0];
            bh[sl*9 +: 9] = h0[8:0];
        end

        // one full raster, de high, background pattern from the coordinates
        for (i = 0; i < W * H; i = i + 1) begin
            px = i % W;
            py = i / W;
            pat = {px[7:0], py[7:0], 8'h5A};
            rgb_in = pat;
            de = 1'b1;
            @(negedge clk);
            @(posedge clk);
            $fwrite(fd_out, "%c%c%c", rgb_out[23:16], rgb_out[15:8],
                    rgb_out[7:0]);
        end
        de = 1'b0;
        ncase = ncase + 1;
        end
    end

    $fclose(fd_cfg);
    $fclose(fd_out);
    $display("DONE cases=%0d", ncase);
    $finish;
end

endmodule
