`timescale 1ns / 1ps
//=============================================================================
// tb_diag_overlay.v - render the on screen diagnostic panel to a PPM file.
//
// Four (source, code) combinations are drawn as a 2x2 grid so the digit
// shapes can be checked before the bitstream goes on the board.  Pixels
// outside the panel are filled with a muted blue so the panel edge and the
// two digit boxes are visible.
//=============================================================================
module tb_diag_overlay;

    reg  [9:0]  px;
    reg  [9:0]  py;
    reg         de;
    reg  [3:0]  code;
    reg  [3:0]  source;
    reg         enable;
    wire [23:0] rgb;
    wire        hit;

    integer     fd;
    integer     row;
    integer     col;
    integer     x;
    integer     y;
    integer     idx;

    reg [3:0]   src_vec  [0:3];
    reg [3:0]   code_vec [0:3];

    diag_overlay dut (
        .px     (px),
        .py     (py),
        .de     (de),
        .code   (code),
        .source (source),
        .enable (enable),
        .rgb    (rgb),
        .hit    (hit)
    );

    initial begin
        src_vec[0]  = 4'd0;  code_vec[0] = 4'd0;   // idle, code 0
        src_vec[1]  = 4'd2;  code_vec[1] = 4'd6;   // group 1, code 6
        src_vec[2]  = 4'd3;  code_vec[2] = 4'd8;   // group 2, code 8
        src_vec[3]  = 4'd1;  code_vec[3] = 4'd15;  // init fail, code F

        de     = 1'b1;
        enable = 1'b1;
        px     = 10'd0;
        py     = 10'd0;
        code   = 4'd0;
        source = 4'd0;

        fd = $fopen("sim/build/diag_overlay_preview.ppm", "w");
        if (fd == 0) begin
            $display("RESULT: FAIL tb_diag_overlay cannot open output file");
            $finish;
        end

        $fwrite(fd, "P3\n800 480\n255\n");

        for (row = 0; row < 2; row = row + 1) begin
            for (y = 0; y < 240; y = y + 1) begin
                for (col = 0; col < 2; col = col + 1) begin
                    idx    = (row * 2) + col;
                    source = src_vec[idx];
                    code   = code_vec[idx];
                    py     = 120 + y;
                    for (x = 0; x < 400; x = x + 1) begin
                        px = 120 + x;
                        #1;
                        if (hit)
                            $fwrite(fd, "%0d %0d %0d ",
                                    rgb[23:16], rgb[15:8], rgb[7:0]);
                        else
                            $fwrite(fd, "64 64 128 ");
                    end
                end
            end
        end

        $fclose(fd);

        // The panel must light up inside its box and stay off outside it.
        px = 10'd0;  py = 10'd0;  source = 4'd3;  code = 4'd8;  #1;
        if (hit !== 1'b0) begin
            $display("RESULT: FAIL tb_diag_overlay panel leaks outside");
            $finish;
        end
        px = 10'd300; py = 10'd300;  #1;
        if (hit !== 1'b1) begin
            $display("RESULT: FAIL tb_diag_overlay panel missing");
            $finish;
        end
        de = 1'b0;  #1;
        if (hit !== 1'b0) begin
            $display("RESULT: FAIL tb_diag_overlay ignores data enable");
            $finish;
        end
        de = 1'b1;  enable = 1'b0;  #1;
        if (hit !== 1'b0) begin
            $display("RESULT: FAIL tb_diag_overlay ignores enable");
            $finish;
        end

        $display("RESULT: PASS tb_diag_overlay");
        $finish;
    end

endmodule
