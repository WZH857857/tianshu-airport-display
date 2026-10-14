//===========================================================================
// tb_label_dump.v -- print the label bitmaps the overlay actually draws.
//
// The bench parks one box (y0 = 64) and walks each tag's own area, one
// character per vision pixel: 24 x 5 for the `87%` readout, 48 x 18 for the
// `摔倒` alarm.  The tag hangs above the box, so a person tag starts at
// vision row 58 and the 18-row alarm tag at row 45.
//
//   '.'  black tag backing        '#'  tag colour (green / red)
//   '-'  picture passes through   '!'  anything else
//
// tools/label_font_check.py reruns this bench and compares the dump with the
// Python twin of the same font (tools/label_font.py).
//===========================================================================
`timescale 1ns / 1ps

module tb_label_dump;

    localparam integer NB = 4;

    reg             de = 1'b1;
    reg  [9:0]      px = 10'd0;
    reg  [9:0]      py = 10'd0;
    reg  [23:0]     rgb_in = 24'h202020;
    reg  [NB-1:0]   found = 4'd0;
    reg  [NB-1:0]   alarm = 4'd0;
    reg  [NB*9-1:0] bx = 0, by = 0, bw = 0, bh = 0;
    reg  [NB*7-1:0] score = 0;
    wire [23:0]     rgb_out;

    integer dx, dy;
    reg [23:0] want;

    box_overlay #(.NB (NB)) dut (
        .de      (de),
        .px      (px),
        .py      (py),
        .rgb_in  (rgb_in),
        .found   (found),
        .alarm   (alarm),
        .bx      (bx),
        .by      (by),
        .bw      (bw),
        .bh      (bh),
        .score   (score),
        .rgb_out (rgb_out)
    );

    // One tag: rows x cols vision pixels from vision (0, ytop).
    task row_dump(input [255:0] tag, input integer ytop,
                  input integer rows, input integer cols);
        begin
            $write("TAG %0s\n", tag);
            for (dy = 0; dy < rows; dy = dy + 1) begin
                $write("ROW %0d ", dy);
                for (dx = 0; dx < cols; dx = dx + 1) begin
                    px = 2 * dx;                 // vx = dx
                    py = (8 * (ytop + dy) + 2) / 3;  // vy = ytop + dy
                    #1;
                    if (rgb_out === 24'h000000)
                        $write(".");
                    else if (rgb_out === want)
                        $write("#");
                    else if (rgb_out === rgb_in)
                        $write("-");
                    else
                        $write("!");
                end
                $write("\n");
            end
        end
    endtask

    initial begin
        // One box with its tag parked at vision (0,45): y0 = 64.
        bx[8:0] = 9'd0;
        by[8:0] = 9'd64;
        bw[8:0] = 9'd100;
        bh[8:0] = 9'd20;
        found[0] = 1'b1;

        want = 24'h00FF60;
        score[6:0] = 7'd87;
        #1;
        row_dump("person87", 58, 5, 24);

        score[6:0] = 7'd42;
        #1;
        row_dump("person42", 58, 5, 24);

        score[6:0] = 7'd9;
        #1;
        row_dump("person09", 58, 5, 24);

        // A confirmed fall swaps the tag for the red `摔倒` legend.
        alarm[0] = 1'b1;
        want = 24'hFF0000;
        #1;
        row_dump("alarm", 45, 18, 48);

        $finish;
    end

endmodule
