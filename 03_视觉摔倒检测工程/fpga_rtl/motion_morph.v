//===========================================================================
// motion_morph.v -- disk(1) erosion + disk(3) dilation
//
// Matches scipy.ndimage.binary_erosion(disk(1)) followed by
// binary_dilation(disk(3)) with border value 0, exactly as tools/fall_proto.py
// runs them.
//
// The two radii are asymmetric on purpose.  A 320x180 monitor-to-camera
// replay of a fall produces a thin, low-contrast motion mask: disk(2)
// erosion cut the falling body into disconnected fragments (best blob
// 84x54, ratio 0.64, never person-like), while disk(1) keeps the silhouette
// whole.  The wider disk(3) dilation then re-joins limbs and head without
// gluing neighbours, which is what lets posture_fsm see a real collapse.
//
// Stage 1 -- cross erosion.  One 3-row column stack plus four one-bit
// delays.  Before the write, stack3[s] holds rows r-3..r-1 of column s, so
// the AND of the centre column's middle row (w3a_cur[1]), its two column
// neighbours' middle row (p1a/p2a) and the previous column's {r-3, r-1}
// bits (q2a/q0a) is the disk(1) cross centred at (r-2, s-1):
//
//     e_a = E[r-2][s-1]
//
// Stage 2 -- diamond dilation.  A per-column 7-row stack holds the eroded
// rows.  Column c is written at slot s = c+1 and shifts in e_a for that
// column, so just after the write bit i of stack7[c] is E[r-2-i][c].  The
// output for row y = r - ROW_LATENCY is the OR of seven vertical lanes;
// lane kf (dy = kf - 3) reads bit 3-dy = 6-kf of the freshly written column
// word, which is erosion row r-5+dy = y+dy:
//
//     kf      0    1    2    3    4    5    6
//     dy     -3   -2   -1    0   +1   +2   +3
//     width   1    5    5    7    5    5    1     (disk(3) row widths)
//
// Each lane runs a centred horizontal OR-window of that width over its own
// bit stream.  A three-bit running count adds the incoming bit and removes
// the bit leaving the window, so the window value at column c is centred on
// c - h with h = (width-1)/2.  All seven lanes must report the same output
// column, so the shorter windows are delayed: the h=2 lanes by one slot and
// the h=0 (single-pixel) lanes by three, both through vpipe.
//
// Column and row alignment: out col = slot - DIL_LATENCY, out row = input
// row - ROW_LATENCY, so the first output row leaves while input row 5 is
// being streamed and the last five output rows come from flush rows.  Per
// row the datapath runs DIL_LATENCY extra zero slots after the 320 real
// pixels; after the last real row it runs ROW_LATENCY zero rows, which
// finish the bottom border and leave every stack zeroed for the next frame.
// Both fit inside the source's horizontal and vertical blanking.
//
// The port is a sparse valid stream, not a free-running one: the module
// advances one slot per de_in pixel, so it can sit directly behind
// gray_front + frame_diff, where a 320-pixel row arrives with idle clocks
// between pixels and between rows.  A frame is HEIGHT counted input rows; no
// frame_start strobe is required.  Output is 320 pixels x HEIGHT rows per
// frame, one row per input row, mask_out/de_out valid together.
//===========================================================================
`timescale 1ns / 1ps

module motion_morph (
    input  wire clk,
    input  wire rst_n,
    input  wire de_in,
    input  wire mask_in,
    output reg  de_out,
    output reg  mask_out
);

localparam integer WIDTH       = 320;
localparam integer HEIGHT      = 180;   // 320x180 working frame
localparam integer DIL_RADIUS  = 3;     // disk radius, 2*R+1 = 7 wide
localparam integer DIL_LATENCY = 1 + DIL_RADIUS;  // out col = input col - 4
localparam integer ROW_LATENCY = 5;               // erosion 2 + dilation 3
// One row is WIDTH real pixels plus DIL_LATENCY zero slots, so exactly
// WIDTH de_out pixels leave the row: the padding the disk needs on its
// right edge must not turn into extra bits in the packed output stream.
localparam integer ROW_SLOTS   = WIDTH + DIL_LATENCY;
localparam integer LAST_SLOT   = ROW_SLOTS - 1;
localparam integer FLUSH_ROWS  = ROW_LATENCY;

localparam [1:0] P_IDLE = 2'd0;
localparam [1:0] P_ROW  = 2'd1;
localparam [1:0] P_FROW = 2'd2;

// ------------------------------------------------------------- column stacks
reg [2:0]  stack3 [0:WIDTH-1];   // {row r-3, r-2, r-1} cross erosion
reg [6:0]  stack7 [0:WIDTH-1];   // 7 newest eroded rows per column

// ------------------------------------------------------------- reset clearing
reg [8:0] clr_addr;
reg       clr_busy;
reg       ready;

// ------------------------------------------------------------- row sequencing
reg [1:0]  phase;
reg [9:0]  slot;          // 0 .. LAST_SLOT
reg [8:0]  row;           // 0..179 real rows, 180 while flushing
reg [3:0]  flush_left;

// ------------------------------------------------------------- erosion state
reg       p1a, p2a;       // cross middle-row bit, 1 and 2 columns back
reg       q2a, q0a;       // cross previous column's {r-3} and {r-1} bits

// ------------------------------------------------------------ dilation state
// One 7-bit history per windowed lane, one 3-bit running count, and a
// 3-bit delay of the lane's window value.  vpipe[0] is one slot old and
// vpipe[2] three slots old; the select below picks whichever matches the
// lane's half-width.  Lanes 0 and 6 (width 1) need no history or count:
// their window value is the lane bit itself.
reg [6:0] h7    [0:6];
reg [2:0] cnt   [0:6];
reg [2:0] vpipe [0:6];

// --------------------------------------------------------------- temporaries
integer   kf;
reg [2:0] w3a_cur, w3a_new;
reg [6:0] w7_cur, w7_new;
reg       e_a, e_b;
reg [6:0] lane_src;
reg       bbit;
reg       v_now;
reg [2:0] ncnt;
reg       d_or;

// per-cycle datapath control
reg       process;
reg       pix;
reg [9:0] dslot;
reg       row_done;

always @* begin
    process  = 1'b0;
    pix      = 1'b0;
    dslot    = slot;
    row_done = 1'b0;
    if (ready && !clr_busy) begin
        if (phase == P_IDLE) begin
            if (de_in) begin
                process = 1'b1;
                pix     = mask_in;
                dslot   = 10'd0;
            end
        end else if (phase == P_ROW) begin
            if (slot < WIDTH) begin
                if (de_in) begin
                    process = 1'b1;
                    pix     = mask_in;
                    dslot   = slot;
                end
            end else begin
                process = 1'b1;          // tail slots 320..323, zeros
                dslot   = slot;
            end
            if (process && (dslot == LAST_SLOT))
                row_done = 1'b1;
        end else begin
            process = 1'b1;              // flush rows, all zeros
            dslot   = slot;
            if (dslot == LAST_SLOT)
                row_done = 1'b1;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clr_addr   <= 9'd0;
        clr_busy   <= 1'b1;
        ready      <= 1'b0;
        phase      <= P_IDLE;
        slot       <= 10'd0;
        row        <= 9'd0;
        flush_left <= 4'd0;
        de_out     <= 1'b0;
        mask_out   <= 1'b0;
        p1a <= 1'b0; p2a <= 1'b0; q2a <= 1'b0; q0a <= 1'b0;
        for (kf = 0; kf < 7; kf = kf + 1) begin
            h7[kf]    <= 7'd0;
            cnt[kf]   <= 3'd0;
            vpipe[kf] <= 3'd0;
        end
    end else begin
        de_out <= 1'b0;

        if (clr_busy) begin
            stack3[clr_addr] <= 3'd0;
            stack7[clr_addr] <= 7'd0;
            if (clr_addr == WIDTH - 1) begin
                clr_busy <= 1'b0;
                ready    <= 1'b1;
            end else begin
                clr_addr <= clr_addr + 9'd1;
            end
        end else if (ready && process) begin
            // ------------------------------------------------ cross erosion
            // slot (s,r) evaluates the centre (s-1, r-2)
            w3a_cur = 3'd0;
            if (dslot < WIDTH) w3a_cur = stack3[dslot];
            w3a_new = {w3a_cur[1:0], pix};
            e_a     = q2a & p1a & q0a & p2a & w3a_cur[1];

            if (dslot < WIDTH) stack3[dslot] <= w3a_new;
            p2a <= p1a;
            p1a <= w3a_cur[1];
            q2a <= w3a_cur[2];
            q0a <= w3a_cur[0];
            e_b     = e_a;

            // ------------------------------------- vertical column stack
            // Column c takes its new erosion row at slot c+1; bit i of the
            // column word is then E[r-2-i][c].  Past the last column the
            // lane streams get zeros, which drains the right-edge windows.
            w7_cur   = 7'd0;
            w7_new   = 7'd0;
            lane_src = 7'd0;
            if ((dslot >= 1) && (dslot <= WIDTH)) begin
                w7_cur = stack7[dslot-1];
                w7_new = {w7_cur[5:0], e_b};
                stack7[dslot-1] <= w7_new;
                lane_src = w7_new;
            end

            // ----------------------------------------- seven diamond lanes
            d_or = 1'b0;
            for (kf = 0; kf < 7; kf = kf + 1) begin
                bbit = lane_src[6-kf];
                if (kf == 3) begin
                    // dy = 0, centred 7-wide window
                    ncnt   = cnt[kf] + {2'b00, bbit} - {2'b00, h7[kf][6]};
                    v_now  = (ncnt != 3'd0);
                    cnt[kf] <= ncnt;
                    h7[kf]  <= {h7[kf][5:0], bbit};
                    vpipe[kf] <= {vpipe[kf][1:0], v_now};
                    d_or   = d_or | v_now;
                end else if ((kf == 0) || (kf == 6)) begin
                    // dy = +/-3, single pixel, delayed three slots
                    v_now  = bbit;
                    vpipe[kf] <= {vpipe[kf][1:0], v_now};
                    d_or   = d_or | vpipe[kf][2];
                end else begin
                    // dy = +/-1, +/-2, centred 5-wide window, delayed one
                    ncnt   = cnt[kf] + {2'b00, bbit} - {2'b00, h7[kf][4]};
                    v_now  = (ncnt != 3'd0);
                    cnt[kf] <= ncnt;
                    h7[kf]  <= {h7[kf][5:0], bbit};
                    vpipe[kf] <= {vpipe[kf][1:0], v_now};
                    d_or   = d_or | vpipe[kf][0];
                end
            end

            if ((dslot >= DIL_LATENCY) && (row >= ROW_LATENCY)) begin
                mask_out <= d_or;
                de_out   <= 1'b1;
            end

            // -------------------------------------------- row sequencing
            if (row_done) begin
                slot <= 10'd0;
                if (phase == P_ROW) begin
                    if (row == HEIGHT - 1) begin
                        row        <= HEIGHT;   // keep the output gate open
                        phase      <= P_FROW;
                        flush_left <= 4'd0;
                    end else begin
                        row <= row + 9'd1;
                    end
                end else if (flush_left == FLUSH_ROWS - 1) begin
                    row   <= 9'd0;
                    phase <= P_IDLE;
                end else begin
                    flush_left <= flush_left + 4'd1;
                end
            end else begin
                slot <= slot + 10'd1;
                if (phase == P_IDLE) begin
                    phase <= P_ROW;
                    slot  <= 10'd1;
                end
            end
        end
    end
end

endmodule
