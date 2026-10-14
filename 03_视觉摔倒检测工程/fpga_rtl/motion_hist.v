//===========================================================================
// motion_hist.v -- a 2-bit "where has this person been" memory behind the
//                 crowd tracker, and the per-slot person box cut out of it
//
// The tracker (multi_bbox + track_assoc) only sees motion, so its box hugs
// whatever part of the person happens to move -- a leg, an arm -- and when
// the person stops it freezes on the last moving part.  A stick figure has
// to be drawn on the whole person, so this module keeps a small memory of
// where the person has been:
//
//   * one cell is 4x4 vision pixels; 80 x 45 cells = 7,200 bits = one 9K
//     block.  There is no full-resolution background array anywhere.
//   * a cell with at least MIN_CELL_PIX motion pixels is set to 3.  Every
//     DECAY frames a sweep drops every cell by one -- but only when the
//     frame's motion was at least MIN_GLOBAL cells, so a quiet scene (or a
//     person who momentarily stands still) keeps its trail instead of
//     fading away.
//   * a cell only accepts motion inside a window grown around one of the
//     published tracked boxes, which is what keeps a flickering door edge
//     out of the map.
//   * per slot, the bounding box of the live cells inside that window,
//     united with the tracked box, is the person box the overlay draws the
//     stick figure in.
//
// This is the RTL twin of fpga/tools/hist_model.py.  Every comparison and
// counter below is integer, so the two stay bit-exact and
// motion_hist_check.py pins them against each other on real clips.
//
// Timing.  The model wants the tracker box of frame i-1 while frame i's
// motion arrives, and frame i's box when the map of frame i is swept.  The
// tracker publishes frame i's boxes only after its four bbox passes, which
// on the board run while frame i+1 is already streaming.  The write side
// therefore cannot look at the boxes as the pixels arrive: the per-cell
// motion flags are buffered in a 3,600-bit RAM and the window test runs at
// the end of the frame, when the newest publish is certain to be frame
// i-1's.  The same end-of-frame pass applies the decay and copies the map
// into a second RAM, so the sweep that follows whenever frame i's boxes
// arrive cannot be disturbed by the traffic of frame i+1.
//
// Cost: three small RAMs (flags 1b x 3600, map 2b x 3600, snapshot
// 2b x 3600) and a 80 x 2b x 2 ping-pong cell counter.  The end-of-frame
// pass is 3,600 + 3,600 clocks, which fits between two frames (the sensor
// leaves about 16k pclk between the last output row and the next frame's
// first motion cell).
//===========================================================================
`timescale 1ns / 1ps

module motion_hist #(
    parameter integer NSLOT        = 4,
    parameter integer CW           = 80,    // cells per row
    parameter integer CH           = 45,    // cell rows
    parameter integer WIDTH        = 320,   // vision pixels per row
    parameter integer HEIGHT       = 180,   // vision rows
    parameter integer DECAY        = 8,     // frames between decay sweeps
    parameter integer MIN_CELL_PIX = 2,     // mask pixels that light a cell
    parameter integer MIN_GLOBAL   = 30,    // motion cells before the map ages
    parameter integer MIN_CELLS    = 5,     // live cells before a box is trusted
    parameter integer WIN_HW_MIN   = 7,     // half window width, cells
    parameter integer WIN_HH_MIN   = 13,    // half window height, cells
    parameter integer HW_MAX       = 5      // tallest figure, w * HW_MAX / 2
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // motion_morph mask stream: WIDTH pixels per row, HEIGHT rows per frame
    input  wire                     de_in,
    input  wire                     mask_in,

    // track_assoc publish: one pulse per frame, held until the next one
    input  wire                     trk_valid,
    input  wire [NSLOT-1:0]         trk_found,
    input  wire [NSLOT*9-1:0]       trk_x,
    input  wire [NSLOT*9-1:0]       trk_y,
    input  wire [NSLOT*9-1:0]       trk_w,
    input  wire [NSLOT*9-1:0]       trk_h,

    // person box for the stick overlay: one pulse per frame, held in between
    output reg                      out_valid,
    output reg  [NSLOT-1:0]         out_found,
    output reg  [NSLOT*9-1:0]       out_x,
    output reg  [NSLOT*9-1:0]       out_y,
    output reg  [NSLOT*9-1:0]       out_w,
    output reg  [NSLOT*9-1:0]       out_h,
    output reg  [NSLOT*13-1:0]      out_cells,
    // Which end of a lying figure the head belongs to: the tracked blob is
    // the part that moves, so the body reaches towards the far side of it.
    // Only meaningful while out_w > out_h.
    output reg  [NSLOT-1:0]         out_dir,

    // ---- board-only diagnosticians ------------------------------------
    // Both are sticky "has this ever happened since reset" flags.  They are
    // the difference between "the map never fills" and "the map fills but the
    // sweep reads nothing", which is the one question a board session cannot
    // answer from the published boxes alone.
    output reg                      dbg_pass_hit,   // pass saw a cell in a window
    output reg                      dbg_sweep_hit   // sweep read a live trail cell
);

localparam integer NC = CW * CH;
localparam integer AW = 12;                       // 3,599 needs 12 bits

localparam [1:0]  MINPIX   = MIN_CELL_PIX;
localparam [6:0]  HWMIN    = WIN_HW_MIN;
localparam [6:0]  HHMIN    = WIN_HH_MIN;
localparam [12:0] MINCEL   = MIN_CELLS;
localparam [11:0] MINGLOB  = MIN_GLOBAL;
localparam [3:0]  DECMAX   = DECAY - 1;
localparam [6:0]  LASTC    = CW - 1;              // 79
localparam [5:0]  LASTR    = CH - 1;              // 44
localparam        LASTUD   = 2'd3;
localparam [3:0]  HMULT    = HW_MAX;              // ph cap = pw * HMULT / 2

// ------------------------------------------------------------ pixel stream
reg [8:0] sx;                 // 0 .. WIDTH-1
reg [8:0] sy;                 // 0 .. HEIGHT-1
reg       cur_bank;

// One saturating 2-bit counter per cell column for the cell row being
// streamed.  Two banks ping-pong so the row that just finished can be read
// out (and cleared) while the next row accumulates into the other bank.
reg [1:0] acc_a [0:CW-1];
reg [1:0] acc_b [0:CW-1];

wire [6:0] cs     = sx[8:2];
wire [1:0] acc_v  = cur_bank ? acc_b[cs] : acc_a[cs];
wire [1:0] acc_n  = (acc_v == 2'b11) ? 2'b11 : acc_v + {1'b0, mask_in};

wire row_done   = de_in && (sx == WIDTH - 1);
wire frame_done = row_done && (sy == HEIGHT - 1);
wire cell_done  = row_done && (sy[1:0] == LASTUD);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sx       <= 9'd0;
        sy       <= 9'd0;
        cur_bank <= 1'b0;
    end else if (de_in) begin
        if (row_done) begin
            sx <= 9'd0;
            if (frame_done) sy <= 9'd0;
            else            sy <= sy + 9'd1;
            if (cell_done)  cur_bank <= ~cur_bank;
        end else begin
            sx <= sx + 9'd1;
        end
    end
end

integer ai;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (ai = 0; ai < CW; ai = ai + 1) begin
            acc_a[ai] <= 2'd0;
            acc_b[ai] <= 2'd0;
        end
    end else begin
        if (de_in) begin
            if (cur_bank) acc_b[cs] <= acc_n;
            else          acc_a[cs] <= acc_n;
        end

        // The row readout clears the bank it just finished so the next pass
        // starts from zero.  Both array writes live in this one block: TD
        // treats a second always block touching acc_* as multiple drivers
        // even though the pixel bank and the readout bank are never the
        // same one.
        if (dec_busy) begin
            if (dec_bank) acc_b[dec_c] <= 2'd0;
            else          acc_a[dec_c] <= 2'd0;
        end
    end
end

// ------------------------------------------------- windows from the boxes
// Both banks hold cell-space windows (x0, x1, y0, y1):
//   cur_*  latched at the start of the end-of-frame pass, i.e. the newest
//          publish available then = the boxes of the previous frame
//   pub_*  latched on every trk_valid, used by the sweep that follows it
// The window leans upwards: a tracked blob is usually the legs or an arm,
// and the part of the person the figure needs is above it.
wire [7:0] wr_x0 [0:NSLOT-1];
wire [7:0] wr_x1 [0:NSLOT-1];
wire [7:0] wr_y0 [0:NSLOT-1];
wire [7:0] wr_y1 [0:NSLOT-1];

genvar gi;
generate
    for (gi = 0; gi < NSLOT; gi = gi + 1) begin : g_win
        wire [8:0] bx = trk_x[gi*9 +: 9];
        wire [8:0] by = trk_y[gi*9 +: 9];
        wire [8:0] bw = trk_w[gi*9 +: 9];
        wire [8:0] bh = trk_h[gi*9 +: 9];

        wire [9:0] cx = {1'b0, bx} + {1'b0, bw[8:1]};      // bx + bw/2
        wire [9:0] cy = {1'b0, by} + {1'b0, bh[8:1]};
        wire [6:0] qw = bw[8:2];                            // bw / 4
        wire [6:0] qh = bh[8:2] + bh[8:3];                  // bh/4 + bh/8
        wire [6:0] hw = (qw > HWMIN) ? qw : HWMIN;
        wire [6:0] hh = (qh > HHMIN) ? qh : HHMIN;
        wire [7:0] upp = {1'b0, hh} + {1'b0, hh[6:1]};      // hh + hh/2
        wire [6:0] dnn = hh >> 1;
        wire [6:0] ccx = cx[8:2];                           // centre cell
        wire [6:0] ccy = cy[8:2];

        assign wr_x0[gi] = (ccx > hw) ? (ccx - hw) : 8'd0;
        assign wr_x1[gi] = (({1'b0, ccx} + {1'b0, hw}) < CW)
                           ? ({1'b0, ccx} + {1'b0, hw}) : CW[7:0];
        assign wr_y0[gi] = (ccy > upp) ? ({1'b0, ccy} - upp) : 8'd0;
        assign wr_y1[gi] = (({1'b0, ccy} + {1'b0, dnn}) < CH)
                           ? ({1'b0, ccy} + {1'b0, dnn}) : CH[7:0];
    end
endgenerate

reg [7:0] cur_x0 [0:NSLOT-1];
reg [7:0] cur_x1 [0:NSLOT-1];
reg [7:0] cur_y0 [0:NSLOT-1];
reg [7:0] cur_y1 [0:NSLOT-1];
reg       cur_v  [0:NSLOT-1];

reg [7:0] pub_x0 [0:NSLOT-1];
reg [7:0] pub_x1 [0:NSLOT-1];
reg [7:0] pub_y0 [0:NSLOT-1];
reg [7:0] pub_y1 [0:NSLOT-1];
reg       pub_v  [0:NSLOT-1];

reg [8:0] pub_bx [0:NSLOT-1];
reg [8:0] pub_by [0:NSLOT-1];
reg [8:0] pub_bw [0:NSLOT-1];
reg [8:0] pub_bh [0:NSLOT-1];

// Last figure a slot was allowed to publish.  The model repeats this when
// the track is alive but its trail is still thin, and drops it the moment
// the track itself dies (HOLD_MAX = 0).
reg        held_v [0:NSLOT-1];
reg [8:0]  held_x [0:NSLOT-1];
reg [8:0]  held_y [0:NSLOT-1];
reg [8:0]  held_w [0:NSLOT-1];
reg [8:0]  held_h [0:NSLOT-1];
reg [12:0] held_c [0:NSLOT-1];

// ------------------------------------------------------- the history maps
// hit : one bit per cell, "at least MIN_CELL_PIX pixels moved here"
// live: 2-bit age of the motion trail (3 fresh .. 0 gone)
// snap: the frozen mate of live that the sweep reads
// The three maps live in explicit 9K BRAMs at the end of this file.  TD
// could not infer these 1/2-bit arrays and turned them into 12k flip-flops
// plus 28k LUTs (SYN-5042 in the hdmi_pnr13 log).

// --------------------------------------------------- cell row readout FSM
// At the end of every cell row (4 vision rows) the saturating counters of
// that row are walked one per clock: the flag goes into hit_ram, the
// accumulator is cleared for the next round, and the frame's global motion
// count is accumulated for the decay decision.
reg        dec_busy;
reg [6:0]  dec_c;
reg [5:0]  dec_r;
reg        dec_bank;
reg [11:0] glob_cnt;
reg        run_req;
reg        pub_req;
reg        glob_clr;

wire [1:0]  dec_v    = dec_bank ? acc_b[dec_c] : acc_a[dec_c];
wire        dec_hit  = (dec_v >= MINPIX);
wire [11:0] dec_addr = {dec_r, 6'b0} + {2'b00, dec_r, 4'b0}
                       + {5'b00000, dec_c};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dec_busy <= 1'b0;
        dec_c    <= 7'd0;
        dec_r    <= 6'd0;
        dec_bank <= 1'b0;
        glob_cnt <= 12'd0;
    end else begin
        if (!dec_busy) begin
            if (cell_done) begin
                dec_busy <= 1'b1;
                dec_c    <= 7'd0;
                // sy is 9 bits wide: sy/4 is sy[8:2], not sy[6:2].  The
                // narrow slice folded cell rows 32..44 back onto 0..12 and
                // wiped their hits with the readout's zeros.
                dec_r    <= sy[8:2];
                dec_bank <= cur_bank;
            end
            // The pass engine raises this after it has read the finished
            // count, so the clear still lives next to the only other
            // glob_cnt write instead of in a second always block.
            if (glob_clr) glob_cnt <= 12'd0;
        end else begin
            // The power-on clear owns the array until it has walked it once;
            // the hit write port below is gated by the same boot_done.
            glob_cnt <= glob_cnt + {11'd0, dec_hit};
            if (dec_c == LASTC)
                dec_busy <= 1'b0;
            else
                dec_c <= dec_c + 7'd1;
        end
    end
end

// ------------------------------------------------------------- pass engine
localparam [2:0] S_BOOT  = 3'd0;
localparam [2:0] S_IDLE  = 3'd1;
localparam [2:0] S_RUN   = 3'd2;
localparam [2:0] S_RTAIL = 3'd3;
localparam [2:0] S_WAIT  = 3'd4;
localparam [2:0] S_SWEEP = 3'd5;
localparam [2:0] S_STAIL = 3'd6;
localparam [2:0] S_FIN   = 3'd7;

reg [2:0]  st;
reg        boot_done;
reg [11:0] boot_a;
reg [5:0]  pr;                 // cell row being walked
reg [6:0]  pc;                 // cell column being walked
reg [11:0] paddr;              // address presented to the RAMs
reg        pv;                 // a read is in flight
reg [11:0] daddr;              // address that came back
reg [6:0]  dc;
reg [5:0]  dr;
reg        dv;
wire [1:0] d_live;
wire [1:0] d_snap;
wire       d_hit;
reg        decay_en;
reg [3:0]  dec_cnt;
reg [3:0]  fin_i;

wire pc_last = (pc == LASTC);
wire [6:0] pc_n = pc_last ? 7'd0 : pc + 7'd1;
wire [5:0] pr_n = pc_last ? (pr + 6'd1) : pr;
wire [11:0] paddr_n = {pr_n, 6'b0} + {2'b00, pr_n, 4'b0}
                      + {5'b00000, pc_n};
wire at_last = (pr == LASTR) && pc_last;

// window membership of the cell that just came back from the map
reg        inwin_cur;
reg [NSLOT-1:0] inwin_pub;
integer wi;
always @* begin
    inwin_cur = 1'b0;
    inwin_pub = {NSLOT{1'b0}};
    for (wi = 0; wi < NSLOT; wi = wi + 1) begin
        if (cur_v[wi] && (dc >= cur_x0[wi]) && (dc < cur_x1[wi]) &&
            (dr >= cur_y0[wi]) && (dr < cur_y1[wi]))
            inwin_cur = 1'b1;
        if (pub_v[wi] && (dc >= pub_x0[wi]) && (dc < pub_x1[wi]) &&
            (dr >= pub_y0[wi]) && (dr < pub_y1[wi]))
            inwin_pub[wi] = 1'b1;
    end
end

// One cell of the end-of-frame pass: this frame's flag writes a 3, then the
// decay sweep (when it runs) drops every non-zero cell by one.  The same
// value is written back to the live map and to the snapshot the sweep reads.
wire       app_v  = d_hit & inwin_cur;
wire [1:0] nxt_v  = app_v ? 2'd3 : d_live;
wire [1:0] nxt_dv = (decay_en && (nxt_v != 2'd0)) ? (nxt_v - 2'd1) : nxt_v;

// sweep accumulators
reg [12:0] sw_cnt  [0:NSLOT-1];
reg [6:0]  sw_minx [0:NSLOT-1];
reg [6:0]  sw_maxx [0:NSLOT-1];
reg [5:0]  sw_miny [0:NSLOT-1];
reg [5:0]  sw_maxy [0:NSLOT-1];

// --------------------------------------------------- finalise arithmetic
// The combinational block below is only meaningful while a slot is being
// finalised; the clamp keeps the array index in range in the last cycle.
wire [1:0] fidx = (fin_i < NSLOT) ? fin_i[1:0] : 2'd0;
integer ki;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st        <= S_BOOT;
        boot_done <= 1'b0;
        boot_a    <= 12'd0;
        pr        <= 6'd0;
        pc        <= 7'd0;
        paddr     <= 12'd0;
        pv        <= 1'b0;
        daddr     <= 12'd0;
        dc        <= 7'd0;
        dr        <= 6'd0;
        dv        <= 1'b0;
        decay_en  <= 1'b0;
        dec_cnt   <= 4'd0;
        fin_i     <= 4'd0;
        out_valid <= 1'b0;
        out_found <= {NSLOT{1'b0}};
        out_x     <= {(NSLOT*9){1'b0}};
        out_y     <= {(NSLOT*9){1'b0}};
        out_w     <= {(NSLOT*9){1'b0}};
        out_h     <= {(NSLOT*9){1'b0}};
        out_cells <= {(NSLOT*13){1'b0}};
        out_dir   <= {NSLOT{1'b0}};
        dbg_pass_hit  <= 1'b0;
        dbg_sweep_hit <= 1'b0;
        for (ki = 0; ki < NSLOT; ki = ki + 1) begin
            cur_x0[ki] <= 8'd0; cur_x1[ki] <= 8'd0;
            cur_y0[ki] <= 8'd0; cur_y1[ki] <= 8'd0;
            cur_v[ki]  <= 1'b0;
            pub_x0[ki] <= 8'd0; pub_x1[ki] <= 8'd0;
            pub_y0[ki] <= 8'd0; pub_y1[ki] <= 8'd0;
            pub_v[ki]  <= 1'b0;
            pub_bx[ki] <= 9'd0; pub_by[ki] <= 9'd0;
            pub_bw[ki] <= 9'd0; pub_bh[ki] <= 9'd0;
            held_v[ki] <= 1'b0;
            held_x[ki] <= 9'd0; held_y[ki] <= 9'd0;
            held_w[ki] <= 9'd0; held_h[ki] <= 9'd0;
            held_c[ki] <= 13'd0;
            sw_cnt[ki]  <= 13'd0;
            sw_minx[ki] <= 7'd0; sw_maxx[ki] <= 7'd0;
            sw_miny[ki] <= 6'd0; sw_maxy[ki] <= 6'd0;
        end
    end else begin
        out_valid <= 1'b0;
        glob_clr  <= 1'b0;

        // ------------------------------------------------ registered reads
        // Port B of the map BRAMs is the registered read: the address wired
        // this cycle comes back on the next one, exactly like the old
        // d_* <= ram[paddr] assignments.
        daddr  <= paddr;
        dc     <= pc;
        dr     <= pr;
        dv     <= pv;

        if (frame_done) run_req <= 1'b1;

        // Sticky sweep evidence: the frozen map handed the sweep a live cell
        // at least once.  d_snap is only consumed in these two states.
        if (((st == S_SWEEP) || (st == S_STAIL)) && dv && (d_snap != 2'd0))
            dbg_sweep_hit <= 1'b1;

        // -------------------------------------------- tracker publish side
        if (trk_valid) begin
            pub_req <= 1'b1;
            for (ki = 0; ki < NSLOT; ki = ki + 1) begin
                pub_x0[ki] <= wr_x0[ki];
                pub_x1[ki] <= wr_x1[ki];
                pub_y0[ki] <= wr_y0[ki];
                pub_y1[ki] <= wr_y1[ki];
                pub_v[ki]  <= trk_found[ki];
                pub_bx[ki] <= trk_x[ki*9 +: 9];
                pub_by[ki] <= trk_y[ki*9 +: 9];
                pub_bw[ki] <= trk_w[ki*9 +: 9];
                pub_bh[ki] <= trk_h[ki*9 +: 9];
            end
        end

        case (st)
        // ------------------------------------------------ power-on clear
        S_BOOT: begin
            pv <= 1'b0;
            if (boot_a == NC - 1) begin
                boot_done <= 1'b1;
                paddr     <= 12'd0;
                st        <= S_IDLE;
            end else begin
                boot_a <= boot_a + 12'd1;
                paddr  <= boot_a + 12'd1;
            end
        end

        // ------------------------------------------------ wait for a frame
        S_IDLE: begin
            if (run_req && !dec_busy && boot_done) begin
                run_req  <= 1'b0;
                decay_en <= (dec_cnt == DECMAX) &&
                            (glob_cnt >= MINGLOB);
                dec_cnt  <= (dec_cnt == DECMAX) ? 4'd0 : dec_cnt + 4'd1;
                glob_clr <= 1'b1;
                for (ki = 0; ki < NSLOT; ki = ki + 1) begin
                    cur_x0[ki] <= wr_x0[ki];
                    cur_x1[ki] <= wr_x1[ki];
                    cur_y0[ki] <= wr_y0[ki];
                    cur_y1[ki] <= wr_y1[ki];
                    cur_v[ki]  <= trk_found[ki];
                end
                pr    <= 6'd0;
                pc    <= 7'd0;
                paddr <= 12'd0;
                pv    <= 1'b1;
                st    <= S_RUN;
            end
        end

        // ------------------------- apply this frame's flags + decay + copy
        S_RUN: begin
            if (dv && app_v) dbg_pass_hit <= 1'b1;
            if (at_last) begin
                pv <= 1'b0;
                st <= S_RTAIL;
            end else begin
                pr    <= pr_n;
                pc    <= pc_n;
                paddr <= paddr_n;
            end
        end

        S_RTAIL: begin
            st <= S_WAIT;
        end

        // --------------------------------- wait for this frame's boxes
        S_WAIT: begin
            if (pub_req) begin
                pub_req <= 1'b0;
                pr    <= 6'd0;
                pc    <= 7'd0;
                paddr <= 12'd0;
                pv    <= 1'b1;
                for (ki = 0; ki < NSLOT; ki = ki + 1) begin
                    sw_cnt[ki]  <= 13'd0;
                    sw_minx[ki] <= 7'd0; sw_maxx[ki] <= 7'd0;
                    sw_miny[ki] <= 6'd0; sw_maxy[ki] <= 6'd0;
                end
                st <= S_SWEEP;
            end
        end

        // -------------------------------------- box of the live trail
        S_SWEEP: begin
            if (dv && (d_snap != 2'd0)) begin
                for (ki = 0; ki < NSLOT; ki = ki + 1) begin
                    if (inwin_pub[ki]) begin
                        sw_cnt[ki] <= sw_cnt[ki] + 13'd1;
                        if (sw_cnt[ki] == 13'd0) begin
                            sw_minx[ki] <= dc;  sw_maxx[ki] <= dc;
                            sw_miny[ki] <= dr;  sw_maxy[ki] <= dr;
                        end else begin
                            if (dc < sw_minx[ki]) sw_minx[ki] <= dc;
                            if (dc > sw_maxx[ki]) sw_maxx[ki] <= dc;
                            if (dr < sw_miny[ki]) sw_miny[ki] <= dr;
                            if (dr > sw_maxy[ki]) sw_maxy[ki] <= dr;
                        end
                    end
                end
            end
            if (at_last) begin
                pv <= 1'b0;
                st <= S_STAIL;
            end else begin
                pr    <= pr_n;
                pc    <= pc_n;
                paddr <= paddr_n;
            end
        end

        S_STAIL: begin
            if (dv && (d_snap != 2'd0)) begin
                for (ki = 0; ki < NSLOT; ki = ki + 1) begin
                    if (inwin_pub[ki]) begin
                        sw_cnt[ki] <= sw_cnt[ki] + 13'd1;
                        if (sw_cnt[ki] == 13'd0) begin
                            sw_minx[ki] <= dc;  sw_maxx[ki] <= dc;
                            sw_miny[ki] <= dr;  sw_maxy[ki] <= dr;
                        end else begin
                            if (dc < sw_minx[ki]) sw_minx[ki] <= dc;
                            if (dc > sw_maxx[ki]) sw_maxx[ki] <= dc;
                            if (dr < sw_miny[ki]) sw_miny[ki] <= dr;
                            if (dr > sw_maxy[ki]) sw_maxy[ki] <= dr;
                        end
                    end
                end
            end
            fin_i <= 4'd0;
            st    <= S_FIN;
        end

        // --------------------------- one slot per clock, then publish all
        S_FIN: begin
            if (fin_i < NSLOT) begin
                if (!pub_v[fin_i]) begin
                    // Track is gone: drop the figure at once (HOLD_MAX = 0).
                    out_found[fin_i*1 +: 1]  <= 1'b0;
                    out_cells[fin_i*13 +: 13] <= 13'd0;
                    out_x[fin_i*9 +: 9] <= 9'd0;
                    out_y[fin_i*9 +: 9] <= 9'd0;
                    out_w[fin_i*9 +: 9] <= 9'd0;
                    out_h[fin_i*9 +: 9] <= 9'd0;
                    out_dir[fin_i*1 +: 1] <= 1'b0;
                    held_v[fin_i] <= 1'b0;
                end else if (sw_cnt[fin_i] < MINCEL) begin
                    // Live track, too little trail: repeat the last trusted
                    // figure, or report nothing if there never was one.
                    out_found[fin_i*1 +: 1]  <= held_v[fin_i];
                    out_cells[fin_i*13 +: 13] <=
                        held_v[fin_i] ? held_c[fin_i] : 13'd0;
                    out_x[fin_i*9 +: 9] <= held_v[fin_i] ? held_x[fin_i]
                                                         : 9'd0;
                    out_y[fin_i*9 +: 9] <= held_v[fin_i] ? held_y[fin_i]
                                                         : 9'd0;
                    out_w[fin_i*9 +: 9] <= held_v[fin_i] ? held_w[fin_i]
                                                         : 9'd0;
                    out_h[fin_i*9 +: 9] <= held_v[fin_i] ? held_h[fin_i]
                                                         : 9'd0;
                    out_dir[fin_i*1 +: 1] <= held_v[fin_i] ? dir_held
                                                           : 1'b0;
                end else begin
                    out_found[fin_i*1 +: 1]  <= 1'b1;
                    out_cells[fin_i*13 +: 13] <= sw_cnt[fin_i];
                    out_x[fin_i*9 +: 9] <= px0_w;
                    out_y[fin_i*9 +: 9] <= py0_w;
                    out_w[fin_i*9 +: 9] <= pw_w;
                    out_h[fin_i*9 +: 9] <= ph_w;
                    out_dir[fin_i*1 +: 1] <= dir_new;
                    held_v[fin_i] <= 1'b1;
                    held_x[fin_i] <= px0_w;
                    held_y[fin_i] <= py0_w;
                    held_w[fin_i] <= pw_w;
                    held_h[fin_i] <= ph_w;
                    held_c[fin_i] <= sw_cnt[fin_i];
                end
                fin_i <= fin_i + 4'd1;
            end else begin
                out_valid <= 1'b1;
                st        <= S_IDLE;
            end
        end

        default: st <= S_BOOT;
        endcase
    end
end

// ------------------------------------------------- finalise slot geometry
wire [8:0] fbx = pub_bx[fidx];
wire [8:0] fby = pub_by[fidx];
wire [8:0] fbw = pub_bw[fidx];
wire [8:0] fbh = pub_bh[fidx];

wire [9:0] lx0 = {1'b0, sw_minx[fidx]} << 2;      // (minx)     * 4
wire [9:0] ly0 = {1'b0, sw_miny[fidx]} << 2;
wire [9:0] lx1 = ({3'b000, sw_maxx[fidx]} + 10'd1) << 2;   // (maxx+1)*4
wire [9:0] ly1 = ({4'b0000, sw_maxy[fidx]} + 10'd1) << 2;

wire [9:0] px0 = (lx0 < {1'b0, fbx}) ? lx0 : {1'b0, fbx};
wire [9:0] py0r = (ly0 < {1'b0, fby}) ? ly0 : {1'b0, fby};
wire [9:0] pxe = {1'b0, fbx} + {1'b0, fbw};
wire [9:0] pye = {1'b0, fby} + {1'b0, fbh};
wire [9:0] px1 = (lx1 > pxe) ? lx1 : pxe;
wire [9:0] py1 = (ly1 > pye) ? ly1 : pye;

wire [9:0] pw_c = px1 - px0;
wire [9:0] ph_c = py1 - py0r;
wire [10:0] cap_c = (({2'b00, pw_c}) + ({pw_c, 2'b0})) >> 1;   // pw*5/2

wire capped = (pw_c <= ph_c) && (ph_c > cap_c[9:0]);
wire [9:0] ph_f = capped ? cap_c[9:0] : ph_c;
wire [9:0] py0_f = capped ? (py1 - cap_c[9:0]) : py0r;

// The published bus is 9 bits per field (the tracker's own convention); the
// union of a cell box and a tracked box can only reach 320/180 pixels, so
// the clamp is a formality that keeps an out-of-range track from wrapping.
wire [8:0] px0_w = (px0 > 319) ? 9'd319 : px0[8:0];
wire [8:0] py0_w = (py0_f > 179) ? 9'd179 : py0_f[8:0];
wire [8:0] pw_w  = (pw_c > 319) ? 9'd319 : pw_c[8:0];
wire [8:0] ph_w  = (ph_f > 179) ? 9'd179 : ph_f[8:0];

// Doubled centres: 2*bx + bw > 2*px + pw is exactly the prototype's
// "tracked blob centre is past the middle of the person box" test without
// the fractional halves.
wire [10:0] fcx2  = {fbx, 1'b0} + {2'b00, fbw};
wire [10:0] pcx2  = {px0_w, 1'b0} + {2'b00, pw_w};
wire [10:0] hcx2  = {held_x[fidx], 1'b0} + {2'b00, held_w[fidx]};
// The model only reports a head end for a lying figure, so an upright box
// publishes 0 here even though the comparison itself still has a meaning.
wire        dir_new  = (pw_w > ph_w) && (fcx2 > pcx2);
wire        dir_held = (held_w[fidx] > held_h[fidx]) && (fcx2 > hcx2);

// ------------------------------------------------------------- map storage
// TD cannot infer 1/2-bit arrays into ERAM (SYN-5042), so the three maps use
// the 9K BRAM primitive directly.  Port A writes and port B reads; both run
// on the vision clock, and their addresses differ by one step, so the block
// RAM's registered read has exactly the timing the old flip-flop arrays had.
// REGMODE_B must be OUTREG for that: the pass FSM advances the address every
// clock and consumes the data one clock later, so a NOREG read would return
// the wrong cell while the address and its row/column metadata move on.
// Three 9K blocks replace 18k flip-flops and roughly 28k LUTs.

wire        boot_clear = (st == S_BOOT);
wire        dec_write  = dec_busy && boot_done;
wire        pass_write = ((st == S_RUN) || (st == S_RTAIL)) && dv;

wire [11:0] hit_wa = boot_clear ? boot_a : dec_addr;
wire        hit_we = boot_clear || dec_write;
wire        hit_wd = boot_clear ? 1'b0 : dec_hit;
wire [8:0]  hit_dob;

wire [11:0] live_wa = boot_clear ? boot_a : daddr;
wire        live_we = boot_clear || pass_write;
wire [1:0]  live_wd = boot_clear ? 2'd0 : nxt_dv;
wire [8:0]  live_dob;

wire [11:0] snap_wa = live_wa;
wire        snap_we = live_we;
wire [1:0]  snap_wd = live_wd;
wire [8:0]  snap_dob;

assign d_live = live_dob[1:0];
assign d_snap = snap_dob[1:0];
assign d_hit  = hit_dob[0];

EG_PHY_BRAM #(
    .MODE         ("DP8K"),
    .DATA_WIDTH_A ("1"),
    .DATA_WIDTH_B ("1"),
    .READBACK     ("OFF"),
    .REGMODE_A    ("NOREG"),
    .REGMODE_B    ("OUTREG"),
    .WRITEMODE_A  ("NORMAL"),
    .WRITEMODE_B  ("NORMAL")
) u_hit_map (
    .doa   (),
    .dob   (hit_dob),
    .dia   ({8'b0, hit_wd}),
    .dib   (9'd0),
    .csa   (3'b111),
    .csb   (3'b111),
    .cea   (1'b1),
    .ocea  (1'b1),
    .clka  (clk),
    .wea   (hit_we),
    .rsta  (1'b0),
    .ceb   (1'b1),
    .oceb  (1'b1),
    .clkb  (clk),
    .web   (1'b0),
    .rstb  (1'b0),
    .addra ({1'b0, hit_wa}),
    .addrb ({1'b0, paddr})
);

EG_PHY_BRAM #(
    .MODE         ("DP8K"),
    .DATA_WIDTH_A ("2"),
    .DATA_WIDTH_B ("2"),
    .READBACK     ("OFF"),
    .REGMODE_A    ("NOREG"),
    .REGMODE_B    ("OUTREG"),
    .WRITEMODE_A  ("NORMAL"),
    .WRITEMODE_B  ("NORMAL")
) u_live_map (
    .doa   (),
    .dob   (live_dob),
    .dia   ({7'b0, live_wd}),
    .dib   (9'd0),
    .csa   (3'b111),
    .csb   (3'b111),
    .cea   (1'b1),
    .ocea  (1'b1),
    .clka  (clk),
    .wea   (live_we),
    .rsta  (1'b0),
    .ceb   (1'b1),
    .oceb  (1'b1),
    .clkb  (clk),
    .web   (1'b0),
    .rstb  (1'b0),
    .addra ({1'b0, live_wa}),
    .addrb ({1'b0, paddr})
);

EG_PHY_BRAM #(
    .MODE         ("DP8K"),
    .DATA_WIDTH_A ("2"),
    .DATA_WIDTH_B ("2"),
    .READBACK     ("OFF"),
    .REGMODE_A    ("NOREG"),
    .REGMODE_B    ("OUTREG"),
    .WRITEMODE_A  ("NORMAL"),
    .WRITEMODE_B  ("NORMAL")
) u_snap_map (
    .doa   (),
    .dob   (snap_dob),
    .dia   ({7'b0, snap_wd}),
    .dib   (9'd0),
    .csa   (3'b111),
    .csb   (3'b111),
    .cea   (1'b1),
    .ocea  (1'b1),
    .clka  (clk),
    .wea   (snap_we),
    .rsta  (1'b0),
    .ceb   (1'b1),
    .oceb  (1'b1),
    .clkb  (clk),
    .web   (1'b0),
    .rstb  (1'b0),
    .addra ({1'b0, snap_wa}),
    .addrb ({1'b0, paddr})
);

endmodule
