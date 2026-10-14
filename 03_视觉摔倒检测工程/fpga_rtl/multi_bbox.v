//===========================================================================
// multi_bbox.v -- top-N motion-blob bounding boxes, all on-chip
//
// The solo detector (proj_bbox.v) answers "where is the one dominant blob".
// A crowd needs several: airport footage has many simultaneous movers, and a
// single box either locks onto the wrong person or stretches across two.
//
// This module keeps proj_bbox's proven pieces -- one packed mask bank pair in
// ERAM, the row projection that picks the vertical band, and the two-cycle-
// per-column histogram scan -- and changes only the pass side:
//
//   pass 0  finds the dominant blob exactly like proj_bbox
//   pass 1  re-scans with pass 0's box (plus MARGIN) excluded
//   pass 2  re-scans with passes 0..1 excluded
//   pass 3  re-scans with passes 0..2 excluded
//
// ... and then trims every pass vertically.  The row projection that freezes
// a frame's vertical band answers "where is there motion", not "how tall is
// this mover": inside a crowd the band spans every person it contains, so a
// box drawn from band_start..band_start+band_len wraps one walker, the row
// above them and the row below.  Each pass therefore continues with a second
// scan over just the columns of the run it just found -- the same pixel
// pipeline, reused -- and keeps the longest run of rows that carry at least
// MIN_PIX of that run's pixels.  The published box and the exclusion
// rectangle both use that span, and the reported area is the pixels inside
// it, so fill = area/(w*h) is again a meaningful posture cue.
//
// Exclusion is a rectangle test on the mask bit as it leaves the read port,
// so it costs a handful of comparators, not a second mask buffer.  A pixel
// inside a lower-numbered box contributes nothing to the column histogram,
// which also drops it from the column run and from the reported area.
//
// Each result keeps its own validity, so a frame with two people publishes
// two found boxes and two empty slots -- exactly what a per-slot tracker
// wants.  Slots are ordered by area within the frame, not by identity: slot 0
// is always the strongest blob this frame.  Hysteresis across frames is the
// tracker's job (track_assoc), not this module's.
//
// Cost vs proj_bbox: same RAM, same histogram, NUM_BLOBS passes per frame
// instead of one.  A dense band pass is ~26k clocks, so four passes are
// ~104k clocks against a 590k-clock 640x480@30 frame -- it still finishes
// inside vertical blanking with room to spare.
//
// Mask stream contract (same as proj_bbox): one pixel per de_in, WIDTH per
// row, HEIGHT rows per frame, no frame_start strobe.
//===========================================================================
`timescale 1ns / 1ps

module multi_bbox #(
    parameter integer WIDTH    = 320,
    parameter integer HEIGHT   = 180,
    // Defaults are the shipped board configuration (fall_camera_hdmi_multi_top
    // passes 4/160 explicitly).  The Python twins in fpga/tools/multi_model.py
    // carry the same numbers, so an override on one side has to move both.
    parameter integer MIN_PIX  = 4,
    parameter integer MIN_AREA = 160,
    parameter integer NUM_BLOBS = 4,
    parameter integer MARGIN    = 8
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       de_in,
    input  wire       mask_in,
    output reg        busy,

    // One pulse per frame; all NUM_BLOBS slots are valid with it.
    output reg                     out_valid,
    output reg  [NUM_BLOBS-1:0]    out_found,
    output reg  [NUM_BLOBS*9-1:0]  out_x,
    output reg  [NUM_BLOBS*9-1:0]  out_y,
    output reg  [NUM_BLOBS*9-1:0]  out_w,
    output reg  [NUM_BLOBS*9-1:0]  out_h,
    output reg  [NUM_BLOBS*17-1:0] out_area
);

localparam integer BPR    = WIDTH / 8;
localparam integer NBYTES = HEIGHT * BPR;
localparam integer AW     = $clog2(2 * NBYTES);
localparam [AW-1:0] BANK_OFF = NBYTES;

localparam [2:0] ST_IDLE   = 3'd0;
localparam [2:0] ST_CLEAR  = 3'd1;
localparam [2:0] ST_ACC    = 3'd2;
localparam [2:0] ST_SPAN   = 3'd3;
localparam [2:0] ST_VSPAN  = 3'd4;

localparam integer PW = $clog2(NUM_BLOBS);

// ------------------------------------------------------------- mask storage
// Identical geometry to proj_bbox: one array, two banks, single write and
// single read port, hinted at the 9K region.  See proj_bbox.v for why the
// address port must stay AW bits wide and why the hint matters.
(* ram_style = "bram_9k" *) reg [7:0] mask_ram [0:2*NBYTES-1];

// ============================================================== stream side
reg        wr_bank;
reg [AW-1:0] waddr;
wire [AW-1:0] waddr_full = wr_bank ? (waddr + BANK_OFF) : waddr;
reg [2:0]  wbit;
reg [7:0]  wshift;
reg [8:0]  sx, sy;
reg [8:0]  row_cnt;
wire [8:0] row_cnt_next = row_cnt + {8'd0, mask_in};

reg        run_active;
reg [8:0]  cur_start;
reg [9:0]  cur_len;
reg        best_valid;
reg [8:0]  best_start;
reg [9:0]  best_len;

wire row_active = (row_cnt_next >= MIN_PIX);

reg        nr_active;
reg [8:0]  nr_start;
reg [9:0]  nr_len;
reg        close_run;
reg [8:0]  close_start;
reg [9:0]  close_len;

always @* begin
    nr_active   = run_active;
    nr_start    = cur_start;
    nr_len      = cur_len;
    close_run   = 1'b0;
    close_start = cur_start;
    close_len   = cur_len;
    if (row_active) begin
        if (!run_active) begin
            nr_active = 1'b1;
            nr_start  = sy;
            nr_len    = 10'd1;
        end else begin
            nr_len = cur_len + 10'd1;
        end
    end else if (run_active) begin
        nr_active = 1'b0;
        close_run = 1'b1;
    end
end

reg [8:0]  fe_start;
reg [9:0]  fe_len;
always @* begin
    fe_start = best_valid ? best_start : 9'd0;
    fe_len   = best_valid ? best_len   : 10'd0;
    if (nr_active) begin
        if (!best_valid || (nr_len > best_len)) begin
            fe_start = nr_start;
            fe_len   = nr_len;
        end
    end else if (close_run) begin
        if (!best_valid || (close_len > best_len)) begin
            fe_start = close_start;
            fe_len   = close_len;
        end
    end
end

reg        frame_done;
reg [8:0]  done_start;
reg [9:0]  done_len;
reg        done_bank;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_bank    <= 1'b0;
        waddr      <= {AW{1'b0}};
        wbit       <= 3'd0;
        wshift     <= 8'd0;
        sx         <= 9'd0;
        sy         <= 9'd0;
        row_cnt    <= 9'd0;
        run_active <= 1'b0;
        cur_start  <= 9'd0;
        cur_len    <= 10'd0;
        best_valid <= 1'b0;
        best_start <= 9'd0;
        best_len   <= 10'd0;
        frame_done <= 1'b0;
        done_start <= 9'd0;
        done_len   <= 10'd0;
        done_bank  <= 1'b0;
    end else begin
        frame_done <= 1'b0;

        if (de_in) begin
            wshift <= {wshift[6:0], mask_in};
            if (wbit == 3'd7) begin
                mask_ram[waddr_full] <= {wshift[6:0], mask_in};
                waddr <= waddr + 14'd1;
                wbit  <= 3'd0;
            end else begin
                wbit <= wbit + 3'd1;
            end

            row_cnt <= row_cnt_next;

            if (sx == WIDTH - 1) begin
                sx      <= 9'd0;
                row_cnt <= 9'd0;
                if (sy == HEIGHT - 1) begin
                    sy         <= 9'd0;
                    waddr      <= {AW{1'b0}};
                    wbit       <= 3'd0;
                    done_start <= fe_start;
                    done_len   <= fe_len;
                    done_bank  <= wr_bank;
                    frame_done <= 1'b1;
                    wr_bank    <= ~wr_bank;
                    run_active <= 1'b0;
                    cur_len    <= 10'd0;
                    best_valid <= 1'b0;
                    best_len   <= 10'd0;
                end else begin
                    sy         <= sy + 9'd1;
                    run_active <= nr_active;
                    cur_start  <= nr_start;
                    cur_len    <= nr_len;
                    if (close_run &&
                        (!best_valid || (close_len > best_len))) begin
                        best_valid <= 1'b1;
                        best_start <= close_start;
                        best_len   <= close_len;
                    end
                end
            end else begin
                sx <= sx + 9'd1;
            end
        end
    end
end

// ========================================================== exclusion table
// Boxes from the passes already completed (this frame) and from the previous
// frame.  Pass k excludes slots 0..k-1 only, which is what makes the sequence
// a chain: pass 1 cannot exclude its own result because it has none yet, and
// a slot that found nothing this frame keeps last frame's rectangle, so a
// person who stops moving does not immediately reappear in the next slot.
reg        ex_valid [0:NUM_BLOBS-1];
reg [8:0]  ex_x     [0:NUM_BLOBS-1];
reg [8:0]  ex_y     [0:NUM_BLOBS-1];
reg [9:0]  ex_x1    [0:NUM_BLOBS-1];
reg [9:0]  ex_y1    [0:NUM_BLOBS-1];

// =============================================================== pass side
reg [2:0]  state;
reg [8:0]  band_start;
reg [9:0]  band_len;
reg        rd_bank;
reg [PW-1:0] pass_idx;
reg [PW:0]   pass_count;

reg [8:0]  clr_idx;
reg [8:0]  acc_row;
reg [8:0]  acc_col;
reg        acc_drain;

// ST_VSPAN reuses acc_row/acc_col (and therefore pix_rd_index and the
// exclusion comparators) to walk the chosen column run one row at a time.
// vcnt counts this row's non-excluded hits; the r_* registers keep the
// longest row run seen so far, exactly like the s_* column scan.
//
// res_start/res_len are combinational results of the ST_SPAN walk and keep
// moving once that state is left (the still-active run counter looks one
// column longer), so the run has to be latched here before the row scan
// starts.  vrun_end falls out of that latch.
reg [8:0]  vrun_start;
reg [8:0]  vrun_end;
reg [9:0]  vrun_len;
reg        vdrain;
reg [1:0]  vdrain_cnt;
reg [9:0]  vcnt;

reg        r_run_active;
reg [8:0]  r_run_start;
reg [9:0]  r_run_len;
reg [17:0] r_run_sum;
reg [8:0]  r_best_start;
reg [9:0]  r_best_len;
reg [17:0] r_best_sum;

// A histogram entry counts row hits for one column inside the selected band,
// so it can never exceed HEIGHT.  Keeping it HEIGHT-wide instead of 16 bits
// nearly halves the read mux and the incrementer that used to sit on the
// 48 MHz critical path.
//
// The array is hinted (and, just as importantly, *not* reset entry by entry)
// so TD can fold it into ERAM like mask_ram: the read address is the stage-A
// register and the data lands in stage-B, and ST_CLEAR already zeroes every
// entry before the first pass of every frame.  Leaving a reset loop in here
// turns all 2,560 bits into flip-flops, which is what made the router fail at
// 100% BRAM32K / 86% cell area.
localparam integer CW = $clog2(HEIGHT + 1);
(* ram_style = "bram_9k" *) reg [CW-1:0] hist [0:WIDTH-1];

// Three-stage pixel pipeline, one pixel per clock:
//
//   A  a_addr/a_col/a_excl <- the band cell for the current acc_col
//   B  b_byte/b_hist/b_col <- mask_ram[a_addr] / hist[a_col]
//   C  hist[b_col]         <- b_hist + 1
//
// The address stage is not decoration.  TD only folds mask_ram into ERAM when
// the read address is a register feeding a registered read (`rd <= ram[addr]`,
// exactly what proj_bbox.v does); reading straight off the live column counter
// turns the whole 14,400-byte bank into flip-flops.  Splitting the stages also
// puts a register between the 320-way hist read mux and the incrementer, which
// is what the -1.1 ns / 21.7 ns `hist_reg` path needed.
reg [AW-1:0] a_addr;
reg [8:0]    a_col;
reg          a_excl;
reg          a_valid;
reg [7:0]    b_byte;
reg [CW-1:0] b_hist;
reg [8:0]    b_col;
reg          b_excl;
reg          b_valid;
reg [1:0]    drain_cnt;

wire [5:0]  acc_byte_idx = acc_col[8:3];
wire [AW-1:0] pix_rd_addr = ((band_start + acc_row) * BPR) + acc_byte_idx;
wire [AW-1:0] pix_rd_index = rd_bank ? (pix_rd_addr + BANK_OFF)
                                     : pix_rd_addr;

// Per-pixel exclusion: is (acc_col, band_start+acc_row) inside any rectangle
// with index < pass_idx?
reg     hit_c;
integer ei;
always @* begin
    hit_c = 1'b0;
    for (ei = 0; ei < NUM_BLOBS; ei = ei + 1) begin
        if ((ei < pass_idx) && ex_valid[ei] &&
            (acc_col >= ex_x[ei]) && (acc_col < ex_x1[ei]) &&
            ((band_start + acc_row) >= ex_y[ei]) &&
            ((band_start + acc_row) < ex_y1[ei])) begin
            hit_c = 1'b1;
        end
    end
end

reg [8:0]  sc;
reg        s_run_active;
reg [8:0]  s_run_start;
reg [9:0]  s_run_len;
reg [17:0] s_run_sum;
reg [8:0]  s_best_start;
reg [9:0]  s_best_len;
reg [17:0] s_best_sum;

reg [CW-1:0] hist_q;
reg        span_phase;
wire [CW-1:0] hist_scan = hist_q;
wire col_active = (hist_scan >= MIN_PIX);

reg        ns_active;
reg [8:0]  ns_start;
reg [9:0]  ns_len;
reg [17:0] ns_sum;
reg        s_close;
reg [8:0]  s_close_start;
reg [9:0]  s_close_len;
reg [17:0] s_close_sum;

always @* begin
    ns_active     = s_run_active;
    ns_start      = s_run_start;
    ns_len        = s_run_len;
    ns_sum        = s_run_sum;
    s_close       = 1'b0;
    s_close_start = s_run_start;
    s_close_len   = s_run_len;
    s_close_sum   = s_run_sum;
    if (col_active) begin
        if (!s_run_active) begin
            ns_active = 1'b1;
            ns_start  = sc;
            ns_len    = 10'd1;
            ns_sum    = hist_scan;
        end else begin
            ns_len = s_run_len + 10'd1;
            ns_sum = s_run_sum + hist_scan;
        end
    end else if (s_run_active) begin
        ns_active = 1'b0;
        s_close   = 1'b1;
    end
end

wire final_new = col_active
                 ? (!(s_best_len != 10'd0) || (ns_len > s_best_len))
                 : (s_close &&
                    (!(s_best_len != 10'd0) || (s_close_len > s_best_len)));
wire [8:0]  final_start = col_active ? ns_start : s_close_start;
wire [9:0]  final_len   = col_active ? ns_len   : s_close_len;
wire [17:0] final_sum   = col_active ? ns_sum   : s_close_sum;

wire [8:0]  res_start = final_new ? final_start : s_best_start;
wire [9:0]  res_len   = final_new ? final_len   : s_best_len;
wire [17:0] res_sum   = final_new ? final_sum   : s_best_sum;

// --------------------------------------------------- row scan (ST_VSPAN)
// The stage-C condition for the pixel leaving B.  It is folded into vcnt the
// same way the histogram entry is above, and it is added combinationally when
// a row closes because the vcnt register updates on that same edge.
wire       v_pix_now    = b_valid && b_byte[7 - b_col[2:0]] && !b_excl;
wire [9:0] v_row_cnt    = vcnt + {9'd0, v_pix_now};
wire       v_row_active = (v_row_cnt >= MIN_PIX);

// Longest-run tracker for the rows of the selected column run ("run" is
// valid while its length is non-zero, exactly like s_best_len below).
reg        nr2_active;
reg [8:0]  nr2_start;
reg [9:0]  nr2_len;
reg [17:0] nr2_sum;
reg        r_close;
reg [8:0]  r_close_start;
reg [9:0]  r_close_len;
reg [17:0] r_close_sum;

always @* begin
    nr2_active    = r_run_active;
    nr2_start     = r_run_start;
    nr2_len       = r_run_len;
    nr2_sum       = r_run_sum;
    r_close       = 1'b0;
    r_close_start = r_run_start;
    r_close_len   = r_run_len;
    r_close_sum   = r_run_sum;
    if (v_row_active) begin
        if (!r_run_active) begin
            nr2_active = 1'b1;
            nr2_start  = acc_row;
            nr2_len    = 10'd1;
            nr2_sum    = v_row_cnt;
        end else begin
            nr2_len = r_run_len + 10'd1;
            nr2_sum = r_run_sum + v_row_cnt;
        end
    end else if (r_run_active) begin
        nr2_active = 1'b0;
        r_close    = 1'b1;
    end
end

wire r_new = v_row_active
             ? (!(r_best_len != 10'd0) || (nr2_len > r_best_len))
             : (r_close &&
                (!(r_best_len != 10'd0) || (r_close_len > r_best_len)));
wire [8:0]  r_res_start = r_new ? (v_row_active ? nr2_start : r_close_start)
                                : r_best_start;
wire [9:0]  r_res_len   = r_new ? (v_row_active ? nr2_len : r_close_len)
                                : r_best_len;
wire [17:0] r_res_sum   = r_new ? (v_row_active ? nr2_sum : r_close_sum)
                                : r_best_sum;

// A blob that spans nearly the whole raster is never a person: that is what
// the crowd clips look like when the dilation glues the entire frame into one
// mask, and drawing it hides every real person behind a frame-filling box.
// Rejecting it here also keeps it out of the exclusion table below, so the
// remaining passes keep scanning the same mask instead of filling up with
// copies of the same giant rectangle.  Keep the margin in step with
// tools/fall_proto.py, which drops blobs this wide before tracking.  The
// height test now looks at the trimmed span, not at the whole band: a person
// standing in a band that happens to cover the frame is fine, a smear is not.
localparam [9:0] SPAN_W_LIMIT = WIDTH  - 8;
localparam [9:0] SPAN_H_LIMIT = HEIGHT - 8;
wire        v_spans = (vrun_len >= SPAN_W_LIMIT) || (r_res_len >= SPAN_H_LIMIT);
wire        v_found = (r_res_len != 10'd0) && (r_res_sum >= MIN_AREA)
                      && !v_spans;

integer i, j;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= ST_IDLE;
        busy        <= 1'b0;
        out_valid   <= 1'b0;
        out_found   <= {NUM_BLOBS{1'b0}};
        out_x       <= {(NUM_BLOBS*9){1'b0}};
        out_y       <= {(NUM_BLOBS*9){1'b0}};
        out_w       <= {(NUM_BLOBS*9){1'b0}};
        out_h       <= {(NUM_BLOBS*9){1'b0}};
        out_area    <= {(NUM_BLOBS*17){1'b0}};
        band_start  <= 9'd0;
        band_len    <= 10'd0;
        rd_bank     <= 1'b0;
        pass_idx    <= {PW{1'b0}};
        pass_count  <= {(PW+1){1'b0}};
        clr_idx     <= 9'd0;
        acc_row     <= 9'd0;
        acc_col     <= 9'd0;
        acc_drain   <= 1'b0;
        vrun_start  <= 9'd0;
        vrun_end    <= 9'd0;
        vrun_len    <= 10'd0;
        vdrain      <= 1'b0;
        vdrain_cnt  <= 2'd0;
        vcnt        <= 10'd0;
        r_run_active <= 1'b0;
        r_run_start  <= 9'd0;
        r_run_len    <= 10'd0;
        r_run_sum    <= 18'd0;
        r_best_start <= 9'd0;
        r_best_len   <= 10'd0;
        r_best_sum   <= 18'd0;
        a_addr      <= {AW{1'b0}};
        a_col       <= 9'd0;
        a_excl      <= 1'b0;
        a_valid     <= 1'b0;
        b_byte      <= 8'd0;
        b_hist      <= {CW{1'b0}};
        b_col       <= 9'd0;
        b_excl      <= 1'b0;
        b_valid     <= 1'b0;
        drain_cnt   <= 2'd0;
        sc          <= 9'd0;
        hist_q      <= {CW{1'b0}};
        span_phase  <= 1'b0;
        s_run_active <= 1'b0;
        s_run_start  <= 9'd0;
        s_run_len    <= 10'd0;
        s_run_sum    <= 18'd0;
        s_best_start <= 9'd0;
        s_best_len   <= 10'd0;
        s_best_sum   <= 18'd0;
        // hist[] is deliberately *not* reset here: ST_CLEAR zeroes all WIDTH
        // entries before the first pass of every frame, and dropping the loop
        // is what lets TD keep the array in ERAM instead of 2,560 flip-flops.
        for (j = 0; j < NUM_BLOBS; j = j + 1) begin
            ex_valid[j] <= 1'b0;
            ex_x[j]     <= 9'd0;
            ex_y[j]     <= 9'd0;
            ex_x1[j]    <= 10'd0;
            ex_y1[j]    <= 10'd0;
        end
    end else begin
        out_valid <= 1'b0;

        // ------------------------------------------------- shared RAM read
        // Exactly one read port per array, issued every clock in every
        // state.  The vertical trim wants mask_ram data in ST_ACC *and*
        // ST_VSPAN; leaving the read in both branches made TD try to merge
        // four RAM ports, and when that merge failed the 16 KB mask and the
        // 320-entry histogram both fell out of ERAM into flip-flops (118k
        // sequential cells -- unroutable).  One `<= ram[addr]` statement
        // restores the simple-dual-port shape the inference understands, and
        // the data is ignored in the states that do not stage a pixel.
        b_byte <= mask_ram[a_addr];

        if (state == ST_IDLE) begin
            busy <= 1'b0;
            if (frame_done) begin
                band_start <= done_start;
                band_len   <= done_len;
                rd_bank    <= done_bank;
                if (done_len == 10'd0) begin
                    // No vertical band at all: publish NUM_BLOBS empty slots
                    // and keep the exclusion table untouched.
                    out_valid <= 1'b1;
                    out_found <= {NUM_BLOBS{1'b0}};
                    out_x <= {(NUM_BLOBS*9){1'b0}};
                    out_y <= {(NUM_BLOBS*9){1'b0}};
                    out_w <= {(NUM_BLOBS*9){1'b0}};
                    out_h <= {(NUM_BLOBS*9){1'b0}};
                    out_area <= {(NUM_BLOBS*17){1'b0}};
                end else begin
                    pass_idx   <= {PW{1'b0}};
                    pass_count <= {(PW+1){1'b0}};
                    clr_idx    <= 9'd0;
                    busy       <= 1'b1;
                    state      <= ST_CLEAR;
                end
            end
        end else if (state == ST_CLEAR) begin
            hist[clr_idx] <= {CW{1'b0}};
            if (clr_idx == WIDTH - 1) begin
                acc_row   <= 9'd0;
                acc_col   <= 9'd0;
                acc_drain <= 1'b0;
                drain_cnt <= 2'd0;
                a_valid   <= 1'b0;
                b_valid   <= 1'b0;
                state     <= ST_ACC;
            end else begin
                clr_idx <= clr_idx + 9'd1;
            end
        end else if (state == ST_ACC) begin
            // ---- C: fold the pixel latched two clocks ago into its column
            // entry.  This is the only place hist is ever written, so the
            // array stays a one-read/one-write-port RAM.
            if (b_valid && b_byte[7 - b_col[2:0]] && !b_excl)
                hist[b_col] <= b_hist + 1'b1;

            // ---- B: registered read.  a_addr/a_col were latched in stage A
            // one clock ago, so both RAM banks see a register on the address
            // port and TD keeps them in ERAM.  The read for column c issues
            // two clocks before the write for c, so the mux and the
            // incrementer never share a cycle.
            b_hist  <= hist[a_col];
            b_col   <= a_col;
            b_excl  <= a_excl;
            b_valid <= a_valid;

            if (!acc_drain) begin
                // ---- A: set up the read for this pixel, then walk the band.
                a_addr  <= pix_rd_index;
                a_col   <= acc_col;
                a_excl  <= hit_c;
                a_valid <= 1'b1;
                if (acc_col == WIDTH - 1) begin
                    acc_col <= 9'd0;
                    if (acc_row == band_len - 10'd1)
                        acc_drain <= 1'b1;
                    else
                        acc_row <= acc_row + 9'd1;
                end else begin
                    acc_col <= acc_col + 9'd1;
                end
            end else begin
                // Last pixel fed: stage B still holds it and stage C issues
                // its write on the next clock.  One idle cycle is enough.
                a_valid <= 1'b0;
                if (drain_cnt == 2'd1) begin
                    sc           <= 9'd0;
                    s_run_active <= 1'b0;
                    s_run_len    <= 10'd0;
                    s_run_sum    <= 18'd0;
                    s_best_start <= 9'd0;
                    s_best_len   <= 10'd0;
                    s_best_sum   <= 18'd0;
                    state        <= ST_SPAN;
                end
                drain_cnt <= drain_cnt + 2'd1;
            end
        end else if (state == ST_VSPAN) begin
            // ----------------------------------------------------- ST_VSPAN
            // Second scan over the columns of the run just found, one row at
            // a time, reusing the ST_ACC pipeline.  A row joins the span when
            // it carries at least MIN_PIX of the run's pixels; the longest
            // such row run becomes the published box.
            if (v_pix_now)
                vcnt <= vcnt + 10'd1;

            b_col   <= a_col;
            b_excl  <= a_excl;
            b_valid <= a_valid;

            if (!vdrain) begin
                a_addr  <= pix_rd_index;
                a_col   <= acc_col;
                a_excl  <= hit_c;
                a_valid <= 1'b1;
                if (acc_col == vrun_end)
                    vdrain <= 1'b1;
                else
                    acc_col <= acc_col + 9'd1;
            end else begin
                a_valid <= 1'b0;
                if (vdrain_cnt == 2'd1) begin
                    // The row's last pixel lands in stage C this cycle, so
                    // v_row_cnt already includes it: feed the run tracker,
                    // then step to the next row (or close the pass).
                    r_run_active <= nr2_active;
                    r_run_start  <= nr2_start;
                    r_run_len    <= nr2_len;
                    r_run_sum    <= nr2_sum;
                    if (r_close &&
                        (!(r_best_len != 10'd0) ||
                         (r_close_len > r_best_len))) begin
                        r_best_start <= r_close_start;
                        r_best_len   <= r_close_len;
                        r_best_sum   <= r_close_sum;
                    end

                    vcnt       <= 10'd0;
                    vdrain     <= 1'b0;
                    vdrain_cnt <= 2'd0;

                    if (acc_row == band_len - 10'd1) begin
                        // Fold the trimmed result into slot pass_idx and
                        // freeze its rectangle for the later passes.
                        out_found[pass_idx*1 +: 1] <= v_found;
                        out_x[pass_idx*9 +: 9] <= vrun_start;
                        out_y[pass_idx*9 +: 9] <= band_start + r_res_start;
                        out_w[pass_idx*9 +: 9] <= vrun_len[8:0];
                        out_h[pass_idx*9 +: 9] <= r_res_len[8:0];
                        out_area[pass_idx*17 +: 17] <= r_res_sum[16:0];

                        if (v_found) begin
                            ex_valid[pass_idx] <= 1'b1;
                            ex_x[pass_idx]  <= vrun_start;
                            ex_y[pass_idx]  <= band_start + r_res_start;
                            // +MARGIN on each side; width and height are
                            // capped so the sums cannot overflow 10 bits.
                            ex_x1[pass_idx] <= vrun_start + vrun_len[8:0]
                                               + MARGIN;
                            ex_y1[pass_idx] <= band_start + r_res_start
                                               + r_res_len[8:0] + MARGIN;
                        end else begin
                            ex_valid[pass_idx] <= 1'b0;
                        end

                        if (pass_count == NUM_BLOBS - 1) begin
                            out_valid <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_IDLE;
                        end else begin
                            pass_count <= pass_count + 1'b1;
                            pass_idx   <= pass_idx + 1'b1;
                            clr_idx    <= 9'd0;
                            state      <= ST_CLEAR;
                        end
                    end else begin
                        acc_row <= acc_row + 9'd1;
                        acc_col <= vrun_start;
                    end
                end else begin
                    vdrain_cnt <= vdrain_cnt + 2'd1;
                end
            end
        end else begin
            // ------------------------------------------------------ ST_SPAN
            if (!span_phase) begin
                if (sc < WIDTH)
                    hist_q <= hist[sc];
                span_phase <= 1'b1;
            end else begin
                s_run_active <= ns_active;
                s_run_start  <= ns_start;
                s_run_len    <= ns_len;
                s_run_sum    <= ns_sum;
                if (s_close &&
                    (!(s_best_len != 10'd0) ||
                     (s_close_len > s_best_len))) begin
                    s_best_start <= s_close_start;
                    s_best_len   <= s_close_len;
                    s_best_sum   <= s_close_sum;
                end

                if (sc == WIDTH - 1) begin
                    span_phase <= 1'b0;
                    if (res_len != 10'd0) begin
                        // A column run was found.  Publish it provisionally
                        // (found stays low) and trim it vertically first.
                        out_found[pass_idx*1 +: 1] <= 1'b0;
                        out_x[pass_idx*9 +: 9] <= res_start;
                        out_y[pass_idx*9 +: 9] <= band_start;
                        out_w[pass_idx*9 +: 9] <= res_len[8:0];
                        out_h[pass_idx*9 +: 9] <= 10'd0;
                        out_area[pass_idx*17 +: 17] <= 18'd0;

                        vrun_start   <= res_start;
                        vrun_len     <= res_len;
                        vrun_end     <= res_start + res_len[8:0] - 9'd1;
                        acc_row      <= 9'd0;
                        acc_col      <= res_start;
                        vcnt         <= 10'd0;
                        vdrain       <= 1'b0;
                        vdrain_cnt   <= 2'd0;
                        r_run_active <= 1'b0;
                        r_run_len    <= 10'd0;
                        r_run_sum    <= 18'd0;
                        r_best_len   <= 10'd0;
                        r_best_sum   <= 18'd0;
                        state        <= ST_VSPAN;
                    end else begin
                        // No column run at all: publish an empty slot and
                        // keep the exclusion table untouched.
                        out_found[pass_idx*1 +: 1] <= 1'b0;
                        out_x[pass_idx*9 +: 9] <= res_start;
                        out_y[pass_idx*9 +: 9] <= band_start;
                        out_w[pass_idx*9 +: 9] <= 10'd0;
                        out_h[pass_idx*9 +: 9] <= band_len[8:0];
                        out_area[pass_idx*17 +: 17] <= 18'd0;
                        ex_valid[pass_idx] <= 1'b0;

                        if (pass_count == NUM_BLOBS - 1) begin
                            out_valid <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_IDLE;
                        end else begin
                            pass_count <= pass_count + 1'b1;
                            pass_idx   <= pass_idx + 1'b1;
                            clr_idx    <= 9'd0;
                            state      <= ST_CLEAR;
                        end
                    end
                end else begin
                    sc         <= sc + 9'd1;
                    span_phase <= 1'b0;
                end
            end
        end
    end
end

endmodule
