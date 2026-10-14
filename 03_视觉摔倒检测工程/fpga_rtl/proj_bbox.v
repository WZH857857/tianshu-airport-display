//===========================================================================
// proj_bbox.v -- dominant-blob bounding box via two-pass projections
//
// Mirrors FallDetector.bbox_of() from tools/fall_proto.py:
//
//   rows    = mask.sum(axis=1)
//   y-span  = longest contiguous run of rows with count >= MIN_PIX
//   x-span  = longest contiguous run of column counts >= MIN_PIX, where the
//             column counts are measured *inside the y-span only*
//   area    = mask pixels inside the y-span and the x-span
//   found   = x-span exists and area >= MIN_AREA
//
// Two concurrent sides, joined by a frame pulse:
//   stream side (writes)  - packs the incoming mask bits into one of two
//                           mask RAM banks, popcounts each row and tracks
//                           the longest row-run.  At the end of row 239 it
//                           publishes {band_start, band_len, bank} and
//                           flips to the other bank.
//   pass side (reads)     - on that pulse, rebuilds the per-column counts
//                           of the selected band in a 320x16 histogram,
//                           scans the histogram for the longest column run
//                           while summing its counts (= area), and emits
//                           found/x/y/w/h/area one frame later.
//
// The ping-pong banks decouple the two sides completely: the pass needs at
// most ~97k clocks (dense band) while a frame is >=420k source clocks on
// the 640x480@60 camera path, so it always finishes before the stream side
// wraps back to the same bank.  No blanking-window assumption is needed.
//
// Mask stream contract: one pixel per de_in, WIDTH per row, HEIGHT rows per
// frame, no frame_start strobe (the row counter frames it).  out_valid is a
// one-cycle pulse; the result is valid with it.
//===========================================================================
`timescale 1ns / 1ps

module proj_bbox #(
    parameter integer WIDTH    = 320,
    parameter integer HEIGHT   = 180,
    parameter integer MIN_PIX  = 3,
    parameter integer MIN_AREA = 200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       de_in,
    input  wire       mask_in,
    output reg        busy,
    output reg        out_valid,
    output reg        out_found,
    output reg [8:0]  out_x,
    output reg [8:0]  out_y,
    output reg [8:0]  out_w,
    output reg [8:0]  out_h,
    output reg [16:0] out_area
);

localparam integer BPR    = WIDTH / 8;         // packed bytes per row
localparam integer NBYTES = HEIGHT * BPR;      // one bank
localparam integer AW     = $clog2(2 * NBYTES); // both banks in 14 bits at 180 rows
// Both ports must be 14 bits wide.  TD sizes an array's ERAM footprint from
// its address port, not from the declared depth, so a 15-bit port here makes
// it plan for 32,768 entries and the 14,400-entry mask loses its 9K mapping.
localparam [AW-1:0] BANK_OFF = NBYTES;         // 7,200 for 320x180

localparam [1:0] ST_IDLE  = 2'd0;
localparam [1:0] ST_CLEAR = 2'd1;
localparam [1:0] ST_ACC   = 2'd2;
localparam [1:0] ST_SPAN  = 2'd3;

// ------------------------------------------------------------- mask storage
// 115,200 bits, i.e. ERAM territory.  The two ping-pong banks share one
// array, so the bank select is folded into the address and the array keeps
// exactly one write port and one read port -- TD only infers ERAM with a
// synchronous read and at most two write ports, and a third port here sends
// the whole thing into LUTs.
//
// The hint is the 9K region on purpose: the 32K region is fully consumed by
// the low half of frame_diff's previous-frame store, and a generic "bram"
// hint here makes TD try the 32K region first, fail, and then drop all three
// arrays (mask + both frame halves) into LUTs.
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

// The last pixel of a row is evaluated on the clock that also schedules the
// counter update, so the run FSM looks at the *next* popcount.
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

// published to the pass side
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
            // --------------------------------------------------- store bit
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
                    // -------------------------------- publish and flip bank
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

// =============================================================== pass side
reg [1:0]  state;
reg [8:0]  band_start;
reg [9:0]  band_len;
reg        rd_bank;

reg [8:0]  clr_idx;
reg [8:0]  acc_row;
reg [5:0]  acc_byte;
reg [2:0]  acc_bit;
reg [1:0]  acc_phase;          // 1 = issue address, 2 = register read, 0 = bits
reg [AW-1:0] rd_addr;
wire [AW-1:0] rd_index = rd_bank ? (rd_addr + BANK_OFF) : rd_addr;
reg [7:0]  rd_data;
reg [8:0]  acc_col;

reg [15:0] hist [0:WIDTH-1];
wire [8:0]  hist_idx = acc_col + {6'd0, acc_bit};
wire [15:0] hist_acc = hist[hist_idx];

reg [8:0]  sc;
reg        s_run_active;
reg [8:0]  s_run_start;
reg [9:0]  s_run_len;
reg [17:0] s_run_sum;
reg [8:0]  s_best_start;
reg [9:0]  s_best_len;
reg [17:0] s_best_sum;

// The histogram is 320 entries wide, so a combinational hist[sc] read plus
// the run comparator was the chip's critical path (18 LUT levels to
// out_found).  The scan therefore walks two cycles per column: one to
// register the entry, one to fold it into the run FSM.  640 clocks is
// nothing against the ~420k-clock frame time.
reg [15:0] hist_q;
reg        span_phase;
wire [15:0] hist_scan = hist_q;
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
            ns_sum    = {2'b00, hist_scan};
        end else begin
            ns_len = s_run_len + 10'd1;
            ns_sum = s_run_sum + {2'b00, hist_scan};
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

integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        busy         <= 1'b0;
        out_valid    <= 1'b0;
        out_found    <= 1'b0;
        out_x        <= 9'd0;
        out_y        <= 9'd0;
        out_w        <= 9'd0;
        out_h        <= 9'd0;
        out_area     <= 17'd0;
        band_start   <= 9'd0;
        band_len     <= 10'd0;
        rd_bank      <= 1'b0;
        clr_idx      <= 9'd0;
        acc_row      <= 9'd0;
        acc_byte     <= 6'd0;
        acc_bit      <= 3'd0;
        acc_phase    <= 2'd0;
        rd_addr      <= {AW{1'b0}};
        rd_data      <= 8'd0;
        acc_col      <= 9'd0;
        sc           <= 9'd0;
        hist_q       <= 16'd0;
        span_phase   <= 1'b0;
        s_run_active <= 1'b0;
        s_run_start  <= 9'd0;
        s_run_len    <= 10'd0;
        s_run_sum    <= 18'd0;
        s_best_start <= 9'd0;
        s_best_len   <= 10'd0;
        s_best_sum   <= 18'd0;
        for (i = 0; i < WIDTH; i = i + 1)
            hist[i] <= 16'd0;
    end else begin
        out_valid <= 1'b0;

        if (state == ST_IDLE) begin
            busy <= 1'b0;
            if (frame_done) begin
                band_start <= done_start;
                band_len   <= done_len;
                rd_bank    <= done_bank;
                if (done_len == 10'd0) begin
                    out_valid <= 1'b1;
                    out_found <= 1'b0;
                    out_x <= 9'd0; out_y <= 9'd0;
                    out_w <= 9'd0; out_h <= 9'd0;
                    out_area <= 17'd0;
                end else begin
                    clr_idx <= 9'd0;
                    busy    <= 1'b1;
                    state   <= ST_CLEAR;
                end
            end
        end else if (state == ST_CLEAR) begin
            hist[clr_idx] <= 16'd0;
            if (clr_idx == WIDTH - 1) begin
                acc_row   <= 9'd0;
                acc_byte  <= 6'd0;
                acc_bit   <= 3'd0;
                acc_phase <= 2'd1;
                acc_col   <= 9'd0;
                state     <= ST_ACC;
            end else begin
                clr_idx <= clr_idx + 9'd1;
            end
        end else if (state == ST_ACC) begin
            if (acc_phase == 2'd1) begin
                rd_addr   <= ((band_start + acc_row) * BPR) + acc_byte;
                acc_phase <= 2'd2;
            end else if (acc_phase == 2'd2) begin
                rd_data <= mask_ram[rd_index];
                acc_phase <= 2'd0;
            end else begin
                if (rd_data[7 - acc_bit])
                    hist[hist_idx] <= hist_acc + 16'd1;

                if (acc_bit == 3'd7) begin
                    acc_bit <= 3'd0;
                    acc_col <= acc_col + 9'd8;
                    if (acc_byte == BPR - 1) begin
                        acc_byte <= 6'd0;
                        if (acc_row == band_len - 10'd1) begin
                            sc           <= 9'd0;
                            s_run_active <= 1'b0;
                            s_run_len    <= 10'd0;
                            s_run_sum    <= 18'd0;
                            s_best_start <= 9'd0;
                            s_best_len   <= 10'd0;
                            s_best_sum   <= 18'd0;
                            state        <= ST_SPAN;
                        end else begin
                            acc_row   <= acc_row + 9'd1;
                            acc_col   <= 9'd0;
                            acc_phase <= 2'd1;
                        end
                    end else begin
                        acc_byte  <= acc_byte + 6'd1;
                        acc_phase <= 2'd1;
                    end
                end else begin
                    acc_bit <= acc_bit + 3'd1;
                end
            end
        end else begin
            // ------------------------------------------------------ ST_SPAN
            if (!span_phase) begin
                // issue the read; hist[sc] lands in hist_q next clock
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
                    out_valid <= 1'b1;
                    out_area  <= res_sum[16:0];
                    out_found <= (res_len != 10'd0) &&
                                 (res_sum >= MIN_AREA);
                    out_x     <= res_start;
                    out_y     <= band_start;
                    out_w     <= res_len[8:0];
                    out_h     <= band_len[8:0];
                    busy      <= 1'b0;
                    state     <= ST_IDLE;
                end else begin
                    sc         <= sc + 9'd1;
                    span_phase <= 1'b0;
                end
            end
        end
    end
end

endmodule
