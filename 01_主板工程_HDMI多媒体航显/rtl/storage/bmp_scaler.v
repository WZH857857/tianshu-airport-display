`timescale 1ns / 1ps
//=============================================================================
// bmp_scaler.v - bilinear resize to DST_W x DST_H
//
// The BMP parser presents one 24 bit source pixel per handshake in bottom-up
// file order.  This module turns that stream into exactly DST_W x DST_H
// pixels for the frame store without changing the downstream interface.
//
// Each source row is first scaled horizontally into a DST_W entry line buffer.
// Two line buffers are ping-ponged, so the previous and current source rows are
// both available when an output row is formed.  The vertical pass blends those
// two rows.  Both passes use the same center-aligned, 8-bit fixed point bilinear
// mapping:
//
//   source position = (destination + 0.5) * source_size / destination_size
//                     - 0.5
//
// Only complete rows are passed to the vertical stage.  This keeps the parser
// handshake simple and lets a source row be reused for every output row whose
// vertical phase selects it.
//
// The phase is carried as a quotient plus a remainder of the fixed denominator
// 2 * destination_size, which keeps the mapping exact.  One 50 MHz cycle cannot
// hold normalise + divide + interpolate, so every phase advance is a three
// stage pipeline (accumulate, normalise, weight) and the scaler spends three
// cycles per destination column.  Image decode is a low throughput path, so the
// extra cycles are free while the timing margin is not.
//=============================================================================
module bmp_scaler #(
    parameter integer DST_W = 640,
    parameter integer DST_H = 480,
    // 0 = portable inferred RAM for simulation, 1 = explicit EG4 BRAM.
    parameter integer BRAM_PRIM = 0
)(
    input  wire        clk,
    input  wire        rst_n,

    // One cycle pulse when a BMP load starts, or when the running one is
    // dropped.  All row buffering and mapping state returns to the start.
    input  wire        frame_rst,

    // Geometry of the picture being streamed in, stable from the first pixel.
    input  wire [15:0] src_w,
    input  wire [15:0] src_h,

    // Source pixel stream, straight from the BMP parser.
    input  wire        s_valid,
    input  wire [23:0] s_rgb,
    input  wire        s_last,
    output wire        s_ready,

    // Destination pixel stream, to the frame store loader.
    output wire        m_valid,
    output wire [23:0] m_rgb,
    input  wire        m_ready,

    // High from the first captured source pixel until the last destination
    // pixel has been accepted.  The parser holds its done pulse while this is
    // high, so the frame store stays enabled until the final row is written.
    output wire        busy,
    output wire [9:0]  dbg_row
);

    localparam [23:0] H_DEN = DST_W * 2;
    localparam [23:0] V_DEN = DST_H * 2;
    localparam [11:0] DST_W_M1 = DST_W - 1;

    localparam [5:0] ST_IDLE   = 6'd0,
                     ST_INIT   = 5'd1,
                     ST_INIT2  = 5'd2,
                     ST_INIT3  = 5'd3,
                     ST_INIT4  = 5'd4,
                     ST_HCAP   = 5'd5,
                     ST_HWR    = 5'd6,
                     ST_HADV1  = 5'd7,
                     ST_HADV2  = 5'd8,
                     ST_HADV3  = 5'd9,
                     ST_VCHECK = 5'd10,
                     ST_VFETCH = 5'd11,
                     ST_VLATCH = 5'd12,
                     ST_VALID  = 5'd13,
                     ST_VADV1  = 5'd14,
                     ST_VADV2  = 5'd15,
                     ST_VADV3  = 5'd16,
                     ST_VADV4  = 6'd17,
                     ST_DONE   = 6'd18;

    reg [5:0]  state;
    reg [15:0] w_src, h_src;

    // Captured source pixel pair and the source column of px_cur.
    reg [15:0] src_col;
    reg [15:0] cap_col;
    reg [23:0] px_prev, px_cur;

    // Horizontal phase.  h_q is the remainder modulo 2*DST_W, so the
    // interpolation weight is h_q / (2*DST_W).
    reg [11:0] hx;
    reg [23:0] h_q;
    reg signed [15:0] h_x0;
    reg [7:0]  h_w;

    // Per picture constants and the start phase, latched while the geometry
    // registers are stable.
    reg [23:0] h_step;
    reg [23:0] h_q0;
    reg signed [15:0] h_x0_0;
    reg [7:0]  h_w0;

    // Horizontal phase advance pipeline.
    reg [23:0] h_sum;
    reg [27:0] h_nrm;
    reg        h_small_r;

    // Which line buffer receives the current source row.
    reg        hbuf_sel;

    // Vertical phase.  v_y0 is the first source row of the pair and v_y1 is
    // the second one, or the same row at the top/bottom edge.
    reg [23:0] v_q;
    reg signed [15:0] v_y0;
    reg [7:0]  v_w;

    reg [23:0] v_step;
    reg [23:0] v_q0;
    reg signed [15:0] v_y0_0;
    reg [7:0]  v_w0;

    // Vertical phase advance pipeline.
    reg [23:0] v_sum;
    reg [27:0] v_nrm;
    reg        v_small_r;
    reg [23:0] v_w_q;

    // Output row position and the source row currently held in the buffers.
    reg [9:0]  rptr;
    reg [9:0]  out_row;
    reg [15:0] src_row;
    reg [23:0] v_data;
    reg        src_last;

    // Right/bottom edge coordinates.  They are latched while the geometry is
    // stable, so every edge test is a compare against a register instead of a
    // comparator sitting behind a subtractor.
    reg  [15:0] w_m1_r, h_m1_r;
    wire [15:0] w_m1 = w_m1_r;
    wire [15:0] h_m1 = h_m1_r;
    wire signed [15:0] w_m1_s = $signed(w_m1_r);
    wire signed [15:0] h_m1_s = $signed(h_m1_r);

    //------------------------------------------------------------------
    // Fixed point helpers.
    //
    // Every value fed to norm_phase stays below 8*den, so three conditional
    // subtractions (4*den, 2*den, den) reduce it in three logic levels instead
    // of an eight step ripple.  The quotient therefore never exceeds six.
    //------------------------------------------------------------------
    function [27:0] norm_phase;
        input [23:0] raw;
        input [23:0] den;
        reg [23:0] q;
        reg [3:0]  n;
        begin
            q = raw;
            n = 4'd0;
            if (q >= (den << 2)) begin
                q = q - (den << 2);
                n = n + 4'd4;
            end
            if (q >= (den << 1)) begin
                q = q - (den << 1);
                n = n + 4'd2;
            end
            if (q >= den) begin
                q = q - den;
                n = n + 4'd1;
            end
            norm_phase = {n, q};
        end
    endfunction

    // Remainder to interpolation weight.  Both denominators are elaboration
    // constants, so this is a constant multiply and shift, not a divider.
    // A remainder is always below its denominator (1280 / 960), so twelve bits
    // hold it exactly.  Feeding the network a full 24 bit word made the tool
    // build twelve extra provably zero inputs, and that constant network was
    // the worst path of the whole design.
    function [7:0] weight_h;
        input [11:0] q;
        begin
            weight_h = (q << 8) / H_DEN;
        end
    endfunction

    function [7:0] weight_v;
        input [11:0] q;
        begin
            weight_v = (q << 8) / V_DEN;
        end
    endfunction

    function [7:0] mix8;
        input [7:0] a;
        input [7:0] b;
        input [7:0] w;
        reg signed [8:0]  d;
        reg signed [17:0] p;
        integer t;
        begin
            if (w == 8'd0)
                mix8 = a;
            else if (w == 8'd255)
                mix8 = b;
            else begin
                d = $signed({1'b0, b}) - $signed({1'b0, a});
                p = d * $signed({1'b0, w});
                t = (p + 18'sd128) >>> 8;
                t = t + a;
                if (t < 0)
                    mix8 = 8'd0;
                else if (t > 255)
                    mix8 = 8'd255;
                else
                    mix8 = t[7:0];
            end
        end
    endfunction

    //------------------------------------------------------------------
    // Start phase: the source position of destination 0 is
    //   (source_size / (2 * destination_size)) - 0.5
    // so its quotient is -1 and its remainder is
    //   2*D - (D - source) = 2*D - delta
    // whenever the picture is smaller than the display.
    //------------------------------------------------------------------
    wire        h_small = (w_src < DST_W);
    wire [23:0] h_delta = h_small ? (DST_W - w_src) : (w_src - DST_W);
    wire [23:0] h_raw   = h_small ? (H_DEN - h_delta) : h_delta;

    wire        v_small = (h_src < DST_H);
    wire [23:0] v_delta = v_small ? (DST_H - h_src) : (h_src - DST_H);
    wire [23:0] v_raw   = v_small ? (V_DEN - v_delta) : v_delta;

    //------------------------------------------------------------------
    // Phase advance stages.  h_nrm / v_nrm hold {quotient, remainder} of the
    // accumulated phase; the pair below turns them into the next tap pair and
    // interpolation weight.
    //------------------------------------------------------------------
    wire signed [15:0] h_dn = $signed({12'd0, h_nrm[27:24]});
    wire signed [15:0] h_x0_n = h_x0 + h_dn;
    wire               h_x0_n_edge = (h_x0_n < 0) || (h_x0_n >= w_m1_s);

    wire signed [15:0] v_dn = $signed({12'd0, v_nrm[27:24]});
    wire signed [15:0] v_y0_n = v_y0 + v_dn;
    wire               v_y0_n_edge = (v_y0_n < 0) || (v_y0_n >= h_m1_s);

    // Start phase weight, also used to reload the phase at the top of a row.
    wire h_start_edge = (h_x0_0 < 0) || (h_x0_0 >= w_m1_s);
    wire v_start_edge = (v_y0_0 < 0) || (v_y0_0 >= h_m1_s);

    // Second tap and edge flag of the phase that is being latched.  Outside
    // the source image both taps collapse to the edge pixel.  Registering
    // these next to the phase keeps the comparators out of the interpolation
    // path: the pixel muxes below only ever see registers.
    wire [15:0] h_x1_n = (h_x0_n < 0) ? 16'd0 :
                         (h_x0_n >= w_m1_s) ? w_m1 :
                         (h_x0_n[15:0] + 16'd1);
    wire [15:0] v_y1_n = (v_y0_n < 0) ? 16'd0 :
                         (v_y0_n >= h_m1_s) ? h_m1 :
                         (v_y0_n[15:0] + 16'd1);

    reg [15:0] h_x1;        // second tap of the column being drawn
    reg        h_edge;      // both horizontal taps are the current pixel
    reg [15:0] v_y1;        // second tap of the row being drawn
    reg        v_same;      // both vertical taps come from one line buffer

    // Same two values for destination 0 of a row.  They are a pure function of
    // the latched start phase, so ST_INIT4 and the row restart in ST_VCHECK
    // load exactly the same pair.
    wire [15:0] h_x1_0 = (h_x0_0 < 0) ? 16'd0 :
                         (h_x0_0 >= w_m1_s) ? w_m1 :
                         (h_x0_0[15:0] + 16'd1);
    wire        h_edge_0 = h_start_edge;
    wire [15:0] v_y1_0 = (v_y0_0 < 0) ? 16'd0 :
                         (v_y0_0 >= h_m1_s) ? h_m1 :
                         (v_y0_0[15:0] + 16'd1);
    wire        v_same_0 = v_start_edge;

    //------------------------------------------------------------------
    // Line buffer ports and horizontal interpolation.
    //------------------------------------------------------------------
    wire [23:0] rd_b0, rd_b1;
    wire [9:0]  line_rd_addr = rptr[9:0];

    // ST_HWR latches the interpolated column, the destination address and the
    // buffer select; the line buffer is written on the following cycle.  The
    // interpolation chain therefore ends on a register instead of on the
    // asynchronous memory data input.
    reg [23:0] h_mix_r;
    reg [9:0]  h_wr_addr;
    reg        h_wr_buf;
    reg        h_wr_pend;

    wire        line_wr_en = h_wr_pend;
    wire [9:0]  line_wr_addr = h_wr_addr;
    wire        line_wr_en_b0 = h_wr_pend && !h_wr_buf;
    wire        line_wr_en_b1 = h_wr_pend &&  h_wr_buf;
    wire        v_read = (state == ST_VFETCH);

    // At either edge both horizontal taps are the current source pixel.
    wire [23:0] h_pix_a = h_edge ? px_cur : px_prev;
    wire [23:0] h_pix_b = px_cur;
    wire [7:0]  h_r = mix8(h_pix_a[23:16], h_pix_b[23:16], h_w);
    wire [7:0]  h_g = mix8(h_pix_a[15:8],  h_pix_b[15:8],  h_w);
    wire [7:0]  h_b = mix8(h_pix_a[7:0],   h_pix_b[7:0],   h_w);
    wire [23:0] h_mix = {h_r, h_g, h_b};

    // The previous row is in the opposite buffer from the current row.  At a
    // clamped vertical edge both taps use the current buffer.
    wire [23:0] v_pix_a = v_same ? (hbuf_sel ? rd_b1 : rd_b0) :
                                   (hbuf_sel ? rd_b0 : rd_b1);
    wire [23:0] v_pix_b = v_same ? (hbuf_sel ? rd_b1 : rd_b0) :
                                   (hbuf_sel ? rd_b1 : rd_b0);
    wire [7:0]  v_r = mix8(v_pix_a[23:16], v_pix_b[23:16], v_w);
    wire [7:0]  v_g = mix8(v_pix_a[15:8],  v_pix_b[15:8],  v_w);
    wire [7:0]  v_b = mix8(v_pix_a[7:0],   v_pix_b[7:0],   v_w);
    wire [23:0] v_mix = {v_r, v_g, v_b};

    assign s_ready = (state == ST_HCAP);
    assign m_valid = (state == ST_VALID);
    assign m_rgb   = v_data;
    assign busy    = (state != ST_IDLE);
    assign dbg_row = out_row;

    //------------------------------------------------------------------
    // Explicit BRAM instances for the board.  The inferred model is used in
    // simulation, where the line buffers do not need vendor RAM.
    //------------------------------------------------------------------
    generate
    if (BRAM_PRIM != 0) begin : g_line_bram
        EG_LOGIC_BRAM #(
            .DATA_WIDTH_A (24),
            .DATA_WIDTH_B (24),
            .ADDR_WIDTH_A (10),
            .ADDR_WIDTH_B (10),
            .DATA_DEPTH_A (DST_W),
            .DATA_DEPTH_B (DST_W),
            .BYTE_ENABLE  (0),
            .BYTE_A       (1),
            .BYTE_B       (1),
            .MODE         ("DP"),
            .REGMODE_A    ("NOREG"),
            .REGMODE_B    ("OUTREG"),
            .WRITEMODE_A  ("NORMAL"),
            .WRITEMODE_B  ("NORMAL"),
            .RESETMODE    ("SYNC"),
            .DEBUGGABLE   ("NO"),
            .PACKABLE     ("NO"),
            .FORCE_KEEP   ("OFF"),
            .INIT_FILE    ("NONE"),
            .FILL_ALL     ("NONE"),
            .IMPLEMENT    ("9K")
        ) u_line_ram0 (
            .doa  (),
            .dob  (rd_b0),
            .dia  (h_mix_r),
            .dib  (24'd0),
            .addra(line_wr_addr),
            .addrb(line_rd_addr),
            .cea  (1'b1),
            .ocea (1'b0),
            .clka (clk),
            .wea  (line_wr_en_b0),
            .rsta (1'b0),
            .ceb  (v_read),
            .oceb (v_read),
            .clkb (clk),
            .web  (1'b0),
            .rstb (1'b0),
            .bea  (1'b1),
            .beb  (1'b1)
        );

        EG_LOGIC_BRAM #(
            .DATA_WIDTH_A (24),
            .DATA_WIDTH_B (24),
            .ADDR_WIDTH_A (10),
            .ADDR_WIDTH_B (10),
            .DATA_DEPTH_A (DST_W),
            .DATA_DEPTH_B (DST_W),
            .BYTE_ENABLE  (0),
            .BYTE_A       (1),
            .BYTE_B       (1),
            .MODE         ("DP"),
            .REGMODE_A    ("NOREG"),
            .REGMODE_B    ("OUTREG"),
            .WRITEMODE_A  ("NORMAL"),
            .WRITEMODE_B  ("NORMAL"),
            .RESETMODE    ("SYNC"),
            .DEBUGGABLE   ("NO"),
            .PACKABLE     ("NO"),
            .FORCE_KEEP   ("OFF"),
            .INIT_FILE    ("NONE"),
            .FILL_ALL     ("NONE"),
            .IMPLEMENT    ("9K")
        ) u_line_ram1 (
            .doa  (),
            .dob  (rd_b1),
            .dia  (h_mix_r),
            .dib  (24'd0),
            .addra(line_wr_addr),
            .addrb(line_rd_addr),
            .cea  (1'b1),
            .ocea (1'b0),
            .clka (clk),
            .wea  (line_wr_en_b1),
            .rsta (1'b0),
            .ceb  (v_read),
            .oceb (v_read),
            .clkb (clk),
            .web  (1'b0),
            .rstb (1'b0),
            .bea  (1'b1),
            .beb  (1'b1)
        );
    end else begin : g_line_model
        reg [23:0] line0 [0:DST_W-1];
        reg [23:0] line1 [0:DST_W-1];
        reg [23:0] rd_b0_r;
        reg [23:0] rd_b1_r;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                rd_b0_r <= 24'd0;
                rd_b1_r <= 24'd0;
            end else if (v_read) begin
                rd_b0_r <= line0[line_rd_addr];
                rd_b1_r <= line1[line_rd_addr];
            end
        end

        always @(posedge clk) begin
            if (line_wr_en_b0)
                line0[line_wr_addr] <= h_mix_r;
            if (line_wr_en_b1)
                line1[line_wr_addr] <= h_mix_r;
        end

        assign rd_b0 = rd_b0_r;
        assign rd_b1 = rd_b1_r;
    end
    endgenerate

    //------------------------------------------------------------------
    // Main sequencer.
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            w_src     <= 16'd0;
            h_src     <= 16'd0;
            src_col   <= 16'd0;
            cap_col   <= 16'd0;
            px_prev   <= 24'd0;
            px_cur    <= 24'd0;
            hx        <= 12'd0;
            h_q       <= 24'd0;
            h_x0      <= 16'sd0;
            h_w       <= 8'd0;
            h_step    <= 24'd0;
            h_q0      <= 24'd0;
            h_x0_0    <= 16'sd0;
            h_w0      <= 8'd0;
            h_sum     <= 24'd0;
            h_nrm     <= 28'd0;
            h_small_r <= 1'b0;
            w_m1_r    <= 16'd0;
            h_m1_r    <= 16'd0;
            h_x1      <= 16'd0;
            h_edge    <= 1'b0;
            h_mix_r   <= 24'd0;
            h_wr_addr <= 10'd0;
            h_wr_buf  <= 1'b0;
            h_wr_pend <= 1'b0;
            hbuf_sel  <= 1'b0;
            v_q       <= 24'd0;
            v_y0      <= 16'sd0;
            v_w       <= 8'd0;
            v_step    <= 24'd0;
            v_q0      <= 24'd0;
            v_y0_0    <= 16'sd0;
            v_w0      <= 8'd0;
            v_sum     <= 24'd0;
            v_nrm     <= 28'd0;
            v_small_r <= 1'b0;
            v_w_q     <= 24'd0;
            v_y1      <= 16'd0;
            v_same    <= 1'b0;
            rptr      <= 10'd0;
            out_row   <= 10'd0;
            src_row   <= 16'd0;
            v_data    <= 24'd0;
            src_last  <= 1'b0;
        end else if (frame_rst) begin
            state    <= ST_IDLE;
            src_col  <= 16'd0;
            cap_col  <= 16'd0;
            px_prev  <= 24'd0;
            px_cur   <= 24'd0;
            hx       <= 12'd0;
            hbuf_sel <= 1'b0;
            h_wr_pend<= 1'b0;
            src_row  <= 16'd0;
            rptr     <= 10'd0;
            out_row  <= 10'd0;
            v_data   <= 24'd0;
            src_last <= 1'b0;
        end else begin
            // One cycle line buffer write pulse.  ST_HWR raises it together
            // with the registered address and data, the next state consumes
            // it, and every other state keeps it low.
            h_wr_pend <= 1'b0;
            case (state)
            ST_IDLE: begin
                if (s_valid) begin
                    // Latch the geometry first; the ST_INIT states then
                    // compute the start phase from these stable registers.
                    w_src    <= src_w;
                    h_src    <= src_h;
                    src_col  <= 16'd0;
                    cap_col  <= 16'd0;
                    px_prev  <= 24'd0;
                    px_cur   <= 24'd0;
                    hx       <= 12'd0;
                    hbuf_sel <= 1'b0;
                    src_row  <= 16'd0;
                    rptr     <= 10'd0;
                    out_row  <= 10'd0;
                    v_data   <= 24'd0;
                    src_last <= 1'b0;
                    state    <= ST_INIT;
                end
            end

            ST_INIT: begin
                h_sum     <= h_raw;
                v_sum     <= v_raw;
                h_small_r <= h_small;
                v_small_r <= v_small;
                h_step    <= {7'd0, w_src, 1'b0};
                v_step    <= {7'd0, h_src, 1'b0};
                w_m1_r    <= w_src - 16'd1;
                h_m1_r    <= h_src - 16'd1;
                state     <= ST_INIT2;
            end

            ST_INIT2: begin
                h_nrm <= norm_phase(h_sum, H_DEN);
                v_nrm <= norm_phase(v_sum, V_DEN);
                state <= ST_INIT3;
            end

            ST_INIT3: begin
                h_q0   <= h_nrm[23:0];
                h_x0_0 <= h_small_r ? -16'sd1 : $signed({12'd0, h_nrm[27:24]});
                v_q0   <= v_nrm[23:0];
                v_y0_0 <= v_small_r ? -16'sd1 : $signed({12'd0, v_nrm[27:24]});
                state  <= ST_INIT4;
            end

            ST_INIT4: begin
                h_w0  <= h_start_edge ? 8'd0 : weight_h(h_q0);
                v_w0  <= v_start_edge ? 8'd0 : weight_v(v_q0);
                h_w   <= h_start_edge ? 8'd0 : weight_h(h_q0);
                v_w   <= v_start_edge ? 8'd0 : weight_v(v_q0);
                h_q   <= h_q0;
                h_x0  <= h_x0_0;
                h_x1  <= h_x1_0;
                h_edge<= h_edge_0;
                v_q   <= v_q0;
                v_y0  <= v_y0_0;
                v_y1  <= v_y1_0;
                v_same<= v_same_0;
                state <= ST_HCAP;
            end

            ST_HCAP: begin
                if (s_valid) begin
                    if (src_col == 16'd0)
                        px_prev <= s_rgb;
                    else
                        px_prev <= px_cur;
                    px_cur   <= s_rgb;
                    cap_col  <= src_col;
                    src_col  <= src_col + 16'd1;
                    if (s_last)
                        src_last <= 1'b1;
                    state <= ST_HWR;
                end
            end

            ST_HWR: begin
                if ((h_x1 == cap_col) && (hx < DST_W)) begin
                    // Register the interpolated column together with its
                    // address and buffer select; the RAM sees only registers,
                    // so the mix multiply is no longer on the way to the
                    // memory data input.
                    h_mix_r   <= h_mix;
                    h_wr_addr <= hx[9:0];
                    h_wr_buf  <= hbuf_sel;
                    h_wr_pend <= 1'b1;
                    if (hx == DST_W_M1) begin
                        hx    <= 12'd0;
                        state <= ST_VCHECK;
                    end else begin
                        hx    <= hx + 12'd1;
                        state <= ST_HADV1;
                    end
                end else begin
                    state <= ST_HCAP;
                end
            end

            ST_HADV1: begin
                h_sum <= h_q + h_step;
                state <= ST_HADV2;
            end

            ST_HADV2: begin
                h_nrm <= norm_phase(h_sum, H_DEN);
                state <= ST_HADV3;
            end

            ST_HADV3: begin
                h_q   <= h_nrm[23:0];
                h_x0  <= h_x0_n;
                h_x1  <= h_x1_n;
                h_edge<= h_x0_n_edge;
                h_w   <= h_x0_n_edge ? 8'd0 : weight_h(h_nrm[23:0]);
                state <= ST_HWR;
            end

            ST_VCHECK: begin
                if (out_row == DST_H) begin
                    state <= ST_DONE;
                end else if (v_y1 == src_row) begin
                    rptr  <= 10'd0;
                    state <= ST_VFETCH;
                end else if (src_last) begin
                    // A malformed geometry must not hold the parser forever.
                    state <= ST_DONE;
                end else begin
                    hx       <= 12'd0;
                    h_q      <= h_q0;
                    h_x0     <= h_x0_0;
                    h_x1     <= h_x1_0;
                    h_edge   <= h_edge_0;
                    h_w      <= h_w0;
                    src_col  <= 16'd0;
                    cap_col  <= 16'd0;
                    src_row  <= src_row + 16'd1;
                    hbuf_sel <= ~hbuf_sel;
                    state    <= ST_HCAP;
                end
            end

            ST_VFETCH: begin
                state <= ST_VLATCH;
            end

            ST_VLATCH: begin
                v_data <= v_mix;
                state  <= ST_VALID;
            end

            ST_VALID: begin
                if (m_ready) begin
                    if (rptr == DST_W_M1[9:0]) begin
                        out_row <= out_row + 10'd1;
                        state   <= ST_VADV1;
                    end else begin
                        rptr  <= rptr + 10'd1;
                        state <= ST_VFETCH;
                    end
                end
            end

            ST_VADV1: begin
                v_sum <= v_q + v_step;
                state <= ST_VADV2;
            end

            ST_VADV2: begin
                v_nrm <= norm_phase(v_sum, V_DEN);
                state <= ST_VADV3;
            end

            ST_VADV3: begin
                // The remainder-to-weight constant division takes about a
                // full clock on its own.  Register the remainder here and
                // split the division into a second stage.
                v_w_q <= v_nrm[23:0];
                state <= ST_VADV4;
            end

            ST_VADV4: begin
                v_q   <= v_nrm[23:0];
                v_y0  <= v_y0_n;
                v_y1  <= v_y1_n;
                v_same<= v_y0_n_edge;
                v_w   <= v_y0_n_edge ? 8'd0 : weight_v(v_w_q[11:0]);
                state <= ST_VCHECK;
            end

            ST_DONE: begin
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
