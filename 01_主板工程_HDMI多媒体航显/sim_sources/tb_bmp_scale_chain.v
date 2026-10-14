`timescale 1ns / 1ps
//=============================================================================
// tb_bmp_scale_chain.v
//
// BMP parser (SCALE_EN = 1) + bmp_scaler: a picture of any size must arrive
// at the frame store loader as exactly 640 x 480 pixels, in bottom up order,
// with every pixel equal to the center-aligned, 8-bit bilinear model.
//
// The byte stream is generated on the fly, so no large file has to be stored:
// a 960 x 720 case would otherwise need a 2 MB array.  The pixel value is a
// pure function of the source coordinate, which lets the checker recompute the
// expected output instead of storing it.
//
// Cases
//   640 x 480   identity, row padding 0
//   300 x 200   upscale on both axes, non integer ratio
//   960 x 720   downscale on both axes, non integer ratio
//   641 x 481   one pixel wider and taller than the display, row padding 1
//
// The output side is deliberately back pressured (one idle cycle in five) so
// the scaler line buffer handshake is exercised, and the parser's hold_done
// path is live because the scaler is the one that stalls it.
//
// The checker mirrors the two-stage RTL exactly: horizontal interpolation is
// rounded to 8 bits per source row, then vertical interpolation is applied to
// those two horizontally scaled rows.  This catches both mapping and
// rounding differences, not just approximate visual agreement.
//=============================================================================
module tb_bmp_scale_chain;

    // 0 = inferred line buffers (the portable model), 1 = the board BRAM macro
    // via the stub in sim/check_stubs.v.  Both must produce the same picture,
    // the board build uses 1.
    parameter integer BRAM_PRIM = 0;

    localparam integer DST_W = 640;
    localparam integer DST_H = 480;
    localparam integer OUT_PIX = DST_W * DST_H;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    always #10 clk = ~clk;

    //------------------------------------------------------------------
    // geometry of the case that is running
    //------------------------------------------------------------------
    integer W, H;
    integer ROW_BYTES, FILE_SIZE;
    integer DATA_OFFSET = 54;

    reg [7:0] hdr [0:53];

    //------------------------------------------------------------------
    // DUTs
    //------------------------------------------------------------------
    reg         start;
    wire        busy, done, error;
    wire [7:0]  error_code;
    reg  [7:0]  data;
    reg         data_valid;
    wire        data_ready;
    wire        px_valid;
    wire [23:0] px_rgb;
    wire        px_last;
    wire [15:0] hdr_w, hdr_h, hdr_bpp;
    wire [31:0] hdr_off;

    wire        sc_ready;
    wire        ld_valid;
    wire [23:0] ld_rgb;
    wire        sc_busy;
    reg         ld_ready;

    bmp_stream_parser #(
        .IMG_W    (DST_W),
        .IMG_H    (DST_H),
        .SCALE_EN (1),
        .MAX_W    (1920),
        .MAX_H    (2048)
    ) dut_parser (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (start),
        .file_size       (FILE_SIZE[31:0]),
        .busy            (busy),
        .done            (done),
        .error           (error),
        .error_code      (error_code),
        .data            (data),
        .data_valid      (data_valid),
        .data_ready      (data_ready),
        .hold_done       (sc_busy),
        .px_valid        (px_valid),
        .px_rgb          (px_rgb),
        .px_last         (px_last),
        .px_ready        (sc_ready),
        .hdr_width       (hdr_w),
        .hdr_height      (hdr_h),
        .hdr_bpp         (hdr_bpp),
        .hdr_data_offset (hdr_off)
    );

    bmp_scaler #(
        .DST_W (DST_W),
        .DST_H (DST_H),
        .BRAM_PRIM (BRAM_PRIM)
    ) dut_scaler (
        .clk       (clk),
        .rst_n     (rst_n),
        .frame_rst (start),
        .src_w     (hdr_w),
        .src_h     (hdr_h),
        .s_valid   (px_valid),
        .s_rgb     (px_rgb),
        .s_last    (px_last),
        .s_ready   (sc_ready),
        .m_valid   (ld_valid),
        .m_rgb     (ld_rgb),
        .m_ready   (ld_ready),
        .busy      (sc_busy),
        .dbg_row   ()
    );

    //------------------------------------------------------------------
    // byte producer: one byte per handshake, derived from the step index
    //------------------------------------------------------------------
    reg [31:0] step;
    reg        restart_producer, producer_enable;

    function [23:0] pattern;
        input integer x;
        input integer y;
        integer v;
        begin
            v = (x * 7919) + (y * 104729);
            pattern = v[23:0];
        end
    endfunction

    function [7:0] file_byte;
        input integer s;
        integer p, r, wth, x, b;
        reg [23:0] px;
        begin
            if (s < 54)
                file_byte = hdr[s];
            else begin
                p   = s - 54;
                r   = p / ROW_BYTES;
                wth = p % ROW_BYTES;
                if (r >= H)
                    file_byte = 8'h00;
                else if (wth < (3 * W)) begin
                    x  = wth / 3;
                    b  = wth % 3;
                    px = pattern(x, r);
                    file_byte = (b == 0) ? px[7:0] :
                                (b == 1) ? px[15:8] : px[23:16];
                end else
                    file_byte = 8'hA5;      // row padding
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data       <= 8'd0;
            data_valid <= 1'b0;
            step       <= 32'd0;
        end else if (restart_producer) begin
            step       <= 32'd0;
            data       <= file_byte(0);
            data_valid <= 1'b0;
        end else if (!producer_enable) begin
            data_valid <= 1'b0;
        end else if (data_valid) begin
            if (data_ready) begin
                step <= step + 32'd1;
                if ((step + 32'd1) >= FILE_SIZE[31:0])
                    data_valid <= 1'b0;
                else begin
                    data       <= file_byte(step + 32'd1);
                    data_valid <= 1'b1;
                end
            end
        end else if (step < FILE_SIZE[31:0]) begin
            data       <= file_byte(step);
            data_valid <= 1'b1;
        end
    end

    //------------------------------------------------------------------
    // output back pressure: idle one cycle in five
    //------------------------------------------------------------------
    reg [2:0] bp_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bp_cnt   <= 3'd0;
            ld_ready <= 1'b1;
        end else begin
            bp_cnt   <= bp_cnt + 3'd1;
            ld_ready <= (bp_cnt != 3'd4);
        end
    end

    //------------------------------------------------------------------
    // checker
    //------------------------------------------------------------------
    integer   out_idx;
    integer   errors;
    integer   mismatch;
    reg       done_seen;

    integer   dbg_count;

    // Diagnostic only: compile/run with +DBG +EARLY to inspect the first
    // few rows of the first case.  The normal regression does not enable it.
    always @(posedge clk) begin
        if (!$test$plusargs("DBG"))
            dbg_count = 0;
        else if (ld_valid && ld_ready) begin
            if ((dbg_count < 5) ||
                ((dbg_count >= 640) && (dbg_count < 645)))
                $display("DBG out=%0d y=%0d x=%0d v_y0=%0d v_y1=%0d v_q=%0d v_w=%0d same=%b hbuf=%b rd0=%06h rd1=%06h va=%06h vb=%06h",
                         dbg_count, dbg_count / DST_W, dbg_count % DST_W,
                         dut_scaler.v_y0, dut_scaler.v_y1, dut_scaler.v_q,
                         dut_scaler.v_w, dut_scaler.v_same, dut_scaler.hbuf_sel,
                         dut_scaler.rd_b0, dut_scaler.rd_b1,
                         dut_scaler.v_pix_a, dut_scaler.v_pix_b);
            dbg_count = dbg_count + 1;
        end
    end

    function [7:0] model_mix8;
        input integer a;
        input integer b;
        input integer w;
        integer d;
        integer t;
        begin
            if (w == 0)
                model_mix8 = a[7:0];
            else if (w == 255)
                model_mix8 = b[7:0];
            else begin
                d = b - a;
                t = (d * w + 128) >>> 8;
                t = t + a;
                if (t < 0)
                    model_mix8 = 8'd0;
                else if (t > 255)
                    model_mix8 = 8'd255;
                else
                    model_mix8 = t[7:0];
            end
        end
    endfunction

    function [23:0] model_mix24;
        input [23:0] a;
        input [23:0] b;
        input integer w;
        begin
            model_mix24 = {
                model_mix8(a[23:16], b[23:16], w),
                model_mix8(a[15:8],  b[15:8],  w),
                model_mix8(a[7:0],   b[7:0],   w)
            };
        end
    endfunction

    task map_axis;
        input integer d;
        input integer dst;
        input integer src;
        output integer p0;
        output integer p1;
        output integer w;
        integer raw;
        integer den;
        begin
            den = 2 * dst;
            raw = (2 * d + 1) * src - dst;
            if (raw < 0) begin
                p0 = 0;
                p1 = (src > 1) ? 1 : 0;
                w  = 0;
            end else begin
                p0 = raw / den;
                if (p0 >= (src - 1)) begin
                    p0 = src - 1;
                    p1 = src - 1;
                    w  = 0;
                end else begin
                    p1 = p0 + 1;
                    w  = ((raw % den) * 256) / den;
                end
            end
        end
    endtask

    task check_output;
        integer y, x, x0, x1, y0, y1, wx, wy;
        reg [23:0] h0, h1, exp;
        begin
            y = out_idx / DST_W;
            x = out_idx % DST_W;
            map_axis(x, DST_W, W, x0, x1, wx);
            map_axis(y, DST_H, H, y0, y1, wy);
            h0  = model_mix24(pattern(x0, y0), pattern(x1, y0), wx);
            h1  = model_mix24(pattern(x0, y1), pattern(x1, y1), wx);
            exp = model_mix24(h0, h1, wy);
            if (ld_rgb !== exp) begin
                mismatch = mismatch + 1;
                if (mismatch <= 5)
                    $display("  mismatch out[%0d] (%0d,%0d) got %06h want %06h",
                             out_idx, x, y, ld_rgb, exp);
            end
            out_idx = out_idx + 1;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_idx   = 0;
            mismatch  = 0;
            done_seen = 1'b0;
        end else begin
            if (done)
                done_seen = 1'b1;
            if (ld_valid && ld_ready) begin
                if (out_idx < OUT_PIX)
                    check_output;
                else
                    mismatch = mismatch + 1;
            end
            if ($test$plusargs("EARLY") && (out_idx > 700))
                $finish;
        end
    end

    //------------------------------------------------------------------
    // stimulus
    //------------------------------------------------------------------
    integer k;
    integer nbytes;

    task build_header;
        begin
            for (k = 0; k < 54; k = k + 1)
                hdr[k] = 8'd0;
            hdr[0]  = 8'h42;            // B
            hdr[1]  = 8'h4D;            // M
            hdr[10] = DATA_OFFSET[7:0];
            hdr[14] = 8'd40;            // BITMAPINFOHEADER
            hdr[18] = W[7:0];
            hdr[19] = W[15:8];
            hdr[20] = W[23:16];
            hdr[21] = W[31:24];
            hdr[22] = H[7:0];
            hdr[23] = H[15:8];
            hdr[24] = H[23:16];
            hdr[25] = H[31:24];
            hdr[26] = 8'd1;             // planes
            hdr[28] = 8'd24;            // bpp
            hdr[30] = 8'd0;             // BI_RGB
        end
    endtask

    task run_case;
        input integer w;
        input integer h;
        input [8*28:1] name;
        begin
            W         = w;
            H         = h;
            ROW_BYTES = 3 * w;
            ROW_BYTES = ROW_BYTES + ((4 - (ROW_BYTES % 4)) % 4);
            FILE_SIZE = DATA_OFFSET + ROW_BYTES * H;
            nbytes    = FILE_SIZE;

            build_header;
            out_idx   = 0;
            mismatch  = 0;
            done_seen = 1'b0;

            @(negedge clk);
            producer_enable  = 1'b0;
            restart_producer = 1'b1;
            @(negedge clk);
            restart_producer = 1'b0;

            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            producer_enable = 1'b1;

            while (!done_seen)
                @(negedge clk);

            // the scaler must be completely drained by now
            if (sc_busy) begin
                $display("FAIL %0s: scaler still busy when the parser finished",
                         name);
                errors = errors + 1;
            end
            if (error) begin
                $display("FAIL %0s: parser error code %0d", name, error_code);
                errors = errors + 1;
            end
            if (mismatch != 0) begin
                $display("FAIL %0s: %0d of %0d pixels differ from bilinear model",
                         name, mismatch, OUT_PIX);
                errors = errors + 1;
            end
            if (out_idx != OUT_PIX) begin
                $display("FAIL %0s: %0d output pixels, expected %0d",
                         name, out_idx, OUT_PIX);
                errors = errors + 1;
            end
            $display("case %0s: %0d x %0d -> %0d x %0d, %0d bytes, %0d pixels, mismatch %0d",
                     name, W, H, DST_W, DST_H, nbytes, out_idx, mismatch);
        end
    endtask

    integer watchdog;

    initial begin
        errors           = 0;
        start            = 1'b0;
        producer_enable  = 1'b0;
        restart_producer = 1'b0;
        W = DST_W; H = DST_H; ROW_BYTES = 3 * DST_W; FILE_SIZE = 0;
        build_header;

        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        run_case(640, 480, "identity  640x480");
        run_case(300, 200, "upscale   300x200");
        run_case(960, 720, "downscale 960x720");
        run_case(641, 481, "odd       641x481");

        if (errors == 0)
            $display("RESULT: PASS tb_bmp_scale_chain");
        else
            $display("RESULT: FAIL tb_bmp_scale_chain errors=%0d", errors);
        $finish;
    end

    // a stuck handshake must not hang the regression
    initial begin
        watchdog = 0;
        forever begin
            @(negedge clk);
            watchdog = watchdog + 1;
            if (watchdog > 40_000_000) begin
                $display("RESULT: FAIL tb_bmp_scale_chain timeout out_idx=%0d busy=%b",
                         out_idx, sc_busy);
                $finish;
            end
        end
    end

endmodule
