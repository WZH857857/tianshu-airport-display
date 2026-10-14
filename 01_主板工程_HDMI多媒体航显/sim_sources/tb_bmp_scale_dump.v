`timescale 1ns / 1ps
//=============================================================================
// tb_bmp_scale_dump.v - stream a real BMP file through the scaling chain and
// write the 640 x 480 stream that the frame store would receive.
//
// Unlike tb_bmp_scale_chain.v, which compares against a formula, this bench
// reads an actual file from disk.  The dump is a plain 24 bit RGB byte stream
// in the order the scaler emits it, that is the BMP row order (bottom row
// first), so a viewer has to flip it vertically - exactly what the frame store
// does with the rows on the way to the screen.
//
// usage:
//   vvp tb_bmp_scale_dump.vvp +bmp=<input.bmp> +out=<dump.rgb> +size=<bytes>
//=============================================================================
module tb_bmp_scale_dump;

    localparam integer DST_W = 640;
    localparam integer DST_H = 480;
    localparam integer OUT_PIX = DST_W * DST_H;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    always #5 clk = ~clk;

    reg [8*512:1] bmp_path;
    reg [8*512:1] out_path;
    integer       fin, fout;
    integer       file_bytes;

    wire [15:0] hdr_w, hdr_h, hdr_bpp;
    wire [31:0] hdr_off;

    reg         start;
    wire        busy, done, error;
    wire [7:0]  error_code;
    reg  [7:0]  data;
    reg         data_valid;
    wire        data_ready;
    wire        px_valid;
    wire [23:0] px_rgb;
    wire        px_last;

    wire        sc_ready;
    wire        ld_valid;
    wire [23:0] ld_rgb;
    wire        sc_busy;

    reg         ld_ready;
    integer     out_count;
    integer     expected;
    reg [31:0]  remaining;
    reg         feeding;

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
        .file_size       (file_bytes[31:0]),
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
        .DST_H (DST_H)
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
    // byte source: one file byte per handshake
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data       <= 8'd0;
            data_valid <= 1'b0;
            remaining  <= 32'd0;
        end else if (!feeding) begin
            data_valid <= 1'b0;
        end else if (data_valid) begin
            if (data_ready) begin
                if (remaining == 32'd0)
                    data_valid <= 1'b0;
                else begin
                    data       <= $fgetc(fin);
                    data_valid <= 1'b1;
                    remaining  <= remaining - 32'd1;
                end
            end
        end else if (remaining != 32'd0) begin
            data       <= $fgetc(fin);
            data_valid <= 1'b1;
            remaining  <= remaining - 32'd1;
        end
    end

    //------------------------------------------------------------------
    // destination dump
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ld_ready  <= 1'b1;
            out_count <= 0;
        end else if (ld_valid && ld_ready) begin
            $fwrite(fout, "%c%c%c", ld_rgb[23:16], ld_rgb[15:8], ld_rgb[7:0]);
            out_count <= out_count + 1;
        end
    end

    integer watchdog;
    reg     seen_done;

    initial begin
        watchdog = 0;
        forever begin
            @(posedge clk);
            watchdog = watchdog + 1;
            if (watchdog > 60_000_000) begin
                $display("RESULT: FAIL tb_bmp_scale_dump timeout out=%0d", out_count);
                $finish;
            end
        end
    end

    always @(posedge clk or negedge rst_n)
        if (!rst_n)
            seen_done <= 1'b0;
        else if (done)
            seen_done <= 1'b1;

    initial begin
        start   = 1'b0;
        feeding = 1'b0;
        seen_done = 1'b0;
        expected  = OUT_PIX;

        if (!$value$plusargs("bmp=%s", bmp_path)) begin
            $display("RESULT: FAIL tb_bmp_scale_dump no +bmp");
            $finish;
        end
        if (!$value$plusargs("out=%s", out_path)) begin
            $display("RESULT: FAIL tb_bmp_scale_dump no +out");
            $finish;
        end
        if (!$value$plusargs("size=%d", file_bytes)) begin
            $display("RESULT: FAIL tb_bmp_scale_dump no +size");
            $finish;
        end

        fin  = $fopen(bmp_path, "rb");
        fout = $fopen(out_path, "wb");
        if (fin == 0 || fout == 0) begin
            $display("RESULT: FAIL tb_bmp_scale_dump cannot open file");
            $finish;
        end

        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (3) @(negedge clk);

        remaining = file_bytes[31:0];
        start     = 1'b1;
        @(negedge clk);
        start   = 1'b0;
        feeding = 1'b1;

        while (!seen_done)
            @(negedge clk);
        repeat (200) @(negedge clk);

        $fclose(fout);
        $fclose(fin);

        if (error) begin
            $display("RESULT: FAIL tb_bmp_scale_dump parser error code %0d", error_code);
            $finish;
        end
        if (out_count != expected) begin
            $display("RESULT: FAIL tb_bmp_scale_dump %0d pixels, expected %0d",
                     out_count, expected);
            $finish;
        end
        $display("RESULT: PASS tb_bmp_scale_dump %0s -> %0s : %0d x %0d, %0d pixels, src %0d x %0d, %0d bytes",
                 bmp_path, out_path, DST_W, DST_H, out_count, hdr_w, hdr_h, file_bytes);
        $finish;
    end

endmodule
