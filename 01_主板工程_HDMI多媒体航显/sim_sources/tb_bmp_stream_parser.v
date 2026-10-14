`timescale 1ns / 1ps
//=============================================================================
// tb_bmp_stream_parser.v - unit test for the BMP byte to pixel converter
//
// A 3 x 2 picture is used so that the per row padding logic is exercised
// (3 pixels = 9 bytes, padded to 12). The picture is bottom up, so the file
// order is row 1 first and the expected pixel order follows the file.
//=============================================================================
module tb_bmp_stream_parser;

    localparam integer W = 3;
    localparam integer H = 2;
    localparam integer ROW_BYTES   = 12;
    localparam integer DATA_OFFSET = 54;
    localparam integer FILE_SIZE   = DATA_OFFSET + ROW_BYTES * H;   // 78

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    always #10 clk = ~clk;

    reg         start;
    reg  [31:0] file_size;
    wire        busy, done, error;
    wire [7:0]  error_code;
    reg  [7:0]  data;
    reg         data_valid;
    wire        data_ready;
    wire        px_valid;
    wire [23:0] px_rgb;
    wire        px_last;
    wire [15:0] hdr_width, hdr_height, hdr_bpp;
    wire [31:0] hdr_data_offset;

    bmp_stream_parser #(
        .IMG_W (W),
        .IMG_H (H)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (start),
        .file_size       (file_size),
        .busy            (busy),
        .done            (done),
        .error           (error),
        .error_code      (error_code),
        .data            (data),
        .data_valid      (data_valid),
        .data_ready      (data_ready),
        .hold_done       (1'b0),
        .px_valid        (px_valid),
        .px_rgb          (px_rgb),
        .px_last         (px_last),
        .px_ready        (1'b1),
        .hdr_width       (hdr_width),
        .hdr_height      (hdr_height),
        .hdr_bpp         (hdr_bpp),
        .hdr_data_offset (hdr_data_offset)
    );

    reg [7:0] img [0:255];
    integer   plen;
    integer   pidx;
    integer   got;
    integer   errors;
    integer   k;
    integer   last_seen;
    reg [23:0] expected [0:63];

    //------------------------------------------------------------------
    // byte producer: holds a byte until the parser takes it
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data       <= 8'd0;
            data_valid <= 1'b0;
            pidx       <= 0;
        end else begin
            if (data_valid) begin
                if (data_ready) begin
                    if (pidx < plen) begin
                        data <= img[pidx];
                        pidx <= pidx + 1;
                    end else
                        data_valid <= 1'b0;
                end
            end else if (pidx < plen) begin
                data       <= img[pidx];
                data_valid <= 1'b1;
                pidx       <= pidx + 1;
            end
        end
    end

    //------------------------------------------------------------------
    // pixel consumer
    //------------------------------------------------------------------
    always @(negedge clk) begin
        if (rst_n && px_valid) begin
            if (got < 64) begin
                if (px_rgb !== expected[got]) begin
                    $display("FAIL: pixel %0d got %06h want %06h",
                             got, px_rgb, expected[got]);
                    errors = errors + 1;
                end
            end
            if (px_last)
                last_seen = last_seen + 1;
            got = got + 1;
        end
    end

    task build_image;
        input integer mode;
        input integer size;
        integer        n;
        begin
            for (n = 0; n < 128; n = n + 1)
                img[n] = 8'h00;

            img[0]  = 8'h42;    // B
            img[1]  = 8'h4D;    // M
            img[2]  = 8'h4E;    // file size 78, informational
            img[3]  = 8'h00;
            img[4]  = 8'h00;
            img[5]  = 8'h00;
            img[10] = 8'd54;    // pixel data offset
            img[11] = 8'h00;
            img[12] = 8'h00;
            img[13] = 8'h00;
            img[14] = 8'd40;    // BITMAPINFOHEADER size
            img[15] = 8'h00;
            img[16] = 8'h00;
            img[17] = 8'h00;
            img[18] = W[7:0];   // width
            img[19] = 8'h00;
            img[20] = 8'h00;
            img[21] = 8'h00;
            img[22] = H[7:0];   // height, positive: bottom up
            img[23] = 8'h00;
            img[24] = 8'h00;
            img[25] = 8'h00;
            img[26] = 8'h01;    // planes
            img[27] = 8'h00;
            img[28] = 8'd24;    // bits per pixel
            img[29] = 8'h00;
            img[30] = 8'h00;    // BI_RGB
            img[31] = 8'h00;
            img[32] = 8'h00;
            img[33] = 8'h00;

            // file order: bottom row (picture row 1) then picture row 0
            // every pixel is stored blue, green, red
            img[54] = 8'h01; img[55] = 8'h02; img[56] = 8'h03;
            img[57] = 8'h04; img[58] = 8'h05; img[59] = 8'h06;
            img[60] = 8'h07; img[61] = 8'h08; img[62] = 8'h09;
            img[63] = 8'hAA; img[64] = 8'hBB; img[65] = 8'hCC;  // padding
            img[66] = 8'h10; img[67] = 8'h11; img[68] = 8'h12;
            img[69] = 8'h13; img[70] = 8'h14; img[71] = 8'h15;
            img[72] = 8'h16; img[73] = 8'h17; img[74] = 8'h18;
            img[75] = 8'hDD; img[76] = 8'hEE; img[77] = 8'hFF;  // padding

            if (mode == 1) img[0]  = 8'h58;             // bad magic
            if (mode == 2) img[28] = 8'd32;             // bad bpp
            if (mode == 3) img[30] = 8'h01;             // RLE compression
            if (mode == 4) img[18] = 8'd4;              // wrong width
            if (mode == 5) begin                        // negative height:
                img[22] = 8'hFE;                        // top down BMP
                img[23] = 8'hFF;
                img[24] = 8'hFF;
                img[25] = 8'hFF;
            end

            file_size = size;
            plen      = size;
        end
    endtask

    task expect_pixel;
        input [23:0] value;
        begin
            expected[got] = value;
            got = got + 1;
        end
    endtask

    task run_case;
        input integer mode;
        input integer size;
        input [7:0]   want_err;
        input integer want_pixels;
        begin
            build_image(mode, size);
            got       = 0;
            pidx      = 0;
            last_seen = 0;
            data_valid = 1'b0;
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            wait (done || error);
            @(negedge clk);

            if (want_err == 8'd0) begin
                if (error) begin
                    $display("FAIL case%0d: unexpected error code %0d",
                             mode, error_code);
                    errors = errors + 1;
                end
                if (got != want_pixels) begin
                    $display("FAIL case%0d: %0d pixels, expected %0d",
                             mode, got, want_pixels);
                    errors = errors + 1;
                end
                if (last_seen != 1) begin
                    $display("FAIL case%0d: px_last seen %0d times",
                             mode, last_seen);
                    errors = errors + 1;
                end
                if ((hdr_width != W) || (hdr_height != H) ||
                    (hdr_bpp != 16'd24) || (hdr_data_offset != 32'd54)) begin
                    $display("FAIL case%0d: header w=%0d h=%0d bpp=%0d off=%0d",
                             mode, hdr_width, hdr_height, hdr_bpp,
                             hdr_data_offset);
                    errors = errors + 1;
                end
            end else begin
                if (!error) begin
                    $display("FAIL case%0d: expected error %0d, none seen",
                             mode, want_err);
                    errors = errors + 1;
                end else if (error_code != want_err) begin
                    $display("FAIL case%0d: error code %0d, expected %0d",
                             mode, error_code, want_err);
                    errors = errors + 1;
                end
            end

            // let the producer drain and settle before the next case
            repeat (4) @(posedge clk);
        end
    endtask

    initial begin
        errors     = 0;
        start      = 1'b0;
        data       = 8'd0;
        data_valid = 1'b0;
        file_size  = 32'd0;
        pidx       = 0;
        plen       = 0;
        got        = 0;
        last_seen  = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // good picture: file row 1 first, then file row 0
        got = 0;
        expect_pixel(24'h03_02_01); expect_pixel(24'h06_05_04);
        expect_pixel(24'h09_08_07); expect_pixel(24'h12_11_10);
        expect_pixel(24'h15_14_13); expect_pixel(24'h18_17_16);
        run_case(0, FILE_SIZE, 8'd0, 6);

        run_case(1, FILE_SIZE, 8'd1, 0);   // magic
        run_case(2, FILE_SIZE, 8'd2, 0);   // bpp
        run_case(3, FILE_SIZE, 8'd3, 0);   // compression
        run_case(4, FILE_SIZE, 8'd4, 0);   // width
        run_case(5, FILE_SIZE, 8'd9, 0);   // top down
        run_case(0, 60,       8'd7, 0);   // truncated file

        if (errors == 0)
            $display("RESULT: PASS tb_bmp_stream_parser");
        else
            $display("RESULT: FAIL tb_bmp_stream_parser errors=%0d", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL tb_bmp_stream_parser timeout busy=%b", busy);
        $finish;
    end

endmodule
