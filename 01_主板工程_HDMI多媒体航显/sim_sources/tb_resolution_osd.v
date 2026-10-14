`timescale 1ns / 1ps
//=============================================================================
// tb_resolution_osd.v - focused check for the board-side resolution readout
//
// The full overlay bench walks every pixel of a frame and is very slow under
// Icarus.  This bench only drives the second row of the top-left panel:
//
//   * the row is blank before the frame store reports a size,
//   * 640x480 produces RES 640X480,
//   * 960x720 produces RES 960X720,
//   * the panel's right edge does not move when the text changes.
//=============================================================================
module tb_resolution_osd;

    localparam [9:0] TOP_X0 = 10'd12;
    localparam [9:0] TOP_X1 = 10'd236;
    localparam [9:0] TOP_Y0 = 10'd12;
    localparam [9:0] RES_Y0 = 10'd38;
    localparam [9:0] RES_Y1 = 10'd46;

    reg        pixel_clk = 1'b0;
    always #20 pixel_clk = ~pixel_clk;

    reg        rst = 1'b1;
    reg        de = 1'b1;
    reg [9:0]  px = 10'd0;
    reg [9:0]  py = 10'd0;
    reg [23:0] base_rgb = 24'h808080;
    reg        ui_enable = 1'b1;
    reg [2:0]  img_slot = 3'd0;
    reg [7:0]  img_count = 8'd6;
    reg [11:0] img_width = 12'd0;
    reg [11:0] img_height = 12'd0;
    reg        img_size_valid = 1'b0;
    reg        frame_tick = 1'b0;
    reg [2:0]  user_bright = 3'd7;
    reg [2:0]  user_contrast = 3'd3;
    reg [1:0]  ui_mode = 2'd0;
    reg        param_flash = 1'b0;
    reg [1:0]  param_sel = 2'd0;
    reg        auto_mode = 1'b0;
    reg        logo_enable = 1'b1;
    reg        subtitle_enable = 1'b0;
    reg        timestamp_enable = 1'b0;
    reg        parallax_enable = 1'b0;
    reg        osd_enable = 1'b0;
    reg        wave_enable = 1'b0;
    reg        audio_rst = 1'b0;
    reg        audio_valid = 1'b0;
    reg [23:0] audio_left = 24'd0;

    wire [23:0] rgb;
    wire        hit;

    ui_overlay dut (
        .pixel_clk       (pixel_clk),
        .rst             (rst),
        .frame_tick      (frame_tick),
        .de              (de),
        .px              (px),
        .py              (py),
        .base_rgb        (base_rgb),
        .ui_enable       (ui_enable),
        .img_slot        (img_slot),
        .img_count       (img_count),
        .img_width       (img_width),
        .img_height      (img_height),
        .img_size_valid  (img_size_valid),
        .user_bright     (user_bright),
        .user_contrast   (user_contrast),
        .ui_mode         (ui_mode),
        .param_flash     (param_flash),
        .param_sel       (param_sel),
        .auto_mode       (auto_mode),
        .logo_enable     (logo_enable),
        .subtitle_enable (subtitle_enable),
        .timestamp_enable (timestamp_enable),
        .parallax_enable (parallax_enable),
        .osd_enable      (osd_enable),
        .wave_enable     (wave_enable),
        .host_text_tog   (1'b0),
        .host_text_addr  (5'd0),
        .host_text_data  (8'd0),
        .host_text_commit_tog (1'b0),
        .host_text_clear_tog  (1'b0),
        .host_link       (1'b0),
        .board_enable    (1'b0),
        .board_wr_tog    (1'b0),
        .board_wr_addr   (8'd0),
        .board_wr_data   (8'd0),
        .audio_rst       (audio_rst),
        .audio_valid     (audio_valid),
        .audio_left      (audio_left),
        .rgb             (rgb),
        .hit             (hit)
    );

    integer errors = 0;
    integer i, j;
    integer cyan_pixels;

    task wait_for_bcd;
        input expected;
        integer guard;
        begin
            guard = 0;
            while ((dut.res_bcd_valid !== expected) && (guard < 20000)) begin
                @(posedge pixel_clk);
                guard = guard + 1;
            end
            if (dut.res_bcd_valid !== expected) begin
                errors = errors + 1;
                $display("FAIL: res_bcd_valid never became %0d", expected);
            end
        end
    endtask

    task count_resolution_pixels;
        output integer count;
        begin
            count = 0;
            for (j = RES_Y0; j < RES_Y1; j = j + 1) begin
                for (i = TOP_X0; i < TOP_X1; i = i + 4) begin
                    px = i[9:0];
                    py = j[9:0];
                    #1;
                    if (hit && (rgb === 24'h00E0FF))
                        count = count + 1;
                end
            end
        end
    endtask

    initial begin
        rst = 1'b1;
        repeat (4) @(posedge pixel_clk);
        rst = 1'b0;
        repeat (4) @(posedge pixel_clk);

        // Before the frame store publishes a size, the second row is blank.
        count_resolution_pixels(cyan_pixels);
        if (cyan_pixels != 0) begin
            errors = errors + 1;
            $display("FAIL: resolution row drawn without a valid size");
        end

        // 640x480 must become BCD 0640 x 0480.
        img_width = 12'd640;
        img_height = 12'd480;
        img_size_valid = 1'b1;
        wait_for_bcd(1'b1);
        if ((dut.res_width_bcd !== 16'h0640) ||
            (dut.res_height_bcd !== 16'h0480)) begin
            errors = errors + 1;
            $display("FAIL: 640x480 became %04h x %04h",
                     dut.res_width_bcd, dut.res_height_bcd);
        end

        count_resolution_pixels(cyan_pixels);
        if (cyan_pixels < 20) begin
            errors = errors + 1;
            $display("FAIL: 640x480 text is missing (%0d pixels)", cyan_pixels);
        end
        $display("RES OSD TB: 640x480 -> %04h x %04h, %0d sampled pixels",
                 dut.res_width_bcd, dut.res_height_bcd, cyan_pixels);

        // A new source size must invalidate the old readout, then update.
        img_width = 12'd960;
        img_height = 12'd720;
        wait_for_bcd(1'b0);
        wait_for_bcd(1'b1);
        if ((dut.res_width_bcd !== 16'h0960) ||
            (dut.res_height_bcd !== 16'h0720)) begin
            errors = errors + 1;
            $display("FAIL: 960x720 became %04h x %04h",
                     dut.res_width_bcd, dut.res_height_bcd);
        end

        count_resolution_pixels(cyan_pixels);
        if (cyan_pixels < 20) begin
            errors = errors + 1;
            $display("FAIL: 960x720 text is missing (%0d pixels)", cyan_pixels);
        end
        $display("RES OSD TB: 960x720 -> %04h x %04h, %0d sampled pixels",
                 dut.res_width_bcd, dut.res_height_bcd, cyan_pixels);

        // Sampling outside the panel must stay background-only.
        px = TOP_X1;
        py = 10'd40;
        #1;
        if (hit) begin
            errors = errors + 1;
            $display("FAIL: overlay painted past the top-left panel");
        end

        if (errors == 0) $display("RES OSD TB: PASS");
        else             $display("RES OSD TB: FAIL with %0d error(s)", errors);
        $finish;
    end

endmodule
