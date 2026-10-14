`timescale 1ns / 1ps
//=============================================================================
// tb_ui_overlay_dump.v - dump one overlay frame as raw RGB
//
// The overlay is driven with a white background so the overlay pixels can be
// classified afterwards: the fixed panels are 0x808080, 0x00E0FF is the logo
// text, 0xFFD060 is the airport clock, 0x7FE7FF is the parallax ticker and
// 0xFFFFFF is the subtitle text. The subtitle background is a row-dependent
// alpha ramp. The preview script composes the dump on top of a real 640x480
// BMP and also builds a short animation that shows the two ticker speeds.
//=============================================================================
module tb_ui_overlay_dump;

    reg        pixel_clk = 1'b0;
    always #20 pixel_clk = ~pixel_clk;

    reg        rst = 1'b1;
    reg        frame_tick = 1'b0;
    reg        de = 1'b1;
    reg [9:0]  px = 10'd0;
    reg [9:0]  py = 10'd0;

    reg        ui_enable = 1'b1;
    reg [2:0]  img_slot = 3'd2;
    reg [7:0]  img_count = 8'd6;
    reg [2:0]  user_bright = 3'd7;
    reg [1:0]  ui_mode = 2'd0;
    reg        param_flash = 1'b0;
    reg [1:0]  param_sel = 2'd0;
    reg [2:0]  user_contrast = 3'd3;
    reg        logo_enable = 1'b1;
    reg        subtitle_enable = 1'b1;
    reg        timestamp_enable = 1'b1;
    reg        parallax_enable = 1'b1;
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
        .base_rgb        (24'hFFFFFF),
        .ui_enable       (ui_enable),
        .img_slot        (img_slot),
        .img_count       (img_count),
        .user_bright     (user_bright),
        .ui_mode         (ui_mode),
        .param_flash     (param_flash),
        .param_sel       (param_sel),
        .auto_mode       (1'b0),
        .user_contrast   (user_contrast),
        .logo_enable     (logo_enable),
        .subtitle_enable (subtitle_enable),
        .timestamp_enable (timestamp_enable),
        .parallax_enable (parallax_enable),
        .osd_enable      (osd_enable),
        .wave_enable     (wave_enable),
        .audio_rst       (audio_rst),
        .audio_valid     (audio_valid),
        .audio_left      (audio_left),
        .rgb             (rgb),
        .hit             (hit)
    );

    integer fd;
    integer i, j, f;
    reg [8*256-1:0] filename;

    task tick_frame;
        begin
            @(negedge pixel_clk);
            frame_tick = 1'b1;
            @(negedge pixel_clk);
            frame_tick = 1'b0;
        end
    endtask

    initial begin
        rst = 1'b1;
        repeat (4) @(posedge pixel_clk);
        rst = 1'b0;
        repeat (4) @(posedge pixel_clk);

        for (f = 0; f < 16; f = f + 1) begin
            $sformat(filename,
                     "sim/build_adv01/adv01_overlay_frame_%02d.raw", f);
            fd = $fopen(filename, "wb");
            if (fd == 0) begin
                $display("ADV01 DUMP: cannot open output file %s", filename);
                $finish;
            end

            for (j = 0; j < 480; j = j + 1) begin
                for (i = 0; i < 640; i = i + 1) begin
                    px = i[9:0];
                    py = j[9:0];
                    #1;
                    $fwrite(fd, "%c%c%c", rgb[23:16], rgb[15:8], rgb[7:0]);
                end
            end

            $fclose(fd);
            tick_frame;
        end

        $display("ADV01 DUMP: 16 frames written");
        $finish;
    end

endmodule
