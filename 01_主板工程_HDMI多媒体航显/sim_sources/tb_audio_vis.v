`timescale 1ns / 1ps
//=============================================================================
// tb_audio_vis.v - advanced item 5 audio visualisation check
//
// The overlay is fed with two sine waves through the same 48 kHz
// audio_valid interface used by the HDMI I2S receiver.  A 300 Hz tone should
// concentrate energy in the low bands, while a 12 kHz tone should concentrate
// it in the top band.  The bench also scans the lower-right panel to verify
// that the waveform and the eight spectrum bars actually reach the RGB output.
//=============================================================================
module tb_audio_vis;

    localparam integer PANEL_W = 256;
    localparam integer PANEL_H = 128;
    localparam [9:0] WAVE_X0 = 10'd384;
    localparam [9:0] WAVE_Y0 = 10'd320;
    localparam [9:0] WAVE_Y1 = 10'd447;
    localparam [9:0] SPEC_Y0 = 10'd384;

    reg        pixel_clk = 1'b0;
    always #20 pixel_clk = ~pixel_clk;          // 25 MHz pixel clock

    reg        rst = 1'b1;
    reg        frame_tick = 1'b0;
    reg        de = 1'b1;
    reg [9:0]  px = 10'd0;
    reg [9:0]  py = 10'd0;
    reg [23:0] base_rgb = 24'h202830;

    reg        ui_enable = 1'b1;
    reg [2:0]  img_slot = 3'd0;
    reg [7:0]  img_count = 8'd6;
    reg [2:0]  user_bright = 3'd7;
    reg [2:0]  user_contrast = 3'd3;
    reg [1:0]  ui_mode = 2'd0;
    reg        param_flash = 1'b0;
    reg [1:0]  param_sel = 2'd0;
    reg        logo_enable = 1'b0;
    reg        subtitle_enable = 1'b0;
    reg        timestamp_enable = 1'b0;
    reg        parallax_enable = 1'b0;
    reg        osd_enable = 1'b0;
    reg        wave_enable = 1'b1;
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
        .user_bright     (user_bright),
        .user_contrast   (user_contrast),
        .ui_mode         (ui_mode),
        .param_flash     (param_flash),
        .param_sel       (param_sel),
        .auto_mode       (1'b0),
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

    integer errors = 0;
    integer n, i, j, bin, bx, by;
    integer panel_hits, wave_hits, spec_hits;
    integer bar_count [0:7];
    integer expected_count;

    reg [5:0] low_band [0:7];
    reg [5:0] high_band [0:7];

    task send_sample;
        input [11:0] sample12;
        begin
            @(negedge pixel_clk);
            audio_left = {sample12, 12'h000};
            audio_valid = 1'b1;
            @(negedge pixel_clk);
            audio_valid = 1'b0;
            repeat (519) @(negedge pixel_clk);
        end
    endtask

    task drive_sine;
        input real frequency;
        input integer total_samples;
        integer k;
        real phase;
        real phase_step;
        integer sample_value;
        begin
            phase = 0.0;
            phase_step = 6.28318530718 * frequency / 48000.0;
            for (k = 0; k < total_samples; k = k + 1) begin
                sample_value = $rtoi(1200.0 * $sin(phase));
                send_sample({sample_value[11:0]});
                phase = phase + phase_step;
            end
        end
    endtask

    task tick_frame;
        begin
            @(negedge pixel_clk);
            frame_tick = 1'b1;
            @(negedge pixel_clk);
            frame_tick = 1'b0;
        end
    endtask

    task reset_audio;
        begin
            audio_rst = 1'b1;
            repeat (4) @(posedge pixel_clk);
            audio_rst = 1'b0;
            repeat (2) @(posedge pixel_clk);
        end
    endtask

    task snapshot_low;
        begin
            for (i = 0; i < 8; i = i + 1)
                low_band[i] = dut.band_level[i];
        end
    endtask

    task snapshot_high;
        begin
            for (i = 0; i < 8; i = i + 1)
                high_band[i] = dut.band_level[i];
        end
    endtask

    task scan_panel;
        begin
            panel_hits = 0;
            wave_hits = 0;
            spec_hits = 0;
            for (i = 0; i < 8; i = i + 1)
                bar_count[i] = 0;

            for (by = WAVE_Y0; by <= WAVE_Y1; by = by + 1) begin
                for (bx = 0; bx < PANEL_W; bx = bx + 1) begin
                    px = WAVE_X0 + bx;
                    py = by;
                    #1;
                    if (hit) begin
                        panel_hits = panel_hits + 1;
                        if ((by <= 10'd383) && (rgb == 24'h00FFFF))
                            wave_hits = wave_hits + 1;
                        if (by >= SPEC_Y0) begin
                            if ((rgb == 24'h00FF80) ||
                                (rgb == 24'h00E0FF) ||
                                (rgb == 24'hFFB020)) begin
                                spec_hits = spec_hits + 1;
                                bar_count[bx / 32] = bar_count[bx / 32] + 1;
                            end
                        end
                    end
                end
            end
        end
    endtask

    task dump_panel;
        input [8*64-1:0] file_name;
        integer fd;
        integer x, y;
        begin
            fd = $fopen(file_name, "wb");
            if (fd == 0) begin
                $display("FAIL: could not open %0s", file_name);
                errors = errors + 1;
            end else begin
                $fwrite(fd, "P6\n256 128\n255\n");
                for (y = WAVE_Y0; y <= WAVE_Y1; y = y + 1) begin
                    for (x = 0; x < PANEL_W; x = x + 1) begin
                        px = WAVE_X0 + x;
                        py = y;
                        #1;
                        $fwrite(fd, "%c%c%c",
                                rgb[23:16], rgb[15:8], rgb[7:0]);
                    end
                end
                $fclose(fd);
            end
        end
    endtask

    initial begin
        repeat (4) @(posedge pixel_clk);
        rst = 1'b0;
        repeat (4) @(posedge pixel_clk);
        reset_audio;

        // A 300 Hz sine wave concentrates energy in the lower bands.
        drive_sine(300.0, 1600);
        tick_frame;
        snapshot_low;
        $display("VIS TB: 300 Hz bands = %0d %0d %0d %0d %0d %0d %0d %0d",
                 low_band[0], low_band[1], low_band[2], low_band[3],
                 low_band[4], low_band[5], low_band[6], low_band[7]);
        if (low_band[2] < 6'd8) begin
            errors = errors + 1;
            $display("FAIL: 300 Hz energy did not reach the low spectrum bands");
        end

        reset_audio;

        // A 12 kHz sine wave is near the top of the 48 kHz audio band.
        drive_sine(12000.0, 1600);
        tick_frame;
        snapshot_high;
        $display("VIS TB: 12 kHz bands = %0d %0d %0d %0d %0d %0d %0d %0d",
                 high_band[0], high_band[1], high_band[2], high_band[3],
                 high_band[4], high_band[5], high_band[6], high_band[7]);
        if (high_band[7] < 6'd32) begin
            errors = errors + 1;
            $display("FAIL: 12 kHz energy did not reach the top spectrum band");
        end
        if (high_band[7] <= low_band[7]) begin
            errors = errors + 1;
            $display("FAIL: high tone did not separate from the low tone");
        end

        // The waveform and all eight bars must be visible in OSD FULL.
        scan_panel;
        $display("VIS TB: panel hits = %0d, waveform pixels = %0d, spectrum pixels = %0d",
                 panel_hits, wave_hits, spec_hits);
        $display("VIS TB: bar pixels = %0d %0d %0d %0d %0d %0d %0d %0d",
                 bar_count[0], bar_count[1], bar_count[2], bar_count[3],
                 bar_count[4], bar_count[5], bar_count[6], bar_count[7]);
        if (panel_hits != (PANEL_W * PANEL_H)) begin
            errors = errors + 1;
            $display("FAIL: audio panel did not cover its full rectangle");
        end
        if (wave_hits == 0) begin
            errors = errors + 1;
            $display("FAIL: waveform never painted");
        end
        if (spec_hits == 0) begin
            errors = errors + 1;
            $display("FAIL: spectrum never painted");
        end
        for (bin = 0; bin < 8; bin = bin + 1) begin
            if (high_band[bin] == 6'd0)
                expected_count = 0;
            else
                expected_count = (high_band[bin] + 6'd1) * 28;
            if (bar_count[bin] != expected_count) begin
                errors = errors + 1;
                $display("FAIL: bar %0d painted %0d pixels, expected %0d",
                         bin, bar_count[bin], expected_count);
            end
        end

        dump_panel("sim/build_adv08/audio_vis.ppm");

        // OSD INFO/ON/OFF and the feature switch must suppress the panel.
        ui_mode = 2'd1;
        repeat (4) @(posedge pixel_clk);
        scan_panel;
        if (panel_hits != 0) begin
            errors = errors + 1;
            $display("FAIL: audio panel painted in OSD INFO");
        end

        ui_mode = 2'd3;
        repeat (4) @(posedge pixel_clk);
        scan_panel;
        if (panel_hits != 0) begin
            errors = errors + 1;
            $display("FAIL: audio panel painted in OSD OFF");
        end

        ui_mode = 2'd0;
        wave_enable = 1'b0;
        repeat (4) @(posedge pixel_clk);
        scan_panel;
        if (panel_hits != 0) begin
            errors = errors + 1;
            $display("FAIL: audio panel painted with the feature disabled");
        end

        if (errors == 0)
            $display("AUDIO VIS TB: PASS");
        else
            $display("AUDIO VIS TB: FAIL with %0d error(s)", errors);
        $finish;
    end

endmodule
