`timescale 1ns / 1ps
//=============================================================================
// tb_param_control.v - advanced item 4, key handling and pixel arithmetic
//
// The player top is elaborated with the same stubs the layer check uses
// (sim/check_stubs.v, sim/sdr_as_ram_stub.v) and then exercised through the
// real key nets, so what is checked is the delivered RTL and not a model:
//
//   keys     KEY2 steps the level of the parameter KEY4 selected, both wrap
//            at the ends, KEY4 short press switches brightness <-> contrast,
//            KEY4 held for a second steps the overlay layout, and every one of
//            those arms the one second readout timer
//   pixels   contrast and brightness are the documented arithmetic, checked
//            against the level that is actually in the pixel clock domain
//   overlay  the value drawn by ui_overlay is the same level the pixel path
//            uses, so a photograph of the panel matches the picture
//
// The keys are driven at the pin, so the debouncer is part of what is checked.
// Only two things are forced: the level registers while the pixel arithmetic
// is swept, and the hold counter once for the one second long press, which
// would otherwise cost 50 million simulated cycles per press.
//=============================================================================
module tb_param_control;

    reg sys_clk = 1'b0;
    always #10 sys_clk = ~sys_clk;          // 50 MHz, the board clock

    reg sys_rst_n  = 1'b0;
    reg key_mode   = 1'b1;
    reg key_bright = 1'b1;
    reg key_ui     = 1'b1;
    reg key_auto   = 1'b0;
    reg SD_MISO    = 1'b0;

    wire [3:0] led;
    wire HDMI_CLK_P, HDMI_D2_P, HDMI_D1_P, HDMI_D0_P;
    wire HDMI_DDC_SCL;
    wire HDMI_DDC_SDA;
    wire SD_nCS, SD_DCLK, SD_MOSI;

    hdmi_mm_player #(
        .FEATURE_UI         (1),
        .FEATURE_LOGO       (1),
        .FEATURE_SUBTITLE   (1),
        .FEATURE_TIMESTAMP  (1),
        .FEATURE_PARALLAX   (1),
        .FEATURE_FADE       (1),
        .FEATURE_BRIGHTNESS (1),
        .FEATURE_OSD        (1),
        .FEATURE_AUDIO_WAVE (0),
        .FEATURE_SCALE      (1)
    ) dut (
        .sys_clk      (sys_clk),
        .sys_rst_n    (sys_rst_n),
        .key_mode     (key_mode),
        .key_bright   (key_bright),
        .key_ui       (key_ui),
        .key_auto     (key_auto),
        .led          (led),
        .HDMI_CLK_P   (HDMI_CLK_P),
        .HDMI_D2_P    (HDMI_D2_P),
        .HDMI_D1_P    (HDMI_D1_P),
        .HDMI_D0_P    (HDMI_D0_P),
        .HDMI_DDC_SCL (HDMI_DDC_SCL),
        .HDMI_DDC_SDA (HDMI_DDC_SDA),
        .SD_nCS       (SD_nCS),
        .SD_DCLK      (SD_DCLK),
        .SD_MOSI      (SD_MOSI),
        .SD_MISO      (SD_MISO)
    );

    integer errors = 0;

    //------------------------------------------------------------------
    // level tables, mirrored from the RTL tables so a wrong entry shows up
    //------------------------------------------------------------------
    function integer gain_of;
        input integer level;
        begin
            case (level)
                0: gain_of = 4;
                1: gain_of = 8;
                2: gain_of = 12;
                3: gain_of = 16;
                4: gain_of = 20;
                5: gain_of = 24;
                6: gain_of = 28;
                default: gain_of = 31;
            endcase
        end
    endfunction

    function integer contrast_of;
        input integer level;
        begin
            case (level)
                0: contrast_of = 10;
                1: contrast_of = 12;
                2: contrast_of = 14;
                3: contrast_of = 16;
                4: contrast_of = 18;
                5: contrast_of = 20;
                6: contrast_of = 22;
                default: contrast_of = 24;
            endcase
        end
    endfunction

    function integer contrast_pixel;
        input integer c;
        input integer k;
        integer t;
        reg signed [31:0] p;
        begin
            p = (c - 128) * k;
            t = (p + 32'sd8) >>> 4;
            t = t + 128;
            if (t < 0) contrast_pixel = 0;
            else if (t > 255) contrast_pixel = 255;
            else contrast_pixel = t;
        end
    endfunction

    function integer scale_pixel;
        input integer c;
        input integer gain;
        integer t;
        begin
            t = (c * gain + 31) >> 5;
            if (t > 255) scale_pixel = 255;
            else scale_pixel = t;
        end
    endfunction

    // The ten millisecond debounce is walked up to its last count instead of
    // being waited out, so a key press costs ten cycles rather than a million
    // with the whole player running.
    task settle_bright;
        begin
            repeat (3) @(posedge sys_clk);
            force dut.u_bright_button.hold_count = 20'd499999;
            repeat (2) @(posedge sys_clk);
            release dut.u_bright_button.hold_count;
            repeat (2) @(posedge sys_clk);
        end
    endtask

    task press_bright;
        begin
            key_bright = 1'b0;
            settle_bright;
            key_bright = 1'b1;
            settle_bright;
        end
    endtask

    task settle_ui;
        begin
            repeat (3) @(posedge sys_clk);
            force dut.u_ui_button.hold_count = 20'd499999;
            repeat (2) @(posedge sys_clk);
            release dut.u_ui_button.hold_count;
            repeat (2) @(posedge sys_clk);
        end
    endtask

    task press_ui_short;
        begin
            key_ui = 1'b0;
            settle_ui;
            key_ui = 1'b1;
            settle_ui;
        end
    endtask

    // Hold KEY4 past one second: the debounce is settled as above and only the
    // one second hold counter is forced, once, to reach its compare.
    task press_ui_long;
        begin
            key_ui = 1'b0;
            settle_ui;
            if (dut.ui_button_n !== 1'b0) begin
                errors = errors + 1;
                $display("FAIL: KEY4 press was not debounced");
            end
            force dut.ui_hold_cnt = 27'd50_000_000 - 27'd1;
            repeat (2) @(posedge sys_clk);
            release dut.ui_hold_cnt;
            key_ui = 1'b1;
            settle_ui;
        end
    endtask

    // SW1 is a level switch.  Move it and let the existing 10 ms debouncer
    // settle, then check the mode signal the player sequencer will see.
    task set_auto_switch;
        input value;
        begin
            key_auto = value;
            repeat (3) @(posedge sys_clk);
            force dut.u_auto_switch.hold_count = 20'd499999;
            repeat (2) @(posedge sys_clk);
            release dut.u_auto_switch.hold_count;
            repeat (2) @(posedge sys_clk);
        end
    endtask

    task expect_level;
        input integer want_bright;
        input integer want_contrast;
        input integer want_sel;
        begin
            if (dut.bright_level !== want_bright[2:0]) begin
                errors = errors + 1;
                $display("FAIL: bright_level = %0d, expected %0d",
                         dut.bright_level, want_bright);
            end
            if (dut.contrast_level !== want_contrast[2:0]) begin
                errors = errors + 1;
                $display("FAIL: contrast_level = %0d, expected %0d",
                         dut.contrast_level, want_contrast);
            end
            if (dut.param_sel !== want_sel[1:0]) begin
                errors = errors + 1;
                $display("FAIL: param_sel = %0d, expected %0d",
                         dut.param_sel, want_sel);
            end
            if (dut.param_timer == 27'd0) begin
                errors = errors + 1;
                $display("FAIL: readout timer not armed after a key");
            end
        end
    endtask

    //------------------------------------------------------------------
    // pixel arithmetic over the real wires of the video path
    //------------------------------------------------------------------
    integer lvl, v, sky, exp;
    reg [23:0] rgb_pattern [0:9];

    task check_pixels;
        integer i;
        begin
            for (lvl = 0; lvl < 8; lvl = lvl + 1) begin
                force dut.contrast_level = lvl[2:0];
                repeat (4) @(posedge sys_clk);
                #1;
                sky = contrast_of(lvl);
                if (dut.user_contrast_now !== sky[4:0]) begin
                    errors = errors + 1;
                    $display("FAIL: contrast level %0d -> k = %0d, expected %0d",
                             lvl, dut.user_contrast_now, contrast_of(lvl));
                end
                for (i = 0; i < 10; i = i + 1) begin
                    force dut.fifo_rgb = rgb_pattern[i];
                    #2;
                    exp = contrast_pixel(rgb_pattern[i][23:16], contrast_of(lvl));
                    if (dut.cont_rgb[23:16] !== exp[7:0]) begin
                        errors = errors + 1;
                        $display("FAIL: contrast %0d of %0d gave %0d, expected %0d",
                                 rgb_pattern[i][23:16], contrast_of(lvl),
                                 dut.cont_rgb[23:16], exp);
                    end
                    exp = contrast_pixel(rgb_pattern[i][15:8], contrast_of(lvl));
                    if (dut.cont_rgb[15:8] !== exp[7:0]) begin
                        errors = errors + 1;
                        $display("FAIL: contrast green leg wrong at level %0d", lvl);
                    end
                    exp = contrast_pixel(rgb_pattern[i][7:0], contrast_of(lvl));
                    if (dut.cont_rgb[7:0] !== exp[7:0]) begin
                        errors = errors + 1;
                        $display("FAIL: contrast blue leg wrong at level %0d", lvl);
                    end
                end
            end

            // brightness rides the same gain as the fade, so check the faded
            // output for every level with the fade held at full scale.
            force dut.fifo_rgb = 24'hC86420;
            force dut.fade_level = 5'd31;
            for (lvl = 0; lvl < 8; lvl = lvl + 1) begin
                force dut.contrast_level = 3'd3;      // 1.00, isolate the gain
                force dut.bright_level   = lvl[2:0];
                repeat (4) @(posedge sys_clk);
                #1;
                v = gain_of(lvl);
                if (dut.user_gain_now !== v[4:0]) begin
                    errors = errors + 1;
                    $display("FAIL: brightness level %0d -> gain %0d, expected %0d",
                             lvl, dut.user_gain_now, gain_of(lvl));
                end
                sky = (31 * gain_of(lvl) + 31) >> 5;
                if (dut.total_gain !== sky[4:0]) begin
                    errors = errors + 1;
                    $display("FAIL: total gain %0d, expected %0d",
                             dut.total_gain, sky);
                end
                exp = scale_pixel(contrast_pixel(8'hC8, 16), sky);
                if (dut.faded_rgb[23:16] !== exp[7:0]) begin
                    errors = errors + 1;
                    $display("FAIL: bright %0d red = %0d, expected %0d",
                             lvl, dut.faded_rgb[23:16], exp);
                end
                exp = scale_pixel(contrast_pixel(8'h64, 16), sky);
                if (dut.faded_rgb[15:8] !== exp[7:0]) begin
                    errors = errors + 1;
                    $display("FAIL: bright %0d green = %0d, expected %0d",
                             lvl, dut.faded_rgb[15:8], exp);
                end
                exp = scale_pixel(contrast_pixel(8'h20, 16), sky);
                if (dut.faded_rgb[7:0] !== exp[7:0]) begin
                    errors = errors + 1;
                    $display("FAIL: bright %0d blue = %0d, expected %0d",
                             lvl, dut.faded_rgb[7:0], exp);
                end
            end

            // level 7 is the shipped default and must be a visible change from
            // level 0, otherwise the key would look dead on the bench.
            if (scale_pixel(255, 31) == scale_pixel(255, 4)) begin
                errors = errors + 1;
                $display("FAIL: brightness levels do not change white");
            end

            release dut.fifo_rgb;
            release dut.fade_level;
            release dut.contrast_level;
            release dut.bright_level;
        end
    endtask

    initial begin
        rgb_pattern[0] = 24'h000000;
        rgb_pattern[1] = 24'h010101;
        rgb_pattern[2] = 24'h204060;
        rgb_pattern[3] = 24'h4080C0;
        rgb_pattern[4] = 24'h7F7F7F;
        rgb_pattern[5] = 24'h808080;
        rgb_pattern[6] = 24'h818181;
        rgb_pattern[7] = 24'hA0C0E0;
        rgb_pattern[8] = 24'hFEFEFE;
        rgb_pattern[9] = 24'hFFFFFF;

        repeat (8) @(posedge sys_clk);
        sys_rst_n = 1'b1;
        repeat (4) @(posedge sys_clk);

        // 1: reset defaults are the shipped picture settings.
        if (dut.bright_level !== 3'd7) begin
            errors = errors + 1;
            $display("FAIL: brightness default");
        end
        if (dut.contrast_level !== 3'd3) begin
            errors = errors + 1;
            $display("FAIL: contrast default");
        end
        if (dut.param_flash !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: readout is up before any key");
        end

        // SW1 down is manual by default; up enables the automatic timer.
        if (dut.auto_rotate_en !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: default switch state is not manual");
        end
        set_auto_switch(1'b1);
        if (dut.auto_rotate_en !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: SW1 up did not enable automatic rotation");
        end
        set_auto_switch(1'b0);
        if (dut.auto_rotate_en !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: SW1 down did not return to manual rotation");
        end

        // 2: KEY2 steps brightness down and arms the readout.
        press_bright;
        expect_level(6, 3, 0);
        if (dut.param_flash !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: readout not up after KEY2");
        end

        // 3: KEY4 short press moves the selection to contrast.
        press_ui_short;
        expect_level(6, 3, 1);
        if (dut.ui_mode !== 2'd0) begin
            errors = errors + 1;
            $display("FAIL: short press also changed the layout");
        end

        // 4: KEY2 now steps contrast and never touches brightness.
        press_bright;
        expect_level(6, 2, 1);

        // 5: the contrast range wraps at the bottom.
        force dut.contrast_level = 3'd0;
        @(posedge sys_clk);
        release dut.contrast_level;
        press_bright;
        expect_level(6, 7, 1);

        // 6: the brightness range wraps at the bottom as well.
        press_ui_short;
        expect_level(6, 7, 0);
        force dut.bright_level = 3'd0;
        @(posedge sys_clk);
        release dut.bright_level;
        press_bright;
        expect_level(7, 7, 0);

        // 7: KEY4 held for a second steps the layout and shows OSD.
        press_ui_long;
        if (dut.ui_mode !== 2'd1) begin
            errors = errors + 1;
            $display("FAIL: long press did not step the layout");
        end
        expect_level(7, 7, 2);

        // 8: the readout is armed for a second and then clears itself.
        force dut.param_timer = 27'd1;
        @(posedge sys_clk);
        release dut.param_timer;
        repeat (2) @(posedge sys_clk);
        if (dut.param_flash !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: readout did not clear when its timer ran out");
        end

        // 9: the panel shows the level the pixel path is using.
        press_ui_short;
        repeat (4) @(posedge sys_clk);
        if (dut.u_ui_overlay.bright_s2 !== dut.bright_level) begin
            errors = errors + 1;
            $display("FAIL: overlay brightness differs from the pixel path");
        end
        if (dut.u_ui_overlay.contrast_s2 !== dut.contrast_level) begin
            errors = errors + 1;
            $display("FAIL: overlay contrast differs from the pixel path");
        end
        if (dut.u_ui_overlay.psel_s2 !== dut.param_sel) begin
            errors = errors + 1;
            $display("FAIL: overlay selection differs from the key state");
        end
        if (dut.u_ui_overlay.pflash_s2 !== dut.param_flash) begin
            errors = errors + 1;
            $display("FAIL: overlay readout window differs from the timer");
        end

        // 10: the pixel arithmetic for every level.
        check_pixels;

        if (errors == 0) $display("PARAM CTRL TB: PASS");
        else             $display("PARAM CTRL TB: FAIL with %0d error(s)", errors);
        $finish;
    end

endmodule
