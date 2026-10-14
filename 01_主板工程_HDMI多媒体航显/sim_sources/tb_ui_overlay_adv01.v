`timescale 1ns / 1ps
//=============================================================================
// tb_ui_overlay_adv01.v - ADV01 overlay checks
//
// The overlay is purely combinational apart from the input synchronisers, so
// the testbench walks px/py with a small delay instead of clocking a whole
// 640x480 frame.  It checks:
//
//   1. the logo panel, airport time panel and subtitle band actually paint,
//   2. the logo text is one line centred in its panel instead of a clipped
//      32 pixel tall font,
//   3. the subtitle is a single 16 pixel tall line, not a doubled copy,
//   4. the subtitle background has a symmetrical vertical alpha gradient,
//   5. a host TIME message updates the HH:MM:SS readout,
//   6. the slow parallax ticker moves every two frames while the main ticker
//      moves every frame,
//   7. nothing is drawn outside the overlay panels while the OSD and the
//      audio waveform are disabled,
//   8. one frame_tick shifts the subtitle left by exactly one pixel.
//=============================================================================
module tb_ui_overlay_adv01;

    localparam [9:0] TOP_X0  = 10'd12;
    localparam [9:0] TOP_Y0  = 10'd12;
    localparam [9:0] TOP_X1  = 10'd236;
    localparam [9:0] TOP_Y1  = 10'd60;
    localparam [9:0] TIME_X0 = 10'd456;
    localparam [9:0] TIME_Y0 = 10'd12;
    localparam [9:0] TIME_X1 = 10'd624;
    localparam [9:0] TIME_Y1 = 10'd44;
    localparam [9:0] TICK_Y0 = 10'd432;
    localparam [9:0] TICK_Y1 = 10'd440;
    localparam [9:0] SUB_Y0  = 10'd448;
    localparam [9:0] SUB_Y1  = 10'd480;

    reg        pixel_clk = 1'b0;
    always #20 pixel_clk = ~pixel_clk;          // 25 MHz pixel clock

    reg        rst = 1'b1;
    reg        frame_tick = 1'b0;
    reg        de = 1'b1;
    reg [9:0]  px = 10'd0;
    reg [9:0]  py = 10'd0;
    reg [23:0] base_rgb = 24'h808080;

    reg        ui_enable = 1'b1;
    reg [2:0]  img_slot = 3'd2;
    reg [7:0]  img_count = 8'd6;
    reg [11:0] img_width = 12'd0;
    reg [11:0] img_height = 12'd0;
    reg        img_size_valid = 1'b0;
    reg [2:0]  user_bright = 3'd7;
    reg [2:0]  user_contrast = 3'd3;
    reg [1:0]  ui_mode = 2'd0;
    reg        param_flash = 1'b0;
    reg [1:0]  param_sel = 2'd0;
    reg        logo_enable = 1'b1;
    reg        subtitle_enable = 1'b1;
    reg        timestamp_enable = 1'b1;
    reg        parallax_enable = 1'b1;
    reg        osd_enable = 1'b0;
    reg        wave_enable = 1'b0;
    reg [7:0]  host_time_hh = 8'h00;
    reg [7:0]  host_time_mm = 8'h00;
    reg [7:0]  host_time_ss = 8'h00;
    reg        host_time_tog = 1'b0;
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
        .auto_mode       (1'b0),
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
        .host_time_hh    (host_time_hh),
        .host_time_mm    (host_time_mm),
        .host_time_ss    (host_time_ss),
        .host_time_tog   (host_time_tog),
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
    integer top_hits, top_cyan, time_hits, time_amber;
    integer tick_hits, tick_cyan, sub_hits, sub_white, stray_hits;
    integer top_min_y, top_max_y, time_min_y, time_max_y;
    integer tick_min_y, tick_max_y, sub_min_y, sub_max_y;
    integer res_cyan;
    integer band_gray [0:31];
    integer distinct_alpha, found_alpha;
    reg [639:0] sig_before, sig_after;
    reg [639:0] tick_sig_before, tick_sig_mid, tick_sig_after;
    reg [167:0] time_sig_before, time_sig_after;

    task scan_frame;
        input integer full;
        begin
            top_hits = 0;  top_cyan = 0;
            time_hits = 0; time_amber = 0;
            tick_hits = 0; tick_cyan = 0;
            sub_hits = 0;  sub_white = 0;
            stray_hits = 0;
            top_min_y = 9999; top_max_y = -1;
            time_min_y = 9999; time_max_y = -1;
            tick_min_y = 9999; tick_max_y = -1;
            sub_min_y = 9999; sub_max_y = -1;

            if (full) begin
                // Scan each OSD panel completely.  This keeps every
                // pixel-based assertion exact without walking the whole
                // screen, which is prohibitively slow in Icarus.
                for (j = TOP_Y0; j < TOP_Y1; j = j + 1) begin
                    for (i = TOP_X0; i < TOP_X1; i = i + 1) begin
                        px = i[9:0];
                        py = j[9:0];
                        #1;
                        if (hit) begin
                            top_hits = top_hits + 1;
                            if (rgb === 24'h00E0FF) begin
                                top_cyan = top_cyan + 1;
                                if (j < top_min_y) top_min_y = j;
                                if (j > top_max_y) top_max_y = j;
                            end
                        end
                    end
                end

                for (j = TIME_Y0; j < TIME_Y1; j = j + 1) begin
                    for (i = TIME_X0; i < TIME_X1; i = i + 1) begin
                        px = i[9:0];
                        py = j[9:0];
                        #1;
                        if (hit) begin
                            time_hits = time_hits + 1;
                            if (rgb === 24'hFFD060) begin
                                time_amber = time_amber + 1;
                                if (j < time_min_y) time_min_y = j;
                                if (j > time_max_y) time_max_y = j;
                            end
                        end
                    end
                end

                for (j = TICK_Y0; j < TICK_Y1; j = j + 1) begin
                    for (i = 0; i < 640; i = i + 1) begin
                        px = i[9:0];
                        py = j[9:0];
                        #1;
                        if (hit) begin
                            tick_hits = tick_hits + 1;
                            if (rgb === 24'h7FE7FF) begin
                                tick_cyan = tick_cyan + 1;
                                if (j < tick_min_y) tick_min_y = j;
                                if (j > tick_max_y) tick_max_y = j;
                            end
                        end
                    end
                end

                for (j = SUB_Y0; j < SUB_Y1; j = j + 1) begin
                    for (i = 0; i < 640; i = i + 1) begin
                        px = i[9:0];
                        py = j[9:0];
                        #1;
                        if (hit) begin
                            sub_hits = sub_hits + 1;
                            if (rgb === 24'hFFFFFF) begin
                                sub_white = sub_white + 1;
                                if (j < sub_min_y) sub_min_y = j;
                                if (j > sub_max_y) sub_max_y = j;
                            end
                        end
                    end
                end

                // Sparse sampling outside every panel is enough for the
                // "stray" assertion; exact checks above cover the panels.
                for (j = 0; j < 480; j = j + 37) begin
                    for (i = 0; i < 640; i = i + 53) begin
                        if (!((i >= TOP_X0 && i < TOP_X1 &&
                               j >= TOP_Y0 && j < TOP_Y1) ||
                              (i >= TIME_X0 && i < TIME_X1 &&
                               j >= TIME_Y0 && j < TIME_Y1) ||
                              (j >= TICK_Y0 && j < TICK_Y1) ||
                              (j >= SUB_Y0 && j < SUB_Y1))) begin
                            px = i[9:0];
                            py = j[9:0];
                            #1;
                            if (hit)
                                stray_hits = stray_hits + 1;
                        end
                    end
                end
            end else begin
                // Short checks after one or more layers are disabled: one
                // representative row through every panel is enough to prove
                // that the enable gates still work.
                for (i = TOP_X0; i < TOP_X1; i = i + 1) begin
                    px = i[9:0];
                    py = 10'd28;
                    #1;
                    if (hit) begin
                        top_hits = top_hits + 1;
                        if (rgb === 24'h00E0FF) top_cyan = top_cyan + 1;
                    end
                end
                for (i = TIME_X0; i < TIME_X1; i = i + 4) begin
                    px = i[9:0];
                    py = 10'd28;
                    #1;
                    if (hit) begin
                        time_hits = time_hits + 1;
                        if (rgb === 24'hFFD060) time_amber = time_amber + 1;
                    end
                end
                for (i = 0; i < 640; i = i + 8) begin
                    px = i[9:0];
                    py = 10'd436;
                    #1;
                    if (hit) begin
                        tick_hits = tick_hits + 1;
                        if (rgb === 24'h7FE7FF) tick_cyan = tick_cyan + 1;
                    end
                end
                for (i = 0; i < 640; i = i + 8) begin
                    px = i[9:0];
                    py = 10'd464;
                    #1;
                    if (hit) begin
                        sub_hits = sub_hits + 1;
                        if (rgb === 24'hFFFFFF) sub_white = sub_white + 1;
                    end
                end
                for (i = 0; i < 640; i = i + 53) begin
                    px = i[9:0];
                    py = 10'd300;
                    #1;
                    if (hit) stray_hits = stray_hits + 1;
                end
            end
        end
    endtask

    // Cyan pixels in the resolution row, i.e. the second text row of the
    // top-left panel (y = TOP_Y0 + 26 .. +34).
    task count_res_row;
        output integer cy;
        begin
            cy = 0;
            for (j = 38; j < 46; j = j + 1) begin
                for (i = 0; i < 240; i = i + 1) begin
                    px = i[9:0];
                    py = j[9:0];
                    #1;
                    if (hit && (rgb === 24'h00E0FF)) cy = cy + 1;
                end
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

    // Find one background (non-glyph) pixel per subtitle row.  The band is
    // always present, so this samples the alpha ramp without depending on
    // where the moving glyphs happen to be.
    task collect_gradient;
        begin
            for (j = 0; j < 32; j = j + 1) begin
                band_gray[j] = -1;
                for (i = 0; i < 64; i = i + 1) begin
                    px = i[9:0];
                    py = SUB_Y0 + j[9:0];
                    #1;
                    if (hit && (rgb !== 24'hFFFFFF)) begin
                        band_gray[j] = rgb[7:0];
                        i = 64;
                    end
                end
            end
        end
    endtask

    task collect_time_sig;
        output [167:0] sig;
        begin
            sig = 168'd0;
            for (i = 0; i < 168; i = i + 1) begin
                px = TIME_X0 + i[9:0];
                for (j = 24; j < 32; j = j + 1) begin
                    py = j[9:0];
                    #1;
                    if (rgb === 24'hFFD060)
                        sig[i] = 1'b1;
                end
            end
        end
    endtask

    // Column signature of the subtitle band, or of the whole band when the
    // glyph rows are the only source of white pixels.
    task collect_sig;
        output [639:0] sig;
        begin
            sig = 640'd0;
            for (i = 0; i < 640; i = i + 1) begin
                px = i[9:0];
                py = 10'd464;
                #1;
                if ((rgb === 24'hFFFFFF) && hit)
                    sig[i] = 1'b1;
            end
        end
    endtask

    task collect_tick_sig;
        output [639:0] sig;
        begin
            sig = 640'd0;
            for (i = 0; i < 640; i = i + 1) begin
                px = i[9:0];
                py = 10'd436;
                #1;
                if ((rgb === 24'h7FE7FF) && hit)
                    sig[i] = 1'b1;
            end
        end
    endtask

    initial begin
        rst = 1'b1;
        repeat (4) @(posedge pixel_clk);
        rst = 1'b0;
        repeat (4) @(posedge pixel_clk);

        // ---- 1. normal operation: logo + time + subtitle ----------------
        scan_frame(1);
        $display("ADV01 TB: top panel hits = %0d, logo pixels = %0d, y = %0d..%0d",
                 top_hits, top_cyan, top_min_y, top_max_y);
        $display("ADV01 TB: time panel hits = %0d, digits = %0d, y = %0d..%0d",
                 time_hits, time_amber, time_min_y, time_max_y);
        $display("ADV01 TB: parallax band hits = %0d, glyph pixels = %0d, y = %0d..%0d",
                 tick_hits, tick_cyan, tick_min_y, tick_max_y);
        $display("ADV01 TB: subtitle hits = %0d, glyph pixels = %0d, y = %0d..%0d",
                 sub_hits, sub_white, sub_min_y, sub_max_y);
        $display("ADV01 TB: stray hits outside the panels = %0d", stray_hits);

        if (top_hits != 10752)  begin errors = errors + 1; $display("FAIL: logo panel is not 224x48"); end
        if (top_cyan == 0)      begin errors = errors + 1; $display("FAIL: logo text never drawn"); end
        if (top_min_y < 20)     begin errors = errors + 1; $display("FAIL: logo text above its centred row"); end
        if (top_max_y > 35)     begin errors = errors + 1; $display("FAIL: logo text taller than a 16 pixel glyph"); end
        if (time_hits != 5376)  begin errors = errors + 1; $display("FAIL: time panel is not 168x32"); end
        if (time_amber == 0)    begin errors = errors + 1; $display("FAIL: airport time text never drawn"); end
        if (time_min_y < 20)    begin errors = errors + 1; $display("FAIL: time text above its centred row"); end
        if (time_max_y > 35)    begin errors = errors + 1; $display("FAIL: time text taller than a 16 pixel glyph"); end
        if (tick_hits != 5120)  begin errors = errors + 1; $display("FAIL: parallax band is not 640x8"); end
        if (tick_cyan == 0)     begin errors = errors + 1; $display("FAIL: parallax text never drawn"); end
        if (tick_min_y < 432)   begin errors = errors + 1; $display("FAIL: parallax text above its band"); end
        if (tick_max_y > 439)   begin errors = errors + 1; $display("FAIL: parallax text below its band"); end
        if (sub_hits != 20480)  begin errors = errors + 1; $display("FAIL: subtitle band is not 640x32"); end
        if (sub_white == 0)     begin errors = errors + 1; $display("FAIL: subtitle text never drawn"); end
        if (sub_min_y < 456)    begin errors = errors + 1; $display("FAIL: subtitle drawn above the centred row"); end
        if (sub_max_y > 471)    begin errors = errors + 1; $display("FAIL: subtitle glyph doubled vertically"); end
        if (stray_hits != 0)    begin errors = errors + 1; $display("FAIL: overlay painted outside its panels"); end

        // ---- 2. one frame tick shifts the text left by one pixel --------
        collect_sig(sig_before);
        tick_frame;
        collect_sig(sig_after);

        if (sig_before[0 +: 380] == 380'd0) begin
            errors = errors + 1;
            $display("FAIL: subtitle signature is empty, cannot verify scrolling");
        end else begin
            for (i = 0; i < 380; i = i + 1) begin
                if (sig_after[i] !== sig_before[i + 1]) begin
                    errors = errors + 1;
                    $display("FAIL: subtitle did not shift at column %0d", i);
                    i = 380;
                end
            end
        end
        if (sig_before === sig_after) begin
            errors = errors + 1;
            $display("FAIL: subtitle did not move after a frame tick");
        end

        // The parallax layer must advance once every two frame ticks.
        collect_tick_sig(tick_sig_before);
        tick_frame;
        collect_tick_sig(tick_sig_mid);
        if ((tick_sig_mid === tick_sig_before) &&
            (tick_sig_after === tick_sig_mid)) begin
            errors = errors + 1;
            $display("FAIL: parallax layer did not move after two frame ticks");
        end
        tick_frame;
        collect_tick_sig(tick_sig_after);
        if ((tick_sig_mid !== tick_sig_before) &&
            (tick_sig_after !== tick_sig_mid)) begin
            errors = errors + 1;
            $display("FAIL: parallax layer moved on both frame ticks");
        end
        if (tick_sig_before[0 +: 380] == 380'd0) begin
            errors = errors + 1;
            $display("FAIL: parallax signature is empty, cannot verify speed");
        end else begin
            for (i = 0; i < 380; i = i + 1) begin
                if (tick_sig_after[i] !== tick_sig_before[i + 1]) begin
                    errors = errors + 1;
                    $display("FAIL: parallax layer did not shift after two frames at column %0d", i);
                    i = 380;
                end
            end
        end

        // ---- 3. subtitle alpha ramp is visible and symmetrical -----------
        collect_gradient;
        distinct_alpha = 0;
        for (i = 0; i < 32; i = i + 1) begin
            if (band_gray[i] < 0) begin
                errors = errors + 1;
                $display("FAIL: no subtitle background at row %0d", i);
            end
            found_alpha = 0;
            for (j = 0; j < i; j = j + 1) begin
                if (band_gray[i] === band_gray[j])
                    found_alpha = 1;
            end
            if (!found_alpha)
                distinct_alpha = distinct_alpha + 1;
        end
        if (band_gray[0] <= band_gray[15]) begin
            errors = errors + 1;
            $display("FAIL: subtitle gradient does not get darker at centre");
        end
        if (band_gray[0] !== band_gray[31]) begin
            errors = errors + 1;
            $display("FAIL: subtitle gradient is not symmetrical");
        end
        if (band_gray[15] !== band_gray[16]) begin
            errors = errors + 1;
            $display("FAIL: subtitle gradient centre rows differ");
        end
        if (distinct_alpha < 8) begin
            errors = errors + 1;
            $display("FAIL: subtitle alpha has too few levels (%0d)", distinct_alpha);
        end
        $display("ADV01 TB: subtitle alpha levels = %0d, edge=%0d centre=%0d",
                 distinct_alpha, band_gray[0], band_gray[15]);

        // ---- 4. a host TIME message drives the airport clock -------------
        collect_time_sig(time_sig_before);
        host_time_hh = 8'h12;
        host_time_mm = 8'h34;
        host_time_ss = 8'h56;
        @(negedge pixel_clk);
        host_time_tog = ~host_time_tog;
        repeat (8) @(posedge pixel_clk);
        if ((dut.rtc_hh !== 8'h12) || (dut.rtc_mm !== 8'h34) ||
            (dut.rtc_ss !== 8'h56)) begin
            errors = errors + 1;
            $display("FAIL: host TIME load did not reach the RTC latch");
        end
        collect_time_sig(time_sig_after);
        if (time_sig_after === time_sig_before) begin
            errors = errors + 1;
            $display("FAIL: host TIME load did not change the display");
        end

        // ---- 5. the enables really gate their own layer ------------------
        logo_enable = 1'b0;
        subtitle_enable = 1'b0;
        timestamp_enable = 1'b0;
        parallax_enable = 1'b0;
        repeat (4) @(posedge pixel_clk);
        scan_frame(0);
        if ((top_hits != 0) || (time_hits != 0) || (tick_hits != 0) ||
            (sub_hits != 0) || (stray_hits != 0)) begin
            errors = errors + 1;
            $display("FAIL: overlay still painted with all four enables low");
        end

        logo_enable = 1'b1;
        repeat (4) @(posedge pixel_clk);
        scan_frame(0);
        if ((top_hits == 0) || (time_hits != 0) ||
            (tick_hits != 0) || (sub_hits != 0)) begin
            errors = errors + 1;
            $display("FAIL: logo, time, parallax and subtitle are not independent");
        end

        // ---- 6. the second panel row reports the source resolution -------
        count_res_row(res_cyan);
        if (res_cyan != 0) begin
            errors = errors + 1;
            $display("FAIL: resolution row drawn before a size was reported");
        end

        img_width = 12'd640;
        img_height = 12'd480;
        img_size_valid = 1'b1;
        wait (dut.res_bcd_valid === 1'b1);
        count_res_row(res_cyan);
        if (res_cyan == 0) begin
            errors = errors + 1;
            $display("FAIL: resolution row stayed empty for 640X480");
        end
        if ((dut.res_width_bcd !== 16'h0640) ||
            (dut.res_height_bcd !== 16'h0480)) begin
            errors = errors + 1;
            $display("FAIL: 640X480 became %04h X %04h",
                     dut.res_width_bcd, dut.res_height_bcd);
        end
        $display("ADV01 TB: resolution row = %0d glyph pixels for 640X480",
                 res_cyan);

        img_width = 12'd960;
        img_height = 12'd720;
        wait (dut.res_bcd_valid === 1'b0);
        wait (dut.res_bcd_valid === 1'b1);
        if ((dut.res_width_bcd !== 16'h0960) ||
            (dut.res_height_bcd !== 16'h0720)) begin
            errors = errors + 1;
            $display("FAIL: 960X720 became %04h X %04h",
                     dut.res_width_bcd, dut.res_height_bcd);
        end

        if (errors == 0) $display("ADV01 TB: PASS");
        else             $display("ADV01 TB: FAIL with %0d error(s)", errors);
        $finish;
    end

endmodule
