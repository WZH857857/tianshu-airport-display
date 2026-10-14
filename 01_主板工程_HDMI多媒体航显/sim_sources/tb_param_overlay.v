`timescale 1ns / 1ps
//=============================================================================
// tb_param_overlay.v - advanced item 4 readout check
//
// Item 4 asks for a display parameter that is adjusted from the board keys and
// for the change to be shown on the picture.  The adjustment itself lives in
// hdmi_mm_player (bright_level / contrast_level); this bench covers the other
// half, the parameter panel of ui_overlay, which is what ends up on screen.
//
// Checks
//   1  BRIGHT n   paints exactly the seven inked cells of that eight slot text
//   2  the digit cell follows the level, the label cells do not move
//   3  CONTRAST n paints ten slots and is a different string from BRIGHT n
//   4  no parameter panel once the one second flash window is over
//   5  no parameter panel when the OSD feature is compiled out
//   6  the panel replaces the airport clock while it is on screen
//
// The panel is also printed as ASCII art, one sample per 2x2 pixel block, so
// the glyphs can be read back by eye instead of only counted.
//=============================================================================
module tb_param_overlay;

    localparam [9:0] PARAM_X0 = 10'd456;
    localparam [9:0] PARAM_Y0 = 10'd12;
    localparam [9:0] TEXT_Y   = PARAM_Y0 + 10'd8;
    localparam integer BLOCKS = 84;      // 168 pixels / 2

    // character cells: 16 pixels = 8 samples wide
    localparam [9:0] BRIGHT_MASK   = 10'b00_1011_1111;   // 0..5 and 7
    localparam [9:0] CONTRAST_MASK = 10'b10_1111_1111;   // 0..7 and 9

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
    reg        osd_enable = 1'b1;
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
    integer glyph_hits;
    integer clock_hits;
    integer panel_hits;

    reg [7:0] art     [0:7][0:BLOCKS-1];
    reg [7:0] ref_art [0:7][0:BLOCKS-1];
    integer   ref_glyph_hits;

    task sample_panel;
        integer r, c;
        begin
            glyph_hits = 0;
            clock_hits = 0;
            panel_hits = 0;
            for (r = 0; r < 8; r = r + 1) begin
                py = TEXT_Y + (r * 2);
                for (c = 0; c < BLOCKS; c = c + 1) begin
                    px = PARAM_X0 + (c * 2);
                    #1;
                    if (rgb == 24'hFFC000) begin
                        art[r][c] = "#";
                        glyph_hits = glyph_hits + 1;
                    end else if (rgb == 24'hFFD060) begin
                        art[r][c] = "C";
                        clock_hits = clock_hits + 1;
                    end else if (hit) begin
                        art[r][c] = "-";
                        panel_hits = panel_hits + 1;
                    end else begin
                        art[r][c] = ".";
                    end
                end
            end
        end
    endtask

    task print_panel;
        input [8*24-1:0] title;
        integer r, c;
        begin
            $display("--- %0s ---", title);
            for (r = 0; r < 8; r = r + 1) begin
                $write("|");
                for (c = 0; c < BLOCKS; c = c + 1)
                    $write("%c", art[r][c]);
                $write("|\n");
            end
        end
    endtask

    // Ink mask of the sampled panel, one bit per 16 pixel character cell.
    function [9:0] cell_mask;
        integer r, c;
        reg [9:0] m;
        begin
            m = 10'd0;
            for (c = 0; c < BLOCKS; c = c + 1)
                for (r = 0; r < 8; r = r + 1)
                    if (art[r][c] == "#")
                        m[c/8] = 1'b1;
            cell_mask = m;
        end
    endfunction

    // Number of 2x2 blocks that differ from the reference inside a column
    // range, i.e. inside a set of character cells.
    function integer diff_blocks;
        input integer c0;
        input integer c1;
        integer r, c, bad;
        begin
            bad = 0;
            for (c = c0; c <= c1; c = c + 1)
                for (r = 0; r < 8; r = r + 1)
                    if (art[r][c] !== ref_art[r][c])
                        bad = bad + 1;
            diff_blocks = bad;
        end
    endfunction

    task snapshot_reference;
        integer r, c;
        begin
            for (r = 0; r < 8; r = r + 1)
                for (c = 0; c < BLOCKS; c = c + 1)
                    ref_art[r][c] = art[r][c];
            ref_glyph_hits = glyph_hits;
        end
    endtask

    task settle;
        begin
            repeat (4) @(posedge pixel_clk);
            #1;
        end
    endtask

    initial begin
        repeat (4) @(posedge pixel_clk);
        rst = 1'b0;
        repeat (2) @(posedge pixel_clk);

        // 1 / 6: brightness readout, clock replaced while it is up.
        param_flash   = 1'b1;
        param_sel     = 2'd0;
        user_bright   = 3'd7;
        user_contrast = 3'd3;
        settle;
        sample_panel;
        print_panel("BRIGHT 7");
        if (cell_mask() !== BRIGHT_MASK) begin
            errors = errors + 1;
            $display("FAIL: BRIGHT readout cells = %010b", cell_mask());
        end
        if (clock_hits != 0) begin
            errors = errors + 1;
            $display("FAIL: airport clock still drawn under the parameter panel");
        end
        if (panel_hits == 0) begin
            errors = errors + 1;
            $display("FAIL: parameter panel has no shaded background");
        end
        snapshot_reference;

        // 2: only the digit cell moves when the level moves.
        user_bright = 3'd3;
        settle;
        sample_panel;
        print_panel("BRIGHT 3");
        if (cell_mask() !== BRIGHT_MASK) begin
            errors = errors + 1;
            $display("FAIL: BRIGHT 3 readout cells = %010b", cell_mask());
        end
        if (diff_blocks(56, 63) == 0) begin
            errors = errors + 1;
            $display("FAIL: brightness digit did not follow the level");
        end
        if (diff_blocks(0, 47) != 0) begin
            errors = errors + 1;
            $display("FAIL: BRIGHT label cells changed with the level");
        end
        if (diff_blocks(64, 83) != 0) begin
            errors = errors + 1;
            $display("FAIL: panel changed outside the readout");
        end
        snapshot_reference;

        // 3: contrast readout is the longer label plus its own digit.
        param_sel     = 2'd1;
        user_contrast = 3'd5;
        settle;
        sample_panel;
        print_panel("CONTRAST 5");
        if (cell_mask() !== CONTRAST_MASK) begin
            errors = errors + 1;
            $display("FAIL: CONTRAST readout cells = %010b", cell_mask());
        end
        if (diff_blocks(0, 47) < 8) begin
            errors = errors + 1;
            $display("FAIL: CONTRAST label is too close to BRIGHT (%0d blocks)",
                     diff_blocks(0, 47));
        end

        // 4: flash window over, the clock comes back.
        // The clock panel shares this rectangle, so its shade is expected;
        // what must be gone is the parameter text.
        param_flash = 1'b0;
        settle;
        sample_panel;
        if (glyph_hits != 0) begin
            errors = errors + 1;
            $display("FAIL: parameter panel drawn outside its flash window");
        end
        if (clock_hits == 0) begin
            errors = errors + 1;
            $display("FAIL: airport clock did not come back");
        end

        // 5: OSD feature compiled out.
        param_flash = 1'b1;
        osd_enable  = 1'b0;
        settle;
        sample_panel;
        if (glyph_hits != 0) begin
            errors = errors + 1;
            $display("FAIL: parameter panel drawn with the OSD feature off");
        end

        if (errors == 0) $display("PARAM TB: PASS");
        else             $display("PARAM TB: FAIL with %0d error(s)", errors);
        $finish;
    end

endmodule
