`timescale 1ns / 1ps

module tb_resolution_readout;
    reg         pixel_clk = 1'b0;
    reg         rst = 1'b1;
    reg         frame_tick = 1'b0;
    reg         de = 1'b0;
    reg  [9:0]  px = 10'd0;
    reg  [9:0]  py = 10'd0;
    reg  [23:0] base_rgb = 24'h000000;
    reg         ui_enable = 1'b1;
    reg  [2:0]  img_slot = 3'd0;
    reg  [7:0]  img_count = 8'd6;
    reg  [11:0] img_width = 12'd1920;
    reg  [11:0] img_height = 12'd1080;
    reg         img_size_valid = 1'b1;
    reg  [2:0]  user_bright = 3'd7;
    reg  [2:0]  user_contrast = 3'd3;
    reg  [1:0]  ui_mode = 2'd0;
    reg         param_flash = 1'b0;
    reg  [1:0]  param_sel = 2'd0;
    reg         auto_mode = 1'b0;
    reg         logo_enable = 1'b1;
    reg         subtitle_enable = 1'b1;
    reg         timestamp_enable = 1'b1;
    reg         parallax_enable = 1'b1;
    reg         osd_enable = 1'b1;
    reg         wave_enable = 1'b1;
    reg         host_text_tog = 1'b0;
    reg  [4:0]  host_text_addr = 5'd0;
    reg  [7:0]  host_text_data = 8'd0;
    reg         host_text_commit_tog = 1'b0;
    reg         host_text_clear_tog = 1'b0;
    reg         host_link = 1'b0;
    reg         audio_rst = 1'b0;
    reg         audio_valid = 1'b0;
    reg  [23:0] audio_left = 24'd0;
    wire [23:0] rgb;
    wire        hit;

    integer errors = 0;
    integer slot;
    reg [7:0] expected [0:12];

    ui_overlay dut (
        .pixel_clk (pixel_clk),
        .rst (rst),
        .frame_tick (frame_tick),
        .de (de),
        .px (px),
        .py (py),
        .base_rgb (base_rgb),
        .ui_enable (ui_enable),
        .img_slot (img_slot),
        .img_count (img_count),
        .img_width (img_width),
        .img_height (img_height),
        .img_size_valid (img_size_valid),
        .user_bright (user_bright),
        .user_contrast (user_contrast),
        .ui_mode (ui_mode),
        .param_flash (param_flash),
        .param_sel (param_sel),
        .auto_mode (auto_mode),
        .logo_enable (logo_enable),
        .subtitle_enable (subtitle_enable),
        .timestamp_enable (timestamp_enable),
        .parallax_enable (parallax_enable),
        .osd_enable (osd_enable),
        .wave_enable (wave_enable),
        .host_text_tog (host_text_tog),
        .host_text_addr (host_text_addr),
        .host_text_data (host_text_data),
        .host_text_commit_tog (host_text_commit_tog),
        .host_text_clear_tog (host_text_clear_tog),
        .host_link (host_link),
        .audio_rst (audio_rst),
        .audio_valid (audio_valid),
        .audio_left (audio_left),
        .rgb (rgb),
        .hit (hit)
    );

    always #20 pixel_clk = ~pixel_clk;

    initial begin
        expected[0]  = 8'h52; // R
        expected[1]  = 8'h45; // E
        expected[2]  = 8'h53; // S
        expected[3]  = 8'h20;
        expected[4]  = 8'h31; // 1
        expected[5]  = 8'h39; // 9
        expected[6]  = 8'h32; // 2
        expected[7]  = 8'h30; // 0
        expected[8]  = 8'h58; // X
        expected[9]  = 8'h31; // 1
        expected[10] = 8'h30; // 0
        expected[11] = 8'h38; // 8
        expected[12] = 8'h30; // 0

        repeat (5) @(posedge pixel_clk);
        rst = 1'b0;
        repeat (120) @(posedge pixel_clk);

        if (!dut.res_bcd_valid) begin
            $display("RESULT: FAIL conversion did not finish");
            errors = errors + 1;
        end
        if (dut.res_width_bcd !== 16'h1920) begin
            $display("RESULT: FAIL width BCD = %04h", dut.res_width_bcd);
            errors = errors + 1;
        end
        if (dut.res_height_bcd !== 16'h1080) begin
            $display("RESULT: FAIL height BCD = %04h", dut.res_height_bcd);
            errors = errors + 1;
        end

        for (slot = 0; slot < 13; slot = slot + 1) begin
            px = 10'd12 + (slot * 16);
            #1;
            if (dut.res_ch !== expected[slot]) begin
                $display("RESULT: FAIL slot %0d char %02h expected %02h",
                         slot, dut.res_ch, expected[slot]);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("RESULT: PASS tb_resolution_readout");
        else
            $display("RESULT: FAIL tb_resolution_readout errors=%0d", errors);
        $finish;
    end

    initial begin
        #200000;
        $display("RESULT: FAIL tb_resolution_readout timeout");
        $finish;
    end
endmodule
