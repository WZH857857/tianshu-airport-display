`timescale 1ns / 1ps
//=============================================================================
// tb_emg_dump.v - dump the fire emergency overlay as a raw RGB frame
//
// All ordinary layers are disabled, so the dumped 640 x 480 frame is exactly
// what the new emergency branch paints.  The frame is written as three bytes
// per pixel and turned into a PNG by scripts/make_emg_preview.py.
//=============================================================================
module tb_emg_dump;

    reg        pixel_clk = 1'b0;
    reg        rst       = 1'b1;
    reg        de        = 1'b1;
    reg [9:0]  px        = 10'd0;
    reg [9:0]  py        = 10'd0;
    reg [23:0] base_rgb  = 24'h101010;

    wire [23:0] rgb;
    wire        hit;

    ui_overlay dut (
        .pixel_clk        (pixel_clk),
        .rst              (rst),
        .frame_tick       (1'b0),
        .de               (de),
        .px               (px),
        .py               (py),
        .base_rgb         (base_rgb),
        .ui_enable        (1'b0),
        .img_slot         (3'd0),
        .img_count        (8'd0),
        .img_width        (12'd640),
        .img_height       (12'd480),
        .img_size_valid   (1'b0),
        .user_bright      (3'd7),
        .user_contrast    (3'd3),
        .ui_mode          (2'd0),
        .param_flash      (1'b0),
        .param_sel        (2'd0),
        .auto_mode        (1'b0),
        .emg_fire         (1'b1),
        .logo_enable      (1'b0),
        .subtitle_enable  (1'b0),
        .timestamp_enable (1'b0),
        .parallax_enable  (1'b0),
        .osd_enable       (1'b0),
        .wave_enable      (1'b0),
        .host_text_tog    (1'b0),
        .host_text_addr   (5'd0),
        .host_text_data   (8'd0),
        .host_text_commit_tog (1'b0),
        .host_text_clear_tog  (1'b0),
        .host_link        (1'b0),
        .host_time_hh     (8'h00),
        .host_time_mm     (8'h00),
        .host_time_ss     (8'h00),
        .host_time_tog    (1'b0),
        .rtc_time_hh      (8'h00),
        .rtc_time_mm      (8'h00),
        .rtc_time_ss      (8'h00),
        .rtc_time_tog     (1'b0),
        .audio_rst        (1'b1),
        .audio_valid      (1'b0),
        .audio_left       (24'd0),
        .board_enable     (1'b0),
        .board_wr_tog     (1'b0),
        .board_wr_addr    (8'd0),
        .board_wr_data    (8'd0),
        .rgb              (rgb),
        .hit              (hit)
    );

    integer fx, x, y;

    task dump_frame;
        input [127:0] path;
        input [6:0]   anim;
        begin
            dut.emg_anim = anim;
            fx = $fopen(path, "wb");
            for (y = 0; y < 480; y = y + 1) begin
                for (x = 0; x < 640; x = x + 1) begin
                    px = x[9:0];
                    py = y[9:0];
                    de = 1'b1;
                    @(posedge pixel_clk);
                    #1;
                    $fwrite(fx, "%c%c%c",
                            rgb[23:16], rgb[15:8], rgb[7:0]);
                end
            end
            $fclose(fx);
            $display("dumped %s (anim=%0d)", path, anim);
        end
    endtask

    always #5 pixel_clk = ~pixel_clk;    // 100 MHz is fine for a dump

    initial begin
        rst = 1'b1;
        repeat (4) @(posedge pixel_clk);
        rst = 1'b0;
        repeat (4) @(posedge pixel_clk);

        dump_frame("emg_dark.raw", 7'd0);
        dump_frame("emg_flash.raw", 7'd32);
        $finish;
    end

endmodule
