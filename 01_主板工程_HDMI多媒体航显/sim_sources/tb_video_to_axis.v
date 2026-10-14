`timescale 1ns / 1ps
//=============================================================================
// tb_video_to_axis.v
//
// Drives the real 640x480 timing generator into the AXIS bridge that feeds the
// HDMI 1.4b transmitter core and checks the framing the core depends on:
//
//   * O_video_valid is I_de delayed by one pixel clock
//   * O_video_data is I_rgb delayed by one pixel clock, so data and valid
//     stay together
//   * O_video_user marks exactly the first active pixel of a frame
//   * O_video_last marks exactly the last active pixel of each line, so a
//     640 x 480 frame produces 480 of them
//   * the number of active pixels handed over is 640 * 480 per frame
//=============================================================================

module tb_video_to_axis;

    localparam integer H_ACTIVE = 640;
    localparam integer V_ACTIVE = 480;
    localparam integer FRAMES   = 3;

    reg pixel_clk = 1'b0;
    always #20 pixel_clk = ~pixel_clk;      // 25 MHz

    reg reset_n = 1'b0;
    reg rst_hi  = 1'b1;

    wire [9:0] x, y;
    wire       hsync, vsync, video_active, frame_tick;

    reg [23:0] rgb;
    always @* rgb = {x[7:0], y[7:0]};

    video_timing_640x480 u_timing (
        .pixel_clk    (pixel_clk),
        .reset_n      (reset_n),
        .x            (x),
        .y            (y),
        .hsync        (hsync),
        .vsync        (vsync),
        .video_active (video_active),
        .frame_tick   (frame_tick)
    );

    wire        ax_user, ax_valid, ax_last;
    wire [23:0] ax_data;

    video_rgb_to_axis_640x480 u_axis (
        .I_clk         (pixel_clk),
        .I_rst         (rst_hi),
        .I_vs          (vsync),
        .I_de          (video_active),
        .I_rgb         (rgb),
        .O_video_user  (ax_user),
        .O_video_valid (ax_valid),
        .O_video_last  (ax_last),
        .O_video_data  (ax_data)
    );

    integer active_pixels;
    integer user_pulses;
    integer last_pulses;
    integer valid_bad;
    integer data_bad;
    integer user_per_frame;
    integer last_per_frame;

    reg        de_prev;
    reg [23:0] rgb_prev;
    integer    frame_count;

    initial begin
        active_pixels   = 0;
        user_pulses     = 0;
        last_pulses     = 0;
        valid_bad       = 0;
        data_bad        = 0;
        user_per_frame  = 0;
        last_per_frame  = 0;
        de_prev         = 1'b0;
        rgb_prev        = 24'd0;
        frame_count     = 0;
    end

    // sample on the falling edge: both the timing generator and the bridge
    // update on the rising edge, so everything is stable here
    always @(negedge pixel_clk) begin
        if (reset_n) begin
            // the bridge registers I_de and I_rgb on the rising edge, which
            // means the outputs here carry the values sampled one clock ago
            if (ax_valid !== de_prev)  valid_bad = valid_bad + 1;
            if (ax_data  !== rgb_prev) data_bad  = data_bad + 1;

            if (ax_valid)         active_pixels = active_pixels + 1;
            if (ax_user)          user_pulses   = user_pulses + 1;
            if (ax_last)          last_pulses   = last_pulses + 1;
            if (ax_user && ax_valid) user_per_frame = user_per_frame + 1;
            if (ax_last && ax_valid) last_per_frame = last_per_frame + 1;
            if (frame_tick)       frame_count   = frame_count + 1;
        end

        de_prev  <= video_active;
        rgb_prev <= rgb;
    end

    integer errors;

    initial begin
        errors = 0;

        repeat (4) @(negedge pixel_clk);
        reset_n = 1'b1;
        rst_hi  = 1'b0;

        wait (frame_count >= FRAMES);
        @(negedge pixel_clk);

        $display("---- video to AXIS ----");
        $display("frames counted             = %0d", frame_count);
        $display("active pixels              = %0d (expect %0d)",
                 active_pixels, FRAMES * H_ACTIVE * V_ACTIVE);
        $display("O_video_user pulses        = %0d", user_pulses);
        $display("O_video_last pulses        = %0d (expect %0d)",
                 last_pulses, FRAMES * V_ACTIVE);
        $display("valid and de disagreement  = %0d", valid_bad);
        $display("data and rgb disagreement  = %0d", data_bad);

        if (active_pixels != FRAMES * H_ACTIVE * V_ACTIVE) begin
            $display("FAIL wrong number of active pixels");
            errors = errors + 1;
        end
        if (last_pulses != FRAMES * V_ACTIVE) begin
            $display("FAIL O_video_last is not one pulse per line");
            errors = errors + 1;
        end
        if (user_pulses < FRAMES - 1 || user_pulses > FRAMES) begin
            $display("FAIL O_video_user is not one pulse per frame");
            errors = errors + 1;
        end
        if (valid_bad != 0) begin
            $display("FAIL O_video_valid does not track I_de");
            errors = errors + 1;
        end
        if (data_bad != 0) begin
            $display("FAIL O_video_data does not track I_rgb");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_video_to_axis");
        else
            $display("RESULT: FAIL tb_video_to_axis errors=%0d", errors);

        $finish;
    end

endmodule
