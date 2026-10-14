//===========================================================================
// fall_camera_hdmi_multi_top.v -- crowd fall detection with HDMI out
//
// VF-E20NG88 (EG4S20NG88).  Camera: the ID-003A MT9M001 gray module that
// ships with the board, not the OV5640 the front end was first written for.
//
//   ID-003A MT9M001, 8-bit gray, PIXCLK 24 MHz
//        |    sensor set to row+column skip -> 640x512 at ~33 fps
//        |
//        +--> cmos_capture_gray8    480 of 512 rows, gray -> {R,G,B}565
//        |                          (gray_front's luma is then an identity)
//        |
//        +--> fall_core_multi_top   gray -> diff -> morph -> multi_bbox
//        |                          -> track_assoc -> posture_bank
//        |                          (4 boxes + 4 alarms, all in the FPGA)
//        |
//        +--> frame_store  (two 640x480 buffers in the EG4S20 on-die SDRAM)
//                   |
//                   +--> 640x480 raster --> box_overlay --> hdmi_tx --> socket
//
// Nothing leaves the FPGA except the TMDS pairs and the status lines: no
// MCU, no soft core, no external frame buffer.  The SDRAM used for the
// display buffer is the 2M x 32 device inside the EG4S20 package; the
// picture memory is never touched by the fall algorithm.
//
// Clocks (all from the 24 MHz oscillator on P13):
//   clk_pixel     25.0 MHz   video raster, overlay, TMDS encoder
//   clk_pixel_5x 125.0 MHz   TMDS serialiser
//   clk_mem      120.0 MHz   SDRAM controller
//   clk_mem_sft  120.0 MHz   180 degrees, SDRAM read capture
//   cmos_pclk     24.0 MHz   camera capture + vision core + frame_store load
//
// The video PLL runs 25.000 / 125.000 MHz, not the 24 MHz of the oscillator:
// the probe bit showed this board's monitors reject the 57.1 Hz raster that a
// plain divide-by-one 24 MHz pixel clock produces, while the exact CEA-861
// 25 MHz pair locks every time.  The camera stays on the 24 MHz oscillator.
//
// The camera side is three blocks that are new with the MT9M001:
//   mt9m001_cfg    13-register boot table over the two-wire bus (0xBA, 8-bit
//                  register addresses, 16-bit values) plus a dynamic port
//                  the exposure loop uses afterwards
//   cmos_capture   falling-edge capture, gray replication, row/column window,
//                  and a fallback decimator for a sensor that kept its
//                  factory 1280x1024 window
//   ae_ctrl        sums the frame, steps shutter (0x09) and then gain (0x35)
//                  into a dead band at a frame boundary, and hides the two
//                  frames that follow the step from the vision core (de and
//                  frame_start gated together, released in the trailing
//                  blanking, so the core never sees a short frame)
//
// Status is unchanged from the vision-only board top:
//   LED0 cfg_done, LED1 armed, LED2 heartbeat, LED3 any slot found,
//   LED4 any slot in FALL, LED5 alarm latch, LED6 geometry error,
//   LED7 frames arriving.
//   UART 115200 prints one line per slot per frame.
//===========================================================================
`timescale 1ns / 1ps

module fall_camera_hdmi_multi_top #(
    parameter integer POWERUP_MS  = 20,
    parameter integer RESET_MS    = 2,
    parameter integer WAIT_FRAMES = 10,
    parameter integer SCCB_HZ     = 250000,
    parameter integer BAUD        = 115200,
    parameter integer POR_CYCLES  = 1048575,
    parameter integer NUM_BLOBS   = 4,
    parameter integer MARGIN      = 8,
    // Board tuning: a monitor-to-camera round trip has less contrast than
    // the direct video the Python model sees, so the threshold has to stay
    // low enough to keep the motion window continuous on a dark monitor.
    // 12 is still high enough that the crowd negatives do not break through:
    // with the disk(1)/disk(3) front end it holds 12/15 GMDCSA24 falls at
    // zero crowd-clip false alarms, while 8 lights crowd02 up.
    // (8/4/160 was the value used with the old disk(2)/disk(2) morphology.)
    parameter integer DIFF_THRESH   = 12,
    parameter integer BBOX_MIN_PIX  = 4,
    parameter integer BBOX_MIN_AREA = 160,
    // DIAGNOSTIC BUILD: 1 points the status lines at motion_hist's publish
    // instead of the tracker box (see slot_scan USE_PBOX).  0 is the release
    // build: HDMI and UART both report the tracked boxes.
    parameter integer EMIT_PBOX   = 0
) (
    input  wire        clk_24m,        // P13
    input  wire        key_n,          // P51, KEY0, low when pressed

    output wire        cmos_xclk,      // P72
    output wire        cmos_sclk,      // P70
    inout  wire        cmos_sdat,      // P71
    input  wire [7:0]  cmos_data,      // P2 P87 P86 P78 P77 P76 P75 P74
    input  wire        cmos_pclk,      // P79
    input  wire        cmos_href,      // P62
    input  wire        cmos_vsync,     // P66

    output wire        HDMI_CLK_P,     // P47
    output wire        HDMI_D2_P,      // P61
    output wire        HDMI_D1_P,      // P55
    output wire        HDMI_D0_P,      // P49

    output wire        led595_clk,     // P59
    output wire        led595_latch,   // P57
    output wire        led595_dout,    // P45
    output wire        uart_txd        // P3
);

    localparam integer FRAME_PIX = 640 * 480;

    // Exposure dead band.  The first board build used 70..150 and the picture
    // parked at ~27% grey, which reads as "too dark" on a monitor; these
    // values keep the loop brightening until the frame mean is at least 45%
    // grey.  The 1/32 step inside ae_ctrl stays untouched: one step has to
    // move a bright pixel by less than frame_diff's 10-count threshold, or the
    // exposure change itself would be handed to the detector as motion.
    localparam integer AE_LOW_MEAN      = 115;
    localparam integer AE_HIGH_MEAN     = 165;
    localparam integer AE_PERIOD_FRAMES = 4;

    // ------------------------------------------------------- clock / reset
    assign cmos_xclk = clk_24m;

    localparam integer POR_END = (POR_CYCLES < 2) ? 2 : POR_CYCLES;

    reg  [19:0] por_cnt;
    reg  [19:0] rel_cnt;
    reg  [1:0]  key_sync;
    reg         rst_n_r;

    initial begin
        por_cnt <= 20'd0;
        rel_cnt <= 20'd0;
        key_sync <= 2'b00;
        rst_n_r <= 1'b0;
    end

    wire key_level = key_sync[1];
    wire por_done  = (por_cnt >= POR_END - 1);
    wire rel_done  = (rel_cnt >= POR_END - 1);

    wire clk_pixel, clk_pixel_5x, lock_video;
    wire clk_mem, clk_mem_sft, lock_mem;

    hdmi_video_pll_25 u_video_pll (
        .refclk       (clk_24m),
        .reset        (~por_done),
        .clk_pixel    (clk_pixel),
        .clk_pixel_5x (clk_pixel_5x),
        .extlock      (lock_video)
    );

    hdmi_mem_pll_24 u_mem_pll (
        .refclk     (clk_24m),
        .reset      (~por_done),
        .clk_mem    (clk_mem),
        .clk_mem_sft(clk_mem_sft),
        .extlock    (lock_mem)
    );

    always @(posedge clk_24m) begin
        key_sync <= {key_sync[0], key_n};
        if (!por_done)      por_cnt <= por_cnt + 20'd1;
        if (!key_level)     rel_cnt <= 20'd0;
        else if (!rel_done) rel_cnt <= rel_cnt + 20'd1;
    end

    always @(posedge clk_24m)
        rst_n_r <= por_done & rel_done & lock_video & lock_mem;

    wire rst_n = rst_n_r;

    // ------------------------------------------- two-wire config (24 MHz)
    // The MT9M001 takes device byte 0xBA, an 8-bit register address and a
    // 16-bit value; the engine also carries the dynamic port the exposure
    // loop below uses, so all wire timing stays in one module.
    wire        cfg_done;
    wire        cfg_busy;
    wire [7:0]  wr_count;
    wire        ae_req_tog;
    wire [7:0]  ae_req_addr;
    wire [15:0] ae_req_data;
    wire        ae_ack_tog;

    mt9m001_cfg #(
        .CLK_HZ     (24000000),
        .SCL_HZ     (SCCB_HZ),       // name kept from the OV5640 front end
        .POWERUP_MS (POWERUP_MS),
        .RESET_MS   (RESET_MS)
    ) u_cfg (
        .clk      (clk_24m),
        .rst_n    (rst_n),
        .scl      (cmos_sclk),
        .sda      (cmos_sdat),
        .cfg_done (cfg_done),
        .cfg_busy (cfg_busy),
        .wr_count (wr_count),
        .dyn_tog  (ae_req_tog),
        .dyn_addr (ae_req_addr),
        .dyn_data (ae_req_data),
        .ack_tog  (ae_ack_tog)
    );

    reg [2:0] cfg_sync;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) cfg_sync <= 3'b000;
        else        cfg_sync <= {cfg_sync[1:0], cfg_done};
    end
    wire cfg_done_pclk = cfg_sync[2];

    // ------------------------------------------------- capture (pclk domain)
    wire        de_565, frame_start_565, armed;
    wire [15:0] pix_565;
    wire [15:0] frame_cnt;
    wire [10:0] line_cnt;
    wire [11:0] pix_cnt;
    wire [7:0]  hw_err;
    wire [7:0]  gray_pix;
    wire        frame_end_565;
    wire        wide_mode;

    cmos_capture_gray8 #(
        .WAIT_FRAMES (WAIT_FRAMES)
    ) u_cap (
        .pclk        (cmos_pclk),
        .rst_n       (rst_n),
        .arm         (cfg_done_pclk),
        .vsync       (cmos_vsync),
        .href        (cmos_href),
        .data        (cmos_data),
        .de          (de_565),
        .pix         (pix_565),
        .gray_out    (gray_pix),
        .frame_start (frame_start_565),
        .frame_end   (frame_end_565),
        .armed       (armed),
        .wide_mode   (wide_mode),
        .frame_cnt   (frame_cnt),
        .line_cnt    (line_cnt),
        .pix_cnt     (pix_cnt),
        .hw_err      (hw_err)
    );

    // -------------------------------------------------- auto exposure (pclk)
    // The loop sees the complete stream; only the vision core is gated.
    wire       ae_freeze;
    wire [1:0] ae_zone;
    wire [7:0] ae_mean;
    wire [15:0] ae_shutter;
    wire [7:0]  ae_gain;

    ae_ctrl #(
        .LOW_MEAN      (AE_LOW_MEAN),
        .HIGH_MEAN     (AE_HIGH_MEAN),
        .PERIOD_FRAMES (AE_PERIOD_FRAMES)
    ) u_ae (
        .clk         (cmos_pclk),
        .rst_n       (rst_n),
        .de          (de_565),
        .gray        (gray_pix),
        .frame_start (frame_start_565),
        .frame_end   (frame_end_565),
        .req_tog     (ae_req_tog),
        .req_addr    (ae_req_addr),
        .req_data    (ae_req_data),
        .ack_tog     (ae_ack_tog),
        .freeze      (ae_freeze),
        .zone        (ae_zone),
        .shutter_dbg (ae_shutter),
        .gain_dbg    (ae_gain),
        .mean_dbg    (ae_mean)
    );

    // A frame that straddles an exposure step is removed whole - de and
    // frame_start are gated together - so every frame the vision core sees is
    // complete and opens with its own frame_start pulse.  The picture path
    // below keeps the ungated stream, so the screen stays live while the loop
    // adjusts.
    wire de_core          = de_565 & ~ae_freeze;
    wire frame_start_core = frame_start_565 & ~ae_freeze;

    wire [23:0] cam_rgb24 = {pix_565[15:11], pix_565[15:13],
                             pix_565[10:5],  pix_565[10:9],
                             pix_565[4:0],   pix_565[4:2]};

    reg [3:0] core_rel_cnt;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)                    core_rel_cnt <= 4'd0;
        else if (core_rel_cnt != 4'hF) core_rel_cnt <= core_rel_cnt + 4'd1;
    end
    wire core_rst_n = (core_rel_cnt == 4'hF);

    // ------------------------------------------- crowd vision core (pclk)
    wire                    bbox_valid;
    wire [NUM_BLOBS-1:0]    bbox_found, alarm;
    wire [NUM_BLOBS*9-1:0]  bbox_x, bbox_y, bbox_w, bbox_h;
    wire [NUM_BLOBS*17-1:0] bbox_area;
    wire [NUM_BLOBS*7-1:0]  bbox_score;
    wire [NUM_BLOBS*3-1:0]  posture_state;
    wire                    pbox_valid;
    wire [NUM_BLOBS-1:0]    pbox_found, pbox_dir;
    wire [NUM_BLOBS*9-1:0]  pbox_x, pbox_y, pbox_w, pbox_h;
    wire [NUM_BLOBS*13-1:0] pbox_cells;
    wire                    pbox_pass_hit, pbox_sweep_hit;
    wire                    bbox_busy;

    fall_core_multi_top #(
        .NUM_BLOBS     (NUM_BLOBS),
        .MARGIN        (MARGIN),
        .DIFF_THRESH   (DIFF_THRESH),
        .BBOX_MIN_PIX  (BBOX_MIN_PIX),
        .BBOX_MIN_AREA (BBOX_MIN_AREA)
    ) u_core (
        .clk           (cmos_pclk),
        .rst_n         (core_rst_n),
        .frame_start   (frame_start_core),
        .de_in         (de_core),
        .pix_in        (pix_565),
        .bbox_valid    (bbox_valid),
        .bbox_found    (bbox_found),
        .bbox_x        (bbox_x),
        .bbox_y        (bbox_y),
        .bbox_w        (bbox_w),
        .bbox_h        (bbox_h),
        .bbox_area     (bbox_area),
        .bbox_score    (bbox_score),
        .pbox_valid    (pbox_valid),
        .pbox_found    (pbox_found),
        .pbox_x        (pbox_x),
        .pbox_y        (pbox_y),
        .pbox_w        (pbox_w),
        .pbox_h        (pbox_h),
        .pbox_cells    (pbox_cells),
        .pbox_dir      (pbox_dir),
        .pbox_pass_hit (pbox_pass_hit),
        .pbox_sweep_hit(pbox_sweep_hit),
        .posture_state (posture_state),
        .alarm         (alarm)
    );

    // -------------------------------------- pclk -> status/overlay snapshot
    // The four boxes are published together on bbox_valid, but posture_bank
    // writes the four states back about 16 clocks later, so the coherent
    // snapshot is taken 24 clocks after the pulse.  The registers hold until
    // the next frame, so a slower domain always samples a settled value.
    reg [NUM_BLOBS-1:0]    s_found, s_alarm;
    reg [NUM_BLOBS*3-1:0]  s_state;
    reg [15:0]             s_frame;
    reg [NUM_BLOBS*9-1:0]  s_x, s_y, s_w, s_h;
    reg [NUM_BLOBS*17-1:0] s_area;
    reg [NUM_BLOBS*7-1:0]  s_score;
    reg [7:0]              s_ae_mean;
    reg [15:0]             s_ae_shut;
    reg [7:0]              s_ae_gain;
    reg [1:0]              s_ae_zone;
    reg                    snap_tog;
    reg [4:0]              snap_delay;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            s_found    <= {NUM_BLOBS{1'b0}};
            s_alarm    <= {NUM_BLOBS{1'b0}};
            s_state    <= {(NUM_BLOBS*3){1'b0}};
            s_frame    <= 16'd0;
            s_x        <= {(NUM_BLOBS*9){1'b0}};
            s_y        <= {(NUM_BLOBS*9){1'b0}};
            s_w        <= {(NUM_BLOBS*9){1'b0}};
            s_h        <= {(NUM_BLOBS*9){1'b0}};
            s_area     <= {(NUM_BLOBS*17){1'b0}};
            s_score    <= {(NUM_BLOBS*7){1'b0}};
            s_ae_mean  <= 8'd0;
            s_ae_shut  <= 16'd0;
            s_ae_gain  <= 8'd0;
            s_ae_zone  <= 2'd0;
            snap_tog   <= 1'b0;
            snap_delay <= 5'd0;
        end else if (bbox_valid) begin
            snap_delay <= 5'd24;
        end else if (snap_delay != 5'd0) begin
            snap_delay <= snap_delay - 5'd1;
            if (snap_delay == 5'd1) begin
                s_found  <= bbox_found;
                s_alarm  <= alarm;
                s_state  <= posture_state;
                s_frame  <= frame_cnt;
                s_x      <= bbox_x;
                s_y      <= bbox_y;
                s_w      <= bbox_w;
                s_h      <= bbox_h;
                s_area   <= bbox_area;
                s_score  <= bbox_score;
                // The exposure loop's own view of the frame, latched with the
                // boxes so the line describes one coherent instant.
                s_ae_mean <= ae_mean;
                s_ae_shut <= ae_shutter;
                s_ae_gain <= ae_gain;
                s_ae_zone <= ae_zone;
                snap_tog <= ~snap_tog;
            end
        end
    end

    // ------------------------------------------- snapshot into 24 MHz side
    reg [2:0] snap_sync;
    reg [2:0] armed_sync;

    always @(posedge clk_24m or negedge rst_n) begin
        if (!rst_n) begin
            snap_sync  <= 3'b000;
            armed_sync <= 3'b000;
        end else begin
            snap_sync  <= {snap_sync[1:0],  snap_tog};
            armed_sync <= {armed_sync[1:0], armed};
        end
    end

    wire snap_pulse = snap_sync[2] ^ snap_sync[1];
    wire armed_24m  = armed_sync[2];

    reg [NUM_BLOBS-1:0]    u_found, u_alarm;
    reg [NUM_BLOBS*3-1:0]  u_state;
    reg [15:0]             u_frame;
    reg [NUM_BLOBS*9-1:0]  u_x, u_y, u_w, u_h;
    reg [NUM_BLOBS*17-1:0] u_area;
    reg [NUM_BLOBS*7-1:0]  u_score;
    reg [7:0]              u_ae_mean;
    reg [15:0]             u_ae_shut;
    reg [7:0]              u_ae_gain;
    reg [1:0]              u_ae_zone;
    reg [25:0]             alarm_hold;
    reg [23:0]             act_cnt;

    always @(posedge clk_24m or negedge rst_n) begin
        if (!rst_n) begin
            u_found    <= {NUM_BLOBS{1'b0}};
            u_alarm    <= {NUM_BLOBS{1'b0}};
            u_state    <= {(NUM_BLOBS*3){1'b0}};
            u_frame    <= 16'd0;
            u_x        <= {(NUM_BLOBS*9){1'b0}};
            u_y        <= {(NUM_BLOBS*9){1'b0}};
            u_w        <= {(NUM_BLOBS*9){1'b0}};
            u_h        <= {(NUM_BLOBS*9){1'b0}};
            u_area     <= {(NUM_BLOBS*17){1'b0}};
            u_score    <= {(NUM_BLOBS*7){1'b0}};
            u_ae_mean  <= 8'd0;
            u_ae_shut  <= 16'd0;
            u_ae_gain  <= 8'd0;
            u_ae_zone  <= 2'd0;
            alarm_hold <= 26'd0;
            act_cnt    <= 24'd0;
        end else begin
            if (snap_pulse) begin
                u_found <= s_found;
                u_alarm <= s_alarm;
                u_state <= s_state;
                u_frame <= s_frame;
                u_x     <= s_x;
                u_y     <= s_y;
                u_w     <= s_w;
                u_h     <= s_h;
                u_area  <= s_area;
                u_score <= s_score;
                u_ae_mean <= s_ae_mean;
                u_ae_shut <= s_ae_shut;
                u_ae_gain <= s_ae_gain;
                u_ae_zone <= s_ae_zone;
                act_cnt <= 24'hFFFFFF;
            end else if (act_cnt != 24'd0) begin
                act_cnt <= act_cnt - 24'd1;
            end

            if (|s_alarm)                    alarm_hold <= 26'h3FFFFFF;
            else if (alarm_hold != 26'd0)    alarm_hold <= alarm_hold - 26'd1;
        end
    end

    // ------------------------------------- SDRAM frame store (two buffers)
    wire        fs_init_done;
    wire        fs_cal_fail;
    wire [3:0]  fs_cal_latency;
    wire        fs_frame_ready;
    wire        fs_img_frame_ready;
    wire        fs_frame_loaded;
    wire [1:0]  fs_cache_ready;
    wire [2:0]  fs_stored_buf;
    wire        fs_stored_pulse;
    wire [2:0]  fs_state_dbg;

    wire        fs_img_ready;
    wire        fs_pix_underflow;
    wire [23:0] fs_pix_rgb;

    // ------------------------------------------------- 640x480 raster
    // The raster and the pixel-side pipeline are declared here because the
    // frame store is the consumer of px/py/video_active.
    wire [9:0] x, y, px, py;
    wire       hsync, vsync, video_active, frame_tick;
    reg        pix_frame_tog;
    reg        hsync_d, vsync_d, de_d;
    reg [9:0]  px_d, py_d;
    wire [23:0] ov_rgb;

    video_timing_640x480 u_timing (
        .pixel_clk    (clk_pixel),
        .reset_n      (rst_n),
        .x            (x),
        .y            (y),
        .px           (px),
        .py           (py),
        .hsync        (hsync),
        .vsync        (vsync),
        .video_active (video_active),
        .frame_tick   (frame_tick)
    );

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n)          pix_frame_tog <= 1'b0;
        else if (frame_tick) pix_frame_tog <= ~pix_frame_tog;
    end

    // The store hands the pixel out one cycle after the raster reports it, so
    // the overlay and the encoder use the same one-cycle delay the vendor
    // player uses.
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            hsync_d <= 1'b1;
            vsync_d <= 1'b1;
            de_d    <= 1'b0;
            px_d    <= 10'd0;
            py_d    <= 10'd0;
        end else begin
            hsync_d <= hsync;
            vsync_d <= vsync;
            de_d    <= video_active;
            px_d    <= px;
            py_d    <= py;
        end
    end

    // on-die SDRAM nets: the store drives them, the macro consumes them
    wire        sdram_clk, sdram_cke, sdram_cs_n;
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;

    // camera -> frame store loader (all in the cmos_pclk domain)
    reg        cam_load_active;
    reg        cam_load_reload;
    reg [18:0] cam_pix_cnt;
    reg        cam_load_done;

    wire cam_accept = cam_load_active && de_565 && !cam_load_done;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            cam_load_active <= 1'b0;
            cam_load_reload <= 1'b0;
            cam_pix_cnt     <= 19'd0;
            cam_load_done   <= 1'b0;
        end else begin
            cam_load_reload <= 1'b0;

            if (frame_start_565 && armed && fs_init_done) begin
                cam_load_active <= 1'b1;
                cam_load_reload <= 1'b1;
                cam_pix_cnt     <= 19'd0;
                cam_load_done   <= 1'b0;
            end

            if (cam_accept) begin
                if (!fs_img_ready) begin
                    // The store is not draining: give up on this frame and
                    // let frame_store's own watchdog discard the partial
                    // write.  The buffer on screen keeps the last good frame.
                    cam_load_active <= 1'b0;
                end else if (cam_pix_cnt == FRAME_PIX - 1) begin
                    cam_load_done   <= 1'b1;
                    cam_load_active <= 1'b0;
                end else begin
                    cam_pix_cnt <= cam_pix_cnt + 19'd1;
                end
            end
        end
    end

    frame_store #(
        .FRAME_LINES (480),
        .FRAME_WORDS (FRAME_PIX),
        .BUF0_BASE   (21'h000000),
        .BUF1_BASE   (21'h080000),
        .CACHE_MODE  (0),
        .BUF_COUNT   (2),
        .DEFER_SWAP  (0)
    ) u_store (
        .clk             (cmos_pclk),
        .mem_clk         (clk_mem),
        .mem_clk_sft     (clk_mem_sft),
        .rst_n           (rst_n),

        .reload          (cam_load_reload),
        .pattern_mode    (2'd0),
        .img_active      (cam_load_active),
        .img_abort       (1'b0),
        .swap_req        (1'b0),
        .img_valid       (cam_accept),
        .img_rgb         (cam_rgb24),
        .img_ready       (fs_img_ready),
        .frame_ready     (fs_frame_ready),
        .img_frame_ready (fs_img_frame_ready),
        .frame_loaded    (fs_frame_loaded),
        .preload_ready   (),
        .swap_done       (),
        .cache_ready     (fs_cache_ready),
        .stored_buf      (fs_stored_buf),
        .stored_pulse    (fs_stored_pulse),
        .init_done       (fs_init_done),
        .cal_fail        (fs_cal_fail),
        .cal_latency     (fs_cal_latency),
        .state_dbg       (fs_state_dbg),

        .sdram_clk       (sdram_clk),
        .sdram_ras_n     (sdram_ras_n),
        .sdram_cas_n     (sdram_cas_n),
        .sdram_we_n      (sdram_we_n),
        .sdram_addr      (sdram_addr),
        .sdram_ba        (sdram_ba),
        .sdram_cs_n      (sdram_cs_n),
        .sdram_cke       (sdram_cke),
        .sdram_dm        (sdram_dm),
        .sdram_dq        (sdram_dq),

        .pix_clk         (clk_pixel),
        .pix_de          (video_active),
        .pix_x           (px),
        .pix_y           (py),
        .pix_frame_tog   (pix_frame_tog),
        .pix_rgb         (fs_pix_rgb),
        .pix_underflow   (fs_pix_underflow)
    );

    // On-die SDRAM macro: the controller hands out the clock it was designed
    // to drive, so the macro uses that output instead of a local clock.
    EG_PHY_SDRAM_2M_32 u_sdram (
        .clk   (sdram_clk),
        .ras_n (sdram_ras_n),
        .cas_n (sdram_cas_n),
        .we_n  (sdram_we_n),
        .addr  (sdram_addr),
        .ba    (sdram_ba),
        .dq    (sdram_dq),
        .cs_n  (sdram_cs_n),
        .dm0   (sdram_dm[0]),
        .dm1   (sdram_dm[1]),
        .dm2   (sdram_dm[2]),
        .dm3   (sdram_dm[3]),
        .cke   (sdram_cke)
    );

    // ---- box snapshot into the pixel domain (toggle handshake, 2 flops) ----
    reg [2:0]              ov_tog_s;
    reg [NUM_BLOBS-1:0]    ov_found_s1, ov_found_s2;
    reg [NUM_BLOBS-1:0]    ov_alarm_s1, ov_alarm_s2;
    reg [NUM_BLOBS*9-1:0]  ov_x_s1,  ov_x_s2;
    reg [NUM_BLOBS*9-1:0]  ov_y_s1,  ov_y_s2;
    reg [NUM_BLOBS*9-1:0]  ov_w_s1,  ov_w_s2;
    reg [NUM_BLOBS*9-1:0]  ov_h_s1,  ov_h_s2;
    reg [NUM_BLOBS*7-1:0]  ov_score_s1, ov_score_s2;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            ov_tog_s   <= 3'b000;
            ov_found_s1 <= {NUM_BLOBS{1'b0}}; ov_found_s2 <= {NUM_BLOBS{1'b0}};
            ov_alarm_s1 <= {NUM_BLOBS{1'b0}}; ov_alarm_s2 <= {NUM_BLOBS{1'b0}};
            ov_x_s1 <= {(NUM_BLOBS*9){1'b0}}; ov_x_s2 <= {(NUM_BLOBS*9){1'b0}};
            ov_y_s1 <= {(NUM_BLOBS*9){1'b0}}; ov_y_s2 <= {(NUM_BLOBS*9){1'b0}};
            ov_w_s1 <= {(NUM_BLOBS*9){1'b0}}; ov_w_s2 <= {(NUM_BLOBS*9){1'b0}};
            ov_h_s1 <= {(NUM_BLOBS*9){1'b0}}; ov_h_s2 <= {(NUM_BLOBS*9){1'b0}};
            ov_score_s1 <= {(NUM_BLOBS*7){1'b0}};
            ov_score_s2 <= {(NUM_BLOBS*7){1'b0}};
        end else begin
            ov_tog_s    <= {ov_tog_s[1:0], snap_tog};
            ov_found_s1 <= s_found; ov_found_s2 <= ov_found_s1;
            ov_alarm_s1 <= s_alarm; ov_alarm_s2 <= ov_alarm_s1;
            ov_x_s1 <= s_x;  ov_x_s2 <= ov_x_s1;
            ov_y_s1 <= s_y;  ov_y_s2 <= ov_y_s1;
            ov_w_s1 <= s_w;  ov_w_s2 <= ov_w_s1;
            ov_h_s1 <= s_h;  ov_h_s2 <= ov_h_s1;
            ov_score_s1 <= s_score; ov_score_s2 <= ov_score_s1;
        end
    end

    wire ov_load = ov_tog_s[2] ^ ov_tog_s[1];

    reg [NUM_BLOBS-1:0]    ov_found, ov_alarm_lat;
    reg [NUM_BLOBS*9-1:0]  ov_x, ov_y, ov_w, ov_h;
    reg [NUM_BLOBS*7-1:0]  ov_score;
    // Red alarm latch, counted in pixel clocks.  0x0FFFFFFF at 25.2 MHz is
    // ~10.6 s -- matched to the tracker's 300-frame lock-on hold so a held
    // box stays red for as long as it is drawn.
    reg [27:0]             ov_hold;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            ov_found     <= {NUM_BLOBS{1'b0}};
            ov_alarm_lat <= {NUM_BLOBS{1'b0}};
            ov_x <= {(NUM_BLOBS*9){1'b0}};
            ov_y <= {(NUM_BLOBS*9){1'b0}};
            ov_w <= {(NUM_BLOBS*9){1'b0}};
            ov_h <= {(NUM_BLOBS*9){1'b0}};
            ov_score <= {(NUM_BLOBS*7){1'b0}};
            ov_hold <= 26'd0;
        end else begin
            if (ov_load) begin
                ov_found <= ov_found_s2;
                ov_x     <= ov_x_s2;
                ov_y     <= ov_y_s2;
                ov_w     <= ov_w_s2;
                ov_h     <= ov_h_s2;
                ov_score <= ov_score_s2;
                if (|ov_alarm_s2) begin
                    ov_alarm_lat <= ov_alarm_lat | ov_alarm_s2;
                    ov_hold      <= 28'h0FFFFFFF;
                end
            end

            if (ov_hold != 28'd0)
                ov_hold <= ov_hold - 28'd1;
            else
                ov_alarm_lat <= {NUM_BLOBS{1'b0}};
        end
    end

    // ---- whole-person figure snapshot into the pixel domain ----
    // motion_hist publishes its grown boxes a few thousand clocks after
    // bbox_valid, then the payload sits still for the rest of the frame.  The
    // pixel domain therefore does not need an event to latch it: three flops
    // per bit deliver the newest figure on their own, and nobody has to get a
    // toggle edge and a 38-bit payload to agree on which cycle they crossed
    // the clock boundary.  The alarm latch above is reused; its slot numbers
    // are the same ones motion_hist carried through.
    reg [NUM_BLOBS-1:0]    p_found, p_dir;
    reg [NUM_BLOBS*9-1:0]  p_x, p_y, p_w, p_h;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            p_found <= {NUM_BLOBS{1'b0}};
            p_dir   <= {NUM_BLOBS{1'b0}};
            p_x     <= {(NUM_BLOBS*9){1'b0}};
            p_y     <= {(NUM_BLOBS*9){1'b0}};
            p_w     <= {(NUM_BLOBS*9){1'b0}};
            p_h     <= {(NUM_BLOBS*9){1'b0}};
        end else if (pbox_valid) begin
            p_found <= pbox_found;
            p_dir   <= pbox_dir;
            p_x     <= pbox_x;
            p_y     <= pbox_y;
            p_w     <= pbox_w;
            p_h     <= pbox_h;
        end
    end

    reg [NUM_BLOBS-1:0]   ovp_found_s1, ovp_found_s2, ovp_found_s3;
    reg [NUM_BLOBS-1:0]   ovp_dir_s1,   ovp_dir_s2,   ovp_dir_s3;
    reg [NUM_BLOBS*9-1:0] ovp_x_s1, ovp_x_s2, ovp_x_s3;
    reg [NUM_BLOBS*9-1:0] ovp_y_s1, ovp_y_s2, ovp_y_s3;
    reg [NUM_BLOBS*9-1:0] ovp_w_s1, ovp_w_s2, ovp_w_s3;
    reg [NUM_BLOBS*9-1:0] ovp_h_s1, ovp_h_s2, ovp_h_s3;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            ovp_found_s1 <= {NUM_BLOBS{1'b0}}; ovp_found_s2 <= {NUM_BLOBS{1'b0}}; ovp_found_s3 <= {NUM_BLOBS{1'b0}};
            ovp_dir_s1   <= {NUM_BLOBS{1'b0}}; ovp_dir_s2   <= {NUM_BLOBS{1'b0}}; ovp_dir_s3   <= {NUM_BLOBS{1'b0}};
            ovp_x_s1 <= {(NUM_BLOBS*9){1'b0}}; ovp_x_s2 <= {(NUM_BLOBS*9){1'b0}}; ovp_x_s3 <= {(NUM_BLOBS*9){1'b0}};
            ovp_y_s1 <= {(NUM_BLOBS*9){1'b0}}; ovp_y_s2 <= {(NUM_BLOBS*9){1'b0}}; ovp_y_s3 <= {(NUM_BLOBS*9){1'b0}};
            ovp_w_s1 <= {(NUM_BLOBS*9){1'b0}}; ovp_w_s2 <= {(NUM_BLOBS*9){1'b0}}; ovp_w_s3 <= {(NUM_BLOBS*9){1'b0}};
            ovp_h_s1 <= {(NUM_BLOBS*9){1'b0}}; ovp_h_s2 <= {(NUM_BLOBS*9){1'b0}}; ovp_h_s3 <= {(NUM_BLOBS*9){1'b0}};
        end else begin
            ovp_found_s1 <= p_found; ovp_found_s2 <= ovp_found_s1; ovp_found_s3 <= ovp_found_s2;
            ovp_dir_s1   <= p_dir;   ovp_dir_s2   <= ovp_dir_s1;   ovp_dir_s3   <= ovp_dir_s2;
            ovp_x_s1 <= p_x; ovp_x_s2 <= ovp_x_s1; ovp_x_s3 <= ovp_x_s2;
            ovp_y_s1 <= p_y; ovp_y_s2 <= ovp_y_s1; ovp_y_s3 <= ovp_y_s2;
            ovp_w_s1 <= p_w; ovp_w_s2 <= ovp_w_s1; ovp_w_s3 <= ovp_w_s2;
            ovp_h_s1 <= p_h; ovp_h_s2 <= ovp_h_s1; ovp_h_s3 <= ovp_h_s2;
        end
    end

    wire [NUM_BLOBS-1:0]   ovp_found = ovp_found_s3;
    wire [NUM_BLOBS-1:0]   ovp_dir   = ovp_dir_s3;
    wire [NUM_BLOBS*9-1:0] ovp_x = ovp_x_s3;
    wire [NUM_BLOBS*9-1:0] ovp_y = ovp_y_s3;
    wire [NUM_BLOBS*9-1:0] ovp_w = ovp_w_s3;
    wire [NUM_BLOBS*9-1:0] ovp_h = ovp_h_s3;

    box_overlay #(
        .NB (NUM_BLOBS)
    ) u_overlay (
        .de         (de_d),
        .px         (px_d),
        .py         (py_d),
        .rgb_in     (fs_pix_rgb),
        .found      (ov_found),
        .alarm      (ov_alarm_lat),
        .bx         (ov_x),
        .by         (ov_y),
        .bw         (ov_w),
        .bh         (ov_h),
        .score      (ov_score),
        .rgb_out    (ov_rgb)
    );

    // ---- diagnostic tap: raw motion_hist publish, held for the UART burst
    // (only used when EMIT_PBOX=1; the registers cost a handful of flops)
    reg                    pbv_r;
    reg [NUM_BLOBS-1:0]    pbf_r, pbd_r;
    reg [NUM_BLOBS*9-1:0]  pbx_r, pby_r, pbw_r, pbh_r;
    reg [NUM_BLOBS*13-1:0] pbc_r;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            pbv_r <= 1'b0;
            pbf_r <= {NUM_BLOBS{1'b0}};
            pbd_r <= {NUM_BLOBS{1'b0}};
            pbx_r <= {(NUM_BLOBS*9){1'b0}};
            pby_r <= {(NUM_BLOBS*9){1'b0}};
            pbw_r <= {(NUM_BLOBS*9){1'b0}};
            pbh_r <= {(NUM_BLOBS*9){1'b0}};
            pbc_r <= {(NUM_BLOBS*13){1'b0}};
        end else if (pbox_valid) begin
            pbv_r <= 1'b1;
            pbf_r <= pbox_found;
            pbd_r <= pbox_dir;
            pbx_r <= pbox_x;
            pby_r <= pbox_y;
            pbw_r <= pbox_w;
            pbh_r <= pbox_h;
            pbc_r <= pbox_cells;
        end
    end

    // {published at least once, the end-of-frame pass saw a motion cell
    // inside a window, the sweep read a live trail cell}.  All three are
    // sticky inside motion_hist / this block, so they can be sampled as they
    // are: the PBOX build prints them in the S line's state digit.
    wire [2:0] pb_flags = {pbv_r, pbox_pass_hit, pbox_sweep_hit};

    // RST_I inside the DVITransmitter is active high, which is why the vendor
    // 640x480 example drives RST_N with the inverted reset.
    wire hdmi_rst = ~rst_n;

    hdmi_tx #(
        .FAMILY ("EG4")
    ) u_hdmi_tx (
        .PXLCLK_I    (clk_pixel),
        .PXLCLK_5X_I (clk_pixel_5x),
        .RST_N       (hdmi_rst),
        .VGA_HS      (hsync_d),
        .VGA_VS      (vsync_d),
        .VGA_DE      (de_d),
        .VGA_RGB     (ov_rgb),
        .HDMI_CLK_P  (HDMI_CLK_P),
        .HDMI_D2_P   (HDMI_D2_P),
        .HDMI_D1_P   (HDMI_D1_P),
        .HDMI_D0_P   (HDMI_D0_P)
    );

    // ------------------------------------------------------------ status
    reg [23:0] hb_cnt;
    reg        hb;
    always @(posedge clk_24m or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt <= 24'd0;
            hb     <= 1'b0;
        end else if (hb_cnt == 24'd12000000 - 24'd1) begin
            hb_cnt <= 24'd0;
            hb     <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 24'd1;
        end
    end

    wire [NUM_BLOBS-1:0] fall_slots;
    genvar gi;
    generate
        for (gi = 0; gi < NUM_BLOBS; gi = gi + 1) begin : g_fall
            assign fall_slots[gi] = (u_state[gi*3 +: 3] == 3'd2);
        end
    endgenerate

    wire [7:0] led_data = {
        (act_cnt != 24'd0),        // LED7 frames arriving
        (hw_err != 8'd0),          // LED6 stream geometry error
        (alarm_hold != 26'd0),     // LED5 alarm latch
        (|fall_slots),             // LED4 any slot in FALL
        (|u_found),                // LED3 any slot found
        hb,                        // LED2 heartbeat
        armed_24m,                 // LED1 streaming
        cfg_done                   // LED0 configuration done
    };

    led595_shift u_led (
        .clk     (clk_24m),
        .rst_n   (rst_n),
        .data    (led_data),
        .clk_out (led595_clk),
        .latch   (led595_latch),
        .dout    (led595_dout)
    );

    // ------------------------------------------------ status lines (UART)
    wire        uart_trig;
    wire [1:0]  uart_slot;
    wire [15:0] uart_frame;
    wire        uart_found;
    wire [8:0]  uart_x, uart_y, uart_w, uart_h;
    wire [16:0] uart_area;
    wire [2:0]  uart_state;
    wire        uart_ready;
    wire        scan_busy;

    slot_scan #(
        .NSLOT    (NUM_BLOBS),
        .USE_PBOX (EMIT_PBOX)
    ) u_scan (
        .clk     (clk_24m),
        .rst_n   (rst_n),
        .snap    (snap_pulse),
        .ready   (uart_ready),
        .frame   (u_frame),
        .found   (u_found),
        .state   (u_state),
        .x       (u_x),
        .y       (u_y),
        .w       (u_w),
        .h       (u_h),
        .area    (u_area),
        .pb_found (pbf_r),
        .pb_x     (pbx_r),
        .pb_y     (pby_r),
        .pb_w     (pbw_r),
        .pb_h     (pbh_r),
        .pb_cells (pbc_r),
        .pb_flags (pb_flags),
        .trig    (uart_trig),
        .slot    (uart_slot),
        .o_frame (uart_frame),
        .o_found (uart_found),
        .o_x     (uart_x),
        .o_y     (uart_y),
        .o_w     (uart_w),
        .o_h     (uart_h),
        .o_area  (uart_area),
        .o_state (uart_state),
        .busy    (scan_busy)
    );

    status_uart #(
        .CLK_HZ    (24000000),
        .BAUD      (BAUD),
        .SHOW_SLOT (1),
        .SHOW_AE   (1)
    ) u_uart (
        .clk       (clk_24m),
        .rst_n     (rst_n),
        .trig      (uart_trig),
        .slot      (uart_slot),
        .frame_cnt (uart_frame),
        .found     (uart_found),
        .bx        (uart_x),
        .by        (uart_y),
        .bw        (uart_w),
        .bh        (uart_h),
        .area      (uart_area),
        .state     (uart_state),
        .ae_mean   (u_ae_mean),
        .ae_shutter(u_ae_shut),
        .ae_gain   (u_ae_gain),
        .ae_zone   (u_ae_zone),
        .tx        (uart_txd),
        .ready     (uart_ready)
    );

endmodule
