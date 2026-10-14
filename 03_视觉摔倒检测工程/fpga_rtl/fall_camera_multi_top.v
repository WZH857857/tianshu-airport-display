//===========================================================================
// fall_camera_multi_top.v -- VF-E20NG88 board top, crowd variant
//
// Same board, same pinout, same camera front end as fall_camera_top.v; the
// only difference is the vision core.  Where the solo build runs
// proj_bbox + one posture machine, this one runs multi_bbox (4 exclusion
// passes) + 4 posture machines, so a frame with four movers publishes four
// boxes instead of one.
//
//   clk_24m (P13) ─┬─ cmos_xclk (P72) ──> OV5640
//                  ├─ SCCB 250 kHz (P70/P71) configures 640x480 RGB565
//                  ├─ 74HC595 status LEDs (P59/P57/P45)
//                  └─ 115200 UART status (P3)
//   cmos_pclk (P79) ── capture ── fall_core_multi_top ── 4 boxes/alarms
//
// Everything is inside the FPGA: no MCU, no soft core, no external frame
// buffer.  The capture, SCCB, LED and UART blocks are the verified solo
// blocks; the new logic is the 4-slot snapshot, the slot_scan walker and the
// slot prefix in status_uart.
//
// LED map (LED0 = bit0):  0 cfg_done, 1 armed, 2 1 Hz heartbeat,
//   3 any slot found, 4 any slot in FALL, 5 alarm latch (2 s), 6 hw_err,
//   7 frames arriving.
//
// UART map: on every frame, four lines in slot order
//   S<slot:1> F<frame:4> <found:1> <x:3> <y:3> <w:3> <h:3> <area:5> <state:1>\r\n
//===========================================================================
`timescale 1ns / 1ps

module fall_camera_multi_top #(
    parameter integer POWERUP_MS  = 20,
    parameter integer RESET_MS    = 2,
    parameter integer WAIT_FRAMES = 10,
    parameter integer SCCB_HZ     = 250000,
    parameter integer BAUD        = 115200,
    parameter integer POR_CYCLES  = 1048575,
    parameter integer NUM_BLOBS   = 4,
    parameter integer MARGIN      = 8
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

    output wire        led595_clk,     // P59
    output wire        led595_latch,   // P57
    output wire        led595_dout,    // P45

    output wire        uart_txd        // P3
);

    // ------------------------------------------------------- clock / reset
    assign cmos_xclk = clk_24m;

    reg [1:0]  key_sync;
    localparam integer POR_END = (POR_CYCLES < 2) ? 2 : POR_CYCLES;
    reg [19:0] por_cnt;
    reg [19:0] rel_cnt;
    reg        rst_n_r;

    initial begin
        key_sync = 2'b00;
        por_cnt  = 20'd0;
        rel_cnt  = 20'd0;
        rst_n_r  = 1'b0;
    end

    wire       key_level = key_sync[1];
    wire       por_done  = (por_cnt >= POR_END - 1);
    wire       rel_done  = (rel_cnt >= POR_END - 1);

    always @(posedge clk_24m) begin
        key_sync <= {key_sync[0], key_n};
        if (!por_done)                 por_cnt <= por_cnt + 20'd1;
        if (!key_level)                rel_cnt <= 20'd0;
        else if (!rel_done)            rel_cnt <= rel_cnt + 20'd1;
    end

    always @(posedge clk_24m)
        rst_n_r <= por_done & rel_done;
    wire rst_n = rst_n_r;

    // ----------------------------------------------------- SCCB (24 MHz)
    wire       cfg_done;
    wire       cfg_busy;
    wire [7:0] wr_count;

    ov5640_sccb_cfg #(
        .CLK_HZ     (24000000),
        .SCCB_HZ    (SCCB_HZ),
        .POWERUP_MS (POWERUP_MS),
        .RESET_MS   (RESET_MS)
    ) u_cfg (
        .clk      (clk_24m),
        .rst_n    (rst_n),
        .scl      (cmos_sclk),
        .sda      (cmos_sdat),
        .cfg_done (cfg_done),
        .cfg_busy (cfg_busy),
        .wr_count (wr_count)
    );

    // ------------------------------------------- cfg_done into pclk domain
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
    wire [9:0]  line_cnt;
    wire [10:0] pix_cnt;
    wire [7:0]  hw_err;

    cmos_capture_rgb565 #(
        .H_ACTIVE    (640),
        .V_ACTIVE    (480),
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
        .frame_start (frame_start_565),
        .armed       (armed),
        .frame_cnt   (frame_cnt),
        .line_cnt    (line_cnt),
        .pix_cnt     (pix_cnt),
        .hw_err      (hw_err)
    );

    reg [3:0] core_rel_cnt;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)                      core_rel_cnt <= 4'd0;
        else if (core_rel_cnt != 4'hF)   core_rel_cnt <= core_rel_cnt + 4'd1;
    end
    wire core_rst_n = (core_rel_cnt == 4'hF);

    // ------------------------------------------- crowd vision core (pclk)
    wire                    bbox_valid;
    wire [NUM_BLOBS-1:0]    bbox_found, alarm;
    wire [NUM_BLOBS*9-1:0]  bbox_x, bbox_y, bbox_w, bbox_h;
    wire [NUM_BLOBS*17-1:0] bbox_area;
    wire [NUM_BLOBS*3-1:0]  posture_state;
    wire                    bbox_busy;

    fall_core_multi_top #(
        .NUM_BLOBS (NUM_BLOBS),
        .MARGIN    (MARGIN)
    ) u_core (
        .clk           (cmos_pclk),
        .rst_n         (core_rst_n),
        .frame_start   (frame_start_565),
        .de_in         (de_565),
        .pix_in        (pix_565),
        .bbox_valid    (bbox_valid),
        .bbox_found    (bbox_found),
        .bbox_x        (bbox_x),
        .bbox_y        (bbox_y),
        .bbox_w        (bbox_w),
        .bbox_h        (bbox_h),
        .bbox_area     (bbox_area),
        .posture_state (posture_state),
        .alarm         (alarm)
    );

    // -------------------------------------- pclk -> 24 MHz status snapshot
    // All four slots are latched on the same pulse, so the 24 MHz side always
    // reports one coherent frame, never a mix of two.
    reg [NUM_BLOBS-1:0]    s_found;
    reg [NUM_BLOBS*3-1:0]  s_state;
    reg [15:0]             s_frame;
    reg [NUM_BLOBS*9-1:0]  s_x, s_y, s_w, s_h;
    reg [NUM_BLOBS*17-1:0] s_area;
    reg                    snap_tog;
    reg                    alarm_tog;
    reg                    alarm_d;
    reg [4:0]              snap_delay;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            s_found   <= {NUM_BLOBS{1'b0}};
            s_state   <= {(NUM_BLOBS*3){1'b0}};
            s_frame   <= 16'd0;
            s_x       <= {(NUM_BLOBS*9){1'b0}};
            s_y       <= {(NUM_BLOBS*9){1'b0}};
            s_w       <= {(NUM_BLOBS*9){1'b0}};
            s_h       <= {(NUM_BLOBS*9){1'b0}};
            s_area    <= {(NUM_BLOBS*17){1'b0}};
            snap_tog  <= 1'b0;
            alarm_tog <= 1'b0;
            alarm_d   <= 1'b0;
            snap_delay<= 5'd0;
        end else begin
            // posture_bank sweeps the four posture contexts through one
            // engine, so the state write-back lands ~16 clocks after
            // bbox_valid.  The snapshot waits 24 clocks before latching; the
            // bbox registers hold their values until the next frame, so
            // waiting loses nothing and the 24 MHz side sees the state that
            // belongs to this frame's boxes.
            if (bbox_valid) begin
                snap_delay <= 5'd24;
            end else if (snap_delay != 5'd0) begin
                snap_delay <= snap_delay - 5'd1;
                if (snap_delay == 5'd1) begin
                    s_found  <= bbox_found;
                    s_state  <= posture_state;
                    s_frame  <= frame_cnt;
                    s_x      <= bbox_x;
                    s_y      <= bbox_y;
                    s_w      <= bbox_w;
                    s_h      <= bbox_h;
                    s_area   <= bbox_area;
                    snap_tog <= ~snap_tog;
                end
            end

            // The bank holds the alarm level until the next frame, so the
            // toggle handshake needs an edge rather than a level.
            alarm_d <= |alarm;
            if ((|alarm) && !alarm_d) alarm_tog <= ~alarm_tog;
        end
    end

    reg [2:0] snap_sync;
    reg [2:0] alarm_sync;
    reg [2:0] armed_sync;

    always @(posedge clk_24m or negedge rst_n) begin
        if (!rst_n) begin
            snap_sync  <= 3'b000;
            alarm_sync <= 3'b000;
            armed_sync <= 3'b000;
        end else begin
            snap_sync  <= {snap_sync[1:0],  snap_tog};
            alarm_sync <= {alarm_sync[1:0], alarm_tog};
            armed_sync <= {armed_sync[1:0], armed};
        end
    end

    wire snap_pulse  = snap_sync[2]  ^ snap_sync[1];
    wire alarm_pulse = alarm_sync[2] ^ alarm_sync[1];
    wire armed_24m   = armed_sync[2];

    reg [NUM_BLOBS-1:0]    u_found;
    reg [NUM_BLOBS*3-1:0]  u_state;
    reg [15:0]             u_frame;
    reg [NUM_BLOBS*9-1:0]  u_x, u_y, u_w, u_h;
    reg [NUM_BLOBS*17-1:0] u_area;
    reg [25:0]             alarm_hold;
    reg [23:0]             act_cnt;

    always @(posedge clk_24m or negedge rst_n) begin
        if (!rst_n) begin
            u_found    <= {NUM_BLOBS{1'b0}};
            u_state    <= {(NUM_BLOBS*3){1'b0}};
            u_frame    <= 16'd0;
            u_x        <= {(NUM_BLOBS*9){1'b0}};
            u_y        <= {(NUM_BLOBS*9){1'b0}};
            u_w        <= {(NUM_BLOBS*9){1'b0}};
            u_h        <= {(NUM_BLOBS*9){1'b0}};
            u_area     <= {(NUM_BLOBS*17){1'b0}};
            alarm_hold <= 26'd0;
            act_cnt    <= 24'd0;
        end else begin
            if (snap_pulse) begin
                u_found <= s_found;
                u_state <= s_state;
                u_frame <= s_frame;
                u_x     <= s_x;
                u_y     <= s_y;
                u_w     <= s_w;
                u_h     <= s_h;
                u_area  <= s_area;
                act_cnt <= 24'hFFFFFF;
            end else if (act_cnt != 24'd0) begin
                act_cnt <= act_cnt - 24'd1;
            end

            if (alarm_pulse)              alarm_hold <= 26'h3FFFFFF;
            else if (alarm_hold != 26'd0) alarm_hold <= alarm_hold - 26'd1;
        end
    end

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

    // "any slot in FALL": state 2 is FALL, and the machines only print one
    // FALL code each, so an equality check per slot is enough.
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
        .NSLOT (NUM_BLOBS)
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
        .SHOW_SLOT (1)
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
        .tx        (uart_txd),
        .ready     (uart_ready)
    );

endmodule
