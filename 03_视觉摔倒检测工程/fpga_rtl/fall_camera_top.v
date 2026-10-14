//===========================================================================
// fall_camera_top.v -- VF-E20NG88 board top: OV5640 -> fall detector -> LEDs
//
//   clk_24m (P13) ─┬─ cmos_xclk (P72) ──> OV5640
//                  ├─ SCCB 250 kHz (P70/P71) configures 640x480 RGB565
//                  ├─ 74HC595 status LEDs (P59/P57/P45)
//                  └─ 115200 UART status (P3)
//   cmos_pclk (P79) ── capture ── fall_core_top ── bbox/alarm
//
// Everything is inside the FPGA: no MCU, no soft core, no external frame
// buffer.  The only external memory in the design is the two on-chip ERAM
// regions the vision core already uses (see fpga/README.md).
//
// Clock domains
//   clk_24m   : SCCB sequencer, LED shift register, UART formatter
//   cmos_pclk : capture, gray_front -> frame_diff -> motion_morph ->
//               proj_bbox -> posture_fsm (48 MHz OV5640 PCLK)
// The only crossings are cfg_done (level, 24M -> pclk) and three toggles
// (bbox_valid, alarm, frame activity) plus a status snapshot (pclk -> 24M).
//
// LED map (LED0 = bit0):  0 cfg_done, 1 armed, 2 1 Hz heartbeat,
//   3 last bbox_found, 4 posture FALL, 5 alarm latch (2 s), 6 hw_err,
//   7 frames arriving.
//
// UART map: on every bounding-box observation one ASCII line
//   F<frame:4> <found:1> <x:3> <y:3> <w:3> <h:3> <area:5> <state:1>\r\n
//===========================================================================
`timescale 1ns / 1ps

module fall_camera_top #(
    parameter integer POWERUP_MS  = 20,
    parameter integer RESET_MS    = 2,
    parameter integer WAIT_FRAMES = 10,
    parameter integer SCCB_HZ     = 250000,
    parameter integer BAUD        = 115200,
    // Power-on/key-release hold in 24 MHz clocks.  1048575 = 20'hFFFFF,
    // i.e. ~44 ms on the board; benches shorten it.
    parameter integer POR_CYCLES  = 1048575
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

    // KEY0 is active low with a board pull-up.  por_cnt holds the whole
    // design in reset for ~44 ms after configuration, rel_cnt de-bounces the
    // key release, and both together give a clean rst_n.
    //
    // The three registers below carry explicit power-up values.  On the
    // device the FPGA configuration already loads FFs with 0, so this only
    // states that dependency instead of hiding it; in RTL simulation it is
    // what makes rst_n leave X and release the design.
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

    // Registered so the async reset used by both clock domains is glitch free
    // (por_done and rel_cnt reach their limit on the same 24 MHz edge).
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

    // Release the vision core reset 16 pclk cycles after the board reset ends,
    // i.e. before cfg_done/armed, so the core is already running when the
    // first armed frame raises frame_start.  It sees no pixels until then
    // because the capture module gates de on armed.
    reg [3:0] core_rel_cnt;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)                      core_rel_cnt <= 4'd0;
        else if (core_rel_cnt != 4'hF)   core_rel_cnt <= core_rel_cnt + 4'd1;
    end
    wire core_rst_n = (core_rel_cnt == 4'hF);

    // -------------------------------------------------- vision core (pclk)
    wire        bbox_valid, bbox_found, alarm;
    wire [8:0]  bbox_x, bbox_y, bbox_w, bbox_h;
    wire [16:0] bbox_area;
    wire [2:0]  posture_state;

    fall_core_top u_core (
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
    reg        s_found;
    reg [2:0]  s_state;
    reg [15:0] s_frame;
    reg [8:0]  s_x, s_y, s_w, s_h;
    reg [16:0] s_area;
    reg        snap_tog;
    reg        alarm_tog;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            s_found   <= 1'b0;
            s_state   <= 3'd0;
            s_frame   <= 16'd0;
            s_x       <= 9'd0;
            s_y       <= 9'd0;
            s_w       <= 9'd0;
            s_h       <= 9'd0;
            s_area    <= 17'd0;
            snap_tog  <= 1'b0;
            alarm_tog <= 1'b0;
        end else begin
            if (bbox_valid) begin
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
            if (alarm) alarm_tog <= ~alarm_tog;
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

    reg        u_found;
    reg [2:0]  u_state;
    reg [15:0] u_frame;
    reg [8:0]  u_x, u_y, u_w, u_h;
    reg [16:0] u_area;
    reg [25:0] alarm_hold;
    reg [23:0] act_cnt;

    always @(posedge clk_24m or negedge rst_n) begin
        if (!rst_n) begin
            u_found    <= 1'b0;
            u_state    <= 3'd0;
            u_frame    <= 16'd0;
            u_x        <= 9'd0;
            u_y        <= 9'd0;
            u_w        <= 9'd0;
            u_h        <= 9'd0;
            u_area     <= 17'd0;
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
                act_cnt <= 24'hFFFFFF;          // ~0.7 s of frame activity
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

    wire [7:0] led_data = {
        (act_cnt != 24'd0),        // LED7 frames arriving
        (hw_err != 8'd0),          // LED6 stream geometry error
        (alarm_hold != 26'd0),     // LED5 alarm latch
        (u_state == 3'd2),         // LED4 posture FALL
        u_found,                   // LED3 last bbox found
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

    status_uart #(
        .CLK_HZ (24000000),
        .BAUD   (BAUD)
    ) u_uart (
        .clk       (clk_24m),
        .rst_n     (rst_n),
        .trig      (snap_pulse),
        .slot      (2'd0),
        .frame_cnt (u_frame),
        .found     (u_found),
        .bx        (u_x),
        .by        (u_y),
        .bw        (u_w),
        .bh        (u_h),
        .area      (u_area),
        .state     (u_state),
        .tx        (uart_txd)
    );

endmodule
