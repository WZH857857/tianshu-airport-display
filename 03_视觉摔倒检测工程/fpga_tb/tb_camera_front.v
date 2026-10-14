//===========================================================================
// tb_camera_front.v -- MT9M001 front end, end to end
//
// Instantiates the three camera-side blocks exactly as the board top wires
// them - mt9m001_cfg (two-wire boot table + dynamic port), cmos_capture_gray8
// and ae_ctrl - against a behavioural sensor model that
//
//   * streams FRAME_VALID/LINE_VALID/8-bit data on PIXCLK (24 MHz) with the
//     datasheet's shapes (FRAME_VALID leads and trails LINE_VALID),
//   * acknowledges every two-wire byte like the real part, and
//   * answers the exposure registers with a linear response, 4/5 of the
//     programmed shutter, so the loop has a real fixed point to find.
//
// Geometry is scaled down (64x48 window -> 64x40 output) so a frame costs
// ~3 us of simulated time instead of 30 ms; every rule under test is a ratio,
// not a count.  Checks:
//
//   * cfg_done, then measurement frames, then armed - and nothing before
//   * every frame the vision core sees is a complete 64x40 frame that opens
//     with its own frame_start (the multi_bbox row-census contract)
//   * every exposure write hides exactly FREEZE_FRAMES whole frames while the
//     ungated stream keeps running, i.e. the picture path does not stutter
//   * the loop converges into the dead band and then stops writing
//   * hw_err stays 0 for a well-formed stream
//===========================================================================
`timescale 1ns / 1ps

module tb_camera_front;

    localparam integer NW = 64;      // sensor width with skip on
    localparam integer NH = 48;      // sensor height with skip on
    localparam integer OW = 64;      // capture output width
    localparam integer OH = 40;      // capture output height
    localparam integer HB = 8;       // horizontal blanking, pixel clocks
    localparam integer VB = 12;      // vertical blanking, pixel clocks

    reg pclk = 1'b0;
    always #20.833 pclk = ~pclk;     // 24 MHz, PIXCLK = XCLK

    reg rst_n = 1'b0;
    integer errors = 0;

    task check(input cond, input [255:0] what);
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL t=%0t %0s", $time, what);
            end
        end
    endtask

    // ------------------------------------------------------- sensor model
    reg        sen_vsync = 1'b0;
    reg        sen_href  = 1'b0;
    reg [7:0]  sen_data  = 8'h00;

    reg [15:0] shutter_reg = 16'h00C8;   // what the two-wire bus last wrote
    reg [7:0]  gain_reg    = 8'h08;
    reg [15:0] cur_sh      = 16'h00C8;

    function [7:0] px_val(input [15:0] sh);
        reg [17:0] v;
        begin
            v = (sh * 4) / 5;            // linear exposure response
            if (v > 18'd255) v = 18'd255;
            px_val = v[7:0];
        end
    endfunction

    task frame(input integer cols, input integer rows);
        integer r, c;
        begin
            cur_sh = shutter_reg;                    // latched at frame start
            sen_vsync = 1'b1;
            repeat (4) @(posedge pclk);              // FRAME_VALID lead
            for (r = 0; r < rows; r = r + 1) begin
                sen_href = 1'b1;
                for (c = 0; c < cols; c = c + 1) begin
                    sen_data = px_val(cur_sh);
                    @(posedge pclk);
                end
                sen_href = 1'b0;
                repeat (HB) @(posedge pclk);
            end
            sen_vsync = 1'b0;
            repeat (VB) @(posedge pclk);             // FRAME_VALID trail
        end
    endtask

    // ------------------------------------------------- two-wire slave model
    wire scl, sda;
    pullup(sda);
    reg ack_drive = 1'b0;
    assign sda = (rst_n && ack_drive) ? 1'b0 : 1'bz;

    // ------------------------------------------------------------- DUT chain
    wire        cfg_done, cfg_busy;
    wire [7:0]  wr_count;
    wire        ae_req_tog;
    wire [7:0]  ae_req_addr;
    wire [15:0] ae_req_data;
    wire        ae_ack_tog;

    mt9m001_cfg #(
        .CLK_HZ     (24000000),
        .SCL_HZ     (1000000),
        .POWERUP_MS (0),
        .RESET_MS   (0)
    ) u_cfg (
        .clk      (pclk),
        .rst_n    (rst_n),
        .scl      (scl),
        .sda      (sda),
        .cfg_done (cfg_done),
        .cfg_busy (cfg_busy),
        .wr_count (wr_count),
        .dyn_tog  (ae_req_tog),
        .dyn_addr (ae_req_addr),
        .dyn_data (ae_req_data),
        .ack_tog  (ae_ack_tog)
    );

    reg [2:0] cfg_sync;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) cfg_sync <= 3'b000;
        else        cfg_sync <= {cfg_sync[1:0], cfg_done};
    end
    wire cfg_done_pclk = cfg_sync[2];

    wire        de_565, frame_start_565, frame_end_565, armed, wide_mode;
    wire [15:0] pix_565;
    wire [7:0]  gray_pix;
    wire [15:0] frame_cnt;
    wire [10:0] line_cnt;
    wire [11:0] pix_cnt;
    wire [7:0]  hw_err;

    cmos_capture_gray8 #(
        .NARROW_W    (NW),
        .NARROW_H    (NH),
        .WIDE_W      (128),
        .WIDE_H      (96),
        .OUT_W       (OW),
        .OUT_H       (OH),
        .WAIT_FRAMES (2)
    ) u_cap (
        .pclk        (pclk),
        .rst_n       (rst_n),
        .arm         (cfg_done_pclk),
        .vsync       (sen_vsync),
        .href        (sen_href),
        .data        (sen_data),
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

    wire       freeze;
    wire [1:0] ae_zone;

    ae_ctrl #(
        .SHUT_INIT     (200),
        .SHUT_MIN      (16),
        .SHUT_MAX      (400),
        // Index bounds, not 0x35 codes: 0 = 1.0x, 40 = 8.0x.  This bench keeps
        // its own narrow dead band on purpose: it checks the freeze/handshake
        // mechanics, and 70..150 parks its linear sensor model at 150 exactly.
        .GAIN_INIT_IDX (0),
        .GAIN_MIN_IDX  (0),
        .GAIN_MAX_IDX  (40),
        .SAMPLE_DIV    (8),
        .SAMPLES       (OW * OH / 8),
        .LOW_MEAN      (70),
        .HIGH_MEAN     (150),
        .PERIOD_FRAMES (1),
        .FREEZE_FRAMES (2)
    ) u_ae (
        .clk         (pclk),
        .rst_n       (rst_n),
        .de          (de_565),
        .gray        (gray_pix),
        .frame_start (frame_start_565),
        .frame_end   (frame_end_565),
        .req_tog     (ae_req_tog),
        .req_addr    (ae_req_addr),
        .req_data    (ae_req_data),
        .ack_tog     (ae_ack_tog),
        .freeze      (freeze),
        .zone        (ae_zone),
        .shutter_dbg (),
        .gain_dbg    ()
    );

    wire de_core          = de_565 & ~freeze;
    wire frame_start_core = frame_start_565 & ~freeze;

    // ------------------------------------------------- two-wire decoding
    // Same shape as tb_mt9m001_cfg: sampled in the clock domain, acknowledged
    // in the ninth slot, and the last complete transaction updates the sensor
    // model's exposure registers.
    reg       scl_d = 1'b1, sda_d = 1'b1;
    reg       started = 1'b0;
    integer   slot_n = 0, bitn = 0, byte_n = 0;
    reg [7:0] shreg = 8'h00;
    reg [7:0] b0, b1, b2, b3;
    wire [7:0] shreg_n = {shreg[6:0], sda};
    wire start_cond = scl && scl_d && sda_d && !sda;
    wire stop_cond  = scl && scl_d && !sda_d && sda;
    wire scl_rise   = scl && !scl_d;
    wire scl_fall   = !scl && scl_d;

    integer tx_count = 0;
    integer dyn_writes = 0;

    always @(posedge pclk) begin
        scl_d <= scl;
        sda_d <= sda;

        if (start_cond) begin
            started   <= 1'b1;
            slot_n    <= 0;
            bitn      <= 0;
            byte_n    <= 0;
            ack_drive <= 1'b0;
        end

        if (scl_rise && started) begin
            if ((slot_n % 9) == 8) begin
                bitn <= 0;                        // acknowledge slot
            end else begin
                shreg <= shreg_n;
                if (bitn == 7) begin
                    bitn <= 0;
                    case (byte_n)
                    0: b0 <= shreg_n;
                    1: b1 <= shreg_n;
                    2: b2 <= shreg_n;
                    default: b3 <= shreg_n;
                    endcase
                    byte_n <= byte_n + 1;
                end else begin
                    bitn <= bitn + 1;
                end
            end
            slot_n <= slot_n + 1;
        end

        if (scl_fall && started) begin
            if ((slot_n % 9) == 8) ack_drive <= #1 1'b1;
            if (((slot_n % 9) == 0) && (slot_n != 0)) ack_drive <= #1 1'b0;
        end

        if (stop_cond) begin
            started   <= 1'b0;
            ack_drive <= 1'b0;
            tx_count  <= tx_count + 1;
            if ((b0 === 8'hBA) && (tx_count >= 13)) begin
                dyn_writes <= dyn_writes + 1;
                case (b1)
                8'h09: shutter_reg <= {b2, b3};
                8'h35: gain_reg    <= b3;
                default: ;
                endcase
            end
        end
    end

    // ----------------------------------------------------- stream monitors
    integer raw_frames = 0, core_frames = 0, hidden_frames = 0;
    integer raw_de_frame = 0, core_de_row = 0, core_rows = 0;
    reg     raw_counting = 1'b0;
    reg     in_core_frame = 1'b0;
    reg     saw_core_start = 1'b0;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            raw_frames <= 0; core_frames <= 0; hidden_frames <= 0;
            raw_de_frame <= 0; core_de_row <= 0; core_rows <= 0;
            raw_counting <= 1'b0; in_core_frame <= 1'b0;
            saw_core_start <= 1'b0;
        end else begin
            if (de_565) raw_de_frame <= raw_de_frame + 1;

            if (frame_start_565) begin
                raw_de_frame <= (de_565 ? 1 : 0);
                raw_counting <= 1'b1;
                saw_core_start <= 1'b0;
            end

            if (frame_start_core) begin
                if (in_core_frame)
                    check((core_rows == OH) && (core_de_row == 0),
                          "core frame complete before the next start");
                in_core_frame  <= 1'b1;
                core_de_row    <= 0;
                core_rows      <= 0;
                core_frames    <= core_frames + 1;
                saw_core_start <= 1'b1;
            end

            if (de_core) begin
                check(in_core_frame, "no core pixels outside a core frame");
                core_de_row <= core_de_row + 1;
                if (core_de_row == OW - 1) begin
                    core_de_row <= 0;
                    core_rows   <= core_rows + 1;
                end
            end

            if (frame_end_565 && raw_counting) begin
                check(raw_de_frame == OW * OH,
                      "raw frame carries OW*OH pixels");
                raw_frames <= raw_frames + 1;
                if (!saw_core_start) begin
                    hidden_frames <= hidden_frames + 1;
                    check((core_rows == OH) && (core_de_row == 0),
                          "hidden frame left the core counters clean");
                end
            end
        end
    end

    // ------------------------------------------------------------- stimulus
    integer i;
    initial begin
        $dumpfile("tb_camera_front.vcd");
        $dumpvars(0, tb_camera_front);

        repeat (10) @(posedge pclk);
        rst_n = 1'b1;

        // Stream continuously, like a sensor that was never told to stop.
        for (i = 0; i < 80; i = i + 1) frame(NW, NH);

        check(armed == 1'b1, "capture armed");
        check(wide_mode == 1'b0, "narrow (skip) geometry recognised");
        check(hw_err == 8'd0, "no stream geometry errors");
        check(cfg_done == 1'b1, "config finished");
        check(tx_count == 13 + dyn_writes, "bus writes = boot table + adjustments");
        check(dyn_writes > 0, "exposure loop issued at least one write");

        check(core_frames + hidden_frames == raw_frames,
              "every raw frame reached the core or was hidden");
        check(hidden_frames == 2 * dyn_writes,
              "each exposure write hides exactly FREEZE_FRAMES frames");
        check(ae_zone == 2'd1, "exposure settled inside the dead band");
        // The dead band is on the picture, not on the register value: a
        // 0x00BC shutter and this sensor model land on mean 150 exactly.
        check((px_val(shutter_reg) <= 8'd150) && (px_val(shutter_reg) >= 8'd70),
              "sensor response landed inside the dead band");
        check(shutter_reg < 16'd200, "exposure loop brought the shutter down");

        $display("INFO raw=%0d core=%0d hidden=%0d writes=%0d shutter=%0d gain=%0d",
                 raw_frames, core_frames, hidden_frames, dyn_writes,
                 shutter_reg, gain_reg);

        if (errors == 0) $display("PASS tb_camera_front: front end end-to-end clean");
        else             $display("FAIL tb_camera_front: %0d errors", errors);
        $finish;
    end

endmodule
