//===========================================================================
// mt9m001_cfg.v -- MT9M001 (ID-003A module) two-wire configuration engine
//
// The board clock is 24 MHz (P13) and XCLK is wired straight to it, so the
// sensor returns PIXCLK = XCLK = 24 MHz (datasheet Table 3: "PIXCLK frequency
// = master clock").
//
// Bus handling is the same shape as ov5640_sccb_cfg.v -- SCL is push-pull,
// SDA is released for the ninth slot of every byte, START/STOP are clean
// levels -- but the MT9M001 frame is: device 0xBA, one *8-bit* register
// address, then a 16-bit big-endian value (datasheet Figure 8).  That is the
// reason this engine cannot be shared with the OV5640 one: the OV5640 takes
// a 16-bit address, and a 16-bit address sent to an MT9M001 would land in
// register 0x00 and corrupt the payload.
//
// Initial table (13 writes):
//
//   0x0D 0x0001  soft reset                     (datasheet / Linux mt9m001.c)
//   0x0D 0x0000  release
//   0x07 0x0000  stop readout while options change
//   0x01 0x000C  row start     = 12
//   0x02 0x0014  column start  = 20
//   0x03 0x03FF  1024 rows  -- halved to 512 by row skip
//   0x04 0x04FF  1280 cols  -- halved to 640 by column skip
//   0x05 0x01F4  horizontal blanking 500 -> 1365-clock row, 32.7 fps
//   0x06 0x0019  vertical blanking 25
//   0x20 0x0018  row skip (bit4) + column skip (bit3)
//   0x35 0x0008  global gain = 1.0x
//   0x09 0x00C8  shutter width 200 rows (ae_ctrl trims this per frame)
//   0x07 0x0002  normal master mode, streaming
//
// Two numbers in there are deliberate:
//
//   * 0x18 puts the sensor in skip mode, so a 1.3 MP part hands the FPGA a
//     640x512 window covering the whole field of view instead of a cropped
//     640x480 one.  The vision core then decimates 2x2 itself and keeps 480
//     of the 512 rows, i.e. exactly the geometry every existing RTL check
//     was pinned down on.
//   * Reg0x05 = 500 costs 8.9 ms of the 30.6 ms frame but buys the four
//     multi_bbox exclusion passes ~407k idle clocks per frame.  At the stock
//     blanking the passes would only get ~148k and would overrun into the
//     next frame's active pixels whenever the motion band is tall.
//
// The table is written once at power-up; cfg_done then rises and stays high.
// Register 0x09 (shutter) and 0x35 (gain) are written again later by
// ae_ctrl through the dynamic port below, which re-uses the same bit engine
// so there is only one place where wire timing lives.
//
// Dynamic port: a toggle handshake.  The producer holds addr/data stable,
// flips dyn_tog, and waits for ack_tog to flip back.  The toggle is
// synchronised here with two flops; the payload is sampled in this domain
// once the toggle has arrived, and the producer keeps it valid until the
// acknowledge, so no multi-bit data crosses the boundary without a
// handshake around it.
//===========================================================================
`timescale 1ns / 1ps

module mt9m001_cfg #(
    parameter integer CLK_HZ     = 24000000,
    parameter integer SCL_HZ     = 100000,
    parameter integer POWERUP_MS = 20,
    parameter integer RESET_MS   = 2
) (
    input  wire clk,
    input  wire rst_n,

    output reg  scl,
    inout  wire sda,

    output reg  cfg_done,
    output wire cfg_busy,
    output reg  [7:0] wr_count,

    input  wire        dyn_tog,
    input  wire [7:0]  dyn_addr,
    input  wire [15:0] dyn_data,
    output reg         ack_tog
);

    localparam integer QUARTER    = (CLK_HZ / SCL_HZ) / 4;
    localparam integer PWR_CYCLES = (CLK_HZ / 1000) * POWERUP_MS;
    localparam integer RST_CYCLES = (CLK_HZ / 1000) * RESET_MS;
    localparam [7:0]   LAST_IDX   = 8'd12;
    localparam [7:0]   RESET_IDX  = 8'd0;

    localparam [3:0]
        S_POWER = 4'd0,   // 20 ms power-on wait
        S_LOAD  = 4'd1,   // latch table entry
        S_START = 4'd2,   // START condition
        S_BITS  = 4'd3,   // 4 bytes + 4 acknowledge slots
        S_STOP  = 4'd4,   // STOP condition
        S_GAP   = 4'd5,   // one idle SCL period between transactions
        S_RSW   = 4'd6,   // reset settle time
        S_DONE  = 4'd7,   // table finished, serve the dynamic port
        S_DYN   = 4'd8;   // latch a dynamic request

    reg [3:0]  state;
    reg [15:0] qcnt;
    reg [1:0]  phase;
    reg [1:0]  stop_ph;
    reg [5:0]  bit_idx;
    reg [7:0]  idx;
    reg [7:0]  cur_addr;
    reg [15:0] cur_data;
    reg        sda_oe;
    reg        sda_out;
    reg [18:0] pwr_cnt;
    reg [16:0] rst_cnt;
    reg        is_dyn;

    reg [1:0]  tog_sync;
    reg        dyn_pending;
    reg [7:0]  pend_addr;
    reg [15:0] pend_data;

    wire sda_in = sda;
    assign sda  = sda_oe ? sda_out : 1'bz;
    assign cfg_busy = (state != S_DONE);

    wire tog_edge = tog_sync[1] ^ tog_sync[0];

    // ------------------------------------------------------- register table
    reg [7:0]  tbl_addr;
    reg [15:0] tbl_data;

    always @(*) begin
        case (idx)
        // >>> MT9M001_TABLE_BEGIN
            8'd0 : begin tbl_addr = 8'h0D; tbl_data = 16'h0001; end
            8'd1 : begin tbl_addr = 8'h0D; tbl_data = 16'h0000; end
            8'd2 : begin tbl_addr = 8'h07; tbl_data = 16'h0000; end
            8'd3 : begin tbl_addr = 8'h01; tbl_data = 16'h000C; end
            8'd4 : begin tbl_addr = 8'h02; tbl_data = 16'h0014; end
            8'd5 : begin tbl_addr = 8'h03; tbl_data = 16'h03FF; end
            8'd6 : begin tbl_addr = 8'h04; tbl_data = 16'h04FF; end
            8'd7 : begin tbl_addr = 8'h05; tbl_data = 16'h01F4; end
            8'd8 : begin tbl_addr = 8'h06; tbl_data = 16'h0019; end
            8'd9 : begin tbl_addr = 8'h20; tbl_data = 16'h0018; end
            8'd10: begin tbl_addr = 8'h35; tbl_data = 16'h0008; end
            8'd11: begin tbl_addr = 8'h09; tbl_data = 16'h00C8; end
            8'd12: begin tbl_addr = 8'h07; tbl_data = 16'h0002; end
        // >>> MT9M001_TABLE_END
        default: begin tbl_addr = 8'h07; tbl_data = 16'h0000; end
        endcase
    end

    // ----------------------------------------------------------- bit decode
    // Byte order: 0xBA (device 0x5D, write), register address, data high,
    // data low.  bit_idx 8/17/26/35 are the acknowledge slots.
    wire is_ack_n = (bit_idx == 6'd7)  || (bit_idx == 6'd16) ||
                    (bit_idx == 6'd25) || (bit_idx == 6'd34);

    wire [5:0] nx = bit_idx + 6'd1;

    wire [5:0] bitpos_n = (nx <= 6'd7)  ? (6'd7  - nx) :
                          (nx <= 6'd16) ? (6'd16 - nx) :
                          (nx <= 6'd25) ? (6'd25 - nx) :
                                          (6'd34 - nx);

    wire [7:0] tx_byte_n = (nx <= 6'd7)  ? 8'hBA :
                           (nx <= 6'd16) ? cur_addr :
                           (nx <= 6'd25) ? cur_data[15:8] : cur_data[7:0];

    wire bit_val_n = tx_byte_n[bitpos_n[2:0]];

    // ------------------------------------------------------------ sequencer
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_POWER;
            qcnt        <= 16'd0;
            phase       <= 2'd0;
            stop_ph     <= 2'd0;
            bit_idx     <= 6'd0;
            idx         <= 8'd0;
            cur_addr    <= 8'h0D;
            cur_data    <= 16'h0000;
            scl         <= 1'b1;
            sda_oe      <= 1'b1;
            sda_out     <= 1'b1;
            pwr_cnt     <= 19'd0;
            rst_cnt     <= 17'd0;
            cfg_done    <= 1'b0;
            wr_count    <= 8'd0;
            is_dyn      <= 1'b0;
            tog_sync    <= 2'b00;
            dyn_pending <= 1'b0;
            pend_addr   <= 8'h09;
            pend_data   <= 16'h0000;
            ack_tog     <= 1'b0;
        end else begin
            tog_sync <= {tog_sync[0], dyn_tog};
            if (tog_edge) begin
                dyn_pending <= 1'b1;
                pend_addr   <= dyn_addr;
                pend_data   <= dyn_data;
            end

            case (state)
            // ------------------------------------------------- power-up wait
            S_POWER: begin
                scl     <= 1'b1;
                sda_oe  <= 1'b1;
                sda_out <= 1'b1;
                if (PWR_CYCLES == 0 || pwr_cnt >= PWR_CYCLES - 1)
                    state <= S_LOAD;
                else
                    pwr_cnt <= pwr_cnt + 19'd1;
            end

            // ---------------------------------------------------- latch entry
            S_LOAD: begin
                cur_addr <= tbl_addr;
                cur_data <= tbl_data;
                is_dyn   <= 1'b0;
                bit_idx  <= 6'd0;
                qcnt     <= 16'd0;
                phase    <= 2'd0;
                state    <= S_START;
            end

            // ------------------------------------- latch a dynamic request
            S_DYN: begin
                cur_addr    <= pend_addr;
                cur_data    <= pend_data;
                is_dyn      <= 1'b1;
                dyn_pending <= 1'b0;
                bit_idx     <= 6'd0;
                qcnt        <= 16'd0;
                phase       <= 2'd0;
                state       <= S_START;
            end

            // ------------------------------------------------ START condition
            // SCL is already high; pull SDA down, hold t_HD;STA, then SCL low.
            S_START: begin
                scl     <= 1'b1;
                sda_oe  <= 1'b1;
                sda_out <= 1'b0;
                if (qcnt == QUARTER - 1) begin
                    qcnt    <= 16'd0;
                    scl     <= 1'b0;
                    sda_out <= 1'b1;      // first bit of 0xBA
                    state   <= S_BITS;
                end else begin
                    qcnt <= qcnt + 16'd1;
                end
            end

            // ------------------------------------------------------- bit engine
            // quarter 0: SCL low, SDA stable
            // quarter 1: SCL high - slave samples SDA
            // quarter 2: SCL high
            // quarter 3: SCL low  - SDA moves to the next bit
            S_BITS: begin
                if (qcnt != QUARTER - 1) begin
                    qcnt <= qcnt + 16'd1;
                end else begin
                    qcnt <= 16'd0;
                    case (phase)
                    2'd0: begin scl <= 1'b1; phase <= 2'd1; end
                    2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                    2'd2: begin scl <= 1'b0; phase <= 2'd3; end
                    default: begin
                        scl   <= 1'b0;
                        phase <= 2'd0;
                        if (bit_idx == 6'd35) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= nx;
                            if (is_ack_n) begin
                                sda_oe  <= 1'b0;      // release for the 9th slot
                                sda_out <= 1'b1;
                            end else begin
                                sda_oe  <= 1'b1;
                                sda_out <= bit_val_n;
                            end
                        end
                    end
                    endcase
                end
            end

            // ------------------------------------------------- STOP condition
            S_STOP: begin
                if (qcnt != QUARTER - 1) begin
                    qcnt <= qcnt + 16'd1;
                end else begin
                    qcnt <= 16'd0;
                    case (stop_ph)
                    2'd0: begin sda_oe <= 1'b1; sda_out <= 1'b0; stop_ph <= 2'd1; end
                    2'd1: begin scl    <= 1'b1;                   stop_ph <= 2'd2; end
                    2'd2: begin sda_out <= 1'b1;                  stop_ph <= 2'd3; end
                    default: begin
                        scl      <= 1'b1;
                        sda_oe   <= 1'b1;
                        sda_out  <= 1'b1;
                        stop_ph  <= 2'd0;
                        wr_count <= wr_count + 8'd1;
                        state    <= S_GAP;
                    end
                    endcase
                end
            end

            // ------------------------------------------------ inter-write gap
            S_GAP: begin
                scl     <= 1'b1;
                sda_oe  <= 1'b1;
                sda_out <= 1'b1;
                if (qcnt != QUARTER * 4 - 1) begin
                    qcnt <= qcnt + 16'd1;
                end else begin
                    qcnt <= 16'd0;
                    if (is_dyn) begin
                        // hand the completed write back to the producer
                        ack_tog <= ~ack_tog;
                        is_dyn  <= 1'b0;
                        state   <= S_DONE;
                    end else if (idx == RESET_IDX) begin
                        rst_cnt <= 17'd0;
                        state   <= S_RSW;
                    end else if (idx == LAST_IDX) begin
                        cfg_done <= 1'b1;
                        state    <= S_DONE;
                    end else begin
                        idx   <= idx + 8'd1;
                        state <= S_LOAD;
                    end
                end
            end

            // ------------------------------ 0x0D=0x0001 settle before 0x0000
            S_RSW: begin
                if (RST_CYCLES == 0 || rst_cnt >= RST_CYCLES - 1) begin
                    idx   <= idx + 8'd1;
                    state <= S_LOAD;
                end else begin
                    rst_cnt <= rst_cnt + 17'd1;
                end
            end

            // ----------------------------------------------------------- done
            default: begin
                scl     <= 1'b1;
                sda_oe  <= 1'b1;
                sda_out <= 1'b1;
                cfg_done <= 1'b1;
                if (dyn_pending) state <= S_DYN;
            end
            endcase
        end
    end

endmodule
