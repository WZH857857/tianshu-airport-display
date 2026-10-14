`timescale 1ns / 1ps
//=============================================================================
// sdram_ctrl.v - SDR SDRAM controller for the EG4S20 on-die 2M x 32bit SDRAM
//
//   device  : EG_PHY_SDRAM_2M_32   (2M words x 32 bit = 8 MiB)
//   mapping : addr[ 8: 0] column, 512 words per row
//             addr[10: 9] bank,   4 banks
//             addr[20:11] row,    1024 rows
//
// Features
//   * power up sequence  : PRE-ALL -> 2 x REF -> LMR
//   * periodic auto refresh (7.8 us), precharge all around the refresh
//   * per bank open row tracking, page hit accesses start immediately
//   * row miss  : single bank PRE -> wait tRP -> ACT -> wait tRCD -> RD/WR
//   * every command is emitted for exactly one clock (no repeated ACT/PRE)
//   * read burst port (up to one full row) and single word write port
//   * read capture latency is measured at power up, so the design does not
//     depend on the exact CL plus output delay of the SDRAM macro
//   * the read capture register is clocked by the falling edge of clk, half a
//     cycle away from the edge that moves dq, so it samples in the middle of
//     the data eye instead of on the transition
//
// Protocol
//   read  : pulse rd_start with rd_addr/rd_len, the burst must stay inside a
//           single row. rd_ack pulses when the request has been latched, then
//           rd_valid pulses once per word and rd_last marks the final word.
//   write : pulse wr_start with wr_addr/wr_data, wr_ack pulses when the word
//           and its address have been latched.
//=============================================================================
module sdram_ctrl #(
    // timings are expressed in clock cycles of "clk"; the frame_store
    // instance drives a 50 MHz (20 ns) clock and passes the values below
    parameter integer T_RP        = 2,          // tRP   20 ns -> 2 cyc
    parameter integer T_RCD       = 2,          // tRCD  20 ns -> 2 cyc
    parameter integer T_MRD       = 2,          // tMRD   2 tCK
    parameter integer T_RFC       = 5,          // tRFC  70 ns -> 5 cyc
    parameter integer T_WR_GAP    = 3,          // tWR + tRAS margin after WR
    parameter integer T_RD_GAP    = 2,          // tRTP margin after RD
    parameter integer REFRESH_CYC = 390,        // 7.80 us at 50 MHz
    parameter integer PWRUP_CYC   = 10000,      // 200 us at 50 MHz
    parameter integer CAL_MIN     = 1,          // first capture delay tried
    parameter integer CAL_MAX     = 10,         // highest capture delay tried
    parameter [20:0]  CAL_ADDR    = 21'h1F_FFFC,// last row, outside the frames
    parameter [10:0]  MODE_REG    = 11'h030     // burst 1, sequential, CL = 3
)(
    input  wire        clk,
    input  wire        rst_n,

    // read burst port ---------------------------------------------------------
    input  wire        rd_start,
    input  wire [20:0] rd_addr,
    input  wire [10:0] rd_len,      // 1 .. 512, must not cross a row boundary
    output reg         rd_ack,
    output reg         rd_busy,
    output reg         rd_valid,
    output reg         rd_last,
    output reg  [31:0] rd_data,

    // write port (one word per handshake) -------------------------------------
    input  wire        wr_start,
    input  wire [20:0] wr_addr,
    input  wire [31:0] wr_data,
    output reg         wr_ack,
    output reg         wr_done,     // the word has been written to the device

    // status ------------------------------------------------------------------
    output reg         init_done,
    output reg         cal_done,
    output reg         cal_fail,
    output reg  [3:0]  cal_latency,
    output reg  [15:0] refresh_cnt,

    // SDRAM pins --------------------------------------------------------------
    output wire        sdram_clk,
    output wire        sdram_cke,
    output wire        sdram_cs_n,
    output wire        sdram_ras_n,
    output wire        sdram_cas_n,
    output wire        sdram_we_n,
    output wire [10:0] sdram_addr,
    output wire [1:0]  sdram_ba,
    output wire [3:0]  sdram_dm,
    inout  wire [31:0] sdram_dq
);

    //------------------------------------------------------------------
    // state encoding - every command state lasts exactly one clock
    //------------------------------------------------------------------
    localparam [4:0] S_POWER     = 5'd0,
                     S_PRE_INIT  = 5'd1,
                     S_PRE_INITW = 5'd2,
                     S_REF_INIT  = 5'd3,
                     S_REF_INITW = 5'd4,
                     S_LMR       = 5'd5,
                     S_LMRW      = 5'd6,
                     S_PRE_REF   = 5'd7,
                     S_PRE_REFW  = 5'd8,
                     S_REF_RUN   = 5'd9,
                     S_REF_RUNW  = 5'd10,
                     S_IDLE      = 5'd11,
                     S_PRE_ONE   = 5'd12,
                     S_PRE_ONEW  = 5'd13,
                     S_ACT       = 5'd14,
                     S_ACTW      = 5'd15,
                     S_RD        = 5'd16,
                     S_WR        = 5'd17,
                     S_GAP       = 5'd18,
                     S_CAL_CHK   = 5'd19;

    localparam [1:0] SRC_NORM = 2'd0,
                     SRC_CALW = 2'd1,
                     SRC_CALR = 2'd2;

    localparam [2:0] CMD_NOP = 3'b111,
                     CMD_ACT = 3'b011,
                     CMD_RD  = 3'b101,
                     CMD_WR  = 3'b100,
                     CMD_PRE = 3'b010,
                     CMD_REF = 3'b001,
                     CMD_LMR = 3'b000;

    reg [4:0]  state;
    reg [15:0] tcnt;
    reg [15:0] ref_cnt;
    reg        ref_pend;
    reg [1:0]  ref_init_cnt;

    //------------------------------------------------------------------
    // request latches
    //------------------------------------------------------------------
    reg        rd_pend;
    reg [20:0] rd_pend_addr;
    reg [10:0] rd_pend_len;
    reg [10:0] rd_issue_cnt;        // RD commands already issued

    reg        wr_pend;
    reg [20:0] wr_pend_addr;
    reg [31:0] wr_pend_data;
    reg        wr_issued;           // a WR command went out on the bus

    // currently latched access
    reg        op_we;
    reg [20:0] op_addr;
    reg [31:0] op_data;
    reg [10:0] op_len;

    //------------------------------------------------------------------
    // read / write arbitration
    //
    // The read port feeds the display and must never miss its line slot, so a
    // pending read is served first and the image loader fills the gaps. A
    // write that keeps losing is force granted once it has been waiting for
    // WR_AGE_MAX clocks, which bounds write latency if the read port ever
    // becomes saturated.
    //------------------------------------------------------------------
    localparam [7:0] WR_AGE_MAX = 8'd64;
    reg [7:0]  wr_wait_age;         // clocks the pending write has waited

    wire       arb_idle = (state == S_IDLE) && cal_done && !ref_pend && !rd_busy;
    wire       gr_wr    = arb_idle && wr_pend &&
                          (!rd_pend || (wr_wait_age >= WR_AGE_MAX));
    wire       gr_rd    = arb_idle && rd_pend && !gr_wr;

    wire [1:0] op_ba  = op_addr[10:9];
    wire [9:0] op_row = op_addr[20:11];
    wire [8:0] op_col = op_addr[8:0];

    //------------------------------------------------------------------
    // open row book keeping
    //------------------------------------------------------------------
    reg        row_open0, row_open1, row_open2, row_open3;
    reg [9:0]  open_row0, open_row1, open_row2, open_row3;

    function hit;
        input [20:0] a;
        begin
            case (a[10:9])
                2'd0:    hit = row_open0 && (open_row0 == a[20:11]);
                2'd1:    hit = row_open1 && (open_row1 == a[20:11]);
                2'd2:    hit = row_open2 && (open_row2 == a[20:11]);
                default: hit = row_open3 && (open_row3 == a[20:11]);
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // calibration data - four different patterns for the latency search
    //------------------------------------------------------------------
    localparam [31:0] CAL_D0 = 32'h1234_5678,
                      CAL_D1 = 32'h9abc_def0,
                      CAL_D2 = 32'h0f1e_2d3c,
                      CAL_D3 = 32'hfeed_5a5a;

    reg [1:0]  src;
    reg [3:0]  cal_sel;
    reg [2:0]  cal_wr_cnt;
    reg [2:0]  cal_rd_cnt;
    reg [2:0]  cap_cnt;
    reg [15:0] cal_to;
    reg [31:0] cal_cap0, cal_cap1, cal_cap2, cal_cap3;

    // returns the pattern written for the n-th calibration word
    function [31:0] cal_pattern;
        input [2:0] n;
        begin
            case (n[1:0])
                2'd0:    cal_pattern = CAL_D0;
                2'd1:    cal_pattern = CAL_D1;
                2'd2:    cal_pattern = CAL_D2;
                default: cal_pattern = CAL_D3;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // read data capture
    //
    // The first RD command of a burst restarts the latency timer. The capture
    // window opens once the timer reaches the calibrated delay, after which
    // one word per clock is captured until the whole burst has been returned.
    // The capture flop itself is clocked by the falling edge of clk, which is
    // half a cycle away from the edge that moves dq - see cap_beat below.
    //------------------------------------------------------------------
    reg [3:0]  rd_wait_cnt;
    reg        rd_cap_act;
    reg [10:0] rd_cap_cnt;
    reg        rd_done;             // the whole burst has been captured
    reg        rd_armed;            // the first RD command of the burst went out

    wire       rd_first = (state == S_RD) && (rd_issue_cnt == 11'd0);

    // The data of the on die SDRAM changes on the rising edge of the very
    // clock this controller runs on, so sampling dq on that edge would sit
    // right on the transition.  The capture register is clocked by the falling
    // edge instead, which puts the sample in the middle of the data eye - the
    // same half cycle of margin the official lab_ex5_i2s reference design gets
    // from its 180 degree Clk_sft.  cal_sel still searches whole cycles, so
    // the search window stays 1 .. CAL_MAX clocks wide.
    wire       cap_beat = rd_armed && !rd_done &&
                          (rd_cap_act || (rd_wait_cnt == cal_sel));

    always @(negedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_data <= 32'd0;
        else if (cap_beat)
            rd_data <= sdram_dq;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_wait_cnt <= 4'd0;
            rd_cap_act  <= 1'b0;
            rd_cap_cnt  <= 11'd0;
            rd_done     <= 1'b0;
            rd_armed    <= 1'b0;
            rd_valid    <= 1'b0;
            rd_last     <= 1'b0;
        end else begin
            rd_valid <= 1'b0;
            rd_last  <= 1'b0;

            if (!rd_busy) begin
                // no burst in flight: the capture window stays closed so no
                // spurious beats can be produced while the bus is idle
                rd_wait_cnt <= 4'd0;
                rd_cap_act  <= 1'b0;
                rd_cap_cnt  <= 11'd0;
                rd_done     <= 1'b0;
                rd_armed    <= 1'b0;
            end else if (rd_first) begin
                // the burst really starts here: only now may the capture
                // window open, the precharge and activate cycles before the
                // first RD command must not produce any beats
                rd_armed    <= 1'b1;
                rd_wait_cnt <= 4'd0;
                rd_cap_act  <= 1'b0;
                rd_cap_cnt  <= 11'd0;
                rd_done     <= 1'b0;
            end else if (rd_armed && !rd_done && !rd_cap_act) begin
                if (rd_wait_cnt != cal_sel) begin
                    rd_wait_cnt <= rd_wait_cnt + 1'b1;
                end else begin
                    rd_valid <= 1'b1;
                    rd_cap_cnt <= 11'd0;
                    if (op_len == 11'd1) begin
                        rd_cap_act <= 1'b0;
                        rd_done    <= 1'b1;
                        rd_last    <= 1'b1;
                    end else begin
                        rd_cap_act <= 1'b1;
                        rd_cap_cnt <= 11'd1;
                    end
                end
            end else if (rd_armed && !rd_done) begin
                rd_valid <= 1'b1;
                if (rd_cap_cnt == op_len - 11'd1) begin
                    rd_cap_act <= 1'b0;
                    rd_cap_cnt <= 11'd0;
                    rd_done    <= 1'b1;
                    rd_last    <= 1'b1;
                end else begin
                    rd_cap_cnt <= rd_cap_cnt + 11'd1;
                end
            end
        end
    end

    //------------------------------------------------------------------
    // command, address and data decode (one clock wide by construction)
    //------------------------------------------------------------------
    reg [2:0]  cmd;
    reg [10:0] addr_d;
    reg [1:0]  ba_d;

    always @* begin
        cmd    = CMD_NOP;
        addr_d = 11'd0;
        ba_d   = 2'd0;
        case (state)
            S_PRE_INIT,
            S_PRE_REF: begin
                cmd    = CMD_PRE;
                addr_d = 11'h400;           // A10 = 1 : all banks
            end
            S_PRE_ONE: begin
                cmd    = CMD_PRE;
                addr_d = 11'h000;           // A10 = 0 : single bank
                ba_d   = op_ba;
            end
            S_REF_INIT,
            S_REF_RUN: cmd = CMD_REF;
            S_LMR: begin
                cmd    = CMD_LMR;
                addr_d = MODE_REG;
            end
            S_ACT: begin
                cmd    = CMD_ACT;
                addr_d = {1'b0, op_row};
                ba_d   = op_ba;
            end
            S_RD: begin
                cmd    = CMD_RD;
                addr_d = {2'b00, op_col};
                ba_d   = op_ba;
            end
            S_WR: begin
                cmd    = CMD_WR;
                addr_d = {2'b00, op_col};
                ba_d   = op_ba;
            end
            default: ;                      // NOP
        endcase
    end

    assign {sdram_ras_n, sdram_cas_n, sdram_we_n} = cmd;
    assign sdram_addr = addr_d;
    assign sdram_ba   = ba_d;
    assign sdram_dm   = (state == S_WR) ? 4'h0 : 4'hf;
    assign sdram_dq   = (state == S_WR) ? op_data : 32'hzzzz_zzzz;
    assign sdram_cke  = 1'b1;
    assign sdram_cs_n = 1'b0;
    assign sdram_clk  = clk;

    //------------------------------------------------------------------
    // main state machine
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_POWER;
            tcnt         <= PWRUP_CYC[15:0];
            ref_cnt      <= REFRESH_CYC[15:0];
            ref_pend     <= 1'b0;
            ref_init_cnt <= 2'd0;
            rd_pend      <= 1'b0;
            rd_pend_addr <= 21'd0;
            rd_pend_len  <= 11'd1;
            rd_issue_cnt <= 11'd0;
            rd_ack       <= 1'b0;
            rd_busy      <= 1'b0;
            wr_pend      <= 1'b0;
            wr_pend_addr <= 21'd0;
            wr_pend_data <= 32'd0;
            wr_ack       <= 1'b0;
            wr_done      <= 1'b0;
            wr_issued    <= 1'b0;
            wr_wait_age  <= 8'd0;
            op_we        <= 1'b0;
            op_addr      <= 21'd0;
            op_data      <= 32'd0;
            op_len       <= 11'd1;
            src          <= SRC_NORM;
            cal_sel      <= CAL_MIN[3:0];
            cal_wr_cnt   <= 3'd0;
            cal_rd_cnt   <= 3'd0;
            cap_cnt      <= 3'd0;
            cal_to       <= 16'hffff;
            cal_cap0     <= 32'd0;
            cal_cap1     <= 32'd0;
            cal_cap2     <= 32'd0;
            cal_cap3     <= 32'd0;
            init_done    <= 1'b0;
            cal_done     <= 1'b0;
            cal_fail     <= 1'b0;
            cal_latency  <= 4'd4;
            refresh_cnt  <= 16'd0;
            row_open0    <= 1'b0;
            row_open1    <= 1'b0;
            row_open2    <= 1'b0;
            row_open3    <= 1'b0;
            open_row0    <= 10'd0;
            open_row1    <= 10'd0;
            open_row2    <= 10'd0;
            open_row3    <= 10'd0;
        end else begin
            rd_ack <= 1'b0;
            wr_ack <= 1'b0;
            // wr_done reports that the WR command of the previous cycle has
            // been picked up by the device, so the data is really in memory
            wr_done   <= wr_issued;
            wr_issued <= 1'b0;

            //--------------------------------------------------
            // refresh interval counter
            //--------------------------------------------------
            if (init_done && cal_done && !ref_pend) begin
                if (ref_cnt == 16'd0) begin
                    ref_pend <= 1'b1;
                    ref_cnt  <= REFRESH_CYC[15:0];
                end else begin
                    ref_cnt <= ref_cnt - 1'b1;
                end
            end

            //--------------------------------------------------
            // request latching
            //--------------------------------------------------
            if (rd_start && !rd_pend && !rd_busy && init_done && cal_done) begin
                rd_pend      <= 1'b1;
                rd_pend_addr <= rd_addr;
                rd_pend_len  <= rd_len;
                rd_ack       <= 1'b1;
            end
            if (wr_start && !wr_pend && init_done && cal_done) begin
                wr_pend      <= 1'b1;
                wr_pend_addr <= wr_addr;
                wr_pend_data <= wr_data;
                wr_ack       <= 1'b1;
            end

            //--------------------------------------------------
            // write starvation guard
            //
            // Counts how long the oldest pending write has been waiting and
            // lets it win the next arbitration slot once the limit is hit.
            //--------------------------------------------------
            if (!wr_pend)
                wr_wait_age <= 8'd0;
            else if (gr_wr)
                wr_wait_age <= 8'd0;
            else if (wr_wait_age != 8'hff)
                wr_wait_age <= wr_wait_age + 1'b1;

            //--------------------------------------------------
            // read burst completion
            //--------------------------------------------------
            if (rd_busy && rd_last)
                rd_busy <= 1'b0;

            case (state)
                //--------------------------------------------------
                S_POWER: begin
                    if (tcnt == 16'd0)
                        state <= S_PRE_INIT;
                    else
                        tcnt <= tcnt - 1'b1;
                end

                //--------------------------------------------------
                S_PRE_INIT: begin
                    row_open0 <= 1'b0;
                    row_open1 <= 1'b0;
                    row_open2 <= 1'b0;
                    row_open3 <= 1'b0;
                    state     <= S_PRE_INITW;
                    tcnt      <= T_RP[15:0];
                end

                S_PRE_INITW: begin
                    if (tcnt == 16'd0) begin
                        state <= S_REF_INIT;
                        tcnt  <= 16'd0;
                    end else begin
                        tcnt <= tcnt - 1'b1;
                    end
                end

                //--------------------------------------------------
                S_REF_INIT: begin
                    state <= S_REF_INITW;
                    tcnt  <= T_RFC[15:0];
                end

                S_REF_INITW: begin
                    if (tcnt == 16'd0) begin
                        if (ref_init_cnt == 2'd0) begin
                            ref_init_cnt <= 2'd1;
                            state        <= S_REF_INIT;
                        end else begin
                            state <= S_LMR;
                        end
                    end else begin
                        tcnt <= tcnt - 1'b1;
                    end
                end

                //--------------------------------------------------
                S_LMR: begin
                    state <= S_LMRW;
                    tcnt  <= T_MRD[15:0];
                end

                S_LMRW: begin
                    if (tcnt == 16'd0) begin
                        state      <= S_IDLE;
                        init_done  <= 1'b1;
                        src        <= SRC_CALW;
                        cal_wr_cnt <= 3'd0;
                        cal_rd_cnt <= 3'd0;
                        cap_cnt    <= 3'd0;
                        cal_sel    <= CAL_MIN[3:0];
                        cal_to     <= 16'hffff;
                    end else begin
                        tcnt <= tcnt - 1'b1;
                    end
                end

                //--------------------------------------------------
                S_PRE_REF: begin
                    row_open0 <= 1'b0;
                    row_open1 <= 1'b0;
                    row_open2 <= 1'b0;
                    row_open3 <= 1'b0;
                    state     <= S_PRE_REFW;
                    tcnt      <= T_RP[15:0];
                end

                S_PRE_REFW: begin
                    if (tcnt == 16'd0)
                        state <= S_REF_RUN;
                    else
                        tcnt <= tcnt - 1'b1;
                end

                //--------------------------------------------------
                S_REF_RUN: begin
                    state       <= S_REF_RUNW;
                    tcnt        <= T_RFC[15:0];
                    refresh_cnt <= refresh_cnt + 1'b1;
                end

                S_REF_RUNW: begin
                    if (tcnt == 16'd0)
                        state <= S_IDLE;
                    else
                        tcnt <= tcnt - 1'b1;
                end

                //--------------------------------------------------
                // arbitration / next access selection
                //--------------------------------------------------
                S_IDLE: begin
                    if (!cal_done) begin
                        //------------------------------------------
                        // power up read capture latency calibration
                        //------------------------------------------
                        if (src == SRC_CALW) begin
                            if (cal_wr_cnt != 3'd4) begin
                                op_we   <= 1'b1;
                                op_addr <= CAL_ADDR + {18'd0, cal_wr_cnt};
                                op_data <= cal_pattern(cal_wr_cnt);
                                op_len  <= 11'd1;
                                if (hit(CAL_ADDR + {18'd0, cal_wr_cnt})) begin
                                    state <= S_WR;
                                end else begin
                                    state <= S_PRE_ONE;
                                    tcnt  <= T_RP[15:0];
                                end
                            end else begin
                                src        <= SRC_CALR;
                                cal_rd_cnt <= 3'd0;
                                cap_cnt    <= 3'd0;
                                cal_to     <= 16'hffff;
                            end
                        end else begin
                            if (cap_cnt == 3'd4 || cal_to == 16'd0) begin
                                state <= S_CAL_CHK;
                            end else if (cal_rd_cnt != 3'd4 && !rd_busy) begin
                                op_we        <= 1'b0;
                                op_addr      <= CAL_ADDR + {18'd0, cal_rd_cnt};
                                op_data      <= 32'd0;
                                op_len       <= 11'd1;
                                rd_issue_cnt <= 11'd0;
                                cal_rd_cnt   <= cal_rd_cnt + 3'd1;
                                if (hit(CAL_ADDR + {18'd0, cal_rd_cnt})) begin
                                    state   <= S_RD;
                                    rd_busy <= 1'b1;
                                end else begin
                                    state <= S_PRE_ONE;
                                    tcnt  <= T_RP[15:0];
                                end
                            end else if (cal_to != 16'd0) begin
                                cal_to <= cal_to - 1'b1;
                            end
                        end
                    end else begin
                        //------------------------------------------
                        // normal operation
                        //------------------------------------------
                        if (ref_pend && !rd_busy) begin
                            state    <= S_PRE_REF;
                            ref_pend <= 1'b0;
                        end else if (gr_wr) begin
                            op_we   <= 1'b1;
                            op_addr <= wr_pend_addr;
                            op_data <= wr_pend_data;
                            op_len  <= 11'd1;
                            wr_pend <= 1'b0;
                            if (hit(wr_pend_addr)) begin
                                state <= S_WR;
                            end else begin
                                state <= S_PRE_ONE;
                                tcnt  <= T_RP[15:0];
                            end
                        end else if (gr_rd) begin
                            op_we        <= 1'b0;
                            op_addr      <= rd_pend_addr;
                            op_data      <= 32'd0;
                            op_len       <= rd_pend_len;
                            rd_issue_cnt <= 11'd0;
                            rd_pend      <= 1'b0;
                            rd_busy      <= 1'b1;
                            if (hit(rd_pend_addr)) begin
                                state <= S_RD;
                            end else begin
                                state <= S_PRE_ONE;
                                tcnt  <= T_RP[15:0];
                            end
                        end
                    end
                end

                //--------------------------------------------------
                S_PRE_ONE: begin
                    case (op_ba)
                        2'd0:    row_open0 <= 1'b0;
                        2'd1:    row_open1 <= 1'b0;
                        2'd2:    row_open2 <= 1'b0;
                        default: row_open3 <= 1'b0;
                    endcase
                    state <= S_PRE_ONEW;
                end

                S_PRE_ONEW: begin
                    if (tcnt == 16'd0)
                        state <= S_ACT;
                    else
                        tcnt <= tcnt - 1'b1;
                end

                //--------------------------------------------------
                S_ACT: begin
                    case (op_ba)
                        2'd0:    begin row_open0 <= 1'b1; open_row0 <= op_row; end
                        2'd1:    begin row_open1 <= 1'b1; open_row1 <= op_row; end
                        2'd2:    begin row_open2 <= 1'b1; open_row2 <= op_row; end
                        default: begin row_open3 <= 1'b1; open_row3 <= op_row; end
                    endcase
                    state <= S_ACTW;
                    tcnt  <= T_RCD[15:0];
                end

                S_ACTW: begin
                    if (tcnt == 16'd0) begin
                        rd_issue_cnt <= 11'd0;
                        if (op_we) begin
                            state <= S_WR;
                        end else begin
                            state   <= S_RD;
                            rd_busy <= 1'b1;
                        end
                    end else begin
                        tcnt <= tcnt - 1'b1;
                    end
                end

                //--------------------------------------------------
                // burst read: one RD command per word, back to back
                //--------------------------------------------------
                S_RD: begin
                    rd_issue_cnt <= rd_issue_cnt + 11'd1;
                    if (rd_issue_cnt == op_len - 11'd1) begin
                        state <= S_GAP;
                        tcnt  <= T_RD_GAP[15:0];
                    end else begin
                        op_addr <= {op_addr[20:9], op_addr[8:0] + 9'd1};
                    end
                end

                //--------------------------------------------------
                // single word write, or the four calibration writes
                //--------------------------------------------------
                S_WR: begin
                    if (src == SRC_CALW) begin
                        cal_wr_cnt <= cal_wr_cnt + 3'd1;
                        if (cal_wr_cnt != 3'd3) begin
                            op_addr <= {op_addr[20:9], op_addr[8:0] + 9'd1};
                            op_data <= cal_pattern(cal_wr_cnt + 3'd1);
                            state   <= S_WR;
                        end else begin
                            state <= S_GAP;
                            tcnt  <= T_WR_GAP[15:0];
                        end
                    end else begin
                        wr_issued <= 1'b1;
                        state <= S_GAP;
                        tcnt  <= T_WR_GAP[15:0];
                    end
                end

                S_GAP: begin
                    if (tcnt == 16'd0)
                        state <= S_IDLE;
                    else
                        tcnt <= tcnt - 1'b1;
                end

                //--------------------------------------------------
                S_CAL_CHK: begin
                    if (cal_cap0 == CAL_D0 && cal_cap1 == CAL_D1 &&
                        cal_cap2 == CAL_D2 && cal_cap3 == CAL_D3) begin
                        cal_latency <= cal_sel;
                        cal_done    <= 1'b1;
                        cal_fail    <= 1'b0;
                        src         <= SRC_NORM;
                        state       <= S_IDLE;
                    end else if (cal_sel >= CAL_MAX[3:0]) begin
                        cal_latency <= cal_sel;
                        cal_done    <= 1'b1;
                        cal_fail    <= 1'b1;
                        src         <= SRC_NORM;
                        state       <= S_IDLE;
                    end else begin
                        cal_sel    <= cal_sel + 1'b1;
                        cal_rd_cnt <= 3'd0;
                        cap_cnt    <= 3'd0;
                        cal_to     <= 16'hffff;
                        src        <= SRC_CALR;
                        state      <= S_IDLE;
                    end
                end

                default: state <= S_POWER;
            endcase

            //--------------------------------------------------
            // capture the four calibration words
            //--------------------------------------------------
            if (rd_valid && !cal_done && cap_cnt != 3'd4) begin
                case (cap_cnt)
                    3'd0:    cal_cap0 <= rd_data;
                    3'd1:    cal_cap1 <= rd_data;
                    3'd2:    cal_cap2 <= rd_data;
                    default: cal_cap3 <= rd_data;
                endcase
                cap_cnt <= cap_cnt + 3'd1;
            end
        end
    end

endmodule
