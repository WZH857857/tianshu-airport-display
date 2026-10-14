`timescale 1ns / 1ps
//=============================================================================
// sdram_model.v - behavioural SDR SDRAM with protocol and timing checks
//
//   * 2M x 32 bit array, the same geometry as EG_PHY_SDRAM_2M_32
//   * READ latency CL, data valid for one clock per READ command
//   * DQ is released (high Z) when no data is being driven
//   * reports tRCD / tRP / tRAS / refresh protocol violations
//=============================================================================
module sdram_model #(
    parameter integer CL     = 3,
    parameter integer T_RCD  = 3,       // cycles, ACT -> RD/WR
    parameter integer T_RP   = 3,       // cycles, PRE -> ACT
    parameter integer T_RAS  = 5,       // cycles, ACT -> PRE
    parameter integer T_RFC  = 8,       // cycles, REF -> REF
    parameter integer MEM_WORDS = 2097152
)(
    input  wire        clk,
    input  wire        cke,
    input  wire        cs_n,
    input  wire        ras_n,
    input  wire        cas_n,
    input  wire        we_n,
    input  wire [10:0] addr,
    input  wire [1:0]  ba,
    inout  wire [31:0] dq,
    input  wire [3:0]  dm
);

    localparam [2:0] CMD_NOP = 3'b111,
                     CMD_ACT = 3'b011,
                     CMD_RD  = 3'b101,
                     CMD_WR  = 3'b100,
                     CMD_PRE = 3'b010,
                     CMD_REF = 3'b001,
                     CMD_LMR = 3'b000;

    reg [31:0] mem [0:MEM_WORDS-1];

    reg [2:0]  cmd;
    reg        open0, open1, open2, open3;
    reg [9:0]  row0, row1, row2, row3;
    integer    act_time [0:3];
    integer    pre_time [0:3];
    integer    ref_time;
    integer    clk_cnt;
    reg [10:0] mode_reg;
    integer    n_rd, n_wr, n_act, n_pre, n_ref, n_lmr;
    integer    n_err;
    integer    k;
    integer    rd_lat;

    reg [31:0] rd_word;
    reg [CL:0] dv;
    reg [31:0] dpipe [0:CL];

    // full address of the current command, using the same linear layout as
    // sdram_ctrl: {row[9:0], ba[1:0], col[8:0]}
    wire [20:0] cur_addr = {row_sel, ba, addr[8:0]};
    wire [9:0]  row_sel  = (ba == 2'd0) ? row0 :
                          (ba == 2'd1) ? row1 :
                          (ba == 2'd2) ? row2 : row3;
    wire        cur_open = (ba == 2'd0) ? open0 :
                           (ba == 2'd1) ? open1 :
                           (ba == 2'd2) ? open2 : open3;

    assign dq = dv[CL] ? dpipe[CL] : 32'hzzzz_zzzz;

    initial begin
        clk_cnt = 0;
        n_rd = 0; n_wr = 0; n_act = 0; n_pre = 0; n_ref = 0; n_lmr = 0;
        n_err = 0;
        ref_time = -1000;
        rd_word  = 32'h0;
        dv       = {(CL+1){1'b0}};
        open0 = 0; open1 = 0; open2 = 0; open3 = 0;
        row0 = 0; row1 = 0; row2 = 0; row3 = 0;
        mode_reg = 11'h000;
        for (k = 0; k < 4; k = k + 1) begin
            act_time[k] = -1000;
            pre_time[k] = -1000;
        end
        for (k = 0; k <= CL; k = k + 1)
            dpipe[k] = 32'h0;
    end

    always @(posedge clk) begin
        clk_cnt = clk_cnt + 1;
        dv[0]    <= (cke && !cs_n && ({ras_n, cas_n, we_n} == CMD_RD));
        dpipe[0] <= (cur_open) ? mem[cur_addr] : 32'hzzzz_zzzz;
        for (k = 1; k <= CL; k = k + 1) begin
            dv[k]    <= dv[k-1];
            dpipe[k] <= dpipe[k-1];
        end

        if (cke && !cs_n) begin
            cmd = {ras_n, cas_n, we_n};
            case (cmd)
                CMD_ACT: begin
                    n_act = n_act + 1;
                    case (ba)
                        2'd0: begin
                            if (open0 && (row0 != addr[9:0])) begin
                                $display("[%0t] SDRAM ERROR: ACT on open bank0 row %0d (new row %0d)",
                                         $time, row0, addr[9:0]);
                                n_err = n_err + 1;
                            end
                            open0 <= 1'b1; row0 <= addr[9:0];
                        end
                        2'd1: begin
                            if (open1 && (row1 != addr[9:0])) begin
                                $display("[%0t] SDRAM ERROR: ACT on open bank1 row %0d (new row %0d)",
                                         $time, row1, addr[9:0]);
                                n_err = n_err + 1;
                            end
                            open1 <= 1'b1; row1 <= addr[9:0];
                        end
                        2'd2: begin
                            if (open2 && (row2 != addr[9:0])) begin
                                $display("[%0t] SDRAM ERROR: ACT on open bank2 row %0d (new row %0d)",
                                         $time, row2, addr[9:0]);
                                n_err = n_err + 1;
                            end
                            open2 <= 1'b1; row2 <= addr[9:0];
                        end
                        default: begin
                            if (open3 && (row3 != addr[9:0])) begin
                                $display("[%0t] SDRAM ERROR: ACT on open bank3 row %0d (new row %0d)",
                                         $time, row3, addr[9:0]);
                                n_err = n_err + 1;
                            end
                            open3 <= 1'b1; row3 <= addr[9:0];
                        end
                    endcase
                    if (clk_cnt - pre_time[ba] < T_RP) begin
                        $display("[%0t] SDRAM ERROR: tRP violation on bank %0d (%0d cycles)",
                                 $time, ba, clk_cnt - pre_time[ba]);
                        n_err = n_err + 1;
                    end
                    act_time[ba] = clk_cnt;
                end

                CMD_RD: begin
                    n_rd = n_rd + 1;
                    if (!cur_open) begin
                        $display("[%0t] SDRAM ERROR: RD to closed bank %0d", $time, ba);
                        n_err = n_err + 1;
                    end
                    if (clk_cnt - act_time[ba] < T_RCD) begin
                        $display("[%0t] SDRAM ERROR: tRCD violation on RD bank %0d (%0d cycles)",
                                 $time, ba, clk_cnt - act_time[ba]);
                        n_err = n_err + 1;
                    end
                end

                CMD_WR: begin
                    n_wr = n_wr + 1;
                    if (!cur_open) begin
                        $display("[%0t] SDRAM ERROR: WR to closed bank %0d", $time, ba);
                        n_err = n_err + 1;
                    end
                    if (clk_cnt - act_time[ba] < T_RCD) begin
                        $display("[%0t] SDRAM ERROR: tRCD violation on WR bank %0d (%0d cycles)",
                                 $time, ba, clk_cnt - act_time[ba]);
                        n_err = n_err + 1;
                    end
                    if (!dm[0]) mem[cur_addr][7:0]   <= dq[7:0];
                    if (!dm[1]) mem[cur_addr][15:8]  <= dq[15:8];
                    if (!dm[2]) mem[cur_addr][23:16] <= dq[23:16];
                    if (!dm[3]) mem[cur_addr][31:24] <= dq[31:24];
                end

                CMD_PRE: begin
                    n_pre = n_pre + 1;
                    if (addr[10]) begin
                        if (clk_cnt - act_time[0] < T_RAS && open0) begin
                            $display("[%0t] SDRAM ERROR: tRAS violation PRE-ALL bank0", $time);
                            n_err = n_err + 1;
                        end
                        open0 <= 1'b0; open1 <= 1'b0;
                        open2 <= 1'b0; open3 <= 1'b0;
                        for (k = 0; k < 4; k = k + 1)
                            pre_time[k] = clk_cnt;
                    end else begin
                        case (ba)
                            2'd0: begin
                                if (clk_cnt - act_time[0] < T_RAS && open0) begin
                                    $display("[%0t] SDRAM ERROR: tRAS violation PRE bank0", $time);
                                    n_err = n_err + 1;
                                end
                                open0 <= 1'b0;
                            end
                            2'd1: begin
                                if (clk_cnt - act_time[1] < T_RAS && open1) begin
                                    $display("[%0t] SDRAM ERROR: tRAS violation PRE bank1", $time);
                                    n_err = n_err + 1;
                                end
                                open1 <= 1'b0;
                            end
                            2'd2: begin
                                if (clk_cnt - act_time[2] < T_RAS && open2) begin
                                    $display("[%0t] SDRAM ERROR: tRAS violation PRE bank2", $time);
                                    n_err = n_err + 1;
                                end
                                open2 <= 1'b0;
                            end
                            default: begin
                                if (clk_cnt - act_time[3] < T_RAS && open3) begin
                                    $display("[%0t] SDRAM ERROR: tRAS violation PRE bank3", $time);
                                    n_err = n_err + 1;
                                end
                                open3 <= 1'b0;
                            end
                        endcase
                        pre_time[ba] = clk_cnt;
                    end
                end

                CMD_REF: begin
                    n_ref = n_ref + 1;
                    if (open0 || open1 || open2 || open3) begin
                        $display("[%0t] SDRAM ERROR: REF with an open bank", $time);
                        n_err = n_err + 1;
                    end
                    if (clk_cnt - ref_time < T_RFC) begin
                        $display("[%0t] SDRAM ERROR: tRFC violation (%0d cycles)",
                                 $time, clk_cnt - ref_time);
                        n_err = n_err + 1;
                    end
                    ref_time = clk_cnt;
                end

                CMD_LMR: begin
                    n_lmr = n_lmr + 1;
                    mode_reg = addr;
                    if (addr[6:4] != 3'b011) begin
                        $display("[%0t] SDRAM WARNING: mode register CAS latency = %0d",
                                 $time, addr[6:4]);
                    end
                end

                default: ;
            endcase
        end
    end

    task show_stats;
        begin
            $display("  SDRAM model stats: ACT=%0d PRE=%0d REF=%0d RD=%0d WR=%0d LMR=%0d errors=%0d",
                     n_act, n_pre, n_ref, n_rd, n_wr, n_lmr, n_err);
        end
    endtask

endmodule
