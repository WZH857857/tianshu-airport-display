`timescale 1ns / 1ps
//=============================================================================
// sdr_as_ram_stub.v - small behavioural stand-in for the encrypted vendor
// SDRAM controller used by the frame store testbench.
//
// This file is only for iverilog.  The TD build uses the encrypted
// vendor/sdr_as_ram.enc.v file instead.
//=============================================================================
module sdr_as_ram #(
    parameter integer self_refresh_open = 1
)(
    input  wire        Sdr_clk,
    input  wire        Sdr_clk_sft,
    input  wire        Rst,
    output reg         Sdr_init_done,
    output reg         Sdr_init_ref_vld,
    output reg         Sdr_busy,
    input  wire        App_ref_req,
    input  wire        App_wr_en,
    input  wire [20:0] App_wr_addr,
    input  wire [3:0]  App_wr_dm,
    input  wire [31:0] App_wr_din,
    input  wire        App_rd_en,
    input  wire [20:0] App_rd_addr,
    output reg         Sdr_rd_en,
    output reg  [31:0] Sdr_rd_dout,
    output wire        SDRAM_CLK,
    output wire        SDR_RAS,
    output wire        SDR_CAS,
    output wire        SDR_WE,
    output wire [1:0]  SDR_BA,
    output wire [10:0] SDR_ADDR,
    output wire [3:0]  SDR_DM,
    inout  wire [31:0] SDR_DQ
);

    // Match the real EG_PHY_SDRAM_2M_32 capacity.  The old 8192-word
    // stand-in was smaller than one 640x480 frame stride and made cache
    // slots above address 0x1FFF read as X.
    reg [31:0] mem [0:2097151];

    reg [2:0]  rd_valid;
    reg [20:0] rd_addr_d0, rd_addr_d1, rd_addr_d2;

    assign SDRAM_CLK = Sdr_clk;
    assign SDR_RAS   = 1'b1;
    assign SDR_CAS   = 1'b1;
    assign SDR_WE    = 1'b1;
    assign SDR_BA    = 2'b00;
    assign SDR_ADDR  = 11'd0;
    assign SDR_DM    = 4'h0;
    assign SDR_DQ    = 32'bz;

    always @(posedge Sdr_clk) begin
        if (Rst) begin
            Sdr_init_done    <= 1'b0;
            Sdr_init_ref_vld <= 1'b0;
            Sdr_busy         <= 1'b0;
            Sdr_rd_en        <= 1'b0;
            Sdr_rd_dout      <= 32'd0;
            rd_valid         <= 3'd0;
            rd_addr_d0       <= 21'd0;
            rd_addr_d1       <= 21'd0;
            rd_addr_d2       <= 21'd0;
        end else begin
            if (!Sdr_init_done)
                Sdr_init_done <= 1'b1;

            Sdr_rd_en <= 1'b0;

            if (App_wr_en) begin
                if (!App_wr_dm[0]) mem[App_wr_addr][7:0]   <= App_wr_din[7:0];
                if (!App_wr_dm[1]) mem[App_wr_addr][15:8]  <= App_wr_din[15:8];
                if (!App_wr_dm[2]) mem[App_wr_addr][23:16] <= App_wr_din[23:16];
                if (!App_wr_dm[3]) mem[App_wr_addr][31:24] <= App_wr_din[31:24];
            end

            rd_addr_d0 <= App_rd_addr;
            rd_addr_d1 <= rd_addr_d0;
            rd_addr_d2 <= rd_addr_d1;
            rd_valid   <= {rd_valid[1:0], App_rd_en};

            if (rd_valid[2]) begin
                Sdr_rd_en   <= 1'b1;
                Sdr_rd_dout <= mem[rd_addr_d2];
            end
        end
    end

endmodule
