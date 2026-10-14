`timescale 1ns / 1ps
//=============================================================================
// async_fifo.v - dual clock, first word fall through FIFO
//
//   * the two sides run on unrelated clocks (SDRAM side and pixel side)
//   * the read is registered, so the memory infers as a dual clock block RAM.
//     The video path delays its timing signals by one pixel clock to stay
//     aligned with the returned pixel
//   * the pointers cross the clock domains in grey code, so a value that is
//     being updated is never sampled in an inconsistent state
//   * wr_level / rd_level are only used for flow control, so being one entry
//     pessimistic because of the synchroniser delay is harmless
//=============================================================================
module async_fifo #(
    parameter integer DW = 24,
    parameter integer AW = 8               // depth = 2 ** AW
)(
    // write side ------------------------------------------------------------
    input  wire          wr_clk,
    input  wire          wr_rst_n,
    input  wire          wr_en,
    input  wire [DW-1:0] wr_data,
    output wire          wr_full,
    output reg  [AW:0]   wr_level,         // entries currently in the FIFO

    // read side -------------------------------------------------------------
    input  wire          rd_clk,
    input  wire          rd_rst_n,
    input  wire          rd_en,
    output wire [DW-1:0] rd_data,
    output wire          rd_empty,
    output reg  [AW:0]   rd_level
);

    localparam integer DEPTH = (1 << AW);

    reg [DW-1:0] mem [0:DEPTH-1];

    reg [AW:0] wr_ptr, rd_ptr;             // binary pointers, one extra bit
    reg [AW:0] rd_gray_s1, rd_gray_s2;     // read pointer in the write domain
    reg [AW:0] wr_gray_s1, wr_gray_s2;     // write pointer in the read domain
    reg [DW-1:0] rd_data_r;

    wire [AW:0] wr_ptr_gray = (wr_ptr >> 1) ^ wr_ptr;
    wire [AW:0] rd_ptr_gray = (rd_ptr >> 1) ^ rd_ptr;

    // grey to binary, used for the level outputs only
    function [AW:0] gray2bin;
        input [AW:0] g;
        integer i;
        begin
            gray2bin[AW] = g[AW];
            for (i = AW - 1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    //------------------------------------------------------------------
    // write side
    //------------------------------------------------------------------
    assign wr_full = (wr_ptr_gray ==
                      {~rd_gray_s2[AW:AW-1], rd_gray_s2[AW-2:0]});

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr      <= {(AW+1){1'b0}};
            rd_gray_s1  <= {(AW+1){1'b0}};
            rd_gray_s2  <= {(AW+1){1'b0}};
            wr_level    <= {(AW+1){1'b0}};
        end else begin
            rd_gray_s1 <= rd_ptr_gray;
            rd_gray_s2 <= rd_gray_s1;

            if (wr_en && !wr_full) begin
                mem[wr_ptr[AW-1:0]] <= wr_data;
                wr_ptr              <= wr_ptr + 1'b1;
            end

            // level = write pointer - read pointer (grey decoded back to
            // binary), only valid for flow control
            wr_level <= wr_ptr - gray2bin(rd_gray_s2);
        end
    end

    //------------------------------------------------------------------
    // read side
    //------------------------------------------------------------------
    assign rd_empty = (rd_ptr_gray == wr_gray_s2);
    assign rd_data  = rd_data_r;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr     <= {(AW+1){1'b0}};
            wr_gray_s1 <= {(AW+1){1'b0}};
            wr_gray_s2 <= {(AW+1){1'b0}};
            rd_level   <= {(AW+1){1'b0}};
        end else begin
            wr_gray_s1 <= wr_ptr_gray;
            wr_gray_s2 <= wr_gray_s1;

            if (rd_en && !rd_empty)
                rd_ptr <= rd_ptr + 1'b1;

            if (rd_en && !rd_empty)
                rd_data_r <= mem[rd_ptr[AW-1:0]];

            rd_level <= gray2bin(wr_gray_s2) - rd_ptr;
        end
    end

endmodule
