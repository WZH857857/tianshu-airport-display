`timescale 1ns / 1ps
//=============================================================================
// voice_bank.v - on chip voice sample memory
//
// One byte per sample, written once while the WAV file is streamed off the
// card and read afterwards while the announcement plays.  The two ports are on
// different clocks (50 MHz stream side, 12.288 MHz audio side); both are
// registered, so the array infers as a dual clock block RAM.  Playback only
// starts after the load has finished, so the read port never sees a location
// that is still being written.
//
// The storage is deliberately split into a 32768 word and an 8192 word array.
// The synthesiser sizes an inferred RAM from its address port rather than from
// the declared array bound, so a single 40960 word array is mapped to a 65536
// deep one and burns all 64 block RAMs of the EG4S20.  Split this way the same
// 40960 samples cost 32 + 8 = 40 blocks, which leaves room for the 20 blocks
// the display path already uses.
//=============================================================================
module voice_bank #(
    parameter integer WORDS = 40960,
    parameter integer AW    = 16
)(
    // stream side -----------------------------------------------------
    input  wire            wr_clk,
    input  wire            wr_en,
    input  wire [AW-1:0]   wr_addr,
    input  wire [7:0]      wr_data,

    // audio side ------------------------------------------------------
    input  wire            rd_clk,
    input  wire [AW-1:0]   rd_addr,
    output wire [7:0]      rd_data
);

    // valid for WORDS in 32769..40960, which is the only size used here
    localparam integer LO_WORDS = 32768;
    localparam integer LO_AW    = 15;
    localparam integer HI_WORDS = WORDS - LO_WORDS;
    localparam integer HI_AW    = 13;

    reg [7:0] mem_lo [0:LO_WORDS-1];
    reg [7:0] mem_hi [0:HI_WORDS-1];

    always @(posedge wr_clk) begin
        if (wr_en) begin
            if (wr_addr >= LO_WORDS[AW-1:0])
                mem_hi[wr_addr[HI_AW-1:0]] <= wr_data;
            else
                mem_lo[wr_addr[LO_AW-1:0]] <= wr_data;
        end
    end

    // Both arrays are read every cycle and the results are selected with a
    // delayed copy of the address, so the data comes out one cycle after the
    // address exactly like the flat array it replaces.
    reg [7:0] rd_lo;
    reg [7:0] rd_hi;
    reg       rd_sel;

    always @(posedge rd_clk) begin
        rd_lo  <= mem_lo[rd_addr[LO_AW-1:0]];
        rd_hi  <= mem_hi[rd_addr[HI_AW-1:0]];
        rd_sel <= (rd_addr >= LO_WORDS[AW-1:0]);
    end

    assign rd_data = rd_sel ? rd_hi : rd_lo;

endmodule
