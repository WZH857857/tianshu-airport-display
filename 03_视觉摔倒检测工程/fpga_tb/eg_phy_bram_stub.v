//===========================================================================
// eg_phy_bram_stub.v -- Icarus stand-in for Anlogic's EG_PHY_BRAM
//
// TD maps the real primitive into an ERAM9K; Icarus knows nothing about it.
// The benches compile this model, which implements the same simple dual port
// shape the RTL uses: port A writes, port B does a registered read.  It is
// deliberately kept out of the TD file lists, where the vendor primitive
// from arch/eagle_macro.v must be the one that elaborates.
//===========================================================================
`timescale 1ns / 1ps

module EG_PHY_BRAM #(
    parameter MODE         = "DP8K",
    parameter DATA_WIDTH_A = "9",
    parameter DATA_WIDTH_B = "9",
    parameter READBACK     = "OFF",
    parameter REGMODE_A    = "NOREG",
    parameter REGMODE_B    = "NOREG",
    parameter WRITEMODE_A  = "NORMAL",
    parameter WRITEMODE_B  = "NORMAL"
) (
    output wire [8:0]  doa,
    output wire [8:0]  dob,
    input  wire [8:0]  dia,
    input  wire [8:0]  dib,
    input  wire [2:0]  csa,
    input  wire [2:0]  csb,
    input  wire        cea, ocea, clka, wea, rsta,
    input  wire        ceb, oceb, clkb, web, rstb,
    input  wire [12:0] addra,
    input  wire [12:0] addrb
);

    reg [8:0] mem [0:8191];
    reg [8:0] doa_r;
    reg [8:0] dob_r;

    assign doa = doa_r;
    assign dob = dob_r;

    always @(posedge clka) begin
        if (cea && (csa == 3'b111) && wea)
            mem[addra] <= dia;
        if (cea && (csa == 3'b111))
            doa_r <= mem[addra];
    end

    always @(posedge clkb) begin
        if (ceb && (csb == 3'b111) && web)
            mem[addrb] <= dib;
        if (ceb && (csb == 3'b111))
            dob_r <= mem[addrb];
    end

endmodule
