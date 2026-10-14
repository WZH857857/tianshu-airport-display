`timescale 1ns / 1ps
//=============================================================================
// pulse_cdc.v - move a one-cycle pulse between two clock domains
//
// A single-cycle pulse cannot be sampled safely by a slower clock: a 20 ns
// pulse (50 MHz domain) can fall entirely inside the 40 ns period of the
// 25 MHz pixel clock and disappear.  The pulse is therefore converted into a
// toggle in the source domain, carried across by a three-flop synchroniser
// and turned back into a one-cycle pulse in the destination domain.
//
// One pulse in gives exactly one pulse out, whatever the phase relationship
// between the two clocks, as long as the toggle is given time to propagate
// (the caller must not issue the next pulse within two destination clocks).
//=============================================================================
module pulse_cdc (
    input  wire clk_src,
    input  wire rst_src,        // async, active high, source domain
    input  wire pulse_in,       // one clk_src cycle

    input  wire clk_dst,
    input  wire rst_dst,        // async, active high, destination domain
    output wire pulse_out       // one clk_dst cycle
);

    reg tog_src;

    always @(posedge clk_src or posedge rst_src) begin
        if (rst_src)
            tog_src <= 1'b0;
        else if (pulse_in)
            tog_src <= ~tog_src;
    end

    reg [2:0] sync_dst;

    always @(posedge clk_dst or posedge rst_dst) begin
        if (rst_dst)
            sync_dst <= 3'd0;
        else
            sync_dst <= {sync_dst[1:0], tog_src};
    end

    assign pulse_out = sync_dst[2] ^ sync_dst[1];

endmodule
