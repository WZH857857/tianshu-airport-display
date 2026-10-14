`timescale 1ns / 1ps
//=============================================================================
// ulaw_codec.v - G.711 mu-law compressor and expander
//
// The voice bank holds one byte per sample, so the announcement is stored in
// mu-law instead of linear PCM.  Compared with 16 bit linear PCM at the same
// 8 kHz sampling rate the byte budget is halved, and because mu-law is a
// logarithmic law the small amplitudes of speech are still resolved about as
// well as they would be with 12 bit linear samples.
//
// Both blocks are pure combinational logic.  ulaw_encode is used while the
// WAV file is streamed into the bank, ulaw_decode on the audio clock while the
// announcement plays.
//=============================================================================

//-----------------------------------------------------------------------------
// linear 16 bit PCM -> 8 bit mu-law
//-----------------------------------------------------------------------------
module ulaw_encode (
    input  wire signed [15:0] pcm,
    output reg        [7:0]  code
);
    localparam [15:0] BIAS = 16'd132;
    localparam [15:0] CLIP = 16'd32635;

    // the mu-law word carries the inverted sample, so the whole code is built
    // and then complemented: a positive sample ends up with the top bit set
    wire              sign = pcm[15];
    wire [15:0] mag_in = pcm[15] ? (~pcm + 16'd1) : pcm;
    wire [15:0] mag    = (mag_in > CLIP) ? CLIP : mag_in;
    wire [15:0] biased = mag + BIAS;

    // exponent is the position of the highest set bit of the biased magnitude
    // minus 7, saturated at 0 (the classic exp_lut[...] table)
    reg [2:0] exp;

    always @* begin
        if      (biased[15]) exp = 3'd7;
        else if (biased[14]) exp = 3'd7;
        else if (biased[13]) exp = 3'd6;
        else if (biased[12]) exp = 3'd5;
        else if (biased[11]) exp = 3'd4;
        else if (biased[10]) exp = 3'd3;
        else if (biased[9])  exp = 3'd2;
        else if (biased[8])  exp = 3'd1;
        else                 exp = 3'd0;
    end

    // mantissa = (biased >> (exp + 3)) & 0xF, that is the four bits that sit
    // directly above the exponent's leading one
    wire [3:0] mant = biased[exp + 6 -: 4];

    always @* code = ~{sign, exp, mant};
endmodule

//-----------------------------------------------------------------------------
// 8 bit mu-law -> linear 16 bit PCM
//-----------------------------------------------------------------------------
module ulaw_decode (
    input  wire [7:0]  code,
    output reg signed [15:0] pcm
);
    wire [7:0]  u    = ~code;
    wire        sign = u[7];
    wire [2:0]  exp  = u[6:4];
    wire [3:0]  mant = u[3:0];

    // (mant << 3) + 0x84 rebuilds the biased magnitude, the shift applies the
    // exponent and subtracting the bias again returns the linear sample
    wire [15:0] mag = (({6'd0, mant, 3'b000} + 16'd132) << exp) - 16'd132;

    always @* pcm = sign ? -$signed({1'b0, mag[14:0]})
                         :  $signed({1'b0, mag[14:0]});
endmodule
