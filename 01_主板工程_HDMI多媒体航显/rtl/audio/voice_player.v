`timescale 1ns / 1ps
//=============================================================================
// voice_player.v - replay the voice bank as 48 kHz I2S samples
//
// Runs on the 12.288 MHz audio master clock, one sample per 48 kHz frame.  The
// bank holds mu-law bytes, so every access is expanded back to linear PCM.
//
// The source rate is whatever the WAV file used, converted with a 16.16 phase
// accumulator: the integer part addresses the bank, the top four fraction bits
// interpolate between two neighbouring samples.  Interpolating instead of
// repeating each sample keeps the step harmonics of an 8 kHz recording out of
// the audible band, which matters because the announcement comes out of a
// television or monitor speaker.
//
// The announcement repeats: after the last sample the output stays silent for
// GAP_FRAMES 48 kHz frames and then starts again from the first sample.
//
// One sample is prepared per frame.  The bank read port is synchronous, so the
// value belonging to an address is only on rd_data on the clock after it was
// asked for; the neighbours are therefore captured one clock later than the
// address is issued, and the mixed value is registered two clocks after that.
// Five audio clocks after the frame tick the sample is stable, long before the
// I2S framer latches it at the following tick.
//=============================================================================
module voice_player #(
    parameter integer WORDS      = 65536,
    parameter integer AW         = 16,
    parameter integer GAP_FRAMES = 72000,   // 1.5 s at 48 kHz
    // mu-law only spans 14 bits, so the expanded samples are shifted back up
    // to full scale before they are sent to the I2S framer
    parameter integer GAIN_SHIFT = 2
)(
    input  wire               clk,          // 12.288 MHz audio clock
    input  wire               rst,          // active high, audio domain

    input  wire               ready,        // bank holds a complete voice
    input  wire               frame_tick,   // one clk per 48 kHz frame
    input  wire [31:0]        phase_inc,    // 16.16 source rate increment
    input  wire [AW:0]        words,        // samples stored in the bank

    output reg  [AW-1:0]      rd_addr,
    input  wire [7:0]         rd_data,

    output reg  signed [23:0] sample,       // next sample for the I2S framer
    output wire               generating    // the voice owns the audio output
);

    assign generating = ready;

    localparam [2:0] VS_PRE  = 3'd0,   // waiting for the next frame
                     VS_RD0  = 3'd1,   // first address on the bank
                     VS_RD1  = 3'd2,   // first neighbour lands, ask for the next
                     VS_CALC = 3'd3,   // second neighbour lands
                     VS_MIX  = 3'd4;   // interpolate and advance the phase

    reg [2:0]  vstate;
    reg [23:0] gap_cnt;
    reg        in_gap;
    reg [31:0] phase;
    reg        last_sample;            // the frame being prepared is the last
    reg signed [15:0] s0, s1;
    reg [7:0]         frac;

    // mu-law expander for the byte the bank returned
    wire signed [15:0] pcm_rd;

    ulaw_decode u_decode (
        .code (rd_data),
        .pcm  (pcm_rd)
    );

    wire [AW-1:0] idx_cur  = phase[AW+15:16];
    wire [AW-1:0] idx_inc  = idx_cur + {{(AW-1){1'b0}}, 1'b1};
    // the second neighbour holds on the final sample instead of wrapping, so
    // the last interpolated value stays inside the announcement
    wire [AW-1:0] idx_wrap = (idx_inc >= words[AW-1:0]) ? idx_cur : idx_inc;

    // 4 bit linear interpolation between the two neighbours
    wire signed [16:0] delta = s1 - s0;
    wire signed [20:0] prod  = delta * $signed({1'b0, frac[7:4]});
    wire signed [16:0] mixed = s0 + (prod >>> 4);

    // The interpolated value is widened to the 24 bit I2S word before the gain
    // is applied.  Declaring this narrower than the result silently drops the
    // top of it: in the 18 bit version that used to sit here, mixed = 32124
    // came out as -81920, the low 18 bits of 4111872, and every loud sample
    // wrapped.  mu-law only spans +/-32124, so 32124 << GAIN_SHIFT fits in
    // these 24 bits for any GAIN_SHIFT up to 8.
    wire signed [23:0] gained = {{7{mixed[16]}}, mixed} <<< GAIN_SHIFT;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vstate      <= VS_PRE;
            gap_cnt     <= 24'd0;
            in_gap      <= 1'b0;
            phase       <= 32'd0;
            last_sample <= 1'b0;
            rd_addr     <= {(AW){1'b0}};
            s0          <= 16'sd0;
            s1          <= 16'sd0;
            frac        <= 8'd0;
            sample      <= 24'sd0;
        end else begin
            case (vstate)
                VS_PRE: begin
                    if (!ready) begin
                        // no voice in the bank: stay silent and start over
                        phase       <= 32'd0;
                        in_gap      <= 1'b0;
                        last_sample <= 1'b0;
                        sample      <= 24'sd0;
                    end else if (in_gap) begin
                        if (frame_tick) begin
                            // the sample that was prepared for this frame has
                            // already gone out, so the silence starts here
                            sample <= 24'sd0;
                            if (gap_cnt == 24'd0) begin
                                in_gap <= 1'b0;
                                phase  <= 32'd0;
                            end else begin
                                gap_cnt <= gap_cnt - 24'd1;
                            end
                        end
                    end else if (frame_tick) begin
                        rd_addr <= idx_cur;
                        // the frame about to be prepared is the last one when
                        // the accumulator already sits on the final sample
                        last_sample <= (idx_cur >=
                                        (words[AW-1:0] - {{(AW-1){1'b0}}, 1'b1}));
                        vstate <= VS_RD0;
                    end
                end

                // the first address is on the bank now, ask for the second
                VS_RD0: begin
                    rd_addr <= idx_wrap;
                    vstate  <= VS_RD1;
                end

                // the first neighbour is on rd_data
                VS_RD1: begin
                    s0     <= pcm_rd;
                    frac   <= phase[15:8];
                    vstate <= VS_CALC;
                end

                // the second neighbour is on rd_data
                VS_CALC: begin
                    s1     <= pcm_rd;
                    vstate <= VS_MIX;
                end

                // both neighbours are in registers, so the mix can be formed
                VS_MIX: begin
                    sample <= gained;
                    if (last_sample) begin
                        in_gap  <= 1'b1;
                        gap_cnt <= GAP_FRAMES[23:0];
                    end else begin
                        phase <= phase + phase_inc;
                    end
                    vstate <= VS_PRE;
                end

                default: vstate <= VS_PRE;
            endcase
        end
    end

endmodule
