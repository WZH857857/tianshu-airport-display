`timescale 1ns / 1ps
//=============================================================================
// hdmi_audio_tone_i2s_64fs.v - 48 kHz stereo background music with an I2S
// style serial output.
//
// Derived from the vendor example
//   lab_ex5_i2s/src/user_source/hdl_source/hdmi_audio_tone_i2s_64fs.v
// The vendor example plays a do-re-mi test scale.  This version plays the
// two-tone chime used before airport public address announcements:
//
//   * a three step loop: E5 (659 Hz), then C5 (523 Hz), then a silent pause.
//     The descending major third is the familiar "ding-dong" airport chime.
//   * a triangle waveform instead of the original square wave, plus a per
//     note attack/decay envelope, so each note sounds like a soft bell
//     rather than like a test tone.
//   * the loop never restarts when the picture changes.  I_cue only mixes a
//     short G5 announcement tick over the running chime, so a picture change
//     stays audible without interrupting the background sequence.
//   * when a voice announcement has been loaded from the card it takes over
//     the output completely (I_voice_en): the chime keeps running but is not
//     sent, so a picture change can neither reset nor mix into the voice.
//     I_voice is the sample for the frame after the O_sample_tick pulse.
//   * I_mute is the fire alarm override: it forces silence regardless of the
//     melody, the cue and the voice, so the alarm leaves only the board
//     buzzer audible.
//
// Timing: O_i2s_BCLK is I_mclk / 2 and one frame is 64 bit clocks per
// channel, so with a 12.288 MHz master clock the frame (sample) rate is
// 48 kHz.  Both channels carry the same sample.
//=============================================================================
module hdmi_audio_tone_i2s_64fs #(
    // Triangle peak is 64 << BGM_SHIFT counts, that is 2.097 M with the
    // default shift, which is the level the original test tone used.
    parameter integer BGM_SHIFT   = 15,
    parameter integer CUE_SHIFT   = 14,     // chime sits 6 dB under the melody
    parameter integer UNIT_FRAMES = 2400,   // duration unit, 50 ms at 48 kHz
    parameter integer CUE_FRAMES  = 9600    // chime length, 200 ms at 48 kHz
)(
    input  wire I_mclk,     // 12.288 MHz audio master clock
    input  wire I_rst,
    input  wire I_cue,      // one cycle pulse: mix the chime, never restart
    input  wire I_voice_en, // 1 = the card voice owns the audio output
    input  wire signed [23:0] I_voice,   // next 48 kHz sample from the bank
    input  wire I_mute,     // 1 = fire alarm: force the output to silence
    output wire O_sample_tick,           // one I_mclk cycle per 48 kHz frame
    output reg  O_i2s_BCLK,
    output reg  O_i2s_LRCK,
    output reg  O_i2s_DOUT
);

    //------------------------------------------------------------------
    // melody tables
    //------------------------------------------------------------------
    // Note numbers 0..14 are C4..C6, 4'hF is a rest.
    // The increments are precomputed as f * 2^32 / 48000.
    function [31:0] note_inc_lut;
        input [3:0] idx;
        begin
            case (idx)
                4'd0: note_inc_lut = 32'd23409862; // C4 261.63 Hz
                4'd1: note_inc_lut = 32'd26276681; // D4 293.66 Hz
                4'd2: note_inc_lut = 32'd29494578; // E4 329.63 Hz
                4'd3: note_inc_lut = 32'd31248410; // F4 349.23 Hz
                4'd4: note_inc_lut = 32'd35075155; // G4 392.00 Hz
                4'd5: note_inc_lut = 32'd39370534; // A4 440.00 Hz
                4'd6: note_inc_lut = 32'd44191930; // B4 493.88 Hz
                4'd7: note_inc_lut = 32'd46819716; // C5 523.25 Hz
                4'd8: note_inc_lut = 32'd52553357; // D5 587.33 Hz
                4'd9: note_inc_lut = 32'd58989149; // E5 659.26 Hz
                4'd10: note_inc_lut = 32'd62496826; // F5 698.46 Hz
                4'd11: note_inc_lut = 32'd70150316; // G5 783.99 Hz
                4'd12: note_inc_lut = 32'd78741067; // A5 880.00 Hz
                4'd13: note_inc_lut = 32'd88383859; // B5 987.77 Hz
                4'd14: note_inc_lut = 32'd93639437; // C6 1046.50 Hz
                default: note_inc_lut = 32'd0;
            endcase
        end
    endfunction

    // One phrase step: {note number, duration in UNIT_FRAMES units}.
    // The loop is the airport public address "ding-dong": E5, then C5, then
    // silence.  At UNIT_FRAMES = 2400 (50 ms) the whole loop is 2.4 s long.
    function [11:0] melody_lut;
        input [3:0] step;
        begin
            case (step)
                4'd0 : melody_lut = {4'd9, 8'd10}; // E5, 500 ms
                4'd1 : melody_lut = {4'd7, 8'd14}; // C5, 700 ms
                default: melody_lut = {4'hF, 8'd24}; // pause, 1.2 s
            endcase
        end
    endfunction

    // The phrase is three steps long; the rest of the 16 entry table is not
    // used but keeps the ROM-style decoder simple.
    localparam [3:0] PHRASE_LAST = 4'd2;

    // 784.0 Hz (G5) announcement tick.  G5 is a fifth above C5 and a minor
    // third above E5, so it sits on top of either chime note without clashing.
    localparam [31:0] CUE_INC = 32'd70150316;

    // Envelope step sizes.  All of them are derived from the two length
    // parameters so a shortened simulation keeps the same envelope shape.
    localparam [11:0] ATK_STEP_V = (UNIT_FRAMES + 7) / 8;   //  about 6 ms
    localparam [11:0] DEC_STEP_V = UNIT_FRAMES * 2;         //  about 100 ms
    localparam [11:0] REL_STEP_V = (UNIT_FRAMES + 7) / 8;   //  about 6 ms
    localparam [11:0] CUE_ATK_V  = (CUE_FRAMES + 63) / 64;  //  about 3 ms
    localparam [11:0] CUE_DEC_V  = (CUE_FRAMES + 15) / 16;  //  about 13 ms

    localparam [11:0] UNIT_LAST  = UNIT_FRAMES - 1;
    localparam [13:0] CUE_LAST   = CUE_FRAMES - 1;

    localparam [1:0] PH_ATT = 2'd0;
    localparam [1:0] PH_DEC = 2'd1;
    localparam [1:0] PH_REL = 2'd2;

    reg  [5:0]  S_bit_cnt;
    reg  [63:0] S_shift_reg;

    // background voice
    reg  [31:0] S_phase_acc;
    reg  [3:0]  S_note_step;
    reg  [7:0]  S_dur_left;     // duration units left in the current step
    reg  [11:0] S_unit_cnt;     // frames elapsed inside the current unit
    reg  [16:0] S_note_pos;     // frames elapsed since the note started
    reg  [2:0]  S_att;          // 0 = full scale, 7 = about -42 dB
    reg  [1:0]  S_env_ph;
    reg  [11:0] S_env_cnt;

    // chime voice
    reg  [31:0] S_cue_acc;
    reg  [13:0] S_cue_pos;
    reg  [2:0]  S_cue_att;
    reg  [1:0]  S_cue_ph;
    reg  [11:0] S_cue_cnt;
    reg         S_cue_run;
    reg         S_cue_req;

    reg  signed [23:0] S_sample_word;
    reg  signed [23:0] S_sample_next;

    //------------------------------------------------------------------
    // current phrase step and waveform
    //------------------------------------------------------------------
    wire [11:0] W_entry    = melody_lut(S_note_step);
    wire [3:0]  W_note_id  = W_entry[11:8];
    wire        W_rest     = (W_note_id == 4'hF);
    wire [31:0] W_note_inc = W_rest ? 32'd0 : note_inc_lut(W_note_id);

    wire [3:0]  W_next_step  = (S_note_step == PHRASE_LAST) ? 4'd0 :
                               S_note_step + 4'd1;
    wire [11:0] W_next_entry = melody_lut(W_next_step);
    wire [7:0]  W_next_dur   = W_next_entry[7:0];
    // step 0 of the phrase, used to seed the very first note
    wire [11:0] W_first_entry = melody_lut(4'd0);

    // last frame of the current duration unit, and of the whole note
    wire W_step_end  = (S_unit_cnt == UNIT_LAST);
    wire W_new_note  = W_step_end && (S_dur_left <= 8'd1);
    // the note is about to enter its last duration unit
    wire W_rel_start = W_step_end && (S_dur_left == 8'd2);

    // triangle wave taken from the top of the phase accumulator
    wire [7:0] W_saw    = S_phase_acc[31:24];
    wire [7:0] W_tri    = W_saw[7] ? (8'd255 - W_saw) : W_saw;  // 0..127
    wire signed [7:0]  W_tri_s = $signed({1'b0, W_tri[6:0]}) - 8'sd64;
    wire signed [23:0] W_tri24 = {{16{W_tri_s[7]}}, W_tri_s};
    wire signed [23:0] W_bgm   = W_rest ? 24'sd0 :
                                 ((W_tri24 <<< BGM_SHIFT) >>> S_att);

    wire [7:0] W_cue_saw = S_cue_acc[31:24];
    wire [7:0] W_cue_tri = W_cue_saw[7] ? (8'd255 - W_cue_saw) : W_cue_saw;
    wire signed [7:0]  W_cue_s = $signed({1'b0, W_cue_tri[6:0]}) - 8'sd64;
    wire signed [23:0] W_cue24 = {{16{W_cue_s[7]}}, W_cue_s};
    wire signed [23:0] W_cue   = S_cue_run ?
                                 ((W_cue24 <<< CUE_SHIFT) >>> S_cue_att) :
                                 24'sd0;

    // Both voices together stay well under full scale with the default
    // shifts, so the clamp below never engages.  It is there so that a later
    // gain change can not wrap the 24 bit I2S word.
    wire signed [24:0] W_sum = {W_bgm[23], W_bgm} + {W_cue[23], W_cue};
    wire signed [23:0] W_mix = (W_sum > 25'sd8388607) ? 24'sh7FFFFF :
                               ((W_sum < -25'sd8388608) ? 24'sh800000 :
                                W_sum[23:0]);

    // A frame ends on the last bit clock of its right channel, which is the
    // cycle that latches the sample for the frame after this one.
    wire W_frame_end = (O_i2s_BCLK == 1'b1) && (S_bit_cnt == 6'd63) &&
                       (O_i2s_LRCK == 1'b1);

    assign O_sample_tick = W_frame_end;

    // The mute request comes from the 50 MHz system clock domain, so it is
    // re-timed here before it touches the sample stream.
    reg mute_s1, mute_s2;
    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            mute_s1 <= 1'b0;
            mute_s2 <= 1'b0;
        end else begin
            mute_s1 <= I_mute;
            mute_s2 <= mute_s1;
        end
    end

    wire signed [23:0] W_next = mute_s2 ? 24'sd0 :
                                (I_voice_en ? I_voice : W_mix);

    //------------------------------------------------------------------
    // envelope stepper, shared by both voices
    //   returns {next phase, next attenuation, next step countdown}
    //------------------------------------------------------------------
    function [16:0] env_step;
        input [1:0]  ph;
        input [2:0]  att;
        input [11:0] cnt;
        input [11:0] atk_step;
        input [11:0] dec_step;
        input [11:0] rel_step;
        reg   [1:0]  n_ph;
        reg   [2:0]  n_att;
        reg   [11:0] n_cnt;
        begin
            if (cnt != 12'd0) begin
                n_ph  = ph;
                n_att = att;
                n_cnt = cnt - 12'd1;
            end else begin
                case (ph)
                    PH_ATT: begin
                        if (att == 3'd0) begin
                            n_ph  = PH_DEC;
                            n_att = 3'd0;
                            n_cnt = dec_step;
                        end else begin
                            n_ph  = PH_ATT;
                            n_att = att - 3'd1;
                            n_cnt = atk_step;
                        end
                    end
                    PH_DEC: begin
                        n_ph  = PH_DEC;
                        n_att = (att == 3'd7) ? 3'd7 : att + 3'd1;
                        n_cnt = dec_step;
                    end
                    default: begin
                        n_ph  = PH_REL;
                        n_att = (att == 3'd7) ? 3'd7 : att + 3'd1;
                        n_cnt = rel_step;
                    end
                endcase
            end
            env_step = {n_ph, n_att, n_cnt};
        end
    endfunction

    wire [16:0] W_env_next = env_step(S_env_ph, S_att, S_env_cnt,
                                      ATK_STEP_V, DEC_STEP_V, REL_STEP_V);
    wire [16:0] W_cue_next = env_step(S_cue_ph, S_cue_att, S_cue_cnt,
                                      CUE_ATK_V, CUE_DEC_V, CUE_DEC_V);

    //------------------------------------------------------------------
    // I2S framing, with one sample rate event per 48 kHz frame
    //------------------------------------------------------------------
    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            O_i2s_BCLK    <= 1'b0;
            O_i2s_LRCK    <= 1'b0;
            O_i2s_DOUT    <= 1'b0;
            S_bit_cnt     <= 6'd0;
            S_shift_reg   <= 64'd0;

            S_phase_acc   <= 32'd0;
            S_note_step   <= 4'd0;
            S_dur_left    <= W_first_entry[7:0];
            S_unit_cnt    <= 12'd0;
            S_note_pos    <= 17'd0;
            S_att         <= 3'd4;
            S_env_ph      <= PH_ATT;
            S_env_cnt     <= ATK_STEP_V;

            S_cue_acc     <= 32'd0;
            S_cue_pos     <= 14'd0;
            S_cue_att     <= 3'd7;
            S_cue_ph      <= PH_DEC;
            S_cue_cnt     <= 12'd0;
            S_cue_run     <= 1'b0;
            S_cue_req     <= 1'b0;

            S_sample_word <= 24'sd0;
            S_sample_next <= 24'sd0;
        end else begin
            // The cue pulse comes from the audio clock domain, but it does
            // not have to land on a frame boundary, so it is latched here.
            if (I_cue)
                S_cue_req <= 1'b1;

            O_i2s_BCLK <= ~O_i2s_BCLK;

            if (O_i2s_BCLK == 1'b1) begin
                O_i2s_DOUT  <= S_shift_reg[63];
                S_shift_reg <= {S_shift_reg[62:0], 1'b0};

                if (S_bit_cnt == 6'd63) begin
                    S_bit_cnt <= 6'd0;

                    if (O_i2s_LRCK == 1'b0) begin
                        // left channel done, the right channel repeats it
                        O_i2s_LRCK  <= 1'b1;
                        S_shift_reg <= {S_sample_word[23:0], 40'd0};
                    end else begin
                        // right channel done: one 48 kHz frame is complete
                        O_i2s_LRCK    <= 1'b0;
                        S_sample_word <= S_sample_next;
                        S_shift_reg   <= {S_sample_next[23:0], 40'd0};
                        S_sample_next <= W_next;

                        //-- background voice ------------------------------
                        S_phase_acc <= S_phase_acc + W_note_inc;

                        if (W_step_end) begin
                            S_unit_cnt <= 12'd0;

                            if (W_new_note) begin
                                // next phrase step, envelope back to attack
                                S_note_step <= W_next_step;
                                S_dur_left  <= W_next_dur;
                                S_note_pos  <= 17'd0;
                                S_att       <= 3'd4;
                                S_env_ph    <= PH_ATT;
                                S_env_cnt   <= ATK_STEP_V;
                            end else begin
                                S_dur_left <= S_dur_left - 8'd1;
                                S_note_pos <= S_note_pos + 17'd1;

                                if (W_rel_start) begin
                                    // last duration unit of this note
                                    S_env_ph  <= PH_REL;
                                    S_env_cnt <= REL_STEP_V;
                                end else begin
                                    {S_env_ph, S_att, S_env_cnt} <= W_env_next;
                                end
                            end
                        end else begin
                            S_unit_cnt <= S_unit_cnt + 12'd1;
                            S_note_pos <= S_note_pos + 17'd1;
                            {S_env_ph, S_att, S_env_cnt} <= W_env_next;
                        end

                        //-- chime voice -----------------------------------
                        if (S_cue_req) begin
                            S_cue_req <= 1'b0;
                            S_cue_run <= 1'b1;
                            S_cue_acc <= 32'd0;
                            S_cue_pos <= 14'd0;
                            S_cue_att <= 3'd3;
                            S_cue_ph  <= PH_ATT;
                            S_cue_cnt <= CUE_ATK_V;
                        end else if (S_cue_run) begin
                            if (S_cue_pos == CUE_LAST) begin
                                S_cue_run <= 1'b0;
                                S_cue_att <= 3'd7;
                            end else begin
                                S_cue_acc <= S_cue_acc + CUE_INC;
                                S_cue_pos <= S_cue_pos + 14'd1;
                                {S_cue_ph, S_cue_att, S_cue_cnt} <= W_cue_next;
                            end
                        end
                    end
                end else begin
                    S_bit_cnt <= S_bit_cnt + 6'd1;
                end
            end
        end
    end

endmodule
