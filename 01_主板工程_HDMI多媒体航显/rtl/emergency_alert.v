`timescale 1ns / 1ps
//=============================================================================
// emergency_alert.v - board level fire alarm annunciation
//
// The player runs on the HX4S20C board (EG4S20BG256).  When the host link
// reports a latched flame alarm this block drives the two annunciators that
// are soldered on that board:
//
//   BUZZER (H11, low level = the transistor conducts and the buzzer sounds)
//
//     The manual only documents the static level, so the pin is toggled at
//     audio rate instead of being held low: a passive buzzer then produces a
//     square tone and an active one is chopped at the same rate.  The alarm
//     is the usual two tone siren: a low pitch for 0.4 s, a high pitch for
//     0.4 s, repeating for as long as the flame alarm is latched.
//
//   Eight digit seven segment display
//
//     SW6 up connects the digits, SW6 down connects the dual colour LED
//     array to the same sixteen pins.  The display is common anode: DIG0..7
//     are the a..g/dp segments (low lights them) and SEL0..7 are the eight
//     digit selects.  On this board the digit commons are driven through
//     inverting transistor drivers, so SEL is low active even though the
//     schematic table calls it "high enable"; all-zero select therefore
//     lights every digit.  Every digit is scanned at 6.1 kHz and the middle
//     three digits (SEL3..SEL5, the centre three of the eight) blink
//     "119", the fire emergency number.
//
//     Both annunciators are driven dark when there is no alarm.  The idle
//     level is all ones: the common anode digits have no selected digit and
//     no lit segments.  The dual colour LED array on the same pins is
//     active high, so SW6 must stay up (digits) for the fire annunciation.
//=============================================================================
module emergency_alert (
    input  wire       clk,          // 50 MHz system clock
    input  wire       rst_n,
    input  wire       fire,         // 1 = latched flame alarm
    output wire       buzzer_n,     // H11, low level = sound
    output reg  [7:0] seg_n,        // DIG0..7 = a,b,c,d,e,f,g,dp, low active
    output reg  [7:0] sel           // SEL0..7 digit select, low active
);

    //------------------------------------------------------------------
    // two tone siren: the pitch steps down/up every 0.4 s
    //------------------------------------------------------------------
    localparam [24:0] SIREN_SWAP = 25'd20_000_000;   // 0.4 s at 50 MHz

    reg [24:0] siren_cnt;
    reg        siren_hi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            siren_cnt <= 25'd0;
            siren_hi  <= 1'b0;
        end else if (siren_cnt == SIREN_SWAP) begin
            siren_cnt <= 25'd0;
            siren_hi  <= ~siren_hi;
        end else begin
            siren_cnt <= siren_cnt + 25'd1;
        end
    end

    reg [15:0] tone_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tone_cnt <= 16'd0;
        else
            tone_cnt <= tone_cnt + 16'd1;
    end

    // about 0.76 kHz and 3.05 kHz, both inside the range of the small
    // magnetic buzzer that is soldered on the board
    wire siren_wave = siren_hi ? tone_cnt[13] : tone_cnt[15];

    assign buzzer_n = ~(fire && siren_wave);

    //------------------------------------------------------------------
    // seven segment scan: scan_cnt[12:10] picks one digit, so a full sweep
    // takes 8192 clocks (164 us) and every digit is refreshed at 6.1 kHz
    //------------------------------------------------------------------
    reg [12:0] scan_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            scan_cnt <= 13'd0;
        else
            scan_cnt <= scan_cnt + 13'd1;
    end

    wire [2:0] digit = scan_cnt[12:10];

    // ~1.5 Hz blink: bit 24 flips every 0.34 s
    reg [24:0] blink_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            blink_cnt <= 25'd0;
        else
            blink_cnt <= blink_cnt + 25'd1;
    end

    wire blink = blink_cnt[24];

    // Common anode segment codes, bit0 = a ... bit6 = g, bit7 = dp.
    // 1 = b,c   9 = a,b,c,d,f,g and the unused digits blank.  SEL0 is the
    // leftmost digit, so SEL3..SEL5 are the centre three and show 119.
    reg [7:0] seg_code;
    always @* begin
        case (digit)
            3'd3:    seg_code = 8'hF9;          // 1
            3'd4:    seg_code = 8'hF9;          // 1
            3'd5:    seg_code = 8'h90;          // 9
            default: seg_code = 8'hFF;          // all segments off
        endcase
    end

    always @* begin
        if (fire && blink) begin
            seg_n = seg_code;
            sel   = ~(8'h01 << digit);
        end else begin
            // All segments and all digit commons high: the common anode
            // digits stay dark while SW6 is up.  The dual colour LED array
            // on the same pins is active high, so SW6 must stay up (digits)
            // for the fire annunciation; with SW6 down the LED array would
            // light instead of showing 119.
            seg_n = 8'hFF;
            sel   = 8'hFF;
        end
    end

endmodule
