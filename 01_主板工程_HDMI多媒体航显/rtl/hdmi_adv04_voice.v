`timescale 1ns / 1ps
//=============================================================================
// hdmi_adv04_voice.v - overlay + fade + size adaptation, with the card voice
// announcement
//
// Everything ADV02/ADV03 deliver is kept as it was: the terminal overlay
// layer, the fade through black on a picture change, the 6 s rotation and the
// picture size adaptation that resizes any source geometry to 640 x 480 on
// the way into the frame store.
//
// Advanced item 4 is enabled here as well: KEY2 adjusts the selected display
// parameter, KEY4 selects it (brightness / contrast) and the value is drawn in
// the parameter panel of the overlay while it is being changed.  A one second
// hold on KEY4 keeps the earlier overlay-layout cycling.
//
// Added here is the airport announcement that comes from the card: the first
// .WAV file in the root directory is streamed straight off the card, one
// mu-law byte per sample, once the six resident frames are cached.  There is
// no on chip voice bank any more, so the announcement may be a minute or
// longer.  The stream is started exactly once and the audio side never looks
// at the picture path again, so changing a picture can neither restart nor
// interrupt the announcement.  When the stream finishes the internal airport
// chime takes over again.  A card without a .WAV file behaves like ADV02.
//
// ADV11 adds the serial host link.  The CH340 bridge that powers the board
// also carries the UART, so a PC application can switch one of the six
// cached pictures, push its own scrolling message, drive the picture-advance
// mode, adjust brightness / contrast and pick the OSD layout while the player
// keeps running.  Everything the host asks for is executed by this FPGA; the
// PC is only the remote control.
//
// ADV14 mirrors the finished TMDS stream onto HDMI socket B as well as
// socket A.  Both connectors therefore carry the identical picture and audio,
// which is what the board allows without a second video pipeline: the single
// transmitter core still drives one TMDS word stream, and only the physical
// output pairs are duplicated onto the B pins.  Only socket A carries DDC,
// because that is the link the transmitter core reads the monitor EDID from.
//
// ADV18 added the UART TIME command.  ADV19 keeps that fallback and also
// reads the DS1307 directly on H15/H16, so the top-right readout can follow
// the real RTC while the untouched E104 console remains on its own board.
//
// ADV21 adds the fire emergency mode.  The E104 emergency board latches a
// flame alarm and the host application mirrors it over the main serial link
// (command 0x23).  While that flag is set the overlay paints the full screen
// evacuation board - hazard rails, the warning triangle with a flame and the
// two text lines - and this top level drives the on-board buzzers and the
// eight digit display through emergency_alert.v.  Clearing the alarm on the
// E104 board (its "danger cleared" button) sends 0x23 with a zero and the
// player returns to the normal picture.
//=============================================================================
`include "emergency_alert.v"

module hdmi_adv04_voice (
    input  wire       sys_clk,          // 50 MHz
    input  wire       sys_rst_n,
    input  wire       key_mode,         // active low, next picture
    input  wire       key_bright,
    input  wire       key_ui,
    input  wire       key_auto,         // SW1: 1 = automatic rotation

    input  wire       uart_rx_pin,      // CH340 TXD -> FPGA RXD, pin F12
    output wire       uart_tx_pin,      // FPGA TXD -> CH340 RXD, pin D12

    // DS1307 RTC on the J1 expansion header: J1.25 = H15, J1.26 = H16.
    output wire       rtc_scl,
    inout  wire       rtc_sda,

    // Fire alarm annunciators on the same board
    output wire       buzzer,           // H11, low level = sound
    output wire [7:0] seg_n,            // DIG0..7, segments a..g/dp
    output wire [7:0] sel,              // SEL0..7 digit selects

    output wire [3:0] led,

    output wire       HDMI_CLK_P,
    output wire       HDMI_D2_P,
    output wire       HDMI_D1_P,
    output wire       HDMI_D0_P,

    output wire       HDMI_DDC_SCL,
    inout  wire       HDMI_DDC_SDA,

    // HDMI socket B: the same TMDS stream as socket A
    output wire       HDMI_B_CLK_P,
    output wire       HDMI_B_D2_P,
    output wire       HDMI_B_D1_P,
    output wire       HDMI_B_D0_P,
    output wire       HDMI_B_DDC_SCL,
    inout  wire       HDMI_B_DDC_SDA,

    output wire       SD_nCS,
    output wire       SD_DCLK,
    output wire       SD_MOSI,
    input  wire       SD_MISO
);

    // Latched flame alarm, mirrored from the host link by uart_ctrl.
    wire emg_fire;

    hdmi_mm_player #(
        .FEATURE_UI         (1),
        .FEATURE_LOGO       (1),
        .FEATURE_SUBTITLE   (1),
        .FEATURE_TIMESTAMP  (1),
        .FEATURE_PARALLAX   (1),
        .FEATURE_FADE       (1),
        // Item 4: real time brightness / contrast adjustment with the value
        // shown on the picture while it changes.
        .FEATURE_BRIGHTNESS (1),
        .FEATURE_OSD        (1),
        // Advanced item 5: live waveform plus the eight-band energy
        // spectrum in the lower-right overlay panel.
        .FEATURE_AUDIO_WAVE (1),
        // Advanced item 3: a picture whose size is not the display size is
        // resized on the way into the frame store instead of being rejected.
        .FEATURE_SCALE      (1),
        .FADE_STEP          (2),
        .FADE_WAIT_TIMEOUT  (180),
        .AUTO_HOLD_CYCLES   (300_000_000),  // 6 s per picture
        // 40 s is the ADV03 value: a 1600 x 1200 picture carries 5.8 MB over
        // the SPI link where a 640 x 480 one carries 0.9 MB, and the 20 s
        // base default would time the load out halfway through such a file.
        .LOAD_CYCLES        (2_000_000_000)
    ) u_player (
        .sys_clk       (sys_clk),
        .sys_rst_n     (sys_rst_n),
        .key_mode      (key_mode),
        .key_bright    (key_bright),
        .key_ui        (key_ui),
        .key_auto      (key_auto),
        .uart_rx_pin   (uart_rx_pin),
        .uart_tx_pin   (uart_tx_pin),
        .rtc_scl       (rtc_scl),
        .rtc_sda       (rtc_sda),
        .emg_fire      (emg_fire),
        .led           (led),
        .HDMI_CLK_P    (HDMI_CLK_P),
        .HDMI_D2_P     (HDMI_D2_P),
        .HDMI_D1_P     (HDMI_D1_P),
        .HDMI_D0_P     (HDMI_D0_P),
        .HDMI_DDC_SCL  (HDMI_DDC_SCL),
        .HDMI_DDC_SDA  (HDMI_DDC_SDA),
        .SD_nCS        (SD_nCS),
        .SD_DCLK       (SD_DCLK),
        .SD_MOSI       (SD_MOSI),
        .SD_MISO       (SD_MISO)
    );

    // Board level fire annunciation: T3 style buzzer pattern plus the eight
    // digit display blinking FIRE.
    emergency_alert u_emergency_alert (
        .clk        (sys_clk),
        .rst_n      (sys_rst_n),
        .fire       (emg_fire),
        .buzzer_n   (buzzer),
        .seg_n      (seg_n),
        .sel        (sel)
    );

    // Socket B is a passive mirror of socket A.  The transmitter core and the
    // TMDS serialisers are instantiated once, and both output pin pairs see
    // the same physical stream.
    assign HDMI_B_CLK_P = HDMI_CLK_P;
    assign HDMI_B_D2_P  = HDMI_D2_P;
    assign HDMI_B_D1_P  = HDMI_D1_P;
    assign HDMI_B_D0_P  = HDMI_D0_P;

    // The core drives the DDC lines with its own tristate control, so the B
    // pair is simply tied to the A pair and shares the same EDID read.
    assign HDMI_B_DDC_SCL = HDMI_DDC_SCL;
    assign HDMI_B_DDC_SDA = HDMI_DDC_SDA;

endmodule
