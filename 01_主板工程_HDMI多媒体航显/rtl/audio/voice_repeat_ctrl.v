`timescale 1ns / 1ps
//=============================================================================
// voice_repeat_ctrl.v - schedule repeated card voice announcements
//
// The first announcement starts as soon as the directory scan has found the
// WAV file and the picture cache is ready.  A successful announcement waits
// gap_seconds before starting the same file again.  A stream error stops the
// loop and leaves the built-in chime in charge.
//
// The gap is counted in one-second ticks derived from the 50 MHz system
// clock.  The tick period is a parameter and the gap length is a port, so one
// controller serves both the one-minute normal announcement and the short
// fire-alarm loop; a shortened tick parameter lets a simulation exercise the
// same state machine without waiting for a real minute.
//=============================================================================
module voice_repeat_ctrl #(
    parameter [25:0] GAP_TICK_CYCLES = 26'd50_000_000
)(
    input  wire clk,
    input  wire rst_n,

    input  wire [7:0] gap_seconds,  // announcement interval
    input  wire voice_ready,   // scan_done && voice_found && cache_ready
    input  wire stream_done,   // one cycle when the WAV data chunk is complete
    input  wire stream_error,  // parser error or watchdog, held until restart

    output reg  start_pulse,   // one cycle to start a new stream session
    output wire waiting        // high while the one-minute gap is running
);

    localparam [1:0] S_FIRST  = 2'd0,
                     S_ACTIVE = 2'd1,
                     S_GAP    = 2'd2,
                     S_STOP   = 2'd3;

    // A zero parameter would make the decrement below wrap.  Real builds use
    // the defaults; the guard keeps a shortened testbench parameter safe.
    localparam [25:0] TICK_LAST =
        (GAP_TICK_CYCLES == 26'd0) ? 26'd0 : (GAP_TICK_CYCLES - 26'd1);
    wire [7:0] second_last =
        (gap_seconds == 8'd0) ? 8'd0 : (gap_seconds - 8'd1);

    reg [1:0]  state;
    reg [25:0] tick_cnt;
    reg [7:0]  second_cnt;

    // Driven straight from the state so the flag is level-true for exactly
    // the cycles spent in the gap.  A registered copy would stay high for one
    // extra cycle and overlap the restart pulse.
    assign waiting = (state == S_GAP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_FIRST;
            tick_cnt   <= 26'd0;
            second_cnt <= 8'd0;
            start_pulse <= 1'b0;
        end else begin
            start_pulse <= 1'b0;

            case (state)
                S_FIRST: begin
                    if (voice_ready) begin
                        start_pulse <= 1'b1;
                        state       <= S_ACTIVE;
                    end
                end

                S_ACTIVE: begin
                    if (stream_error) begin
                        state <= S_STOP;
                    end else if (stream_done) begin
                        tick_cnt   <= 26'd0;
                        second_cnt <= 8'd0;
                        state      <= S_GAP;
                    end
                end

                S_GAP: begin
                    if (voice_ready) begin
                        if (tick_cnt == TICK_LAST) begin
                            tick_cnt <= 26'd0;
                            if (second_cnt == second_last) begin
                                second_cnt  <= 8'd0;
                                start_pulse <= 1'b1;
                                state       <= S_ACTIVE;
                            end else begin
                                second_cnt <= second_cnt + 8'd1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 26'd1;
                        end
                    end
                end

                S_STOP: begin
                    // A malformed file or a card stall does not create an
                    // endless retry loop.  Reset the board to try again.
                end

                default: state <= S_FIRST;
            endcase
        end
    end

endmodule
