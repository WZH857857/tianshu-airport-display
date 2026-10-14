`timescale 1ns / 1ps
//=============================================================================
// fade_ctrl.v - frame-boundary fade for the picture path
//
// The picture cache still performs its normal buffer swap.  This block only
// changes the video gain:
//
//   request_pulse -> fade to black -> issue_swap -> wait for the new frame
//                 -> fade back up
//
// All gain changes happen on the video frame boundary, so a fade is uniform
// over the screen instead of moving diagonally with the raster.
//=============================================================================
module fade_ctrl #(
    parameter integer STEP = 4,
    // Frames to wait for the new picture after the swap has been issued.
    // 0 disables the watchdog.  Without it a missing frame_loaded pulse (or a
    // picture that never loads) would leave the screen black forever,
    // because player_ctrl waits for swap_done without a timeout of its own.
    parameter integer WAIT_TIMEOUT = 0
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       frame_tick,
    input  wire       request_pulse,
    output reg        issue_swap,
    input  wire       image_change_pulse,
    output reg  [4:0] level
);

    localparam [1:0] F_IDLE      = 2'd0,
                     F_DARKEN    = 2'd1,
                     F_REQUEST   = 2'd2,
                     F_WAIT_IMG  = 2'd3;

    reg [1:0] state;
    reg [1:0] bright_state;
    reg       pending_request;
    reg [15:0] wait_cnt;

    localparam [15:0] WAIT_LIMIT = WAIT_TIMEOUT;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= F_IDLE;
            bright_state<= 2'd0;
            pending_request <= 1'b0;
            issue_swap  <= 1'b0;
            level       <= 5'd31;
            wait_cnt    <= 16'd0;
        end else begin
            issue_swap <= 1'b0;

            // Remember a request that arrives while a fade is already in
            // progress.  player_ctrl waits for swap_done, so losing this
            // pulse would leave the picture sequencer stuck.
            if (request_pulse)
                pending_request <= 1'b1;

            case (state)
                F_IDLE: begin
                    if (pending_request && (bright_state == 2'd0)) begin
                        pending_request <= 1'b0;
                        state <= F_DARKEN;
                    end else if (!pending_request &&
                                 (bright_state == 2'd0) &&
                                 image_change_pulse) begin
                        // The first real picture has no preceding swap request.
                        // Start it from black as if it had just been swapped.
                        level <= 5'd0;
                        bright_state <= 2'd1;
                    end
                end

                F_DARKEN: begin
                    if (frame_tick) begin
                        if (level <= STEP) begin
                            level      <= 5'd0;
                            issue_swap <= 1'b1;
                            wait_cnt   <= 16'd0;
                            state      <= F_REQUEST;
                        end else begin
                            level <= level - STEP;
                        end
                    end
                end

                F_REQUEST: begin
                    // issue_swap was asserted for one pixel clock.  The top
                    // level converts that pulse to a toggle and the system
                    // clock domain accepts it as the real frame-store request.
                    state <= F_WAIT_IMG;
                end

                F_WAIT_IMG: begin
                    if (image_change_pulse) begin
                        level        <= 5'd0;
                        bright_state <= 2'd1;
                        wait_cnt     <= 16'd0;
                        state        <= F_IDLE;
                    end else if ((WAIT_LIMIT != 16'd0) && frame_tick) begin
                        if (wait_cnt >= (WAIT_LIMIT - 16'd1)) begin
                            // The new picture never arrived.  Fade back in on
                            // whatever the frame store is showing instead of
                            // staying black; player_ctrl keeps its own retry
                            // logic and will request another swap later.
                            level        <= 5'd0;
                            bright_state <= 2'd1;
                            wait_cnt     <= 16'd0;
                            state        <= F_IDLE;
                        end else begin
                            wait_cnt <= wait_cnt + 16'd1;
                        end
                    end
                end

                default: state <= F_IDLE;
            endcase

            // The fade-in is advanced by the same frame tick used by the
            // fade-out.  bright_state is intentionally small and separate so
            // the main state machine stays easy to follow.
            if ((state == F_IDLE) && (bright_state != 2'd0)) begin
                if (frame_tick) begin
                    if (level >= (5'd31 - STEP)) begin
                        level        <= 5'd31;
                        bright_state<= 2'd0;
                    end else begin
                        level <= level + STEP;
                    end
                end
            end
        end
    end

endmodule
