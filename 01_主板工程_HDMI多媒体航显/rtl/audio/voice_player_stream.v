`timescale 1ns / 1ps
//=============================================================================
// voice_player_stream.v - replay a mu-law byte stream at 48 kHz
//
// The card side pushes one mu-law byte per source sample into an
// asynchronous FIFO.  This module reads that FIFO with the audio clock and
// linearly interpolates the source rate up to the 48 kHz HDMI frame rate.
//
// async_fifo has a registered read port.  Asserting rd_en at cycle N makes
// rd_data valid at cycle N+1.  A two-entry audio-side queue below keeps the
// fetched word until interpolation needs it; the queue is small on purpose
// and removes the race where a newly fetched word could be overwritten by the
// same frame that consumes the previous one.
//
// phase is a 16.16 position between cur_sample and q_front.  It may become
// >= 1.0 after a frame; the next frame normalises it by advancing one source
// sample.  This keeps the interpolation phase aligned with the actual frame
// instead of delaying every wrap by one 48 kHz frame.
//
// The stream is played once.  After the source is done, a short stable-empty
// qualification makes sure the grey-code write pointer from the 50 MHz domain
// has really caught up before the built-in chime is released.
//=============================================================================
module voice_player_stream #(
    parameter integer GAIN_SHIFT = 7,
    parameter integer DRAIN_HOLD = 3
)(
    input  wire               clk,          // 12.288 MHz audio clock
    input  wire               rst,          // active high, audio domain

    input  wire               start_tog,    // one toggle when streaming starts
    input  wire               done_tog,     // one toggle when the card is done
    input  wire               frame_tick,   // one tick per 48 kHz frame
    input  wire [31:0]        phase_inc,    // 16.16 source rate increment

    input  wire [7:0]         fifo_data,
    input  wire               fifo_empty,
    output reg                fifo_rd_en,

    output reg  signed [23:0] sample,
    output reg                generating
);

    // Synchronise the two event levels into the audio clock domain.
    reg [2:0] start_sync;
    reg [2:0] done_sync;
    wire      start_pulse = start_sync[2] ^ start_sync[1];
    wire      done_pulse  = done_sync[2]  ^ done_sync[1];

    reg               running;
    reg               source_done;
    reg               cur_valid;
    reg               rd_pending;
    reg               rd_capture;
    reg signed [15:0] cur_sample;
    reg [31:0]        phase;
    reg [1:0]         drain_cnt;

    // Two-entry queue between the registered FIFO output and interpolation.
    reg signed [15:0] q_mem [0:1];
    reg [0:0]         q_head;
    reg [1:0]         q_count;

    wire signed [15:0] pcm_rd;
    wire signed [15:0] q_front = q_mem[q_head];
    wire signed [15:0] q_other = q_mem[q_head ^ 1'b1];
    wire [0:0]         q_tail  = q_head ^ q_count[0];
    wire               push    = rd_capture;
    wire               frame_fire = running && frame_tick;

    // A phase at or above 1.0 means the next source sample is the current
    // sample for this frame.  One advance per frame is enough because the
    // supported source rates never exceed 48 kHz.
    wire               advance = frame_fire && cur_valid &&
                                 (phase >= 32'd65536) &&
                                 (q_count != 2'd0);
    wire               load_first = frame_fire && !cur_valid &&
                                    (q_count != 2'd0);
    wire               pop = load_first || advance;

    wire [31:0]        eff_phase = advance ? (phase - 32'd65536) : phase;
    wire signed [15:0] eff_cur = (!cur_valid || advance) ? q_front
                                                         : cur_sample;
    wire signed [15:0] eff_next = (!cur_valid) ? q_front :
                                  (advance ? ((q_count == 2'd2) ? q_other
                                                                : q_front)
                                           : q_front);

    ulaw_decode u_decode (
        .code (fifo_data),
        .pcm  (pcm_rd)
    );

    // Interpolate from the current sample towards the queue front.  The phase
    // fractional part is eight bits.  The multiply is kept wide and shifted
    // by eight, so a full-scale sample step cannot overflow the product.
    wire signed [16:0] delta = eff_next - eff_cur;
    wire signed [25:0] prod  = delta * $signed({1'b0, eff_phase[15:8]});
    wire signed [16:0] mixed = eff_cur + (prod >>> 8);

    function signed [23:0] gain_pcm;
        input signed [16:0] value;
        begin
            gain_pcm = {{7{value[16]}}, value} <<< GAIN_SHIFT;
        end
    endfunction

    // Only finish after the 50 MHz write pointer has had time to cross the
    // grey-code synchroniser.  Without this qualification a stale rd_empty
    // can release the chime while the final FIFO word is still in flight.
    wire drain_ready = (drain_cnt >= DRAIN_HOLD);
    wire stable_empty = source_done && fifo_empty && !rd_pending &&
                        !rd_capture && (q_count == 2'd0);
    wire can_finish = stable_empty && drain_ready;
    wire last_frame_done = (phase + phase_inc) >= 32'd65536;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            start_sync <= 3'd0;
            done_sync  <= 3'd0;
            running    <= 1'b0;
            source_done <= 1'b0;
            cur_valid  <= 1'b0;
            rd_pending <= 1'b0;
            rd_capture <= 1'b0;
            cur_sample <= 16'sd0;
            phase      <= 32'd0;
            drain_cnt  <= 2'd0;
            q_head     <= 1'b0;
            q_count    <= 2'd0;
            fifo_rd_en <= 1'b0;
            sample     <= 24'sd0;
            generating <= 1'b0;
        end else begin
            start_sync <= {start_sync[1:0], start_tog};
            done_sync  <= {done_sync[1:0],  done_tog};
            fifo_rd_en <= 1'b0;

            if (start_pulse) begin
                running     <= 1'b1;
                source_done <= 1'b0;
                cur_valid   <= 1'b0;
                rd_pending  <= 1'b0;
                rd_capture  <= 1'b0;
                cur_sample  <= 16'sd0;
                phase       <= 32'd0;
                drain_cnt   <= 2'd0;
                q_head      <= 1'b0;
                q_count     <= 2'd0;
                sample      <= 24'sd0;
                generating  <= 1'b1;
            end else begin
                if (done_pulse)
                    source_done <= 1'b1;

                // Request, wait one cycle for rd_data to become valid, then
                // push it into the audio-side queue on the following edge.
                if (rd_pending) begin
                    rd_pending <= 1'b0;
                    rd_capture <= 1'b1;
                end else if (rd_capture) begin
                    rd_capture <= 1'b0;
                end else if (running && (q_count < 2'd2) && !fifo_empty) begin
                    fifo_rd_en <= 1'b1;
                    rd_pending <= 1'b1;
                end

                // Stable-empty timer for the drain path.
                if (source_done && fifo_empty && !rd_pending &&
                    !rd_capture && (q_count == 2'd0)) begin
                    if (drain_cnt < DRAIN_HOLD)
                        drain_cnt <= drain_cnt + 2'd1;
                end else begin
                    drain_cnt <= 2'd0;
                end

                if (frame_fire) begin
                    if (!cur_valid) begin
                        if (q_count != 2'd0) begin
                            cur_sample <= q_front;
                            cur_valid  <= 1'b1;
                            phase      <= phase_inc;
                            sample     <= gain_pcm({q_front[15], q_front});
                        end else begin
                            sample <= 24'sd0;
                            if (can_finish) begin
                                running    <= 1'b0;
                                generating <= 1'b0;
                            end
                        end
                    end else if (q_count != 2'd0) begin
                        if (advance) begin
                            cur_sample <= q_front;
                            phase      <= phase - 32'd65536 + phase_inc;
                        end else begin
                            phase <= phase + phase_inc;
                        end
                        sample <= gain_pcm(mixed);
                    end else begin
                        sample <= gain_pcm({{1{cur_sample[15]}},
                                            cur_sample});
                        if (can_finish && last_frame_done) begin
                            running    <= 1'b0;
                            cur_valid  <= 1'b0;
                            generating <= 1'b0;
                        end else if (last_frame_done) begin
                            phase <= phase + phase_inc - 32'd65536;
                        end else begin
                            phase <= phase + phase_inc;
                        end
                    end
                end

                // Queue updates use the pre-edge push/pop values.
                if (push && pop) begin
                    q_mem[q_tail] <= pcm_rd;
                    q_head        <= q_head + 1'b1;
                end else if (push) begin
                    q_mem[q_tail] <= pcm_rd;
                    q_count       <= q_count + 2'd1;
                end else if (pop) begin
                    q_head  <= q_head + 1'b1;
                    q_count <= q_count - 2'd1;
                end
            end
        end
    end

endmodule
