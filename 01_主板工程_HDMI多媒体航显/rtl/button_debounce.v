`timescale 1ns / 1ps

module button_debounce #(
    parameter integer DEBOUNCE_CYCLES = 250000
) (
    input  wire clock,
    input  wire reset_n,
    input  wire button_n,
    output reg  pressed_tick,
    output reg  stable_n
);

    reg [1:0]  sync_pipe;
    reg [19:0] hold_count;

    always @(posedge clock or negedge reset_n) begin
        if (!reset_n) begin
            sync_pipe    <= 2'b11;
            stable_n     <= 1'b1;
            hold_count   <= 20'd0;
            pressed_tick <= 1'b0;
        end else begin
            sync_pipe <= {sync_pipe[0], button_n};
            pressed_tick <= 1'b0;

            if (sync_pipe[1] != stable_n) begin
                if (hold_count == DEBOUNCE_CYCLES - 1) begin
                    stable_n     <= sync_pipe[1];
                    pressed_tick <= ~sync_pipe[1];
                    hold_count   <= 20'd0;
                end else begin
                    hold_count <= hold_count + 1'b1;
                end
            end else begin
                hold_count <= 20'd0;
            end
        end
    end

endmodule
