//===========================================================================
// led595_shift.v -- 8-bit 74HC595 LED driver (P59 clk / P57 latch / P45 dout)
//
// Shift order is MSB first, so data[0] lands on QA (LED0) and data[7] on QH.
// The shift clock is slow (default 100 kHz) which is far more than enough for
// the HC595 and keeps the three board nets quiet.  A write is only started
// when the requested byte differs from what is already latched.
//===========================================================================
`timescale 1ns / 1ps

module led595_shift #(
    parameter integer CLK_HZ   = 24000000,
    parameter integer SHIFT_HZ = 100000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,

    output reg        clk_out,
    output reg        latch,
    output reg        dout
);

    localparam integer TICK_DIV = CLK_HZ / SHIFT_HZ;

    localparam [1:0] L_IDLE = 2'd0, L_SHIFT = 2'd1, L_LATCH = 2'd2;

    reg [1:0] st;
    reg [7:0] div;
    reg [3:0] bitn;
    reg [7:0] shown;

    wire tick = (div == TICK_DIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= L_IDLE;
            div     <= 8'd0;
            bitn    <= 4'd0;
            clk_out <= 1'b0;
            latch   <= 1'b0;
            dout    <= 1'b0;
            shown   <= 8'h00;
        end else begin
            case (st)
            L_IDLE: begin
                clk_out <= 1'b0;
                latch   <= 1'b0;
                div     <= 8'd0;
                bitn    <= 4'd0;
                if (data != shown) begin
                    dout <= data[7];
                    st   <= L_SHIFT;
                end
            end

            L_SHIFT: begin
                if (tick) begin
                    div <= 8'd0;
                    if (!clk_out) begin
                        clk_out <= 1'b1;              // rising edge shifts
                        if (bitn == 4'd7) begin
                            st <= L_LATCH;
                        end else begin
                            dout <= data[6 - bitn];
                            bitn <= bitn + 4'd1;
                        end
                    end else begin
                        clk_out <= 1'b0;
                    end
                end else begin
                    div <= div + 8'd1;
                end
            end

            L_LATCH: begin
                clk_out <= 1'b0;
                if (!latch) begin
                    latch <= 1'b1;                    // >= one full tick
                    div   <= 8'd0;
                end else if (tick) begin
                    latch <= 1'b0;
                    shown <= data;
                    div   <= 8'd0;
                    st    <= L_IDLE;
                end else begin
                    div <= div + 8'd1;
                end
            end

            default: st <= L_IDLE;
            endcase
        end
    end

endmodule
