`timescale 1ns / 1ps
//=============================================================================
// edid_retry.v - EDID read trigger with retry
//
// The vendor HDMI 1.4b transmitter core exposes "start an EDID read" as a one
// cycle pulse and reports the bytes it got back on O_edid_read_valid. The
// vendor top level fires that pulse exactly once a few milliseconds after
// configuration, so a display that is powered on later is never probed.
//
// This block keeps pulsing I_edid_read_trig once per RETRY_CYCLES until the
// first byte comes back, then stops.
//=============================================================================
module edid_retry #(
    parameter [24:0] RETRY_CYCLES = 25'd24_999_999   // 1 s at 25 MHz
)(
    input  wire clk,
    input  wire rst,        // active high
    input  wire edid_valid, // a byte was read back from the sink
    output reg  trig
);

    reg [24:0] cnt;
    reg        got_valid;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt       <= 25'd0;
            got_valid <= 1'b0;
            trig      <= 1'b0;
        end else begin
            trig <= 1'b0;

            if (edid_valid)
                got_valid <= 1'b1;

            if (!got_valid) begin
                if (cnt == RETRY_CYCLES - 1'b1) begin
                    cnt  <= 25'd0;
                    trig <= 1'b1;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else begin
                cnt <= 25'd0;
            end
        end
    end

endmodule
