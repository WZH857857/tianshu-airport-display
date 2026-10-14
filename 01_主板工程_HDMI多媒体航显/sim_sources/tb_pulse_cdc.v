`timescale 1ns / 1ps
//=============================================================================
// tb_pulse_cdc.v - pulse crossing from the 50 MHz system clock into the
// 25 MHz pixel clock
//
// The old fade path fed frame_loaded straight into the pixel domain.  A
// one-cycle 20 ns pulse cannot be sampled reliably by a 40 ns clock, so this
// test launches 200 pulses at pseudo-random phases and requires that every
// single one arrives.  A direct sampler is included as a contrast: it is the
// behaviour the design had before pulse_cdc was inserted.
//=============================================================================
module tb_pulse_cdc;

    reg clk_src = 1'b0;     // 50 MHz -> 20 ns period
    reg clk_dst = 1'b0;     // 25 MHz -> 40 ns period
    reg rst_src = 1'b1;
    reg rst_dst = 1'b1;
    reg pulse_in = 1'b0;

    wire pulse_out;

    integer sent      = 0;
    integer received  = 0;
    integer direct    = 0;
    integer i;
    integer errors    = 0;

    // $random can return a negative value and a negative delay is not a
    // shift in phase, so the launch offset comes from a small LFSR instead.
    reg [15:0] lfsr = 16'hACE1;

    always @(posedge clk_src)
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

    pulse_cdc dut (
        .clk_src   (clk_src),
        .rst_src   (rst_src),
        .pulse_in  (pulse_in),
        .clk_dst   (clk_dst),
        .rst_dst   (rst_dst),
        .pulse_out (pulse_out)
    );

    always #10 clk_src = ~clk_src;
    always #20 clk_dst = ~clk_dst;

    always @(posedge clk_dst) begin
        if (pulse_out)
            received = received + 1;
        // What the previous design did: sample the short pulse directly.
        if (pulse_in)
            direct = direct + 1;
    end

    task send_pulse;
        begin
            @(negedge clk_src);
            pulse_in = 1'b1;
            @(negedge clk_src);
            pulse_in = 1'b0;
        end
    endtask

    initial begin
        repeat (4) @(negedge clk_src);
        rst_src = 1'b0;
        rst_dst = 1'b0;
        repeat (4) @(negedge clk_src);

        for (i = 0; i < 200; i = i + 1) begin
            #((lfsr % 40) + 1);      // land anywhere in the pixel period
            sent = sent + 1;
            send_pulse;
            repeat (4) @(negedge clk_dst);   // let the toggle settle
        end

        repeat (4) @(negedge clk_dst);

        $display("ADV02 TB: pulses sent = %0d, delivered by pulse_cdc = %0d",
                 sent, received);
        $display("ADV02 TB: direct sampling of the same pulses saw %0d",
                 direct);

        if (received != sent) begin
            $display("FAIL: pulse_cdc lost %0d pulse(s)", sent - received);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_pulse_cdc");
        else
            $display("RESULT: FAIL tb_pulse_cdc");

        $finish;
    end

endmodule
