`timescale 1ns / 1ps

module tb_emergency_alert_seg119;
    reg clk;
    reg rst_n;
    reg fire;
    wire buzzer_n;
    wire [7:0] seg_n;
    wire [7:0] sel;

    emergency_alert dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .fire     (fire),
        .buzzer_n (buzzer_n),
        .seg_n    (seg_n),
        .sel      (sel)
    );

    integer failures;

    task check_digit;
        input [2:0] digit;
        input [7:0] expected_seg;
        begin
            dut.scan_cnt = {3'b000, digit, 10'b0};
            dut.blink_cnt[24] = 1'b1;
            #1;
            if ((seg_n !== expected_seg) || (sel !== ~(8'h01 << digit))) begin
                $display("FAIL digit=%0d seg=%02h expected=%02h sel=%02h expected_sel=%02h",
                         digit, seg_n, expected_seg, sel, ~(8'h01 << digit));
                failures = failures + 1;
            end else begin
                $display("PASS digit=%0d seg=%02h sel=%02h", digit, seg_n, sel);
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        forever #10 clk = ~clk;
    end

    initial begin
        failures = 0;
        rst_n = 1'b0;
        fire = 1'b0;
        #21;
        rst_n = 1'b1;
        #1;

        if ((seg_n !== 8'hFF) || (sel !== 8'hFF)) begin
            $display("FAIL idle seg=%02h sel=%02h", seg_n, sel);
            failures = failures + 1;
        end else begin
            $display("PASS idle seg=ff sel=ff");
        end

        fire = 1'b1;
        check_digit(3'd3, 8'hF9);
        check_digit(3'd4, 8'hF9);
        check_digit(3'd5, 8'h90);

        if (failures == 0) begin
            $display("TB_EMERGENCY_ALERT_SEG119: PASS");
        end else begin
            $display("TB_EMERGENCY_ALERT_SEG119: FAIL (%0d errors)", failures);
            $fatal(1);
        end
        $finish;
    end
endmodule
