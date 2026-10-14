`timescale 1ns / 1ps
//=============================================================================
// tb_ulaw.v - G.711 mu-law compressor / expander
//
// The reference values are the ones the standard encoder produces (0 -> 0xFF,
// full scale -> 0x80 / 0x00, 1000 -> 0xCE, ...) together with the value the
// standard decoder returns for that code.
//=============================================================================
module tb_ulaw;

    reg  signed [15:0] pcm_in;
    wire        [7:0]  code;
    reg         [7:0]  code_in;
    wire signed [15:0] pcm_out;

    integer errors;
    integer i;
    reg signed [15:0] expected_dec;
    integer diff;
    integer tol;

    ulaw_encode u_enc (
        .pcm  (pcm_in),
        .code (code)
    );

    ulaw_decode u_dec (
        .code (code_in),
        .pcm  (pcm_out)
    );

    task check_pair;
        input signed [15:0] pcm;
        input        [7:0]  expect_code;
        input signed [15:0] expect_pcm;
        begin
            pcm_in = pcm;
            #1;
            if (code !== expect_code) begin
                $display("FAIL: enc(%0d)=%02x expected %02x",
                         pcm, code, expect_code);
                errors = errors + 1;
            end
            code_in = expect_code;
            #1;
            if (pcm_out !== expect_pcm) begin
                $display("FAIL: dec(%02x)=%0d expected %0d",
                         expect_code, pcm_out, expect_pcm);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        errors = 0;
        pcm_in = 16'sd0;
        code_in = 8'h00;

        check_pair(16'sd0,     8'hFF, 16'sd0);
        check_pair(16'sd1,     8'hFF, 16'sd0);
        check_pair(16'sd10,    8'hFE, 16'sd8);
        check_pair(16'sd100,   8'hF2, 16'sd104);
        check_pair(16'sd255,   8'hE7, 16'sd260);
        check_pair(16'sd1000,  8'hCE, 16'sd988);
        check_pair(16'sd8000,  8'hA0, 16'sd7932);
        check_pair(16'sd12345, 8'h97, 16'sd12412);
        check_pair(-16'sd1,    8'h7F, 16'sd0);
        check_pair(-16'sd100,  8'h72, -16'sd104);
        check_pair(-16'sd1000, 8'h4E, -16'sd988);
        check_pair(-16'sd8000, 8'h20, -16'sd7932);
        check_pair(-16'sd23456, 8'h08, -16'sd23932);
        check_pair(16'sd32767, 8'h80, 16'sd32124);
        check_pair(-16'sd32768, 8'h00, -16'sd32124);

        // expanding a code and compressing it again must be a fixed point, so
        // a sample that was stored once is not degraded any further by the
        // encoder sitting in front of the bank
        for (i = 0; i < 256; i = i + 1) begin
            code_in = i[7:0];
            #1;
            expected_dec = pcm_out;
            pcm_in = expected_dec;
            #1;
            code_in = code;
            #1;
            if (pcm_out !== expected_dec) begin
                $display("FAIL: code %02x -> %0d -> %02x -> %0d",
                         i[7:0], expected_dec, code, pcm_out);
                errors = errors + 1;
            end
        end

        // and the lossy part must stay inside the mu-law error bound
        for (i = -32768; i < 32768; i = i + 37) begin
            pcm_in = i[15:0];
            #1;
            code_in = code;
            #1;
            expected_dec = i[15:0];
            diff = (pcm_out > expected_dec) ? (pcm_out - expected_dec)
                                            : (expected_dec - pcm_out);
            tol = (expected_dec < 0 ? -expected_dec : expected_dec) >> 3;
            if (tol < 4)
                tol = 4;
            if (diff > tol) begin
                $display("FAIL: %0d -> %02x -> %0d error %0d > %0d",
                         expected_dec, code, pcm_out, diff, tol);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("RESULT: PASS tb_ulaw");
        else
            $display("RESULT: FAIL tb_ulaw errors=%0d", errors);
        $finish;
    end

endmodule
