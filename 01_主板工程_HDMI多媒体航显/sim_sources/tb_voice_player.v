`timescale 1ns / 1ps
//=============================================================================
// tb_voice_player.v - replay of the mu-law voice bank on the audio clock
//
// A four entry bank is played at a 48 kHz source rate, so the phase
// accumulator steps exactly one entry per frame and the interpolator has to
// return the stored value itself.  The bench checks
//   * the bank is silent until the loader says it is ready
//   * the four entries come out expanded by GAIN_SHIFT
//   * the announcement is followed by silence and then starts over
//
// Two players run side by side on the same bank.  The first one carries the
// small gain the interpolation checks use, the second one the GAIN_SHIFT = 7
// the board ships with: at that gain a full scale mu-law code decodes to
// 4111872, which does not fit in the 18 bit register the player used to
// shift into, so this is the case that catches a narrow gain path.
//=============================================================================
module tb_voice_player;

    localparam integer GAP = 6;

    reg                clk;
    reg                rst;
    reg                ready;
    reg                frame_tick;
    reg  [31:0]        phase_inc;
    reg  [16:0]        words;
    wire [15:0]        rd_addr;
    wire [7:0]         rd_data;
    wire signed [23:0] sample;
    wire               generating;

    reg  [7:0]  bank [0:3];
    reg  [7:0]  bank_q;
    reg  [7:0]  bank_q7;
    reg  [3:0]  tick_div;
    integer     errors;
    integer     i;
    integer     silent_frames;
    reg signed [23:0] got;
    reg signed [23:0] got7;
    reg signed [23:0] exp [0:3];
    reg signed [23:0] exp7 [0:3];
    wire [15:0]        rd_addr7;
    wire [7:0]         rd_data7;
    wire signed [23:0] sample7;

    voice_player #(
        .WORDS      (65536),
        .AW         (16),
        .GAP_FRAMES (GAP),
        .GAIN_SHIFT (2)
    ) dut (
        .clk        (clk),
        .rst        (rst),
        .ready      (ready),
        .frame_tick (frame_tick),
        .phase_inc  (phase_inc),
        .words      (words),
        .rd_addr    (rd_addr),
        .rd_data    (rd_data),
        .sample     (sample),
        .generating (generating)
    );

    voice_player #(
        .WORDS      (65536),
        .AW         (16),
        .GAP_FRAMES (GAP),
        .GAIN_SHIFT (7)
    ) dut7 (
        .clk        (clk),
        .rst        (rst),
        .ready      (ready),
        .frame_tick (frame_tick),
        .phase_inc  (phase_inc),
        .words      (words),
        .rd_addr    (rd_addr7),
        .rd_data    (rd_data7),
        .sample     (sample7),
        .generating ()
    );

    always #10 clk = ~clk;

    // one 48 kHz frame is 16 clocks in this bench
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tick_div   <= 4'd0;
            frame_tick <= 1'b0;
        end else begin
            tick_div   <= tick_div + 4'd1;
            frame_tick <= (tick_div == 4'd15);
        end
    end

    // the bank read port is registered, like the BRAM it stands in for
    always @(posedge clk)
        bank_q <= bank[rd_addr[1:0]];

    always @(posedge clk)
        bank_q7 <= bank[rd_addr7[1:0]];

    assign rd_data = bank_q;
    assign rd_data7 = bank_q7;

    // let the converter run and hand back the sample that belongs to the
    // frame which starts on the next tick
    task next_frame;
        begin
            @(posedge clk);
            while (!frame_tick)
                @(posedge clk);
            repeat (8) @(posedge clk);
            got = sample;
            got7 = sample7;
        end
    endtask

    initial begin
        clk       = 1'b0;
        rst       = 1'b1;
        ready     = 1'b0;
        frame_tick = 1'b0;
        phase_inc = 32'd65536;          // 48 kHz source rate
        words     = 17'd4;
        errors    = 0;

        // G.711 codes for 988, 7932, 32124 and -32124
        bank[0] = 8'hCE;
        bank[1] = 8'hA0;
        bank[2] = 8'h80;
        bank[3] = 8'h00;

        exp[0] = 24'sd3952;
        exp[1] = 24'sd31728;
        exp[2] = 24'sd128496;
        exp[3] = -24'sd128496;

        // the same four codes at the gain the board uses
        exp7[0] = 24'sd126464;
        exp7[1] = 24'sd1015296;
        exp7[2] = 24'sd4111872;
        exp7[3] = -24'sd4111872;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        if (generating != 1'b0) begin
            $display("FAIL: generating high before the bank is ready");
            errors = errors + 1;
        end

        //------------------------------------------------------------------
        // no voice loaded: the output must stay quiet
        //------------------------------------------------------------------
        for (i = 0; i < 4; i = i + 1) begin
            next_frame;
            if (got !== 24'sd0) begin
                $display("FAIL: not ready frame %0d sample=%0d", i, got);
                errors = errors + 1;
            end
            if (got7 !== 24'sd0) begin
                $display("FAIL: not ready gain7 frame %0d sample=%0d",
                         i, got7);
                errors = errors + 1;
            end
        end

        //------------------------------------------------------------------
        // ready: the four entries are played back to back
        //------------------------------------------------------------------
        ready = 1'b1;
        if (generating != 1'b1) begin
            $display("FAIL: generating low with a loaded bank");
            errors = errors + 1;
        end

        for (i = 0; i < 4; i = i + 1) begin
            next_frame;
            if (got !== exp[i]) begin
                $display("FAIL: frame %0d sample=%0d expected %0d",
                         i, got, exp[i]);
                errors = errors + 1;
            end
            if (got7 !== exp7[i]) begin
                $display("FAIL: gain7 frame %0d sample=%0d expected %0d",
                         i, got7, exp7[i]);
                errors = errors + 1;
            end
        end

        //------------------------------------------------------------------
        // the gap, then the announcement repeats from the first entry
        //------------------------------------------------------------------
        silent_frames = 0;
        next_frame;
        while ((got == 24'sd0) && (silent_frames < 16)) begin
            silent_frames = silent_frames + 1;
            next_frame;
        end

        if (silent_frames < GAP) begin
            $display("FAIL: only %0d silent frames, expected at least %0d",
                     silent_frames, GAP);
            errors = errors + 1;
        end

        if (got !== exp[0]) begin
            $display("FAIL: restart sample=%0d expected %0d", got, exp[0]);
            errors = errors + 1;
        end
        for (i = 1; i < 4; i = i + 1) begin
            next_frame;
            if (got !== exp[i]) begin
                $display("FAIL: restart frame %0d sample=%0d expected %0d",
                         i, got, exp[i]);
                errors = errors + 1;
            end
        end

        //------------------------------------------------------------------
        // a fractional rate has to land between the two neighbours: half a
        // sample per frame puts the second frame a quarter of the way along
        // the step, which is exactly halfway between the entries
        //------------------------------------------------------------------
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        phase_inc = 32'd32768;
        ready     = 1'b1;
        repeat (2) @(posedge clk);

        next_frame;
        if (got !== exp[0]) begin
            $display("FAIL: half rate frame 0 sample=%0d expected %0d",
                     got, exp[0]);
            errors = errors + 1;
        end
        next_frame;
        if (got !== 24'sd17840) begin
            $display("FAIL: half rate frame 1 sample=%0d expected 17840",
                     got);
            errors = errors + 1;
        end
        next_frame;
        if (got !== exp[1]) begin
            $display("FAIL: half rate frame 2 sample=%0d expected %0d",
                     got, exp[1]);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS tb_voice_player");
        else
            $display("RESULT: FAIL tb_voice_player errors=%0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_voice_player timeout");
        $finish;
    end

endmodule
