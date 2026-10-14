`timescale 1ns / 1ps
//=============================================================================
// tb_frame_store.v - loader / SDRAM / FIFO / pixel stream integration test
//
//   * FRAME_LINES is reduced to 4 so one picture walk is quick, the logic
//     under test is exactly the same as in the 480 line build
//   * the video timing model is 640 active + 160 blank pixels per line and
//     4 active + 20 blank lines per frame
//   * the first video frame is ignored: the streamer arms on a frame edge and
//     the FIFO is still filling, so the picture is valid from frame two on
//   * every pixel of the second frame is compared against pattern_pixel
//=============================================================================
module tb_frame_store;

    localparam integer LINES       = 4;
    localparam integer FRAME_WORDS = 640 * LINES;
    localparam integer ROWS        = LINES + 20;   // active + vertical blank
    localparam integer H_TOTAL     = 800;

    reg clk     = 1'b0;
    reg pix_clk = 1'b0;
    reg rst_n   = 1'b0;

    always #10 clk = ~clk;                 // 50 MHz, rising at 10, 30, 50 ...

    initial begin
        #20;
        forever #20 pix_clk = ~pix_clk;    // 25 MHz, rising at 20, 60, 100 ...
    end

    //------------------------------------------------------------------
    // frame buffer under test
    //------------------------------------------------------------------
    wire        sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [10:0] sdram_addr;
    wire [1:0]  sdram_ba;
    wire        sdram_cs_n, sdram_cke;
    wire [3:0]  sdram_dm;
    wire [31:0] sdram_dq;

    wire [23:0] pix_rgb;
    wire        pix_uf;
    reg         pix_de;
    reg         pix_frame_tog;
    wire        frame_ready, frame_loaded, init_done, cal_fail;
    wire [2:0]  ld_state;

    frame_store #(
        .FRAME_LINES (LINES),
        .FRAME_WORDS (FRAME_WORDS),
        .BUF0_BASE   (21'h000000),
        .BUF1_BASE   (21'h001000),
        .FIFO_AW     (8)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .reload        (1'b0),
        .pattern_mode  (2'd0),
        .img_active    (1'b0),
        .img_abort     (1'b0),
        .swap_req      (1'b0),
        .img_valid     (1'b0),
        .img_rgb       (24'd0),
        .img_ready     (),
        .frame_ready   (frame_ready),
        .frame_loaded  (frame_loaded),
        .preload_ready (),
        .swap_done     (),
        .init_done     (init_done),
        .cal_fail      (cal_fail),
        .state_dbg     (ld_state),
        .sdram_ras_n   (sdram_ras_n),
        .sdram_cas_n   (sdram_cas_n),
        .sdram_we_n    (sdram_we_n),
        .sdram_addr    (sdram_addr),
        .sdram_ba      (sdram_ba),
        .sdram_cs_n    (sdram_cs_n),
        .sdram_cke     (sdram_cke),
        .sdram_dm      (sdram_dm),
        .sdram_dq      (sdram_dq),
        .pix_clk       (pix_clk),
        .pix_de        (pix_de),
        .pix_frame_tog (pix_frame_tog),
        .pix_rgb       (pix_rgb),
        .pix_underflow (pix_uf)
    );

    sdram_model #(
        .CL    (3),
        .T_RCD (3),
        .T_RP  (3),
        .T_RAS (5),
        .T_RFC (8)
    ) g_model (
        .clk   (clk),
        .cke   (sdram_cke),
        .cs_n  (sdram_cs_n),
        .ras_n (sdram_ras_n),
        .cas_n (sdram_cas_n),
        .we_n  (sdram_we_n),
        .addr  (sdram_addr),
        .ba    (sdram_ba),
        .dq    (sdram_dq),
        .dm    (sdram_dm)
    );

    //------------------------------------------------------------------
    // video timing model in the pixel clock domain
    //------------------------------------------------------------------
    integer   px, py;
    reg [2:0] frames_done;
    reg       check_en;
    integer   pix_index;       // active pixel counter of the current frame
    integer   errors;
    integer   checked;

    reg [9:0]  exp_x, exp_y;
    reg [23:0] exp_rgb;
    integer    wr_ovf, rd_unf;
    integer    xwr;

    always @(posedge clk) begin
        if (dut.fifo_wr_en && (^dut.fifo_wr_data === 1'bx)) begin
            xwr = xwr + 1;
            if (xwr <= 6)
                $display("X_DATA_WRITE at %0t rstate=%0d rcnt=%0d rd_addr=%08h rd_valid=%b",
                         $time, dut.rstate, dut.rcnt, dut.rd_addr, dut.rd_valid);
        end
    end

    always @(posedge clk) begin
        if (dut.u_fifo.wr_en && dut.u_fifo.wr_full) begin
            wr_ovf = wr_ovf + 1;
            if (wr_ovf <= 5)
                $display("WRITE_WHEN_FULL at %0t", $time);
        end
    end

    always @(posedge pix_clk) begin
        if (dut.u_fifo.rd_en && dut.u_fifo.rd_empty) begin
            rd_unf = rd_unf + 1;
            if (rd_unf <= 5)
                $display("READ_WHEN_EMPTY at %0t", $time);
        end
    end

    function [23:0] expected_pixel;
        input [9:0] fx;
        input [9:0] fy;
        reg   [2:0] bi;
        begin
            if      (fx < 10'd80)  bi = 3'd0;
            else if (fx < 10'd160) bi = 3'd1;
            else if (fx < 10'd240) bi = 3'd2;
            else if (fx < 10'd320) bi = 3'd3;
            else if (fx < 10'd400) bi = 3'd4;
            else if (fx < 10'd480) bi = 3'd5;
            else if (fx < 10'd560) bi = 3'd6;
            else                   bi = 3'd7;

            case (bi)
                3'd0:    expected_pixel = 24'hFFFFFF;
                3'd1:    expected_pixel = 24'hFFFF00;
                3'd2:    expected_pixel = 24'h00FFFF;
                3'd3:    expected_pixel = 24'h00FF00;
                3'd4:    expected_pixel = 24'hFF00FF;
                3'd5:    expected_pixel = 24'hFF0000;
                3'd6:    expected_pixel = 24'h0000FF;
                default: expected_pixel = 24'h000000;
            endcase

            if (fy < 10'd60)
                expected_pixel = 24'h202020;
            if (fy > 10'd419)
                expected_pixel = 24'h808080;
            if ((fx < 10'd4) || (fx > 10'd635) ||
                (fy < 10'd4) || (fy > 10'd475))
                expected_pixel = 24'hFFFFFF;
        end
    endfunction

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            px            <= 0;
            py            <= 0;
            pix_de        <= 1'b0;
            pix_frame_tog <= 1'b0;
            frames_done   <= 3'd0;
            check_en      <= 1'b0;
            pix_index     <= 0;
        end else begin
            pix_de <= (px < 640) && (py < LINES);

            if ((px < 640) && (py < LINES))
                pix_index <= pix_index + 1;

            if (px == H_TOTAL - 1) begin
                px <= 0;
                if (py == ROWS - 1) begin
                    py            <= 0;
                    pix_frame_tog <= ~pix_frame_tog;
                    pix_index     <= 0;
                    if (frames_done < 3'd3)
                        frames_done <= frames_done + 3'd1;
                    if (frames_done == 3'd1)
                        check_en <= 1'b1;   // frame two is the checked one
                end else begin
                    py <= py + 1;
                end
            end else begin
                px <= px + 1;
            end
        end
    end

    // the FIFO output is one pixel clock ahead of the data enable path, so
    // the n-th active cycle carries image pixel n-1
    always @(negedge pix_clk) begin
        if (rst_n && check_en && pix_de && (pix_index >= 2)) begin
            exp_x = (pix_index - 2) % 640;
            exp_y = (pix_index - 2) / 640;
            exp_rgb = expected_pixel(exp_x, exp_y);
            checked = checked + 1;
            if (pix_rgb !== exp_rgb) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("MISMATCH index=%0d x=%0d y=%0d got=%06h want=%06h rp=%0d wp=%0d rl=%0d wl=%0d empty=%b",
                             pix_index - 2, exp_x, exp_y, pix_rgb, exp_rgb,
                             dut.u_fifo.rd_ptr, dut.u_fifo.wr_ptr,
                             dut.u_fifo.rd_level, dut.u_fifo.wr_level,
                             dut.u_fifo.rd_empty);
            end
        end
    end

    //------------------------------------------------------------------
    // run
    //------------------------------------------------------------------
    initial begin
        checked = 0;
        errors  = 0;
        wr_ovf  = 0;
        rd_unf  = 0;
        xwr     = 0;
        #100;
        rst_n = 1'b1;
    end

    initial begin
        wait (rst_n === 1'b1);
        wait (frame_ready === 1'b1);
        wait (check_en === 1'b1);
        repeat (ROWS * H_TOTAL + 8) @(posedge pix_clk);

        $display("--------------------------------------------------");
        $display("tb_frame_store: checked=%0d errors=%0d", checked, errors);
        $display("tb_frame_store: write_when_full=%0d read_when_empty=%0d",
                 wr_ovf, rd_unf);
        $display("tb_frame_store: x_data_writes=%0d", xwr);
        $display("tb_frame_store: init_done=%b cal_fail=%b frame_ready=%b",
                 init_done, cal_fail, frame_ready);
        $display("tb_frame_store: sdram reads=%0d writes=%0d refresh=%0d protocol_errors=%0d",
                 g_model.n_rd, g_model.n_wr, g_model.n_ref, g_model.n_err);

        if ((errors == 0) && (g_model.n_err == 0) && !cal_fail && init_done)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");

        $finish;
    end

    initial begin
        #20_000_000;
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
