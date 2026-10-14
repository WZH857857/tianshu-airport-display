`timescale 1ns / 1ps
//=============================================================================
// tb_flight_board.v - checks the real-time flight information overlay
//
// The board is intentionally independent of the UART parser: this bench writes
// bytes through the same one-toggle-per-byte handshake the controller uses,
// then walks the pixel grid and verifies the address decode, the row flash and
// the animated rails.
//=============================================================================
module tb_flight_board;

    reg        pixel_clk = 1'b0;
    always #20 pixel_clk = ~pixel_clk;          // 25 MHz

    reg        rst_n = 1'b0;
    reg        frame_tick = 1'b0;
    reg [9:0]  px = 10'd0;
    reg [9:0]  py = 10'd0;
    reg        enable = 1'b0;
    reg        wr_tog = 1'b0;
    reg [7:0]  wr_addr = 8'd0;
    reg [7:0]  wr_data = 8'd0;

    wire       in_board;
    wire       glyph_y_ok;
    wire [7:0] ch;
    wire [3:0] gx, gy;
    wire [2:0] row;
    wire       row_flash;
    wire       border_hit;
    wire [23:0] breath_rgb;

    integer errors = 0;

    flight_board dut (
        .pixel_clk  (pixel_clk),
        .rst_n      (rst_n),
        .frame_tick (frame_tick),
        .px         (px),
        .py         (py),
        .enable     (enable),
        .wr_tog     (wr_tog),
        .wr_addr    (wr_addr),
        .wr_data    (wr_data),
        .in_board   (in_board),
        .glyph_y_ok (glyph_y_ok),
        .ch         (ch),
        .gx         (gx),
        .gy         (gy),
        .row        (row),
        .row_flash  (row_flash),
        .border_hit (border_hit),
        .breath_rgb (breath_rgb)
    );

    task write_byte;
        input [7:0] addr;
        input [7:0] data;
        begin
            @(posedge pixel_clk);
            wr_addr = addr;
            wr_data = data;
            wr_tog  = ~wr_tog;
            repeat (6) @(posedge pixel_clk);
        end
    endtask

    task tick_frame;
        begin
            @(posedge pixel_clk);
            frame_tick = 1'b1;
            @(posedge pixel_clk);
            frame_tick = 1'b0;
        end
    endtask

    // The character buffer is block RAM, so its registered read and the
    // matching pixel-side controls are available one pixel clock later.
    task settle_pixel;
        begin
            @(posedge pixel_clk);
            #1;
        end
    endtask

    reg [23:0] old_breath;

    initial begin
        $dumpfile("tb_flight_board.vcd");
        $dumpvars(0, tb_flight_board);

        repeat (4) @(posedge pixel_clk);
        rst_n = 1'b1;
        repeat (4) @(posedge pixel_clk);
        enable = 1'b1;
        repeat (4) @(posedge pixel_clk);

        // Row 0, column 7.  Address = row * 40 + column = 7.
        write_byte(8'd7, 8'h43);                 // C

        // Pixel inside the glyph cell: x = 7*16 + 9, y = 96 + 8 + 5.
        px = 10'd121;
        py = 10'd109;
        settle_pixel();
        if (!in_board || !glyph_y_ok || (row !== 3'd0) ||
            (gx !== 4'd4) || (gy !== 4'd2) || (ch !== 8'h43)) begin
            $display("FAIL: row0 decode is in=%b yok=%b row=%0d gx=%0d gy=%0d ch=%02x",
                     in_board, glyph_y_ok, row, gx, gy, ch);
            errors = errors + 1;
        end
        if (!row_flash) begin
            $display("FAIL: a written row did not start its update flash");
            errors = errors + 1;
        end

        // The two-pixel rail at the top and bottom of the board is visible.
        px = 10'd300;
        py = 10'd96;
        settle_pixel();
        if (!in_board || !border_hit) begin
            $display("FAIL: top rail did not paint");
            errors = errors + 1;
        end
        py = 10'd287;
        settle_pixel();
        if (!in_board || !border_hit) begin
            $display("FAIL: bottom rail did not paint");
            errors = errors + 1;
        end

        // Row 3, column 39.  Address = 3 * 40 + 39 = 159.
        write_byte(8'd159, 8'h5A);               // Z
        px = 10'd639;
        py = 10'd205;
        settle_pixel();
        if (!in_board || !glyph_y_ok || (row !== 3'd3) ||
            (gx !== 4'd7) || (gy !== 4'd2) || (ch !== 8'h5A)) begin
            $display("FAIL: row3 decode is row=%0d gx=%0d gy=%0d ch=%02x",
                     row, gx, gy, ch);
            errors = errors + 1;
        end

        // Row 5, column 39.  Address = 5 * 40 + 39 = 239.
        write_byte(8'd239, 8'h51);               // Q
        px = 10'd639;
        py = 10'd269;
        settle_pixel();
        if (!in_board || !glyph_y_ok || (row !== 3'd5) ||
            (gx !== 4'd7) || (gy !== 4'd2) || (ch !== 8'h51)) begin
            $display("FAIL: row5 decode is row=%0d gx=%0d gy=%0d ch=%02x",
                     row, gx, gy, ch);
            errors = errors + 1;
        end

        // Flash lasts for thirty frame ticks and then releases.
        old_breath = breath_rgb;
        repeat (30) tick_frame();
        settle_pixel();
        if (row_flash) begin
            $display("FAIL: update flash did not time out");
            errors = errors + 1;
        end

        // The rail colour advances through its four-phase breathing cycle.
        repeat (64) tick_frame();
        if (breath_rgb === old_breath) begin
            $display("FAIL: breathing rail colour did not advance");
            errors = errors + 1;
        end

        // Disabling the panel must suppress the whole overlay band.
        enable = 1'b0;
        repeat (4) @(posedge pixel_clk);
        px = 10'd121;
        py = 10'd109;
        #1;
        if (in_board) begin
            $display("FAIL: disabled board still claims pixels");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS: tb_flight_board");
        else
            $display("FAIL: tb_flight_board with %0d errors", errors);
        $finish;
    end

endmodule
