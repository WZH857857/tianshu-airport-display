//===========================================================================
// tb_mhist_probe.v -- minimal probe: stream one packed mask frame into
// motion_hist and count hit_ram, live_ram, snap_ram 200 clocks later.
//
// No tracker pulses, no sweep: this isolates the motion accumulator and the
// cell-row readout from everything else.
//
//   +IN=<packed masks>  +FRAMES=<n>
//===========================================================================
`timescale 1ns / 1ps

module tb_mhist_probe;

localparam integer W    = 320;
localparam integer H    = 180;
localparam integer ROWB = W / 8;
localparam integer FB   = ROWB * H;
localparam integer NC   = 80 * 45;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg de_in = 1'b0;
reg mask_in = 1'b0;

wire             pv_w;
wire [3:0]       pf_w;
wire [35:0]      px_w, py_w, pw_w, ph_w;
wire [51:0]      pc_w;
wire [3:0]       pd_w;

motion_hist #(.NSLOT(4)) u_dut (
    .clk(clk), .rst_n(rst_n), .de_in(de_in), .mask_in(mask_in),
    .trk_valid(1'b0), .trk_found(4'd0), .trk_x(36'd0), .trk_y(36'd0),
    .trk_w(36'd0), .trk_h(36'd0),
    .out_valid(pv_w), .out_found(pf_w), .out_x(px_w), .out_y(py_w),
    .out_w(pw_w), .out_h(ph_w), .out_cells(pc_w), .out_dir(pd_w));

reg [1023:0] in_path;
integer fd_in, nframes, f, n, ci, got, i, hit_n, live_n, snap_n;
reg [7:0] fbuf [0:FB-1];

always #5 clk = ~clk;

task count_maps;
    begin
        hit_n = 0; live_n = 0; snap_n = 0;
        for (i = 0; i < NC; i = i + 1) begin
            hit_n  = hit_n  + u_dut.u_hit_map.mem[i][0];
            live_n = live_n + (u_dut.u_live_map.mem[i][1:0] != 0);
            snap_n = snap_n + (u_dut.u_snap_map.mem[i][1:0] != 0);
        end
    end
endtask

// Watch the running total of hit bits so we can see which cell row's readout
// wipes bits that were just written.
reg prev_busy = 1'b0;
integer j, ones, bad;
reg [11:0] chk_addr;
reg        chk_en;

always @(posedge clk) begin
    if (u_dut.dec_busy && u_dut.dec_hit) begin
        chk_addr <= u_dut.dec_addr;
        chk_en   <= 1'b1;
    end else begin
        chk_en <= 1'b0;
    end
end

always @(negedge clk)
    if (chk_en) $display("  WCHK addr=%0d now=%b", chk_addr,
                         u_dut.u_hit_map.mem[chk_addr][0]);

// Trace a watched cell: who turns it back to 0, and when?
always @(posedge clk)
    if (u_dut.dec_busy && (u_dut.dec_addr < 12'd100))
        $display("  LOWW addr=%0d val=%b dec_r=%0d dec_c=%0d sx=%0d sy=%0d",
                 u_dut.dec_addr, u_dut.dec_hit, u_dut.dec_r, u_dut.dec_c,
                 u_dut.sx, u_dut.sy);

reg w43 = 1'b0, w44 = 1'b0;
integer tr_n = 0;
always @(negedge clk) begin
    if (u_dut.u_hit_map.mem[43][0] !== w43) begin
        if (tr_n < 20)
            $display("  TRACE[43] %b -> %b  st=%0d dec_busy=%0d sx=%0d sy=%0d",
                     w43, u_dut.u_hit_map.mem[43][0], u_dut.st, u_dut.dec_busy,
                     u_dut.sx, u_dut.sy);
        w43 = u_dut.u_hit_map.mem[43][0];
        tr_n = tr_n + 1;
    end
    if (u_dut.u_hit_map.mem[44][0] !== w44) begin
        if (tr_n < 20)
            $display("  TRACE[44] %b -> %b  st=%0d dec_busy=%0d sx=%0d sy=%0d",
                     w44, u_dut.u_hit_map.mem[44][0], u_dut.st, u_dut.dec_busy,
                     u_dut.sx, u_dut.sy);
        w44 = u_dut.u_hit_map.mem[44][0];
        tr_n = tr_n + 1;
    end
end
always @(posedge clk) begin
    prev_busy <= u_dut.dec_busy;
    if (u_dut.dec_busy && !prev_busy) $display("  ROW r=%0d start", u_dut.dec_r);
end
always @(negedge clk)
    if (!u_dut.dec_busy && prev_busy) begin
        ones = 0;
        bad = 0;
        for (j = 0; j < NC; j = j + 1) begin
            if (u_dut.u_hit_map.mem[j][0] === 1'b1) ones = ones + 1;
            else if (u_dut.u_hit_map.mem[j][0] !== 1'b0) begin
                if (bad < 4) $display("    hit_ram[%0d] = %b", j,
                                      u_dut.u_hit_map.mem[j][0]);
                bad = bad + 1;
            end
        end
        $display("  ROW r=%0d end, hit_ram ones=%0d bad=%0d", u_dut.dec_r,
                 ones, bad);
    end

initial begin
    if (!$value$plusargs("IN=%s", in_path) ||
        !$value$plusargs("FRAMES=%d", nframes)) begin
        $display("ERROR need IN and FRAMES");
        $finish;
    end
    fd_in = $fopen(in_path, "rb");
    if (fd_in == 0) begin
        $display("ERROR cannot open %0s", in_path);
        $finish;
    end

    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    for (f = 0; f < nframes; f = f + 1) begin
        got = $fread(fbuf, fd_in);
        if (got != FB) begin
            $display("ERROR short read frame %0d: %0d", f, got);
            $finish;
        end
        for (n = 0; n < FB; n = n + 1) begin
            for (ci = 7; ci >= 0; ci = ci - 1) begin
                @(negedge clk);
                mask_in = fbuf[n][ci];
                de_in   = 1'b1;
                @(posedge clk);
            end
        end
        @(negedge clk);
        de_in = 1'b0;
        for (n = 0; n < 200; n = n + 1) begin
            @(negedge clk);
            @(posedge clk);
        end
        count_maps;
        $display("PROBE frame %0d hit_ram=%0d live_ram=%0d snap_ram=%0d st=%0d",
                 f, hit_n, live_n, snap_n, u_dut.st);
    end
    $fclose(fd_in);
    $display("DONE");
    $finish;
end

endmodule
