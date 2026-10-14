//===========================================================================
// tb_motion_hist.v -- motion_hist alone, one model frame per step
//
// The crowd-path bench (tb_multi_top) exercises motion_hist behind the real
// tracker, which is exactly what the board does but makes a mismatch hard to
// attribute: the tracker's publish lands somewhere inside the next frame.
// This bench drives the module directly instead, with one publish per frame
// placed after the end-of-frame pass, and dumps the frozen map as well as
// the boxes, so motion_hist_check.py can tell a window bug from a geometry
// bug.
//
// Input : +IN=<packed masks>  1 bit per vision pixel, MSB first inside each
//                              byte, 40 bytes per row, 3,600 bits per frame
//                              (same packing as morph_check.pack_masks)
//         +TRK=<text>         four lines per frame: found x y w h
//         +FRAMES=<n>
// Output: PBOX <publish> <slot> <found> <x> <y> <w> <h> <cells> on stdout
//         +OUT=<file>         the same lines to a file
//         +DUMP=<file>        optional: 3,600 bytes per publish, the frozen
//                            2-bit map, one byte per cell
// plusargs:
//         +WAIT=<n>           clocks between the last pixel and the publish
//                             pulse (default 300; the end-of-frame pass is
//                             3,600 clocks, so the pulse must not land
//                             before the pass has taken its window snapshot)
//===========================================================================
`timescale 1ns / 1ps

module tb_motion_hist;

localparam integer NB   = 4;
localparam integer W    = 320;
localparam integer H    = 180;
localparam integer ROWB = W / 8;            // 40
localparam integer FB   = ROWB * H;         // 7,200 bytes per frame
localparam integer NC   = 80 * 45;          // 3,600 cells

reg clk = 1'b0;
reg rst_n = 1'b0;
reg de_in = 1'b0;
reg mask_in = 1'b0;
reg trk_valid = 1'b0;
reg [NB-1:0]    trk_found;
reg [NB*9-1:0]  trk_x, trk_y, trk_w, trk_h;

wire             pbox_valid;
wire [NB-1:0]    pbox_found;
wire [NB*9-1:0]  pbox_x, pbox_y, pbox_w, pbox_h;
wire [NB*13-1:0] pbox_cells;
wire [NB-1:0]    pbox_dir;

motion_hist #(
    .NSLOT (NB)
) u_dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .de_in     (de_in),
    .mask_in   (mask_in),
    .trk_valid (trk_valid),
    .trk_found (trk_found),
    .trk_x     (trk_x),
    .trk_y     (trk_y),
    .trk_w     (trk_w),
    .trk_h     (trk_h),
    .out_valid (pbox_valid),
    .out_found (pbox_found),
    .out_x     (pbox_x),
    .out_y     (pbox_y),
    .out_w     (pbox_w),
    .out_h     (pbox_h),
    .out_cells (pbox_cells),
    .out_dir   (pbox_dir)
);

always #5 clk = ~clk;

reg [1023:0] in_path, trk_path, out_path, dump_path;
integer fd_in, fd_trk, fd_out, fd_dump;
integer nframes, wait_n, f, n, k, got, wcnt, ci;
reg [7:0] fbuf [0:FB-1];
reg [7:0] snap_byte;
integer t0, t1, t2, t3, t4;
reg [NB-1:0]   fr_found;
reg [NB*9-1:0] fr_x, fr_y, fr_w, fr_h;
reg dump_on;
reg dbg_on;
reg [31:0] pub_cnt;
integer dbi, hit_n, live_n, snap_n, hit_pass;
integer wr_hit_cnt;

// Count the cells the cell-row readout actually wrote as hits for this frame.
always @(posedge clk)
    if (u_dut.dec_busy && u_dut.dec_hit) wr_hit_cnt = wr_hit_cnt + 1;

always @(posedge clk)
    if (dbg_on && u_dut.dec_busy && u_dut.dec_hit)
        $display("    HITW r=%0d c=%0d addr=%0d", u_dut.dec_r, u_dut.dec_c,
                 u_dut.dec_addr);

always @(posedge clk)
    if (pbox_valid) pub_cnt <= pub_cnt + 32'd1;

initial begin
    if (!$value$plusargs("IN=%s", in_path) ||
        !$value$plusargs("TRK=%s", trk_path) ||
        !$value$plusargs("OUT=%s", out_path) ||
        !$value$plusargs("FRAMES=%d", nframes)) begin
        $display("ERROR missing plusarg (need IN, TRK, OUT, FRAMES)");
        $finish;
    end
    if (!$value$plusargs("WAIT=%d", wait_n)) wait_n = 300;
    dump_on = $value$plusargs("DUMP=%s", dump_path);
    dbg_on = $test$plusargs("DBG");

    fd_in = $fopen(in_path, "rb");
    fd_trk = $fopen(trk_path, "r");
    fd_out = $fopen(out_path, "w");
    if (dump_on) fd_dump = $fopen(dump_path, "wb");
    if (fd_in == 0 || fd_trk == 0 || fd_out == 0) begin
        $display("ERROR cannot open input files");
        $finish;
    end

    trk_found = {NB{1'b0}};
    trk_x = {(NB*9){1'b0}}; trk_y = {(NB*9){1'b0}};
    trk_w = {(NB*9){1'b0}}; trk_h = {(NB*9){1'b0}};
    fr_found = {NB{1'b0}};
    fr_x = {(NB*9){1'b0}}; fr_y = {(NB*9){1'b0}};
    fr_w = {(NB*9){1'b0}}; fr_h = {(NB*9){1'b0}};
    pub_cnt = 32'd0;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    for (f = 0; f < nframes; f = f + 1) begin
        wr_hit_cnt = 0;
        got = $fread(fbuf, fd_in);
        if (got != FB) begin
            $display("ERROR short mask read at frame %0d: %0d", f, got);
            $finish;
        end
        for (k = 0; k < NB; k = k + 1) begin
            got = $fscanf(fd_trk, "%d %d %d %d %d\n",
                          t0, t1, t2, t3, t4);
            if (got != 5) begin
                $display("ERROR tracked read at frame %0d slot %0d", f, k);
                $finish;
            end
            fr_found[k] = t0[0];
            fr_x[k*9 +: 9] = t1[8:0];
            fr_y[k*9 +: 9] = t2[8:0];
            fr_w[k*9 +: 9] = t3[8:0];
            fr_h[k*9 +: 9] = t4[8:0];
        end

        // ---- one frame of mask pixels, one per clock
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

        if (dbg_on) begin
            while (u_dut.st != 3'd2) @(posedge clk);   // S_RUN
            hit_pass = 0;
            for (dbi = 0; dbi < NC; dbi = dbi + 1)
                hit_pass = hit_pass + u_dut.u_hit_map.mem[dbi][0];
            $display("DBG frame %0d pass-start hit_ram=%0d", f, hit_pass);
            if (dbg_on)
                $display("    sample hit_ram[43]=%0d [123]=%0d [204]=%0d [3599]=%0d",
                         u_dut.u_hit_map.mem[43][0], u_dut.u_hit_map.mem[123][0],
                         u_dut.u_hit_map.mem[204][0], u_dut.u_hit_map.mem[3599][0]);
        end

        // ---- let the end-of-frame pass take its window snapshot
        for (n = 0; n < wait_n; n = n + 1) begin
            @(negedge clk);
            @(posedge clk);
        end

        // ---- publish this frame's trackers
        @(negedge clk);
        trk_found = fr_found;
        trk_x = fr_x; trk_y = fr_y; trk_w = fr_w; trk_h = fr_h;
        trk_valid = 1'b1;
        @(negedge clk);
        trk_valid = 1'b0;

        // ---- wait for the sweep to publish
        wcnt = 0;
        while (!pbox_valid && (wcnt < 60000)) begin
            @(negedge clk);
            wcnt = wcnt + 1;
        end
        if (wcnt >= 60000) begin
            $display("ERROR no publish for frame %0d", f);
            $finish;
        end
        for (k = 0; k < NB; k = k + 1) begin
            $display("PBOX %0d %0d %0d %0d %0d %0d %0d %0d",
                     pub_cnt, k, pbox_found[k],
                     pbox_x[k*9 +: 9], pbox_y[k*9 +: 9],
                     pbox_w[k*9 +: 9], pbox_h[k*9 +: 9],
                     pbox_cells[k*13 +: 13]);
            $fdisplay(fd_out, "%0d %0d %0d %0d %0d %0d %0d",
                      pbox_found[k],
                      pbox_x[k*9 +: 9], pbox_y[k*9 +: 9],
                      pbox_w[k*9 +: 9], pbox_h[k*9 +: 9],
                      pbox_cells[k*13 +: 13]);
            $display("PDIR %0d %0d %0d", pub_cnt, k, pbox_dir[k]);
        end
        if (dbg_on) begin
            hit_n = 0; live_n = 0; snap_n = 0;
            for (dbi = 0; dbi < NC; dbi = dbi + 1) begin
                hit_n  = hit_n  + u_dut.u_hit_map.mem[dbi][0];
                live_n = live_n + (u_dut.u_live_map.mem[dbi][1:0] != 0);
                snap_n = snap_n + (u_dut.u_snap_map.mem[dbi][1:0] != 0);
            end
            $display("DBG frame %0d hit=%0d live=%0d snap=%0d decay_cnt=%0d wrote_hits=%0d",
                     f, hit_n, live_n, snap_n, u_dut.dec_cnt, wr_hit_cnt);
            for (k = 0; k < NB; k = k + 1)
                $display("    slot %0d cur %0d [%0d %0d] x [%0d %0d] pub %0d [%0d %0d] x [%0d %0d] sw_cnt=%0d",
                         k, u_dut.cur_v[k],
                         u_dut.cur_y0[k], u_dut.cur_y1[k],
                         u_dut.cur_x0[k], u_dut.cur_x1[k],
                         u_dut.pub_v[k],
                         u_dut.pub_y0[k], u_dut.pub_y1[k],
                         u_dut.pub_x0[k], u_dut.pub_x1[k],
                         u_dut.sw_cnt[k]);
        end
        if (dump_on) begin
            for (n = 0; n < NC; n = n + 1) begin
                snap_byte = {6'b0, u_dut.u_snap_map.mem[n][1:0]};
                $fwrite(fd_dump, "%c", snap_byte);
            end
        end
        $display("frame %0d done, publish %0d", f, pub_cnt);
    end

    $fclose(fd_in);
    $fclose(fd_trk);
    $fclose(fd_out);
    if (dump_on) $fclose(fd_dump);
    $display("DONE publishes=%0d", pub_cnt);
    $finish;
end

endmodule
