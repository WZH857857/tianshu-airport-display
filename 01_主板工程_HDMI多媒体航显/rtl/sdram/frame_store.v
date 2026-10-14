`timescale 1ns / 1ps
//=============================================================================
// frame_store.v - two picture buffers on the on-die SDRAM
//
// The loader-facing and display-facing interfaces are the same as the
// earlier frame_store, but the memory scheduler is now the Anlogic lab_ex5
// frame_read_write path.  That path uses the vendor controller exactly the
// way the supplied reference design does:
//
//   * writes are collected in a 512 word clock crossing FIFO and committed in
//     256 word bursts, alternating with display reads;
//   * reads are issued in 256 word bursts into a 512 word display FIFO;
//   * a frame is reported ready only after write_finish, not when the last
//     BMP pixel is merely enqueued.
//
// The two frame bases are far enough apart that a picture can be built in the
// off-screen buffer while the other one is being displayed.
//=============================================================================
module frame_store #(
    parameter integer FRAME_LINES = 480,
    parameter integer FRAME_WORDS = 640 * 480,
    parameter [20:0]  BUF0_BASE   = 21'h000000,
    parameter [20:0]  BUF1_BASE   = 21'h080000,
    parameter integer FIFO_AW     = 8,      // kept for source compatibility
    parameter integer CMD_AW      = 7,      // kept for source compatibility
    parameter integer CACHE_MODE  = 0,      // 1 = keep every loaded frame in its slot
    parameter integer BUF_COUNT   = 6,
    parameter [20:0]  BUF_STRIDE  = 21'h050000,
    parameter integer DEFER_SWAP  = 0       // 1 = wait for swap_req
)(
    input  wire        clk,             // BMP loader domain, 50 MHz
    input  wire        mem_clk,         // SDRAM controller, 125 MHz
    input  wire        mem_clk_sft,     // SDRAM controller, 125 MHz 180 deg
    input  wire        rst_n,

    input  wire        reload,          // one cycle when a BMP load starts
    input  wire [1:0]  pattern_mode,    // fallback pattern selection
    input  wire        img_active,      // BMP parser is streaming
    input  wire        img_abort,       // discard a partially loaded frame
    input  wire        swap_req,        // publish a deferred preload
    input  wire        emg_slot,        // the load in flight is an emergency placard
    input  wire        emg_slot_sel,    // 0 = fire placard slot, 1 = exit placard slot
    input  wire        swap_target_en,  // pick one specific slot (host jump)
    input  wire [2:0]  swap_target,
    input  wire        img_valid,       // one decoded pixel is waiting
    input  wire [23:0] img_rgb,
    input  wire [11:0] img_width,       // source BMP size for this load
    input  wire [11:0] img_height,
    output wire        img_ready,       // loader may present the pixel
    output reg         frame_ready,     // at least one frame has been written
    output reg         img_frame_ready, // at least one real picture is buffered
    output reg         frame_loaded,    // pulse when a new buffer goes live
    output reg         preload_ready,   // a complete off-screen frame is ready
    output reg         swap_done,       // display accepted the deferred frame
    output reg [BUF_COUNT-1:0] cache_ready,
    output reg [2:0]  stored_buf,
    output wire [2:0] display_buf,
    output reg [11:0] display_width,
    output reg [11:0] display_height,
    output reg        display_size_valid,
    output reg        stored_pulse,
    output wire        init_done,
    output wire        cal_fail,
    output wire [3:0]  cal_latency,
    output reg  [2:0]  state_dbg,

    // SDRAM device (EG_PHY_SDRAM_2M_32)
    output wire        sdram_clk,
    output wire        sdram_ras_n,
    output wire        sdram_cas_n,
    output wire        sdram_we_n,
    output wire [10:0] sdram_addr,
    output wire [1:0]  sdram_ba,
    output wire        sdram_cs_n,
    output wire        sdram_cke,
    output wire [3:0]  sdram_dm,
    inout  wire [31:0] sdram_dq,

    // pixel side
    input  wire        pix_clk,
    input  wire        pix_de,
    input  wire [9:0]  pix_x,
    input  wire [9:0]  pix_y,
    input  wire        pix_frame_tog,
    output wire [23:0] pix_rgb,
    output reg         pix_underflow
);

    localparam [20:0] FRAME_WORDS_21 = FRAME_WORDS;
    localparam [18:0] FRAME_LAST     = FRAME_WORDS - 1;
    localparam integer BUF_AW        = 3;

    // The SDRAM only holds six full 640x480 frames (2M x 32 words), which is
    // exactly the picture ring, so the placards cannot have dedicated slots
    // above it.  A placard borrows the ring slot that is not being displayed
    // and its cache_ready bit is cleared, which keeps the slideshow off it.
    // When the alarm clears the first normal load rebuilds the whole ring.
    reg        emg_wr_slot;
    reg        emg_wr_sel;
    reg        emg_fire_ready;
    reg        emg_exit_ready;
    reg  [BUF_AW-1:0] emg_dst_slot;
    reg  [BUF_AW-1:0] emg_fire_slot_r;
    reg  [BUF_AW-1:0] emg_exit_slot_r;

    // Cache mode keeps every completed picture in its own SDRAM slot.  The
    // default mode keeps the original two-buffer behaviour for the older
    // regression tests.
    function [20:0] slot_base;
        input [BUF_AW-1:0] slot;
        begin
            if (CACHE_MODE != 0)
                slot_base = BUF0_BASE + (slot * BUF_STRIDE);
            else
                slot_base = slot[0] ? BUF1_BASE : BUF0_BASE;
        end
    endfunction

    //------------------------------------------------------------------
    // vendor SDRAM controller
    //------------------------------------------------------------------
    wire        sdr_init_done;
    wire        sdr_init_ref_vld;
    wire        sdr_busy;
    wire        sdr_rd_en;
    wire [31:0] sdr_rd_dout;

    wire        app_rd_en;
    wire [20:0] app_rd_addr;
    wire        app_wr_en;
    wire [20:0] app_wr_addr;
    wire [31:0] app_wr_din;
    wire [3:0]  app_wr_dm;

    // A partial BMP frame may need to be abandoned after a file error.  Only
    // the frame scheduler is reset in that case; the SDRAM controller keeps
    // its initialization and the currently displayed frame stays untouched.
    reg  abort_rst;
    reg  write_abort;
    wire frame_rst = ~rst_n | abort_rst;
    wire sdram_rst = ~rst_n;

    // The vendor controller exposes no calibration result, so cal_fail is
    // reused as a sticky "a display read underflow happened since reset"
    // flag.  Unlike pix_underflow (cleared at every video frame edge), this
    // bit stays high, which makes the event visible on the board LEDs.
    reg pix_underflow_seen;

    assign init_done   = sdr_init_done;
    assign cal_fail    = pix_underflow_seen;
    assign cal_latency = 4'd0;
    assign sdram_cs_n  = 1'b0;
    assign sdram_cke   = 1'b1;

    sdr_as_ram #(
        .self_refresh_open (1)
    ) u_sdram_ctrl (
        .Sdr_clk          (mem_clk),
        .Sdr_clk_sft      (mem_clk_sft),
        .Rst              (sdram_rst),
        .Sdr_init_done    (sdr_init_done),
        .Sdr_init_ref_vld (sdr_init_ref_vld),
        .Sdr_busy         (sdr_busy),
        .App_ref_req      (1'b0),
        .App_wr_en        (app_wr_en),
        .App_wr_addr      (app_wr_addr),
        .App_wr_dm        (app_wr_dm),
        .App_wr_din       (app_wr_din),
        .App_rd_en        (app_rd_en),
        .App_rd_addr      (app_rd_addr),
        .Sdr_rd_en        (sdr_rd_en),
        .Sdr_rd_dout      (sdr_rd_dout),
        .SDRAM_CLK        (sdram_clk),
        .SDR_RAS          (sdram_ras_n),
        .SDR_CAS          (sdram_cas_n),
        .SDR_WE           (sdram_we_n),
        .SDR_BA           (sdram_ba),
        .SDR_ADDR         (sdram_addr),
        .SDR_DM           (sdram_dm),
        .SDR_DQ           (sdram_dq)
    );

    //------------------------------------------------------------------
    // official frame scheduler
    //------------------------------------------------------------------
    wire        read_req_ack;
    wire        read_finish;
    wire [31:0] read_data;
    wire        read_empty;
    wire        read_fifo_aclr;
    wire        write_req_ack;
    wire        write_finish;
    wire        write_full;
    wire        write_fifo_aclr;

    // The vendor scheduler produces one-cycle pulses on mem_clk.  Sampling
    // those directly in a slower clock domain can miss them completely, so
    // each handshake is converted to a toggle in its source domain and then
    // recovered by an XOR edge detector in the destination domain.
    reg         write_req_ack_d;
    reg         write_finish_d;
    reg         write_ack_tog_mem;
    reg         write_finish_tog_mem;

    always @(posedge mem_clk or negedge rst_n) begin
        if (!rst_n) begin
            write_req_ack_d     <= 1'b0;
            write_finish_d      <= 1'b0;
            write_ack_tog_mem   <= 1'b0;
            write_finish_tog_mem<= 1'b0;
        end else begin
            write_req_ack_d <= write_req_ack;
            write_finish_d  <= write_finish;

            if (write_req_ack && !write_req_ack_d)
                write_ack_tog_mem <= ~write_ack_tog_mem;
            if (write_finish && !write_finish_d)
                write_finish_tog_mem <= ~write_finish_tog_mem;
        end
    end

    // Both schedulers now receive a complete 21-bit base address.  Cache
    // mode derives it from the slot number; the legacy two-buffer mode keeps
    // using only slots 0 and 1.
    reg  [BUF_AW-1:0] wr_buf;
    reg  [BUF_AW-1:0] read_buf;
    reg  [BUF_AW-1:0] wr_slot;
    wire [20:0] read_addr  = slot_base(read_buf);
    reg         read_req;
    reg         read_en;
    reg         write_req;
    reg         write_en;
    wire [31:0] write_data = {8'h00, img_rgb};

    // A placard load is aimed at the borrowed ring slot; normal loads keep
    // the round-robin destination.
    wire [20:0] wr_slot_base = emg_wr_slot ? slot_base(emg_dst_slot) :
                                             slot_base(wr_buf);

    frame_read_write #(
        .WRITE_V_FLIP (1),
        .FRAME_WIDTH  (640),
        .FRAME_HEIGHT (FRAME_LINES)
    ) u_frame_rw (
        .rst              (frame_rst),
        .write_abort      (~rst_n | write_abort),
        .mem_clk          (mem_clk),
        .Sdr_init_done    (sdr_init_done),
        .Sdr_init_ref_vld (sdr_init_ref_vld),
        .Sdr_busy         (sdr_busy),

        .App_rd_en        (app_rd_en),
        .App_rd_addr      (app_rd_addr),
        .Sdr_rd_en        (sdr_rd_en),
        .Sdr_rd_dout      (sdr_rd_dout),

        .read_clk         (pix_clk),
        .read_req         (read_req),
        .read_req_ack     (read_req_ack),
        .read_finish      (read_finish),
        .read_addr        (read_addr),
        .read_len         (FRAME_WORDS_21),
        .read_en          (read_en),
        .read_data        (read_data),
        .read_empty       (read_empty),
        .read_fifo_aclr   (read_fifo_aclr),

        .App_wr_en        (app_wr_en),
        .App_wr_addr      (app_wr_addr),
        .App_wr_din       (app_wr_din),
        .App_wr_dm        (app_wr_dm),

        .write_clk        (clk),
        .write_req        (write_req),
        .write_req_ack    (write_req_ack),
        .write_finish     (write_finish),
        .write_addr       (wr_slot_base),
        .write_len        (FRAME_WORDS_21),
        .write_en         (write_en),
        .write_data       (write_data),
        .write_full       (write_full),
        .write_fifo_aclr  (write_fifo_aclr)
    );

    //------------------------------------------------------------------
    // loader-side write control
    //------------------------------------------------------------------
    localparam [1:0] W_IDLE        = 2'd0,
                     W_WAIT_ACK    = 2'd1,
                     W_STREAM      = 2'd2,
                     W_WAIT_FINISH = 2'd3;

    reg  [1:0]  wstate;
    reg         wr_started;
    reg         wr_done;
    reg  [18:0] wr_count;
    reg         pending_load;
    reg         reload_d;

    reg         wa_tog_s1, wa_tog_s2, wa_tog_s3;
    reg         wf_tog_s1, wf_tog_s2, wf_tog_s3;
    reg         wr_aclr_s1, wr_aclr_s2;
    reg         ready_tog;
    reg  [BUF_AW-1:0] ready_buf;
    reg  [BUF_AW-1:0] preload_buf;

    reg         consume_tog_s1, consume_tog_s2, consume_tog_s3;
    reg  [BUF_AW-1:0] disp_buf_s1, disp_buf_s2;
    wire        write_ack_pulse    = wa_tog_s2 ^ wa_tog_s3;
    wire        write_finish_pulse = wf_tog_s2 ^ wf_tog_s3;
    wire        consume_pulse      = consume_tog_s2 ^ consume_tog_s3;

    reg  [23:0] abort_cnt;

    // Set in the pixel clock domain when the reader has accepted a newly
    // written buffer.  This is the handshake that prevents overwriting a
    // finished picture before the display has switched to it.
    reg         consume_tog_pix;
    reg  [BUF_AW-1:0] disp_buf_pix;
    reg         display_started;

    // Source dimensions stay with the picture slot.  The display side only
    // needs the size of the slot it has actually accepted, so the selected
    // value is latched in the pixel clock domain at the same handshake that
    // makes a new frame live.
    // One entry per ring slot.  Placards borrow ring slots while the alarm is
    // active, so no extra table entries (or out-of-range SDRAM addresses) are
    // needed.
    reg [11:0] slot_width  [0:BUF_COUNT-1];
    reg [11:0] slot_height [0:BUF_COUNT-1];

    // Destination of the next placard load.  The fire placard goes one slot
    // past whatever is on screen; the exit preload goes one slot past the
    // fire placard.  Keeping the two loads apart lets the player fetch the
    // exit poster early without ever overwriting the fire poster.  If that
    // second slot somehow holds the live frame (alarm re-armed before the
    // release rebuild finished), step past the live slot instead.
    wire [BUF_AW-1:0] emg_after_disp =
        (disp_buf_s2 == BUF_COUNT-1) ? {BUF_AW{1'b0}} :
                                       disp_buf_s2 + 1'b1;
    wire [BUF_AW-1:0] emg_after_fire =
        (emg_fire_slot_r == BUF_COUNT-1) ? {BUF_AW{1'b0}} :
                                           emg_fire_slot_r + 1'b1;
    wire [BUF_AW-1:0] emg_next_slot =
        (emg_slot_sel && emg_fire_ready) ?
        ((emg_after_fire == disp_buf_s2) ? emg_after_disp : emg_after_fire) :
        emg_after_disp;

    // Return the first ready slot in ring order after the currently
    // displayed slot.  "first" is used for the initial picture.
    function [BUF_AW-1:0] pick_ready_slot;
        input [BUF_AW-1:0] current;
        input              first;
        integer            k;
        reg                found;
        reg [3:0]          idx;
        begin
            pick_ready_slot = {BUF_AW{1'b0}};
            found = 1'b0;
            for (k = 0; k < BUF_COUNT; k = k + 1) begin
                if (first)
                    idx = k;
                else
                    idx = current + k + 1;
                if (idx >= BUF_COUNT)
                    idx = idx - BUF_COUNT;
                if (!found && cache_ready[idx[BUF_AW-1:0]]) begin
                    pick_ready_slot = idx[BUF_AW-1:0];
                    found = 1'b1;
                end
            end
        end
    endfunction

    wire        write_pending = (CACHE_MODE != 0) ? 1'b0 :
                                (ready_tog ^ consume_tog_s2);
    wire [BUF_AW-1:0] next_wr_buf = (CACHE_MODE != 0) ? wr_slot :
                                (frame_ready ?
                                 (disp_buf_s2[0] ? 3'd0 : 3'd1) : 3'd0);
    wire        cache_any_ready = |cache_ready;
    reg  [BUF_AW-1:0] next_swap_buf;

    // Keep this in an explicit sensitivity list.  The slot decision reads
    // cache_ready inside the function, which some simulators do not infer
    // from a function call in a continuous assignment.
    always @(ready_buf or display_started or cache_ready)
        next_swap_buf = pick_ready_slot(ready_buf, !display_started);

    assign img_ready = (wstate == W_STREAM) && img_active &&
                       !wr_done && !write_full && !wr_aclr_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wstate           <= W_IDLE;
            write_req        <= 1'b0;
            write_en         <= 1'b0;
            wr_started       <= 1'b0;
            wr_done          <= 1'b0;
            wr_count         <= 19'd0;
            wr_buf           <= 3'd0;
            wr_slot          <= 3'd0;
            pending_load     <= 1'b0;
            reload_d         <= 1'b0;
            write_abort      <= 1'b0;
            wa_tog_s1        <= 1'b0;
            wa_tog_s2        <= 1'b0;
            wa_tog_s3        <= 1'b0;
            wf_tog_s1        <= 1'b0;
            wf_tog_s2        <= 1'b0;
            wf_tog_s3        <= 1'b0;
            wr_aclr_s1       <= 1'b1;
            wr_aclr_s2       <= 1'b1;
            ready_tog        <= 1'b0;
            ready_buf        <= 3'd0;
            preload_buf      <= 3'd0;
            preload_ready    <= 1'b0;
            swap_done        <= 1'b0;
            cache_ready      <= 6'd0;
            stored_buf       <= 3'd0;
            stored_pulse     <= 1'b0;
            emg_wr_slot      <= 1'b0;
            emg_wr_sel       <= 1'b0;
            emg_fire_ready   <= 1'b0;
            emg_exit_ready   <= 1'b0;
            emg_dst_slot     <= 3'd0;
            emg_fire_slot_r  <= 3'd0;
            emg_exit_slot_r  <= 3'd0;
            display_started  <= 1'b0;
            consume_tog_s1   <= 1'b0;
            consume_tog_s2   <= 1'b0;
            consume_tog_s3   <= 1'b0;
            disp_buf_s1      <= 3'd0;
            disp_buf_s2      <= 3'd0;
            abort_rst        <= 1'b0;
            abort_cnt        <= 24'd0;
            frame_ready      <= 1'b0;
            img_frame_ready  <= 1'b0;
            state_dbg        <= 3'd0;
        end else begin
            reload_d <= reload;

            // synchronize the memory-side completion pulses
            wa_tog_s1 <= write_ack_tog_mem;
            wa_tog_s2 <= wa_tog_s1;
            wa_tog_s3 <= wa_tog_s2;
            wf_tog_s1 <= write_finish_tog_mem;
            wf_tog_s2 <= wf_tog_s1;
            wf_tog_s3 <= wf_tog_s2;
            wr_aclr_s1 <= write_fifo_aclr;
            wr_aclr_s2 <= wr_aclr_s1;

            // synchronize the pixel-side buffer-consumed handshake
            consume_tog_s1 <= consume_tog_pix;
            consume_tog_s2 <= consume_tog_s1;
            consume_tog_s3 <= consume_tog_s2;
            disp_buf_s1    <= disp_buf_pix;
            disp_buf_s2    <= disp_buf_s1;

            // defaults; the state machine below overrides them as needed
            write_req <= 1'b0;
            write_en  <= 1'b0;
            abort_rst <= 1'b0;
            write_abort <= 1'b0;
            swap_done <= consume_pulse;
            stored_pulse <= 1'b0;
            state_dbg <= {1'b0, wstate};

            // A completed deferred frame stays off screen until the
            // sequencer asks for it.  The pixel-side consume handshake is
            // the confirmation that the display has actually changed buffer.
            if (CACHE_MODE != 0) begin
                // A host jump names its slot directly, so it is honoured even
                // before the first frame has reached the screen.  A key press
                // or the hold timer keeps the round robin order of the cache.
                if (swap_req && swap_target_en &&
                    (cache_ready[swap_target] ||
                     (emg_fire_ready && (swap_target == emg_fire_slot_r)) ||
                     (emg_exit_ready && (swap_target == emg_exit_slot_r)))) begin
                    ready_buf       <= swap_target;
                    ready_tog       <= ~ready_tog;
                    img_frame_ready <= 1'b1;
                end else if (swap_req && cache_any_ready && display_started) begin
                    ready_buf       <= next_swap_buf;
                    ready_tog       <= ~ready_tog;
                    img_frame_ready <= 1'b1;
                end
            end else if ((DEFER_SWAP != 0) && swap_req && preload_ready) begin
                ready_buf    <= preload_buf;
                ready_tog    <= ~ready_tog;
                preload_ready <= 1'b0;
            end

            // A hard KEY1 skip can restart the BMP load before the current
            // frame reaches FRAME_LAST, or just after the last pixel was
            // accepted but before write_finish commits it.  Flush the write
            // side immediately in either case; the display side and the last
            // complete buffer stay untouched.
            if (img_abort && ((wstate != W_IDLE) || pending_load || wr_done)) begin
                write_req    <= 1'b0;
                pending_load <= 1'b0;
                wr_started   <= 1'b0;
                wr_done      <= 1'b0;
                wr_count     <= 19'd0;
                abort_cnt    <= 24'd0;
                write_abort  <= 1'b1;
                preload_ready <= 1'b0;
                wstate       <= W_IDLE;
            end else begin
                if (reload && !reload_d && img_active)
                    pending_load <= 1'b1;

                if (!img_active && (wstate == W_IDLE))
                    pending_load <= 1'b0;

                case (wstate)
                    W_IDLE: begin
                        if (pending_load && !write_pending &&
                            !preload_ready && img_active) begin
                            pending_load     <= 1'b0;
                            // Freeze the destination choice for the whole
                            // transfer.
                            emg_wr_slot      <= emg_slot;
                            emg_wr_sel       <= emg_slot_sel;
                            if (emg_slot) begin
                                emg_dst_slot <= emg_next_slot;
                                // The borrowed slot leaves the rotate set
                                // until the ring is rebuilt.
                                cache_ready[emg_next_slot] <= 1'b0;
                            end else if (emg_fire_ready || emg_exit_ready) begin
                                // First normal load after the alarm: restart
                                // the ring one slot past the placard that is
                                // still on screen, then rebuild every slot.
                                cache_ready    <= {BUF_COUNT{1'b0}};
                                emg_fire_ready <= 1'b0;
                                emg_exit_ready <= 1'b0;
                                wr_buf <= (disp_buf_s2 == BUF_COUNT-1) ?
                                          {BUF_AW{1'b0}} :
                                          disp_buf_s2 + 1'b1;
                                wr_slot <= (disp_buf_s2 == BUF_COUNT-1) ?
                                           {BUF_AW{1'b0}} :
                                           disp_buf_s2 + 1'b1;
                            end else begin
                                wr_buf <= next_wr_buf;
                            end
                            wr_started       <= 1'b0;
                            wr_done          <= 1'b0;
                            wr_count         <= 19'd0;
                            write_req        <= 1'b1;
                            wstate           <= W_WAIT_ACK;
                        end
                    end

                    W_WAIT_ACK: begin
                        write_req <= 1'b1;
                        if (write_ack_pulse) begin
                            write_req  <= 1'b0;
                            wr_started <= 1'b1;
                            wr_count   <= 19'd0;
                            wr_done    <= 1'b0;
                            wstate     <= W_STREAM;
                        end
                    end

                    W_STREAM: begin
                        if (img_valid && img_active && !wr_done &&
                            !write_full && !wr_aclr_s2) begin
                            write_en <= 1'b1;
                            if (wr_count == FRAME_LAST) begin
                                wr_done <= 1'b1;
                                wstate  <= W_WAIT_FINISH;
                            end else begin
                                wr_count <= wr_count + 19'd1;
                            end
                        end

                        // A BMP parse error can drop img_active before the
                        // frame is complete.  Keep the old picture live and
                        // discard the partial write after a short grace
                        // period.
                        if (!img_active && !wr_done) begin
                            if (abort_cnt == 24'hFF_FFFF) begin
                                abort_rst    <= 1'b1;
                                pending_load <= 1'b0;
                                wr_started   <= 1'b0;
                                wr_done      <= 1'b0;
                                abort_cnt    <= 24'd0;
                                wstate       <= W_IDLE;
                            end else begin
                                abort_cnt <= abort_cnt + 24'd1;
                            end
                        end else begin
                            abort_cnt <= 24'd0;
                        end
                    end

                    W_WAIT_FINISH: begin
                        if (write_finish_pulse) begin
                            frame_ready <= 1'b1;
                            if (emg_wr_slot) begin
                                if (emg_wr_sel) begin
                                    emg_exit_ready      <= 1'b1;
                                    emg_exit_slot_r     <= emg_dst_slot;
                                    slot_width[emg_dst_slot]  <= img_width;
                                    slot_height[emg_dst_slot] <= img_height;
                                end else begin
                                    emg_fire_ready      <= 1'b1;
                                    emg_fire_slot_r     <= emg_dst_slot;
                                    slot_width[emg_dst_slot]  <= img_width;
                                    slot_height[emg_dst_slot] <= img_height;
                                end
                            end else begin
                                slot_width[wr_buf]  <= img_width;
                                slot_height[wr_buf] <= img_height;
                            end
                            if (CACHE_MODE != 0) begin
                                if (!emg_wr_slot)
                                    cache_ready[wr_buf] <= 1'b1;
                                stored_buf          <= emg_wr_slot ? emg_dst_slot :
                                                                     wr_buf;
                                stored_pulse        <= 1'b1;
                                // The first cached frame becomes the initial
                                // live picture.  Later frames stay resident
                                // until the sequencer asks for a slot swap.
                                if (!display_started && !emg_wr_slot) begin
                                    ready_buf       <= wr_buf;
                                    ready_tog       <= ~ready_tog;
                                    display_started <= 1'b1;
                                    img_frame_ready <= 1'b1;
                                end
                                if (!emg_wr_slot) begin
                                    if (wr_slot == BUF_COUNT - 1)
                                        wr_slot <= {BUF_AW{1'b0}};
                                    else
                                        wr_slot <= wr_slot + 1'b1;
                                end
                            end else begin
                                img_frame_ready <= 1'b1;
                                if ((DEFER_SWAP != 0) && frame_ready) begin
                                    preload_buf   <= wr_buf;
                                    preload_ready <= 1'b1;
                                end else begin
                                    ready_buf <= wr_buf;
                                    ready_tog <= ~ready_tog;
                                end
                            end
                            wr_done         <= 1'b0;
                            wr_count        <= 19'd0;
                            wstate          <= W_IDLE;
                        end
                    end

                    default: begin
                        wstate <= W_IDLE;
                    end
                endcase
            end
        end
    end

    //------------------------------------------------------------------
    // display-side frame reader
    //
    // The official reader has a request/acknowledge interface.  A request is
    // made at the first pixel of a video frame and held until ack; the FIFO
    // is then allowed to stream.  If no new picture is pending, the same
    // buffer is re-read for the next frame.
    //------------------------------------------------------------------
    localparam [1:0] R_IDLE     = 2'd0,
                     R_WAIT_ACK = 2'd1,
                     R_STREAM   = 2'd2;

    reg  [1:0]  rstate;
    reg         read_started;
    reg  [18:0] rd_pix_cnt;
    reg         pending_new;
    reg  [BUF_AW-1:0] pending_buf;
    reg         req_is_new;
    reg         have_img_p;

    reg         rt_s1, rt_s2, rt_s3;
    reg  [BUF_AW-1:0] rb_s1, rb_s2;
    reg         read_ack_tog_mem;
    reg         read_req_ack_d;
    reg         ra_tog_s1, ra_tog_s2, ra_tog_s3;
    reg         rd_aclr_s1, rd_aclr_s2;
    reg         ptog_s1, ptog_s2, ptog_s3;
    reg         ab_s1, ab_s2;

    wire new_frame_ready = rt_s2 ^ rt_s3;
    wire read_ack_pulse  = ra_tog_s2 ^ ra_tog_s3;
    wire pix_frame_edge  = ptog_s2 ^ ptog_s3;
    wire read_abort      = ab_s1 | ab_s2;

    always @(posedge mem_clk or negedge rst_n) begin
        if (!rst_n) begin
            read_req_ack_d    <= 1'b0;
            read_ack_tog_mem  <= 1'b0;
        end else begin
            read_req_ack_d <= read_req_ack;
            if (read_req_ack && !read_req_ack_d)
                read_ack_tog_mem <= ~read_ack_tog_mem;
        end
    end

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            rt_s1          <= 1'b0;
            rt_s2          <= 1'b0;
            rt_s3          <= 1'b0;
            rb_s1          <= 3'd0;
            rb_s2          <= 3'd0;
            ra_tog_s1      <= 1'b0;
            ra_tog_s2      <= 1'b0;
            ra_tog_s3      <= 1'b0;
            rd_aclr_s1     <= 1'b1;
            rd_aclr_s2     <= 1'b1;
            ptog_s1        <= 1'b0;
            ptog_s2        <= 1'b0;
            ptog_s3        <= 1'b0;
            ab_s1          <= 1'b0;
            ab_s2          <= 1'b0;
            rstate         <= R_IDLE;
            read_buf       <= 3'd0;
            read_req       <= 1'b0;
            read_started   <= 1'b0;
            rd_pix_cnt     <= 19'd0;
            pending_new    <= 1'b0;
            pending_buf    <= 3'd0;
            req_is_new     <= 1'b0;
            have_img_p     <= 1'b0;
            consume_tog_pix<= 1'b0;
            disp_buf_pix   <= 3'd0;
            display_width  <= 12'd0;
            display_height <= 12'd0;
            display_size_valid <= 1'b0;
            frame_loaded   <= 1'b0;
            pix_underflow  <= 1'b0;
            pix_underflow_seen <= 1'b0;
        end else begin
            // synchronizers
            rt_s1    <= ready_tog;
            rt_s2    <= rt_s1;
            rt_s3    <= rt_s2;
            rb_s1    <= ready_buf;
            rb_s2    <= rb_s1;
            ra_tog_s1 <= read_ack_tog_mem;
            ra_tog_s2 <= ra_tog_s1;
            ra_tog_s3 <= ra_tog_s2;
            rd_aclr_s1 <= read_fifo_aclr;
            rd_aclr_s2 <= rd_aclr_s1;
            ptog_s1  <= pix_frame_tog;
            ptog_s2  <= ptog_s1;
            ptog_s3  <= ptog_s2;
            ab_s1    <= abort_rst;
            ab_s2    <= ab_s1;

            frame_loaded <= 1'b0;

            if (read_abort) begin
                rstate       <= R_IDLE;
                read_req     <= 1'b0;
                read_started <= 1'b0;
                rd_pix_cnt   <= 19'd0;
                pending_new  <= 1'b0;
            end else begin
                if (new_frame_ready) begin
                    pending_new <= 1'b1;
                    pending_buf <= rb_s2;
                end

                if (pix_frame_edge)
                    pix_underflow <= 1'b0;

                case (rstate)
                    R_IDLE: begin
                        if (pix_frame_edge && (pending_new || have_img_p)) begin
                            if (pending_new) begin
                                read_buf  <= pending_buf;
                                req_is_new<= 1'b1;
                                pending_new <= 1'b0;
                            end else begin
                                read_buf   <= disp_buf_pix;
                                req_is_new <= 1'b0;
                            end
                            read_req   <= 1'b1;
                            read_started <= 1'b0;
                            rd_pix_cnt <= 19'd0;
                            rstate     <= R_WAIT_ACK;
                        end
                    end

                    R_WAIT_ACK: begin
                        read_req <= 1'b1;
                        if (read_ack_pulse) begin
                            read_req     <= 1'b0;
                            read_started <= 1'b1;
                            rd_pix_cnt   <= 19'd0;
                            if (req_is_new) begin
                                disp_buf_pix    <= read_buf;
                                display_width   <= slot_width[read_buf];
                                display_height  <= slot_height[read_buf];
                                display_size_valid <= 1'b1;
                                consume_tog_pix <= ~consume_tog_pix;
                                have_img_p      <= 1'b1;
                                frame_loaded    <= 1'b1;
                            end
                            rstate <= R_STREAM;
                        end
                    end

                    R_STREAM: begin
                        if (rd_pix_cnt == FRAME_LAST &&
                            pix_de && !read_empty) begin
                            rd_pix_cnt   <= 19'd0;
                            read_started <= 1'b0;
                            rstate       <= R_IDLE;
                        end else if (rd_pix_cnt < FRAME_LAST &&
                                     pix_de && !read_empty) begin
                            rd_pix_cnt <= rd_pix_cnt + 19'd1;
                        end
                    end

                    default: rstate <= R_IDLE;
                endcase

                if (pix_de && read_started && read_empty) begin
                    pix_underflow <= 1'b1;
                    pix_underflow_seen <= 1'b1;
                end
            end
        end
    end

    // `read_en` is combinational because it is also the FIFO read request.
    // The FIFO presents the word one pixel clock later, exactly the delay the
    // top level applies to its video timing signals.
    always @* begin
        read_en = (rstate == R_STREAM) && read_started &&
                  pix_de && !read_empty && !rd_aclr_s2;
    end

    //------------------------------------------------------------------
    // fallback picture and pixel output
    //------------------------------------------------------------------
    // Final build uses a quiet airport-blue fallback instead of the old
    // diagnostic colour-bar generator.  This recovers the LUT budget needed
    // by the real-time flight-board layer while keeping a stable picture from
    // power-up until the first BMP reaches SDRAM.
    wire [23:0] live_rgb_now = 24'h102A44;
    reg  [23:0] live_rgb;
    reg         pix_hit;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            live_rgb <= 24'h000000;
            pix_hit  <= 1'b0;
        end else begin
            live_rgb <= live_rgb_now;
            pix_hit  <= read_en;
        end
    end

    assign pix_rgb = pix_hit ? read_data[23:0] : live_rgb;

    // The UI overlay uses the slot that the reader is actually displaying,
    // not the slot that the background loader happens to be filling.
    assign display_buf = disp_buf_pix;

endmodule
