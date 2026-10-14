`timescale 1ns / 1ps
//=============================================================================
// player_ctrl.v - top level sequencer for the picture player
//
//   1. wait for the card to finish its init sequence, then ask for a
//      directory scan
//   2. load a picture: start the file stream, start the BMP parser and tell
//      the frame store to take its pixels from that stream
//   3. legacy mode preloads the next file into an off-screen buffer.  Cache
//      mode keeps every loaded picture in its own SDRAM slot, then a key
//      press or the automatic hold timer only changes the displayed slot.
//   4. in cache mode a key press never waits for the current SPI transfer;
//      it switches to any slot that has already been cached
//   5. the automatic hold timer is optional.  With auto_rotate_en = 0 the
//      player stays on the current picture until KEY1 is pressed; with
//      auto_rotate_en = 1 the timer advances the resident slots normally.
//   6. any failure (no card, no picture, broken file, watchdog) falls back to
//      the built in test pattern, so the HDMI output never goes dead
//   7. while emg_mode is high the fire alarm owns the screen: the slideshow
//      stops and the two TF card placards (EMGFIRE.BMP / EMGEXIT.BMP) are
//      loaded into cache slots and alternate every EMG_HOLD_CYCLES.  When the
//      alarm clears the directory is scanned again and the normal cache is
//      rebuilt, so no placard survives into the slideshow.
//
// All timing is in 50 MHz system clock cycles.
//=============================================================================
module player_ctrl #(
    parameter integer HOLD_CYCLES  = 150_000_000,   // 3 s per picture
    parameter integer SWAP_TIMEOUT_CYCLES = 200_000_000, // 4 s to get swap_done
    parameter integer RETRY_CYCLES = 50_000_000,    // 1 s between retries
    parameter integer LOAD_CYCLES  = 100_000_000,   // 2 s load watchdog
    parameter integer SCAN_CYCLES  = 100_000_000,   // 2 s scan watchdog
    parameter integer PRELOAD      = 1,             // 1 = defer the swap
    parameter integer CACHE_MODE   = 0,             // 1 = all slots resident
    parameter integer CACHE_FRAMES = 6,             // maximum cached pictures
    parameter integer EMG_HOLD_CYCLES = 250_000_000 // 5 s per placard
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       key_pressed,      // one cycle pulse, already debounced
    input  wire       auto_rotate_en,   // 1 = use HOLD_CYCLES auto advance
    input  wire       jump_req,         // host asked for one specific slot
    input  wire [2:0] jump_slot,

    // fire emergency placards read from the TF card
    input  wire       emg_mode,         // 1 = latched flame alarm
    input  wire       emg_card_free,    // 1 = file reader is not streaming
    input  wire       emg_fire_found,   // EMGFIRE.BMP is on the card
    input  wire       emg_exit_found,   // EMGEXIT.BMP is on the card
    input  wire [2:0] stored_buf,       // slot that just finished writing
    output reg        emg_pic,          // a placard is the live picture
    output wire       emg_imgs_ready,   // both placards resident (or unavailable)
    // The frame store owns two placard slots above the picture ring.  These
    // two signals say which one the load in flight must land in; they are held
    // for the whole load and drop once the swap has been requested.
    output wire       emg_slot,
    output wire       emg_slot_sel,
    output reg  [1:0] load_src,         // 0 list, 1 WAV, 2/3 placards

    input  wire       sd_init_done,
    output reg        start_scan,
    input  wire       scan_done,
    input  wire       scan_error,
    input  wire [3:0] scan_fault_code,
    input  wire [7:0] file_count,

    output reg  [7:0] file_index,
    output reg        load_start,
    output reg        load_abort,
    input  wire       load_busy,
    input  wire       load_done,
    input  wire       load_error,

    output reg        bmp_start,
    input  wire       bmp_done,
    input  wire       bmp_error,

    output reg        img_active,
    output reg        frame_abort,      // discard the partially loaded frame
    output reg        reload,
    input  wire       preload_ready,    // off-screen frame is complete
    output reg        swap_req,         // request the frame-boundary swap
    input  wire       swap_done,        // display accepted the new buffer
    input  wire       stored_pulse,     // cache mode: a slot finished writing
    output reg        swap_target_en,   // the pending swap is a host jump
    output reg  [2:0] swap_target,
    output reg  [2:0] state_dbg,
    output reg        card_ok,
    output reg        file_error,
    output reg  [3:0] fs_fault_code,
    output wire       cache_ready
);

    localparam [2:0] C_WAIT_SD  = 3'd0,
                     C_SCAN     = 3'd1,
                     C_LOAD_REQ = 3'd2,
                     C_LOADING  = 3'd3,
                     C_HOLD     = 3'd4,
                     C_RETRY    = 3'd5,
                     C_FALLBACK = 3'd6,
                     C_RESTART  = 3'd7;

    reg [2:0]  state;
    reg [31:0] timer;
    reg [31:0] swap_timer;
    reg [2:0]  seq;             // small step counter inside a load request
    reg        preload_active;  // current load is not the first live picture
    reg        preload_done;    // BMP stream finished, waiting for write_finish
    reg        swap_wanted;     // key or hold timer requested the next frame
    reg        swap_pending;    // swap_req sent, waiting for swap_done

    // Resident-frame cache mode.  The frame_store owns the SDRAM slots; this
    // block only loads files until the slots are filled and then cycles the
    // displayed slot through the swap handshake.
    localparam [1:0] CC_LOAD_REQ  = 2'd0,
                     CC_LOADING   = 2'd1,
                     CC_WAIT_STORE= 2'd2,
                     CC_HOLD      = 2'd3;
    localparam [7:0] CACHE_LIMIT  = CACHE_FRAMES;

    reg [1:0] cache_state;
    reg [7:0] cache_count;
    reg [7:0] cache_attempts;

    // Emergency placard override.  emg_which selects the placard that should
    // be on screen; the slot registers remember where each one landed so a
    // later switch is only a cache swap and never touches the card again.
    reg        emg_which;       // 0 = EMGFIRE, 1 = EMGEXIT
    reg        emg_op;          // a placard load is in flight
    reg        emg_load_which;  // placard currently being loaded
    reg        emg_preload;     // load is only filling a cache slot
    reg        emg_preload_tried;
    reg        emg_preload_fail; // placard 2 did not fit in its window
    reg        emg_fire_ok;     // placard resident in a cache slot
    reg        emg_exit_ok;
    reg [2:0]  emg_fire_slot;
    reg [2:0]  emg_exit_slot;
    reg [31:0] emg_timer;
    // Set when the alarm clears: the first picture of the rebuilt cache is
    // handed to the display with a targeted swap, so the placard does not
    // stay on screen until the next key press or hold timeout.
    reg        resume_swap;
    // Keeps the emergency layer disabled while the placard is still the live
    // frame.  It clears only after the normal picture swap has completed.
    reg        resume_pending;

    wire [7:0] cache_next_attempt = cache_attempts + 8'd1;
    wire [7:0] cache_target = (file_count > CACHE_LIMIT) ?
                              CACHE_LIMIT : file_count;

    // The card-side voice stream may only take over the file reader after
    // the resident picture cache has finished filling.  state alone is not
    // enough: C_HOLD is entered before the six background loads are done.
    assign cache_ready = (state == C_HOLD) &&
                         (cache_state == CC_HOLD) &&
                         (cache_count != 8'd0);

    // The evacuation announcement must not grab the card until both placards
    // are resident.  Otherwise the FIRE.WAV stream wins the reader arbitration
    // and the second placard never finishes loading on hardware.  If the
    // second placard is absent, or its short fetch window expires, the voice
    // is released anyway so the alarm is never left silent.
    assign emg_imgs_ready = (emg_fire_ok && emg_exit_ok) ||
                            !emg_exit_found || emg_preload_fail;

    // timer[27] costs one bit instead of a 32-bit compare: 2.7 s at 50 MHz,
    // which is a generous window for one 640x480 BMP read.
    wire emg_preload_timeout = emg_op && emg_preload && timer[27];

    // True on the clock edge that forwards a key press or the hold timer to
    // the frame store.  The hold counter must not also be bumped on that
    // edge: both writes live in this always block, so the later one wins and
    // the restart below would be silently dropped.
    // A targeted swap (host jump or emergency placard) names its slot, so it
    // is honoured even before two resident pictures exist.
    wire cache_swap_issue = (state == C_HOLD) && swap_wanted && !swap_pending &&
                            ((cache_count >= 8'd2) || swap_target_en);

    // Keep the frame store aimed at the placard slot for the whole load and
    // release it only once the placard swap has been handed over.
    assign emg_slot     = emg_op && (emg_preload || !swap_pending);
    assign emg_slot_sel = emg_load_which;

    // next file in the directory, wrapping around
    wire [7:0] next_index =
        ((file_index + 8'd1) >= file_count) ? 8'd0 : (file_index + 8'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= C_WAIT_SD;
            timer       <= 28'd0;
            swap_timer  <= 32'd0;
            seq         <= 3'd0;
            start_scan  <= 1'b0;
            file_index  <= 8'd0;
            load_start  <= 1'b0;
            load_abort  <= 1'b0;
            bmp_start   <= 1'b0;
            img_active  <= 1'b0;
            frame_abort <= 1'b0;
            reload      <= 1'b0;
            swap_req    <= 1'b0;
            preload_active <= 1'b0;
            preload_done   <= 1'b0;
            swap_wanted    <= 1'b0;
            swap_pending   <= 1'b0;
            swap_target    <= 3'd0;
            swap_target_en <= 1'b0;
            cache_state    <= CC_LOAD_REQ;
            cache_count    <= 8'd0;
            cache_attempts <= 8'd0;
            emg_which      <= 1'b0;
            emg_op         <= 1'b0;
            emg_load_which <= 1'b0;
            emg_preload    <= 1'b0;
            emg_preload_tried <= 1'b0;
            emg_preload_fail <= 1'b0;
            emg_fire_ok    <= 1'b0;
            emg_exit_ok    <= 1'b0;
            emg_fire_slot  <= 3'd0;
            emg_exit_slot  <= 3'd0;
            emg_timer      <= 32'd0;
            emg_pic        <= 1'b0;
            resume_swap    <= 1'b0;
            resume_pending <= 1'b0;
            load_src       <= 2'd0;
            state_dbg   <= 3'd0;
            card_ok     <= 1'b0;
            file_error  <= 1'b0;
            fs_fault_code <= 4'd0;
        end else begin
            start_scan <= 1'b0;
            load_start <= 1'b0;
            load_abort <= 1'b0;
            bmp_start  <= 1'b0;
            frame_abort <= 1'b0;
            reload     <= 1'b0;
            swap_req   <= 1'b0;
            load_src   <= 2'd0;
            state_dbg  <= state;

            if (CACHE_MODE != 0) begin
                // Latch a key request while the background cache is being
                // filled.  As soon as at least two slots are ready, the
                // request is forwarded to the frame store.
                if ((state == C_HOLD) && key_pressed)
                    swap_wanted <= 1'b1;

                // Host slot request.  It rides the same swap handshake as
                // KEY1, so the picture change goes through the fade and the
                // audio stream is not touched at all.
                if ((state == C_HOLD) && jump_req && !swap_pending) begin
                    swap_target    <= jump_slot;
                    swap_target_en <= 1'b1;
                    swap_wanted    <= 1'b1;
                    timer          <= 32'd0;
                end

                if ((state == C_HOLD) && swap_pending && swap_done) begin
                    swap_pending   <= 1'b0;
                    swap_wanted    <= 1'b0;
                    swap_timer     <= 32'd0;
                    swap_target_en <= 1'b0;
                    if (resume_pending) begin
                        resume_pending <= 1'b0;
                        emg_pic        <= 1'b0;
                    end
                end else if ((state == C_HOLD) && swap_pending) begin
                    // A lost or stalled frame-store acknowledgement must not
                    // block every later KEY1 request forever.  The fade
                    // controller has its own black-screen watchdog; this
                    // sequencer-side watchdog clears the pending handshake
                    // after that can finish and lets the next request retry.
                    if (swap_timer >= SWAP_TIMEOUT_CYCLES) begin
                        swap_pending   <= 1'b0;
                        swap_timer     <= 32'd0;
                        if (!resume_pending) begin
                            swap_wanted    <= 1'b0;
                            swap_target_en <= 1'b0;
                        end
                    end else begin
                        swap_timer <= swap_timer + 32'd1;
                    end
                end

                if (cache_swap_issue) begin
                    swap_req     <= 1'b1;
                    swap_pending <= 1'b1;
                    swap_wanted  <= 1'b0;
                    swap_timer   <= 32'd0;
                    // Restart the automatic picture hold from this moment, so
                    // a key press is always followed by a full HOLD_CYCLES.
                    timer        <= 32'd0;
                end

                case (state)
                    C_WAIT_SD: begin
                        timer <= 32'd0;
                        if (sd_init_done) begin
                            start_scan <= 1'b1;
                            timer      <= 32'd0;
                            state      <= C_SCAN;
                        end
                    end

                    C_SCAN: begin
                        state_dbg <= C_SCAN;
                        if (scan_error) begin
                            timer <= 32'd0;
                            card_ok <= 1'b0;
                            file_error <= 1'b1;
                            fs_fault_code <= scan_fault_code;
                            reload <= 1'b1;
                            state  <= C_FALLBACK;
                        end else if (scan_done) begin
                            timer <= 32'd0;
                            if (file_count == 8'd0) begin
                                if (emg_fire_found || emg_exit_found) begin
                                    // A card that carries only the two alarm
                                    // placards still has to be able to raise
                                    // the fire screen.
                                    card_ok        <= 1'b1;
                                    file_error     <= 1'b0;
                                    fs_fault_code  <= 4'd0;
                                    file_index     <= 8'd0;
                                    cache_state    <= CC_HOLD;
                                    cache_count    <= 8'd0;
                                    cache_attempts <= 8'd0;
                                    swap_wanted    <= 1'b0;
                                    swap_pending   <= 1'b0;
                                    reload         <= 1'b1;
                                    state          <= C_HOLD;
                                end else begin
                                    card_ok <= 1'b0;
                                    file_error <= 1'b1;
                                    fs_fault_code <= 4'd2;
                                    reload <= 1'b1;
                                    state  <= C_FALLBACK;
                                end
                            end else begin
                                card_ok <= 1'b1;
                                file_error <= 1'b0;
                                fs_fault_code <= 4'd0;
                                file_index <= 8'd0;
                                cache_state <= CC_LOAD_REQ;
                                cache_count <= 8'd0;
                                cache_attempts <= 8'd0;
                                seq <= 3'd0;
                                swap_wanted <= 1'b0;
                                swap_pending <= 1'b0;
                                state <= C_HOLD;
                            end
                        end else if (timer == SCAN_CYCLES) begin
                            card_ok <= 1'b0;
                            file_error <= 1'b1;
                            fs_fault_code <= 4'd3;
                            reload <= 1'b1;
                            state  <= C_FALLBACK;
                        end else begin
                            timer <= timer + 32'd1;
                        end
                    end

                    C_HOLD: begin
                        // Alarm cleared: throw the placards away and rebuild
                        // the picture cache straight from the directory that
                        // the card reader has already scanned.  A fresh scan
                        // is deliberately avoided: start_scan is a one cycle
                        // pulse that a reader still draining the fire
                        // announcement would drop, and the old code then
                        // dead-ended in C_FALLBACK with the placard frozen on
                        // screen until KEY1 was pressed.  The rebuild also
                        // waits for the card to be free for the same reason.
                        if (!emg_mode && !resume_swap && !resume_pending &&
                            emg_card_free && (emg_pic || emg_op)) begin
                            emg_op         <= 1'b0;
                            emg_load_which <= 1'b0;
                            emg_preload    <= 1'b0;
                            emg_preload_tried <= 1'b0;
                            emg_preload_fail <= 1'b0;
                            emg_fire_ok    <= 1'b0;
                            emg_exit_ok    <= 1'b0;
                            emg_timer      <= 32'd0;
                            emg_which      <= 1'b0;
                            load_abort     <= 1'b1;
                            frame_abort    <= 1'b1;
                            img_active     <= 1'b0;
                            reload         <= 1'b1;
                            file_index     <= 8'd0;
                            seq            <= 3'd0;
                            cache_state    <= CC_LOAD_REQ;
                            cache_count    <= 8'd0;
                            cache_attempts <= 8'd0;
                            swap_wanted    <= 1'b0;
                            swap_pending   <= 1'b0;
                            swap_target_en <= 1'b0;
                            resume_swap    <= 1'b1;
                            // emg_pic stays high until the first normal
                            // frame reaches the display.
                            resume_pending <= emg_pic;
                            timer          <= 32'd0;
                            state          <= C_HOLD;
                        end else begin
                        case (cache_state)
                            CC_LOAD_REQ: begin
                                state_dbg <= C_LOAD_REQ;
                                seq <= seq + 3'd1;
                                img_active <= 1'b1;
                                case (seq)
                                    3'd0: begin
                                        load_start <= 1'b1;
                                        load_src   <= emg_op ?
                                                      (emg_load_which ?
                                                       2'd3 : 2'd2) :
                                                      2'd0;
                                        reload <= 1'b1;
                                        timer <= 32'd0;
                                    end
                                    3'd3: bmp_start <= 1'b1;
                                    3'd4: begin
                                        seq <= 3'd0;
                                        timer <= 32'd0;
                                        cache_state <= CC_LOADING;
                                    end
                                    default: ;
                                endcase
                            end

                            CC_LOADING: begin
                                state_dbg <= C_LOADING;
                                timer <= timer + 32'd1;
                                if (bmp_done) begin
                                    load_abort <= 1'b1;
                                    img_active <= 1'b0;
                                    file_error <= 1'b0;
                                    fs_fault_code <= 4'd0;
                                    timer <= 32'd0;
                                    cache_state <= CC_WAIT_STORE;
                                end else if (bmp_error || load_error ||
                                             (timer == LOAD_CYCLES) ||
                                             emg_preload_timeout) begin
                                    load_abort <= 1'b1;
                                    img_active <= 1'b0;
                                    frame_abort <= 1'b1;
                                    fs_fault_code <= bmp_error ? 4'd6 :
                                                     load_error ? 4'd5 : 4'd7;
                                    timer <= 32'd0;
                                    if (emg_op) begin
                                        // A placard that will not load must not
                                        // pull the player into the fallback
                                        // pattern; leave the alarm screen up.
                                        emg_op      <= 1'b0;
                                        if (emg_preload)
                                            emg_preload_fail <= 1'b1;
                                        cache_state <= CC_HOLD;
                                    end else begin
                                        cache_attempts <= cache_next_attempt;
                                        if (cache_next_attempt >= cache_target) begin
                                            if (cache_count == 8'd0) begin
                                                card_ok <= 1'b0;
                                                file_error <= 1'b1;
                                                reload <= 1'b1;
                                                state  <= C_FALLBACK;
                                            end else begin
                                                file_error <= 1'b0;
                                                cache_state <= CC_HOLD;
                                            end
                                        end else begin
                                            file_index <= next_index;
                                            seq <= 3'd0;
                                            cache_state <= CC_LOAD_REQ;
                                        end
                                    end
                                end
                            end

                            CC_WAIT_STORE: begin
                                state_dbg <= C_HOLD;
                                timer <= timer + 32'd1;
                                if (stored_pulse) begin
                                    cache_count <= cache_count + 8'd1;
                                    cache_attempts <= cache_next_attempt;
                                    timer <= 32'd0;
                                    if (emg_op) begin
                                        // Remember where the placard landed
                                        // and either hand the display straight
                                        // to it or keep it as a cache preload.
                                        emg_op    <= 1'b0;
                                        if (emg_load_which) begin
                                            emg_exit_ok   <= 1'b1;
                                            emg_exit_slot <= stored_buf;
                                        end else begin
                                            emg_fire_ok   <= 1'b1;
                                            emg_fire_slot <= stored_buf;
                                        end
                                        if (emg_preload) begin
                                            emg_preload <= 1'b0;
                                            cache_state <= CC_HOLD;
                                        end else begin
                                            emg_pic   <= 1'b1;
                                            emg_timer <= 32'd0;
                                            emg_which <= emg_load_which;
                                            swap_target    <= stored_buf;
                                            swap_target_en <= 1'b1;
                                            swap_wanted    <= 1'b1;
                                            cache_state    <= CC_HOLD;
                                        end
                                    end else begin
                                        if (resume_swap && !emg_mode) begin
                                            // First picture of the rebuilt
                                            // cache: replace the placard that
                                            // is still live on the display.
                                            resume_swap    <= 1'b0;
                                            swap_target    <= stored_buf;
                                            swap_target_en <= 1'b1;
                                            swap_wanted    <= 1'b1;
                                        end
                                        if (cache_next_attempt >= cache_target) begin
                                            cache_state <= CC_HOLD;
                                        end else begin
                                            file_index <= next_index;
                                            seq <= 3'd0;
                                            cache_state <= CC_LOAD_REQ;
                                        end
                                    end
                                end else if ((timer == LOAD_CYCLES) ||
                                             emg_preload_timeout) begin
                                    load_abort <= 1'b1;
                                    frame_abort <= 1'b1;
                                    fs_fault_code <= 4'd7;
                                    timer <= 32'd0;
                                    if (emg_op) begin
                                        emg_op      <= 1'b0;
                                        emg_preload <= 1'b0;
                                        if (emg_preload)
                                            emg_preload_fail <= 1'b1;
                                        cache_state <= CC_HOLD;
                                    end else begin
                                        cache_attempts <= cache_next_attempt;
                                        if (cache_next_attempt >= cache_target) begin
                                            if (cache_count == 8'd0) begin
                                                card_ok <= 1'b0;
                                                file_error <= 1'b1;
                                                reload <= 1'b1;
                                                state  <= C_FALLBACK;
                                            end else begin
                                                file_error <= 1'b0;
                                                cache_state <= CC_HOLD;
                                            end
                                        end else begin
                                            file_index <= next_index;
                                            seq <= 3'd0;
                                            cache_state <= CC_LOAD_REQ;
                                        end
                                    end
                                end
                            end

                            CC_HOLD: begin
                                state_dbg <= C_HOLD;
                                if ((cache_count == 8'd0) && !emg_mode &&
                                    !emg_fire_found && !emg_exit_found) begin
                                    card_ok <= 1'b0;
                                    file_error <= 1'b1;
                                    fs_fault_code <= 4'd7;
                                    reload <= 1'b1;
                                    state <= C_FALLBACK;
                                end else begin
                                    card_ok <= 1'b1;
                                    file_error <= 1'b0;

                                    if (emg_mode) begin
                                        // The flame alarm owns the screen:
                                        // the slideshow timer and every host
                                        // request stay parked until it clears.
                                        // A re-armed alarm also cancels any
                                        // pending resume swap.
                                        resume_swap <= 1'b0;
                                        resume_pending <= 1'b0;
                                        if (!emg_op) begin
                                            if (!emg_pic) begin
                                                if (emg_fire_ok && !swap_pending) begin
                                                    emg_pic        <= 1'b1;
                                                    emg_which      <= 1'b0;
                                                    emg_timer      <= 32'd0;
                                                    emg_preload_tried <= 1'b0;
                                                    swap_target    <= emg_fire_slot;
                                                    swap_target_en <= 1'b1;
                                                    swap_wanted    <= 1'b1;
                                                end else if (emg_fire_found &&
                                                             emg_card_free && !swap_pending) begin
                                                    emg_op      <= 1'b1;
                                                    emg_load_which <= 1'b0;
                                                    emg_preload <= 1'b0;
                                                    emg_preload_tried <= 1'b0;
                                                    img_active  <= 1'b1;
                                                    seq         <= 3'd0;
                                                    timer       <= 32'd0;
                                                    cache_state <= CC_LOAD_REQ;
                                                end else if (emg_exit_ok && !swap_pending) begin
                                                    emg_pic        <= 1'b1;
                                                    emg_which      <= 1'b1;
                                                    emg_timer      <= 32'd0;
                                                    emg_preload_tried <= 1'b0;
                                                    swap_target    <= emg_exit_slot;
                                                    swap_target_en <= 1'b1;
                                                    swap_wanted    <= 1'b1;
                                                end else if (emg_exit_found &&
                                                             emg_card_free && !swap_pending) begin
                                                    emg_op      <= 1'b1;
                                                    emg_load_which <= 1'b1;
                                                    emg_preload <= 1'b0;
                                                    emg_preload_tried <= 1'b0;
                                                    img_active  <= 1'b1;
                                                    seq         <= 3'd0;
                                                    timer       <= 32'd0;
                                                    cache_state <= CC_LOAD_REQ;
                                                end
                                            end else if (!emg_preload_tried &&
                                                         !emg_exit_ok &&
                                                         emg_exit_found &&
                                                         emg_card_free &&
                                                         !swap_pending) begin
                                                // Fetch the second placard as
                                                // soon as the first one is
                                                // live, before FIRE.WAV takes
                                                // the card.  After this point
                                                // both switches are SDRAM-only.
                                                emg_op         <= 1'b1;
                                                emg_load_which <= 1'b1;
                                                emg_preload    <= 1'b1;
                                                emg_preload_tried <= 1'b1;
                                                img_active     <= 1'b1;
                                                seq            <= 3'd0;
                                                timer          <= 32'd0;
                                                cache_state    <= CC_LOAD_REQ;
                                            end else if (!emg_preload_tried &&
                                                         !emg_fire_ok &&
                                                         emg_fire_found &&
                                                         emg_card_free &&
                                                         !swap_pending) begin
                                                emg_op         <= 1'b1;
                                                emg_load_which <= 1'b0;
                                                emg_preload    <= 1'b1;
                                                emg_preload_tried <= 1'b1;
                                                img_active     <= 1'b1;
                                                seq            <= 3'd0;
                                                timer          <= 32'd0;
                                                cache_state    <= CC_LOAD_REQ;
                                            end else if (emg_timer == EMG_HOLD_CYCLES) begin
                                                emg_timer <= 32'd0;
                                                if (emg_which) begin
                                                    // exit placard -> fire warning
                                                    if (emg_fire_ok && !swap_pending) begin
                                                        emg_which      <= 1'b0;
                                                        swap_target    <= emg_fire_slot;
                                                        swap_target_en <= 1'b1;
                                                        swap_wanted    <= 1'b1;
                                                    end else if (emg_fire_found &&
                                                                 emg_card_free && !swap_pending) begin
                                                        emg_op      <= 1'b1;
                                                        emg_load_which <= 1'b0;
                                                        emg_preload <= 1'b0;
                                                        img_active  <= 1'b1;
                                                        seq         <= 3'd0;
                                                        timer       <= 32'd0;
                                                        cache_state <= CC_LOAD_REQ;
                                                    end
                                                end else begin
                                                    // fire warning -> exit placard
                                                    if (emg_exit_ok && !swap_pending) begin
                                                        emg_which      <= 1'b1;
                                                        swap_target    <= emg_exit_slot;
                                                        swap_target_en <= 1'b1;
                                                        swap_wanted    <= 1'b1;
                                                    end else if (emg_exit_found &&
                                                                 emg_card_free && !swap_pending) begin
                                                        emg_op      <= 1'b1;
                                                        emg_load_which <= 1'b1;
                                                        emg_preload <= 1'b0;
                                                        img_active  <= 1'b1;
                                                        seq         <= 3'd0;
                                                        timer       <= 32'd0;
                                                        cache_state <= CC_LOAD_REQ;
                                                    end
                                                end
                                            end else begin
                                                emg_timer <= emg_timer + 32'd1;
                                            end
                                        end
                                    end else if (!swap_pending && !cache_swap_issue) begin
                                        if (!auto_rotate_en)
                                            // Manual mode: keep the hold
                                            // counter parked so switching
                                            // back to AUTO starts a full
                                            // display interval.
                                            timer <= 32'd0;
                                        else if (timer == HOLD_CYCLES)
                                            swap_wanted <= 1'b1;
                                        else
                                            timer <= timer + 32'd1;
                                    end
                                end
                            end

                            default: cache_state <= CC_LOAD_REQ;
                        endcase
                        end
                    end

                    C_FALLBACK: begin
                        state_dbg <= C_FALLBACK;
                        img_active <= 1'b0;
                        if (key_pressed) begin
                            file_error <= 1'b0;
                            fs_fault_code <= 4'd0;
                            start_scan <= 1'b1;
                            timer <= 32'd0;
                            cache_state <= CC_LOAD_REQ;
                            cache_count <= 8'd0;
                            cache_attempts <= 8'd0;
                            swap_wanted <= 1'b0;
                            swap_pending <= 1'b0;
                            state <= C_SCAN;
                        end
                    end

                    default: state <= C_WAIT_SD;
                endcase
            end else begin

            case (state)
                C_WAIT_SD: begin
                    timer <= 28'd0;
                    if (sd_init_done) begin
                        start_scan <= 1'b1;
                        timer      <= 28'd0;
                        state      <= C_SCAN;
                    end
                end

                C_SCAN: begin
                    if (scan_error) begin
                        timer <= 28'd0;
                        card_ok  <= 1'b0;
                        file_error <= 1'b1;
                        fs_fault_code <= scan_fault_code;
                        reload   <= 1'b1;
                        state    <= C_FALLBACK;
                    end else if (scan_done) begin
                        timer <= 28'd0;
                        if (file_count == 8'd0) begin
                            card_ok  <= 1'b0;
                            file_error <= 1'b1;
                            fs_fault_code <= 4'd2;
                            reload   <= 1'b1;
                            state    <= C_FALLBACK;
                        end else begin
                            card_ok    <= 1'b1;
                            file_error <= 1'b0;
                            fs_fault_code <= 4'd0;
                            file_index <= 8'd0;
                            preload_active <= 1'b0;
                            preload_done   <= 1'b0;
                            swap_wanted    <= 1'b0;
                            swap_pending   <= 1'b0;
                            state      <= C_LOAD_REQ;
                        end
                    end else if (timer == SCAN_CYCLES) begin
                        // the card never answered, show the test pattern
                        card_ok    <= 1'b0;
                        file_error <= 1'b1;
                        fs_fault_code <= 4'd3;
                        reload     <= 1'b1;
                        state      <= C_FALLBACK;
                    end else
                        timer <= timer + 28'd1;
                end

                C_LOAD_REQ: begin
                    if (key_pressed && !preload_active) begin
                        // Before the first picture is live a key press is a
                        // hard skip: abort the in-flight request and start
                        // the next file from a clean sequence.
                        load_abort <= 1'b1;
                        img_active <= 1'b0;
                        frame_abort <= 1'b1;
                        file_index <= next_index;
                        seq        <= 3'd0;
                        timer      <= 28'd0;
                        swap_wanted <= 1'b0;
                        swap_pending <= 1'b0;
                        preload_done <= 1'b0;
                        state      <= C_RESTART;
                    end else begin
                        // Once a picture is live, a press just remembers the
                        // requested swap.  It must not abort the preload.
                        if (key_pressed && preload_active && (PRELOAD != 0))
                            swap_wanted <= 1'b1;

                        // one cycle to start the file stream, a few more before
                        // the parser is told how long the file is
                        seq        <= seq + 3'd1;
                        img_active <= 1'b1;
                        case (seq)
                            3'd0: begin
                                load_start <= 1'b1;
                                reload     <= 1'b1;     // frame store takes pixels
                                timer      <= 28'd0;
                            end
                            3'd3: bmp_start <= 1'b1;
                            3'd4: begin
                                seq   <= 3'd0;
                                timer <= 28'd0;
                                state <= C_LOADING;
                            end
                            default: ;
                        endcase
                    end
                end

                C_LOADING: begin
                    timer <= timer + 28'd1;
                    if (key_pressed && !preload_active) begin
                        // Before the first picture is live, keep the old hard
                        // skip behavior: do not wait for the current BMP.
                        load_abort <= 1'b1;
                        img_active <= 1'b0;
                        frame_abort <= 1'b1;
                        file_index <= next_index;
                        timer      <= 28'd0;
                        seq        <= 3'd0;
                        swap_wanted <= 1'b0;
                        swap_pending <= 1'b0;
                        preload_done <= 1'b0;
                        state      <= C_RESTART;
                    end else begin
                        if (key_pressed && preload_active && (PRELOAD != 0))
                            swap_wanted <= 1'b1;

                        if (bmp_done) begin
                        // every pixel is in, drop the rest of the file
                            load_abort <= 1'b1;
                            img_active <= 1'b0;
                            file_error <= 1'b0;
                            fs_fault_code <= 4'd0;
                            timer      <= 28'd0;

                            if (PRELOAD != 0) begin
                                if (!preload_active) begin
                                    // First picture is live.  Start the
                                    // off-screen preload immediately.
                                    preload_active <= 1'b1;
                                    preload_done   <= 1'b0;
                                    file_index     <= next_index;
                                    state          <= C_LOAD_REQ;
                                end else begin
                                    // The BMP stream is complete; the frame
                                    // store will raise preload_ready after
                                    // its final SDRAM write finishes.
                                    preload_done <= 1'b1;
                                    state        <= C_HOLD;
                                end
                            end else begin
                                state <= C_HOLD;
                            end
                        end else if (bmp_error || load_error ||
                                     (timer == LOAD_CYCLES)) begin
                            load_abort <= 1'b1;
                            img_active <= 1'b0;
                            frame_abort <= 1'b1;
                            file_error <= 1'b1;
                            fs_fault_code <= bmp_error ? 4'd6 :
                                             load_error ? 4'd5 : 4'd7;
                            timer      <= 28'd0;

                            if (PRELOAD != 0 && preload_active) begin
                                // Keep the current picture on screen.  The
                                // retry state will try the following file.
                                preload_done <= 1'b0;
                                state        <= C_RETRY;
                            end else begin
                                card_ok <= 1'b0;
                                reload  <= 1'b1; // pattern takes over
                                state   <= C_RETRY;
                            end
                        end
                    end
                end

                C_HOLD: begin
                    if (PRELOAD != 0) begin
                        if (swap_pending) begin
                            if (swap_done) begin
                                // The display has accepted the preloaded
                                // buffer.  The old live buffer is now free,
                                // so start loading the following file.
                                swap_pending  <= 1'b0;
                                swap_wanted   <= 1'b0;
                                preload_done  <= 1'b0;
                                file_index    <= next_index;
                                timer         <= 28'd0;
                                state         <= C_LOAD_REQ;
                            end
                        end else begin
                            if (key_pressed ||
                                (auto_rotate_en && (timer == HOLD_CYCLES)))
                                swap_wanted <= 1'b1;

                            if (swap_wanted && preload_ready) begin
                                swap_req     <= 1'b1;
                                swap_pending <= 1'b1;
                                timer        <= 28'd0;
                            end else if (!auto_rotate_en) begin
                                timer <= 28'd0;
                            end else if (timer != HOLD_CYCLES) begin
                                timer <= timer + 28'd1;
                            end
                        end
                    end else begin
                        if (key_pressed ||
                            (auto_rotate_en && (timer == HOLD_CYCLES))) begin
                            timer      <= 28'd0;
                            img_active <= 1'b1;
                            file_index <= next_index;
                            state      <= C_LOAD_REQ;
                        end else if (!auto_rotate_en) begin
                            timer <= 28'd0;
                        end else
                            timer <= timer + 28'd1;
                    end
                end

                C_RETRY: begin
                    if (key_pressed || (timer == RETRY_CYCLES)) begin
                        timer      <= 28'd0;
                        img_active <= 1'b1;
                        file_index <= next_index;
                        state      <= C_LOAD_REQ;
                    end else
                        timer <= timer + 28'd1;
                end

                C_FALLBACK: begin
                    img_active <= 1'b0;
                    if (key_pressed) begin
                        file_error <= 1'b0;
                        fs_fault_code <= 4'd0;
                        start_scan <= 1'b1;
                        timer      <= 28'd0;
                        preload_active <= 1'b0;
                        preload_done   <= 1'b0;
                        swap_wanted    <= 1'b0;
                        swap_pending   <= 1'b0;
                        state      <= C_SCAN;
                    end
                end

                C_RESTART: begin
                    // Give the file reader one cycle to see load_abort before
                    // the next file request is issued.  A second press during
                    // this short state advances again instead of being lost.
                    img_active <= 1'b0;
                    if (key_pressed) begin
                        load_abort <= 1'b1;
                        frame_abort <= 1'b1;
                        file_index <= next_index;
                        timer      <= 28'd0;
                        seq        <= 3'd0;
                    end else begin
                        timer <= 28'd0;
                        seq   <= 3'd0;
                        preload_active <= 1'b0;
                        preload_done   <= 1'b0;
                        swap_wanted    <= 1'b0;
                        swap_pending   <= 1'b0;
                        state <= C_LOAD_REQ;
                    end
                end

                default: state <= C_WAIT_SD;
            endcase
            end
        end
    end

endmodule
