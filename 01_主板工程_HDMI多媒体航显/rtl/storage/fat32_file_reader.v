`timescale 1ns / 1ps
//=============================================================================
// fat32_file_reader.v - read only FAT32/FAT16 root directory and file stream
//
// Scope:
//   * MBR partition table, or a super floppy BPB at LBA 0
//   * FAT32, 512 byte sectors
//   * short 8.3 directory entries in the root directory
//   * BMP files for the picture player, plus the first .WAV file found
//     (normally VOICE.WAV) for the optional voice stream
//   * FIRE.WAV, kept separately as the fire alarm announcement and streamed
//     from the same engine with load_src = 4
//   * EMGFIRE.BMP / EMGEXIT.BMP, remembered separately as the two fire
//     emergency placards and kept out of the picture playlist
//   * one selected file at a time, streamed byte by byte
//
// The sector_reader companion holds a complete sector until release is
// pulsed. All parsing below therefore sees a stable 512 byte window and can
// use four independent byte read ports for little endian fields.
//=============================================================================
module fat32_file_reader #(
    parameter integer MAX_FILES = 64
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sd_init_done,

    // directory scan
    input  wire        start_scan,
    output reg         scan_busy,
    output reg         scan_done,
    output reg         scan_error,
    output reg  [3:0]  scan_fault_code,
    output reg  [7:0]  file_count,

    // selected file stream
    input  wire [7:0]  file_index,
    input  wire [2:0]  load_src,       // 0 = picture list, 1 = WAV,
                                       // 2 = EMGFIRE.BMP, 3 = EMGEXIT.BMP,
                                       // 4 = FIRE.WAV
    input  wire        load_start,
    input  wire        load_abort,     // drop a running stream, no error
    output reg         load_busy,
    output reg         load_done,
    output reg         load_error,
    output reg  [31:0] selected_start_cluster,
    output reg  [31:0] selected_size,
    output reg         voice_found,
    output reg  [31:0] voice_start_cluster,
    output reg  [31:0] voice_size,
    // Fire emergency placards.  They are matched by name instead of joining
    // the picture list, so the slideshow keeps its normal six slots.
    output reg         emg_fire_found,
    output reg         emg_exit_found,
    // Fire alarm announcement.  Matched by name so it never enters the
    // picture list and never becomes the normal VOICE.WAV entry.
    output reg         fire_voice_found,
    output wire [7:0]  file_data,
    output reg         file_data_valid,
    input  wire        file_data_ready,

    // sector reader
    output reg         sec_start,
    output reg  [31:0] sec_lba,
    input  wire        sec_busy,
    input  wire        sec_ready,
    input  wire        sec_error,
    input  wire [3:0]  sec_error_code,
    output wire [8:0]  sec_raddr0,
    output reg  [8:0]  sec_raddr1,
    output reg  [8:0]  sec_raddr2,
    output reg  [8:0]  sec_raddr3,
    input  wire [7:0]  sec_rdata0,
    input  wire [7:0]  sec_rdata1,
    input  wire [7:0]  sec_rdata2,
    input  wire [7:0]  sec_rdata3,
    output reg         sec_release,
    output wire [5:0]  dbg_state
);

    localparam [5:0] S_IDLE        = 6'd0,
                     S_REQ_WAIT    = 6'd1,
                     S_ISSUE_MBR   = 6'd2,
                     S_PARSE_MBR   = 6'd3,
                     S_ISSUE_VBR   = 6'd4,
                     S_PARSE_VBR   = 6'd5,
                     S_VBR_CHECK   = 6'd6,
                     S_ISSUE_DIR   = 6'd7,
                     S_PARSE_DIR   = 6'd8,
                     S_DIR_ADVANCE = 6'd9,
                     S_ISSUE_DFAT  = 6'd10,
                     S_DFAT_SET    = 6'd11,
                     S_DFAT_LATCH  = 6'd12,
                     S_SCAN_FINISH = 6'd13,
                     S_ISSUE_FILE  = 6'd14,
                     S_FILE_DATA   = 6'd15,
                     S_ISSUE_FFAT  = 6'd16,
                     S_FFAT_SET    = 6'd17,
                     S_FFAT_LATCH  = 6'd18,
                     S_FILE_FINISH = 6'd19,
                     S_FAIL        = 6'd20,
                     S_REL         = 6'd21,
                     S_FILE_NEXT   = 6'd22,
                     S_DIR_LBA_MUL = 6'd23,
                     S_DIR_LBA_ADD = 6'd24,
                     S_VBR_DIR_ADD = 6'd25;

    localparam [1:0] CTX_SCAN = 2'd0,
                     CTX_FILE = 2'd1;

    reg [5:0]  state;
    reg [5:0]  req_return_state;
    reg [5:0]  after_release_state;
    reg [1:0]  context;
    reg        req_started;
    reg        scan_start_q;
    reg        load_start_q;

    //------------------------------------------------------------------
    // mount and BPB
    //------------------------------------------------------------------
    reg [31:0] part_lba [0:3];
    reg [7:0]  part_type [0:3];
    reg [1:0]  mbr_idx;
    reg [2:0]  mbr_phase;
    reg        mbr_sig_ok;
    reg [7:0]  mbr_sig_lo, mbr_sig_hi;
    reg [31:0] mount_lba;

    reg [7:0]  bps_lo, bps_hi;
    reg [7:0]  sectors_per_cluster;
    reg [7:0]  reserved_lo, reserved_hi;
    reg [7:0]  num_fats;
    reg [7:0]  root_entries_lo, root_entries_hi;
    reg [7:0]  fat16_lo, fat16_hi;
    reg [7:0]  fat32_b0, fat32_b1, fat32_b2, fat32_b3;
    reg [7:0]  root_cluster_b0, root_cluster_b1;
    reg [7:0]  root_cluster_b2, root_cluster_b3;
    reg [7:0]  vbr_sig_lo, vbr_sig_hi;
    reg [2:0]  vbr_step;
    reg        mount_is_fat32;

    wire [15:0] bps             = {bps_hi, bps_lo};
    wire [15:0] reserved        = {reserved_hi, reserved_lo};
    wire [15:0] root_entries    = {root_entries_hi, root_entries_lo};
    wire [15:0] fat16_size      = {fat16_hi, fat16_lo};
    wire [31:0] fat32_size      = {fat32_b3, fat32_b2, fat32_b1, fat32_b0};
    wire [31:0] root_cluster    = {root_cluster_b3, root_cluster_b2,
                                   root_cluster_b1, root_cluster_b0};
    wire        vbr_sig_ok      = (vbr_sig_lo == 8'h55) &&
                                  (vbr_sig_hi == 8'hAA);
    wire        use_fat32       = (fat32_size != 32'd0) &&
                                  (root_entries == 16'd0);
    wire [31:0] chosen_fat_size = use_fat32 ? fat32_size
                                            : {16'd0, fat16_size};
    wire [15:0] root_dir_sectors= (root_entries + 16'd15) >> 4;
    wire [31:0] fat_start_next  = mount_lba + {16'd0, reserved};
    wire [31:0] root_dir_start_next =
        fat_start_next + ({24'd0, num_fats} * chosen_fat_size);
    wire [31:0] data_start_next =
        root_dir_start_next + {16'd0, root_dir_sectors};
    wire        vbr_valid = (bps == 16'd512) &&
                            (sectors_per_cluster != 8'd0) &&
                            (num_fats != 8'd0) &&
                            (root_entries == 16'd0) &&
                            (fat32_size != 32'd0) &&
                            vbr_sig_ok;

    reg [31:0] fat_start;
    reg [31:0] root_dir_start;
    reg [31:0] data_start;
    reg [15:0] root_dir_sectors_reg;
    reg [31:0] fat_size;

    // Choose the first FAT partition in the MBR.
    reg [31:0] first_part_lba;
    reg        first_part_is_fat32;
    reg        first_part_found;
    integer    part_scan;

    always @* begin
        first_part_lba    = 32'd0;
        first_part_is_fat32 = 1'b0;
        first_part_found  = 1'b0;
        for (part_scan = 0; part_scan < 4; part_scan = part_scan + 1) begin
            if (!first_part_found &&
                ((part_type[part_scan] == 8'h0B) ||
                 (part_type[part_scan] == 8'h0C))) begin
                first_part_lba      = part_lba[part_scan];
                first_part_is_fat32 = 1'b1;
                first_part_found    = 1'b1;
            end else if (!first_part_found &&
                         ((part_type[part_scan] == 8'h04) ||
                          (part_type[part_scan] == 8'h06) ||
                          (part_type[part_scan] == 8'h0E))) begin
                first_part_lba      = part_lba[part_scan];
                first_part_is_fat32 = 1'b0;
                first_part_found    = 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // directory and file state
    //------------------------------------------------------------------
    reg [31:0] dir_lba;
    reg [31:0] cluster_lba_off;
    reg [15:0] dir_sector_index;
    reg [7:0]  dir_entry_idx;
    reg [2:0]  dir_step;
    reg        dir_end_seen;
    reg [7:0]  dir_first_byte;
    reg [7:0]  dir_attr;
    reg [7:0]  dir_ext0, dir_ext1, dir_ext2;
    reg [7:0]  dir_cl_hi0, dir_cl_hi1;
    reg [7:0]  dir_cl_lo0, dir_cl_lo1;
    reg [7:0]  dir_size0, dir_size1, dir_size2, dir_size3;
    reg [31:0] cur_cluster;
    reg [7:0]  sector_in_cluster;
    reg [31:0] next_cluster;

    reg [31:0] file_start [0:MAX_FILES-1];
    reg [31:0] file_size  [0:MAX_FILES-1];
    reg [7:0]  list_count;

    // The tail of the array carries the three files that are matched by name:
    // FIRE.WAV and the two emergency placards.  Parking their cluster and size
    // here keeps 256 bits out of the slice-limited fabric; the picture list
    // stops below the reserved slots.
    localparam [7:0] FIRE_SLOT = MAX_FILES - 1;
    localparam [7:0] EMG_FIRE_SLOT = MAX_FILES - 2;
    localparam [7:0] EMG_EXIT_SLOT = MAX_FILES - 3;

    // One read address covers every user of the arrays: the selected picture,
    // the fire announcement or one of the placards.  Keeping a single muxed
    // port is what lets the tool infer LUT RAM instead of a flop bank.
    wire [7:0]  file_rd_index = (load_src == 3'd4) ? FIRE_SLOT :
                                (load_src == 3'd2) ? EMG_FIRE_SLOT :
                                (load_src == 3'd3) ? EMG_EXIT_SLOT :
                                file_index;
    wire [31:0] file_rd_start = file_start[file_rd_index];
    wire [31:0] file_rd_size  = file_size[file_rd_index];

    // The first 8.3 .WAV entry in the root directory is remembered separately
    // from the BMP list so the picture player's file numbering is untouched.
    reg [31:0] voice_start_cluster_r;
    reg [31:0] voice_size_r;

    // FIRE.WAV is a second, independent .WAV entry.  It must not be mistaken
    // for the normal announcement when it happens to be listed first, so the
    // name is latched while the entry is parsed and only the remaining WAV
    // entries take part in the "first .WAV" selection.  Its cluster and size
    // live in FIRE_SLOT of the picture arrays.
    reg        wav_fire_head_q;

    // Emergency placards are matched on the 8.3 name while the directory
    // entry is parsed.  The first four characters land in emg_*_head_q, the
    // remaining four are compared in the decision step, so no name cache is
    // needed; their cluster and size live in EMG_FIRE_SLOT / EMG_EXIT_SLOT.
    reg        emg_fire_head_q;
    reg        emg_exit_head_q;

    reg [31:0] file_remaining;
    reg [9:0]  stream_idx;
    reg [23:0] stall_cnt;
    reg [2:0]  fail_code;

    wire [8:0] dir_base = {dir_entry_idx[3:0], 5'b00000};
    wire [8:0] mbr_base = 9'd446 + {3'd0, mbr_idx, 4'd0};
    wire [8:0] fat_byte_addr =
        ({24'd0, cur_cluster} * 32'd4) & 32'h0000_01FF;
    wire [31:0] fat_value = {sec_rdata3, sec_rdata2,
                             sec_rdata1, sec_rdata0} & 32'h0FFF_FFFF;
    assign sec_raddr0  = (state == S_FILE_DATA) ? stream_idx[8:0]
                                                : raddr0_reg;
    assign file_data   = sec_rdata0;
    assign dbg_state   = state;

    function [7:0] is_upper;
        input [7:0] value;
        begin
            is_upper = value & 8'hDF;
        end
    endfunction

    wire bmp_extension =
        (is_upper(dir_ext0) == 8'h42) &&
        (is_upper(dir_ext1) == 8'h4D) &&
        (is_upper(dir_ext2) == 8'h50);

    wire wav_extension =
        (is_upper(dir_ext0) == 8'h57) &&
        (is_upper(dir_ext1) == 8'h41) &&
        (is_upper(dir_ext2) == 8'h56);

    // "EMGFIRE " / "EMGEXIT " are matched on the fly: the first four name
    // bytes are compared while they are still on the sector data bus and only
    // the "E/M/G" prefix plus the fourth character are latched.
    wire [7:0] dn0 = is_upper(sec_rdata0);
    wire [7:0] dn1 = is_upper(sec_rdata1);
    wire [7:0] dn2 = is_upper(sec_rdata2);
    wire [7:0] dn3 = is_upper(sec_rdata3);

    wire emg_prefix_ok = (dn0 == 8'h45) && (dn1 == 8'h4D) && (dn2 == 8'h47);
    wire emg_fire_head  = emg_prefix_ok && (dn3 == 8'h46);     // EMGF
    wire emg_exit_head  = emg_prefix_ok && (dn3 == 8'h45);     // EMGE

    // is_upper() clears bit 5, so the space padding in an 8.3 short name
    // arrives here as 8'h00 rather than 8'h20.  Compare against the folded
    // value or the EMGFIRE / EMGEXIT entries are never recognised.
    wire emg_fire_hit = emg_fire_head_q &&
                        (dn0 == 8'h49) && (dn1 == 8'h52) &&     // I R
                        (dn2 == 8'h45) && (dn3 == 8'h00);       // E space
    wire emg_exit_hit = emg_exit_head_q &&
                        (dn0 == 8'h58) && (dn1 == 8'h49) &&     // X I
                        (dn2 == 8'h54) && (dn3 == 8'h00);       // T space

    // "FIRE.WAV" -> 8.3 name "FIRE    WAV".  The first four name bytes are
    // compared on the fly (F I R E) and the tail must be blank padding, so
    // names such as "FIRE1234" cannot steal the slot.
    wire wav_fire_head = (dn0 == 8'h46) && (dn1 == 8'h49) &&
                         (dn2 == 8'h52) && (dn3 == 8'h45);
    wire wav_fire_hit  = wav_fire_head_q &&
                         ((dn0 | dn1 | dn2 | dn3) == 8'h00);

    // Directory entry classification, shared by the decision step below.
    wire dir_eligible = (dir_first_byte != 8'hE5) &&
                        ((dir_attr & 8'h0F) != 8'h0F) &&
                        ((dir_attr & 8'h10) == 8'h00);
    wire bmp_entry = dir_eligible && bmp_extension;
    wire wav_entry = dir_eligible && wav_extension;

    // The picture arrays carry exactly one write port.  Slideshow entries use
    // list_count, the reserved tail slots carry the two placards and FIRE.WAV,
    // and everything is muxed into a single write statement so the tool keeps
    // inferring LUT RAM instead of a flop bank.
    wire        arr_we     = (bmp_entry && (emg_fire_hit || emg_exit_hit ||
                                            (list_count < EMG_EXIT_SLOT))) ||
                             (wav_entry && wav_fire_hit);
    wire [7:0]  arr_waddr  = (wav_entry && wav_fire_hit) ? FIRE_SLOT :
                             (bmp_entry && emg_fire_hit) ? EMG_FIRE_SLOT :
                             (bmp_entry && emg_exit_hit) ? EMG_EXIT_SLOT :
                             list_count;
    wire [31:0] arr_wstart = {dir_cl_hi1, dir_cl_hi0,
                              dir_cl_lo1, dir_cl_lo0};
    wire [31:0] arr_wsize  = {dir_size3, dir_size2,
                              dir_size1, dir_size0};

    // A separate register holds the parser read address; the file stream
    // overrides it combinationally with its own byte index.
    reg [8:0] raddr0_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                  <= S_IDLE;
            req_return_state       <= S_IDLE;
            after_release_state    <= S_IDLE;
            context                <= CTX_SCAN;
            req_started            <= 1'b0;
            scan_start_q           <= 1'b0;
            load_start_q           <= 1'b0;
            scan_busy              <= 1'b0;
            scan_done              <= 1'b0;
            scan_error             <= 1'b0;
            scan_fault_code        <= 4'd0;
            file_count             <= 8'd0;
            list_count             <= 8'd0;
            load_busy              <= 1'b0;
            load_done              <= 1'b0;
            load_error             <= 1'b0;
            selected_start_cluster <= 32'd0;
            selected_size          <= 32'd0;
            voice_found            <= 1'b0;
            voice_start_cluster    <= 32'd0;
            voice_size             <= 32'd0;
            voice_start_cluster_r  <= 32'd0;
            voice_size_r           <= 32'd0;
            emg_fire_found         <= 1'b0;
            emg_exit_found         <= 1'b0;
            emg_fire_head_q        <= 1'b0;
            emg_exit_head_q        <= 1'b0;
            fire_voice_found       <= 1'b0;
            wav_fire_head_q        <= 1'b0;
            file_data_valid        <= 1'b0;
            sec_start              <= 1'b0;
            sec_lba                <= 32'd0;
            raddr0_reg             <= 9'd0;
            sec_raddr1             <= 9'd0;
            sec_raddr2             <= 9'd0;
            sec_raddr3             <= 9'd0;
            sec_release            <= 1'b0;
            part_lba[0]            <= 32'd0;
            part_lba[1]            <= 32'd0;
            part_lba[2]            <= 32'd0;
            part_lba[3]            <= 32'd0;
            part_type[0]           <= 8'd0;
            part_type[1]           <= 8'd0;
            part_type[2]           <= 8'd0;
            part_type[3]           <= 8'd0;
            mbr_idx                <= 2'd0;
            mbr_phase              <= 3'd0;
            mbr_sig_ok             <= 1'b0;
            mbr_sig_lo             <= 8'd0;
            mbr_sig_hi             <= 8'd0;
            mount_lba              <= 32'd0;
            bps_lo                 <= 8'd0;
            bps_hi                 <= 8'd0;
            sectors_per_cluster    <= 8'd0;
            reserved_lo            <= 8'd0;
            reserved_hi            <= 8'd0;
            num_fats               <= 8'd0;
            root_entries_lo        <= 8'd0;
            root_entries_hi        <= 8'd0;
            fat16_lo               <= 8'd0;
            fat16_hi               <= 8'd0;
            fat32_b0               <= 8'd0;
            fat32_b1               <= 8'd0;
            fat32_b2               <= 8'd0;
            fat32_b3               <= 8'd0;
            root_cluster_b0        <= 8'd0;
            root_cluster_b1        <= 8'd0;
            root_cluster_b2        <= 8'd0;
            root_cluster_b3        <= 8'd0;
            vbr_sig_lo             <= 8'd0;
            vbr_sig_hi             <= 8'd0;
            vbr_step               <= 3'd0;
            mount_is_fat32         <= 1'b1;
            fat_start              <= 32'd0;
            root_dir_start         <= 32'd0;
            data_start             <= 32'd0;
            root_dir_sectors_reg   <= 16'd0;
            fat_size               <= 32'd0;
            dir_lba                <= 32'd0;
            dir_sector_index       <= 16'd0;
            dir_entry_idx          <= 8'd0;
            dir_step               <= 3'd0;
            dir_end_seen           <= 1'b0;
            dir_first_byte         <= 8'd0;
            dir_attr               <= 8'd0;
            dir_ext0               <= 8'd0;
            dir_ext1               <= 8'd0;
            dir_ext2               <= 8'd0;
            dir_cl_hi0             <= 8'd0;
            dir_cl_hi1             <= 8'd0;
            dir_cl_lo0             <= 8'd0;
            dir_cl_lo1             <= 8'd0;
            dir_size0              <= 8'd0;
            dir_size1              <= 8'd0;
            dir_size2              <= 8'd0;
            dir_size3              <= 8'd0;
            cur_cluster            <= 32'd0;
            sector_in_cluster      <= 8'd0;
            next_cluster           <= 32'd0;
            file_remaining         <= 32'd0;
            stream_idx             <= 10'd0;
            stall_cnt              <= 24'd0;
            fail_code              <= 3'd0;
        end else begin
            sec_start     <= 1'b0;
            sec_release   <= 1'b0;
            load_done     <= 1'b0;
            scan_start_q  <= start_scan;
            load_start_q  <= load_start;

            case (state)
                //------------------------------------------------------
                // idle and top level requests
                //------------------------------------------------------
                S_IDLE: begin
                    scan_busy <= 1'b0;
                    load_busy <= 1'b0;
                    if (start_scan && !scan_start_q && sd_init_done) begin
                        scan_busy       <= 1'b1;
                        scan_done       <= 1'b0;
                        scan_error      <= 1'b0;
                        scan_fault_code <= 4'd0;
                        load_error      <= 1'b0;
                        list_count      <= 8'd0;
                        file_count      <= 8'd0;
                        voice_found     <= 1'b0;
                        voice_start_cluster_r <= 32'd0;
                        voice_size_r    <= 32'd0;
                        emg_fire_found  <= 1'b0;
                        emg_exit_found  <= 1'b0;
                        emg_fire_head_q <= 1'b0;
                        emg_exit_head_q <= 1'b0;
                        fire_voice_found <= 1'b0;
                        wav_fire_head_q <= 1'b0;
                        dir_entry_idx   <= 8'd0;
                        dir_step        <= 3'd0;
                        dir_end_seen    <= 1'b0;
                        mbr_idx         <= 2'd0;
                        mbr_phase       <= 3'd0;
                        context         <= CTX_SCAN;
                        sec_lba         <= 32'd0;
                        sec_start       <= 1'b1;
                        req_return_state<= S_PARSE_MBR;
                        state           <= S_REQ_WAIT;
                    end else if (load_start && !load_start_q && scan_done &&
                                 !scan_error &&
                                 ((load_src == 3'd1 && voice_found) ||
                                  (load_src == 3'd2 && emg_fire_found) ||
                                  (load_src == 3'd3 && emg_exit_found) ||
                                  (load_src == 3'd4 && fire_voice_found) ||
                                  (load_src == 3'd0 &&
                                   (file_index < file_count)))) begin
                        case (load_src)
                            3'd1: begin
                                selected_start_cluster <= voice_start_cluster_r;
                                selected_size          <= voice_size_r;
                                cur_cluster            <= voice_start_cluster_r;
                                file_remaining         <= voice_size_r;
                            end
                            // Everything except the normal announcement is
                            // addressed through file_rd_index: pictures by
                            // file_index, the placards and FIRE.WAV by their
                            // reserved slots.
                            default: begin
                                selected_start_cluster <= file_rd_start;
                                selected_size          <= file_rd_size;
                                cur_cluster            <= file_rd_start;
                                file_remaining         <= file_rd_size;
                            end
                        endcase
                        sector_in_cluster      <= 8'd0;
                        stream_idx             <= 10'd0;
                        stall_cnt              <= 24'd0;
                        load_busy              <= 1'b1;
                        load_done              <= 1'b0;
                        load_error             <= 1'b0;
                        context                <= CTX_FILE;
                        state                  <= S_ISSUE_FILE;
                    end
                end

                S_REQ_WAIT: begin
                    sec_start <= 1'b0;
                    if (sec_ready)
                        state <= req_return_state;
                    else if (sec_error) begin
                        fail_code <= 3'd4;
                        state <= S_FAIL;
                    end
                end

                //------------------------------------------------------
                // MBR partition table
                //------------------------------------------------------
                S_ISSUE_MBR: begin
                    sec_lba          <= 32'd0;
                    sec_start        <= 1'b1;
                    req_return_state <= S_PARSE_MBR;
                    state            <= S_REQ_WAIT;
                end

                S_PARSE_MBR: begin
                    case (mbr_phase)
                        3'd0: begin
                            raddr0_reg <= mbr_base + 9'd4;
                            sec_raddr1 <= 9'd0;
                            sec_raddr2 <= 9'd0;
                            sec_raddr3 <= 9'd0;
                            mbr_phase  <= 3'd1;
                        end
                        3'd1: begin
                            part_type[mbr_idx] <= sec_rdata0;
                            raddr0_reg <= mbr_base + 9'd8;
                            sec_raddr1 <= mbr_base + 9'd9;
                            sec_raddr2 <= mbr_base + 9'd10;
                            sec_raddr3 <= mbr_base + 9'd11;
                            mbr_phase  <= 3'd2;
                        end
                        3'd2: begin
                            part_lba[mbr_idx] <= {sec_rdata3, sec_rdata2,
                                                  sec_rdata1, sec_rdata0};
                            if (mbr_idx == 2'd3) begin
                                raddr0_reg <= 9'd510;
                                sec_raddr1 <= 9'd511;
                                sec_raddr2 <= 9'd0;
                                sec_raddr3 <= 9'd0;
                                mbr_phase  <= 3'd3;
                            end else begin
                                mbr_idx   <= mbr_idx + 2'd1;
                                mbr_phase <= 3'd0;
                            end
                        end
                        3'd3: begin
                            mbr_sig_lo <= sec_rdata0;
                            mbr_sig_hi <= sec_rdata1;
                            mbr_sig_ok <= (sec_rdata0 == 8'h55) &&
                                          (sec_rdata1 == 8'hAA);
                            mbr_phase  <= 3'd4;
                        end
                        default: begin
                            if (first_part_found) begin
                                mount_lba      <= first_part_lba;
                                mount_is_fat32 <= first_part_is_fat32;
                                sec_release    <= 1'b1;
                                after_release_state <= S_ISSUE_VBR;
                                state          <= S_REL;
                            end else if (mbr_sig_ok) begin
                                // Super floppy: the BPB starts at LBA 0.
                                mount_lba      <= 32'd0;
                                mount_is_fat32 <= 1'b0;
                                sec_release    <= 1'b1;
                                after_release_state <= S_ISSUE_VBR;
                                state          <= S_REL;
                            end else begin
                                fail_code <= 3'd1;
                                state <= S_FAIL;
                            end
                        end
                    endcase
                end

                //------------------------------------------------------
                // volume boot record
                //------------------------------------------------------
                S_ISSUE_VBR: begin
                    sec_lba          <= mount_lba;
                    sec_start        <= 1'b1;
                    req_return_state <= S_PARSE_VBR;
                    state            <= S_REQ_WAIT;
                end

                S_PARSE_VBR: begin
                    case (vbr_step)
                        3'd0: begin
                            raddr0_reg <= 9'd11;
                            sec_raddr1 <= 9'd12;
                            sec_raddr2 <= 9'd13;
                            sec_raddr3 <= 9'd14;
                            vbr_step   <= 3'd1;
                        end
                        3'd1: begin
                            bps_lo                 <= sec_rdata0;
                            bps_hi                 <= sec_rdata1;
                            sectors_per_cluster    <= sec_rdata2;
                            reserved_lo            <= sec_rdata3;
                            raddr0_reg             <= 9'd15;
                            sec_raddr1             <= 9'd16;
                            sec_raddr2             <= 9'd17;
                            sec_raddr3             <= 9'd18;
                            vbr_step               <= 3'd2;
                        end
                        3'd2: begin
                            reserved_hi            <= sec_rdata0;
                            num_fats               <= sec_rdata1;
                            root_entries_lo        <= sec_rdata2;
                            root_entries_hi        <= sec_rdata3;
                            raddr0_reg             <= 9'd22;
                            sec_raddr1             <= 9'd23;
                            sec_raddr2             <= 9'd36;
                            sec_raddr3             <= 9'd37;
                            vbr_step               <= 3'd3;
                        end
                        3'd3: begin
                            fat16_lo               <= sec_rdata0;
                            fat16_hi               <= sec_rdata1;
                            fat32_b0               <= sec_rdata2;
                            fat32_b1               <= sec_rdata3;
                            raddr0_reg             <= 9'd38;
                            sec_raddr1             <= 9'd39;
                            sec_raddr2             <= 9'd44;
                            sec_raddr3             <= 9'd45;
                            vbr_step               <= 3'd4;
                        end
                        3'd4: begin
                            fat32_b2               <= sec_rdata0;
                            fat32_b3               <= sec_rdata1;
                            root_cluster_b0        <= sec_rdata2;
                            root_cluster_b1        <= sec_rdata3;
                            raddr0_reg             <= 9'd46;
                            sec_raddr1             <= 9'd47;
                            sec_raddr2             <= 9'd510;
                            sec_raddr3             <= 9'd511;
                            vbr_step               <= 3'd5;
                        end
                        3'd5: begin
                            root_cluster_b2        <= sec_rdata0;
                            root_cluster_b3        <= sec_rdata1;
                            vbr_sig_lo             <= sec_rdata2;
                            vbr_sig_hi             <= sec_rdata3;
                            vbr_step               <= 3'd6;
                            state                  <= S_VBR_CHECK;
                        end
                        default: begin
                            fail_code <= 3'd1;
                            state <= S_FAIL;
                        end
                    endcase
                end

                S_VBR_CHECK: begin
                    if (!vbr_valid) begin
                        fail_code <= 3'd1;
                        state <= S_FAIL;
                    end else begin
                        mount_is_fat32       <= use_fat32;
                        fat_size             <= chosen_fat_size;
                        fat_start            <= fat_start_next;
                        root_dir_start       <= root_dir_start_next;
                        data_start           <= data_start_next;
                        root_dir_sectors_reg <= root_dir_sectors;
                        sec_release          <= 1'b1;
                        if (use_fat32) begin
                            cur_cluster       <= root_cluster;
                            sector_in_cluster <= 8'd0;
                            // The root-directory LBA is built over two cycles.
                            // Done in one cycle the root_entries -> root_dir_sectors
                            // -> data_start_next -> multiply -> add chain measured
                            // 23.9 ns against the 20 ns sys_clk period.
                            cluster_lba_off     <= (root_cluster - 32'd2) *
                                                   {24'd0, sectors_per_cluster};
                            after_release_state <= S_VBR_DIR_ADD;
                        end else begin
                            dir_sector_index  <= 16'd0;
                            dir_lba           <= root_dir_start_next;
                            after_release_state <= S_ISSUE_DIR;
                        end
                        state                  <= S_REL;
                    end
                end

                // Second half of the FAT32 root-directory LBA build.  The
                // multiply result was registered in S_VBR_CHECK and data_start
                // has been stable since then, so only one 32-bit add is left
                // in this cycle.
                S_VBR_DIR_ADD: begin
                    dir_lba <= data_start + cluster_lba_off;
                    state   <= S_ISSUE_DIR;
                end

                //------------------------------------------------------
                // root directory
                //------------------------------------------------------
                S_ISSUE_DIR: begin
                    sec_lba          <= dir_lba;
                    sec_start        <= 1'b1;
                    req_return_state <= S_PARSE_DIR;
                    state            <= S_REQ_WAIT;
                end

                S_PARSE_DIR: begin
                    case (dir_step)
                        3'd0: begin
                            raddr0_reg <= dir_base;
                            sec_raddr1 <= dir_base + 9'd11;
                            sec_raddr2 <= dir_base + 9'd8;
                            sec_raddr3 <= dir_base + 9'd9;
                            dir_step   <= 3'd1;
                        end
                        3'd1: begin
                            dir_first_byte <= sec_rdata0;
                            dir_attr       <= sec_rdata1;
                            dir_ext0       <= sec_rdata2;
                            dir_ext1       <= sec_rdata3;
                            raddr0_reg     <= dir_base + 9'd10;
                            sec_raddr1     <= dir_base + 9'd20;
                            sec_raddr2     <= dir_base + 9'd21;
                            sec_raddr3     <= dir_base + 9'd26;
                            dir_step       <= 3'd2;
                        end
                        3'd2: begin
                            dir_ext2   <= sec_rdata0;
                            dir_cl_hi0 <= sec_rdata1;
                            dir_cl_hi1 <= sec_rdata2;
                            dir_cl_lo0 <= sec_rdata3;
                            raddr0_reg <= dir_base + 9'd27;
                            sec_raddr1 <= dir_base + 9'd28;
                            sec_raddr2 <= dir_base + 9'd29;
                            sec_raddr3 <= dir_base + 9'd30;
                            dir_step   <= 3'd3;
                        end
                        3'd3: begin
                            dir_cl_lo1 <= sec_rdata0;
                            dir_size0  <= sec_rdata1;
                            dir_size1  <= sec_rdata2;
                            dir_size2  <= sec_rdata3;
                            raddr0_reg <= dir_base + 9'd31;
                            sec_raddr1 <= 9'd0;
                            sec_raddr2 <= 9'd0;
                            sec_raddr3 <= 9'd0;
                            dir_step   <= 3'd4;
                        end
                        3'd4: begin
                            // +31 is on raddr0.  The entry decision has been
                            // moved to step 7, so the size high byte is kept
                            // in dir_size3 from here on.
                            dir_size3 <= sec_rdata0;
                            // Read the first half of the 8.3 name.
                            raddr0_reg <= dir_base;
                            sec_raddr1 <= dir_base + 9'd1;
                            sec_raddr2 <= dir_base + 9'd2;
                            sec_raddr3 <= dir_base + 9'd3;
                            dir_step   <= 3'd5;
                        end
                        3'd5: begin
                            // The sector buffer reads combinationally, so the
                            // bytes addressed in step 4 are on the bus during
                            // this step: latch the "EMGF" / "EMGE" prefix now,
                            // before the second half of the name replaces it.
                            emg_fire_head_q <= emg_fire_head;
                            emg_exit_head_q <= emg_exit_head;
                            wav_fire_head_q <= wav_fire_head;
                            // Read the second half of the 8.3 name.
                            raddr0_reg <= dir_base + 9'd4;
                            sec_raddr1 <= dir_base + 9'd5;
                            sec_raddr2 <= dir_base + 9'd6;
                            sec_raddr3 <= dir_base + 9'd7;
                            dir_step   <= 3'd6;
                        end
                        3'd6: begin
                            // sec_rdata0..3 now hold name bytes 4..7; the
                            // decision step compares them against the tail.
                            dir_step <= 3'd7;
                        end
                        3'd7: begin
                            // sec_rdata0..3 now hold name bytes 4..7.

                            if (dir_first_byte == 8'h00) begin
                                dir_end_seen <= 1'b1;
                                sec_release  <= 1'b1;
                                after_release_state <= S_SCAN_FINISH;
                                state        <= S_REL;
                            end else begin
                                if ((dir_first_byte != 8'hE5) &&
                                    ((dir_attr & 8'h0F) != 8'h0F) &&
                                    ((dir_attr & 8'h10) == 8'h00) &&
                                    bmp_extension) begin
                                    if (emg_fire_hit) begin
                                        emg_fire_found <= 1'b1;
                                    end else if (emg_exit_hit) begin
                                        emg_exit_found <= 1'b1;
                                    end else if (list_count < FIRE_SLOT) begin
                                        list_count <= list_count + 8'd1;
                                    end
                                end

                                if ((dir_first_byte != 8'hE5) &&
                                    ((dir_attr & 8'h0F) != 8'h0F) &&
                                    ((dir_attr & 8'h10) == 8'h00) &&
                                    wav_extension) begin
                                    if (wav_fire_hit) begin
                                        fire_voice_found <= 1'b1;
                                    end else if (!voice_found) begin
                                voice_start_cluster_r <= arr_wstart;
                                voice_size_r <= arr_wsize;
                                        voice_found <= 1'b1;
                                    end
                                end

                        // Single write port for both picture arrays.
                        if (arr_we) begin
                            file_start[arr_waddr] <= arr_wstart;
                            file_size [arr_waddr] <= arr_wsize;
                        end

                                if (dir_entry_idx == 8'd15) begin
                                    sec_release         <= 1'b1;
                                    after_release_state <= S_DIR_ADVANCE;
                                    state               <= S_REL;
                                end else begin
                                    dir_entry_idx <= dir_entry_idx + 8'd1;
                                    dir_step      <= 3'd0;
                                end
                            end
                        end
                        default: begin
                            fail_code <= 3'd1;
                            state <= S_FAIL;
                        end
                    endcase
                end

                S_DIR_ADVANCE: begin
                    if (dir_end_seen) begin
                        sec_release <= 1'b1;
                        state <= S_SCAN_FINISH;
                    end
                    else if (mount_is_fat32) begin
                        if (sector_in_cluster + 8'd1 <
                            sectors_per_cluster) begin
                            sector_in_cluster <= sector_in_cluster + 8'd1;
                            dir_entry_idx     <= 8'd0;
                            dir_step          <= 3'd0;
                            dir_lba           <= dir_lba + 32'd1;
                            sec_lba           <= dir_lba + 32'd1;
                            sec_start         <= 1'b1;
                            req_return_state  <= S_PARSE_DIR;
                            state             <= S_REQ_WAIT;
                        end else begin
                            state <= S_ISSUE_DFAT;
                        end
                    end else begin
                        if (({16'd0, dir_sector_index} + 32'd1) <
                            {16'd0, root_dir_sectors_reg}) begin
                            dir_sector_index <= dir_sector_index + 16'd1;
                            dir_entry_idx    <= 8'd0;
                            dir_step         <= 3'd0;
                            dir_lba          <= dir_lba + 32'd1;
                            sec_lba          <= dir_lba + 32'd1;
                            sec_start        <= 1'b1;
                            req_return_state <= S_PARSE_DIR;
                            state            <= S_REQ_WAIT;
                        end else begin
                            sec_release <= 1'b1;
                            state <= S_SCAN_FINISH;
                        end
                    end
                end

                S_ISSUE_DFAT: begin
                    // FAT entry byte offset inside its sector.
                    sec_lba          <= fat_start +
                                        ((({24'd0, cur_cluster} * 32'd4) >> 9));
                    sec_start        <= 1'b1;
                    req_return_state <= S_DFAT_SET;
                    state            <= S_REQ_WAIT;
                end

                S_DFAT_SET: begin
                    raddr0_reg <= fat_byte_addr;
                    sec_raddr1 <= fat_byte_addr + 9'd1;
                    sec_raddr2 <= fat_byte_addr + 9'd2;
                    sec_raddr3 <= fat_byte_addr + 9'd3;
                    state      <= S_DFAT_LATCH;
                end

                S_DFAT_LATCH: begin
                    next_cluster <= fat_value;
                    sec_release  <= 1'b1;
                    if (fat_value >= 32'h0FFF_FFF8) begin
                        after_release_state <= S_SCAN_FINISH;
                        state               <= S_REL;
                    end else if (fat_value == 32'h0FFF_FFF7) begin
                        fail_code <= 3'd1;
                        state <= S_FAIL;
                    end else begin
                        cur_cluster       <= fat_value;
                        sector_in_cluster <= 8'd0;
                        dir_entry_idx     <= 8'd0;
                        dir_step          <= 3'd0;
                        after_release_state <= S_DIR_LBA_MUL;
                        state               <= S_REL;
                    end
                end

                // The LBA of a newly followed cluster is built in two cycles:
                // the subtraction and the sector multiply are registered
                // first, the data area base is added to that registered offset
                // afterwards.  Done in one cycle this chain measured 20.9 ns
                // against a 20 ns sys_clk period after routing.
                S_DIR_LBA_MUL: begin
                    cluster_lba_off <= (next_cluster - 32'd2) *
                                       {24'd0, sectors_per_cluster};
                    state           <= S_DIR_LBA_ADD;
                end

                S_DIR_LBA_ADD: begin
                    dir_lba <= data_start + cluster_lba_off;
                    state   <= S_ISSUE_DIR;
                end

                S_SCAN_FINISH: begin
                    file_count <= list_count;
                    voice_start_cluster <= voice_start_cluster_r;
                    voice_size          <= voice_size_r;
                    scan_done  <= 1'b1;
                    scan_busy  <= 1'b0;
                    file_data_valid <= 1'b0;
                    state      <= S_IDLE;
                end

                //------------------------------------------------------
                // file data stream
                //------------------------------------------------------
                S_ISSUE_FILE: begin
                    sec_lba          <= data_start +
                                        ((cur_cluster - 32'd2) *
                                         {24'd0, sectors_per_cluster}) +
                                        {24'd0, sector_in_cluster};
                    sec_start        <= 1'b1;
                    req_return_state <= S_FILE_DATA;
                    state            <= S_REQ_WAIT;
                end

                S_FILE_DATA: begin
                    if (sec_ready) begin
                        file_data_valid <= 1'b1;
                        if (file_data_valid && file_data_ready) begin
                            file_data_valid <= 1'b0;
                            if (file_remaining <= 32'd1) begin
                                file_remaining <= 32'd0;
                                load_busy      <= 1'b0;
                                sec_release    <= 1'b1;
                                state          <= S_FILE_FINISH;
                            end else begin
                                file_remaining <= file_remaining - 32'd1;
                                if (stream_idx == 10'd511) begin
                                    sec_release <= 1'b1;
                                    file_data_valid <= 1'b0;
                                    if (sector_in_cluster + 8'd1 <
                                        sectors_per_cluster) begin
                                        after_release_state <= S_FILE_NEXT;
                                        state               <= S_REL;
                                    end else begin
                                        after_release_state <= S_ISSUE_FFAT;
                                        state               <= S_REL;
                                    end
                                end else begin
                                    stream_idx <= stream_idx + 10'd1;
                                end
                            end
                        end
                    end
                    if (file_data_valid && !file_data_ready) begin
                        if (stall_cnt == 24'hFF_FFFF) begin
                            load_busy   <= 1'b0;
                            load_error  <= 1'b1;
                            file_data_valid <= 1'b0;
                            sec_release <= 1'b1;
                            state       <= S_FAIL;
                        end else
                            stall_cnt <= stall_cnt + 24'd1;
                    end else
                        stall_cnt <= 24'd0;
                end

                S_ISSUE_FFAT: begin
                    sec_lba          <= fat_start +
                                        ((({24'd0, cur_cluster} * 32'd4) >> 9));
                    sec_start        <= 1'b1;
                    req_return_state <= S_FFAT_SET;
                    state            <= S_REQ_WAIT;
                end

                S_FFAT_SET: begin
                    raddr0_reg <= fat_byte_addr;
                    sec_raddr1 <= fat_byte_addr + 9'd1;
                    sec_raddr2 <= fat_byte_addr + 9'd2;
                    sec_raddr3 <= fat_byte_addr + 9'd3;
                    state      <= S_FFAT_LATCH;
                end

                S_FFAT_LATCH: begin
                    next_cluster <= fat_value;
                    sec_release  <= 1'b1;
                    if (fat_value >= 32'h0FFF_FFF8) begin
                        load_busy  <= 1'b0;
                        load_error <= 1'b1;
                        file_data_valid <= 1'b0;
                        state      <= S_IDLE;
                    end else begin
                        cur_cluster       <= fat_value;
                        sector_in_cluster <= 8'd0;
                        stream_idx        <= 10'd0;
                        file_data_valid   <= 1'b0;
                        after_release_state <= S_ISSUE_FILE;
                        state               <= S_REL;
                    end
                end

                S_FILE_NEXT: begin
                    sector_in_cluster <= sector_in_cluster + 8'd1;
                    stream_idx        <= 10'd0;
                    sec_lba           <= data_start +
                        ((cur_cluster - 32'd2) *
                         {24'd0, sectors_per_cluster}) +
                        {24'd0, sector_in_cluster + 8'd1};
                    sec_start         <= 1'b1;
                    req_return_state  <= S_FILE_DATA;
                    state             <= S_REQ_WAIT;
                end

                S_FILE_FINISH: begin
                    load_busy <= 1'b0;
                    load_done <= 1'b1;
                    state     <= S_IDLE;
                end

                S_FAIL: begin
                    sec_release <= 1'b1;
                    file_data_valid <= 1'b0;
                    if (context == CTX_FILE) begin
                        load_busy  <= 1'b0;
                        load_error <= 1'b1;
                    end else begin
                        scan_busy  <= 1'b0;
                        scan_error <= 1'b1;
                        scan_fault_code <= (sec_error_code != 4'd0) ?
                                           sec_error_code : 4'd1;
                    end
                    state <= S_IDLE;
                end

                S_REL: begin
                    state <= after_release_state;
                end

                default: state <= S_IDLE;
            endcase

            // The BMP parser knows when the last pixel of the file has been
            // taken; any trailer after that (a colour profile, for example)
            // is of no interest, so the stream is dropped without an error.
            if (load_abort && (context == CTX_FILE) && (state != S_IDLE)) begin
                sec_release     <= 1'b1;
                file_data_valid <= 1'b0;
                load_busy       <= 1'b0;
                load_done       <= 1'b0;
                state           <= S_IDLE;
            end
        end
    end

endmodule
