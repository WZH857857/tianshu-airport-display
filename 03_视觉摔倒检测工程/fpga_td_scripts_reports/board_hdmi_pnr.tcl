#===========================================================================
# board_hdmi_pnr.tcl -- full place & route for `fall_camera_hdmi_multi_top`
#
# Run:  <TD>/bin/td_commands_prompt.exe board_hdmi_pnr.tcl  (from this dir)
#
# Same flow as board_multi_pnr.tcl, but the design now carries the camera ->
# SDRAM frame store -> 640x480 raster -> overlay -> TMDS chain as well, so the
# utilisation is the number to watch: synthesis estimates 13803 of 19600 LUT
# and 16 of 16 BRAM32K.  Set PNR_CONGEST=1 to switch on the vendor congestion
# recovery recipe if the default placement misses.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_board_hdmi"
file mkdir $outdir

puts "\[hdmi_pnr\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

set_param flow qor_monitor on
set_param rtl reg_threshold 0
set_param gate pack_seq_in_io off
set_param place pr_strategy 1

if {[info exists ::env(PR_STRATEGY)]} {
    puts "\[hdmi_pnr\] set_param place pr_strategy $::env(PR_STRATEGY)"
    set_param place pr_strategy $::env(PR_STRATEGY)
}

# Extra placement effort: the design sits at ~84% LUT, so the default
# (fast) optimiser trades routability for effort it does not have.  These
# four knobs are what let PNR20 close with setup WNS +1.536 ns.
if {[info exists ::env(PNR_EFFORT)] && $::env(PNR_EFFORT) == 1} {
    puts "\[hdmi_pnr\] high placement effort on"
    set_param place cn_opt_effort high
    set_param place critical_hfn_opt on
    set_param place ce_weight on
    set_param place enhanced_beeline_opt 3
}

if {[info exists ::env(PNR_SEED)]} {
    puts "\[hdmi_pnr\] set_param place seed $::env(PNR_SEED)"
    set_param place seed $::env(PNR_SEED)
}

if {[info exists ::env(PNR_CONGEST)] && $::env(PNR_CONGEST) == 1} {
    puts "\[hdmi_pnr\] congestion-recovery params on"
    set_param route effort routability
    set_param place opt_timing high
    set_param place ofv 70
    set_param place opt_max_density 50
    set_param place target_density 0.6
    set_param place target_density_pp 0.7
    set_param place aggressive_opt 20
    set_param place enhanced_beeline_opt 2
    set_param place enhanced_gp on
    set_param place non_crit_opt on
    set_param place post_place_opt high
    set_param place enable_congestion_recovery on
    set_param route post_route_opt high
}

if {[info exists ::env(PNR_FIX_HOLD)] && $::env(PNR_FIX_HOLD) == 1} {
    puts "\[hdmi_pnr\] route fix_hold on"
    set_param route fix_hold on
}

cd "$root"
puts "\[hdmi_pnr\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/multi_bbox.v \
                rtl/track_assoc.v rtl/person_score.v rtl/posture_fsm.v rtl/posture_bank.v rtl/slot_scan.v \
                rtl/status_uart.v rtl/cmos_capture_gray8.v rtl/mt9m001_cfg.v rtl/ae_ctrl.v \
                rtl/led595_shift.v rtl/motion_hist.v rtl/fall_core_multi_top.v \
                rtl/fall_camera_hdmi_multi_top.v \
                rtl/hdmi/eg4_pll_24.v rtl/hdmi/box_overlay.v \
                rtl/hdmi/vendor/video_timing_640x480.v rtl/hdmi/vendor/global_def.v \
                rtl/hdmi/vendor/afifo_16_32_256.v rtl/hdmi/vendor/afifo_32_16_256.v \
                rtl/hdmi/vendor/frame_fifo_read.v rtl/hdmi/vendor/frame_fifo_write.v \
                rtl/hdmi/vendor/frame_read_write.v rtl/hdmi/vendor/frame_store.v \
                rtl/hdmi/vendor/pattern_pixel.v rtl/hdmi/vendor/async_fifo.v \
                rtl/hdmi/vendor/sdr_as_ram.enc.v rtl/hdmi/vendor/sdr_init_ref.enc.v \
                rtl/hdmi/vendor/sdr_wrrd.enc.v rtl/hdmi/vendor/hdmi_tx.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds.enc.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds_dat.enc.vhd \
                rtl/hdmi/enc_file/TMDSEncoder.enc.vhd \
                rtl/hdmi/enc_file/DVITransmitter.enc.vhd} -top fall_camera_hdmi_multi_top

puts "\[hdmi_pnr\] read_adc"
read_adc "$root/td/fall_camera_hdmi_top.adc"

# Same mapping recipe as board_hdmi_synth.tcl.  Keep both scripts on identical
# set_param values or the PNR mslice answer stops matching the synthesis
# estimate.  SYN_AREA=1 is the official AreaOptimizeHigh recipe, SYN_AREA=2
# the medium one: it keeps LUT4/5 mixing, which the packer needs to pair
# registers with logic (area_prior leaves 3849 lone registers = 3849 slices).
if {[info exists ::env(SYN_AREA)] && $::env(SYN_AREA) != 0} {
    set area_mode $::env(SYN_AREA)
    puts "\[hdmi_pnr\] area-first mapping params on (mode $area_mode)"
    set_param rtl  resource_sharing auto
    set_param rtl  shifter_style shifter_reg
    # Mode 3 leaves the gate mapper alone: both official area recipes build
    # ~3.5k LUT6 cells, and a LUT6 fills a BLE on its own, so the packer has
    # nothing left to pair 3.8k registers with (5148 mslices).
    if {$area_mode != 3} {
        set_param gate multi_thread_mapping partition
        if {$area_mode == 1} {
            set_param gate add_map_lut_struct high
            set_param gate map_mode area_prior
        } else {
            set_param gate add_map_lut_struct on
            set_param gate map_mode balance_explore
        }
    }
}

puts "\[hdmi_pnr\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/hdmi_pnr_rtl.area"

puts "\[hdmi_pnr\] read_sdc"
read_sdc "$root/td/fall_camera_hdmi_top.sdc"

puts "\[hdmi_pnr\] optimize_gate"
optimize_gate

puts "\[hdmi_pnr\] legalize_phy_inst"
legalize_phy_inst

puts "\[hdmi_pnr\] read_sdc (re-read before P&R)"
read_sdc "$root/td/fall_camera_hdmi_top.sdc"
report_area -file "$outdir/hdmi_pnr_gate.area"

puts "\[hdmi_pnr\] place"
place
report_qor -step place -file "$outdir/hdmi_place.qor"
update_timing -mode manhattan
report_timing_summary -file "$outdir/hdmi_place.timing"

puts "\[hdmi_pnr\] route"
route
puts "\[hdmi_pnr\] fix_hold"
fix_hold
export_db "$outdir/hdmi_route.db"
report_qor -step route -file "$outdir/hdmi_route.qor"
report_area -file "$outdir/hdmi_route.area"

puts "\[hdmi_pnr\] update_timing -mode final"
update_timing -mode final
report_timing_summary -file "$outdir/hdmi_route.timing"
flow_status -file "$outdir/hdmi_flow.status"

puts "\[hdmi_pnr\] bitgen"
bitgen -bit "$outdir/hdmi_multi.bit"

puts "\[hdmi_pnr\] DONE"
