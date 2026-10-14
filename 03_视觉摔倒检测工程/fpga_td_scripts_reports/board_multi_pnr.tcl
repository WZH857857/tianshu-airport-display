#===========================================================================
# board_multi_pnr.tcl -- full place & route for the crowd board top
#
# Run:  <TD>/bin/td_commands_prompt.exe board_multi_pnr.tcl  (from this dir)
#
# Command order follows the vendor non-project example (SWUG105 3.2.1):
# read_adc must precede optimize_rtl, read_sdc must precede optimize_gate.
#
# The crowd core is the solo core plus three more posture machines and the
# slot scan front end, so the utilisation is the interesting number here:
# set PNR_SEED=<n> to try another placement without editing this file.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_board_multi"
file mkdir $outdir

puts "\[multi_pnr\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

set_param flow qor_monitor on
set_param rtl reg_threshold 0
set_param gate pack_seq_in_io off
set_param place pr_strategy 1

if {[info exists ::env(PR_STRATEGY)]} {
    puts "\[multi_pnr\] set_param place pr_strategy $::env(PR_STRATEGY)"
    set_param place pr_strategy $::env(PR_STRATEGY)
}

if {[info exists ::env(PNR_SEED)]} {
    puts "\[multi_pnr\] set_param place seed $::env(PNR_SEED)"
    set_param place seed $::env(PNR_SEED)
}

# PNR_CONGEST=1 applies the vendor "CongestionHigh" recipe from
# doc/strategies/official_strategy_ph1.json: spread the cells (lower target
# density) and turn on the congestion-recovery passes.  Needed because the
# 4-slot core sits near 77% LUT; seeds 2 and 3 both ended with PHY-8023
# shorts inside u_core/u_bbox.
if {[info exists ::env(PNR_CONGEST)] && $::env(PNR_CONGEST) == 1} {
    puts "\[multi_pnr\] congestion-recovery params on"
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

cd "$root"
puts "\[multi_pnr\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/multi_bbox.v rtl/track_assoc.v rtl/person_score.v rtl/posture_fsm.v rtl/posture_bank.v rtl/slot_scan.v rtl/status_uart.v rtl/cmos_capture_rgb565.v rtl/ov5640_sccb_cfg.v rtl/led595_shift.v rtl/motion_hist.v rtl/fall_core_multi_top.v rtl/fall_camera_multi_top.v} -top fall_camera_multi_top

puts "\[multi_pnr\] read_adc"
read_adc "$root/td/fall_camera_top.adc"

puts "\[multi_pnr\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/multi_pnr_rtl.area"

puts "\[multi_pnr\] read_sdc"
read_sdc "$root/td/fall_camera_top.sdc"

puts "\[multi_pnr\] optimize_gate"
optimize_gate

puts "\[multi_pnr\] legalize_phy_inst"
legalize_phy_inst

puts "\[multi_pnr\] read_sdc (re-read before P&R)"
read_sdc "$root/td/fall_camera_top.sdc"
report_area -file "$outdir/multi_pnr_gate.area"

puts "\[multi_pnr\] place"
place
report_qor -step place -file "$outdir/multi_place.qor"
update_timing -mode manhattan
report_timing_summary -file "$outdir/multi_place.timing"

puts "\[multi_pnr\] route"
route
puts "\[multi_pnr\] fix_hold"
fix_hold
export_db "$outdir/multi_route.db"
report_qor -step route -file "$outdir/multi_route.qor"
report_area -file "$outdir/multi_route.area"

puts "\[multi_pnr\] update_timing -mode final"
update_timing -mode final
report_timing_summary -file "$outdir/multi_route.timing"
report_timing_path -file "$outdir/multi_route_worst.path" -ep_num 10 -path_num 1
flow_status -file "$outdir/multi_flow.status"

puts "\[multi_pnr\] bitgen"
bitgen -bit "$outdir/multi.bit"

puts "\[multi_pnr\] DONE"
