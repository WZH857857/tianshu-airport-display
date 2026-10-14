#===========================================================================
# board_pnr.tcl -- full place & route for the board-level top
#
# Run:  <TD>/bin/td_commands_prompt.exe board_pnr.tcl   (from this dir)
#
# This is the design that goes on the VF-E20NG88: OV5640 (SCCB + DVP RGB565)
# -> gray_front -> frame_diff -> motion_morph -> proj_bbox -> posture_fsm ->
# 74595 LEDs + 115200 UART.  No PLL, no external memory, no soft core.
#
# Command order follows the vendor non-project example (SWUG105 3.2.1):
# read_adc must precede optimize_rtl, read_sdc must precede optimize_gate.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_board"
file mkdir $outdir

puts "\[board_pnr\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

set_param flow qor_monitor on
set_param rtl reg_threshold 0
set_param gate pack_seq_in_io off
set_param place pr_strategy 1

# Placement is seeded.  The design sits at ~73% slices, and with the corrected
# posture_fsm reference-net declarations (HDL-5373) the netlist is ~200 LUT
# larger than the run that first routed clean, so one placement can end with a
# single shorted tile.  Set PNR_SEED=<n> to try another placement without
# editing this file; unset means "tool default", which is what the committed
# numbers were produced with.
if {[info exists ::env(PNR_SEED)]} {
    puts "\[board_pnr\] set_param place seed $::env(PNR_SEED)"
    set_param place seed $::env(PNR_SEED)
}

cd "$root"
puts "\[board_pnr\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/proj_bbox.v rtl/posture_fsm.v rtl/cmos_capture_rgb565.v rtl/ov5640_sccb_cfg.v rtl/led595_shift.v rtl/status_uart.v td/fall_core_top.v rtl/fall_camera_top.v} -top fall_camera_top

puts "\[board_pnr\] read_adc"
read_adc "$root/td/fall_camera_top.adc"

puts "\[board_pnr\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/board_pnr_rtl.area"

puts "\[board_pnr\] read_sdc"
read_sdc "$root/td/fall_camera_top.sdc"

puts "\[board_pnr\] optimize_gate"
optimize_gate

puts "\[board_pnr\] legalize_phy_inst"
legalize_phy_inst

puts "\[board_pnr\] read_sdc (re-read before P&R)"
read_sdc "$root/td/fall_camera_top.sdc"
report_area -file "$outdir/board_pnr_gate.area"

puts "\[board_pnr\] place"
place
report_qor -step place -file "$outdir/board_place.qor"
update_timing -mode manhattan
report_timing_summary -file "$outdir/board_place.timing"

puts "\[board_pnr\] route"
route
puts "\[board_pnr\] fix_hold"
fix_hold
export_db "$outdir/board_route.db"
report_qor -step route -file "$outdir/board_route.qor"
report_area -file "$outdir/board_route.area"

puts "\[board_pnr\] update_timing -mode final"
update_timing -mode final
report_timing_summary -file "$outdir/board_route.timing"
report_timing_path -file "$outdir/board_route_worst.path" -ep_num 10 -path_num 1
flow_status -file "$outdir/board_flow.status"

puts "\[board_pnr\] DONE"
