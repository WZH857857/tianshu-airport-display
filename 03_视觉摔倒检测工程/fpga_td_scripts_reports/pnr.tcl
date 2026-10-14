#===========================================================================
# pnr.tcl -- place & route for the fall-detection vision core
#
# Run:  <TD>/bin/td_commands_prompt.exe pnr.tcl   (from this directory)
#
# synth.tcl stops after optimize_gate; this script continues the same
# non-project flow through placement and routing, printing timing after each
# stage so a bad placement is visible before routing starts:
#
#   legalize_phy_inst -> place -> update_timing -mode manhattan
#                     -> route -> update_timing -mode final
#
# The top level here is fall_core_pnr_top, not fall_core_top: the device only
# has 66 user pads and the diagnostic top exports 79 (PHY-9016).  The wrapper
# keeps every register of the core and folds the bbox outputs into an 8-bit
# signature, so the timing/area answer still describes the real core.
#
# The command order follows the vendor's non-project example (SWUG105 3.2.1):
# read_adc must precede optimize_rtl, read_sdc must precede optimize_gate.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out"
file mkdir $outdir

puts "\[pnr\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

# Vendor defaults for a first P&R pass; qor_monitor is what report_qor needs.
set_param flow qor_monitor on
set_param rtl reg_threshold 0
set_param gate pack_seq_in_io off
set_param place pr_strategy 1

# The project path contains a space, so step into it and keep every source
# path relative -- read_hdl splits its braced list on whitespace.
cd "$root"
puts "\[pnr\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/proj_bbox.v rtl/posture_fsm.v td/fall_core_top.v td/fall_core_pnr_top.v} -top fall_core_pnr_top

puts "\[pnr\] read_adc"
read_adc "$root/td/fall_core_pins.adc"

puts "\[pnr\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/pnr_rtl.area"

puts "\[pnr\] read_sdc"
read_sdc "$root/td/fall_core.sdc"

puts "\[pnr\] optimize_gate"
optimize_gate

puts "\[pnr\] legalize_phy_inst"
legalize_phy_inst

puts "\[pnr\] read_sdc (re-read before P&R)"
read_sdc "$root/td/fall_core.sdc"
report_area -file "$outdir/pnr_gate.area"

puts "\[pnr\] place"
place
report_qor -step place -file "$outdir/pnr_place.qor"
update_timing -mode manhattan
report_timing_summary -file "$outdir/pnr_place.timing"

puts "\[pnr\] route"
route
export_db "$outdir/fall_core_route.db"
report_qor -step route -file "$outdir/pnr_route.qor"
report_area -file "$outdir/pnr_route.area"

puts "\[pnr\] update_timing -mode final"
update_timing -mode final
report_timing_summary -file "$outdir/pnr_route.timing"
# report_timing_path has no -nworst switch; the endpoint/path counts are
# -ep_num and -path_num (SWUG105 4.8.1).
report_timing_path -file "$outdir/pnr_route_worst.path" -ep_num 10 -path_num 1
flow_status -file "$outdir/pnr_flow.status"

puts "\[pnr\] DONE"
