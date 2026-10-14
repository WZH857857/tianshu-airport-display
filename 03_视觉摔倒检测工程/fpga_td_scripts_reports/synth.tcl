#===========================================================================
# synth.tcl -- non-project TD flow for the fall-detection vision core
#
# Run:  <TD>/bin/td_commands_prompt.exe synth.tcl   (from this directory)
#
# Stops after gate-level optimization: that is where ERAM/DSP mapping is
# final and the area report is meaningful.  Place & route is a separate
# script so a long P&R run never hides a synthesis result.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out"
file mkdir $outdir

# The device database name must stay relative: TD resolves it against its
# own arch/ directory, and an absolute path breaks the package lookup.
puts "\[synth\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

# read_hdl takes a braced list of relative paths.  The project path contains
# a space, so step into it first and name every source relative to that --
# TD splits the braced list on whitespace and would otherwise cut the path.
cd "$root"
puts "\[synth\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/proj_bbox.v rtl/posture_fsm.v td/fall_core_top.v} -top fall_core_top

puts "\[synth\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/fall_core_rtl.area"

puts "\[synth\] read_sdc"
read_sdc "$root/td/fall_core.sdc"

puts "\[synth\] optimize_gate"
optimize_gate
report_area -file "$outdir/fall_core_gate.area"
report_timing_summary -file "$outdir/fall_core_gate.timing"
flow_status -file "$outdir/synth_flow.status"

puts "\[synth\] export_db"
export_db "$outdir/fall_core_gate.db"

puts "\[synth\] DONE"
