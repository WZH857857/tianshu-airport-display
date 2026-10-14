#===========================================================================
# probe_overlay.tcl -- area-only synthesis of one box_overlay variant
#
# Run from this directory:
#   td_commands_prompt.exe probe_overlay.tcl
#
# The file name is taken from the OVERLAY_FILE environment variable so the
# same probe can compare the production overlay against a candidate without
# touching the project scripts.
#===========================================================================

set root "E:/FPGA guo sai/fall_detection/fpga"
set file "$root/rtl/hdmi/box_overlay.v"
if {[info exists ::env(OVERLAY_FILE)]} {
    set file $::env(OVERLAY_FILE)
}

set outdir "$root/td/out_probe_overlay"
file mkdir $outdir

import_device eagle_s20.db -package EG4S20NG88
set srcdir [file dirname $file]
set srcfile [file tail $file]
cd $srcdir
read_hdl -file $srcfile -top box_overlay
optimize_rtl
optimize_gate
report_area -file "$outdir/overlay_gate.area"
flow_status -file "$outdir/overlay_flow.status"
puts "\[probe_overlay\] DONE $file"
