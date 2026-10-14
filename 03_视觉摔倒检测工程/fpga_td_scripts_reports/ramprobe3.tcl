#===========================================================================
# ramprobe3.tcl -- compare ERAM9K vs ERAM32K mapping for a 76,800 x 8 array
#===========================================================================
set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out"
file mkdir $outdir

import_device eagle_s20.db -package EG4S20NG88

cd "$root"
read_hdl -file {td/ramprobe3.v} -top ramprobe3

optimize_rtl
read_sdc "$root/td/fall_core.sdc"
optimize_gate
report_area -file "$outdir/ramprobe3_gate.area"

puts "\[ramprobe3\] DONE"
