set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out"
file mkdir $outdir
import_device eagle_s20.db -package EG4S20NG88
cd "$root"
read_hdl -file {td/ramprobe5.v} -top ramprobe5
optimize_rtl
read_sdc "$root/td/fall_core.sdc"
optimize_gate
report_area -file "$outdir/ramprobe5_gate.area"
puts "\[ramprobe5\] DONE"
