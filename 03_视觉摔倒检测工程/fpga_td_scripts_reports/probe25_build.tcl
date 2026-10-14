#===========================================================================
# probe25_build.tcl -- colour-bar probe at the exact 25.000 MHz raster
#
# Same flow as probe_build.tcl, one difference: the video PLL is asked for
# 25 / 125 MHz (CEA-861 640x480) instead of 24 / 120 MHz.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_probe25"
file mkdir $outdir

puts "\[probe25\] import_device"
import_device eagle_s20.db -package EG4S20NG88

set_param flow qor_monitor on
set_param rtl reg_threshold 0
set_param gate pack_seq_in_io off
set_param place pr_strategy 1

cd "$root"
puts "\[probe25\] read_hdl"
read_hdl -file {rtl/hdmi/eg4_pll_24.v rtl/hdmi/vendor/video_timing_640x480.v \
                rtl/hdmi/vendor/global_def.v rtl/led595_shift.v \
                rtl/hdmi/vendor/hdmi_tx.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds.enc.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds_dat.enc.vhd \
                rtl/hdmi/enc_file/TMDSEncoder.enc.vhd \
                rtl/hdmi/enc_file/DVITransmitter.enc.vhd \
                rtl/probe/hdmi_probe_top.v rtl/probe/hdmi_probe25_top.v \
                } -top hdmi_probe25_top

puts "\[probe25\] read_adc"
read_adc "$root/td/probe.adc"

puts "\[probe25\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/probe25_rtl.area"

puts "\[probe25\] read_sdc"
read_sdc "$root/td/probe25.sdc"

puts "\[probe25\] optimize_gate"
optimize_gate

puts "\[probe25\] legalize_phy_inst"
legalize_phy_inst
read_sdc "$root/td/probe25.sdc"
report_area -file "$outdir/probe25_gate.area"

puts "\[probe25\] place"
place
report_timing_summary -file "$outdir/probe25_place.timing"

puts "\[probe25\] route"
route
fix_hold
update_timing -mode final
report_timing_summary -file "$outdir/probe25_route.timing"
flow_status -file "$outdir/probe25_flow.status"

puts "\[probe25\] bitgen"
bitgen -bit "$outdir/hdmi_probe25.bit"

puts "\[probe25\] DONE"
