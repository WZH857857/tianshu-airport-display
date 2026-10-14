#===========================================================================
# probe_build.tcl -- build the minimal HDMI colour-bar probe bit
#
# Run:  <TD>/bin/td_commands_prompt.exe probe_build.tcl  (from this dir)
#
# Contains only: PLL + 640x480 raster + colour bars + TMDS transmitter.
# No camera, no SDRAM frame store, no vision core.  Used to decide whether
# the "no picture" fault is in the HDMI/PLL chain or further downstream.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_probe"
file mkdir $outdir

puts "\[probe\] import_device"
import_device eagle_s20.db -package EG4S20NG88

set_param flow qor_monitor on
set_param rtl reg_threshold 0
set_param gate pack_seq_in_io off
set_param place pr_strategy 1

cd "$root"
puts "\[probe\] read_hdl"
read_hdl -file {rtl/hdmi/eg4_pll_24.v rtl/hdmi/vendor/video_timing_640x480.v \
                rtl/hdmi/vendor/global_def.v rtl/led595_shift.v \
                rtl/hdmi/vendor/hdmi_tx.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds.enc.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds_dat.enc.vhd \
                rtl/hdmi/enc_file/TMDSEncoder.enc.vhd \
                rtl/hdmi/enc_file/DVITransmitter.enc.vhd \
                rtl/probe/hdmi_probe_top.v} -top hdmi_probe_top

puts "\[probe\] read_adc"
read_adc "$root/td/probe.adc"

puts "\[probe\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/probe_rtl.area"

puts "\[probe\] read_sdc"
read_sdc "$root/td/probe.sdc"

puts "\[probe\] optimize_gate"
optimize_gate

puts "\[probe\] legalize_phy_inst"
legalize_phy_inst
read_sdc "$root/td/probe.sdc"
report_area -file "$outdir/probe_gate.area"

puts "\[probe\] place"
place
report_timing_summary -file "$outdir/probe_place.timing"

puts "\[probe\] route"
route
fix_hold
update_timing -mode final
report_timing_summary -file "$outdir/probe_route.timing"
flow_status -file "$outdir/probe_flow.status"

puts "\[probe\] bitgen"
bitgen -bit "$outdir/hdmi_probe.bit"

puts "\[probe\] DONE"
