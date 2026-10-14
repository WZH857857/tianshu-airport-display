#===========================================================================
# board_synth.tcl -- non-project TD synthesis of the board-level top
#
# Run:  <TD>/bin/td_commands_prompt.exe board_synth.tcl   (from this dir)
#
# Same style as synth.tcl but for `fall_camera_top`: SCCB + OV5640 capture +
# vision core + 74595 LEDs + 115200 UART, with the real board pinout.
# Stops after gate optimization so the ERAM/area answer is available before
# the long place & route run.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_board"
file mkdir $outdir

puts "\[board_synth\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

# The project path contains a space, so step into it and keep every source
# path relative -- read_hdl splits its braced list on whitespace.
cd "$root"
puts "\[board_synth\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/proj_bbox.v rtl/posture_fsm.v rtl/cmos_capture_rgb565.v rtl/ov5640_sccb_cfg.v rtl/led595_shift.v rtl/status_uart.v td/fall_core_top.v rtl/fall_camera_top.v} -top fall_camera_top

puts "\[board_synth\] read_adc"
read_adc "$root/td/fall_camera_top.adc"

puts "\[board_synth\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/board_rtl.area"

puts "\[board_synth\] read_sdc"
read_sdc "$root/td/fall_camera_top.sdc"

puts "\[board_synth\] optimize_gate"
optimize_gate
report_area -file "$outdir/board_gate.area"
report_timing_summary -file "$outdir/board_gate.timing"
flow_status -file "$outdir/board_synth_flow.status"

puts "\[board_synth\] export_db"
export_db "$outdir/board_gate.db"

puts "\[board_synth\] DONE"
