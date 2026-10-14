#===========================================================================
# board_multi_synth.tcl -- non-project TD synthesis of the crowd board top
#
# Run:  <TD>/bin/td_commands_prompt.exe board_multi_synth.tcl  (from this dir)
#
# Same flow as board_synth.tcl, but for `fall_camera_multi_top`:
# fall_core_multi_top (multi_bbox + 4 x posture_fsm) + slot_scan + the
# slot-prefixed status_uart.  Pin and timing constraints are the solo files:
# the port list and the pad map are identical, only the core changed.
# Stops after gate optimization so the ERAM/area answer is available before
# the long place & route run.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_board_multi"
file mkdir $outdir

puts "\[multi_synth\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

cd "$root"
puts "\[multi_synth\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/multi_bbox.v rtl/track_assoc.v rtl/person_score.v rtl/posture_fsm.v rtl/posture_bank.v rtl/slot_scan.v rtl/status_uart.v rtl/cmos_capture_rgb565.v rtl/ov5640_sccb_cfg.v rtl/led595_shift.v rtl/motion_hist.v rtl/fall_core_multi_top.v rtl/fall_camera_multi_top.v} -top fall_camera_multi_top

puts "\[multi_synth\] read_adc"
read_adc "$root/td/fall_camera_top.adc"

puts "\[multi_synth\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/multi_rtl.area"

puts "\[multi_synth\] read_sdc"
read_sdc "$root/td/fall_camera_top.sdc"

puts "\[multi_synth\] optimize_gate"
optimize_gate
report_area -file "$outdir/multi_gate.area"
report_timing_summary -file "$outdir/multi_gate.timing"
flow_status -file "$outdir/multi_synth_flow.status"

puts "\[multi_synth\] export_db"
export_db "$outdir/multi_gate.db"

puts "\[multi_synth\] DONE"
