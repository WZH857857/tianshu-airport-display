#===========================================================================
# board_hdmi_synth.tcl -- synthesis of `fall_camera_hdmi_multi_top`
#
# Run:  <TD>/bin/td_commands_prompt.exe board_hdmi_synth.tcl  (from this dir)
#
# Same vision core as board_multi_synth.tcl plus the camera->SDRAM->HDMI
# picture path: the vendor on-die SDRAM controller and frame store, the
# 640x480 raster, the box overlay and the vendor TMDS transmitter core.
# Stops after gate optimization so the area answer is available before the
# much longer place & route run.
#===========================================================================

set root   "E:/FPGA guo sai/fall_detection/fpga"
set outdir "$root/td/out_board_hdmi"
file mkdir $outdir

puts "\[hdmi_synth\] import_device eagle_s20.db -package EG4S20NG88"
import_device eagle_s20.db -package EG4S20NG88

cd "$root"
puts "\[hdmi_synth\] read_hdl"
read_hdl -file {rtl/gray_front.v rtl/frame_diff.v rtl/motion_morph.v rtl/multi_bbox.v \
                rtl/track_assoc.v rtl/person_score.v rtl/posture_fsm.v rtl/posture_bank.v rtl/slot_scan.v \
                rtl/status_uart.v rtl/cmos_capture_gray8.v rtl/mt9m001_cfg.v rtl/ae_ctrl.v \
                rtl/led595_shift.v rtl/motion_hist.v rtl/fall_core_multi_top.v \
                rtl/fall_camera_hdmi_multi_top.v \
                rtl/hdmi/eg4_pll_24.v rtl/hdmi/box_overlay.v \
                rtl/hdmi/vendor/video_timing_640x480.v rtl/hdmi/vendor/global_def.v \
                rtl/hdmi/vendor/afifo_16_32_256.v rtl/hdmi/vendor/afifo_32_16_256.v \
                rtl/hdmi/vendor/frame_fifo_read.v rtl/hdmi/vendor/frame_fifo_write.v \
                rtl/hdmi/vendor/frame_read_write.v rtl/hdmi/vendor/frame_store.v \
                rtl/hdmi/vendor/pattern_pixel.v rtl/hdmi/vendor/async_fifo.v \
                rtl/hdmi/vendor/sdr_as_ram.enc.v rtl/hdmi/vendor/sdr_init_ref.enc.v \
                rtl/hdmi/vendor/sdr_wrrd.enc.v rtl/hdmi/vendor/hdmi_tx.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds.enc.vhd \
                rtl/hdmi/enc_file/SerializerN_1_lvds_dat.enc.vhd \
                rtl/hdmi/enc_file/TMDSEncoder.enc.vhd \
                rtl/hdmi/enc_file/DVITransmitter.enc.vhd} -top fall_camera_hdmi_multi_top

puts "\[hdmi_synth\] read_adc"
read_adc "$root/td/fall_camera_hdmi_top.adc"

# Mapping recipes (official_strategy "AreaOptimizeHigh" / Medium): shared
# arithmetic, register-based shifters, and either area-prior LUT mapping
# (mode 1) or the balanced one (mode 2, keeps the LUT4/5 mix the packer needs
# to pair registers with logic).  The percent label pushed the design over the
# 4900 mslice limit, so this runs the same RTL through a cheaper mapper
# instead of cutting features.
if {[info exists ::env(SYN_AREA)] && $::env(SYN_AREA) != 0} {
    set area_mode $::env(SYN_AREA)
    puts "\[hdmi_synth\] area-first mapping params on (mode $area_mode)"
    set_param rtl  resource_sharing auto
    set_param rtl  shifter_style shifter_reg
    # Mode 3 leaves the gate mapper alone: both official area recipes build
    # ~3.5k LUT6 cells, and a LUT6 fills a BLE on its own, so the packer has
    # nothing left to pair 3.8k registers with.
    if {$area_mode != 3} {
        set_param gate multi_thread_mapping partition
        if {$area_mode == 1} {
            set_param gate add_map_lut_struct high
            set_param gate map_mode area_prior
        } else {
            set_param gate add_map_lut_struct on
            set_param gate map_mode balance_explore
        }
    }
}

puts "\[hdmi_synth\] optimize_rtl"
optimize_rtl
report_area -file "$outdir/hdmi_rtl.area"

puts "\[hdmi_synth\] read_sdc"
read_sdc "$root/td/fall_camera_hdmi_top.sdc"

puts "\[hdmi_synth\] optimize_gate"
optimize_gate
report_area -file "$outdir/hdmi_gate.area"

puts "\[hdmi_synth\] report_timing_summary"
report_timing_summary -file "$outdir/hdmi_gate.timing"
flow_status -file "$outdir/hdmi_synth_flow.status"

puts "\[hdmi_synth\] DONE"
