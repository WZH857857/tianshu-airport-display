# Create the isolated ADV01 project.  The shared base RTL is reused, while
# the top-level wrapper enables only the layer/subtitle feature.

set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

if {[file exists ./hdmi_adv01_subtitle.al]} {
    file delete -force ./hdmi_adv01_subtitle.al
}

create_project hdmi_adv01_subtitle . -family EG4 -device EG4S20BG256
open_project ./hdmi_adv01_subtitle.al
top_module hdmi_adv01_subtitle

# design and storage / display path
add_files {./rtl/video_timing_640x480.v}
add_files {./rtl/button_debounce.v}
add_files {./rtl/async_fifo.v}
add_files {./rtl/pattern_pixel.v}
add_files {./rtl/diag_overlay.v}
add_files {./rtl/pulse_cdc.v}
add_files {./rtl/fade_ctrl.v}
add_files {./rtl/ui_overlay.v}

add_files {./rtl/sdram/vendor/global_def.v}
add_files {./rtl/sdram/vendor/sdr_as_ram.enc.v}
add_files {./rtl/sdram/vendor/sdr_init_ref.enc.v}
add_files {./rtl/sdram/vendor/sdr_wrrd.enc.v}
add_files {./rtl/sdram/vendor/afifo_16_32_256.v}
add_files {./rtl/sdram/vendor/afifo_32_16_256.v}
add_files {./rtl/sdram/vendor/frame_fifo_write.v}
add_files {./rtl/sdram/vendor/frame_fifo_read.v}
add_files {./rtl/sdram/vendor/frame_read_write.v}
add_files {./rtl/sdram/frame_store.v}
add_files {./rtl/sd/spi_master.v}
add_files {./rtl/sd/sd_card_cmd.v}
add_files {./rtl/sd/sd_card_sec_read_write.v}
add_files {./rtl/sd/sd_card_top.v}
add_files {./rtl/sd/sector_reader.v}
add_files {./rtl/storage/fat32_file_reader.v}
add_files {./rtl/storage/bmp_stream_parser.v}
add_files {./rtl/player_ctrl.v}

# official HDMI 1.4b transmitter core with audio islands
add_files {./rtl/audio/hdmi_audio_tone_i2s_64fs.v}
add_files {./rtl/audio/video_rgb_to_axis_640x480.v}
add_files {./rtl/audio/edid_retry.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/I2S_receiver.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/audio_arc_calculate.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/hdmi_phy_warpper.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/lane_lvds_10_1.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/hdmi1.4b_transmitter_core/hdmi_1_4b_transmitter_core_wrapper.enc.v}

# base top, ADV01 wrapper, clocks and constraints
add_files {./rtl/hdmi_mm_player.v}
add_files {./rtl/hdmi_adv01_subtitle.v}
add_files {./al_ip/video_pll.v}
add_files {./al_ip/sys_pll.v}
add_files {./al_ip/PLL_HDMI_AUDIO.v}
add_files {./constr/hdmi_adv01_subtitle.sdc}
add_files {./constr/hdmi_mm_player.adc}

close_project
puts "Created $project_dir/hdmi_adv01_subtitle.al"
