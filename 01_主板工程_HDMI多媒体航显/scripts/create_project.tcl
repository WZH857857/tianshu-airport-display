# Create the Tang Dynasty project from the checked-in source tree.
#
# Every file the design needs is listed explicitly, in the order the modules
# have to be readable, so nothing depends on the TD auto search.
#
# Usage:  td_commands_prompt.exe  scripts/create_project.tcl

set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

create_project hdmi_mm_player . -family EG4 -device EG4S20BG256
open_project ./hdmi_mm_player.al
top_module hdmi_mm_player

# ---- design ---------------------------------------------------------------
add_files {./rtl/video_timing_640x480.v}
add_files {./rtl/button_debounce.v}
add_files {./rtl/async_fifo.v}
add_files {./rtl/pattern_pixel.v}
add_files {./rtl/diag_overlay.v}

# ---- vendor SDRAM controller, taken verbatim from lab_ex5_i2s ------------
# The hand written controller needed a power up latency search that never
# completed on hardware.  These encrypted sources are the controller the
# vendor reference design uses on this exact device.
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

# ---- HDMI 1.4b transmitter, audio included ---------------------------------
# The official 2026 lab_ex5_i2s core.  Unlike the DEMO 31 DVI encoder it
# builds the data islands that carry the audio sample packets and the ACR
# packets, which is the only way to get sound out of the HDMI socket.
#
# The DEMO 31 chain (rtl/enc_file/*.enc.vhd plus rtl/hdmi_tx.vhd) is NOT part
# of this build.  It stays on disk and in
# scripts/create_project.before_audio_*.tcl, that is the project the video
# only fallback bitstream is built from.
add_files {./rtl/audio/hdmi_audio_tone_i2s_64fs.v}
add_files {./rtl/audio/video_rgb_to_axis_640x480.v}
add_files {./rtl/audio/edid_retry.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/I2S_receiver.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/audio_arc_calculate.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/hdmi_phy_warpper.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/lane_lvds_10_1.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/hdmi1.4b_transmitter_core/hdmi_1_4b_transmitter_core_wrapper.enc.v}

# ---- top ------------------------------------------------------------------
add_files {./rtl/hdmi_mm_player.v}

# ---- IP and constraints ---------------------------------------------------
# video_pll.v is added directly: loading the .ipc from a script makes TD look
# for its generated file with a path that does not resolve (PRJ-7002).
# Use the official lab_ex5_i2s PLL, not the DEMO31 clk_wize output order.
add_files {./al_ip/video_pll.v}
add_files {./al_ip/sys_pll.v}
add_files {./al_ip/PLL_HDMI_AUDIO.v}
add_files {./constr/hdmi_mm_player.sdc}
add_files {./constr/hdmi_mm_player.adc}

close_project
puts "Created $project_dir/hdmi_mm_player.al"
