# Create a separate project for the proven DVI/TMDS video path with the
# corrected KEY1 sequencer.  The audio-enabled top is deliberately excluded.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

create_project hdmi_keyfix_retime . -family EG4 -device EG4S20BG256
open_project ./hdmi_keyfix_retime.al
top_module hdmi_mm_player

add_files {./rtl/video_timing_640x480.v}
add_files {./rtl/button_debounce.v}
add_files {./rtl/async_fifo.v}
add_files {./rtl/pattern_pixel.v}
add_files {./rtl/diag_overlay.v}

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

add_files {./rtl/enc_file/SerializerN_1_lvds.enc.vhd}
add_files {./rtl/enc_file/SerializerN_1_lvds_dat.enc.vhd}
add_files {./rtl/enc_file/TMDSEncoder.enc.vhd}
add_files {./rtl/enc_file/DVITransmitter.enc.vhd}
add_files {./rtl/hdmi_tx.vhd}

add_files {./rtl/hdmi_mm_player.before_audio_20260925_134858.v}

add_files {./al_ip/video_pll.v}
add_files {./al_ip/sys_pll.v}
add_files {./constr/hdmi_mm_player_retime.sdc}
add_files {./constr/hdmi_mm_player_retime.adc}

close_project
puts "Created $project_dir/hdmi_keyfix_retime.al"
