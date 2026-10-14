set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

create_project hdmi_core_probe . -family EG4 -device EG4S20BG256
open_project ./hdmi_core_probe.al
top_module hdmi_core_probe

add_files {./rtl/hdmi_core_probe.v}
add_files {./rtl/audio/hdmi_audio_tone_i2s_64fs.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/I2S_receiver.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/audio_arc_calculate.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/hdmi_phy_warpper.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/lane_lvds_10_1.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/startup_pulse.v}
add_files {./vendor/lab_ex5_i2s/src/user_source/hdl_source/hdmi1.4b_transmitter_core/hdmi_1_4b_transmitter_core_wrapper.enc.v}
add_files {./al_ip/video_pll.v}
add_files {./al_ip/PLL_HDMI_AUDIO.v}
add_files {./constr/hdmi_core_probe.adc}
add_files {./constr/hdmi_core_probe.sdc}

close_project
puts "Created $project_dir/hdmi_core_probe.al"
