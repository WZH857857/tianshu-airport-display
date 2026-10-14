# Add the official lab_ex5_i2s video PLL to the existing audio release
# project without recreating the project file.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_audio_release.al
add_files {./al_ip/video_pll.v}
close_project
puts "Added al_ip/video_pll.v to hdmi_audio_release.al"
