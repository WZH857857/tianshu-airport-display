# Read and elaborate the isolated audio project without place and route.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_audio_release.al
commit_param -step design
elaborate -top hdmi_mm_player
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_mm_player.sdc
puts "AUDIO RELEASE CHECK: elaborate and constraints read"
close_project
