# Rebuild the advanced player with the repeating card announcement
# (advanced item 2): the WAV on the card plays once, waits sixty seconds and
# starts again from the first sample.  This script only changes the voice
# scheduler, so the video, fade, scale, OTA and overlay logic is untouched.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV09_SEED)]} {
    set place_seed $::env(ADV09_SEED)
} else {
    set place_seed 1
}
puts "ADV09 VOICE REPEAT: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv04_voice.sdc
export_db hdmi_adv09_voicerepeat_elaborate.db
puts "ADV09 VOICE REPEAT: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv09_voicerepeat_rtl.area
export_db hdmi_adv09_voicerepeat_rtl.db
puts "ADV09 VOICE REPEAT: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv09_voicerepeat_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv09_voicerepeat_gate.ts
report_timing_summary -file hdmi_adv09_voicerepeat_gate.timing
export_db hdmi_adv09_voicerepeat_gate.db
puts "ADV09 VOICE REPEAT: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv09_voicerepeat_place.qor
report_timing_summary -file hdmi_adv09_voicerepeat_place.timing
report_area -io_info -file hdmi_adv09_voicerepeat_phy.area
export_db hdmi_adv09_voicerepeat_place.db
puts "ADV09 VOICE REPEAT: place done"

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv09_voicerepeat_route.qor
report_timing_status -file hdmi_adv09_voicerepeat_phy.ts
report_timing_summary -file hdmi_adv09_voicerepeat_final.timing
report_timing_exception -file hdmi_adv09_voicerepeat_exception.timing
export_db hdmi_adv09_voicerepeat_pr.db
puts "ADV09 VOICE REPEAT: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv09_voicerepeat.bit
export_bitgen_param -file .bitgen_adv09_voicerepeat_param.f
puts "ADV09 VOICE REPEAT: bitgen done"
