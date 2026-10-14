# Headless audio release build.  This uses its own .al file so the verified
# video-only project open in the TD GUI is not modified.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_audio_release.al

set_param place effort high
set_param route effort high
set_param place seed 1

commit_param -step design
elaborate -top hdmi_mm_player
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_mm_player.sdc
export_db hdmi_audio_release_elaborate.db
puts "AUDIO RELEASE: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_audio_release_rtl.area
export_db hdmi_audio_release_rtl.db
puts "AUDIO RELEASE: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_audio_release_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_audio_release_gate.ts
report_timing_summary -file hdmi_audio_release_gate.timing
export_db hdmi_audio_release_gate.db
puts "AUDIO RELEASE: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_audio_release_place.qor
report_timing_summary -file hdmi_audio_release_place.timing
report_area -io_info -file hdmi_audio_release_phy.area
export_db hdmi_audio_release_place.db
puts "AUDIO RELEASE: place done"

commit_param -step route
route
update_timing -mode final
report_qor -step route -file hdmi_audio_release_route.qor
report_timing_status -file hdmi_audio_release_phy.ts
report_timing_summary -file hdmi_audio_release_final.timing
report_timing_exception -file hdmi_audio_release_exception.timing
export_db hdmi_audio_release_pr.db
puts "AUDIO RELEASE: route done"

commit_param -step bitgen
bitgen -bit hdmi_advanced_release_ADV_V1_20260926.bit
export_bitgen_param -file .bitgen_audio_release_param.f
puts "AUDIO RELEASE: bitgen done"
