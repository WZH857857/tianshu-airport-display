# Rebuild the advanced player with advanced item 5 audio visualisation
# enabled: the existing waveform plus the new eight-band filter-bank spectrum.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV08_SEED)]} {
    set place_seed $::env(ADV08_SEED)
} else {
    set place_seed 1
}
puts "ADV08 AUDIO VIS: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv04_voice.sdc
export_db hdmi_adv08_audiovis_elaborate.db
puts "ADV08 AUDIO VIS: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv08_audiovis_rtl.area
export_db hdmi_adv08_audiovis_rtl.db
puts "ADV08 AUDIO VIS: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv08_audiovis_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv08_audiovis_gate.ts
report_timing_summary -file hdmi_adv08_audiovis_gate.timing
export_db hdmi_adv08_audiovis_gate.db
puts "ADV08 AUDIO VIS: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv08_audiovis_place.qor
report_timing_summary -file hdmi_adv08_audiovis_place.timing
report_area -io_info -file hdmi_adv08_audiovis_phy.area
export_db hdmi_adv08_audiovis_place.db
puts "ADV08 AUDIO VIS: place done"

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv08_audiovis_route.qor
report_timing_status -file hdmi_adv08_audiovis_phy.ts
report_timing_summary -file hdmi_adv08_audiovis_final.timing
report_timing_exception -file hdmi_adv08_audiovis_exception.timing
export_db hdmi_adv08_audiovis_pr.db
puts "ADV08 AUDIO VIS: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv08_audiovis.bit
export_bitgen_param -file .bitgen_adv08_audiovis_param.f
puts "ADV08 AUDIO VIS: bitgen done"
