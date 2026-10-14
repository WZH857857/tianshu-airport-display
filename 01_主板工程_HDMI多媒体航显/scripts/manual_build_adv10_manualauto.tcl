# Rebuild the advanced player with switch-selected picture advance:
# SW1 down = manual, KEY1 advances one picture; SW1 up = automatic, the
# resident pictures rotate every six seconds.  The audio, fade, scale and
# overlay paths are unchanged from ADV09.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV10_SEED)]} {
    set place_seed $::env(ADV10_SEED)
} else {
    set place_seed 1
}
puts "ADV10 MANUAL/AUTO: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv04_voice.sdc
export_db hdmi_adv10_manualauto_elaborate.db
puts "ADV10 MANUAL/AUTO: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv10_manualauto_rtl.area
export_db hdmi_adv10_manualauto_rtl.db
puts "ADV10 MANUAL/AUTO: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv10_manualauto_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv10_manualauto_gate.ts
report_timing_summary -file hdmi_adv10_manualauto_gate.timing
export_db hdmi_adv10_manualauto_gate.db
puts "ADV10 MANUAL/AUTO: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv10_manualauto_place.qor
report_timing_summary -file hdmi_adv10_manualauto_place.timing
report_area -io_info -file hdmi_adv10_manualauto_phy.area
export_db hdmi_adv10_manualauto_place.db
puts "ADV10 MANUAL/AUTO: place done"

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv10_manualauto_route.qor
report_timing_status -file hdmi_adv10_manualauto_phy.ts
report_timing_summary -file hdmi_adv10_manualauto_final.timing
report_timing_exception -file hdmi_adv10_manualauto_exception.timing
export_db hdmi_adv10_manualauto_pr.db
puts "ADV10 MANUAL/AUTO: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv10_manualauto.bit
export_bitgen_param -file .bitgen_adv10_manualauto_param.f
puts "ADV10 MANUAL/AUTO: bitgen done"
