# Rebuild the advanced player with the scaler line buffer forced into BRAM.
# Kept separate from the earlier build so the failed high-utilization bitstreams
# remain available for comparison.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV07_SEED)]} {
    set place_seed $::env(ADV07_SEED)
} else {
    set place_seed 1
}
puts "ADV07 BRAM SCALE: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv04_voice.sdc
export_db hdmi_adv07_bramscale_elaborate.db
puts "ADV07 BRAM SCALE: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv07_bramscale_rtl.area
export_db hdmi_adv07_bramscale_rtl.db
puts "ADV07 BRAM SCALE: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv07_bramscale_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv07_bramscale_gate.ts
report_timing_summary -file hdmi_adv07_bramscale_gate.timing
export_db hdmi_adv07_bramscale_gate.db
puts "ADV07 BRAM SCALE: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv07_bramscale_place.qor
report_timing_summary -file hdmi_adv07_bramscale_place.timing
report_area -io_info -file hdmi_adv07_bramscale_phy.area
export_db hdmi_adv07_bramscale_place.db
puts "ADV07 BRAM SCALE: place done"

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv07_bramscale_route.qor
report_timing_status -file hdmi_adv07_bramscale_phy.ts
report_timing_summary -file hdmi_adv07_bramscale_final.timing
report_timing_exception -file hdmi_adv07_bramscale_exception.timing
export_db hdmi_adv07_bramscale_pr.db
puts "ADV07 BRAM SCALE: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv07_bramscale.bit
export_bitgen_param -file .bitgen_adv07_bramscale_param.f
puts "ADV07 BRAM SCALE: bitgen done"
