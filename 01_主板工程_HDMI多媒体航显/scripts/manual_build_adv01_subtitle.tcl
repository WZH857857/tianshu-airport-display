# Headless build for ADV01 (layer overlay + scrolling subtitle).
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv01_subtitle.al

set_param place effort high
set_param route effort high

# The FAT32 state decode path is the critical one in this design, and its
# slack depends on the placement seed.  The seed is taken from the
# ADV01_SEED environment variable so a run can be retried without editing
# the script; seed 1 keeps the previous, reproducible default.
if {[info exists ::env(ADV01_SEED)]} {
    set place_seed $::env(ADV01_SEED)
} else {
    set place_seed 1
}
puts "ADV01 SUBTITLE: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv01_subtitle
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv01_subtitle.sdc
export_db hdmi_adv01_subtitle_elaborate.db
puts "ADV01 SUBTITLE: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv01_subtitle_rtl.area
export_db hdmi_adv01_subtitle_rtl.db
puts "ADV01 SUBTITLE: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv01_subtitle_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv01_subtitle_gate.ts
report_timing_summary -file hdmi_adv01_subtitle_gate.timing
export_db hdmi_adv01_subtitle_gate.db
puts "ADV01 SUBTITLE: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv01_subtitle_place.qor
report_timing_summary -file hdmi_adv01_subtitle_place.timing
report_area -io_info -file hdmi_adv01_subtitle_phy.area
export_db hdmi_adv01_subtitle_place.db
puts "ADV01 SUBTITLE: place done"

commit_param -step route
route
update_timing -mode final
report_qor -step route -file hdmi_adv01_subtitle_route.qor
report_timing_status -file hdmi_adv01_subtitle_phy.ts
report_timing_summary -file hdmi_adv01_subtitle_final.timing
report_timing_exception -file hdmi_adv01_subtitle_exception.timing
export_db hdmi_adv01_subtitle_pr.db
puts "ADV01 SUBTITLE: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv01_subtitle.bit
export_bitgen_param -file .bitgen_adv01_subtitle_param.f
puts "ADV01 SUBTITLE: bitgen done"
