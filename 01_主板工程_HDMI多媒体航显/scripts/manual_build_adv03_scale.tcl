# Headless build for ADV03 (ADV01 overlays + ADV02 fade + picture scaling).
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv03_scale.al

set_param place effort high
set_param route effort high

# The FAT32 state decode path is the critical one in this design, and its
# slack depends on the placement seed.  The seed is taken from the
# ADV03_SEED environment variable so a run can be retried without editing
# the script.
if {[info exists ::env(ADV03_SEED)]} {
    set place_seed $::env(ADV03_SEED)
} else {
    set place_seed 1
}
puts "ADV03 SCALE: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv03_scale
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv03_scale.sdc
export_db hdmi_adv03_scale_elaborate.db
puts "ADV03 SCALE: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv03_scale_rtl.area
export_db hdmi_adv03_scale_rtl.db
puts "ADV03 SCALE: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv03_scale_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv03_scale_gate.ts
report_timing_summary -file hdmi_adv03_scale_gate.timing
export_db hdmi_adv03_scale_gate.db
puts "ADV03 SCALE: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv03_scale_place.qor
report_timing_summary -file hdmi_adv03_scale_place.timing
report_area -io_info -file hdmi_adv03_scale_phy.area
export_db hdmi_adv03_scale_place.db
puts "ADV03 SCALE: place done"

commit_param -step route
route
update_timing -mode final
report_qor -step route -file hdmi_adv03_scale_route.qor
report_timing_status -file hdmi_adv03_scale_phy.ts
report_timing_summary -file hdmi_adv03_scale_final.timing
report_timing_exception -file hdmi_adv03_scale_exception.timing
export_db hdmi_adv03_scale_pr.db
puts "ADV03 SCALE: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv03_scale.bit
export_bitgen_param -file .bitgen_adv03_scale_param.f
puts "ADV03 SCALE: bitgen done"
