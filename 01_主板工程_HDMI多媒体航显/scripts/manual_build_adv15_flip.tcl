# Rebuild the advanced player with the SW2 picture-orientation switch.
#
# ADV15 keeps the ADV13/ADV14 HDMI A+B output and flight-board feature set.
# It adds the display-side vertical flip selected by board switch SW2:
#   SW2 down = normal, SW2 up = stored picture shown bottom-up.
#
# The project and top name stay hdmi_adv04_voice because TD rewrites the .al
# file on every build and the existing project is the one that already
# carries the hand-added source entries.
#
# The design is very close to the EG4S20 slice limit, so the placement seed
# is the main timing knob.  Override it with ADV15_SEED when a seed fails
# timing.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV15_SEED)]} {
    set place_seed $::env(ADV15_SEED)
} else {
    set place_seed 1
}
puts "ADV15 SW2 FLIP: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv15_flip_elaborate.db
puts "ADV15 SW2 FLIP: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv15_flip_rtl.area
export_db hdmi_adv15_flip_rtl.db
puts "ADV15 SW2 FLIP: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv15_flip_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv15_flip_gate.ts
report_timing_summary -file hdmi_adv15_flip_gate.timing
export_db hdmi_adv15_flip_gate.db
puts "ADV15 SW2 FLIP: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv15_flip_place.qor
report_timing_summary -file hdmi_adv15_flip_place.timing
report_area -io_info -file hdmi_adv15_flip_phy.area
export_db hdmi_adv15_flip_place.db
puts "ADV15 SW2 FLIP: place done"

# Rapid seed screening: set ADV15_STAGE=place to stop after placement and
# inspect hdmi_adv15_flip_place.qor without paying for a full route.
if {[info exists ::env(ADV15_STAGE)] && $::env(ADV15_STAGE) eq "place"} {
    puts "ADV15 SW2 FLIP: ADV15_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv15_flip_route.qor
report_timing_status -file hdmi_adv15_flip_phy.ts
report_timing_summary -file hdmi_adv15_flip_final.timing
report_timing_exception -file hdmi_adv15_flip_exception.timing
export_db hdmi_adv15_flip_pr.db
puts "ADV15 SW2 FLIP: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv15_flip.bit
export_bitgen_param -file .bitgen_adv15_flip_param.f
puts "ADV15 SW2 FLIP: bitgen done"
