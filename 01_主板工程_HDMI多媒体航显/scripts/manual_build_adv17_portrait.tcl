# Rebuild the advanced player with the SW2 landscape/portrait switch.
#
# ADV17 keeps the ADV14 HDMI A+B output, the flight board, the voice stream
# and the complete advanced feature set.  It changes only the display-side
# orientation mapping:
#   SW2 down = normal landscape
#   SW2 up   = portrait mapping for a monitor physically rotated clockwise
#
# The project and top name stay hdmi_adv04_voice because TD rewrites the .al
# file on every build and the existing project is the one that already
# carries the hand-added source entries.
#
# The design is close to the EG4S20 slice limit, so the placement seed is the
# main timing knob.  Override it with ADV17_SEED when a seed fails timing.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV17_SEED)]} {
    set place_seed $::env(ADV17_SEED)
} else {
    set place_seed 4
}
puts "ADV17 SW2 PORTRAIT: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv17_portrait_elaborate.db
puts "ADV17 SW2 PORTRAIT: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv17_portrait_rtl.area
export_db hdmi_adv17_portrait_rtl.db
puts "ADV17 SW2 PORTRAIT: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv17_portrait_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv17_portrait_gate.ts
report_timing_summary -file hdmi_adv17_portrait_gate.timing
export_db hdmi_adv17_portrait_gate.db
puts "ADV17 SW2 PORTRAIT: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv17_portrait_place.qor
report_timing_summary -file hdmi_adv17_portrait_place.timing
report_area -io_info -file hdmi_adv17_portrait_phy.area
export_db hdmi_adv17_portrait_place.db
puts "ADV17 SW2 PORTRAIT: place done"

if {[info exists ::env(ADV17_STAGE)] && $::env(ADV17_STAGE) eq "place"} {
    puts "ADV17 SW2 PORTRAIT: ADV17_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv17_portrait_route.qor
report_timing_status -file hdmi_adv17_portrait_phy.ts
report_timing_summary -file hdmi_adv17_portrait_final.timing
report_timing_exception -file hdmi_adv17_portrait_exception.timing
export_db hdmi_adv17_portrait_pr.db
puts "ADV17 SW2 PORTRAIT: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv17_portrait.bit
export_bitgen_param -file .bitgen_adv17_portrait_param.f
puts "ADV17 SW2 PORTRAIT: bitgen done"
