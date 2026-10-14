# Rebuild the advanced player with the real-time clock link.
#
# ADV18 keeps the ADV14 HDMI A+B output, the flight board, the voice stream
# and the complete advanced feature set.  It removes the unused SW2 portrait
# experiment and adds command 22 TIME: the host console loads hh/mm/ss BCD
# into the overlay, which then keeps the clock running locally.
#
# The project and top name stay hdmi_adv04_voice because TD rewrites the .al
# file on every build and the existing project is the one that already
# carries the hand-added source entries.
#
# The design is close to the EG4S20 slice limit, so the placement seed is the
# main timing knob.  Override it with ADV18_SEED when a seed fails timing.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV18_SEED)]} {
    set place_seed $::env(ADV18_SEED)
} else {
    set place_seed 4
}
puts "ADV18 REAL TIME: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv18_time_elaborate.db
puts "ADV18 REAL TIME: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv18_time_rtl.area
export_db hdmi_adv18_time_rtl.db
puts "ADV18 REAL TIME: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv18_time_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv18_time_gate.ts
report_timing_summary -file hdmi_adv18_time_gate.timing
export_db hdmi_adv18_time_gate.db
puts "ADV18 REAL TIME: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv18_time_place.qor
report_timing_summary -file hdmi_adv18_time_place.timing
report_area -io_info -file hdmi_adv18_time_phy.area
export_db hdmi_adv18_time_place.db
puts "ADV18 REAL TIME: place done"

if {[info exists ::env(ADV18_STAGE)] && $::env(ADV18_STAGE) eq "place"} {
    puts "ADV18 REAL TIME: ADV18_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv18_time_route.qor
report_timing_status -file hdmi_adv18_time_phy.ts
report_timing_summary -file hdmi_adv18_time_final.timing
report_timing_exception -file hdmi_adv18_time_exception.timing
export_db hdmi_adv18_time_pr.db
puts "ADV18 REAL TIME: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv18_time.bit
export_bitgen_param -file .bitgen_adv18_time_param.f
puts "ADV18 REAL TIME: bitgen done"
