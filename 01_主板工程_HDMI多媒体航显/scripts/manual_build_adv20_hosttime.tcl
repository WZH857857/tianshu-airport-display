# Rebuild the advanced player with the host-driven wall clock.
#
# ADV20 keeps the complete ADV19 feature set.  The overlay now treats the
# upper-computer TIME command as the authoritative clock source, keeps a
# DS1307 readback only as a fallback for the case where the host never spoke,
# and free runs the seconds from the 25 MHz pixel clock so the panel keeps
# counting after the serial link drops.
#
# The project and top name stay hdmi_adv04_voice because TD rewrites the .al
# file on every build and that project already carries the hand-added sources.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV20_SEED)]} {
    set place_seed $::env(ADV20_SEED)
} else {
    set place_seed 4
}
puts "ADV20 host clock: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv20_hosttime_elaborate.db
puts "ADV20 host clock: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv20_hosttime_rtl.area
export_db hdmi_adv20_hosttime_rtl.db
puts "ADV20 host clock: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv20_hosttime_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv20_hosttime_gate.ts
report_timing_summary -file hdmi_adv20_hosttime_gate.timing
export_db hdmi_adv20_hosttime_gate.db
puts "ADV20 host clock: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv20_hosttime_place.qor
report_timing_summary -file hdmi_adv20_hosttime_place.timing
report_area -io_info -file hdmi_adv20_hosttime_phy.area
export_db hdmi_adv20_hosttime_place.db
puts "ADV20 host clock: place done"

if {[info exists ::env(ADV20_STAGE)] && $::env(ADV20_STAGE) eq "place"} {
    puts "ADV20 host clock: ADV20_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
set_param route fix_hold on
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv20_hosttime_route.qor
report_timing_status -file hdmi_adv20_hosttime_phy.ts
report_timing_summary -file hdmi_adv20_hosttime_final.timing
report_timing_exception -file hdmi_adv20_hosttime_exception.timing
export_db hdmi_adv20_hosttime_pr.db
puts "ADV20 host clock: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv20_hosttime.bit
export_bitgen_param -file .bitgen_adv20_hosttime_param.f
puts "ADV20 host clock: bitgen done"
