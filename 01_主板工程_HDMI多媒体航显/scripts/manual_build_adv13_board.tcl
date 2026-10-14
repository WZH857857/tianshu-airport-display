# Rebuild the advanced player with the real-time flight information board.
#
# ADV13 adds rtl/ui/flight_board.v and the A5 5A protocol commands 0x20 BOARD
# and 0x21 BVIS.  The project and top name stay hdmi_adv04_voice because TD
# rewrites the .al file on every build and the existing project is the one that
# already carries the hand-added source entries.
#
# The design is close to the EG4S20 slice limit, so the placement seed is the
# main timing knob.  Override it with the ADV13_SEED environment variable when
# a seed fails timing.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV13_SEED)]} {
    set place_seed $::env(ADV13_SEED)
} else {
    set place_seed 3
}
puts "ADV13 FLIGHT BOARD: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv13_board_elaborate.db
puts "ADV13 FLIGHT BOARD: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv13_board_rtl.area
export_db hdmi_adv13_board_rtl.db
puts "ADV13 FLIGHT BOARD: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv13_board_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv13_board_gate.ts
report_timing_summary -file hdmi_adv13_board_gate.timing
export_db hdmi_adv13_board_gate.db
puts "ADV13 FLIGHT BOARD: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv13_board_place.qor
report_timing_summary -file hdmi_adv13_board_place.timing
report_area -io_info -file hdmi_adv13_board_phy.area
export_db hdmi_adv13_board_place.db
puts "ADV13 FLIGHT BOARD: place done"

# Rapid seed screening: set ADV13_STAGE=place to stop after placement and
# inspect hdmi_adv13_board_place.qor without paying for a full route.
if {[info exists ::env(ADV13_STAGE)] && $::env(ADV13_STAGE) eq "place"} {
    puts "ADV13 FLIGHT BOARD: ADV13_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv13_board_route.qor
report_timing_status -file hdmi_adv13_board_phy.ts
report_timing_summary -file hdmi_adv13_board_final.timing
report_timing_exception -file hdmi_adv13_board_exception.timing
export_db hdmi_adv13_board_pr.db
puts "ADV13 FLIGHT BOARD: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv13_board.bit
export_bitgen_param -file .bitgen_adv13_board_param.f
puts "ADV13 FLIGHT BOARD: bitgen done"
