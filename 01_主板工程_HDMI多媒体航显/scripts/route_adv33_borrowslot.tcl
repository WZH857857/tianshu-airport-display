# Route an already-placed adv33 database and generate a fresh bitstream.
# This avoids repeating synthesis/placement while trying routing strategies.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

if {[info exists ::env(ADV33_PLACE_DB)]} {
    set place_db $::env(ADV33_PLACE_DB)
} else {
    set place_db "adv33_borrowslot_s1_place.db"
}
if {[info exists ::env(ADV33_TAG)]} {
    set tag $::env(ADV33_TAG)
} else {
    set tag "adv33_borrowslot_routed"
}
if {[info exists ::env(ADV33_ROUTE_EFFORT)]} {
    set route_effort $::env(ADV33_ROUTE_EFFORT)
} else {
    set route_effort "routability"
}

puts "ADV33 route-only: place_db=$place_db tag=$tag effort=$route_effort"
import_db $place_db
set_param route effort $route_effort
commit_param -step route
route

set_param route fix_hold on
fix_hold
update_timing -mode final
report_qor -step route -file ${tag}_route.qor
report_timing_status -file ${tag}_phy.ts
report_timing_summary -file ${tag}_final.timing
report_timing_exception -file ${tag}_exception.timing
export_db ${tag}_pr.db
puts "ADV33 route-only: route done"

commit_param -step bitgen
bitgen -bit ${tag}.bit
export_bitgen_param -file .bitgen_${tag}_param.f
puts "ADV33 route-only: bitgen done"
