# Rebuild the advanced player with the fire-voice handoff fix.
#
# The alarm transition aborts the card-side WAV parser.  The audio-side
# player must be allowed to drain the old FIFO contents before the next
# announcement starts, otherwise the two voices overlap at the switch.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
if {[info exists ::env(ADV28_ROUTE_EFFORT)]} {
    set_param route effort $::env(ADV28_ROUTE_EFFORT)
} else {
    set_param route effort high
}

if {[info exists ::env(ADV28_SEED)]} {
    set place_seed $::env(ADV28_SEED)
} else {
    set place_seed 1
}
if {[info exists ::env(ADV28_TAG)]} {
    set adv28_tag $::env(ADV28_TAG)
} else {
    set adv28_tag "adv28_firevoice2"
}
puts "ADV28 fire voice handoff: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db ${adv28_tag}_elaborate.db
puts "ADV28 fire voice handoff: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file ${adv28_tag}_rtl.area
export_db ${adv28_tag}_rtl.db
puts "ADV28 fire voice handoff: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea ${adv28_tag}_gate.area
legalize_phy_inst
update_timing
report_timing_status -file ${adv28_tag}_gate.ts
report_timing_summary -file ${adv28_tag}_gate.timing
export_db ${adv28_tag}_gate.db
puts "ADV28 fire voice handoff: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file ${adv28_tag}_place.qor
report_timing_summary -file ${adv28_tag}_place.timing
report_area -io_info -file ${adv28_tag}_phy.area
export_db ${adv28_tag}_place.db
puts "ADV28 fire voice handoff: place done"

if {[info exists ::env(ADV28_STAGE)] && $::env(ADV28_STAGE) eq "place"} {
    puts "ADV28 fire voice handoff: ADV28_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
set_param route fix_hold on
fix_hold
update_timing -mode final
report_qor -step route -file ${adv28_tag}_route.qor
report_timing_status -file ${adv28_tag}_phy.ts
report_timing_summary -file ${adv28_tag}_final.timing
report_timing_exception -file ${adv28_tag}_exception.timing
export_db ${adv28_tag}_pr.db
puts "ADV28 fire voice handoff: route done"

commit_param -step bitgen
bitgen -bit ${adv28_tag}.bit
export_bitgen_param -file .bitgen_${adv28_tag}_param.f
puts "ADV28 fire voice handoff: bitgen done"
