# Rebuild the advanced player with the fire emergency mode.
#
# ADV21 keeps the complete ADV20 feature set.  New is the fire emergency:
# the host link command 0x23 mirrors the latched flame alarm of the E104
# board into uart_ctrl, ui_overlay paints a full screen evacuation board
# while the flag is set, and emergency_alert drives the on-board buzzer
# (H11) and the eight digit display through the newly assigned pins.
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

if {[info exists ::env(ADV21_SEED)]} {
    set place_seed $::env(ADV21_SEED)
} else {
    set place_seed 4
}
# Output prefix, so several place seeds can be built side by side without
# overwriting each other's reports.  Default keeps the original file names.
if {[info exists ::env(ADV21_TAG)]} {
    set adv21_tag $::env(ADV21_TAG)
} else {
    set adv21_tag "hdmi_adv21_emergency"
}
puts "ADV21 fire emergency: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db ${adv21_tag}_elaborate.db
puts "ADV21 fire emergency: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file ${adv21_tag}_rtl.area
export_db ${adv21_tag}_rtl.db
puts "ADV21 fire emergency: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv21_emergency_gate.area
legalize_phy_inst
update_timing
report_timing_status -file ${adv21_tag}_gate.ts
report_timing_summary -file ${adv21_tag}_gate.timing
export_db ${adv21_tag}_gate.db
puts "ADV21 fire emergency: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file ${adv21_tag}_place.qor
report_timing_summary -file ${adv21_tag}_place.timing
report_area -io_info -file ${adv21_tag}_phy.area
export_db ${adv21_tag}_place.db
puts "ADV21 fire emergency: place done"

if {[info exists ::env(ADV21_STAGE)] && $::env(ADV21_STAGE) eq "place"} {
    puts "ADV21 fire emergency: ADV21_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
set_param route fix_hold on
fix_hold
update_timing -mode final
report_qor -step route -file ${adv21_tag}_route.qor
report_timing_status -file ${adv21_tag}_phy.ts
report_timing_summary -file ${adv21_tag}_final.timing
report_timing_exception -file ${adv21_tag}_exception.timing
export_db ${adv21_tag}_pr.db
puts "ADV21 fire emergency: route done"

commit_param -step bitgen
bitgen -bit ${adv21_tag}.bit
export_bitgen_param -file .bitgen_${adv21_tag}_param.f
puts "ADV21 fire emergency: bitgen done"
