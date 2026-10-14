# Headless build for ADV04 (ADV02 overlay + fade, with the card voice).
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

# The streaming player, the resampler and the scaler are not listed in the
# project file: TD rewrites the .al on open and drops hand-added sources.  They
# reach the netlist through the `include block at the top of
# rtl/hdmi_mm_player.v instead, so nothing has to be patched here.
import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

# The FAT32 state decode path is the critical one in this design, and its
# slack depends on the placement seed, so the seed can be overridden from the
# environment without editing the script.
if {[info exists ::env(ADV04_SEED)]} {
    set place_seed $::env(ADV04_SEED)
} else {
    set place_seed 1
}
puts "ADV04 VOICE: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv04_voice.sdc
export_db hdmi_adv04_voice_elaborate.db
puts "ADV04 VOICE: elaborate done"

# Set ADV04_STOP_AFTER_ELABORATE=1 to check the source list and the black box
# report in about a minute instead of running the whole flow.
if {[info exists ::env(ADV04_STOP_AFTER_ELABORATE)]} {
    puts "ADV04 VOICE: stopping after elaborate"
    exit
}

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv04_voice_rtl.area
export_db hdmi_adv04_voice_rtl.db
puts "ADV04 VOICE: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv04_voice_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv04_voice_gate.ts
report_timing_summary -file hdmi_adv04_voice_gate.timing
export_db hdmi_adv04_voice_gate.db
puts "ADV04 VOICE: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv04_voice_place.qor
report_timing_summary -file hdmi_adv04_voice_place.timing
report_area -io_info -file hdmi_adv04_voice_phy.area
export_db hdmi_adv04_voice_place.db
puts "ADV04 VOICE: place done"

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv04_voice_route.qor
report_timing_status -file hdmi_adv04_voice_phy.ts
report_timing_summary -file hdmi_adv04_voice_final.timing
report_timing_exception -file hdmi_adv04_voice_exception.timing
export_db hdmi_adv04_voice_pr.db
puts "ADV04 VOICE: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv04_voice.bit
export_bitgen_param -file .bitgen_adv04_voice_param.f
puts "ADV04 VOICE: bitgen done"
