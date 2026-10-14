# Rebuild the advanced player with the DS1307 RTC wired directly to H15/H16.
#
# ADV19 keeps the complete ADV18 feature set, the UART TIME command and the
# E104 console untouched.  The new i2c_rtc block reads hh/mm/ss once a second
# on the HDMI board; the overlay gives that source priority and falls back to
# the UART value when the RTC is absent.
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

if {[info exists ::env(ADV19_SEED)]} {
    set place_seed $::env(ADV19_SEED)
} else {
    set place_seed 4
}
puts "ADV19 DS1307 RTC: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv19_rtc_elaborate.db
puts "ADV19 DS1307 RTC: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv19_rtc_rtl.area
export_db hdmi_adv19_rtc_rtl.db
puts "ADV19 DS1307 RTC: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv19_rtc_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv19_rtc_gate.ts
report_timing_summary -file hdmi_adv19_rtc_gate.timing
export_db hdmi_adv19_rtc_gate.db
puts "ADV19 DS1307 RTC: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv19_rtc_place.qor
report_timing_summary -file hdmi_adv19_rtc_place.timing
report_area -io_info -file hdmi_adv19_rtc_phy.area
export_db hdmi_adv19_rtc_place.db
puts "ADV19 DS1307 RTC: place done"

if {[info exists ::env(ADV19_STAGE)] && $::env(ADV19_STAGE) eq "place"} {
    puts "ADV19 DS1307 RTC: ADV19_STAGE=place, stopping after placement"
    return
}

commit_param -step route
route
set_param route fix_hold on
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv19_rtc_route.qor
report_timing_status -file hdmi_adv19_rtc_phy.ts
report_timing_summary -file hdmi_adv19_rtc_final.timing
report_timing_exception -file hdmi_adv19_rtc_exception.timing
export_db hdmi_adv19_rtc_pr.db
puts "ADV19 DS1307 RTC: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv19_rtc.bit
export_bitgen_param -file .bitgen_adv19_rtc_param.f
puts "ADV19 DS1307 RTC: bitgen done"
