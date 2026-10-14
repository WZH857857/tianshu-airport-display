# Rebuild the advanced player with the serial host link (ADV11).
#
# Everything from ADV10 is unchanged: manual / automatic picture advance on
# SW1, KEY1, the fade, the scaler, the card voice announcement and the OSD.
# What is new is the CH340 UART: six cached pictures can be selected from the
# PC, the scrolling message can be replaced, and the mode, brightness,
# contrast and OSD layout can be driven remotely.
#
# The project file and the top level keep the ADV04 names on purpose - TD
# rewrites hdmi_adv04_voice.al from its own source list on every build, so
# reusing the existing project avoids losing hand added source entries.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

if {[info exists ::env(ADV11_SEED)]} {
    set place_seed $::env(ADV11_SEED)
} else {
    set place_seed 2
}
puts "ADV11 UART HOST: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv11_uart_elaborate.db
puts "ADV11 UART HOST: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv11_uart_rtl.area
export_db hdmi_adv11_uart_rtl.db
puts "ADV11 UART HOST: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv11_uart_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv11_uart_gate.ts
report_timing_summary -file hdmi_adv11_uart_gate.timing
export_db hdmi_adv11_uart_gate.db
puts "ADV11 UART HOST: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv11_uart_place.qor
report_timing_summary -file hdmi_adv11_uart_place.timing
report_area -io_info -file hdmi_adv11_uart_phy.area
export_db hdmi_adv11_uart_place.db
puts "ADV11 UART HOST: place done"

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv11_uart_route.qor
report_timing_status -file hdmi_adv11_uart_phy.ts
report_timing_summary -file hdmi_adv11_uart_final.timing
report_timing_exception -file hdmi_adv11_uart_exception.timing
export_db hdmi_adv11_uart_pr.db
puts "ADV11 UART HOST: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv11_uart.bit
export_bitgen_param -file .bitgen_adv11_uart_param.f
puts "ADV11 UART HOST: bitgen done"
