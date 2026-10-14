# Quick structural check for the HDMI A + B wrapper.
#
# Elaborates the current RTL and reads the real pin file, then stops.  This
# catches a wrong port name, a missing pin assignment or an unusable LVDS33
# location long before a full place and route run is paid for.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al

set_param place effort high
set_param route effort high

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
puts "ADV14 CHECK: elaborate and pin assignment done"

commit_param -step rtl
optimize_rtl
report_area -file adv14_hdmib_rtl.area
puts "ADV14 CHECK: optimize_rtl done"
