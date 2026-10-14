# Rebuild the advanced player with the sticky underflow diagnostic (ADV12).
#
# Everything from ADV11 is unchanged.  The only RTL change is inside
# frame_store.v: a sticky "display read FIFO underflow was seen since reset"
# bit is exported through the otherwise unused cal_fail port, and
# hdmi_mm_player.v shows it on LED4 (MSB of the LED word).  A healthy picture
# shows 0010; after the first underflow the LEDs read 1010 until reset.
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

if {[info exists ::env(ADV12_SEED)]} {
    set place_seed $::env(ADV12_SEED)
} else {
    set place_seed 1
}
puts "ADV12 UNDERFLOW DIAG: place seed = $place_seed"
set_param place seed $place_seed

commit_param -step design
elaborate -top hdmi_adv04_voice
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv11_uart.sdc
export_db hdmi_adv12_ufdiag_elaborate.db
puts "ADV12 UNDERFLOW DIAG: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_adv12_ufdiag_rtl.area
export_db hdmi_adv12_ufdiag_rtl.db
puts "ADV12 UNDERFLOW DIAG: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_adv12_ufdiag_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_adv12_ufdiag_gate.ts
report_timing_summary -file hdmi_adv12_ufdiag_gate.timing
export_db hdmi_adv12_ufdiag_gate.db
puts "ADV12 UNDERFLOW DIAG: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_adv12_ufdiag_place.qor
report_timing_summary -file hdmi_adv12_ufdiag_place.timing
report_area -io_info -file hdmi_adv12_ufdiag_phy.area
export_db hdmi_adv12_ufdiag_place.db
puts "ADV12 UNDERFLOW DIAG: place done"

commit_param -step route
route
fix_hold
update_timing -mode final
report_qor -step route -file hdmi_adv12_ufdiag_route.qor
report_timing_status -file hdmi_adv12_ufdiag_phy.ts
report_timing_summary -file hdmi_adv12_ufdiag_final.timing
report_timing_exception -file hdmi_adv12_ufdiag_exception.timing
export_db hdmi_adv12_ufdiag_pr.db
puts "ADV12 UNDERFLOW DIAG: route done"

commit_param -step bitgen
bitgen -bit hdmi_adv12_ufdiag.bit
export_bitgen_param -file .bitgen_adv12_ufdiag_param.f
puts "ADV12 UNDERFLOW DIAG: bitgen done"
