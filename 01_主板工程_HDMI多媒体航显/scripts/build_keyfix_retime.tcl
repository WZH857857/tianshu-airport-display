set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_keyfix_retime.al

# Use the tool's high-effort placement and routing for this smaller design.
set_param place effort high
set_param route effort high
set_param place seed 1

commit_param -step design
elaborate -top hdmi_mm_player
read_adc ./constr/hdmi_mm_player_retime.adc
clear_sdc -user
read_sdc ./constr/hdmi_mm_player_retime.sdc
export_db hdmi_keyfix_retime_elaborate.db
puts "RETIME BUILD: elaborate done"

commit_param -step rtl
optimize_rtl
report_area -file hdmi_keyfix_retime_rtl.area
export_db hdmi_keyfix_retime_rtl.db
puts "RETIME BUILD: optimize_rtl done"

commit_param -step gate
optimize_gate -packarea hdmi_keyfix_retime_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_keyfix_retime_gate.ts
report_timing_summary -file hdmi_keyfix_retime_gate.timing
export_db hdmi_keyfix_retime_gate.db
puts "RETIME BUILD: optimize_gate done"

commit_param -step place
place
update_timing -mode manhattan
report_qor -step place -file hdmi_keyfix_retime_place.qor
report_timing_summary -file hdmi_keyfix_retime_place.timing
export_db hdmi_keyfix_retime_place.db
puts "RETIME BUILD: place done"

commit_param -step route
route
update_timing -mode final
report_qor -step route -file hdmi_keyfix_retime_route.qor
report_timing_status -file hdmi_keyfix_retime_phy.ts
report_timing_summary -file hdmi_keyfix_retime_final.timing
report_timing_exception -file hdmi_keyfix_retime_exception.timing
export_db hdmi_keyfix_retime_pr.db
puts "RETIME BUILD: route done"

commit_param -step bitgen
bitgen -bit hdmi_keyfix_retime.bit
export_bitgen_param -file .bitgen_retime_param.f
puts "RETIME BUILD: bitgen done"
