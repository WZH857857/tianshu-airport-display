set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_core_probe.al

set_param place effort high
set_param route effort high
set_param place seed 1

commit_param -step design
elaborate -top hdmi_core_probe
read_adc ./constr/hdmi_core_probe.adc
clear_sdc -user
read_sdc ./constr/hdmi_core_probe.sdc

commit_param -step rtl
optimize_rtl

commit_param -step gate
optimize_gate
legalize_phy_inst
update_timing

commit_param -step place
place
update_timing -mode manhattan

commit_param -step route
route
update_timing -mode final
report_qor -step route -file hdmi_core_probe_route.qor
report_timing_summary -file hdmi_core_probe_final.timing

commit_param -step bitgen
bitgen -bit hdmi_core_probe.bit
export_bitgen_param -file .bitgen_core_probe_param.f
puts "HDMI CORE PROBE: done"
