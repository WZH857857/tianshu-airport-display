# Repair the small post-route hold violations without rerunning synthesis.
# The routed database is produced by manual_build_adv04_voice.tcl.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv04_voice.al
import_db ./hdmi_adv04_voice_pr.db

set_param route fix_hold on
fix_hold
update_timing -mode final

report_qor -step route -file hdmi_adv06_fixhold.qor
report_timing_status -file hdmi_adv06_fixhold.ts
report_timing_summary -file hdmi_adv06_fixhold.timing
export_db hdmi_adv06_fixhold.db
puts "ADV06 FIXHOLD: route database updated"

commit_param -step bitgen
bitgen -bit hdmi_adv06_fixhold.bit
puts "ADV06 FIXHOLD: bitgen done"
