# Fast constraint check: parse the ADC and SDC after elaboration and print
# the clocks TD actually created.  This stops before synthesis.

set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_mm_player.al

commit_param -step design
elaborate -top hdmi_mm_player
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_mm_player.sdc

update_timing
puts "CONSTRAINT CHECK: clocks = [get_clocks *]"
report_timing_status -file constraint_check.ts
report_timing_summary -file constraint_check.rpt
puts "CONSTRAINT CHECK: done"
