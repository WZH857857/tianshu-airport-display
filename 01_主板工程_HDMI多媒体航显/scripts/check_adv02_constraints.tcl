# Fast ADV02 constraint check: elaborate the wrapper, load its hierarchy-aware
# SDC, and print the clocks TD actually creates.

set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_adv02_fade.al

commit_param -step design
elaborate -top hdmi_adv02_fade
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_adv02_fade.sdc

# Both update_timing and report_timing_status crash this TD build before place,
# so the check only reads the constraints and dumps the clocks they created.
set fp [open hdmi_adv02_fade_constraint_check.txt w]
puts $fp "ADV02 CONSTRAINT CHECK: clocks = [get_clocks *]"
foreach clk [get_clocks *] {
    puts $fp "  [get_clock_info -name $clk] period = [get_clock_info -period $clk]"
}
close $fp
puts "ADV02 CONSTRAINT CHECK: done"
close_project
