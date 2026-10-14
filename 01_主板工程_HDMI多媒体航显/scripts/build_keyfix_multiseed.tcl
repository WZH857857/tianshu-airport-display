set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_keyfix_retime.al

set_param place effort high
set_param route effort high
set_seed_param -num 4 -init 1 -step 1 -jobs 1 -save 1

puts "MULTISEED BUILD: parameters configured"
report_seed_param

catch { reset_runs syn_1 }
catch { reset_runs phy_1 }

launch_runs syn_1
wait_run syn_1 -quiet
puts "MULTISEED BUILD: synthesis done"

launch_runs phy_1 -jobs 1
wait_run phy_1 -quiet
puts "MULTISEED BUILD: physical design done"

save_best_bits
puts "MULTISEED BUILD: best bits saved"

close_project
puts "MULTISEED BUILD: done"
