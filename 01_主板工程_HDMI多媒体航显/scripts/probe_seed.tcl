set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_keyfix_retime.al

puts "===== seed parameters before ====="
report_seed_param

set_seed_param -num 4 -init 1 -step 1 -jobs 1 -save 1

puts "===== seed parameters after ====="
report_seed_param
report_param -stage place -detail -all

close_project
