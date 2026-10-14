set script_dir [file dirname [file normalize [info script]]]
cd [file normalize [file join $script_dir ..]]
import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_mm_player.al
puts "===== place parameters ====="
report_param -stage place -detail -all
puts "===== route parameters ====="
report_param -stage route -detail -all
puts "===== seed help ====="
set_seed_param -help
close_project
