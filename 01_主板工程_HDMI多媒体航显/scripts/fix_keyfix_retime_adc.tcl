set script_dir [file dirname [file normalize [info script]]]
cd [file normalize [file join $script_dir ..]]
open_project ./hdmi_keyfix_retime.al
catch { remove_source ./constr/hdmi_mm_player.adc }
catch { remove_source ./constr/hdmi_mm_player_retime.adc }
add_files {./constr/hdmi_mm_player_retime.adc}
close_project
puts "RETIME PROJECT: ADC updated"
