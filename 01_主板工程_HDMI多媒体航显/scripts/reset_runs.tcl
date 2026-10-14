# Reset the synthesis and physical design runs so a fresh build can start.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

open_project ./hdmi_mm_player.al
reset_runs syn_1
reset_runs phy_1
close_project
puts "Runs reset."
