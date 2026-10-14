# Run synthesis, place-and-route, timing analysis, and bit generation.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

open_project ./hdmi_mm_player.al

# make the run reproducible: clear any previous result before relaunching
catch { reset_runs syn_1 }
catch { reset_runs phy_1 }

launch_runs syn_1
wait_run syn_1 -quiet

launch_runs phy_1
wait_run phy_1 -quiet

save_best_bits
close_project
puts "Build finished. See $project_dir/hdmi_mm_player_Runs/best_result"
