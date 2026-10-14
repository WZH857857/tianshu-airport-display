# Full build without the run manager: the headless run manager in TD 6.2.1
# writes a run.bat that points at a non existing executable name, so the
# flow below calls the same commands DefaultFlow.tcl would call.
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
cd $project_dir

# load the device library first, otherwise the macro library is empty and
# elaborate trips an internal assertion
import_device eagle_s20.db -package EG4S20BG256
open_project ./hdmi_mm_player.al

# read design ------------------------------------------------------------
commit_param -step design
elaborate -top hdmi_mm_player
read_adc ./constr/hdmi_mm_player.adc
clear_sdc -user
read_sdc ./constr/hdmi_mm_player.sdc
export_db hdmi_mm_player_elaborate.db
puts "MANUAL BUILD: elaborate done"

# rtl optimization -------------------------------------------------------
commit_param -step rtl
optimize_rtl
report_area -file hdmi_mm_player_rtl.area
export_db hdmi_mm_player_rtl.db
puts "MANUAL BUILD: optimize_rtl done"

# gate optimization ------------------------------------------------------
commit_param -step gate
optimize_gate -packarea hdmi_mm_player_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_mm_player_gate.ts
report_timing_summary -file hdmi_mm_player_gate.timing
export_db hdmi_mm_player_gate.db
puts "MANUAL BUILD: optimize_gate done"

# place ------------------------------------------------------------------
commit_param -step place
place
update_timing -mode manhattan
report_timing_summary -file hdmi_mm_player_place.timing
report_area -io_info -file hdmi_mm_player_phy.area
export_db hdmi_mm_player_place.db
puts "MANUAL BUILD: place done"

# route ------------------------------------------------------------------
commit_param -step route
route
update_timing -mode final
report_timing_status -file hdmi_mm_player_phy.ts
report_timing_summary -file hdmi_mm_player_pr.timing
report_timing_exception -file hdmi_mm_player_exception.timing
export_db hdmi_mm_player_pr.db
puts "MANUAL BUILD: route done"

# bit stream -------------------------------------------------------------
commit_param -step bitgen
bitgen -bit hdmi_mm_player.bit
export_bitgen_param -file .bitgen_param.f
puts "MANUAL BUILD: bitgen done"
