# Dump every set_param knob TD knows about, then quit.
import_device eagle_s20.db -package EG4S20NG88
report_param -file "E:/FPGA guo sai/fall_detection/fpga/td/out_board_multi/param_dump.txt"
puts "[param] dumped"
