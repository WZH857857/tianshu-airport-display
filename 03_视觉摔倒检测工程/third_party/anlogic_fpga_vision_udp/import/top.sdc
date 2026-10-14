
#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************
create_clock -name {clk} -period 20.000 -waveform {0.000 10.000}
create_clock -name {phy1_rgmii_rx_clk} -period 8.000 -waveform {0.000 4.000} [get_ports {phy1_rgmii_rx_clk}]
#create_generated_clock -name sd_card_clk -source [get_ports {clk}] -master_clock {clk} -phase 0 -multiply_by 2 [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
#create_generated_clock -name ext_mem_clk -source [get_ports {clk}] -master_clock {clk} -phase 270 -multiply_by 2 [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
#create_generated_clock -name flash_sys_clk -source [get_ports {clk}] -master_clock {clk} -phase 0 -multiply_by 2 [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
#create_generated_clock -name video_clk -source [get_ports {clk}] -master_clock {clk} -phase 0 -multiply_by 2 [get_pins {video_pll_m0/pll_inst.clkc[0]}]
#derive_clocks -instance [get_cells {sys_pll_m0/pll_inst u_time_top/pll_core/pll_inst }]
rename_clock -name {sd_card_clk} [get_clocks {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]

derive_clocks
rename_clock -name {pll_inst_125M_0} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[0]}]
rename_clock -name {pll_inst_125M_1} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[1]}]
#rename_clock -name {pll_inst_12p5M} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[2]}]
rename_clock -name {pll_inst_25M} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[3]}]

create_generated_clock -name {udp_clk_125m} -source [get_pins {u_clk_gen/u_pll_0/pll_inst.clkc[1]}] -master_clock {pll_inst_125M_1} -divide_by 1.000 -phase 0.000 -add [get_nets {udp_clk}]
#create_generated_clock -name {udp_clk_12p5m} -add -source [get_pins {u_clk_gen/u_pll_0/pll_inst.clkc[2]}] -master_clock {pll_inst_12p5M} -divide_by 1.000 [get_nets {udp_clk}]
#create_generated_clock -name {udp_clk_1p25m} -add -source [get_pins {u_clk_gen/u_pll_0/pll_inst.clkc[2]}] -master_clock {pll_inst_12p5M} -divide_by 10.000 [get_nets {udp_clk}]
set_clock_groups -exclusive -group [get_clocks {udp_clk_125m}]
#set_clock_groups -exclusive -group [get_clocks {udp_clk_12p5m}]
#set_clock_groups -exclusive -group [get_clocks {udp_clk_1p25m}]


set_clock_groups -exclusive -group [get_clocks {phy1_rgmii_rx_clk}]
#**************************************************************
# Set False Path
#**************************************************************

#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************


#**************************************************************
# Set Input Transition
#**************************************************************
create_clock -name {cam_pclk} -period 40.000 -waveform {0.000 20.000} [get_ports {cam_pclk}]
