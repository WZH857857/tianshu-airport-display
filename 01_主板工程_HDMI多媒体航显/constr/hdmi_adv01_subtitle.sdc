# ADV01 timing constraints.  The design uses a thin wrapper around the
# verified player, so the generated-clock nets live one hierarchy level below
# the top module.  The clock definitions themselves are unchanged from the
# base project; only the net references are qualified with u_player/.

create_clock -name sys_clk -period 20 -waveform {0 10} [get_ports {sys_clk}]

create_generated_clock -name {pixel_clk_5x} -source [get_ports {sys_clk}] \
    -master_clock {sys_clk} -multiply_by 2.500 \
    [get_nets {u_player/pixel_clk_5x}]
create_generated_clock -name {pixel_clk} -source [get_ports {sys_clk}] \
    -master_clock {sys_clk} -divide_by 2.000 \
    [get_nets {u_player/pixel_clk}]

create_generated_clock -name {mem_clk} -source [get_ports {sys_clk}] \
    -master_clock {sys_clk} -multiply_by 2.500 \
    [get_nets {u_player/mem_clk}]

create_generated_clock -name {audio_mclk} -source [get_ports {sys_clk}] \
    -master_clock {sys_clk} -divide_by 4 \
    [get_nets {u_player/audio_mclk}]

set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk}] \
    -group [get_clocks {pixel_clk}] \
    -group [get_clocks {pixel_clk_5x}] \
    -group [get_clocks {mem_clk}] \
    -group [get_clocks {audio_mclk}]
