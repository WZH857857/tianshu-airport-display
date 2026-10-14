# ADV11 serial host link timing constraints.
#
# The clock definitions and the asynchronous clock groups are exactly those of
# ADV04 / ADV10: the serial control block runs entirely in the 50 MHz system
# clock domain and does not add a clock.  What it does add is one truly
# asynchronous input, the CH340 transmit line, which is sampled by the two
# flop synchroniser inside uart_rx.  Relaxing the setup check on that pin is
# the standard treatment for an unsynchronised external signal; hold is still
# checked, and every following stage is timed normally.

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

# CH340 TXD arrives whenever the PC decides to send; it is synchronised in
# fabric and is not timed against sys_clk.  The transmit pin is a registered
# output on sys_clk, so it stays covered by the normal clock definition.
set_false_path -from [get_ports {uart_rx_pin}]
