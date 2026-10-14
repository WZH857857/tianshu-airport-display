# hdmi_mm_player timing constraints
#
# The design has five clock domains:
#   sys_clk       50.000 MHz  input pin, TF card, FAT32, player, frame loader
#   pixel_clk     25.000 MHz  pixel PLL clk1, video timing, TMDS encoder
#   pixel_clk_5x 125.000 MHz  pixel PLL clk0, TMDS serialiser
#   mem_clk      125.000 MHz  memory PLL clk1, vendor SDRAM controller
#   mem_clk_sft  125.000 MHz  memory PLL clk2, 180 degrees, only the read
#                             capture flop in the vendor core runs on it
#
# Without a constraint file TD folds every PLL output into the 50 MHz input
# clock and then times the 125 MHz serialiser against 20 ns, so this file is
# what actually gives place and route a real clock picture.
#
# 1. board input clock, 50 MHz
create_clock -name sys_clk -period 20 -waveform {0 10} [get_ports {sys_clk}]
#
# 2. constrain the PLL outputs explicitly
#
# TD 6.2.1 cannot reach the PLL pins through the IP wrappers: derive_clocks
# emits [get_pins {pll_inst.clkc[n]}] without the parent path, and the fully
# qualified u_pixel_pll/pll_inst.clkc[n] form matches nothing either.  Both
# die with USR-8108 / USR-8159 and leave every register on sys_clk.
#
# The clock nets themselves do resolve by name, so the generated clocks are
# attached there.  The multiply/divide factors are the ones TD itself derives
# from the IP parameters: pixel PLL 50 MHz * 2.5 = 125 MHz and /2 = 25 MHz,
# pixel PLL: C0 = 125 MHz, C1 = 25 MHz.
create_generated_clock -name {pixel_clk_5x} -source [get_ports {sys_clk}] \
    -master_clock {sys_clk} -multiply_by 2.500 [get_nets {pixel_clk_5x}]
create_generated_clock -name {pixel_clk} -source [get_ports {sys_clk}] \
    -master_clock {sys_clk} -divide_by 2.000 [get_nets {pixel_clk}]
#
# 2b. the memory controller clock.  The vendor SDRAM controller runs at
#     125 MHz and captures read data with a second 125 MHz output shifted by
#     180 degrees, exactly like the lab_ex5_i2s reference design.
#
#     Only the 0 degree output is declared.  Giving the 180 degree output its
#     own generated clock makes the tool analyse every path between the two
#     as a 4 ns half cycle and produces a wall of false violations; the
#     reference design leaves that output alone.  The capture edge is a
#     physical property of the PLL output, not something place and route has
#     to schedule.
create_generated_clock -name {mem_clk} -source [get_ports {sys_clk}] \
    -master_clock {sys_clk} -multiply_by 2.500 [get_nets {mem_clk}]
# 
# 3. clock domain relationships.
# 
#    Every crossing in this design is deliberately asynchronous:
#      sys_clk    <-> pixel_clk     grey coded async FIFOs, toggle
#                                   synchronisers, the SDRAM frame store
#                                   command stream and its status flags
#      pixel_clk  ->  pixel_clk_5x  the TMDS serialiser holds the parallel word
#                                   while a divide by two toggle is resynchronised
#                                   into the 5x domain and gates the load
#                                   (SerializerN_1_lvds.vhd)
#      sys_clk    <-> mem_clk       command FIFO from the loader, read FIFO to
#                                   the pixel side, status synchronisers
set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk}] \
    -group [get_clocks {pixel_clk}] \
    -group [get_clocks {pixel_clk_5x}] \
    -group [get_clocks {mem_clk}]
