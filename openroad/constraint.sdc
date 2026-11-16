# =============================================================================
# constraint.sdc - timing constraints for the OpenROAD (sky130hd) flow
# Same intent as synthesis/spi_top.sdc, written with port-based names because
# Yosys does not keep the Genus register names.
# =============================================================================

# System clock, 100 MHz
create_clock -name clk_i -period 10.000 [get_ports clk_i]

# Virtual clock used as the reference for every synchronous I/O delay.
create_clock -name clk_io -period 10.000

# Clock tree insertion delay measured after CTS on sky130hd (about 1.3 ns).
# Applied as ideal latency on clk_i before CTS and as the latency of the
# virtual I/O clock at every stage, so the I/O reference stays aligned with
# the clock the flops actually see.
set CLK_INSERTION 1.300
set_clock_latency $CLK_INSERTION [get_clocks clk_i]
set_clock_latency $CLK_INSERTION [get_clocks clk_io]

# Flash SPI clock, clk_i / 4 = 25 MHz, defined at the output port.
# (The sensor-bus SCLK stays inside the chip and is only ever sampled as data.)
create_generated_clock -name sclk_flash \
    -source [get_ports clk_i] \
    -divide_by 4 \
    [get_ports flash_sclk_o]

set_clock_uncertainty -setup 0.150 [get_clocks clk_i]
set_clock_uncertainty -hold  0.050 [get_clocks clk_i]

# Synchronous I/O. Clock ports carry no data and get no I/O delay.
set data_inputs {}
foreach p [all_inputs] {
  if { [get_full_name $p] ne "clk_i" } { lappend data_inputs $p }
}
set data_outputs {}
foreach p [all_outputs] {
  if { [get_full_name $p] ne "flash_sclk_o" } { lappend data_outputs $p }
}

set_input_delay  -max  2.000 -clock clk_io $data_inputs
set_input_delay  -min  0.500 -clock clk_io $data_inputs
set_output_delay -max  5.000 -clock clk_io $data_outputs
set_output_delay -min -0.500 -clock clk_io $data_outputs

# Asynchronous inputs: reset and signals that enter through two-flop
# synchronisers.
set_false_path -from [get_ports rst_i]
set_false_path -from [get_ports uart_rx_i]
set_false_path -from [get_ports sleep_i]
set_false_path -from [get_ports bod_i]

# MCU register-readback SPI port: oversampled by clk_i through synchronisers,
# unrelated to clk_i, so not timed against it.
set_false_path -from [get_ports {sclk_s_i cs_s_i mosi_s_i}]
set_false_path -to   [get_ports miso_s_o]
