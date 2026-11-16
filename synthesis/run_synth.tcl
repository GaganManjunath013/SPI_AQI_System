# =============================================================================
# run_synth.tcl - Cadence Genus synthesis script for spi_top
# Run from the repository root:  genus -f synthesis/run_synth.tcl
# =============================================================================

set DESIGN   "spi_top"
set RTL_DIR  "rtl"
set SDC_FILE "synthesis/spi_top.sdc"
set OUT_DIR  "synthesis/out"
file mkdir $OUT_DIR

# Library setup: point these at your own standard-cell library.
set_db init_lib_search_path /path/to/your/90nm/lib
set_db library slow.lib

# RTL, sub-modules first and the top module last.
# spi_flash_model.v, spi_top_props.sv and the testbench are not synthesised.
set RTL_FILES {
    spi_master.v
    spi_sensor_slave.v
    spi_processing_slave.v
    uart_tx.v
    uart_rx.v
    alarm_bram.v
    spi_flash_ctrl.v
    spi_slave.v
    spi_top.v
}
foreach f $RTL_FILES {
    read_hdl $RTL_DIR/$f
}

elaborate $DESIGN
check_design -unresolved

read_sdc $SDC_FILE

set_db syn_generic_effort medium
set_db syn_map_effort     medium
set_db syn_opt_effort     medium

syn_generic
syn_map
syn_opt

report_timing > $OUT_DIR/${DESIGN}_timing.rpt
report_area   > $OUT_DIR/${DESIGN}_area.rpt
report_power  > $OUT_DIR/${DESIGN}_power.rpt
report_gates  > $OUT_DIR/${DESIGN}_gatecount.rpt

# Netlist and constraints for place and route
write_hdl > $OUT_DIR/${DESIGN}_netlist.v
write_sdc > $OUT_DIR/${DESIGN}_out.sdc

puts "Synthesis finished. Reports are in $OUT_DIR."
