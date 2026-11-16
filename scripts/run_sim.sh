#!/usr/bin/env bash
# Compile and run the full-system testbench with Icarus Verilog.
# Run from the repository root:  bash scripts/run_sim.sh
set -euo pipefail

iverilog -o sim_out \
  rtl/spi_top.v rtl/spi_master.v rtl/spi_sensor_slave.v rtl/spi_processing_slave.v \
  rtl/spi_slave.v rtl/alarm_bram.v rtl/spi_flash_ctrl.v rtl/spi_flash_model.v \
  rtl/uart_tx.v rtl/uart_rx.v \
  tb/spi_tb.v

vvp sim_out | tee results/simulation/regression.txt
