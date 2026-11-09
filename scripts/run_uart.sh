#!/bin/bash
set -e
mkdir -p sim
iverilog -g2012 -I rtl -o sim/tb_uart.vvp rtl/uart_rx.v rtl/uart_tx.v rtl/cmd_parser.v tb/tb_uart.v
vvp sim/tb_uart.vvp
