#!/bin/bash
set -e
mkdir -p sim
python scripts/gen_pid.py
iverilog -g2012 -I rtl -o sim/tb_pid.vvp rtl/pid_ctrl.v tb/tb_pid.v
vvp sim/tb_pid.vvp
