#!/bin/bash
set -e
mkdir -p sim results
python scripts/gen_pid.py > /dev/null
iverilog -g2012 -I rtl -o sim/tb_loop1.vvp rtl/traj_gen.v rtl/pid_ctrl.v tb/tb_loop1.v
vvp sim/tb_loop1.vvp
python scripts/plot_loop1.py
