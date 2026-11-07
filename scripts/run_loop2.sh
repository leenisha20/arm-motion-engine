#!/bin/bash
set -e
mkdir -p sim results
python scripts/gen_pid.py > /dev/null
python scripts/gen_fopid.py > /dev/null
iverilog -g2012 -I rtl -o sim/tb_loop2.vvp rtl/traj_gen.v rtl/pid_ctrl.v rtl/fopid_ctrl.v tb/tb_loop2.v
vvp sim/tb_loop2.vvp
python scripts/plot_loop2.py
