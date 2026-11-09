#!/bin/bash
set -e
mkdir -p sim
python scripts/gen_pid.py > /dev/null
iverilog -g2012 -I rtl -o sim/tb_axis.vvp rtl/traj_gen.v rtl/pid_ctrl.v rtl/joint_axis.v tb/tb_axis.v
vvp sim/tb_axis.vvp
