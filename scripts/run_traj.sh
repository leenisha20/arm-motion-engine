#!/bin/bash
set -e
mkdir -p sim
python model/traj_model.py > /dev/null
iverilog -g2012 -I rtl -o sim/tb_traj.vvp rtl/traj_gen.v tb/tb_traj.v
vvp sim/tb_traj.vvp
