#!/bin/bash
set -e
mkdir -p sim
python scripts/gen_fopid.py
iverilog -g2012 -I rtl -o sim/tb_fopid.vvp rtl/fopid_ctrl.v tb/tb_fopid.v
vvp sim/tb_fopid.vvp
