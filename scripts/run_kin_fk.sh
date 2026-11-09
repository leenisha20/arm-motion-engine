#!/bin/bash
set -e
mkdir -p sim
python scripts/gen_kin.py
iverilog -g2012 -I rtl -o sim/tb_kin_fk.vvp rtl/cordic_pipe.v rtl/kin_fk.v tb/tb_kin_fk.v
vvp sim/tb_kin_fk.vvp
