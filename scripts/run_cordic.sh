#!/bin/bash
set -e
mkdir -p sim
python model/cordic_model.py
python scripts/gen_atan.py
iverilog -g2012 -I rtl -o sim/tb_cordic.vvp rtl/cordic_pipe.v tb/tb_cordic.v
vvp sim/tb_cordic.vvp
