#!/bin/bash
set -e
mkdir -p sim
python model/cordic_vec_model.py
python scripts/gen_atan_vec.py
iverilog -g2012 -I rtl -o sim/tb_cordic_vec.vvp rtl/cordic_vec.v tb/tb_cordic_vec.v
vvp sim/tb_cordic_vec.vvp
