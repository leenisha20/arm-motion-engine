#!/bin/bash
set -e
mkdir -p sim
python scripts/gen_kin.py
python scripts/gen_atan_vec.py
python model/ik_hw_model.py
iverilog -g2012 -I rtl -o sim/tb_kin_ik.vvp rtl/cordic_pipe.v rtl/cordic_vec.v rtl/isqrt_pipe.v rtl/kin_ik.v tb/tb_kin_ik.v
vvp sim/tb_kin_ik.vvp
