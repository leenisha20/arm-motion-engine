#!/bin/bash
set -e
mkdir -p sim
python scripts/gen_isqrt.py
iverilog -g2012 -I rtl -o sim/tb_isqrt.vvp rtl/isqrt_pipe.v tb/tb_isqrt.v
vvp sim/tb_isqrt.vvp
