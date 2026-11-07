#!/bin/bash
set -e
mkdir -p sim
iverilog -g2012 -I rtl -o sim/tb_periph.vvp rtl/quad_dec.v rtl/pwm_gen.v tb/tb_periph.v
vvp sim/tb_periph.vvp
