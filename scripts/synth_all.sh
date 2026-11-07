#!/bin/bash
mkdir -p results/synth
FILES=$(ls rtl/*.v | tr '\n' ' ')
for m in cordic_pipe cordic_vec isqrt_pipe kin_fk kin_ik traj_gen pid_ctrl fopid_ctrl quad_dec pwm_gen uart_rx uart_tx cmd_parser; do
  yosys -q -p "read_verilog -sv -I rtl $FILES; hierarchy -top $m; synth_xilinx -family xc7 -top $m; tee -o results/synth/$m.txt stat" > results/synth/$m.log 2>&1
  echo "== $m"
  if [ -f results/synth/$m.txt ]; then
    grep -E "cells|LUT[0-9]|DSP48|FD[CPRS]E|CARRY4|RAM" results/synth/$m.txt | sed 's/^ *//'
  else
    echo "SYNTH FAILED, first log lines:"
    head -5 results/synth/$m.log
  fi
done
