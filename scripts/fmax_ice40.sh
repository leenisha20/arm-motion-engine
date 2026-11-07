#!/bin/bash
mkdir -p results/synth
for m in cordic_pipe cordic_vec isqrt_pipe; do
  echo "== $m"
  yosys -q -p "read_verilog -sv -I rtl rtl/$m.v; synth_ice40 -top $m -json results/synth/${m}_ice40.json" > results/synth/${m}_ice40_yosys.log 2>&1 || { echo "yosys failed"; head -5 results/synth/${m}_ice40_yosys.log; continue; }
  nextpnr-ice40 --hx8k --package ct256 --json results/synth/${m}_ice40.json --freq 100 > results/synth/${m}_ice40_pnr.log 2>&1
  grep -E "Max frequency|ERROR" results/synth/${m}_ice40_pnr.log | tail -2
  grep -E "ICESTORM_LC:" results/synth/${m}_ice40_pnr.log | tail -1
done
