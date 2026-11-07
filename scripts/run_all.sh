#!/bin/bash
# Runs every self-checking testbench; exits non-zero if any does not print "RESULT: PASS".
fail=0
for s in cordic cordic_vec kin_fk kin_ik roundtrip traj isqrt pid fopid periph uart loop1 loop2 axis arm arm_uart pair; do
  if [ ! -x scripts/run_$s.sh ]; then echo "FAIL $s (script missing)"; fail=1; continue; fi
  out=$(./scripts/run_$s.sh 2>&1)
  if echo "$out" | grep -q "RESULT: PASS"; then
    echo "PASS $s"
  else
    echo "FAIL $s"; echo "$out" | tail -15; fail=1
  fi
done
exit $fail
