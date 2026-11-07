#!/bin/bash
set -e
mkdir -p sim
python scripts/gen_kin.py > /dev/null
python scripts/gen_atan_vec.py > /dev/null
python model/ik_hw_model.py > /dev/null
python scripts/gen_pid.py > /dev/null
iverilog -g2012 -I rtl -o sim/tb_arm.vvp rtl/cordic_pipe.v rtl/cordic_vec.v rtl/isqrt_pipe.v rtl/kin_ik.v rtl/kin_fk.v rtl/traj_gen.v rtl/pid_ctrl.v rtl/joint_axis.v rtl/arm_top.v tb/tb_arm.v
vvp sim/tb_arm.vvp
