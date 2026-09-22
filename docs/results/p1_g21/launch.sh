#!/bin/bash
# G21 E1-E4, one process per (experiment, t_fold). Logs next to the json.
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
EV=../../../ros2_ws/src/reachability_gng/test/eval_sched_heur.py
M="--map1 ../../../ros2_ws/src/reachability_gng/data/cap_g1_rail160.npz --map2 ../../../ros2_ws/src/reachability_gng/data/cap_g2_rail160.npz"
for tf in 0 50.8 126.8; do
  nohup python3 run_g21.py e1 --t-fold $tf > g21_e1_tf$tf.log 2>&1 &
  nohup python3 run_g21.py e4 --t-fold $tf > g21_e4_tf$tf.log 2>&1 &
  nohup bash -c "python3 $EV part2 $M --t-fold $tf --out g21_e2_tf$tf.json && python3 $EV part2lb $M --t-fold $tf --out g21_e2_tf$tf.json" > g21_e2_tf$tf.log 2>&1 &
  nohup python3 $EV ablate $M --t-fold $tf --out g21_e3_tf$tf.json > g21_e3_tf$tf.log 2>&1 &
done
