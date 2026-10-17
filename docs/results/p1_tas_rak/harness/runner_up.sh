#!/usr/bin/env bash
# EXTRA=--plan-only untuk plan-only. ARMS default arm_4 (TAS-RAK). --tau-max 14. Gantry TIDAK diperintah runner.
source $(dirname $0)/env.sh; trap - INT
exec python3 -u $H/p1_demo/demo_runner.py --mon $L/p1tr_mon_samples.csv --tau-max 14 $EXTRA \
  --tracks-topic /rack_box/tracks --arms ${ARMS:-arm_4} --out ${OUT:-$H/p1_tas_rak/hw}
