#!/usr/bin/env bash
# EXTRA=--plan-only untuk H3; kosong untuk jalan bergerak. ARMS (default arm_1 arm_2 = perilaku lama; gerbang sel §A).
# --tau-max 14 (operator 2026-10-12). MON = sampel monitor. OUT = folder run JSON.
source $(dirname $0)/env.sh; trap - INT
exec python3 -u $H/p1_demo/demo_runner.py --mon ${MON:-$L/p1dzh_mon_samples.csv} --tau-max 14 $EXTRA \
  --tracks-topic /demo_zone/tracks --arms ${ARMS:-arm_1 arm_2} --out ${OUT:-$H/p1_demo_zone_hw/hw}
