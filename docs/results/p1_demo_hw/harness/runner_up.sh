#!/usr/bin/env bash
# EXTRA=--plan-only for step 4; empty for step 5. --tau-max 14 (operator 2026-10-12).
source $(dirname $0)/env.sh; trap - INT
exec python3 -u /home/user1/Documents/ceiling_arm/docs/results/p1_demo/demo_runner.py --mon /tmp/p1demo_mon_samples.csv --tau-max 14 $EXTRA --out /home/user1/Documents/ceiling_arm/docs/results/p1_demo_hw/hw
