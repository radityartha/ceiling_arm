#!/usr/bin/env bash
source $(dirname $0)/env.sh; trap - INT
exec python3 -u /home/user1/Documents/ceiling_arm/docs/results/p1_demo/demo_runner.py --mon /tmp/claude-1001/-home-user1-Documents-ceiling-arm/c032767e-b00c-429e-bddd-e55e3eb2950d/scratchpad/g58_mon_samples.csv $EXTRA --out /home/user1/Documents/ceiling_arm/docs/results/p1_g59/mock
