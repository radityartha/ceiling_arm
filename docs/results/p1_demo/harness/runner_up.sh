#!/usr/bin/env bash
source $(dirname $0)/env.sh; trap - INT
exec python3 -u /home/user1/Documents/ceiling_arm/docs/results/p1_demo/demo_runner.py --mon /tmp/demo_off_mon_samples.csv --out /home/user1/Documents/ceiling_arm/docs/results/p1_demo/mock
