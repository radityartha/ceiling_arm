#!/usr/bin/env bash
source $(dirname $0)/g61_env.sh; trap - INT
exec python3 -u /home/user1/Documents/ceiling_arm/docs/results/p1_g22/js_record.py $SP/g61_joint_states.csv
