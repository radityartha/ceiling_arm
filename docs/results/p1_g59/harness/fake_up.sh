#!/usr/bin/env bash
source $(dirname $0)/env.sh; trap - INT
exec python3 -u /home/user1/Documents/ceiling_arm/docs/results/p1_demo/fake_tracks.py --tid $1 --label bottle --xyz $2
