#!/usr/bin/env bash
source $(dirname $0)/env.sh
trap - INT
exec ros2 run reachability_gng reach_dwell_monitor --ros-args -p csv_log:=/tmp/demo_off_mon
