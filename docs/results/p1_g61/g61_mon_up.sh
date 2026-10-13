#!/usr/bin/env bash
source $(dirname $0)/g61_env.sh; trap - INT
exec ros2 run reachability_gng reach_dwell_monitor --ros-args -p csv_log:=$SP/g61_mon
