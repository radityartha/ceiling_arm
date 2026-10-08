#!/usr/bin/env bash
source $(dirname $0)/env.sh; trap - INT
exec ros2 run reachability_gng target_cli --ros-args -p voice_topic:=/voice/transcript -p status_timeout:=900.0 < $(dirname $0)/cli.fifo
