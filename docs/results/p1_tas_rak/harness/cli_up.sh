#!/usr/bin/env bash
# target_cli atas /rack_box/tracks (remap). 'g' hanya lewat $L/cli.fifo SESUDAH izin operator untuk SATU jalan.
source $(dirname $0)/env.sh; trap - INT
[ -p $L/cli.fifo ] || mkfifo $L/cli.fifo
exec ros2 run reachability_gng target_cli --ros-args -p voice_topic:=/voice/transcript -p status_timeout:=900.0 \
  -r /detected_objects/tracks:=/rack_box/tracks < $L/cli.fifo
