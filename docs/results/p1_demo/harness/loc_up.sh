#!/usr/bin/env bash
source $(dirname $0)/env79.sh; trap - INT
R=""
for ns in rgbd rgbd2; do for s in "" _labels _conf; do R="$R -r /$ns/instance_segmentation$s:=/$ns/seg/instance_segmentation$s"; done; done
exec ros2 run reachability_gng object_localizer --ros-args $R
