#!/usr/bin/env bash
# Build a STATIC GNG topo map from ONE RGBD camera.
#
#   ros2_ws/src/reachability_gng/build_topo.sh [camera_ns]
#
#   camera_ns : rgbd (default) | rgbd2   -- which camera to capture from.
#               (fusion of both is deferred until the cameras are mounted +
#                calibrated; when ready:  CAMS="['rgbd','rgbd2']" build_topo.sh)
#
# Output: /tmp/topo_static_<camera_ns>.npz  (override with OUT=...)
#
# Prereq: cameras up (realsense_dual.launch.py). depth_cloud is auto-started here
# if it is not already running (it publishes /<ns>/depth_cloud for BOTH cameras).
#
# Knobs via env vars (defaults match map_topo_static / README section 8a):
#   CAPTURE=8.0  MAX_NODES=1800  MAX_Z=1.75  SELF_FILTER=false
#
# Then view it in RViz:
#   ros2 run reachability_gng topo_static_pub --ros-args -p map_file:=<OUT>
set -euo pipefail

NS="${1:-rgbd}"
# CAMS lets you override the namespace LIST directly (for the future 2-cam fusion
# build); by default it is just the single NS passed as $1.
CAMS="${CAMS:-['$NS']}"
OUT="${OUT:-/tmp/topo_static_${NS}.npz}"
CAPTURE="${CAPTURE:-8.0}"
MAX_NODES="${MAX_NODES:-1800}"
MAX_Z="${MAX_Z:-1.75}"
SELF_FILTER="${SELF_FILTER:-false}"

# Auto-start depth_cloud if no /<ns>/depth_cloud publisher is up yet.
if ! ros2 topic info "/${NS}/depth_cloud" 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
  echo "=== depth_cloud not publishing on /${NS}/depth_cloud -- starting it ==="
  ros2 run reachability_gng depth_cloud > /tmp/depth_cloud.log 2>&1 &
  # give it a moment to latch camera_info + start deprojecting
  for _ in $(seq 1 15); do
    ros2 topic info "/${NS}/depth_cloud" 2>/dev/null | grep -q 'Publisher count: [1-9]' && break
    sleep 1
  done
fi

echo "=== building static topo map from ${CAMS} -> ${OUT} ==="
ros2 run reachability_gng map_topo_static --ros-args \
  -p "camera_namespaces:=${CAMS}" \
  -p "self_filter:=${SELF_FILTER}" \
  -p "capture_seconds:=${CAPTURE}" \
  -p "max_nodes:=${MAX_NODES}" \
  -p "max_z:=${MAX_Z}" \
  -p "output:=${OUT}"

echo "=== done -> ${OUT} ==="
echo "view:  ros2 run reachability_gng topo_static_pub --ros-args -p map_file:=${OUT}"
