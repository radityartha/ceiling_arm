#!/usr/bin/env bash
# Build a STATIC GNG topo map, fusing BOTH RGBD cameras by default.
#
#   ros2_ws/src/reachability_gng/build_topo.sh [camera_ns]
#
#   camera_ns : if passed, capture from THIS SINGLE camera only (rgbd | rgbd2)
#               instead of the default two-camera fusion. CAMS=... overrides
#               the namespace LIST directly regardless of $1.
#
# Output: /tmp/topo_static.npz  (single-camera runs: /tmp/topo_static_<ns>.npz;
#         override with OUT=...)
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

NS="${1:-}"
if [ -n "$NS" ]; then
  CAMS="${CAMS:-['$NS']}"
  OUT="${OUT:-/tmp/topo_static_${NS}.npz}"
else
  NS="rgbd"                          # used only for the depth_cloud probe below
  CAMS="${CAMS:-['rgbd','rgbd2']}"
  OUT="${OUT:-/tmp/topo_static.npz}"
fi
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
