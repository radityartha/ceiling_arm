#!/usr/bin/env bash
# Build a STATIC GNG topo map -- dual-camera fusion + TF self-filter by default
# (validated procedure, 2026-07-30 -- see README.md section 8b).
#
#   ros2_ws/src/reachability_gng/build_topo.sh
#
# Prereq: both cameras up + calibrated (realsense_dual.launch.py /
# extrinsics_view.launch.py) AND the real arm+gantry bringup up with live
# joint_states (my_workcell.launch.py use_fake_hardware:=false) so self-filter
# TF resolves for all 4 arms -- see README.md section 8b for the full checklist
# (duplicate-process check, gantry initial_positions sanity, etc.) before
# trusting a capture. depth_cloud is auto-started here if not already running.
#
# Output: /tmp/topo_static.npz (override with OUT=...)
#
# Single-camera capture is still available for a quick/degraded check:
#   ./build_topo.sh rgbd2                # -> /tmp/topo_static_rgbd2.npz, single cam
#   CAMS="['rgbd']" ./build_topo.sh       # explicit single-cam override
#
# Other knobs via env vars (defaults match map_topo_static):
#   CAPTURE=8.0  MAX_NODES=1800  MAX_Z=1.75  MAX_X_FROM_CAMERA=2.5  SELF_FILTER=true
# MAX_X_FROM_CAMERA: drop points farther than this (m) from EACH camera's own
# world-X position (not a fixed world-X band); <=0 disables.
#
# Then view it in RViz:
#   ros2 run reachability_gng topo_static_pub --ros-args -p map_file:=<OUT>
set -euo pipefail

# No arg -> dual-camera fusion (the default, validated procedure). A camera_ns
# arg switches to single-camera capture (old behaviour) unless CAMS/OUT
# explicitly override it.
NS="${1:-}"
if [ -n "$NS" ]; then
  CAMS="${CAMS:-['$NS']}"
  OUT="${OUT:-/tmp/topo_static_${NS}.npz}"
else
  NS="rgbd"   # still used below to pick which namespace's depth_cloud to check
  CAMS="${CAMS:-['rgbd','rgbd2']}"
  OUT="${OUT:-/tmp/topo_static.npz}"
fi
CAPTURE="${CAPTURE:-8.0}"
MAX_NODES="${MAX_NODES:-1800}"
MAX_Z="${MAX_Z:-1.75}"
SELF_FILTER="${SELF_FILTER:-true}"
MAX_X_FROM_CAMERA="${MAX_X_FROM_CAMERA:-2.5}"

# Auto-start depth_cloud if no /<ns>/depth_cloud publisher is up yet. A single
# `ros2 topic info` call can false-negative on a busy graph (each invocation
# re-does DDS discovery from scratch, which takes a moment) -- seen in practice
# spawning a DUPLICATE depth_cloud alongside an already-running one. Retry a
# few times before concluding it's really not there.
depth_cloud_up() {
  for _ in $(seq 1 5); do
    ros2 topic info "/${NS}/depth_cloud" 2>/dev/null | grep -q 'Publisher count: [1-9]' && return 0
    sleep 1
  done
  return 1
}

if ! depth_cloud_up; then
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
  -p "max_x_from_camera:=${MAX_X_FROM_CAMERA}" \
  -p "output:=${OUT}"

echo "=== done -> ${OUT} ==="
echo "view:  ros2 run reachability_gng topo_static_pub --ros-args -p map_file:=${OUT}"
