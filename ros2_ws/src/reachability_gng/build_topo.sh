#!/usr/bin/env bash
# Build a STATIC GNG topo map -- dual-camera fusion + TF self-filter by default
# (validated procedure, 2026-07-30 -- see README.md section 8b).
#
#   ros2_ws/src/reachability_gng/build_topo.sh
#
# Prereq: cameras calibrated (see extrinsics_view.launch.py) AND the real
# arm+gantry bringup up with live joint_states (my_workcell.launch.py
# use_fake_hardware:=false) so self-filter TF resolves for all 4 arms -- see
# README.md section 8b for the full checklist (duplicate-process check,
# gantry initial_positions sanity, etc.) before trusting a capture.
#
# realsense_dual.launch.py and depth_cloud are auto-started here if not
# already running (checked via raw /<ns>/depth and /<ns>/depth_cloud
# publisher presence) -- no need to run them in a separate terminal first.
#
# Output: /tmp/topo_static.npz (override with OUT=...)
#
# Single-camera capture is still available for a quick/degraded check:
#   ./build_topo.sh rgbd2                # -> /tmp/topo_static_rgbd2.npz, single cam
#   CAMS="['rgbd']" ./build_topo.sh       # explicit single-cam override
#
# Other knobs via env vars (defaults match map_topo_static):
#   CAPTURE=8.0  MAX_NODES=1800  MAX_Z=1.75  MAX_X_FROM_CAMERA=2.5  SELF_FILTER=true
#   SAVE_CLOUD=/tmp/topo_cloud_a.npz   also dump the RAW fitted cloud ('' = off)
# SAVE_CLOUD is worth setting for any capture you might want to revisit: a
# capture needs a cleared scene, so re-running one is not free. With the cloud
# on disk, a different fit / an order-invariance replay / a quality comparison
# costs nothing later; without it, they cost another cleared scene.
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

# A single `ros2 topic info` call can false-negative on a busy graph (each
# invocation re-does DDS discovery from scratch, which takes a moment) --
# seen in practice spawning a DUPLICATE node alongside an already-running one.
# Retry a few times before concluding a topic's publisher is really not there.
topic_up() {
  for _ in $(seq 1 5); do
    ros2 topic info "$1" 2>/dev/null | grep -q 'Publisher count: [1-9]' && return 0
    sleep 1
  done
  return 1
}

# Auto-start the RealSense driver(s) for any namespace in CAMS whose raw
# /<ns>/depth isn't publishing yet -- depth_cloud has nothing to deproject
# without it, so map_topo_static hangs forever waiting for points with no
# further log output (looks identical to "still capturing" -- see incident
# 2026-08-02).
need1=false; need2=false
for ns in $(echo "${CAMS}" | grep -oE "'[a-zA-Z0-9_]+'" | tr -d "'"); do
  if ! topic_up "/${ns}/depth"; then
    if [ "$ns" = rgbd ]; then need1=true; fi
    if [ "$ns" = rgbd2 ]; then need2=true; fi
  fi
done

if [ "$need1" = true ] || [ "$need2" = true ]; then
  echo "=== camera(s) not publishing (rgbd:${need1} rgbd2:${need2}) -- starting realsense_dual.launch.py ==="
  ros2 launch reachability_gng realsense_dual.launch.py \
    enable1:="${need1}" enable2:="${need2}" > /tmp/realsense_dual.log 2>&1 &
  # RealSense enumeration + first frames can take a while longer than depth_cloud
  ok=false
  for _ in $(seq 1 30); do
    ok=true
    if [ "$need1" = true ] && ! topic_up /rgbd/depth; then ok=false; fi
    if [ "$need2" = true ] && ! topic_up /rgbd2/depth; then ok=false; fi
    if [ "$ok" = true ]; then break; fi
    sleep 1
  done
  if [ "$ok" != true ]; then
    echo "=== WARNING: camera(s) still not publishing after 30s -- check /tmp/realsense_dual.log ==="
  fi
fi

# Auto-start depth_cloud if no /<ns>/depth_cloud publisher is up yet.
if ! topic_up "/${NS}/depth_cloud"; then
  echo "=== depth_cloud not publishing on /${NS}/depth_cloud -- starting it ==="
  ros2 run reachability_gng depth_cloud > /tmp/depth_cloud.log 2>&1 &
  # give it a moment to latch camera_info + start deprojecting
  for _ in $(seq 1 15); do
    topic_up "/${NS}/depth_cloud" && break
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
  -p "output:=${OUT}" \
  -p "save_cloud:=${SAVE_CLOUD:-}"

echo "=== done -> ${OUT} ==="
echo "view:  ros2 run reachability_gng topo_static_pub --ros-args -p map_file:=${OUT}"
