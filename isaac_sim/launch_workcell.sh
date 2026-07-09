#!/usr/bin/env bash
# One-command launch of the 4-arm workcell Isaac Sim digital twin:
#   Terminal 1) Isaac Sim GUI + ROS 2 bridge   (isaac venv)
#   Terminal 2) MoveIt + ros2_control          (system Humble + overlays)
#   Terminal 3) RViz MotionPlanning            (default; or `demo` to script the arms)
#
# Usage:
#   ./launch_workcell.sh            # bridge + bringup + RViz
#   ./launch_workcell.sh demo       # bridge + bringup + looping 4-arm demo (no RViz)
#   ./launch_workcell.sh headless   # bridge + bringup only (no RViz, no demo)
#
# Ctrl-C stops everything. Watch it in noVNC: http://<server>:22380/vnc.html
set -u

REPO="/srv/data/users/raditya/arm_WS/ceiling_arm"
ISAAC_ACT="/srv/data/users/raditya/isaacsim/activate_isaacsim.sh"
KORTEX_WS="/srv/data/users/raditya/kortex_min_ws/install/setup.bash"
WORKCELL_WS="/srv/data/users/raditya/workcell_overlay_ws/install/setup.bash"
GNG_WS="$REPO/ros2_ws/install/setup.bash"   # this repo's ws: reachability_gng clouds
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DISPLAY=:22380
LOG=/tmp/workcell_launch; mkdir -p "$LOG"
MODE="${1:-gng}"
PIDS=()

# Mode -> model scope. Default/gng/headless use the TABLE_1-ONLY model so
# gantry_2/arm_3/arm_4 are absent in move_group + RViz AND hidden in Isaac
# (GNG_HIDE_T2=1). full/demo keep the original 4-arm workcell.
case "$MODE" in
  full|demo) export GNG_HIDE_T2=0
             BRINGUP="isaac_sim/workcell/ros/bringup.launch.py"
             RVIZ_LAUNCH="isaac_sim/workcell/ros/rviz.launch.py";;
  *)         export GNG_HIDE_T2=1
             BRINGUP="isaac_sim/workcell/ros/bringup_table1.launch.py"
             RVIZ_LAUNCH="isaac_sim/workcell/ros/rviz_table1.launch.py";;
esac

cleanup() { echo; echo ">>> stopping..."; kill -9 "${PIDS[@]}" 2>/dev/null; exit 0; }
trap cleanup INT TERM

wait_for() {  # wait_for <logfile> <pattern> <timeout_s>
  local f="$1" pat="$2" t="${3:-180}" i=0
  while true; do
    grep -qiE "$pat" "$f" 2>/dev/null && return 0
    sleep 2; i=$((i+2))
    [ "$i" -ge "$t" ] && { echo "!! timed out waiting for: $pat"; return 1; }
  done
}

# Self-clean: kill any leftovers from a previous run so we never double-start
# (no need to run stop.sh first). Same pattern as stop.sh.
_STALE='ros2_bridge_gui.py|ros2_bridge.py|ros2 launch isaac|ros2_control_node|move_group|rviz2|moveit_demo.py|robot_state_publisher|controller_manager/spawner|lib/reachability_gng/visualize|gng_clouds.launch.py|rgbd2?_camera_optical|perception.launch.py|lib/reachability_gng/object_localizer|lib/reachability_gng/reachability_check|lib/reachability_gng/reachability_cloud|lib/reachability_gng/collision_cloud|lib/reachability_gng/object_collision|lib/reachability_gng/octomap_refresher|lib/reachability_gng/seg_cloud|lib/reachability_gng/seg_router|lib/reachability_gng/static_collision|lib/reachability_gng/table_slab'
echo ">>> [0/3] clearing any previous session..."
_old=$(pgrep -f "$_STALE" 2>/dev/null | tr '\n' ' ')
[ -n "$_old" ] && { kill -9 $_old 2>/dev/null; sleep 2; echo "    cleared."; } || echo "    nothing running."

echo ">>> [1/3] Isaac Sim + bridge (first boot ~1-2 min)..."
( set +u; source "$ISAAC_ACT"; export ROS_DOMAIN_ID RMW_IMPLEMENTATION DISPLAY GNG_HIDE_T2
  cd "$REPO/isaac_sim/workcell"; exec python ros2_bridge_gui.py ) > "$LOG/bridge.log" 2>&1 &
PIDS+=($!)
wait_for "$LOG/bridge.log" "Ctrl-C to stop" 240 || cleanup
echo "    bridge up."

echo ">>> [2/3] MoveIt + ros2_control..."
( set +u; source /opt/ros/humble/setup.bash; source "$KORTEX_WS"; source "$WORKCELL_WS"
  export ROS_DOMAIN_ID RMW_IMPLEMENTATION
  cd "$REPO"; exec ros2 launch "$BRINGUP" ) > "$LOG/bringup.log" 2>&1 &
PIDS+=($!)
wait_for "$LOG/bringup.log" "You can start planning now" 120 || cleanup
echo "    move_group + controllers up ($BRINGUP)."

# Static TFs world -> {rgbd,rgbd2}_camera_optical (ROS optical convention) so the
# two Isaac RGBD cameras (/rgbd/*, /rgbd2/*) show up in RViz and detections can be
# transformed to `world`. Quaternions computed from each camera's eye/target in
# CAMERAS (ros2_bridge_gui.py) — keep in sync if you move a camera.
echo ">>> [2.5] camera static TFs (world -> rgbd/rgbd2 optical)..."
( set +u; source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID RMW_IMPLEMENTATION
  # x shifted 3.0 -> 4.35 to follow rgbd's eye/target +1.35 X shift in
  # ros2_bridge_gui.py (matches polish.py's work-table move to cx=2.9); the
  # quaternion is unchanged because that shift was a pure translation.
  exec ros2 run tf2_ros static_transform_publisher \
    --x 4.35 --y 1.2 --z 2.05 \
    --qx -0.435026 --qy -0.703886 --qz 0.477652 --qw 0.295205 \
    --frame-id world --child-frame-id rgbd_camera_optical ) > "$LOG/camera_tf.log" 2>&1 &
PIDS+=($!)
( set +u; source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID RMW_IMPLEMENTATION
  # y shifted -0.6 -> -1.2 to follow rgbd2's eye/target -0.6 Y shift in
  # ros2_bridge_gui.py (puts cam2 0.84 m off gantry_2 in Y, mirroring cam1);
  # pure translation, so the quaternion is unchanged.
  exec ros2 run tf2_ros static_transform_publisher \
    --x -0.6 --y -1.2 --z 2.05 \
    --qx -0.703886 --qy 0.435026 --qz -0.295205 --qw 0.477652 \
    --frame-id world --child-frame-id rgbd2_camera_optical ) > "$LOG/camera2_tf.log" 2>&1 &
PIDS+=($!)

case "$MODE" in
  headless) echo ">>> [3/3] skipped (headless).";;
  demo)
    echo ">>> [3/3] looping 4-arm MoveIt demo..."
    ( set +u; source /opt/ros/humble/setup.bash; source "$KORTEX_WS"; source "$WORKCELL_WS"
      export ROS_DOMAIN_ID RMW_IMPLEMENTATION
      cd "$REPO"; exec python3 isaac_sim/workcell/ros/moveit_demo.py ) > "$LOG/demo.log" 2>&1 &
    PIDS+=($!);;
  *)
    echo ">>> [3/3] GNG reachability clouds + RViz MotionPlanning..."
    # clouds publish from THIS repo's workspace (kept separate from the Isaac
    # overlay to avoid workspace shadowing); they reach RViz over DDS.
    if [ ! -f /tmp/arm1_model.npz ] || [ ! -f /tmp/arm2_model.npz ]; then
      echo "    !! /tmp/arm{1,2}_model.npz missing — run ros2_ws/src/reachability_gng/build_maps.sh first"
    fi
    ( set +u; source /opt/ros/humble/setup.bash; source "$GNG_WS"
      export ROS_DOMAIN_ID RMW_IMPLEMENTATION
      cd "$REPO"; exec ros2 launch reachability_gng gng_clouds.launch.py ) > "$LOG/gng_clouds.log" 2>&1 &
    PIDS+=($!)
    # RGBD perception is NOT started here anymore: it runs alongside the executor
    # in pick_stack.launch.py (a separate terminal) so perception/executor code can
    # be restarted fast without rebooting Isaac. Start it with:
    #   ros2 launch reachability_gng pick_stack.launch.py execute:=true
    ( set +u; source /opt/ros/humble/setup.bash; source "$KORTEX_WS"; source "$WORKCELL_WS"
      export ROS_DOMAIN_ID RMW_IMPLEMENTATION DISPLAY
      cd "$REPO"; exec ros2 launch "$RVIZ_LAUNCH" ) > "$LOG/rviz.log" 2>&1 &
    PIDS+=($!);;
esac

echo
echo "=================================================================="
echo " Workcell running. Open noVNC: http://<server>:22380/vnc.html"
case "$MODE" in
  full|demo) echo " 4-arm workcell. RViz: Planning Group -> arm_1..4 / gantry_1/2 -> Plan & Execute";;
  *)         echo " TABLE_1 GNG view (gantry_2/arm_3/4 hidden). RViz: Planning Group -> gantry_1_with_arm_1/_2 -> Plan & Execute";;
esac
echo " Logs: $LOG/   |   Press Ctrl-C here to stop everything."
echo "=================================================================="
wait
