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
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DISPLAY=:22380
LOG=/tmp/workcell_launch; mkdir -p "$LOG"
MODE="${1:-rviz}"
PIDS=()

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

echo ">>> [1/3] Isaac Sim + bridge (first boot ~1-2 min)..."
( source "$ISAAC_ACT"; export ROS_DOMAIN_ID RMW_IMPLEMENTATION DISPLAY
  cd "$REPO/isaac_sim/workcell"; exec python ros2_bridge_gui.py ) > "$LOG/bridge.log" 2>&1 &
PIDS+=($!)
wait_for "$LOG/bridge.log" "Ctrl-C to stop" 240 || cleanup
echo "    bridge up."

echo ">>> [2/3] MoveIt + ros2_control..."
( source /opt/ros/humble/setup.bash; source "$KORTEX_WS"; source "$WORKCELL_WS"
  export ROS_DOMAIN_ID RMW_IMPLEMENTATION
  cd "$REPO"; exec ros2 launch isaac_sim/workcell/ros/bringup.launch.py ) > "$LOG/bringup.log" 2>&1 &
PIDS+=($!)
wait_for "$LOG/bringup.log" "You can start planning now" 120 || cleanup
echo "    move_group + 11 controllers up."

case "$MODE" in
  headless) echo ">>> [3/3] skipped (headless).";;
  demo)
    echo ">>> [3/3] looping 4-arm MoveIt demo..."
    ( source /opt/ros/humble/setup.bash; source "$KORTEX_WS"; source "$WORKCELL_WS"
      export ROS_DOMAIN_ID RMW_IMPLEMENTATION
      cd "$REPO"; exec python3 isaac_sim/workcell/ros/moveit_demo.py ) > "$LOG/demo.log" 2>&1 &
    PIDS+=($!);;
  *)
    echo ">>> [3/3] RViz MotionPlanning..."
    ( source /opt/ros/humble/setup.bash; source "$KORTEX_WS"; source "$WORKCELL_WS"
      export ROS_DOMAIN_ID RMW_IMPLEMENTATION DISPLAY
      cd "$REPO"; exec ros2 launch isaac_sim/workcell/ros/rviz.launch.py ) > "$LOG/rviz.log" 2>&1 &
    PIDS+=($!);;
esac

echo
echo "=================================================================="
echo " Workcell running. Open noVNC: http://<server>:22380/vnc.html"
[ "$MODE" = "rviz" ] && echo " RViz: Planning Group -> arm_1..4 / table_1/2 -> Plan & Execute"
echo " Logs: $LOG/   |   Press Ctrl-C here to stop everything."
echo "=================================================================="
wait
