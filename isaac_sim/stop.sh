#!/usr/bin/env bash
# Stop everything started by launch_workcell.sh (Isaac, MoveIt, controllers, RViz, demo).
echo ">>> stopping Isaac Sim + ROS stack..."
PIDS=$(pgrep -f "ros2_bridge_gui.py|ros2_bridge.py|ros2 launch isaac|ros2_control_node|move_group|rviz2|moveit_demo.py|robot_state_publisher|controller_manager/spawner" 2>/dev/null | tr '\n' ' ')
[ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null
sleep 2
pgrep -af "ros2_bridge_gui.py|ros2 launch isaac|ros2_control_node|move_group|rviz2" 2>/dev/null | grep -vE "grep|gvfsd" \
  && echo "!! some processes still up (rerun)" || echo "ALL STOPPED."
