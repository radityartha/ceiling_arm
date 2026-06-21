---
name: isaac-sim-setup
description: How to run Isaac Sim 4.5 and view it via noVNC on this PC, for the ceiling_arm digital twin
metadata:
  type: project
---

Isaac Sim 4.5.0 is pip-installed in a Python 3.10 venv at `/srv/data/users/raditya/isaacsim`.
Activate with `source /srv/data/users/raditya/isaacsim/activate_isaacsim.sh`, then `python <script>.py`.

**noVNC viewing:** the user's TigerVNC desktop is `DISPLAY=:22380`; noVNC/websockify serves it on
port 22380 (`http://<server>:22380/vnc.html`). Launch GUI scripts with `headless: False` and
`DISPLAY=:22380 python view_scene.py &` (background). RTX renders fine over this software X server
despite the GPU caveat. Isaac swallows `print` to stdout in headless mode — write verification to a file.

**Digital twin work** lives in `isaac_sim/single_arm/` and `isaac_sim/workcell/` (each: import_urdf.py,
test_scene.py, view_scene.py, generated .urdf + .usd + meshes/). Both verified: imported to USD,
physics-stable under gravity, joint-controllable. Workcell = 44 DOF (2 table linear, 2 table rotation,
24 arm, 16 gripper), tables anchored at z=2.05 (ceiling) with arms hanging down.

**ROS 2 bridge (step 1 = joint states OUT, WORKING):** `isaac_sim/workcell/ros2_joint_state_pub.py`
publishes /joint_states (44 joints @200Hz) + /clock via an OmniGraph (OnPlaybackTick -> ROS2Context
+ ROS2PublishJointState + ROS2PublishClock + IsaacReadSimulationTime). Three gotchas, all hit and
solved: (1) the loop MUST use `world.step(render=True)` — render pumps the graph so OnPlaybackTick
fires; render=False advertises the topic but publishes no data. (2) DDS interop: use
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on BOTH sides (Isaac bundles cyclone; it's the system default
here). FastDDS gave discovery-only — data blocked by Isaac-vs-system shared-memory mismatch; UDP-only
FastDDS XML broke discovery. (3) PublishJointState `targetPrim` must be the exact prim with
ArticulationRootAPI (auto-detected in the script as `/World/workcell/root_joint`), not `/World/workcell`.
Run with `ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

**ROS 2 bridge (step 2 = BIDIRECTIONAL, WORKING):** `isaac_sim/workcell/ros2_bridge.py` adds
ROS2SubscribeJointState -> IsaacArticulationController to the publish graph. Subscribes
/joint_command (sensor_msgs/JointState); send name+position for any subset of the 44 joints and only
those are driven. ArticulationController uses `inputs:robotPath` = the ArticulationRootAPI prim (same
auto-detected ART path). Set stiff gains via `wc.get_articulation_controller().set_gains(1e5,1e4)`
before building the graph so commands are reached. Verified: `ros2 topic pub /joint_command ... {name:
['t1_a1_joint_2'], position:[0.6]}` -> /joint_states reported that joint at 0.5992. NOTE `ros2 topic
echo --field position` prints `array('d', [...])` (numpy array repr), not `- val` YAML lines.
Both halves of the bridge work.

**MoveIt -> Isaac integration (SINGLE ARM, WORKING):** full digital-twin loop verified — MoveIt OMPL
plans, executes via FollowJointTrajectory -> ros2_control topic_based -> /isaac_joint_commands -> Isaac
-> /isaac_joint_states -> feedback. Files: `isaac_sim/single_arm/ros2_bridge.py` (Isaac side, kortex
topics) + `isaac_sim/single_arm/ros/{bringup.launch.py,ros2_controllers.yaml,moveit_test.py}`.
Build/setup done: `topic_based_ros2_control` apt-installed; kortex_api wheel pip-installed; built
`kortex_description` + `kinova_gen3_lite_moveit_config` in a CLEAN overlay at
`/srv/data/users/raditya/kortex_min_ws` against /opt/ros/humble (the vendored ros2_kortex_ws is
version-skewed/broken — its ros2_controllers source references removed control_msgs .desired/.actual
fields; do NOT build ros2_control from it, use apt). bringup builds robot_description from kortex
`robots/gen3_lite_gen3_lite_2f.xacro` with `sim_isaac:=true` (-> topic_based on /isaac_joint_commands,
/isaac_joint_states) and feeds the installed gen3_lite moveit config to move_group. Run: start
ros2_bridge.py (isaac venv), then `ros2 launch .../ros/bringup.launch.py` (source /opt/ros/humble +
kortex_min_ws), both with ROS_DOMAIN_ID=42 RMW=cyclonedds. Verified: direct FJT goal -> joint_2=0.5006;
MoveGroup plan+execute (joint_2->0.8, joint_4->-0.5) -> SUCCESS, Isaac reached 0.7991/-0.5096.
No moveit_py installed; drive planning via rclpy MoveGroup action client (moveit_test.py /
moveit_demo.py). GUI viewer: ros2_bridge_gui.py (headless:False, DISPLAY=:22380) shows MoveIt
driving the arm live in noVNC.

**Gripper via MoveIt (WORKING):** added gen3_lite_2f_gripper_controller
(position_controllers/GripperActionController, joint right_finger_bottom_joint) to
ros/ros2_controllers.yaml + bringup. The kortex sim_isaac xacro makes a SECOND topic_based system
(GripperHardwareInterface) for the 4 finger joints that publishes to the SAME /isaac_joint_commands as
the arm system (2 publishers, 1 subscriber, different joint sets -> no conflict). topic_based applies
the ros2_control mimic params (multiplier -1) so all 4 fingers move from one command. Verified
open(0.96)/close(0.0) visible in Isaac. Command:
`ros2 action send_goal /gen3_lite_2f_gripper_controller/gripper_cmd control_msgs/action/GripperCommand
"{command: {position: 0.96, max_effort: 50.0}}"`. Note: GripperActionController often reports
stalled:true with no object (cosmetic; allow_stalling:true). Mimic multiplier is -1 here (not the
physically accurate -0.676 for tips) - fine for open/close, refine if precise grasping needed.
**4-ARM WORKCELL via MoveIt (WORKING):** all 4 arms (arm_1..4) + 4 grippers controlled by MoveIt
through Isaac. Files: `isaac_sim/workcell/ros2_bridge.py` / `ros2_bridge_gui.py` (isaac topics,
shared /isaac_joint_commands + /isaac_joint_states; grippers handled natively by topic_based, no
extra coupling) + `isaac_sim/workcell/ros/{bringup.launch.py,ros2_controllers.yaml,moveit_demo.py}`.
Two source edits were REQUIRED (both additive, in repo + kortex):
  (1) `ros2_ws/src/workcell_description/urdf/workcell.urdf.xacro`: added `sim_isaac` arg and threaded
      `sim_isaac="$(arg sim_isaac)"` into all 4 load_robot calls (was hardcoded to mock).
  (2) kortex `grippers/gen3_lite_2f/urdf/gen3_lite_2f.ros2_control.xacro`: changed
      `<ros2_control name="GripperHardwareInterface">` -> `name="${prefix}GripperHardwareInterface"`
      (4 grippers had duplicate hardware names -> ros2_control_node aborts "name is duplicated").
Build robot_description with `sim_isaac:=true use_fake_hardware:=false` (both true => mock AND
topic_based both emit -> conflict). Built overlay `/srv/data/users/raditya/workcell_overlay_ws`
(workcell_description + workcell_moveit_config) against kortex_min_ws + apt. Bringup uses
MoveItConfigsBuilder("trailer_workcell", "workcell_moveit_config") + moveit_controllers_per_arm.yaml.
move_group logs harmless "table_X_with_arm is not a chain" (those coupled groups have no IK; we plan
arm_1..4 chains). Verified: all 4 arms reach POSE_A (joint_2~0.6, joint_4~-0.4) in /isaac_joint_states.
`ros2 topic echo` CLI was flaky (use --no-daemon).

**TABLES bridged too (WORKING):** added a `<xacro:if value="$(arg sim_isaac)">` TableTopicBased
topic_based ros2_control system (t1/t2 linear+rotation) in workcell.urdf.xacro, plus table_1/2_controller
(JTC) in ros/ros2_controllers.yaml + bringup. Verified table_1 linear->0.15, rotation->0.30 in Isaac.
So the FULL workcell (4 arms + 4 grippers + 2 tables) is ROS2-controllable in Isaac.
**Polished scene in GUI bridge:** ros2_bridge_gui.py now calls polish.build_room()/add_lights()/
recolor_tables() so the MoveIt demo runs in the room+work-table scene (no add_default_ground_plane).
Demo: `python3 isaac_sim/workcell/ros/moveit_demo.py` cycles the 4 arms.

**Gripper (Gen3 Lite 2F) = 1-DOF via SOFTWARE coupling in `isaac_sim/gripper.py`** (not PhysX
mimic — importer's parse_mimic produced broken/flailing fingers: "needs a finite limit" errors and
no gearing). Master joint = `right_finger_bottom_joint`; couplings (from the gazebo mimic plugin in
gen3_lite_2f_transmission_macro.xacro): right_finger_tip = -0.676*m+0.149, left_finger_bottom =
-1.0*m, left_finger_tip = -0.676*m+0.149. DIRECTION (verified in sim): increasing master OPENS;
OPEN=0.96 (pad gap ~0.108 m), CLOSED=-0.09. gripper.py auto-detects 1 or 4 grippers from dof-name
suffixes. Grasp tests (grasp_test.py) pass: genuine grasp, cube held with ~0 drop. Note an
arm-pointing-UP grasp can FALSE-PASS (cube rests on a finger) — validate with arm pointing down.

**URDF generation gotcha:** the repo's `ros2_kortex`/`livox` submodules are NOT checked out and the
workspace isn't built. Other kortex installs on the PC (MRTA ones) are Docker symlink-builds with
dangling `/home/colcon_ws/...` links — unusable. To resolve `$(find kortex_description)` for xacro,
build a clean ament prefix at `/srv/data/users/raditya/tmp/kdesc_prefix` symlinking the real source
tree at `/srv/data/users/raditya/workspace/ros2_kortex_ws/src/ros2_kortex/kortex_description`
(and `workcell_description` from this repo), then `export AMENT_PREFIX_PATH=$PFX:/opt/ros/humble`.
