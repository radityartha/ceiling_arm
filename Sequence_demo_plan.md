# CeilingArm Sequence Demo — Implementation Plan

Source choreography: `Sequence movement ceilingArm demo.pdf`
Runner: [run_sequence_demo.py](ros2_ws/src/workcell_description/scripts/run_sequence_demo.py)
Launch: [sequence_demo.launch.py](ros2_ws/src/workcell_moveit_config/launch/sequence_demo.launch.py)

## What this does

A one-shot ROS 2 node drives the workcell through a fixed pick/handoff choreography:
- **Tables** via the `move_dual_table` service (`moving_table_interfaces/srv/MovingTable`).
  `go_to_table` is *relative*, so the runner tracks commanded position (start = home 0 mm/0°)
  and sends `delta = target − tracked`, then polls `/joint_states` until the table settles.
- **Arms** via MoveIt's `move_action` (joint-space `JointConstraint` goals), groups `arm_2/3/4`.
  Speed capped at `vel_scale`/`acc_scale` = 0.1 so large swings don't trip the Kinova e-stop.
- **Grippers** via each gripper's `GripperCommand` action (`/gripper_N_controller/gripper_cmd`)
  — NOT MoveGroup, because a Kinova gripper stalls on contact and MoveGroup reports that as
  CONTROL_FAILED. A stall or reached-goal both count as success.

The sequence runs in order and aborts on the first hard failure.

## How to run

Terminal A (workcell: arms + MoveIt + RViz + per-arm controllers):
```bash
cd ~/Documents/moonshot_project && ./scripts/start_single_rviz.sh
```
Wait until all 4 arms connect (`ros2 topic echo /joint_states --once` shows non-zero values).

Terminal B (table controller + sequence runner):
```bash
source /opt/ros/humble/setup.bash && source ~/Documents/moonshot_project/ros2_ws/install/setup.bash
ros2 launch workcell_moveit_config sequence_demo.launch.py
```

Tunable launch args: `gripper_grip_deg` (49), `gripper_open_deg` (0), `gripper_max_effort` (50),
`linear_speed` (3000), `rotate_speed` (1000), `vel_scale`/`acc_scale` (0.1),
`motor_settle_s` (1.0), `table_timeout_s` (120), `startup_delay_s` (3).

## The sequence (current)

Arm joint values are in **degrees**, order joint_1…joint_6. Grip = 49°, open = 0°.

| # | Action | Target |
|---|--------|--------|
| 1 | table2 move | linear **650 mm**, angle **90°** |
| 2 | arm4 approach 1 | `[90, -43, -56, -90, 76, 0]` |
| 3 | arm4 approach 2 | `[90, -10, -63, -90, 35, 0]` |
| 3 | gripper4 | **grip** (49°) |
| 4 | arm4 reach | `[90, -75, -130, -90, 30, 2]` |
| 6 | arm4 lift | `[0, -60, -115, -90, 30, 2]` |
| 7 | arm4 carry | `[0, -15, -90, -90, 15, 0]` |
| 7b | arm3 pre-approach | `[0, -55, -95, 90, -44, 90]` |
| 8 | arm3 approach | `[0, 0, -71, 89, -11, 90]` |
| 8 | gripper3 | **grip** (49°) |
| 9 | gripper4 | **release** (20°) |
| 9 | arm4 retreat | `[0, -60, -115, -90, 30, 2]` |
| 10 | arm3 move | `[97, 99, 80, 106, -78, 90]` |
| 11 | table1 move | **home** (0 mm, 0°) |
| 12a | arm2 approach 1 | `[28, -19, -120, 90, 9, -90]` |
| 12b | arm2 approach 2 | `[28, 21, -86, 90, 16, -90]` |
| 12 | gripper2 | **grip** (49°) |
| 13 | gripper3 | **loosen** (0°) |
| 13 | arm3 retreat | `[63, 93, 130, 148, -119, 90]` |
| 14 | arm2 place | `[0, 0, 0, 90, 0, -90]` |
| 14 | gripper2 | **release** (0°) |

(PDF numbering skips "5"; kept for traceability. Steps 4 & 6 joint_6 ≈ ±2° — trivial.)

## Mapping reference

| MoveIt group | URDF prefix | mount | arm IP |
|---|---|---|---|
| arm_1 | t1_a1 | table-1 (right plate, swapped) | 192.168.2.13 |
| arm_2 | t1_a2 | table-1 (left plate, swapped) | 192.168.2.12 |
| arm_3 | t2_a1 | table-2 (right plate, swapped) | 192.168.2.11 |
| arm_4 | t2_a2 | table-2 (left plate, swapped) | 192.168.2.10 |

Grippers: `gripper_N` → joint `t{1,2}_a{1,2}_right_finger_bottom_joint`,
action `/gripper_N_controller/gripper_cmd`.

## Fixes that made it work (history)

1. **Controller routing** — `single_rviz_workcell.launch.py` now builds move_group +
   per-arm spawners directly (not via `generate_demo_launch`, whose sub-launches rebuild
   config from the wrong default `moveit_controllers.yaml`). Uses
   `config/moveit_controllers_per_arm.yaml` (one FollowJointTrajectory controller per arm),
   so execution doesn't route through the coupled `table_N_with_arm_controller` (whose table
   mock joints lack a velocity interface → CONTROL_FAILED).
2. **Left/right reversed** — swapped each arm's mount plate + orientation in
   `workcell.urdf.xacro` (IPs stay with the arm prefix). Required a `colcon build` because
   the installed URDF was a stale copy.
3. **Per-arm mock** — added `armN_fake` xacro args so an unreachable arm can be mocked
   without aborting the whole `ros2_control_node`.
4. **Gripper** — switched from MoveGroup to direct `GripperCommand` action; stall = success.
5. **Speed cap** — `vel_scale`/`acc_scale` = 0.1 to avoid Kinova protective stops.

## Known open items (verify on next run)

- **Gripper may not physically close** — `move_gripper` currently continues even when the
  action reports "did not reach goal and not stalled". If the gripper does nothing, check
  command direction (open vs close), `gripper_grip_deg`, or `gripper_max_effort`.
- **Step 8 (arm3) failed once with code 99999** (MoveIt planning failure — likely arm3 goal
  collides with where arm4 is parked, or is unreachable). Re-verify with the new step 2/3
  poses; if it persists, adjust the arm3 approach joints.

## Files

- `ros2_ws/src/workcell_description/scripts/run_sequence_demo.py` — the runner node (installed
  via `install(PROGRAMS ...)` in `workcell_description/CMakeLists.txt`; symlinked, edits are live).
- `ros2_ws/src/workcell_moveit_config/launch/sequence_demo.launch.py` — starts
  `dual_table_controller` + the runner.
- `ros2_ws/src/workcell_moveit_config/launch/single_rviz_workcell.launch.py` — workcell bringup.
- `ros2_ws/src/workcell_description/urdf/workcell.urdf.xacro` — arm mounts + per-arm mock args.
