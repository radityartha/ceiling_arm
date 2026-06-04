# Take-Bottle Sequence — Implementation Plan

Runner: [run_take_bottle.py](ros2_ws/src/workcell_description/scripts/run_take_bottle.py)
Launch: [take_bottle_demo.launch.py](ros2_ws/src/workcell_moveit_config/launch/take_bottle_demo.launch.py)

## What this does

A one-shot ROS 2 node drives the workcell through a fixed pick/handoff choreography:
- **Tables** via the `move_dual_table` service (`moving_table_interfaces/srv/MovingTable`).
  `move_table` is *relative*, so the runner tracks commanded position (start = home 0 mm/0°)
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
ros2 launch workcell_moveit_config take_bottle_demo.launch.py
```

Tunable launch args: `gripper_grip_deg` (49), `gripper_open_deg` (0), `gripper_max_effort` (50),
`linear_speed` (3000), `rotate_speed` (1000), `vel_scale`/`acc_scale` (0.1),
`motor_settle_s` (1.0), `table_timeout_s` (120), `startup_delay_s` (3).

Build (first time or after script changes):
```bash
cd ~/Documents/moonshot_project/ros2_ws
colcon build --packages-select workcell_description workcell_moveit_config --symlink-install
```

## The sequence

Arm joint values are in **degrees**, order joint_1…joint_6. Grip = 49°, open = 0°.

| # | Action | Target |
|---|--------|--------|
| 1 | table2 move | linear **880 mm**, angle **90°** |
| 2 | arm4 approach 1 | `[90, -43, -56, -90, 76, 90]` |
| 3 | arm4 approach 2 | `[90, 5, 2, -92, 62, 90]` |
| 3 | gripper4 | **grip** (49°) |
| 4 | arm4 lift | `[90, -75, -130, -90, 20, 90]` |
| 7 | arm4 carry | `[0, -15, -90, -90, 0, 90]` |
| 7b | arm3 pre-approach | `[-3, -16, -61, 95, -46, 90]` |
| 8 | arm3 approach | `[-3, 17, -28, 90, -45, 90]` |
| 8 | gripper3 | **grip** (49°) |
| 9 | gripper4 | **release** (20°) |
| 9 | arm4 retreat | `[0, -60, -115, -90, 30, 2]` |
| 10 | arm3 move | `[98, 66, 20, 108, -45, 82]` |
| 11 | table1 move | **home** (0 mm, 0°) |
| 12a | arm2 approach 1 | `[28, -19, -120, 90, 9, -90]` |
| 12b | arm2 approach 2 | `[28, 21, -86, 90, 16, -90]` |
| 12 | gripper2 | **grip** (49°) |
| 13 | gripper3 | **loosen** (0°) |
| 13 | arm3 retreat | `[63, 93, 130, 148, -119, 117]` |
| 14 | arm2 place | `[0, -45, -99, 90, 55, -90]` |
| 14 | gripper2 | **release** (0°) |
| 15 | all arms home | `[90, -150, -150, 0, 0, 0]` |
| 16 | table1 + table2 home | **0 mm, 0°** |

## Mapping reference

| MoveIt group | URDF prefix | mount | arm IP |
|---|---|---|---|
| arm_1 | t1_a1 | table-1 (right plate, swapped) | 192.168.2.13 |
| arm_2 | t1_a2 | table-1 (left plate, swapped) | 192.168.2.12 |
| arm_3 | t2_a1 | table-2 (right plate, swapped) | 192.168.2.11 |
| arm_4 | t2_a2 | table-2 (left plate, swapped) | 192.168.2.10 |

Grippers: `gripper_N` → joint `t{1,2}_a{1,2}_right_finger_bottom_joint`,
action `/gripper_N_controller/gripper_cmd`.

## Files

- `ros2_ws/src/workcell_description/scripts/run_take_bottle.py` — the runner node (installed
  via `install(PROGRAMS ...)` in `workcell_description/CMakeLists.txt`; symlinked, edits are live).
- `ros2_ws/src/workcell_moveit_config/launch/take_bottle_demo.launch.py` — starts
  `dual_table_controller` + the runner.
- `ros2_ws/src/workcell_moveit_config/launch/single_rviz_workcell.launch.py` — workcell bringup.
