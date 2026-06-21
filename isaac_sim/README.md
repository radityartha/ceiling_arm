# Isaac Sim digital twin

A NVIDIA **Isaac Sim 4.5** digital twin of the ceiling workcell (2 motorized tables +
4 Kinova Gen3 Lite arms + grippers), driven by the project's real **ROS 2 / MoveIt**
stack through a topic bridge. The goal is software-in-the-loop testing: plan and
execute with the same MoveIt + ros2_control you run on hardware, against a simulated
robot instead of the real arms.

Built bottom-up and verified at each step: **URDF → USD import → physics → grippers →
ROS 2 bridge → MoveIt control**, single arm first, then the full workcell scene.

---

## Environment

Isaac Sim 4.5 is pip-installed in a venv. Activate it for any `python` script here:

```bash
source /srv/data/users/raditya/isaacsim/activate_isaacsim.sh
```

ROS 2 interop uses **CycloneDDS** on a dedicated domain (both Isaac and ROS sides):

```bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

GUI viewers render to the noVNC desktop on `DISPLAY=:22380`
(`http://<server>:22380/vnc.html`).

---

## Layout

```
isaac_sim/
├── gripper.py              # 1-DOF software coupling for the Gen3 Lite 2F gripper
├── single_arm/             # one Gen3 Lite arm (the proven pipeline)
│   ├── import_urdf.py      # URDF -> USD (URDF importer)
│   ├── test_scene.py       # physics: stability + joint control (headless, self-checking)
│   ├── view_scene.py       # GUI viewer (noVNC)
│   ├── grasp_test.py       # spawn a cube and grasp it (1-DOF gripper)
│   ├── view_grasp.py       # GUI grasp demo (grips + carries a cube)
│   ├── ros2_bridge.py      # ROS 2 bridge, kortex topics (headless)
│   ├── ros2_bridge_gui.py  # same, with GUI for noVNC
│   ├── gen3_lite.urdf/.usd # generated model + meshes/
│   └── ros/                # ROS 2 side
│       ├── bringup.launch.py     # move_group + topic_based ros2_control + JTC + gripper
│       ├── ros2_controllers.yaml
│       ├── moveit_test.py        # one MoveGroup plan+execute
│       └── moveit_demo.py        # loop through poses (visual demo)
└── workcell/               # full cell: 4 arms + 2 tables (44 DOF)
    ├── import_urdf.py / test_scene.py / view_scene.py
    ├── polish.py / view_polished.py / polish_check.py   # room + lights + work table
    ├── grasp_test.py
    ├── ros2_joint_state_pub.py   # bridge step 1: joint states out
    ├── ros2_bridge.py            # bridge step 2: bidirectional
    └── workcell.urdf/.usd + meshes/
```

`*.usd`, `configuration/`, and `meshes/` are **regenerable** by `import_urdf.py`
(from the kortex xacro). The `.py`/`.yaml`/`.launch.py` files are the source of truth.

---

## Pipeline

1. **URDF generation** — `xacro` the kortex `gen3_lite_gen3_lite_2f.xacro` (single arm)
   or this repo's `workcell.urdf.xacro` (full cell), with meshes copied locally and
   paths rewritten to relative so the model is self-contained.
2. **USD import** — `import_urdf.py` runs the Isaac URDF importer: `fix_base=true`
   (anchors `world` / the ceiling), merge fixed joints, mesh collisions, position drives.
3. **Physics** — `test_scene.py` drops the model into a scene, tunes joint drives,
   and verifies it holds pose under gravity and follows joint commands.
4. **Gripper** — `gripper.py` couples the 4 finger joints to one master
   (`right_finger_bottom_joint`); `grasp_test.py` confirms a real grasp.
5. **ROS 2 bridge** — an OmniGraph publishes `/…joint_states` and subscribes to
   `/…joint_commands`, mapping to the articulation.
6. **MoveIt** — `topic_based_ros2_control` makes ros2_control talk those topics, so
   `move_group` plans (OMPL) and executes through the standard controllers into Isaac.

---

## Run it: single-arm MoveIt demo

```bash
# Terminal 1 — Isaac (GUI in noVNC) + bridge
source /srv/data/users/raditya/isaacsim/activate_isaacsim.sh
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DISPLAY=:22380
python isaac_sim/single_arm/ros2_bridge_gui.py

# Terminal 2 — MoveIt + ros2_control (topic_based)
source /opt/ros/humble/setup.bash
source /srv/data/users/raditya/kortex_min_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch isaac_sim/single_arm/ros/bringup.launch.py

# Terminal 3 — make it move
source /opt/ros/humble/setup.bash
source /srv/data/users/raditya/kortex_min_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 isaac_sim/single_arm/ros/moveit_demo.py        # arm cycles poses
ros2 action send_goal /gen3_lite_2f_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.96, max_effort: 50.0}}"   # open
```

The arm plans/executes via MoveIt and the gripper opens/closes — visible in noVNC.

### One-time ROS setup (already done on this machine)

```bash
sudo apt install -y ros-humble-topic-based-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
python3 -m pip install --user dependencies/kortex_api-2.6.0.post3-py3-none-any.whl
# clean overlay built against /opt/ros/humble (NOT the version-skewed ros2_kortex_ws):
cd /srv/data/users/raditya/kortex_min_ws && colcon build   # kortex_description + kinova_gen3_lite_moveit_config
```

---

## Status

| Capability | Single arm | Workcell (4 arms + tables) |
|---|---|---|
| URDF → USD import | ✅ | ✅ (44 DOF) |
| Physics stable + controllable | ✅ | ✅ |
| 1-DOF gripper + grasping | ✅ | ✅ |
| Polished scene (room/lights/work table) | — | ✅ |
| ROS 2 bridge (states + commands) | ✅ | ✅ |
| **MoveIt plans + executes (arm + gripper)** | ✅ | next |

**Next:** scale MoveIt control to the 4-arm workcell — per-arm command/state topics
(one `topic_based` system and one Isaac subscribe→controller pair per arm) plus the
2 table joints, driven via this repo's `workcell_moveit_config`.

---

## Gotchas (learned the hard way)

- **GUI loop must `world.step(render=True)`** — the OmniGraph (and thus the ROS publish
  node) only ticks on the render/app-update path; `render=False` advertises topics but
  publishes no data.
- **DDS interop** — use CycloneDDS on both sides. FastDDS gave discovery-only (data
  blocked by Isaac↔system shared-memory mismatch).
- **`targetPrim` / `robotPath`** must be the exact prim with `ArticulationRootAPI`
  (`…/root_joint`), not the reference root; the scripts auto-detect it.
- **Don't build ros2_control from `ros2_kortex_ws`** — its vendored source is
  version-skewed (references removed `control_msgs` fields). Use apt + a clean overlay.
- **Gripper** — PhysX hard-mimic was unreliable; we couple in software (`gripper.py`).
  Direction: increasing the master joint *opens* (open ≈ 0.96, closed ≈ −0.09).
```
