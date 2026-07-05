# reachability_gng

Growing Neural Gas (GNG) **reachability + capability map** for the redundant
**arm + table** system, used to **seed MoveIt IK**. It is **arm-list-driven**:
one separate map is built per arm. `gantry_1` currently carries `arm_1` and
`arm_2` (each 8 DOF = 2 table + 6 arm joints); adding `arm_3`/`arm_4` later is a
one-line config addition. Target deliverable: Year-1 conference paper
(IEEE SMC / IAS / SII / AROB).

## Idea

Each GNG node holds a vector `[task | q]`:

- `task` = end-effector pose features (xyz, optionally + quaternion)
- `q`    = the 8-DOF config `[t1_linear, t1_rotation, t1_a1_joint_1..6]`

BMU search uses only the `task` dims (so nodes tile the reachable workspace),
while adaptation moves the whole vector (so each node's `q` becomes a
representative IK seed for its workspace cell). The table joints are **first-class
DOF**: the map shows how table motion *extends* reachability, and distinct table
placements for the same pose separate into different nodes — keeping seeds valid.

Node coloring in RViz encodes **manipulability** `w = sqrt(det(J Jᵀ))`
(Yoshikawa index): blue = low (near-singular / workspace edge), red = high
(dexterous). This is the "capability" layer on top of binary reachability.

## Environment / how to run

- **No Isaac Sim needed.** This whole package is kinematic: `data_gen`/`train`
  use Pinocchio FK; `seed_ik`/`eval ik` use MoveIt `/compute_ik`; `visualize`
  is plain RViz. Isaac is only for *executing* motions under physics (a later
  step), not for any result here.
- **Run from the repo root** — the config's `urdf:` path is repo-root-relative.
- **Make the module importable**, either by sourcing the colcon install:
  ```bash
  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
  ```
  or, for the ROS-free offline scripts, with `PYTHONPATH`:
  ```bash
  PYTHONPATH=ros2_ws/src/reachability_gng python3 -m reachability_gng.data_gen ...
  ```
- **Dependencies:** `data_gen` needs `pinocchio` (`pip install pin`); `train`
  needs only NumPy; the ROS nodes need a sourced ROS 2 Humble + this package
  built (`colcon build --packages-select reachability_gng --symlink-install`).

## Pipeline

```
data_gen.py   sample q -> FK (Pinocchio) -> [pose, q, manip]   ->  dataset.npz
train.py      build [task|q], GNG.fit, annotate hits+manip+hold->  model.npz (+_stats)
seed_ik.py    PoseStamped -> GNG seed -> /compute_ik            ->  ik_solution
visualize.py  load model; nodes/edges                          ->  RViz MarkerArray
eval.py       volume comparison + IK benchmark                 ->  paper numbers
gantry_reach_executor  object -> energy-ranked seed pool -> IK -> MoveGroup plan/exec
```

### 1. Flattened URDF (already present)
`data_gen` reads `workcell_description/urdf/workcell_full.urdf`. It already
exists with the table rail set to **0–3.0 m** and Gen3 Lite mesh paths rewritten
to `package://kortex_description/...`. Regenerating it with `xacro` requires the
`ros2_kortex` submodule populated (it currently isn't), so prefer editing the
flattened file directly for now.

### 2. Sample FK data + train (offline, no ROS) — recommended dense recipe

**One map per arm, in one command** — loops over the arm configs and writes
`/tmp/<name>_model.npz` (+ `_stats.npz`) for each. Adding `arm_3`/`arm_4` later
is a one-line addition to the `ARMS` array in the script (plus a config):
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2_ws/src/reachability_gng/build_maps.sh        # -> /tmp/arm1_model.npz, /tmp/arm2_model.npz
# override the dense recipe via env vars, e.g.:  N=20000 LAM=120 build_maps.sh
```

Or a single arm by hand (the script just loops this):
```bash
python3 -m reachability_gng.data_gen \
    --config ros2_ws/src/reachability_gng/config/arm1_table1.yaml \
    --out /tmp/arm1_dataset.npz --n 80000
python3 -m reachability_gng.train --dataset /tmp/arm1_dataset.npz \
    --out /tmp/arm1_model.npz --task pos --max-nodes 3000 --lam 60 --epochs 2 \
    --boundary-nodes 600 \
    --config ros2_ws/src/reachability_gng/config/arm1_table1.yaml
# -> 3000 nodes (600 pinned boundary shell + interior); node hull reaches the
#    TRUE arm+table reach surface exactly (--boundary-nodes; see gotcha below).
#    Omit --boundary-nodes (or set 0) for the legacy centroid-only map that falls
#    ~0.26 m short at the edge.
# --config adds per-node `hold` (gravity holding cost) to _stats.npz for the
# energy-aware executor (section 7); omit it and hold defaults to 0.
```

**Node count = pinned boundary shell + interior; the interior is set by `lam`:**
```
nodes ≈ BOUNDARY  +  min( max_nodes − BOUNDARY , n × epochs / lam )
```
The `BOUNDARY` pinned shell nodes (default 600, see "boundary seeding" below) are
independent of `lam`; only the *interior* count follows `n × epochs / lam`. To
hit a target interior count `M`, set `lam ≈ n × epochs / M`.

#### Tuning: change sample count / node density
`build_maps.sh` takes the knobs as env vars (defaults `N=80000 LAM=60
MAX_NODES=3000 EPOCHS=2 BOUNDARY=600 BOUNDARY_TAU=0.4` → 3000 nodes = 600 pinned
shell + ~2400 interior):
```bash
N=200000 ros2_ws/src/reachability_gng/build_maps.sh    # more FK samples (denser data, slower)
LAM=160  ros2_ws/src/reachability_gng/build_maps.sh    # ~1600 nodes (600 shell + ~1000 interior)
BOUNDARY=1000 ros2_ws/src/reachability_gng/build_maps.sh  # denser edge shell
BOUNDARY=0 ros2_ws/src/reachability_gng/build_maps.sh     # legacy centroid-only map (no shell)
```
**Fewer nodes, same coverage — now safe:** the boundary is pinned, so raising
`LAM` only thins the *interior*; the outer extent and edge fidelity are held by
the shell (unlike the legacy map, where fewer nodes ragged-ified/shrank the
boundary). Pick `LAM ≈ N×EPOCHS / (target interior nodes)`, e.g. `LAM=110`→~1450,
`LAM=160`→~1000, `LAM=320`→~500 interior. Lowering `N` still shrinks coverage
(samples define what the shell can cover), so keep `N` high. After rebuilding,
reload the clouds (`pkill -9 -f lib/reachability_gng/visualize` then relaunch
`gng_clouds.launch.py` / `launch_workcell.sh`).

### 3a. Quick cloud-only look in RViz (no MoveIt)
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch reachability_gng view_gng.launch.py model_path:=/tmp/arm1_model.npz
# color_by:=hits to color nodes by reachability density instead of manipulability
```
Launches `robot_state_publisher` + `joint_state_publisher_gui` (drag the
sliders to pose the arm/table; the table rotation rests at its **90° home**) +
the GNG `MarkerArray` + RViz. The cloud is **static** — it's the union over
*all* configs (incl. the full 3 m rail and ±180° table rotation), so it does
not change with the sliders; the robot is just one pose inside it.

### 3b. Both clouds + MoveIt MotionPlanning in one RViz (fake hardware)
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch reachability_gng gng_moveit.launch.py     # uses /tmp/arm{1,2}_model.npz
```
Brings up `move_group` + a ros2_control **mock** joint source + the workcell
controllers on **fake hardware** (no real arms, no Isaac Sim, no LIDAR), one
`visualize` node **per arm** (arm_1 **green**, arm_2 **orange**, nodes colored
by manipulability), and RViz with the **MotionPlanning** plugin. Pick group
`gantry_1_with_arm_1` or `gantry_1_with_arm_2` to **Plan/Execute** while both
reachability clouds are shown. There is one RViz and one `/joint_states` source
(the broadcaster) — do **not** also start `joint_state_publisher_gui`.

The `robot_description` is the **flattened `workcell_full.urdf`** (not
`trailer_workcell.urdf.xacro`): the `ros2_kortex` submodule is incomplete here
so the xacro fails to expand, and the flattened urdf is the source of truth
(same model the GNG clouds use). For controller execution it carries `mock_components`
ros2_control with **unique** per-arm/gripper hardware names plus a `table_system`
block for the 4 table joints, so the fake controllers load and Plan/Execute
works without the xacro.

### 3c. Both clouds over the **Isaac Sim** digital twin
Plan/Execute against the Isaac-driven robot while the clouds are shown. One
command runs Isaac bridge + move_group (topic_based) + clouds + GNG RViz:
```bash
./isaac_sim/launch_workcell.sh         # default 'gng' mode: TABLE_1-ONLY view
```
The default **`gng`** mode uses a **gantry_1-only** model (`isaac_sim/workcell/ros/`
`table1_isaac.urdf` + `trailer_table1.srdf`, made by `make_table1_model.py`) and
hides the `t2_*` prims in Isaac, so **gantry_2/arm_3/arm_4 are gone in both Isaac
and RViz**. Use `./isaac_sim/launch_workcell.sh full` (or `demo`) for the original
4-arm cell. Build the maps first (`build_maps.sh` → `/tmp/arm{1,2}_model.npz`).
Clouds are in the `world` frame so they line up with the ceiling-mounted arms;
planning `gantry_1_with_arm_1`/`_2` executes split across the `arm_1`+`gantry_1`
controllers. (Everything shares `ROS_DOMAIN_ID=42` + `rmw_cyclonedds_cpp`.)
See **README.html** for the short quick-start.

**Isaac gotchas (hard-won):**
- **NumPy in the Isaac venv must be 1.26.x.** `pip install pin`/anything that pulls
  NumPy 2.x into `/srv/.../isaacsim/env_isaacsim` makes Isaac crash on boot
  (`module 'numpy' has no attribute '_no_nep50_warning'`). Fix:
  `…/env_isaacsim/bin/python -m pip install "numpy==1.26.0"`. GNG's offline tools
  use **system** Python (separate), so they're unaffected.
- **Table joints have a dedicated command topic** `/isaac_table_commands` (arms +
  grippers stay on `/isaac_joint_commands`). The bridge has a separate
  subscriber→ArticulationController for it. Without this, the arm command flood
  starved the table's setpoints → the rail/rotation froze mid-move.
- **Heavy table joints get drive force + gains set at runtime** in the bridge
  (the URDF importer baked a tiny effort, e.g. 10 N·m on rotation → it couldn't
  turn the load). The bridge sets `UsdPhysics.DriveAPI` maxForce high + tuned kp/kd.
- **Joint limits live in `isaac_sim/workcell/workcell.urdf`** (the source of
  `workcell.usd`). After editing limits/effort there, **re-import**:
  `python isaac_sim/workcell/import_urdf.py` (Isaac venv). The rail is 0–3.0 m,
  rotation ±180°, home −90°.
- **MoveIt's collision octomap silently stays empty** unless two conditions hold —
  and then the arm plans straight through the work table (the RViz green/grey
  clouds are `reachability_cloud`/`seg_cloud`, **visualisation only**; they never
  collide):
  1. `collision_cloud` must publish in the **`world`** frame (the octomap map
     frame). In the camera optical frame the octomap updater drops every cloud
     (no error logged) and the octree is never built.
  2. `octomap_refresher`'s `period` must stay **≫ the updater's per-cloud time**
     (≈0.25 s at `stride=6`, ≈1 s at `stride=3`). At the old 1 s period it
     `/clear_octomap`-ed the map faster than it rebuilt, so the planner kept
     seeing an empty world.
  Verify the **collision** octomap directly:
  `ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 32}}"`
  — `resolution` should read `0.03` and `data` be non-empty.
- **move_group aborts slow gantry executions.** Isaac runs slower than realtime,
  so a long gantry move trips move_group's execution-duration monitor and aborts
  mid-motion (short arm-only moves finish inside the tolerance and succeed).
  `bringup_table1.launch.py` sets `trajectory_execution.execution_duration_monitoring:
  false` (+ large `allowed_execution_duration_scaling` / `allowed_goal_duration_margin`)
  so slow-but-correct motions are not killed; the plan was already collision-checked.

### 4. GNG-seeded MoveIt IK (Phase 2)
Needs `move_group` running so `/compute_ik` exists (e.g. `gng_moveit.launch.py`
above), plus the per-arm SRDF group. `seed_ik` is generic — point it at an
arm via `model_path` / `group` / `ee_frame`:
```bash
# arm_1
ros2 run reachability_gng seed_ik --ros-args -p model_path:=/tmp/arm1_model.npz \
    -p group:=gantry_1_with_arm_1 -p ee_frame:=t1_a1_tool_frame -p use_gng_seed:=true
# arm_2 (same node, different params)
ros2 run reachability_gng seed_ik --ros-args -p model_path:=/tmp/arm2_model.npz \
    -p group:=gantry_1_with_arm_2 -p ee_frame:=t1_a2_tool_frame -p use_gng_seed:=true
# set use_gng_seed:=false for the no-seed baseline.
# then publish a target on /gng_seed_ik/target_pose (geometry_msgs/PoseStamped)
```
Logs IK success/solve-time, republishes the solution on `/gng_seed_ik/ik_solution`.
`build_ik_request` / `solve_ik` are reused by `eval.py`.

### 5. Evaluation (Phase 3 — the paper's numbers)
```bash
# offline reachable-volume comparison (table-locked vs table-active datasets)
python3 -m reachability_gng.eval volume --datasets locked.npz active.npz --res 0.05
# IK benchmark (needs move_group)
python3 -m reachability_gng.eval ik --model /tmp/model.npz --dataset /tmp/dataset.npz \
    --config ros2_ws/src/reachability_gng/config/arm1_table1.yaml \
    --methods gng none random --n 500 --csv results.csv
```
`--config` enables the KDL random-restart baseline and manipulability scoring.

### 6. RGBD perception → object reachability (Isaac digital twin)

Two RGBD cameras in Isaac (in `isaac_sim/workcell/ros2_bridge_gui.py`) watch the
gantry_1 corridor from opposite ends of the prismatic rail; ground-truth instance
segmentation labels each object. Perception nodes turn that into object positions
in `world` and check them against the GNG cloud. The whole stack comes up with
`pick_stack.launch.py` (perception **+** the executor, Section 7) or, on its own,
`perception.launch.py`. `object_localizer` also publishes the target's
**size-adaptive box** on `/target_collision_boxes` (tracked object top → the
executor's stand-off height). Background nodes are quieted to WARN in the launch
so the shared terminal shows the executor's pick pipeline; `reachability_cloud`
stays at INFO and prints each object's **% reachable** only when it changes.

Cameras publish, per namespace `rgbd` / `rgbd2`: `/<ns>/rgb`, `/<ns>/depth`,
`/<ns>/camera_info`, `/<ns>/instance_segmentation` (+ `_labels`). The
`world→<ns>_camera_optical` static TFs are published by `launch_workcell.sh`.

| node | in → out | purpose |
|---|---|---|
| `object_localizer` | depth + seg (both cams) → `/detected_objects` (PoseArray) + `/detected_objects/markers` | deproject masked depth → centroid → `world` (tf2), fuse + dedup across cameras |
| `reachability_check` | `/detected_objects` → `/reachability/markers` | per-object green/red **CUBE** + text (arm, dist, manipulability) via nearest GNG node |
| `reachability_cloud` | depth + seg → `/reachability/voxels` (default) **or** `/<ns>/reachability_cloud` | per-part reachability that follows the object's shape; **voxel** mode classifies each voxel green/red and reports **% reachable by volume** |
| `seg_colorizer` | seg (32SC1) → `/<ns>/instance_segmentation_color` (rgb8) | colourised mask for an RViz Image / rqt panel |
| `seg_cloud` | depth + seg → `/<ns>/seg_cloud` (PointCloud2) | scene cloud coloured by segmentation id |
| `collision_cloud` | depth (objects excluded) → `/<ns>/collision_cloud` (frame `world`) | environment-only cloud feeding MoveIt's octomap (graspables kept out, see `object_collision`). Published already in the `world` map frame and subsampled (`stride`, default 6) so the octomap updater keeps up — see the octomap gotcha below |
| `object_collision` | depth + seg → `/planning_scene` | each object as an exact `CollisionObject` box + attach/detach for grasp (`/object_collision/command`) |
| `octomap_refresher` | timer (`period`, default 60 s) → `/clear_octomap` | periodically flush stale arm/object voxels so a moved arm doesn't bake in. **Gated by `pause_gate`** — never clears during a pick (wiping the octomap mid-execution invalidates the running plan → move_group aborts −3). `period` **must stay well above** the updater's per-cloud time, or it wipes the map faster than it rebuilds (see gotcha) |
| `map_static` / `static_collision` | one-shot map → `/planning_scene` | map STATIC known geometry (work table, cabinet, fridge, …) ONCE into boxes, publish them as reliable occlusion-free collision geometry. Map one piece per run with `name:=` (and `roi:=[xmin,xmax,ymin,ymax]` when several are in view); boxes append to a shared list |

Reachability rule: an object (or voxel) is **reachable** if its distance to the
nearest GNG node is ≤ `reach_radius` (default 0.12 m); the node's stored `q` (incl.
table DOF) and manipulability come along for free. `reachability_cloud` fuses the
two cameras on one shared world voxel grid (overlap auto-dedups → a fair volumetric
%), and **debounces** voxels over `voxel_ttl` (default 1.0 s) so physics jitter /
async camera frames don't flicker the colours. Segmentation, localization and the
clouds adapt to **any object shape** (cube, can, bottle, ball) as long as the prim
carries a semantic label — only the fixed-size CUBE marker is cube-specific; use
the voxel/point cloud for arbitrary shapes.

```bash
# everything (Isaac + cameras + MoveIt + GNG clouds + perception + RViz):
./isaac_sim/launch_workcell.sh

# or just the perception nodes against a running bridge:
ros2 launch reachability_gng perception.launch.py
ros2 run reachability_gng reachability_cloud -p voxel_size:=0.0   # raw-point mode
ros2 run reachability_gng seg_colorizer      # 2D colour-mask helper
ros2 run reachability_gng seg_cloud          # segmentation scene cloud
```

### 7. Energy-aware arm selection + base placement (Phase 5)

`gantry_reach_executor` turns a detected object into an actual MoveIt plan,
deciding **which arm** (arm_1 vs arm_2, both on the *shared* gantry_1) and
**which 8-DOF goal config** by **energy**, not by nearest seed. GNG supplies the
seed/goal; MoveIt does the collision-aware planning (and, with `execute:=true`,
execution) against the live octomap. Per pick (`~/pick`, data = object index):

1. **Pool** every arm's GNG nodes within `pool_radius` (task-space) of the
   object — the reachability filter. Density-adaptive by default
   (`pool_radius_factor × node spacing`, ×2.5 → ~19 candidates), so the pool is
   independent of the GNG `lam`.
2. **Score** each pooled candidate by
   ```
   J = w_gantry_lin·d_gantry_lin + w_gantry_rot·d_gantry_rot
       + w_arm·d_arm + w_dist·ee_dist − w_manip·manip
   ```
   `d_*` = joint travel from the **current** state (gantry travel is the
   dominant, expensive term — the gantry is the heavy Modbus platform). The
   gantry's linear (prismatic, metres) and rotation (radians) axes carry
   **separate weights** because their units and cost differ. `ee_dist` = the
   arm's **current** tool-frame distance (via TF, Euclidean metres) to the
   object — one value **per arm**, so this term biases the allocation toward the
   arm whose end-effector is already nearer (set `w_dist=0` to disable). `manip`
   = node manipulability. (The per-node task-space `dist` still gates the pool
   and is logged for the rank-by-distance diagnostic; the gravity `hold` cost is
   CSV-only.) J is the objective, so a *farther* seed with lower J can still win.
3. **Search — round-robin across arms** (best-per-arm first, so a failing arm
   can't monopolise all `max_attempts` before the other is tried). Per candidate:
   IK to a **pre-grasp that stands off `box_clearance` m above the object's TOP**
   — a *dynamic, size-adaptive* height from `object_localizer`'s tracked box
   (a tall object's centroid sits inside its body, so a fixed centroid offset
   collides with the object's own octomap voxels → IK −31; standing off the top
   clears the whole gripper). Orientation = `grasp_orientation`, default top-down
   `(1,0,0,0)` (centroid gives no real orientation → identity quat is unreachable
   → −31; `grasp_yaw_samples` free yaws are tried). `ik_avoid_collisions` (default
   true) makes a *colliding* solution return −31 too; set false to A/B a −31 (if
   IK then passes, the −31 was a collision, not unreachable). The search runs
   **plan-only** so it never moves the arm — the **first** candidate that plans
   wins. `box_clearance` is live-tunable (`ros2 param set … box_clearance 0.15`).
4. **Execute once** (`execute:=true`): only the chosen candidate is
   planned-and-executed, then confirmed against the LIVE `/joint_states`
   (move_group's result can beat Isaac's slower physics). If execution aborts
   (e.g. env-change −3), **fall through to the next candidate** so the arm keeps
   trying. Perception is frozen for the whole pick (`pause_gate`: `collision_cloud`,
   `object_collision`, `octomap_refresher`, `reachability_cloud` all pause) so a
   mid-plan scene bump can't invalidate the plan.
5. **Log** per-pick CSV: chosen arm, J + components, **rank-by-J vs
   rank-by-distance**, resulting gantry placement, IK/plan time, optional
   trajectory energy.

**`pick_stack.launch.py` — perception + executor in one terminal.** Since the
perception nodes (Section 6) and the executor are always run together for a pick,
`pick_stack.launch.py` includes both, so only `launch_workcell.sh` (Isaac +
move_group + RViz) and the `pick_cli` menu are separate. **`launch_workcell.sh`
no longer starts `perception.launch.py`** — it lives in `pick_stack` so
perception/executor code can be restarted fast without rebooting Isaac.
```bash
# Terminal 1: Isaac + move_group + RViz (rarely restarted)
./isaac_sim/launch_workcell.sh
# Terminal 2: perception + executor (restart this after any perception/executor edit)
ros2 launch reachability_gng pick_stack.launch.py execute:=true box_clearance:=0.15
# Terminal 3: interactive picker (must be its own terminal — it reads the keyboard)
ros2 run reachability_gng pick_cli
```
Or drive just the executor (perception already up), plan-only, with CSV:
```bash
ros2 launch reachability_gng gantry_pick.launch.py csv:=/tmp/picks.csv        # plan-only
ros2 topic pub --once /gantry_reach_executor/pick std_msgs/String "{data: '0'}"
column -t -s, /tmp/picks.csv
```

**Grasp vs approach-only mode.** Default (`carve_target:=true`,
`allow_target_collision:=true`) = *grasp* mode: the target is carved out of the
octomap and ACM-allowed so the gripper may enter it. Set **both false** for
*approach-only*: the target stays a hard octomap obstacle and the EE only stands
off above it (`box_clearance`) without ever touching it.

**Interactive picker (`pick_cli`).** Lists the live objects from
`/detected_objects` (labels from `/detected_objects/markers`), shows a cheap
straight-line distance from each object to every arm's current tool frame (a
geometric proxy, **not** the executor's energy J), and fires a pick when you type
its index (which also publishes the object's label on `/grasp_target` so
perception carves/boxes it). Also drives the detector (`y`/`i`/`p` = YOLOE /
Isaac / set prompts) and accepts a natural-language request ("get me a box").
Watch the executor terminal for the chosen arm / plan result.

**Energy weights.** What drives the ranking is each term's *influence* =
`weight × its spread across the pool`. The calibrated defaults live in one place
only — the node's `declare_parameter` calls in `gantry_reach_executor.py`:
`w_gantry_lin=1, w_gantry_rot=1, w_arm=2, w_dist=1, w_manip=0` (the gantry linear
and rotation axes are tuned separately; `w_dist` rewards the arm whose current
end-effector is nearer the object; set `w_dist=0` to disable). The `hold` term
was removed from J (still logged to the CSV for reference). The launch does not
override them. Override live, e.g. `-p w_manip:=300` → picks more dexterous
configs; `-p w_manip:=0` → flips toward shorter-travel / lower-manip. Naming:
gantry/arm travel = *minimum-joint-travel* term, optional `∫|τ·q̇|dt` = true
*mechanical energy* (post-plan, `compute_traj_energy`).

**Holding cost.** `train.py --config <arm yaml>` annotates each node with
`hold = ‖gravity torque‖` at the node's own q (Pinocchio), stored in
`_stats.npz`. Without `--config`, `hold=0` (older maps stay compatible; augment
them in place rather than retraining, to keep a tuned map).

**Single-object regime.** The idle arm still *rides the shared gantry* — keep it
tucked/clear. Simultaneous **two-object** allocation over one gantry (joint base
placement + inter-arm collision) is the multi-arm extension, not implemented.

## Status

**Phase 1 (done):** GNG core (`gng.py`, unit-tested; incremental adjacency
index so it scales to thousands of nodes), FK sampler (`data_gen.py`), trainer
(`train.py`), seed lookup (`seed_server.py`), RViz viz (`visualize.py`).

**Phase 2 (done):** GNG-seeded MoveIt IK via `/compute_ik` (`seed_ik.py`,
`use_gng_seed` A/B switch). SRDF group `gantry_1_with_arm_1` (gantry_1 + arm_1,
8 DOF) and its KDL solver entry added in `workcell_moveit_config`.

**Phase 3 (done):** `eval.py` — `volume` (offline arm-only vs arm+table reach
gain) and `ik` (GNG seed vs no-seed vs KDL random-restart: success rate, solve
time, manipulability).

**Phase 4 (done):** RGBD perception → object reachability on the Isaac twin
(section 6). Two-camera ground-truth segmentation + localization in `world`,
per-object green/red classification + manipulability, and a shape-faithful
voxel reachability cloud with a per-object **% reachable by volume** metric
(camera-fused, jitter-debounced). Works for arbitrary object shapes.

**Phase 5 (done):** energy-aware arm selection + base placement
(`gantry_reach_executor`, section 7). Per-node `hold` (gravity) cost in
`_stats.npz`; `GNG.query_radius` pool retrieval; two-arm energy ranking
`J(d_gantry_lin, d_gantry_rot, d_arm, hold, manip)` with collision-aware MoveGroup plan/execute
and ranked-candidate fallback; per-pick CSV. Hardened for live execution: arm
round-robin fallback, a **size-adaptive top-down stand-off** (`box_clearance`
above the tracked object box), **plan-only search → execute-once** with
keep-trying-on-abort, and perception frozen per pick. **Open-vocab detection
(YOLOE)** via `seg_router` replaces the Isaac ground-truth masks at runtime.
Validated live on the Isaac twin (plan-only and real execution). Remaining here:
multi-object batch, weight study across scenes.

Remaining for the paper: a Zacharias-style voxel-capability-map baseline, the
table-aware node-separation ablation as a flag, and plotting scripts.

## Notes / gotchas

- **Cloud is config-independent.** It's the whole reachable workspace; the
  slider pose only moves the robot. At `t1_linear=0` the arm alone reaches a
  ~0.7 m sphere (X up to ~+0.4); the cloud extends to X≈+3.6 only because other
  samples slid the table out. That is correct.
- **GNG leaves a small boundary gap — fixed by boundary seeding.** Plain GNG
  nodes settle at Voronoi-cell centroids, so the node hull sits ~one half-cell
  inside the true reachable surface (measured ~0.26 m short in max radius at
  2668 nodes). This is why a genuinely-reachable object near the edge could score
  0% reach. `train.py --boundary-nodes N` (default `BOUNDARY=600` in
  `build_maps.sh`) fixes it: it detects the outer-surface FK samples purely by
  kNN one-sidedness (no voxels/scipy — the same enclosure idea the runtime gate
  uses), farthest-point-samples `N` of them into a fixed shell (`knn_edges`
  connectivity), and **pins** those nodes — never moved by adaptation, never
  pruned/deleted. The interior GNG then grows inside a hull anchored on the true
  surface (measured shortfall 0.000 m). Tune with `--boundary-tau` (higher =
  fewer, more clearly-outer points). Set `BOUNDARY=0` for the legacy map. Pinned
  nodes carry each surface sample's own `q`, so they double as valid IK seeds.
- **Joint limits** in `config/arm1_table1.yaml` match the URDF (verified). Table
  rail is sampled 0–3.0 m.

## Test
```bash
cd ros2_ws/src/reachability_gng && python3 -m pytest test/ -q
```
