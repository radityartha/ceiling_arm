# README_TOPO — the topological maps: build them, look at them, trust them

Task-focused companion to [README.md](README.md). That file documents the whole
package in pipeline order; this one answers the three questions that actually
come up when working with the maps:

1. **How do I build a map?** (§1 environment map · §2 action map)
2. **How do I look at it?** (§3)
3. **How much of it can I believe?** (§4 determinism · §5 cost)

Deeper background stays where it is — don't duplicate it: `README.md §8a`
(static/dynamic topo layers + `depth_cloud`), `§8b` (the validated real-hardware
capture procedure, incl. the duplicate-process check), `§8c` (live collision).
Method decisions and their evidence live in
[docs/p1_g5_msbl_gcs.md](../../../docs/p1_g5_msbl_gcs.md) and
[docs/p1_g6_map.md](../../../docs/p1_g6_map.md).

**Two different maps share the word "GNG" and are constantly confused.**
Keep them apart:

| | Environment map | Action map |
|---|---|---|
| Maps | the ROOM (xyz surfaces) | `xyz → q` (arm configurations) |
| Network | **MS-BL-GNG** (`bl_gng.py`) | **GCS** (`gcs.py`) or GNG baseline |
| Built by | `build_topo.sh` (needs cameras) | `build_maps.sh` (offline, no ROS) |
| Files | `/tmp/topo_static*.npz` | `/tmp/arm{1..4}_gcsx.npz` / `_model.npz` |
| Static or live | static capture + live `env_gng` | static, per arm |
| Cost | ~20 min per capture | ~3 min for all 4 arms |

The npz layouts are **identical**, so `GNG.load` will happily open a GCS or
MS-BL file and vice versa. Convenient (no consumer needed changing), and a
foot-gun: loading without error is not evidence you loaded the right thing.

---

## 1. Environment map — `build_topo.sh`

```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash

# default: 2 cameras fused, arm self-filter ON
ros2_ws/src/reachability_gng/build_topo.sh

# always worth it: keep the raw cloud too
SAVE_CLOUD=/tmp/topo_cloud_a.npz OUT=/tmp/topo_static_a.npz \
  ros2_ws/src/reachability_gng/build_topo.sh

# arms physically removed -> self-filter must be OFF (see gotcha 1)
SELF_FILTER=false SAVE_CLOUD=/tmp/topo_cloud_a.npz OUT=/tmp/topo_static_a.npz \
  ros2_ws/src/reachability_gng/build_topo.sh

# single camera, quick degraded check
ros2_ws/src/reachability_gng/build_topo.sh rgbd2
```

Knobs (defaults match the `map_topo_static` node):
`CAPTURE=8.0 MAX_NODES=1800 MAX_Z=1.75 MAX_X_FROM_CAMERA=2.5 SELF_FILTER=true`.
The RealSense drivers and `depth_cloud` are auto-started if their topics are not
already publishing, so no separate terminal is needed.

What happens to the points, in order — worth knowing, because every later
number inherits it:

```
/<ns>/depth_cloud (world frame, geometry only, no segmentation gate)
  -> crop      min_z / max_z / max_x_from_camera (per-camera, not a world band)
  -> voxel     leaf 0.02 m
  -> outlier   radius 0.05 m, min 3 neighbours
  -> self-filter (TF capsules around the 4 arms)        [if SELF_FILTER=true]
  -> accumulate CAPTURE seconds, merge, voxel again
  -> subsample to fit_max_points=12000, drawn from CONTENT-sorted order
  -> MS-BL-GNG fit  -> max_nodes, pinned  -> OUT
                                          -> SAVE_CLOUD (the FITTED pool)
```

⚠️ `SAVE_CLOUD` stores the **fitted pool**, i.e. everything above already
applied — not the raw sensor cloud. So "distance between two saved clouds" has a
**2 cm voxel noise floor** by construction, and must never be quoted as D455
depth accuracy.

### Why `SAVE_CLOUD` is not optional in practice

A capture needs a cleared scene, so re-running one costs a cleared room, not a
command. With the cloud on disk, every later question about that scene — a
different fit, an order-invariance replay, a quality comparison, a `grow` sweep —
costs nothing. Without it, each one costs another cleared room. Everything in §4
and §5 below was measured from saved clouds, with the cameras already off.

---

## 2. Action map — `build_maps.sh`

Offline, no ROS graph, no cameras.

```bash
# from the REPO ROOT (config 'urdf:' paths are repo-root-relative)
ALGO=gcs ros2_ws/src/reachability_gng/build_maps.sh   # -> /tmp/arm{1..4}_gcsx.npz
ros2_ws/src/reachability_gng/build_maps.sh            # -> /tmp/arm{1..4}_model.npz
```

Shared recipe: `N=80000 MAX_NODES=3000 LAM=60 EPOCHS=2 BOUNDARY=600`.
The two write to **different files** on purpose, so a GCS build can never
clobber the GNG baseline the ablation compares against — the only difference
between the two arms of that ablation must be the network.

`train.py --algo` still defaults to `gng`, and the runtime consumers still load
`arm{n}_model.npz`. That is deliberate: which one ships is a manuscript
decision, not a build decision.

---

## 3. Looking at a map

```bash
# map only
ros2 launch reachability_gng view_topo_static.launch.py \
    map_file:=/tmp/topo_static_a.npz

# map + live colour clouds  <- "does this map match the room?"
ros2 launch reachability_gng view_topo_static.launch.py \
    map_file:=/tmp/topo_static_a.npz with_cloud:=true

# ...and start the 2 cameras as well (only if nothing else has them)
ros2 launch reachability_gng view_topo_static.launch.py \
    map_file:=/tmp/topo_static_a.npz with_cloud:=true with_cameras:=true
```

One Ctrl-C stops everything. RViz renders on noVNC display `:1` —
`http://<pc-ip>:22380/vnc.html`.

Displays: `TopoStatic` (the map) · `ColorCloud rgbd`/`rgbd2` (what the room looks
like, **on**) · `DepthCloud rgbd`/`rgbd2` (**off**).

Turn `DepthCloud` on when a map has a hole. It is the only display that
distinguishes *the map missed it* from *the points were never there* —
`ColorCloud` cannot answer that, because it is not what was fitted.

Action map instead: `ros2 run reachability_gng visualize` (publishes
`~/gng_markers`). Note it calls `GNG.load`, which opens a `_gcsx.npz` without
complaint (see the layout note above) — correct output, but do not read it as
"GCS has its own visualiser".

---

## 4. Determinism — what is guaranteed, and what is not

Three levels, deliberately separated because they get conflated (definitions
locked in `docs/p1_g5_msbl_gcs.md §A1`):

| | Treatment | The real question |
|---|---|---|
| **D1** | same pool, same row order | is anything unseeded? |
| **D2** | same pool, **rows permuted** | **the real condition** — a recapture returns the same points in another order |
| **D3** | different/perturbed pool | stability against sensor jitter |

**The claim you may make is D2: "the static map does not depend on the order
samples are presented in."** Not "the map is identical between captures" — that
is D3, different point clouds, and no algorithm makes it identical.

Measured on a REAL capture (2026-08-13, cleared scene, arms removed, production
settings; `docs/p1_g6_map.md §B`):

| | GNG | MS-BL-GNG |
|---|---|---|
| **D2 bit-identical** | ❌ | ✅ |
| `max\|ΔW\|` under row permutation | **2.712 m** | **0.0** |
| D3 recapture (A↔B) mean-NN | 0.0478 m | **0.0377 m** |
| D3 recapture hausdorff | **0.5750 m** | 0.7947 m ⚠ |
| D3 95%-subsample mean-NN | 0.0257 m | **0.0206 m** |
| map drift ÷ sensor noise | 2.035× | **1.606×** |
| QE_mean · cov@5cm | 0.0352 m · 0.866 | **0.0305 m** · **0.945** |

Sensor noise floor for that room, measured first so map drift is attributable at
all: `cloud_a ↔ cloud_b` = 0.0235 m mean-NN.

Two things this table is not allowed to say:

- **`n_comp` (5 → 10) is neither better nor worse.** There is no ground truth for
  the component count of a real scene. Where ground truth exists (synthetic S1,
  which must be 2), MS-BL is correct.
- **MS-BL is not uniformly more stable.** It moves the *whole* map less (mean-NN
  wins everywhere) but its *furthest* node can move more (hausdorff loses on
  recapture ⚠). The same split showed up on synthetic S1/S4 in Session C.

Reproduce, from saved clouds, no cameras:

```bash
cd ros2_ws/src/reachability_gng/test
python3 analyze_field_capture.py --algo gng --json /tmp/g6_field.jsonl
python3 analyze_field_capture.py --algo bl  --json /tmp/g6_field.jsonl \
    --saved-map-a /tmp/topo_static_a.npz --saved-map-b /tmp/topo_static_b.npz
python3 bench_topo_determinism.py --algo bl --scene proxy   # synthetic scenes
```

`--saved-map-*` reuses the maps the capture already produced instead of refitting
them (~20 min each). It also makes the D2 check compare the *saved* map against a
permuted refit, so passing it proves the cloud/map pair on disk round-trips.

---

## 5. Cost — and the one knob that moves it

At production settings (`max_nodes=1800`, 12000 points): GNG **~25 s**, MS-BL
**~1400 s (~23 min)**. That ~40× is real and must be stated, not hidden.

**It is not a property of batch learning.** A batch pass is vectorised — that is
why `env_gng` got *cheaper* online when it moved to MS-BL (one matrix op per
perception tick instead of ~800 Python `step()` calls). It is a property of the
**growth cadence**: the source (`Meso-HSR/GNG.h`, *"add neuron per every it
iteration"*) adds **one node per batch**, so reaching `max_nodes` costs
`max_nodes` passes, while online GNG inserts every `lam` samples *within* a pass.

`fit(grow=k)` adds k nodes per batch — growth then needs `max_nodes/k` batches.
Measured on the real cloud (`test/bench_bl_grow.py`, production settings):

| grow | seconds | speedup | QE_mean | cov@5cm | D2 bit-identical |
|---|---|---|---|---|---|
| 1 (source cadence) | 1406.7 | 1.0× | 0.03048 | 0.945 | ✅ |
| 4 | 370.2 | **3.8×** | 0.03071 (+0.8%) | 0.940 | ✅ |
| 8 | 189.4 | **7.4×** | 0.03087 (+1.3%) | 0.941 | ✅ |
| 16 | 99.7 | **14.1×** | 0.03099 (+1.7%) | 0.936 | ✅ |

So the cost is ~linear in `1/grow` (14.1× at k=16, against a 16× ideal), quality
moves under 2%, node count stays exactly 1800 throughout, and — the part that
matters — **order-invariance survives the knob at every k**. D2 is the whole
reason MS-BL is here; a speedup that broke it would be worthless.

23 minutes → 100 seconds still passes the `QE ≤ 1.05×` and `cov@5cm ≥ −0.02`
parity thresholds against the GNG baseline, with room to spare.

**The shipping path is still `grow=1`.** These numbers say a change is available
and safe, not that it has been made; picking a default is a manuscript decision,
and the capture whose numbers are quoted in `docs/p1_g6_map.md` used `grow=1`.

Not yet measured, flagged as the next suspect: `learn_batch` allocates an
`(n, n)` co-activation matrix per batch (26 MB at n=1800) and scans it with
`np.triu`. That may be a large share of the remaining time and could go without
changing results at all.

---

## 6. Gotchas that have actually cost time

1. **`SELF_FILTER=false` when the arms are physically removed.**
   `robot_state_publisher` keeps broadcasting all four arm TFs from the URDF
   whether or not the arms exist, so the self-filter resolves and punches
   arm-shaped holes in a **real** cloud at **phantom** arm poses. Related failure
   from the other direction (2026-07-31): the self-filter silently no-ops when
   arm TF is missing — it now warns loudly instead.
2. **Check for duplicate processes before trusting any capture.** Two drivers on
   one camera serial, or two `static_transform_publisher`s on one frame, and ROS
   silently picks one. Command in `README.md §8b` step 0.
3. **Don't start a second RealSense driver over a running one.** A node that
   cannot find its serial retries forever, contends for the USB bus, and takes
   the *other* camera offline too. Hence `with_cameras:=false` by default in §3.
4. **`map_topo_static` waiting forever looks exactly like "still capturing".**
   Usually the cameras publish nothing to deproject; check `/tmp/realsense_dual.log`
   and `/tmp/depth_cloud.log`.
5. **A correct import and a correct function do not prove they are connected.**
   For several hours the repo claimed the static layer ran MS-BL while the node
   still called the GNG path (`docs/p1_g5_msbl_gcs.md §B7c`). Check what the
   *node calls*, not what the *module imports*.
