# Capability maps — the single feasibility oracle for P1

`cap_g1_rail160.npz`, `cap_g2_rail160.npz` — one per gantry, 4.4 MB each.

`docs/p1_state.md §3` calls these **"satu-satunya oracle kelayakan"**: every
reachability, co-feasibility, handover and scheduling number in P1 (sessions
G5–G8) is derived from them. They were committed here on **2026-08-14** because
until then they existed **only in `/tmp`**, with no copy anywhere — a reboot
would have made all of G7/G8 unreproducible without re-running the 2M-sample
sweep.

## Grid

| axis | values | range | step | why |
|---|---|---|---|---|
| `lin` | 33 | 0.00 … 1.60 m | 0.05 m | rail end stop **measured** at ~1656 mm (Sesi A, 2026-08-13); 1.60 m is the operational limit |
| `rot` | 72 | −180° … +175° | 5° | full circle, **half-open** `[−180°, +180°)` — the axis is cyclic, so +180° would duplicate −180° |

`|P| = 33 × 72 = 2376` poses per gantry.

⚠️ These supersede the older `cap_g1.npz` / `cap_g2.npz` (41 × 72, rail 0–2 m).
Those contained **8 ghost columns** — gantry poses that do not physically exist,
because the URDF and `bridge.max_mm` claimed 2000 mm. Do not use or cite them.

## Restoring after a `/tmp` wipe

Every entry point defaults to `/tmp/cap_g{1,2}_rail160.npz`, so the cheapest fix
is to put them back:

```bash
cp ros2_ws/src/reachability_gng/data/cap_g*_rail160.npz /tmp/
```

Or override per invocation: `--map1 <path> --map2 <path>`.

## Regenerating from scratch (expensive — prefer restoring)

```bash
python3 -m reachability_gng.irm_sweep sweep --lin-step 0.05   # RAIL_MAX_M = 1.60
```

## Contents

`nodes`, `edges`, `masks` (multi-tolerance), `canon` (canonical config index),
`lin`, `rot`, `tols`, `gantry`. Loaded via
`reachability_gng.capability.CapabilityMap.load(path)`.

Arm slot 1 of a gantry is slot 0 rolled by half the rotation axis
(`np.roll(mask, 36, axis=rot)`) — exact, not an approximation.
