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

---

# `irm_cloud_pol.npz` — the cloud `canon` indexes into

34 MB, 500 000 samples. Committed here on **2026-08-15** (G12 U0) for exactly the
reason the capability maps were: it existed **only in `/tmp`** for all of G11,
and every arm polyline of that session is read against it.

🔴 **`canon` in the capability maps is an index into a cloud that is NOT stored
with them.** Read against the wrong cloud it yields arm polylines that look
entirely plausible and are wrong, with **no error signal**. The three values that
pin the provenance (`p1_g11 §B1`, re-proved as L1a):

| | |
|---|---|
| cloud | `irm_cloud_pol.npz`, md5 `d62c2f5427591ef99a46f76f69dc5c32` |
| approach filter | **45°** → 89 170 of 500 000 samples kept |
| canonical policy | `manip`, `k_cand = 64` |
| result | `masks` **and** `canon` rebuild **BIT-IDENTICAL** for both maps |

Fields: `poly` (500000, 3, 3) base-frame arm_link/wrist/tool, `axis`, `manip`,
`sigmin` (the only cloud that has it — this is how the right one was identified),
`q`, `frames`, `seed`.

`reachability_gng.sched_arm.CLOUD` resolves to this copy first and falls back to
`/tmp/irm_cloud_pol.npz`, so no restore step is needed. Verify at any time with:

```bash
python3 test/verify_sched_arm.py l1      # L1a must print IDENTICAL for g1 and g2
```
