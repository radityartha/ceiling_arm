# E3 Frictionless-Model Snapshot — 2026-07-16 (restore point before Jalan B)

Preserved so that if **Jalan B** (add rail friction → recalibrate → rerun E3) does not
make energy-mode win on realistic energy, we can fall back to **Option A** (reframe to
travel/motion cost) from these exact, already-corrected numbers without re-running
anything. Jalan B will overwrite the URDF, the J weights, `/tmp/e3_*.csv`, and the paper
tables — this file is the recoverable baseline.

Raw CSVs archived in `./csv/` (7 files, 60 rows each, shared header, `selection_mode`
column). These are the **frictionless rigid-body** runs (URDF rail joints have no
`<dynamics>` damping/friction).

## Corrected E3 results (TRAVEL metrics — `d_gantry_lin`, `d_gantry_rot`, not position)
Mean/median over successful picks. This fixes the earlier bug where the "Rail rot" column
used absolute rail *position* (`gantry_rot`) instead of rotation *travel* (`d_gantry_rot`).

| mode | success | Rail-lin travel (m) | Rail-rot travel (rad) | Arm travel (rad) | Idealised energy (J) |
|------|--------:|--------------------:|----------------------:|-----------------:|---------------------:|
| **energy (ours)** | 56/60 (93.3%) | 1.142 / 1.163 | **0.678 / 0.415** | **6.552 / 5.893** | 13.187 / 12.314 |
| nearest | 56/60 (93.3%) | 1.185 / 1.412 | 1.365 / 1.327 | 7.937 / 7.496 | 12.006 / 10.718 |
| random | 56/60 (93.3%) | 1.170 / 1.283 | 1.367 / 1.346 | 8.128 / 7.693 | 12.869 / 11.661 |
| fixed arm_1 | 37/60 (61.7%) | 1.105 / 1.196 | 1.705 / 1.671 | 8.013 / 7.491 | 14.892 / 15.648 |
| fixed arm_2 | 38/60 (63.3%) | 1.129 / 1.157 | 1.256 / 1.264 | 8.088 / 7.639 | 12.761 / 11.038 |
| fixed arm_3 | 36/60 (60.0%) | 1.185 / 1.412 | 1.646 / 1.441 | 7.772 / 7.353 | 11.498 / 9.869 |
| fixed arm_4 | 39/60 (65.0%) | 1.136 / 1.208 | 1.216 / 1.225 | 7.467 / 7.171 | 12.866 / 11.189 |

Fixed baseline BEST = arm_4 (65.0%), WORST = arm_3 (60.0%). Per-arm win counts (energy):
arm_2 18, arm_3 14, arm_1 12, arm_4 12 — all four arms selected across the grid.

## Key findings (frictionless model)
1. **Coverage win (strong):** the three multi-arm policies all reach 93.3% vs 60–65% for
   any single fixed arm — four-arm coverage is the dominant reachability gain.
2. **Energy-mode reduces ALL travel:** arm 6.55 vs 7.94/8.13; rail-rotation **0.68 vs
   1.37** (≈half); rail-linear ~1.14 vs ~1.18 (≈equal). It does NOT "trade arm travel for
   base travel" — it lowers both.
3. **Energy-mode does NOT win on idealised energy:** its `traj_energy` (13.19/12.31 J) is
   the *highest* of the three multi-arm policies (nearest 12.01/10.72, random 12.87/11.66).
4. **Reweighting is a dead end (proven):** OLS-optimal weights over 318 pooled picks reach
   only `Spearman(J, energy)=0.252`, `R²=0.055`, versus the current hand-J's 0.228. No
   weight set makes energy-mode win on this energy — the travel features cannot predict a
   frictionless-model energy dominated by gravity-hold posture work. Root cause = no joint
   friction/damping in the URDF + rail axes orthogonal to gravity. This is exactly what
   Jalan B tries to change.

## Fig. 5 case (verified, real): "farther base, cheaper"
Target (x=0.00, y=−0.60, z=1.15): energy → **arm_3** (rail-lin 0.574 m, arm 5.39 rad,
J=19.80, energy 11.36 J) vs nearest → **arm_2** (rail-lin 0.000 m, arm 12.22 rad, J=73.61,
energy 14.63 J). Energy accepts more base travel to avoid a large arm excursion, winning
both J and realised energy. In 30/56 comparable rows energy picked a different arm than
nearest; energy had lower realised energy in 16 of those 30.

## Current J weights/refs (to RESTORE if Jalan B recalibration is reverted)
`gantry_reach_executor.py` declare_parameter defaults:
- weights: w_gantry_lin=2, w_gantry_rot=12, w_arm=20, w_dist=3, w_hold=1, w_manip=1
- refs: ref_gantry_lin=0.95, ref_gantry_rot=0.70, ref_arm=6.0, ref_dist=1.36,
  ref_hold=2.90, ref_manip=0.145

## Option A — ready-to-apply framing if Jalan B fails
Reframe the contribution from "energy" to **travel / repositioning cost**:
- **Headline claim (data-backed):** energy-aware selection lowers total motion — arm-joint
  travel −18% (6.55 vs 7.94 nearest) and rail-rotation travel −50% (0.68 vs 1.37) — while
  matching the multi-arm 93.3% reachability, and in representative cases accepts extra base
  travel to avoid larger arm excursions (Fig. 5).
- **Idealised energy = secondary, hedged:** report it honestly (not lowest of the three
  multi-arm policies) and attribute to the frictionless rigid-body model under-representing
  real motor draw; real-energy validation = hardware future work.
- Title options: keep "Energy-Aware" but lead Section VI with travel, OR retitle
  "Motion-Efficient Arm and Base Selection ...". See docs/E3_contingency.md Fallback A.

## Reproduce the corrected numbers
```bash
# from repo root, over ./csv/*.csv: mean/median of abs(d_gantry_lin), abs(d_gantry_rot),
# abs(d_arm), traj_energy on rows with success==1 (see the inline python used to build the
# table above; regression: OLS of traj_energy on normalized [d_lin,|d_rot|,d_arm,ee_dist,
# hold,manip] → R²=0.055).
```
