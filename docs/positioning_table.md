# Positioning against related work

Draft source for **Table 1** of P1/P2. Built 2026-07-31.

**Evidence level per row** — how deeply the entry was checked:
`[F]` abstract fetched and read; `[S]` search-result summary only, abstract not
fetched; `[K]` background knowledge. **Every `[S]` and `[K]` row must be verified
against the full paper before this table goes into a submission.**

---

## Table A — system and hardware axes

| # | Work | Domain | Agents | Moving base | 2 agents share 1 base | Force/torque sensing assumed | Ev |
|---|------|--------|--------|-------------|----------------------|------------------------------|----|
| 1 | Reuleaux (Makhal & Gordon, 2017) | single manipulator | 1 | mobile base (planar) | n/a | none | [S] |
| 2 | Closed-Chain Manipulation of Large Objects (RA-L 2017) | multi-arm | 2+ | fixed | no | **required** | [S] |
| 3 | Adaptive hybrid position/force, dual-arm (J. Franklin Inst. 2017) | dual-arm | 2 | fixed | no | **required** | [S] |
| 4 | Cooperative Task Spaces via similarity transf. (2025) | multi-arm / humanoid / hands | 2+ | not addressed | no | not used | [F] |
| 5 | CLAS: Central Latent Action Spaces (2022) | multi-arm manipulation | 2 (dual-arm) | not specified | no | not mentioned | [F] |
| 6 | DCMRTA (ICRA 2024) | mobile robots | many | mobile, 1 per agent | no | n/a (no contact) | [S] |
| 7 | Dynamic Coalition Formation policies (2024) | mobile robots | many | mobile, 1 per agent | no | n/a (no contact) | [S] |
| 8 | LEGO equivariant GNN (2025) | swarm robots | variable | mobile, 1 per agent | no | **no contact at all** | [F] |
| 9 | Centralized Permutation Equivariant Policy (2025) | MARL benchmarks | variable | n/a | no | n/a | [S] |
| 10 | TRANSIC (2024) | single arm | 1 | fixed | contact-rich, no F/T claim | [S] |
| 11 | RoboTwin 2.0 (2025) | bimanual | 2 | fixed | no | not required | [S] |
| 12 | Dual-arm multi peg-in-hole (RCIM 2025) | dual-arm | 2 | fixed | no | **required** | [S] |
| — | **This work** | **multi-arm on gantries** | **4** | **2 linear+rotary gantries** | **YES, 2 arms per gantry** | **NONE — no joint torque sensor, no admittance mode** | — |

## Table B — method and evaluation axes

| # | Work | Learning layer | Energy as objective | Env representation | Transfer across agent count | Sim / Real | Ev |
|---|------|----------------|---------------------|--------------------|-----------------------------|-----------|----|
| 1 | Reuleaux | none (analytic) | no | voxel + sphere-discretised reachability map | n/a | sim + real | [S] |
| 2 | Closed-Chain Manipulation | none (model-based control) | no | none | no | mostly analysis | [S] |
| 3 | Adaptive hybrid pos/force | adaptive control, not RL | no | none | no | sim | [S] |
| 4 | Cooperative Task Spaces | none (geometric algebra) | no | none | partial (N-arm formulation) | unclear from abstract | [F] |
| 5 | CLAS | RL, **continuous latent action** | no | none | no | **sim only** | [F] |
| 6 | DCMRTA | RL, **discrete task allocation** | no | graph over tasks | yes | sim | [S] |
| 7 | Dynamic Coalition Formation | RL, discrete allocation | no | cross-attention over tasks | yes | sim | [S] |
| 8 | LEGO | RL, continuous motion | no | Euclidean proximity graph | **yes, zero-shot** | sim + real | [F] |
| 9 | Centralized Perm. Equivariant | RL (CTDE) | no | permutation-equivariant encoder | yes | sim benchmarks | [S] |
| 10 | TRANSIC | **residual policy** + human correction | no | none | n/a | sim + real | [S] |
| 11 | RoboTwin 2.0 | imitation / VLA benchmark | no | none | no | sim + real | [S] |
| 12 | Dual-arm peg-in-hole | hierarchical RL | no | none | no | sim + real | [S] |
| — | **This work** | **RL at allocation (macro) + residual (micro), embedded in the GNG graph** | **YES — measured J is both reward and metric** | **learned GNG topological map, two-layer static/dynamic** | **YES — permutation-equivariant, 2-arm → 4-arm** | **sim + real** | — |

---

## The gap, stated in one paragraph

Force-aware multi-arm co-manipulation (rows 2, 3, 12) is mature but uniformly
assumes force/torque sensing that a Gen3 Lite does not have. Coalition-formation
RL (rows 6, 7) and equivariant multi-agent policies (rows 8, 9) scale well across
agent counts but are evaluated on **mobile or swarm agents that are not
physically coupled** — no closed kinematic chain, and no notion of two agents
being forced to share one moving base. Reachability-based base placement (row 1)
is analytic, voxel-based, and single-arm. **No prior work combines all four of:
(i) a closed kinematic chain, (ii) agents constrained to share a moving base,
(iii) no force/torque sensing, and (iv) measured energy as the coordination
objective.** That intersection is this thesis.

## Axes where this work is strongest, in order

1. **Shared moving base (gantry).** Nothing in the surveyed set has it. Strongest
   and least contestable claim. Must not be dropped from the formulation.
2. **No force/torque sensing.** Turns a hardware limitation into the premise, and
   rows 2/3/12 become the motivation rather than competitors.
3. **Measured energy as objective.** Nobody else in the set optimises it.
4. **Learned topological (GNG) graph as the policy graph.** Row 8 uses a plain
   Euclidean proximity graph; using the environment's own learned topology is a
   genuine difference, and it is the lab-lineage contribution.
5. **4 arms.** Real but the weakest axis on its own — arm count alone is not a
   contribution, so never lead with it.

## Reviewer attacks to pre-empt

- *"Why not just add F/T sensors?"* → payload budget (0.5 kg/arm), cost x4, and
  the stated goal of a method that works on low-cost hardware. Show the
  current-based load-sharing estimate validated against the instrumented object.
- *"Is the gantry constraint not just a redundant DOF?"* → show the measured
  result that a naive world-frame intersection gives 0% co-manipulation
  feasibility while the gantry-local formulation finds a common reachable point
  at ~35% of test positions.
- *"Your RL is only doing discrete allocation."* → answer with the meso layer
  (continuous 4-arm coordination) plus the residual compliance layer, and with the
  measured sample-efficiency gain from action masking.
- *"Sim-only?"* → both papers ship real execution. Non-negotiable lab requirement.

## TODO before submission

- [ ] Verify every `[S]` row against the full paper; several force-sensing and
      sim/real cells are inferred from abstracts only.
- [ ] Add 2-3 rows from the dual-arm compliance review (Annual Reviews in Control
      2026) once read — it will name the strongest direct competitors.
- [ ] Fill in the DCMRTA row properly (PDF fetch failed, binary).
- [ ] Decide which subset of columns survives into the printed Table 1; 6 columns
      is the practical maximum for a two-column IEEE page.
