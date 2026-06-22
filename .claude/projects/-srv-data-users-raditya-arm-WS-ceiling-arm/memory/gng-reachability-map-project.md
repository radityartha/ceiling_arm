---
name: gng-reachability-map-project
description: Sensei-assigned project to build a GNG topological reachability/action-map IK for the ceiling-arm + moving-table cell
metadata:
  type: project
---

Sensei gave Raditya the `FRD-01/` reference: a Growing Neural Gas / Growing Cell Structures topological map of a rock-drill **boom's** reachable workspace, simulated in ODE (C++, header-as-source in `FRD-01/main.cpp`, `nRobot.h`). Nodes hold 7-D vectors (3 task xyz + 4 joint), trained from FK "teaching data" sampled by PD-driving the joints; edges form a roadmap; used as an "action map" = learned approximate IK + path roadmap. Also has a multi-arm "virtual boom" inflated-shell collision-risk layer (`RobotNo=5`).

Task: implement something **like that** for the workcell (4 Kinova Gen3 Lite + 2 moving tables). Decisions made 2026-06-21:
- Scope: **seed-MoveIt**, NOT full action-map IK. GNG map = global reachability/redundancy oracle (table placement + arm selection + IK seed); KDL/MoveIt closes the mm. (Reversed the earlier "full action-map IK" choice — building the map is identical either way; only the recall step differs, so deferring was free.)
- Novelty must come from **Naoyuki Kubota lab** research identity (GNG/topological mapping, computational intelligence, Informationally Structured Space, multi-objective behavior coordination, lifelong/online topological learning) — NOT from the IK, which is delegated to KDL.
- **Headline novelty chosen (2026-06-21): Unified ISS** — couple an environment-GNG grown online from the Livox `/livox/points` with the reachability/capability-GNG over the redundant table+arm config; the link between them resolves "what is reachable right now given obstacles + which table placement + which arm." Completes FRD-01's dormant `t=1` sensing-GNG mode. Fold multi-objective node attributes (manipulability, table-travel cost, collision-risk, joint-limit margin) in as the redundancy-resolution mechanism; online edge-aging as the lifelong property.
- OPEN RISK: Raditya is **unsure what prior Kubota-lab GNG/ISS code/papers exist**. Must confirm before committing — Kubota has many ISS/GNG publications; positioning the novelty against them is the immediate next step (ask sensei / lit review).

PUBLICATION TARGET (2026-06-21): **minimum Q2 journal** (realistic targets: Intelligent Service Robotics, J. Intelligent & Robotic Systems, Sensors/Applied Sciences SI; JRM/ALR are Kubota home venues but Q3/Q4 fallback). Q2 bar = novel mechanism + quantitative experiments + baselines + real-hardware validation.
- Validation: **sim (Isaac) + real-hardware demo** on the actual cell (Raditya has access) — targets upper-Q2/Q1.
- Task split: quantitative core = **pure reachability/planning benchmark** (coverage %, query latency, seed-MoveIt speedup vs cold MoveIt, recall error vs KDL); application demo = **pick-and-place / kitting** on real hardware. Polishing (`isaac_sim/workcell/polish.py`) = future-work teaser only.
- ISS centerpiece scenario: the **other table/arm moving = dynamic obstacle** seen by Livox env-GNG → online re-resolution of table placement/arm → MoveIt re-plan. This justifies the env-GNG and uses the unique 2-table + overhead-LIDAR hardware.
- Baselines to beat: fixed-base capability map (Reuleaux-style), fixed/random table placement, nearest-arm, cold MoveIt (no seed), static map.
- Scope cuts (future work, keep paper finishable): full simultaneous multi-arm scheduling, deep lifelong learning, learned-IK.

PhD CONTEXT (2026-06-21): Raditya is a PhD student in Kubota lab. Graduation requires **2 international journals (Q1 or Q2) + 1 international conference**; a 3rd journal is "very recommended". This Unified ISS system is the **fundamental/basic** of the whole PhD. Agreed thesis spine: "Coupled topological representations (Unified ISS) for perception-action integration in redundant multi-arm workcells — single-arm foundation → multi-arm coordination → adaptive/collaborative operation." One thread, one reusable codebase/hardware, papers = chapters.

Publication arc (sequenced by completion certainty, not ambition):
1. **Conference (Yr1, de-risk):** reachability-GNG capability map, single arm+table, seed-MoveIt, sim. Venues: IEEE SMC / IAS / IEEE-SICE SII / AROB.
2. **Journal 1 (Q2, foundation, graduation insurance):** full Unified ISS coupling, single arm+table, online dynamic-obstacle re-resolution, sim + real-HW demo. Targets: Intelligent Service Robotics, JIRS, Sensors. Publish FIRST + aggressively.
3. **Journal 2 (Q1 target, depth):** 4-arm + 2-table coordinated redundancy resolution + collision-aware scheduling over shared workspace. Targets IEEE T-ASE / Robotics & Autonomous Systems / T-Cybernetics; **Q2 fallback allowed (ISR/Sensors) — must not gate graduation.**
4. **Journal 3 (recommended upside):** pick one — lifelong/adaptive ISS, OR human-collaborative safety (Claude's pick, highest impact, Kubota "robot partners" lineage), OR task-level multi-objective sequencing.

PRIOR-ART SCAN RESULT (2026-06-21, automated web first-pass — NOT a substitute for Scholar/IEEE Xplore + lab archive + sensei):
- YELLOW FLAG. Individual pieces are ALL published: reachability/capability maps (Zacharias, RM4D, RichMap); **obstacle-aware base placement by coupling environment map + (inverse) reachability map = iDRM, Inv-Reach Net (arxiv 2410.21059), MoMa-Pos (arxiv 2403.19940), occupancy-grid+IRM fusion (MDPI Applied Sci 13/8510)** — THIS is the main threat, it's basically the naive "couple env + reach for base placement" framing; SOM/GNG for redundancy/IK (KSOM, Springer s41315-024-00360-z); multi-arm collision-aware scheduling w/ topological guidance (crowded).
- DO NOT claim as novel: "obstacle-aware reachability-based base placement", "reachability map", "neural-gas for IK".
- GENUINE GAP: (1) Kubota ISS/GNG lineage has only ever been mobile-robot nav / SLAM / point-cloud ISS — NEVER manipulator reachability (open within lab, but "novel-in-lab" is weak alone). (2) **Shared, actuated, rail-CONSTRAINED base coupling MULTIPLE arms** — all prior base-placement work is single-arm-on-one-free-wheeled-base. Raditya has 4 ceiling arms on 2 shared constrained moving tables (moving 1 table repositions 2 arms + changes shared-workspace collision geometry). This is the defensible novelty.
- REPOSITIONED NOVELTY: a *single unified GNG topological structure* jointly representing environment(ISS) + per-arm reachability + shared-workspace multi-arm coupling on constrained actuated bases; novelty = unification + shared-constrained-base multi-arm setting, NOT the coupling itself.
- IMPLICATION: Journal 1 (single-arm coupling) is at risk of overlapping iDRM/Inv-Reach Net — must foreground GNG-unification + moving-table specifics, or reframe. Multi-arm (Journal 2) is now both the Q1 lever AND the safest novelty → consider making it the thesis spine.
- TODO before writing: read iDRM, Inv-Reach Net, MoMa-Pos, KSOM-redundancy in depth; ask sensei (a) any lab GNG/ISS-on-manipulator work? (b) any multi-arm/shared-base work? (c) internal GNG/ISS codebase?

Risk rule: minimum degree = Conference + Journal 1 + Journal 2(Q1-or-Q2-fallback). Get Journal 1 ACCEPTED before sinking months into the ambitious multi-arm Journal 2. Journal 3 = upside, start only after 2&3 submitted. Prior-work check now protects the WHOLE thesis novelty — do it before Journal 1. Build the Python GNG + Isaac data-gen + ROS service as a reusable platform from day one.
- Platform: **Python + Isaac Sim** for FK/teaching-data generation; re-implement GNG in Python/NumPy (do NOT port the C++).

Key design stance I argued: model nodes in **table frame** (invariant to table motion) over `table_N_with_arm` redundant config (table_lin, table_rot, q1..q6); use GNG as global redundancy-resolver + coarse IK + roadmap, then **local-refine** (KDL/MoveIt IK seeded from node) for mm accuracy, since Kinova kitting/polishing needs more precision than the drill boom. Validate recall error against MoveIt IK for a quantitative result.

Open questions still to resolve before coding: position-only vs full 6-DOF pose metric; target task (kitting vs polishing — see `isaac_sim/workcell/polish.py`); whether Isaac scene has full workcell or only `single_arm`. Related: [[isaac-sim-setup]].
