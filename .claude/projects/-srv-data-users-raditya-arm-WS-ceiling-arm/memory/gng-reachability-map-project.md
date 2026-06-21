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
- Platform: **Python + Isaac Sim** for FK/teaching-data generation; re-implement GNG in Python/NumPy (do NOT port the C++).

Key design stance I argued: model nodes in **table frame** (invariant to table motion) over `table_N_with_arm` redundant config (table_lin, table_rot, q1..q6); use GNG as global redundancy-resolver + coarse IK + roadmap, then **local-refine** (KDL/MoveIt IK seeded from node) for mm accuracy, since Kinova kitting/polishing needs more precision than the drill boom. Validate recall error against MoveIt IK for a quantitative result.

Open questions still to resolve before coding: position-only vs full 6-DOF pose metric; target task (kitting vs polishing — see `isaac_sim/workcell/polish.py`); whether Isaac scene has full workcell or only `single_arm`. Related: [[isaac-sim-setup]].
