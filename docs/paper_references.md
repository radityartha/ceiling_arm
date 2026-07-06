# Paper References — master bibliography + per-section mapping

Companion to [experiment_plan.md](experiment_plan.md). Central citation list for the
IEEE paper *"Energy-Aware Arm and Base Selection for a Ceiling-Rail Dual-Arm Robot
Using a Base-Aware GNG Capability Map."*

**Rules for this file**
- Every entry below appeared in a real web search (July 2026). None are fabricated.
- `⚠verify` = title/venue/arXiv id confirmed, but full author list / pages / exact
  venue must be checked against the source before camera-ready.
- Bias to **recent (≥2024)** for relevance; keep the 3 foundational classics
  (Zacharias / Yoshikawa / Fritzke) that anchor the method.
- Do not write a non-trivial claim without a tag from this list. If no source
  exists yet, mark `[NEED CITE: ...]` in the draft rather than inventing one.
- A tag's author-year label (e.g. `Lisondra2025`) MUST match the verified first
  author. Never guess a surname for the tag — verify the author first, or use a
  neutral tag. (Caught once: a `Kim2025` tag whose real authors were Lisondra et al.)

---

## Master list (BibTeX-style tags)

### Foundational (must-cite classics)
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `Zacharias2007` | F. Zacharias, C. Borst, G. Hirzinger, "Capturing robot workspace structure: representing robot capabilities," *IEEE/RSJ IROS*, pp. 3229–3236, DOI 10.1109/IROS.2007.4399105. ✅verified (dblp) | 2007 | canonical capability map (arm-only, binary/voxel) |
| `Yoshikawa1985` | T. Yoshikawa, "Manipulability of Robotic Mechanisms," *Int. J. Robotics Research* 4(2), pp. 3–9. ✅verified (SAGE) | 1985 | manipulability index w=√det(JJᵀ) |
| `Fritzke1995` | B. Fritzke, "A Growing Neural Gas Network Learns Topologies," *Advances in NIPS 7*, MIT Press, pp. 625–632 (presented 1994). ✅verified (NeurIPS proc.) | 1995 | GNG algorithm |

### Reachability / capability / base placement (recent)
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `Makhal2018` | A. Makhal, A. K. Goins, "Reuleaux: Robot Base Placement by Reachability Analysis," *IEEE Int. Conf. Robotic Computing (IRC)* (arXiv:1710.01328). ✅verified (dblp) | 2018 | inverse-reachability base placement (standard baseline ref) |
| `MoMaPos2024` | B. Shao, N. Cao, Y. Ding, X. Wang, F. Gu, C. Chen, "MoMa-Pos: An Efficient Object-Kinematic-Aware Base Placement Optimization Framework for Mobile Manipulation," arXiv:2403.19940 (ICRA 2025). ✅verified | 2024 | object-aware base placement, repositioning cost |
| `BaSeNet2024` | L. Naik, S. Kalkan, S. L. Sørensen, M. B. Kjærgaard, N. Krüger, "BaSeNet: A Learning-based Mobile Manipulator Base Pose Sequence Planning for Pickup Tasks," IEEE/RSJ IROS 2024; arXiv:2406.08653. ✅verified | 2024 | base pose sequence, travel cost |
| `InvReach2022` | T. Sandakalum, N. X. Yao, M. H. Ang, "Inv-Reach Net: Deciding mobile platform placement for a given task," IEEE-RAS Humanoids 2022 (IEEE Xplore 10000186). ✅verified | 2022 | inverse-reachability base placement, obstacle-aware |
| `RM4D2024` | M. Rudorfer, "RM4D: A Combined Reachability and Inverse Reachability Map for Common 6-/7-axis Robot Arms by Dimensionality Reduction to 4D," arXiv:2410.06968 (ICRA 2025). ✅verified | 2024 | recent reachability-map representation |

### Energy / efficiency
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `EnergySLR2024` | A. Gupta, "Energy Efficiency in Robotics Software: A Systematic Literature Review (2020–2024)," arXiv:2508.12170 (pub. 2025; review scope 2020–2024). ✅verified | 2025 | frames energy-awareness as timely |

### Perception (enabler)
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `Wang2025` | A. Wang, L. Liu, H. Chen, Z. Lin, J. Han, G. Ding, "YOLOE: Real-Time Seeing Anything," *ICCV* 2025; arXiv:2503.07465. ✅verified | 2025 | open-vocab detection+segmentation front end |

### Simulation platform
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `IsaacSim2026` | S. Gao, M. Pagnucco, T. Bednarz, Y. Song, "NVIDIA Isaac Sim: Enabling Scalable, GPU-Accelerated Simulation for Robotics," arXiv:2606.03551. ✅verified | 2026 | cite for the sim platform (survey) |
| `Mittal2023` | M. Mittal, C. Yu, Q. Yu, J. Liu, N. Rudin, D. Hoeller, et al., "Orbit: A Unified Simulation Framework for Interactive Robot Learning Environments," *IEEE RA-L* 8(6). ✅verified | 2023 | Isaac Lab/ORBIT foundation (alt/added sim cite) |

### System software & hardware stack (Section III)
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `Coleman2014` | D. Coleman, I. Șucan, S. Chitta, N. Correll, "Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study," arXiv:1404.3785. ✅verified | 2014 | MoveIt planning framework |
| `Sucan2012` | I. A. Șucan, M. Moll, L. E. Kavraki, "The Open Motion Planning Library," *IEEE Robotics & Automation Magazine* 19(4), pp. 72–82. ✅verified | 2012 | OMPL motion planners |
| `Carpentier2019` | J. Carpentier, G. Saurel, G. Buondonno, J. Mirabel, F. Lamiraux, O. Stasse, N. Mansard, "The Pinocchio C++ library...," *IEEE/SICE SII*, pp. 614–619, DOI 10.1109/SII.2019.8700380. ✅verified | 2019 | FK + gravity-torque (map build) |
| `Kinova` | Kinova Robotics, "Gen3 lite robot — user guide / product specifications," kinovarobotics.com. (manufacturer ref, not peer-reviewed) | — | 6-DOF arm hardware spec |

### Learned IK (for "Why GNG" / Related Work contrast)
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `IKDiffuser2025` | Z. Zhang, Z. Jiao, "IKDiffuser: A Diffusion-based Generative Inverse Kinematics Solver for Kinematic Trees," arXiv:2506.13087. ✅verified | 2025 | contrast: learned IK vs GNG-seeded classical IK |
| ~~`IKReview2024`~~ | DROPPED — could not fetch MDPI page (403); metadata unconfirmed. Do not cite until verified. | — | — |

### Dual-arm coordination / selection
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `RoboPARA2025` | S. Duan, P. Ren, N. Jiang, Z. Che, J. Tang, Z. Fan, Y. Sun, W. Wu, "RoboPARA: Dual-Arm Robot Planning with Parallel Allocation and Recomposition Across Tasks," arXiv:2506.06683 (ICLR 2026). ✅verified | 2025 | dual-arm task allocation (contrast: we allocate by energy) |
| `DAGPlan2024` | Z. Gao, Y. Mu, J. Qu, M. Hu, S. Peng, C. Hou, L. Guo, P. Luo, S. Zhang, Y. Lu, "DAG-Plan: Generating Directed Acyclic Dependency Graphs for Dual-Arm Cooperative Planning," arXiv:2406.09953 (ICRA 2026). ✅verified | 2024 | dual-arm assignment |

### Service robotics context (motivation)
| Tag | Reference | Year | Note |
|-----|-----------|------|------|
| `Lisondra2025` | M. Lisondra, B. Benhabib, G. Nejat, "Embodied AI with Foundation Models for Mobile Service Robots: A Systematic Review," arXiv:2505.20503. ✅verified | 2025 | domestic service-robot frontier, human environments |
| `IFR2025` | International Federation of Robotics, *World Robotics 2025 – Service Robots* (Executive Summary). | 2025 | market growth / relevance stat |
| `Jayathilaka2025` | W. A. D. M. Jayathilaka et al., "A Novel Hybrid Robot Configuration for Enhanced Accessibility and Space Efficiency," Springer *Proceedings in Technology Transfer*, DOI 10.1007/978-981-96-1399-1_22. ✅verified (title/DOI/topic; ⚠full author list + exact year to confirm) | 2025 | ACADEMIC anchor: ceiling-suspended manipulator for ground objects, space efficiency (¶1) — REPLACES ToyotaTRI |
| ~~`ToyotaTRI2024`~~ | DROPPED as a citation — replaced by academic `Jayathilaka2025` per user. (IEEE Spectrum news; keep only as informal background if ever needed.) | 2024 | — |

---

## Per-section citation plan

**I. Introduction** — `IFR2025`, `Kim2025`, `ToyotaTRI2024` (¶1); `MoMaPos2024`,
`BaSeNet2024`, `EnergySLR2024` (¶2); `Zacharias2007`, `Makhal2018`, `MoMaPos2024`,
`PlaceNet2025`, `Yoshikawa1985` (¶3); `Fritzke1995`, `Yoshikawa1985`, `Wang2025` (¶4).

**II. Related Work**
- *Reachability/capability maps:* `Zacharias2007`, `Makhal2018`, `RM4D2024` — contrast:
  ours carries q+manip+hold per node (not binary voxel) and is base-aware.
- *Base placement:* `MoMaPos2024`, `BaSeNet2024`, `PlaceNet2025` — contrast: single-arm
  where-to-stand vs our which-arm-and-base under energy.
- *Redundancy resolution / manipulability:* `Yoshikawa1985`, `IKReview2024`.
- *Learned IK:* `IKDiffuser2025`, `IKReview2024` — why lightweight GNG seed over heavy
  learned IK models.
- *Dual-arm allocation:* `RoboPARA2025`, `DAGPlan2024` — contrast: task/temporal
  allocation vs energy-based arm selection.

**III. System** — `IsaacSim2026` / `Mittal2023` (sim), `Wang2025` (perception enabler).
Kinova Gen3 Lite / MoveIt / OMPL cites still `[NEED CITE]` — add when writing.

**IV. Base-Aware GNG Capability Map** — `Fritzke1995` (GNG), `Yoshikawa1985` (manip
layer), `Zacharias2007` (contrast to voxel capability map), `RM4D2024` (recent map alt).

**V. Energy-Aware Selection (J)** — `Yoshikawa1985` (manip term), `EnergySLR2024`
(energy framing), `MoMaPos2024`/`BaSeNet2024` (base-cost precedent).

**VI. Experiments** — `IsaacSim2026` (platform), `Wang2025` (E4/E5 perception),
`Makhal2018`/`Zacharias2007` (E2 voxel-seed baseline lineage).

**VII. Conclusion / Limitations** — reuse `IsaacSim2026` for sim-to-real discussion.

---

## Still `[NEED CITE]` (find before those sections)
- Kinova Gen3 Lite spec / official reference (III).
- MoveIt 2 (Coleman et al.) + OMPL (Şucan et al. 2012) for planning stack (III/V).
- Pinocchio (Carpentier et al.) for FK/gravity-torque computation (IV).
- Optional academic ceiling/overhead-manipulation paper to pair with `ToyotaTRI2024`.
