#### B7.A 🔒 Protokol + dugaan (SEBELUM bring-up)

Urutan = G35b B7 apa adanya, kode G36 (probe `retract_check`, S18 B&B, `pose_to_g` PARK): self-test env + interarm
(+ `--cross`) → remount_check → bring-up nyata (env_map:=true) → K2 jari → g1 PARK → 0 (DRY, `--move`; di luar makespan) →
plan-only NYATA k=3 R10 lalu R0 (`g36_planonly.sh`, sampel independen) → `g36b_run.sh` R0 DRY/`--move` → `g36b_home.sh`
R0hm R0 0 → R10 DRY/`--move` → `g36b_home.sh` R10hm R10 1.5 (PARKIR) → matikan. Gerbang plan-only gagal (< 3/3 salah satu
varian) → varian itu TIDAK dijalankan, operator ditanya. Fase per event = `p1_g35/hw_replay.phases()` pada log baru.

| # | Dugaan |
|---|---|
| D247 | Makespan R0 dalam **460–515 s** (G35b 629.67: antar-lengan −120…−130, retract −30, saring pulang +12). |
| D248 | Makespan R10 dalam **480–550 s**. |
| D249 | **\|Δ(R10 − R0)\| ≤ 30 s**; tanda TIDAK diduga (G34b +66 dikurangi hemat rotasi G35 ~71 → ≈ −5; ulang sampel R10 menambah). |
| D250 | Plan-only NYATA **R10 3/3, R0 3/3**; RETRACT-BLOCKED terpicu **≥ 1×** di 3 sampel R10 (mock 0.8/sampel). |
| D251 | Eksekusi: R0 6/6 dan R10 6/6, 14/14 rc 0 per varian, nol auto-stop; **semua retract tak-kosong lurus** (MoveIt `return_rest` tidak terpicu), termasuk ev 11 R10 g2 @ −10°. |
| D252 | Fase antar-lengan per tugas HW **≤ 12 s** (6/6 tugas R0; G35b 24.1–37.6 s). |
| D253 | Torsi puncak terukur semua event **≤ 12 N·m** (G35b 10.72). |
| D254 | K2: `*_right_finger_bottom_joint` \|q\| ≤ 0.01 rad keempat lengan (G36 B6.1: 0.0001 / 0 / 0 / −0.0002). |

