# P1 / G50 — rehearsal mock 11 jadwal baru (G43 s2 s11 s14 s16 s17, G49 s4 s5 s6 s9 s12 s13)

> Sesi 2026-10-11 pagi (operator: "lakukan DRY-CLI dan smoke di mock untuk 11 jadwal baru, siap pakai saat HW"). NOL gerak
> HW; mock `ROS_DOMAIN_ID=77`. Pola = G47 (Gerbang 4). **§A DIKUNCI SEBELUM mengukur** (salinan
> `docs/results/p1_g50/g50_A_locked.md` + sha256). §B menang atas §A. (Dokumen HW Fase 6 pindah ke G51.)

## A. Protokol — DIKUNCI

### A0. Pengungkapan

- 11 jadwal R180 lolos (i)–(iv″) (G43 B, G49 B2): makespan 191–470 s, |rot| ≤ 110°, Δrot ≤ 105°, (iii) tanpa konflik.
- Kebutuhan rotasi per gantry 11 jadwal: **g1 −110…+90°, g2 −35…+95°** → melebihi rencana sweep G48 (g1 −125…+70, g2 ±35)
  di g1 +85/+90 dan g2 +50/+55/+95.
- Alat = G47: `g31_screen.py --task-cli --final-retract --repeats 3` dengan `G47_GOAL_TOL_DEG=0.05`; smoke `run_g32
  --final-retract --move --rot-max 125 --goal-tol-deg 0.05` lalu pulang `g47_home.sh` (retrace) + K0.

### A1. 🔒 Uji

1. DRY-CLI k = 3 untuk 11 seed (plan `p1_g43/g43_candidates.json` varian `R180` untuk 2 11 14 16 17;
   `p1_g49/g49_candidates.json` varian `R180` untuk 4 5 6 9 12 13). Lolos ⟺ 3/3.
2. Smoke `--move` (fake HW, BUKAN data) untuk SETIAP seed yang lolos (1); pulang retrace + K0 sesudah tiap seed.
3. Rencana sweep diperluas (`g48_sweep_plan.json`/`legs`, versi baru disimpan, lama tidak ditimpa): stasiun dari EdgeMatrix
   dengan sapuan 0 → kebutuhan CLEAR; rehearsal mock kaki-kaki tambahan.
4. Mock tidak bersamaan job offline Pool 15.

### A2. 🔒 Gerbang (siap HW untuk 11 jadwal)

DRY-CLI 3/3 dan smoke semua tugas SUCCESS, 0 HALTED, pulang retrace OK (K0 LULUS) per seed; sweep diperluas lolos mock.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D420 | DRY-CLI **11/11** seed 3/3. |
| D421 | PLANNED percobaan pertama ≥ **97 %** tugas (0.05°). |
| D422 | 0 START-JUMP, 0 START-UNSCREENED. |
| D423 | Smoke **11/11** seed semua tugas SUCCESS, pulang OK. |
| D424 | Ada stasiun sweep CLEAR untuk g1 0 → +90 dan g2 0 → +95. |
