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

## B. Hasil terukur

Mock 77 bersih (launch SIG_DFL, SigIgn `0x1001001`, 7/7 controller, tanpa node ganda), REST + (1.50, 0)/(0, 0), K0 LULUS,
monitor + perekam hidup; tidak ada job offline bersamaan.

### B1. DRY-CLI — [log](results/p1_g50/g50_run.log), [JSON](results/p1_g50/g50_drycli.json)

06:35–07:41. **11/11 seed 3/3** (s2 s11 s14 s16 s17 dari G43, s4 s5 s6 s9 s12 s13 dari G49). 198 tugas, 198 percobaan:
**198 PLANNED percobaan pertama (100 %)**, 0 GOAL-MISMATCH, 0 START-JUMP, 0 START-UNSCREENED; ujung median 0.35 mm, maks
0.97 mm → D420 ✓, D421 ✓, D422 ✓.

### B2. Smoke `--move` (fake HW, BUKAN data) — [run](results/p1_g50/g50_run.log), log per seed `results/p1_g50/g50_smoke_s*.log`, [arsip](results/p1_g50/smoke/)

07:41–09:41. **11/11 seed 6/6 tugas SUCCESS (66/66)**, 168/168 event rc 0, 0 HALTED; semua tugas `['PLANNED']` percobaan
pertama; 0 START-JUMP; torsi prediksi terketat median 90 %, maks 93 %. Pulang retrace (`g47_home.sh`) + **K0 LULUS 11/11**.
Makespan mock 300.6–592.0 s (tidak dinilai). → D423 ✓.

### B3. Sweep v2 — rencana `results/p1_g50/g50_sweep_{plan,legs}.json`, `g50_sweep.sh`

Kebutuhan rotasi 17 jadwal siap-HW: **g1 −125…+90°, g2 −35…+95°**. Stasiun G48 tetap CLEAR (EdgeMatrix): g1 lin 0.65 (0 → −125
dan 0 → +90, min 93.9 mm), g2 lin 0.15 (0 → −35 dan 0 → +95, min 253.5 mm) → D424 ✓. Kaki: g1 30 (±10…±70, +90, −90/−110/−125,
PARK), g2 20 (±10…±35, +50/+70/+95, (0, 0)). Log sweep kini menyimpan baris G33/S28 (G48 B1 terpotong).
- **Rehearsal mock sweep v2** ([g1](results/p1_g50/g50_mock_sweep_g1.log), [g2](results/p1_g50/g50_mock_sweep_g2.log)), 09:42–09:57:
  **50/50 kaki rc 0** (g1 30, g2 20), G33 sapuan min **86.6 mm** (semua CLEAR, margin 50 mm).

### B4. Papan skor D420–D424

| # | Dugaan | Hasil |
|---|---|---|
| D420 | DRY-CLI 11/11 3/3 | ✓ |
| D421 | PLANNED percobaan pertama ≥ 97 % | ✓ 100 % (198/198) |
| D422 | 0 START-JUMP / START-UNSCREENED | ✓ |
| D423 | smoke 11/11 SUCCESS + pulang | ✓ 66/66, K0 11/11 |
| D424 | stasiun sweep CLEAR sampai g1 +90 / g2 +95 | ✓ (+ mock 50/50) |

**Tally G50: 5/5. Gerbang (siap HW 11 jadwal): LULUS.** Kandidat HW kini **17 jadwal** (runbook §5). Mock dimatikan bersih
(node 0, tanpa crash dump). NOL gerak HW.
