# P1 / G47 — rehearsal mock penuh untuk kandidat R180 (Fase 4 kampanye)

> Kampanye Fase 4. NOL gerak HW; mock `ROS_DOMAIN_ID=77` (fake HW). Dasar: Gerbang 1 LULUS (G43, kandidat s0 s3 s7 s10 s18
> R180 + s8 jadwal G42f; ROT_NEED 125° / Δrot 95°), START-JUMP (G44), K0 (G45). **§A DIKUNCI SEBELUM mengukur** (salinan
> `docs/results/p1_g47/g47_A_locked.md` + sha256). §B menang atas §A dan DITULIS. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Pengungkapan

- Kode yang disiapkan (belum diuji): probe `node._plan_verdicts` (verdict TIAP percobaan rencana) → DRY JSON
  `plans[arm].verdicts`, jalur `--move` `plan_verdicts`; `g31_screen.task_cli` membawa `verdicts` (celah G42 B1);
  `pose_to_g --rot-max` (default 10 = amplop HW G30; S23″ target ≤ rot-max, terukur ≤ rot-max + 0.5; maks 175);
  `run_g32 --rot-max` (diteruskan ke `pose_to_g`; jadwal dengan |rot| > rot-max ditolak SEBELUM event 0).
- G42 DRY-CLI: 92.2 % percobaan pertama, 6/90 GOAL-MISMATCH (ujung OMPL di kotak sendi ±0.2°), semua lolos percobaan 2.
- Smoke mock lama (G36) memakai `run_g32` + monitor + perekam `p1_g22/js_record.py` di domain 77.

### A1. 🔒 Uji

1. **DRY-CLI k = 3** `g31_screen.py --task-cli --final-retract --repeats 3`: s0 s3 s7 s10 s18 (`g43_candidates_6.json`,
   varian `R180`) dan s8 (`g42f_candidates.json`, varian `G42`). Lolos ⟺ 3/3 PLANNED. Dicatat per tugas: verdict tiap
   percobaan (`verdicts`), GOAL-MISMATCH, START-JUMP.
2. **Smoke `--move` (fake HW, BUKAN data)**: mock dibawa ke REST + p0 (`return_rest`, `pose_to_g … 1.5 0`), monitor + perekam
   hidup; `run_g32 --final-retract --move --rot-max 125` pada **s0** (rotasi terbesar, 125°) dan **s7** (70°, gain
   terbesar). K0 G45 = prasyarat (di dalam run_g32). Dicatat: event selesai, verdict penilai monitor per tugas, percobaan
   rencana per tugas (`plan_verdicts`), GOAL-MISMATCH, torsi prediksi terketat (log probe), rc tiap alat. Antar seed: pulang
   ke REST + p0.
3. **A/B toleransi tujuan** HANYA bila GOAL-MISMATCH > 5 % percobaan di (1): tujuan sendi tugas ±0.05° (bukan ±0.2°) pada
   seed yang sama k = 3; dipakai hanya bila PLANNED tidak turun dan GOAL-MISMATCH turun.
4. Mock tidak dijalankan bersamaan job offline Pool 15 (G43 dihentikan/selesai dulu).

### A2. 🔒 Gerbang 4

DRY-CLI 3/3 untuk 6/6 kandidat; smoke s0 + s7: semua tugas SUCCESS di penilai, urutan event = event DRY-CLI, tidak ada
HALTED / penolakan akhir penyaring saat eksekusi (percobaan ulang yang lalu lolos dihitung, bukan penolakan).

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D390 | DRY-CLI: **6/6** kandidat 3/3. |
| D391 | Percobaan rencana pertama PLANNED ≥ **90 %** tugas. |
| D392 | GOAL-MISMATCH **4–9 %** percobaan (→ A/B dijalankan bila > 5 %). |
| D393 | **0** START-JUMP di DRY-CLI dan smoke. |
| D394 | Smoke s0 dan s7: **6/6 tugas SUCCESS** masing-masing, 0 HALTED. |
| D395 | `pose_to_g` di smoke menerima semua traverse (S12/S23″/G33/S28) termasuk −30 → −125° (s0). |
