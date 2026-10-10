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

## B. Hasil terukur

### B1. DRY-CLI putaran 1 — [log](results/p1_g47/g47_drycli.log), [JSON](results/p1_g47/g47_drycli.json)

Mock 77 bersih (launch SIG_DFL, SigIgn `0x1001001`, 7/7 controller), REST + (1.50, 0)/(0, 0), K0 LULUS, monitor + perekam
hidup; job G43 dihentikan dulu (CPU bebas). 18:05–18:29.

- **s0 (rot 125°) 3/3, s7 3/3, s18 3/3**; s10, s3, s8 masing-masing gagal SATU tugas dengan **`START-UNSCREENED`** (sampel 2,
  2, 1). Per percobaan: 73 tugas, 79 percobaan — PLANNED 70, **GOAL-MISMATCH 6 (7.6 %)**, START-UNSCREENED 3; START-JUMP 0;
  PLANNED percobaan pertama 65/73 = **89.0 %**.
- **Diagnosa START-UNSCREENED (data):** log move_group berisi tepat **3** baris `failed to send response to
  /check_state_validity (timeout): client will not receive response` — respons pertama dari proses probe BARU hilang karena
  pembaca respons klien belum cocok (race DDS); `start_valid` mengirim sekali dan menunggu 10 s (11.1 s tercatat). Galat
  alat saya (G44), bukan verdict validitas. **Perbaikan (putaran 2):** `start_valid` mengirim ulang ≤ 3× (10 s tiap kali);
  tetap tidak ada jawaban → `START-UNSCREENED` (tetap menolak). Uji ulang: s10, s3, s8 k = 3 (seed yang lolos tidak diulang).
- GOAL-MISMATCH 7.6 % > 5 % → **A/B A1.3 wajib**: tujuan sendi TUGAS ±0.05° (retract REST tetap ±0.2°) lewat flag probe baru
  `--goal-tol-deg` (default 0.2 = argv lama), di-set untuk DRY-CLI lewat env `G47_GOAL_TOL_DEG`; seed A/B = s7 + s18 (lolos
  di putaran 1: 36 tugas, 41 percobaan, 5 GOAL-MISMATCH) k = 3.

### B2. Putaran 2 + A/B — [log](results/p1_g47/g47_drycli2.log), [JSON](results/p1_g47/g47_drycli2.json), [A/B JSON](results/p1_g47/g47_ab005.json)

- Dengan kirim-ulang `start_valid`: **s10 3/3, s3 3/3, s8 (G42f) 3/3**, 0 START-UNSCREENED → bersama putaran 1 (s0, s7, s18):
  **6/6 kandidat 3/3** pada toleransi 0.2°. GOAL-MISMATCH putaran 2: 5/59 percobaan (8.5 %).
- **A/B toleransi tujuan sendi tugas 0.05°** (s7 + s18, k = 3, sama seed dengan putaran 1):

| | tol 0.2° (putaran 1) | tol 0.05° |
|---|---|---|
| tugas / percobaan | 36 / 41 | 36 / **36** |
| GOAL-MISMATCH | 5 (12.2 %) | **0** |
| PLANNED percobaan pertama | 32/36 | **36/36** |
| ujung vs target median / maks | 1.24 / 4.19 mm | **0.37 / 1.12 mm** |

  PLANNED tidak memburuk, GOAL-MISMATCH turun → **dipakai** (`run_g32 --goal-tol-deg 0.05`; default probe tetap 0.2 =
  argv lama). Karena setelan berubah, s0 s3 s10 s8 diulang k = 3 dengan 0.05° (B3) sebelum smoke.

### B3. DRY-CLI tol 0.05° — [log](results/p1_g47/g47_drycli3.log), [JSON](results/p1_g47/g47_drycli3.json)

s0, s3, s10 (R180) + s8 (G42f): **4/4 seed 3/3**, 72/72 tugas PLANNED percobaan PERTAMA (0 GOAL-MISMATCH, 0 START-JUMP,
0 START-UNSCREENED), ujung median **0.31 mm**, maks 1.00 mm. Bersama A/B (s7, s18): **6/6 kandidat 3/3 pada setelan yang
dipakai (0.05°)**.

### B4. Smoke `--move` s0 (fake HW, BUKAN data) — [log](results/p1_g47/g47_smoke_s0.log), [arsip](results/p1_g47/smoke/)

- K0 LULUS, `rotasi jadwal maks 125.0 deg, --rot-max 125` diterima. **16/16 event rc 0, 6/6 tugas SUCCESS** (penilai monitor),
  termasuk traverse g1 (0.75, −30) → (0.80, −125) (ev 4, 29.8 s). Tiap tugas `plan_verdicts` = `['PLANNED']` (percobaan
  pertama), torsi prediksi terketat 82–90 % (j2/j3), ujung 0.20–0.81 mm, 0 START-JUMP. Makespan mock 434.30 s (waktu mock
  tidak dinilai: retract + muat penyaring). `run_g32` rc 134 = pola benign keluar.
- 🔴 **Temuan — rute pulang tidak dijamin jadwal:** langkah pulang langsung g1 (0.80, −125) → PARK (1.50, 0) **DITOLAK G33**
  (sapuan lin × rot terlalu dekat peta). Penjadwal menjamin hanya tepi ANTAR stop; pulang sesudah stop terakhir tidak pernah
  disaring. Di HW gantry bisa tertinggal berputar −125°. **Perbaikan:** `results/p1_g47/g47_home.sh` pulang dengan
  **menelusuri balik stop jadwal sendiri** (setiap kaki = tepi yang jadwal sudah pakai, persegi sapuan sama, CLEAR; S13″
  mengizinkannya) lalu p0 dan PARK; rute dimulai dari pose TERUKUR (sudah di home → tidak ada kaki; tidak di pose jadwal mana
  pun → TOLAK, rute tidak dikarang). Diuji: g1 (0.80, −125) → (0.75, −30) → (1.50, 0) OK, K0 LULUS. Versi pertama skrip
  menggerakkan g2 keluar-masuk padahal sudah di home (galat saya, mock) → diperbaiki sebelum smoke s7.

### B5. Smoke `--move` s7 — [log](results/p1_g47/g47_smoke_s7.log)

K0 LULUS (≥ 102 sampel/sendi), rotasi jadwal 70° ≤ 125. **18/18 event rc 0, 6/6 tugas SUCCESS**, semua `['PLANNED']`
percobaan pertama, torsi prediksi terketat 76–90 %, ujung 0.05–0.56 mm, 0 START-JUMP; makespan mock 515.30 s. Pulang
retrace: g1 (1.45, 70) → (1.55, 40) → (1.30, −15) → (1.50, 0), g2 (1.50, 5) → (1.50, −5) → (0, 0); K0 LULUS.

### B6. Papan skor D390–D395

| # | Dugaan | Hasil |
|---|---|---|
| D390 | DRY-CLI 6/6 3/3 | ✗ putaran 1 (3 START-UNSCREENED = galat alat B1) → ✓ putaran 2 dan 0.05° |
| D391 | PLANNED percobaan pertama ≥ 90 % | ✗ 89.0 % (0.2°); 100 % dengan 0.05° |
| D392 | GOAL-MISMATCH 4–9 % | ✓ 7.6 % |
| D393 | 0 START-JUMP | ✓ 0 (DRY-CLI 241 percobaan, smoke 12 tugas) |
| D394 | smoke s0 + s7 6/6 SUCCESS, 0 HALTED | ✓ |
| D395 | `pose_to_g` menerima semua traverse jadwal termasuk −125° | ✓ (pulang langsung ke PARK DITOLAK = di luar jadwal, B4) |

**Tally G47: 4/6.** **Gerbang 4 LULUS:** DRY-CLI 3/3 untuk 6/6 kandidat pada setelan dipakai (0.05°), smoke s0 + s7 12/12
tugas SUCCESS, event = event DRY-CLI, 0 HALTED / penolakan akhir.

## C. Keadaan akhir

NOL gerak HW. Mock 77 di REST + (1.50, 0)/(0, 0), monitor + perekam hidup. Kode: probe `start_valid` kirim-ulang ≤ 3×,
`--goal-tol-deg` + `TASK_GOAL_TOL_DEG`, `_plan_verdicts`; `run_g32 --goal-tol-deg / --rot-max`; `pose_to_g --rot-max`;
`g31_screen.task_cli` verdict per percobaan + env `G47_GOAL_TOL_DEG`; `results/p1_g47/g47_home.sh` (pulang retrace).
Perintah HW: `run_g32 … --final-retract --rot-max <ROT_NEED> --goal-tol-deg 0.05`, pulang dengan `g47_home.sh`.
