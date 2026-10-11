# Keadaan kampanye P1 G43+ (docs/p1_campaign_prompt.md)

> Diperbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Fase LULUS tidak diulang.

## Fase aktif

**Fase 7** (OFFLINE, `docs/p1_campaign_prompt.md` FASE 7) mulai sesi berikut: 7a banding P1′-serial → 7b runner paralel (mock).
Fase 1–5 LULUS; Fase 6 dihentikan operator 2026-10-11 17:48 (6a–6c selesai, 6d 5/5 run 6/6).

## Gerbang

| Fase | Gerbang | Status | Bukti |
|---|---|---|---|
| 1 | seed lolos R180 ≥ 6, keenam seed G42f tetap lolos, makespan ≤ R35, predikat sama ketat | **LULUS** | `docs/p1_g43_rot180.md` B3; percepatan G46 `docs/p1_g46_sched_speed.md` |
| 2 | START-JUMP: kontrol positif ditolak, 100 % rencana HW tetap diterima | **LULUS** (putaran 2) | `docs/p1_g44_start_jump.md` B5, commit 6ab5370 |
| 3 | K0 terukur: semua kasus uji benar | **LULUS** | `docs/p1_g45_k0.md` B3, commit 4ad8394 |
| 4 | rehearsal mock: DRY-CLI 3/3 semua kandidat + smoke `--move` | **LULUS** | `docs/p1_g47_rehearsal.md` B6, commit bfc752e |
| 5 | SIAP HW | **LULUS** (operator 2026-10-10 20:12 menerima R1–R9) | `docs/p1_g48_hw_ready.md`, `docs/p1_hw_runbook.md`, commit d81c929 |
| 6 | HW (hanya bila operator "lanjut") | 6a–6c **LULUS**, 6d sebagian (5 run 6/6) | `docs/p1_g51_hw.md` B1–B15 |

## Keputusan operator

- 2026-10-10: rotasi TIDAK dibatasi ±10°; maksimalkan pemakaian rotasi (memori feedback-rotation-max-campaign-prompt).
- 2026-10-10 16:10 (operator): (1) batas mekanis rotasi tiap gantry **±180° bisa** (kabel/stopper/driver tidak membatasi);
  (2) operator **di lokasi besok 2026-10-11** → cek origin g2 dengan kamera (Fase 6a) besok; (3) arah sweep rotasi **aman
  semua** (+ dan −). Sweep HW tetap bertahap dan TIAP gerak tetap ditanya (aturan Fase 6).
- 2026-10-10 20:12 (operator, Gerbang 5): risiko R1–R9 DITERIMA; **lanjut Fase 6 besok mulai 6a** (cek origin g2 kamera),
  lalu 6b sweep (tiap kaki ditanya).

## ROT_NEED

**Sesudah G50 (17 jadwal siap-HW): g1 −125…+90°, g2 −35…+95°** (sweep v2 `results/p1_g50/g50_sweep.sh`). Lama:

**|rot| 125°, Δrot 95°** (s0 g1 (0.80, −125°)); tanpa s0 70° / 65°. Kandidat HW: s0 s3 s7 s10 s18 (R180, `results/p1_g43/g43_candidates_6.json`) + s8 jadwal G42f (`results/p1_g42/g42f_candidates.json`).

## Risiko terbuka

- HW rotasi terverifikasi hanya ±10° (G30). Origin g2 ≈ −35 mm (G36b) belum dicek kamera. g1 PARKIR 1.50.
- probe `--goal-joints` belum pernah `--move` (bahkan mock). Penyaring Fix-Start-State belum ada.

## Langkah berikut

**Fase 6 (G51) dihentikan operator 17:48 sesudah s0.** 6a–6c SELESAI (amplop HW g1 −125…+90, g2 −35…+95). 6d: s7 R180/R35,
s10 R180/R35, s0 R180 = 5/5 run 6/6, pulang + K0 LULUS; stack MATI (node 0), sel g1 PARK / g2 0 / REST. R180 lebih lambat di
runner SERIAL (s7 +2.3 %, s10 +24 %). Berikut (offline, tanpa HW): banding makespan terukur vs prediksi P1′-serial untuk 5 run,
dan putuskan perlu runner paralel (dua gantry bersamaan) agar keuntungan R180 bisa diukur; opsional: jadwal ulang 5 jadwal di R90
(pertanyaan operator ±90°). Sisa blok HW: s3, s18, s8 (+ R35) dan 11 jadwal R180.
