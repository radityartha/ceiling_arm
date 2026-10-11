# Keadaan kampanye P1 G43+ (docs/p1_campaign_prompt.md)

> Diperbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Fase LULUS tidak diulang.

## Fase aktif

**G58–G60 OFFLINE** (prompt `docs/p1_g58_offline_prompt.md`, sesi 2026-10-12). G58 LULUS 08:04, G59 LULUS 08:31 (mock mati bersih).
G60 aktif: §A dikunci (`docs/p1_g60_excl_sched.md`, sha256 `7af94f7b…`), job `docs/results/p1_g60/g60_job.sh` (Pool 15) mulai 08:31
→ `p1_g60/g60_job.log` (dapat-dilanjut: tempel ulang job bila putus).
Sebelumnya: KAMPANYE G43–G54 SELESAI — Gerbang 7 LULUS 2026-10-11.

## Gerbang

| Fase | Gerbang | Status | Bukti |
|---|---|---|---|
| 1 | seed lolos R180 ≥ 6, keenam seed G42f tetap lolos, makespan ≤ R35, predikat sama ketat | **LULUS** | `docs/p1_g43_rot180.md` B3; percepatan G46 `docs/p1_g46_sched_speed.md` |
| 2 | START-JUMP: kontrol positif ditolak, 100 % rencana HW tetap diterima | **LULUS** (putaran 2) | `docs/p1_g44_start_jump.md` B5, commit 6ab5370 |
| 3 | K0 terukur: semua kasus uji benar | **LULUS** | `docs/p1_g45_k0.md` B3, commit 4ad8394 |
| 4 | rehearsal mock: DRY-CLI 3/3 semua kandidat + smoke `--move` | **LULUS** | `docs/p1_g47_rehearsal.md` B6, commit bfc752e |
| 5 | SIAP HW | **LULUS** (operator 2026-10-10 20:12 menerima R1–R9) | `docs/p1_g48_hw_ready.md`, `docs/p1_hw_runbook.md`, commit d81c929 |
| 6 | HW (hanya bila operator "lanjut") | 6a–6c **LULUS**, 6d sebagian (5 run 6/6) | `docs/p1_g51_hw.md` B1–B15 |
| 7a | tabel 17 jadwal R90 + DRY-CLI 3/3 jadwal baru | **LULUS** (6/6) | `docs/p1_g52_r90.md` B3–B5, commit 8abd81c |
| 7b | tabel + penjelasan selisih > 15 % | **LULUS** (2/3) | `docs/p1_g53_p1p_serial.md` B1–B4, commit f69a1e9 |
| 7c | runner paralel: A2.1, DRY 5/5, smoke 5/5, kontrol negatif | **LULUS** (4/6) | `docs/p1_g54_parallel.md` B1–B5 |
| 7 | runbook HW paralel + risiko → operator | **LULUS** (operator 2026-10-11: terima P1–P6) | `docs/p1_hw_runbook_parallel.md` |
| G55 | jadwal P1″ + seleksi sim; DRY-CLI + smoke paralel | **LULUS** (5/6) | `docs/p1_g55_par_sched.md` B5 |
| G56 | persiapan hari HW: injeksi gerak, fault, sweep v3, analisis P2 | 2/3 (D482 salah rumus) | `docs/p1_g56_hw_day.md` B3 |
| G58 | blok seed lama vs baru: smoke paralel mock 11/11 + tabel siap-HW | **LULUS** (5/5) | `docs/p1_g58_seed_block.md` B1–B3 |
| G59 | DEMO-OFF ulang (kode sekarang) + prompt DEMO-HW + cek kamera | **LULUS** (4/6; 2 cacat diperbaiki: regex runner, `g` ganda) | `docs/p1_g59_demo.md` B1–B4 |

## Keputusan operator

- 2026-10-10: rotasi TIDAK dibatasi ±10°; maksimalkan pemakaian rotasi (memori feedback-rotation-max-campaign-prompt).
- 2026-10-10 16:10 (operator): (1) batas mekanis rotasi tiap gantry **±180° bisa** (kabel/stopper/driver tidak membatasi);
  (2) operator **di lokasi besok 2026-10-11** → cek origin g2 dengan kamera (Fase 6a) besok; (3) arah sweep rotasi **aman
  semua** (+ dan −). Sweep HW tetap bertahap dan TIAP gerak tetap ditanya (aturan Fase 6).
- 2026-10-11 (operator, Gerbang 7): risiko runner paralel P1–P6 DITERIMA. HW paralel = sesi lain, tiap gerak ditanya, mulai uji
  stop-semua (runbook paralel §3).
- 2026-10-10 20:12 (operator, Gerbang 5): risiko R1–R9 DITERIMA; **lanjut Fase 6 besok mulai 6a** (cek origin g2 kamera),
  lalu 6b sweep (tiap kaki ditanya).

## ROT_NEED

**Sesudah G50 (17 jadwal siap-HW): g1 −125…+90°, g2 −35…+95°** (sweep v2 `results/p1_g50/g50_sweep.sh`). Lama:

**|rot| 125°, Δrot 95°** (s0 g1 (0.80, −125°)); tanpa s0 70° / 65°. Kandidat HW: s0 s3 s7 s10 s18 (R180, `results/p1_g43/g43_candidates_6.json`) + s8 jadwal G42f (`results/p1_g42/g42f_candidates.json`).

## Risiko terbuka

- HW rotasi terverifikasi hanya ±10° (G30). Origin g2 ≈ −35 mm (G36b) belum dicek kamera. g1 PARKIR 1.50.
- probe `--goal-joints` belum pernah `--move` (bahkan mock). Penyaring Fix-Start-State belum ada.

## Laporan akhir kampanye (G43–G54)

| Fase | Dokumen | Tally | Inti |
|---|---|---|---|
| 1 | G43 (+G46, G49) | — | R180: 17/20 seed lolos (i)–(iv″) (R35 6/20) |
| 2 | G44 | — | START-JUMP: kontrol positif ditolak, rencana HW diterima |
| 3 | G45 | — | K0 terukur prasyarat runner |
| 4 | G47 (+G50) | — | DRY-CLI 3/3 + smoke 17 jadwal |
| 5 | G48 | — | runbook HW serial, risiko R1–R9 diterima |
| 6 | G51 | — | HW: amplop g1 −125…+90 / g2 −35…+95; 5 run 6/6 (serial) |
| 7a | G52 | 6/6 | R90: 5/5 jadwal ulang ≤ 1.043× R180; 16/17 siap-HW (s11 butuh g2 −85) |
| 7b | G53 | 2/3 | HW serial dalam ±5 % P1′-serial; komponen saling menutup; P1″ LOO ≤ 4.84 % |
| 7c | G54 | 4/6 | runner paralel (mock): s7 R180 380 vs R35 464 s; s10 R180 453 > R35 394; stop-semua ≤ 0.5 s |

Dugaan G52–G54: 12/15 (G52 6/6, G53 2/3, G54 4/6). Gagal: D452 (c_rovh terlalu besar), D460 (move_group serial),
D463 (s0 mock/sim 1.201).

**Keadaan HW:** tidak ada gerak HW di Fase 7. Sel terakhir (G51): g1 PARK 1.50, g2 0, lengan REST, stack mati. Mock 77 mati bersih.

**Risiko sisa (diterima):** P1–P6 runner paralel; s11 R90 butuh kaki sweep g2 → −85; HW paralel belum pernah; penjadwal belum
memakai model P1″ + traverse eksklusif (s10 R180 kalah di runner paralel).

## Langkah berikut

Operator 2026-10-12: offline dulu, baru HW → sesi baru dengan prompt `docs/p1_g58_offline_prompt.md` (G58 blok seed, G59 DEMO, G60
penjadwal sadar traverse eksklusif); sesudahnya sesi HW dengan `docs/p1_g57_hw_prompt.md` (diperbarui di akhir G60).
