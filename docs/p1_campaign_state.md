# Keadaan kampanye P1 G43+ (docs/p1_campaign_prompt.md)

> Diperbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Fase LULUS tidak diulang.

## Fase aktif

**Fase 6** (HW) dijadwalkan 2026-10-11 mulai 6a. Fase 1–5 LULUS.
**Fase 3 LULUS** (G45). Persiapan Fase 4 (kode) paralel selagi G43 menghitung.

## Gerbang

| Fase | Gerbang | Status | Bukti |
|---|---|---|---|
| 1 | seed lolos R180 ≥ 6, keenam seed G42f tetap lolos, makespan ≤ R35, predikat sama ketat | **LULUS** | `docs/p1_g43_rot180.md` B3; percepatan G46 `docs/p1_g46_sched_speed.md` |
| 2 | START-JUMP: kontrol positif ditolak, 100 % rencana HW tetap diterima | **LULUS** (putaran 2) | `docs/p1_g44_start_jump.md` B5, commit 6ab5370 |
| 3 | K0 terukur: semua kasus uji benar | **LULUS** | `docs/p1_g45_k0.md` B3, commit 4ad8394 |
| 4 | rehearsal mock: DRY-CLI 3/3 semua kandidat + smoke `--move` | **LULUS** | `docs/p1_g47_rehearsal.md` B6, commit bfc752e |
| 5 | SIAP HW | **LULUS** (operator 2026-10-10 20:12 menerima R1–R9) | `docs/p1_g48_hw_ready.md`, `docs/p1_hw_runbook.md`, commit d81c929 |
| 6 | HW (hanya bila operator "lanjut") | — | |

## Keputusan operator

- 2026-10-10: rotasi TIDAK dibatasi ±10°; maksimalkan pemakaian rotasi (memori feedback-rotation-max-campaign-prompt).
- 2026-10-10 16:10 (operator): (1) batas mekanis rotasi tiap gantry **±180° bisa** (kabel/stopper/driver tidak membatasi);
  (2) operator **di lokasi besok 2026-10-11** → cek origin g2 dengan kamera (Fase 6a) besok; (3) arah sweep rotasi **aman
  semua** (+ dan −). Sweep HW tetap bertahap dan TIAP gerak tetap ditanya (aturan Fase 6).
- 2026-10-10 20:12 (operator, Gerbang 5): risiko R1–R9 DITERIMA; **lanjut Fase 6 besok mulai 6a** (cek origin g2 kamera),
  lalu 6b sweep (tiap kaki ditanya).

## ROT_NEED

**|rot| 125°, Δrot 95°** (s0 g1 (0.80, −125°)); tanpa s0 70° / 65°. Kandidat HW: s0 s3 s7 s10 s18 (R180, `results/p1_g43/g43_candidates_6.json`) + s8 jadwal G42f (`results/p1_g42/g42f_candidates.json`).

## Risiko terbuka

- HW rotasi terverifikasi hanya ±10° (G30). Origin g2 ≈ −35 mm (G36b) belum dicek kamera. g1 PARKIR 1.50.
- probe `--goal-joints` belum pernah `--move` (bahkan mock). Penyaring Fix-Start-State belum ada.

## Langkah berikut

**Malam 10-10 (operator: "analisis yang gagal, perbaiki"):** G49 `docs/p1_g49_exists_branch.md` — torsi per cabang terpakai (∃)
alih-alih ∀ cabang; K-SUP 40/40. Job `results/p1_g49/g49_make.py run` (seed 1 4 5 6 9 12 13 15 19; dapat dilanjut, log
`g49_run.log`). Sesudahnya: (iv′)/(iv″) seed baru, diagnosa sisa, §B G49, commit.
**2026-10-11, operator di lokasi:** ikuti `docs/p1_hw_runbook.md` §0–§3 (remount_check → TANYA bring-up nyata → K0 → kamera +
capture g2 origin, tanpa gerak → keputusan Δx, baseline rgbd −0.0936 m). Setiap gerak nyata DITANYA. Dokumen: `docs/p1_g50_hw.md`.
