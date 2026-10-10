# Keadaan kampanye P1 G43+ (docs/p1_campaign_prompt.md)

> Diperbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Fase LULUS tidak diulang.

## Fase aktif

**Fase 4** (G47, `docs/p1_g47_rehearsal.md`) aktif. Fase 1–3 LULUS (G43+G46, G44, G45).
**Fase 3 LULUS** (G45). Persiapan Fase 4 (kode) paralel selagi G43 menghitung.

## Gerbang

| Fase | Gerbang | Status | Bukti |
|---|---|---|---|
| 1 | seed lolos R180 ≥ 6, keenam seed G42f tetap lolos, makespan ≤ R35, predikat sama ketat | **LULUS** | `docs/p1_g43_rot180.md` B3; percepatan G46 `docs/p1_g46_sched_speed.md` |
| 2 | START-JUMP: kontrol positif ditolak, 100 % rencana HW tetap diterima | **LULUS** (putaran 2) | `docs/p1_g44_start_jump.md` B5, commit 6ab5370 |
| 3 | K0 terukur: semua kasus uji benar | **LULUS** | `docs/p1_g45_k0.md` B3, commit 4ad8394 |
| 4 | rehearsal mock: DRY-CLI 3/3 semua kandidat + smoke `--move` | — | |
| 5 | SIAP HW | — | |
| 6 | HW (hanya bila operator "lanjut") | — | |

## Keputusan operator

- 2026-10-10: rotasi TIDAK dibatasi ±10°; maksimalkan pemakaian rotasi (memori feedback-rotation-max-campaign-prompt).
- 2026-10-10 16:10 (operator): (1) batas mekanis rotasi tiap gantry **±180° bisa** (kabel/stopper/driver tidak membatasi);
  (2) operator **di lokasi besok 2026-10-11** → cek origin g2 dengan kamera (Fase 6a) besok; (3) arah sweep rotasi **aman
  semua** (+ dan −). Sweep HW tetap bertahap dan TIAP gerak tetap ditanya (aturan Fase 6).

## ROT_NEED

**|rot| 125°, Δrot 95°** (s0 g1 (0.80, −125°)); tanpa s0 70° / 65°. Kandidat HW: s0 s3 s7 s10 s18 (R180, `results/p1_g43/g43_candidates_6.json`) + s8 jadwal G42f (`results/p1_g42/g42f_candidates.json`).

## Risiko terbuka

- HW rotasi terverifikasi hanya ±10° (G30). Origin g2 ≈ −35 mm (G36b) belum dicek kamera. g1 PARKIR 1.50.
- probe `--goal-joints` belum pernah `--move` (bahkan mock). Penyaring Fix-Start-State belum ada.

## Langkah berikut

G47 (Fase 4): DRY-CLI 0.2° 6/6 kandidat 3/3 (putaran 2 sesudah perbaikan kirim-ulang `start_valid`); A/B 0.05° dipakai
(GOAL-MISMATCH 0/36). Berjalan: `results/p1_g47/g47_drycli3.sh` (s0 s3 s10 s8 dengan 0.05°, log `g47_drycli3.log`). Lalu smoke
`bash results/p1_g47/g47_smoke.sh 0 <abs>/p1_g43/g43_candidates_6.json R180` dan seed 7 (mock harus di REST + p0; skrip
memulangkan sesudahnya). Mock 77 + monitor + perekam hidup (scratchpad g47_*). Sesudah Fase 4: G43 14 seed sisa malam ini.
