# Keadaan kampanye P1 G43+ (docs/p1_campaign_prompt.md)

> Diperbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Fase LULUS tidak diulang.

## Fase aktif

**Fase 7** (OFFLINE, `docs/p1_campaign_prompt.md` FASE 7): 7a G52 LULUS, 7b G53 LULUS, 7c G54 LULUS; **Gerbang 7 menunggu
jawaban operator atas risiko P1–P6** (`docs/p1_hw_runbook_parallel.md` §5). Rencana awal 7a = G52: 7a jadwal ulang R90 (±90°) → 7b banding P1′-serial → 7c runner paralel (mock).
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
| 7a | tabel 17 jadwal R90 + DRY-CLI 3/3 jadwal baru | **LULUS** (6/6) | `docs/p1_g52_r90.md` B3–B5, commit 8abd81c |
| 7b | tabel + penjelasan selisih > 15 % | **LULUS** (2/3) | `docs/p1_g53_p1p_serial.md` B1–B4, commit f69a1e9 |
| 7c | runner paralel: A2.1, DRY 5/5, smoke 5/5, kontrol negatif | **LULUS** (4/6) | `docs/p1_g54_parallel.md` B1–B5 |
| 7 | runbook HW paralel + risiko → operator | runbook DITULIS; risiko P1–P6 **ditanyakan** | `docs/p1_hw_runbook_parallel.md` |

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

**Tanya operator: risiko P1–P6 runner paralel diterima? (runbook paralel §5).** Bila diterima → Gerbang 7 LULUS = kampanye SELESAI;
HW berikut (sesi lain, tiap gerak ditanya): uji stop-semua HW (runbook paralel §3), lalu s7 R180 vs R35 paralel sama-sesi, s10, s12 R90,
s11 R90 sesudah kaki sweep g2 → −85. Mock MATI bersih (node 0, crash dump sesi dihapus).
