# P1 / G35 — OVERHEAD PENYARING LINGKUNGAN: ukur, percepat TANPA mengubah verdict (OFFLINE, nol gerak)

> Sesi G35, 2026-10-08. Lanjutan [p1_g34_rail_calib.md](p1_g34_rail_calib.md) §B7.4 (Δ R10 − R0 = +65.96 s vs model
> −50.87 s; traverse berotasi 55.3 / 81.1 s SEBELUM kirim), §B8.1 (G34b lupa mengunci dugaan).
> **§A ditulis dan DIKUNCI SEBELUM profil apa pun dijalankan.** §B diisi sesudah. §B menang atas §A; konflik DITULIS.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan, B8.1)

- Kode: `scripts/env_collision.py` (`EnvChecker.check` = 52 geometri × `coal.distance` ke octree 136 558 voxel per
  sampel; `screen_trajectory` = semua sampel, semua geometri, tanpa cache), `pose_to_g.py` (S28 `sweep_rot` lalu G33
  `EnvChecker().screen_trajectory(rect_points)`, EnvChecker dibangun per proses), `reach_dwell_probe.screen_env`
  (EnvChecker di-cache pada node, per proses), `return_rest.main` (EnvChecker per proses).
- **Satu log HW**: `hw/g34hw_R10_ev07_traverse.log` — stempel S28 → G33 = 846.61 → 891.28 (**44.7 s**, n 671 rect),
  start → S28 = 8.6 s. Dugaan D226 di bawah ditulis SESUDAH melihat angka ini (dibulatkan, bukan diuji ulang olehnya).
- Arsip lintasan: plan-only jsonl G33/G34/G34b (titik sendi lengan, 47–142 titik), V28 183 rencana (`v28_plans.jsonl.gz`),
  kontrol `g33_controls.py` (jendela keadaan TERUKUR G32-HW, `ec.check` per konfigurasi).

### A1. 🔒 Profil (offline, mesin yang sama dengan HW, tanpa ROS graph)

Per alat (`pose_to_g` traverse, `reach_dwell_probe` tugas, `return_rest` retract): waktu (a) impor + bangun EnvChecker
(URDF, 52 true_hull, octree), (b) per sampel `check`, (c) jumlah sampel dari log HW G34b. Rekonstruksi
pra-kirim = (a) + n × (b) + penyaring lain (S28 / antar-lengan / torsi / MoveIt diukur dari stempel log) dicocokkan
dengan B7.4 dan stempel log per event **±10 %**.

### A2. 🔒 Percepatan — hanya yang EKSAK

1. **Branch-and-bound Lipschitz** di `screen_trajectory`: d(geometri, sampel k) ≥ d(sampel j) − perpindahan maks
   titik geometri j→k (|Δt| + 2 sin(θ/2)·r_maks, r_maks dari verteks hull lokal). Pasangan (geometri, k) dilewati hanya
   bila batas bawah − slack > 0 DAN ≥ minimum sementara → minimum global, geometri, indeks = sama seperti kode lama.
   Badan diam (perpindahan 0) otomatis dihitung sekali.
2. Muat sekali / cache bila (a) dominan (mis. hull disimpan) — hasil geometri harus identik.
3. **Sampel sapuan TIDAK dijarangkan** (rect_points, titik MoveIt tetap). MARGIN = TOLAK tidak berubah.

### A3. 🔒 Regresi WAJIB (lama vs baru, berdampingan, mesin sama)

Semua lintasan arsip: plan-only jsonl G33 (R0/R10), G34 (reg3/g34/g34n/g34b_hw, R0/R10), V28 183, sapuan traverse
seed 36 R0/R10 (rect_points dari keadaan HW G34b), retract lurus G34b, + jendela kontrol `g33_controls` (P+, N−, R10
ev0–7) sebagai lintasan. Lulus bila **verdict identik 100 %**, |Δ d_min| ≤ 0.5 mm, geometri terburuk identik, dan
**(P+) ev 8 COLLIDE tetap, tolak ≥ 1.7 s sebelum kontak** (prefix). Gagal → kode lama dikembalikan, DITULIS.

### A4. 🔒 Dugaan (SEBELUM profil)

| # | Dugaan |
|---|---|
| D225 | Bangun EnvChecker 2–5 s per proses; hull (52 × scipy ConvexHull) > 50 % dari itu, octree < 1 s. |
| D226 | `check` per sampel (52 geometri) 50–80 ms, hampir sama antar alat (biaya = jumlah geometri, bukan jenis lintasan). |
| D227 | Rekonstruksi (a) + n×(b) + penyaring lain cocok dengan pra-kirim B7.4 per event ±10 %. |
| D228 | B&B eksak ≥ 5× pada sapuan traverse berotasi (671/946) dan ≥ 3× pada lintasan tugas (~128 titik). |
| D229 | Regresi A3 lulus 100 % (verdict, geometri, d_min bit-identik). |
| D230 | Sesudah percepatan: overhead lingkungan ≤ 10 s per traverse berotasi, ≤ 3 s per tugas; Δ(R10 − R0) prediksi **negatif**. |
| D231 | Sesudah percepatan, overhead terbesar yang tersisa BUKAN lingkungan (S28 / antar-lengan / start proses). |

## B. Hasil terukur

(diisi sesudah)
