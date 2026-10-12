# P1 / RGBD2-ZONE — koreksi planar ekstrinsik rgbd2 di zona meja demo arm_1 (penunjuk arm_1 + tanda unting + botol)

> Sesi 2026-10-12 malam, lanjutan [p1_arm1_zone.md](p1_arm1_zone.md) B6 F1 (galat +0.15 m = kamera rgbd2 jarak jauh). Operator:
> "ya lanjutkan"; model **3-DOF planar** (x, y, yaw world); **kalibrasi saja**, DEMO-HW tidak di sesi ini. §A dikunci (salinan
> `results/p1_rgbd2_zone/rgbd2_zone_A_locked.md` + sha256) sebelum gerak pertama. Data: [results/p1_rgbd2_zone/](results/p1_rgbd2_zone/).

## A. Protokol — DIKUNCI

### A0. Gerbang + luring + plan-check (NOL gerak)

**Gerbang operator:** K0 = akhir ARM1-ZONE, e-stop fisik tiap gerak, lampu, botol putih 7 cm, tanda M1 (1.75, 0.55) di meja, tak ada
task server ✓. Bring-up diizinkan: PID 464636, SigIgn `0x1001001`, 4× '6', 7/7 active, table1 + table2 ARMED, `env_static_map` True
(sesudah publish ulang), **K0 LULUS** (g1 1.49926 / 0°, g2 0.00090 / 0°), monitor + perekam `/tmp/p1rz_*` tumbuh.

| # | Isi |
|---|---|
| A0.1 | **Model.** Korespondensi = TRACK botol (rantai demo penuh: YOLOE → `object_localizer`, ekstrinsik incumbent) ↔ xy tanda (TCP arm_1 dari sendi terukur). Bias permukaan botol (kamera melihat sisi dekat) ikut terserap — itu yang dibutuhkan demo. T_corr = [Rz(θ), (tx, ty, 0)] di world; pose kamera baru = T_corr · T_lama → arg `tf2_*` (`static_transform_publisher`: R = Rz(yaw)Ry(pitch)Rx(roll) = scipy `'xyz'`). **Sah hanya di zona meja ini** (arah pandang + tinggi sama); di tempat lain bisa lebih buruk. Default launch TIDAK diubah; hasil = berkas harness BARU, dipakai hanya atas keputusan operator. |
| A0.2 | [rgbd2_zone_fit.py](results/p1_rgbd2_zone/rgbd2_zone_fit.py) `--self-test` PASS: konvensi Euler round-trip eksak; geser planar (2.5°, (−0.05, −0.20)) pulih eksak tanpa noise; noise 1 cm → θ meleset 0.9° tetapi uji-tahan di dalam zona 3.5 mm (→ zona-saja, A0.1); menggeser kamera dengan T_corr memindah titik world tepat sebesar T_corr. [log](results/p1_rgbd2_zone/self_test.log) |
| A0.3 | **Plan-check** (dari REST, z 1.22 tool-bawah, `--plan-attempts 8`, penyaring tak diubah) [logs](results/p1_rgbd2_zone/plancheck/): C2 (2.00, 0.55) PLANNED j3 92 %; C3 (1.85, 0.68) PLANNED j3 91 %; C4 (2.05, 0.68) PLANNED j2 96 %; C5 (1.65, 0.65) PLANNED j2 95 %; C6 (1.90, 0.60) PLANNED j3 92 %; C7 (1.95, 0.75) PLANNED **j2 99 % → DIBUANG** (MARGIN = TOLAK; batas sesi > 97 %); C8 (1.70, 0.75) **TORQUE-UNSAFE 8/8** (j2 113 %) → dibuang. |
| A0.4 | **Tanda (6):** M1 (1.75, 0.55) = tanda ARM1-ZONE (TCP terukur (1.7497, 0.5493)); C2, C3, C4, C5, C6 baru. **Fit = M1, C2, C4, C5** (x 1.65–2.05, y 0.55–0.68). **Uji-tahan = C6 (dalam), C3 (tepi atas)** — dipilih SEBELUM data kamera. Rencana gerak dari tanda ke tanda disaring ulang penuh oleh probe. |

### A1. 🔒 Urutan (setiap gerak nyata ditanya, satu per persetujuan)

1. Gerak arm_1: REST → C5 → C3 → C6 → C2 → C4 (`reach_dwell_probe --arms arm_1 --target X,Y,1.22 --approach 0 --trials 1
   --tau-max 14 --plan-attempts 8 --move`), di tiap tanda: TCP (TF, 3 s), operator menandai unting + label, konfirmasi di meja.
   Lalu `return_rest --arms arm_1 arm_2 --move`, K0.
2. Kamera incumbent (`p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280`) + localizer; botol di tiap 6 tanda bergantian, operator menjauh,
   `track_median` 15 s per tanda.
3. Fit (4 tanda) → arg rgbd2 baru → harness baru `cam_up_rgbd2zone.sh`. Kamera diluncurkan ulang dengan arg baru; cek TF hidup =
   prediksi; botol di C6 lalu C3 → track 15 s. (Info: botol di 4 tanda fit juga, dan `arm_fk_check --arm-filter 0.03` REST.)
4. Matikan bersih (node 0 di domain 0 dan 79, 0 `static_transform_publisher`), K0, arsip, laporan.

### A2. 🔒 Kriteria

| # | Lulus bila |
|---|---|
| R1 | fit sah: 4 tanda fit ada track botol tunggal; \|θ\| ≤ 10°, \|t\| ≤ 0.30 m (kalau tidak → fit tak sah, BERHENTI) |
| R2 | **uji-tahan C6 dan C3, ekstrinsik BARU, kamera HIDUP: \|track − tanda\|xy ≤ 5 cm KEDUANYA, z ∈ [0.79, 0.84] keduanya** |
| R3 | TF hidup `world → rgbd2_camera_optical` = prediksi fit (≤ 1 mm, ≤ 0.01°) |
| R4 | penunjuk: setiap gerak MOVED + dwell SUCCESS, TCP ≤ 5 mm dari perintah, torsi terukur ≤ 14 |

R1 ✓ + R2 ✓ + R3 ✓ → arg baru DITERIMA sebagai kandidat ekstrinsik demo (harness baru; operator memutuskan pemakaian; default
launch tidak diubah). Lainnya → tidak dipakai, incumbent tetap. Berhenti + amankan: fault / LED merah / REJECTED / NOT armed sesudah
ARMED / torsi > 14 / benturan / kabel / gerak tak diminta. Kriteria tidak diubah sesudah melihat data.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D580 | 5 gerak tanda + pulang: semua MOVED/SUCCESS, TCP ≤ 5 mm, torsi terukur puncak ≤ 12 N·m. |
| D581 | Ekstrinsik incumbent: galat botol di 6 tanda 0.12–0.18 m, arah ≈ (0.7, 0.7). |
| D582 | Fit: \|θ\| ≤ 5°, residu RMS fit ≤ 2 cm. |
| D583 | R2 lulus dengan kedua uji-tahan ≤ 3 cm. |
| D584 | R3 lulus. |
| D585 | Dengan ekstrinsik baru, rgbd2:arm_3 (REST, dekat) tersaring \|t_xy\| > 5 cm (koreksi zona-saja memperburuk tempat lain). |

## B. Hasil terukur

(diisi sesudah §A dikunci)
