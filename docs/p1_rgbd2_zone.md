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

§A sha256 `50674782…` ([salinan](results/p1_rgbd2_zone/rgbd2_zone_A_locked.md)), commit 687d803.

### B1. Penunjuk arm_1 — GERAK NYATA #1–#7 (semua ditanya satu per satu)

| # | gerak | rencana | TCP terukur | galat | torsi puncak probe |
|---|---|---|---|---|---|
| 1 | REST → C5 (1.65, 0.65) | PLANNED | (1.6506, 0.6503, 1.2180) | 2.0 mm | **13.03** (j2, transit; diam di C5 j2 ≈ 9.7) |
| 2 | → C3 (1.85, 0.68) | PLANNED | (1.8486, 0.6786, 1.2188) | 2.0 mm | 10.17 |
| 3 | → C6 (1.90, 0.60) | 2× TORQUE-UNSAFE → PLANNED | (1.9004, 0.6004, 1.2205) | 0.7 mm | 8.61 |
| 4 | → C2 (2.00, 0.55) | PLANNED | (2.0007, 0.5493, 1.2208) | 1.1 mm | 6.15 |
| 5 | → N1 (1.94, 0.50) | PLANNED | (1.9389, 0.4984, 1.2208) | 2.1 mm | 5.34 |
| 6 | → N2 (1.94, 0.68) | 3× TORQUE-UNSAFE → PLANNED | (1.9388, 0.6806, 1.2195) | 1.4 mm | 10.97 |
| 7 | `return_rest` | lurus | — | 0.061° | 5.45 |

- **Penyimpangan §A (disetujui operator, SEBELUM data kamera):** tanda C2 jatuh **di luar meja, tepat di tepi** (tepi meja ≈ x 2.00 =
  ujung jauh rel) → C2 dan C4 (x 2.05) tidak bisa menampung botol; fit C2/C4 diganti **N1 (1.94, 0.50)**, **N2 (1.94, 0.68)** (plan-check
  PLANNED j3 90 / 92 %). C4 tidak pernah dikunjungi. Uji-tahan C6/C3 dan kriteria TIDAK berubah.
- Semua tanda lain di meja (konfirmasi operator). K0 LULUS sesudah #7. Operator sempat menulis "posisikan gantry dari home lagi";
  diklarifikasi → "lanjut rekaman", gantry tidak diperintah.

### B2. Botol di 6 tanda, ekstrinsik incumbent — NOL gerak

`track_median` 15 s per tanda ([tracks/](results/p1_rgbd2_zone/tracks/)); sebaran p5–p95 ≤ 7 mm (C5 y 26 mm, transisi awal); conf 0.73–0.85;
`cyan bottle` wastafel (3.08, −0.34) tetap ada. M1 vs sesi ARM1-ZONE: 1.3 cm (penempatan ulang).

| tanda | peran | TCP xy | track (incumbent) | galat xy | \|galat\| |
|---|---|---|---|---|---|
| M1 | fit | (1.7497, 0.5493) | (1.8686, 0.6625, 0.825) | (+0.119, +0.113) | 0.164 |
| C5 | fit | (1.6506, 0.6503) | (1.8522, 0.7578, 0.833) | (+0.202, +0.108) | 0.229 |
| N1 | fit | (1.9389, 0.4984) | (2.0708, 0.5528, 0.817) | (+0.132, +0.054) | 0.143 |
| N2 | fit | (1.9388, 0.6806) | (2.1068, 0.7539, 0.810) | (+0.168, +0.073) | 0.183 |
| C6 | uji-tahan | (1.9004, 0.6004) | (2.0332, 0.6777, 0.815) | (+0.133, +0.077) | 0.154 |
| C3 | uji-tahan | (1.8486, 0.6786) | (2.0214, 0.7518, 0.810) | (+0.173, +0.073) | 0.188 |

### B3. Fit — **R1 GAGAL → fit tak sah, BERHENTI**

[fit_data.json](results/p1_rgbd2_zone/fit_data.json) → [fit_result.json](results/p1_rgbd2_zone/fit_result.json): **θ = 13.10°** (batas 10°),
**t = (+0.051, −0.517), \|t\| = 0.519 m** (batas 0.30). R1 ✗ → per §A2 fit tidak dipakai, validasi hidup (R2/R3) tidak dijalankan sebagai
penerimaan; incumbent tetap. Kriteria tidak diubah.

- Pengungkapan: batas \|t\| di R1 kurang tepat — t bergantung pada pusat rotasi (origin world, ~2 m dari zona), jadi θ besar otomatis
  memberi t besar. θ 13° sendiri sudah gagal. Yaw rgbd2 13° akan menggeser lengan dekat ~0.25 m → model planar menambal galat lain
  (tilt / jarak) secara lokal — persis peringatan A0.1.
- Info luring (BUKAN penerimaan): sesudah koreksi, fit 0.8–3.4 cm (RMS 2.4 cm), uji-tahan **C6 2.3 cm, C3 0.5 cm**.

### B4. Validasi hidup — INFORMASI saja (operator memilih sesudah R1 gagal; BUKAN penerimaan) — NOL gerak

- Operator bertanya apakah botol perlu dipindah-pindah lagi untuk memastikan. Jawaban: pengulangan sudah terukur (M1 lintas sesi
  1.3 cm, sebaran per tanda ≤ 7 mm) dan penolakan θ 13° bukan soal noise; selisih prediksi-luring vs hidup di C6/C3 di bawah ini =
  keterulangan penempatan + konvensi.
- Baseline incumbent sesi ini (REST, tersaring): rgbd2 arm_1 +0.119, arm_2 0.068, arm_3 0.020 (= ARM1-ZONE).
- Kamera diluncurkan ulang dengan arg fit (`tf2_x:=0.1311 tf2_y:=-1.5450 … tf2_yaw:=-0.6903`; hanya arg baris perintah, TIDAK ada
  harness baru). TF hidup = prediksi: 0.00 mm, 0.0018° (R3-info ✓; pembulatan arg 4 desimal).
- Botol di uji-tahan: **C6 track (1.8861, 0.6125, 0.815) → 1.9 cm** (prediksi luring 2.3); **C3 (1.8556, 0.6730, 0.810) → 0.9 cm**
  (prediksi 0.5); z keduanya ∈ [0.79, 0.84]. Prediksi luring vs hidup ≤ 1.2 cm.
- Di luar zona: botol wastafel bergeser 19 cm di world (y −0.34 → −0.15); `arm_fk_check` REST: rgbd2 arm_1 (z 1.15–1.45, 2.7 m)
  masih **8.1 cm**; arm_2 dan arm_3 **hilang dari tabel** (< 100 titik dalam jendela 0.25 m → tergeser > 25 cm). Koreksi planar
  hanya benar di permukaan meja zona ini.

### B5. Akhir

- Lengan REST, K0 akhir LULUS (g1 1.49926 / 0°, g2 0.00090 / 0°); gantry tidak diperintah. Kamera, localizer, monitor, perekam
  SIGINT per PID (0 sisa ≤ 5 s); launch 464636 SIGINT → 12 s; `ros2 node list --no-daemon` **0 di domain 0 dan 79**; 0
  `static_transform_publisher`. Crash dump `move_group` shutdown dihapus. Arsip [hw/](results/p1_rgbd2_zone/hw/)
  (`p1rz_joint_states.csv.gz` tidak di-commit). Torsi puncak terukur sesi 13.03 N·m (t1_a1_joint_2, transit ke C5); 0 fault /
  REJECTED / gerak tak diminta. Launch default dan harness demo TIDAK diubah.

### B6. Papan skor

| # | Kriteria | Hasil |
|---|---|---|
| R1 | fit sah (\|θ\| ≤ 10°, \|t\| ≤ 0.30) | **✗** θ 13.10°, \|t\| 0.519 |
| R2 | uji-tahan hidup ≤ 5 cm | — tidak dinilai (R1); info: C6 1.9, C3 0.9 cm |
| R3 | TF = prediksi | — info ✓ (0.00 mm / 0.0018°) |
| R4 | penunjuk | ✓ 6/6 MOVED+SUCCESS, TCP ≤ 2.1 mm, torsi ≤ 13.03 |

| # | Dugaan | Hasil |
|---|---|---|
| D580 | torsi ≤ 12 | ✗ 13.03 (transit ke C5; prediksi plan-check j2 95 %) |
| D581 | incumbent 0.12–0.18 m, arah (0.7, 0.7) | ✗ 0.143–0.229, arah lebih ke x (C5 0.229) |
| D582 | \|θ\| ≤ 5°, RMS ≤ 2 cm | ✗ 13.1°, 2.4 cm |
| D583 | R2 lulus ≤ 3 cm | — tidak diuji formal (info ≤ 1.9 cm) |
| D584 | R3 | ✓ (info) |
| D585 | koreksi merusak lengan dekat > 5 cm | ✓ (> 25 cm) |

**Tally 2/5 diuji. Keputusan per §A: fit TIDAK dipakai, incumbent tetap.** Temuan: galat rgbd2 di meja zona demo 0.14–0.23 m,
bukan galat yaw/geser kaku kamera (yaw yang menutupnya = 13° dan merusak lengan dekat > 25 cm); secara LOKAL di permukaan meja,
koreksi planar menutupnya ke 1–2 cm (uji-tahan hidup).

### B7. Langkah berikut (keputusan operator)

- **(A) Koreksi zona-saja untuk demo**: fase baru terkunci, kriteria ditulis untuk zona (θ bebas, penerimaan = uji-tahan BARU
  hidup ≤ 5 cm di permukaan meja, botol hanya di zona meja, track di luar kotak meja ditolak); tanda M1/C3/C5/C6/N1/N2 masih di meja
  → tanpa gerak lengan untuk fit; arg dipakai HANYA di harness demo. Lalu DEMO-HW.
- **(B) Kalibrasi 6-DOF rgbd2 yang benar**: titik 3D tersebar (botol di kotak bertinggi terukur di tanda, lengan dekat tersaring), nilai
  di meja demo DAN lengan dekat.
