# P1 / CALIB-CHARUCO — kalibrasi ulang ekstrinsik D455 dengan papan ChArUco, acuan = ujung jari arm_2 (world URDF)

> Sesi 2026-10-12 (lanjutan [p1_demo_hw.md](p1_demo_hw.md) F1). Permintaan operator: "kalibrasi pakai ChArUco lagi; arm_2
> menghalangi → pindah dulu". §A dikunci (salinan `results/p1_calib_charuco/calib_A_locked.md` + sha256) sebelum gerak pertama.

## A. Protokol — DIKUNCI

### A0. Keputusan operator + pengungkapan

| # | Isi |
|---|---|
| A0.1 | Papan di **lokasi lama** (2026-07-30: board (0.849, 0.181, 0.593)); permukaan terukur operator **0.625 m** dari lantai. |
| A0.2 | Posisi world papan diukur dengan **lengan sebagai penunjuk** (operator), **2 titik** (asal + arah +X) → posisi + yaw. Sebab: kalibrasi 07-30 memakai pita dari sudut gantry dan hasilnya ~10 cm dari world URDF (G34); world URDF = world MoveIt. |
| A0.3 | Penunjuk = **arm_2** (bukan arm_1): dengan g1 PARKIR 1.50 arm_1 berjarak ~1.07 m (jangkauan tool-bawah ~0.75 m 3D, DEMO-OFF F1), arm_2 0.31 m. Ujung jari tak bisa turun ke 0.625 → titik penunjuk di **z 1.30** (3D ≤ 0.75 m dari basis), operator memakai **unting** dari ujung jari (~0.68 m) dan menandai titik di permukaan. Posisi titik = FK `t1_a2_tool_frame` dari sendi TERUKUR saat dwell (bukan perintah). |
| A0.4 | arm_2 REST memotong garis rgbd → papan (garis lewat (1.13, 0.40, 1.07); arm_2 REST bawah (1.10–1.14, 0.29–0.34, 0.96–1.19)). Disingkirkan dengan **rel g1 1.50 → 1.30 sementara**, lengan REST (`pose_to_g`, tersaring) — keputusan operator; kembali PARKIR 1.50 di akhir. |
| A0.5 | Kode kalibrasi tidak diubah: `calibrate_extrinsics.py --board-xyz X Y Z --board-rpy 0 0 YAW --namespaces rgbd rgbd2 --camera-hint-xyz 1.8 0.8 2.1 0.0 -1.1 2.1`. Fallback chessboard polos (ArUco tidak terbaca di ~2 m, memori). Launch default TIDAK diubah sesi ini; hasil dipakai lewat arg launch sampai diterima operator. |
| A0.6 | RMS reprojeksi TIDAK dipakai untuk memilih kalibrasi (memori 2026-08-13). Penilai = acuan FK lengan (metode `p1_demo_hw.md` B1) + `verify_extrinsics.py`. |

### A1. 🔒 Urutan (setiap gerak nyata ditanya, satu langkah per persetujuan)

1. Bring-up nyata (pola DEMO-HW B2), K0, monitor + perekam.
2. Penunjuk plan-only arm_2: P1 = (0.85, 0.18, 1.30), P2 = (1.025, 0.18, 1.30) (+0.175 m = panjang grid papan arah X).
3. Gerak arm_2 → P1 (`reach_dwell_probe --arms arm_2 --target … --approach 0 --trials 1 --tau-max 14 --move`); baca TCP FK;
   operator menandai titik unting. Gerak → P2, sama. `return_rest --arms arm_1 arm_2 --move`.
4. Operator meletakkan papan: sudut ORIGIN di tanda P1, sudut grid arah +X di tanda P2, rata, muka ke atas. Ukur tebal papan.
5. Rel g1 1.50 → 1.30 (`pose_to_g`, lengan REST). Kamera domain 0; cek papan terdeteksi di rgbd DAN rgbd2.
6. `calibrate_extrinsics.py` (board-xyz = P1 terukur dengan z = 0.625 + tebal, yaw = atan2 dari P1→P2 terukur).
7. Penilaian (kamera diluncurkan ulang dengan hasil baru vs lama-terkoreksi G34, adegan sama): tabel FK lengan (g1 1.30) +
   `verify_extrinsics.py`; lalu rel g1 → 1.50, tabel FK lengan diulang (zona arm_1).
8. Lengan REST, g1 PARKIR 1.50, K0, matikan bersih.

### A2. 🔒 Kriteria terima kalibrasi baru

| # | Lulus bila |
|---|---|
| C1 | kedua kamera solve; papan terdeteksi di keduanya |
| C2 | tabel FK lengan (median − URDF, koreksi radius 4 cm): **semua** lengan yang terlihat \|Δxy\| ≤ 5 cm di kedua kamera, termasuk **rgbd2 arm_1 di g1 1.50** (DEMO-HW: +0.15 m) |
| C3 | `verify_extrinsics.py` antar-kamera ≤ 5 cm |
| C4 | tidak lebih buruk dari incumbent (launch + G34) di lengan dekat (selisih ≤ 1 cm) |

Gagal C2–C4 → hasil TIDAK dipakai (dicatat), incumbent tetap. Berhenti + amankan: fault / LED merah / REJECTED / NOT armed
sesudah ARMED / torsi > 14 / benturan / kabel / gerak tak diminta.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D560 | Plan-only arm_2 P1 dan P2 PLANNED dalam ≤ 2 `g` (titik dekat basis, xy ≤ 0.31 m). |
| D561 | TCP terukur saat dwell ≤ 5 mm dari perintah. |
| D562 | Dengan g1 1.30 papan terdeteksi di kedua kamera (fallback chessboard). |
| D563 | Kalibrasi baru lulus C2 untuk lengan dekat (≤ 5 cm) tetapi rgbd2 arm_1 @ g1 1.50 tetap > 5 cm (papan satu pose di ~2.2 m tidak mengikat tilt pada 3 m). |
| D564 | C3 ≤ 5 cm. |

## B. Hasil terukur

(diisi sesudah kunci)
