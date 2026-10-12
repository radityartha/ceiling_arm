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

§A sha256 `6fb176a0…` ([salinan](results/p1_calib_charuco/calib_A_locked.md)), commit 0b53b54. Data: [results/p1_calib_charuco/](results/p1_calib_charuco/).

### B1. Bring-up + penunjuk arm_2 (18:33–18:46) — GERAK NYATA #1–#3

- Bring-up 1 kali: PID 435154, SigIgn `0x1001001`, 4× '6', 7 spawner mati (`Failed loading`, balapan dikenal) tetapi 7/7 active;
  table2 ARMED, table1 NOT armed sampai perintah g1 pertama (#4) lalu ARMED; env map di scene; K0 LULUS; JTC g1 ref = 1.49999.
- Plan-only: P1 PLANNED sampel 1 (j3 92 %), P2 PLANNED sampel 2 (sampel 1 j2 102 % ditolak) → **D560 ✓**.
- #1 arm_2 → P1: ['TORQUE-UNSAFE', 'TORQUE-UNSAFE', 'PLANNED'] (j2 15.87 / 15.59 ditolak), MOVED, dwell SUCCESS, torsi puncak terukur
  10.35 N·m. TCP (TF, sendi terukur, 4 s) **(0.8505, 0.1804, 1.2981)**, std 0.0 mm.
- #2 arm_2 → P2: PLANNED sampel 1 (j3 88 %), torsi puncak 5.04. TCP **(1.0237, 0.1815, 1.3001)** → **D561 ✓** (≤ 2 mm).
- Operator menandai P1/P2 dengan unting (permukaan laci putih 0.625 m). #3 `return_rest` rc 0, torsi puncak 6.80; K0 LULUS.
- Papan: titik ORIGIN di P1, tepi +X lewat P2, tebal ~1 mm → board (0.8505, 0.1804, 0.626), yaw atan2(P1→P2) = 0.0064 rad.

### B2. Rel + kalibrasi (18:47–18:56) — GERAK NYATA #4–#5

- #4 g1 1.500 → **1.300923** (galat 0.92 mm, stop g31 s33 R0), table1 ARMED. rgbd solve (RMS 0.33 px); **rgbd2 tidak mendeteksi papan**.
  Proyeksi link ke citra rgbd2: yang menutupi papan = **arm_3 (gantry 2)**, bukan arm_2 (link arm_2 di v ≤ 306 px, papan v 354–411).
- #5 (penyimpangan dari §A, disetujui operator) g2 0 → **0.449520** (stop g31 s21 R0; DRY S28 502 mm, env 154 mm). Kedua kamera solve:
  rgbd RMS 0.31, rgbd2 0.45 px (kandidat 1).
- **Bug konvensi** (kode, tidak diubah): titik ORIGIN tercetak (`generate_charuco_board.py`) = corner ChArUco **id0** = (0.035, 0.035) di
  frame papan, sedangkan `calibrate_extrinsics.py` memakai `--board-xyz` sebagai titik (0, 0) = **sudut LUAR grid** (`getChessboardCorners`
  mulai di 0.035). Docstring keduanya menyatakan sebaliknya → setiap kalibrasi yang memasukkan posisi titik ORIGIN bergeser
  (+3.5, −3.5) cm di frame papan (kemungkinan juga 07-30). Kandidat 2 = solve ulang dengan sudut luar (0.8153, 0.2152).
- Geometri papan dicek di citra: titik ORIGIN fisik = corner grid terakhir terdeteksi (pelabelan 180° dipilih hint), arah +X grid ke
  P2, −Y ke kiri atas = world −y → penempatan sesuai. (Sumbu yang digambar `drawFrameAxes` tidak cocok dengan pelabelan terbalik —
  artefak gambar debug saja.)

### B3. Penilaian (nol gerak) — tabel FK lengan ([arm_fk_check.py](results/p1_calib_charuco/arm_fk_check.py)), \|Δxy\| m

| adegan | kamera:lengan (jarak) | incumbent (launch + G34) | kandidat 1 (titik ORIGIN) | kandidat 2 (sudut luar) |
|---|---|---|---|---|
| g1 1.30, g2 0.45 | rgbd:arm_1 (0.9 m) | 0.083 | 0.168 | 0.118 |
| | rgbd:arm_2 (1.2 m) | **0.037** | 0.120 | 0.071 |
| | rgbd:arm_3 (1.7 m) | 0.078 | 0.172 | 0.122 |
| | rgbd2:arm_4 (1.0 m) | **0.016** | 0.145 | 0.092 |
| | rgbd2:arm_3 (1.4 m) | 0.070 | 0.185 | 0.133 |
| | rgbd2:arm_2 (2.0 m) | 0.075 | 0.131 | 0.082 |
| | rgbd2:arm_1 (2.5 m) | 0.162 | 0.190 | 0.143 |
| g1 1.50, g2 0 | rgbd:arm_2 (1.1 m) | **0.043** | — | 0.073 |
| | rgbd:arm_4 (2.5 m) | **0.036** | — | 0.085 |
| | rgbd2:arm_3 (1.2 m) | **0.044** | — | 0.119 |
| | rgbd2:arm_2 (2.1 m) | 0.069 | — | 0.092 |
| | **rgbd2:arm_1 (2.7 m)** | **0.149** | — | **0.155** |

- Kandidat 2 bergeser bersama ≈ (+0.08, −0.05) dari incumbent di kedua kamera (juga di jarak 1 m) → selisih kerangka acuan, bukan galat
  PnP acak. `verify_extrinsics.py` (C3) tidak dijalankan: papan tidak terlihat kedua kamera bersamaan pada adegan penilaian
  incumbent/kandidat 2 yang terakhir (dicatat, Rule 12).
- **rgbd2 arm_1 ≈ +0.15 m dengan KEDUA set ekstrinsik** yang berbeda 6–10 cm → galat di arm_1 kemungkinan **bukan** ekstrinsik kamera:
  (a) posisi fisik arm_1/gantry-1 ujung jauh ≠ URDF (G34 hanya rel ≤ 1.45; g1 kini 1.50), atau (b) titik bukan-lengan (tirai/jendela/
  kabel) masuk jendela 0.25 m di 2.7 m. Belum dibedakan.
- Catatan metrik: C2 membandingkan permukaan lengan REST dengan URDF — data yang sama dengan fitting G34 → incumbent diuntungkan;
  ChArUco beracuan TCP mengukur "di mana TCP fisik saat FK = X", yang lebih dekat dengan kebutuhan demo. Kriteria TIDAK diubah.

### B4. Kembali + matikan (18:56–19:00) — GERAK NYATA #6–#7

- #6 g2 → 0.000901 (galat 0.90 mm); #7 g1 → **1.499262** (galat −0.74 mm). K0 LULUS (lengan REST, g1 1.4993, g2 0.0009). Papan
  dibiarkan di laci (operator sempat bertanya apakah dikembalikan ke lokasi lama — tidak perlu: posisi lama tak pernah terukur di world URDF).
- Kamera, monitor, perekam, launch SIGINT per PID; node 0 di domain 0 dan 79; 0 `static_transform_publisher` basi. Crash dump shutdown
  (`move_group`, `relay`) dihapus. Arsip log `.gz`; `p1cal_joint_states.csv.gz` (22 MB) tidak di-commit.
- 7 gerak nyata, semuanya ditanya satu per satu; torsi puncak terukur maks 10.35 N·m; 0 fault / REJECTED.

### B5. Kriteria + dugaan

| # | Hasil |
|---|---|
| C1 | ✓ (rgbd2 butuh g2 0.45) |
| C2 | **✗** — kandidat 1 dan 2: lengan 7–19 cm; incumbent juga gagal rgbd2:arm_1 |
| C3 | — tidak diuji |
| C4 | **✗** — kandidat lebih buruk 3–13 cm di lengan dekat |
| D560 | ✓ |
| D561 | ✓ ≤ 2 mm |
| D562 | ✗ (g1 1.30 saja tidak cukup: arm_3 menutupi rgbd2; perlu g2 0.45) |
| D563 | ✗ harfiah (lengan dekat juga > 5 cm); rgbd2 arm_1 tetap > 5 cm ✓ |
| D564 | — tidak diuji |

**Keputusan per §A: kandidat TIDAK dipakai, incumbent (launch default + koreksi G34 via arg) tetap. Tally 2/5 dugaan teruji.**

### B6. Temuan + langkah berikut

- **F1** bug konvensi titik ORIGIN vs `--board-xyz` (3.5 cm per sumbu) — perbaiki docstring/kode sebelum kalibrasi ChArUco berikut.
- **F2** galat +0.15 m di arm_1 identik untuk dua ekstrinsik → cek dulu apakah arm_1/gantry-1 di 1.50 benar posisinya (mis. lengan sebagai
  penunjuk di zona demo + unting ke penggaris di meja, atau kamera melihat TCP arm_1 di atas tanda) sebelum mengkalibrasi kamera lagi.
- **F3** penunjuk lengan + unting: akurat (TCP ≤ 2 mm dari perintah), cepat (2 titik ~8 menit), aman (torsi ≤ 10.4). Bisa dipakai lagi
  untuk validasi zona demo.
