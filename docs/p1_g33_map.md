# P1 / G33 — PETA 3D LINGKUNGAN: penyaring lengan-vs-lingkungan AKTIF sebelum gerak apa pun

> Sesi G33, 2026-10-07. Pemicu: G32-HW arm_3 menabrak rak ([p1_g32_hw.md](p1_g32_hw.md) §B3).
> **§A ditulis dan DIKUNCI SEBELUM kamera dinyalakan.** §B diisi sesudah. Bila §B bertentangan dengan §A, **§B menang**
> dan pertentangannya ditulis (§B-konflik).

## A0. Gerbang (jawaban operator 2026-10-07, sebelum apa pun)

| | Jawaban |
|---|---|
| K0 sel sejak akhir G32-HW C1 | **tidak berubah** (4 lengan REST, rel 0/0, rot 0/0, rak tidak dipindah) |
| K1 kamera / rak terlihat / ruang kosong | **ya, semua siap** |
| K2 ukuran pita rak | tidak ada → tidak ada pembanding pita |
| Operator | pergi sementara: "lakukan sesuai rekomendasimu hingga sukses" → **nol aktuasi** (lengan DAN gantry) selama operator pergi |

## A0.1 Diagnosa plugin octomap (nol gerak) — FAKTA, sebelum §A

- Pesan log G32 `... class occupancy_map_monitor/PointCloudOctomapUpdater ... does not exist. Declared types are ` — daftar
  tipe **kosong** → tidak ada satu pun paket yang mendaftarkan plugin `OccupancyMapUpdater`.
- `ros-humble-moveit-ros-perception` **TIDAK terpasang** (`apt-cache policy`: Installed (none), kandidat 2.5.10);
  MoveIt terpasang 2.5.9 (25 paket), `moveit_ros_occupancy_map_monitor` 2.5.9 ada (monitornya ada, pluginnya tidak).
- `default_sensor` / `kinect_depthimage` **memang dari `sensors_3d.yaml` kita** (salinan contoh MoveIt Setup Assistant,
  topik `/head_mount_kinect/...` yang tidak ada di sel ini) — bukan dari tempat lain.
- Perbaikan langsung (`sudo apt install`) **butuh password** (tidak ada di sesi ini). Membangun 2.5.9 dari sumber GitHub
  **ditolak pengklasifikasi izin** ("Code from External") → klon dinonaktifkan (`COLCON_IGNORE`, install dihapus) supaya
  bring-up berikut tidak memuatnya diam-diam. **Keputusan operator diperlukan** (lihat §C).
- Konsekuensi untuk sesi ini: octomap HIDUP tidak bisa diaktifkan. Peta STATIS beku tidak butuh plugin itu — ia masuk
  ke planning scene sebagai `CollisionObject` biasa (jalur `/planning_scene`, sama seperti `static_collision.py`).
  Urutan operator (plugin → kamera → peta beku → octomap hidup) **dibalik sebagian dan ditulis**: peta beku dulu.

## A. Protokol — DIKUNCI sebelum kamera dinyalakan

### A1. 🔒 Penangkapan

- Kamera: `realsense_dual.launch.py` (ekstrinsik default di file = kalibrasi 2026-07-30, tidak disentuh),
  `with_color_cloud:=false`; `depth_cloud` (frame `world`). Cek proses basi (`static_transform_publisher`, driver ganda) dulu.
- **Tidak ada `my_workcell.launch.py` / TF robot.** Robot DIAM, jadi self-filter memakai **FK pinocchio + geometri tabrakan
  URDF** (model yang sama dengan `interarm_collision.py`, URDF hidup = xacro sekarang, dicek identik: 52 geometri) pada
  **konfigurasi terukur**: baris terakhir `g32hw_joint_states2.csv.gz` (24 sendi lengan ≤ 0.10° dari REST, rel 0.76/0.36 mm,
  rot 0/0) + K0. Jari gripper = netral (0); padding menutupinya.
- Rekam ≥ 10 s per kamera; simpan **awan mentah ter-dedup 1 cm per kamera + jumlah hit** ke `docs/results/p1_g33/`
  (BUKAN `/tmp` — memori G6: `/tmp` hilang). Peta dibangun OFFLINE dari berkas itu.

### A2. 🔒 Pembangunan peta (offline, deterministik)

1. Crop sel: x ∈ [−1.2, 3.0], y ∈ [−2.0, 2.0], z ∈ [0.30, 2.00] m. z < 0.30 (lantai) tak terjangkau lengan;
   **z > 2.00 = koridor rel/plafon** (dasar rotation link 2.0025) tidak dipetakan → dilaporkan sebagai daerah buta.
2. Voxel 2 cm (= `octomap_resolution`), buang voxel terisolasi (< 3 tetangga dalam 5 cm), lalu buang voxel yang di-hit
   hanya oleh < 2 frame (derau).
3. **Self-filter**: buang voxel yang pusatnya ≤ **0.05 m** dari geometri tabrakan robot mana pun (52 = 4×11 lengan/gripper
   + 8 struktur gantry) pada konfigurasi A1. **Ketat:** sendi konfigurasi hilang, nama geometri ≠ 52 yang diharapkan,
   atau berkas konfigurasi tidak ada → **TOLAK** (keluar ≠ 0), tidak ada jalur diam.
4. **Koridor gantry**: voxel di dalam volume sapuan struktur gantry (platform + rotation link + pelat; rel 0…1.6 m,
   rot ±35°) + 0.05 m dibuang dan **dihitung** — itu rel sendiri atau benda yang sudah ditabrak platform.
5. Keluaran `docs/results/p1_g33/env_static_map.npz`: pusat voxel, resolusi, frame `world`, provenans
   (waktu tangkap, konfigurasi, padding, hitungan tiap langkah).

### A3. 🔒 Konsumen (nama ketat, hilang = TOLAK)

- **MoveIt**: node `env_static_map_pub` → satu `CollisionObject` id **`env_static_map`** (kotak per voxel, sisi
  res + 2×0.05 = padding lingkungan sama dengan penyaring skrip) ke `/planning_scene`, transient-local, diulang periodik.
- **Penyaring skrip** `scripts/env_collision.py` `EnvChecker`: coal OcTree dari voxel peta vs 52 geometri robot (hull sejati,
  ⊇ mesh → konservatif); vonis `COLLIDE` (d ≤ 0) / `MARGIN` (d < 0.05) / `CLEAR`. **MARGIN = TOLAK** di jalur HW.
  Berkas peta tidak ada → TOLAK.

### A4. 🔒 Kontrol (lulus/gagal)

- **(P+) KONTROL POSITIF — GERBANG UTAMA:** lintasan yang BENAR-BENAR dijalankan arm_3 di G32 ev 8
  (`g32hw_joint_states1.csv.gz`, 432.49 s → puncak torsi ~437.7 s, g2 (0.60, −10°)) **harus** mencapai d < 0.05 m
  (MARGIN/COLLIDE) pada `EnvChecker`, dan konfigurasi kontak harus INVALID di MoveIt dengan `env_static_map`.
  Tiga rencana tersimpan R10 k=8 dilaporkan (bukan gerbang: OMPL acak, belum tentu lewat rak).
  **Penyaring yang tidak menolak ev 8 = GAGAL, apa pun log-nya.**
- **(N−) KONTROL NEGATIF:** lintasan R0 seed 36 yang benar-benar dijalankan (6/6 tanpa kontak, `g32hw_joint_states1`)
  harus CLEAR (d ≥ 0.05) — kalau tidak, peta/padding terlalu gemuk dan itu DITULIS.
- **(N) self-filter:** voxel peta ≤ 0.05 m dari robot di konfigurasi tangkap = 0 (benar menurut konstruksi — dilaporkan,
  bukan bukti) dan di ≥ 2 pose tugas R0 (informatif); MoveIt REST tanpa "start state in collision".
- **(P) filter MATI:** voxel ≤ 0.05 m dari tiap lengan dilaporkan per lengan; ≥ 1 untuk lengan yang terlihat kamera
  (lengan dengan 0 titik = tidak terlihat, dilaporkan, bukan gagal).
- **Sisa pelat:** voxel di cangkang 0.05–0.15 m dari geometri gantry dilaporkan; bila ada gugus padat → bentuk/padding
  platform diperbesar (DITULIS).

### A5. 🔒 Dugaan (SEBELUM data)

| # | Dugaan |
|---|---|
| D213 | Rak terlihat: ≥ 50 voxel dalam 0.30 m dari titik kontak model ev 8. |
| D214 | (P+) lintasan nyata ev 8 → MARGIN/COLLIDE (d_min < 0.05). |
| D215 | (N−) lintasan nyata R0 → semua CLEAR. |
| D216 | (P) 4/4 lengan terlihat dengan filter mati. |
| D217 | Ada sisa pelat carriage (cangkang 0.05–0.15 m platform) > 0 voxel. |
| D218 | Dari rencana tersimpan G22–G32 yang bisa disaring ulang, 5–20 % kini ditolak. |

### A6. Yang TIDAK dikerjakan sesi ini

- Gerak lengan / gantry (operator pergi). Tangkapan ke-2 (gantry di posisi lain) → G34.
- Octomap hidup (plugin tidak terpasang, §A0.1).
- Ganti pose simpan default (evaluasi offline sesudah peta ada; bukan sesi ini).

## B. Hasil terukur

> Data: `docs/results/p1_g33/` — awan mentah `capture_a.npz` (durabel, bukan /tmp), peta kanonik `env_static_map.npz`
> (= `env_map_reg3.npz`, laporan `env_static_map_report.json`), kontrol `g33_controls_reg3.{log,json}`,
> MoveIt `g33_moveit_check_reg3.json`, gambar `g33_map_views.png`. Peta antara (v2, rp*, reg, reg2) disimpan sebagai jejak §B1–B3.

### B0. Tangkapan (2026-10-07 ~12:04, robot DIAM, nol aktuasi)

| | rgbd (241122302297) | rgbd2 (234222303079) |
|---|---|---|
| frame / 20 s | 36 (~1.9 Hz, `depth_cloud` stride 2) | 32 |
| sel 1 cm / di crop | 607 803 / 247 202 | 541 175 / 224 435 |

Tak ada proses basi sebelum mulai; node ganda 0. Peringatan `Motion Module force pause` = IMU (dimatikan), diabaikan.

### B1. Peta pertama (§A apa adanya, pad 0.05) — gagal kontrol negatif, sebabnya BAYANGAN LENGAN

- 81 213 voxel. (P) filter mati: 4/4 lengan terlihat (t1_a1 199, t1_a2 303, t2_a1 86, t2_a2 115 voxel dibuang) → **D216 ✓**.
- **2 476 voxel di cangkang 0.05–0.15 m dari lengan**, hampir semua dilihat SATU kamera → "kulit" lengan bergeser.
- Pad 0.15: (P+) ev 8 MARGIN 0.017 (lulus tipis), tapi (N−) R0 ev01 MARGIN 0.027 dan **ev11 COLLIDE −0.024** pada eksekusi
  nyata tanpa kontak; MoveIt (kotak bin 4 cm) **53/135** konfigurasi R0 INVALID → peta tak bisa dipakai.
- Ditambah **filter bayangan sinar** (pengamatan yang sinar kameranya lewat ≤ 0.05 m dari robot dibuang; voxel tetap bila
  kamera lain melihatnya bersih) → ekor flying-pixel di bawah lengan hilang, tapi ev11 tetap COLLIDE: gugusnya 0.15–0.18 m
  dari gripper t2_a1 saat tangkap, hanya rgbd.

### B2. 🔴 Temuan utama: peta kamera vs model robot (URDF) bergeser ~0.1 m

Per lengan × kamera, offset titik-ke-permukaan lengan (ICP translasi) **semuanya searah (+x, −y)**, dari dua kamera di pojok
BERLAWANAN (jadi bukan bias visibilitas): rgbd t1_a1 (0.093, −0.048), t1_a2 (0.054, −0.040), t2_a1 (0.097, −0.070),
t2_a2 (0.055, −0.082); rgbd2 t1_a1 (0.054, −0.075), t2_a1 (0.045, −0.105) m. Kedua kamera saling sepakat (kalibrasi 07-30
<5 cm tetap benar) — yang tidak sepakat adalah **dunia kamera vs dunia URDF**. Akibat: tiap penyaring yang menaruh robot
(URDF) di peta kamera salah ~10 cm. Ini menjelaskan (P+) yang hanya MARGIN meski kontak nyata.

**Registrasi ke robot** (`build_env_map.py --register`): robot DIAM di konfigurasi terukur = target kalibrasi; ICP
titik-ke-permukaan bertanda, 4-DOF (translasi + yaw) per kamera, 400 titik/lengan:

| | t (m) | yaw | median \|d\| sebelum → sesudah |
|---|---|---|---|
| rgbd | (−0.079, +0.078, +0.005) | −0.15° | 8.3 → **1.6 cm** |
| rgbd2 | (−0.053, +0.102, +0.004) | −0.27° | 8.1 → **0.9 cm** (hanya t1_a1, t2_a1 terlihat) |

Peta dikoreksi; **ekstrinsik di `realsense_dual.launch.py` TIDAK diubah** (koreksi hidup di peta + laporannya). Gerbang:
median sesudah > 3 cm → TOLAK. Penyebab akar (URDF `t*_base_link` y ±0.36 / pelat carriage, atau pergeseran asal encoder
gantry sejak kalibrasi 07-25 — memori gantry-origin 09-21) **belum dipisahkan** → G34.

### B3. Penyetelan sesudah registrasi (B menang atas A, ditulis)

| versi | self-pad | sinar | catatan | (P+) | R0 bukan-CLEAR |
|---|---|---|---|---|---|
| reg | 0.05 (pusat voxel) | 0.05 | cangkang lengan 2 476 → 330 | COLLIDE −0.012 | ev00 MARGIN 0.035, ev01 COLLIDE −0.029 |
| reg2 | 0.05 + 0.0173 (KUBUS) | 0.05 | self-filter diukur ke pusat, penyaring ke kubus coal → beda ≤ res·√3/2 | — | ev01 MARGIN 0.042 (4 voxel 0.068–0.17 m dari t1_a1) |
| **reg3 (kanonik)** | **0.20 + 0.0173** | **0.05** | ekor bayangan rgbd teramati ≤ 0.17 m | **COLLIDE −0.012** | **ev13 MARGIN 0.026** (lihat B4) |

Harga self-pad 0.20: zona 20 cm di sekitar keempat lengan REST **di home** = DAERAH BUTA (tidak dipetakan, bukan "bebas").
Bayangan sinar membuang 7 523 voxel (rgbd 3 543 / rgbd2 4 456 suara) = kolom di bawah lengan REST = juga TAK DIKETAHUI.
Keduanya diisi tangkapan ke-2 dengan gantry di posisi lain (G34). Koridor gantry: 0 voxel. Sisa pelat carriage
(cangkang 0.05–0.30 m struktur): **0** → **D217 ✗**, bentuk platform tidak diubah.

Peta kanonik: **71 763 voxel 2 cm**, bbox x −1.19…2.99, y −1.95…1.57, z 0.31…1.95; rgbd-saja 34 377, rgbd2-saja 35 412,
keduanya 1 974.

### B4. Kontrol (peta kanonik reg3)

**(P+) GERBANG — LULUS.** Lintasan NYATA arm_3 ev 8 (`g32hw_joint_states1`, g2 0.5996 m / −9.98°): d_min **−0.012 m
COLLIDE** (`t2_a1_right_finger_dist`, −0.41 s); saat puncak torsi −0.002; **ditolak pertama 2.0 s sebelum kontak**.
MoveIt `/check_state_validity` dengan `env_static_map`: VALID −3.0 s, **INVALID −2.0/−1.0/−0.5/0 s**, kontak hanya
gripper/jari t2_a1 ↔ `env_static_map`. Rak teridentifikasi: puncak z ≈ 1.05 m di tepi y ≈ −0.55…−0.63 (sesudah
registrasi), dilihat kedua kamera; 1 622 voxel ≤ 0.30 m dari arm_3 saat kontak → **D213 ✓**. **D214 ✓** (sebelum
registrasi hanya MARGIN 0.017 — lulus karena margin, bukan karena geometri).

**(N−) R0 nyata (6/6, tanpa kontak):** 13/14 event CLEAR (d_min 0.069–0.216 m). **ev13 (t5 arm_4 → (0.93, −0.46, 1.00)
@ g2 1.45) MARGIN 0.026** — voxel terdekatnya tepi atas RAK (dilihat kedua kamera, 0.53 m dari robot saat tangkap, jadi
bukan bayangan): eksekusi R0 nyata itu **nyaris menabrak rak** (~2.6 cm ke hull; hull ⊇ mesh). d > 0 konsisten dengan
"tanpa kontak"; margin 5 cm menolaknya → **D215 ✗** (dugaan "semua CLEAR" meleset karena geometri nyata, bukan peta gemuk).
R10 ev 0–7 nyata (tanpa kontak): **8/8 CLEAR**.

**(N) self-filter:** voxel ≤ 0.05 m dari robot di konfigurasi tangkap = 0 (konstruksi). MoveIt REST: **VALID, tanpa kontak**.
MoveIt 135 konfigurasi R0 tersampel (tiap 5 s): **1 INVALID** (ev13, gripper t2_a2 ↔ peta — kasus nyaris-tabrak yang sama).

### B5. Penyaring lingkungan di jalur HW (langkah 3)

| jalur | dipasang | gagal muat / peta hilang |
|---|---|---|
| MoveIt planning scene | `scripts/env_static_map_pub.py` → `env_static_map` (16 086 kotak = kolom voxel 2 cm + 0.05 m, `--once` memverifikasi via `/get_planning_scene`) | exit 2 |
| `reach_dwell_probe._plan_and_screen` | `screen_env` sesudah layar antar-lengan, SELALU (bukan hanya `other_arm`); juga cek `env_static_map` ADA di scene | `ENV-UNSCREENED` / `ENV-COLLIDE` |
| `return_rest.py` | `EnvChecker` di samping se-gantry + antar-gantry | MENOLAK rc 1 |
| `pose_to_g.py` (G32) | sapuan (lin, rot) `rect_points` vs peta, sesudah S28 | REFUSE rc 1 |
| `g31_screen.py` | retract + traverse (task lewat `_plan_and_screen`) | ENV-MARGIN/COLLIDE → ditolak |

`run_g32.py` hanya menggerakkan lewat tiga alat di atas. `scripts/env_collision.py --self-test` LULUS (kotak di lengan →
COLLIDE, 1 m jauhnya → CLEAR, sendi hilang & peta hilang → raise).
⚠ Kotak MoveIt = inflasi L∞: di sudut kotak MoveIt sampai √3× lebih ketat dari 5 cm. Bin 4 cm membuatnya jauh lebih parah
(53/135) → bin = resolusi peta.

### B6. Plan-only di mock (langkah 4) dan rencana lama yang kini ditolak

Mock `ROS_DOMAIN_ID=77` (`use_fake_hardware:=true enable_gantry_bridge:=false`, launch PID 2013363), `env_static_map` (reg3)
di scene, `g31_screen.py` seed 36 k-of-k 3 dengan penyaring G33 ([log](results/p1_g33/g33_planonly.log),
[g33_planonly.sh](results/p1_g33/g33_planonly.sh)):

- **R0: 3/3 LOLOS** (18/18 PLANNED). Tugas t5 arm_4 (yang dieksekusi R0 lewat 2.6 cm dari rak) kini direncana dengan jarak
  **58.9 mm** ke peta — MoveIt menghindari tepi rak sendiri.
- **R10: DITOLAK di sampel 1.** ev 8 (t0 arm_3 @ g2 0.60/−10°, tugas yang menabrak) dan ev 10 kini **PLANNED** dengan
  jalur menjauhi rak; lalu **ev 11 retract g2 @ −10° (interpolasi sendi lurus dari t5 ke REST) → lingkungan −15.7 mm
  COLLIDE**. Bahaya kedua R10 yang sebelumnya tak terlihat: pulang lurus dari tepi rak menembus rak. Di HW `return_rest`
  kini MENOLAK di titik itu → lengan tertahan di pose tugas → **retract perlu jalur pulang yang direncana MoveIt
  (sadar-lingkungan)** bila garis lurus terhalang (G34).
- Peringatan proses: run pertama memakai default `env_static_map.npz` = peta v2 LAMA (tanpa registrasi) dan menolak R0
  traverse g1 (MARGIN 27.5 mm); dihentikan, peta kanonik diganti reg3, diulang. Nama berkas default sekarang = reg3.

Rencana tersimpan disaring ulang (`g33_rescreen_old.py`, [json](results/p1_g33/g33_rescreen_old.json)); G22–G27 tidak
menyimpan lintasan → tidak bisa disaring ulang:

| berkas | lintasan | CLEAR | MARGIN | COLLIDE |
|---|---|---|---|---|
| p1_g28/v28_plans | 183 | 183 | 0 | 0 |
| p1_g28y/v28_after_plans | 183 | 182 | 1 (arm_3, g2 1.55) | 0 |
| p1_g31/g31_screen_plans (mock, rot) | 306 | 286 | 15 | **5** (arm_3 @ g2 0.70/−15°; arm_4 @ g2 1.35/0) |
| p1_g32/g32_screen_plans | 18 | 15 | 3 (arm_3 @ g2 0.60/−10°) | 0 |
| p1_g32hw R0 (plan-only NYATA) | 18 | 17 | 1 (arm_4 @ g2 1.45 = t5, B4) | 0 |
| p1_g32hw R10 (plan-only NYATA) | 18 | 16 | 2 (arm_3 @ g2 0.60/−10°, k=10) | 0 |
| **total** | **726** | **699** | **22** | **5** → ditolak **27 = 3.7 %** |

**Ke-27 penolakan semuanya lengan gantry 2 (sisi rak), sebagian besar dengan g2 berotasi.** G28 (rot 0) 1/366. Ini
menjelaskan pola G32-HW: R0 (rot 0) lolos tanpa kontak, R10 (−10°) menabrak. → **D218 ✗** (diduga 5–20 %, terukur 3.7 %).

### B8. Sesudah operator kembali (izin: sudo, hapus klon, tangkapan ke-2)

- **Perception dipasang — tapi satu paket TIDAK cukup.** `ros-humble-moveit-ros-perception` 2.5.10 di atas MoveIt 2.5.9:
  `ldd -r` → `libmoveit_ros_occupancy_map_monitor.so.2.5.10 => not found` + simbol `OccupancyMapUpdater` tak ter-resolve
  (soname berversi). Maka **seluruh MoveIt di-upgrade 2.5.9 → 2.5.10** (26 paket + `moveit_msgs` 2.2.1 → 2.2.3; tak ada
  paket lain; tak ada biner workspace yang menaut libmoveit). Sesudahnya 0 simbol hilang.
- **Mock dengan MoveIt 2.5.10** (domain 77, launch 2037905 → SIGINT 10 s, node list 0): kedua updater dimuat
  (`Listening to '/rgbd/collision_cloud'` / `'/rgbd2/collision_cloud'`), nol error octomap; error lain = baseline balapan
  spawner yang sama dengan G32 mock; **7/7 controller active**. Kontrol MoveIt **identik** dengan 2.5.9: REST VALID, ev 8
  INVALID sejak −2.0 s, R0 1/135 ([json](results/p1_g33/g33_moveit_check_reg3_moveit2510.json)).
- **Octomap HIDUP terverifikasi:** awan sintetis 8 000 titik (frame `world`) ke `/rgbd/collision_cloud` → octomap planning
  scene 0 → **20 010 B, res 0.02**. Yang belum: tak ada penerbit `collision_cloud` di bring-up HW (node `collision_cloud`
  reachability_gng) — tahap berikut.
- Klon `ros2_ws/src/moveit2_perception` **dihapus**.
- **Tangkapan ke-2: alat siap, gerak TIDAK terjadi.** `capture_cloud.py --joints` merekam `/joint_states` sendiri + TOLAK
  bila robot bergerak > 0.5° / 9 mm selama tangkap; `build_env_map.py` kini memproses tiap tangkapan dengan konfigurasinya
  sendiri (`--config <csv>|capture` per tangkapan; registrasi, self-filter, bayangan sinar per tangkapan) lalu
  menggabungkan — regresi pada `capture_a` **identik voxel-per-voxel** (71 763). **Bring-up HW nyata DITOLAK
  pengklasifikasi izin** (sama seperti G32 §B5(2)) → gerak gantry menunggu operator (§C).

### B7. Papan skor D213–D218

| # | Dugaan | Hasil |
|---|---|---|
| D213 | rak terlihat ≥ 50 voxel | ✓ 1 622 |
| D214 | (P+) MARGIN/COLLIDE | ✓ COLLIDE −0.012 (sesudah registrasi; sebelum: MARGIN 0.017) |
| D215 | (N−) R0 semua CLEAR | ✗ 13/14; ev13 MARGIN 0.026 = nyaris-tabrak rak NYATA |
| D216 | 4/4 lengan terlihat filter mati | ✓ |
| D217 | sisa pelat carriage > 0 | ✗ 0 |
| D218 | 5–20 % rencana lama ditolak | ✗ 3.7 % (27/726), semua gantry 2 |

Tak ada dugaan yang menebak offset dunia kamera↔URDF ~10 cm — temuan terbesar sesi ini datang dari residual self-filter,
bukan dari dugaan.

### B-konflik (B menang atas A)

1. §A0.1 urutan operator (plugin → kamera → peta beku → octomap hidup) dibalik: plugin tak terpasang (butuh sudo / izin kode eksternal).
2. §A2.3 self-pad 0.05 → **0.20** (+ setengah diagonal voxel); §A2 tak punya filter sinar → ditambah (0.05).
3. §A0/A1 "ekstrinsik tidak disentuh" — launch tidak disentuh, tapi **peta diregistrasi 4-DOF ke robot** (B2).
4. §A3 MoveIt: bin 4 cm (draf) → 2 cm.
5. §A4 (N−) "harus CLEAR" — 1/14 MARGIN, nyata (B4).

## C. Keadaan akhir (Rule 12)

- **Nol aktuasi.** Lengan dan gantry tidak digerakkan (operator pergi); sel tetap: 4 lengan REST, rel 0.76/0.36 mm, rot 0/0.
- Kamera (domain 0): launch 2009454 SIGINT; `depth_cloud` mengabaikan SIGINT di wrapper → SIGINT anak 2009464 lalu TERM;
  `ros2 node list --no-daemon` **0**, nol proses realsense/static_tf sisa. Mock (domain 77): launch 2013363 SIGINT → keluar
  10 s; node list **0**. Crash dump sesi (`ros2` 12:03 = `topic hz` di-timeout, `move_group` 12:49 = crash shutdown 186 MB)
  dihapus. Disk 56 GB.
- **Diubah:** `scripts/reach_dwell_probe.py` (`screen_env`, `ENV-UNSCREENED`/`ENV-COLLIDE`), `scripts/return_rest.py`,
  `docs/results/p1_g32/pose_to_g.py`, `docs/results/p1_g31/g31_screen.py`, `sensors_3d.yaml` (2 sensor sampah dibuang —
  **belum diuji di bring-up**), `CLAUDE.md` (1 baris). **Baru:** `scripts/env_collision.py`, `scripts/env_static_map_pub.py`,
  `docs/results/p1_g33/` (tangkap, bangun peta, kontrol, rescreen, plan-only).
- **Sistem:** MoveIt 2.5.9 → **2.5.10** (26 paket) + `moveit-ros-perception` 2.5.10 (izin sudo operator, B8). Klon dihapus.
- **Tidak dikerjakan / belum bisa:** tangkapan ke-2 (bring-up HW ditolak pengklasifikasi — operator menyalakan stack, lalu
  perintah di bawah); penerbit `collision_cloud` untuk octomap hidup di HW; penyebab akar offset kamera↔URDF; pose simpan
  tinggi; HW ulang R10.
- **Tangkapan ke-2 (operator, lengan tetap REST, semua titik = henti R0 seed 36 yang sudah dijalankan):**
  `my_workcell.launch.py use_fake_hardware:=false enable_gantry_bridge:=true` → `realsense_dual.launch.py
  with_color_cloud:=false` + `depth_cloud stride:=2 min_depth:=0.3 max_depth:=4.5` → `pose_to_g.py --gantry 1 --seed 36
  --variant R0 0.90 0 --move` dan `--gantry 2 ... 1.45 0 --move` (S28 + penyaring lingkungan) → `capture_cloud.py --out
  capture_b.npz --seconds 20 --joints` → `pose_to_g` keduanya ke `0 0` → `build_env_map.py --capture capture_a.npz
  capture_b.npz --config ../p1_g32hw/g32hw_joint_states2.csv.gz capture --register --self-pad 0.20 --ray-pad 0.05`.
- ⚠ `env_collision.EnvChecker` dibangun ±8 s (hull 52 geometri) per proses; probe men-cache-nya per node.
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi multi-langkah panjang; dilaporkan.

## D. Prompt G34 (salin ke chat BARU)

**Rekomendasi: Opus, effort TINGGI** — gerak nyata pertama sesudah tabrakan; kesalahan registrasi / daerah buta gagal DIAM.

```
Sesi G34 -- HW ULANG R10 DENGAN PENYARING LINGKUNGAN + TANGKAPAN KE-2. Repo ceiling_arm, branch feat/rgbd-topo-deploy.
BACA PENUH: CLAUDE.md; docs/p1_g33_map.md (B2 offset kamera<->URDF, B3 daerah buta, B4 kontrol, B6 R10 retract, C);
scripts/env_collision.py; scripts/env_static_map_pub.py; docs/results/p1_g33/build_env_map.py.
FAKTA G33: peta beku docs/results/p1_g33/env_static_map.npz (71 763 voxel, DIREGISTRASI ke robot 4-DOF: dunia kamera
vs URDF ~0.1 m!); (P+) lintasan nyata ev 8 COLLIDE, ditolak 2.0 s sebelum kontak; R0 plan-only 3/3; R10 ditolak di ev 11
retract g2 @-10 (interpolasi lurus menembus rak -15.7 mm). MoveIt 2.5.10 + perception terpasang: updater dimuat,
octomap hidup terbukti dengan awan sintetis (mock); di HW belum ada penerbit /rgbd*/collision_cloud.
GERBANG (tanya operator SEBELUM apa pun):
 K0. Sel sama seperti akhir G33 (4 lengan REST, rel ~0/0, rot 0/0, rak tidak dipindah)? Benda lain berubah?
 K1. (G33 B8: MoveIt kini 2.5.10 + perception; octomap hidup terbukti di mock.) Nyalakan penerbit collision_cloud di HW
     sekarang, atau tetap peta beku saja di G34?
 K2. Izin gerak gantry untuk TANGKAPAN KE-2 (lengan REST, g1/g2 ke rel ~0.8, rot 0) -- mengisi daerah buta B3.
1. Penyebab akar offset B2 (nol gerak): bandingkan t registrasi dengan URDF t*_base (y +-0.36, z 2.05) + memori kalibrasi
   (pojok gantry-1 via t1_a1_tool_frame 07-25) + memori gantry-origin 09-21. Putuskan: perbaiki URDF / ekstrinsik, atau
   tetap registrasi-per-peta (DITULIS). Registrasi dengan gantry di posisi lain = uji apakah offset konstan.
2. Tangkapan ke-2 (bila K2): gantry di posisi lain, build_env_map --register dengan DUA tangkapan (dua konfigurasi:
   perlu dukungan --config per tangkapan -- tulis); kontrol P+/N- ulang; laporkan daerah buta yang tersisa.
3. Retract sadar-lingkungan: bila interpolasi lurus return_rest ditolak lingkungan, rencanakan pulang via MoveIt
   (env_static_map di scene) + semua penyaring (antar-lengan, torsi, lingkungan); plan-only R10 seed 36 harus 3/3.
4. HW R10 seed 36 HANYA bila: (P+) lulus pada peta final, R10 plan-only 3/3, operator di e-stop. Bring-up nyata +
   env_static_map_pub.py (keep-alive) + cek /get_planning_scene; R0 sama-sesi untuk delta.
5. Laporan §B/§C, p1_state + tally, prompt G35 + rekomendasi model/effort.
ATURAN: tak ada gerak tanpa env_static_map di scene DAN EnvChecker di skrip; mock/HW dimatikan bersih (PID launch ASLI,
node list --no-daemon = 0); B menang atas A dan DITULIS; job > 10 menit via setsid nohup; pkill -f membunuh shell sendiri.
```
