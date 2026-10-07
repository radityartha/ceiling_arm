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
locked 2026-10-07T12:03:07+09:00
