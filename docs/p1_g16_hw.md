# P1 / G16-HW — §8c langkah 2: reach-and-dwell ke target terpersepsi

> Sesi G16, disiapkan **2026-08-16**, dijalankan **2026-08-17** saat lengan
> dipasang. Menggantikan prompt G16 di [p1_g15_dense.md §C](p1_g15_dense.md).
>
> **§A ditulis dan DIKUNCI SEBELUM perangkat keras dipasang** — jadi sebelum
> satu sampel pun ada. §B diisi sesudah. Kalau §B bertentangan dengan §A, yang
> menang **§B**, dan pertentangannya ditulis eksplisit.

---

## A0. 🔴 KENAPA G16 FAKTORIAL DIBATALKAN

[p1_g15_dense.md §C](p1_g15_dense.md) mengusulkan menyelesaikan faktorial
4 sudut. **Dibatalkan**, dan alasannya dihitung, bukan dirasakan:

| | |
|---|---|
| Keuntungan informasi yang diharapkan | **rendah** — prior DUNIA (LONGGAR) sudah menang **3 dari 3** faktor; 493 bukti dua-sisi `Δ_arm = 0` lawan satu positif |
| Biaya | ~1 sesi penuh untuk memperkuat angka nol yang sudah punya 493 bukti |
| Yang tidak dikerjakan sementara itu | **§8c langkah 2 dari 5 belum pernah dimulai**, dan tenggat grasp **2026-11-13** tinggal ~3 bulan |

🔒 **Faktorial 4 sudut TIDAK dicabut, ia DITURUNKAN PRIORITASNYA.**
`test/dense_g15.py` sudah bisa menjalankan `DENSE-P0XC` dengan satu perintah;
ia dikerjakan kalau ada sesi luang, dan sampai itu terjadi keempat sudut
dilaporkan **TIDAK DIUKUR** dengan namanya (`p1_g15 §B7.5`).

**Yang membunuh paper ini bukan `Δ_arm`** — melainkan handover yang tidak
pernah terjadi dan heuristik yang tidak lolos ambangnya sendiri
(`p1_g8 §B2`: max gap **24.31 %** lawan ambang ≤ 10 %). Dua-duanya butuh
perangkat keras, dan perangkat kerasnya baru dipasang besok.

---

## A. Protokol — DIKUNCI SEBELUM PERANGKAT KERAS DIPASANG

### A1. 🔒 DEFINISI SUKSES — TIDAK DITULIS ULANG

Definisi sukses tugas **sudah terkunci sejak 2026-08-13**
([p1_g4_reach_dwell.md §A1](p1_g4_reach_dwell.md)) dan **dipakai apa adanya**:

```
posisi   : ||p_tool - p_cmd||        <  5 mm
orientasi: sudut(a_tool, a_cmd)      <  5 deg   (sumbu approach, roll bebas)
dwell    : 2.0 s KONTINU, >= 10 Hz   (satu sampel di luar toleransi = RESET)
```

🔴 **Dilarang menggeser ambang ini setelah melihat data.** `pos_tol` /
`ori_tol_deg` / `dwell` adalah parameter ROS supaya sebuah run bisa
**dilaporkan** pada toleransi lain, **bukan** supaya palangnya bisa dipindah —
dan setiap baris CSV mencatat ambang yang dipakai menilainya.

### A2. 🔒 APA YANG MEMBUAT LANGKAH 2 "LULUS" — dikunci sebelum ada sampel

| | Dikunci |
|---|---|
| **Jumlah percobaan** | **10 percobaan** pada **satu lengan** (`arm_1`), target terpersepsi, pose berbeda-beda |
| **LULUS** | **≥ 8 dari 10** memenuhi definisi A1 penuh |
| **Dilaporkan apa pun hasilnya** | pembagian **TIGA ARAH** per percobaan (A3), galat posisi/orientasi p50 dan p95, laju sampel aktual, dan `load`/waktu |
| **Percobaan DILARANG dibuang** | percobaan yang gagal karena sebab mesin (A5) dilaporkan sebagai **TIDAK VALID** dengan sebabnya disebut, dan **diulang**, bukan dihapus diam-diam |

### A3. 🔒 PEMBAGIAN TIGA ARAH — supaya "gagal" tidak jadi satu ember

Ini yang membedakan kegagalan **metode** dari kegagalan **eksekusi**, dan
harus dipisah sebelum data ada:

| Vonis | Artinya | Menyalahkan |
|---|---|---|
| **NO-PLAN** | MoveIt tidak menemukan rencana ke pose perintah | kelayakan / peta kapabilitas |
| **REACHED-NOT-HELD** | masuk toleransi ≥ 1 sampel tapi tidak pernah 2.0 s kontinu | **pelacakan / stabilitas** — inilah yang definisi dwell ada untuk menangkap |
| **SUCCESS** | A1 penuh | — |

🔒 **`REACHED-NOT-HELD` wajib dilaporkan terpisah.** Kalau dilebur ke "gagal",
angka yang dihasilkan tidak dapat membedakan "tidak sampai" dari "tidak
ditahan", dan itu persis perbedaan yang membuat definisi dwell dipilih
(`p1_g4 §A1`: rata-rata tidak bisa membedakan ditahan dari dilewati).

### A4. 🔴 PEMASANGAN ULANG — apa yang BASI dan apa yang TIDAK

Lengan dilepas **2026-08-13** untuk membebaskan pandangan `rgbd2` ke board.
Pemasangan ulang **bisa menggeser pose mounting**. Yang harus dipisahkan
sebelum ada yang mengklaim apa pun:

| Hal | Status setelah pasang ulang | Kenapa |
|---|---|---|
| **L2 (akurasi eksekusi, < 5 mm)** | ✅ **TIDAK basi** | diukur dari TF `world → t*_a*_tool_frame`, yaitu **FK dari sendi TERUKUR**. Ia menangkap galat servo, dan **secara konstruksi tidak melihat** galat mounting |
| **Ekstrinsik kamera RGBD** | ✅ **TIDAK basi** | kamera tidak disentuh; kalibrasi 2026-07-30 berdiri |
| **`corner_world_xyz = (−0.151, 0.181, 2.023)`** (2026-07-25) | 🔴 **BASI** | referensi sudut relatif terhadap struktur yang lengannya dibongkar |
| **Klaim akurasi ABSOLUT ke dunia** | 🔴 tidak diklaim, dan tetap tidak diklaim | `p1_g4 §A2` sudah mengunci ini. Butuh pengukuran eksternal yang tidak ada |

🔒 **Konsekuensi yang dikunci:** pergeseran mounting **tidak membatalkan
langkah 2**, karena langkah 2 mengukur L2 dan L2 berbasis FK. Yang wajib
dilakukan adalah **mengatakannya**, bukan mengukurnya ulang dengan alat yang
tidak ada. Kalau di kemudian hari akurasi absolut diklaim, ia mewarisi
pergeseran ini dan harus diukur eksternal lebih dulu.

⚠️ **Board masih ada di dalam ruang kerja lengan.** Keluarkan sebelum langkah
2 — **kecuali** sengaja dipakai sebagai target persepsi, yang sah dan harus
disebut sebagai pilihan.

### A5. 🔴 SEBAB MESIN YANG SUDAH DIKENAL — kegagalan karena ini BUKAN kegagalan metode

Diperiksa oleh `scripts/remount_check.py` **sebelum** gerak apa pun. Semuanya
sudah pernah memakan satu sesi debugging masing-masing:

| Gejala | Sebab terukur | Rujukan |
|---|---|---|
| `ros2_control_node` **menggantung selamanya** di "Connecting to robot" | `enp112s0` naik **tanpa IP** di subnet 192.168.2.x | [[network-192-168-2-subnet]] |
| `ros2_control_node` **SIGABRT (−6)** saat start | satu lengan masih **mid-boot**; `Actuator count reported by robot is '0'` → protobuf fatal → **keempat lengan mati bersama** | bug vendor `kortex_driver` |
| `ros2_control_node` **SIGPIPE (−13)** mid-sesi | `INVALID_USER_SESSION_ACCESS` — klien kedua membuka sesi (web UI Kinova di tab browser!) | tutup tab `https://192.168.2.1x` |
| MoveIt `CONTROL_FAILED (−4)` | **`move_group` ganda** dari sesi sebelumnya | `ps aux \| grep move_group` |
| `/joint_states` tepat **10 Hz** | itu `dual_table_controller` **sendirian** — stack ros2_control sudah mati | jangan salahkan gripper |
| Layanan meja "not initialized" | `_initialize_tables()` **one-shot**, tanpa retry | restart node |
| TF kamera aneh / tak berubah | `static_transform_publisher` **basi menumpuk** (pernah 12) | pindai `/proc/*/cmdline`, **bukan** `pgrep` |
| Self-filter diam-diam no-op | TF lengan hilang | [[gng-collision-static-self-collision-fix]] |

🔒 **Aturan atribusi, dikunci:** percobaan yang gagal dengan salah satu gejala
di atas dicatat **TIDAK VALID (mesin)** dengan gejalanya disebut, dan
**diulang**. Ia **tidak** masuk penyebut 10 percobaan A2, dan ia **tidak**
dilaporkan sebagai kegagalan metode.

### A6. 🔴 KESELAMATAN — palang yang tidak boleh dilewati

| # | Aturan | Angka |
|---|---|---|
| **S1** | Pose istirahat lengan = **MENGGANTUNG** (radius 0.52 m) | — |
| **S2** | 🔴 **JANGAN PERNAH** memerintahkan tuck `[0, 2.6, 2.6, 0, 0, 0]` | **12.78 / 14 N·m** — dibatalkan penjaga torsi; itu pose awal sistem **fake**, bukan pose nyata ([[ceiling-arm-fold-is-torque-limited]]) |
| **S3** | Torsi dipantau dari `/joint_states` (`effort`, N·m nyata, ~96 Hz, 28 nilai) sepanjang sesi | |
| **S4** | `fault_controller` **tidak di-spawn** → red LED butuh **reset fisik**. Jadi pencegahan harus di bawah ambang firmware; sekali `ARMSTATE_IN_FAULT` mengunci, sudah terlambat | [[torque-fault-prevention]] |
| **S5** | Gantry: batas operasional **1600 mm** (end stop ~1656 mm). `jog` (op 10–13) **tetap tidak terjaga** | [[gantry-linear-travel-limit]] |
| **S6** | Kecepatan naik **bertahap**, berhenti pada batas terukur | `p1_g4 §A5` |
| **S7** | 🔴 **Tidak ada gerak tanpa persetujuan eksplisit manusia.** Skrip default **read-only**; menggerakkan butuh flag `--move` yang diketik sengaja | |

### A7. 🔒 URUTAN, dan gerbangnya

```
TAHAP 0  remount_check.py            READ-ONLY, tanpa ROS
         port USB, IP subnet, ICMP 4 lengan, proses basi, kamera
         GERBANG: semua HIJAU sebelum meluncurkan apa pun

TAHAP 1  my_workcell.launch.py       bring-up, MASIH tanpa gerak lengan
         remount_check.py --ros
         /joint_states ~96 Hz & 28 nilai (BUKAN 10 Hz), effort ada,
         4 controller aktif, TF world->t1_a1_tool_frame hidup
         GERBANG: semua HIJAU

TAHAP 2  torsi istirahat MENGGANTUNG, dicatat
         GERBANG: di bawah ambang, tidak ada red LED

TAHAP 3  REGRESI gerak terkecil yang sudah pernah berhasil:
         arm_1 joint_6 +5 deg lewat ros2_control
         GERBANG: bergerak, kembali, tidak ada fault

TAHAP 4  §8c LANGKAH 2 -- reach_dwell_probe.py, 10 percobaan
         monitor menilai, probe memerintah, keduanya TERPISAH
```

🔒 **Tahap tidak dilompati.** Tahap 3 adalah **regresi**, bukan eksperimen — ia
sudah pernah berhasil (`arm_1` joint_6 5°, 2026-08-12), jadi kegagalan di sana
berarti pemasangan ulang, bukan metode.

### A7b. 🔴 PERINTAH KONKRET — diverifikasi 2026-08-16, bukan diingat

#### 🔴 PRASYARAT: WORKSPACE HARUS DIBANGUN ULANG DULU

Diperiksa hari ini:

| | |
|---|---|
| `ros2_ws/install/` | **KOSONG** |
| `install/` (akar repo) | ada, tetapi tertanggal **30 Juli** |
| `reach_dwell_monitor` | ❌ **TIDAK TERPASANG** — ditulis 13 Agustus, jadi ia lahir **setelah** build terakhir |

➜ `ros2 run reachability_gng reach_dwell_monitor` **AKAN GAGAL** besok kalau
workspace tidak dibangun ulang. Ini **tidak butuh perangkat keras** — kerjakan
malam ini atau pagi sebelum memasang lengan.

```bash
cd ~/Documents/ceiling_arm/ros2_ws
./build_all.sh
source install/setup.bash
ros2 pkg executables reachability_gng | grep reach_dwell_monitor   # HARUS muncul
```

⚠️ Disk akar **100 % (16 G sisa)**. Kalau build gagal, curigai disk penuh
lebih dulu, bukan kode.

#### TAHAP 0 — sebelum dan sesudah memasang

```bash
cd ~/Documents/ceiling_arm
python3 scripts/remount_check.py          # bersihkan proses basi DENGAN PID dulu
# ... pasang lengan, beri JEDA setelah power-on (LED tidak amber) ...
python3 scripts/remount_check.py          # sekarang ICMP harus 4/4
```

#### TAHAP 1 — bring-up, **HANYA `arm_1` yang nyata**

```bash
cd ~/Documents/ceiling_arm/ros2_ws && source install/setup.bash
ros2 launch workcell_moveit_config my_workcell.launch.py \
    use_fake_hardware:=false \
    arm2_fake:=true arm3_fake:=true arm4_fake:=true \
    2>&1 | tee /tmp/g16_t1.log
```

🔴 **Dua argumen yang mudah terlewat dan masing-masing punya harga:**

| Argumen | Kenapa | Kalau lupa |
|---|---|---|
| `use_fake_hardware:=false` | **default-nya `true`** | Anda menguji lengan **palsu** dan mengira berhasil |
| `arm{2,3,4}_fake:=true` | bug mid-boot `kortex_driver` membunuh **keempat** controller bersama | memaparkan 4 lengan padahal langkah 2 hanya butuh 1 — **memperbesar area ledakan 4×** tanpa alasan |

⚠️ `2>&1 | tee` **wajib**: pesan abort C++ hanya keluar ke stderr konsol, dan
tanpa itu SIGABRT tidak meninggalkan jejak apa pun di `~/.ros/log`.

```bash
# terminal lain
cd ~/Documents/ceiling_arm && python3 scripts/remount_check.py --ros
```

#### TAHAP 2–3 — torsi istirahat, lalu regresi gerak terkecil

```bash
ros2 topic echo /joint_states --field effort --once     # S3: catat torsi MENGGANTUNG
python3 scripts/hardware_check.py --arms                # regresi via MoveIt
```

#### TAHAP 4 — langkah 2

```bash
# terminal A: PENILAI (memerintah NOL)
ros2 run reachability_gng reach_dwell_monitor --ros-args \
    -p arms:="['arm_1']" -p tool_frames:="['t1_a1_tool_frame']" \
    -p csv_log:=/tmp/g16_step2

# terminal B: PILIH TARGET YANG TERBUKTI TERJANGKAU -- jangan mengarang angka.
# Baca dulu posisi rel NYATA gantry_1, lalu minta kandidatnya:
ros2 topic echo /joint_states --once | grep -A30 t1_linear
python3 scripts/reachable_targets.py --arm arm_1 --lin <hasil_di_atas>

# DRY RUN dulu, selalu. HARI PERTAMA target TETAP, persepsi dilewati (A8b).
python3 scripts/reach_dwell_probe.py --arm arm_1 --approach 0 \
    --target <salin_dari_reachable_targets> --trials 10
# baru setelah jalur MoveIt disambung DAN tahap 3 lulus: tambahkan --move
```

#### Skrip alternatif — mana yang boleh, mana yang tidak

| Skrip | Status | Pakai untuk |
|---|---|---|
| `scripts/start_single_arm.sh` | ✅ **DIPERBAIKI 2026-08-16** — `WS` tadinya menunjuk `~/Documents/moonshot_project/ros2_ws`, path dari repo lain yang **tidak ada**, jadi setiap run mati di `source` | **tahap 3** (regresi). Ia membawa **satu** lengan nyata, jadi lengan yang mid-boot hanya menjatuhkan controller-nya sendiri. ❌ **Tidak cukup untuk tahap 4**: tanpa LIDAR, `/detected_object_pose` tidak pernah terbit |
| `scripts/start_single_rviz.sh` | ✅ path benar | ❌ membawa **keempat** lengan nyata — justru yang A7b hindari |
| `my_workcell.launch.py` + `arm{2,3,4}_fake:=true` | ✅ | **tahap 1 dan 4** — bringup penuh dengan hanya satu lengan nyata |

⚠️ `start_single_arm.sh` masih memakai `pkill -f` di `cleanup()`. Di sini ia
tidak cocok dengan shell-nya sendiri (`start_single_arm.sh` ≠
`single_arm_tables`), jadi **tidak diubah** — mengubah semantik pembersihan
proses pada malam sebelum sesi perangkat keras adalah risiko yang salah untuk
diambil. Tetap jangan tiru polanya.

### A8. 🔒 INSTRUMEN — dan kenapa yang MENILAI bukan yang MEMERINTAH

| Berkas | Peran | Status |
|---|---|---|
| `reachability_gng/reach_dwell_monitor.py` | **penilai murni** — baca TF, nilai, catat. Memerintah **nol** | ✅ ada, tervalidasi 5/5 |
| `scripts/reach_dwell_probe.py` | **pemerintah** — persepsi → pose perintah → publikasi ke monitor → gerakkan lengan | 🆕 **BARU sesi ini** |
| `scripts/remount_check.py` | gerbang tahap 0 dan 1, read-only | 🆕 **BARU sesi ini** |
| `scripts/reachable_targets.py` | pilih target yang **terbukti terjangkau** dari peta kapabilitas, offline | 🆕 **BARU sesi ini** |

#### A8b. 🔴 SUMBER PERSEPSI BERUBAH — LIDAR DIHAPUS, dan hari pertama TIDAK memakai persepsi

Ditemukan saat menyiapkan sesi ini: `livox_ros_driver2` **tidak punya sumber**
di checkout ini (direktori kosong, dan **tidak ada `.gitmodules`** untuk
memulihkannya). Akibatnya `build_all.sh` mati di baris 7 dan
`/detected_object_pose` **tidak akan pernah terbit**. Atas keputusan pengguna
(2026-08-16) seluruh jalur LIDAR **DIHAPUS**; sel ini **RGBD saja**.

| | |
|---|---|
| Sumber pose objek sekarang | `object_localizer` → **`/target_object`**, dari rantai RGBD (`rgbd_perception.launch.py` → segmentasi instans → localizer) |
| Yang dihapus | `lidar_filter.py`, `lidar_processor.py`, `save_pcd.py`, `lidar_filter.launch.py`, updater octomap `livox_lidar`, TF statis `livox_frame`, tahap livox di `build_all.sh` |

🔒 **HARI PERTAMA MEMAKAI `--target X,Y,Z`, BUKAN PERSEPSI — dan itu bukan
kompromi.** Kriteria A1 menilai **PERINTAH → TERCAPAI** (L2); galat persepsi
(L3) **secara eksplisit bukan bagian darinya**. Jadi dari mana `p_cmd` berasal
**tidak mengubah besaran yang diukur** — ia hanya mengubah pose mana yang
diukur. Memakai pose tetap mengukur **L2 yang sama persis** sambil mengeluarkan
kamera, model segmentasi, dan ekstrinsik dari permukaan kegagalan. Kalau
keduanya digabung di hari pertama dan gagal, kegagalannya **tidak dapat
diatribusikan** — dan A5 ada justru untuk mencegah itu.

➜ Persepsi disambung **setelah** L2 terbukti, dan L3 dilaporkan terpisah.

🔴 **TAPI target tetap TIDAK BOLEH dikarang.** Kalau posenya tidak terjangkau,
percobaan 1 kembali sebagai `NO-PLAN` dan itu **bukan** kegagalan metode — ia
kegagalan memilih titik. `scripts/reachable_targets.py` membacanya dari peta
kapabilitas (offline, tanpa perangkat keras) pada posisi rel yang **DIUKUR**:

```bash
python3 scripts/reachable_targets.py --arm arm_1 --lin 0.550
```

Contoh keluaran pada `lin = 0.550`, `rot = 0` — **1014 dari 3132 node
terjangkau**, dan kandidat terdalamnya (paling jauh dari tepi jangkauan, jadi
paling tahan terhadap pergeseran mounting A4): `0.929, 0.247, 1.240`.

⚠️ Pakai `--approach 0`. Offset approach 10 cm default ada untuk menghindari
objek; di sini tidak ada objek, dan offset itu akan **memerintahkan pose yang
tidak pernah diperiksa keterjangkauannya**.

⚠️ `--lin 0.550` berasal dari enkoder **2026-08-13** (`p1_g4 §B3`: g1 =
550.009 mm). Enkoder AZ absolut jadi ia bertahan lintas power-cycle, tetapi
**tetap dibaca ulang** dari `/joint_states` sebelum dipakai.

🔴 **Pemisahan ini WAJIB dan bukan kerapian.** `reach_dwell_monitor` docstring
sudah menyatakannya: *"whatever scores success must not also be what chooses
candidates, otherwise the criterion can drift toward the result."*
`reach_dwell_probe` memublikasikan **pose yang ia perintahkan** ke
`/reach_dwell/target/<arm>` **sebelum** menggerakkan, jadi monitor menilai
`p_cmd` yang sama yang dikirim ke controller — bukan pose yang dipersepsi, dan
bukan pose yang dicapai.

🔒 **L3 dilaporkan terpisah, tidak masuk kriteria sukses:** galat persepsi =
(objek nyata → pose perintah), 3–5 cm. Sukses = (pose perintah → pose
tercapai), < 5 mm. Mencampurnya membuat reviewer membaca "menjangkau dalam
5 cm" (`p1_g4 §A2`).

### A9. 🔒 Papan skor §7.2 — dugaan sesi ini, ditulis SEBELUM perangkat keras dipasang

Papan skor berdiri di **30 meleset, 17 tepat** (`p1_g15 §B6`).
Prior yang relevan, dan ini pertama kalinya prior itu diuji di perangkat keras:
**kendala DUNIA yang belum diukur: tebak LONGGAR. Kode/integrasi SENDIRI:
tebak lebih lambat, lebih rumit, lebih salah.**

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D48** | Langkah 2 **TIDAK lulus** ≥ 8/10 pada sesi pertama, dan sebab dominannya **integrasi**, bukan metode | **kode/integrasi sendiri** | prior kode-sendiri, plus delapan mode kegagalan mesin terdaftar di A5 yang semuanya pernah memakan satu sesi. Ditulis supaya "gagal hari pertama" tidak dibaca sebagai vonis metode |
| **D49** | Dari percobaan yang gagal, **< 20 %** berupa `REACHED-NOT-HELD` | **dunia** | L2 adalah FK dari sendi terukur pada lengan yang **diam**; sekali masuk toleransi, servo menahannya. Yang langka adalah **sampai**, bukan **menahan**. Kalau ini meleset, artinya ada getaran/drift yang belum pernah terukur — dan itu temuan yang lebih menarik daripada dugaannya |
| **D50** | `remount_check.py` menemukan **≥ 1** dari mode A5 aktif pada bring-up pertama | **dunia (mesin)** | delapan mode, semuanya berulang, dan stack baru saja dibongkar. Kalau nol, itu sendiri layak dicatat |

🔒 Dugaan yang instrumennya tidak mengukur besaran yang didalilkan **TIDAK
DINILAI** (D37 G14). Dugaan yang tidak membelah menurut jenis ketika
kenyataannya membelah dihitung **MELESET** (D46 G15).

### A10. Rule 6

Rule 6 (30 000 token) adalah **PENGECUALIAN EKSPLISIT** untuk sesi
protokol-panjang P1, dinyatakan di muka. Anggaran yang mengikat dan
dilaporkan: **10 percobaan**, dan **gerbang per tahap** A7.

---

## B. Hasil terukur

> §A dikunci 2026-08-16, sebelum perangkat keras dipasang. Semua angka di bawah
> keluar sesudahnya. **§A TIDAK ditulis ulang.**

### B0. Garis dasar SEBELUM pemasangan — dijalankan 2026-08-16

`scripts/remount_check.py` dijalankan **hari ini, sebelum lengan dipasang**,
supaya kegagalan besok bisa dibedakan antara **yang dibawa pemasangan** dan
**yang sudah ada sejak awal**. Ini bukan hasil langkah 2; ini garis dasar.

| Pemeriksaan | Hasil hari ini | Bacaan |
|---|---|---|
| `/dev/ttyUSB0`, `/dev/ttyUSB1` | ✅ ada | gantry akan init |
| IP subnet | ✅ **192.168.2.100/24** | mode gantung `ros2_control_node` **tidak** aktif |
| ICMP 4 lengan | ❌ **keempatnya diam** | **diharapkan** — lengan dilepas/tidak bertenaga. Harus jadi ✅ besok |
| web UI Kinova | ✅ tidak ada yang membuka | mode SIGPIPE(−13) tidak aktif |
| `move_group`, `ros2_control_node` basi | ✅ bersih | |
| **`static_transform_publisher` basi** | 🔴 **7 proses** | **mode kegagalan terdaftar A5, AKTIF SEKARANG** |
| **`realsense2_camera_node` basi** | 🔴 **2 proses** | idem |

🔴 **Tujuh `static_transform_publisher` adalah persis bug yang sudah
terdokumentasi**: ia menumpuk tiap peluncuran `realsense_dual` /
`extrinsics_view`, dan semuanya memublikasikan nilai **BERBEDA** ke frame
**YANG SAMA**. Gejalanya "saya sudah ubah config tapi tampilannya tidak pernah
berubah". Kalau dibiarkan sampai besok, ia akan mencemari TF kamera **dan**
membuat setiap kegagalan persepsi tampak seperti kegagalan metode.

🔒 **Dibersihkan SEBELUM pemasangan, dan dengan PID** (`pkill -f` juga membunuh
shell yang memuat polanya — mode kegagalan tersendiri):

```bash
python3 scripts/remount_check.py          # baca PID terkini, jangan salin yang lama
kill <PID...>                             # PID, bukan pola
python3 scripts/remount_check.py          # harus bersih sebelum lanjut
```

⚠️ **Tidak dieksekusi sesi ini** — proses itu milik sesi kamera yang mungkin
masih dipakai. Keputusan mematikannya ada pada operator, dan ia adalah
prasyarat tahap 0 besok, bukan pekerjaan hari ini.

🔒 **D50 BELUM dinilai.** Ia menyebut "bring-up **pertama**", yaitu tahap 0/1
**besok setelah pemasangan** — bukan garis dasar hari ini. Yang hari ini
tunjukkan adalah bahwa instrumennya **bekerja** dan bahwa dua mode A5 sudah
aktif sebelum lengan disentuh.

### B1. Langkah 2

_(diisi 2026-08-17)_

---

## C. Prompt sesi berikutnya — G16-HW (salin ke chat BARU, 2026-08-17)

> **Rekomendasi: Opus 5, effort TINGGI.** Naik dari SEDANG, dan alasannya
> spesifik — bukan karena soalnya lebih rumit, melainkan karena **ini sesi
> pertama di P1 yang bisa merusak perangkat keras**. Tiga hal yang menuntut itu:
> (1) satu-satunya kode yang harus ditulis sesi ini, `move_to()`, adalah persis
> kode yang **menggerakkan lengan nyata**; (2) palang A6/S2 (tolak pose tuck)
> **dideklarasikan tapi BELUM ditegakkan** — tidak ada tempat yang membentuk
> perintah sendi sampai `move_to` ditulis, jadi penegakannya lahir bersama
> risikonya; (3) `fault_controller` tidak di-spawn, jadi satu red LED = **reset
> FISIK**, dan tidak ada undo. Sisanya sudah dikunci di §A dan tinggal diikuti.

```
Sesi G16-HW -- p1_state.md 8c LANGKAH 2: satu lengan reach-and-dwell.
SESI PERANGKAT KERAS. Lengan dipasang hari ini. Ada risiko fisik.

BACA DULU, DAN §A SUDAH DIKUNCI KEMARIN -- JANGAN DITULIS ULANG:
1. docs/p1_g16_hw.md -- SELURUHNYA. Ia ditulis SEBELUM perangkat keras
   dipasang, jadi sebelum satu sampel pun ada. Khususnya:
   A0  (kenapa faktorial G16 dibatalkan -- JANGAN dibuka lagi)
   A1  (definisi sukses TERKUNCI sejak G4, DIPAKAI APA ADANYA)
   A2  (LULUS := >= 8 dari 10; dikunci sebelum ada data)
   A3  (pembagian TIGA ARAH: NO-PLAN / REACHED-NOT-HELD / SUCCESS)
   A4  (pemasangan ulang: L2 dan ekstrinsik TIDAK basi; corner-ref BASI)
   A5  (DELAPAN mode kegagalan mesin + aturan atribusi)
   A6  (keselamatan S1-S7)
   A7  (urutan tahap 0-4, TIDAK dilompati) + A7b (perintah konkret)
   A8  (penilai vs pemerintah) + A8b (LIDAR DIHAPUS; hari 1 TANPA persepsi)
   B0  (garis dasar sebelum pemasangan, sudah diisi)
2. docs/p1_g4_reach_dwell.md A1/A2 -- definisi sukses dan TIGA lapis toleransi
3. scripts/: remount_check.py, reachable_targets.py, reach_dwell_probe.py

=== KEADAAN FISIK ===
Lengan 4x DIPASANG hari ini. Sesi ini MEMAKAI perangkat keras nyata.
Disk root 100 % (16 G sisa) -- kegagalan tulis dibaca sebagai disk penuh dulu.

=== SUDAH SELESAI SEMALAM (2026-08-16), JANGAN ULANGI ===
- Proses basi DIBERSIHKAN: 5 yatim + 1 pohon launch realsense. /proc bersih
  untuk keempat pola. (Bunuh dengan PID; induk mati meninggalkan anak yatim --
  itu terjadi semalam dan anaknya harus dibunuh terpisah.)
- Workspace DIBANGUN ULANG: 22 paket, reach_dwell_monitor SUDAH TERPASANG.
  Verifikasi: ros2 pkg executables reachability_gng | grep reach_dwell_monitor
- LIDAR/Livox DIHAPUS TOTAL. Sel ini RGBD saja. build_all.sh dulu MATI di
  baris 7 karena livox tak bersumber -- itulah sebab reach_dwell_monitor tidak
  pernah terpasang, bukan lupa build.
- start_single_arm.sh DIPERBAIKI (dulu menunjuk repo moonshot_project).

=== TUGAS, BERURUTAN. TAHAP TIDAK DILOMPATI (A7). ===
0. python3 scripts/remount_check.py           -> ICMP harus 4/4
1. bring-up, HANYA arm_1 nyata (A7b), lalu remount_check.py --ros
   -> /joint_states ~96 Hz dan 28 nilai. TEPAT 10 Hz = ros2_control MATI.
2. torsi istirahat MENGGANTUNG, dicatat
3. REGRESI arm_1 joint_6 +5 deg. Ini sudah pernah BERHASIL (2026-08-12), jadi
   gagal di sini = pemasangan ulang, BUKAN metode.
4. LANGKAH 2, 10 percobaan. Target dari reachable_targets.py (--approach 0),
   BUKAN angka karangan. Monitor menilai, probe memerintah, proses TERPISAH.

=== SATU-SATUNYA KODE YANG DITULIS SESI INI ===
reach_dwell_probe.move_to() -- sengaja dibiarkan NotImplementedError semalam.
Disambung SETELAH tahap 3 LULUS, dengan lengan terpasang dan ada yang
mengawasi. Saat menyambungnya, TEGAKKAN A6/S2 DI SITU: tolak goal yang vektor
sendinya dekat FORBIDDEN_TUCK. Konstantanya sudah ada, penegakannya belum.
Pakai ulang jalur MoveIt hardware_check.py --arms; jangan tulis penggerak baru.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- use_fake_hardware DEFAULT true. Lupa = menguji lengan PALSU dan mengira lulus.
- arm{2,3,4}_fake:=true. Bug mid-boot kortex_driver membunuh KEEMPAT controller
  bersama; langkah 2 cuma butuh satu lengan.
- 2>&1 | tee WAJIB: pesan abort C++ hanya ke stderr konsol, tidak ke ~/.ros/log.
- Tab browser ke https://192.168.2.1x = sesi Kortex kedua = SIGPIPE(-13)
  mid-sesi. TUTUP.
- pkill -f membunuh shell yang memuat polanya. PAKAI PID. Dan membunuh induk
  launch MENINGGALKAN ANAK YATIM -- periksa ulang setelah kill.
- `tail` pada proses latar MENELAN keluaran sampai proses selesai.
- Skrip di /tmp tidak mewarisi cwd repo: sys.path ABSOLUT.
- --approach default 0.10 memerintahkan pose yang keterjangkauannya TIDAK
  pernah diperiksa. Pakai --approach 0 untuk uji L2.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 30 meleset, 17 tepat.
  Dugaan sesi ini D48-D50 SUDAH ditulis di muka (A9). Nilai apa adanya.
  Prior KODE SENDIRI: pembacaan kode menemukan LETAK, tidak dapat
  memperkirakan AKIBAT. Prior PALING BERGUNA: setiap temuan nyata datang dari
  UJI YANG DIJALANKAN -- delapan sesi berturut-turut.
- ATRIBUSI (A5): percobaan yang gagal karena mode mesin dicatat TIDAK VALID
  dengan gejalanya disebut, DIULANG, dan TIDAK masuk penyebut 10. Ia BUKAN
  kegagalan metode. Ini yang menjaga hari pertama tidak salah dibaca.
- DILARANG menggeser ambang A1 (5 mm / 5 deg / 2.0 s) setelah melihat data.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 lima, G11 empat, G12 dua, G13 dua, G14 empat, G15 dua.
- Rule 6 PENGECUALIAN EKSPLISIT untuk sesi protokol-panjang P1 (A10).
- Isi §B docs/p1_g16_hw.md. Akhiri dengan prompt sesi berikutnya.
```
