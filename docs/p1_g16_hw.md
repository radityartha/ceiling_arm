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
| **Jumlah percobaan** | **10 percobaan** pada **satu lengan** (`arm_1`), **pose berbeda-beda** — dibangkitkan `reachable_targets.py --spread 10` dan **dikunci sebagai daftar sebelum percobaan pertama** (§C tugas 4). Titik yang NO-PLAN dilaporkan begitu, **tidak** diganti |
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
python3 scripts/reachable_targets.py --arm arm_1 --lin <hasil_di_atas> --spread 10

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
| `scripts/reachable_targets.py` | bangkitkan **daftar titik percobaan** yang terbukti terjangkau + **tersebar** (`--spread 10`), offline dari peta kapabilitas | 🆕 **BARU sesi ini** |

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

### B1. Gerbang tahap 0–2 — dijalankan 2026-08-17 SETELAH pemasangan

#### B1.1 TAHAP 0 — GERBANG LULUS, nol temuan

| Pemeriksaan | Hasil | Lawan garis dasar B0 |
|---|---|---|
| `/dev/ttyUSB0`, `/dev/ttyUSB1` | ✅ ada | sama |
| IP subnet | ✅ `192.168.2.100/24` | sama |
| **ICMP 4 lengan** | ✅ **4/4 menjawab** | 🔄 **berubah dari 0/4** — inilah bukti pemasangan berhasil |
| web UI Kinova (443) | ✅ keempatnya tertutup | sama |
| `move_group` / `ros2_control_node` basi | ✅ bersih | sama |
| **`static_transform_publisher` basi** | ✅ **0** | 🔄 **berubah dari 7** — pembersihan semalam bertahan |
| **`realsense2_camera_node` basi** | ✅ **0** | 🔄 **berubah dari 2** |

#### B1.2 🔴 D50 MELESET — dan itu hasil, bukan ketiadaan hasil

> **D50** (A9): *"`remount_check.py` menemukan **≥ 1** dari mode A5 aktif pada
> bring-up pertama."*

**Nol dari delapan mode A5 aktif di tahap 0.** A9 sudah menuliskan di muka apa
artinya kalau ini terjadi — *"Kalau nol, itu sendiri layak dicatat"* — jadi ini
dinilai **MELESET**, bukan dibuang.

Kenapa meleset, dan ini yang membuatnya berguna: tujuh dari delapan mode A5
adalah **higienis proses**, bukan sifat perangkat keras. B0 menemukan 9 proses
basi semalam dan semuanya **dibunuh sebelum pemasangan**. Jadi D50 sebenarnya
memprediksi *"pembersihan semalam tidak akan bertahan"*, dan ia bertahan.
Papan skor §7.2 → **31 meleset, 17 tepat**.

#### B1.3 TAHAP 1 — bring-up, `arm_1` NYATA

`use_fake_hardware:=false arm2_fake:=true arm3_fake:=true arm4_fake:=true`

| Bukti | Nilai |
|---|---|
| `Connecting to robot at 192.168.2.13` → `Session created` | ✅ lengan **nyata**, bukan palsu |
| **`Actuator count reported by robot is '6'`** | ✅ **bukan `'0'`** — mode SIGABRT(−6) mid-boot **tidak** terpicu |
| `/joint_states` | ✅ **93.9 Hz, 37 nilai**, `effort` terisi |
| TF `world → t{1,2}_a{1,2}_tool_frame` | ✅ keempatnya hidup |
| `dual_table_controller` | ✅ kedua meja init, `Publishing joint states at 10.0 Hz` |

#### B1.4 🔴 PERTENTANGAN §B lawan §A — DUA, dan keduanya soal INSTRUMEN

Aturan: kalau §B bertentangan dengan §A, **§B menang** dan pertentangannya
ditulis. G10 lima, G11 empat, G12 dua, G13 dua, G14 empat, G15 dua, **G16 dua**.

**(1) `remount_check.py --ros` melaporkan GERBANG GAGAL atas stack-nya sendiri.**

A7b menyuruh menjalankan `remount_check.py --ros` *setelah* bring-up. Tapi
pemindaian proses basi tahap 0 ikut jalan, dan ia menemukan:

```
FAIL  proses basi: move_group          PID [1447582]
FAIL  proses basi: ros2_control_node   PID [1447586]
```

Kedua PID itu adalah **anak dari launch yang baru saja diminta A7b**. Ini
**positif palsu secara konstruksi**: `--ros` hanya bermakna kalau stack hidup,
dan pemindaian basi hanya bermakna kalau stack mati. Keduanya tidak bisa benar
sekaligus. Tahap 1 **DILULUSKAN**; kegagalan itu artefak instrumen, bukan mesin.

➜ Perbaikan yang benar (untuk sesi mendatang, **tidak** dikerjakan hari ini —
sesi perangkat keras bukan tempat mengubah gerbang keselamatan): `--ros`
seharusnya melewati pemindaian proses basi, atau melaporkannya sebagai INFO.

**(2) Cek lebar `/joint_states` LULUS karena alasan yang salah.**

`remount_check.py:187` menguji `width >= 28`. Terbaca **37**, jadi ✅. Tetapi 37
= `arm_1` (6+1) + tiga lengan **palsu** (6+4 masing-masing) = 7+30. **Nol sendi
gantry** ada di pesan itu. Ambang `>= 28` lolos justru karena gripper palsu
menyumbang 12 sendi ekstra yang menutupi kekurangannya.

Penyebabnya **bukan** kerusakan: `joint_state_broadcaster` (~94 Hz) dan
`dual_table_controller` (10 Hz) menerbitkan ke topik `/joint_states` yang
**sama** dalam **pesan terpisah** — 661 pesan dalam 6 s, **60** di antaranya
memuat sendi gantry. Digabung per-nama hasilnya **41 nama unik**.

🔴 **Konsekuensi yang wajib dipatuhi siapa pun yang membaca `/joint_states` di
sel ini:** `--once` mengembalikan **pesan mana pun yang kebetulan tiba**, jadi
`ros2 topic echo /joint_states --once | grep t1_linear` — persis perintah di
A7b tugas 4 — **gagal diam-diam ~90 % waktu**. Harus digabung sepanjang jendela
≥ 0.5 s. Pembacaan B1.5 di bawah memakai jendela 6 s.

#### B1.5 TAHAP 2 — torsi istirahat MENGGANTUNG, dan posisi rel TERUKUR

Pose istirahat `arm_1` terbaca (rad):
`j1 −0.4635, j2 +0.1071, j3 +0.1292, j4 −1.3865, j5 −0.1765, j6 +1.7388`
— **bukan** `FORBIDDEN_TUCK`; lengan menggantung bebas (S1).

| Sendi | effort (N·m) |
|---|---|
| `t1_a1_joint_1` | −0.021 |
| `t1_a1_joint_2` | **+0.069** ← puncak |
| `t1_a1_joint_3` | −0.007 |
| `t1_a1_joint_4` | −0.007 |
| `t1_a1_joint_5` | +0.007 |
| `t1_a1_joint_6` | −0.003 |
| `t1_a1_right_finger_bottom_joint` | `nan` |

🔒 **Puncak 0.069 N·m** lawan tuck **12.78 / 14 N·m** dan `--tau-max` 10.0 →
**185× margin**. GERBANG TAHAP 2 **LULUS**.

⚠️ Effort gripper adalah **`nan`**. Penjaga torsi probe (`_on_js`) memakai
`abs(eff) > tau_peak`, dan setiap perbandingan dengan `nan` bernilai False —
jadi ia **tidak** memicu abort palsu, tetapi gripper **tidak terpantau**. Dicatat,
tidak diubah: perilakunya aman ke arah yang benar.

**Posisi rel, DIBACA ULANG dari perangkat keras (bukan diasumsikan):**

| Sendi | Terukur 2026-08-17 | 2026-08-13 (`p1_g4 §B3`) |
|---|---|---|
| **`t1_linear_joint`** | **0.550009 m = 550.009 mm** | **550.009 mm** |
| `t1_rotation_joint` | −0.000175 rad ≈ 0° | — |
| `t2_linear_joint` | −0.000010 m | — |
| `t2_rotation_joint` | −0.030543 rad | — |

🔒 **Identik sampai mikrometer lintas pembongkaran, power-cycle, dan pemasangan
ulang.** Enkoder absolut AZ bertahan persis seperti yang diklaim A8b — dan
sekarang itu **terukur**, bukan diwarisi. `--lin 0.550 --rot 0` sah.

#### B1.6 🔒 DAFTAR TITIK PERCOBAAN — DIKUNCI SEBELUM PERCOBAAN PERTAMA

Dibangkitkan dari peta kapabilitas pada rel **terukur** di atas, **sebelum**
satu percobaan pun dijalankan:

```
python3 scripts/reachable_targets.py --arm arm_1 --lin 0.550 --rot 0 --spread 10
```

`1014 dari 3132 node terjangkau` (L1, tol 5 cm); jangkauan
`x 0.29..1.64  y −0.32..0.60  z 1.00..1.40`.
**Pemisahan min 32.8 cm, median 45.7 cm** — A2 menuntut pose BERBEDA, dan ini
memenuhinya (tanpa `--spread`, sepuluh teratas adalah tetangga grid ~7 cm,
yaitu satu pose diukur sepuluh kali).

| # | target (x, y, z) world | | # | target (x, y, z) world |
|---|---|---|---|---|
| 1 | `0.929, 0.247, 1.240` | | 6 | `0.857, 0.529, 1.000` |
| 2 | `1.286, 0.106, 1.080` | | 7 | `0.571, 0.318, 1.160` |
| 3 | `1.214, 0.529, 1.240` | | 8 | `0.786, 0.529, 1.400` |
| 4 | `1.000, −0.106, 1.400` | | 9 | `1.214, 0.176, 1.400` |
| 5 | `0.714, −0.035, 1.080` | | 10 | `0.643, 0.106, 1.400` |

🔒 Urutan ini **dikunci**. Titik yang kembali `NO-PLAN` dilaporkan sebagai
`NO-PLAN` (A3) dan **tidak diganti** dengan titik yang lebih mudah.

#### B1.7 🔴 TAHAP 3 TERBLOKIR — controller lengan TIDAK AKTIF

Ini blocker nyata sesi ini, dan ia ditemukan **sebelum** ada perintah gerak.

```
joint_state_broadcaster        active
gripper_1..4_controller        active
gantry_1_with_arm_controller   INACTIVE   <--
gantry_2_with_arm_controller   INACTIVE   <--
```

`gantry_N_with_arm_controller` adalah **satu-satunya** controller trajektori
lengan yang di-spawn `my_workcell.launch.py`
([my_workcell.launch.py:106-112](../ros2_ws/src/workcell_moveit_config/launch/my_workcell.launch.py#L106-L112)),
dan MoveIt merutekan grup `arm_1` ke sana lewat `moveit_controllers.yaml`.
Aktivasinya ditolak:

```
Not acceptable command interfaces combination:
  Start interfaces: [ t1_linear_joint/position, t1_rotation_joint/position,
                      t1_a1_joint_1..6/position, t1_a2_joint_1..6/position ]
  Not existing:     [ t1_linear_joint/position, t1_rotation_joint/position ]
```

**Antarmuka perintah lengan ADA; yang hilang hanya yang gantry.** Sebabnya
bukan kerusakan melainkan **default yang disengaja**:

```python
# my_workcell.launch.py:60
DeclareLaunchArgument("enable_gantry_bridge", default_value="false", ...)
# "Defaults to false even on real hardware so a first real-mode launch
#  doesn't move the tables until reviewed."
```

Tanpa bridge itu, URDF tidak mengekspor antarmuka **perintah** gantry, jadi
controller gabungan 14-sendi tidak pernah bisa aktif — dan karena ia satu-satunya
jalur eksekusi lengan, **tahap 3 dan tahap 4 dua-duanya terblokir**.

➜ Ini **persis dugaan D48**: *"sebab dominannya **integrasi**, bukan metode."*
Dinilai penuh di B2 setelah tahap 3/4 selesai.

**Catatan penting soal risiko:** `gantry_1_with_arm_controller` menyetel
`allow_partial_joints_goal: true` ([ros2_controllers.yaml:170](../ros2_ws/src/workcell_moveit_config/config/ros2_controllers.yaml#L170)).
Rencana untuk grup `arm_1` memuat **6 sendi lengan saja**, jadi sendi gantry
**tidak masuk goal dan tidak diperintahkan** — gantry tetap diam di 550.009 mm
selama langkah 2. Mengaktifkan bridge membuat controller bisa aktif; ia **tidak**
membuat gantry bergerak.

### B2. Blocker DIBUKA, dan TAHAP 3 LULUS

#### B2.1 Keputusan operator (2026-08-17), dan kenapa risikonya kecil

Diputuskan **`enable_gantry_bridge:=true`**, nol perubahan kode:

```bash
ros2 launch workcell_moveit_config my_workcell.launch.py \
    use_fake_hardware:=false \
    arm2_fake:=true arm3_fake:=true arm4_fake:=true \
    enable_gantry_bridge:=true
```

Hasil — **ketujuh controller AKTIF**, termasuk yang tadinya `INACTIVE`:

```
gantry_1_with_arm_controller  active   <-- tadinya INACTIVE
gantry_2_with_arm_controller  active
joint_state_broadcaster       active
gripper_1..4_controller       active
```

🔒 Ini **mengonfirmasi diagnosis B1.7 secara kausal**, bukan sekadar korelasi:
satu flag diubah, dan tepat gejala yang diprediksi hilang.

#### B2.2 🔴 Penjaga slam-to-zero bridge TERBUKTI BEKERJA di perangkat keras nyata

Temuan tak terduga, dan berharga. Saat bridge naik:

```
WARN  bridge: table1 NOT armed -- ignoring command 0.0mm/0.0deg while the
      gantry is at 550.0mm/-0.0deg. No active controller is holding this joint.
INFO  bridge: table1 ARMED at 550.0mm/-0.0deg -- now following ros2_control commands
```

Bridge **menerima perintah 0.0 mm** dan **MENOLAKNYA**, lalu arm pada 550.0 mm.
Itu persis bug *uncommanded-slam-to-zero* yang diperbaiki 2026-08-12
([[gantry-bridge-status]]) — dan ini pertama kalinya **penjaganya** terlihat
memicu pada perangkat keras nyata. Kalau tidak ada, gantry akan melesat 550 mm
ke 0 tanpa diperintah, saat lengan tergantung di atasnya.

Terverifikasi sesudahnya: `t1_linear_joint = 0.550009 m` — **tidak bergerak
sedikit pun** akibat mengaktifkan bridge.

#### B2.3 TAHAP 3 — GERBANG LULUS

A7 mendefinisikan tahap 3 sebagai *"gerak terkecil yang sudah pernah berhasil:
`arm_1` joint_6 +5 deg lewat ros2_control"*. Dijalankan persis begitu — goal
**satu sendi** (`allow_partial_joints_goal`), jadi 13 sendi lain, **termasuk
kedua sendi gantry, tidak ada di goal dan tidak pernah diperintahkan**.

| | Nilai |
|---|---|
| mulai | `+1.73880 rad` (+99.63°) |
| target | `+1.82607 rad` (+104.63°) |
| **tercapai** | `+1.82595 rad` — galat **−0.007°** |
| `error_code` | **0** |
| **kembali** | `+1.73884 rad` — galat **+0.002°** |
| **torsi puncak** | **0.592 N·m** |
| red LED / fault | **tidak ada** |
| gantry sesudahnya | `0.550009 m` — **tidak bergerak** |
| istirahat sesudahnya | `\|tau\| maks **0.013 N·m** |

🔒 **GERBANG TAHAP 3 LULUS.** Torsi puncak **0.592 lawan batas firmware 14 N·m**
= margin **24×**; lawan tuck 12.78 N·m = **21×**. Pemasangan ulang **tidak**
merusak jalur eksekusi: lengan bergerak, kembali, tidak ada fault — kriteria A7
persis.

⚠️ **Pertentangan ketiga §B lawan §A (kecil, dan tidak diambil).** A7b memberi
perintah konkret `hardware_check.py --arms`, tetapi itu memerintahkan **pose
home** (`j3` +68°, `j4` +79°, `j6` −100° dari posisi menggantung) pada **keempat**
lengan — jauh lebih besar dari *"gerak terkecil"* yang **A7 sendiri** definisikan,
dan sweep `j3` +68° melawan gravitasi justru tempat torsi memuncak. Dipilih **A7
(niat terkunci)** di atas A7b (perintah kemudahan), sesuai Rule 7. Home **tidak**
diperintahkan sesi ini.

#### B2.4 🔒 Papan skor §7.2 — apa yang dinilai dan apa yang TIDAK

| # | Vonis | Alasan |
|---|---|---|
| **D48** | ⏸️ **TIDAK DINILAI** | ia menyebut hasil **10 percobaan langkah 2**, dan langkah 2 **tidak dijalankan**. Menilainya dari blocker saja akan melanggar aturan A9 (dugaan yang instrumennya tidak mengukur besaran yang didalilkan tidak dinilai). **Catatan:** *mekanismenya* terlihat persis seperti yang didalilkan — yang menghentikan sesi ini adalah **integrasi** (`enable_gantry_bridge`), bukan metode. Itu bukti pendukung, **bukan** skor |
| **D49** | ⏸️ **TIDAK DINILAI** | nol percobaan, jadi nol kegagalan untuk dibagi |
| **D50** | ❌ **MELESET** | nol dari delapan mode A5 aktif — lihat B1.2 |

➜ Papan skor: **31 meleset, 17 tepat**.

#### B2.5 Yang TIDAK dikerjakan, dan kenapa — dinyatakan terbuka (Rule 12)

| Belum dikerjakan | Sebab |
|---|---|
| `reach_dwell_probe.move_to()` | masih `NotImplementedError`. **Sengaja** — A6/S7 menuntut persetujuan manusia eksplisit untuk gerak, dan persetujuan sesi ini dibatasi pada **regresi tahap 3 saja**, berhenti sebelum percobaan langkah 2 |
| **Tahap 4 — 10 percobaan langkah 2** | butuh `move_to()`, yang butuh persetujuan gerak di atas |
| `hardware_check.py --arms` (pose home) | lihat B2.3 — sengaja tidak diambil |
| Persepsi (`/target_object`) | A8b: hari pertama **tanpa** persepsi, sesuai rencana. Daftar B1.6 adalah pose tetap |

🔒 **Yang SUDAH terbukti dan tidak perlu diulang sesi berikutnya:** tahap 0, 1, 2
dan 3 semuanya LULUS di perangkat keras nyata, daftar sepuluh titik **sudah
dikunci** (B1.6), posisi rel **terukur** 550.009 mm, dan jalur eksekusi
`gantry_1_with_arm_controller` **terbukti menggerakkan lengan nyata dengan
`error_code = 0`**. Sesi berikutnya mulai dari `move_to()`.

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
4. BUAT TITIK PERCOBAAN DULU, SEBELUM percobaan pertama dijalankan:
     ros2 topic echo /joint_states --once | grep -A30 t1_linear   # lin NYATA
     python3 scripts/reachable_targets.py --arm arm_1 --lin <lin> --spread 10
   SALIN kesepuluh barisnya ke docs/p1_g16_hw.md §B1 APA ADANYA, sebagai
   daftar terkunci, SEBELUM satu percobaan pun jalan. Alasannya bukan kerapian:
   memilih ulang titik setelah melihat hasil = memilih hasilnya.
   -> --spread WAJIB. Tanpa itu titik teratas adalah tetangga grid ~7 cm,
      yaitu SATU pose diukur 10x, dan A2 menuntut pose BERBEDA. Dengan
      --spread pemisahannya min ~33 cm.
   -> --approach 0 WAJIB. Offset 10 cm default memerintahkan pose yang
      keterjangkauannya TIDAK pernah diperiksa.
   -> Titik yang ternyata NO-PLAN TETAP dilaporkan sebagai NO-PLAN (A3).
      Ia TIDAK diganti dengan titik lain yang lebih mudah.
5. LANGKAH 2, 10 percobaan memakai daftar itu. Monitor menilai, probe
   memerintah, proses TERPISAH.

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

---

### B3. `move_to()` disambung; DRY RUN 10 titik; PERCOBAAN 1 dijalankan

Mandat gerak dari operator: **percobaan 1 sendirian, lalu lapor.** Dipatuhi.

#### B3.1 `move_to()` — rencanakan, PERIKSA, baru eksekusi

Jalur MoveIt `hardware_check.py --arms` dipakai ulang (action `MoveGroup`,
skala kecepatan 15 %); tidak ada penggerak baru. Satu penyimpangan, dan
alasannya A6/S2:

> **Goal berupa POSE tidak menyebut sendi sama sekali.** Jadi tidak ada apa pun
> untuk diperiksa terhadap `FORBIDDEN_TUCK` sampai MoveIt menghasilkan
> lintasan. Karena itu `move_to()` memakai `plan_only=True` dulu, memeriksa
> **setiap** titik lintasan terhadap tuck (bukan hanya titik akhir — tuck di
> tengah jalan sama berbahayanya), baru mengeksekusi lewat `ExecuteTrajectory`.

Merencanakan gratis; mengeksekusi tidak. Ini menempatkan penegakan S2 tepat di
titik perintah sendi benar-benar ada, sesuai amanat A8.

Toleransi perencana sengaja **lebih ketat** dari palang penilaian (2 mm / 2°
lawan 5 mm / 5°) supaya yang diukur adalah pelacakan eksekusi, bukan seberapa
longgar daerah goal membiarkan perencana berhenti. Roll dibebaskan
(`absolute_z_axis_tolerance = 2π`) persis seperti A1 menuntut.

#### B3.2 DRY RUN kesepuluh titik — 8 PLANNED, 2 NO-PLAN

Dijalankan dengan `--plan-check` (minta rencana, **jangan** eksekusi):

| # | target | rencana | | # | target | rencana |
|---|---|---|---|---|---|---|
| 1 | `0.929, 0.247, 1.240` | ✅ PLANNED | | 6 | `0.857, 0.529, 1.000` | ✅ PLANNED |
| **2** | `1.286, 0.106, 1.080` | 🔴 **NO-PLAN** | | 7 | `0.571, 0.318, 1.160` | ✅ PLANNED |
| 3 | `1.214, 0.529, 1.240` | ✅ PLANNED | | 8 | `0.786, 0.529, 1.400` | ✅ PLANNED |
| 4 | `1.000, −0.106, 1.400` | ✅ PLANNED | | 9 | `1.214, 0.176, 1.400` | ✅ PLANNED |
| **5** | `0.714, −0.035, 1.080` | 🔴 **NO-PLAN** | | 10 | `0.643, 0.106, 1.400` | ✅ PLANNED |

🔒 Keduanya **dilaporkan sebagai NO-PLAN dan TIDAK diganti** (A3/A8b).

🔴 **Konsekuensi aritmetika, dan ia muncul SEBELUM satu percobaan pun bergerak:**
plafon sekarang **8 dari 10**, sedangkan A2 menuntut **≥ 8 dari 10**. Jadi
langkah 2 hanya LULUS kalau **kedelapan** percobaan tersisa SUKSES penuh —
nol margin. Ini bukan alasan menggeser A2; ini fakta yang harus dilaporkan.

**Kenapa NO-PLAN padahal peta kapabilitas bilang terjangkau:** peta memakai
L1 = **5 cm** dan hanya menanyakan "ada solusi di dekat sini". MoveIt memakai
IK eksak + pemeriksaan tabrakan. Selisih itu **memang** lapisan yang A2 g4
definisikan, dan sekarang terukur: **2 dari 10 (20 %)** titik L1-terjangkau
tidak punya rencana nyata.

#### B3.3 🔴 PERCOBAAN 1 — penilai bilang SUKSES, pemerintah bilang TORQUE-ABORT

Inilah kenapa A8 memisahkan penilai dari pemerintah. Keduanya benar tentang
hal yang **berbeda**.

**Penilai (`reach_dwell_monitor`, `/tmp/g16_step2_summary.csv`):**

```
event   arm    pos_err_max_mm  ori_err_max_deg  n_window_samples  rate_hz  n_tf_fail  instrument_ok
success arm_1  4.342           2.257            42                20.0     0          True
```

| Kriteria A1 (TERKUNCI) | Diukur | Palang | |
|---|---|---|---|
| posisi | **4.342 mm** (mean 1.615) | < 5 mm | ✅ |
| orientasi | **2.257°** (mean 2.237) | < 5° | ✅ |
| dwell | **2.0 s KONTINU**, 42 sampel @ 20.0 Hz | ≥ 10 Hz | ✅ |
| `instrument_ok` | **True**, `n_tf_fail = 0` | — | ✅ |

MoveIt: `Motion plan was computed successfully` → `Execution completed: SUCCEEDED`.

➜ **Menurut definisi sukses terkunci dan penilai independen, percobaan 1 adalah
SUCCESS penuh.** Ambang A1 **tidak disentuh**.

**Pemerintah (`reach_dwell_probe`)** mencatat **`TORQUE-ABORT`**: penjaga
`--tau-max` **10.0 N·m** terlampaui oleh puncak **10.398 N·m** di
`t1_a1_joint_2`, ~1.9 s **sebelum** jendela dwell selesai. Probe berhenti
memantau; lengan tidak diperintah berhenti, jadi dwell tetap tercapai dan
penilai mencatatnya.

**Torsi terukur sepanjang percobaan** (`peak_effort_nm` dari penilai):

| sendi | j1 | **j2** | j3 | j4 | j5 | j6 |
|---|---|---|---|---|---|---|
| N·m | 1.251 | **10.398** | 3.986 | 0.499 | 0.539 | 0.592 |

| Keadaan | \|tau\| maks |
|---|---|
| istirahat MENGGANTUNG | **0.069** |
| **puncak transien saat menjangkau** | **10.398** |
| **tahan statis di target** | **5.886** |
| puncak saat kembali (12 s, pelan) | 6.092 |
| istirahat setelah kembali | 0.337 |

🔴 **Temuan fisik, dan ini BUKAN kegagalan metode:** menjangkau ke luar dari
langit-langit membebani bahu (`joint_2`) jauh lebih berat daripada yang
diantisipasi siapa pun. Kesepuluh titik dipilih **murni dari keterjangkauan
kinematik** — peta kapabilitas **tidak menyaring torsi sama sekali**. Sepupu
temuan [[ceiling-arm-fold-is-torque-limited]]: di sel ini, **torsi, bukan
kinematika, yang mengikat**.

⚠️ Nilai `--tau-max = 10.0` berasal dari skrip semalam, dipilih terhadap tuck
12.78 N·m **tanpa satu pun pengukuran torsi menjangkau**. Ia sekarang terbukti
**di bawah torsi menjangkau yang sah**. Ia **bukan** ambang A1 dan tidak
tunduk pada larangan menggeser ambang — tetapi menggesernya tetap **keputusan
operator**, bukan keputusan saya, dan tidak digeser sesi ini.

🔒 **Tidak ada fault.** Tidak ada red LED, tidak ada `Kortex exception`, tidak
ada SIGPIPE. Lengan **dikembalikan ke pose MENGGANTUNG** dengan interpolasi
sendi langsung 12 s (bukan rencana MoveIt — tujuannya sudah diketahui baik, dan
perencana bebas merutekan lewat konfigurasi yang tidak): galat maks **0.033°**,
puncak 6.092 N·m, istirahat 0.337 N·m.

#### B3.4 🔴 PERTENTANGAN §B lawan §A — TIGA LAGI, semuanya INSTRUMEN

G16 kini **enam** total. G10 lima, G11 empat, G12 dua, G13 dua, G14 empat,
G15 dua, **G16 enam**.

**(4) `REACHED-NOT-HELD` MUSTAHIL terdeteksi — pembagian A3 runtuh diam-diam.**
`reach_dwell_monitor` menerbitkan ke `/reach_dwell/status` **hanya saat dwell
BERHASIL** (`_publish` dipanggil di dua tempat: `concurrent` dan `success`). Ia
**tidak pernah** menerbitkan status per-sampel, jadi kunci `in_tol` yang
`reach_dwell_probe.verdict()` cari **tidak pernah ada di topik**. Akibatnya
setiap kegagalan runtuh menjadi `NO-PLAN` — persis peleburan yang A3 menyatakan
**wajib** dihindari. Diperbaiki tanpa menyentuh penilai: probe membaca
`<csv_log>_samples.csv`, yang penilai sudah tulis dan `flush()` tiap sampel.

**(5) A3 kekurangan satu ember: direncanakan, dieksekusi, TAPI meleset.** A3
mengandaikan kegagalan hanya NO-PLAN atau REACHED-NOT-HELD. Perangkat keras
nyata punya yang ketiga: rencana ada, eksekusi SUCCEEDED, lengan berhenti
**di luar 5 mm dan tidak pernah menyentuh toleransi sama sekali**. Memaksanya
ke salah satu ember lama akan menyalahkan peta kapabilitas atas galat
pelacakan. Ditambahkan sebagai **`EXEC-MISS`**, dilaporkan terpisah.

**(6) 🔴 BLOKER: `publish_target()` KALAH BALAPAN dengan penemuan DDS.**
Log penilai memuat **tepat satu** `target set` dan **satu** `cleared` untuk
**sebelas** publikasi (10 dry run + 1 percobaan nyata):

```
1786942881.32  cleared: arm_1
1786942881.41  arm_1: target set (0.929, 0.247, 1.240) in world
1786943043.30  >>> arm_1: SUCCESS ...
```

`publish_target()` menerbitkan lalu spin **5 × 0.02 s = 0.1 s**, jauh lebih
pendek dari penemuan DDS untuk publisher yang baru dibuat di proses baru.
**Sepuluh dari sebelas publikasi HILANG.**

🔴 **Kenapa ini menyelamatkan sesi, bukan merusaknya:** percobaan 1 dinilai
benar **hanya karena kebetulan** — target yang tertinggal di penilai (dari dry
run titik 1) **kebetulan koordinat yang sama** dengan percobaan nyata titik 1.
Untuk percobaan 2–10 koordinatnya **berbeda**, jadi penilai akan menilai
kesembilan percobaan berikutnya terhadap **target titik 1 yang basi** — dan
hasilnya akan terlihat **masuk akal**, bukan rusak. Jaminan A8b (*"probe
memublikasikan pose yang ia perintahkan SEBELUM menggerakkan, jadi monitor
menilai `p_cmd` yang sama"*) **tidak benar-benar berlaku** di percobaan 1; ia
diwarisi dari dry run.

➜ **Wajib diperbaiki sebelum percobaan 2:** `publish_target()` harus menunggu
`get_subscription_count() > 0` (lalu beri jeda kecil), dan probe harus
**memverifikasi** penilai menerima target — jangan percaya penerbitan.

#### B3.5 Skor: D48/D49 masih BELUM dinilai

Satu percobaan valid bukan sepuluh. **D48 dan D49 tetap TIDAK DINILAI**;
menilainya dari n=1 melanggar A9. Papan skor tetap **31 meleset, 17 tepat**.

Catatan pendukung (bukan skor): kedua penemuan pemblokir sesi ini —
`enable_gantry_bridge` dan balapan penemuan DDS — adalah **integrasi**, dan
keduanya ditemukan dengan **menjalankan**, bukan membaca kode. Prior "kode
sendiri: lebih lambat, lebih rumit, lebih salah" bertahan.

### B4. Tahap 4 DIHENTIKAN pada percobaan 4 — torsi, bukan metode

Setelah bloker B3.4(6) diperbaiki (`publish_target` menunggu pencocokan DDS —
diverifikasi: `target set` baru mendarat), kesepuluh percobaan dijalankan dari
daftar terkunci B1.6. **Dihentikan di percobaan 4 atas alasan keselamatan.**

Perubahan protokol yang ditambahkan sesi ini dan harus disebut: **lengan
dikembalikan ke pose MENGGANTUNG sebelum setiap percobaan**, supaya tiap
percobaan berangkat dari konfigurasi yang sama. Tidak ada di protokol asli;
ditambahkan setelah percobaan 1 mengukur transien 10.4 N·m, karena
merencanakan target-ke-target akan merutekan antara dua pose yang sudah
terbebani tanpa ada yang memeriksa apa yang dilewatinya.

#### B4.1 Hasil, empat percobaan

| # | target | vonis probe | penilai (A1) | pos maks | ori maks | **torsi puncak** |
|---|---|---|---|---|---|---|
| 1 | `0.929, 0.247, 1.240` | ✅ SUCCESS | success | **4.281 mm** | 1.731° | 9.71 N·m |
| 2 | `1.286, 0.106, 1.080` | 🔴 NO-PLAN | — | — | — | 1.83 |
| 3 | `1.214, 0.529, 1.240` | ✅ SUCCESS | success | **4.015 mm** | 0.299° | 10.86 |
| **4** | `1.000, −0.106, 1.400` | ⚠️ **TORQUE-ABORT** | **success** | **2.822 mm** | 0.433° | 🔴 **17.61** |
| 5–10 | — | **TIDAK DIJALANKAN** | | | | |

(Percobaan 1 pra-perbaikan, B3.3, juga sukses: 4.342 mm / 2.257°.)

🔴 **Percobaan 4 memenuhi A1 SEPENUHNYA** — 2.822 mm, 0.433°, dwell 2.0 s, 42
sampel @ 20.03 Hz — **sambil menarik 17.61 N·m**. Pola yang sama seperti B3.3:
probe berhenti memantau, lengan tidak diperintah berhenti, dwell tetap tercapai,
penilai mencatatnya. Jadi kegagalan di sini **sama sekali bukan kegagalan
metode**; metodenya bekerja di keempat pose yang punya rencana.

#### B4.2 🔴 BATAS TORSI: angka 14 N·m BENAR — lengan dijalankan 126 % DARI RATING

> **Koreksi terhadap pembacaan pertama saya.** Sesaat setelah abort saya menulis
> bahwa angka 14 N·m "terbantah" karena 17.61 N·m terukur tanpa fault. **Itu
> salah arah.** Batasnya diverifikasi dari sumber vendor dan ternyata **tepat
> 14**; yang terjadi adalah lengan melampauinya, bukan angkanya keliru.

Sumbernya bukan ingatan dan bukan dokumen kami — melainkan makro URDF Kinova
sendiri, `kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro`,
yang memang di-`include` oleh `workcell_description/urdf/gen3_lite_full.xacro`:

| sendi | `effort` (N·m) | puncak terukur percobaan 4 | |
|---|---|---|---|
| `joint_1` | 10 | 1.251 | ✅ |
| **`joint_2`** | **14** | **17.61** | 🔴 **126 % dari rating** |
| `joint_3` | 10 | 3.986 | ✅ |
| `joint_4` | 7 | 0.499 | ✅ |
| `joint_5` | 7 | 0.539 | ✅ |
| `joint_6` | 7 | 0.592 | ✅ |

🔴 **Cabang berbahaya yang terkonfirmasi.** Dari dua kemungkinan yang ditulis
saat abort, yang berlaku adalah yang buruk: batasnya memang 14, lengan ditarik
**26 % di atasnya**, dan **tidak ada apa pun yang bereaksi**. `ros2_control` di
sini hanya memakai antarmuka perintah **posisi** — ia tidak menegakkan batas
`effort` sama sekali — dan `fault_controller` tidak di-spawn. Jadi satu-satunya
hal yang menghentikan gerak itu adalah penjaga `--tau-max` di skrip kami,
yaitu penjaga di ruang pengguna, bukan proteksi.

**Spesifikasi vendor (dicari online 2026-08-17, atas permintaan operator).**
Gen3 Lite memakai aktuator **KA-75+** (sendi besar) dan **KA-58** (pergelangan):

| Aktuator | torsi **nominal** | torsi **puncak** (batas perangkat lunak) |
|---|---|---|
| **KA-75+** | **12.0 N·m** | **30.5 N·m** |
| **KA-58** | 3.6 N·m | 6.8 N·m |

Korroborasi silang yang meyakinkan: **KA-58 puncak 6.8 N·m** lawan `effort="7"`
di URDF untuk `joint_4/5/6` — cocok. Jadi angka URDF memang berasal dari
keluarga aktuator ini.

🔴 **Ini akhirnya MENJELASKAN kenapa 17.61 N·m tidak menimbulkan fault**, dan
jawabannya bukan "batasnya lebih longgar dari dugaan":

| Ambang | Nilai | 17.61 N·m terhadapnya |
|---|---|---|
| KA-75+ **nominal** (kontinu) | 12.0 N·m | 🔴 **147 %** |
| URDF `joint_2` `effort` | 14 N·m | 🔴 **126 %** |
| KA-75+ **puncak** (batas perangkat lunak) | 30.5 N·m | ✅ 58 % |

Jadi lengan berjalan **jauh di atas torsi kontinu** tetapi **di bawah puncak**,
dan firmware hanya menjatuhkan fault pada puncak. Melampaui **nominal** bukan
kejadian instan — ia persoalan **termal / siklus kerja**, yang justru sebabnya
tidak ada yang berbunyi. Itu **bukan** izin untuk mengabaikannya.

⚠️ PDF lembar spesifikasi resmi Kinova **tidak berhasil diambil langsung**
(fetch gagal / domain diblokir); angka di atas berasal dari hasil pencarian yang
mengutip lembar spesifikasi KA-75+/KA-58 Kinova, dan dikuatkan oleh kecocokan
KA-58 ↔ URDF. **Perlu dikonfirmasi dari PDF asli** sebelum dijadikan dasar
keputusan keselamatan permanen.

🔒 **Konsekuensi, dan ini membalik rencana:**

1. **`--tau-max` TIDAK dinaikkan — dan 12.0 ternyata terpilih tepat**, karena
   ia berimpit dengan torsi **nominal** KA-75+ (12.0 N·m). Itu ambang yang
   benar secara fisik untuk gerak berulang, bukan sekadar angka aman.
   **13.5 yang dipakai untuk pemulihan sudah di atas nominal** dan tidak boleh
   dipakai sebagai nilai percobaan.
2. **Sebagian target B1.6 TIDAK aman diperintahkan, berapa pun palangnya.**
   Percobaan 4 butuh > 14 N·m hanya untuk sampai. Itu sifat **pose**, bukan
   sifat penjaga.
3. **Peta kapabilitas butuh penyaringan torsi.** Ia menjawab "ada solusi IK di
   dekat sini" dan diam total soal apakah menahannya melewati rating sendi.
   Inilah celah desain yang sebenarnya dibuka sesi ini.

#### B4.3 Torsi istirahat naik 25× — dan uji 10 menit MEMBANTAH dugaan pertama

Pose **identik** (menggantung, B1.5), sendi sama:

| Kapan | \|tau\| maks istirahat |
|---|---|
| setelah bring-up, sebelum gerak apa pun | **0.069 N·m** |
| setelah regresi tahap 3 | 0.013 |
| setelah pemulihan percobaan 1 | 0.337 |
| setelah pemulihan percobaan 4 | **1.764** |

**Uji 0b dijalankan: 10 menit diam, NOL gerak** (`/tmp/g16_idle_torque.log`):

```
menit   0.07  1.07  2.07  3.07  4.07  5.07  6.07  7.07  8.07  9.07 10.07
|tau|  1.626 1.672 1.626 1.672 1.695 1.580 1.695 1.603 1.603 1.695 1.603
```

Simpangan posisi sepanjang 10 menit: **0.0013°**.

🔴 **Dugaan pertama saya SALAH dan dicoret.** Melihat 1.764 → 1.557 di dua
bacaan berjarak beberapa menit, saya menulis bahwa torsinya "meluruh, jadi
termal/windup lebih mungkin". Uji ini menunjukkan ia **tidak meluruh sama
sekali** — ia berosilasi stabil di **1.58–1.70 N·m** selama 10 menit. Dua
bacaan itu **derau, bukan tren**, dan saya menafsirkannya sebagai tren.

**Hipotesis terdepan sekarang, dan cara membedakannya:** pada bacaan 0.069 N·m,
lengan baru saja bring-up dan **belum pernah menerima perintah** — JTC belum
menahan apa pun. Setelah lintasan dieksekusi, controller **aktif menyervo** ke
titik perintah terakhir, dan itu menghasilkan torsi menahan sungguhan yang
memang **tidak akan pernah meluruh** selama controller aktif. Konsisten dengan
osilasi datar dan simpangan 0.0013°.

➜ **Uji yang membedakan** (belum dijalankan): bring-up baru, baca torsi
istirahat (harus ~0.069), lalu perintahkan lintasan yang berakhir **di pose
istirahat yang sama**, baca lagi. Kalau naik ke ~1.6 tanpa lengan berpindah,
sebabnya adalah menyervo-vs-pasif dan **bukan** termal maupun mekanis.

#### B4.4 Pemulihan `return_rest` sendiri GAGAL sekali

Sebelum percobaan 5, pengembalian ke rest **abort di 13.08 N·m** dan
**meninggalkan lengan 99.4° dari rest**, menahan **9.597 N·m statis** dengan
bahu terentang (`joint_2 = 1.84 rad`). Dipulihkan dengan gerak **30 detik**
(torsi dinamis minimal) dan palang 13.5: puncak 9.872 N·m, galat akhir 0.028°.

🔴 **Pelajaran yang harus masuk protokol:** penjaga torsi yang **membatalkan**
gerak pemulihan lebih berbahaya daripada tidak ada penjaga — ia meninggalkan
lengan **terdampar dalam keadaan terbebani**. Gerak pemulihan butuh aturan
berbeda dari gerak percobaan: lambat, dan diselesaikan.

#### B4.5 Yang sebenarnya sudah dipelajari tentang L2

Empat pengukuran A1 valid, semuanya LULUS palang, tapi marginnya tipis:

| | pos maks | pos rata-rata |
|---|---|---|
| percobaan 1 (pra-perbaikan) | 4.342 mm | 1.615 |
| percobaan 1 | 4.281 | 1.916 |
| percobaan 3 | 4.015 | 2.454 |
| percobaan 4 | 2.822 | 2.409 |

**Galat maksimum menempel di 80–87 % dari palang 5 mm** (kecuali percobaan 4).
Laju sampel 20.0–20.1 Hz, `n_tf_fail = 0`, `instrument_ok = True` di keempatnya.
Jadi L2 **terpenuhi, tapi nyaris** — dan itu temuan yang berguna untuk naskah,
karena 5 mm dipilih di atas seluruh sumber galat mekanis terukur (`p1_g4 §A1`).

#### B4.6 Skor: D48/D49 tetap BELUM dinilai

Empat percobaan bukan sepuluh, dan A2 menuntut penyebut 10. **D48 dan D49
tetap TIDAK DINILAI.** Papan skor tetap **31 meleset, 17 tepat**.

Sejauh ini 2 SUCCESS + 1 NO-PLAN + 1 (A1-sukses tetapi torsi-abort) dari 4;
enam sisanya belum dijalankan. Angka **tidak** diekstrapolasi.

### B5. 🔒 LANGKAH 2 SELESAI — kesepuluh percobaan dijalankan

Dijalankan setelah penyaring torsi disambung. Daftar B1.6 dipakai **apa adanya**,
urutan asli, tanpa penggantian.

| # | target | vonis probe | penilai A1 | pos maks | ori maks | torsi puncak |
|---|---|---|---|---|---|---|
| 1 | `0.929,0.247,1.240` | ✅ SUCCESS | success | 4.281 mm | 1.731° | 9.71 |
| 2 | `1.286,0.106,1.080` | 🔴 NO-PLAN | — | — | — | 1.83 |
| 3 | `1.214,0.529,1.240` | ✅ SUCCESS | success | 4.015 | 0.299° | 10.86 |
| 4 | `1.000,−0.106,1.400` | ⚠️ TORQUE-ABORT | **success** | 2.822 | 0.433° | 🔴 **17.61** |
| 5 | `0.714,−0.035,1.080` | 🔴 NO-PLAN | — | — | — | 1.95 |
| 6 | `0.857,0.529,1.000` | ✅ SUCCESS | success | 2.912 | 0.688° | 9.05 |
| 7 | `0.571,0.318,1.160` | ⚠️ TORQUE-ABORT | **success** | 4.396 | 0.782° | 🔴 **15.05** |
| 8 | `0.786,0.529,1.400` | ✅ SUCCESS | success | 4.667 | 0.995° | 10.72 |
| 9 | `1.214,0.176,1.400` | ✅ SUCCESS | success | 3.446 | 0.824° | 11.68 |
| 10 | `0.643,0.106,1.400` | ⚠️ TORQUE-ABORT | **success** | 2.910 | 2.175° | 🔴 **16.44** |

Kesembilan baris `success` penilai: `n_tf_fail = 0`, `instrument_ok = True`,
laju 20.0–20.1 Hz, 41–42 sampel per jendela. Instrumen sehat di semuanya.

#### B5.1 🔒 A2 — DUA pembacaan, dan keduanya harus disebut

**Pembacaan 1 — kriteria A1 sebagaimana DIKUNCI.** A1 adalah posisi, orientasi,
dan dwell. **Torsi bukan bagian dari A1.** Penilai independen mencatat
**8 dari 10** memenuhi A1 penuh; dua sisanya NO-PLAN.

> **8 / 10 ≥ 8 → LANGKAH 2 LULUS**, tepat di palang, nol margin.

**Pembacaan 2 — kelayakan berulang.** Tiga dari delapan sukses itu
(percobaan 4, 7, 10) dicapai pada torsi **di atas rating 14 N·m `joint_2`**, dan
kedelapan-delapannya di atas nominal KA-75+ 12.0 N·m. Pose-pose itu **tidak
dapat diulang dengan aman**. Kalau ketiganya dikeluarkan: **5 / 10 → TIDAK
LULUS**.

🔒 **Yang berlaku secara formal adalah pembacaan 1**, karena A1/A2 dikunci
sebelum data ada dan **dilarang digeser sesudahnya** — menambahkan syarat torsi
sekarang persis pelanggaran yang aturan itu cegah. **Tetapi pembacaan 2 wajib
ikut dilaporkan di naskah**, karena "berhasil sekali di 17.61 N·m pada sendi
berating 14" bukan kemampuan yang boleh diklaim. Keduanya ditulis; tidak ada
yang dipilih diam-diam.

#### B5.2 Galat L2 — sepuluh percobaan

| | nilai |
|---|---|
| pos maks, rentang | **2.822 – 4.667 mm** (semua < 5) |
| pos maks, p50 / p95 | **3.73 / 4.63 mm** |
| pos rata-rata, rentang | 1.042 – 2.454 mm |
| ori maks, rentang | 0.299 – 2.257° (semua < 5) |
| laju sampel | 20.0 – 20.1 Hz |

🔴 **Galat maksimum menempel di 56–93 % palang 5 mm, p95 = 4.63 mm.** L2
terpenuhi, tetapi **nyaris**, dan tanpa satu pun percobaan punya margin nyaman.
Itu temuan naskah tersendiri: 5 mm dipilih di atas seluruh sumber galat mekanis
terukur (`p1_g4 §A1`), jadi yang tersisa adalah galat pelacakan servo — dan ia
mengisi hampir seluruh anggaran.

#### B5.3 🔒 D48 dan D49 DINILAI

| # | Dugaan | Vonis |
|---|---|---|
| **D48** | "Langkah 2 **TIDAK lulus** ≥8/10 pada sesi pertama, sebab dominan **integrasi**" | ❌ **MELESET** |
| **D49** | "Dari percobaan yang gagal, **< 20 %** berupa `REACHED-NOT-HELD`" | ✅ **TEPAT** |
| D50 | "`remount_check` menemukan ≥1 mode A5 aktif" | ❌ MELESET (B1.2) |

**D48 MELESET** dan ini konjungsi: ia LULUS 8/10, jadi konjungsi pertamanya
salah, dan A9 menghitung konjungsi gagal sebagai meleset **utuh** — walaupun
bagian "integrasi" terbukti benar (dua pemblokir sesi ini, `enable_gantry_bridge`
dan balapan DDS, dua-duanya integrasi). Prior "kode sendiri: lebih lambat,
lebih rumit, lebih salah" **terlalu pesimistis** di sini: metodenya bekerja pada
percobaan pertama yang punya rencana, dan tetap bekerja di kedelapannya.

**D49 TEPAT, dan instrumennya benar-benar mampu mengukurnya sekarang** —
`REACHED-NOT-HELD` baru bisa dideteksi setelah perbaikan B3.4(4). Nol dari dua
kegagalan berupa REACHED-NOT-HELD (0 % < 20 %). Alasan yang didalilkan juga
terkonfirmasi: **yang langka adalah SAMPAI, bukan MENAHAN** — kesembilan
kedatangan bertahan 2.0 s penuh tanpa satu pun reset jendela.

➜ Papan skor §7.2: **32 meleset, 18 tepat**.

#### B5.4 Kalibrasi penyaring torsi — selisihnya ADITIF

Prediksi RNEA lawan terukur, enam pasang:

| prediksi | terukur | selisih |
|---|---|---|
| 2.55 | 9.71 | +7.2 |
| 7.60 | 10.86 | +3.3 |
| 8.76 | 17.61 | +8.9 |
| 3.92 | 9.05 | +5.1 |
| 7.77 | 15.05 | +7.3 |
| 4.14 | 10.72 | +6.6 |
| 8.60 | 16.44 | +7.8 |

**Selisih ≈ +6.6 ± 1.8 N·m, hampir bebas dari nilai prediksi.** Itu tanda
**gesekan sendi + rugi strain-wave gear** yang tidak dimodelkan — suku nyaris
konstan — bukan kesalahan dinamika; konsisten dengan cek statis gravitasi yang
cocok dalam 3.6 %. `effort` Kortex yang diturunkan dari arus motor akan
menghasilkan tanda tangan persis seperti ini.

Selisih rata-rata **+6.59 N·m, sd 1.86**.

➜ Penyaring **bisa diselamatkan**: saring pada `prediksi + 6.6 N·m` terhadap
**rating tiap sendi** (bukan satu angka global — `joint_2` berating 14, sedangkan
pergelangan hanya 7).

🔴 **Koreksi: bukan 7/7.** Saat pertama menulis ini saya mengklaim pemisahannya
sempurna 7 dari 7. Diperiksa ulang dengan menjalankan kriterianya, hasilnya
**6 dari 7**:

| # | prediksi + 6.6 | keputusan | terukur | benar? |
|---|---|---|---|---|
| 1 | 9.15 | lolos | 9.71 aman | ✅ |
| **3** | **14.20** | **TOLAK** | **10.86 aman** | ❌ **tolak-palsu** |
| 4 | 15.36 | TOLAK | 17.61 tidak aman | ✅ |
| 6 | 10.52 | lolos | 9.05 aman | ✅ |
| 7 | 14.37 | TOLAK | 15.05 tidak aman | ✅ |
| 8 | 10.74 | lolos | 10.72 aman | ✅ |
| 10 | 15.20 | TOLAK | 16.44 tidak aman | ✅ |

**Ketiga pose tidak aman tertangkap semua**; satu-satunya kesalahan adalah
**penolakan terhadap pose yang ternyata aman** — yaitu galat ke arah **tidak
bergerak**, arah yang benar untuk penyaring keselamatan.

⚠️ Ambang pada **prediksi mentah** memang memisahkan ketujuhnya sempurna, tetapi
hanya di dalam celah **7.60 … 7.77 N·m (lebar 0.17)**. Itu jauh terlalu tipis
untuk dipercaya dari 7 sampel, dan memilihnya berarti mencocokkan ambang ke
data yang sudah dilihat. Offset + rating dipakai justru karena lebih tumpul.

#### B5.5 Keadaan akhir

Lengan **MENGGANTUNG**, galat 0.026° dari rest, `|tau|` istirahat 0.763 N·m,
gantry tetap **0.550009 m**, ketujuh controller `active`, **nol fault** sepanjang
sesi (nol `Kortex exception`, nol red LED, nol SIGPIPE).

---

### B6. Ruang kerja AMAN-TORSI — 30 % dari yang terjangkau ternyata tidak aman

Tugas lanjutan setelah B5, dikerjakan **offline, tanpa perangkat keras**
([scripts/torque_safe_workspace.py](../scripts/torque_safe_workspace.py)).
Pertanyaannya lahir langsung dari B5.1: tiga dari delapan sukses menuntut torsi
di atas rating `joint_2`, jadi **terjangkau** dan **aman** bukan himpunan yang
sama — dan selama ini hanya yang pertama pernah dipetakan.

Metode: himpunan terjangkau dari peta kapabilitas pada rel **terukur**
(sumber yang sama dengan `reachable_targets.py`, jadi keduanya sepakat by
construction); IK per node; torsi **gravitasi** di konfigurasi itu lewat
pinocchio; dikoreksi offset **+6.6 N·m** dari B5.4; dibandingkan ke rating tiap
sendi.

| | |
|---|---|
| node terjangkau (L1, 5 cm) | **1014** dari 3132 |
| IK tidak konvergen | **113** (dilaporkan, tidak dibuang) |
| **AMAN-TORSI** | **628 / 901 = 69.7 %** |
| **TIDAK AMAN** | **273 / 901 = 30.3 %** |
| sendi pengikat | **`joint_2` 263×**, `joint_4` 10× |
| kotak aman | x 0.36–1.57, y −0.25–0.60, z 1.00–1.40 |

🔴 **Hampir sepertiga ruang kerja yang "terjangkau" tidak dapat dijangkau dengan
aman**, dan yang mengikat hampir selalu **bahu** (`joint_2`, 96 % dari kasus).
Ini menjelaskan B5.1 secara struktural, bukan kebetulan: tiga percobaan yang
melewati rating bukan sial, melainkan sampel dari 30 % yang memang tidak aman —
dan peta kapabilitas tidak punya cara memberi tahu, karena ia tidak pernah
menanyakan torsi.

⚠️ **Batas klaim, dan ini wajib ikut:** perhitungan ini **statis saja**. RNEA
penuh butuh lintasan, dan lintasan baru ada setelah MoveIt merencanakan. G16
menunjukkan statis akurat (9.253 prediksi lawan 9.597 terukur, 3.6 %), tetapi
puncak yang benar-benar memicu penjaga adalah **transien saat bergerak**. Jadi
peta ini adalah syarat **PERLU, bukan CUKUP**: pose yang gagal di sini tidak
mungkin aman, sedangkan pose yang lolos masih bisa melonjak saat transit.
Dilaporkan begitu, bukan sebagai sertifikat keselamatan.

➜ **Konsekuensi untuk langkah 3:** target dua-lengan harus diambil dari 628
node aman ini, bukan dari 1014 node terjangkau. `/tmp/torque_safe_nodes.npy`.

---

---

## D. Prompt sesi berikutnya — G17-HW (salin ke chat BARU)

> **Rekomendasi: Opus 5, effort TINGGI.** Tetap TINGGI. Langkah 2 sudah selesai,
> jadi alasannya bukan lagi "kode penggerak belum ada". Sekarang: §8c langkah 3
> menambah lengan KEDUA pada gantry yang SAMA, jadi tabrakan antar-lengan
> menjadi nyata untuk pertama kalinya; dan tiga dari delapan sukses langkah 2
> menuntut torsi **di atas rating `joint_2`**, jadi ruang kerja yang aman
> ternyata lebih kecil dari ruang kerja yang terjangkau — dan itu belum dipetakan.

```
Sesi G17-HW -- p1_state.md 8c LANGKAH 3: DUA lengan, jendela dwell BERSAMA.
SESI PERANGKAT KERAS. Langkah 2 SELESAI. Risiko fisik NYATA dan BARU:
dua lengan pada gantry yang sama bisa BERTABRAKAN.

BACA DULU:
1. docs/p1_g16_hw.md -- §A DIKUNCI, JANGAN DITULIS ULANG. §B DIUKUR.
   B5.1  (8/10 LULUS, TAPI 3 dari 8 di atas rating torsi -- DUA pembacaan)
   B5.2  (galat L2 mengisi hampir seluruh anggaran: p95 = 4.63 dari 5 mm)
   B5.3  (D48 MELESET, D49 TEPAT, papan skor 32/18)
   B5.4  (penyaring torsi: selisih ADITIF +6.6 N.m -- pakai offset ini)
   B4.2  (KA-75+ nominal 12.0 / puncak 30.5; joint_2 URDF 14)
   B4.4  (gerak PEMULIHAN tidak boleh dibatalkan penjaga)
   B3.4  (ENAM pertentangan B-vs-A, semuanya instrumen)
2. docs/p1_g4_reach_dwell.md A1 -- definisi sukses N-lengan SERENTAK:
   ada SATU jendela 2.0 s di mana SEMUA lengan memenuhi toleransinya BERSAMAAN.
   Bukan "masing-masing sukses di episode ini" -- itu bisa dipenuhi bergantian,
   dan bergantian persis yang dilakukan prior work.

=== SUDAH SELESAI 2026-08-17, JANGAN ULANGI ===
- Tahap 0-3 LULUS. Langkah 2 SELESAI: 8/10 pada kriteria A1 terkunci.
- move_to() DITULIS: plan_only -> cek FORBIDDEN_TUCK tiap titik -> cek torsi
  RNEA -> ExecuteTrajectory. Bekerja di 8 pose nyata.
- publish_target() balapan DDS DIPERBAIKI (tunggu get_subscription_count>0).
- EXEC-MISS ditambahkan; REACHED-NOT-HELD dibaca dari CSV penilai.
- Penyaring torsi pinocchio/RNEA disambung DAN dikalibrasi (B5.4).
- Batas aktuator dicari dari vendor: KA-75+ 12.0 nominal / 30.5 puncak.

=== SUDAH SELESAI 2026-08-17 SESUDAH LANGKAH 2, JANGAN ULANGI ===
- Offset kalibrasi +6.6 N.m SUDAH diterapkan di reach_dwell_probe.py
  (TORQUE_MODEL_OFFSET_NM + JOINT_EFFORT_LIMIT per sendi). Penyaring kini
  PENCEGAH, bukan detektor. Validasi ulang: 6/7 (bukan 7/7 -- klaim itu
  dikoreksi di B5.4); ketiga pose tidak aman tertangkap, satu tolak-palsu
  yang konservatif.
- RUANG KERJA AMAN-TORSI arm_1 @ lin=0.550 SUDAH dipetakan (B6):
  628 dari 901 node = 69.7 % aman; 30.3 % TIDAK aman; joint_2 mengikat 96 %.
  Simpanan: /tmp/torque_safe_nodes.npy
  scripts/torque_safe_workspace.py --arm arm_1 --lin 0.550

=== TUGAS ===
1. AMBIL TARGET LANGKAH 3 DARI /tmp/torque_safe_nodes.npy (628 node aman),
   BUKAN dari 1014 node terjangkau. Pakai --spread supaya pose BERBEDA
   (A2), dan KUNCI daftarnya di dokumen SEBELUM percobaan pertama.
   Untuk arm_2 petakan ulang: python3 scripts/torque_safe_workspace.py
   --arm arm_2 --lin 0.550  (arm_2 = PARTNER di gantry yang sama).
2. LANGKAH 3: arm_1 + arm_2 (dua-duanya di gantry_1), jendela dwell BERSAMA.
   - bring-up: arm3_fake:=true arm4_fake:=true, DUA lengan nyata.
     PERINGATAN: bug mid-boot kortex_driver membunuh KEEMPAT controller; beri
     jeda setelah power-on dan periksa LED tiap lengan.
   - monitor: -p arms:="['arm_1','arm_2']"
       -p tool_frames:="['t1_a1_tool_frame','t1_a2_tool_frame']"
     Ia SUDAH mendukung jendela bersama (cari 'concurrent' di kodenya).
   - TABRAKAN: kedua lengan berbagi gantry_1. Periksa rencana terhadap
     tabrakan antar-lengan SEBELUM eksekusi. Octomap/self-filter TIDAK cukup
     -- lihat [[gng-collision-static-self-collision-fix]].
3. NILAI dugaan baru yang DITULIS DI MUKA sebelum data diambil. Minimal satu
   harus tentang jendela BERSAMA, karena itu besaran yang belum pernah diukur.
4. Tulis hasil di docs/p1_g17_hw.md (§B p1_g16_hw.md sudah ~700 baris; mulai
   dokumen baru dan tautkan balik, jangan sambung terus).

=== PERINTAH ===
ros2 launch workcell_moveit_config my_workcell.launch.py \
    use_fake_hardware:=false arm3_fake:=true arm4_fake:=true \
    enable_gantry_bridge:=true 2>&1 | tee /tmp/g17_t1.log
ros2 control list_controllers        # KETUJUHNYA active
# enable_gantry_bridge WAJIB: tanpa itu gantry_N_with_arm_controller INACTIVE
# dan TIDAK ADA gerak lengan sama sekali. Ini memakan sebagian besar G16.

=== KESELAMATAN ===
- Kembalikan lengan ke MENGGANTUNG sebelum tiap percobaan (dipakai di G16).
- Gerak PEMULIHAN: LAMBAT (30 s) dan HARUS DISELESAIKAN, jangan dibatalkan.
- --tau-max 12.0 (= nominal KA-75+). JANGAN dinaikkan untuk lolos.
- fault_controller TIDAK di-spawn: red LED = reset FISIK, tanpa undo.
- ros2_control TIDAK menegakkan batas effort sama sekali. Satu-satunya
  perlindungan adalah penyaring pra-eksekusi di move_to().

=== JEBAKAN YANG SUDAH DIUKUR ===
- /joint_states DUA penerbit, pesan TERPISAH. `--once` gagal diam-diam ~90 %.
  GABUNGKAN per-nama >= 0.5 s.
- Lebar /joint_states >= 28 lolos karena alasan salah (gripper palsu).
- remount_check.py --ros menandai launch-nya SENDIRI sebagai basi.
- `set -u` + `source install/setup.bash` = COLCON_TRACE unbound variable.
- kill -INT ke PID INDUK ros2 launch; butuh ~20 s; jangan -9.
- Loop /proc yang mencocokkan nama proses mencocokkan SHELL-nya sendiri.
- use_fake_hardware DEFAULT true. 2>&1 | tee WAJIB.
- Tab browser ke 192.168.2.1x = SIGPIPE(-13). TUTUP.
- --approach 0 WAJIB.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 32 meleset, 18 tepat.
  Tulis dugaan BARU di muka SEBELUM mengambil data, dan nilai apa adanya.
  Pelajaran G16: prior "kode sendiri lebih salah" TERLALU PESIMISTIS untuk
  metode (D48 meleset); ia tetap tepat untuk INTEGRASI (dua pemblokir).
- DILARANG menggeser ambang A1 (5 mm / 5 deg / 2.0 s).
- DILARANG mengganti target yang NO-PLAN dengan yang lebih mudah.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 lima, G11 empat, G12 dua, G13 dua, G14 empat, G15 dua, G16 ENAM.
- Rule 6 PENGECUALIAN EKSPLISIT untuk sesi protokol-panjang P1 (A10).
- A6/S7: tidak ada gerak tanpa persetujuan manusia eksplisit.
```
