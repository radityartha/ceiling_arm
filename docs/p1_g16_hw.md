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

### A8. 🔒 INSTRUMEN — dan kenapa yang MENILAI bukan yang MEMERINTAH

| Berkas | Peran | Status |
|---|---|---|
| `reachability_gng/reach_dwell_monitor.py` | **penilai murni** — baca TF, nilai, catat. Memerintah **nol** | ✅ ada, tervalidasi 5/5 |
| `scripts/reach_dwell_probe.py` | **pemerintah** — persepsi → pose perintah → publikasi ke monitor → gerakkan lengan | 🆕 **BARU sesi ini** |
| `scripts/remount_check.py` | gerbang tahap 0 dan 1, read-only | 🆕 **BARU sesi ini** |

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
