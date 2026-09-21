# P1 / G18-HW — §8c langkah 3 di perangkat keras NYATA: DUA lengan, jendela dwell BERSAMA

Lanjutan [p1_g17_hw.md](p1_g17_hw.md), yang menyiapkan langkah 3 **penuh** dengan
nol gerak perangkat keras. G17 **tidak** disambung; dokumen ini dimulai baru.

Yang **diwarisi apa adanya dan tidak ditulis ulang**: g17 §A1 (sukses N-lengan =
SATU jendela 2.0 s BERSAMA), §A2 (10 pasangan, LULUS ≥ 8/10 CONCURRENT), §A3
(S8–S11), §A9 (dugaan D51–D56, dikunci sebelum data), daftar terkunci
[g17 §B2](p1_g17_hw.md); dan dari [p1_g16_hw.md](p1_g16_hw.md) §A1, §A3, §A5,
§A6 (S1–S7), §A10.

🔒 **Tidak ada satu pun kriteria yang digeser sesi ini.** A1 tetap 5 mm / 5° /
2.0 s; `--tau-max 12.0`; daftar B2 dipakai apa adanya, urutan asli.

---

## B. Hasil terukur

### B0. SEBELUM percobaan pertama — 2026-09-21

Semua temuan di bawah ini terjadi **sebelum** satu pun percobaan B2 dijalankan.
Tiga di antaranya akan menghasilkan data yang tampak sah padahal tidak.

#### B0.1 Gerbang tahap 0 GAGAL: `arm_2` tidak ada di jaringan

`remount_check.py`: ICMP `arm_2` (192.168.2.12) 5/5 gagal, ARP **`INCOMPLETE`**
(tidak ada MAC sama sekali → lapisan fisik, bukan routing). Operator memulihkan
secara fisik; sesudahnya `REACHABLE` (MAC `9c:eb:e8:13:2f:f7`), gerbang lulus.

#### B0.2 🔴 Penilai TIDAK BISA LAHIR, dan validatornya menyembunyikannya

`ros2 run reachability_gng reach_dwell_monitor` → `StopIteration`.
`build/…/entry_points.txt` basi: tidak memuat `reach_dwell_monitor`,
`capability_pub`, `irm_sweep` yang ada di `setup.py` (commit `0a4fc86` menambah
`capability_pub` sesudah build terakhir). Jebakan g16 A7b, terulang.

Yang lebih penting: `validate_reach_dwell_monitor.py` melempar stderr penilai ke
`DEVNULL`, jadi penilai yang tidak lahir tampil sebagai **A/C/E FAIL, B/D PASS**
— dua kasus negatif **lulus karena tidak ada yang menilai**. E adalah kasus
jendela bersama, yaitu instrumen D51.

**Perbaikan** (`608bd02`): rebuild `--packages-select reachability_gng`; validator
memeriksa proses **lebih dulu** dan keluar `rc=2 MONITOR FAILED TO START`.
Kontrol negatif (entry point sengaja disalahkan) → `rc=2` ✅.

#### B0.3 🔴 Penilai BOCOR satu proses per run — dan yang bocor MENILAI run berikutnya

`ros2 run` melahirkan penilai sebagai **anak**; `mon.terminate()` membunuh
pembungkusnya saja. Terukur: **empat** penilai basi menumpuk dari empat run
validator. Dibuktikan berbahaya, bukan sekadar kotor: dengan entry point
**sengaja dirusak**, kelima kasus tetap **PASS** — penilai lama yang menjawab.

Konsekuensi: validasi 5/5 pertama sesi ini **tidak sah** dan dibuang. Yang
berlaku adalah run bersih: **0 penilai sebelum, 5/5 PASS, 0 sesudah**.

**Perbaikan** (`608bd02`): `start_new_session=True` + `killpg`;
`remount_check.py` kini memindai `reach_dwell_monitor` sebagai proses basi
(mode A5 yang belum terdaftar).

#### B0.4 🔴 Keadaan repo yang ter-commit TIDAK BISA bring-up, di mode MANA PUN

```
what(): Hardware name GenericSystem is duplicated. Please provide a unique 'name' in the URDF.
```

`ros2_control_node` abort (−6) **setelah** kedua lengan nyata terhubung
(`Actuator count '6'` di `.13` **dan** `.12`) — jadi **bukan** mode mid-boot A5.
`gen3_lite_macro.xacro:45` menamai tiap lengan **palsu** `GenericSystem` tanpa
prefiks (dan gripper `GripperHardwareInterface`); jalur nyata sudah berprefiks.

Uji menentukan, nol kontak lengan: `use_fake_hardware:=true` penuh — mode yang
g17 pakai untuk 10 pasangan — **abort identik**. Pustaka `ros2_control` tidak
berubah sejak 2026-07-24 (sebelum g16). Jadi g16/g17 berjalan pada working tree
yang **tidak pernah di-commit**.

Perbaikannya ditemukan di `ros2_ws/src/ros2_kortex` **stash@{0}** *"uncommitted
kortex driver changes before branch switch"*: prefiks mock + gripper, **dan**
patch `kortex_driver` (sesi diulang 5× @1 s; `refreshFeedbackSafe()`
menggantikan 8 `RefreshFeedback()` mentah, sehingga satu paket jatuh tidak lagi
`std::terminate()` seluruh node — keempat lengan). Diterapkan dengan persetujuan
operator, di-commit `e712295` (branch lokal `ceiling-arm-fixes`; remote-nya hulu
Kinova, jadi commit ini **hanya ada di mesin ini**, dan repo induk tidak punya
`.gitmodules`).

Sekelas dengan g17 B0.6 (`return_rest` dipakai g16 tapi tidak di-commit).

#### B0.5 Kebocoran proses peluncuran — `move_group` palsu yang mengiklankan action server yang sama

`$!` setelah `nohup ros2 launch … &` menangkap **pembungkus**, bukan proses
`ros2 launch`; `kill -INT` ke sana meninggalkan anak-anaknya. Terukur: satu
`move_group` **mode palsu** dan satu `robot_state_publisher` kedua (nama node
sama) hidup berdampingan dengan stack nyata — persis A5 "move_group ganda →
CONTROL_FAILED". Dibersihkan per-PID, diverifikasi `ros2 node list --no-daemon`
(cache daemon menampilkan hantu). Sekali, saya juga meluncurkan stack kedua
sebelum yang pertama mati — kesalahan saya; dihentikan sebelum ada perintah.

#### B0.6 `robot_state_publisher` dan `move_group` memakai URDF **mode palsu**

URDF yang disajikan `/robot_state_publisher` memuat `TableFakeHardware` dan
`t1_a{1,2}_GenericSystem` walau `use_fake_hardware:=false`; `ros2_control_node`
mendapat yang benar. Dibandingkan numerik dengan xacro hari-H: **geometri lengan
dan gantry identik, massa/COM 0 selisih** → penyaring torsi RNEA tidak
terpengaruh. Satu-satunya selisih: **4 sendi jari gripper** `arm_1`/`arm_2`
(rpy, batas). Penyaring antar-lengan melihat pose jari yang sedikit salah;
margin terketat g17 224 mm lawan palang 50 mm. Mekanisme sama berlaku di g16/g17.

#### B0.7 🔴 Origin enkoder gantry_1 ≠ home fisik

Rel terbaca **0.000 mm**; operator melihat carriage **~55 cm dari home**. Log ROS:
preset terakhir 2026-08-11; 2026-08-26 dikendarai 121085 pulsa (1268 mm) → 0.
Sebabnya (preset di luar ROS, atau selip) tidak dapat ditentukan dari software.

Bahaya yang sempat terbuka: end stop jauh (~1656 mm dari home, [[gantry-linear-travel-limit]])
berada di **enkoder ≈ 1106 mm**, sedangkan guard bridge mengizinkan **1600**.

⚠️ Saya sempat **salah menyimpulkan** "model software benar" dengan mengasumsikan
enkoder 0 = home; operator mengoreksinya. Tidak ada gerak gantry yang dikirim
selama asumsi itu berlaku.

**Pemulihan (operator):** carriage dibawa ke home fisik — titik tetap yang **sama**
dengan origin 2026-08-11 — lalu preset. Terbaca **0.000000 / 0.000000**.

Juga terukur: `go_to_absolute` menganggap |pos − target| ≤ **50 pulsa** "sudah
sampai" = **0.5° rotasi / 0.52 mm linear**. Rotasi −1.05° → 0° sempat berhenti di
−0.44° karena itu (sebelum preset).

#### B0.8 Dua `dual_table_controller` — kunci port yang menyelamatkan

Setelah homing, `dual_table_controller` + `table_keyboard.py` milik operator
masih memegang `/dev/ttyUSB0`/`1`. Node gantry stack saya gagal
`Could not exclusively lock port` — kunci eksklusif **mencegah** dua pengendali
memerintah motor yang sama. Init node gantry sekali jalan (A5), jadi stack
di-restart setelah operator menutup prosesnya.

#### B0.9 Gerak fisik pertama sesi ini — kedua lengan ke REST, lalu rel ke 0.550

| Gerak | Hasil |
|---|---|
| `return_rest.py --arms arm_1 arm_2 --move` (30 s) | galat akhir **0.034°**, torsi puncak **4.489 N·m** `t1_a1_joint_2`, antar-lengan CLEAR min **569.8 mm**, nol fault — **gerak fisik pertama `arm_2` di proyek ini** |
| rel 0 → 0.550 m (goal parsial rel-saja, 30 s, via bridge) | **0.549737 m** (−0.26 mm; g16 0.550009), 38 dispatch, nol penolakan; **lengan bergeser maks 0.018°** selama rel berjalan |

Torsi istirahat: **0.16 / 0.14 N·m** (pasif, belum pernah diperintah) →
**1.20 / 2.38 N·m** setelah lintasan (menyervo). Konsisten dengan hipotesis
g16 B4.3 (menyervo lawan pasif); bukan uji pembeda yang ia usulkan (pose awal
berbeda), jadi dicatat, tidak dinilai.

`topic_based_ros2_control` hanya menerbitkan perintah bila perintah ≠ state;
di 0.000000 tepat, bridge tidak pernah menerima pesan dan **tidak ARMED** sampai
gerak pertama. Bukan kerusakan — ARMED pada setpoint pertama (0.38 mm < `arm_tol` 5 mm).

#### B0.10 Pertentangan §B lawan §A — G18

G10 lima, G11 empat, G12 dua, G13 dua, G14 empat, G15 dua, G16 enam, G17 enam.

| # | Pertentangan |
|---|---|
| **(1)** | "penilai tervalidasi 5/5" (g17) tidak dapat direproduksi: build basi, penilai tidak lahir (B0.2) |
| **(2)** | validator dapat **lulus tanpa penilainya sendiri** — penilai bocor menilai run berikutnya (B0.3) |
| **(3)** | keadaan ter-commit tidak bisa bring-up; g16/g17 berjalan di working tree tak ter-commit (B0.4) |
| **(4)** | `move_group`/RSP memakai URDF mode palsu walau `use_fake_hardware:=false` (B0.6) |
| **(5)** | protokol mengandaikan rel di 0.550; origin enkoder ternyata bergeser dari home fisik (B0.7) |
| **(6)** | probe memasukkan `TORQUE-ABORT` **dan** `HALTED` ke ember "TIDAK VALID (mesin, A5, di LUAR penyebut)" — bertentangan dengan g16 B5.1 (A1 tanpa torsi) dan g17 B3 (HALTED dihitung di dalam 10). Lihat B2.2 |
| **(7)** | `p1_state.md` §8c basi (langkah 2 masih "BERIKUTNYA") dan mendefinisikan langkah 3 sebagai *"kopling geser-π terlihat"* — besaran yang **tidak** diukur protokol g17 maupun sesi ini |

**G18: TUJUH.**

---

### B1. Plan-check di rel NYATA, kedua lengan di REST — nol gerak

`--dual --plan-check`, 10 pasangan B2, rel 0.549737 m, penilai terpisah
(`/tmp/g18_plancheck`) supaya CSV hari-H tidak tercemar.

| | |
|---|---|
| rencana `PLANNED` | **20 / 20** |
| jarak antar-lengan min sepanjang rencana | **278.6 – 622.8 mm** |
| rencana ditolak torsi lalu diulang | pasangan 4 arm_1 (102 % → lolos 2nd), pasangan 6 arm_1 (101 %, 101 % → lolos 3rd) |

Mekanisme ulang g17 B1.10 **bekerja di perangkat keras nyata**. Tetapi enam
rencana yang lolos berada di **94–99.8 %** rating `joint_2` setelah +6.6.

🔴 **`--tau-max 12.0` TIDAK memperketat `joint_2`.** `move_to()`:
`lim = min(14, max(tau_max, 14))` untuk `joint_2` → selalu **rating 14**; sendi
lain `min(rating, tau_max)`. Angka 12.0 hanya dipakai **penjaga hidup** yang
**melapor** (`TORQUE-ABORT`) tanpa membatalkan atau menghentikan lengan. Jadi
pelindung sesungguhnya untuk `joint_2` adalah prediksi + 6.6 ≤ 14. Ini desain
g17 (rekomendasi g16 B5.4), **tidak diubah**; disampaikan ke operator sebelum
izin gerak.

---

### B2. 🔒 LANGKAH 3 — sepuluh pasangan di perangkat keras NYATA

Daftar B2 **apa adanya, urutan asli, tanpa penggantian**. Tiap percobaan:
`return_rest` (30 s) → `arm_1` → `arm_2` **berurutan** (S11) → penilai
independen mengamati. Izin operator: pasangan 1 sendiri, lalu 2–10 dengan
auto-stop (torsi terukur > 14, fault, node mati, `return_rest` gagal, INVALID
mesin). Auto-stop terpicu dua kali — keduanya oleh label probe (pertentangan
6), bukan bahaya; operator mengizinkan lanjut tiap kali.

**Vonis dari PENILAI INDEPENDEN** (`reach_dwell_monitor`, bukan probe):

| # | vonis A1 N-lengan | arm_1 pos maks / **settled** | arm_2 pos maks / **settled** | torsi puncak terukur | probe |
|---|---|---|---|---|---|
| 1 | ✅ CONCURRENT | 2.32 / **1.48** | 3.39 / **1.41** | 9.25 `a1_j2` | CONCURRENT |
| 2 | ✅ CONCURRENT | 2.73 / **1.08** | 3.34 / **2.02** | **12.71** `a1_j2` | TORQUE-ABORT |
| 3 | ✅ CONCURRENT | 4.75 / **1.03** | 4.49 / **1.79** | 6.46 `a2_j2` | CONCURRENT |
| 4 | ✅ CONCURRENT | 3.84 / **0.83** | 4.75 / **2.81** | 11.31 `a1_j2` | CONCURRENT |
| 5 | ✅ CONCURRENT | 4.93 / **1.66** | 4.43 / **2.20** | 5.53 `a2_j3` | CONCURRENT |
| 6 | ❌ **PARTIAL** | 2.76 / **1.26** | — (TORQUE-UNSAFE 3/3, tidak bergerak) | 10.97 `a1_j2` | HALTED |
| 7 | ✅ CONCURRENT | 3.27 / **1.63** | 4.11 / **1.83** | 10.19 `a1_j2` | CONCURRENT |
| 8 | ✅ CONCURRENT | 4.04 / **1.57** | 4.94 / **1.51** | **13.56** `a2_j2` | TORQUE-ABORT |
| 9 | ✅ CONCURRENT | 3.66 / **0.95** | 4.30 / **1.67** | **13.01** `a2_j2` | TORQUE-ABORT |
| 10 | ✅ CONCURRENT | 4.16 / **1.66** | 3.60 / **1.93** | 6.80 `a1_j2` | CONCURRENT |

Semua jendela 41–42 sampel @ 20.0 Hz, `instrument_ok`, `n_tf_fail = 0`.
**Nol fault, nol Kortex exception, nol red LED** sepanjang sesi.
**STAGGERED = 0.** Orientasi maks ≤ 2.99° di semua lengan.

#### B2.1 🔒 A2 — DUA pembacaan, keduanya harus disebut (preseden g16 B5.1)

**Pembacaan 1 — A1 sebagaimana DIKUNCI** (posisi, orientasi, dwell bersama;
torsi **bukan** bagian A1):

> **9 / 10 CONCURRENT ≥ 8 → LANGKAH 3 LULUS**, margin satu.

**Pembacaan 2 — torsi.**

| ambang | percobaan melewatinya | kalau dikeluarkan |
|---|---|---|
| rating `joint_2` **14 N·m** (definisi g16 B5.1) | **0** (g16: 3 dari 8) | 9 / 10 |
| nominal KA-75+ **12 N·m** | 3 (pasangan 2, 8, 9: 12.71 / 13.56 / 13.01) | **6 / 10 → tidak lulus** |

Yang berlaku formal adalah pembacaan 1. Pembacaan 2 wajib ikut di naskah: tiga
dari sembilan sukses dicapai **di atas torsi kontinu** aktuator.

#### B2.2 Pasangan 6 — penyaring bekerja, dan g17 B3.2(b) terkonfirmasi di perangkat keras

Ketiga rencana `arm_2` ditolak **sebelum gerak**: `joint_2` → 15.36 / 15.39 /
14.63 (110 / 110 / 105 %). Saat plan-check (arm_1 di **rest**) rencana arm_2
yang sama lolos di 12.71 (92 %). Adegan dengan arm_1 **di targetnya** membuat
OMPL memilih lintasan lain. Persis yang g17 B3.2(b) peringatkan: saringan
pemilihan arm_2 dilakukan pada kondisi yang salah.

Dihitung **di dalam penyebut** (kegagalan kelayakan, bukan mesin), **tidak
diganti** (A2), sama seperti g17 B3.

#### B2.3 🔒 Tugas 3 — apakah arm_1 MENAHAN di bawah gravitasi selagi arm_2 terbang?

**Ya.** Dari sampel penilai, dijendelakan pada waktu eksekusi `arm_2` dari log
`move_group`:

| pasangan | eksekusi arm_2 | galat arm_1 selama itu | drift vs 5 s sebelum |
|---|---|---|---|
| 1 | 8.7 s | 1.46 – 1.66 mm | **+0.17** |
| 2 | 12.6 s | 1.24 – 1.26 | +0.01 |
| 3 | 9.0 s | 0.95 – 0.97 | +0.01 |
| 4 | 8.5 s | 0.83 – 0.84 | +0.01 |
| 5 | 12.7 s | 1.74 – 1.76 | +0.01 |
| 7 | 13.0 s | 1.60 – 1.61 | +0.00 |
| 8 | 12.5 s | 1.55 – 1.57 | +0.02 |
| 9 | 8.7 s | 0.92 – 0.94 | +0.00 |
| 10 | 9.1 s | 1.55 – 1.88 | +0.32 ¹ |

¹ rentang yang sama (1.56–1.89) sudah ada **sebelum** arm_2 bergerak — arm_1
masih settle; bukan disebabkan arm_2.

Galat arm_1 tertinggi selama arm_2 bergerak **1.88 mm**; gangguan terbesar yang
dapat diatribusikan **0.17 mm**. B1.1 g17 membuktikan sisi-**perintah**
(JTC menahan); ini sisi-**fisik**: servo posisi Kortex menahan beban gravitasi
di pose target selama reaksi dari lengan sebelah di gantry yang sama.

Pelengkap: selama rel bergerak 550 mm dengan kedua lengan di REST, lengan
bergeser maks **0.018°** (B0.9).

#### B2.4 Offset torsi terukur − RNEA, 19 rencana yang dieksekusi

| | `arm_1` (10) | `arm_2` (9) |
|---|---|---|
| selisih `joint_2` | +1.19 … **+7.06**, rata-rata **+4.48**, sd 2.01 | +2.22 … +6.43 |
| selisih pergelangan j4–j6 | ≤ 0.92 | ≤ 0.48 |

🔴 **+6.6 BUKAN batas atas.** Pasangan 6 arm_1: prediksi 3.91 + 6.6 = 10.51,
terukur **10.97** — melewati prediksi terkoreksi 0.46 N·m. Pasangan 8 arm_2:
margin tinggal **0.17 N·m** (13.73 lawan 13.56). Pada data g16 offset tampak
"hampir bebas dari prediksi"; pada arm_2 di sini selisihnya **membesar dengan
prediksi** (prediksi 0.8–1.9 → +2.2…+4.6; prediksi 4.8–7.1 → +6.0…+6.4). Tidak
ada yang melewati 14, tetapi rencana yang diprediksi 13.9 (diizinkan) dapat
terukur ~14.4. Tidak ada ambang yang diubah; dicatat untuk keputusan operator.

Offset pergelangan 1.98 (g17 B1.2) terbukti konservatif di semua 19 rencana.

---

### B3. 🔒 D51–D56 DINILAI

| # | Dugaan | Terukur | Vonis |
|---|---|---|---|
| **D51** | dari percobaan yang KEDUA lengannya sampai, ≥ 8/10 CONCURRENT | kedua lengan sampai di **9**; **9 / 9 CONCURRENT** (9/10 keseluruhan) | ✅ **TEPAT** |
| **D52** | STAGGERED = 0 | **0** | ✅ **TEPAT** |
| **D53** | pemblokir dominan INTEGRASI, bukan metode; ≥ 1 memakan > 30 menit | pemblokir integrasi banyak (B0.2–B0.6) tetapi masing-masing < 30 menit (terlama: stash ros2_kortex ≈ 17 menit, 11:32 → 11:49); satu-satunya > 30 menit adalah **origin gantry** (≈ 40 menit), jenis **keadaan perangkat keras / kalibrasi**, bukan integrasi kode | ❌ **MELESET** ² |
| **D54** | penyaring antar-lengan menyala 0/10 | `INTERARM-COLLIDE` **0**; min sepanjang rencana dieksekusi **402.4 mm** | ✅ **TEPAT** |
| **D55** | offset PINDAH ke arm_2: selisih ∈ 4.8…8.4 untuk ≥ 4 dari 5 pose pertama | pasangan 1–5: +3.35, +2.22, +4.56, **+6.03**, +3.13 → **1 / 5** | ❌ **MELESET** |
| **D56** | galat posisi maks arm_2 p95 dalam ±1 mm dari 4.63 | lihat bawah | ✅ **TEPAT** ³ |

² Konjungsi; dan kenyataannya **membelah menurut jenis** (banyak integrasi pendek
lawan satu perangkat keras panjang) — aturan D46 G15: meleset utuh. Waktu dari
cap waktu log, bukan stopwatch.

³ **D56, dua bacaan, dan mana yang dihitung.** Operator menginstruksikan: pakai
`pos_err_max_settled_mm`, bukan `pos_err_max` (g17 B1.3: artefak saat-masuk).

| statistik | arm_1 (sesi ini) | arm_2 | selisih |
|---|---|---|---|
| `pos_err_max` p95 (bunyi D56 apa adanya, lawan 4.63) | 4.85 | **4.86** | +0.23 dari 4.63 — lolos, tetapi **kedua** lengan menempel di ambang, artefak |
| **`pos_err_max_settled` p95** | **1.66** | **2.57** | **+0.91** — lolos, tipis |

Yang **dihitung** adalah baris settled, dengan pembanding arm_1 **sesi yang sama**
(4.63 g16 adalah statistik artefak, jadi tidak dapat dipakai sebagai acuan
settled). Tercatat: arm_2 **konsisten ~0.5 mm lebih besar** dari arm_1
(p50 1.83 lawan 1.37) — "identik" secara mekanis, tidak identik secara L2.

➜ Papan skor §7.2: **32 meleset / 18 tepat → 34 meleset / 22 tepat.**
Pelajaran: prior "integrasi sendiri lebih salah" kembali **tepat untuk jumlah
pemblokir**, tetapi yang **mahal** sesi ini adalah keadaan dunia (origin gantry)
— dan D55 meleset karena satu kalibrasi 7 titik di satu lengan diperlakukan
sebagai konstanta keluarga aktuator.

---

## C. Keadaan akhir, dan yang BELUM dikerjakan (Rule 12)

- Lengan dikembalikan ke **REST** setelah pasangan 10 (izin operator): galat
  akhir 0.024°, torsi puncak 8.47 N·m `t1_a1_joint_2`, antar-lengan min 448.6 mm.
- Rel 0.549737 m, rotasi 0.000; stack hidup; nol fault.
- **Belum diperbaiki, disengaja:** label probe (pertentangan 6); saringan
  pemilihan arm_2 dengan arm_1 di target (B2.2, sekarang terukur di perangkat
  keras); SRDF 112/121 pasangan "Never" (g17 B0.1 — jarak min 278.6 mm, belum
  mendesak); URDF palsu di RSP/move_group (B0.6); `ros2_control_node` membuat
  crash dump saat **shutdown** (SIGINT); `/tmp/g17_step3.json` di-hardcode dan
  ditimpa tiap run.
- **Keputusan operator yang terbuka:** apakah +6.6 perlu diganti model yang
  membesar dengan prediksi, atau `joint_2` dibandingkan ke nominal 12 (B2.4).
- `p1_state.md` §8c belum diperbarui (pertentangan 7).
- Data mentah: [results/p1_g18/](results/p1_g18/) — perekam `/joint_states`
  semua percobaan (gzip), sampel & ringkasan penilai, log per percobaan, skrip
  runner/analisis.

## D. Langkah berikutnya

Menurut `p1_state.md` §8c: **langkah 4 — gerak gantry antar tugas, biaya setup
nyata**. Prasyarat yang sesi ini buka: origin gantry diverifikasi **fisik**
sebelum gerak apa pun ([[gantry-origin-offset-2026-09]]), dan batas end stop
~1656 mm dari home.

**Rekomendasi: Opus 5, effort TINGGI** — langkah 4 menambahkan gerak gantry
pada lengan yang sedang menahan pose, dan kesalahan di sana (origin, end stop,
bridge yang tidak ARMED) bersifat **diam** sampai perangkat keras menabrak.
