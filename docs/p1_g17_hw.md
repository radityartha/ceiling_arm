# P1 / G17-HW — §8c langkah 3: DUA lengan, jendela dwell BERSAMA

Lanjutan [p1_g16_hw.md](p1_g16_hw.md), yang **selesai**: langkah 2 lulus 8/10
pada kriteria A1 terkunci ([§B5.1](p1_g16_hw.md)), dengan pembacaan kedua yang
wajib ikut — tiga dari delapan sukses itu menuntut torsi di atas rating
`joint_2`. Dokumen ini dimulai baru karena §B di sana sudah ~700 baris.

Yang **diwarisi apa adanya dan tidak ditulis ulang**: §A1 (definisi sukses),
§A3 (pembagian vonis), §A4 (apa yang basi setelah pasang ulang), §A5 (sebab
mesin), §A6 (palang keselamatan S1–S7), §A10 (Rule 6 dikecualikan).

---

## A. Protokol — DIKUNCI SEBELUM PERANGKAT KERAS DIPASANG

### A1. 🔒 DEFINISI SUKSES N-LENGAN — diwarisi, tidak digeser

Dari [p1_g4_reach_dwell.md §A1](p1_g4_reach_dwell.md), per lengan:

```
posisi   : ||p_tool - p_cmd||        <  5 mm
orientasi: sudut(a_tool, a_cmd)      <  5 deg   (sumbu approach, roll bebas)
dwell    : 2.0 s KONTINU, >= 10 Hz   (satu sampel di luar toleransi = RESET)
```

Dan untuk N lengan, yang merupakan **besaran baru sesi ini**:

> Sukses N-lengan = ada **SATU** jendela 2.0 s di mana **SEMUA** lengan
> memenuhi toleransinya **BERSAMAAN**.

🔴 **Bukan** "masing-masing lengan sukses di suatu saat dalam episode ini". Itu
dapat dipenuhi dengan **bergantian**, dan bergantian persis yang dilakukan
prior work (`p1_plan.md §2c`, Harada 2015 "either… or"). Sesi ini melaporkan
keduanya secara terpisah dan **tidak pernah** menjumlahkannya:

| Vonis | Artinya |
|---|---|
| **CONCURRENT** | jendela BERSAMA 2.0 s ada — **ini** hasilnya |
| **STAGGERED** | kedua lengan sukses, tetapi **tidak pernah bersamaan** — BUKAN sukses |
| **PARTIAL** | satu lengan sukses, satu tidak |
| **NEITHER** | tidak ada yang sukses |

### A2. 🔒 APA YANG MEMBUAT LANGKAH 3 "LULUS" — dikunci sebelum ada sampel

| | Dikunci |
|---|---|
| **Jumlah percobaan** | **10 pasangan** target, `arm_1` + `arm_2` pada gantry 1, pose berbeda-beda, daftar dikunci di [B0.3](#b03-🔒-daftar-pasangan-target--dikunci-sebelum-percobaan-pertama) **sebelum percobaan pertama** |
| **LULUS** | **≥ 8 dari 10** menghasilkan **CONCURRENT** |
| **Sumber target** | himpunan **AMAN-TORSI** ([g16 §B6](p1_g16_hw.md)), bukan himpunan terjangkau — inilah koreksi langsung dari g16 B5.1 |
| **Dilaporkan apa pun hasilnya** | CONCURRENT / STAGGERED / PARTIAL / NEITHER per percobaan, galat pos & ori p50/p95 **per lengan**, torsi puncak, dan jarak antar-lengan minimum sepanjang tiap rencana |
| **Percobaan DILARANG dibuang** | sebab mesin (A5) → **TIDAK VALID**, disebut sebabnya, diulang |

Ambang 10 dan palang 8 diwarisi dari A2 langkah 2 **secara analogi**, dan
dinyatakan di sini **sebelum** data ada supaya tidak ada yang bisa diklaim
memilihnya belakangan.

### A3. 🔒 Palang keselamatan yang BARU untuk langkah 3

S1–S7 g16 tetap berlaku. Tambahan, karena dua lengan berbagi satu ruang:

| # | Aturan | Angka |
|---|---|---|
| **S8** | Setiap rencana disaring terhadap lengan pasangan **SEBELUM** eksekusi, pada konfigurasinya yang **TERUKUR**, bukan yang diasumsikan | margin **50 mm** |
| **S9** | Penyaring yang tidak dapat berjalan = **MENOLAK bergerak**, sama seperti model torsi yang tidak tersedia | `INTERARM-UNSCREENED` |
| **S10** | 🔴 SRDF **tidak boleh** dipakai sebagai cadangan penyaring — lihat [B0.1](#b01--112-dari-121-pasangan-tabrakan-antar-lengan-dimatikan-di-srdf) | — |
| **S11** | Kedua lengan digerakkan **BERURUTAN**, tidak serentak. Yang diukur adalah apakah jendela bersama ADA, bukan apakah keduanya terbang bersamaan | — |

### A9. 🔒 Papan skor §7.2 — dugaan sesi ini, DITULIS SEBELUM DATA DIAMBIL

Papan skor masuk sesi ini: **32 meleset, 18 tepat**. Pelajaran g16 yang dibawa:
prior "kode sendiri lebih salah" **terlalu pesimistis untuk METODE**, tetap
**tepat untuk INTEGRASI**. Dugaan di bawah dipasang mengikuti pemisahan itu.

| # | Dugaan | Dasar |
|---|---|---|
| **D51** | 🔴 **jendela BERSAMA** — dari percobaan yang KEDUA lengannya sampai, **≥ 8/10** menghasilkan CONCURRENT | g16 B5.3: kesembilan kedatangan menahan 2.0 s penuh, **nol** reset jendela. Yang langka adalah SAMPAI, bukan MENAHAN. Kalau `arm_1` benar-benar menahan sementara `arm_2` terbang, jendela bersama nyaris otomatis |
| **D52** | **STAGGERED = 0** | tidak ada mekanisme yang membuat `arm_1` keluar toleransi setelah sampai — ia dikunci kontrol posisi. STAGGERED > 0 berarti ada yang melayang, dan itu temuan sendiri |
| **D53** | pemblokir dominan sesi ini bertipe **INTEGRASI**, bukan metode; **≥ 1** memakan > 30 menit | dua pemblokir g16 (bridge gantry, balapan DDS) dua-duanya integrasi. `arm_2` **belum pernah** digerakkan di perangkat keras nyata dalam proyek ini |
| **D54** | penyaring antar-lengan menyala pada **0 dari 10** pasangan terkunci | jarak minimum di pose TUJUAN sudah terukur 262 mm ([B0.3](#b03-🔒-daftar-pasangan-target--dikunci-sebelum-percobaan-pertama)), dan pemisahan farthest-point memisahkan kedua lengan ke sisi-x yang berlawanan. Kalau ia menyala, berarti **transit** jauh lebih sempit daripada tujuan — itu hasil yang lebih menarik daripada dugaannya |
| **D55** | offset kalibrasi **+6.6 N·m PINDAH** ke `arm_2`: selisih (terukur − prediksi RNEA) jatuh di **4.8…8.4 N·m** untuk ≥ 4 dari 5 pose pertama `arm_2` | B5.4 mengukurnya di `arm_2`… **tidak**, di `arm_1` saja. Kalau ia gesekan + rugi gearbox, ia sifat keluarga aktuator dan harus pindah. Kalau tidak pindah, penyaring torsi `arm_2` **tidak terkalibrasi** dan itu harus diketahui sebelum dipercaya |
| **D56** | galat posisi maks `arm_2` p95 berada dalam **±1 mm** dari 4.63 mm milik `arm_1` | L2 adalah galat pelacakan servo dari perangkat keras yang identik |

🔒 Keenam dugaan ditulis **sebelum** satu pun percobaan langkah 3 dijalankan,
dan dinilai apa adanya. D51 adalah yang wajib menurut tugas 3: ia tentang
jendela bersama, besaran yang **belum pernah diukur sama sekali**.

---

## B. Hasil terukur

### B0. TAHAP OFFLINE — dijalankan 2026-08-17, **nol gerak**

Semuanya di bawah ini diukur tanpa menyalakan satu lengan pun. Itu disengaja:
setiap temuan di bawah akan lebih mahal kalau ditemukan dengan lengan bergerak.

#### B0.1 🔴 112 dari 121 pasangan tabrakan antar-lengan DIMATIKAN di SRDF

Diukur pada berkas yang benar-benar dimuat sel ini,
`workcell_moveit_config/config/trailer_workcell.srdf` (di-symlink ke `install/`,
jadi tidak ada salinan basi):

| | |
|---|---|
| link `t1_a1_*` / `t1_a2_*` bergeometri tabrakan | 11 / 11 |
| pasangan silang bergeometri | **121** |
| **`<disable_collisions … reason="Never"/>`** | **112** |
| masih diperiksa MoveIt | **9** — semuanya pergelangan lawan lengan/lengan-bawah |
| gantry 2 | sama: 143 entri silang, semua `"Never"` |

Yang **mati** mencakup `arm↔arm`, `forearm↔forearm`, `base↔apa pun`, dan
**seluruh** pasangan gripper.

`"Never"` adalah laporan MoveIt Setup Assistant bahwa ia **tidak pernah
menyampel** kedua link bersentuhan. Di sini ia salah, dan geometrinya yang
membuktikan:

| | |
|---|---|
| jarak dudukan `arm_1`↔`arm_2` di gantry 1 | **0.800 m** (diukur dari URDF hidup) |
| jangkauan `base_link → tool_frame` | **1.005 m** (sampel acak 20k) |
| → tiap lengan dapat menyapu **0.205 m melewati dudukan lengan satunya** | |

Bukti langsungnya, dihitung offline: menggiring **kedua** tool ke titik dunia
yang sama `(0.55, 0.36, 1.40)` membuat `t1_a1_left_finger_prox_link` dan
`t1_a2_left_finger_prox_link` **bersentuhan** — dan pasangan itu persis
`trailer_workcell.srdf:339`, dimatikan `reason="Never"`.

🔴 **Konsekuensinya:** MoveIt akan merencanakan `arm_1` **menembus** `arm_2` dan
mengembalikan rencana itu sebagai **SAH**. Ini bukan celah octomap dan bukan
celah self-filter — dua hal yang [[gng-collision-static-self-collision-fix]]
tangani; ini matriks tabrakan SRDF itu sendiri, satu lapis di bawahnya.

⚠️ **Perbaikan SRDF TIDAK dilakukan sesi ini, dan itu keputusan, bukan
kelalaian.** Membuang 112 entri itu mengubah perilaku perencana untuk seluruh
sel, sehingga langkah 3 akan mengukur sistem yang **berbeda** dari yang diukur
langkah 2 — dan pose yang lalu kembali `NO-PLAN` sementara A2 melarang
menggantinya. Perlindungan sesi ini adalah penyaring pra-eksekusi (B0.4), yang
sudah terbukti menyala. Perbaikan SRDF direkomendasikan sebagai perubahan
tersendiri, dan **D54 adalah yang menentukan apakah ia mendesak**.

#### B0.2 Ruang kerja AMAN-TORSI `arm_2` — dipetakan, dan cocok dengan `arm_1`

`python3 scripts/torque_safe_workspace.py --arm arm_2 --lin 0.550`

| | `arm_1` (g16 B6) | **`arm_2` (baru)** |
|---|---|---|
| terjangkau @ lin=0.550 | 1014 / 3132 | **694 / 3132** |
| IK tidak konvergen | 113 | **63** |
| **AMAN-TORSI** | 628 / 901 = **69.7 %** | **445 / 631 = 70.5 %** |
| TIDAK AMAN | 30.3 % | **29.5 %** |
| sendi pengikat | `joint_2` **96 %** | `joint_2` **95.7 %** |
| kotak aman | x 0.36…1.57 | **x 0.00…0.71** |

Dua lengan yang identik secara mekanis memberi **69.7 %** dan **70.5 %**, dengan
`joint_2` mengikat 96 % dan 95.7 %. Kesamaan itu menguatkan bahwa temuan g16 B6
adalah sifat **geometri pemasangan di langit-langit**, bukan sifat satu unit.

🔴 Irisan kedua himpunan aman hanya **21 node (3.3 % dari milik `arm_1`)**, di
kotak x 0.43…0.71. Jadi keamanan torsi sendiri sudah **memisahkan** kedua lengan
ke sisi ruang kerja yang berbeda, jauh sebelum ada yang memikirkan tabrakan.

⚠️ Batas klaim yang sama seperti g16 B6 berlaku utuh: **statis saja**, jadi
syarat **PERLU, bukan CUKUP**.

#### B0.3 ~~DAFTAR PASANGAN TARGET~~ — 🔴 DIGANTIKAN oleh [B2](#b2--daftar-pasangan-terkunci-yang-berlaku--saringan-tingkat-lintasan-33)

⚠️ **Daftar di bawah TIDAK dipakai.** Ia dipilih dari peta AMAN-TORSI **statis**,
dan B1.8 mengukur bahwa hanya **5 dari 10** pasangannya layak di tingkat
lintasan — tidak dapat mencapai palang A2 ≥ 8/10. Ia ditinggalkan **sebelum ada
satu pun data perangkat keras nyata**, jadi tidak ada hasil yang dipilih-pilih;
yang diperbaiki adalah **kriteria pemilihan**, bukan palangnya. Disimpan apa
adanya sebagai catatan bagaimana daftar itu terbentuk.

`python3 scripts/dual_arm_targets.py --pairs 10` — farthest-point per lengan
(metode identik `reachable_targets.py`, jadi keduanya sebanding), lalu tiap
pasangan disaring pada **konfigurasi tujuannya** dengan pemeriksa B0.4.

| # | target `arm_1` | target `arm_2` | jarak lengan @tujuan | jarak titik |
|---|---|---|---|---|
| 1 | `1.000, 0.318, 1.240` | `0.214, 0.247, 1.240` | 507.0 mm | 788.9 |
| 2 | `0.857, 0.035, 1.080` | `0.429, 0.318, 1.000` | 350.4 mm | 519.4 |
| 3 | `1.214, 0.106, 1.080` | `0.429, 0.035, 1.160` | 616.0 mm | 792.9 |
| 4 | `1.071, 0.035, 1.400` | `0.286, 0.529, 1.160` | 478.8 mm | 958.7 |
| 5 | `0.786, 0.388, 1.000` | `0.071, 0.176, 1.000` | 527.0 mm | 745.0 |
| 6 | `1.286, 0.459, 1.160` | `0.429, 0.176, 1.400` | 455.6 mm | 933.8 |
| 7 | `0.786, 0.529, 1.320` | `0.500, 0.388, 1.240` | **262.5 mm** ← terketat | 328.6 |
| 8 | `0.786, 0.176, 1.400` | `0.214, 0.459, 1.400` | 506.2 mm | 637.4 |
| 9 | `1.071, 0.600, 1.320` | `0.000, 0.388, 1.240` | 547.3 mm | 1095.1 |
| 10 | `1.071, 0.529, 1.000` | `0.214, 0.035, 1.400` | 664.2 mm | 1067.2 |

**Nol pasangan ditolak** penyaring tujuan. Itu bukan penyaring yang lunak —
ia sudah terbukti menyala (B0.4); penyebabnya B0.2: keamanan torsi memisahkan
kedua lengan lebih dulu. Simpanan: `/tmp/g17_pairs.npy`.

🔒 Urutan ini dikunci. Titik yang `NO-PLAN` dilaporkan `NO-PLAN`, **tidak
diganti**.

#### B0.4 Pemeriksa tabrakan antar-lengan — ditulis, dan DIVALIDASI menyala

[scripts/interarm_collision.py](../scripts/interarm_collision.py). pinocchio +
hpp-fcl atas 121 pasangan silang — **hanya** pasangan silang; tabrakan-diri satu
lengan, lengan-lawan-dunia, dan lengan-lawan-octomap tetap urusan MoveIt dan
**tidak** dimatikan di sana.

Pemeriksa yang belum pernah menyala bukan bukti apa-apa, jadi ia diuji pada dua
konfigurasi yang jawabannya sudah diketahui:

| Uji | Hasil |
|---|---|
| A. dua lengan MENGGANTUNG | 622.7 mm → **BEBAS** ✅ |
| B. dua tool ke titik dunia yang SAMA | 0.0 mm → **TABRAKAN** ✅ (`left_finger_prox` ↔ `left_finger_prox`) |
| C. lintasan `arm_1` menembus `arm_2` yang ditahan | **COLLIDE** di titik 28/29 ✅ |
| D. lintasan `arm_1` ke target pasangan 1, `arm_2` menggantung | **CLEAR**, 508.9 mm ✅ |

Disambung ke [scripts/reach_dwell_probe.py](../scripts/reach_dwell_probe.py)
`move_to()`, **sesudah** penyaring torsi dan **sebelum** `ExecuteTrajectory` —
titik yang sama, dan alasan yang sama seperti g16 B4.2: penjaga yang hanya
**mengamati** sudah terlambat begitu ia berbunyi.

⚠️ Batasnya, dinyatakan: pemeriksaan **diskret di titik lintasan**, seperti
semua pemeriksaan tabrakan di sini. Margin 50 mm ada untuk menyerap itu; ia
bukan bukti.

#### B0.5 🔴 KEDUA lengan gantry 1 berbagi SATU controller

`moveit_controllers.yaml` (yang dipakai `my_workcell.launch.py`) merutekan
**14 sendi** lewat satu `gantry_1_with_arm_controller`: gantry (2) + `arm_1` (6)
+ `arm_2` (6). `allow_partial_joints_goal: true`, jadi rencana satu-lengan
memang diterima — itu sebabnya g16 berhasil.

🔴 **Tetapi langkah 3 mengirim rencana `arm_2` ke controller yang sama, selagi
`arm_1` menahan pose dari goal sebelumnya.** Goal baru **mem-preempt** goal
lama. Apakah JTC lalu menahan sendi `arm_1` pada perintah terakhirnya, atau
melepasnya, **belum pernah diuji di sel ini** — dan D51 seluruhnya bergantung
pada jawabannya: kalau `arm_1` melayang > 5 mm saat `arm_2` terbang, jendela
bersama tidak akan pernah ada, dan penyebabnya **bukan** metode.

➜ **Ini dapat diuji di `use_fake_hardware:=true` dengan risiko fisik NOL**, dan
harus diuji begitu sebelum lengan nyata dinyalakan. Kalau JTC melepas, jalan
keluarnya sudah ada di repo: `moveit_controllers_per_arm.yaml`, satu controller
per lengan, dipakai `single_rviz_workcell.launch.py`.

#### B0.6 🔴 CELAH: `return_rest` tidak pernah di-commit

Protokol menuntut lengan dikembalikan ke MENGGANTUNG sebelum tiap percobaan,
dan prompt G17 menyebutnya "dipakai di G16". Ia **tidak ada di repo** — dicari
di `scripts/`, di seluruh pohon, dan di riwayat git: satu-satunya kemunculan
`return_rest` adalah namanya sendiri di `p1_g16_hw.md:1120`. G16 menuliskannya
inline (interpolasi sendi langsung 12 s, bukan rencana MoveIt) dan tidak
menyimpannya.

Jadi prasyarat protokol langkah 3 **hilang**, dan sekarang dibutuhkan untuk
**dua** lengan. Ia harus ditulis sebelum percobaan pertama, dengan aturan g16
B4.4: **lambat (30 s), dan HARUS DISELESAIKAN** — penjaga torsi yang
membatalkan gerak pemulihan meninggalkan lengan terdampar dalam keadaan
terbebani, dan itu lebih berbahaya daripada tidak ada penjaga.

#### B0.7 🔴 PERTENTANGAN §B lawan §A — TIGA, semuanya sebelum ada gerak

G10 lima, G11 empat, G12 dua, G13 dua, G14 empat, G15 dua, G16 enam, **G17
tiga sejauh ini**.

**(1) Risiko tabrakan bukan berbentuk octomap/self-filter.** Prompt mengarahkan
ke [[gng-collision-static-self-collision-fix]], yang benar tetapi satu lapis di
atas masalahnya: yang bocor adalah **matriks tabrakan SRDF**, 112 dari 121
pasangan silang mati (B0.1). Self-filter yang sempurna pun tidak menutupnya.

**(2) `return_rest` diasumsikan ada; ia tidak ada** (B0.6).

**(3) Ketergantungan yang tidak pernah disebut protokol: satu controller untuk
dua lengan** (B0.5). Seluruh definisi jendela bersama mengandaikan `arm_1`
menahan pose selagi `arm_2` bergerak, dan tidak ada baris protokol mana pun
yang memeriksa bahwa lapisan controller mengizinkannya.

Yang **terkonfirmasi** (bukan pertentangan): prompt menyatakan
`reach_dwell_monitor` sudah mendukung jendela bersama. Ia benar — jendela
bersama dimulai saat lengan **TERAKHIR** masuk toleransi dan mati begitu salah
satu keluar (`reach_dwell_monitor.py:243-264`), dan tugas yang sudah sukses
sengaja **tetap** dihitung (`Task.reported` komentar, baris 79-83). Tidak ada
yang perlu diubah di penilai.

---

### B1. TAHAP PERANGKAT KERAS PALSU — dijalankan 2026-08-17, lengan DIMATIKAN operator

`use_fake_hardware:=true`, ketujuh controller `active`, rel disetel ke 0.550 m
supaya konfigurasinya mewakili yang terukur. Mock component memantulkan
perintah ke state, jadi **eksekusi persis**: yang diuji di sini adalah
**pipa dan sisi-perintah**, bukan kemampuan.

#### B1.1 ✅ B0.5 TERJAWAB — JTC **menahan** lengan yang tidak disebut goal baru

Dua bukti terpisah:

| Uji | Hasil |
|---|---|
| goal parsial rel-saja (1 dari 14 sendi) mem-preempt goal sebelumnya | `arm_1` dan `arm_2` tetap **0.0000°** dari posisi sebelumnya |
| alur `--dual` penuh, pasangan 5 | `arm_1` SUCCESS pada t=679.9; jendela bersama tertutup t=722.6 — ia menahan **42.7 s** melewati seluruh perencanaan **dan** eksekusi `arm_2`, masih di dalam 5 mm |

`>>> 2-ARM CONCURRENT dwell held 2.0s (common window) -- arms ['arm_1', 'arm_2']`

➜ Risiko terbesar B0.5 **tidak terjadi**. `moveit_controllers_per_arm.yaml`
**tidak** diperlukan. Seluruh pipa dua-lengan — penerbitan dua target sebelum
gerak, penyaring torsi, penyaring antar-lengan, vonis CONCURRENT/STAGGERED,
pencatatan — bekerja ujung ke ujung.

⚠️ Yang ini **tidak** buktikan: apakah lengan NYATA menahan di bawah gravitasi.
Mock tidak melorot. Sisi-perintah aman; sisi-fisik masih milik perangkat keras.

#### B1.2 🔴 PEMBLOKIR: penyaring torsi menolak separuh rencana, dan yang mengikat PERGELANGAN

Kesepuluh pasangan terkunci di-`--plan-check` (dry run, nol gerak). 20 rencana
diminta:

| | |
|---|---|
| `PLANNED` | **9** |
| `TORQUE-UNSAFE` | **9** |
| `NO-PLAN` | 2 |
| **pasangan yang KEDUA lengannya bisa merencanakan** | **2 dari 10** (pasangan 5 dan 10) |

Sendi pengikat pada 18 rencana yang jadi:

| sendi | mengikat | rating | rentang % |
|---|---|---|---|
| **`joint_4`** (pergelangan) | **13 / 18 = 72 %** | 7 N·m | 96–101 % |
| `joint_2` (bahu) | 5 / 18 = 28 % | 14 N·m | 100–106 % |

🔴 **Sebabnya aritmetika, bukan fisika.** Offset kalibrasi **+6.6 N·m**
diterapkan ke **setiap** sendi terhadap ratingnya sendiri. Untuk pergelangan
berating 7.0 itu berarti **94.3 % rating terpakai sebelum ada torsi diprediksi
sama sekali** — sisa anggarannya **0.4 N·m**.

**Dan pengukuran G16 sendiri membantah offset itu ada di pergelangan:**

| Bukti | Nilai | Implikasi |
|---|---|---|
| g16 **B4.2**, puncak terukur percobaan 4 | `joint_4` **0.499**, `joint_5` 0.539, `joint_6` 0.592 N·m | offset +6.6 konstan akan memaksa pembacaan **≥ 6.6**. Tidak |
| g16 **B1.5**, effort istirahat | `t1_a1_joint_4` **−0.007 N·m** | gesekan 6.6 N·m mustahil menyisakan pembacaan 0.007 saat diam |
| g16 **B5.4**, data kalibrasi | ketujuh pasang adalah puncak **`joint_2`** | offset **tidak pernah diukur** di pergelangan |
| vendor (g16 B4.2) | pergelangan = **KA-58**, nominal **3.6 N·m** | suku gesekan 6.6 N·m melampaui torsi kontinu aktuatornya sendiri |

Jadi rekomendasi B5.4 — "saring pada prediksi + 6.6 terhadap rating tiap
sendi" — **diterapkan apa adanya menghasilkan penolakan hampir menyeluruh di
pergelangan**, karena data kalibrasinya hanya pernah menyentuh bahu.

⚠️ Peta AMAN-TORSI (g16 B6, dan `arm_2` di B0.2) memakai offset yang sama, tetapi
**tidak** terkena separah ini: ia memakai torsi **gravitasi statis**, yang di
pergelangan sangat kecil, sehingga anggaran 0.4 N·m biasanya cukup. Yang
menembusnya adalah suku **dinamis** RNEA pada lintasan. Itu sebabnya B6
melaporkan `joint_2` mengikat 96 %, sementara di sini `joint_4` mengikat 72 %.

**Usul perbaikan — TIDAK diterapkan, ini keputusan operator** (preseden g16
B3.3: menggeser nilai penjaga adalah keputusan operator, bukan keputusan saya).
Skalakan offset menurut keluarga aktuator, bukan satu angka:

```
joint_1..3  (KA-75+, nominal 12.0)  offset 6.6              <- sebagaimana dikalibrasi
joint_4..6  (KA-58,  nominal  3.6)  offset 6.6 x 3.6/12.0 = 1.98
```

1.98 N·m masih **3.3×** di atas puncak pergelangan terukur G16 (0.592), jadi ia
tetap konservatif — tetapi `joint_4` pasangan 1 turun dari 101 % ke **35 %**.

🔒 Ini **bukan** menggeser ambang A1, yang dilarang. A1 adalah 5 mm / 5 deg /
2.0 s dan tidak disentuh. Ini memperbaiki **di mana** sebuah konstanta
kalibrasi terukur boleh diterapkan.

#### B1.3 🔴 `pos_err_max` adalah ARTEFAK saat-masuk — dan itu mengoreksi g16 B5.2

Penilai melaporkan pasangan 5 sebagai `pos max 4.71 mm` (`arm_1`) dan
`4.98 mm` (`arm_2`) — di **mode palsu**, yang eksekusinya persis. Jejak
sampelnya menjelaskan kenapa:

| | `arm_1` | `arm_2` |
|---|---|---|
| sampel **pertama** jendela (saat masuk toleransi) | **4.71 mm** ← inilah "pos max" | **4.98 mm** |
| sampel **terakhir** (sudah diam) | **0.53 mm** | **0.76 mm** |
| rata-rata jendela | 0.54 | 0.77 |
| maks di **25 % terakhir** jendela | **0.53** | **0.76** |
| FK langsung dari sendi akhir vs `p_cmd` | **0.53 mm** | **0.76 mm** |

Jendela dwell **dimulai pada sampel yang melintasi ambang**, jadi sampel
pertamanya nyaris selalu tepat di bawah 5 mm — apa pun perangkat kerasnya.
`pos_err_max` karena itu **satu sampel saat menyeberang**, bukan ukuran
akurasi tertahan.

🔴 **Konsekuensi untuk g16 B5.2.** Di sana p95 dari `pos_err_max` = 4.63 mm
dibaca sebagai: *"yang tersisa adalah galat pelacakan servo — dan ia mengisi
hampir seluruh anggaran"*. Mode palsu menghasilkan **4.71 dan 4.98** dengan
galat servo **nol menurut konstruksi**. Statistik itu **tidak dapat
membedakan** servo sempurna dari servo buruk, jadi ia tidak menopang
kesimpulan tersebut.

⚠️ Ini **tidak** berarti galat tertahan perangkat keras nyata adalah 0.5 mm —
mock tidak punya galat servo dan lengan nyata punya. Yang dibuktikan: `pos_err_max`
tidak mengukurnya. Ukuran yang benar adalah galat di bagian **tertahan**
jendela, dan `pos_err_mean` (g16: 1.042–2.454 mm) sudah lebih dekat ke sana
walau masih memuat transien pendekatan.

➜ **Usul, tidak diterapkan:** penilai menambahkan satu kolom terlapor —
maksimum atas 25 % terakhir jendela. Itu **tidak menggeser ambang mana pun**
dan tidak mengubah satu pun vonis; preseden penambahan pelaporan tanpa
menyentuh palang sudah ada (g16 B3.4: `EXEC-MISS`, `REACHED-NOT-HELD`).

🔴 **D56 dengan demikian adalah dugaan tentang artefak**, bukan tentang lengan.
Ia tetap dinilai apa adanya, dan keterbatasannya dicatat di sini.

#### B1.4 Penyaring antar-lengan: menyala NOL kali — D54 didukung, belum dinilai

Pada setiap rencana yang sampai ke tahap penyaringan, jarak antar-lengan
minimum sepanjang lintasan:

| | |
|---|---|
| rentang terukur | **507.3 – 622.7 mm** |
| margin | 50 mm |
| `INTERARM-COLLIDE` | **0** |

Pasangan terketat sepanjang lintasan bukan gripper melainkan
`arm_link ↔ arm_link` dan `upper_wrist ↔ upper_wrist`. Konsisten dengan D54,
tetapi **D54 belum dinilai**: ia dugaan tentang percobaan perangkat keras
nyata, dan mode palsu memakai perencana serta SRDF yang sama sehingga geometri
rencananya representatif, bukan identik.

#### B1.5 `return_rest` ditulis dan divalidasi

[scripts/return_rest.py](../scripts/return_rest.py). Mode palsu mulai di
**tuck** (99.63° dari rest — persis artefak sistem palsu yang A6/S2 peringatkan),
dan pemulihan dua-lengan 30 s berjalan sampai selesai: penyaring antar-lengan
`CLEAR` minimum 622.7 mm, galat akhir 0.000°. Interpolasi sendi langsung, bukan
rencana MoveIt, dan penjaga torsi **melapor tanpa membatalkan** (g16 B4.4).

#### B1.6 🔴 PERTENTANGAN §B lawan §A — dua lagi; **G17 LIMA**

**(4) Rekomendasi g16 B5.4, diterapkan apa adanya, memblokir langkah 3.**
"+6.6 terhadap rating tiap sendi" menolak 9 dari 18 rencana, dan yang mengikat
**pergelangan** pada 72 % — sementara B1.5 dan B4.2 g16 sendiri mengukur
pergelangan pada 0.007 dan 0.592 N·m (B1.2).

**(5) `pos_err_max` tidak mengukur apa yang g16 B5.2 katakan ia ukur** (B1.3).

Catatan atas pertentangan (3): risiko satu-controller-dua-lengan **tidak
terjadi** — JTC menahan (B1.1). Ia tetap dicatat sebagai kelalaian protokol,
karena tidak ada baris protokol mana pun yang memeriksanya lebih dulu; ia
kebetulan aman, bukan terbukti aman sebelumnya.

#### B1.7 Perbaikan diterapkan (persetujuan operator) — dan apa yang tersisa

Operator menyetujui **skala per keluarga aktuator** (B1.2) dan **kolom terlapor
baru** (B1.3). Keduanya diterapkan; penilai divalidasi ulang dengan
`test/validate_reach_dwell_monitor.py` — **kelima kasus A–E LULUS**, termasuk
kasus E jendela bersama, jadi penambahan kolom tidak menyentuh satu pun vonis.

Kesepuluh pasangan di-`--plan-check` ulang, 20 rencana, **kedua lengan di
pose REST** — sama seperti jalan "sebelum", jadi perbandingannya sah:

| | sebelum | sesudah |
|---|---|---|
| `PLANNED` | 9 | **14** |
| `TORQUE-UNSAFE` | 9 | **3** |
| `NO-PLAN` | 2 | 3 |
| **pasangan yang KEDUA lengannya merencanakan** | 2 / 10 | **5 / 10** |
| **`joint_4` (pergelangan) mengikat** | **13 / 18** | **0 / 17** |

✅ **Pergelangan hilang SEPENUHNYA dari daftar pengikat: 13/18 → 0/17.** Yang
mengikat sekarang `joint_2` 10× dan `joint_3` 7× — keduanya KA-75+, tempat
offset itu benar-benar dikalibrasi. Ketiga penolakan torsi yang tersisa
mengikat `joint_2` pada 102–107 %, tepat kelas pose yang G16 ukur menarik
15.05–17.61 N·m. Penyaringnya kini menolak apa yang seharusnya ia tolak, dan
berhenti menolak yang lain.

⚠️ **KOREKSI, dan ia mengubah angka yang sempat dilaporkan.** Jalan "sesudah"
yang pertama dijalankan **bukan** dari rest: percobaan `--move` pasangan 5
meninggalkan kedua lengan di posenya (`return_rest` berikutnya melaporkan
**170.59°** dari rest), sehingga rencananya berangkat dari konfigurasi yang
salah. Angka yang sempat dilaporkan dari jalan itu — 12 `PLANNED`, 4
`TORQUE-UNSAFE`, **3/10** — **tidak sah** dan digantikan tabel di atas.
Yang **tidak** terpengaruh adalah mekanismenya: lantai 6.6/7 = 94.3 % adalah
aritmetika murni, tidak bergantung lintasan maupun pose awal.

⚠️ `NO-PLAN` tetap bervariasi antar-jalan (OMPL stokastik, 5 percobaan, 15 s),
jadi ia tidak boleh dibandingkan seketat angka torsi.

#### B1.8 🔴 A2 TIDAK DAPAT LULUS dengan daftar terkunci — dan sebabnya terukur

Hanya **5 dari 10** pasangan yang kedua lengannya menghasilkan rencana yang
lolos penyaringan (pasangan 1, 4, 5, 7, 9). Palang A2 adalah
**≥ 8 / 10 CONCURRENT**. Jadi langkah 3 **tidak dapat lulus**, dan itu
diketahui **sebelum** satu lengan nyata bergerak.

Rinciannya, dari jalan bersih:

| pasangan | `arm_1` | `arm_2` | |
|---|---|---|---|
| 1 | PLANNED | PLANNED | ✅ |
| 2 | PLANNED | NO-PLAN | |
| 3 | NO-PLAN | TORQUE-UNSAFE | |
| 4 | PLANNED | PLANNED | ✅ |
| 5 | PLANNED | PLANNED | ✅ |
| 6 | TORQUE-UNSAFE | PLANNED | |
| 7 | PLANNED | PLANNED | ✅ |
| 8 | PLANNED | NO-PLAN | |
| 9 | PLANNED | PLANNED | ✅ |
| 10 | PLANNED | TORQUE-UNSAFE | |

🔴 **Sebab pokoknya adalah cara daftar itu dipilih.** Ia diambil dari peta
AMAN-TORSI, yang **statis** — dan [g16 B6](p1_g16_hw.md) sudah menyatakannya
sendiri sebagai syarat **PERLU, BUKAN CUKUP**. B1.7 mengukur seberapa tidak
cukup:

> Peta statis melebih-lebihkan kelayakan tingkat-lintasan sekitar **2×**
> (10 pasangan lolos statis → **5** pasangan lolos rencana + torsi dinamis).

Itu **bukan** kegagalan metode dan bukan alasan menggeser palang. Ia
pengukuran atas jarak antara "ada solusi IK yang menahan gravitasi di sana" dan
"ada lintasan yang dapat sampai ke sana dalam rating sendi".

🔒 **Apa yang TIDAK boleh dilakukan:** mengganti pasangan `NO-PLAN` atau
`TORQUE-UNSAFE` dengan yang lebih mudah **setelah** melihat hasilnya. A2
melarangnya, dan larangan itu tetap berlaku.

**Yang sah, dan bedanya penting:** memperbaiki **kriteria pemilihan** dan
mengunci daftar BARU sebelum ada data perangkat keras nyata. Belum ada satu
pun percobaan langkah 3 di perangkat keras nyata, jadi tidak ada hasil yang
bisa dipilih-pilih. Kriteria yang benar adalah menyaring pada tingkat
**lintasan** — rencana MoveIt + RNEA + antar-lengan — bukan pada gravitasi
statis.

#### B1.9 🔴 `TORQUE-UNSAFE` adalah sifat LINTASAN, bukan sifat POSE — terukur

Ditemukan saat menyiapkan daftar baru: target `1.286, 0.459, 1.160` ditolak
`TORQUE-UNSAFE` pada jalan bersih B1.8, lalu **lolos** `PLANNED` pada saringan
berikutnya — lengan sama, pose awal sama (rest), penyaring sama, `--tau-max`
sama. Satu-satunya yang berbeda adalah lintasan yang kebetulan ditemukan OMPL.

Diukur, 10 ulangan per target, `arm_1`, semuanya dari rest:

| target | 10 ulangan | kelas |
|---|---|---|
| `0.786, 0.529, 1.320` | **10 PLANNED** | layak **stabil** |
| `1.214, 0.106, 1.080` | **10 NO-PLAN** | tidak layak **stabil** |
| `1.286, 0.459, 1.160` | **8 PLANNED / 2 TORQUE-UNSAFE** | **marginal, ~20 %** |

🔒 Jadi vonisnya **tidak** seragam acak — ada tiga kelas, dan hanya kelas ketiga
yang stokastik. Itu penting untuk dua hal:

**(a) Kriteria pemilihan "lolos sekali" tidak cukup.** Target marginal yang
kebetulan lolos saat pemilihan akan menyumbang ~20 % kegagalan pada hari
percobaan, dan kegagalan itu akan tampak sebagai kegagalan metode padahal ia
undian perencana. Daftar baru karena itu **BELUM dikunci**: kriterianya harus
menuntut lolos **berulang** (mis. k dari k), bukan sekali.

**(b) Sebagian dari "peta statis melebih-lebihkan 2×" (B1.8) sebenarnya varians
perencana**, bukan selisih statis-lawan-dinamis. Klaim itu karena itu dibatasi:
yang terukur adalah 10 → 5 pada **satu** sampel per pasangan, dan sebagian dari
selisih itu akan pulih dengan pengambilan sampel berulang. Besarannya belum
dipisahkan.

⚠️ **Keselamatan TIDAK terpengaruh.** Penyaring selalu menyaring lintasan yang
**akan benar-benar dieksekusi**, jadi rencana tak aman tetap ditolak. Yang
stokastik adalah **kelayakan**, bukan perlindungan.

#### B1.10 Konsekuensi B1.9 yang diterapkan — asimetris, dan sengaja

Operator menyerahkan keputusannya; diambil dan dinyatakan terbuka di sini.
Dua sisi dari satu perbaikan, keduanya memisahkan kelayakan **pose** dari
undian **perencana**:

| | Sebelum | Sesudah |
|---|---|---|
| **eksekusi** (`move_to`) | 1 rencana; ditolak = target hangus | minta sampai **3** lintasan, pakai yang **pertama lolos SEMUA saringan** |
| **pemilihan** (`plan_screen`) | lolos **1×** cukup | harus lolos **3 dari 3** sampel independen |

🔒 **Keselamatan tidak bergeser, dan ini bagian yang harus jelas.** Setiap
rencana tetap melewati penyaring tuck, torsi RNEA per-sendi, dan antar-lengan
secara penuh; hanya rencana yang lolos **semuanya** yang pernah dieksekusi.
Yang berubah adalah **arti** sebuah vonis penolakan: dari "sampel ini jelek"
menjadi **"tidak ada lintasan aman ditemukan dalam 3 percobaan"** — yang justru
lebih dekat ke apa yang A3 maksud dengan kegagalan kelayakan.

⚠️ **TIDAK diulang:** `TORQUE-UNSCREENED`, `INTERARM-UNSCREENED`, `EXEC-FAIL`.
Ketiganya berarti penyaring atau mesinnya **tidak tersedia**; mengulang alat
yang rusak hanya menyembunyikan bahwa ia rusak.

Asimetrinya disengaja: pemilihan **ketat** (target harus andal layak untuk
berhak masuk daftar terkunci), eksekusi **permisif** (target yang baik tidak
boleh hilang karena satu undian jelek). Pemilihan memakai `attempts=1` per
ulangan justru supaya logika ulang tidak menutupi varians yang sedang diukur.

#### B1.11 Penyaring antar-lengan sesudah perbaikan — masih NOL

| | |
|---|---|
| jarak minimum sepanjang rencana | **223.9 – 568.6 mm** |
| terketat | pasangan 8, `t1_a1_right_finger_dist` ↔ `t1_a2_arm_link` |
| `INTERARM-COLLIDE` | **0** |

Lebih dekat dari sebelum perbaikan (507 mm → 224 mm), karena rencana yang lolos
sekarang berbeda dan menjangkau lebih jauh. Masih **4.5×** di atas margin 50 mm.

---

### B2. 🔒 DAFTAR PASANGAN TERKUNCI YANG BERLAKU — saringan tingkat-lintasan 3/3

Menggantikan B0.3. Tiap kandidat harus menghasilkan rencana MoveIt yang lolos
penyaring tuck, torsi RNEA per-sendi, **dan** antar-lengan — **tiga kali dari
tiga sampel perencana independen** (B1.9, B1.10). Dijalankan di perangkat keras
palsu, kedua lengan di **rest**, rel **0.550 m**.

| | `arm_1` | `arm_2` |
|---|---|---|
| kandidat diuji (kolam farthest-point) | 32 | 32 |
| **lolos 3/3** | **21** | **19** |
| ditolak `TORQUE-UNSAFE` | 7 | 8 |
| ditolak `NO-PLAN` | 4 | 5 |

Pasangan ditolak penyaring tabrakan-tujuan: **0**.

🔒 **DIKUNCI 2026-08-17, SEBELUM ADA SATU PUN DATA PERANGKAT KERAS NYATA.**

| # | target `arm_1` | target `arm_2` | jarak lengan @tujuan |
|---|---|---|---|
| 1 | `1.000, 0.318, 1.240` | `0.214, 0.247, 1.240` | 507.0 mm |
| 2 | `0.857, 0.035, 1.080` | `0.286, 0.529, 1.160` | 389.7 mm |
| 3 | `0.786, 0.388, 1.000` | `0.071, 0.176, 1.000` | 527.0 mm |
| 4 | `1.286, 0.459, 1.160` | `0.500, 0.388, 1.240` | **383.9 mm** |
| 5 | `0.786, 0.529, 1.320` | `0.000, 0.388, 1.240` | 435.6 mm |
| 6 | `0.786, 0.176, 1.400` | `0.214, 0.035, 1.400` | 508.7 mm |
| 7 | `1.071, 0.600, 1.320` | `0.214, 0.388, 1.000` | 563.8 mm |
| 8 | `1.071, 0.529, 1.000` | `0.286, −0.035, 1.240` | 634.6 mm |
| 9 | `1.000, 0.247, 1.000` | `0.357, 0.318, 1.320` | 500.6 mm |
| 10 | `0.714, 0.318, 1.240` | `0.071, 0.247, 1.160` | 415.4 mm |

Titik yang `NO-PLAN` pada hari percobaan dilaporkan `NO-PLAN`, **tidak
diganti**. Simpanan: `/tmp/g17_pairs.npy`.

#### B2.1 ⚠️ Satu target di daftar ini DIKETAHUI marginal — dan aritmetikanya

Target `arm_1` pasangan **4** adalah `1.286, 0.459, 1.160`, yaitu tepat pose
yang B1.9 ukur **8 PLANNED / 2 TORQUE-UNSAFE dari 10**. Ia lolos saringan 3/3
— yang memang bisa terjadi: pada p = 0.8, peluang lolos 3/3 adalah
0.8³ = **51 %**. Jadi k = 3 **menipiskan** pose marginal, tidak
menghapusnya, dan itu harus disebut alih-alih dibiarkan tersirat.

Kenapa ia tetap dapat diterima, dan ini justru inti dari asimetri B1.10:

| | rumus | pada p = 0.8 |
|---|---|---|
| lolos **pemilihan** (3 dari 3) | p³ | 51 % |
| berhasil **eksekusi** (≥ 1 dari 3) | 1 − (1 − p)³ | **99.2 %** |

Pemilihan yang ketat membuang pose ber-p rendah; pengulangan saat eksekusi
membuat pose ber-p tinggi yang lolos hampir pasti berhasil. Pose dengan
p = 0.8 karena itu **bukan** ancaman bagi penyebut — yang akan gagal adalah
pose ber-p rendah, dan justru itu yang saringan 3/3 buang.

---

### B3. Kesepuluh pasangan dijalankan PENUH di perangkat keras PALSU

Tiap percobaan didahului `return_rest` dua-lengan, seperti protokol G16. Tujuannya
satu hal yang belum pernah teruji: `arm_2` merencanakan ketika `arm_1` **sudah
bertengger di targetnya**, bukan menggantung — adegan perencanaan yang berbeda,
dan satu-satunya cara mengujinya adalah eksekusi berurutan sungguhan.

🔴 **Ini BUKAN hasil ilmiah.** Mock membuat state = perintah, jadi CONCURRENT
nyaris gratis. Yang diuji adalah **pipa**, bukan kemampuan.

| vonis | jumlah |
|---|---|
| `CONCURRENT` | **7** |
| `HALTED` (torsi) | 2 |
| `NEITHER` | 1 → **TIDAK VALID (instrumen)**, lihat B3.1 |

Setelah bug B3.1 diperbaiki, pasangan 10 diulang **3×**: **CONCURRENT 3/3**.
Tally pipa menjadi **8 CONCURRENT / 2 HALTED**.

Penyaring antar-lengan: **nol** `INTERARM-COLLIDE` sepanjang 10 pasangan,
termasuk pada kondisi `arm_1`-di-target yang belum pernah diuji sebelumnya.

#### B3.1 🔴 BUG SAYA SENDIRI: `clear` mendahului `target` di kode, tetapi TIBA belakangan

Pasangan 10 melaporkan `NEITHER` sementara kedua lengan sebenarnya duduk **1.2 mm**
dan **1.3 mm** dari targetnya — diverifikasi dengan FK langsung dari `/joint_states`.
Log penilai menunjukkan sebabnya:

```
1787008955.027  arm_1: target set (0.714, 0.318, 1.240) in world
1787008955.028  arm_2: target set (0.071, 0.247, 1.160) in world
1787008955.028  cleared: arm_1     <-- 1 ms SESUDAH
1787008955.029  cleared: arm_2
```

`dual_trial` memanggil `clear()` **sebelum** `publish_target()`, tetapi keduanya
topik terpisah dengan waktu pencocokan DDS terpisah, dan `clear` **tiba belakangan**
— menghapus target yang baru dipasang. Penilai kehilangan tugas aktif, berhenti
menyampel, lalu melaporkan `NEITHER`.

🔴 **Kenapa ini berbahaya, bukan sekadar salah:** ia menghasilkan vonis yang
**masuk akal** (`NEITHER` di antara CONCURRENT lain), bukan kerusakan yang kentara —
persis sifat yang membuat g16 B3.4(6) hampir lolos. Ia juga bergantung waktu, jadi
ia mengenai satu percobaan dari sepuluh dan bisa mengenai yang mana pun.

**Perbaikan:** `clear` **dihapus** dari `dual_trial`. Ia mubazir sejak awal —
`_on_target` sudah mengganti `tasks[arm]` dan mereset `concurrent_since` /
`concurrent_reported`, jadi menargetkan ulang sebuah lengan sudah melakukan semua
yang `clear` lakukan. Menghapusnya meniadakan balapannya, bukan mencoba
mengalahkannya dengan jeda.

#### B3.2 ⚠️ k = 3 terlalu lemah: 2 dari 20 rencana gagal meski lolos 3/3

| pasangan | lengan | target | kondisi saat PEMILIHAN | kondisi saat PERCOBAAN |
|---|---|---|---|---|
| 4 | `arm_2` | `0.500, 0.388, 1.240` | `arm_1` di **rest** | `arm_1` di **target** |
| 6 | `arm_1` | `1.071, 0.600, 1.320` | `arm_2` di rest | `arm_2` di rest — **sama** |

Keduanya `TORQUE-UNSAFE` **setelah 3 kali ulang** saat percobaan, padahal lolos
3-dari-3 saat pemilihan. Dua sebab berbeda, dan keduanya harus dipisah:

**(a) Pasangan 6 adalah varians perencana murni** — kondisinya identik antara
pemilihan dan percobaan. Gagal 3× berturut-turut setelah lolos 3× berturut-turut
berarti p-nya di sekitar 0.5, bukan 0.8. Pada p = 0.5, peluang lolos 3/3 adalah
12.5 % dan peluang gagal 3/3 juga 12.5 % — sama-sama mungkin. **k = 3 tidak cukup
untuk mencirikan p**; ia hanya membuang yang paling buruk.

**(b) Pasangan 4 disaring pada kondisi yang SALAH.** Saat pemilihan, `arm_2`
disaring dengan `arm_1` menggantung; saat percobaan, `arm_1` berada di targetnya.
Torsi RNEA `arm_2` tidak bergantung pada `arm_1`, tetapi **lintasan yang dipilih
perencana** bergantung — adegan tabrakannya berbeda. Jadi saringan `arm_2`
mengukur situasi yang tidak akan pernah terjadi.

➜ **Untuk sesi berikutnya, dua perbaikan TERPISAH:** naikkan k (dan laporkan p per
kandidat, bukan lolos/tidak), dan saring `arm_2` dengan `arm_1` **ditempatkan di
target pasangannya**, bukan di rest.

#### B3.3 🔴 PERTENTANGAN §B lawan §A — satu lagi; **G17 ENAM**

**(6) Instrumen dapat kehilangan targetnya sendiri karena urutan DDS** (B3.1).
Protokol mengandaikan "terbitkan target sebelum gerak" cukup untuk menjamin penilai
menilai `p_cmd` yang benar. G16 sudah memperbaiki satu balapan pada jalur itu; ini
balapan **kedua**, pada jalur `clear`, dan ia menghasilkan vonis yang tampak masuk
akal alih-alih kerusakan yang kentara.

---

## C. Yang BELUM dikerjakan — dinyatakan terbuka (Rule 12)

- 🔴 **Nol percobaan langkah 3 pada perangkat keras NYATA.** Tidak ada lengan
  fisik yang bergerak sesi ini; operator mematikan lengan sebelum tahap palsu.
  **D51–D56 semuanya BELUM DINILAI**; papan skor tetap **32 meleset, 18 tepat**.
- 🔴 **`arm_2` masih BELUM PERNAH digerakkan di perangkat keras nyata** dalam
  proyek ini. Seluruh bukti dua-lengan sesi ini berasal dari mock.
- ⚠️ **Daftar B2 dikunci dengan k = 3, dan B3.2 mengukur k = 3 terlalu lemah.**
  Dua dari 20 rencana gagal saat percobaan meski lolos 3/3. Daftar tetap
  DIKUNCI (mengubahnya setelah melihat hasil percobaan palsu akan melanggar
  semangat A2), tetapi tingkat kegagalan yang diharapkan **bukan nol** dan
  harus dilaporkan bersama hasilnya.
- ⚠️ **Saringan `arm_2` memakai kondisi yang salah** (`arm_1` di rest, bukan di
  target pasangannya) — B3.2(b). Perbaikannya belum ditulis.
- Perbaikan SRDF **tidak dilakukan**, sengaja (B0.1). Jarak terukur 224–623 mm
  di sepuluh pasangan, jadi belum mendesak — tetapi belum pernah diuji pada
  lintasan yang benar-benar berdesakan.
- **Belum diukur sama sekali:** galat L2 nyata `arm_2`, transfer offset torsi
  ke `arm_2` (D55), dan apakah `arm_1` menahan pose di bawah **gravitasi**
  selagi `arm_2` terbang. B1.1 hanya membuktikan sisi-**perintah**; mock tidak
  melorot.

### C1. Ringkasan yang DIKERJAKAN sesi ini

| | |
|---|---|
| pertentangan §B lawan §A | **ENAM** (B0.7 tiga, B1.6 dua, B3.3 satu) |
| cacat sistem diwarisi yang ditemukan | 4 (SRDF, offset pergelangan, `pos_err_max`, `return_rest` hilang) |
| bug yang saya buat sendiri lalu perbaiki | 1 (balapan `clear`/`target`, B3.1) |
| skrip baru | `interarm_collision.py`, `dual_arm_targets.py`, `return_rest.py` |
| gerak perangkat keras fisik | **NOL** |

---

## D. Prompt sesi berikutnya — G18-HW (salin ke chat BARU)

**Rekomendasi: Opus 5, effort TINGGI.** Naik, bukan turun. G17 mengerjakan
seluruh bagian yang bisa dikerjakan tanpa lengan, jadi yang tersisa untuk G18
adalah **hanya** bagian berisiko fisik: dua lengan nyata pada gantry yang sama,
`arm_2` yang **belum pernah** digerakkan di perangkat keras dalam proyek ini,
dan penyaring pra-eksekusi sebagai **satu-satunya** perlindungan karena
`ros2_control` tidak menegakkan batas effort dan `fault_controller` tidak
di-spawn. Tidak ada lagi kerja persiapan yang bisa menyerap kesalahan.

```
Sesi G18-HW -- p1_state.md 8c LANGKAH 3 di PERANGKAT KERAS NYATA.
SESI PERANGKAT KERAS. Seluruh persiapan SELESAI di G17. Yang tersisa
hanyalah bagian yang berbahaya: DUA lengan nyata berbagi gantry_1.

BACA DULU:
1. docs/p1_g17_hw.md -- §A DIKUNCI. §B TERUKUR.
   B2    (DAFTAR 10 PASANGAN TERKUNCI -- pakai APA ADANYA, jangan pilih ulang)
   B2.1  (pasangan 4 arm_1 DIKETAHUI marginal p~0.8; aritmetikanya di sana)
   B0.1  (SRDF mematikan 112 dari 121 pasangan tabrakan antar-lengan)
   B1.1  (JTC MENAHAN lengan yang tidak disebut goal baru -- sisi PERINTAH saja)
   B1.2  (offset torsi PER KELUARGA AKTUATOR: 6.6 sendi 1-3, 1.98 sendi 4-6)
   B1.3  (pos_err_max adalah ARTEFAK saat-masuk; pakai pos_err_max_settled_mm)
   B3.1  (balapan clear/target -- SUDAH diperbaiki, jangan kembalikan clear)
   B3.2  (k=3 terlalu lemah; 2 dari 20 rencana tetap gagal saat percobaan)
2. docs/p1_g16_hw.md B4.2 (rating sendi), B4.4 (gerak PEMULIHAN tidak boleh
   dibatalkan), A5 (sebab mesin), A6 (S1-S7).

=== SUDAH SELESAI DI G17, JANGAN ULANGI ===
- scripts/interarm_collision.py DITULIS + DIVALIDASI (4 uji, menyala benar).
  Tersambung ke move_to() sebelum ExecuteTrajectory.
- scripts/return_rest.py DITULIS (G16 tidak pernah meng-commit-nya).
  Interpolasi sendi 30 s, dua lengan sekaligus, penjaga MELAPOR tanpa
  membatalkan.
- scripts/dual_arm_targets.py --plan-screen DITULIS; daftar B2 SUDAH dikunci.
- reach_dwell_probe.py --dual (vonis CONCURRENT/STAGGERED/PARTIAL/NEITHER),
  --plan-attempts 3, offset per-aktuator.
- reach_dwell_monitor: kolom pos_err_max_settled_mm ditambahkan; uji validasi
  5/5 LULUS. TIDAK ada ambang yang digeser.
- SELURUH pipa dijalankan 10 pasangan di perangkat keras PALSU:
  8 CONCURRENT / 2 HALTED(torsi). Itu membuktikan PIPA, bukan kemampuan.

=== TUGAS ===
1. Bring-up DUA lengan nyata, jalankan 10 pasangan TERKUNCI B2 apa adanya.
2. NILAI D51-D56 apa adanya. D51 wajib: ia satu-satunya tentang jendela
   BERSAMA, dan besaran itu BELUM PERNAH diukur di perangkat keras.
   CATATAN untuk D56: B1.3 membuktikan besaran yang ia duga adalah ARTEFAK.
   Nilai apa adanya, lalu laporkan ulang memakai pos_err_max_settled_mm.
3. Yang BELUM diuji dan hanya perangkat keras bisa menjawab:
   apakah arm_1 menahan pose di bawah GRAVITASI selagi arm_2 terbang.
   B1.1 hanya menguji sisi PERINTAH; mock tidak melorot.
4. Tulis hasil di docs/p1_g18_hw.md, tautkan balik. JANGAN sambung g17.

=== PERINTAH ===
cd ros2_ws && source install/setup.bash
ros2 launch workcell_moveit_config my_workcell.launch.py \
    use_fake_hardware:=false arm3_fake:=true arm4_fake:=true \
    enable_gantry_bridge:=true 2>&1 | tee /tmp/g18_t1.log
ros2 control list_controllers        # KETUJUHNYA active

# penilai DUA lengan
ros2 run reachability_gng reach_dwell_monitor --ros-args \
  -p arms:="['arm_1','arm_2']" \
  -p tool_frames:="['t1_a1_tool_frame','t1_a2_tool_frame']" \
  -p csv_log:=/tmp/g18_step3

# SEBELUM tiap percobaan
python3 scripts/return_rest.py --arms arm_1 arm_2 --move

# tiap pasangan (daftar B2)
python3 scripts/reach_dwell_probe.py --dual --approach 0 --trials 1 \
  --tau-max 12.0 --target <a1> --target2 <a2> --move \
  --monitor-csv /tmp/g18_step3_samples.csv

=== KESELAMATAN ===
- enable_gantry_bridge:=true WAJIB, kalau tidak TIDAK ADA gerak lengan.
- --tau-max 12.0 (nominal KA-75+). JANGAN dinaikkan agar lolos.
- fault_controller TIDAK di-spawn: red LED = reset FISIK.
- ros2_control TIDAK menegakkan batas effort. SATU-SATUNYA perlindungan
  adalah penyaring pra-eksekusi di move_to(). Jangan lewati, jangan longgarkan.
- Gerak PEMULIHAN: lambat dan HARUS DISELESAIKAN (g16 B4.4).
- Bug mid-boot kortex_driver membunuh KEEMPAT controller; beri jeda setelah
  power-on, periksa LED tiap lengan.
- A6/S7: tidak ada gerak tanpa persetujuan manusia eksplisit.

=== JEBAKAN YANG SUDAH DIUKUR ===
- /joint_states DUA penerbit, pesan TERPISAH. `--once` gagal ~90 %.
  GABUNGKAN per-nama >= 0.5 s.
- Mode palsu MULAI DI TUCK (99.63 deg dari rest). Perangkat keras nyata tidak.
- Rel mode palsu mulai di 0.000; yang TERUKUR adalah 0.550.
- Rencanakan HANYA dari pose rest. G17 sempat merencanakan dari pose sisa
  percobaan sebelumnya dan menghasilkan angka yang salah.
- Jangan rekonstruksi kunci cache dari log 3-desimal: grid berjarak ~1/28,
  jadi 0.318 sebenarnya 0.3176 dan setiap kunci meleset.
- Keluaran python lewat pipa DI-BUFFER; pakai stderr/log untuk progres.
- kill -INT ke PID INDUK ros2 launch; butuh ~20 s; jangan -9.
- Tab browser ke 192.168.2.1x = SIGPIPE(-13). TUTUP.
- --approach 0 WAJIB.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 32 meleset, 18 tepat.
- DILARANG menggeser ambang A1 (5 mm / 5 deg / 2.0 s).
- DILARANG mengganti target NO-PLAN/TORQUE-UNSAFE dengan yang lebih mudah.
  Daftar B2 dikunci SEBELUM ada data perangkat keras.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 lima, G11 empat, G12 dua, G13 dua, G14 empat, G15 dua, G16 enam,
  G17 ENAM.
- Rule 6 PENGECUALIAN EKSPLISIT untuk sesi protokol-panjang P1 (A10).
```
