# Prompt sesi berikutnya — port MS-BL-GNG (2 layer lingkungan) + GCS (action map)

> Disiapkan 2026-08-13 di akhir Sesi B. Salin blok di bawah sebagai prompt sesi baru.
> **Tidak butuh perangkat keras** (tanpa gantry, tanpa lengan, tanpa kamera).
> Satu-satunya bagian yang tidak murni offline adalah paritas IK (`eval.py ik`),
> yang butuh **move_group** hidup — dan itu bisa jalan dengan `use_fake_hardware`.

---

Sesi C — port MS-BL-GNG untuk dua layer peta lingkungan, dan GCS untuk action map.

BACA DULU, berurutan:
1. docs/p1_state.md          — §4 (tabel representasi) dan §5 (keputusan terkunci)
2. docs/p1_plan.md §3        — sub-bagian "Utang teknis metode". HANYA bagian itu
                               yang sah dari plan untuk tugas ini.
3. docs/p1_g4_reach_dwell.md — §A1/§B0, sebagai contoh disiplin: kriteria sukses
                               ditulis SEBELUM kode jalan, instrumen divalidasi
                               terhadap ground truth sintetis sebelum dipakai.

JANGAN pakai p1_plan.md §1/§3-lapisan/§4 sebagai formulasi paper (sudah dibuang) dan
jangan kutip angka §2. Sah dari plan: §2b–§2e, §3 utang teknis, §6, §7.

=== TUGAS ===

Ganti GNG Fritzke polos di TIGA tempat: dua layer peta lingkungan -> MS-BL-GNG,
dan action map -> GCS.

Ketiganya sekarang memakai `from reachability_gng.gng import GNG, GNGParams`:

  layer STATIS   : reachability_gng/map_topo_static.py  (+ topo_static_pub.py)
  layer DINAMIS  : reachability_gng/env_gng.py
  ACTION MAP     : reachability_gng/train.py:170-171   -> /tmp/arm{1..4}_model.npz

=== PENUGASAN ALGORITMA — SUDAH DIPUTUSKAN PENGGUNA 2026-08-13, JANGAN DIBUKA LAGI ===

  layer STATIS   map_topo_static.py (+topo_static_pub.py)  -> MS-BL-GNG
  layer DINAMIS  env_gng.py                                -> MS-BL-GNG
  ACTION MAP     xyz -> q  (train.py -> arm{1..4}_model.npz) -> GCS
  INDEX CAPABILITY                                          -> tetap **grid**, JANGAN diubah

Ini persis tabel p1_state.md §4. Dua catatan supaya tidak tergelincir:

- **"Action map" ≠ "capability map".** Action map memetakan satu titik tugas ke
  konfigurasi `q` representatif (GNG 8-DOF `[task|q]`, dipakai seeding IK).
  Capability map menyimpan, per node target, mask boolean atas grid pose gantry
  `(linear, rotation)` 33×72. Yang "grid" di §5.2 adalah **cara node target
  diindeks di capability map**, bukan action map-nya.
- **Index capability tetap grid — itu keputusan TERUKUR, bukan selera.** Topo IoU
  72.9% vs grid 91.4% pada distribusi realistis, jenuh terhadap jumlah node,
  latihan, dan boundary pinning. Sebabnya struktural: GNG/GCS beradaptasi ke
  **kerapatan data**, sementara error mask ditentukan **gradien mask** — tidak
  berkorelasi, jadi mengganti GNG dengan GCS **tidak memperbaikinya**.
  Jangan sekalian meng-GCS-kan `capability.py --index`.

=== 🔴 KENDALA INTEGRASI: 10 FILE MEMUAT ACTION MAP ===

Kalau action map pindah ke GCS, **`gcs.py` harus API-kompatibel dengan `GNG`**,
kalau tidak 10 file ikut berubah dan itu bukan perubahan bedah lagi.

Permukaan API minimum yang dipakai konsumen:

    GCS.load(path)          (classmethod, seperti gng.py:287)
    .query(task_vec, k)     (gng.py:215)
    .query_radius(task_vec, radius, max_k)   (gng.py:222)
    .W                      matriks node [task|q]
    .task_dim

Konsumen (verifikasi 2026-08-13): `seed_ik.py`, `seed_server.py`,
`gantry_reach_executor.py`, `reach_fusion.py`, `visualize.py`, `eval.py`,
`reachability_check.py`, `reachability_cloud.py`, `topo_static_pub.py`,
`env_gng.py`.

Model lama `/tmp/arm{1..4}_model.npz` **tetap harus bisa dimuat** oleh jalur GNG
lama — itu baseline ablation. Jadi jangan pakai ulang nama berkas yang sama untuk
keluaran GCS; pakai sufiks tersendiri (mis. `arm1_gcs.npz` — perhatikan berkas
dengan nama itu SUDAH ADA di /tmp dari eksperimen lama, jangan tertukar).

=== YANG SUDAH TERKUNCI — JANGAN DIUBAH, JANGAN DIUKUR ULANG ===

- **JANGAN SENTUH `gng.py`.** 4 model terlatih (`/tmp/arm{1..4}_model.npz`) dan
  `test/test_gng.py` bergantung padanya. Buat **file baru**: `bl_gng.py` dan
  `gcs.py`. `gng.py` tetap berguna sebagai baseline ablation.
- **MS-BL-GNG dan GCS BUKAN kontribusi.** Keduanya dikutip, bukan diklaim:
    * Ardilla, Saputra, Kubota, "Multi-Scale Batch-Learning Growing Neural Gas
      Efficiently for Dynamic Data Distributions", Int. J. Automation Technology,
      17(3), 206–216, 2023.
    * Fritzke, Growing Cell Structures.
- **Index peta kapabilitas = grid** (p1_state §5.2, terukur: topo IoU 72.9% vs
  grid 91.4%). Tugas ini menyentuh layer LINGKUNGAN dan ACTION MAP — **bukan**
  index capability. Jangan sekalian mengubahnya ke topo/GCS.
- **Peta kapabilitas sudah disapu ulang 2026-08-13** ke rel fisik 1.60 m:
  `/tmp/cap_g1_rail160.npz`, `/tmp/cap_g2_rail160.npz` (33×72, bukan 41×72).
  Yang lama (`cap_g1.npz`, `cap_g2.npz`) memuat 8 pose rel yang TIDAK ADA.
  Kalau butuh peta kapabilitas, pakai yang `_rail160`.
- Rel linier fisik mentok ~1656 mm; batas operasional **1600 mm**.
  `irm_sweep.RAIL_MAX_M = 1.60` sudah jadi default.

=== SUMBER PORT — PATH SUDAH DIVERIFIKASI 2026-08-13 ===

MS-BL-GNG:
  Meso-HSR/GNG.h:940   `void MS_GNG_learning(int m)  //  Multi-scale Batch-Learning`
  struktur pendukung   MSID / GNGinfo[m].MSL (level) / GNGinfo[m].MSD (no)
  Macro-HSR/GNG.h **identik** dengan Meso-HSR/GNG.h (md5 sama) — pakai yang mana pun.

GCS:
  FRD-01/nRobot.h:435  GCS_init
  FRD-01/nRobot.h:499  GCS_add
  FRD-01/nRobot.h:538  GCS_learning
  FRD-01/nRobot.h:593  GCS_learning2

🔴 **PERBAIKAN SIMPLEKS — WAJIB, dan ini inti kenapa nama "GCS" tidak boleh dipakai
sembarangan.** Implementasi sensei **tidak memelihara invarian simpleks**: saat
menambah node baru `g` di antara `h` dan `k`, ia tidak menyambungkan `g` ke tetangga
BERSAMA `h` dan `k`. Bloknya ada tapi **di-comment out**:

  Meso-HSR/GNG.h:634-640   blok `if (GNGinfo[m].type==1){  //  GCS` yang dikomentari

Tanpa langkah itu jaring berdegradasi jadi graf biasa — bukan GCS lagi. Memperbaikinya
adalah **GCS Fritzke standar**, jadi bukan kontribusi, tapi wajib kalau nama "GCS"
dipakai di naskah. Reviewer di society ini mengenal GCS.

⚠️ **JEBAKAN: ada TIGA versi nRobot.h yang BERBEDA di repo ini.** Jangan asal buka
yang pertama ketemu:

  FRD-01/nRobot.h                     1239 baris  GCS_add@499   <- yang dikutip plan
  Artha-HSR-01/ori/nRobot.h           1910 baris  GCS_add@704   } identik satu sama
  Artha-HSR-01/Artha-HSR-01/nRobot.h  1910 baris  GCS_add@704   } lain (md5 sama)
  Meso-HSR/nRobot.h                   1996 baris  GCS_add@675

Bandingkan dulu sebelum memilih; kalau memakai selain FRD-01, catat alasannya.

=== KRITERIA SUKSES — TULIS DAN KUNCI SEBELUM MENJALANKAN APA PUN ===

Ini pelajaran G3/G4: kriteria yang ditulis setelah melihat hasil tidak bernilai.

1. **DETERMINISME — ini kemenangan utama yang bisa diukur.**
   Input cloud yang sama -> peta statis **bit-identik** antar-run.
   SEKARANG GAGAL: `gng.py` belajar per-sampel secara online, jadi peta statis
   berubah tiap kali dibangun. Update rata-rata batch MS-BL yang membuatnya
   deterministik.
   ➜ **UKUR DULU KONDISI SEKARANG**: jalankan `map_topo_static` dua kali atas cloud
   yang sama, diff hasilnya, catat angkanya. Tanpa angka "sebelum", tidak ada yang
   bisa dibuktikan sesudahnya.

2. **Paritas kualitas — diukur, bukan diasumsikan.**
   Peta baru tidak boleh lebih buruk dari GNG sekarang pada metrik yang disebut
   eksplisit (mis. quantization error, cakupan node, jarak NN median). Tetapkan
   metriknya SEBELUM port, lalu laporkan sebelum/sesudah.

3. **Paritas action map — metriknya sudah ada, jangan bikin baru.**
   `eval.py ik` sudah membandingkan strategi seeding (`gng | none | random`) atas
   pose reachable held-out: success rate, waktu solve, manipulability. Tambahkan
   `gcs` sebagai strategi keempat dan laporkan **berdampingan** dengan `gng`.
   ⚠️ Sub-perintah `ik` **butuh move_group hidup** (`/compute_ik`) — itu satu-satunya
   bagian tugas ini yang tidak murni offline. `eval.py volume` tetap offline.
   Kalau move_group tidak dijalankan, katakan paritas IK **belum diuji**; jangan
   diganti metrik proksi diam-diam.

4. **Invarian simpleks GCS — bisa jadi assertion.**
   Setelah tiap `add`, tetangga bersama tersambung. Tulis sebagai tes yang GAGAL
   pada implementasi yang belum diperbaiki dan LULUS setelah diperbaiki. Itu bukti
   perbaikannya nyata, bukan klaim.

5. **`gng.py` tidak berubah** — `test/test_gng.py` tetap hijau, 4 model lama tetap
   termuat lewat jalur GNG (baseline ablation).

=== ATURAN WAJIB ===

- **§7.2 p1_state: UKUR, JANGAN MENDUGA.** Sepanjang korpus P1 sudah **tujuh** dugaan
  meleset, dan **semuanya ke arah yang sama**: menduga kendala lebih mengikat
  daripada kenyataannya. Kalau sebuah angka bisa diputuskan dengan pengukuran, ukur.
- **§7.1: ground truth dulu.** Untuk port, itu berarti: uji terhadap keluaran yang
  diketahui (data sintetis dengan struktur yang sudah jelas) sebelum melepasnya ke
  cloud nyata.
- Validasi instrumen/algoritma terhadap kebenaran-dasar yang tidak bisa ia pengaruhi.
  Contoh yang sudah ada: `test/validate_reach_dwell_monitor.py`.
- Tidak ada perangkat keras yang dibutuhkan. Kalau perlu cloud nyata, pakai rekaman —
  jangan nyalakan kamera/lengan untuk ini.
- Lapor jujur: kalau paritas kualitas tidak tercapai, katakan, jangan dihaluskan.

=== KONTEKS YANG MEMBUAT INI PENTING UNTUK PAPER ===

Dua utang, keduanya bisa ditanyakan reviewer secara langsung:

1. Naskah menyebut **MS-BL-GNG**, kode menjalankan **GNG Fritzke online**. Akibat
   yang paling mudah diserang: **peta "statis" berubah tiap run** — masalah
   reproduktifitas, bukan sekadar kerapian.
2. Naskah menyebut **GCS**, sementara implementasi yang ada tidak memelihara
   invarian simpleks, jadi secara teknis ia bukan GCS.

Keduanya kelas yang sama: **nama algoritma di naskah harus cocok dengan yang
benar-benar berjalan.**
