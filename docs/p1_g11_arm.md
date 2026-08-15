# P1 / G11 — REACH-1: tabrakan LENGAN–LENGAN, lubang model terakhir

> Sesi G11, 2026-08-14. Melanjutkan [p1_g10_sched4.md](p1_g10_sched4.md).
> G10 menutup pertanyaan struktur gantry–gantry: koordinasi berbiaya
> **≤ 1.01%** makespan (`p1_g10 §B6`). Yang tersisa sebagai alasan **setiap**
> angka rugi masih batas bawah adalah tabrakan **lengan–lengan** antar gantry
> (`p1_g10 §B12.2`).
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.**
> §B diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**. §A tidak ditulis ulang belakangan
> (disiplin `p1_g7`–`p1_g10`).
>
> Sesi ini **sepenuhnya offline**. Lengan 4× masih dilepas.
>
> **Rekomendasi model: Opus 5, effort TINGGI.** Alasannya spesifik untuk sesi
> ini: G7–G10 semuanya menambah kendala ke model yang **sudah punya semua
> variabelnya**. Sesi ini tidak. Predikat lengan–lengan adalah fungsi dari
> **konfigurasi lengan**, dan konfigurasi lengan **tidak ada di model
> penjadwalan sama sekali** (`p1_g7 §A4.3`: gerak lengan di dalam satu pose = 0).
> Jadi predikatnya mustahil didefinisikan tanpa **menambah variabel ke model**,
> dan penambahan itu punya sifat yang berbahaya: ia bisa menyelinap masuk lewat
> implementasi, terlihat wajar, dan tidak punya sinyal error. §A2.1 mengunci
> variabel itu **sebagai model**, bukan sebagai detail.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci / terverifikasi di | Dipakai bagaimana |
|---|---|---|
| Model intra-gantry, `T_traverse`, dwell 2.0 s, makespan, handover = SATU tugas dua-lengan | `p1_g7 §A1`/`§A2`, V0–V4 | nol perubahan |
| Peta `cap_g{1,2}_rail160.npz` (33×72, \|P\| = 2376) | `p1_state §3` | oracle kelayakan |
| `sched.solve_exact` | `p1_g7 §B1` | batas bawah yang sah (Lemma 3) |
| Predikat struktur `BLOCK`, lintasan A2.3, sertifikat A2.4 | `p1_g9 §B1`/`§B2`, W0–W1, dijalankan ulang di `p1_g10 §B1.1` | dipakai apa adanya |
| Gerbang sadar-tunggu `validate_coupled` | `p1_g10 §B2`, 17 mutasi 0 lolos | **wajib** dipakai untuk setiap jadwal |
| W2 (`verify_sched_coupled.Brute`) + P0–P4 | `p1_g10 §B9` | oracle, dipakai satu arah |
| `solve_coupled2` + dive | `p1_g10 §B9`, 7/8 palang | dipakai; lihat U1 |
| `irm_sweep.polyline_min_dist` / `seg_seg_dist` | `p1_g2 §10`, exact segment–segment (Ericson), diadu brute force sampai **2e-05 m** | **predikat jarak lengan, dipakai apa adanya** |
| `irm_sweep.base_pose` | `p1_g2 §1`, pinocchio 1e-16 | rantai URDF |
| `irm_sweep.canonical_table` (policy `manip`) | `p1_g2 §10`, biaya komit ~2.3 poin, sensitivitas policy ±0.23 | resolusi redundansi kanonik |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi:**
`sched.py`, `sched_heur.py`, `sched_coll.py`, `sched_coupled.py`,
`test/verify_sched_exact.py`, `test/verify_sched_coll.py`,
`test/verify_sched_coupled.py`, `test/gate_sched_coupled.py`,
`test/eval_sched_heur.py`, `test/eval_sched_coll.py`.
Aturan `p1_g10 §A0` berlaku utuh: **bug di berkas beku dilaporkan sebagai
temuan, diperbaiki di tempatnya, pembekuannya dinyatakan gugur**. G10 memakai
aturan itu tiga kali dan dua di antaranya bug yang mengubah kebenaran.

⚠️ `irm_sweep.py` **tidak** dibekukan — ia belum pernah punya gerbang seketat
`sched*`. Tapi setiap fungsinya yang dipakai di sini dikutip di atas beserta
verifikasinya, dan kalau salah satunya disunting, verifikasinya **ikut
dijalankan ulang** dan dilaporkan.

### A1. Yang dibangun sesi ini, dan yang TIDAK

Berurutan, dan urutannya wajib (`p1_state §7.1`):

1. **U1** — vonis atas W2b(ii) (A3-U1). **Argumen ditulis di §A, bukan setelah
   melihat hasil.**
2. **U2** — `gen_real_rotcrowded`, probe S2 yang benar-benar mengikat, lalu K3
   diukur ulang di atasnya. Ini membayar utang D10/D17 yang sudah menghindari
   penilaian **dua sesi**.
3. **Predikat lengan–lengan** (A2.1–A2.4) + buktinya (A3-K1).
4. **Pengukuran**: apakah ia mengikat, dan berapa.

**TIDAK dibangun, dan tidak boleh menyelinap masuk:** perencanaan gerak lengan,
`T_fold ≠ 0`, kapabilitas dinamis, tabrakan lengan–lingkungan, dan heuristik
baru apa pun sebelum A3-K2 lulus.

### A2. PENAMBAHAN MODEL — inilah bagian yang berbahaya, dikunci sekarang

#### A2.1 🔒 KONFIGURASI LENGAN sebagai fungsi keadaan jadwal

Model penjadwalan sampai G10 tidak pernah menyebut di mana lengan berada. Untuk
predikat lengan–lengan ia harus. Dikunci:

```
lengan a milik gantry g, pada waktu t:

  KANONIK   bila a sedang mengerjakan tugas i pada perhentian yang jendela
            dwell-nya memuat t
            -> q = canonical_table(a, target_i, pose_g), policy `manip`
  MENGGANTUNG  selain itu -- traverse, menunggu, menganggur, dan selama
            perhentian yang tugasnya TIDAK ditugaskan ke a
            -> q = pose istirahat, lurus ke bawah
```

🔒 **`MENGGANTUNG` bukan pilihan gaya; ia satu-satunya pose yang terukur benar
di perangkat keras nyata**, dan itu penting karena `p1_state §7.2` melarang
mengarang konstanta. Terukur 2026-08-13 pada `arm_1`:

| Pose | Radius sapuan dari sumbu rotasi | Torsi |
|---|---|---|
| **menggantung** | **0.52 m** | **≈ 0.00 N·m** (sejajar gravitasi) |
| tuck terdokumentasi `[0, 2.6, 2.6, 0, 0, 0]` | 0.55 m | **GAGAL** — joint_2 mencapai 12.78 dari 14 N·m, 0.41 rad ke dalam gerak 2.39 rad |
| terentang | **1.16 m** terukur | — |

Jadi tuck yang `initial_positions.yaml` sebut **tidak bisa dijalankan** dan juga
**lebih longgar** daripada menggantung. Memodelkan lengan sebagai "terlipat"
selama traverse akan salah dua kali sekaligus.

⚠️ **Konsekuensi yang tidak boleh dihaluskan:** aturan ini memperkenalkan
transisi **tarik-masuk** (sebelum traverse) dan **julurkan** (sesudah), dan
model masih menghargainya **nol** (`T_fold = 0.0`, `p1_g7 §A4.3`). Perlakuannya
sama persis dengan sebelumnya — disebut, tidak diukur, tidak diarang. Tapi
sekarang arahnya diketahui dan **asimetris**: memori perangkat keras mencatat
tarik-masuk dibantu gravitasi (murah) dan julurkan melawan gravitasi
(terbatas torsi). `p1_g2 §15b` mengasumsikan "lipat + traverse + buka lipat"
yang simetris, dan asumsi itu **salah**. Memperbaikinya adalah pekerjaan G12,
bukan sesi ini, dan ia dicatat di sini supaya tidak hilang.

#### A2.2 🔒 TIGA REZIM, dan angkanya sudah memberi tahu di mana risikonya

| Rezim | Radius lengan yang berlaku | vs `R_MAX` struktur = 0.455 m |
|---|---|---|
| traverse × traverse | 0.52 / 0.52 | **+0.065 m** |
| traverse × dwell | 0.52 / 1.16 | |
| **dwell × dwell** | **1.16 / 1.16** | **2.5×** |

🔒 **LEMMA A — traverse hampir seluruhnya sudah tercakup G9/G10.** Selama
traverse lengan menggantung pada radius 0.52 m, yaitu **6.5 cm** lebih besar
dari jejak struktur yang `p1_g9`/`p1_g10` sudah modelkan exact. Maka rezim
traverse × traverse tidak bisa jauh berbeda dari `BLOCK` yang sudah diukur, dan
**bisa didekati sebagai `BLOCK` dengan `c_clear` dinaikkan** — yang sapuan
`p1_g10 §B10` **sudah** ukur: pada `c_clear = 0.10` m vonisnya tetap
BUKAN MAHAL. ∎
🔴 Ini **dugaan yang harus diukur, bukan jalan pintas yang boleh dipakai**:
jejak lengan menggantung adalah **silinder di ujung pelat**, bukan pembesaran
seragam jejak batang, jadi kesetaraannya harus dicek, bukan diasumsikan (A3-K1
uji L2).

➜ **Maka taruhan sesi ini ada di `dwell × dwell`**, dan hanya di sana. Di sana
kedua lengan terentang, keduanya **statis**, dan pertanyaannya adalah pertanyaan
kelayakan bersama — yaitu pertanyaan yang `p1_g2 §10` **sudah** jawab sebagai
geometri (**4.19%** konfigurasi inter-gantry bertabrakan) tapi **belum pernah**
jawab sebagai **kendala penjadwalan**.

#### A2.3 🔒 PREDIKAT — dan batas atas aritmetik vs radius terukur

```
ARM_BLOCK( (p1, U1), (p2, U2) ; c_arm )  <=>
    min over a in arms(g1), b in arms(g2) of
        polyline_min_dist( W_a(p1, U1), W_b(p2, U2) )   <=  c_arm
```
dengan `W_a` = polyline dunia base→siku→pergelangan→tool pada konfigurasi yang
A2.1 tetapkan, dan `polyline_min_dist` **fungsi yang sudah ada dan sudah diadu
brute force sampai 2e-05 m** (`p1_g2 §10`).

🔺 **Dua angka beredar untuk "jangkauan lengan" dan keduanya BENAR — jangan
dirata-ratakan** (`Rule 7`):

* **1.4 m** (`p1_g2 §15c`, dan dikutip `p1_state §350`, `p1_g9`, `p1_g10`) adalah
  **batas aritmetik**: `0.4` offset pelat `+ 1.0` jangkauan lengan. Ia
  mengandaikan lengan menunjuk radial keluar pada rentangan penuh.
* **1.16 m** (terukur 2026-08-13) adalah **radius sapuan yang benar-benar
  dicapai** pada pose terentang.

🔴 Yang mengikat adalah **maksimum atas tabel kanonik yang benar-benar dipakai
instance**, dan itu **belum pernah dihitung**. G11 menghitungnya dan
melaporkannya sebagai angka **sebelum** memakai salah satunya. 1.4 dipakai hanya
sebagai prapenyaring yang sah (ia batas atas); 1.16 tidak boleh dipakai sebagai
prapenyaring sama sekali, karena ia bukan batas atas.

🔒 **`c_arm = 0.0` adalah DEFAULT TERKUNCI**, perlakuan yang sama persis dengan
`c_clear` (`p1_g9 §A2.2`) dan `T_fold`: margin tidak pernah diukur, jadi tidak
diarang. Sensitivitas dilaporkan sebagai sapuan `c_arm ∈ {0.00, 0.05, 0.10,
0.15, 0.20}` — lima nilai, bukan tiga, karena `p1_g2 §10` mengukur ambang
kualitatifnya ada di antara **0.15** (0.00% pasangan hilang) dan **0.20** m
(0.62%), dan sapuan yang melewatkan ambangnya sendiri tidak menjawab apa pun.

⚠️ **Bias yang diketahui dan arahnya:** proksi polyline **meremehkan** volume
sapuan (siku menonjol), jadi setiap angka rugi dari predikat ini tetap **batas
bawah** (`p1_g2 §10`, batasan terakhir). Menggemukkannya jadi kapsul menuntut
**radius baru** yang tidak pernah diukur; kalau G11 melakukannya, radius itu
disebut sebagai **konstanta yang diarang** dan dilaporkan terpisah, tidak
dicampur ke angka utama.

#### A2.4 🔒 APA YANG BERUBAH DI MODEL PENJADWALAN — sesedikit mungkin

`ARM_BLOCK` bergantung pada **himpunan tugas** di sebuah perhentian, bukan hanya
pose. Itu satu-satunya perbedaan struktural dari `BLOCK`, dan ia serius: pasangan
`(pose, pose)` yang aman untuk satu pasangan tugas bisa terlarang untuk pasangan
tugas lain. Konsekuensinya dikunci sekarang supaya tidak ditemukan lagi di tengah
implementasi:

1. **Tabel pra-hitung `BLOCK` (2376 × 2376) TIDAK punya analog** untuk
   `ARM_BLOCK` — ia akan berukuran `(2376 × 2^n)²`. Predikatnya karena itu
   dievaluasi **on demand**, dan `p1_g8 §B4` / `p1_g9 §B3` / `p1_g10 §B3`
   Pertentangan 1 (**tiga sesi berturut-turut**) semuanya bilang hal yang sama:
   **ukur pemeriksanya sebelum menjalankan sapuan apa pun**, dan beri setiap
   loop yang memanggilnya batas eksplisit.
2. **Kendala waktu kontinu A2.4 tetap berlaku apa adanya**, dengan `ARM_BLOCK`
   menggantikan `BLOCK` selama jendela dwell dan `BLOCK`-diperbesar selama
   traverse (A2.2 Lemma A, setelah diuji).
3. **Gerak menghindar tetap bagian model** (`p1_g9` Pertentangan 2), dan
   himpunan pose amannya sekarang **berbeda**: Lemma 1 dihitung untuk struktur
   (`|rot| < 31.7°`), dan untuk lengan menggantung radius 0.52 m ambangnya
   bergeser. **Lemma 1 versi lengan diturunkan ulang di §B, bukan diasumsikan
   selamat.**

### A3. Kriteria yang DIKUNCI

#### U1 — vonis atas W2b(ii). **Jalur (b), dan alasannya ditulis SEKARANG.**

`p1_g10 §B9` mencatat W2b(ii) (`no_dive`) gagal 6/12, jadi palang A3-K2 G10
lulus 7 dari 8 dan "ground truth" tidak diucapkan. §C G10 memberi dua jalur.
**Dipilih (b): kriterianya diubah, dengan argumen.** Argumennya:

1. **W2b(ii) mengukur konfigurasi yang tidak pernah dipakai.** `no_dive` bukan
   mode operasi; ia diagnostik. Menuntutnya lulus adalah menuntut jalur mati
   punya performa.
2. **W2 sudah menguji hal yang sama, lebih keras.** W2 mengadu solver **lengkap
   dengan dive** melawan enumerator yang tidak berbagi satu baris pun dengan
   pencariannya, pada 31 instance yang 15 di antaranya tabrakannya mengikat, dan
   menemukan **0 kali `solver > W2`**. W2b(ii) mematikan tabrakan sama sekali,
   jadi ia menguji **lebih sedikit**.
3. **Alasan asli W2b dijalankan dua kali tetap dihormati.** `p1_g10 §A2.3`
   menuntutnya karena dengan tabrakan mati, dive di akar menjawab langsung dan
   mesin ekspansinya tidak pernah jalan — gerbang-yang-tidak-menyala. Tapi
   **W2 menyalakan mesin itu**: 10 dari 40 instance S1 dan 15 instance W2 yang
   mengikat semuanya melewati B&B. Kekhawatirannya sudah terjawab oleh uji lain.

🔒 **PALANG YANG MENGGANTIKANNYA, dikunci sekarang:** W2b dijalankan **sekali**,
dengan dive, 12/12 sampai 1e-9 — **dan** W2 wajib melaporkan **≥ 10 instance
yang tabrakannya mengikat** dan **0 kali `solver > W2`**. Baris `no_dive` tetap
dijalankan dan **dilaporkan sebagai angka diagnostik**, tidak sebagai palang.

🔴 Kalau §B menemukan alasan jalur (b) salah, itu ditulis sebagai pertentangan,
dan palangnya kembali ke 8 baris.

#### U2 — `gen_real_rotcrowded`, dan kenapa bentuk lamanya gagal

`p1_g10 §B7`: `gen_real_crowded` memadatkan **`lin`**, dan (N1) menuntut
**`rot`**. 40/40 instance S2 kembali `Δ = 0` lewat Lemma 4 dan solver
terkopelnya **tidak pernah jalan**. Bentuk yang benar sudah terbukti mengikat di
`verify_sched_coupled.gen_small_crowded`. Dikunci untuk peta nyata:

```
gen_real_rotcrowded(n, seed, n_mr, gantries):
  p0 kedua gantry = pose grid terdekat ke (lin = 0.80, rot = 0)   <- aman, Lemma 1
  kolam simpul    = simpul yang himpunan pose-layaknya DIDOMINASI oleh
                    |sin rot| >= 0.525  (yaitu (N1) terpenuhi di sana)
  ditolak         kalau tugasnya layak di pose mana pun dengan |rot| < 31.7 deg
```

Yaitu: probe memilih **tugas yang memaksa rotasi besar**, bukan tugas yang
kebetulan berdekatan di `x`. Ia tetap **rancangan probe**, bukan konstanta
fisik, dan **tidak pernah** dirata-ratakan ke S1.

🔒 **Gerbang probe, supaya kegagalan G10 tidak terulang senyap:** kalau
`gen_real_rotcrowded` menghasilkan **< 8 dari 40** instance yang `Δ > 0` atau
route-nya bukan `lemma4`, probe itu dinyatakan **GAGAL BERFUNGSI** dan
dilaporkan begitu, bukan dilaporkan sebagai "tidak mengikat".

#### K1 — BAGAIMANA PREDIKAT LENGAN DIBUKTIKAN

Setara W0 `p1_g9`. Semua wajib lulus.

| | Apa yang diadu | Independen dalam hal apa |
|---|---|---|
| **L0** | `polyline_min_dist` vs oracle sampling titik pada kedua polyline | nol rumus jarak segmen |
| **L1** | polyline dunia vs `irm_sweep.base_pose` + rantai FK, pada 2 000 `(pose, target)` acak | mengikat predikat ke rantai yang sudah dicek pinocchio 1e-16 |
| **L2** | **Lemma A**: jejak lengan menggantung vs `BLOCK` dengan `c_clear` naik | menguji jalan pintas A2.2, bukan mengasumsikannya |
| **L3** | reproduksi `p1_g2 §10`: intra-gantry **1.27%** konfigurasi, **0.00%** pasangan hilang ≤ 0.15 m, **0.62%** pada 0.20 m | angka yang sudah terbit; kalau tidak reproduksi, ada yang berubah dan itu temuan |
| **L4** | `ARM_BLOCK` **tidak vacuous**: ada `(p1, U1, p2, U2)` yang terlarang, dengan contoh tertutup | P4-nya sesi ini — kalau nol, seluruh §B kosong |
| **L5** | radius sapuan maksimum atas tabel kanonik, diadu dengan batas aritmetik 1.4 m | menyelesaikan konflik A2.3 dengan pengukuran |

🔴 **`L4` adalah yang paling mungkin gagal, dan kalau ia gagal itu TEMUAN, bukan
kegagalan sesi:** `p1_g2 §10` sudah mengukur 0.00% pasangan target hilang sampai
clearance 0.15 m. Kalau `ARM_BLOCK` ternyata tidak pernah melarang kombinasi
`(pose, tugas)` yang optimum benar-benar pakai, maka **lubang model terakhir
ternyata bukan lubang**, dan itu kalimat yang naskah butuhkan — asal ia
**diukur**, bukan diasumsikan.

#### K2 — PALANG, SEBAGAI ANGKA

| | Palang |
|---|---|
| L0–L5 | semua lulus |
| U1 | W2b(i) 12/12; W2 ≥ 10 instance mengikat, 0 kali `solver > W2` |
| U2 | probe lulus gerbang probe (≥ 8/40 mengikat) **atau** dilaporkan GAGAL BERFUNGSI |
| Gerbang | **100%** jadwal yang dilaporkan lolos `validate_coupled`, 0 pelanggaran |
| Regresi | `n4_s2_mr1`, `n4_s9_mr1` tetap `Δ = 0.000`; S3 tetap 20/20 nol |
| S1 | vonis A3-K3 tetap dapat ditentukan di bawah `ARM_BLOCK` |

#### K3 — VONIS, aturan dua-sisi `p1_g10 §A3-K3` DIPAKAI ULANG APA ADANYA

Ambang **5.0%** tidak digeser. `Δ_lo = 0` untuk kurungan, `Δ_hi = UB − LB`,
mean atas **seluruh** instance pada kedua ekstrem, vonis sah jika keduanya
jatuh di sisi yang sama dari 5.0%. Aturan itu **bekerja** di G10 — ia mengubah
"TIDAK DAPAT DITENTUKAN" jadi vonis sah tanpa membuang satu instance pun.

Yang **baru** dan wajib dilaporkan terpisah:

```
Delta_struct  = terkopel(BLOCK saja)            - takterkopel     [G10: <= 1.01%]
Delta_full    = terkopel(BLOCK dan ARM_BLOCK)   - takterkopel
Delta_arm     = Delta_full - Delta_struct       <- SUMBANGAN LENGAN, angka sesi ini
```

#### K4 — YANG DILAPORKAN, TERLEPAS DARI HASILNYA

1. `Δ_struct`, `Δ_full`, `Δ_arm` per set, dengan vonis dua-sisi. S1 dan S2
   terpisah.
2. Sapuan `c_arm ∈ {0.00, 0.05, 0.10, 0.15, 0.20}`.
3. Fraksi kombinasi `(p1, U1, p2, U2)` yang `ARM_BLOCK` pada instance nyata —
   jalur data, analog K5.4 G10.
4. **Biaya pemeriksa**: detik per evaluasi `ARM_BLOCK`, dan berapa kali
   dipanggil. Diukur **sebelum** sapuan apa pun (A2.4 poin 1).
5. Radius sapuan maksimum terukur (L5), dan Lemma 1 versi lengan.
6. Setiap anggaran yang tersentuh, seperti `p1_g10 §K5.8`.
7. Jumlah jadwal yang gagal gerbang, per penjadwal.

### A4. Yang TIDAK dimodelkan — setelah sesi ini

1. **Gerak lengan tetap tidak direncanakan.** Predikat ini mengevaluasi
   konfigurasi **kanonik dan menggantung**, bukan lintasan di antaranya. Transisi
   tarik-masuk/julurkan tetap berbiaya **0**.
2. `T_fold = 0.0`, dan `p1_g2 §15b` yang simetris **sudah diketahui salah**
   (A2.1).
3. Proksi polyline **meremehkan** volume sapuan → angka rugi tetap batas bawah.
4. Kanonik = `manip`; policy lain tidak disapu.
5. Exact tetap terhadap grid 33 × 72 dan terhadap himpunan kandidat waktu mulai
   `p1_g10 §A2.2`.
6. Tabrakan lengan–**lingkungan** dan lengan–struktur (lengan sendiri vs batang
   gantry lawan) **tidak** dimodelkan sesi ini.

### A5. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor: **19 meleset, 6 tepat** (`p1_g10 §B11`). Prior dunia: **LONGGAR**
(tanpa contoh tandingan setelah D9 dibalik). Prior kode sendiri: **lebih lambat,
lebih rumit, lebih salah**, 4 dari 4, dan ditulis di sisi pesimis supaya bisa
dinilai.

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D20** | `Δ_arm` pada S1 **≤ 1.0%** dan vonis gabungan tetap BUKAN MAHAL | dunia | `p1_g2 §10` mengukur 0.00% pasangan target hilang ≤ 0.15 m, dan geser-π membuat lengan se-gantry justru jarang bertemu |
| **D21** | `ARM_BLOCK` **TIDAK vacuous** (L4 lulus): ada kombinasi terlarang, tapi optimumnya selalu punya jalan keluar | dunia | analog persis mutex `p1_g7 §B3` Q1/Q2: mengikat secara lokal (10.9%), berbiaya nol |
| **D22** | Pemeriksa `ARM_BLOCK` **≥ 100× lebih mahal** dari `BLOCK` per evaluasi | **kode sendiri** | `BLOCK` = 9 pasangan primitif tervektorisasi; `ARM_BLOCK` = 4 pasangan lengan × 3×3 segmen × tabel kanonik yang harus dicari. Ditulis pesimis dengan sengaja |
| **D23** | `gen_real_rotcrowded` **lulus** gerbang probe (≥ 8/40 mengikat) | **kode sendiri** | `gen_small_crowded` sudah mengikat pada 15/31; tapi peta nyata punya 2376 pose dan jauh lebih banyak jalan keluar, jadi angkanya ditahan di 8, bukan 40 |
| **D24** | Rezim **traverse × traverse** tidak menambah apa pun di atas `BLOCK` dengan `c_clear = 0.10` (Lemma A lulus L2) | dunia | selisih radiusnya 6.5 cm dan `p1_g10 §B10` sudah mengukur `c_clear = 0.10` tetap BUKAN MAHAL |

### A6. Berkas

| Berkas | Isi |
|---|---|
| `reachability_gng/sched_arm.py` | konfigurasi A2.1, `ARM_BLOCK`, Lemma 1 versi lengan, `gen_real_rotcrowded` |
| `test/verify_sched_arm.py` | L0–L5 |
| `test/eval_sched_arm.py` | K3/K4, sapuan `c_arm`, `Δ_arm` |
| `docs/p1_g11_arm.md` | dokumen ini |

### A7. Urutan kerja, dan apa yang terjadi kalau waktu habis

```
1. U1 (vonis, sudah ditulis di A3-U1) + U2 (probe) + K3 ulang pada S2 benar
2. BIAYA PEMERIKSA ARM_BLOCK diukur       <- SEBELUM sapuan apa pun
3. predikat + L0-L5                        <- GERBANG: tidak lanjut kalau gagal
4. Delta_arm pada S1 dan S2 + K4
5. sapuan c_arm
```

🔴 **Apa pun yang tidak tercapai dilaporkan sebagai TIDAK DIUKUR, sebagai
kalimat eksplisit di §B.** Dilarang menuliskan "diperkirakan tidak berubah"
untuk sesuatu yang tidak dijalankan.

---

## B. Hasil terukur

> §A dikunci 2026-08-14 sebelum `sched_arm.py` ada. Semua angka di bawah keluar
> sesudahnya.
>
> ⏳ **Belum dimulai.**
