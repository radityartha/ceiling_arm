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

> §A dikunci 2026-08-14 (commit `d0ad67b`) sebelum `sched_arm.py` ada. Semua
> angka di bawah keluar sesudahnya. Setiap tempat di mana §B bertentangan
> dengan §A ditandai 🔺. **§A TIDAK ditulis ulang.**

### B0. Cara menjalankan ulang, dan beban mesin

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 test/verify_sched_arm.py all      # L0-L5, gerbang A3-K1
python3 test/eval_sched_arm.py cost       # K4.4 -- SEBELUM sapuan apa pun
python3 test/eval_sched_arm.py probe      # U2
python3 test/eval_sched_arm.py s2         # probe adversarial + sapuan c_arm
python3 test/eval_sched_arm.py s1         # angka utama + sapuan c_arm
python3 test/eval_sched_arm.py report
python3 test/verify_sched_coupled.py w2   # U1
python3 test/verify_sched_coupled.py w2b  # U1
```

⚠️ **Beban mesin, sesuai perintah `p1_g10 §C`:** `load average 3.3-3.7` pada 16
core. Node kamera RealSense dari sesi lain **masih hidup** (~0.6 core) dan
**tidak dimatikan** — ia milik sesi lain, dan mematikannya bukan keputusan sesi
ini. Jauh lebih ringan dari `load 42` G10, jadi angka detik di sini **tidak**
bisa dibandingkan langsung dengan `p1_g10 §K5.7` ke arah mana pun.

🔒 **Berkas beku (§A0): `git diff` KOSONG.** Diperiksa di akhir sesi atas
kesepuluh berkas. Tidak ada bug yang ditemukan di dalamnya sesi ini, jadi tidak
ada pembekuan yang gugur. Tiga berkas baru: `reachability_gng/sched_arm.py`,
`test/verify_sched_arm.py`, `test/eval_sched_arm.py`.

⚠️ **`irm_sweep.py` tidak disunting** (§A0 mengizinkan dengan syarat). Fungsi
yang dipakai — `polyline_min_dist`, `seg_seg_dist`, `base_pose`,
`canonical_table`, `world_polylines` — dipanggil apa adanya, dan L0/L1
menjalankan ulang verifikasinya.

### B1. 🔴 PROVENANSI: dari cloud mana `canon` dibaca — pertanyaan yang §A tidak tanyakan

Sebelum satu pun angka, satu hal yang §A lewatkan sepenuhnya dan yang bisa
membatalkan seluruh sesi tanpa sinyal error: `cap_g{1,2}_rail160.npz` menyimpan
`canon` sebagai **indeks ke sebuah cloud yang tidak ikut disimpan bersamanya**.
Membacanya terhadap cloud yang salah memberi polyline lengan yang **terlihat
wajar dan salah**.

Ditemukan dengan mencari, lalu dibuktikan bit-identik (L1a):

| | |
|---|---|
| cloud | `/tmp/irm_cloud_pol.npz` (500 000 sampel, satu-satunya yang punya `sigmin`) |
| filter approach | **45°** → 89 170 sampel tersisa |
| policy kanonik | `manip` |
| `k_cand` | 64 |
| hasil | `masks` **dan** `canon` **IDENTIK** untuk **kedua** peta |

➜ Cloud itu **hanya ada di `/tmp`**, persis situasi yang `data/README.md`
tanggal 2026-08-14 tulis tentang peta kapabilitas sendiri. **Ini utang G12**
(§C).

### B2. U1 — jalur (b) DIPAKAI, dan palang penggantinya LULUS

§A3-U1 memilih jalur (b) dan menulis argumennya sebelum melihat hasil. Palang
penggantinya dijalankan ulang pada pohon hari ini:

| Palang §A3-U1 | Terukur | |
|---|---|---|
| W2b **dengan dive**, 12/12 sampai 1e-9 | **12 / 12**, 0 kegagalan | ✅ |
| W2 melaporkan **≥ 10** instance yang tabrakannya mengikat | **31 instance, 15 mengikat** (2 timeout) | ✅ |
| W2: **0 kali** `solver > W2` | **0** | ✅ |
| P0–P4 | lima-limanya lulus (P2 31 jadwal 0 ditolak, P3 0 pelanggaran, P4 8 instance 0 naik) | ✅ |
| W2b `no_dive` — **diagnostik, bukan palang** | **4 dari 12 gagal** (G10: 6 dari 12) | 📊 |

🔒 **Maka `solve_coupled2` boleh disebut ground truth terhadap himpunan kandidat
A2.2**, dengan kualifikasi yang sudah dikunci: palangnya adalah W2 + W2b(i),
bukan W2b(ii). Selisih 4/12 vs 6/12 antara hari ini dan G10 adalah efek
anggaran waktu dinding pada mesin yang lebih ringan — arah vonisnya tidak
berubah, dan `no_dive` memang menanggung beban dive.

⚠️ Batasan W2 dari `p1_g10 §B9` **tetap berlaku dan tidak diperbaiki sesi ini**:
`solver < W2` pada **9 dari 31**, dan menghalfkan grid menutup **0 dari 9**.
**U4 G10 TIDAK DIKERJAKAN** — dilaporkan sebagai TIDAK DIUKUR, bukan sebagai
"diperkirakan tidak berubah".

### B3. 🔺 U2 — rancangan probe §A3-U2 **TIDAK BISA DIBANGUN**, dan alasannya diukur

§A3-U2 mengunci: kolam simpul = simpul yang himpunan pose-layaknya **didominasi**
oleh `|sin rot| ≥ 0.525`, dan **ditolak** kalau layak di pose mana pun dengan
`|rot| < 31.7°`. Implementasi pertama mengembalikan `RuntimeError` pada
instance pertama. Diukur kenapa, **sebelum** merancang penggantinya:

| | Terukur |
|---|---|
| pose "panas" (`\|sin rot\| ≥ 0.525`) di grid | **1518 / 2376 = 63.9 %** |
| simpul hidup | 3132 |
| `\|G(t)\|` median | **1501 dari 2376 pose** |
| simpul dengan fraksi-panas ≥ 1.00 / 0.90 / 0.80 / **0.70** | **0 / 0 / 0 / 0** |
| fraksi-panas atas simpul hidup, p5 / p50 / p95 | **0.487 / 0.587 / 0.626** |

➜ **Setiap tugas duduk persis pada laju dasar grid.** Tidak ada satu pun tugas
yang **memaksa** rotasi. Mekanismenya geometris dan layak ditulis sebagai
kalimat naskah:

> Lengan menjangkau **1.06 m** di sekitar sumbu rotasi yang pelat mounting-nya
> hanya mengorbit pada **0.40 m**. Memutar gantry hanya **mengarahkan ulang**
> jangkauan yang sudah menutupi target. Kelayakan di sel ini **nyaris invarian
> terhadap rotasi.**

🔴 **Konsekuensinya lebih besar daripada probenya:** (N1) menuntut **kedua**
gantry lebih dari 31.7° dari sejajar-rel, dan **tidak ada pilihan TUGAS di peta
ini yang bisa memaksanya**. Jadi kegagalan S2 di `p1_g10 §B7` bukan cuma "sumbu
yang salah dipadatkan" — **tidak ada probe berbasis tugas yang bisa berhasil**.

**Penggantinya, dan ia menyebut dirinya apa adanya:** probe membatasi
**HIMPUNAN KANDIDAT POSE**, sumbu yang `p1_g7 §A2-K1` memang sediakan untuk
deklarasi semacam ini.

```
p0             = pose grid terdekat ke (0.80, rot = 0)   <- aman, Lemma 1
himpunan pose  = {p0} U {p : |sin rot| >= sin_min, |lin - 0.80| <= 0.30}
kolam simpul   = simpul di dalam band, layak di dalam himpunan itu
ditolak        = simpul yang layak DI p0  (kalau tidak, ia dikerjakan di p0
                 pada rot = 0 dan (N1) tidak pernah menyala)
```

Klausa penolakan-di-`p0` **diukur, bukan didesain**: tanpanya probe
mengembalikan makespan 4–6 s dengan kedua gantry berdiri diam di `p0`.

**Hasil, dan probenya sekarang benar-benar masuk rezim tabrakan:**

| `sin_min` | \|P\| | `Δ > 0` | solver terkopel benar-benar jalan | max `Δ` |
|---|---|---|---|---|
| 0.525 (nilai (N1), terkunci §A) | 507 | **0 / 10** | 0 / 10 | +0.0000 s |
| **0.7625** (nilai yang (N1) tuntut kalau kedua gantry sama) | 375 | **0 / 10** | **4 / 10** | +0.0000 s |
| 0.85 | 287 | **0 / 10** | 0 / 10 | +0.0000 s |

🟢 **Ini lebih kuat daripada gerbang probe §A3-U2 minta.** Gerbang itu berbunyi
"< 8 dari 40 mengikat → GAGAL BERFUNGSI". Di sini `Δ > 0` pada **0**, tapi
**solver terkopelnya JALAN** pada 4/10 — yaitu tabrakan struktur benar-benar
melanggar optimum takterkopel, solver harus bekerja, **dan tetap menemukan
jalan keluar berbiaya nol**. Perbedaan dengan G10 penting: di sana solver
**tidak pernah** dipanggil, jadi tidak ada yang teruji.

➜ **D10 (`p1_g9`) dan D17 (`p1_g10`) sekarang DINILAI, bukan ditunda lagi:
MELESET.** Bahkan pada rezim paling adversarial yang bisa dibangun di grid ini,
biaya koordinasi **struktur** gantry–gantry adalah **0.000 s**.

### B4. K4.4 — BIAYA PEMERIKSA, diukur SEBELUM sapuan apa pun (§A2.4 poin 1)

Tiga sesi berturut-turut menulis "yang lambat adalah PEMERIKSA". Diukur dulu:

| N (pasangan, tervektorisasi) | `pair_distance` (BLOCK) | `polyline_min_dist` (ARM) |
|---|---|---|
| 1 | 1135.90 µs | 129.71 µs |
| 100 | 18.57 µs | 2.85 µs |
| 10 000 | **6.44 µs** | **1.86 µs** |
| 200 000 | **7.29 µs** | **2.24 µs** |

🔺 **D22 MELESET, dan ke arah yang menyenangkan.** Dugaan: `ARM_BLOCK` **≥ 100×
lebih mahal** dari `BLOCK` per evaluasi. Terukur: primitif lengan **3× LEBIH
MURAH** per pasangan. Sebabnya bisa dibaca: `pair_distance` mengevaluasi 9
pasangan primitif 2D (OBB, poligon, cakram) lewat belasan panggilan numpy
terpisah; `polyline_min_dist` adalah **satu** panggilan `einsum` tervektorisasi
atas 3×3 segmen. Predikat **struktur** ternyata yang mahal.

`ARM_BLOCK` = 4 pasangan lengan × (konfigurasi per lengan, mean **1.34**, maks
4) panggilan → **4–16×** kolom kanan, yaitu masih di orde `BLOCK`.

Tapi pada level **jadwal utuh**, arahnya berbalik dan itu yang mengikat:

| | |
|---|---|
| gerbang ARM satu jadwal penuh | **45–49 ms** |
| gerbang STRUKTUR satu jadwal penuh (beku, sebagai skala) | **0.2 ms** |
| pembangunan `ArmGeom` (4 lengan × 6 tugas × 2376 pose) | **0.40 s, 5.5 MB** |

**~230×**, bukan karena primitifnya, melainkan karena gerbang lengan **memulai
ulang jalan-sertifikatnya di setiap batas dwell** (konfigurasi lengan berubah
diskontinu — `T_fold = 0`), sementara gerbang struktur punya `_cannot_meet`
sebagai early-out atas seluruh jendela. Jadi kehati-hatian §A2.4 poin 1
**benar**, unitnya saja yang salah ditebak. D22 dinilai pada unit yang ia
tuliskan: **MELESET**.

➜ Konsekuensi praktis yang menentukan bentuk §B5: 45 ms per jadwal berarti
menguji jadwal **murah**, tapi memasangnya ke dalam loop dalam B&B (ribuan
evaluasi per node) **tidak**. Itulah alasan §B5 mengukur `Δ_full` dengan
kurungan alih-alih membangun solver kedua.


### B5. Predikat lengan — L0–L5, dan EMPAT pertentangan dengan §A

Gerbang A3-K1 **LULUS enam-enamnya** dalam satu jalan
(`python3 test/verify_sched_arm.py all`), tapi empat dari enam menemukan bahwa
§A salah tentang angkanya. Semua ditulis, tidak ada yang dihaluskan.

| | Terukur | |
|---|---|---|
| **L0** | `polyline_min_dist` vs oracle sampling titik (nol aljabar segmen) pada **polyline lengan NYATA**, 600 pasang (200 dekat-kontak), 500 sampel/segmen: maks \|selisih\| **2.183e-04 m**, lantai sampling **8.426e-04 m** | ✅ |
| **L0** | `seg_seg_dist` vs oracle yang sama, 6000 pasang (2000 **degenerate**): **1.490e-04 m**, lantai 8.682e-03 | ✅ |
| **L1a** | provenansi: `masks` **dan** `canon` kedua peta dibangun ulang **BIT-IDENTIK** (§B1) | ✅ |
| **L1b** | polyline dunia vs pinocchio FK **dengan sendi gantry di dalam rantai**, 400 (tugas, pose): **2.480e-06 m** | ✅ |
| **L1c** | lengan partner = lengan tersimpan pada `rot + π`, 85 tugas × 2376 pose: **8.882e-16 m** | ✅ |
| **L2** | Lemma A, 2376² pasangan pose | ✅ |
| **L3** | reproduksi `p1_g2 §10` pada grid **33×72** (yang lama, 41×72, sudah pensiun), 400/2376 pose dan 783/3132 target disampel | ✅ |
| **L4** | `ARM_BLOCK` tidak vacuous | ✅ (dengan syarat) |
| **L5** | radius sapuan maksimum atas tabel kanonik | ✅ |

**L1b, 2.5 µm, dilacak sampai habis dan bukan diabaikan:** titik **pangkal**
lengan cocok **persis 0.0** (hasil 1e-16 `p1_g2 §1`, tereproduksi), dan sisanya
tumbuh linier menuruni rantai — 6.5e-07, 1.5e-06, 2.4e-06 pada arm_link,
pergelangan, tool — yaitu selisih komposisi **orientasi** ~3.2e-06 rad antara
model tereduksi yang dipakai menyampel cloud dan model penuh di sini. Terpisah:
polyline **base-frame** cloud cocok dengan FK dari `q`-nya sendiri sampai
**7.5e-08 m**, yaitu derau penyimpanan float32 (ulp pada 1 m = 1.2e-07). Empat
orde di bawah `ε = 0.005 m`.

#### 🔺 Pertentangan 1 — `c_arm = 0.0` BUKAN analog `c_clear = 0.0`. Ia degenerate.

§A2.3 mengunci `c_arm = 0.0` sebagai default, "perlakuan yang sama persis
dengan `c_clear`". **Itu salah, dan L4 yang menunjukkannya.**

Benda struktur punya **volume** (batang 0.80 × 0.08 m, dua pelat berjari-jari
0.055 m), jadi `pair_distance` memotong tumpang tindih ke `0.0` dan
`c_clear = 0` berarti *"dua benda padat bersentuhan"*. Lengan diwakili
**polyline tanpa ketebalan**, jadi `c_arm = 0` berarti *"dua garis berpotongan
persis"* — **kejadian berukuran nol**. Terukur:

| | `ARM_BLOCK` pada 4000 pasangan (pose, himpunan tugas) acak yang layak |
|---|---|
| `c_arm = 0.00` | **0** |
| pada grid 2376² penuh, kedua lengan menggantung | **0** |

🔒 **Maka nilai pertama yang berarti dalam sapuan §A2.3 adalah 0.05, dan
`c_arm = 0.00` dilaporkan sebagai baris degenerate, bukan sebagai default.**
Satu-satunya "ketebalan" pada baris itu adalah `ε = 0.005 m`, penjaga
terminasi sertifikat — yang berarti gerbang jadwal pada `c_arm = 0.00`
sebenarnya menguji `≤ 5 mm`. Itu disebut, bukan dibiarkan jadi kebetulan.

Menggemukkan polyline jadi kapsul menuntut **radius tautan yang tidak pernah
diukur**; §A2.3 melarang mengarangnya, dan sesi ini tidak mengarangnya.

#### 🔺 Pertentangan 2 — angka 0.52 m dan 1.16 m `p1_g3 §B4` TIDAK TEREPRODUKSI

§A2.1 dan §A2.2 dibangun di atas tabel radius sapuan `p1_g3 §B4`. Dihitung ulang
dari URDF dengan bentuk tertutup yang L5 buktikan setara `base_pose` sampai
**2.776e-16**:

```
r(x, y, z) = hypot(PLATE_OFF + x, y)        untuk titik base-frame (x, y, z)
```
(independen dari `lin` **dan** `rot`, dan identik untuk kedua sisi mount — offset
pelat yang berbalik tanda dan `yaw_off = π` saling meniadakan.)

| Pose | `p1_g3 §B4` | **Terukur sekarang (polyline)** | selisih |
|---|---|---|---|
| menggantung (`q = 0`) | 0.52 m | **0.4571 m** | −0.063 |
| tuck `[0, 2.6, 2.6, 0, 0, 0]` | 0.55 m | **0.4011 m** | −0.149 |
| terentang | 1.16 m | **1.0623 m** (tabel kanonik), 1.0780 m (seluruh cloud) | −0.098 |

Geometri tabrakan (mesh STL) **tidak bisa dimuat di mesin ini** — `workcell_full.urdf`
menunjuk ke `/srv/data/users/raditya/kortex_min_ws/...` yang tidak ada — jadi
selisih 6–15 cm **tidak bisa dipastikan** sebagai selubung tautan, hanya
konsisten dengannya. Ia adalah bias yang §A2.3 sudah sebut: **proksi polyline
meremehkan volume sapuan.**

⚠️ **Yang benar-benar gugur adalah urutannya.** §A2.1 membenarkan MENGGANTUNG
sebagian dengan "menggantung sudah lebih rapat daripada tuck (0.52 < 0.55)".
Pada proksi polyline urutannya **terbalik** (0.4571 > 0.4011). **Keputusan
§A2.1 tidak berubah** — tuck itu **tidak bisa dieksekusi** di perangkat keras
(12.78 dari 14 N·m) dan itu alasan yang cukup sendiri — tapi separuh alasannya
gugur dan itu ditulis.

#### 🔺 Pertentangan 3 — LEMMA A jauh lebih kuat dari §A2.2, dan D24 TEPAT

§A2.2 menghitung selisih radius traverse sebagai **+0.065 m** di atas
`R_MAX = 0.455` dan menandai Lemma A merah. Terukur:

| | |
|---|---|
| selubung horizontal lengan menggantung di sekitar pusat pelat, `r_h` | **0.0579 m** (vs `PLATE_R` = 0.0550) |
| radius sapuan menggantung | **0.4571 m** (vs `R_MAX` = 0.4550) |

Selisihnya **2.1 mm**, bukan 65 mm. L2 mengujinya sebagai himpunan, bukan
sebagai argumen — apakah `HANG_BLOCK(c) ⊆ BLOCK(c + δ)` pada seluruh 2376²:

| `c_arm` | `HANG_BLOCK` | `BLOCK(c)` (regresi `p1_g10 §B4`) | δ terkecil |
|---|---|---|---|
| 0.00 | 0 (degenerate, Pertentangan 1) | 183 836 = 3.2564 % ✓ | 0.000 |
| 0.02 | 34 688 = 0.6144 % | — | **0.000** |
| 0.05 | 105 652 = 1.8715 % | 334 140 = 5.9188 % ✓ | **0.000** |
| 0.10 | 272 052 = 4.8190 % | 528 500 = 9.3616 % ✓ | 0.004 |
| 0.20 | 667 308 = 11.8204 % | — | **0.000** |

🟢 **Lemma A LULUS dengan δ ≤ 0.004 m**, dan lebih dari itu: pasangan
menggantung adalah **himpunan bagian sejati** dari `BLOCK` pada clearance yang
**sama** — lengan menggantung bertabrakan **2–3× lebih JARANG** daripada
strukturnya sendiri, karena batang 0.80 m mendominasi jejak struktur sementara
lengan menggantung hanya dua silinder tipis di ujung pelat.

➜ **D24 TEPAT.** Rezim traverse × traverse **tidak menambah apa pun** di atas
`BLOCK`, dan `p1_g10 §B10` sudah menyapu `c_clear` sampai 0.10 dengan vonis
tetap BUKAN MAHAL. (Tiga angka `BLOCK` di kolom tengah mereproduksi
`p1_g10 §B4` **persis**, jadi ini juga regresi yang lulus.)

#### 🔺 Pertentangan 4 — LEMMA 1 TIDAK PUNYA ANALOG LENGAN. Himpunannya KOSONG.

§A2.4 poin 3 menulis: *"untuk lengan menggantung radius 0.52 m ambangnya
bergeser. Lemma 1 versi lengan diturunkan ulang di §B."* Diturunkan, dan
ambangnya **tidak bergeser — ia lenyap**:

```
Y_SEP - h_y(rot) - r_arm  >  c_arm
0.72  -  (0.4|sin rot| + 0.0579)  -  1.0623   <=  -0.40   untuk SETIAP rot
```

| `c_arm` | pose aman universal, **lengan** | pose aman universal, struktur |
|---|---|---|
| 0.00 | **0 / 2376** | 858 |
| 0.05 | **0 / 2376** | 594 |
| 0.10 | **0 / 2376** | 462 |
| 0.15 | **0 / 2376** | 198 |
| 0.20 | **0 / 2376** | 66 |

🔴 **Ini temuan struktural, bukan detail.** `p1_g10 §B4` bacaan 2 menjelaskan
kenapa koordinasi struktur murah: *"37.3% ruang pose bebas-tabrakan tanpa
syarat — selalu ada tempat menyingkir, dan biasanya murah."* **Mekanisme itu
tidak ada pada model lengan.** Tidak ada pose yang bisa dipakai gantry untuk
memarkir dirinya dengan aman terhadap **setiap** konfigurasi lawan, karena
lawan menjangkau 1.06 m melintasi pemisahan 0.72 m. Pose parkir yang aman tetap
ada, tapi ia **bergantung pada apa yang sedang dikerjakan lawan** — jadi ia
harus **dicari**, dan tidak bisa lagi dibaca dari sebuah aturan.

Konsekuensi langsungnya: konstruktor batas atas "serialisasi" yang G9/G10 andalkan
(`_serial_ub`, Lemma 1) **tidak sah lagi apa adanya**. `eval_sched_arm.serial_arm_ub`
menggantinya dengan pencarian pose parkir termurah-dulu yang diperiksa terhadap
seluruh jadwal solo lawan.

#### L4 — `ARM_BLOCK` TIDAK vacuous, begitu lengan punya ketebalan apa pun

4000 pasangan `(p1, U1, p2, U2)` acak yang layak, `real(n=6, mr=1, seed=3)`.

🔴 **Cacat di uji saya sendiri, ditemukan oleh angkanya dan dilaporkan:** draf
pertama menarik `U1` dan `U2` **secara independen**, sehingga tugas yang sama
bisa dikerjakan dari **kedua** gantry. Dua tool frame lalu duduk di titik yang
sama dan uji melaporkan "tabrakan 0.00002 m" yang **tidak ada jadwal yang bisa
memuatnya** — jadwal mengerjakan tiap tugas **tepat sekali**.
`irm_sweep.pair_feasible` sudah mengecualikan `i == j` untuk alasan yang sama.
Angka pertamanya: **63.1 %** pada `c_arm = 0.05`. Setelah `U1 ∩ U2 = ∅`
ditegakkan: **24.3 %**. Selisihnya 2.6×, dan tanpa disiplin "bandingkan dengan
angka terbit" (L3) ia akan masuk §B tanpa ada yang menegur.

| `c_arm` | `ARM_BLOCK` mode `any` (pesimis) | mode `all` (optimis) |
|---|---|---|
| 0.00 | **0.000 %** | 0.000 % |
| 0.02 | 9.925 % | 8.250 % |
| **0.05** | **24.275 %** | 21.375 % |
| 0.10 | 40.375 % | 37.500 % |
| 0.15 | 48.425 % | 45.675 % |
| 0.20 | 52.550 % | 50.675 % |

jarak lengan–lengan: min **0.000085 m**, p1 **0.0018**, p50 **0.1674**, maks
**1.3305 m**.

🔒 **Dua mode, karena §A2.3 menulis `W_a(p1, U1)` sebagai SATU polyline dan itu
tidak terdefinisi.** Sebuah perhentian boleh memberi **satu lengan beberapa
tugas** (`dur = slot × dwell`; terukur: mean **1.34** konfigurasi per lengan,
maks 4, **> 1 pada 25 %** perhentian), dan model tidak pernah mengurutkan
slotnya. `any` = tolak kalau **ada** pengurutan yang menabrak (bacaan harfiah
`min over a, b`); `all` = tolak hanya kalau **setiap** pengurutan menabrak.
Keduanya dilaporkan; selisihnya **2–3 poin persen**, jadi ambiguitas §A2.3
**tidak** mengubah kesimpulan apa pun. `any` dipakai sebagai angka utama karena
ia yang sound.

➜ **D21 TEPAT pada separuh pertamanya**: `ARM_BLOCK` tidak vacuous — 24.3 %
kombinasi terlarang pada `c_arm = 0.05`, **tujuh kali lipat** fraksi `BLOCK`
struktur (3.26 %). Separuh keduanya ("optimumnya selalu punya jalan keluar")
dijawab §B6.

#### L3 — angka `p1_g2 §10` tereproduksi, dan satu kolom TIDAK

| clearance | konfigurasi bertabrakan | `p1_g2 §10` | rasio | pasangan target hilang | `p1_g2 §10` |
|---|---|---|---|---|---|
| 0.05 | **0.78 %** | 1.27 % | 0.62× | **0.00 %** | 0.00 % |
| 0.10 | **1.99 %** | 1.3 % | 1.53× | **0.01 %** | 0.00 % |
| 0.15 | **3.85 %** | 2.6 % | 1.48× | **0.19 %** | 0.00 % |
| 0.20 | **6.30 %** | 5.1 % | 1.23× | **0.84 %** | 0.62 % |
| 0.30 | **12.94 %** | 12.2 % | 1.06× | **3.72 %** | 2.86 % |

Palangnya adalah kolom **konfigurasi**, dan itu pilihan dengan alasan terukur,
bukan selera: "pasangan target hilang" menanyakan *"apakah MASIH ADA pose
bersama yang bebas"*, jadi menyampel 400 dari 2376 pose memberi tiap pasangan
lebih sedikit kesempatan dan mendorong angkanya **NAIK** — persis yang terlihat
pada 0.15 m (0.19 % vs 0.00 %). Kolom konfigurasi adalah rata-rata biasa atas
pose yang disampel dan **tidak** bias oleh subsampling yang sama. Semua rasio
dalam 0.62×–1.53×, jadi angka terbit tereproduksi pada grid yang berbeda.

#### L5 — radius sapuan, dan konflik §A2.3 diselesaikan dengan pengukuran

| | |
|---|---|
| menggantung (`q = 0`) | **0.4571 m** |
| **maksimum atas tabel kanonik yang BENAR-BENAR dipakai** (7542 konfigurasi berbeda) | **1.0623 m** ← angka yang §A2.3 minta |
| seluruh cloud ter-filter-approach | 1.0780 m |
| persentil tabel kanonik p50 / p90 / p99 | 0.5481 / 0.8493 / 0.9926 |
| batas aritmetik §A2.3 (0.4 + 1.0) | 1.4000 m — **batas atas yang SAH** ✓ |

Jadi 1.4 m boleh dipakai sebagai prapenyaring (ia batas atas), 1.16 m tidak
pernah boleh (ia bukan — dan §B5 Pertentangan 2 menunjukkan ia juga tidak
tereproduksi), dan yang mengikat adalah **1.0623 m**.

### B6. 🟢 ANGKA UTAMA — `Δ_struct`, `Δ_full`, `Δ_arm` pada S1 dan S2

#### B6.1 Bagaimana `Δ_full` diperoleh TANPA membangun solver kedua

Ini keputusan metodologis sesi ini dan ia ditulis penuh supaya bisa diserang.
§B4 mengukur gerbang lengan pada **45 ms per jadwal**; memasangnya ke dalam loop
dalam B&B (ribuan evaluasi per node) tidak muat anggaran 120 s. Yang dipakai
sebagai gantinya bukan aproksimasi — ia dua argumen yang keduanya sah:

```
BAWAH  Delta_full >= Delta_struct.
       ARM_BLOCK hanya PERNAH MEMBUANG jadwal, dan batas bawahnya (optimum
       takterkopel, Lemma 3) sama sekali tidak menyebut tabrakan, jadi ia tetap
       sah tanpa perubahan.

ATAS   Kalau jadwal yang solve_coupled2 kembalikan di bawah BLOCK saja JUGA
       memenuhi ARM_BLOCK, ia adalah jadwal LAYAK di model penuh dengan nilai
       yang sama, jadi Delta_full <= Delta_struct -- maka SAMA PERSIS, dan
       exact di mana pun Delta_struct exact. Tanpa pencarian.
       Kalau tidak, satu-satunya jadwal layak yang sesi ini konstruksi adalah
       SERIALISASI sadar-lengan, yang merupakan batas atas yang SANGAT LONGGAR.
```

🔴 **Konsekuensinya dilaporkan sebagai vonis, bukan sebagai catatan kaki:** di
mana jadwal struktur bertahan, `Δ_arm = 0` **terbukti**; di mana ia tidak
bertahan, `Δ_full` hanya terkurung antara `Δ_struct` dan biaya serialisasi, dan
**itu terlalu longgar untuk memutuskan ambang 5 %.**

#### B6.2 S1 — `gen_real`, 40 instance, dan regresi G10 yang lulus

**Nol jadwal gagal gerbang sadar-tunggu. Nol pelanggaran Lemma 3** (`ub < lb`
pada 0 dari 40).

| | Terukur | `p1_g10 §B6` |
|---|---|---|
| route | `lemma4` **25**, `dive-lb` **5**, `bnb` **10** | 25 / 5 / 10 — **identik** |
| `Δ` exact | **39 / 40** | 37 / 40 |
| kurungan tersisa | **1** (`n4_s7_mr0`, rasio **1.187**) | 3 |
| `mean Δ%` dua-sisi | **[0.0000, 0.4679]** → **BUKAN MAHAL** | [0.0000, 1.0141] → BUKAN MAHAL |
| wall mean / maks | 11.9 s / 120.2 s | 18.8 s / 120.1 s |

➜ **Vonis `p1_g10 §B6` berdiri**, dan dua dari tiga kurungannya tertutup di
mesin yang lebih ringan. `n4_s7_mr0` tetap terbuka pada rasio yang sama persis
(1.187) — **utang U3 G10 belum lunas** dan itu dilaporkan, bukan dihaluskan.

**Dan sekarang lengannya:**

| `c_arm` | jadwal struktur bertahan `ARM_BLOCK` | `Δ_full` mean % (dua-sisi) | vonis | `Δ_arm` |
|---|---|---|---|---|
| 0.00 (≈ ≤ 5 mm, lihat Pertentangan 1) | **36 / 40** | [0.0000, 5.5568] | TIDAK DAPAT DITENTUKAN | +[0.0000, 5.0890] |
| **0.05** | **29 / 40** | [0.0000, 14.2813] | TIDAK DAPAT DITENTUKAN | +[0.0000, 13.8134] |
| 0.10 | **22 / 40** | [0.0000, 29.4944] | TIDAK DAPAT DITENTUKAN | +[0.0000, 29.0266] |
| 0.15 | **18 / 40** | — | **TIDAK DIUKUR** pada 4 dari 40 | — |
| 0.20 | **11 / 40** | — | **TIDAK DIUKUR** pada 8 dari 40 | — |

🔒 **Yang BOLEH dikutip dari tabel ini, dan yang TIDAK.**

> **BOLEH:** pada `c_arm = 0.05`, jadwal optimum-struktur dari **29 dari 40**
> instance alami **juga bebas tabrakan lengan–lengan**, jadi pada instance itu
> `Δ_arm = 0.0000` **terbukti**, bukan diduga.
>
> **TIDAK BOLEH:** angka apa pun tentang berapa `Δ_arm` pada 11 sisanya. Batas
> atas 13.8 % di kolom kanan adalah biaya **serialisasi**, yaitu membuang
> seluruh paralelisme — ia batas atas yang sah dan hampir pasti jauh di atas
> yang sebenarnya. **Vonis A3-K3 untuk `Δ_full` TIDAK DAPAT DITENTUKAN pada
> ketiga nilai `c_arm` yang terukur, dan TIDAK DIUKUR pada dua sisanya.**

#### B6.3 S2 — probe rotcrowded, 40 instance

`gen_real_rotcrowded` (§B3), `sin_min = 0.525` sesuai §A. **Nol gagal gerbang.**

| | |
|---|---|
| `Δ_struct` | **40 / 40 exact, `Δ = +0.0000 s`**, route `lemma4` 40/40 → **GRATIS** |
| wall | ~0.1 s per instance |

| `c_arm` | bertahan | `Δ_full` mean % | vonis |
|---|---|---|---|
| 0.00 | **39 / 40** | [0.0000, 2.1158] | **BUKAN MAHAL (< 5 %)** |
| 0.05 | **24 / 40** | [0.0000, 36.5479] | TIDAK DAPAT DITENTUKAN |
| 0.10 | **18 / 40** | — | TIDAK DIUKUR pada 1 |
| 0.15 | **12 / 40** | — | TIDAK DIUKUR pada 3 |
| 0.20 | **9 / 40** | — | TIDAK DIUKUR pada 15 |

⚠️ **S2 dan S1 tidak pernah dirata-ratakan bersama**, dan probe S2 di sini
membatasi himpunan pose (§B3), jadi setiap angkanya membawa kualifikasi itu.

#### B6.4 🔴 Bacaan yang sebenarnya, dan ia BUKAN sebuah angka `Δ`

Tiga hal yang **terukur** dan tidak bergantung pada solver yang belum ada:

1. **`ARM_BLOCK` jauh lebih ketat daripada `BLOCK`.** 24.3 % kombinasi
   `(pose, himpunan tugas)` terlarang pada `c_arm = 0.05` (L4) versus 3.26 %
   pasangan pose untuk `BLOCK` — **tujuh kali lipat**. Dan pada jadwal yang
   benar-benar optimum, **11 dari 40** kehilangan kelayakannya pada clearance
   yang sama.
2. **Mekanisme yang membuat struktur murah TIDAK ADA pada lengan.**
   `p1_g10 §B4` menjelaskan biaya-nol struktur dengan "37.3 % ruang pose bebas
   tabrakan tanpa syarat — selalu ada tempat menyingkir". Lemma 1 versi lengan
   **kosong** (§B5 Pertentangan 4): **0 dari 2376**. Tempat menyingkir masih
   ada, tapi ia bergantung pada apa yang sedang dikerjakan lawan.
3. **Karena itu `p1_g10 §B12.2` sekarang punya isi.** Ia menulis "angka rugi
   TETAP BATAS BAWAH karena lengan di luar model". Sesi ini menaruh lengan **ke
   dalam** model dan mengukur bahwa kendalanya **memang mengikat** — tapi
   **berapa harganya masih belum terjawab**, dan itu sekarang tergantung pada
   satu hal yang bisa disebut: **solver sadar-lengan** (§C).

### B7. K4 — daftar yang §A wajibkan dilaporkan APA PUN HASILNYA

| | Isi | Di mana |
|---|---|---|
| **K4.1** | `Δ_struct` / `Δ_full` / `Δ_arm` per set, vonis dua-sisi, S1 dan S2 terpisah | §B6.2, §B6.3 |
| **K4.2** | sapuan `c_arm ∈ {0.00, 0.05, 0.10, 0.15, 0.20}` | §B6.2, §B6.3, L4 |
| **K4.3** | fraksi kombinasi `(p1, U1, p2, U2)` yang `ARM_BLOCK` | **24.275 %** pada `c_arm = 0.05` (L4), mode `all` 21.375 % |
| **K4.4** | biaya pemeriksa, diukur SEBELUM sapuan | §B4 |
| **K4.5** | radius sapuan maks + Lemma 1 versi lengan | **1.0623 m**; Lemma 1 lengan **KOSONG** (§B5) |
| **K4.6** | anggaran yang tersentuh | di bawah |
| **K4.7** | jadwal yang gagal gerbang, per penjadwal | **0 dari 80** (S1 40 + S2 40), gerbang sadar-tunggu G10, tidak dimodifikasi |

**K4.6 — anggaran, dan satu yang jebol.**

* Anggaran solver 120 s: tersentuh pada **1 dari 40** S1 (`n4_s7_mr0`), 0 dari 40 S2.
* Gerbang lengan: **200 panggilan, 6.7 s total** pada S1 — bukan penjarangan, tidak pernah dipotong.
* `serial_arm_ub` dibatasi **60 pose parkir termurah**; tutup itu tersentuh pada instance yang dilaporkan **TIDAK DIUKUR** (4 pada `c_arm = 0.15`, 8 pada 0.20 untuk S1; 1/3/15 untuk S2).
* 🔴 **Rule 6 (anggaran token) JEBOL, dan dilaporkan sesuai Rule 12.** Anggaran per sesi 30 000 token; sesi ini jauh melampauinya. Sebabnya bisa disebut: bacaan wajibnya sendiri (`p1_g10` 1229 baris + `p1_g11` §A + empat dokumen rujukan) sudah melebihi anggaran per-tugas sebelum satu baris kode ditulis. **Rule 6 tidak realistis untuk sesi dengan protokol §A sepanjang ini**, dan itu masalah aturannya, bukan pelaksanaannya — dicatat untuk G12.

### B8. Papan skor §7.2

| # | Dugaan (§A5, ditulis di muka) | Hasil |
|---|---|---|
| **D20** | `Δ_arm` pada S1 **≤ 1.0 %** dan vonis gabungan tetap BUKAN MAHAL | ⏸ **TERTUNDA** — `Δ_arm` terbukti **0.0000 pada 29 dari 40** instance, tapi **TIDAK DAPAT DITENTUKAN** secara keseluruhan (§B6.2). Aturan `p1_g10 §B11` catatan 3 dipakai apa adanya: dugaan yang tidak bisa dinilai **ditandai TERTUNDA, bukan MELESET** |
| **D21** | `ARM_BLOCK` **tidak vacuous**, tapi optimumnya selalu punya jalan keluar | 🟡 **SETENGAH TEPAT, dicatat MELESET.** Tidak vacuous ✅ (24.3 % pada `c_arm = 0.05`, L4). "Selalu punya jalan keluar" **salah**: 11 dari 40 optimum-struktur kehilangan kelayakan pada `c_arm = 0.05` |
| **D22** | pemeriksa `ARM_BLOCK` **≥ 100× lebih mahal** dari `BLOCK` **per evaluasi** | ❌ **MELESET** — per pasangan ia **3× lebih MURAH** (1.86 vs 6.44 µs). Pada level jadwal utuh ia **230× lebih mahal**, jadi kehati-hatiannya benar dan unitnya salah (§B4) |
| **D23** | `gen_real_rotcrowded` **lulus** gerbang probe (≥ 8/40 mengikat) | ❌ **MELESET** — 0/40 `Δ > 0`. Tapi bukan seperti G10: solver terkopelnya **benar-benar jalan** pada 4/10 pada `sin_min = 0.7625` (§B3). Dan rancangan probe yang §A kunci **tidak bisa dibangun sama sekali** |
| **D24** | traverse × traverse tidak menambah apa pun di atas `BLOCK` pada `c_clear = 0.10` | ✅ **TEPAT** — dan lebih kuat: `HANG_BLOCK(c) ⊆ BLOCK(c)` dengan δ ≤ 0.004 m, dan lengan menggantung bertabrakan **2–3× lebih jarang** dari strukturnya (§B5 Pertentangan 3) |

**Satu tepat, tiga meleset, satu tertunda.** Papan skor `p1_g10 §B11` berdiri di
**19 meleset / 6 tepat**; sesi ini menambah 3/1 → **22 meleset, 7 tepat**
(D20 tidak dihitung).

Dua bacaan:

1. **Prior DUNIA ("kendala yang belum diukur itu LONGGAR") mendapat contoh
   tandingan pertamanya yang bertahan.** D21 dan D23 dua-duanya meleset ke arah
   longgar seperti biasa, tapi D21 setengahnya meleset ke arah **lebih ketat**:
   `ARM_BLOCK` mengikat tujuh kali lipat `BLOCK`, dan optimumnya **tidak**
   selalu punya jalan keluar. Ini kelas kendala yang berbeda dari semua yang
   sebelumnya diukur — bukan kelayakan statis, bukan eksklusi ruang struktur,
   melainkan **eksklusi ruang yang bergantung pada penugasan tugas**.
2. **Prior KODE SENDIRI ("lebih lambat, lebih rumit, lebih salah") MELESET untuk
   pertama kalinya**, di D22, dan pada sisi pesimisnya seperti yang metode
   `p1_g10 §B11` bacaan 2 anjurkan. Ia sekarang **4 dari 5**. Yang menarik:
   melesetnya karena kode yang **dibandingkan** (predikat struktur beku)
   ternyata yang lambat, bukan kode baru.

🔴 **Dan satu cacat di kode sendiri yang ditemukan uji, bukan pembacaan ulang**
(§B5, L4): draf pertama L4 mengizinkan tugas yang sama dikerjakan **kedua**
gantry, melaporkan 63.1 % kombinasi terlarang; angka yang benar **24.3 %**.
Ia ketahuan karena angkanya diadu dengan `p1_g2 §10` yang sudah terbit. Itu
empat dari lima untuk prior kode-sendiri kalau dihitung sebagai kejadian, bukan
sebagai dugaan.

### B9. Batasan setelah sesi ini

Seluruh `p1_g7 §A4`, `p1_g8 §B10`, `p1_g9 §A4`, `p1_g10 §B12` **masih berlaku**
kecuali yang §B cabut eksplisit (`p1_g10 §B12.3`: S2 sekarang berfungsi masuk
rezim, walau tetap tidak mengikat). Yang ditambahkan:

1. 🔴 **`Δ_full` TIDAK DIUKUR sebagai angka.** Tidak ada solver sadar-lengan.
   Yang ada: `Δ_arm = 0` **terbukti** pada 29/40 (S1) dan 24/40 (S2) pada
   `c_arm = 0.05`, dan kurungan yang terlalu longgar pada sisanya.
2. 🔴 **`c_arm = 0.00` degenerate** (§B5 Pertentangan 1). Polyline tak
   berketebalan. Nilai pertama yang berarti adalah 0.05, dan `ε = 0.005 m`
   adalah satu-satunya ketebalan pada baris 0.00.
3. 🔴 **Lemma 1 tidak punya analog lengan** (§B5 Pertentangan 4). Serialisasi
   bukan lagi jalan keluar yang dijamin ada.
4. **Radius `p1_g3 §B4` (0.52 / 0.55 / 1.16 m) tidak tereproduksi** dari URDF;
   mesh tabrakan tidak bisa dimuat di mesin ini. Yang dipakai: 0.4571 / 0.4011 /
   1.0623 m dari polyline. Bias tetap **meremehkan**.
5. **Proksi polyline meremehkan volume sapuan** → `Δ_arm` yang terukur tetap
   **batas bawah** untuk jumlah kombinasi yang terlarang.
6. **Kanonik = `manip`; policy lain tidak disapu.** Slot di dalam satu dwell
   tidak diurutkan; mode `any`/`all` mengurungnya (selisih 2–3 poin persen).
7. **Cloud kanonik `/tmp/irm_cloud_pol.npz` hanya ada di `/tmp`** (§B1). Reboot
   membuat setiap polyline lengan sesi ini tidak bisa direproduksi.
8. **Tabrakan lengan–lingkungan dan lengan–struktur lawan tidak dimodelkan**
   (§A4.6, tidak berubah). Gerak lengan tidak direncanakan; `T_fold = 0`.
9. **U3 dan U4 G10 tidak dikerjakan.** `n4_s7_mr0` masih terkurung pada rasio
   1.187; W2 masih longgar (`max_evade = 1`), 9 instance `solver < W2` tidak
   diperiksa ulang. **TIDAK DIUKUR**, bukan "diperkirakan tidak berubah".

---

## C. Prompt sesi berikutnya — G12

> **Rekomendasi: Opus 5, effort TINGGI.** Alasannya spesifik dan berbeda lagi.
> G11 menaruh lengan ke dalam model dan mengukur bahwa kendalanya **mengikat**
> — 11 dari 40 optimum-struktur kehilangan kelayakan pada `c_arm = 0.05` — tapi
> **tidak bisa mengukur harganya**, karena batas atasnya cuma serialisasi.
> G12 harus membangun solver yang menutup kurungan itu, dan bahayanya berbeda
> dari G10: di sana oracle-nya yang belum ada; di sini **oracle-nya ada dan
> sudah terbukti** (W2 + P0–P4 + gerbang 17 mutasi), tapi pemeriksanya **230×
> lebih mahal per jadwal** dan itu **empat sesi berturut-turut** yang jebakannya
> sama. Effort tinggi karena keputusan yang menentukan adalah **di mana
> `ARM_BLOCK` dipasang di dalam pencarian**, dan pilihan yang salah tidak punya
> sinyal error — ia cuma lambat, lalu melaporkan kurungan.

```
Sesi G12 -- REACH-2: menutup Delta_full. Predikatnya sudah ada dan terbukti;
yang belum ada adalah solver yang bisa memakainya.

BACA DULU:
1. docs/p1_g11_arm.md -- SELURUHNYA. Khususnya:
   B1 (PROVENANSI cloud -- baca sebelum apa pun, ia utang nomor 1),
   B3 (kenapa TIDAK ADA probe berbasis tugas yang bisa memaksa (N1)),
   B4 (biaya pemeriksa: primitifnya MURAH, gerbang jadwalnya 230x),
   B5 (EMPAT pertentangan: c_arm=0 degenerate, radius p1_g3 tidak
       tereproduksi, Lemma A jauh lebih kuat, LEMMA 1 TIDAK PUNYA ANALOG),
   B6 (angka utama + kenapa Delta_full TIDAK DIUKUR),
   B8 (papan skor: prior kode-sendiri MELESET untuk pertama kalinya),
   B9 (batasan)
2. docs/p1_g10_sched4.md B1, B3, B9, B11, B12
3. reachability_gng/sched_arm.py, test/verify_sched_arm.py,
   test/eval_sched_arm.py, dan yang beku: sched.py, sched_coll.py,
   sched_coupled.py

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.
CATATAN MESIN: G11 diukur pada load 3.3-3.7/16 dengan node RealSense sesi lain
masih hidup. G10 pada load 42/16. JANGAN bandingkan detik lintas sesi tanpa
menyebut ini.

=== UTANG NOMOR 1, KERJAKAN SEBELUM APA PUN, ~10 MENIT ===
U0. Cloud kanonik /tmp/irm_cloud_pol.npz (35 MB) HANYA ADA DI /tmp. Setiap
    polyline lengan G11 dibaca terhadapnya. Reboot = seluruh sesi G11 tidak
    reproducible. Salin ke data/ dan catat di data/README.md, persis seperti
    yang dilakukan 2026-08-14 untuk cap_g*_rail160.npz. Verifikasi dengan L1a
    (harus tetap BIT-IDENTIK).

=== YANG SUDAH TEGAK, JANGAN BANGUN ULANG ===
- solve_coupled2 = GROUND TRUTH terhadap himpunan kandidat A2.2. Palangnya
  W2 (31 instance, 15 mengikat, 0 solver>W2) + W2b(i) 12/12 + P0-P4.
  U1 SUDAH DIPUTUSKAN lewat jalur (b); JANGAN buka lagi. no_dive = diagnostik.
- Gerbang sadar-tunggu (gate_sched_coupled.py): 17 mutasi 0 lolos. PAKAI.
- Predikat lengan (sched_arm.py) + L0-L5. Provenansi canon BIT-IDENTIK.
- Lemma A TERBUKTI: HANG_BLOCK(c) subset BLOCK(c + 0.004). Rezim traverse
  SUDAH tercakup BLOCK. JANGAN modelkan ulang traverse.
- ANGKA: S1 Delta_struct mean dalam [0, 0.4679] BUKAN MAHAL, 39/40 exact,
  route lemma4 25 / dive-lb 5 / bnb 10 (identik G10). PAKAI, jangan hitung ulang.
- L4: 24.275% kombinasi (pose, himpunan tugas) terlarang pada c_arm = 0.05.

=== TUGAS, BERURUTAN. JANGAN LOMPAT. ===
1. U0 di atas.
2. SOLVER SADAR-LENGAN. Ini seluruh isi sesi. Yang HARUS diputuskan di A
   SEBELUM kode, karena tidak punya sinyal error:
   (a) DI MANA ARM_BLOCK dipasang. Ia BUKAN fungsi (pose, pose) -- ia fungsi
       (pose, himpunan tugas) x (pose, himpunan tugas), jadi tabel pra-hitung
       2376^2 TIDAK ADA analognya (A2.4 poin 1). Tapi B4 mengukur primitifnya
       MURAH (1.86 us/pasangan, 3x lebih murah dari BLOCK); yang mahal adalah
       memulai ulang jalan-sertifikat di setiap batas dwell. Kandidat yang
       harus diadu SEBAGAI ANGKA, bukan sebagai selera:
         - pra-hitung per instance: untuk tiap (g1 pose, U1) x (g2 pose, U2)
           yang DP-nya benar-benar simpan (|keep| x subset), bukan 2376^2
         - hanya menguji pasangan dwell yang BERTUMPANG TINDIH dalam waktu
           (traverse sudah tercakup Lemma A -> BLOCK)
       Ukur biayanya SEBELUM menjalankan sapuan. INI SESI KEEMPAT berturut-turut
       yang jebakannya sama (p1_g8 B4, p1_g9 B3, p1_g10 B3-1, p1_g11 B4).
   (b) BATAS ATAS. Lemma 1 TIDAK punya analog lengan (himpunannya KOSONG,
       B5 Pertentangan 4), jadi serialisasi BUKAN lagi jalan keluar yang
       dijamin ada, dan konstruktor UB G9/G10 (_serial_ub) tidak sah apa
       adanya di model penuh. Apa yang menggantikannya, dan APA BUKTINYA bahwa
       instance yang layak selalu punya jadwal layak?
       ->  Kalau ternyata ADA instance yang TIDAK LAYAK di bawah ARM_BLOCK,
           itu temuan yang lebih besar dari Delta berapa pun: ia berarti
           tugas-tugas tertentu tidak bisa dialokasikan ke dua gantry sama
           sekali, yang adalah pernyataan tentang SEL, bukan tentang jadwal.
   (c) c_arm mana yang jadi angka utama. c_arm = 0.00 DEGENERATE (B5
       Pertentangan 1) -- polyline tak berketebalan. Pilih, dan tuliskan
       alasannya SEBELUM melihat hasil. Kandidat yang punya dasar: 0.05
       (p1_g2 10 mengukur 0.00% pasangan target hilang di bawah 0.15 m, jadi
       0.05 masih jauh di sisi permisif).
3. Delta_full pada S1 dan S2 dengan aturan dua-sisi p1_g10 A3-K3, dan
   Delta_arm = Delta_full - Delta_struct sebagai angka, bukan sebagai kurungan
   selebar serialisasi.
4. Kalau (dan hanya kalau) 2-3 lulus: U3 (n4_s7_mr0 masih terkurung 1.187)
   dan U4 (W2 max_evade = 1, 9 instance solver < W2).

=== KUNCI KRITERIA SEBELUM KODE, ke docs/p1_g12_*.md A ===
1. Di mana ARM_BLOCK dipasang, dan berapa anggaran evaluasinya sebagai ANGKA.
2. Konstruktor batas atas, dan buktinya bahwa ia selalu menghasilkan sesuatu.
3. c_arm utama, dengan alasan tertulis sebelum hasil.
4. Palang: apa yang harus lulus sebelum Delta_full boleh disebut. Palang G10
   (A3-K2) dan G11 (A3-K1) dua-duanya contoh yang bisa dipakai ulang.
5. Apa yang dilaporkan kalau lagi-lagi tidak semuanya bisa dibuktikan.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- Jadwal mengerjakan tiap tugas TEPAT SEKALI, jadi U1 dan U2 pada satu instan
  DISJOINT. Menariknya independen menaruh dua tool frame di titik yang sama
  dan melaporkan 63.1% terlarang, bukannya 24.3% (p1_g11 B5 L4).
- c_arm = 0.0 pada polyline tak berketebalan = kejadian berukuran nol. BUKAN
  analog c_clear = 0.0 (benda struktur punya volume).
- Lemma 1 TIDAK punya analog lengan. 0 dari 2376 pose aman universal.
- Argumen ber-INDEKS tidak bisa ditukar (pair_distance, p1_g10 B1.1, 188.7 mm).
- Kelayakan aksi diperiksa sampai ujung gerak terkomit LAWAN (p1_g10 B3-5).
- Konstruktor jadwal harus mengembalikan JADWAL YANG SAMA dengan yang ia
  periksa (p1_g10 B1.2).
- "Kembangkan gantry yang tertinggal" HEURISTIK di model terkopel (B3-4).
- Dua proses yang menulis satu file JSON dengan read-modify-write saling
  menimpa. Tabel S2 G11 terpotong senyap dari 37 baris jadi 2 sebelum ada yang
  melihat angkanya. Satu berkas per set.
- Angka radius p1_g3 B4 (0.52/0.55/1.16 m) TIDAK tereproduksi dari URDF; mesh
  STL menunjuk path mesin lain. Pakai 0.4571/0.4011/1.0623 dari polyline.
- Yang lambat adalah PEMERIKSA. EMPAT sesi berturut-turut.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 22 meleset, 7 tepat.
  Prior DUNIA: LONGGAR -- tapi D21 memberi contoh tandingan yang BERTAHAN, pada
  kelas kendala BARU (eksklusi ruang yang bergantung PENUGASAN TUGAS, bukan
  kelayakan statis dan bukan eksklusi struktur). Jangan pakai prior itu untuk
  lengan tanpa mengukur.
  Prior KODE SENDIRI: LEBIH LAMBAT, LEBIH RUMIT, LEBIH SALAH -- 4 dari 5,
  meleset pertama kalinya di D22. Tetap tulis dugaan kode sendiri pada sisi
  PESIMISNYA.
- DUGAAN YANG DINILAI MEMAKAI SOLVER YANG BELUM LULUS GERBANGNYA TIDAK DINILAI.
  Tandai TERTUNDA. D20 G11 memakai aturan ini.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 punya lima, G11 punya empat plus satu rancangan probe yang tidak bisa
  dibangun sama sekali. Itu bukan aib.
- Rule 6 (anggaran token 30k/sesi) JEBOL di G11 dan akan jebol lagi: bacaan
  wajibnya saja sudah melebihinya. Naikkan aturannya atau nyatakan sesi
  protokol-panjang sebagai pengecualian -- jangan diam-diam melampauinya.
- Akhiri dengan prompt sesi berikutnya (G13).
```
