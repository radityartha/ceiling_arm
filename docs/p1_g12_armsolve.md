# P1 / G12 — REACH-2: menutup `Δ_full`. Solver sadar-lengan.

> Sesi G12, 2026-08-15. Melanjutkan [p1_g11_arm.md](p1_g11_arm.md).
> G11 menaruh lengan **ke dalam** model, membuktikan predikatnya (L0–L5), dan
> mengukur bahwa ia **mengikat**: 24.275% kombinasi `(pose, himpunan tugas)`
> terlarang pada `c_arm = 0.05`, **tujuh kali** fraksi `BLOCK` struktur, dan
> **11 dari 40** optimum-struktur kehilangan kelayakannya. Yang G11 **tidak**
> bisa lakukan adalah menyebut **harganya**, karena satu-satunya batas atas yang
> ia punya adalah serialisasi (`p1_g11 §B6.1`).
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode solver sesi ini dijalankan.**
> §B diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**. §A tidak ditulis ulang belakangan
> (disiplin `p1_g7`–`p1_g11`).
>
> Sesi ini **sepenuhnya offline**. Lengan 4× masih dilepas.
>
> **Rekomendasi model: Opus 5, effort TINGGI.** Alasannya spesifik: keputusan
> yang menentukan sesi ini adalah **di mana `ARM_BLOCK` dipasang di dalam
> pencarian**, dan pilihan yang salah **tidak punya sinyal error** — ia hanya
> lambat, lalu melaporkan kurungan yang tidak bisa memutuskan ambang 5%. Itu
> **empat sesi berturut-turut** dengan jebakan yang sama (`p1_g8 §B4`,
> `p1_g9 §B3`, `p1_g10 §B3-1`, `p1_g11 §B4`).

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci / terverifikasi di | Dipakai bagaimana |
|---|---|---|
| Model intra-gantry, `T_traverse`, dwell 2.0 s, makespan, handover = SATU tugas dua-lengan | `p1_g7 §A1`/`§A2`, V0–V4 | nol perubahan |
| Peta `cap_g{1,2}_rail160.npz` (33×72, \|P\| = 2376) | `p1_state §3` | oracle kelayakan |
| `sched.solve_exact` | `p1_g7 §B1` | batas bawah yang sah (Lemma 3) |
| Predikat struktur `BLOCK`, lintasan A2.3, sertifikat | `p1_g9 §B1`/`§B2`, W0–W1 | apa adanya |
| Gerbang sadar-tunggu `validate_coupled` | `p1_g10 §B2`, 17 mutasi 0 lolos | **wajib** untuk setiap jadwal |
| `solve_coupled2` = **GROUND TRUTH** thd himpunan kandidat A2.2 | `p1_g11 §B2` (W2 31/15 mengikat/0 solver>W2, W2b(i) 12/12, P0–P4) | oracle, satu arah. **U1 TIDAK dibuka lagi.** |
| Predikat lengan `sched_arm.py` + L0–L5, provenansi canon BIT-IDENTIK | `p1_g11 §B5`, dan L1a **dijalankan ulang** sesi ini (§B0) | apa adanya |
| **Lemma A**: `HANG_BLOCK(c) ⊆ BLOCK(c + 0.004)` pada seluruh 2376² | `p1_g11 §B5` Pertentangan 3 | **dasar Lemma B, §A2.2** |
| `Δ_struct` S1: mean ∈ [0, 0.4679] BUKAN MAHAL, 39/40 exact, route 25/5/10 | `p1_g11 §B6.2` | **regresi**, tidak dihitung ulang sebagai temuan baru |
| L4: 24.275% `(pose, himpunan tugas)` terlarang pada `c_arm = 0.05` | `p1_g11 §B5` | angka pembanding |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi:**
`sched.py`, `sched_heur.py`, `sched_coll.py`, `sched_coupled.py`,
`sched_arm.py`, `test/verify_sched_exact.py`, `test/verify_sched_coll.py`,
`test/verify_sched_coupled.py`, `test/gate_sched_coupled.py`,
`test/verify_sched_arm.py`, `test/eval_sched_arm.py`.

⚠️ `sched_arm.py` **masuk daftar beku sesi ini** — G11 membuktikannya L0–L5 dan
setiap angka G12 dibaca terhadapnya. Satu pengecualian yang sudah terjadi
**sebelum** §A dikunci dan dicatat di sini supaya tidak terlihat seperti
penyelundupan: **U0** (§A1 poin 0) mengubah `CLOUD` dari path `/tmp` menjadi
resolver `data/`-dulu. L1a dijalankan ulang sesudahnya dan `masks`+`canon`
tetap **BIT-IDENTIK** untuk kedua peta. Commit `c49db77`.

Aturan `p1_g10 §A0` berlaku utuh: **bug di berkas beku dilaporkan sebagai
temuan, diperbaiki di tempatnya, pembekuannya dinyatakan gugur.** G10 memakai
aturan itu tiga kali; G11 nol kali.

### A1. Yang dibangun sesi ini, dan yang TIDAK

Berurutan, dan urutannya wajib (`p1_state §7.1`):

0. **U0** — cloud kanonik keluar dari `/tmp`. **SUDAH SELESAI** sebelum §A
   dikunci (commit `c49db77`), karena ia utang reproducibility, bukan hasil.
1. **Biaya pemasangan diukur** (A2.5) — **SEBELUM** sapuan apa pun.
2. **Solver sadar-lengan** (A2.1–A2.4). Ini seluruh isi sesi.
3. **Gerbangnya** (A3-M0…M5). Tidak ada angka `Δ_full` yang boleh disebut
   sebelum gerbang lulus.
4. **`Δ_full` dan `Δ_arm`** pada S1 dan S2, aturan dua-sisi `p1_g10 §A3-K3`.
5. Kalau (dan hanya kalau) 2–4 lulus: **U3** (`n4_s7_mr0`, rasio 1.187) dan
   **U4** (W2 `max_evade = 1`, 9 instance `solver < W2`).

**TIDAK dibangun, dan tidak boleh menyelinap masuk:** perencanaan gerak lengan,
`T_fold ≠ 0`, kapabilitas dinamis, tabrakan lengan–lingkungan, lengan–struktur
lawan, kapsul/ketebalan tautan yang radiusnya tidak pernah diukur, policy
kanonik selain `manip`, dan heuristik baru apa pun sebelum A3-M3 lulus.

### A2. DI MANA `ARM_BLOCK` DIPASANG — keputusan utama, dikunci sekarang

#### A2.1 🔒 TIDAK ADA SOLVER KEDUA. Predikatnya yang disuntikkan.

Ini keputusan yang paling mudah salah, jadi alasannya ditulis penuh.

Godaannya adalah menulis `solve_arm()` baru di samping `solve_coupled2`. Itu
**ditolak**, dengan alasan yang bukan selera:

> `solve_coupled2` adalah satu-satunya artefak di seluruh P1 yang punya palang
> selengkap ini — W2 (31 instance, 15 mengikat, 0 kali `solver > W2`), W2b(i)
> 12/12 sampai 1e-9, P0–P4, dan gerbang 17-mutasi. **Fork = seluruh palang itu
> hangus**, dan diganti oleh palang baru yang harus dibangun dari nol di sesi
> yang sama yang juga harus mengukur `Δ_full`. Prior kode-sendiri
> (`p1_state §7.2`: *lebih lambat, lebih rumit, lebih salah*, 4 dari 5) berlaku
> paling keras persis di sini.

🔒 **DIKUNCI: `ARM_BLOCK` masuk lewat PENGGANTIAN PREDIKAT, bukan lewat
pencarian baru.** Loop B&B, `_dive`, urutan aksi, `_split_best`, `PoseCost`,
aturan `proved` — **nol baris berubah**, dan `sched_coupled.py` tetap beku.
Yang diganti hanya nama-nama yang loop itu panggil, lewat satu context manager
eksplisit di `sched_armfull.py`:

| Nama di namespace `sched_coupled` | Diganti dengan | Kenapa |
|---|---|---|
| `feasible_starts` | `arm_feasible_starts` | satu-satunya tempat aksi diuji kelayakannya |
| `schedule_conflict` | struktur **dan** lengan | jalur Lemma 4 (baris 613) tidak boleh lolos tanpa uji lengan |
| `_repair_ub` | sadar-lengan | incumbent yang tidak layak = `best_m` palsu = pemangkasan **tidak sound** |
| `_serial_ub` | `arm_serial_ub` (A2.4) | idem, dan Lemma 1 tidak punya analog |

⚠️ **`safe_poses` SENGAJA TIDAK diganti**, dan konsekuensinya dicatat sekarang
sebagai batasan, bukan ditemukan belakangan: ia hanya membangun **himpunan
target menghindar** dan `h_safe` (baris 775). Soundness datang dari uji, bukan
dari himpunan itu — tapi `h_safe` bisa membuat cabang menghindar **dilewati**
saat `h` aman-struktur namun memblokir-lengan. Itu **kehilangan kelengkapan**,
bukan kehilangan soundness, dan ia dilaporkan sebagai batasan di §B tanpa
menunggu ditemukan.

#### A2.2 🔒 LEMMA B — regime `menggantung × menggantung` DIHAPUS, tidak diuji

Inilah yang membuat pemasangan ini murah, dan ia **turunan**, bukan asumsi.

```
LEMMA B.  Misalkan sebuah jadwal bebas BLOCK pada clearance c_struct,
          dengan c_struct >= c_arm + delta,  delta = 0.004 m.
          Maka pada SETIAP instan di mana KEDUA gantry menggantung,
          ARM_BLOCK(c_arm) TIDAK berlaku.

BUKTI.    L2 (p1_g11 B5 Pertentangan 3) mengukur, pada SELURUH 2376^2 pasangan
          pose, HANG_BLOCK(c) subset BLOCK(c + delta) dengan delta <= 0.004 m.
          Bebas BLOCK(c_arm + delta) pada setiap instan => bebas
          HANG_BLOCK(c_arm) pada setiap instan.  ∎
```

🔴 **Syarat pakainya dikunci, karena L2 mengukur `δ` pada grid `c` diskret
{0.00, 0.02, 0.05, 0.10, 0.20} dan bukan pada semua `c`:** Lemma B hanya boleh
dipakai pada nilai `c_arm` yang `δ`-nya **dijalankan ulang** sesi ini pada
seluruh 2376² (uji **L6a**, A3-M2). Kalau `δ > 0.004` pada suatu `c_arm`, yang
dipakai adalah `δ` terukur di situ, bukan 0.004.

➜ Konsekuensi: solver dijalankan dengan `c_clear = c_arm + δ` untuk predikat
**struktur**, dan uji lengan **hanya** perlu menjangkau instan di mana
**setidaknya satu** gantry sedang dwell dengan tugas. Regime `traverse × traverse`
tidak pernah disentuh oleh kode lengan sama sekali.

⚠️ Ini **menaikkan** `c_clear` struktur dari 0.0 ke `c_arm + δ` = 0.054 m.
`p1_g10 §B10` sudah menyapu `c_clear` sampai 0.10 dengan vonis tetap BUKAN
MAHAL, jadi kenaikan itu **sudah terukur tidak mengubah vonis struktur** — tapi
ia mengubah `Δ_struct` sebagai angka, jadi **`Δ_struct` dilaporkan pada
`c_clear` yang sama** supaya `Δ_arm = Δ_full − Δ_struct` mengurangkan dua hal
yang sebanding. Basis G11 (`c_clear = 0`) tetap dilaporkan sebagai regresi.

#### A2.3 🔒 `arm_feasible_starts` — dekomposisi TIGA KASUS

`feasible_starts(tr, q, dur, t_min, other, …)` mengembalikan waktu mulai di mana
gantry boleh traverse ke `q` lalu menahan `dur`. Versi lengannya memanggil
**yang beku apa adanya** untuk mendapat kandidat struktur, lalu **membuang**
kandidat yang melanggar lengan. Ia tidak pernah mengembalikan waktu yang lebih
awal dari yang beku → arahnya sama dengan seluruh tumpukan ini: **batas atas**.

Pada satu kandidat start `s`, aksi menempati `[s, s+T)` traverse + `[s+T, s+T+dur)`
dwell di `(p2, U, assign)`. Lawan `other` punya masa depan yang sudah ter-commit.
Tiga kasus, dan hanya dua yang butuh kerja:

| | Keadaan lengan | Ditangani |
|---|---|---|
| **(i)** | kita menggantung (traverse / `dur = 0`), lawan menggantung | **Lemma B — nol evaluasi** |
| **(ii)** | kita dwell statis, lawan dwell statis, jendelanya **bertumpang tindih** | **SATU** `arm_block((p2,U),(p',U'))` per pasangan jendela |
| **(iii)** | satu pihak dwell statis, pihak lain **menggantung** (traverse, menunggu, parkir) | jalan-sertifikat Lipschitz, **hanya** atas potongan waktu itu |

Kasus (iii) memakai `V_ARM` (satu pihak bergerak), bukan `V_REL_ARM` — separuh
kecepatan penutupan, jadi langkah sertifikatnya **2× lebih panjang** daripada
gerbang penuh G11.

🔒 **Cache dikunci sebagai LAZY MEMO, bukan tabel pra-hitung.** `p1_g11 §A2.4`
poin 1 benar bahwa tabel `(2376 × 2ⁿ)²` tidak ada; tapi kandidat "pra-hitung per
instance atas `|keep| × subset`" juga **tidak dipilih**, dan alasannya diukur
bukan diselerakan (A2.5, D25): ia membayar **seluruh** hasil kali di muka
sementara B&B menyentuh sebagian kecilnya. Memo `dict[(p1,U1,p2,U2) → jarak]`
memberi penghematan yang sama tanpa biaya bangun. **Kalau A2.5 mengukur
sebaliknya, §B menuliskannya sebagai pertentangan dan memakai pra-hitung.**

#### A2.4 🔒 BATAS ATAS — dan pengakuan bahwa ia TIDAK dijamin ada

`p1_g11 §B5` Pertentangan 4: **Lemma 1 tidak punya analog lengan. 0 dari 2376
pose aman universal.** Maka `_serial_ub` G9/G10 — yang bersandar pada "selalu ada
pose aman untuk diparkir" — **tidak sah lagi apa adanya**, dan §A tidak boleh
berpura-pura punya penggantinya yang terjamin.

🔒 **`arm_serial_ub`, dan satu-satunya perubahan nyata dari G11 adalah TUTUPNYA
DICABUT.** G11 `eval_sched_arm.serial_arm_ub` mencari pose parkir
**60 termurah-dulu** lalu menyerah. Itu membuat "tidak ada batas atas" tidak bisa
dibedakan dari "anggarannya habis" — dan G11 melaporkan 4 (pada `c_arm = 0.15`)
dan 8 (pada 0.20) instance TIDAK DIUKUR persis karena itu. Sesi ini mencari
**SELURUH \|P\| = 2376** pose parkir. Biayanya bisa dihitung di muka dan itulah
alasan tutupnya dicabut:

```
2376 pose parkir  x  <= 6 jendela dwell lawan  x  4 pasangan lengan
    = <= 57 024 polyline_min_dist,  pada 1.86 us (p1_g11 B4)  =  ~0.11 s
```

➜ **Tutup 60 pose itu tidak pernah menghemat apa pun yang berarti**, dan ia
membeli ketidaktahuan seharga 12 instance. Dicabut.

🔴 **APA BUKTINYA bahwa instance yang layak selalu punya jadwal layak? TIDAK
ADA, dan §A menolak mengarangnya.** Yang dikunci sebagai gantinya adalah
**pengukurannya**, sebagai uji tersendiri:

```
LEMMA C (dugaan, DIUKUR di B, tidak diasumsikan):
  untuk setiap perhentian (p, U, assign) yang jadwal solo sebuah gantry pakai,
  ADA setidaknya satu dari 2376 pose di mana gantry lawan bisa MENGGANTUNG
  tanpa ARM_BLOCK.
```

* **Lemma C lulus** → serialisasi ada, `arm_serial_ub` selalu mengembalikan
  sesuatu, dan kurungan `Δ_full` selalu terdefinisi.
* **Lemma C gagal pada suatu `(p, U)`** → ada perhentian yang **tidak ada tempat
  bagi gantry lawan di seluruh sel**. Itu **temuan yang lebih besar daripada
  `Δ` berapa pun** — ia pernyataan tentang **SEL**, bukan tentang jadwal — dan
  ia dilaporkan sebagai judul §B, bukan sebagai catatan kaki.

🔒 **Tiga tingkat pelaporan yang dibedakan tegas, dikunci sekarang:**

| Yang boleh ditulis | Kapan |
|---|---|
| `Δ_full` **exact** | solver mengembalikan jadwal **dan** `proved` |
| `Δ_full` **terkurung** `[lo, hi]` | ada jadwal layak, optimalitas tidak terbukti |
| **TIDAK ADA JADWAL DITEMUKAN (anggaran X)** | pencarian + serialisasi dua-duanya kosong |
| **TERBUKTI TIDAK LAYAK** | **hanya** kalau Lemma C gagal pada perhentian yang **setiap** alokasi wajib pakai. Kehabisan anggaran **tidak pernah** boleh ditulis begini. |

#### A2.5 🔒 ANGGARAN EVALUASI, SEBAGAI ANGKA — diukur SEBELUM sapuan

Ini sesi **kelima** dengan jebakan yang sama. Yang dikunci, dengan angka, dan
diukur sebelum satu instance S1 dijalankan:

| | Anggaran | Dari mana angkanya |
|---|---|---|
| `arm_feasible_starts` **overhead** atas `feasible_starts` beku | **≤ 5×** | pemasangan yang benar mengganti jalan-sertifikat penuh (45 ms, G11 §B4) dengan ≤ 6 evaluasi statis + potongan (iii) |
| satu evaluasi kasus (ii) | **≤ 30 µs** | 4 pasangan × mean 1.34 konfigurasi (G11 §B4) × 1.86 µs ≈ 13 µs, ×2 kelonggaran |
| `arm_serial_ub` per instance | **≤ 1.0 s** | 57 024 evaluasi × 1.86 µs = 0.11 s, ×9 kelonggaran |
| wall per instance | **120 s, TIDAK dilonggarkan** | `p1_g10 §K4`, sama dengan G7/G9/G10/G11 |

🔒 **ATURAN KEPUTUSAN, ditulis sebelum angkanya dilihat:** kalau overhead
terukur **> 5×**, pemasangan A2.3 dinyatakan **SALAH**, dan yang dipakai adalah
kandidat kedua (pra-hitung per instance atas `|keep| × subset` yang DP benar-benar
simpan). Peralihan itu ditulis sebagai pertentangan §B, bukan diputuskan diam-diam.

#### A2.6 🔒 `c_arm` UTAMA = **0.05**, alasan ditulis SEBELUM hasil

| # | Alasan |
|---|---|
| 1 | `c_arm = 0.00` **degenerate** (`p1_g11 §B5` Pertentangan 1): polyline tak berketebalan → "dua garis berpotongan persis" = kejadian berukuran nol; terukur **0** dari 4000 pasangan dan **0** pada 2376² penuh. Satu-satunya ketebalannya adalah `ε = 0.005 m`, penjaga terminasi — yaitu baris itu diam-diam menguji "≤ 5 mm". |
| 2 | `p1_g2 §10` mengukur **0.00%** pasangan target hilang sampai clearance **0.15 m**. Maka 0.05 masih **jauh** di sisi permisif: ia tidak membuang satu pun pasangan target yang sel ini butuhkan. |
| 3 | 0.05 = **10× `ε`**, jadi gerbangnya menguji `c_arm` dan bukan penjaga terminasinya sendiri. Ini yang gagal pada baris 0.00. |
| 4 | Semua angka utama G11 (L4 = 24.275%, 29/40 bertahan, 11/40 hilang) diukur di 0.05. Memilih nilai lain memaksa membangun ulang pembandingnya, dan `p1_state §7.2` melarang mengganti basis pembanding setelah melihat hasil. |

Sapuan penuh `c_arm ∈ {0.00, 0.05, 0.10, 0.15, 0.20}` tetap dilaporkan; **0.00
dilaporkan sebagai baris degenerate**, bukan sebagai default (`p1_g11` mencabut
default `c_arm = 0.0` §A2.3 secara eksplisit).

### A3. PALANG — apa yang harus lulus SEBELUM `Δ_full` boleh disebut

Setara `p1_g10 §A3-K2` dan `p1_g11 §A3-K1`. **Semua wajib lulus.**

| | Apa yang diadu | Independen dalam hal apa |
|---|---|---|
| **M0** | **TRANSPARANSI.** Solver bertambal dengan uji lengan **dimatikan** (`c_arm = −∞`) vs `solve_coupled2` apa adanya: makespan, route, `proved`, `n_nodes` **identik** pada 40 S1 + 31 bahan bakar W2 | membuktikan tambalannya tidak mengubah pencarian — gerbang-yang-menyala untuk mekanisme suntikannya sendiri |
| **M1** | **SOUNDNESS.** 100% jadwal yang dilaporkan lolos `validate_coupled` **dan** `sched_arm.arm_schedule_conflict` (gerbang penuh G11, jalan waktu, **bukan** jalur cepat solver) | jalur cepat A2.3 diadu dengan pemeriksa lambat yang tidak berbagi satu baris pun dengannya |
| **M2** | **L6 — dekomposisi = jalan penuh.** (a) `δ` Lemma B dijalankan ulang pada 2376² di setiap `c_arm` sapuan; (b) uji cepat vs `arm_schedule_conflict` pada ≥ 2000 jadwal acak, **0 selisih vonis** | (ii)/(iii) diadu dengan predikat yang L0–L5 sudah buktikan |
| **M3** | **W3 — oracle sadar-lengan.** Enumerator brute force pada instance kecil (`n ∈ {2,3}`, `\|P\| ∈ {3,4}`), 0 kali `solver > W3`, dan **≥ 10 instance yang LENGANnya mengikat** | tidak berbagi satu baris pun dengan pencarian, pola W2 `p1_g10 §K1` |
| **M4** | Lemma 3: `ub ≥ lb` pada **0 pelanggaran**, seluruh S1 + S2 | batas bawah takterkopel tidak menyebut tabrakan sama sekali |
| **M5** | **REGRESI G11.** `Δ_struct` pada `c_clear = 0` mereproduksi `p1_g11 §B6.2`: route `lemma4` 25 / `dive-lb` 5 / `bnb` 10, 39/40 exact, mean ∈ [0, 0.4679] | angka terbit; kalau tidak reproduksi, ada yang berubah dan **itu temuan** |

🔴 **`M3` adalah yang paling mungkin gagal**, dan alasannya bisa disebut di
muka: W2 sudah longgar sebagai batas atas (`max_evade = 1`, `p1_g10 §B9`, 9 dari
31 `solver < W2` **belum** ditutup — utang U4). Menambahkan lengan hanya
menambah cara untuk longgar. **Kalau W3 tidak bisa dibangun cukup ketat, itu
ditulis, dan `Δ_full` dilaporkan sebagai kurungan dengan kualifikasi eksplisit,
bukan sebagai angka.**

### A4. VONIS — aturan dua-sisi `p1_g10 §A3-K3` DIPAKAI ULANG APA ADANYA

Ambang **5.0%** tidak digeser. `Δ_lo = 0` untuk kurungan, `Δ_hi = UB − LB`, mean
atas **seluruh** instance pada kedua ekstrem, vonis sah jika keduanya jatuh di
sisi yang sama dari 5.0%. Instance **dilarang** dibuang. S1 dan S2 **tidak
pernah** dirata-ratakan bersama.

```
Delta_struct = terkopel(BLOCK saja, c_clear = c_arm + delta) - takterkopel
Delta_full   = terkopel(BLOCK dan ARM_BLOCK)                 - takterkopel
Delta_arm    = Delta_full - Delta_struct     <- ANGKA SESI INI, bukan kurungan
                                                selebar serialisasi
```

### A5. YANG DILAPORKAN, TERLEPAS DARI HASILNYA

1. `Δ_struct`, `Δ_full`, `Δ_arm` per set, vonis dua-sisi, S1 dan S2 terpisah.
2. Sapuan `c_arm ∈ {0.00, 0.05, 0.10, 0.15, 0.20}`, dengan 0.00 ditandai degenerate.
3. **Biaya pemasangan** (A2.5) sebagai tabel, diukur **sebelum** sapuan.
4. **Lemma C**: lulus/gagal, dan pada berapa `(p, U)`.
5. Berapa instance berubah **route** karena lengan (`lemma4 → bnb`), dan berapa
   yang anggarannya tersentuh.
6. Jumlah jadwal yang gagal gerbang, per penjadwal. Target **0**.
7. Setiap instance yang masuk kategori **TIDAK ADA JADWAL DITEMUKAN**, dengan
   anggarannya, dan **tidak pernah** disebut "tidak layak".
8. Papan skor §7.2 (A6), dinilai.

### A6. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor: **22 meleset, 7 tepat** (`p1_g11 §B8`). Prior **dunia**: LONGGAR,
tapi D21 memberi contoh tandingan yang **bertahan** pada kelas kendala baru
(eksklusi ruang yang bergantung **penugasan tugas**). Prior **kode sendiri**:
lebih lambat / lebih rumit / lebih salah, **4 dari 5** (meleset pertama di D22),
dan tetap ditulis di sisi **pesimis**.

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D25** | Overhead `arm_feasible_starts` atas yang beku **≥ 5×**, yaitu tepat di batas A2.5 dan mungkin melewatinya | **kode sendiri** | ditulis pesimis dengan sengaja: kasus (iii) tetap jalan-sertifikat, dan empat sesi berturut-turut salah menebak biaya pemeriksa |
| **D26** | **Lemma C LULUS** pada 100% perhentian S1 di `c_arm = 0.05` | dunia | L4 mengukur 75.7% kombinasi `(pose,U)×(pose,U)` **bebas**; menuntut **satu** dari 2376 pose menggantung gagal semua adalah menuntut ekor yang jauh |
| **D27** | `Δ_arm` pada S1 di `c_arm = 0.05` **≤ 2.0%**, dan vonis `Δ_full` **BUKAN MAHAL** | dunia | 29/40 sudah `Δ_arm = 0` terbukti; sisanya punya 75.7% ruang keluar, dan biaya keluar dibatasi ulang-posisi bukan serialisasi |
| **D28** | Lengan mengubah **route** pada **≥ 10 dari 40** S1 (`lemma4`/`dive-lb` → `bnb`) | dunia | 11/40 optimum-struktur kehilangan kelayakan (G11 §B6.2), jadi Lemma 4 tidak bisa lagi menjawabnya |
| **D29** | Uji cepat A2.3 **TIDAK** setuju dengan `arm_schedule_conflict` pada percobaan pertama (M2 gagal sekali sebelum lulus) | **kode sendiri** | tiga dari tiga bug G10 dan satu dari satu G11 ditemukan oleh **uji yang dijalankan**, bukan pembacaan ulang |

### A7. Berkas

| Berkas | Isi |
|---|---|
| `reachability_gng/sched_armfull.py` | context manager suntikan, `arm_feasible_starts`, `arm_serial_ub`, `arm_repair_ub`, `solve_armfull` |
| `test/verify_sched_armfull.py` | M0–M5, termasuk W3 |
| `test/eval_sched_armfull.py` | `Δ_full`/`Δ_arm`, sapuan `c_arm`, Lemma C |
| `docs/p1_g12_armsolve.md` | dokumen ini |

### A8. Urutan kerja, dan apa yang terjadi kalau waktu habis

```
1. U0                                       <- SUDAH SELESAI, commit c49db77
2. BIAYA PEMASANGAN (A2.5)                  <- SEBELUM sapuan apa pun
3. solver + M0 (transparansi)               <- GERBANG: tidak lanjut kalau gagal
4. M1, M2, M4, M5                           <- GERBANG
5. M3 (W3)                                  <- GERBANG untuk kata "exact"
6. Lemma C + Delta_full/Delta_arm S1        <- angka utama
7. S2 + sapuan c_arm
8. U3, U4
```

🔴 **Apa pun yang tidak tercapai dilaporkan sebagai TIDAK DIUKUR, sebagai
kalimat eksplisit di §B.** Dilarang menuliskan "diperkirakan tidak berubah"
untuk sesuatu yang tidak dijalankan.

⚠️ **Rule 6 (anggaran token 30 000/sesi) akan JEBOL lagi dan itu dinyatakan di
muka, bukan dilaporkan sesudahnya.** `p1_g11 §B7` K4.6 sudah mencatat bahwa
bacaan wajibnya sendiri melebihi anggaran per-tugas sebelum satu baris kode
ditulis. **Sesi protokol-panjang P1 dinyatakan sebagai pengecualian eksplisit
terhadap Rule 6**, dan yang dilaporkan sebagai gantinya adalah anggaran yang
benar-benar mengikat: 120 s wall per instance, dan tabel A2.5.

---

## B. Hasil terukur

> §A dikunci 2026-08-15 sebelum `sched_armfull.py` ada. Semua angka di bawah
> keluar sesudahnya. Setiap tempat di mana §B bertentangan dengan §A ditandai
> 🔺. **§A TIDAK ditulis ulang.**

*(diisi sesi ini)*
