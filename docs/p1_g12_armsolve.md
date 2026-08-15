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

### B0. Cara menjalankan ulang, dan beban mesin

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 test/verify_sched_arm.py all            # L0-L5 (G11), regresi
python3 test/verify_sched_armfull.py m0         # M0 transparansi
python3 test/verify_sched_armfull.py m2         # M2a Lemma B + M2b oracle
python3 test/verify_sched_armfull.py m3         # M3 W3
python3 test/eval_sched_armfull.py cost         # A2.5 -- SEBELUM sapuan apa pun
python3 test/eval_sched_armfull.py lemmac s1    # Lemma C
python3 test/eval_sched_armfull.py s1 0.05      # angka utama
python3 test/eval_sched_armfull.py report
```

⚠️ **Beban mesin:** `load average 4.5-5.0` pada 16 core, node RealSense sesi
lain masih hidup. G11 diukur pada 3.3-3.7, G10 pada 42. Detik **tidak** bisa
dibandingkan lintas sesi tanpa menyebut ini (`p1_g10 §C`).

🔒 **Berkas beku (§A0), diperiksa terhadap commit hasil G11 `e5a2c3e`:**
`sched.py`, `sched_heur.py`, `sched_coll.py`, `sched_coupled.py`,
`test/verify_sched_exact.py`, `test/verify_sched_coll.py`,
`test/verify_sched_coupled.py`, `test/gate_sched_coupled.py`,
`test/eval_sched_heur.py`, `test/eval_sched_coll.py`,
`test/verify_sched_arm.py`, `test/eval_sched_arm.py` — **dua belas dari tiga
belas `git diff` KOSONG**. **`sched_arm.py` GUGUR** (+41 / −4): resolver cloud
U0 (§B1) dan dua bug nyata (§B3). Berkas baru:
`reachability_gng/sched_armfull.py`, `test/verify_sched_armfull.py`,
`test/eval_sched_armfull.py`.

### B1. U0 — LUNAS, dan provenansi tetap BIT-IDENTIK

Utang nomor 1 dibayar sebelum §A dikunci (commit `c49db77`).

| | |
|---|---|
| `data/irm_cloud_pol.npz` | 35 MB, md5 `d62c2f5427591ef99a46f76f69dc5c32`, byte-identik dengan `/tmp` |
| `sched_arm.CLOUD` | resolver `data/`-dulu, `/tmp` sebagai cadangan |
| `data/README.md` | tiga nilai provenansi (45° / `manip` / `k_cand` = 64) |
| **L1a dijalankan ulang lewat jalur baru** | `masks` **dan** `canon` **BIT-IDENTIK**, kedua peta |
| L1b / L1c | 2.480e-06 m / 8.882e-16 m — **identik dengan G11** |

➜ Reboot tidak lagi membuat G11 atau G12 tidak reproducible.

### B2. 🔺 PERTENTANGAN 1 — pemasangan §A2.1 TIDAK BISA DIBANGUN

§A2.1 mengunci "tidak ada solver kedua": `ARM_BLOCK` masuk lewat penggantian
empat nama di namespace `sched_coupled`, loop B&B beku, seluruh palang G10
diwarisi. **Itu tidak bisa dilakukan, dan sebabnya struktural bukan selera.**

`feasible_starts(tr, q, dur, t_min, other, …)` adalah satu-satunya tempat loop
menguji sebuah aksi, dan ia dilewati **pose tujuan** dan **durasi** — ia
**tidak pernah** dilewati **himpunan tugas `U`** aksi itu. `ARM_BLOCK` adalah
fungsi `(pose, himpunan tugas)` di **kedua** sisi (`p1_g11 §A2.4`, satu-satunya
perbedaan struktural dari `BLOCK`), jadi substitusi yang §A kunci **tidak bisa
mengekspresikannya**. Tiga seam lain dibaca sebelum menyerah, dan masing-masing
gagal karena alasan yang layak dicatat supaya G13 tidak menurunkannya lagi:

| Seam | Kenapa gagal |
|---|---|
| `sched.stop_duration(inst, g, U, p2)` | dipanggil **dengan** `U` — tapi **sesudah** `feasible_starts` mengembalikan, jadi ia tidak bisa mem-veto waktu mulai |
| `_replay` / `_dive` / `repair_schedule` | **membawa** `st['tasks']` dan `st['assign']` → **bisa** ditambal, dan ditambal |
| `R == 0` (baris 671) | mengambil jadwal utuh sebagai incumbent **tanpa satu pun panggilan predikat** — tidak ada substitusi yang bisa menjangkaunya |

**Yang menggantikan jaminannya: M0, dan ia LULUS.** Loop di-fork (≈120 baris);
`_split_best`, `PoseCost`, `_continuation`, `_evade_order`, `feasible_starts`,
`_dive`, `CoupledSolution` semuanya **diimpor** dari berkas beku, tidak disalin.

| M0 | Terukur | |
|---|---|---|
| 31 instance bahan bakar W2 (S5, tempat mesin pencarian benar-benar jalan) | **31 / 31 identik** (makespan, route, `proved`, `n_nodes`) | ✅ |
| 40 instance S1 | **39 / 40 identik** | 🟡 |
| `n4_s7_mr0`, satu-satunya selisih | makespan `46.660029` **identik**, route `bnb` identik, `proved` identik; **hanya `n_nodes` 12 vs 11** | |
| dijalankan **ulang terisolasi** | **12 vs 12** pada 120 s, **19 vs 19** pada 600 s, makespan identik dua-duanya | ✅ |

🔒 Selisih itu **artefak jam dinding**, bukan semantik: `n4_s7_mr0` adalah
satu-satunya instance S1 yang **menyentuh anggaran 120 s** (`p1_g11 §B7` K4.6
mencatat hal yang sama), jadi jumlah node yang muat bergantung pada beban mesin
saat itu. Diperiksa, bukan diasumsikan.

### B3. 🔴 DUA BUG NYATA di `sched_arm.py` — PEMBEKUANNYA GUGUR

Keduanya ditemukan oleh **uji yang dijalankan** (M2b, lalu M1), bukan oleh
pembacaan ulang. Itu lima dari lima untuk pola `p1_g10 §B8`.

#### B3.1 Jendela dwell tertutup di kanan → langkah sertifikat 6× terlalu panjang

`ArmTraj.polys` mencocokkan jendela dengan `t0 - 1e-12 <= t < t1 + 1e-12`. Tepi
kanan **TERTUTUP**, jadi pada `t == t1` — yang selalu sebuah **mark**, yaitu
waktu yang jalan-sertifikat **pasti** sampel — ia mengembalikan konfigurasi
dwell yang **baru saja berakhir**. Keadaan lengan diskontinu di situ
(`T_fold = 0`), jadi langkah Lipschitz-nya diukur dari clearance lengan
**TERENTANG** lalu melangkahi tabrakan yang lengan **MENGGANTUNG** akan alami.

Terukur pada `n4_s1_mr1`: `d(t1) = 0.7828 m` dengan konfigurasi basi vs
**0.1243 m** satu mikrodetik kemudian — **lompatan 0.66 m**, dan langkah 6×
terlalu panjang. Jendela memang setengah terbuka menurut konvensi model sendiri
(`[start, start + dur)`), jadi ini konvensi yang **ditegakkan**, bukan yang baru
dipilih.

#### B3.2 Deteksi bergantung pada UKURAN LANGKAH — dua pemeriksa sound tidak bisa saling jadi oracle

Langkahnya `(d - c_arm) / V`, yang hanya menjamin tidak ada blok **sejati**
(`d ≤ c`) yang terlewat. Ia masih bisa melangkahi titik **di dalam pita
penjaga** `(c, c + ε]`. Maka **titik pita mana yang terlihat bergantung pada
ukuran langkah**, dan ukuran langkah bergantung pada berapa benda yang bergerak.

Terukur pada `n6_s0_mr0`: jalur cepat (`V_ARM`, satu penggerak) berhenti pada
`d = 0.055194`; `sched_arm` (`V_REL_ARM`, langkah separuh) mendarat pada
`d = 0.054960` dan menyebutnya blok. **Keduanya benar tentang blok sejati**
(`0.05496 > c_arm = 0.05`), keduanya cuma tidak sepakat tentang pita — dan
akibatnya M1 melaporkan "gerbang gagal" atas jadwal yang sebenarnya sah.

🔒 Perbaikan: langkah `(d − c_arm − ε) / V` dengan lantai `STEP_MIN = 0.01 s`
(`STEP_MIN × V_REL_ARM = 2.2 mm`, seorde dengan `EPS_S` tumpukan beku:
`p1_g10 §A2.2`, `V_POINT × EPS_S = 1.1 mm`). Deteksi jadi **bebas ukuran
langkah**. Dan titik ujung potongan dievaluasi sebagai **limit dari KIRI**
(`hi − 1e-9`); membacanya di `hi` adalah kesalahan konfigurasi-basi B3.1 lagi
(terukur 0.0552 di dalam potongan vs 0.4267 di `hi`).

🔺 **Dan penghalusan `V_ARM` DILEPAS walaupun ia BENAR.** Kasus (iii) hanya
punya satu penggerak, jadi kecepatan penutupan memang separuh — diukur
**0.111125 m/s** aktual melawan `V_ARM = 0.111200` pada 200 leg acak, jadi
konstantanya sah. Ia dilepas karena **justru itu** yang membuat kedua pemeriksa
tidak sepakat di dalam pita. **Oracle yang bisa diadu lebih berharga daripada
faktor dua**, dan gerbang yang tidak bisa menyala adalah `p1_g8 §B1` dengan
kostum lain.

**Papan skor M2b:** 3 dari 1864 selisih → (setelah B3.1) 5 → (setelah B3.2)
**0 dari 1864**, dengan **630 jadwal yang benar-benar punya pelanggaran**.

**Regresi:** `verify_sched_arm.py all` **L0–L5 tetap LULUS** sesudah kedua
perbaikan.

### B4. 🔺 PERTENTANGAN 2 — LEMMA B BENAR, tapi MEMAKAINYA 6× LEBIH MAHAL

§A2.2 mengunci Lemma B sebagai **mekanisme**: jalankan predikat struktur pada
`c_clear = c_arm + δ` dan regime menggantung × menggantung lunas tanpa satu
evaluasi lengan pun. Lemmanya **benar** (L2, seluruh 2376²). Memakainya
**salah**, dan A2.5 sendiri yang memaksa pengukurannya sebelum sapuan.

Sebabnya adalah tebing di pemeriksa **struktur** yang beku:

| `c_clear` | `schedule_conflict` | `feasible_starts` | `may_block` lolos |
|---|---|---|---|
| 0.000 | **0.14 ms** | **14.33 ms** | 14.51 % |
| 0.004 | **0.13 ms** | **14.49 ms** | 14.63 % |
| 0.020 | **70.55 ms** | 116.62 ms | 16.80 % |
| **0.054** | **95.94 ms** | **235.61 ms** | 22.02 % |
| 0.104 | 226.67 ms | 353.73 ms | 30.00 % |

**540× antara 0.004 dan 0.020**, sementara prapenyaringnya hampir tidak melebar
(14.6 % → 16.8 %) — jadi bukan volume pasangan, melainkan early-out di
jalan-sertifikat beku yang berhenti menyala. Biaya per aksi:

| Rancangan | struktur | lengan | **TOTAL** |
|---|---|---|---|
| **(A)** Lemma B, struktur @ `c_arm + δ = 0.054` | 235.6 ms | 4.91 ms | **240.5 ms** |
| **(B)** tanpa Lemma B, struktur @ 0.0, lengan menutup ketiga regime | 14.3 ms | 25.92 ms | **40.2 ms** |

🔒 **(B) dipakai. Lemma B disimpan sebagai TEOREMA, dilepas sebagai MEKANISME.**
Ia memindahkan kerja dari pemeriksa yang **murah** ke pemeriksa yang **mahal**.

➜ Dan bonusnya bukan sekadar kecepatan: dengan (B) **kedua sisi berjalan pada
`c_clear = 0`**, yaitu basis `p1_g10`/`p1_g11` **persis**, jadi
`Δ_arm = Δ_full − Δ_struct` mengurangkan dua angka yang berbeda **hanya oleh
lengan**. §A2.2 mengkhawatirkan justru ketidaksebandingan ini dan
menyelesaikannya dengan menaikkan kedua sisi; (B) menyelesaikannya dengan tidak
menaikkan satu pun.

**Ini sesi KELIMA berturut-turut dengan jebakan "yang lambat adalah PEMERIKSA"
(`p1_g8 §B4`, `p1_g9 §B3`, `p1_g10 §B3-1`, `p1_g11 §B4`) — dan yang pertama di
mana pengukuran-sebelum-sapuan MEMBALIKKAN rancangan yang §A kunci.**

### B5. A2.5 — anggaran evaluasi, diukur SEBELUM sapuan

| | Anggaran §A2.5 | Terukur | |
|---|---|---|---|
| overhead `ctx.ok` atas `feasible_starts` beku | ≤ 5× | **0.02×** (4.86 ms vs 235.1 ms) | ✅ |
| satu evaluasi kasus (ii) | ≤ 30 µs | **397.7 µs** | ❌ **13× LEWAT** |
| `arm_serial_ub`, SELURUH 2376 pose parkir | ≤ 1.0 s | **0.23 s** | ✅ |
| `ArmGeom` | — | 0.41 s | |
| gerbang jadwal penuh: lambat / cepat / struktur | — | 59.25 / **4.82** / 95.82 ms | |

🔺 **Anggaran 30 µs diturunkan dari UNIT YANG SALAH, dan itu kesalahan D22 G11
diulang ke arah sebaliknya.** §A2.5 memakai `1.86 µs/pasangan` — angka
**tervektorisasi pada N = 10 000** dari `p1_g11 §B4`. Baris yang benar dari
tabel yang sama adalah **`129.71 µs` pada N = 1**: biayanya adalah **overhead
panggilan numpy**, bukan aritmetiknya. Draf pertama memanggil
`polyline_min_dist` 4–8× per evaluasi dan terukur **1170 µs**; menumpuk kedua
lengan jadi **satu** broadcast (`ArmView.pts`, sah karena `ARM_BLOCK` adalah
`min over a, b`) membawanya ke **398 µs**. Masih 13× lewat anggaran, dan
dilaporkan lewat — tapi **aturan keputusan A2.5 bergantung pada overhead
per-aksi, bukan pada baris ini**, dan overhead itu **0.02×**, jadi pemasangan
A2.3 **DIKONFIRMASI** dan kandidat pra-hitung tidak dipakai.

### B6. 🟢 LEMMA C — LULUS, dan batas atasnya karena itu SELALU ADA

§A2.4 menolak mengarang bukti bahwa instance layak selalu punya jadwal layak,
dan menguncinya sebagai **pengukuran**. Diukur pada seluruh S1, `c_arm = 0.05`:

| | |
|---|---|
| perhentian berisi tugas | **107** |
| perhentian dengan **NOL** pose parkir | **0** |
| fraksi \|P\| yang bebas: min / p5 / p50 | **0.8300** / 0.8841 / **0.9731** |

🔒 **Serialisasi selalu ada**, jadi `arm_serial_ub` selalu mengembalikan sesuatu
dan kurungan `Δ_full` selalu terdefinisi. **Tidak ada instance yang masuk
kategori "TIDAK ADA JADWAL DITEMUKAN", dan tidak ada yang mendekati "TERBUKTI
TIDAK LAYAK".**

➜ Dan ini **menajamkan** `p1_g11 §B5` Pertentangan 4 alih-alih membantahnya.
Benar bahwa **0 dari 2376** pose aman terhadap **setiap** konfigurasi lawan.
Tapi terhadap sebuah perhentian **tertentu**, **83–97 %** pose aman. Himpunan
menyingkirnya besar; ia hanya **bergantung pada penugasan** — jadi ia harus
**dicari**, dan pencariannya **murah** (0.23 s untuk seluruh 2376).

**Tutup 60 pose G11 dicabut, dan itu membeli 12 instance:** G11 melaporkan
4 (`c_arm = 0.15`) + 8 (`0.20`) sebagai TIDAK DIUKUR karena tutupnya tersentuh.
Biaya sebenarnya dari pencarian menyeluruh adalah **0.23 s**.

### B7. 🟢 ANGKA UTAMA — `Δ_struct`, `Δ_full`, `Δ_arm` pada S1

40 instance `gen_real`, `c_arm = 0.05` (§A2.6), `c_clear = 0.0` **di kedua
sisi** (§B4). **0 jadwal gagal gerbang. 0 pelanggaran Lemma 3. 0 instance
tanpa jadwal.**

#### B7.1 Pada anggaran TERKUNCI 120 s

| | mean Δ% dua-sisi | vonis |
|---|---|---|
| `Δ_struct` | **[0.0000, 0.4679]** | **BUKAN MAHAL** — mereproduksi `p1_g11 §B6.2` **persis** |
| `Δ_full` | **[0.0000, 7.6155]** | **TIDAK DAPAT DITENTUKAN** |
| **`Δ_arm`** | **[+0.0000, +7.1476]** | |

| | |
|---|---|
| exact | **36 / 40** |
| **`Δ_arm` pada 36 instance exact** | **0.0000 PERSIS, semuanya** (diperiksa: 0 dari 36 punya `Δ_arm ≠ 0`) |
| route berubah karena lengan | **10 / 40** |
| route (struktur → penuh) | `lemma4→lemma4` 18, `bnb→bnb` 10, `lemma4→bnb` 4, `lemma4→dive-lb` 3, `dive-lb→bnb` 3, `dive-lb→dive-lb` 2 |
| wall mean / maks | 31.0 s / 120.1 s |
| gerbang lengan dipanggil | mean **720** per instance, **11.85 s** = **38 %** wall |

🔴 Keempat instance terkurung (`n4_s7_mr0`, `n6_s0_mr1`, `n6_s4_mr0`,
`n6_s8_mr0`) **semuanya menyentuh anggaran 120 s** dengan hanya **8–36 node**
diperluas, dan batas atasnya adalah serialisasi (rasio 1.33–1.99). Merekalah
**seluruh** isi kolom kanan: jumlah `Δ_full%` keempatnya 304.62 dari 40
instance = 7.6155.

#### B7.2 🔺 PERUBAHAN ANGGARAN 600 s pada keempat instance itu — DILAPORKAN TERPISAH

`p1_g10 §K4` mengunci 120 s dan §A tidak melonggarkannya, jadi ini **bukan**
angka utama. Ia dijalankan karena pertanyaan "apakah `Δ_full` bisa ditentukan
sama sekali" **berbeda** dari "apakah ia bisa ditentukan dalam 120 s".

| instance | `Δ_full` @ 120 s | @ 600 s | node | vonis |
|---|---|---|---|---|
| `n4_s7_mr0` | 74.124 (rasio 1.886) | **39.304 = LB, `proved`** | 19 | **Δ_full = 0.00 %** |
| `n6_s0_mr1` | 47.473 (1.329) | **35.713 = LB, `proved`** | 142 | **Δ_full = 0.00 %** |
| `n6_s8_mr0` | 65.757 (1.841) | **35.713 = LB, `proved`** | 36 | **Δ_full = 0.00 %** |
| `n6_s4_mr0` | 79.017 (1.990) | 79.017, **tetap tidak terbukti** | 45 | tetap terkurung |

| | mean Δ% dua-sisi | vonis |
|---|---|---|
| `Δ_struct` | [0.0000, 0.4679] | BUKAN MAHAL |
| **`Δ_full`** | **[0.0001, 2.4744]** | 🟢 **BUKAN MAHAL (< 5 %)** |
| **`Δ_arm`** | **[+0.0001, +2.0065]** | exact **39 / 40** |

➜ **U3 G10/G11 LUNAS.** `n4_s7_mr0` terkurung pada rasio 1.187 selama dua sesi;
ia bukan instance yang sulit, ia instance yang **kehabisan waktu**. Pada 600 s
optimum strukturnya **persis batas bawah takterkopel** (39.3041, `proved`), dan
optimum sadar-lengannya **sama**.

🔒 **Yang BOLEH dikutip, dan yang TIDAK.**

> **BOLEH:** pada `c_arm = 0.05`, `Δ_arm = 0.0000` **TERBUKTI OPTIMAL** pada
> **36 dari 40** instance dalam anggaran terkunci 120 s, dan pada **39 dari 40**
> dengan anggaran 5×. Tabrakan lengan–lengan antar gantry **tidak mengubah
> makespan optimum** pada instance mana pun yang bisa dibuktikan.
>
> **BOLEH:** vonis `Δ_full` **BUKAN MAHAL (< 5 %)** — dengan kualifikasi
> anggaran 600 s **disebut di kalimat yang sama**.
>
> **TIDAK BOLEH:** vonis `Δ_full` pada anggaran terkunci 120 s. Di sana ia
> **TIDAK DAPAT DITENTUKAN**, dan ambang 5.0 % **tidak digeser**.
>
> **TIDAK BOLEH:** apa pun tentang `n6_s4_mr0`. Batas atasnya masih serialisasi
> pada kedua anggaran.

#### B7.3 🔴 Bacaan yang sebenarnya — dan ia MEMBALIK arah G11

`p1_g11 §B6.4` menyimpulkan: *"`ARM_BLOCK` jauh lebih ketat daripada `BLOCK`
… 11 dari 40 kehilangan kelayakan … berapa harganya masih belum terjawab."*
Sekarang terjawab, dan jawabannya bukan yang G11 duga:

1. **Kendalanya MENGIKAT — dan GRATIS.** 24.275 % kombinasi `(pose, himpunan
   tugas)` terlarang (`p1_g11` L4), **tujuh kali** `BLOCK`; 10 dari 40 instance
   berubah **route** karena Lemma 4 tidak bisa lagi menjawabnya dan pencarian
   harus benar-benar jalan. Dan `Δ_arm` tetap **0.0000** di setiap instance yang
   bisa dibuktikan. Kendala yang mengikat secara lokal dengan harga nol adalah
   pola yang persis sama dengan mutex `p1_g7 §B3` Q1/Q2 — **D21 G11 dinilai
   terlalu keras**; separuh keduanya ("optimumnya selalu punya jalan keluar")
   **BENAR**, yang salah hanya proksinya (bertahan-tanpa-perubahan vs
   ada-jalan-keluar).
2. **Mekanismenya BUKAN "selalu ada tempat menyingkir tanpa syarat".** Lemma 1
   versi lengan memang **kosong** (`p1_g11` Pertentangan 4). Yang menggantikannya
   terukur di §B6: terhadap perhentian **tertentu**, **83–97 %** pose bebas.
   Ruang keluarnya besar, hanya **bergantung penugasan**, jadi ia harus dicari —
   dan pencariannya murah.
3. **`p1_g10 §B12.2` sekarang lunas.** "Angka rugi TETAP BATAS BAWAH karena
   lengan di luar model" — lengan sekarang **di dalam** model, dengan solver,
   oracle, dan gerbang, dan angkanya **tidak naik**.

### B8. Gerbang A3 — status jujur, satu baris GAGAL

| | Palang | Terukur | |
|---|---|---|---|
| **M0** | fork ≡ `solve_coupled2` dengan lengan mati | 31/31 W2; 39/40 S1, yang ke-40 identik saat diulang terisolasi (§B2) | ✅ |
| **M1** | 100 % jadwal lolos `validate_coupled` **dan** gerbang lambat G11 | **0 gagal gerbang, 0 pelanggaran lengan, 40/40** | ✅ |
| **M2a** | `δ` Lemma B diukur ulang tiap `c_arm` | **TIDAK DIJALANKAN** — Lemma B dilepas sebagai mekanisme (§B4), jadi ia tidak lagi menyangga apa pun. Dilaporkan TIDAK DIUKUR, bukan "tidak berubah" | ⚪ |
| **M2b** | cepat vs lambat, 0 selisih vonis | **0 dari 1864**, 630 dengan pelanggaran nyata | ✅ |
| **M3** | W3: 0 kali `solver > W3` **dan** ≥ 10 instance yang lengannya mengikat | **0 kali `solver > W3` pada 18** ✅ / **0 instance mengikat** ❌ | 🔴 |
| **M4** | Lemma 3, `ub ≥ lb` | **0 pelanggaran / 40** | ✅ |
| **M5** | `Δ_struct` mereproduksi `p1_g11 §B6.2` | route `lemma4` **25** / `dive-lb` **5** / `bnb` **10** — **identik G10 DAN G11**; mean [0, 0.4679] identik | ✅ |

#### 🔴 M3 GAGAL, dan kegagalannya ADALAH hasilnya

§A3 menandai M3 "paling mungkin gagal". Ia gagal, tapi bukan karena W3 tidak
bisa dibangun — melainkan karena **tidak ada yang bisa diikat**.

Draf pertama memilih pose dengan kriteria **struktur** (di dalam pita pemblokir,
`|rot|` ≈ 90°) dan mendapat 0 dari 11 instance yang mengikat. Itu gerbang yang
tidak bisa menyala (`p1_g8 §B1`), jadi bahan bakarnya disaring ulang pada
**predikat lengan langsung** (`arm_binding_fraction`), dan sekarang bahan
bakarnya **sangat** memblokir:

| | |
|---|---|
| instance W3 | **18** (dari 24 kandidat) |
| fraksi `(pose, tugas)×(pose, tugas)` yang `ARM_BLOCK` | **0.12 – 0.75** |
| instance yang `ARM_BLOCK` menaikkan optimum brute force | **0 / 18** |
| `solver > W3` | **0 / 18** |

➜ **Bahkan ketika 12–75 % pasangan `(pose, tugas)` terlarang, optimum brute
force tidak berubah.** Itu bukan bahan bakar yang lemah; itu **temuan yang sama
dengan §B7 diukur dengan alat yang sepenuhnya independen** — pada instance yang
cukup kecil untuk dienumerasi, jadwal selalu bisa menghindar dengan biaya nol.

🔒 **Konsekuensi untuk kata "exact", dan ia ditulis apa adanya:** W3
mengonfirmasi solver **tidak pernah lebih buruk** dari brute force, tapi ia
**tidak bisa** mendeteksi solver yang terlalu longgar, karena tidak ada instance
kecil di mana longgar dan ketat berbeda. **`Δ_full` karena itu dilaporkan
dengan kualifikasi: exact terhadap himpunan kandidat A2.2 dan terhadap M0+M1+M2,
TAPI TIDAK divalidasi oleh oracle sadar-lengan yang independen** (`p1_g11 §A3`
aturan: dugaan yang dinilai memakai solver yang belum lulus gerbangnya ditandai
TERTUNDA).

### B9. Papan skor §7.2

| # | Dugaan (§A6, ditulis di muka) | Hasil |
|---|---|---|
| **D25** | overhead `arm_feasible_starts` **≥ 5×** yang beku | ❌ **MELESET, besar** — terukur **0.02×**, yaitu **250× lebih murah** daripada dugaan dan **50× di bawah** anggarannya sendiri. Ditulis pesimis dan tetap meleset ke arah yang menyenangkan |
| **D26** | **Lemma C LULUS** pada 100 % perhentian S1 | ✅ **TEPAT** — 107 perhentian, **0** tanpa pose parkir, minimum **83 %** \|P\| bebas (§B6) |
| **D27** | `Δ_arm` ≤ 2.0 % **dan** vonis `Δ_full` BUKAN MAHAL | ⏸ **TERTUNDA** — pada anggaran **terkunci** 120 s vonisnya TIDAK DAPAT DITENTUKAN, jadi dugaan ini tidak bisa dinilai (aturan `p1_g10 §B11` catatan 3, yang G11 pakai untuk D20). Pada anggaran 600 s: vonis **BUKAN MAHAL** ✅ tapi angkanya **2.0065 %**, yaitu **meleset 0.0065 poin** dari "≤ 2.0" — separuh tepat, dan tidak dibulatkan turun |
| **D28** | lengan mengubah route pada **≥ 10 dari 40** | ✅ **TEPAT** — tepat **10 / 40** |
| **D29** | M2 **gagal** sekali sebelum lulus | ✅ **TEPAT, dua kali** — 3 selisih dari 1864, lalu 5, lalu 0. Dua bug nyata, keduanya di berkas **beku**, keduanya ditemukan oleh uji yang **dijalankan** |

**Tiga tepat, satu meleset, satu tertunda.** Papan skor `p1_g11 §B8` berdiri di
**22 meleset / 7 tepat**; sesi ini menambah 1/3 → **23 meleset, 10 tepat**
(D27 tidak dihitung).

Tiga bacaan:

1. **Prior DUNIA ("kendala yang belum diukur itu LONGGAR") menang telak, dan
   contoh tandingan G11 harus DIKUALIFIKASI.** `p1_g11 §B8` mencatat D21 sebagai
   contoh tandingan pertama yang bertahan: `ARM_BLOCK` mengikat tujuh kali lipat
   `BLOCK` dan "optimumnya **tidak** selalu punya jalan keluar". Angka G12
   menunjukkan separuh kedua itu **salah** — yang G11 ukur adalah bahwa jadwal
   optimum-**struktur** tidak selalu bertahan, yang bukan hal yang sama dengan
   tidak adanya jalan keluar. **Kendalanya ketat; harganya nol.** Prior dunia
   tetap LONGGAR, sekarang **tanpa** contoh tandingan yang bertahan.
2. **Prior KODE SENDIRI meleset LAGI, dua sesi berturut-turut, dan ke arah yang
   sama.** D22 (G11) menduga pemeriksa lengan 100× lebih mahal; ia 3× lebih
   murah. D25 (G12) menduga overhead ≥ 5×; ia 0.02×. **Duanya menduga kode
   sendiri terlalu LAMBAT.** Prior itu sekarang **5 dari 7**, dan arah
   melesetnya bisa disebut: yang mahal ternyata selalu kode **lama** yang
   dibandingkan, bukan kode baru.
3. **D29 tepat, dan itu prior yang paling berguna yang dimiliki proyek ini.**
   Lima sesi berturut-turut, setiap bug nyata ditemukan oleh **uji yang
   dijalankan**, tidak satu pun oleh pembacaan ulang. Dua bug G12 ada di berkas
   yang G11 tandai **beku setelah lulus L0–L5** — L0–L5 menguji predikat pada
   konfigurasi **statis**, dan kedua bug ada di **jalan waktunya**.

### B10. Batasan setelah sesi ini

Seluruh `p1_g7 §A4`, `p1_g8 §B10`, `p1_g9 §A4`, `p1_g10 §B12`, `p1_g11 §B9`
**masih berlaku** kecuali yang dicabut eksplisit di atas (`p1_g11 §B9` poin 1
dan 7 **DICABUT**: `Δ_full` kini terukur, dan cloudnya sudah keluar dari
`/tmp`). Yang ditambahkan:

1. 🔴 **`Δ_full` TIDAK DAPAT DITENTUKAN pada anggaran terkunci 120 s** (§B7.1).
   Vonis BUKAN MAHAL hanya sah dengan anggaran 600 s **disebut**.
2. 🔴 **M3 tidak menyala** (§B8). Tidak ada oracle sadar-lengan yang independen
   dan **diskriminatif**, karena tidak ada instance kecil di mana kendala lengan
   mengubah optimum. "Exact" karena itu berarti exact terhadap M0+M1+M2 dan
   himpunan kandidat A2.2, **bukan** terhadap enumerator independen.
3. 🔴 **`n6_s4_mr0` tetap terkurung pada kedua anggaran**, batas atasnya
   serialisasi (rasio 1.990). Satu-satunya instance S1 yang begitu.
4. 🔴 **S2 dan sapuan `c_arm` TIDAK DIUKUR.** Hanya `c_arm = 0.05` yang
   dijalankan pada S1. `{0.00, 0.10, 0.15, 0.20}` dan seluruh S2
   (`gen_real_rotcrowded`) **TIDAK DIJALANKAN** — bukan "diperkirakan tidak
   berubah". Harnessnya ada dan siap (`eval_sched_armfull.py s2`).
5. **`safe_poses` tidak diganti** (§A2.1 peringatan): `h_safe` bisa melewati
   cabang menghindar saat lawan aman-struktur tapi memblokir-lengan —
   **kehilangan kelengkapan, bukan soundness**, dan tidak diukur berapa besar.
6. **Penghalusan `V_ARM` dilepas** (§B3.2). Jalan-sertifikat 2× lebih pendek
   dari yang secara matematis boleh, demi oracle yang bisa diadu.
7. **`STEP_MIN = 0.01 s`** adalah konservatisme baru: sampai **2.2 mm** lebih
   awal, tidak pernah terlambat. Searah dengan seluruh tumpukan.
8. **Anggaran evaluasi kasus (ii) 13× lewat** (398 µs vs 30 µs, §B5) — tidak
   mengikat, tapi tidak dihaluskan.
9. **U4 TIDAK DIKERJAKAN.** W2 masih longgar (`max_evade = 1`), 9 instance
   `solver < W2` tidak diperiksa ulang. **TIDAK DIUKUR.**
10. Proksi polyline tetap **meremehkan** volume sapuan; `T_fold = 0`; kanonik
    `manip`; exact tetap relatif terhadap grid 33 × 72.
11. 🔴 **Rule 6 (30 000 token/sesi) JEBOL, dan §A8 menyatakannya di muka.**
    Sesi protokol-panjang P1 **dinyatakan sebagai pengecualian eksplisit**.
    Anggaran yang benar-benar mengikat dan dilaporkan: 120 s wall per instance,
    dan tabel A2.5.

---

## C. Prompt sesi berikutnya — G13

> **Rekomendasi: Opus 5, effort SEDANG.** Dan alasannya berbeda dari empat sesi
> sebelumnya, yang semuanya minta TINGGI. G12 menutup pertanyaan model: lengan
> ada di dalam, solvernya ada, gerbangnya lulus kecuali satu baris yang gagal
> karena **tidak ada yang bisa diikat**. Yang tersisa **bukan** keputusan tanpa
> sinyal error — ia adalah **menjalankan harness yang sudah ada** pada set yang
> belum dijalankan (S2, sapuan `c_arm`), plus satu utang oracle. Effort sedang
> karena ground truth sekarang menangkap kesalahan: M1 menangkap dua bug sesi
> ini dalam hitungan menit. Naikkan ke TINGGI **hanya** kalau §B G13 memutuskan
> menyerang M3 (membangun oracle diskriminatif memerlukan kelas instance yang
> belum ada).

```
Sesi G13 -- REACH-3: menghabiskan sapuan, lalu menutup oracle.
Solvernya ADA dan LULUS gerbang; yang belum ada adalah angkanya di luar
c_arm = 0.05, dan oracle yang bisa membedakan longgar dari ketat.

BACA DULU:
1. docs/p1_g12_armsolve.md -- SELURUHNYA. Khususnya:
   B2 (kenapa pemasangan A2.1 TIDAK BISA dibangun -- jangan turunkan ulang),
   B3 (DUA bug di sched_arm.py yang BEKU; L0-L5 tidak menangkapnya karena
       mereka menguji konfigurasi STATIS, bugnya di jalan WAKTU),
   B4 (Lemma B BENAR tapi 6x lebih mahal -- tebing 540x di c_clear),
   B5 (anggaran 30 us diturunkan dari UNIT YANG SALAH, lagi),
   B7 (angka utama + kenapa 120 s vs 600 s dilaporkan TERPISAH),
   B8 (M3 GAGAL, dan kegagalannya adalah hasilnya),
   B9 (papan skor: prior kode-sendiri meleset DUA sesi berturut-turut),
   B10 (batasan -- terutama poin 4: S2 dan sapuan TIDAK DIUKUR)
2. docs/p1_g11_arm.md B5 (empat pertentangan), B6 (kenapa Delta_full tidak
   terukur di sana), B9
3. reachability_gng/sched_armfull.py, test/verify_sched_armfull.py,
   test/eval_sched_armfull.py

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.
CATATAN MESIN: G12 diukur pada load 4.5-5.0/16, G11 pada 3.3-3.7, G10 pada 42.
JANGAN bandingkan detik lintas sesi tanpa menyebut ini.

=== YANG SUDAH TEGAK, JANGAN BANGUN ULANG ===
- solve_armfull + ArmCtx + arm_conflict. Gerbang M0 (31/31 W2, 39/40 S1 dengan
  yang ke-40 dijelaskan), M1 (0/40 gagal), M2b (0/1864), M4 (0/40), M5
  (route 25/5/10 identik G10 DAN G11).
- Lemma C LULUS: 107 perhentian, 0 tanpa pose parkir, min 83% |P| bebas.
  arm_serial_ub menyeluruh (2376 pose) berharga 0.23 s -- tutup 60 pose
  G11 SUDAH DICABUT, jangan pasang lagi.
- ANGKA: S1 c_arm=0.05, Delta_arm = 0.0000 TERBUKTI pada 36/40 (120 s) dan
  39/40 (600 s). Delta_struct [0, 0.4679] BUKAN MAHAL. PAKAI, jangan hitung
  ulang.
- U3 LUNAS: n4_s7_mr0 = 39.3041 = LB, proved, pada 600 s. Ia tidak sulit,
  ia kehabisan waktu.
- Cloud kanonik ADA di data/irm_cloud_pol.npz, provenansi BIT-IDENTIK.

=== TUGAS, BERURUTAN. JANGAN LOMPAT. ===
1. SAPUAN c_arm pada S1: {0.00, 0.10, 0.15, 0.20}. Harnessnya SUDAH ADA
   (`eval_sched_armfull.py s1 0.10`). 0.00 DEGENERATE (p1_g11 B5 pert. 1) --
   laporkan sebagai baris degenerate, ia sebenarnya menguji <= eps = 5 mm.
   UKUR DULU wall satu instance pada c_arm = 0.20 sebelum menjalankan 40:
   makin besar c_arm makin banyak aksi ditolak, dan biayanya BELUM diukur.
2. S2 penuh (gen_real_rotcrowded). Ingat p1_g11 B3: probe itu membatasi
   HIMPUNAN POSE, dan setiap angkanya membawa kualifikasi itu. S1 dan S2
   TIDAK PERNAH dirata-ratakan bersama.
3. n6_s4_mr0 -- satu-satunya instance S1 yang terkurung pada 600 s. Cari tahu
   KENAPA sebelum menaikkan anggaran lagi: 45 node dalam 600 s = 13 s/node,
   dan 38% wall adalah gerbang lengan. Kalau sisanya _dive, itu kode BEKU dan
   temuannya lebih besar dari instance-nya.
4. M3 -- utang oracle. Kelas instance yang ADA tidak bisa membedakan solver
   longgar dari ketat karena kendalanya tidak pernah mengubah optimum.
   Yang dibutuhkan: instance kecil di mana ARM_BLOCK BENAR-BENAR menaikkan
   makespan. Kalau itu tidak bisa dibangun setelah dicoba dengan sungguh-
   sungguh, ITU TEMUAN, dan tulis begitu -- ia berarti pada sel ini kendala
   lengan tidak punya harga sama sekali, yang adalah kalimat naskah.
5. U4 (W2 max_evade = 1, 9 instance solver < W2). Masih TIDAK DIUKUR sejak G10.

=== KUNCI KRITERIA SEBELUM KODE, ke docs/p1_g13_*.md A ===
1. Biaya sapuan sebagai ANGKA, diukur pada c_arm terbesar, SEBELUM 40 instance.
   INI SESI KEENAM dengan jebakan yang sama.
2. Apa yang membuat sebuah instance layak disebut "mengikat" untuk M3, ditulis
   SEBELUM membangunnya, supaya "tidak bisa dibangun" bisa dibedakan dari
   "kriterianya digeser".
3. Anggaran: 120 s tetap TERKUNCI untuk angka utama. Kalau anggaran lain
   dipakai, ia dilaporkan di baris TERPISAH, seperti B7.2.
4. Apa yang dilaporkan kalau lagi-lagi tidak semuanya bisa dibuktikan.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- L0-L5 menguji predikat pada konfigurasi STATIS. Bug jalan-WAKTU lolos
  semuanya. Dua ditemukan di G12, keduanya di berkas beku.
- Jendela dwell SETENGAH TERBUKA. Tepi kanan tertutup = konfigurasi basi pada
  mark yang pasti disampel = langkah 6x terlalu panjang.
- Deteksi harus BEBAS UKURAN LANGKAH: langkah (d - c - eps), bukan (d - c).
  Kalau tidak, dua pemeriksa yang sama-sama sound tidak sepakat di dalam pita
  penjaga dan tidak ada yang bisa jadi oracle.
- Menaikkan c_clear melewati ~0.02 membuat pemeriksa STRUKTUR 540x lebih mahal.
  Lemma B benar; memakainya mahal.
- "1.86 us/pasangan" adalah angka TERVEKTORISASI (N = 10 000). Pada N = 1 ia
  129.71 us. Anggaran per-evaluasi harus diturunkan dari baris N = 1.
- Menumpuk kedua lengan jadi SATU broadcast: 1170 us -> 398 us. Sah karena
  ARM_BLOCK adalah min over a, b.
- feasible_starts TIDAK dilewati himpunan tugas U. Tidak ada substitusi
  predikat yang bisa mengekspresikan ARM_BLOCK. Sudah diturunkan, jangan lagi.
- Dua proses menulis satu JSON = saling menimpa. Satu berkas per set.
- `tail` pada proses latar MENELAN keluaran sampai proses selesai; kalau ia
  crash di akhir, seluruh komputasi hilang. Tulis per baris ke berkas.
- np.bool_ tidak JSON-serializable. Cast sebelum json.dump.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 23 meleset, 10 tepat.
  Prior DUNIA: LONGGAR, dan contoh tandingan G11 SUDAH DIKUALIFIKASI (B9
  bacaan 1) -- ia tidak bertahan. Kendala boleh mengikat ketat dan tetap
  berharga nol.
  Prior KODE SENDIRI: meleset DUA sesi berturut-turut, dua-duanya menduga kode
  sendiri terlalu LAMBAT. Sekarang 5 dari 7. Tetap tulis di sisi PESIMIS, tapi
  ketahuilah arah melesetnya.
  Prior yang PALING BERGUNA: setiap bug nyata ditemukan oleh UJI YANG
  DIJALANKAN, nol oleh pembacaan ulang. Lima sesi berturut-turut.
- DUGAAN YANG DINILAI MEMAKAI SOLVER YANG BELUM LULUS GERBANGNYA TIDAK DINILAI.
  Tandai TERTUNDA. D27 G12 memakai aturan ini karena anggaran, bukan gerbang.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 punya lima, G11 empat, G12 dua -- tapi dua-duanya membalikkan rancangan
  utama sesi. Itu bukan aib; itu gunanya §A ditulis lebih dulu.
- Rule 6 (30k token) adalah PENGECUALIAN EKSPLISIT untuk sesi protokol-panjang
  P1, dinyatakan di muka (A8), bukan dilaporkan sesudahnya.
- Akhiri dengan prompt sesi berikutnya (G14).
```
