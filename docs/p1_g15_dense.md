# P1 / G15 — REACH-5: KEPADATAN TUGAS, kandidat terakhir yang berdiri.

> Sesi G15, 2026-08-16. Melanjutkan [p1_g14_reach4.md](p1_g14_reach4.md).
> G14 menghabiskan **ruang pose** sebagai penjelasan: 432 bukti `Δ_arm = 0` di
> S1 (285 sesi G14 + 147 G13), nol contoh tandingan, melintasi `|P|` 2376 → 148
> **dan** melintasi masker pose S2 sendiri. Satu-satunya harga positif yang
> pernah terbukti tetap satu: `n6_s0_mr1` di S2, +5.40 %.
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.** §B
> diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**. §A tidak ditulis ulang belakangan.
>
> Sesi ini **sepenuhnya offline**. Lengan 4× masih dilepas.
>
> **CATATAN MESIN, di muka:** `load average` awal sesi **4.70 / 3.50 / 4.03**
> pada 16 core. G14 diukur pada 4.4–7.2, G13 pada 6–8.5, G12 pada 4.5–5.0,
> G11 pada 3.3–3.7, G10 pada 42. Detik **tidak** dibandingkan lintas sesi tanpa
> menyebut ini.
>
> ⚠️ **Ruang disk:** root 1.9 T pada **100 % (16 G tersisa)** saat sesi mulai.
> Artefak sesi ini ~20 KB per sel, jadi tidak mengikat — tapi setiap kegagalan
> tulis JSON harus dibaca sebagai kemungkinan disk penuh, bukan bug.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang, dan yang BEKU

Seluruh `p1_g12 §A0`, `p1_g13 §A0`, `p1_g14 §A0` berlaku utuh. Yang ditambahkan
sebagai **sudah tegak** dan **tidak dihitung ulang**:

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| **U4 LUNAS**, 9/9 keberangkatan solver di luar grid W2 | `p1_g14 §B5` | **TUTUP.** Tidak dibuka, tidak diuji ulang |
| Anggaran = **anggaran PENCARIAN** 120 s; wall **tidak dibatasi**, dilaporkan terpisah (K1–K4) | `p1_g14 §B1` | dipakai apa adanya; opsi (a) **tidak dibuka ulang** (harganya ≈ 1334 instance-run, sudah dihitung) |
| Lemma C **BUKAN** prapenyaring — 79 % penolakan di luar jangkauannya | `p1_g14 §B4` | `sched_armfull.py` **TETAP BEKU**; M0 **tidak** perlu diulang |
| Tangga `\|P\|`: 12 sel, 480 instance-run, 0 kegagalan gerbang, **0 positif** | `p1_g14 §B7` + `/tmp/g14_psweep_*.json` | **dibaca**, tidak dihitung ulang |
| Sapuan G13 10 sel (S1+S2 × `c_arm` 0.00–0.20, 40 instance/sel) | `p1_g13 §B5` + `/tmp/g13_eval_*.json` | **basis pembanding berpasangan-kunci** (A2.3) |
| `Δ_arm = +0.5000` (+5.40 %) pada `n6_s0_mr1` S2, `c_arm` 0.15 **dan** 0.20 | `p1_g13 §B5.4` | **satu-satunya** contoh tandingan; objek sesi ini |
| M3 LULUS dan DISKRIMINATIF; `brute_arm` dan `arm_first_block` sudah diperbaiki | `p1_g13 §B3`, `§B4`, `§B6` | oracle dipakai apa adanya |
| `\|P\|` S1 = **2376**, `\|P\|` S2 = **507** | artefak G13 | angka pangkal |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi** (17 berkas):
`sched.py`, `sched_heur.py`, `sched_coll.py`, `sched_coupled.py`,
`sched_arm.py`, `sched_armfull.py`, `test/verify_sched_exact.py`,
`test/verify_sched_coll.py`, `test/verify_sched_coupled.py`,
`test/gate_sched_coupled.py`, `test/verify_sched_arm.py`,
`test/eval_sched_arm.py`, `test/verify_sched_armfull.py`,
`test/eval_sched_armfull.py`, `test/m3_bind_g13.py`, `test/diag_g13.py`,
**`test/psweep_g14.py`**.

⚠️ `psweep_g14.py` **masuk daftar beku sesi ini**. Tugas 1 memakai ulang
`shrink` / `build` / `report` / `verdict` / `_exact` **dengan mengimpornya**,
bukan dengan menyalin atau menulis ulang — prompt G15 menuntut itu eksplisit,
dan `p1_g13 §B3` sudah mengukur harganya: **instrumen kedua dari BENTUK yang
sama mewarisi cacat yang sama.**

Aturan `p1_g10 §A0` berlaku utuh: **bug di berkas beku dilaporkan sebagai
temuan, diperbaiki di tempatnya, pembekuannya dinyatakan gugur.** G10 tiga kali,
G11 nol, G12 dua, G13 dua, G14 nol (dua **temuan anggaran**, bukan bug berkas).

### A1. Yang dikerjakan sesi ini, BERURUTAN

Urutannya wajib dan **tidak dilompati**:

```
1. GENERATOR KEPADATAN TUGAS   -- gen_real_dense, R3 gaya A2.2
2. UKUR Delta_arm di sel itu   -- c_arm {0.15, 0.20}, biaya DIUKUR DULU
3. TANGGA PEMISAH / VONIS      -- pita kolam tugas, atau kalimat INTERAKSI
4. AUDIT TITIK PEMERIKSAAN ANGGARAN
5. LANTAI USAHA M3             -- kalau ada waktu
```

**TIDAK dibangun, dan tidak boleh menyelinap masuk:** solver ketiga, heuristik
sadar-lengan, perencanaan gerak lengan, `T_fold ≠ 0`, kapsul/ketebalan tautan,
policy kanonik selain `manip`, `c_arm` di luar {0.05, 0.10, 0.15, 0.20},
pelonggaran anggaran **pencarian** 120 s untuk angka utama, perbaikan performa
pada berkas beku, dan **faktor keempat yang dikarang** kalau tugas 2 nihil
(prompt G15 melarangnya eksplisit; A2.4 di bawah mengunci apa yang boleh
ditulis sebagai gantinya).

---

### A2. KRITERIA YANG DIKUNCI SEBELUM KODE

#### A2.0 🔴 CONFOUND KEEMPAT — S1 dan S2 berbeda dalam EMPAT hal, bukan tiga

`p1_g14 §A2.2` menamai tiga: kardinalitas `|P|`, geometri himpunan pose,
kepadatan tugas. **Ada yang keempat, dan ia terbaca langsung dari kedua
generator** (`sched.gen_real:439-440` vs `sched_arm.gen_real_rotcrowded:576`):

| | S1 `gen_real` | S2 `gen_real_rotcrowded` |
|---|---|---|
| `p0` kedua gantry | pose grid terdekat **`(lin = ref.lin[0] = 0.00, rot = 0)`** | pose grid terdekat **`(lin = x_c = 0.80, rot = 0)`** |

Grid rel = `0.00, 0.05, …, 1.60` (33 kolom). Jadi `p0` S1 dan `p0` S2 terpisah
**0.80 m rel = ~25 s traverse** (§5.6 `p1_state`), pada masalah yang
74–92 % makespan-nya adalah gerak gantry (`p1_g7 §B5`). Itu **bukan** perbedaan
kosmetik, dan kedua lengan percobaan G14 membawa `p0` S1 (`psweep_g14.shrink`
mempertahankan `full.p0`) — jadi (4) **tidak pernah diuji** di G14 juga.

🔒 **Konsekuensi yang dikunci sekarang:** percobaan sesi ini punya **DUA
LENGAN**, bukan satu, dengan prioritas eksplisit:

| Lengan | `p0` | Kolam tugas | Ruang pose | Apa yang dijawab |
|---|---|---|---|---|
| **`DENSE`** (prioritas 1) | **S1** (`lin 0.00`) | S2 (`\|x − 0.80\| ≤ 0.30`, tolak kalau bisa di `p0`) | **S1 PENUH, `\|P\| = 2376`** | apakah **kepadatan tugas** sendirian menghasilkan harga |
| **`DENSE-P0XC`** (prioritas 2) | **S2** (`lin 0.80`) | idem | idem | apakah **letak `p0`** ikut diperlukan |

`DENSE` adalah yang prompt G15 minta, dan ia yang dijalankan lebih dulu.
`DENSE-P0XC` dijalankan **hanya kalau aturan biaya A2.5 mengizinkan**; kalau
tidak, ia dilaporkan **TIDAK DIUKUR** dengan namanya — dilarang menulis
"diperkirakan sama".

#### A2.1 🔒 TUGAS 1 — APA YANG MEMBUAT KEPADATAN TUGAS "TERKENDALI"

Kunci kriteria nomor 1 prompt G15. Ditulis **sebelum generatornya ada**.

`gen_real_dense(n, seed, n_mr, x_c = 0.80, band = 0.30, p0_mode)` adalah
`gen_real` dengan **tepat dua** perubahan, dan tidak satu pun yang lain:

1. kandidat tugas ditarik dari `pool = {node : |x_node − x_c| ≤ band}`
   alih-alih dari seluruh `ref.nodes`;
2. kandidat yang **dapat dikerjakan di `p0`** ditolak (`at_p0 → reject`),
   dengan `at_p0` diuji pada **pose `p0` itu sendiri** di grid penuh —
   `hand` untuk tugas MR, `reach` untuk tugas SR, sama persis dengan predikat
   penerimaan `gen_real`.

Ruang pose, peta, `DWELL`, `t_fold`, kanonik, dan struktur `Instance` **tidak
disentuh**. `p0` ditentukan oleh `p0_mode ∈ {'s1', 'xc'}` (A2.0).

**🔒 EMPAT SYARAT R3, diperiksa per instance, pelanggaran apa pun ⟹ instance
itu dilaporkan TIDAK DAPAT DIBANGUN dengan namanya, bukan dibuang diam-diam:**

1. **`p0` dipertahankan** dan ada di himpunan pose (trivial di grid penuh, tetap
   diperiksa dan dilaporkan sebagai kolom).
2. **Semua tugas terjangkau**: setiap tugas SR punya ≥ 1 pose jangkau pada ≥ 1
   gantry; setiap tugas MR punya ≥ 1 pose irisan-ketat. (Predikat penerimaan
   generator sudah menjaminnya; R3 **memverifikasinya ulang di luar generator**,
   karena `p1_g13 §B3` mengukur bahwa instrumen yang memeriksa dirinya sendiri
   mewarisi cacatnya sendiri.)
3. **Deterministik**: `rng = np.random.default_rng(seed)` — **konvensi yang
   sama persis** dengan `gen_real` dan `gen_real_rotcrowded`, supaya kunci
   `(n, seed, mr)` berarti benih yang sama di ketiga generator.
4. **Pergeseran `lb` dilaporkan** — lihat A2.2, yang mengubah cara ambangnya
   dipakai.
5. **Tolakan dilaporkan sebagai angka**: `rejects`, dan berapa fraksi `pool`
   yang tersingkir oleh syarat `at_p0`. `max_reject = 200 000`; melampauinya
   ⟹ TIDAK DAPAT DIBANGUN, disebut namanya.

**🔒 BERAPA TAK-TERBANGUN YANG MEMBATALKAN PERCOBAANNYA — dikunci sekarang:**

| Tak-terbangun dari 40 | Vonis |
|---|---|
| **≤ 20 %** (≤ 8) | sel dijalankan dan dilaporkan sebagai angka utama |
| **20–50 %** | sel **tetap** dijalankan, tetapi setiap kalimatnya membawa kualifikasi jumlah-instance yang sama persis dengan `GEO-S2` 17/40 (`p1_g14 §B7`) |
| **> 50 %** | 🔴 **TEMUAN, bukan kegagalan**: kepadatan tugas dan ruang pose **tidak dapat dipisahkan di sel ini sama sekali**, dan itulah kalimat naskahnya. Sel yang bisa dibangun tetap dijalankan dan dilaporkan, tetapi **dilarang** dikutip sebagai jawaban tugas 2 |

Prompt G15 menuntut kalimat terakhir itu ditulis di muka; ia ditulis di sini.

#### A2.2 🔺 SYARAT `lb` A2.2-G14 TIDAK BERLAKU DI SINI, dan alasannya ditulis sekarang

`p1_g14 §A2.2` syarat 4 mengunci: rung yang menaikkan `lb` median > 10 %
**bukan lagi instance yang sama**, dan positif di sana tidak boleh dikutip. Itu
benar **di G14**, karena kedua lengan G14 memakai **tugas yang identik** dengan
S1 dan hanya membuang pose.

**Sesi ini memakai tugas yang BERBEDA — itu justru knob-nya.** Maka:

🔒 **Ambang `lb` 10 % TIDAK dipakai sebagai palang di sini.** Memakainya akan
membatalkan percobaan berdasarkan efek yang percobaan ini ada untuk menghasilkan.
`lb` tetap **dilaporkan** per sel (median, dan pergeserannya terhadap `lb` S1
kunci yang sama) sebagai **angka deskriptif**, bukan sebagai gerbang.

🔒 **Yang menggantikannya sebagai palang** adalah sifat yang `Δ_arm` memang
punya sejak awal dan yang tidak pernah bergantung pada perbandingan lintas
instance: **`Δ_arm = makespan_full − makespan_struct` dihitung DI DALAM satu
instance yang sama.** Dua sisi, dua solver, satu instance. Ambang 5.0 %
(`p1_g13 §A2.4`) **tidak digeser**.

Ini pertentangan dengan §A G14 yang dinyatakan **di muka**, bukan ditemukan
belakangan — dan ia dinyatakan karena kalau tidak, tugas 1 akan tampak
melanggar palang yang dikunci sesi lalu.

#### A2.3 🔒 TUGAS 2 — APA YANG MEMBEDAKAN "HARGA MUNCUL" DARI "INSTANCE LAIN"

Kunci kriteria nomor 2. Definisi `p1_g14 §A2.2` dipakai **apa adanya, tidak
ditulis ulang, tidak dilonggarkan**:

| | Definisi, dikunci (salinan verbatim A2.2 G14) |
|---|---|
| **HARGA MUNCUL** | ada ≥ 1 instance dengan `Δ_arm > 1e-9` di mana **sisi struktur DAN sisi penuh dua-duanya `exact`/`proved`** |
| **KALIMAT NASKAH** | positif **naik monoton** ketika kepadatan dinaikkan, pada ≥ 3 titik pita berturut-turut (A2.6). Positif tunggal terisolasi **BUKAN** kalimat naskah — ia mendapat kualifikasi satu-instance yang sama persis dengan `p1_g13 §B5.4` |
| **JAWABAN NEGATIF ADALAH JAWABAN** | 0 positif pada `DENSE` di kedua `c_arm` ⟹ **kepadatan tugas sendirian bukan mekanismenya**, ditulis sebagai temuan |
| **DILARANG** | menyebut `Δ_arm` positif ketika sisi **struktur** tidak terbukti — selisih dua batas atas bukan selisih dua optimum |

🔺 **KONTROL "instance yang sama di S1 penuh" YANG PROMPT MINTA TIDAK ADA, dan
itu dinyatakan sekarang.** Prompt G15 tugas 2 menuntut: *"instance yang sama
harus `Δ_arm = 0` terbukti di S1 penuh."* Instance itu **tidak dapat dibangun**:
`DENSE` berbeda dari S1 justru pada **himpunan tugasnya**, jadi "instance yang
sama dengan tugas lain" adalah kontradiksi istilah. Yang ada, dan yang dipakai:

🔒 **KONTROL BERPASANGAN-KUNCI.** Untuk setiap kunci `(n, seed, mr)`, barisnya
di `/tmp/g13_eval_s1_{0.15,0.20}.json` memberi `Δ_arm = 0` **terbukti** pada
`|P| = 2376` dengan tugas `gen_real`. Perbandingannya adalah **kunci lawan
kunci**, bukan instance lawan instance, dan **setiap kalimat yang memakainya
menyebut itu**. Ini pelemahan nyata dibanding rancangan prompt, dan ia ditulis
di §A supaya ia bukan pembelaan belakangan.

#### A2.4 🔒 KALAU TUGAS 2 NIHIL — apa PERSIS yang boleh dan tidak boleh ditulis

Kunci kriteria nomor 3. **Ditulis sebelum hasilnya dilihat.**

**BOLEH ditulis:**

1. "Tidak satu pun dari tiga faktor S2 yang diuji secara terpisah — kardinalitas
   ruang pose (`p1_g14 §B7`, 8 sel), geometri ruang pose (`p1_g14 §B7`, 4 sel),
   kepadatan tugas (sesi ini) — mereproduksi harga koordinasi lengan."
2. "Yang tersisa sebagai penjelasan adalah **INTERAKSI** antara faktor-faktor
   itu: `p1_g14 §B2` sudah mengukur bahwa pose dan tugas S2 **bukan dua knob
   bebas** — masker pose S2 tak-terbangun pada 23 dari 40 instance S1 justru
   karena kolam tugasnya berbeda."
3. Contoh tandingan S2 `n6_s0_mr1` tetap dikutip **persis seperti
   `p1_g13 §B5.4` mengkutipnya**: satu instance, satu sel probe, dengan
   kualifikasi satu-instance dan kualifikasi "S2 adalah probe DESIGN, bukan
   konstanta fisik" (`sched_arm.gen_real_rotcrowded` docstring).
4. Angka nol itu sendiri, dengan penyebutnya: berapa instance terbukti dua sisi.

**DILARANG ditulis:**

1. **Faktor keempat yang dikarang.** Prompt G15 melarangnya eksplisit. Kalau
   ketiganya nihil, yang ditulis adalah "interaksi", bukan kandidat baru yang
   tidak diukur. (⚠️ **`p0` (A2.0) BUKAN pengecualian atas larangan ini** — ia
   bukan karangan, ia terbaca dari kode dan ia **diuji** oleh lengan
   `DENSE-P0XC`. Kalau lengan itu **tidak** dijalankan, `p0` disebut sebagai
   **confound TIDAK DIUKUR**, bukan sebagai penjelasan.)
2. "Kendala lengan tidak punya harga" — 1 contoh tandingan terbukti melarang
   kalimat itu selamanya.
3. "S2 adalah anomali / artefak" tanpa mekanisme terukur. Nol penjelasan bukan
   bukti ketiadaan penjelasan.
4. Merata-ratakan angka `DENSE` ke dalam baris utama S1 atau S2. `DENSE` adalah
   **probe**, pola `p1_g11 §B3`, dan setiap angkanya membawa kualifikasi itu.
5. Mengubah ambang 5.0 %, mengubah definisi `Δ_arm`, atau mengklaim positif dari
   sisi yang tidak terbukti.

#### A2.5 🔒 BIAYA DIUKUR SEBELUM 40 INSTANCE — SESI KEDELAPAN DENGAN JEBAKAN YANG SAMA

`p1_g8 §B4`, `p1_g9 §B3`, `p1_g10 §B3-1`, `p1_g11 §B4`, `p1_g12 §B4`,
`p1_g13 §B2`, `p1_g14 §B3` — **tujuh sesi berturut-turut** di mana yang lambat
ternyata **pemeriksanya**; G14 mengukur gerbang lengan **33–70 % wall** di
keempat probe. Dan `p1_g14 §B3` mengukur bahwa biaya **TIDAK monoton dalam
`|P|`**. Sel ini adalah `|P| = 2376` **penuh** — ujung mahal dari tangga itu.

**Probe:** kunci `n6_s0_mr1` (nama contoh tandingan S2) dan `n6_s4_mr0`
(terburuk diketahui), pada `DENSE`, `c_arm = 0.20`.

🔒 **ATURAN KEPUTUSAN, ditulis sebelum angkanya dilihat:**

| Terukur (wall per instance) | Yang dijalankan |
|---|---|
| kedua probe **< 60 s** | `DENSE` **dan** `DENSE-P0XC`, 40 instance × `c_arm` {0.15, 0.20} = 4 sel |
| probe terburuk **60–300 s** | `DENSE` saja, 40 × {0.15, 0.20} = 2 sel. `DENSE-P0XC` **TIDAK DIUKUR**, disebut namanya |
| probe terburuk **> 300 s** | `DENSE` × `c_arm` **0.20 saja** = 1 sel. `c_arm = 0.15` dan `DENSE-P0XC` **TIDAK DIUKUR**, disebut namanya |
| probe **gagal / tidak mengembalikan jadwal** | **TEMUAN**, ditulis sebagai judul, dan sel berhenti sampai sebabnya diketahui |

🔴 **Instance DILARANG dibuang untuk menghemat waktu.** Yang boleh dikorbankan
adalah **sel utuh**, disebut namanya. (`p1_g14 §A2.2`, dipakai apa adanya.)

#### A2.6 🔒 TUGAS 3 — TANGGA PEMISAH, dikunci sekarang supaya bukan post-hoc

**Kalau tugas 2 POSITIF:** kepadatan adalah kontinum, bukan biner. Pita kolam
tugas dilebarkan dan dicari di mana harganya hilang:

```
band in {0.30, 0.60, 1.00, 2.00}      (2.00 = seluruh peta: nodes x in [0, 2])
c_arm: sel yang memberi positif di tugas 2
```

`band = 2.00` adalah **S1 dengan penolakan `at_p0` saja** — jadi tangganya
adalah kontinum sejati dari `DENSE` ke S1, dan titik di mana harga hilang adalah
**kalimat naskahnya**: *harga koordinasi ditentukan oleh seberapa dipaksa kedua
gantry berbagi wilayah.*

Biaya tangga diukur ulang dengan aturan A2.5 sebelum dijalankan.

**Kalau tugas 2 NIHIL:** tangga **tidak dijalankan** (tidak ada harga untuk
dicari hilangnya), dan yang ditulis adalah A2.4 apa adanya.

#### A2.7 🔒 TUGAS 4 — AUDIT TITIK PEMERIKSAAN ANGGARAN

`p1_g14 §B9.2`: dua anggaran jebol pada hari yang sama, **dua-duanya** karena
diperiksa di tempat yang salah, bukan karena nilainya salah (120 s hanya di
dalam loop B&B; tutup RUN M3 2400 s hanya di antara kandidat, jebol 15 menit).

Untuk **masing-masing** dari `W2_BUDGET`, `EVADE_ATTEMPTS`, `max_states`,
`ACTION_BUDGET`, dilaporkan **tiga kolom**, dan tidak boleh kurang:

1. **Di mana ia diperiksa** — berkas:baris, dan pada level rekursi/loop mana.
2. **Berapa besar satu iterasi bisa melampauinya** — batas atas yang dapat
   dinyatakan, atau **TIDAK TERBATAS** kalau iterasinya sendiri tidak berbatas.
3. **Apakah ada angka yang sudah dikutip yang bergantung padanya.**

🔒 Ini **audit deskriptif**, bukan perbaikan. Berkas beku **tidak disentuh**.
Temuan apa pun dilaporkan sebagai temuan menurut `p1_g10 §A0` dan diputuskan
sesi berikutnya — kecuali ia membatalkan angka sesi ini, yang harus dikatakan.

⚠️ Prior `D29` berlaku: **pembacaan ulang kode punya rekam jejak 0 temuan / 2
dugaan meleset** (`p1_g14 §B8` bacaan 3). Audit ini tetap dijalankan karena ia
**deskriptif** (di mana diperiksa) dan bukan **prediktif** (apa akibatnya) —
pembelahan yang `p1_g14 §B8` bacaan 1 justru buat.

#### A2.8 🔒 TUGAS 5 — LANTAI USAHA M3, kalau ada waktu

`p1_g14 §B9.8`: 7 kelas **TIDAK DIUKUR**, lantai usaha tidak terpenuhi
(2 kelas dari ≥ 5; 28 kandidat dari ≥ 300). Dijalankan **hanya kalau tugas 1–4
selesai**, lewat `m3_bind_g13.py` yang **BEKU**, dengan tutup diperiksa **DI
DALAM `classify()`**, bukan di antara kandidat.

🔺 Menambahkan pemeriksaan tutup di dalam `classify()` **berarti menyentuh
berkas beku.** Maka dikunci: itu dilakukan **hanya** sebagai `wall_cap` yang
diteruskan lewat parameter yang sudah ada, atau **tidak dilakukan sama sekali**
dan tugas 5 dilaporkan **TIDAK DIUKUR**. Berkas beku tidak diedit untuk
kenyamanan.

#### A2.9 🔒 KONKURENSI DAN BEBAN MESIN

`p1_g13 §A2.2` + `p1_g14 §A2.6` berlaku utuh:

1. **Konkurensi maksimum 3 pekerjaan latar.** Tidak dinaikkan ketika terlihat
   lambat.
2. `load average` dicatat **awal dan akhir** setiap sel dan masuk tabel §B. Sel
   tanpa catatan beban **tidak dilaporkan**.
3. Pembenarannya adalah beban **di bawah 16 core**, bukan angka konkurensinya.
4. Satu berkas per (lengan, `c_arm`); worker paralel punya himpunan sel yang
   **DISJOIN**. Dua proses menulis satu JSON = saling menimpa (`p1_g13`).
5. Tulis **per baris dengan `flush=True`**; `tail` pada proses latar menelan
   keluaran.
6. `np.bool_` tidak JSON-serializable.
7. `pkill -f <pola>` juga membunuh shell yang memuat pola itu. **Pakai PID.**

#### A2.10 🔒 APA YANG DILAPORKAN KALAU LAGI-LAGI TIDAK SEMUANYA BISA DIBUKTIKAN

Kunci kriteria nomor 4. Per sel, **wajib**, terlepas dari hasilnya:

1. `Δ_struct`, `Δ_full`, `Δ_arm` sebagai kurungan dua-sisi + vonis.
2. Jumlah `exact`/`proved`, dan jumlah **TIDAK ADA JADWAL DITEMUKAN** dengan
   **DUA** angka (anggaran pencarian **dan** wall) — `p1_g14 §A2.1` K2.
3. Jumlah jadwal yang gagal gerbang, per penjadwal. Target **0**. (R1/R2)
4. `load average` awal/akhir dan konkurensinya.
5. Sel/lengan yang **tidak dijalankan** disebut namanya sebagai kalimat
   **TIDAK DIUKUR**. Dilarang menulis "diperkirakan tidak berubah".
6. Instance **TIDAK DAPAT DIBANGUN** dengan namanya, dan fraksi `pool` yang
   tersingkir oleh `at_p0` (A2.1 syarat 5).
7. Papan skor A4, dinilai. Dugaan yang dinilai memakai solver yang belum lulus
   gerbangnya **ditandai TERTUNDA**; dugaan yang instrumennya tidak mengukur
   besaran yang didalilkan **juga tidak dinilai** (D37 G14).

---

### A3. PALANG — regresi yang harus lulus sebelum angka baru disebut

| | Apa yang diadu | Kalau gagal |
|---|---|---|
| **R1** | Setiap jadwal yang dikembalikan lolos `validate_coupled` **dan** `sa.arm_schedule_conflict` | sel itu **tidak dilaporkan sebagai angka**, dilaporkan sebagai kegagalan gerbang |
| **R2** | Lemma 3 (`ub ≥ lb`) pada setiap instance | idem |
| **R3** | Keempat syarat A2.1 diperiksa **di luar generator**, per instance | instance itu **TIDAK DAPAT DIBANGUN**, disebut namanya |
| **R4** 🆕 | `gen_real_dense(band = 2.00, at_p0 dimatikan)` **≡ `gen_real`** pada 40 kunci: `node_idx` identik | generatornya bukan `gen_real` + dua perubahan → **temuan**, sesi berhenti sampai sebabnya diketahui |

**R4 adalah palang yang paling penting sesi ini** dan ia dikunci di muka: ia
adalah satu-satunya uji yang membuktikan bahwa `gen_real_dense` benar-benar
*"`gen_real` dengan tepat dua perubahan"* dan bukan generator ketiga yang
kebetulan bernama mirip. Tanpanya, `Δ_arm` yang berbeda tidak dapat dikaitkan
pada kepadatan tugas — persis jebakan M3 G12.

### A4. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor berdiri di **28 meleset, 15 tepat** (`p1_g14 §B8`).
Prior **DUNIA**: LONGGAR, dan G14 menaikkannya lagi — **432 bukti `Δ_arm = 0`**
di S1 melawan **satu** positif di S2; kendala lengan pada sel ini **gratis atau
mustahil**, "ada tapi lebih mahal" = **2 dari 56** (`p1_g14 §B6`).
Prior **KODE SENDIRI**, dibelah dua menurut `p1_g14 §B8` bacaan 1: dugaan dari
**membaca alur kontrol** meleset (0 dari 2 di G14); dugaan dari **sifat
aritmetika** tepat (D43).

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D44** | `DENSE` memberi **0** `Δ_arm` positif terbukti pada **kedua** `c_arm` | **dunia** | prior LONGGAR, dan ia baru saja bertahan terhadap percobaan yang dirancang mematahkannya dari dua arah (D38 ✅, D39 ❌). Ditulis sadar bahwa ia **mengecewakan** kalau tepat: kalau D44 tepat, ketiga faktor S2 nihil sendiri-sendiri dan yang tersisa adalah interaksi (A2.4) |
| **D45** | Fraksi **TIDAK ADA JADWAL DITEMUKAN** pada `DENSE` **naik** di atas baris S1 kunci yang sama (S1 `c_arm` 0.15: 4 dari 40) | **dunia** | `p1_g14 §B6` mengukur perilaku **biner**: `rerouted` 0/56, `infeasible` 36/56. Kalau kepadatan tugas mengikat sama sekali, ia mengikat dengan **menghapus** jadwal, bukan dengan memahalkannya. D44 + D45 bersama = "biner, dan sisi yang menyala adalah sisi mustahil" |
| **D46** | Syarat `at_p0` menyingkirkan **≥ 50 %** kolam tugas pada `DENSE` (`p0` di `lin 0.00`) | **kode sendiri, ARITMETIKA** | jangkauan lengan 1.06 m di sekitar sumbu yang mengorbit 0.40 m (`gen_real_rotcrowded` docstring), jadi gantry di `lin 0.00` menutupi `x` sampai ~1.46 m — yang memuat **seluruh** pita `0.50–1.10`. Diturunkan dari aritmetika jangkauan, bukan dari alur kontrol; itu belahan yang D43 buktikan berguna |
| **D47** | Audit A2.7 menemukan **≥ 1** dari empat anggaran yang diperiksa **hanya di antara iterasi**, dengan overshoot satu-iterasi **TIDAK TERBATAS** | **kode sendiri, ALUR KONTROL** | dua contoh sudah ditemukan pada hari yang sama di dua instrumen berbeda (`p1_g14 §B8` bacaan 4). Tetapi ia diturunkan dari **membaca alur kontrol**, yang rekam jejaknya **0 dari 2** — ditulis justru supaya belahan D29 diuji lagi, bukan supaya dipercaya |

### A5. Berkas

| Berkas | Isi | Status |
|---|---|---|
| `test/dense_g15.py` | tugas 1–3: `gen_real_dense`, R3/R4, probe biaya, sel, `report` | **BARU** |
| `test/psweep_g14.py` | diimpor (`shrink`, `build`, `verdict`, `_exact`, `keys40`) | 🔒 **BEKU** |
| `test/m3_bind_g13.py`, seluruh `sched*.py` | — | 🔒 **BEKU** |
| `docs/p1_g15_dense.md` | dokumen ini | — |

### A6. Rule 6

**Rule 6 (anggaran 30 000 token/sesi) adalah PENGECUALIAN EKSPLISIT untuk sesi
protokol-panjang P1, dinyatakan di muka**, sebagaimana `p1_g12 §A8`,
`p1_g13 §A6`, `p1_g14 §A6`. Anggaran yang benar-benar mengikat dan dilaporkan:
**anggaran pencarian 120 s per instance**, dan **aturan biaya A2.5** yang
memutuskan berapa sel dijalankan.

---

## B. Hasil terukur

> §A dikunci 2026-08-16 sebelum satu baris kode sesi ini dijalankan. Semua
> angka di bawah keluar sesudahnya. Setiap tempat di mana §B bertentangan
> dengan §A ditandai 🔺. **§A TIDAK ditulis ulang.**

### B0. Cara menjalankan ulang, dan beban mesin

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 test/dense_g15.py gates                 # R4 + R3 + sensus kolam
python3 test/dense_g15.py probe                 # A2.5, biaya SEBELUM sel
python3 test/dense_g15.py park n6_s0_mr1 0.20   # kenapa "tidak ada jadwal"
python3 test/dense_g15.py cell DENSE 0.15       # satu sel
python3 test/dense_g15.py report
python3 /tmp/g15_audit_maxstates.py             # tugas 4, temuan 1 (B4.1)
python3 /tmp/g15_m3floor.py                     # tugas 5 (B5)
```

⚠️ Dua skrip terakhir ada di `/tmp` dan memakai `sys.path` **absolut** — skrip
di `/tmp` tidak mewarisi cwd repo, dan versi pertamanya gagal persis begitu.

⚠️ **Beban mesin:** awal sesi `load average` **4.70**, selama pengukuran
**3.70–5.93** pada **16 core** dengan **maksimum 3 pekerjaan latar** (A2.9).
Itu **di bawah jenuh**, dan itulah pembenarannya, bukan angka konkurensinya.
G14 diukur pada 4.4–7.2, G13 6–8.5, G12 4.5–5.0, G11 3.3–3.7, G10 42.

🔒 **Berkas beku (§A0), diperiksa:** `git diff` **KOSONG** pada ketujuh belas
berkas. Berkas baru: `test/dense_g15.py`.

### B1. 🟢 TUGAS 1 — R4 LULUS, dan kepadatan tugas TERNYATA DAPAT DIPISAHKAN

**R4, palang terpenting sesi ini (A3):**

> `gen_real_dense(band = 2.00, at_p0 dimatikan)` **≡ `sched.gen_real`** pada
> **40 / 40 kunci** — `node_idx` **dan** `p0` identik.

Jadi *"`gen_real` dengan tepat dua perubahan"* adalah **fakta terukur**, bukan
deskripsi. Tanpa R4, `Δ_arm` yang berbeda tidak dapat dikaitkan pada kepadatan
tugas — persis jebakan M3 `p1_g12`.

**Sensus kolam tugas** (A2.1 syarat 5), `x_c = 0.80`:

| `band` | node dalam kolam | dari | fraksi peta |
|---|---|---|---|
| **0.30** | **864** | 3132 | **27.6 %** |
| 0.60 | 1836 | 3132 | 58.6 % |
| 1.00 | 2808 | 3132 | 89.7 % |
| 2.00 | 3132 | 3132 | 100.0 % |

**Tingkat penolakan `at_p0`** — dan di sinilah confound keempat (A2.0)
memperlihatkan dirinya sebagai angka:

| lengan | `p0` | SR ditolak | SR terpakai | MR ditolak | MR terpakai |
|---|---|---|---|---|---|
| **`DENSE`** | pose 36 (`lin 0.00`, `rot 0°`) | **618 / 864 = 71.5 %** | **246** | **0 / 864 = 0.0 %** | **704** |
| **`DENSE-P0XC`** | pose 1188 (`lin 0.80`, `rot 0°`) | **826 / 864 = 95.6 %** | **38** | **462 / 864 = 53.5 %** | **242** |

🔴 **Itu bukan detail — itu mengukur seberapa besar letak `p0` mengubah
soalnya.** Dengan `p0` S2 (`lin 0.80`, tepat di tengah kolam), syarat `at_p0`
menyisakan **38 dari 864** node SR: kolam tugas efektif S2 **22× lebih kecil**
daripada kolam `DENSE`. Jadi `p0` bukan knob independen dari kepadatan tugas —
ia **pengali**-nya. `p1_g14 §B9.4` sudah mengukur bentuk yang sama untuk pose
dan tugas ("bukan dua knob bebas"); ini contoh keduanya, pada sumbu ketiga.

**R3 atas 40 kunci × dua lengan, `band = 0.30`:**

| lengan | TIDAK DAPAT DIBANGUN |
|---|---|
| `DENSE` | **0 / 40** |
| `DENSE-P0XC` | **0 / 40** |

> ## ➜ **0 % tak-terbangun.** Aturan A2.1 menempatkannya di pita **≤ 20 %**:
> ## sel dijalankan dan dilaporkan sebagai **angka utama**, tanpa kualifikasi
> ## jumlah-instance.

🔴 **Dan ini menjawab kekhawatiran yang prompt G15 tulis eksplisit.** Prompt
mengunci: *"Kalau tak-terbangun > 50 %, itu TEMUAN — kepadatan tugas dan ruang
pose tidak bisa dipisahkan di sel ini sama sekali."* Terukur **0 %**, jadi
kebalikannya yang benar: **kepadatan tugas DAPAT dipisahkan dari ruang pose**,
dan `DENSE` adalah pemisahan itu. Bandingkan dengan arah sebaliknya, yang
`p1_g14 §B2` ukur: memasang **pose** S2 pada **tugas** S1 gagal pada **23 dari
40**. Asimetrinya sendiri informatif — tugas S2 muat di ruang pose S1, tetapi
pose S2 tidak muat pada tugas S1.

### B2. 🟢 BIAYA DIUKUR SEBELUM 40 INSTANCE, dan salah satu probe TIDAK MENGEMBALIKAN JADWAL

A2.5, aturan keputusan dikunci sebelum angkanya dilihat. `DENSE`,
`c_arm = 0.20`, `|P| = 2376`:

| instance | wall | makespan | route / terbukti | node | gerbang lengan (`ctx`) |
|---|---|---|---|---|---|
| `n6_s0_mr1` | **284.69 s** | **inf** | `bnb`, tidak | **0** | 0 panggilan, 0.0 s |
| `n6_s4_mr0` | **77.03 s** | 11.0647 | `bnb`, **terbukti** | 38 | 14 800 panggilan, 67.8 s = **88 %** |

➜ Probe terburuk **284.69 s**, jatuh di pita **60–300 s** ⟹ menurut aturan yang
dikunci di muka: **`DENSE` saja, 40 instance × `c_arm` {0.15, 0.20} = 2 sel.**
🔺 **`DENSE-P0XC` karena itu TIDAK DIUKUR**, dan disebut namanya di sini.

➜ **Dan gerbang lengan 88 % wall pada probe yang selesai.** Ini **sesi
KEDELAPAN** berturut-turut dengan pola yang sama: **yang mahal adalah
PEMERIKSA** (`p1_g8 §B4` … `p1_g14 §B3`). 88 % adalah angka tertinggi yang
pernah diukur untuk pola ini; G14 mengukur 33–70 %.

#### B2.1 🔴 Kenapa `n6_s0_mr1` tidak mengembalikan jadwal — DIUKUR, bukan dinalar

A2.5 mengunci: probe yang tidak mengembalikan jadwal adalah **temuan**, dan sel
**berhenti sampai sebabnya diketahui**. Sebabnya diukur dengan `_park_profile`
yang **diimpor dari `diag_g14.py`**, bukan ditulis ulang (`p1_g13 §B3`:
instrumen kedua dari bentuk yang sama mewarisi cacat yang sama). Seluruh
**2376 pose × 2 arah**:

| arah | S1 struct/parkir | S2 lengan/parkir | S3 struct/ekor | S4 lengan/ekor | TERIMA |
|---|---|---|---|---|---|
| lead 1, park 2 | 1226 | 410 | 245 | 495 | **0** |
| lead 2, park 1 | 0 | 1052 | 0 | 1324 | **0** |
| **TOTAL** | **1226 (25.8 %)** | **1462 (30.8 %)** | **245 (5.2 %)** | **1819 (38.3 %)** | **0** |

| Fase | Penolakan | |
|---|---|---|
| **PARKIR** | 2688 | **56.6 %** |
| **EKOR SERIAL** | 2064 | **43.4 %** |
| **DITERIMA** | **0** | — |

➜ **Sebabnya diketahui, dan ia kategori yang sudah bernama:** sama persis
dengan baris TIDAK ADA JADWAL S1 yang `p1_g14 §B4` profilkan (19 008 penolakan,
0 penerimaan). **Tidak ada pose yang lolos keempat tahap**, jadi tidak ada bug
di berkas beku — `arm_serial_ub` benar-benar tidak punya apa pun untuk
dikembalikan. Syarat berhenti A2.5 **dilepas**, dan sel dijalankan.

🔴 **Tapi pembagiannya BERGESER, dan itu angka baru.** Bandingkan dengan
`p1_g14 §B4` pada S1:

| | predikat **STRUKTUR** (S1+S3) | predikat **LENGAN** (S2+S4) |
|---|---|---|
| S1, `c_arm = 0.15` (`p1_g14 §B4`) | **52.3 %** | **47.7 %** |
| `DENSE`, `c_arm = 0.20` (sesi ini) | **31.0 %** | **69.1 %** |

Di bawah kepadatan tugas, **predikat lengan mengambil alih penolakan**: dari
47.7 % ke **69.1 %**. Kendala lengan pada sel `DENSE` jelas **aktif** — ia
menolak lebih dari dua pertiga pose parkir. Apakah ia **berharga** adalah
pertanyaan yang berbeda, dan itu §B3.

#### B2.2 🔺 `ctx.calls` TIDAK MENGUKUR kerja lengan di konstruktor pra-loop

Probe `n6_s0_mr1` melaporkan **0 panggilan gerbang lengan, 0.0 s**, sambil
memakai **284.69 s** wall dan menolak **3281 pose lewat predikat lengan**
(§B2.1). Kedua angka itu tidak bertentangan: `arm_serial_ub`
(`sched_armfull.py:390`) memanggil `schedule_arm_conflict(...)` **langsung**,
bukan lewat `ctx.block()` (`:344`), jadi `ctx.calls` / `ctx.t_arm` hanya
menghitung kerja lengan **di dalam loop B&B**.

➜ Ini **penjelasan mekanistik untuk D37 G14**, yang `p1_g14 §B8` tandai
**TIDAK DINILAI** karena "instrumennya tidak mengukur besaran yang
didalilkan". Sekarang diketahui **kenapa** instrumennya tidak mengukurnya, dan
letaknya disebut. Ini bukan bug — `ctx` memang budget loop — tetapi **kolom
`arm_calls` / `arm_t` di setiap tabel P1 adalah angka LOOP, bukan angka
instance**, dan itu belum pernah dikatakan.

### B3. 🟢 TUGAS 2 — JAWABANNYA NEGATIF LAGI, dan kali ini buktinya LEBIH KUAT

**2 sel × 40 instance = 80 instance-run. 0 kegagalan gerbang (R1), 0 pelanggaran
Lemma 3 (R2), 0 pelanggaran gerbang lengan lambat.**

Kriteria A2.3 dipakai **apa adanya**, disalin verbatim dari `p1_g14 §A2.2`:
`Δ_arm` positif dihitung **hanya** kalau sisi struktur **dan** sisi penuh
dua-duanya `exact`/`proved`.

| lengan | `c_arm` | `\|P\|` | dibangun | terbukti | **POSITIF** | kurungan `Δ_arm` | `lb` vs S1 | wall mean/maks | > anggaran pencarian | load |
|---|---|---|---|---|---|---|---|---|---|---|
| `DENSE` | 0.15 | 2376 | **40** | **33** | **0** | [+0.0000, +14.2952] | **−57.5 %** | 31.4 / 240.9 s | 7 (maks +120.9 s) | 4.58 → 7.10 |
| `DENSE` | 0.20 | 2376 | **40** | **28** | **0** | [−0.0000, +25.9773] | **−57.5 %** | 57.3 / 300.1 s | 12 (maks +180.1 s) | 4.58 → 3.94 |

> ## ➜ **61 bukti dua-sisi `Δ_arm = 0`, NOL contoh tandingan**, pada tugas S2
> ## di ruang pose S1 penuh.

🔴 **DAN INI BUKAN "NOL LAGI YANG LEBIH LEMAH" — ia nol yang LEBIH KUAT.**
`p1_g14 §B9.5` memperingatkan bahwa nol positif di sel dengan sepertiga
instance terbukti adalah bukti **lebih lemah**. Kualifikasi itu **berbalik** di
sini, dan angkanya dari artefak G13 yang sama:

| pada `c_arm` yang sama | S1 (`p1_g13`) | **`DENSE` (sesi ini)** |
|---|---|---|
| 0.15 — terbukti dua sisi | 23 / 40 | **33 / 40** |
| 0.15 — TIDAK ADA JADWAL | 4 / 40 | **1 / 40** |
| 0.20 — terbukti dua sisi | 19 / 40 | **28 / 40** |
| 0.20 — TIDAK ADA JADWAL | 6 / 40 | **3 / 40** |
| `lb` median | 39.304 s | **16.227 s** |

Proporsi terbukti `DENSE` adalah yang **tertinggi** dari sel `c_arm ∈ {0.15,
0.20}` mana pun yang pernah diukur di P1 — lebih tinggi daripada S1 sendiri,
jauh lebih tinggi daripada `GEO-ROT` (11–18 / 40) dan `GEO-S2` (6–8 / 17).

#### B3.1 🔺 PERTENTANGAN 1 — kepadatan tugas membuat instance LEBIH KECIL, dan tanda kalimat naskah A2.6 TERBALIK

`lb` median turun dari **39.304 s** ke **16.227 s**, yaitu **−57.5 %**, dan itu
bukan artefak: ia mekanismenya. Tugas yang dikerumunkan ke pita `x_c ± 0.30`
menuntut **lebih sedikit gerak gantry**, dan `p1_g7 §B5` sudah mengukur bahwa
**74–92 % makespan adalah gerak gantry**. Jadi hipotesis prompt — bahwa
memaksa kedua gantry berbagi wilayah menaikkan harga koordinasi — **punya
tanda yang salah pada sel ini**: berbagi wilayah berarti **tidak perlu pergi ke
mana-mana**, dan jadwalnya jadi lebih pendek, lebih mudah dibuktikan, dan lebih
jarang gagal.

🔒 **Dan §B2.1 sudah mengukur bahwa ini BUKAN karena kendala lengan jadi tidak
aktif.** Di `DENSE`, predikat lengan menolak **69.1 %** pose parkir — naik dari
47.7 % di S1. Kendala lengan **lebih aktif** dan **tetap tidak berharga**.
Itu, dan bukan angka nol itu sendiri, adalah kalimat yang layak dikutip:
**aktivitas kendala dan harga kendala adalah dua hal yang berbeda pada sel
ini**, dan sekarang keduanya terukur pada instance yang sama.

#### B3.2 🔒 KONTROL BERPASANGAN-KUNCI, dan apa yang ia TIDAK katakan

Ketiga kunci `DENSE` tanpa jadwal (`n4_s0_mr1`, `n4_s1_mr1`, `n6_s0_mr1`)
**punya** jadwal terbukti di S1 pada kunci yang sama — jadi baris TIDAK ADA
JADWAL bukan warisan kunci, ia properti instance `DENSE`. Sebaliknya, keempat
kunci S1 tanpa jadwal **tidak** muncul di daftar `DENSE`.

⚠️ Kontrol ini **berpasangan-kunci, bukan se-instance** (A2.3, dinyatakan di
muka). Ia tidak dapat mengatakan "instance ini kehilangan harganya"; ia hanya
dapat mengatakan "pada benih yang sama, keluarga tugas yang dikerumunkan
berperilaku begini". Instance se-instance **tidak dapat dibangun**, dan
alasannya kontradiksi istilah, bukan kekurangan usaha.

#### B3.3 🔺 TANGGA PEMISAH A2.6 TIDAK DIJALANKAN — sesuai aturan, bukan karena waktu

A2.6 mengunci: tangga pita `{0.30, 0.60, 1.00, 2.00}` dijalankan **kalau tugas
2 positif**, karena ia mencari **di mana harganya hilang**. Tugas 2 **nihil**,
jadi tidak ada harga untuk dicari hilangnya, dan tangga **tidak dijalankan**.
Ini keputusan yang dikunci sebelum hasilnya dilihat, dan disebut di sini supaya
ia tidak terbaca sebagai sel yang terlewat. Rung `band` 0.60 / 1.00 / 2.00
adalah **TIDAK DIUKUR**, dengan namanya.

### B4. 🟢 TUGAS 4 — AUDIT TITIK PEMERIKSAAN ANGGARAN

A2.7 menuntut **tiga kolom** per anggaran, dan tidak boleh kurang.

| Anggaran | Nilai | **Di mana diperiksa** | **Overshoot satu iterasi** | **Angka yang bergantung** |
|---|---|---|---|---|
| `ACTION_BUDGET` | 400 | `sched_coupled.py:734`, di **puncak setiap iterasi** loop aksi, satu node — kondisi yang **sama** juga memeriksa `time_budget` (yaitu anggaran pencarian 120 s) | **satu aksi** — tetapi rekursi anak terjadi **di dalam** badan loop, jadi granularitas efektif pemeriksaan 120 s adalah **satu SUBPOHON**, bukan satu aksi. Pembangunan daftar `actions` (subset × pose, `:700-731`) mendahului loop dan **tidak diperiksa sama sekali** | setiap angka `solve_coupled2` / `solve_armfull` |
| `EVADE_ATTEMPTS` | 60 | `sched_coupled.py:779` / `sched_armfull.py:643`, di puncak loop evade, **SEBELUM** `evade_tried += 1` | **NOL.** `evade_tried` tidak pernah melewati 60 | — |
| `max_states` | 400 000 | `m3_bind_g13.py:146`, `verify_sched_armfull.py:340`, di puncak `rec()` **sesudah** `seen += 1` | **NOL state** | M3: `BINDING`, `vacuous`/`rerouted` (`p1_g14 §B6`) |
| `W2_BUDGET` | 240 s | `verify_sched_coupled.py:206`, di puncak `_rec`, **per node** | **satu node penuh**: bangun `acts` (pose × subset) lalu `\|acts\| × \|starts\|` uji `ref_conflict`, semuanya sebelum anak mana pun memeriksa lagi | W2 (31 instance), `p1_g9 §A3-K1` |

🟢 **`EVADE_ATTEMPTS` adalah satu-satunya dari empat yang diperiksa di tempat
yang benar** — sebelum penambah, bukan sesudahnya. Overshoot **nol**, exact.

🔴 **TEMUAN 1 — `max_states` diperiksa di tempat yang benar tetapi
EFEKNYA SENYAP.** `enum_opt` (`m3_bind_g13.py:187`) mengembalikan
`(best, cnt)` dan **membuang `seen`**; `brute_arm`
(`verify_sched_armfull.py`) mengembalikan `best` saja dan **membuang `seen`**.
Jadi enumerasi yang **terpotong** tidak dapat dibedakan dari yang **tuntas** —
dan keduanya adalah instrumen yang memutuskan `BINDING` dan pembagian
`vacuous`/`rerouted`/`binding` yang `p1_g14 §B6` kutip sebagai kolom utamanya.
Enumerasi terpotong memberi **taksiran-lebih** optimum, yaitu **kelas
konsekuensi yang sama persis** dengan bug `break` yang `p1_g13 §B3` sudah
perbaiki sekali — **sumber kedua, belum diperbaiki, untuk mode kegagalan yang
sama.**

🔴 **TEMUAN 2 — `W2_BUDGET` tidak melihat konstruktor `Brute`, dan itu BENTUK
YANG SAMA dengan temuan 120 s.** `self.t0` di-set di `solve()`
(`verify_sched_coupled.py:186`), sedangkan `__init__` (`:178-183`) sudah
menghitung `self.dur` untuk **SELURUH** `(g, U, p)` — yaitu
`2 × (2ⁿ − 1) × |P|` panggilan `sched.stop_duration` — **di luar anggaran**.
Ini contoh **ketiga** dari `p1_g14 §B9.2` ("setiap anggaran harus menyebut
titik pemeriksaannya"), ditemukan di instrumen ketiga.

🔒 **Apakah temuan 2 membatalkan angka yang sudah dikutip? TIDAK, dan
sebabnya aritmetika, bukan kelonggaran:** bahan bakar W2 adalah
`gen_small_crowded` dengan `n ≤ 3` dan `|P| ≤ 4`
(`verify_sched_coupled.py:275`), jadi prakomputasinya `2 × 7 × 4 = 56`
panggilan — di bawah kebisingan. Cacatnya **laten**: ia menggigit hanya kalau
`Brute` diarahkan ke instance peta nyata (`n = 6`, `|P| = 2376` ⟹ **299 376**
panggilan `stop_duration` sebelum jam mulai berjalan). **Vonis U4 tidak
tersentuh** — ia memakai keanggotaan `Brute._starts`, bukan anggarannya.

#### B4.1 🟢 TEMUAN 1 DIUKUR — tutupnya TIDAK pernah menggigit pada kelas yang G14 kutip

Temuan 1 diturunkan dari **membaca kode**, dan prior D29 melarang mempercayai
itu. `seen` tidak dikembalikan, jadi ia tidak dapat diamati langsung — tetapi ia
dapat **dikurung**: kalau menurunkan tutupnya **400×** tidak mengubah
`(best, cnt)`, enumerasinya selesai jauh di bawah 400 000.

Tutup `{1000, 10 000, 400 000}`, dua kelas yang `p1_g14 §B6` benar-benar
laporkan, 3 seed × `c_arm ∈ {∞, 0.05}`:

| kelas | dibangun | sepakat | **tidak sepakat** | terlambat |
|---|---|---|---|---|
| `forced-n2P3` | 3 | 6 / 6 | **0** | 33.2 s |
| `forced-n2P4` | 3 | 6 / 6 | **0** | 33.2 s |

➜ **Tutupnya tidak menggigit.** Kolom `vacuous`/`rerouted`/`binding` di
`p1_g14 §B6` **tidak terpotong**, dan angka itu berdiri.

🔒 **Cacatnya tetap dilaporkan sebagai cacat laten**, karena yang diukur adalah
kelas yang **kebetulan kecil**: `forced-n4P3` — kelas yang menghabiskan lebih
dari 20 menit dalam satu panggilan `classify()` (`p1_g14 §B6` Pertentangan 4) —
**tidak diukur di sini**, dan ia justru kandidat paling mungkin memicu tutupnya.
Yang boleh ditulis: *"pada dua kelas yang dikutip, tutupnya tidak menggigit"*.
Yang **tidak** boleh: *"`max_states` tidak pernah menggigit"*.

🔴 **Dan sifat yang membuat cacat ini kurang berbahaya dari kelihatannya,
dinyatakan sebagai aritmetika bukan sebagai penenangan:** pohon rekursi
`brute_arm`/`enum_opt` **tidak bergantung pada `c_arm`** — `c_arm` hanya
menyaring di **daun**. Jadi pemotongan mengenai jalan `m_free` dan `m_arm`
**secara simetris**, pada prefiks yang sama, dan **tidak dapat membalik
`BINDING` lewat asimetri**. Yang bisa ia lakukan: membuat **kedua**-nya
taksiran-lebih atas optimum — kelas konsekuensi yang sama dengan bug `break`
`p1_g13 §B3`.

### B5. 🟡 TUGAS 5 — LANTAI USAHA M3: satu kelas LUNAS PENUH, tiga TIDAK DIUKUR

Dijalankan lewat `m3_bind_g13.classify()` / `brute_arm()` / `gen_forced()`
yang **BEKU** — dipanggil, tidak diedit (A2.8). Empat kelas yang `p1_g14 §B6`
sebut namanya sebagai TIDAK DIUKUR dijadwalkan; tutup RUN 2400 s.

| kelas | kandidat | dibangun | brute 2 arah | **BIND** | **BIND @ `c = 0.05`** | `vacuous` | `rerouted` | `infeasible` | selisih enumerator |
|---|---|---|---|---|---|---|---|---|---|
| **`G12-baseline`** | **14** | **14** | **56** | **11** | **0** | **32** | **13** | **0** | **0** |
| `G12-baseline-P4` | 0 | 0 | 0 | — | — | — | — | — | — |
| `forced-n2P3-dw2` | 0 | 0 | 0 | — | — | — | — | — | — |
| `forced-n2P2` | 0 | 0 | 0 | — | — | — | — | — | — |

> ## ➜ **BINDING pada `c_arm = 0.05`: 0 dari 56**, lagi — sekarang pada kelas
> ## **baseline** dan bukan hanya pada kelas paksa.

🟢 **Ini menguatkan `p1_g14 §B6` dengan bukti 5.5× lebih banyak.** G14 hanya
sempat menjalankan `G12-baseline` sebagai "reproduksi, jalan pertama" dan
melaporkan **2** BINDING sebelum tutupnya memotong. Kelas penuh memberi
**11** BINDING — **semuanya di `c_arm ≥ 0.10`**, nol di 0.05. Jadi "harga nol
di nilai utama" bertahan pada 56 instance baseline yang di-brute-force dua arah,
bukan hanya pada 56 instance paksa.

🔴 **Dan sekarang pembandingnya punya PENYEBUT YANG SAMA** — yang G14 tidak
punya (ia mengadu 0/56 dengan 9/32):

| | `G12-baseline` (sesi ini) | `forced-*`, tuas 3+4 (`p1_g14 §B6`) |
|---|---|---|
| brute 2 arah | **56** | **56** |
| `rerouted` | **13** | **0** |
| `infeasible` | **0** | **36** |
| BIND @ 0.05 | **0** | **0** |

➜ Bacaan `p1_g14 §B9.9` **terkonfirmasi dan menajam**: menutup jalan keluar
"menunggu" tidak memahalkan optimum, ia **memindahkan** instance dari kolom
`rerouted` (13 → 0) ke kolom `infeasible` (0 → 36). Kendala lengan di sel ini
**biner**, dan sekarang terlihat sebagai **perpindahan kolom pada penyebut yang
sama**, bukan sebagai dua persentase dari dua eksperimen.

🔒 **`disagree` = 0 dari 56:** `brute_arm` dan `enum_opt` sepakat pada setiap
instance — silang-periksa yang enumerator beku tidak pernah punya, dan ia lulus.

⚠️ **LANTAI USAHA TETAP TIDAK TERPENUHI, dan A2.10 menuntut angkanya:**

| Lantai usaha (`p1_g13 §A2.3`) | Dituntut | G14 | **+ sesi ini** |
|---|---|---|---|
| kelas berbeda | ≥ 5 | 2 ❌ | **3** ❌ |
| kandidat dibangkitkan | ≥ 300 | 28 ❌ | **42** ❌ |
| brute-force dua arah | ≥ 40 | 56 ✅ | **112** ✅ |

**TIDAK DIUKUR, disebut namanya:** `G12-baseline-P4`, `forced-n2P3-dw2`,
`forced-n2P2` (0 kandidat masing-masing, tutup RUN habis di kelas pertama), dan
dari G14 masih: `forced-n4P3` (sebagian), `forced-n4P4`, `forced-n4P3-dw2`.

➜ Karena lantai usaha **masih** tidak terpenuhi, larangan `p1_g14 §B9.8`
**tetap berlaku**: kalimat *"tuas 3+4 tidak bisa membuat instance mengikat di
0.05"* **DILARANG**. Yang boleh sekarang lebih kuat dari G14, tetapi masih
terkurung: **"pada 112 instance yang di-brute-force dua arah lewat tiga kelas —
dua paksa dan satu baseline — 0 mengikat di `c_arm = 0.05`."**

#### B5.1 🔺 PERTENTANGAN 2 — argumen saya sendiri untuk tutup 2400 s TERLALU OPTIMIS 3×

A2.8 mengizinkan tutup yang diperiksa **di antara kandidat** dengan pembenaran
eksplisit: audit §B4.1 mengukur `classify()` pada kelas `n ≤ 3` **≤ 33 s**,
jadi overshoot satu kandidat tidak bisa jauh. Terukur: run berhenti pada
**2508 s**, yaitu **108 s di atas** tutupnya — **3.3× lebih besar** dari batas
yang saya nyatakan.

Sebabnya aritmetika, dan saya melewatkannya: §B4.1 mengukur **satu pasang
`enum_opt`**, sedangkan satu kandidat menjalankan `classify()` **empat kali**
(sapuan `c_arm` 0.05–0.20), dan setiap `classify()` memanggil `brute_arm` dua
kali **plus** `enum_opt` dua kali. Batas per-kandidat yang benar ≈ 8× yang saya
pakai.

➜ Ini **contoh KEEMPAT** dari `p1_g14 §B9.2` dalam dua sesi, dan yang paling
tajam: kali ini anggarannya jebol **setelah** saya menulis pembenaran tertulis
bahwa ia tidak akan jebol. **Menyebut titik pemeriksaan tidak cukup — batas
per-iterasinya harus diturunkan dari iterasi yang SEBENARNYA dijalankan, bukan
dari pengukuran terdekat yang tersedia.**

### B6. Papan skor §7.2

Papan skor berdiri di **28 meleset, 15 tepat** (`p1_g14 §B8`).

| # | Dugaan (§A4, ditulis di muka) | Hasil |
|---|---|---|
| **D44** | `DENSE` memberi **0** `Δ_arm` positif terbukti pada kedua `c_arm` | ✅ **TEPAT** — 0 dari **61** bukti dua-sisi. Prior DUNIA (LONGGAR) menang untuk kesekian kalinya, kali ini terhadap faktor yang G14 sebut sebagai **satu-satunya kandidat tersisa** |
| **D45** | Fraksi TIDAK ADA JADWAL pada `DENSE` **naik** di atas S1 kunci yang sama | ❌ **MELESET, dan arahnya adalah temuannya** — ia **TURUN**: 1 vs 4 pada 0.15, 3 vs 6 pada 0.20, sementara proporsi terbukti **naik** (33 vs 23; 28 vs 19). Mekanisme D45 (kendala biner ⟹ menghapus jadwal) benar sebagai deskripsi `p1_g14 §B6`, tetapi salah diterapkan: kepadatan tugas tidak mengetatkan instance, ia **mengecilkannya** (`lb` −57.5 %) |
| **D46** | `at_p0` menyingkirkan **≥ 50 %** kolam tugas pada `DENSE` | 🟡 **SEPARUH, dan dihitung sebagai MELESET** — SR **71.5 %** ✅, MR **0.0 %** ❌. Dugaannya tidak membelah menurut jenis tugas, dan kenyataannya membelah total. Sebab MR-nya bersih: handover menuntut irisan-ketat, dan pada `p0` (`lin 0.00`) **tidak satu pun** node di pita `x ∈ [0.50, 1.10]` dapat dijangkau **kedua** gantry. Dihitung meleset supaya penilaian tidak murah hati |
| **D47** | Audit menemukan ≥ 1 anggaran yang diperiksa **hanya di antara iterasi**, overshoot satu-iterasi **TIDAK TERBATAS** | ✅ **TEPAT** — `W2_BUDGET`: konstruktor `Brute` seluruhnya di luar anggaran (§B4 temuan 2), plus ekspansi satu node tidak diperiksa |

**Dua tepat, dua meleset** → papan skor **30 meleset, 17 tepat**.

Tiga bacaan:

1. 🔴 **D47 adalah dugaan pertama dari PEMBACAAN ALUR KONTROL yang TEPAT**
   (sebelumnya 0 dari 2: D40, D41 di G14). Tetapi ia tepat **karena
   pertanyaannya berbeda**, dan itu penajaman yang layak dipakai:
   D40/D41 menanyakan **apa akibatnya** (fase mana yang mendominasi; apakah
   Lemma C perlu) — pertanyaan tentang **perilaku**, yang pembacaan tidak bisa
   jawab. D47 menanyakan **di mana barisnya** — pertanyaan tentang **teks**,
   yang pembacaan memang bisa jawab. ➜ Prior D29 diperbaiki, bukan dicabut:
   **pembacaan kode dapat menemukan LETAK, tidak dapat memperkirakan AKIBAT.**
2. **Prior DUNIA (LONGGAR) menang lagi, dan sekarang ia telah bertahan terhadap
   ketiga faktor S2 satu per satu** — kardinalitas (G14 D38), geometri (G14
   D39), kepadatan tugas (D44). Nol positif dari **493** bukti dua-sisi lintas
   tiga keluarga instance, melawan **satu** positif yang pernah ada.
3. 🔴 **D45 meleset ke arah yang membalik hipotesis prompt.** Prompt G15
   membingkai kepadatan tugas sebagai hal yang **memaksa** kedua gantry berbagi
   wilayah, dan karena itu menaikkan harga. Terukur: berbagi wilayah membuat
   jadwalnya **lebih pendek** (`lb` −57.5 %). Kalimat naskah A2.6 yang dikunci
   di muka — *"harga koordinasi ditentukan oleh seberapa dipaksa kedua gantry
   berbagi wilayah"* — **tidak dapat ditulis**, dan tandanya terbalik.

### B7. Batasan setelah sesi ini

Seluruh `p1_g7 §A4`, `p1_g8 §B10`, `p1_g9 §A4`, `p1_g10 §B12`, `p1_g11 §B9`,
`p1_g12 §B10`, `p1_g13 §B10`, `p1_g14 §B9` **masih berlaku** kecuali yang
dicabut eksplisit. **DICABUT:** `p1_g14 §B9.3` ("kandidat yang tersisa dan
TIDAK DIUKUR: kepadatan tugas") — §B3 mengukurnya. Yang ditambahkan:

1. 🔴 **KETIGA faktor S2 sekarang NIHIL secara terpisah.** Kardinalitas ruang
   pose, geometri ruang pose (`p1_g14 §B7`), dan kepadatan tugas (§B3). Total
   **493 bukti dua-sisi `Δ_arm = 0`** lintas tiga keluarga instance, **nol
   contoh tandingan**, melawan satu positif terbukti yang hanya pernah muncul
   di S2. Yang tersisa menurut A2.4 adalah **INTERAKSI**, dan A2.4 melarang
   mengarang faktor keempat sebagai gantinya.
2. 🔴 **Ada confound KEEMPAT yang terbaca dari kode dan BELUM DIUJI: letak
   `p0`** (§A2.0). Ia **bukan** karangan — `DENSE-P0XC` dibangun, lulus R3
   40/40, dan **TIDAK DIUKUR** hanya karena aturan biaya A2.5. §B1 mengukur
   bahwa ia **pengali**, bukan knob bebas: dengan `p0` S2 syarat `at_p0`
   menyisakan **38 dari 864** node SR, lawan **246** dengan `p0` S1 — beda
   **22×**. Ini sumbu tunggal paling menjanjikan yang tersisa.
3. 🔴 **Kepadatan tugas menurunkan `lb` 57.5 %.** Harga S2 karena itu **tidak
   dapat** dikaitkan pada kerumunan tugas; kalau ada, kerumunan bekerja ke arah
   **sebaliknya**. Kalimat naskah A2.6 yang dikunci di muka tidak dapat ditulis.
4. 🟢 **Aktivitas kendala ≠ harga kendala, dan keduanya terukur pada instance
   yang sama** (§B2.1, §B3.1): predikat lengan menolak **69.1 %** pose parkir di
   `DENSE` (naik dari 47.7 % di S1) sambil memberi **nol** harga terbukti.
5. ⚠️ **`DENSE-P0XC` TIDAK DIUKUR**; tangga pita `band` **0.60 / 1.00 / 2.00**
   TIDAK DIUKUR (A2.6, karena tugas 2 nihil). Disebut namanya, tidak
   diperkirakan.
6. 🔴 **`ctx.calls` / `ctx.t_arm` adalah angka LOOP, bukan angka instance**
   (§B2.2): `arm_serial_ub` memanggil `schedule_arm_conflict` langsung. Setiap
   kolom `arm_calls` / `arm_t` di seluruh P1 membawa kualifikasi ini, dan
   **itulah sebabnya D37 G14 tidak dapat dinilai**.
7. 🔴 **Dua cacat anggaran laten, belum diperbaiki** (§B4): konstruktor `Brute`
   di luar `W2_BUDGET` (tidak menyentuh angka yang dikutip — bahan bakar W2
   punya `|P| ≤ 4`), dan pemotongan `max_states` yang **senyap** di
   `enum_opt`/`brute_arm` (diukur tidak menggigit pada dua kelas yang dikutip,
   §B4.1). Berkas beku **tidak disentuh**; keputusan perbaikan milik G16.
8. ⚠️ **Wall terus melampaui anggaran pencarian**: **19 dari 80** instance-run,
   overshoot maksimum **+180.1 s**. Sesuai A2.1 G14 opsi (b) ini **dilaporkan**,
   bukan dibatasi, dan tidak satu pun angka `Δ` bergantung padanya.
9. Proksi polyline tetap **meremehkan** volume sapuan; `T_fold = 0`; kanonik
   `manip`; exact tetap relatif terhadap grid 33 × 72; `ArmView` tetap tidak
   meng-cache konfigurasi MENGGANTUNG (`p1_g13 §B10.9`).
10. 🔴 **Rule 6 JEBOL**, dan §A6 menyatakannya di muka.

---

## C. Prompt faktorial — ⛔ DIGANTIKAN, JANGAN DIPAKAI SEBAGAI SESI BERIKUTNYA

> # ⛔ BACA INI DULU
>
> **Prompt di bawah ini BUKAN sesi berikutnya.** Ia digantikan pada hari yang
> sama juga (2026-08-16) oleh:
>
> # ➜ **[docs/p1_g16_hw.md](p1_g16_hw.md)**
>
> **Sesi berikutnya adalah PERANGKAT KERAS**, bukan faktorial offline.
> Alasannya dihitung di [p1_g16_hw.md §A0](p1_g16_hw.md): keuntungan informasi
> 4 sudut sisanya **rendah** (prior sudah menang 3 dari 3 faktor, 493 bukti),
> sementara **§8c langkah 2 dari 5 belum pernah dimulai** dan tenggat grasp
> **2026-11-13** tinggal ~3 bulan.
>
> Prompt di bawah **tidak dihapus** karena rancangan faktorialnya tetap sah dan
> siap pakai — ia **diturunkan prioritasnya**, dan dikerjakan hanya kalau ada
> sesi luang setelah perangkat keras jalan. Sampai itu terjadi, keempat sudut
> dilaporkan **TIDAK DIUKUR** dengan namanya (§B7.5).

### C1. (arsip) Prompt faktorial G16 — dipakai NANTI, bukan berikutnya

> **Rekomendasi: Opus 5, effort SEDANG.** Sama seperti G15, dan alasannya
> sekarang lebih kuat, bukan lebih lemah: sesi ini tidak menemukan satu pun
> kejutan yang menuntut penalaran dalam — yang ia lakukan adalah **membangun
> generator dengan palang ekuivalensi (R4), menjalankan dua sel, dan membaca
> kolomnya**. G16 punya bentuk yang sama persis dan **rancangan faktorialnya
> sudah lengkap di bawah**, jadi tidak ada keputusan desain yang tersisa. Yang
> menuntut kehati-hatian tinggal satu, dan ia bukan penalaran melainkan
> disiplin: **jangan mengarang faktor kelima** ketika sudut faktorial habis.

**Kerangka yang sesi ini buat, dan yang membuat G16 bisa ditulis sebagai
daftar:** S1 dan S2 berbeda pada **tiga sumbu biner**, bukan satu. Seluruh P1
sampai sekarang telah menguji **tiga dari delapan sudut**:

```
sudut = (himpunan POSE, kolam TUGAS, letak p0)

(S1,S1,S1)  = S1        432 bukti Delta_arm = 0        G13 + G14
(S2,S1,S1)  = GEO-S2    0 positif / 14 terbukti        G14 B7
(S1,S2,S1)  = DENSE     0 positif / 61 terbukti        G15 B3   <-- sesi ini
(S2,S2,S2)  = S2        SATU positif (+5.40 %)         G13 B5.4

BELUM DIUJI, keempatnya:
(S1,S1,S2)  p0 saja
(S1,S2,S2)  = DENSE-P0XC   -- SUDAH DIBANGUN, lulus R3 40/40, TIDAK DIUKUR
(S2,S1,S2)
(S2,S2,S1)
```

```
Sesi G16 -- REACH-6: INTERAKSI. Tiga faktor nihil sendiri-sendiri; sudut
faktorialnya tinggal empat, dan semuanya bisa dibangun dengan kode yang ADA.
G15 menyingkirkan kepadatan tugas: 61 bukti dua-sisi Delta_arm = 0 pada tugas
S2 di ruang pose S1 PENUH, dengan proporsi terbukti TERTINGGI yang pernah
diukur di P1 pada c_arm ini (33/40 dan 28/40, lawan 23/40 dan 19/40 di S1).
Satu-satunya harga positif yang pernah terbukti tetap satu: n6_s0_mr1 di S2.

BACA DULU:
1. docs/p1_g15_dense.md -- SELURUHNYA. Khususnya:
   A2.0 (confound KEEMPAT: p0 S1 = lin 0.00, p0 S2 = lin 0.80 -- 0.80 m rel),
   B1 (R4 LULUS 40/40; p0 adalah PENGALI: 38 vs 246 node SR terpakai),
   B2.1 (predikat lengan menolak 69.1 % pose parkir, naik dari 47.7 %),
   B2.2 (ctx.calls adalah angka LOOP -- kenapa D37 G14 tak dapat dinilai),
   B3.1 (lb turun 57.5 % -- kerumunan MENGECILKAN instance, tandanya TERBALIK),
   B4 + B4.1 (audit anggaran: dua cacat laten, satu diukur tidak menggigit),
   B6 bacaan 1 (pembacaan kode menemukan LETAK, bukan AKIBAT -- D29 diperbaiki),
   B7 (batasan -- terutama 1, 2, 3, 6, 7)
2. docs/p1_g14_reach4.md B7 (dua lengan pose, nihil), B9
3. test/dense_g15.py -- gen_real_dense, R4, build/cell/report. PAKAI ULANG,
   JANGAN TULIS ULANG. Sudut baru = argumen p0_mode + masker pose, bukan
   generator baru.

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.
CATATAN MESIN: G15 diukur pada load 3.6-7.1 dari 16 core, <=3 pekerjaan latar.
G14 4.4-7.2, G13 6-8.5, G12 4.5-5.0, G11 3.3-3.7, G10 42.
Disk root 100 % (16 G sisa) -- artefak kecil, tapi kegagalan tulis JSON
dibaca sebagai disk penuh dulu, bukan bug.

=== YANG SUDAH TEGAK, JANGAN BANGUN ULANG ===
- U4 LUNAS (G14 B5). Anggaran = anggaran PENCARIAN 120 s, wall dilaporkan
  terpisah (G14 B1). Lemma C BUKAN prapenyaring (G14 B4). sched_armfull.py
  TETAP BEKU, M0 tidak perlu diulang. TUTUP SEMUA.
- Tangga |P| 12 sel (G14 B7) dan sapuan G13 10 sel: dibaca dari /tmp, bukan
  dihitung ulang.
- DENSE 2 sel, 80 instance-run, 0 kegagalan gerbang: /tmp/g15_dense_DENSE_*.json
- R4 LULUS: gen_real_dense(band=2.00, at_p0 mati) == gen_real pada 40/40.
  Palang itu WAJIB diulang untuk setiap sudut baru.
- max_states tidak menggigit pada forced-n2P3 / forced-n2P4 (G15 B4.1).

=== TUGAS, BERURUTAN. JANGAN LOMPAT. ===
1. DENSE-P0XC. Sudah dibangun, lulus R3 40/40, tinggal dijalankan:
     python3 test/dense_g15.py cell DENSE-P0XC 0.15
   Ia sudut (S1,S2,S2) dan ia MURAH untuk dimulai karena tidak ada kode baru.
   BIAYA DIUKUR DULU (sesi kesembilan dengan jebakan yang sama) -- probe-nya
   sudah ada: `probe DENSE-P0XC`.
2. SUDUT p0-SAJA (S1,S1,S2): pose S1 penuh, tugas S1, p0 di lin 0.80. Itu
   satu argumen, bukan generator baru. Ia yang mengisolasi faktor (4) sendirian
   -- kembaran DENSE-P0XC seperti CARD<->GEO di G14.
   PALANG: R4 gaya G15 -- dengan band=2.00 dan at_p0 mati ia harus == gen_real
   kecuali pada p0. Kalau tidak, itu generator lain, dan hasilnya tak dapat
   dikaitkan.
3. DUA SUDUT SISANYA (S2,S1,S2) dan (S2,S2,S1) kalau anggaran mengizinkan.
   Dengan itu kedelapan sudut terisi dan pertanyaannya TERJAWAB atau
   TERBUKTI TIDAK DAPAT DIJAWAB dengan alat ini. Sudut yang tidak dijalankan
   disebut NAMANYA.
4. KALAU KEDELAPAN SUDUT NIHIL KECUALI S2 SENDIRI: itu hasilnya, dan yang
   ditulis adalah bahwa harga S2 adalah INTERAKSI TIGA-ARAH atau artefak
   SATU INSTANCE -- dan bahwa P1 tidak dapat membedakan keduanya dengan
   40 kunci. JANGAN mengarang faktor kelima. Pertimbangkan sebaliknya:
   ULANGI n6_s0_mr1 S2 pada 40 benih baru. Kalau ia satu-satunya dari 40,
   kualifikasi satu-instance jadi PERMANEN dan naskahnya berhenti mengejarnya.
5. DUA CACAT ANGGARAN (G15 B4), keputusan bukan audit lagi:
   (a) konstruktor Brute di luar W2_BUDGET -- laten, tidak menyentuh angka
       yang dikutip karena bahan bakar W2 punya |P| <= 4. Perbaiki atau
       nyatakan laten dengan syarat pemakaiannya.
   (b) pemotongan max_states SENYAP di enum_opt/brute_arm -- ukur pada
       forced-n4P3, kelas yang paling mungkin memicunya dan yang G14 tidak
       sempat ukur.
   Keduanya di berkas BEKU: p1_g10 A0 berlaku (perbaiki di tempatnya,
   pembekuan dinyatakan gugur, dilaporkan sebagai temuan).

=== KUNCI KRITERIA SEBELUM KODE, ke docs/p1_g16_*.md A ===
1. Apa yang membuat sebuah SUDUT dapat dibandingkan dengan sudut lain, dan
   R4 versi apa yang membuktikannya. DITULIS SEBELUM generatornya ada.
2. Definisi "harga muncul" A2.2 G14 dipakai APA ADANYA. Jangan tulis ulang.
3. Apa PERSIS yang boleh ditulis kalau kedelapan sudut nihil -- termasuk
   apakah "artefak satu instance" boleh disebut, dan bukti apa yang
   dituntut untuk menyebutnya. DITULIS SEBELUM hasilnya dilihat.
4. Apa yang dilaporkan kalau lagi-lagi tidak semuanya bisa dibuktikan.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- DELAPAN SESI: yang lambat adalah PEMERIKSA. G15 mengukur 88 % wall di
  gerbang lengan pada satu probe -- angka tertinggi sejauh ini.
- ctx.calls / ctx.t_arm TIDAK menghitung kerja lengan di konstruktor pra-loop.
  Jangan pakai kolom itu untuk menilai biaya per instance.
- Biaya TIDAK monoton dalam |P| (G14 B3), dan TIDAK monoton dalam kepadatan
  tugas (G15: kerumunan MENURUNKAN lb 57.5 %).
- Anggaran harus menyebut TITIK PEMERIKSAANNYA. TIGA contoh terukur sekarang:
  120 s (dalam loop), tutup RUN M3 (antar kandidat), W2_BUDGET (konstruktor
  di luar sama sekali).
- `tail` pada proses latar MENELAN keluaran. G15 kehilangan satu audit
  1500 s persis begitu, sesudah jebakan itu ditulis di prompt-nya sendiri.
  Tulis per baris ke berkas sendiri, dan tunggu dengan PID.
- Skrip di /tmp tidak mewarisi cwd repo: pakai sys.path ABSOLUT.
- classify() mengembalikan `kind` dan `agree`, BUKAN kunci boolean per kelas.
- Dua proses menulis satu JSON = saling menimpa. Satu berkas per sel.
- pkill -f <pola> juga membunuh shell yang memuat pola itu. Pakai PID.
- np.bool_ tidak JSON-serializable.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 30 meleset, 17 tepat.
  Prior DUNIA: LONGGAR -- dan ia kini bertahan terhadap KETIGA faktor S2
  satu per satu. 493 bukti dua-sisi Delta_arm = 0 lintas tiga keluarga
  instance, nol contoh tandingan, melawan SATU positif.
  Prior KODE SENDIRI, dipertajam G15: pembacaan kode dapat menemukan LETAK
  (D47 tepat: "di mana barisnya"), tidak dapat memperkirakan AKIBAT
  (D40, D41 meleset: "fase mana yang dominan", "apakah ia syarat perlu").
  Ajukan dugaan kode HANYA dalam bentuk pertama.
  Prior PALING BERGUNA: setiap temuan nyata datang dari UJI YANG DIJALANKAN.
  DELAPAN sesi berturut-turut.
- DUGAAN YANG DINILAI MEMAKAI SOLVER YANG BELUM LULUS GERBANGNYA TIDAK
  DINILAI. Dugaan yang instrumennya tidak mengukur besaran yang didalilkan
  juga TIDAK DINILAI (D37 G14). Dugaan yang tidak membelah menurut jenis
  tugas ketika kenyataannya membelah dihitung MELESET (D46 G15).
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 lima, G11 empat, G12 dua, G13 dua, G14 empat, G15 DUA.
- Rule 6 (30k token) adalah PENGECUALIAN EKSPLISIT untuk sesi protokol-panjang
  P1, dinyatakan di muka (A6), bukan dilaporkan sesudahnya.
- Akhiri dengan prompt sesi berikutnya (G17).
```
