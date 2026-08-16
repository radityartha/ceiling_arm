# P1 / G16 — REACH-6: INTERAKSI. Mengisi kisi faktorial, atau membuktikan kisinya bukan S2.

> Sesi G16, 2026-08-16. Melanjutkan [p1_g15_dense.md](p1_g15_dense.md).
> G15 menyingkirkan **kepadatan tugas** sebagai faktor tunggal: 61 bukti dua-sisi
> `Δ_arm = 0` pada tugas S2 di ruang pose S1 **penuh**, dengan proporsi terbukti
> **tertinggi** yang pernah diukur di P1 (33/40 dan 28/40). Ketiga faktor S2
> sekarang nihil sendiri-sendiri; total **493** bukti dua-sisi `Δ_arm = 0`
> lintas tiga keluarga instance, melawan **satu** positif terbukti:
> `n6_s0_mr1` di S2, +5.40 %.
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.** §B
> diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**. §A tidak ditulis ulang belakangan.
>
> Sesi ini **sepenuhnya offline**. Lengan 4× masih dilepas.
>
> **CATATAN MESIN, di muka:** `load average` awal sesi **3.65 / 3.42 / 3.54**
> pada 16 core. G15 diukur pada 3.6–7.1, G14 4.4–7.2, G13 6–8.5, G12 4.5–5.0,
> G11 3.3–3.7, G10 42. Detik **tidak** dibandingkan lintas sesi tanpa menyebut ini.
>
> ⚠️ **Ruang disk:** root 1.9 T pada **100 % (16 G tersisa)**. Artefak sesi ini
> ~20 KB per sel. Setiap kegagalan tulis JSON dibaca sebagai **disk penuh dulu**,
> bukan bug.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang, dan yang BEKU

Seluruh `p1_g12 §A0`, `p1_g13 §A0`, `p1_g14 §A0`, `p1_g15 §A0` berlaku utuh.
Yang ditegakkan ulang sebagai **sudah selesai, tidak dihitung ulang**:

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| **U4 LUNAS**, 9/9 | `p1_g14 §B5` | **TUTUP.** Tidak dibuka |
| Anggaran = **anggaran PENCARIAN** 120 s; wall dilaporkan terpisah | `p1_g14 §B1` | apa adanya |
| Lemma C **BUKAN** prapenyaring | `p1_g14 §B4` | `sched_armfull.py` **TETAP BEKU**, M0 tidak diulang |
| Tangga `\|P\|` 12 sel + sapuan G13 10 sel | `/tmp/g14_psweep_*.json`, `/tmp/g13_eval_*.json` | **dibaca**, tidak dihitung ulang |
| `DENSE` 2 sel, 80 instance-run, 0 kegagalan gerbang | `/tmp/g15_dense_DENSE_*.json` | dibaca |
| **R4 LULUS** 40/40: `gen_real_dense(band=2.00, at_p0 mati)` ≡ `gen_real` | `p1_g15 §B1` | **fondasi seluruh kisi sesi ini** (A2.1) |
| `max_states` tidak menggigit pada `forced-n2P3` / `forced-n2P4` | `p1_g15 §B4.1` | dibaca; `forced-n4P3` tetap TIDAK DIUKUR |
| Definisi **HARGA MUNCUL** | `p1_g14 §A2.2` | **verbatim, tidak ditulis ulang** (A2.2) |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi** (18 berkas): ketujuh belas
berkas `p1_g15 §A0`, **ditambah `test/dense_g15.py`**.

⚠️ `dense_g15.py` masuk daftar beku **karena ia yang diminta dipakai ulang**.
Konsekuensi yang dikunci sekarang: sudut baru **tidak boleh** menyalin
`cell()` / `report()` / `r3()`. Ia **mengimpor** keduanya dan hanya menyediakan
`build()`-nya sendiri (A5). `p1_g13 §B3` sudah mengukur harga alternatifnya:
**instrumen kedua dari BENTUK yang sama mewarisi cacat yang sama.**

Aturan `p1_g10 §A0` berlaku utuh: **bug di berkas beku dilaporkan sebagai
temuan, diperbaiki di tempatnya, pembekuannya dinyatakan gugur.** Tally:
G10 tiga, G11 nol, G12 dua, G13 dua, G14 nol, G15 nol (dua **temuan anggaran**).
Tugas 5 sesi ini **sengaja** mengaktifkan aturan itu untuk cacat G15 §B4 (A2.6).

### A1. Yang dikerjakan sesi ini, BERURUTAN

```
1. DENSE-P0XC          -- sudah dibangun, lulus R3 40/40. BIAYA DIUKUR DULU.
2. SUDUT p0-SAJA       -- (S1,S1,S2). Satu argumen, bukan generator baru.
3. DUA SUDUT SISANYA   -- (S2,S1,S2), (S2,S2,S1), kalau anggaran mengizinkan.
4. R5 -- REKONSTRUKSI  -- apakah sudut (S2,S2,S2) kisi ini = S2 SUNGGUHAN.
5. DUA CACAT ANGGARAN  -- keputusan, bukan audit lagi.
```

🔺 **Tugas 4 (R5) TIDAK ADA di prompt G16 dan ia ditambahkan di sini, sebelum
kode, dengan alasannya.** Ia bukan faktor kelima — ia **palang**, dan A2.1
menjelaskan kenapa tanpa dia keempat sudut baru tidak dapat dikaitkan pada
apa pun. Ia dijalankan **sebelum** sudut bertopeng-pose (urutan A1 adalah
urutan penulisan, bukan urutan eksekusi; urutan eksekusi dikunci di A2.5).

**TIDAK dibangun, dan tidak boleh menyelinap masuk:** solver ketiga, heuristik
sadar-lengan, perencanaan gerak lengan, `T_fold ≠ 0`, kapsul/ketebalan tautan,
policy kanonik selain `manip`, `c_arm` di luar {0.05, 0.10, 0.15, 0.20},
pelonggaran anggaran pencarian 120 s untuk angka utama, dan — larangan yang
prompt G16 tulis dengan huruf besar — **faktor kelima yang dikarang** kalau
kedelapan sudut nihil. A2.3 mengunci apa yang boleh ditulis sebagai gantinya.

---

### A2. KRITERIA YANG DIKUNCI SEBELUM KODE

#### A2.0 🔒 KISI FAKTORIAL, dan pengakuan bahwa ia BUKAN kisi ortogonal

Prompt G16 menulis sudut sebagai `(himpunan POSE, kolam TUGAS, letak p0)`.
Keadaan sebelum sesi ini:

```
(S1,S1,S1) = S1          432 bukti Delta_arm = 0     G13 + G14
(S2,S1,S1) = GEO-S2      0 positif / 14 terbukti     G14 B7
(S1,S2,S1) = DENSE       0 positif / 61 terbukti     G15 B3
(S2,S2,S2) = S2          SATU positif (+5.40 %)      G13 B5.4
(S1,S1,S2) = P0XC        BELUM DIUJI     <- tugas 2
(S1,S2,S2) = DENSE-P0XC  DIBANGUN, TIDAK DIUKUR      <- tugas 1
(S2,S1,S2) = GEO-P0XC    BELUM DIUJI     <- tugas 3
(S2,S2,S1) = GEO-DENSE   BELUM DIUJI     <- tugas 3
```

🔴 **Sumbu TUGAS dan sumbu p0 TIDAK bebas, dan itu terbaca dari kode, bukan
diduga.** `gen_real_dense` menolak kandidat yang **dapat dikerjakan di `p0`**
(perubahan 2, `dense_g15.py:130`). Jadi memindahkan `p0` **juga memindahkan
kolam tugasnya**. `p1_g15 §B1` sudah mengukur besarnya: `p0` S1 menyisakan
**246** node SR terpakai, `p0` S2 menyisakan **38** — beda **22×**.

🔒 **Konsekuensi yang dikunci sekarang, bukan dibela belakangan:** kontras `p0`
pada kolom tugas `S2` **bukan kontras bersih**. Yang boleh ditulis adalah
*"memindahkan `p0` ke `lin 0.80` sambil membiarkan penolakan `at_p0` mengikuti"*
— itu **definisi S2 sendiri**, dan itulah gunanya. Yang **dilarang** adalah
menulis *"efek `p0` sendirian"* dari selisih `DENSE` ↔ `DENSE-P0XC`.
Sudut yang memberi kontras `p0` **bersih** hanya satu: `P0XC` ↔ `S1`, karena di
kolom tugas `S1` penolakan `at_p0` **mati** dan `node_idx`-nya identik (R4).
Itu, dan bukan `DENSE-P0XC`, yang mengisolasi faktor (4).

#### A2.1 🔒 KRITERIA 1 — APA YANG MEMBUAT SEBUAH SUDUT DAPAT DIBANDINGKAN

Kunci kriteria nomor 1 prompt G16. **Ditulis sebelum generatornya ada.**

Sebuah sudut **dapat dibandingkan** dengan sudut lain kalau dan hanya kalau
keduanya keluar dari **satu keluarga generator yang sama**, berbeda **hanya**
pada argumen yang menamai sudutnya. Keluarga itu dikunci:

```
corner(pose, task, p0) :=
    inst = dense_g15.gen_real_dense(n, seed, mr,
               band        = 0.30 if task == S2 else 2.00,
               reject_at_p0= True  if task == S2 else False,
               p0_mode     = 'xc'  if p0   == S2 else 's1')
    if pose == S2:  inst = psweep_g14.shrink(inst, s2_mask() U {p0}, tag)
```

Tidak ada generator baru. `shrink` dan `s2_mask` **diimpor dari
`psweep_g14.py` yang BEKU**; `gen_real_dense` diimpor dari `dense_g15.py` yang
beku sesi ini. **Sudut baru = argumen + masker, sesuai perintah prompt.**

**Yang membuat kisi ini berdiri adalah R4 yang sudah LULUS:**
`corner(S1,S1,S1)` ≡ `sched.gen_real` pada 40/40 kunci (`p1_g15 §B1`). Jadi
titik pangkal kisi **terbukti** identik dengan keluarga S1 yang menghasilkan
432 bukti. Setiap sudut lain adalah S1 ditambah argumen bernama.

🔒 **R4′ — palang per-sudut, dikunci sekarang.** Untuk setiap sudut baru,
sebelum selnya dijalankan, diperiksa bahwa ia berbeda dari tetangganya **hanya**
pada sumbu yang menamainya:

| Pasangan | Yang WAJIB identik | Yang boleh berbeda |
|---|---|---|
| `S1` ↔ `P0XC` | `node_idx` (40/40) | `p0` saja |
| `GEO-S2` ↔ `GEO-P0XC` | `node_idx`, himpunan pose | `p0` saja |
| `DENSE` ↔ `GEO-DENSE` | `node_idx`, `p0` | himpunan pose saja |
| `DENSE-P0XC` ↔ `GEO-DENSE-P0XC`(=rekonstruksi S2) | `node_idx`, `p0` | himpunan pose saja |

Gagal ⟹ **TEMUAN**, sudut itu **tidak dijalankan**, dan sebabnya ditulis.

#### A2.1.1 🔴 R5 — APAKAH SUDUT `(S2,S2,S2)` KISI INI SAMA DENGAN S2 SUNGGUHAN

**Ini palang paling penting sesi ini, dan ia ditulis sebelum diukur.**

Membaca `sched_arm.gen_real_rotcrowded:576-625` di samping
`dense_g15.gen_real_dense:121-139` memperlihatkan **dua perbedaan teks** yang
belum pernah disebut di P1:

| | `gen_real_dense` (kisi ini) | `gen_real_rotcrowded` (S2 sungguhan) |
|---|---|---|
| predikat penerimaan tugas diuji pada | **grid PENUH** (`m = hd[0]` / `r[0].any`) | **di dalam masker pose** (`hd[0][keep]` / `r[0][keep]`) |
| `at_p0` diuji pada | `p0` di grid penuh | `p0` **di dalam masker** |
| tugas duplikat | **TIDAK disaring** (sengaja, demi R4) | **DISARING** (`cand not in chosen`) |

🔒 **Maka `corner(S2,S2,S2)` adalah REKONSTRUKSI S2, bukan S2.** R5 mengukur
apakah rekonstruksi itu sama:

> **R5:** `corner(S2,S2,S2)` vs `sa.gen_real_rotcrowded` pada 40 kunci —
> `node_idx`, `|P|`, `p0`, dan **`Δ_arm` pada `n6_s0_mr1` di `c_arm` 0.15**.

🔒 **Vonis R5 dikunci sekarang, sebelum angkanya dilihat:**

| Hasil R5 | Yang WAJIB ditulis |
|---|---|
| **≡ pada 40/40** | kisi memuat fenomenanya; kedelapan sudut dapat dikaitkan; interaksi dapat diuji |
| **≢, tetapi rekonstruksi tetap memberi positif `n6_s0_mr1`** | kisi memuat fenomenanya **lewat jalan lain**; perbedaannya disebut, dan setiap kalimat interaksi membawanya |
| **≢, dan rekonstruksi TIDAK memberi positif** | 🔴 **TEMUAN, dan ia lebih besar daripada keempat sudutnya**: satu-satunya positif P1 **tidak dapat dijangkau** oleh kisi faktorialnya, jadi kedelapan sudut **tidak dapat menjelaskannya**, dan kalimat "interaksi tiga-arah" **DILARANG** — yang benar adalah *harga S2 bergantung pada sesuatu di `gen_real_rotcrowded` yang bukan ketiga sumbu itu*, dengan dua kandidat terukur di tabel atas |

Tanpa R5, sesi ini berisiko mengisi delapan sudut dengan rapi dan menyimpulkan
"interaksi" tentang keluarga instance yang **tidak memuat contoh tandingannya**
— persis jebakan M3 `p1_g12`, dalam bentuk yang lebih halus.

#### A2.2 🔒 KRITERIA 2 — "HARGA MUNCUL", DIPAKAI APA ADANYA

Prompt G16: *"Definisi 'harga muncul' A2.2 G14 dipakai APA ADANYA. Jangan tulis
ulang."* Disalin verbatim, tidak dilonggarkan, tidak diperketat:

| | Definisi, dikunci |
|---|---|
| **HARGA MUNCUL** | ada ≥ 1 instance dengan `Δ_arm > 1e-9` di mana **sisi struktur DAN sisi penuh dua-duanya `exact`/`proved`** |
| **KALIMAT NASKAH** | positif naik monoton pada ≥ 3 titik berturut-turut. Positif tunggal terisolasi **BUKAN** kalimat naskah — ia mendapat kualifikasi satu-instance `p1_g13 §B5.4` |
| **DILARANG** | menyebut `Δ_arm` positif ketika sisi struktur tidak terbukti — selisih dua batas atas bukan selisih dua optimum |

Ambang 5.0 % (`p1_g13 §A2.4`) **tidak digeser**. `Δ_arm` tetap besaran
**DI DALAM satu instance** (dua solver, satu instance), jadi ia tidak menuntut
keterbandingan lintas instance — dan karena itu palang `lb` 10 % G14
**tetap tidak berlaku** di sini, persis alasan `p1_g15 §A2.2`. `lb` dilaporkan
sebagai **angka deskriptif** per sudut.

**Kontrol tetap BERPASANGAN-KUNCI** (`p1_g15 §A2.3`), bukan se-instance, dan
setiap kalimat yang memakainya menyebut itu.

#### A2.3 🔒 KRITERIA 3 — APA PERSIS YANG BOLEH DITULIS KALAU KEDELAPAN SUDUT NIHIL

**Ditulis sebelum hasilnya dilihat.** Ini kunci kriteria nomor 3, dan prompt
menuntut dua hal spesifik: apakah "artefak satu instance" boleh disebut, dan
bukti apa yang dituntut untuk menyebutnya.

**BOLEH ditulis:**

1. Angka nol dengan penyebutnya, per sudut, dan totalnya lintas P1.
2. "Tidak satu pun dari tiga faktor S2, **maupun kombinasi mana pun dari
   ketiganya yang diuji**, mereproduksi harga koordinasi lengan."
3. **"INTERAKSI TIGA-ARAH"** — **hanya kalau R5 ≡ atau R5 memberi positif.**
   Kalau R5 gagal total, kalimat ini **DILARANG** dan yang ditulis adalah baris
   ketiga tabel A2.1.1.
4. **"ARTEFAK SATU INSTANCE"** — hanya dengan bukti yang dikunci sekarang:

   🔒 **Syarat menyebut "artefak satu instance", ketiganya WAJIB:**
   - (a) `n6_s0_mr1` di S2 diulang pada **40 benih baru** dari generator yang
     **sama** (`gen_real_rotcrowded`), `c_arm` 0.15;
   - (b) hasilnya: **≤ 1 dari 40** memberi `Δ_arm` positif terbukti dua sisi;
   - (c) sisi struktur **dan** sisi penuh terbukti pada ≥ 20 dari 40, supaya
     nol-nya bukan nol karena ketidaktahuan (`p1_g14 §B9.5`).

   Kalau (a) dijalankan dan (b) atau (c) gagal, yang ditulis adalah angkanya
   apa adanya, **bukan** kata "artefak". Kalau (a) tidak dijalankan, kata
   "artefak" **DILARANG** dan diganti dengan **"TIDAK DIUKUR"**.
5. Kualifikasi "S2 adalah probe DESIGN, bukan konstanta fisik", selalu.

**DILARANG ditulis:**

1. **Faktor kelima yang dikarang.** Prompt G16 melarangnya dengan huruf besar.
   Tidak ada pengecualian sesi ini — kedua kandidat A2.1.1 (predikat dalam
   masker, penyaringan duplikat) **bukan** faktor: mereka **perbedaan
   generator**, mereka **terbaca dari kode**, dan R5 **mengukurnya**. Kalau R5
   tidak dijalankan, keduanya disebut **PERBEDAAN GENERATOR TIDAK DIUKUR**,
   bukan penjelasan.
2. "Kendala lengan tidak punya harga" — satu contoh tandingan terbukti
   melarang kalimat itu selamanya.
3. "S2 anomali" tanpa mekanisme terukur. Nol penjelasan ≠ bukti ketiadaan.
4. Merata-ratakan angka sudut mana pun ke baris utama S1 atau S2. Setiap sudut
   adalah **probe** (pola `p1_g11 §B3`).
5. Mengubah ambang 5.0 %, definisi `Δ_arm`, atau mengklaim positif dari sisi
   yang tidak terbukti.

#### A2.4 🔒 KRITERIA 4 — APA YANG DILAPORKAN KALAU LAGI-LAGI TIDAK SEMUANYA TERBUKTI

Per sel, **wajib**, terlepas dari hasilnya (`p1_g15 §A2.10`, dipakai apa adanya
+ dua kolom baru):

1. `Δ_struct`, `Δ_full`, `Δ_arm` sebagai kurungan dua-sisi + vonis.
2. Jumlah `exact`/`proved`, dan jumlah **TIDAK ADA JADWAL DITEMUKAN** dengan
   **DUA** angka (anggaran pencarian **dan** wall).
3. Jumlah jadwal gagal gerbang, per penjadwal. Target **0**. (R1/R2)
4. `load average` awal/akhir dan konkurensinya.
5. Sel/sudut yang **tidak dijalankan** disebut **NAMANYA** sebagai kalimat
   **TIDAK DIUKUR**. Dilarang "diperkirakan tidak berubah".
6. Instance **TIDAK DAPAT DIBANGUN** dengan namanya + fraksi `pool` tersingkir.
7. 🆕 **`dupes` per sel** — jumlah instance dengan tugas duplikat. A2.1.1
   menjadikannya kolom, bukan properti senyap.
8. 🆕 **Status R4′ dan R5** per sudut, sebelum angkanya.
9. Papan skor A4, dinilai. Aturan penilaian `p1_g15` berlaku: dugaan yang
   instrumennya tidak mengukur besaran yang didalilkan **tidak dinilai**;
   dugaan yang tidak membelah ketika kenyataannya membelah **MELESET**.

🔒 **Dan kalau proporsi terbukti sebuah sudut jatuh di bawah `GEO-S2`
(6–8 / 17 ≈ 40 %), nol positifnya dilaporkan dengan peringatan
`p1_g14 §B9.5`: nol dari sedikit bukti adalah bukti LEBIH LEMAH.** Dikunci di
muka supaya ia tidak dipilih belakangan menurut arah hasilnya.

#### A2.5 🔒 BIAYA DIUKUR SEBELUM 40 INSTANCE — SESI KESEMBILAN DENGAN JEBAKAN YANG SAMA

`p1_g8 §B4` … `p1_g15 §B2`: **delapan sesi berturut-turut** di mana yang lambat
ternyata **pemeriksanya**. G15 mengukur **88 %** wall di gerbang lengan — angka
tertinggi P1. Dan biaya **TIDAK monoton** dalam `|P|` (G14 §B3) **maupun** dalam
kepadatan tugas (G15 §B3.1). Tandanya **tidak dapat diduga** dari sudut lain.

**Probe:** kunci `n6_s0_mr1` (nama contoh tandingan) dan `n6_s4_mr0` (terburuk
diketahui), `c_arm = 0.20`, **satu probe per sudut baru sebelum selnya**.

🔒 **ATURAN KEPUTUSAN, ditulis sebelum angkanya dilihat**, per sudut:

| Probe terburuk (wall/instance) | Yang dijalankan untuk sudut itu |
|---|---|
| **< 60 s** | 2 sel: `c_arm` {0.15, 0.20} × 40 |
| **60–300 s** | 2 sel {0.15, 0.20} × 40, **tetapi** sudut itu memakan satu slot penuh dari tiga slot konkurensi |
| **> 300 s** | **1 sel**, `c_arm` **0.20 saja**; `c_arm = 0.15` **TIDAK DIUKUR**, disebut namanya |
| **gagal / tidak mengembalikan jadwal** | bukan penghenti otomatis: `p1_g15 §B2.1` sudah **mengukur** sebabnya (0 pose lolos keempat tahap, kategori bernama). Yang wajib: jalankan `park` pada sudut itu **kalau probe-nya `inf`**, dan laporkan pembagian fase. Sel berhenti **hanya** kalau profilnya BERBEDA dari kategori bernama itu |

🔒 **URUTAN EKSEKUSI, dikunci sekarang** (bukan urutan penulisan A1):

```
0. R4' + R5 gates                 -- MURAH, tanpa solver, wajib lebih dulu
1. probe DENSE-P0XC               -- lalu selnya
2. probe P0XC       (sudut bersih) -- lalu selnya
3. probe GEO-DENSE, GEO-P0XC      -- lalu selnya, kalau anggaran mengizinkan
4. cacat anggaran (A2.6)
5. ulangan 40-benih S2 (A2.3 syarat 4a) -- kalau anggaran mengizinkan
```

🔴 **Instance DILARANG dibuang untuk menghemat waktu.** Yang boleh dikorbankan
adalah **sel utuh**, disebut namanya. (`p1_g14 §A2.2`, apa adanya.)

#### A2.6 🔒 DUA CACAT ANGGARAN G15 §B4 — KEPUTUSAN, bukan audit lagi

Prompt G16 menuntut keputusan. Dikunci sekarang, sebelum menyentuh apa pun:

**(a) Konstruktor `Brute` di luar `W2_BUDGET`** (`verify_sched_coupled.py:178-186`:
`self.t0 = 0.0` di `__init__`, `self.dur` dihitung untuk seluruh `(g, U, p)`,
`t0` baru di-set di `solve()`).
🔒 **KEPUTUSAN: DIPERBAIKI DI TEMPATNYA.** Ia perbaikan satu baris
(`t0` mulai di `__init__`), berkas beku ⟹ `p1_g10 §A0` berlaku: pembekuan
`verify_sched_coupled.py` **dinyatakan gugur** dan dilaporkan sebagai temuan.
**Syarat wajib**: W2 dijalankan ulang **sesudah** perbaikan dan hasilnya harus
**identik** dengan yang dikutip (`p1_g9 §A3-K1`, 31 instance) — bahan bakar W2
punya `|P| ≤ 4`, jadi 56 panggilan prakomputasi, jauh di bawah 240 s. Kalau
hasilnya **berubah**, perbaikannya **dikembalikan** dan cacatnya dilaporkan
sebagai laten. Pembenaran "laten" saja **ditolak**: `p1_g15 §B5.1` baru saja
mengukur bahwa pembenaran tertulis tentang anggaran meleset **3.3×**.

**(b) Pemotongan `max_states` SENYAP** di `enum_opt`/`brute_arm`.
🔒 **KEPUTUSAN: DIUKUR DULU pada `forced-n4P3`**, kelas yang paling mungkin
memicunya dan yang G14 tidak sempat ukur, dengan metode kurungan G15 §B4.1
(tutup {1000, 10 000, 400 000}; kalau `(best, cnt)` tak berubah, enumerasinya
selesai jauh di bawah tutup).
- Kalau tutupnya **tidak menggigit** di sana juga ⟹ cacat tetap **laten**,
  dilaporkan dengan syarat pemakaiannya, berkas **tidak disentuh**.
- Kalau tutupnya **menggigit** ⟹ 🔴 **angka `vacuous`/`rerouted`/`binding`
  `p1_g14 §B6` GUGUR untuk kelas itu**, dan `seen`/flag terpotong
  **dikembalikan** dari `enum_opt`/`brute_arm` (perbaikan di tempatnya,
  pembekuan gugur, dilaporkan).
- 🔒 **Tutup wall untuk pengukuran ini: 1200 s, diperiksa DI ANTARA kandidat,
  dan batas overshoot-nya diturunkan dari iterasi yang SEBENARNYA dijalankan**
  — satu kandidat = 4 × `classify()` = 8 × (`brute_arm` + `enum_opt`), pelajaran
  `p1_g15 §B5.1`. Kalau habis: **TIDAK DIUKUR**, disebut namanya.

#### A2.7 🔒 KONKURENSI DAN BEBAN MESIN

`p1_g13 §A2.2` + `p1_g14 §A2.6` + `p1_g15 §A2.9` berlaku utuh:

1. **Konkurensi maksimum 3 pekerjaan latar.** Tidak dinaikkan ketika lambat.
2. `load average` dicatat awal dan akhir setiap sel; sel tanpa catatan beban
   **tidak dilaporkan**.
3. Pembenarannya beban **di bawah 16 core**, bukan angka konkurensinya.
4. Satu berkas per (sudut, `c_arm`); worker paralel punya sel **DISJOIN**.
5. Tulis **per baris dengan `flush=True`** ke berkas log **sendiri**, tunggu
   dengan **PID**. `tail` pada proses latar **menelan keluaran** — G15
   kehilangan satu audit 1500 s persis begitu, **sesudah** jebakan itu ditulis
   di prompt-nya sendiri.
6. `np.bool_` tidak JSON-serializable. `pkill -f` membunuh shell-nya sendiri.
7. Skrip di `/tmp` **tidak** mewarisi cwd repo: `sys.path` **ABSOLUT**.

---

### A3. PALANG — regresi yang harus lulus sebelum angka baru disebut

| | Apa yang diadu | Kalau gagal |
|---|---|---|
| **R1** | Setiap jadwal lolos `validate_coupled` **dan** `sa.arm_schedule_conflict` | sel **tidak dilaporkan sebagai angka**, dilaporkan sebagai kegagalan gerbang |
| **R2** | Lemma 3 (`ub ≥ lb`) pada setiap instance | idem |
| **R3** | Keempat syarat `p1_g15 §A2.1` diperiksa **di luar** generator, per instance | instance **TIDAK DAPAT DIBANGUN**, disebut namanya |
| **R4** | ✅ sudah LULUS di G15 (`gen_real_dense(2.00, at_p0 mati)` ≡ `gen_real`) | — |
| **R4′** 🆕 | Setiap sudut baru berbeda dari tetangganya **hanya** pada sumbu yang menamainya (tabel A2.1) | sudut itu **TIDAK DIJALANKAN**, temuan ditulis |
| **R5** 🆕 | `corner(S2,S2,S2)` vs `gen_real_rotcrowded` pada 40 kunci | **bukan** penghenti — ia **hasil**, dan vonisnya dikunci di tabel A2.1.1 |

### A4. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor berdiri di **30 meleset, 17 tepat** (`p1_g15 §B6`).
Prior **DUNIA**: LONGGAR, dan ia bertahan terhadap **ketiga** faktor S2 satu per
satu — 493 bukti dua-sisi, nol contoh tandingan, melawan satu positif.
Prior **KODE SENDIRI**, dipertajam G15 §B6 bacaan 1: pembacaan kode dapat
menemukan **LETAK** (D47 ✅), tidak dapat memperkirakan **AKIBAT** (D40, D41 ❌).
Dugaan kode hanya diajukan dalam bentuk pertama, atau sebagai **aritmetika**
(belahan yang D43 dan D46-SR buktikan berguna).

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D48** | `DENSE-P0XC` memberi **0** `Δ_arm` positif terbukti pada kedua `c_arm` | **dunia** | prior LONGGAR. Ia sudut ketiga yang membawa **dua** faktor S2 sekaligus dan tetap bukan S2 penuh |
| **D49** | `corner(S2,S2,S2)` **≢** `gen_real_rotcrowded` pada `node_idx`: **≥ 30 dari 40** kunci berbeda | **kode sendiri, TEKS** | bukan prediksi akibat, melainkan bacaan teks: predikat penerimaan diuji pada grid **penuh** vs di dalam `keep` (A2.1.1), jadi himpunan kandidat yang diterima berbeda kecuali setiap node yang terjangkau di grid penuh juga terjangkau di dalam masker — yang `p1_g15 §B1` sudah bantah (38 vs 246) |
| **D50** | Sudut **`P0XC`** (`p0` bersih, tugas S1, pose S1 penuh) memberi **0** positif **dan** `lb` median **TURUN** ≥ 5 % di bawah S1 | **dunia + aritmetika** | dua bagian, dinilai **terpisah dan ketat**. Bagian 2 aritmetika murni: `p0` pindah dari ujung rel (`lin 0.00`) ke tengah (`lin 0.80`), tugas tersebar di seluruh peta, dan 74–92 % makespan adalah gerak gantry (`p1_g7 §B5`) |
| **D51** | Instance ber-**duplikat** ≥ **8 dari 40** pada `DENSE-P0XC`, dan **lebih banyak** daripada pada `DENSE` | **kode sendiri, ARITMETIKA** | kolam SR terpakai 38 vs 246 (`p1_g15 §B1`); 6 tarikan dari 38 memberi P(duplikat) ≈ 1 − ∏(1 − k/38) ≈ 33 %, lawan ≈ 6 % dari 246. Diturunkan dari aritmetika ulang tahun, bukan dari alur kontrol |
| **D52** | Cacat `max_states` **TIDAK** menggigit pada `forced-n4P3` (tutup 400 000 tidak tercapai) | **kode sendiri, AKIBAT** | ⚠️ ditulis sadar bahwa ini bentuk **AKIBAT**, yang rekam jejaknya buruk (D40, D41 ❌; D47 ✅ hanya karena ia LETAK). Ditulis justru supaya belahan D29 diuji lagi pada bentuk yang lemah, bukan supaya dipercaya |

### A5. Berkas

| Berkas | Isi | Status |
|---|---|---|
| `test/corner_g16.py` | kisi 8 sudut, R4′, R5, `build` — **mengimpor** `cell`/`report`/`r3` dari `dense_g15`, `shrink`/`s2_mask`/`keys40`/`verdict`/`_exact` dari `psweep_g14` | **BARU** |
| `test/dense_g15.py` | dipakai ulang, **tidak diedit** | 🔒 **BEKU sesi ini** |
| `test/psweep_g14.py`, `test/m3_bind_g13.py`, seluruh `sched*.py` | — | 🔒 **BEKU** |
| `test/verify_sched_coupled.py` | A2.6(a): pembekuan **dinyatakan gugur** kalau perbaikan dilakukan | 🔓 bersyarat |
| `docs/p1_g16_interact.md` | dokumen ini | — |

### A6. Rule 6

**Rule 6 (anggaran 30 000 token/sesi) adalah PENGECUALIAN EKSPLISIT untuk sesi
protokol-panjang P1, dinyatakan di muka**, sebagaimana `p1_g12 §A8`,
`p1_g13 §A6`, `p1_g14 §A6`, `p1_g15 §A6`. Anggaran yang benar-benar mengikat dan
dilaporkan: **anggaran pencarian 120 s per instance**, **aturan biaya A2.5**,
dan **tutup 1200 s A2.6(b)**.

---

## B. Hasil terukur

> §A dikunci 2026-08-16 sebelum satu baris kode sesi ini dijalankan. Semua
> angka di bawah keluar sesudahnya. Setiap tempat di mana §B bertentangan
> dengan §A ditandai 🔺. **§A TIDAK ditulis ulang.**

_(diisi selama sesi)_
