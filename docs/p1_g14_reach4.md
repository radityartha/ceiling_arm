# P1 / G14 — REACH-4: kenapa S2, dan apa arti "tidak ditemukan".

> Sesi G14, 2026-08-16. Melanjutkan [p1_g13_sweep.md](p1_g13_sweep.md).
> G13 menutup **sapuan** (9+1 sel, S1 dan S2, `c_arm` 0.00–0.20, 0 kegagalan
> gerbang nyata) dan menutup **oracle** (M3 menyala: 11 instance mengikat,
> 0 `solver > W3`, 0 `solver < W3`). Yang tersisa adalah tiga pertanyaan yang
> baru **bisa diajukan** setelah G13, dan ketiganya **tidak punya sinyal
> error**: salahnya hanya terlihat sebagai angka yang masuk akal.
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.** §B
> diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**. §A tidak ditulis ulang belakangan.
>
> Sesi ini **sepenuhnya offline**. Lengan 4× masih dilepas.
>
> **CATATAN MESIN, di muka:** `load average` awal sesi **3.46 / 3.26 / 3.96**
> pada 16 core. G13 diukur pada 6–8.5, G12 pada 4.5–5.0, G11 pada 3.3–3.7,
> G10 pada 42. Detik **tidak** dibandingkan lintas sesi tanpa menyebut ini.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang, dan yang BEKU

Seluruh `p1_g12 §A0` dan `p1_g13 §A0` berlaku utuh. Yang ditambahkan sebagai
**sudah tegak** dan **tidak dihitung ulang**:

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| Sapuan penuh S1 + S2 × `c_arm` {0.00…0.20}, 40 instance/sel | `p1_g13 §B5` | dibaca dari `/tmp/g13_eval_{s1,s2}_{c}.json`, **tidak dijalankan ulang** |
| `Δ_arm = 0.0000` TERBUKTI: S1 kelima `c_arm` (147 bukti), S2 sampai 0.10 (73) | `p1_g13 §B5.3` | **basis pembanding** tugas 2 |
| `Δ_arm = +0.5000` (+5.40 %) pada `n6_s0_mr1` S2, `c_arm` 0.15 **dan** 0.20 | `p1_g13 §B5.4` | **satu-satunya** contoh tandingan; objek tugas 2 |
| M3 LULUS dan DISKRIMINATIF (11 mengikat, 0/0 kedua arah) | `p1_g13 §B4` | oracle dipakai apa adanya |
| `brute_arm` **sudah diperbaikai** (`continue`, bukan `break`) | `p1_g13 §B3` | angka M3 `p1_g12 §B8` **dilarang dikutip** |
| `arm_first_block` / `ArmTraj.moving()` **sudah diperbaiki** (titik tengah potongan) | `p1_g13 §B6` | jalan lambat sah sebagai oracle di S2 |
| Lemma C LULUS di 0.05 / 0.15 / 0.20 (107 perhentian, 0 tanpa pose parkir) | `p1_g13 §B5.5` | **dan ia HANYA menyertifikasi fase PARKIR** |
| M2a LULUS (δ ≤ 0.004 pada 2376² di kelima `c_arm`) | `p1_g13 §B6` | tutup |
| U4: `max_evade` **BUKAN** sebabnya (0/9, 10× node) | `p1_g13 §B8` | **jangan diulang** |
| `feasible_starts` tidak dilewati `U` → substitusi predikat mustahil | `p1_g12 §B2` | **jangan diturunkan ulang** |
| Lemma B teorema, **bukan** mekanisme (tebing 540×) | `p1_g12 §B4` | kedua sisi jalan di `c_clear = 0.0` |
| `\|P\|` S1 = **2376**, `\|P\|` S2 = **507** | terbaca dari artefak G13 | angka pangkal tugas 2 |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi:**
`sched.py`, `sched_heur.py`, `sched_coll.py`, `sched_coupled.py`,
`sched_arm.py`, `sched_armfull.py`, `test/verify_sched_exact.py`,
`test/verify_sched_coll.py`, `test/verify_sched_coupled.py`,
`test/gate_sched_coupled.py`, `test/verify_sched_arm.py`,
`test/eval_sched_arm.py`, `test/verify_sched_armfull.py`,
`test/eval_sched_armfull.py`, **`test/m3_bind_g13.py`**, **`test/diag_g13.py`**.

⚠️ `m3_bind_g13.py` dan `diag_g13.py` **masuk daftar beku sesi ini**: tugas 4
menjalankan `m3()` yang sudah ada, **tidak** menulis ulang generatornya, supaya
"tuas 3+4 tidak menaikkan fraksi mengikat" diukur oleh instrumen yang sama yang
melaporkan 11 mengikat.

Aturan `p1_g10 §A0` berlaku utuh: **bug di berkas beku dilaporkan sebagai
temuan, diperbaiki di tempatnya, pembekuannya dinyatakan gugur.** G10 tiga kali,
G11 nol, G12 dua, G13 dua.

🔓 **Satu-satunya berkas yang BOLEH diubah, dan hanya kalau tugas 3 memilih
(a):** `sched_armfull.py`. Kalau itu terjadi, **M0 dijalankan ulang adalah
GERBANG** (A3), bukan catatan.

### A1. Yang dikerjakan sesi ini, BERURUTAN

Urutannya wajib dan **tidak dilompati**:

```
1. KEPUTUSAN ANGGARAN 120 s   -- ditulis di A2.1 SEBELUM mengukur apa pun
2. KENAPA S2 DAN BUKAN S1     -- tangga |P| pada S1, DUA lengan percobaan
3. TIDAK ADA JADWAL DITEMUKAN -- diagnosis dulu, baru pilih (a) atau (b)
4. TUAS 3 + 4 M3              -- gen_forced, di c_arm = 0.05
5. U4 sisa                    -- grid seragam vs waktu mulai kontinu
```

**TIDAK dibangun, dan tidak boleh menyelinap masuk:** solver ketiga, heuristik
sadar-lengan, perencanaan gerak lengan, `T_fold ≠ 0`, kapsul/ketebalan tautan,
policy kanonik selain `manip`, `c_arm` di luar {0.05, 0.10, 0.15, 0.20},
pelonggaran anggaran **pencarian** 120 s untuk angka utama, dan perbaikan
performa pada berkas beku (`ArmView` cache MENGGANTUNG, `p1_g13 §B10.9`, tetap
utang terukur).

---

### A2. KRITERIA YANG DIKUNCI SEBELUM KODE

#### A2.1 🔒 TUGAS 1 — KEPUTUSAN ANGGARAN, DITULIS SEBELUM MENGUKUR APA PUN

`p1_g13 §B2a` mengukur bahwa "anggaran 120 s TERKUNCI" (`p1_g10 §K4`) **tidak
pernah** menjadi batas wall: ia hanya diperiksa di dalam loop B&B dan
`feasible_starts`; konstruktor pra-loop (`solve_exact`, `_repair_ub`, `_dive`
akar, `arm_serial_ub`) tidak melihatnya. **97 dari 400** instance G13
melewatinya, dengan overshoot maksimum **317.4 s**.

**KEPUTUSAN, dikunci sekarang, sebelum satu pengukuran sesi ini:**

> ## ➜ **(b) DIPILIH.** Anggaran 120 s **didefinisikan ulang** sebagai
> ## **ANGGARAN PENCARIAN**, dan **wall dilaporkan terpisah**.

**Harga opsi (a), dihitung sebelum memilih supaya pilihannya bukan kenyamanan.**
Menegakkan 120 s sebagai batas wall berarti menyisipkan pemeriksaan waktu ke
dalam konstruktor pra-loop, yaitu **mengubah `sched_coupled.py` dan
`sched_armfull.py` yang beku** — sehingga **setiap** angka yang pernah dikutip
harus dijalankan ulang, bukan hanya yang overshoot:

| Sumber | Instance-run yang punya artefak | Solver-run (2 per instance-run) |
|---|---|---|
| `/tmp/g9_eval.json` | 60 | 120 |
| `/tmp/g10_eval.json` + `_prefix5` | 280 | 560 |
| `/tmp/g11_eval_s{1,2}.json` | 400 | 800 |
| `/tmp/g12_eval_s1.json` | 40 | 80 |
| `/tmp/g13_eval_*.json` (10 sel) | 400 | 800 |
| **subtotal berartefak (G9–G13)** | **1180** | **2360** |
| G7 (V2 96 + V3 8) dan G8 (himpunan evaluasi 50) — **artefaknya sudah tidak ada di `/tmp`**, jadi harus dibangkitkan ulang dari generator | ≈ 154 | ≈ 308 |
| **TOTAL** | **≈ 1334** | **≈ 2670** |

Ditambah: **tujuh dokumen** (`p1_g7`…`p1_g13`) yang setiap tabelnya ikut
berubah, **M0 harus lulus ulang** karena solvernya disentuh, dan seluruh vonis
dua-sisi bergeser karena instance yang tadinya mendapat 437 s wall kini
dipotong. Itu bukan "menjalankan ulang sapuan" — itu **membangun ulang seluruh
rantai bukti P1**.

**Alasan (b) BUKAN sekadar bahwa (a) mahal.** Tiga alasan substantif, dan
ketiganya sudah terukur di `p1_g13 §B10.1`:

1. **Tidak satu pun angka `Δ` bergantung pada wall.** Setiap makespan yang
   dikutip adalah **bukti optimalitas** (bebas anggaran) atau **batas atas yang
   sah** (bebas anggaran). Vonis dua-sisi bergantung pada `exact`/`proved`,
   yaitu pada anggaran **pencarian** — yang memang seragam untuk semua
   instance, dan memang 120 s.
2. **(a) merusak sesuatu yang sekarang benar.** Memotong `arm_serial_ub` di
   tengah membuat "tidak ada batas atas" bergantung pada **beban mesin**, yaitu
   persis penyakit tutup-60-pose G11 yang `p1_g12 §B6` cabut dan yang membeli
   12 instance. Batas atas yang hilang karena jam adalah batas atas yang tidak
   bisa direproduksi.
3. **Overshoot-nya bukan properti solver, ia properti `c_arm`.** Pada sel utama
   G12 (S1, `c_arm = 0.05`) overshoot maksimum **0.1 s**; S2 **nol di kelima
   sel**. Ia muncul hanya di `c_arm ≥ 0.10` pada S1, di mana `arm_serial_ub`
   menyapu ribuan pose. Menegakkan wall akan **menghapus** justru pengukuran
   yang membuat gejalanya terlihat.

**Yang WAJIB berubah karena (b) dipilih — ini harganya, dan ia bukan nol:**

| # | Perubahan | Cakupan |
|---|---|---|
| K1 | Frasa "anggaran 120 s per instance" → **"anggaran pencarian 120 s"** di setiap tempat ia muncul | naskah, `p1_g7`–`p1_g14` |
| K2 | Label `TIDAK ADA JADWAL DITEMUKAN (anggaran 120 s)` → **`TIDAK ADA JADWAL DITEMUKAN (pencarian 120 s; wall X.X s)`** — **dua** angka, selalu | `p1_g13 §B5.5`, `report()` |
| K3 | Setiap tabel sel melaporkan **wall mean/maks DAN jumlah instance dengan wall > anggaran pencarian** | tabel §B5 G13, tabel §B sesi ini |
| K4 | Pernyataan eksplisit bahwa wall **tidak dibatasi** dan kenapa | naskah, sekali, di bagian metode |

🔴 **DILARANG mencampur.** Tidak ada baris sesi ini yang menegakkan wall pada
sebagian instance dan tidak pada yang lain. Kalau sebuah pengukuran sesi ini
butuh tutup wall (tugas 2 dan 4 punya tutup **RUN**, bukan tutup per instance),
tutupnya disebut sebagai **tutup RUN** dan sel yang tidak tercapai dilaporkan
**TIDAK DIUKUR**, bukan diperkirakan.

🔒 **Verifikasi K2/K3 adalah bagian sesi ini, bukan janji:** `report()` beku,
jadi pelabelan ulang dilakukan di berkas **baru** yang membaca artefak G13 apa
adanya dan mencetak label yang benar. Kalau tidak dijalankan → **TIDAK DIUKUR**.

#### A2.2 🔒 TUGAS 2 — KENAPA S2, dan APA yang membuatnya "MENJAWAB"

Ini kunci kriteria nomor 2 dari prompt G14, dan seluruh gunanya adalah supaya
"harga muncul" bisa dibedakan dari "ini instance lain". Ini **persis jebakan M3
G12**, jadi kriterianya ditulis sebelum generatornya ada.

**FAKTA PANGKAL, terbaca dari artefak G13, bukan diingat:**

| | S1 (`gen_real`) | S2 (`gen_real_rotcrowded`) |
|---|---|---|
| `\|P\|` | **2376** (33 lin × 72 rot penuh) | **507** |
| Himpunan pose | seluruh grid | **pita**: `\|sin rot\| ≥ 0.525` **DAN** `\|lin − 0.80\| ≤ 0.30`, plus `p0` |
| Kolam tugas | seluruh peta | hanya node dengan `\|x − 0.80\| ≤ 0.30` |
| Tugas di `p0` | boleh | **DITOLAK** (`at_p0` → reject) |
| `Δ_struct` | [0, 0.4679] | **[0, 0] GRATIS** di kelima `c_arm` |
| `Δ_arm > 0` terbukti | **0** dari 147 | **1** (`n6_s0_mr1`, 0.15 dan 0.20) |

🔴 **CONFOUND, DINAMAI DI MUKA — S2 berbeda dari S1 dalam TIGA hal, bukan
satu.** Hipotesis prompt ("harga muncul ketika ruang keluar dibatasi") hanya
menyebut yang pertama:

1. **Kardinalitas** `|P|`: 507 vs 2376 (4.69×).
2. **Geometri** himpunan pose: pita sempit di sekitar rotasi ±90°, di mana
   lengan menyapu melintasi pemisahan gantry 0.72 m — bukan sampel acak.
3. **Kepadatan tugas**: seluruh tugas di pita `x_c ± 0.30` dan **tidak ada**
   yang bisa dikerjakan di `p0`, jadi kedua gantry **dipaksa** ke wilayah yang
   sama.

Menyusutkan `|P|` secara acak pada S1 menguji **(1) saja**. Karena itu
percobaan ini punya **DUA LENGAN**, dan keduanya dikunci sekarang:

| Lengan | Apa yang diubah dari S1 | Apa yang dijawab |
|---|---|---|
| **KARDINALITAS** | `\|P\|` disusutkan dengan **subsampel acak berseed** dari grid penuh; tugas, peta, `p0` **TIDAK berubah** | apakah **jumlah** pose yang menentukan harga |
| **GEOMETRI** | himpunan pose diganti dengan **masker S2 yang persis** (`\|P\| = 507`, pita yang sama); tugas S1 **TIDAK berubah** | apakah **bentuk** ruang keluar yang menentukan harga |

Lengan GEOMETRI adalah yang memisahkan (2) dari (1): ia punya `|P|` **identik**
dengan lengan KARDINALITAS rung 507, dan berbeda **hanya** pada pose mana yang
dipertahankan. Kalau keduanya dijalankan dan hanya satu yang memberi harga,
pertanyaannya **terjawab**. Kalau hanya satu lengan sempat dijalankan, itu
dilaporkan **TIDAK DIUKUR** untuk yang lain — dilarang menulis "diperkirakan".

🔺 **(3) tetap TIDAK terkendali dan itu ditulis sekarang, bukan belakangan.**
Kedua lengan memakai tugas S1, jadi kepadatan tugas tidak pernah diuji. Kalau
kedua lengan nihil, kandidat yang **tersisa dan tidak tersingkir** adalah
kepadatan tugas, dan itulah yang ditulis sebagai batasan — bukan "lengan tidak
punya harga".

**🔒 TANGGA `|P|`, dikunci sekarang** (S2 = 507 ada di dalamnya sebagai
jangkar):

```
KARDINALITAS:  |P| in {1188, 594, 507, 297, 148}     (2376 = baris G13, tidak diulang)
GEOMETRI:      |P| = 507  (masker gen_real_rotcrowded, tugas gen_real)
c_arm:         {0.15, 0.20}   -- HANYA di mana contoh tandingan S2 hidup
instance:      40 kunci S1 yang sama persis (n in {4,6} x seed 0..9 x mr in {0,1})
```

**🔒 APA YANG MEMBUAT SEBUAH SUSUTAN "TERKENDALI"** — empat syarat, semuanya
diperiksa per instance per rung, dan pelanggaran apa pun membuat instance itu
dilaporkan **TIDAK DAPAT DIBANGUN pada rung itu**, bukan dibuang diam-diam:

1. `p0` **selalu** dipertahankan (start state kedua gantry tetap ada).
2. Setiap tugas tetap **dapat dijangkau** oleh setidaknya satu gantry di
   setidaknya satu pose yang dipertahankan; tugas MR tetap punya pose
   irisan-ketat. (Kalau tidak: resampel, `≤ 200` percobaan berseed, lalu
   menyerah dan lapor.)
3. Subsampelnya **deterministik**: `rng = default_rng(70000 + 100*seed + rung)`.
4. **Pergeseran `lb` dilaporkan.** Batas bawah takterkopel `solve_exact` boleh
   naik ketika pose dibuang — itu **wajar** dan bukan cacat, tapi rung yang
   menaikkan `lb` median lebih dari **10 %** dinyatakan **BUKAN LAGI INSTANCE
   YANG SAMA**, dan `Δ_arm` positif di sana **tidak** boleh dikutip sebagai
   jawaban tugas 2. Ambang 10 % dikunci sekarang.

**🔒 APA YANG MEMBUAT TUGAS 2 "MENJAWAB"** — ditulis sebelum generatornya ada:

| | Definisi, dikunci |
|---|---|
| **HARGA MUNCUL** | ada ≥ 1 instance dengan `Δ_arm > 1e-9` di mana **sisi struktur DAN sisi penuh dua-duanya `exact`/`proved`** |
| **ITU RUNG-nya, bukan instance-nya** | `(n, seed, mr)` yang **sama** punya `Δ_arm = 0` **terbukti** pada rung penuh `\|P\| = 2376` — yang berlaku untuk keempat puluh, di kelima `c_arm` (147 bukti) |
| **KALIMAT NASKAH** | jumlah positif **naik monoton** ketika `\|P\|` turun, pada ≥ 3 rung berturut-turut. Positif tunggal terisolasi **BUKAN** kalimat naskah — ia mendapat kualifikasi satu-instance yang sama persis dengan `p1_g13 §B5.4` |
| **JAWABAN NEGATIF ADALAH JAWABAN** | 0 positif sampai `\|P\| = 148` pada lengan KARDINALITAS ⟹ **kardinalitas bukan mekanismenya**, dan itu ditulis sebagai temuan, bukan sebagai kegagalan |
| **PEMISAHAN** | KARDINALITAS nihil + GEOMETRI positif ⟹ **bentuk**, bukan jumlah. Keduanya positif ⟹ jumlah cukup. Keduanya nihil ⟹ kandidat tersisa adalah kepadatan tugas (3), **TIDAK DIUKUR** |

**🔒 YANG DILARANG:**

1. Menyusutkan `|P|` sampai instance **tidak layak** dan menyebut itu harga.
2. `c_arm` di luar {0.15, 0.20} pada tugas 2.
3. Merata-ratakan angka rung ke dalam baris utama S1. Rung adalah **probe**,
   pola `p1_g11 §B3`, dan setiap angkanya membawa kualifikasi itu.
4. Menyebut `Δ_arm` positif ketika sisi **struktur** tidak terbukti — selisih
   dua batas atas bukan selisih dua optimum.
5. Menggeser ambang 5.0 % atau ambang `lb` 10 %.

**🔒 BIAYA SAPUAN DIUKUR SEBELUM 40 INSTANCE — SESI KETUJUH DENGAN JEBAKAN YANG
SAMA.** `p1_g8 §B4`, `p1_g9 §B3`, `p1_g10 §B3-1`, `p1_g11 §B4`, `p1_g12 §B4`,
`p1_g13 §B2` — enam sesi berturut-turut di mana **yang lambat ternyata
pemeriksanya**, dan G13 mengukur `arm_serial_ub` **3500× lebih mahal** di ujung
sapuan daripada di titik yang G12 memverifikasinya. Jadi:

**Probe:** `n6_s0_mr1` (nama yang sama dengan contoh tandingan S2) dan
`n6_s4_mr0` (terburuk diketahui), pada rung **148** dan **1188**,
`c_arm = 0.20` — **ujung** tangga di kedua arah, karena arah biayanya melawan
dirinya sendiri: `|P|` kecil membuat `arm_serial_ub` murah (sedikit pose
disapu) **tetapi** membuat ruang keluar sempit sehingga pencarian lebih sering
harus benar-benar jalan.

🔒 **ATURAN KEPUTUSAN, ditulis sebelum angkanya dilihat:**

| Terukur pada probe (wall per instance) | Yang dilakukan |
|---|---|
| kedua probe **< 60 s** | tangga penuh 5 rung × 2 `c_arm` dijalankan, konkurensi A2.6 |
| probe terburuk 60–300 s | tangga dipotong jadi **3 rung** {1188, 507, 148} × 2 `c_arm`; rung yang dibuang disebut namanya **TIDAK DIUKUR** |
| probe terburuk **> 300 s** | hanya rung **507** (KARDINALITAS + GEOMETRI) × 2 `c_arm` dijalankan — itu perbandingan minimum yang masih menjawab pertanyaannya, karena `\|P\|` identik di kedua lengan |
| probe **gagal / tidak mengembalikan jadwal** | itu **temuan**, ditulis sebagai judul, dan tangga berhenti sampai sebabnya diketahui |

🔴 **Instance DILARANG dibuang untuk menghemat waktu.** Yang boleh dikorbankan
adalah **rung utuh**, disebut namanya.

#### A2.3 🔒 TUGAS 3 — "TIDAK ADA JADWAL DITEMUKAN": DIAGNOSIS DULU, BARU PILIH

Prompt menuntut memilih (a) atau (b) **dengan mengukur, bukan menebak**. Maka
yang dikunci di sini bukan pilihannya — yang dikunci adalah **pengukuran yang
memilih**, dan **aturan keputusannya**, keduanya sebelum angkanya ada.

**Yang diukur:** pada instance tanpa jadwal (S1 `c_arm = 0.15`: `n4_s4_mr1`,
`n6_s1_mr1`, `n6_s2_mr1`, `n6_s4_mr1`; S2 `c_arm = 0.20`: 11 instance, diambil
**4 pertama** kalau wall mengikat, dan sisanya disebut TIDAK DIUKUR) —
**profil penolakan per pose parkir**, empat tahap, sesuai urutan di
`arm_serial_ub`:

| Tahap | Apa yang menolak | Fase |
|---|---|---|
| **S1** | `sc.schedule_conflict(trial)` | **PARKIR** (struktur) |
| **S2** | `schedule_arm_conflict(trial)` | **PARKIR** (lengan) |
| **S3** | `sc.schedule_conflict(full)` | **EKOR SERIAL** (struktur) |
| **S4** | `schedule_arm_conflict(full)` | **EKOR SERIAL** (lengan) |

Diukur untuk **kedua** arah (`lead`, `park`), atas **seluruh 2376** (atau 507)
pose, plus: **berapa pose yang lolos S1–S2 tetapi jatuh di S3–S4** — itu angka
yang langsung memutuskan apakah Lemma C bisa jadi prapenyaring.

🔒 **ATURAN KEPUTUSAN, ditulis sebelum angkanya dilihat:**

| Terukur | Pilihan | Kenapa |
|---|---|---|
| ≥ 80 % penolakan di **S3–S4** (ekor serial) | **(b)** | Lemma C **hanya** menyertifikasi fase parkir (`p1_g13 §B5.5` Pertentangan 2). Prapenyaring yang menyaring fase yang bukan penyebabnya tidak membeli apa pun, dan mengubah berkas beku untuk itu adalah harga tanpa barang |
| ≥ 80 % penolakan di **S2** (parkir, lengan) | **(a) BOLEH dipertimbangkan** | di situ Lemma C memang predikatnya. Tapi lihat palang di bawah |
| campuran | **(b)**, dan pembagiannya dilaporkan sebagai angka | |
| ada pose yang lolos **keempat** tahap tetapi `arm_serial_ub` tetap mengembalikan `inf` | **BUG di berkas beku** → temuan, pembekuan gugur, `p1_g10 §A0` | |

🔴 **PALANG SEBELUM (a) BOLEH DIPAKAI — dan ia kemungkinan besar TIDAK BISA
DILEWATI, yang dinyatakan sekarang supaya bukan alasan belakangan.** Prompt
menuntut: *"BUKTIKAN ia syarat perlu sebelum memakainya"*. Pembacaan kode
mengatakan ia **tidak** perlu, dan alasannya spesifik: `lemma_c()` menuntut pose
parkir bebas terhadap **SETIAP** perhentian bertugas `lead`, sedangkan gantry
parkir baru **tiba** di pose itu pada `T0 = leg_duration(cur, P[p])`. Perhentian
`lead` yang **selesai sebelum `T0`** tidak pernah bertemu gantry parkir di pose
itu. Jadi himpunan Lemma C adalah **subhimpunan tegas** dari himpunan yang sah,
dan memakainya sebagai saringan keras bisa **membuang pose parkir yang valid** —
yaitu memproduksi "tidak ditemukan" yang **tidak benar**, persis penyakit yang
tugas ini ada untuk menyembuhkan.

🔒 Maka: **(a) hanya boleh dipakai kalau ketidak-perluan itu DIBANTAH oleh
pengukuran** — yaitu ditunjukkan bahwa pada instance-instance ini tidak ada pose
yang diterima `arm_serial_ub` tetapi ditolak `lemma_c`. Kalau **ada** satu saja,
Lemma C sebagai saringan keras **gugur**, dan yang tersisa adalah memakainya
sebagai **URUTAN** — yang sah (urutan tidak mengubah himpunan yang bisa
diterima) tetapi **mengubah pose mana yang ditemukan lebih dulu**, dan karena
loop `break` pada penerimaan pertama, ia **mengubah nilai batas atasnya**. Itu
bukan optimisasi netral dan tidak boleh disebut begitu.

🔒 **Kalau (a) tetap dipilih:** M0 dijalankan ulang (31/31 W2 **dan** 40/40 S1,
`p1_g12 §B2`) adalah **GERBANG**. **Tidak ada angka baru sebelum ia lulus**, dan
kalau ia tidak sempat dijalankan, seluruh hasil tugas 3 dilaporkan
**TERTUNDA** — bukan dilaporkan dengan catatan.

#### A2.4 🔒 TUGAS 4 — TUAS 3 + 4, dan pertanyaannya BUKAN "bisakah dibangun"

`p1_g13 §B4` sudah membangun 11 instance mengikat **tanpa** tuas 3 dan 4 — yang
membukanya adalah tuas 6 (`c_arm`). Jadi pertanyaan A2.3 G13 sudah lunas, dan
yang tersisa **berbeda**:

> **Apakah menutup jalan keluar "MENUNGGU" menaikkan fraksi mengikat di
> `c_arm = 0.05`, yaitu di NILAI UTAMA?**

Itu pertanyaan yang lain, karena 11 instance mengikat G13 **semuanya** di
`c_arm ≥ 0.10`, dan pada 0.05 hasilnya **0** — mereproduksi G12. Kalau tuas 3+4
memberi mengikat di 0.05, maka "harga nol di nilai utama" adalah pernyataan
tentang **bentuk instance**, bukan tentang clearance. Kalau tidak, ia pernyataan
yang lebih kuat.

🔒 **Dikunci:**

1. Dijalankan lewat `test/m3_bind_g13.py m3` yang **BEKU**. Definisi `BINDING`,
   diskretisasi `brute_arm`, dan generator `gen_forced` **tidak disentuh**.
2. **Angka yang menjawab:** jumlah `BINDING` pada `c_arm = 0.05` di kelas-kelas
   `forced-*`, dibandingkan dengan **0** pada `G12-baseline` di 0.05.
3. **Lantai usaha A2.3 G13 berlaku utuh** dan statusnya dilaporkan apa adanya:
   ≥ 5 kelas, ≥ 300 kandidat, ≥ 40 brute-force dua arah. G13 mencapai
   1 / 10 / 32 dan mengatakannya. Kalau sesi ini juga tidak mencapainya, itu
   ditulis dengan angkanya.
4. **Pembagian tiga arah** (`vacuous` / `rerouted` / `binding`) dilaporkan per
   kelas **apa pun hasilnya** — itu kalimat naskahnya, terlepas dari kolom
   ketiga.
5. **Tutup RUN 2400 s** (A2.1 K-larangan: ini tutup RUN, bukan tutup per
   instance). Kelas yang tidak tercapai dilaporkan **TIDAK DIUKUR** dengan
   namanya.
6. `gen_forced` mengembalikan `None` pada semua seed adalah **hasil**: tuas
   tidak tersedia di sel ini. `p1_g13 §B10.6` sudah mengukur tuas 3 **tipis**
   (6 + 6 dari 864; 90 % node dijangkau kedua gantry).

#### A2.5 🔒 TUGAS 5 — U4, dan apa yang dianggap MENUTUP

`p1_g13 §B8` menyingkirkan `max_evade` (0/9, 10× node) dan `p1_g10` sudah
menyingkirkan penghalusan grid (0/9). **Satu kandidat tersisa:** `Brute._starts`
menyampel waktu mulai pada grid **seragam** `{t_min, t_min + 0.25, …} ∪ {hi}`
berjangkar di jam gantry sendiri, sementara `feasible_starts` solver menghitung
waktu mulai layak **kontinu**.

🔒 **Uji, dikunci:** untuk setiap dari 9 instance `solver < W2`, **putar ulang
jadwal solver** dan pada setiap perhentian periksa apakah `depart` ada di
`Brute._starts(t[g], tr[h])` **pada keadaan rekursi yang sama** (jam gantry
sendiri + trajektori lawan yang sudah dikomit), toleransi `1e-9`.

| Terukur | Vonis |
|---|---|
| ≥ 1 `depart` di **luar** grid pada **≥ 5 dari 9** | kandidat **DIKONFIRMASI** sebagai mekanisme; celah `solver < W2` adalah **artefak diskretisasi W2**, bukan kegagalan solver, dan U4 lunas |
| semua `depart` **di dalam** grid pada **kesembilan** | kandidat **TERBANTAH**; celahnya tidak punya kandidat tersisa yang bernama sama sekali — hasil yang **lebih tidak menyenangkan**, dan ditulis begitu |
| di antaranya (1–4 dari 9) | **SEBAGIAN**; jumlahnya dilaporkan, dan tidak dipromosikan ke "lunas" |

🔴 `solver < W2` **bukan** kegagalan soundness menurut `p1_g9 §A3-K1` yang
sudah dikunci: `solver > W2` adalah kegagalan solver, `solver < W2` adalah grid
yang kasar. Uji ini **menguji aturan itu**, tidak mengasumsikannya.

#### A2.6 🔒 KONKURENSI DAN BEBAN MESIN

Aturan `p1_g13 §A2.2` berlaku utuh, dengan satu penajaman yang G13 sendiri
tuliskan:

1. **Konkurensi maksimum 3 pekerjaan latar** sepanjang sesi. Tidak dinaikkan
   ketika terlihat lambat.
2. `load average` dicatat **awal dan akhir** setiap sel/rung, dan masuk tabel
   §B. Sel tanpa catatan beban **tidak dilaporkan**.
3. **Pembenarannya adalah beban di BAWAH jumlah core (16), bukan angka
   konkurensinya.** Kalau load melewati 16, itu dicatat dan sel yang terkena
   dilabeli.
4. Setiap pekerjaan latar menulis **per baris ke berkasnya sendiri**
   (`flush=True`), satu berkas per (lengan, rung, `c_arm`). `tail` pada proses
   latar menelan keluaran sampai selesai — `p1_g13` sudah mengukur itu.
5. `np.bool_` tidak JSON-serializable.

#### A2.7 🔒 APA YANG DILAPORKAN KALAU LAGI-LAGI TIDAK SEMUANYA BISA DIBUKTIKAN

Kunci kriteria nomor 4. Per rung/sel, **wajib**, terlepas dari hasilnya:

1. `Δ_struct`, `Δ_full`, `Δ_arm` sebagai kurungan dua-sisi + vonis.
2. Jumlah `exact` / `proved`, dan jumlah **TIDAK ADA JADWAL DITEMUKAN** dengan
   **DUA** angka (anggaran pencarian **dan** wall) — A2.1 K2.
3. Jumlah jadwal yang gagal gerbang, per penjadwal. Target **0**. (R1/R2)
4. `load average` awal/akhir dan konkurensinya.
5. Rung/kelas yang **tidak dijalankan** disebut namanya sebagai kalimat
   **TIDAK DIUKUR**. Dilarang menulis "diperkirakan tidak berubah".
6. Pergeseran `lb` per rung (A2.2 syarat 4), dan instance yang **tidak dapat
   dibangun** pada rung itu, dengan namanya.
7. Papan skor A4, dinilai. Dugaan yang dinilai memakai solver yang belum lulus
   gerbangnya **ditandai TERTUNDA**, tidak dinilai.

---

### A3. PALANG — regresi yang harus lulus sebelum angka baru disebut

| | Apa yang diadu | Kalau gagal |
|---|---|---|
| **R0** | Pelabelan ulang A2.1 K2/K3 atas artefak G13 mereproduksi `p1_g13 §B5` **persis** pada kolom yang tidak diubah (`Δ`, `exact`, route) | pembacanya berubah semantik → **temuan**, sesi berhenti |
| **R1** | Setiap jadwal yang dikembalikan pada setiap rung lolos `validate_coupled` **dan** `sa.arm_schedule_conflict` | rung itu **tidak dilaporkan sebagai angka**, dilaporkan sebagai kegagalan gerbang |
| **R2** | Lemma 3 (`ub ≥ lb`) pada setiap rung | idem |
| **R3** 🆕 | Setiap instance rung memenuhi keempat syarat A2.2 (p0 ada, semua tugas terjangkau, seed deterministik, `lb` dilaporkan) | instance itu **TIDAK DAPAT DIBANGUN**, disebut namanya |
| **M0** | **hanya kalau tugas 3 memilih (a):** `solve_armfull(c_arm = −inf)` ≡ `solve_coupled2` pada 31/31 W2 dan 40/40 S1 | **GERBANG**. Tidak ada angka baru. Tugas 3 dilaporkan TERTUNDA |

R1 dan R2 sudah terpasang di dalam harness evaluasi (kolom `gate` dan
`arm_slow`); ini pernyataan bahwa kolomnya **dibaca**, bukan kode baru.

### A4. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor berdiri di **25 meleset, 12 tepat** (`p1_g13 §B9`).
Prior **DUNIA**: LONGGAR — dengan **satu** contoh tandingan terbukti
(`p1_g13 §B5.4`), yang datang dari **himpunan pose dibatasi**, bukan dari
clearance yang dinaikkan.
Prior **KODE SENDIRI**: arahnya **tidak lagi bisa disebut** (5 dari 9; dua sesi
meleset "terlalu lambat", G13 dua meleset "terlalu cepat"). Yang bertahan lebih
sempit: **yang mahal selalu PEMERIKSA** — enam sesi berturut-turut.

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D37** | Overshoot wall di atas anggaran pencarian pada S1 `c_arm ≥ 0.15` **didominasi `arm_serial_ub`**: ≥ 80 % dari kelebihan detik ada di satu panggilan itu | **kode sendiri** | `p1_g13 §B2b` mengukur 138 s dari 139.92 s pada satu probe. Ditulis sebagai dugaan karena satu probe bukan 16 instance, dan karena prior kode-sendiri sudah meleset ke **kedua** arah |
| **D38** | Lengan **KARDINALITAS** memberi **0** `Δ_arm` positif terbukti pada **setiap** rung sampai `\|P\| = 148` | dunia | prior LONGGAR, dan `p1_g13 §B5.3` memberi 147 bukti nol pada `\|P\| = 2376`. Kalau jumlah pose saja yang penting, S1 pada 507 seharusnya sudah berperilaku seperti S2 — dan dugaan ini mengatakan **tidak** |
| **D39** | Lengan **GEOMETRI** (`\|P\| = 507`, masker S2, tugas S1) memberi **≥ 1** `Δ_arm` positif terbukti pada `c_arm ∈ {0.15, 0.20}` | dunia | ini dugaan yang **melawan** prior LONGGAR dan ditulis begitu dengan sadar. Mekanisme yang didalilkan: pita ±90° adalah tempat lengan menyapu melintasi pemisahan 0.72 m, jadi bukan **sedikit** pose melainkan **pose yang salah**. D38 + D39 bersama = "bentuk, bukan jumlah" |
| **D40** | Profil tugas 3: ≥ 80 % penolakan pose parkir ada di **EKOR SERIAL** (tahap S3–S4), bukan di fase parkir | **kode sendiri** | `p1_g13 §B5.5` Pertentangan 2 sudah menunjukkan Lemma C lulus sementara `arm_serial_ub` kosong; satu-satunya bagian yang Lemma C tidak sentuh adalah ekornya. Konsekuensinya: tugas 3 → **(b)** |
| **D41** | Lemma C **BUKAN** syarat perlu: ada ≥ 1 pose yang `arm_serial_ub` terima tetapi `lemma_c` tolak, dan sebabnya **waktu tiba `T0`** | **kode sendiri** | diturunkan dari pembacaan kode (A2.3), jadi ia **dugaan lemah** menurut prior D29: nol bug pernah ditemukan oleh pembacaan ulang, enam sesi berturut-turut. Ditulis justru supaya ia diuji, bukan dipercaya |
| **D42** | Tuas 3 + 4 memberi **0** `BINDING` pada `c_arm = 0.05` | dunia | prior LONGGAR menang lagi; dan `p1_g13 §B10.6` mengukur tuas 3 **tipis** (6+6 dari 864) sehingga "dipaksa per gantry" hampir tidak bisa dipaksa di sel ini |
| **D43** | U4: `depart` solver ada **di luar** grid W2 pada **≥ 5 dari 9** instance | **kode sendiri** | `feasible_starts` mengembalikan waktu mulai kontinu dari perhitungan jarak; bilangan real sembarang tidak pernah jatuh di grid seragam 0.25 s. Kalau ini meleset, celah `solver < W2` kehilangan kandidat terakhirnya |

### A5. Berkas

| Berkas | Isi | Status |
|---|---|---|
| `test/psweep_g14.py` | tugas 2: tangga `\|P\|`, dua lengan, probe biaya, `report` | **BARU** |
| `test/diag_g14.py` | tugas 1 (pelabelan ulang K2/K3), tugas 3 (profil penolakan pose parkir), tugas 5 (grid W2) | **BARU** |
| `test/m3_bind_g13.py` | tugas 4, dijalankan apa adanya | 🔒 **BEKU** |
| `test/eval_sched_armfull.py`, `verify_sched_armfull.py`, seluruh `sched*.py` | — | 🔒 **BEKU** |
| `docs/p1_g14_reach4.md` | dokumen ini | — |

### A6. Rule 6

**Rule 6 (anggaran 30 000 token/sesi) adalah PENGECUALIAN EKSPLISIT untuk sesi
protokol-panjang P1, dinyatakan di muka**, sebagaimana `p1_g12 §A8` dan
`p1_g13 §A6`. Anggaran yang benar-benar mengikat dan dilaporkan: **anggaran
pencarian 120 s per instance** (A2.1), **tutup RUN** tugas 2 dan tugas 4
(A2.2/A2.4), dan **biaya sapuan** yang diukur sebelum tangga (A2.2).

---

## B. Hasil terukur

> §A dikunci 2026-08-16 sebelum satu baris kode sesi ini dijalankan. Semua
> angka di bawah keluar sesudahnya. Setiap tempat di mana §B bertentangan
> dengan §A ditandai 🔺. **§A TIDAK ditulis ulang.**

### B0. Cara menjalankan ulang, dan beban mesin

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 test/diag_g14.py relabel              # tugas 1 (A2.1 K2/K3)
python3 test/psweep_g14.py probe              # tugas 2, biaya SEBELUM tangga
python3 test/psweep_g14.py cell CARD-507 0.15 # satu sel tangga
python3 test/psweep_g14.py report             # tangga + kriteria A2.2
python3 test/diag_g14.py park s1 0.15         # tugas 3
python3 -c "import sys;sys.path[:0]=['.','test'];import m3_bind_g13 as m;m.m3(14,wall_cap=2400.)"
python3 test/diag_g14.py grid                 # tugas 5
```

⚠️ **Beban mesin:** awal sesi `load average` **3.46**, selama pengukuran
**3.9–7.2** pada **16 core** dengan **3 pekerjaan latar** (A2.6). Itu **di bawah
jenuh** — jumlah proses runnable tidak pernah melewati jumlah core — dan itulah
pembenarannya, bukan angka konkurensinya. G13 diukur pada 6–8.5, G12 4.5–5.0,
G11 3.3–3.7, G10 42.

🔒 **Berkas beku (§A0), diperiksa:** `git diff` **KOSONG** pada keenam belas
berkas. Berkas baru: `test/psweep_g14.py`, `test/diag_g14.py`.

### B1. 🟢 TUGAS 1 — anggaran (b) DILAKSANAKAN, dan satu fakta baru jatuh keluar

Keputusan §A2.1 **(b)** dikunci sebelum pengukuran apa pun dan **dilaksanakan**:
`diag_g14.py relabel` membaca kesepuluh artefak G13 apa adanya dan mencetak
setiap sel dengan **dua** angka.

**R0 LULUS:** kolom yang tidak dilabeli ulang mereproduksi `p1_g13 §B5` persis —
40 instance per sel, `wall mean/maks` identik (S1 20.9/31.0/45.5/96.6/108.9 s;
S2 0.1/4.8/30.2/60.2/82.9 s), dan **97 dari 400** melewati anggaran pencarian,
angka yang sama dengan `p1_g13 §B10.1`.

| sel | wall mean | maks | > anggaran pencarian | overshoot maks | TIDAK ADA JADWAL (label BARU) |
|---|---|---|---|---|---|
| S1 0.00 | 20.9 s | 120.1 s | 2 | **0.1 s** | — |
| S1 0.05 | 31.0 s | 120.1 s | 4 | **0.1 s** | — |
| S1 0.10 | 45.5 s | 144.3 s | 9 | 24.3 s | — |
| S1 0.15 | 96.6 s | 437.4 s | 16 | **317.4 s** | `n4_s4_mr1` (pencarian 120 s; **wall 405.2 s**), `n6_s1_mr1` (**304.1 s**), `n6_s2_mr1` (**350.7 s**), `n6_s4_mr1` |
| S1 0.20 | 108.9 s | 380.2 s | 19 | 260.2 s | `n4_s4_mr1` (**378.1 s**), `n4_s5_mr1` (**231.6 s**), `n6_s1_mr1` (**261.2 s**), +3 |
| S2 0.00–0.20 | 0.1–82.9 s | 120.0 s | 0 / 0 / 7 / 15 / 25 | **0.0 s** | 12 instance, **semuanya wall 120.0 s** |

🔴 **Fakta yang label lama menyembunyikan, dan ia bukan kosmetik.** Baris TIDAK
ADA JADWAL di **S1** memakai **304–405 s wall** sebelum menyerah; baris yang
sama di **S2** memakai **tepat 120.0 s**, nol overshoot, di **kelima** sel.
Artinya **dua kategori itu tidak punya sebab yang sama**:

- **S1**: waktunya habis di **konstruktor pra-loop** (`arm_serial_ub` menyapu
  2376 pose, `p1_g13 §B2b`) — ia menyerah setelah membayar penuh.
- **S2**: waktunya habis di **loop pencarian** (anggaran 120 s mengikat persis),
  karena `|P| = 507` membuat sapuan parkir murah dan tidak pernah jadi biaya.

➜ Label `TIDAK ADA JADWAL DITEMUKAN (anggaran 120 s)` yang lama menyamakan
keduanya. Label baru memisahkannya **tanpa satu pengukuran baru pun**, dan itu
persis alasan K2 ada. Ini juga **memprediksi** hasil tugas 3 sebelum tugas 3
dijalankan: pada S1 sebabnya ada di sapuan pose parkir, dan itu yang diprofilkan
di §B3.

🔒 **Tidak satu pun angka `Δ` berubah** — sesuai (b), dan sesuai `p1_g13
§B10.1`: setiap makespan adalah bukti optimalitas atau batas atas yang sah, dan
vonis dua-sisi bergantung pada `exact`/`proved`, yaitu pada anggaran
**pencarian**, yang seragam 120 s di setiap instance di setiap sel.

**Harga (a) yang tidak jadi dibayar, dihitung sebelum memilih:** ≈ **1334
instance-run** / ≈ **2670 panggilan solver** (1180 / 2360 yang masih punya
artefak di `/tmp`, plus G7 dan G8 yang artefaknya sudah hilang dan harus
dibangkitkan ulang), tujuh dokumen, dan M0 harus lulus ulang karena solvernya
disentuh.

### B2. 🔺 PERTENTANGAN 1 — lengan GEOMETRI yang §A kunci hanya bisa dibangun pada 17 dari 40

§A2.2 mengunci **dua** lengan percobaan, dan lengan GEOMETRI-nya adalah "masker
S2 yang persis, `|P| = 507`". Diukur **sebelum** selnya dijalankan, sebagaimana
syarat R3 menuntut:

| masker | `\|P\|` | instance S1 **TIDAK DAPAT DIBANGUN** |
|---|---|---|
| S2 penuh (`\|sin rot\| ≥ 0.525` **DAN** `\|lin − 0.80\| ≤ 0.30`) | **507** | **23 / 40** |
| S2 **hanya rotasi** (`\|sin rot\| ≥ 0.525`, seluruh rel) | **1519** | **0 / 40** |

Sebabnya struktural dan sudah tertulis di `gen_real_rotcrowded`: pita `lin`
bukan pembatas **pose**, ia pembatas **kolam tugas** yang menyamar. S2 menarik
tugasnya dari `x_c ± 0.30`; S1 menariknya dari seluruh peta, jadi 23 dari 40
instance S1 punya tugas yang **tidak terjangkau sama sekali** di dalam pita.

🔺 **Konsekuensi, ditulis bukan dihaluskan:** lengan GEOMETRI seperti yang §A
kunci **tetap dijalankan** (pada 17 instance yang bisa dibangun, 23 sisanya
dilaporkan namanya per R3) — tetapi ia **tidak lagi cukup sendirian**, karena
17 instance dengan tugas yang tersaring bukan lagi "S1 dengan pose lain".
Maka ditambahkan **GEO-ROT** (`|P| = 1519`, hanya separuh **rotasi** dari
pembatasan S2 — yaitu justru separuh yang mekanisme hipotesisnya sebut: dekat
±90° lengan menyapu melintasi pemisahan gantry 0.72 m), yang bisa dibangun pada
**40 dari 40**, plus kembaran kardinalitasnya `CARD-1519`.

Ini **penambahan** pada rancangan §A, bukan penggantian, dan ia lahir dari
pengukuran yang §A wajibkan (R3) — bukan dari hasil yang tidak disukai. Desain
kembar A2.2 tetap utuh dan sekarang punya **dua** pasang ber-`|P|` identik:

```
CARD-507  <-> GEO-S2      |P| = 507    (17 instance)
CARD-1519 <-> GEO-ROT     |P| = 1519   (40 instance)
```

### B3. 🔺 PERTENTANGAN 2 — BIAYA TANGGA, dan tangganya DIPOTONG

A2.2 mengunci probe pada rung **148** dan **1188**, `c_arm = 0.20`, dengan
aturan keputusan ditulis sebelum angkanya dilihat. Terukur:

| instance | rung | wall | makespan | route / terbukti | node | gerbang lengan |
|---|---|---|---|---|---|---|
| `n6_s0_mr1` | **148** | **24.85 s** | 37.3041 | `bnb`, **terbukti** | 17 | 1270 panggilan, 14.6 s = **59 %** |
| `n6_s4_mr0` | **148** | **120.00 s** | 80.6082 | `bnb`, tidak | 10 | 3123 panggilan, 40.1 s = **33 %** |
| `n6_s0_mr1` | **1188** | **120.01 s** | 36.0025 | `bnb`, tidak | 82 | 8587 panggilan, 79.8 s = **67 %** |
| `n6_s4_mr0` | **1188** | **120.07 s** | 79.0166 | `bnb`, tidak | 13 | 4805 panggilan, 84.2 s = **70 %** |

➜ Probe terburuk **120.07 s**, jatuh di pita **60–300 s** ⟹ menurut aturan yang
dikunci di muka, **tangga DIPOTONG jadi 3 rung: {1188, 507, 148}**.
🔺 Rung **594** dan **297** karena itu **TIDAK DIUKUR**, dan disebut namanya di
sini. `CARD-1519` **tetap dijalankan** karena ia bukan rung tangga melainkan
**kontrol ber-`|P|` identik** untuk GEO-ROT (§B2) — desain kembar A2.2 menuntut
itu, dan tanpanya lengan GEOMETRI tidak bisa dipisahkan dari kardinalitas.

🔴 **Dan satu hal yang probe ini ukur dan `p1_g13` tidak bisa:** biaya **TIDAK
monoton dalam `|P|`.** Menyusutkan 2376 → 1188 **menaikkan** node dari 11
(`p1_g13 §B2`) ke 82 pada `n6_s0_mr1` sementara wall tetap mentok; menyusutkan
lagi ke 148 membuat instance yang sama **terbukti dalam 24.85 s**. Dua mekanisme
yang §A2.2 sebut sebagai saling melawan **dua-duanya nyata**, dan titik baliknya
ada di antara 148 dan 1188.

➜ **Dan gerbang lengan 33–70 % wall di keempat probe.** Ini **sesi KETUJUH**
berturut-turut dengan pola yang sama: **yang mahal adalah PEMERIKSA**
(`p1_g8 §B4`, `p1_g9 §B3`, `p1_g10 §B3-1`, `p1_g11 §B4`, `p1_g12 §B4`,
`p1_g13 §B2`).

### B4. 🟢 TUGAS 3 — "TIDAK ADA JADWAL" DIPROFILKAN, dan Lemma C gugur sebagai prapenyaring karena DUA alasan, bukan satu

Keempat instance S1 tanpa jadwal pada `c_arm = 0.15`, **seluruh 2376 pose × 2
arah**, empat tahap penolakan `arm_serial_ub` dihitung satu per satu.
**19 008 penolakan, 0 penerimaan.**

| arah | S1 struct/parkir | S2 lengan/parkir | S3 struct/ekor | S4 lengan/ekor | TERIMA | Lemma C bebas |
|---|---|---|---|---|---|---|
| `n4_s4_mr1` 1→2 | 51 | **1150** | 14 | **1161** | 0 | 2138 / 2376 |
| `n4_s4_mr1` 2→1 | **1731** | 264 | 0 | 381 | 0 | 2178 / 2376 |
| `n6_s1_mr1` 1→2 | **1389** | 188 | 0 | 799 | 0 | 2098, 2288 / 2376 |
| `n6_s1_mr1` 2→1 | **1702** | 434 | 7 | 233 | 0 | 2040, 1878 / 2376 |
| `n6_s2_mr1` 1→2 | **1493** | 285 | 0 | 598 | 0 | 2058, 2110 / 2376 |
| `n6_s2_mr1` 2→1 | **1743** | 227 | 18 | 388 | 0 | 2124, 2050 / 2376 |
| `n6_s4_mr1` 1→2 | 51 | **1178** | 14 | **1133** | 0 | 2096 / 2376 |
| `n6_s4_mr1` 2→1 | **1731** | 264 | 0 | 381 | 0 | 2146 / 2376 |
| **TOTAL** | **9891 (52.0 %)** | **3990 (21.0 %)** | **53 (0.3 %)** | **5074 (26.7 %)** | **0** | — |

| Fase | Penolakan | |
|---|---|---|
| **PARKIR** — yang Lemma C sertifikasi | **13 881** | **73.0 %** |
| **EKOR SERIAL** — yang Lemma C **diam** soalnya | **5127** | **27.0 %** |
| **DITERIMA** | **0** | — |

➜ **Aturan keputusan A2.3 → (b).** Campuran, bukan ≥ 80 % di mana pun, jadi
opsi (a) tidak dibuka. Dan **tidak ada pose yang lolos keempat tahap** — jadi
tidak ada bug di berkas beku; `arm_serial_ub` benar-benar tidak punya apa pun
untuk dikembalikan pada instance ini.

🔴 **Tapi angka yang sebenarnya penting bukan pembagian 73/27 — melainkan
kolom pertama.** **52 % penolakan ada di `sc.schedule_conflict` fase parkir**,
yaitu **predikat STRUKTUR**. Lemma C sama sekali tidak memodelkan itu: ia hanya
mengukur `ARM_BLOCK` antara konfigurasi dwell satu gantry dan pose menggantung
gantry lawan. Jadi Lemma C gagal sebagai prapenyaring karena **dua** alasan yang
saling bebas:

1. ia diam soal **ekor serial** (27 %) — yang `p1_g13 §B5.5` Pertentangan 2
   sudah namai, dan
2. ia diam soal **predikat struktur** (52 %) — yang **belum pernah** disebut,
   dan yang sendirian lebih besar daripada seluruh alasan (1).

**Bersama: 79 % penolakan ada di luar jangkauan Lemma C.** Yang tersisa di
dalam jangkauannya hanya 21 %.

🔒 **Dan itu terlihat langsung dari kolom kanan tabel:** Lemma C melaporkan
**1878–2288 dari 2376** pose "bebas" (79–96 %), sementara tahap S2 — satu-satunya
tahap yang predikatnya sama — menolak 188–1178. Prapenyaring yang meloloskan
~88 % kandidat pada masalah yang menolak **100 %** kandidat tidak menyaring
apa-apa. **Itu bukan argumen tentang kebenaran, itu aritmetika.**

➜ **Vonis: (b) DITERIMA DAN DILAPORKAN SEBAGAI KURUNGAN.** `sched_armfull.py`
**tidak disentuh**, M0 **tidak perlu dijalankan ulang**, dan baris
`Δ_full TIDAK DAPAT DITENTUKAN pada c_arm ≥ 0.15` (`p1_g13 §B10.2`) **berdiri**
— sekarang dengan mekanismenya terukur, bukan disimpulkan.

🔺 **Dan ini MEMPERKUAT §B1.** Label baru sudah memprediksi persis ini: baris
S1 memakai 304–405 s wall karena ia **menyapu 2376 pose dan menolak setiap
satu**, membayar penuh untuk hasil kosong. Sekarang tahu **di mana** ia menolak.

#### B4.1 🔺 PERTENTANGAN 3 — D41 SALAH, dan ia salah karena diturunkan dari PEMBACAAN

§A2.3 menulis, dari pembacaan kode, bahwa Lemma C **bukan** syarat perlu:
`lemma_c()` menuntut pose parkir bebas terhadap **setiap** perhentian bertugas
`lead`, sementara gantry parkir baru tiba pada `T0`, jadi perhentian yang tutup
sebelum `T0` tidak pernah bertemu dengannya. **Diuji, tidak dinalar** — pada 10
instance S1, `c_arm = 0.15`:

| | |
|---|---|
| instance dengan pose parkir **DITERIMA** `arm_serial_ub` | **4** |
| dari keempatnya, pose yang `lemma_c` sebut **TERBLOKIR** | **0** |
| instance yang parkir **di `p0`** (tidak bergerak, `T0 = 0`) | **5** |
| instance dengan `arm_serial_ub → inf` | **1** (`n4_s4_mr1`) |

➜ 🔺 **D41 MELESET.** Pada setiap kasus terukur, `lemma_c` **sepakat**.

🔒 **Dan kelima kasus `p0` bukan lubang, ia pengecualian yang bersih:** kalau
pose parkir yang diterima **adalah** `p0`, maka `T0 = 0` dan **tidak ada**
perhentian `lead` yang bisa tutup sebelum kedatangan — mekanisme yang D41
dalilkan **tidak bisa menyala di sana secara konstruksi**. Jadi sampel yang
benar-benar menguji D41 adalah **4**, dan itu **kecil**; yang boleh ditulis
adalah "terbantah pada sampel yang diukur", bukan "Lemma C perlu".

➜ **Ini contoh ketujuh berturut-turut dari prior D29** (`p1_g13 §B9` bacaan 3):
**setiap** temuan nyata datang dari uji yang dijalankan, dan dugaan yang
diturunkan dari **pembacaan ulang kode** meleset lagi. Kali ini pembacaannya
adalah milik sesi ini sendiri, ditulis di §A, dan itu justru gunanya ia ditulis.

🔒 **Tidak mengubah vonis (b).** D41 adalah palang untuk **membuka** (a);
karena aturan keputusan A2.3 sudah menutup (a) lewat pembagian 52/21/0.3/27
(§B4), palang itu tidak pernah perlu dilewati. `sched_armfull.py` tetap beku,
M0 tidak dijalankan ulang.

### B5. 🟢 TUGAS 5 — U4 LUNAS. Celah `solver < W2` adalah DISKRETISASI W2.

`p1_g10 §B9` membuka celah ini; G10 menyingkirkan penghalusan grid (0/9),
`p1_g13 §B8` menyingkirkan `max_evade` (0/9, dengan 10× node). Satu kandidat
tersisa dan **belum pernah diukur**. Diukur sekarang, dan ia **uji lima menit
yang G10 tidak jalankan**:

Untuk kesembilan instance `solver < W2`, jadwal solver **diputar ulang** dan
setiap keberangkatan diadu dengan `Brute._starts(t[g], tr[h])` **pada keadaan
rekursi yang sama** — yaitu himpunan waktu yang W2 **benar-benar bisa** coba.

| instance | solver | W2 | perhentian | **di luar grid** | contoh (gantry, `depart`, grid terdekat) |
|---|---|---|---|---|---|
| `crowd_n2P3mr0s0` | 13.3018 | 14.5539 | 3 | **1** | (1, **2.663661**, 2.75) |
| `crowd_n2P3mr0s2` | 12.8593 | 13.0164 | 3 | **1** | (1, **2.357007**, 2.25) |
| `crowd_n2P4mr0s0` | 13.4523 | 13.9500 | 3 | **1** | (1, **2.770319**, 2.75) |
| `crowd_n2P4mr0s1` | 13.3258 | 14.2212 | 3 | **1** | (2, **2.283601**, 2.25) |
| `crowd_n2P4mr0s2` | 13.6908 | 14.2421 | 3 | **1** | (1, **2.934511**, 3.0) |
| `crowd_n3P3mr0s0` | 15.3018 | 16.5539 | 3 | **1** | (1, **2.663661**, 2.75) |
| `crowd_n3P3mr0s2` | 14.8593 | 15.0164 | 3 | **1** | (1, **2.357007**, 2.25) |
| `crowd_n2P3mr1s0` | 13.3018 | 14.5539 | 3 | **1** | (1, **2.663661**, 2.75) |
| `crowd_n2P3mr1s2` | 12.8593 | 13.0164 | 3 | **1** | (1, **2.357007**, 2.25) |

> ## ➜ **9 dari 9. A2.5 VONIS: DIKONFIRMASI.**

✅ **D43 TEPAT.** Celah `solver < W2` **bukan** kegagalan solver dan **bukan**
kegagalan oracle — ia **artefak diskretisasi** pada W2. `feasible_starts`
menghitung waktu mulai layak secara **kontinu** dari jarak; `Brute._starts`
menyampel `{t_min, t_min + 0.25, …} ∪ {hi}`. Bilangan seperti **2.663661**
tidak pernah jatuh di grid seragam mana pun **berapa pun halusnya** — dan itu
menjelaskan, secara **retrospektif dan tepat**, kenapa uji G10 menghalfkan grid
dan menutup **0 dari 9**. Uji itu benar kelasnya dan salah instrumennya.

🔒 **Aturan `p1_g9 §A3-K1` TEGAK, dan sekarang TERUKUR, bukan diasumsikan:**
*"`solver > W2` adalah kegagalan solver; `solver < W2` adalah grid yang kasar."*
Sisi kedua sudah dua sesi menjadi asumsi. Ia sekarang fakta pada kesembilan
instance, dengan bilangan penyebabnya dikutip.

➜ **U4 LUNAS.** Tiga kandidat diajukan sejak G10; dua tersingkir oleh
pengukuran (grid halving G10, `max_evade` G13), yang ketiga **dikonfirmasi**.

### B6. 🟡 TUGAS 4 — TUAS 3+4 SEPARUH TERUKUR, dan jawabannya di `c_arm = 0.05` adalah NOL

`m3_bind_g13.py` dijalankan **apa adanya** (berkas beku, §A0). Satu-satunya
parameter yang diubah adalah **urutan daftar kelas** — kelas `forced-*` didahulukan,
karena `p1_g13 §B4` sudah mengukur bahwa tutup wall memotong setelah kelas
pertama, dan pertanyaan §A2.4 ada di kelas `forced`, bukan di baseline.

| kelas | kandidat | dibangun | `fmax` | brute 2 arah | **BIND** | `vacuous` | `rerouted` | **`infeasible`** | selisih enumerator |
|---|---|---|---|---|---|---|---|---|---|
| `forced-n2P3` | 14 | 14 | 1.00 | 23 | **1** (`c=0.15`) | 7 | **0** | **15** | 0 |
| `forced-n2P4` | 14 | 14 | 1.00 | 33 | **1** (`c=0.10`) | 11 | **0** | **21** | 0 |
| **jumlah** | **28** | **28** | | **56** | **2** | **18** | **0** | **36** | **0** |
| `G12-baseline` (reproduksi, jalan pertama) | — | — | 1.00 | — | 2 (`c=0.10`, `c=0.15`, seed 0) | — | — | — | 0 |

> ## ➜ **BINDING pada `c_arm = 0.05`: 0 dari 56.**
> Kedua pengikat yang tuas 3+4 hasilkan ada di **`c_arm ≥ 0.10`**.

✅ **D42 TEPAT.** Menutup jalan keluar "menunggu" — tugas dipaksa per gantry
(tuas 3) **dan** beban seimbang (tuas 4) — **tidak** membuat kendala lengan
mengikat di nilai utama. Prior DUNIA (LONGGAR) menang lagi, dan sekarang ia
bertahan terhadap tuas yang **dirancang khusus untuk mematahkannya**.

🔴 **Dan dua kolom yang lebih menarik daripada kolom BIND.**

1. **`rerouted` = 0 dari 56**, melawan **9 dari 32** pada `G12-baseline`
   (`p1_g13 §B4`). Mekanisme "mengikat dan gratis" — `ARM_BLOCK` membuang
   sebagian optimum sementara yang lain bertahan dengan makespan sama —
   **menghilang** begitu tugas tidak bisa berpindah gantry. Itu justru
   mekanisme yang tuas 3 dirancang untuk tutup, dan **ia memang tertutup**.
2. **`infeasible` = 36 dari 56 (64 %)**, kategori yang **tidak muncul sama
   sekali** di tabel `G12-baseline` G13 (0 dari 32). Menutup jalan keluar
   "menunggu" tidak menaikkan optimum — ia **menghapus optimumnya**. Instance
   itu jadi tak-layak di bawah `ARM_BLOCK`, bukan lebih mahal.

🔒 **Itu bacaan yang bisa dipakai, dan ia lebih tajam daripada "0 mengikat":**
pada sel ini kendala lengan berperilaku **biner**, bukan bertingkat. Selama ada
cara menyingkir ia **gratis**; ketika tidak ada, jadwalnya **tidak ada**. Di
antara keduanya — "ada tapi lebih mahal" — hampir kosong: **2 dari 56**.

⚠️ **YANG TIDAK TERCAPAI, dan A2.3/A2.4 menuntut ini disebut dengan angkanya:**

| Lantai usaha | Dituntut | Tercapai |
|---|---|---|
| kelas berbeda | ≥ 5 | **2** ❌ |
| kandidat dibangkitkan | ≥ 300 | **28** ❌ |
| brute-force dua arah | ≥ 40 | **56** ✅ |

🔺 **PERTENTANGAN 4 — dan ia BENTUKNYA SAMA PERSIS dengan §B1.** §A2.4 mengunci
**tutup RUN 2400 s**. Run itu **dihentikan pada ≈ 3300 s**, yaitu **15 menit di
atas tutupnya**, karena `m3()` memeriksa tutupnya **hanya di antara kandidat** —
dan satu panggilan `classify()` pada kelas `forced-n4P3` (`n = 4`, jadi `2⁴`
subset dalam `enum_opt`) berjalan lebih dari 20 menit sendirian. **Anggaran yang
diperiksa hanya di antara iterasi bukan anggaran**, dan itu **temuan yang sama
persis** dengan `p1_g13 §B2a` tentang anggaran 120 s — ditemukan lagi, di
instrumen lain, di sesi yang sedang menulis tentang temuan itu. Tujuh kelas
karena itu **TIDAK DIUKUR**, dan namanya disebut: `forced-n4P3` (sebagian),
`forced-n4P4`, `forced-n2P3-dw2`, `forced-n4P3-dw2`, `forced-n2P2`,
`G12-baseline` (penuh), `G12-baseline-P4`.

➜ Karena lantai usaha **tidak** terpenuhi, kalimat **"tuas 3+4 tidak bisa
membuat instance mengikat di 0.05"** **TIDAK BOLEH** ditulis. Yang boleh:
**"pada 56 instance yang di-brute-force dua arah lewat dua kelas paksa,
0 mengikat di `c_arm = 0.05`, dan 64 % justru menjadi tak-layak."**

### B7. 🟢 TUGAS 2 — JAWABANNYA NEGATIF, DAN ITU JAWABAN

**12 sel × 40 instance = 480 instance-run. 0 kegagalan gerbang (R1/R2).**

**Kriteria A2.2 dikunci sebelum generatornya ada**, dan dipakai apa adanya:
`Δ_arm` positif dihitung **hanya** kalau sisi struktur **dan** sisi penuh
dua-duanya `exact`/`proved`. Selisih dua batas atas **bukan** harga.

| lengan | `c_arm` | `\|P\|` | dibangun | terbukti | **POSITIF** | pergeseran `lb` | wall mean/maks |
|---|---|---|---|---|---|---|---|
| `CARD-1519` | 0.15 | 1519 | 40 | 32 | **0** | **+0.00 %** | 61.5 / 251.2 s |
| `CARD-1519` | 0.20 | 1519 | 40 | 26 | **0** | **+0.00 %** | 73.2 / 240.7 s |
| `CARD-1188` | 0.15 | 1188 | 40 | 28 | **0** | **+0.00 %** | 60.7 / 269.7 s |
| `CARD-1188` | 0.20 | 1188 | 40 | 24 | **0** | **+0.00 %** | 68.6 / 264.1 s |
| `CARD-507` | 0.15 | 507 | 40 | 34 | **0** | **+0.00 %** | 40.9 / 242.0 s |
| `CARD-507` | 0.20 | 507 | 40 | 31 | **0** | **+0.00 %** | 59.7 / 240.7 s |
| `CARD-148` | 0.15 | 148 | 40 | 36 | **0** | +4.03 % | 23.4 / 123.6 s |
| `CARD-148` | 0.20 | 148 | 40 | 31 | **0** | +4.03 % | 39.7 / 138.2 s |
| **`GEO-S2`** | 0.15 | **507** | 17 | 8 | **0** | +7.10 % | 75.9 / 227.2 s |
| **`GEO-S2`** | 0.20 | **507** | 17 | 6 | **0** | +7.10 % | 94.0 / 223.2 s |
| **`GEO-ROT`** | 0.15 | **1519** | 40 | 18 | **0** | +3.77 % | 91.0 / 241.3 s |
| **`GEO-ROT`** | 0.20 | **1519** | 40 | 11 | **0** | +3.77 % | 109.5 / 242.4 s |
| **JUMLAH** | | | **434** | **285** | **0** | | |

🔒 **Pasangan kembar ber-`\|P\|` IDENTIK — inti rancangan A2.2:**

| pasangan | `\|P\|` | acak (CARD) | masker S2 (GEO) |
|---|---|---|---|
| 0.15 | **1519** | 32 terbukti, **0 positif** | 18 terbukti, **0 positif** |
| 0.20 | **1519** | 26 terbukti, **0 positif** | 11 terbukti, **0 positif** |
| 0.15 | **507** | 34 terbukti, **0 positif** | 8 terbukti, **0 positif** |
| 0.20 | **507** | 31 terbukti, **0 positif** | 6 terbukti, **0 positif** |

Seluruh pergeseran `lb` **di bawah** ambang 10 % A2.2 syarat 4, dan pada rung
1188 dan 507 ia **nol persis** — jadi instance itu benar-benar instance yang
sama, bukan masalah lain yang kebetulan bernama sama.

> ## ➜ **285 bukti `Δ_arm = 0`, NOL contoh tandingan**, pada `\|P\|` yang
> ## disusutkan sampai **16×** (2376 → 148) dan pada himpunan pose S2 sendiri.
>
> Ditambah **147 bukti** G13 pada `\|P\| = 2376` ⟹ **432 bukti total di S1, nol
> contoh tandingan**, melawan **satu** positif terbukti yang hanya pernah muncul
> di S2 (`p1_g13 §B5.4`).

✅ **D38 TEPAT.** Lengan KARDINALITAS memberi 0 positif di setiap rung.
❌ **D39 MELESET.** Lengan GEOMETRI juga memberi 0 — dan D39 ditulis di §A
sebagai dugaan yang **melawan** prior LONGGAR, dengan mekanismenya disebut
(pita ±90° adalah tempat lengan menyapu melintasi pemisahan 0.72 m). Mekanisme
itu **tidak** menghasilkan harga.

🔒 **JAWABAN TUGAS 2, dan A2.2 sudah menyatakan di muka bahwa bentuk ini adalah
jawaban:** pada tugas S1, **bukan kardinalitas ruang pose dan bukan geometri
ruang pose** yang menghasilkan harga koordinasi lengan. Kedua lengan percobaan
nihil, termasuk pasangan ber-`|P|` **identik** (`CARD-507` ↔ `GEO-S2`, kedua
`c_arm`) yang seluruh gunanya adalah memisahkan keduanya.

➜ **Yang tersisa adalah faktor (3), KEPADATAN TUGAS**, dan §A2.2 sudah menamainya
di muka sebagai **tidak terkendali by design**: setiap tugas S2 ada di pita
`x_c ± 0.30` dan **tidak satu pun** bisa dikerjakan di `p0`, jadi kedua gantry
**dipaksa** ke wilayah yang sama. Kedua lengan sesi ini membawa tugas S1, yang
tersebar di seluruh peta. **Itu kandidat yang tersisa, dan ia TIDAK DIUKUR** —
bukan "diperkirakan penyebabnya".

🔺 **Dan §B2 sudah memberi bukti tak-langsung yang menunjuk ke sana:** masker S2
tidak bisa dipasang pada **23 dari 40** instance S1 justru karena tugas S1 tidak
terjangkau di dalam pita. Pembatasan pose S2 dan kepadatan tugas S2 **bukan dua
knob** — yang kedua adalah alasan yang pertama bisa ada. Memisahkannya menuntut
generator baru (tugas S2, pose S1 penuh), dan itu **pekerjaan G15**, bukan
klaim sesi ini.

⚠️ **Kualifikasi yang wajib menyertai kalimat mana pun di atas:**
`GEO-S2` hanya punya **17 dari 40** instance (R3, §B2), dan proporsi terbukti
di sel `GEO-ROT` rendah (**18/40** di 0.15 dan **11/40** di 0.20) karena baris **TIDAK ADA JADWAL
DITEMUKAN** naik ketika ruang pose menyempit. Nol positif di sel dengan
separuh instance terbukti adalah bukti yang **lebih lemah** daripada nol positif
di sel dengan 36 dari 40, dan itu ditulis, bukan dihaluskan.

### B8. Papan skor §7.2

| # | Dugaan (§A4, ditulis di muka) | Hasil |
|---|---|---|
| **D37** | overshoot wall S1 `c_arm ≥ 0.15` didominasi `arm_serial_ub` (≥ 80 % kelebihan detik) | 🟡 **TIDAK DINILAI** — profil §B4 mengukur **di mana pose ditolak**, bukan **berapa detik** tiap tahap. Ia konsisten dengan D37 (sapuan penuh 2376 pose × 4 predikat, 0 diterima) tapi tidak mengukur pembagian detiknya. Ditandai **TIDAK DIUKUR**, bukan dinilai separuh |
| **D38** | lengan KARDINALITAS: **0** positif terbukti di setiap rung sampai `\|P\| = 148` | ✅ **TEPAT** — 0 dari **208** bukti dua-sisi pada 8 sel CARD, `\|P\|` 1519 → 148 |
| **D39** | lengan GEOMETRI (`\|P\| = 507`, masker S2): **≥ 1** positif terbukti | ❌ **MELESET** — 0 dari 14 (GEO-S2) dan 0 dari 29 (GEO-ROT). Ditulis sebagai dugaan yang **melawan** prior LONGGAR, dan prior LONGGAR menang lagi |
| **D40** | ≥ 80 % penolakan pose parkir ada di **EKOR SERIAL** | ❌ **MELESET** — ekor serial hanya **27.0 %**. Arah melesetnya **informatif**: yang dominan adalah **struktur fase parkir, 52.0 %**, yang Lemma C juga tidak modelkan. Kesimpulan (b) **tidak berubah**, tapi alasannya lebih kuat daripada yang D40 dalilkan |
| **D41** | Lemma C **bukan** syarat perlu; sebabnya waktu tiba `T0` | ❌ **MELESET** — 4 dari 4 pose yang diterima juga bebas menurut `lemma_c`. Sampelnya **4**, dan itu disebut |
| **D42** | tuas 3+4 memberi **0** `BINDING` pada `c_arm = 0.05` | ✅ **TEPAT** — 0 dari 56 brute-force dua arah, dua kelas paksa |
| **D43** | `depart` solver di luar grid W2 pada **≥ 5 dari 9** | ✅ **TEPAT, dan maksimal** — **9 dari 9** |

**Tiga tepat, tiga meleset, satu tidak dinilai.** Papan skor `p1_g13 §B9`
berdiri di **25 meleset / 12 tepat**; sesi ini menambah **3/3** →
**28 meleset, 15 tepat**.

Empat bacaan:

1. 🔴 **Prior KODE SENDIRI meleset DUA KALI dalam satu sesi, dan dua-duanya
   adalah dugaan yang diturunkan dari MEMBACA kode** — D40 (dari `p1_g13 §B5.5`)
   dan D41 (dari membaca `arm_serial_ub` dan `lemma_c` berdampingan di §A2.3).
   D43, satu-satunya dugaan kode-sendiri yang **tepat**, adalah satu-satunya yang
   diturunkan dari **sifat aritmetika** (bilangan real tidak jatuh di grid
   seragam), bukan dari menelusuri alur kontrol. Itu pembelahan yang lebih
   berguna daripada "prior kode sendiri 5 dari 11".
2. **Prior DUNIA (LONGGAR) menang untuk kesekian kalinya, dan kali ini terhadap
   percobaan yang dirancang untuk mematahkannya dari DUA arah sekaligus** —
   menyusutkan ruang pose 16× (D38) dan memasang geometri yang justru
   menghasilkan satu-satunya contoh tandingan yang pernah ada (D39). Nol dari
   285. Kendala lengan pada sel ini **gratis atau mustahil**, hampir tidak
   pernah di antaranya — dan §B6 mengukur hal yang sama dari sisi lain
   (`rerouted` 0/56, `infeasible` 36/56).
3. 🔴 **Prior D29 bertahan ke sesi KETUJUH, dan sesi ini mempertajam syaratnya.**
   Dua bug/temuan nyata sesi ini — anggaran M3 jebol (§B6 Pertentangan 4) dan
   `GEO-S2` tak-terbangun 23/40 (§B2) — **dua-duanya** ditemukan oleh uji yang
   dijalankan. Nol oleh pembacaan ulang; dan **dua dugaan** yang berasal dari
   pembacaan ulang (D40, D41) **dua-duanya meleset**. Pembacaan kode sekarang
   punya rekam jejak yang bisa dikutip: **0 temuan, 2 dugaan meleset**.
4. 🔺 **Temuan §B1 muncul lagi di dalam sesi yang menulis tentangnya.** Anggaran
   120 s tidak mengikat wall karena hanya diperiksa di dalam loop; tutup RUN
   2400 s M3 jebol 15 menit karena hanya diperiksa di antara kandidat. **Bentuk
   yang sama, instrumen yang berbeda, ditemukan pada hari yang sama.** Pelajaran
   yang bisa dipakai bukan "perbaiki anggaran itu" melainkan: **setiap anggaran
   di tumpukan ini harus disebut TITIK PEMERIKSAANNYA, bukan hanya nilainya.**

### B9. Batasan setelah sesi ini

Seluruh `p1_g7 §A4`, `p1_g8 §B10`, `p1_g9 §A4`, `p1_g10 §B12`, `p1_g11 §B9`,
`p1_g12 §B10`, `p1_g13 §B10` **masih berlaku** kecuali yang dicabut eksplisit.
**DICABUT:** `p1_g13 §B10.10` (celah `solver < W2` — §B5 menutupnya).
Yang ditambahkan:

1. 🔴 **Anggaran didefinisikan ulang sebagai ANGGARAN PENCARIAN** (§B1, A2.1
   opsi (b)). Wall **tidak dibatasi di mana pun** di tumpukan. Empat perubahan
   naskah K1–K4 **wajib** dan K2/K3 sudah dilaksanakan sebagai kode
   (`diag_g14.py relabel`). Opsi (a) tidak dipilih; harganya ≈ 1334 instance-run
   / ≈ 2670 panggilan solver, dihitung **sebelum** memilih.
2. 🔴 **Setiap anggaran harus menyebut TITIK PEMERIKSAANNYA.** Dua contoh
   terukur pada hari yang sama: 120 s (di dalam loop B&B saja) dan tutup RUN M3
   2400 s (di antara kandidat saja, jebol 15 menit). Belum diaudit: `W2_BUDGET`,
   `EVADE_ATTEMPTS`, `max_states`, `ACTION_BUDGET`.
3. 🔴 **Harga lengan pada tugas S1 BUKAN dari ruang pose.** 285 bukti sesi ini
   + 147 bukti G13 = **432 bukti `Δ_arm = 0` di S1, nol contoh tandingan**,
   melintasi `\|P\|` 2376 → 148 dan melintasi masker S2 sendiri. Kandidat yang
   **tersisa dan TIDAK DIUKUR**: **kepadatan tugas** (setiap tugas S2 di
   `x_c ± 0.30` dan tidak satu pun bisa dikerjakan di `p0`).
4. ⚠️ **`GEO-S2` hanya 17/40 instance** (R3): masker S2 tidak bisa dipasang
   pada tugas S1 karena pita `lin` S2 adalah pembatas **kolam tugas** yang
   menyamar. Pose dan tugas S2 **bukan dua knob bebas**.
5. ⚠️ **Proporsi terbukti turun ketika ruang pose menyempit atau berpindah:**
   `GEO-ROT` 11–18 dari 40, `GEO-S2` 6–8 dari 17. Nol positif di sel dengan
   sepertiga instance terbukti adalah bukti **lebih lemah** daripada nol positif
   di sel 36/40. Rung `CARD-594` dan `CARD-297` **TIDAK DIUKUR** (§B3).
6. 🔴 **Lemma C tidak bisa jadi prapenyaring `arm_serial_ub`**, dan sekarang
   dengan **dua** alasan terukur, bukan satu: ekor serial (27 %) **dan**
   predikat struktur fase parkir (52 %) — total **79 %** di luar jangkauannya.
   `sched_armfull.py` **tidak disentuh**; M0 tidak dijalankan ulang.
7. **Baris TIDAK ADA JADWAL DITEMUKAN diterima sebagai KURUNGAN** (A2.3 opsi
   (b)). 45 dari 480 instance-run sesi ini, dengan label dua-angka.
8. **Lantai usaha M3 TIDAK terpenuhi** (2 kelas dari ≥ 5, 28 kandidat dari
   ≥ 300; brute-force 56 dari ≥ 40 **terpenuhi**). Tujuh kelas TIDAK DIUKUR,
   namanya di §B6. Kalimat "tuas 3+4 tidak bisa membuat instance mengikat"
   **dilarang**.
9. 🟢 **`rerouted` runtuh ke 0/56 di bawah tuas 3+4** sementara `infeasible`
   naik ke 36/56. Pada sel ini kendala lengan **biner**: gratis, atau tidak ada
   jadwal. "Ada tapi lebih mahal" = **2 dari 56**.
10. Proksi polyline tetap **meremehkan** volume sapuan; `T_fold = 0`; kanonik
    `manip`; exact tetap relatif terhadap grid 33 × 72; `safe_poses` tetap tidak
    diganti; `ArmView` tetap tidak meng-cache konfigurasi MENGGANTUNG
    (`p1_g13 §B10.9`).
11. 🔴 **Rule 6 JEBOL**, dan §A6 menyatakannya di muka.

---

## C. Prompt sesi berikutnya — G15

> **Rekomendasi: Opus 5, effort SEDANG.** Turun dari TINGGI, dan alasannya
> spesifik: G14 menutup tiga dari lima pertanyaan **secara definitif** (U4
> lunas 9/9; anggaran diputuskan dan dilaksanakan; Lemma C gugur dengan angka),
> dan yang tersisa adalah **satu hipotesis tunggal dengan satu generator untuk
> membangunnya**. Itu pekerjaan yang punya sinyal error: kalau generatornya
> salah, instance-nya tak-terbangun atau `lb`-nya melonjak, dan dua-duanya
> kelihatan. Yang **tidak** punya sinyal error hanya satu — memutuskan apa yang
> membuat kepadatan tugas "terkendali" — dan itu sudah punya cetakan yang
> terbukti: desain kembar A2.2 sesi ini.

```
Sesi G15 -- REACH-5: KEPADATAN TUGAS, kandidat terakhir yang berdiri.
G14 menghabiskan ruang pose sebagai penjelasan. 432 bukti Delta_arm = 0 di S1
(285 sesi ini + 147 G13), nol contoh tandingan, melintasi |P| 2376 -> 148 DAN
melintasi masker pose S2 sendiri. Satu-satunya harga positif yang pernah
terbukti tetap satu: n6_s0_mr1 di S2, +5.40%.

BACA DULU:
1. docs/p1_g14_reach4.md -- SELURUHNYA. Khususnya:
   B1 (anggaran = anggaran PENCARIAN; wall TIDAK dibatasi; K1-K4),
   B2 (masker S2 tak-terbangun 23/40 -- pose dan tugas S2 BUKAN dua knob),
   B4 + B4.1 (79% penolakan pose parkir di luar jangkauan Lemma C; D41 meleset),
   B5 (U4 LUNAS 9/9 -- JANGAN dibuka lagi),
   B6 (tuas 3+4: 0 mengikat di 0.05, rerouted 0/56, infeasible 36/56),
   B7 (kedua lengan NIHIL; desain kembar ber-|P| identik -- PAKAI CETAKAN INI),
   B8 bacaan 1 dan 3 (pembacaan kode: 0 temuan, 2 dugaan meleset),
   B9 (batasan -- terutama 2, 3, 4, 5, 8)
2. docs/p1_g13_sweep.md B5.4 (contoh tandingan itu sendiri), B5.5
3. reachability_gng/sched_arm.py gen_real_rotcrowded (KENAPA S2 begitu),
   test/psweep_g14.py (shrink/build/report -- pakai ulang, jangan tulis ulang)

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.
CATATAN MESIN: G14 diukur pada load 4.4-7.2 dari 16 core, 3 pekerjaan latar.
G13 6-8.5, G12 4.5-5.0, G11 3.3-3.7, G10 42.

=== YANG SUDAH TEGAK, JANGAN BANGUN ULANG ===
- U4 LUNAS. 9/9 keberangkatan solver di luar grid W2. TUTUP.
- Anggaran = anggaran PENCARIAN 120 s. Wall dilaporkan terpisah. Opsi (a)
  DITOLAK dengan harganya dihitung (1334 instance-run). JANGAN dibuka lagi.
- Lemma C BUKAN prapenyaring: 79% penolakan di luar jangkauannya. Opsi (a)
  tugas 3 DITUTUP. sched_armfull.py TETAP BEKU, M0 tidak perlu diulang.
- Tangga |P|: 12 sel, 480 instance-run, 0 kegagalan gerbang. Angkanya di
  /tmp/g14_psweep_{arm}_{c}.json dan di B7. PAKAI, jangan hitung ulang.
- Sapuan G13 (10 sel) dan angka utamanya tetap sah -- hanya LABELNYA berubah.

=== TUGAS, BERURUTAN. JANGAN LOMPAT. ===
1. GENERATOR KEPADATAN TUGAS. Bangun gen_real_dense: tugas S2 (kolam
   x_c +- 0.30, DITOLAK kalau bisa dikerjakan di p0) dengan ruang pose S1
   PENUH (|P| = 2376). Itu memisahkan faktor (3) dari (1) dan (2), dan ia
   satu-satunya kombinasi yang G14 tidak bisa bangun.
   PALANG: R3 gaya A2.2 -- p0 dipertahankan, semua tugas terjangkau,
   pergeseran lb dilaporkan, instance tak-terbangun disebut namanya.
   Kalau tak-terbangun > 50%, itu TEMUAN, bukan kegagalan: ia berarti
   kepadatan tugas dan ruang pose tidak bisa dipisahkan di sel ini sama
   sekali, dan naskahnya harus mengatakan itu.
2. UKUR Delta_arm di sel itu, c_arm {0.15, 0.20}, anggaran pencarian 120 s.
   BIAYA DIUKUR DULU (sesi kedelapan dengan jebakan yang sama).
   Kriteria "menjawab" DITULIS SEBELUM generatornya ada, pakai cetakan A2.2:
   positif = Delta_arm > 0 dengan KEDUA sisi terbukti; dan instance yang sama
   harus Delta_arm = 0 terbukti di S1 penuh.
3. TANGGA PEMISAH kalau tugas 2 positif: kepadatan adalah kontinum, bukan
   biner. Lebarkan pita kolam tugas {0.30, 0.60, 1.00, seluruh peta} dan cari
   di mana harganya hilang. Itu KALIMAT NASKAH: harga koordinasi ditentukan
   oleh seberapa dipaksa kedua gantry berbagi wilayah.
   Kalau tugas 2 NIHIL: maka tidak ada satu pun faktor S2 yang mereproduksi
   harganya secara terpisah, dan yang tersisa adalah INTERAKSI. Katakan itu,
   dan jangan mengarang faktor keempat.
4. AUDIT TITIK PEMERIKSAAN ANGGARAN (B9.2). W2_BUDGET, EVADE_ATTEMPTS,
   max_states, ACTION_BUDGET: untuk masing-masing, di mana ia diperiksa dan
   berapa besar satu iterasi bisa melampauinya. Dua sudah terbukti salah pada
   hari yang sama. Ini murah dan ia mencegah angka yang salahnya tak terlihat.
5. LANTAI USAHA M3 (B9.8) kalau ada waktu: 7 kelas TIDAK DIUKUR. Jalankan
   dengan tutup yang diperiksa DI DALAM classify(), bukan di antara kandidat.

=== KUNCI KRITERIA SEBELUM KODE, ke docs/p1_g15_*.md A ===
1. Apa yang membuat kepadatan tugas "terkendali", dan berapa tak-terbangun
   yang membuat percobaannya batal. DITULIS SEBELUM generatornya ada.
2. Apa yang membedakan "harga muncul" dari "instance lain" -- pakai definisi
   A2.2 G14 apa adanya, jangan tulis ulang, jangan longgarkan.
3. Kalau tugas 2 nihil: apa PERSIS yang boleh dan tidak boleh ditulis tentang
   contoh tandingan S2. Ditulis SEBELUM hasilnya dilihat.
4. Apa yang dilaporkan kalau lagi-lagi tidak semuanya bisa dibuktikan.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- TUJUH SESI: yang lambat adalah PEMERIKSA. G14 mengukur gerbang lengan
  33-70% wall di keempat probe.
- Anggaran harus menyebut TITIK PEMERIKSAANNYA, bukan hanya nilainya. Dua
  jebol pada hari yang sama (120 s di dalam loop; tutup RUN M3 di antara
  kandidat, lewat 15 menit).
- Biaya TIDAK monoton dalam |P|. 2376 -> 1188 MENAIKKAN node 11 -> 82;
  1188 -> 148 membuat instance yang sama terbukti dalam 24.85 s.
- Dua proses menulis satu JSON = saling menimpa. Satu berkas per sel, dan
  worker paralel harus punya himpunan sel yang DISJOIN.
- pkill -f <pola> juga membunuh shell yang memuat pola itu di baris
  perintahnya. Pakai PID.
- Pembacaan ulang kode: 0 temuan, 2 dugaan meleset (D40, D41). Dugaan dari
  sifat ARITMETIKA (D43) tepat. Bedakan keduanya.
- Instrumen kedua dari BENTUK yang sama mewarisi cacat yang sama (G13 B3).
- Gerbang yang hanya pernah jalan di S1 belum diuji (G13 B6).
- Tepi kanan TERTUTUP adalah kelas bug berulang (G12 B3.1, G13 B6).
- `tail` pada proses latar menelan keluaran; tulis per baris dengan flush.
- np.bool_ tidak JSON-serializable.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 28 meleset, 15 tepat.
  Prior DUNIA: LONGGAR -- dan G14 menaikkannya lagi: 432 bukti Delta_arm = 0
  di S1 melawan satu positif di S2. Kendala lengan pada sel ini GRATIS atau
  MUSTAHIL; "ada tapi lebih mahal" = 2 dari 56 (B6).
  Prior KODE SENDIRI: pisahkan dua jenis. Dugaan dari MEMBACA alur kontrol
  meleset (0 dari 2 di G14). Dugaan dari sifat ARITMETIKA tepat.
  Prior PALING BERGUNA: setiap temuan nyata datang dari UJI YANG DIJALANKAN,
  nol dari pembacaan ulang. TUJUH sesi berturut-turut.
- DUGAAN YANG DINILAI MEMAKAI SOLVER YANG BELUM LULUS GERBANGNYA TIDAK
  DINILAI. Tandai TERTUNDA. Dugaan yang instrumennya tidak mengukur besaran
  yang didalilkan juga TIDAK DINILAI (D37 G14).
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 lima, G11 empat, G12 dua, G13 dua, G14 EMPAT.
- Rule 6 (30k token) adalah PENGECUALIAN EKSPLISIT untuk sesi protokol-panjang
  P1, dinyatakan di muka (A6), bukan dilaporkan sesudahnya.
- Akhiri dengan prompt sesi berikutnya (G16).
```
