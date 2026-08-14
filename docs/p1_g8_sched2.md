# P1 / G8 — SCHED-2: heuristik, baseline, dan optimality gap

> Sesi G8, 2026-08-14. Melanjutkan [p1_g7_sched.md](p1_g7_sched.md).
> Langkah **3 dan 4** dari `p1_state §7.1`.
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode heuristik dijalankan.**
> §B diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**, bukan dihaluskan. §A tidak ditulis
> ulang belakangan — kesalahannya dibiarkan bisa dibaca (disiplin `p1_g7`).
>
> Sesi ini **sepenuhnya offline**. Tidak ada kamera, gantry, lengan,
> `move_group`. Lengan 4× masih dilepas.
>
> **Rekomendasi model: Opus 5, effort TINGGI, untuk seluruh sesi.** Bukan karena
> tugasnya besar, tapi karena **tiga dari empat fasenya gagal secara senyap**:
> heuristik + baseline diperiksa keras oleh gap-vs-exact dan gerbang K4, tapi
> kerangka evaluasi (rumus gap, identitas instance antar penjadwal), batas bawah
> K3 (batas yang tidak sah tetap mencetak rasio yang rapi — buktinya matematis,
> bukan komputasional), dan penulisan §B (persis jebakan `p1_g7 §B3`: angka
> benar, mekanisme salah, tidak ada yang menegur) semuanya tidak punya sinyal
> error. Jangan turun ke model kecil untuk bagian "tinggal menjalankan" — bagian
> menjalankan di sini adalah bagian menafsirkan.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| Model penjadwalan (sumber daya, timeline, SR/MR, mutex, `dur = 2.0 × max(a,b,z)`) | `p1_g7 §A1` | dibaca apa adanya, nol perubahan |
| `T_traverse = max(T_lin, T_rot)`, offset per sumbu yang bergerak | `p1_state §5.6` + `p1_g7 §A0` | dipakai, tidak diturunkan ulang |
| Dwell 2.0 s kontinu | `p1_state §5.8 / §8b` | durasi tugas |
| Definisi makespan (K2) | `p1_g7 §A2-K2` | tidak ditawar |
| Handover = SATU tugas dua-lengan (K3) | `p1_g7 §A2-K3` | tidak ditawar |
| Peta `cap_g{1,2}_rail160.npz` (33×72, \|P\| = 2376) | `p1_state §3` | satu-satunya oracle kelayakan |
| `sched.solve_exact` | `p1_g7 §B1` (V0–V4) | **GROUND TRUTH**. Nol perubahan. |
| `sched.gen_real`, `gen_random_small` | `p1_g7 §A3` | generator, seed sama dengan G7 |
| `gng.py`, `capability.py`, `irm_sweep.py` | `p1_g5 §A0` | dibaca saja |

**Audit ulang G7 sebelum sesi ini dimulai** (2026-08-14, prasyarat: heuristik
tidak boleh dibandingkan terhadap ground truth yang belum diperiksa sendiri):
V0–V4 dijalankan ulang → **5/5 PASS**, `EXACTNESS ESTABLISHED`. Mutex terbukti
hidup di jalur data (`zone ⊆ reach`, non-trivial) **dan** di solver (tabel `dur`
berbeda di 4712/34608 pasangan (subset, pose), maks +4.0 s). `mutex cost =
0.000 s` direproduksi 10/10 seed. Satu kesalahan prosa ditemukan di `p1_g7 §B3`
dan **dicatat sebagai koreksi bertanggal di tempatnya**, bukan di sini
(mekanisme "butuh ≥3 tugas zona" salah; 2 sudah cukup, P4 sendiri buktinya).
Angka-angka G7 tidak berubah.

### A1. Yang dibangun sesi ini, dan yang TIDAK

Dibangun: **empat penjadwal non-exact** + kerangka evaluasi + laporan gap.
`p1_state §6` menuntut empat baseline; yang keempat (`MIP/DP-optimal`) **sudah
ada** = `solve_exact`, jadi yang ditulis sesi ini tiga baseline + satu heuristik
usulan.

**TIDAK dibangun sesi ini, dan tidak boleh menyelinap masuk:** tabrakan
struktur gantry–gantry (`p1_g7 §A4.1`), `T_fold` ≠ 0, kapabilitas dinamis,
perubahan apa pun pada model atau pada solver exact. Kalau salah satunya harus
berubah, ia ditulis sebagai pertentangan §A/§B yang eksplisit.

### A2. Rancangan — diturunkan dari `p1_g7 §B5`, bukan dari selera

Fakta yang mengarahkan rancangan, semuanya sudah **terukur** di G7:

- **74–92% makespan adalah gerak gantry** (§B5). Jadi objektif sebenarnya adalah
  **tur pose gantry yang bergantung urutan dengan kendala cakupan** — bukan
  penugasan lengan.
- **MR = kendala POSE, bukan kendala lengan** (§B4). Hanya 46.9% node punya pose
  handover sama sekali; p10 = 26 pose dari 2376. Pose handover dipilih **lebih
  dulu**.
- **Mutex tidak menggerakkan makespan** (§B3). Heuristik tidak boleh
  menghabiskan usaha di sana; mutex tetap **dihormati** (lewat `stop_duration`)
  tapi tidak dioptimalkan secara khusus.
- **Kedua gantry hanya berkopling lewat alokasi** (§B7.2). Heuristik **tidak
  boleh** mencoba menyinkronkan waktu antar gantry — itu memodelkan sesuatu yang
  tidak ada di model.

**H — heuristik usulan: `pose-tour`.** Empat tahap, dikunci sekarang:

1. **Pose handover dulu.** Untuk tiap tugas MR, kandidat pose = `hand[i]`.
   Dipilih sebelum apa pun yang lain, karena himpunannya paling langka (§B4).
2. **Cakupan (set cover) berbobot traverse.** Pilih himpunan pose `S` yang
   menutup semua tugas, greedy dengan skor
   `(tugas baru yang tertutup) / (tambahan traverse terhadap tur sekarang)`.
   Bukan cakupan murni: §B5 bilang yang mahal adalah jarak, bukan jumlah stop.
3. **Tur.** Urutkan `S` dengan nearest-neighbour dari `p0`, lalu perbaiki dengan
   **2-opt + or-opt** sampai tidak ada perbaikan. Ini bagian yang §B5 ramalkan
   menentukan menang-kalah.
4. **Perbaikan lokal atas penugasan tugas → perhentian**, dengan durasi
   perhentian dihitung **exact** lewat `sched.stop_duration` (yang sudah lolos
   V1): pindahkan tugas antar perhentian, buang perhentian kosong, dan coba
   ganti tiap pose dengan tetangga grid yang lebih murah. Ulang sampai stabil.

**Alokasi tugas → gantry** (hanya untuk instance 2-gantry): karena kopling hanya
lewat alokasi, dipakai penyeimbangan sederhana — tugas ke gantry yang menaikkan
makespan-nya paling sedikit, lalu perbaikan pindah-tugas satu langkah. Tidak ada
sinkronisasi waktu.

⚠️ **Batasan implementasi yang dikunci sekarang karena ia membentuk kode:**
heuristik **DILARANG** memakai `sched._dur_table` atau apa pun yang berukuran
`2ⁿ`. Itu satu-satunya alasan `solve_exact` berhenti di `n = 10`, dan seluruh
guna sesi ini adalah berjalan jauh di atasnya.

**Tiga baseline pembanding** (`p1_state §6`), sengaja dibuat *lemah dengan cara
yang berbeda-beda* supaya bisa menunjukkan **komponen mana** yang penting:

| Baseline | Aturan | Kelemahan yang disengaja |
|---|---|---|
| `fixed` | tugas dibagi rata ke gantry/lengan di muka; tiap tugas dikerjakan di pose terbaiknya sendiri; urutan = urutan indeks tugas | tidak ada tur, tidak ada batching |
| `greedy` | dari pose sekarang, pindah ke pose **terdekat** yang melayani ≥1 tugas tersisa; kerjakan sebanyak mungkin di sana | tur miopik, cakupan tidak direncanakan |
| `sequential` | satu tugas per perhentian, tanpa batching, tanpa konkurensi dua lengan; urutan nearest-neighbour | membayar traverse per tugas |

### A3. Kriteria yang DIKUNCI

#### K1 — AMBANG GAP, sebagai angka

```
gap(instance) = (makespan_heuristik - makespan_exact) / makespan_exact * 100 %

CUKUP BAIK  :=  mean gap <= 5.0 %   DAN   max gap <= 10.0 %
                atas SELURUH himpunan uji K2, tanpa membuang instance apa pun.
```

Alasannya, tiga-tiganya angka, bukan selera:

1. **Efek terkecil yang harus tetap terbaca oleh naskah adalah biaya MR**, yang
   terukur `+7.02 s` pada makespan rata-rata `46.9 s` (`ablate`, `n = 6`, mr = 1,
   1 gantry, 10 seed) = **15.0%**. Heuristik dengan mean gap 5% menyisakan margin
   **3×** terhadap efek itu. Di atas ~15% heuristiknya akan menelan efek yang
   justru ingin dilaporkan.
2. **5% dari makespan tipikal 35–47 s adalah 1.75–2.35 s ≈ satu slot dwell
   (2.0 s)**, yaitu kuantum waktu terkecil yang dimiliki model untuk *kerja*.
   Gap di bawah satu slot **tidak mungkin** berasal dari pengepakan tugas yang
   lebih baik — hanya dari tur. Ambangnya jatuh persis di batas itu, bukan di
   angka bulat yang enak.
3. **max 10% = dua slot.** Di atas itu turnya salah secara struktural, bukan
   sekadar kalah tie-break, dan itu harus terbaca sebagai kegagalan, bukan
   sebagai "sedikit lebih buruk".

🔴 Kalau ambang ini tidak tercapai, §B melaporkan **TIDAK TERCAPAI sebagai
angka** dan menyebut penyebabnya. **Dilarang** menaikkan ambang setelah melihat
hasil, dan **dilarang** membuang instance yang buruk dari rata-rata.

#### K2 — HIMPUNAN INSTANCE UJI, ditetapkan sebelum melihat hasil apa pun

Seluruhnya mode `real` (`gen_real`), seed **sama dengan G7** supaya bisa
dibandingkan langsung.

**Bagian I — bisa dibandingkan ke exact** (`n ≤ 10`, `p1_g7 §B2`):

| n | seed | gantry | n_mr | jumlah |
|---|---|---|---|---|
| 4 | 0–9 | (1,) dan (1,2) | 0 dan 1 | 40 |
| 6 | 0–9 | (1,) dan (1,2) | 0 dan 1 | 40 |
| 8 | 0–9 | (1,) dan (1,2) | 0 dan 1 | 40 |
| 10 | 0–4 | (1,) dan (1,2) | 0 dan 1 | 20 |
| | | | **total** | **140** |

`n = 10` dibatasi 5 seed karena satu solve exact di sana ≈ 84 s (§B2); 20
instance ≈ 42 menit dinding, dan itu batas yang diterima sadar. Anggaran total
exact untuk Bagian I ≈ **33 menit**, dihitung dari tabel §B2 — bukan ditebak.

**Bagian II — di atas jangkauan exact:**

| n | seed | gantry | n_mr |
|---|---|---|---|
| 12, 16, 20, 30, 50 | 0–4 | (1,2) | 0 dan 1 |

= 50 instance. Di sini **tidak ada exact**, dan K3 mengatur apa yang boleh
ditulis.

#### K3 — APA YANG DILAPORKAN DI ATAS `n = 10`

**Dilarang keras menyebut selisih terhadap heuristik lain sebagai "gap".** Yang
dilaporkan adalah **kurungan (bracket)**, dengan kedua sisinya sah:

```
UB(instance) = makespan heuristik TERBAIK yang lolos validate_schedule()
LB(instance) = max( LB_subset , LB_analitik )
dilaporkan   = UB, LB, dan rasio UB/LB  -- disebut "rasio kurungan", BUKAN gap
```

**`LB_subset` — batas bawah yang exact, dan ini yang utama.** Optimum dari
**sembarang himpunan bagian tugas adalah batas bawah yang sah** untuk instance
penuh. Buktinya ditulis sekarang supaya bisa dibantah sebelum dipakai:

> Ambil jadwal optimum untuk `A ∪ B`. Hapus tugas di luar `A` dari setiap
> perhentian. Perhentian yang jadi kosong dibuang dan turnya dipotong — total
> traverse **tidak naik** karena `T` memenuhi ketaksamaan segitiga
> (`p1_g7 §A0`). Durasi tiap perhentian `2.0 × max(a, b, z)` **tidak naik**
> karena `a`, `b`, `z` semuanya monoton terhadap himpunan tugas. Jadi waktu
> selesai tiap gantry tidak naik, jadi maksimumnya tidak naik. Maka
> `OPT(A) ≤ OPT(A ∪ B)`. ∎

Dipakai dengan `|A| = 8` (≈17 s per solve, §B2), **3 himpunan bagian acak
ber-seed per instance**, diambil maksimumnya. Biaya ≈ 52 s per instance × 50
instance ≈ 43 menit, dijalankan latar belakang.

**`LB_analitik` — murah, dan jadi lantai kalau `LB_subset` longgar:**

```
LB_reach = max_i [ min_g min_{p layak untuk i di g} T_g(p0_g, p) ] + dwell
LB_work  = dwell * ceil( ( n_MR + ceil(n_SR / 2) ) / G )
LB_analitik = max(LB_reach, LB_work)
```

`LB_reach` sah karena tugas `i` harus dikerjakan di suatu gantry pada pose yang
layak untuknya, dan mencapainya dari `p0` berbiaya ≥ traverse minimum, lalu
masih harus di-dwell. `LB_work` sah karena tiap MR menuntut slot sendiri, tiap
gantry butuh `ceil(s_g/2)` slot untuk SR-nya, dan `Σ_g ceil(s_g/2) ≥
ceil(n_SR/2)`, sehingga gantry terburuk ≥ rata-ratanya. Keduanya digabung dengan
`max`, **bukan** dijumlahkan — penjumlahan tidak sah karena tugas terjauh belum
tentu ada di gantry yang bebannya terberat.

#### K4 — GERBANG KEABSAHAN: heuristik yang melanggar model tidak dilaporkan

**Setiap** jadwal dari **setiap** penjadwal (heuristik dan ketiga baseline, di
Bagian I **dan** Bagian II) wajib lolos `validate_schedule()` dari
`test/verify_sched_exact.py` — fungsi yang sama, tanpa modifikasi, yang sudah
dipakai V4 untuk memeriksa solver exact.

`validate_schedule()` memutar ulang jadwal dari aturan model dengan oracle
independen (`ref_traverse`, `ref_stop_slots`), jadi ia mengecek: tiap tugas
dikerjakan **tepat sekali**, tiap penugasan lengan sah di pose itu, tiap MR
benar-benar handover, panjang perhentian benar termasuk **mutex**, waktu mulai
konsisten, dan makespan = maks atas gantry.

🔴 **Jadwal yang gagal → angkanya TIDAK MASUK laporan sama sekali.** Bukan
dilaporkan dengan catatan kaki. Angka dari jadwal yang tidak bisa dieksekusi
adalah angka palsu, dan itu persis cara heuristik "menang" secara tidak sengaja.
Jumlah kegagalan dilaporkan sebagai angka.

#### K5 — YANG DILAPORKAN, TERLEPAS DARI HASILNYA

1. Tabel gap per konfigurasi `(n, gantry, n_mr)`: mean, median, p90, max.
2. Empat penjadwal diadu pada instance **yang sama persis** (seed sama).
3. Waktu dinding tiap penjadwal — heuristik yang bagus tapi selambat exact tidak
   ada gunanya.
4. Jumlah instance yang gagal `validate_schedule()`, per penjadwal.
5. Bagian II: UB, LB, rasio kurungan — **bukan** gap.
6. Bagian dari makespan yang berupa traverse, untuk memeriksa apakah §B5 tetap
   berlaku di `n = 50`.

### A4. Yang TIDAK dimodelkan — masih sama, disebut ulang karena §B akan menggodanya

Seluruh `p1_g7 §A4` dan `§B7` **berlaku tanpa perubahan**. Yang paling mudah
dilupakan saat menulis angka gap:

1. 🔴 **Tabrakan struktur gantry–gantry belum dimodelkan.** Semua angka rugi
   tetap **BATAS BAWAH**. Heuristik yang "menang" di sini belum tentu menang
   pada model yang memodelkannya.
2. `T_fold = 0.0`, tidak pernah diukur. Ia menambah konstanta ke **setiap**
   traverse, jadi ia **memperbesar** pangsa setup — artinya ia **menguntungkan**
   heuristik yang bagus turnya. Arah biasnya diketahui, besarnya tidak.
3. Gerak lengan di dalam satu pose = 0.
4. Exact hanya exact **terhadap grid 33 × 72**. Gap yang dilaporkan adalah gap
   terhadap optimum **pada grid itu**.

### A5. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor sekarang **11 meleset, 1 tepat**, dan **10 dari 11 ke arah yang sama**:
menduga kendala lebih mengikat daripada kenyataannya. Prior kerja yang benar:
**kendala yang belum diukur itu LONGGAR.** Dugaan sesi ini ditulis sekarang
supaya bisa dinilai:

| # | Dugaan | Arah kalau pola berulang |
|---|---|---|
| **D4** | `pose-tour` mencapai ambang K1 (mean ≤ 5%, max ≤ 10%) | pola meramalkan **lebih mudah** dari dugaan — mungkin jauh di bawah 5% |
| **D5** | `sequential` adalah yang terburuk telak, > 50% di atas optimum | pola meramalkan **kurang buruk** dari dugaan |
| **D6** | `greedy` mengalahkan `fixed` | tidak ada arah prior yang jelas; ini murni diukur |
| **D7** | Tur (tahap 3) menyumbang lebih banyak daripada perbaikan lokal (tahap 4) — konsekuensi langsung §B5 | pola meramalkan §B5 **terlalu percaya diri**; tahap 4 mungkin penting |
| **D8** | Pada `n = 50`, 2 gantry, pangsa traverse tetap > 70% (§B5 tetap berlaku) | pola meramalkan pangsanya **turun** lebih cepat dari dugaan |

Kelimanya **diukur**, tidak diperdebatkan. Hasilnya masuk §B apa adanya,
termasuk kalau semuanya meleset lagi ke arah yang sama.

### A6. Berkas

| Berkas | Isi |
|---|---|
| `reachability_gng/sched_heur.py` | heuristik `pose-tour` + tiga baseline + batas bawah + CLI |
| `test/eval_sched_heur.py` | kerangka evaluasi: Bagian I (gap) dan Bagian II (kurungan), gerbang K4 |
| `docs/p1_g8_sched2.md` | dokumen ini |

`sched.py` dan `test/verify_sched_exact.py` **dibaca dan diimpor saja, nol
perubahan** — mengubah ground truth di sesi yang mengukur terhadap ground truth
adalah cara paling rapi untuk menipu diri sendiri.

---

## B. Hasil terukur

> §A dikunci sebelum `sched_heur.py` ada. Semua angka di bawah keluar sesudahnya.
> **Empat tempat di mana §B bertentangan dengan §A ditandai 🔺 dan ditulis
> eksplisit. §A TIDAK ditulis ulang** — ia tetap seperti saat dikunci, termasuk
> kalimat-kalimatnya yang ternyata salah (disiplin `p1_g7`).

### B0. Cara menjalankan ulang

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 -m reachability_gng.sched_heur check              # H0, bentuk tertutup
python3 test/eval_sched_heur.py part1                     # Bagian I, 27 menit
python3 test/eval_sched_heur.py part2                     # Bagian II, menit
python3 test/eval_sched_heur.py part2lb                   # LB_subset, 46 menit
python3 test/eval_sched_heur.py holdout                   # seed 10-14, tahan
python3 test/eval_sched_heur.py ablate                    # D7
python3 test/eval_sched_heur.py report
```

Peta `cap_g{1,2}_rail160.npz` dibaca apa adanya. `sched.py` dan
`verify_sched_exact.py` **tidak disentuh** — `git diff` keduanya kosong.

### B1. Gerbang K4 lebih dulu: 1 250 jadwal, **0 pelanggaran**

Ditulis pertama karena semua angka lain tidak berarti apa-apa kalau jadwalnya
tidak bisa dieksekusi.

| Himpunan | Jadwal diperiksa | Gagal `validate_schedule()` |
|---|---|---|
| Bagian I (140 instance × 5 penjadwal) | 700 | **0** |
| Bagian II (50 × 5) | 250 | **0** |
| Held-out (60 × 5) | 300 | **0** |
| Solver exact sendiri (V4 ulang, 140) | 140 | **0** |
| | **1 250** | **0** |

Tidak ada satu pun angka di §B yang datang dari jadwal yang gagal diputar ulang.

⚠️ **Dicatat karena ini melemahkan bukti, bukan menguatkannya: gerbang ini tidak
pernah sekali pun menolak jadwal di sesi ini.** Ia lolos pada percobaan pertama
untuk kelima penjadwal. Gerbang yang tidak pernah menyala memberi bukti lebih
lemah daripada gerbang yang pernah menangkap sesuatu, dan itu tidak boleh
dibaca sebagai "heuristiknya terbukti benar". Yang benar dikatakan: fungsinya
**sama persis** dengan yang dipakai V4 untuk memeriksa solver exact, ia
membangun ulang traverse dari `ref_traverse` dan panjang perhentian dari
`ref_stop_slots` (enumerasi slot tuntas, tidak menyentuh bentuk tertutup mana
pun), jadi ia **bisa** menyala — ia hanya tidak perlu.

**H0 — bentuk tertutup yang dipakai heuristik diadu dengan `sched.stop_duration`.**
`sched_heur.stop_cost` meminimalkan `slots = m + max(s_A, s_B, z)` dalam
`O(|U|)` alih-alih menyapu `2^|SR|` penugasan lengan seperti `stop_duration`
(yang mustahil di `n = 50`). Keduanya diadu **tuntas**: 43 212 kasus
(himpunan tugas, pose) pada mask acak **dan** pada peta nyata → **0
ketidakcocokan, maks |selisih| 0.000e+00**. Versi tervektorisasi `dur_vec`
(dipakai di `pose-tour+wide` dan di gerbang Bagian II) diadu ulang dengan
`stop_cost`: 18 104 kasus, **0 ketidakcocokan**.

🔺 **Pertentangan 1 dengan §A2.** §A2 tahap 4 mengunci "durasi perhentian
dihitung exact lewat `sched.stop_duration`". Itu **tidak dijalankan seperti
tertulis**: `stop_duration` menyapu `2^|SR|` dan mati di Bagian II. Yang dipakai
adalah bentuk tertutup yang sama diminimalkan secara langsung, diadu tuntas
dengan `stop_duration` seperti di atas. Nilainya identik; jalannya tidak.

### B2. 🔺 K1 — **TIDAK TERCAPAI.** Rata-rata lolos telak, maksimum gagal telak

140 instance, seluruh himpunan K2, **tanpa satu pun dibuang**:

| penjadwal | n | mean % | median % | p90 % | max % | wall/instance | K1 |
|---|---|---|---|---|---|---|---|
| **`pose-tour`** | 140 | **2.45** | **0.00** | 7.17 | **24.31** | **9.0 ms** | **GAGAL** |
| `greedy` | 140 | 14.04 | 12.56 | 25.51 | 56.24 | 1.0 ms | gagal |
| `sequential` | 140 | 16.72 | 14.50 | 28.80 | 56.24 | 1.3 ms | gagal |
| `fixed` | 140 | 165.54 | 151.69 | 272.01 | 369.55 | 1.0 ms | gagal |
| `solve_exact` | 140 | 0 | 0 | 0 | 0 | 11 669 ms | — |

```
K1  mean gap 2.454 %  <= 5.0   ✓ LOLOS
    max  gap 24.311 % <= 10.0  ✗ GAGAL
    -> CUKUP BAIK sebagaimana didefinisikan A3-K1: TIDAK TERCAPAI
```

Ambang **tidak dinaikkan** dan **tidak ada instance dibuang**, sesuai A3-K1.

Yang perlu dicatat supaya angkanya tidak dibaca terlalu suram:

- `pose-tour` **persis optimal pada 72 dari 140 instance** (51.4%), dan
  median gap-nya **0.00%**.
- Ia **1 300× lebih cepat** dari exact (9.0 ms vs 11.7 s rata-rata). Seluruh 700
  jadwal heuristik Bagian I butuh **5.87 detik**; solver exact butuh **27.2
  menit** untuk 140 instance yang sama (anggaran §A memperkirakan 33 menit —
  perkiraan itu tepat dalam 20%).
- Kegagalannya **terpusat di instance 1 gantry**:

| belahan | n | mean % | max % | K1 pada belahan ini |
|---|---|---|---|---|
| 1 gantry | 70 | 3.81 | **24.31** | GAGAL |
| 2 gantry | 70 | **1.09** | **8.30** | **LOLOS** |

  Hanya **4 dari 140** instance melewati langit-langit 10%, dan keempatnya
  1 gantry.
- Dan satu angka yang tidak menyenangkan, dicatat karena ia justru yang paling
  informatif: pada **4 dari 140** instance Bagian I, `greedy` **dan**
  `sequential` menghasilkan makespan **lebih pendek** dari `pose-tour`. Yaitu:
  heuristik yang dirancang khusus kalah dari baseline miopik pada instance yang
  sama di mana ia melewati langit-langit K1. Di Bagian II (`n` = 12…50) itu
  **tidak pernah** terjadi — `pose-tour` terbaik atau seri pada **50 dari 50**.

**Tabel gap per konfigurasi (K5.1).** Kolom `mean`…`max` adalah `pose-tour`;
tiga kolom terakhir mean baseline pada instance yang sama persis. Sel `max` yang
melewati langit-langit 10% ditebalkan.

| n | G | mr | # | mean % | median % | p90 % | max % | `greedy` mean | `sequential` mean | `fixed` mean |
|---|---|---|---|---|---|---|---|---|---|---|
| 4 | 1 | 0 | 10 | 2.19 | 0.95 | 4.97 | 7.91 | 11.4 | 12.1 | 104 |
| 4 | 1 | 1 | 10 | 1.52 | 0.41 | 3.66 | 4.44 | 6.0 | 6.5 | 104 |
| 4 | 2 | 0 | 10 | 0.00 | 0.00 | 0.00 | 0.00 | 10.6 | 15.7 | 70 |
| 4 | 2 | 1 | 10 | 0.55 | 0.00 | 2.36 | 3.29 | 3.1 | 5.9 | 92 |
| 6 | 1 | 0 | 10 | 3.30 | 3.12 | 8.01 | 9.21 | 16.3 | 17.3 | 182 |
| 6 | 1 | 1 | 10 | 2.55 | 2.11 | 6.51 | 6.78 | 11.6 | 12.9 | 175 |
| 6 | 2 | 0 | 10 | 0.56 | 0.00 | 1.72 | 4.11 | 16.0 | 18.6 | 102 |
| 6 | 2 | 1 | 10 | 0.54 | 0.00 | 2.34 | 3.13 | 9.1 | 13.0 | 134 |
| 8 | 1 | 0 | 10 | 5.83 | 7.34 | 9.63 | **10.63** | 21.2 | 23.5 | 247 |
| 8 | 1 | 1 | 10 | 5.96 | 3.45 | 14.93 | **22.10** | 15.3 | 16.9 | 231 |
| 8 | 2 | 0 | 10 | 1.75 | 0.61 | 3.54 | 8.30 | 18.2 | 21.9 | 170 |
| 8 | 2 | 1 | 10 | 1.69 | 1.31 | 4.16 | 4.47 | 13.7 | 17.5 | 194 |
| 10 | 1 | 0 | 5 | 4.48 | 3.97 | 7.71 | 8.01 | 23.6 | 26.5 | 280 |
| 10 | 1 | 1 | 5 | 6.21 | 3.02 | 15.86 | **24.31** | 17.4 | 19.9 | 245 |
| 10 | 2 | 0 | 5 | 2.37 | 2.33 | 4.77 | 4.79 | 27.8 | 38.6 | 245 |
| 10 | 2 | 1 | 5 | 2.75 | 3.92 | 5.05 | 5.60 | 19.5 | 19.4 | 254 |

Perhatikan **kolom `median`**: ia 0.00 pada lima konfigurasi dan ≤ 4% pada
semuanya. Distribusi gap-nya **bukan** distribusi yang lebarnya tumbuh — ia
distribusi yang hampir seluruhnya nol dengan **beberapa kegagalan terisolasi**.
Itulah kenapa mean lolos dan max gagal, dan kenapa §B3 mencari satu keputusan
yang salah, bukan kelemahan yang tersebar.

**Ini bukan keberuntungan seed.** Diulang pada **60 instance held-out** (seed
10–14, `n` = 4/6/8, tidak pernah dilihat saat apa pun dirancang): mean **2.21%**,
median 0.97%, max **11.64%** — verdict yang sama, dan sekali lagi max-nya jatuh
pada instance 1 gantry (`n8_s12_g1_mr1`), sementara belahan 2-gantry lolos
(mean 1.00%, max 6.96%).

### B3. Ekor gap itu **BUKAN turnya** — ia pilihan pose handover. Ditelusuri ke data

Keempat instance yang melewati langit-langit, dan **tiga dari empat punya tugas
MR**. Mekanismenya tidak ditebak; ia dibaca dari jadwalnya:

| instance | exact | `pose-tour` | gap | pose handover exact | pose handover heuristik |
|---|---|---|---|---|---|
| `n10_s3_g1_mr1` | 54.540 | 67.800 | +24.31% | lin 1300, rot −130° (`T(p0,·)` = 41.67 s) | lin **1600**, rot −25° (**51.22 s**) |
| `n8_s3_g1_mr1` | 52.250 | 63.800 | +22.10% | lin 1300, rot −130° (41.67 s) | lin **1600**, rot −25° (**51.22 s**) |
| `n8_s8_g1_mr1` | 45.884 | 52.369 | +14.13% | lin 950, rot −80° (30.53 s) | lin **1250**, rot −155° (**40.08 s**) |
| `n8_s4_g1_mr0` | 61.025 | 67.510 | +10.63% | — (tanpa MR) | — |

Pada `n8_s3_g1_mr1` angkanya tutup persis: gap total **11.549 s** = panjang tur
**+9.55 s** (51.80 vs 42.25) + dwell **+2.0 s** (12.0 vs 10.0). Dan `9.55 s`
adalah **tepat** selisih `T(p0, lin1600) − T(p0, lin1300)`. Ekornya adalah satu
keputusan tunggal: pose handover yang salah, di ujung rel.

🔺 **Pertentangan 2 dengan §A2.** §A2 tahap 1 mengunci "**Pose handover dulu** …
karena himpunannya paling langka". Arah dari `p1_g7 §B4` benar — MR memang
kendala **pose**. Yang salah adalah **operasionalisasinya**: memilihnya
**paling dulu** berarti memilihnya saat turnya masih `[p0]` saja, sehingga skor
`cakupan / traverse` diukur terhadap tur yang belum ada. Pose jauh yang kebetulan
menutup banyak tugas memenangkan rasio itu, lalu **seluruh tur harus ke sana**.
Kelangkaan menuntut pose handover dipilih **secara sadar**; ia tidak menuntut
dipilih **lebih dulu**, dan §A2 mencampur kedua hal itu.

Instance keempat gagal dengan cara lain, dan itu juga terbaca dari jadwalnya:
perhentian exact `[2, 3, 3]` tugas versus heuristik `[1, 1, 6]`. Cakupan greedy
mengambil pose bercakupan-maksimum lebih dulu; dua tugas sisanya lalu jadi
perhentian tunggal masing-masing, sementara optimum memilih satu pose (lin 400,
rot −125°) yang menutup **keduanya sekaligus**.

**Dekomposisi agregat** (20 instance `n = 8`, 1 gantry, mutex on/off, kedua
nilai mr):

```
mean gap 2.945 s  =  dwell +2.000 s  +  traverse +0.945 s
perhentian: exact 3.05 rata-rata,  heuristik 2.65   <-- heuristik pakai LEBIH SEDIKIT
```

Jadi heuristiknya **terlalu rakus mengepak**, bukan terlalu boros berhenti —
kebalikan dari yang §A2 tahap 2 antisipasi ("yang mahal adalah jarak, bukan
jumlah stop").

**Mutex, sekali lagi, hampir tidak berbiaya** — bahkan untuk heuristik yang
sama sekali tidak mengoptimalkannya: **1 dari 53 perhentian** heuristik membayar
mutex, total **2.0 s** pada 20 instance. `p1_g7 §B3` diperluas: mutex tidak
menggerakkan makespan **bukan hanya untuk optimum**, tapi juga untuk penjadwal
yang mengabaikannya. §A2 benar menyuruh tidak menghabiskan usaha di sana.

### B4. 🔺 Pertentangan 3 — K4 **tidak bisa dijalankan seperti terkunci** di atas `n = 10`

`validate_schedule()` memanggil `ref_stop_slots()`, yang menempatkan setiap tugas
pada slot eksplisit secara **tuntas**: `2^|SR| × slots^|SR|` penempatan. Di
`n ≤ 10` itu murah dan itulah kenapa V4 bisa memakainya. Terukur di Bagian II:

| penjadwal | perhentian terbesar (`n = 50`) | penempatan yang harus dienumerasi |
|---|---|---|
| `pose-tour` | 12 tugas | **3.7e16** |
| `greedy` | 16 tugas | **1.2e24** |
| `fixed`, `sequential` | 1 tugas | 2 |

Ini ditemukan dengan cara yang mahal: dua proses evaluasi masing-masing membakar
**25 menit CPU pada satu instance `n = 20`** sebelum penyebabnya jelas. Bukan
heuristiknya yang lambat — `pose_tour` menyelesaikan instance itu dalam
**0.065 s**; yang menggantung adalah **gerbangnya**.

**Yang dilakukan, ditulis apa adanya karena ia melemahkan K4:**

- **Bagian I dan held-out: K4 dijalankan persis seperti terkunci**, 1 000 jadwal,
  `validate_schedule()` tanpa modifikasi. Di sini gerbangnya utuh.
- **Bagian II: anggaran 5e6 penempatan per perhentian.** Perhentian di bawah
  anggaran diperiksa dengan `ref_stop_slots` yang **sama persis**; yang di atas
  anggaran **panjangnya** diambil dari `dur_vec`. Semua aturan lain (tiap tugas
  tepat sekali, penugasan lengan sah di pose itu, MR benar-benar handover, waktu
  mulai vs `ref_traverse`, makespan = maks atas gantry) tetap diperiksa dengan
  cara yang keras.

Cakupannya dilaporkan sebagai angka, bukan disembunyikan:

| penjadwal | perhentian di-brute-force | di atas anggaran |
|---|---|---|
| `pose-tour` | 207 | 55 (**21.0%**) |
| `pose-tour+wide` | 197 | 51 (20.6%) |
| `greedy` | 714 | 33 (4.4%) |
| `fixed` / `sequential` | 1 280 | 0 (0.0%) |

Untuk 21% perhentian itu, rantai kepercayaannya **satu mata rantai lebih
panjang** dari yang K4 minta: `dur_vec` ← diadu tuntas dengan
`sched.stop_duration` (18 104 kasus, 0 ketidakcocokan) ← V1 mengadunya dengan
`ref_stop_slots` itu sendiri. Itu bukan independensi penuh, dan disebut begitu.

### B5. Baseline — D5 **MELESET**, D6 **TEPAT**, dan yang penting bukan batching

| | Aturan | mean gap | max gap |
|---|---|---|---|
| `greedy` | tur miopik, cakupan tak direncanakan | **14.04%** | 56.24% |
| `sequential` | satu tugas per perhentian | **16.72%** | 56.24% |
| `fixed` | tanpa tur, tanpa batching | **165.54%** | 369.55% |

- **D5 MELESET.** Dugaan: `sequential` terburuk telak, > 50% di atas optimum.
  Terukur: **16.72%**, dan ia **bukan** yang terburuk — `fixed` sepuluh kali
  lebih buruk. Arah melesetnya persis yang diramalkan papan skor: **kurang buruk
  dari dugaan**.
- **D6 TEPAT.** `greedy` (14.04%) mengalahkan `fixed` (165.54%), dan bukan tipis.

Yang tidak ditanyakan §A tapi terbaca langsung dari ketiganya: jarak
`fixed → greedy` adalah **151 poin**, jarak `greedy → sequential` hanya **2.7
poin**. Satu-satunya perbedaan `greedy` vs `sequential` adalah batching; satu-
satunya perbedaan `fixed` vs `greedy` adalah **mengunjungi pose yang dekat**.
➜ **Yang membeli makespan adalah tidak mengunjungi pose yang jauh, bukan
menumpuk tugas di satu perhentian.**

### B6. 🔺 Pertentangan 4 — D7 **MELESET**: tahap 3 (tur) menyumbang **NOL**

40 instance, `n` = 8/12/20/50, 2 gantry, seed 0–4, mr 0/1. Tiap tahap dimatikan
dan diukur, bukan diperdebatkan:

| varian | apa yang dimatikan | penalti mean | penalti max | lebih buruk pada |
|---|---|---|---|---|
| `nn-only` | 2-opt + or-opt | **+0.00%** | +0.00% | **0/40** |
| `cover-order` | seluruh penataan ulang tur | **+0.00%** | +0.00% | **0/40** |
| `no-refine` | tahap 4 (perbaikan lokal) | **+0.43%** | +3.82% | 15/40 |
| `neither` | tahap 3 **dan** 4 | +0.43% | +3.82% | 15/40 |

Nol, pada setiap `n` dari 8 sampai 50, pada setiap seed. Sebabnya terukur:
optimum maupun heuristik memakai **2–4 perhentian per gantry** (rata-rata
**2.62** di Bagian II). Dengan 2–4 simpul, urutan sisipan yang sudah dihasilkan
set cover **sudah merupakan lintasan optimal** — tidak ada yang tersisa untuk
diperbaiki 2-opt.

🔺 **Ini membatasi `p1_g7 §B5`.** §B5 menyimpulkan "masalah sebenarnya adalah
**tur pose gantry yang bergantung urutan** (TSP kecil dengan cakupan)" dan §A2
membangun tahap 3 di atas kalimat itu. Terukur: **bagian "bergantung urutan"-nya
tidak ada.** Yang menentukan adalah **pose mana yang masuk himpunan**, bukan
**urutan** kunjungannya. §B5 tetap benar bahwa biayanya ada di gerak gantry —
ia salah menempatkan biaya itu pada urutan. Dan §B5 sendiri sudah menandai
kalimat itu sebagai dugaan yang wajib diukur G8; G8 mengukurnya dan ia meleset.

Konsisten dengan §B3: ekor gap datang dari **satu pose yang salah dipilih**,
bukan dari satu tur yang salah diurutkan.

### B7. Bagian II — kurungan di atas `n = 10` (K3), dan **D8 MELESET**

50 instance, `n` = 12/16/20/30/50, 2 gantry, seed 0–4, mr 0/1. **Tidak ada exact
di sini**, jadi tidak ada satu pun angka di bagian ini disebut *gap*.

`UB` = makespan penjadwal terbaik yang lolos K4. `LB` = `max(LB_subset,
LB_analitik)`. `LB_subset` dihitung persis seperti dikunci: 3 himpunan bagian
acak ber-seed berukuran 8, exact masing-masing, diambil maksimumnya — **43.2 s
per instance, 36.0 menit total** (§A memperkirakan 43 menit).

| n | mr | UB mean | LB_analitik | LB_subset | LB | **rasio kurungan** mean | max | pangsa traverse |
|---|---|---|---|---|---|---|---|---|
| 12 | 0 | 38.564 | 33.484 | 35.566 | 35.566 | **1.083** | 1.101 | 80.3% |
| 12 | 1 | 47.097 | 42.079 | 43.795 | 43.795 | **1.082** | 1.224 | 84.7% |
| 16 | 0 | 42.474 | 34.758 | 36.076 | 36.076 | **1.182** | 1.233 | 77.4% |
| 16 | 1 | 49.305 | 42.079 | 42.130 | 42.937 | **1.151** | 1.207 | 82.1% |
| 20 | 0 | 45.110 | 36.031 | 38.171 | 38.171 | **1.183** | 1.203 | 74.0% |
| 20 | 1 | 52.360 | 42.715 | 40.515 | 43.631 | **1.206** | 1.336 | 80.2% |
| 30 | 0 | 51.719 | 36.667 | 37.313 | 37.549 | **1.380** | 1.424 | 65.7% |
| 30 | 1 | 57.824 | 42.715 | 38.104 | 42.715 | **1.360** | 1.490 | 69.6% |
| 50 | 0 | 64.425 | 40.806 | 39.622 | 42.006 | **1.534** | 1.612 | 58.9% |
| 50 | 1 | 70.484 | 42.715 | 41.451 | 43.597 | **1.617** | 1.657 | 60.1% |

Nol instance dengan `UB < LB` — kalau ada satu saja, salah satu batasnya tidak
sah dan seluruh bagian ini gugur.

**Kedua batas dipakai, dan keduanya perlu.** `LB_subset` menang (lebih ketat) di
`n = 12` pada 8/10 instance, tapi di `n = 30` dan `n = 50` `LB_analitik` menang
pada 5/10 dan 4/10 — karena optimum dari 8 tugas berhenti tumbuh sementara
instance penuh terus tumbuh. Menggabungkan dengan `max` (bukan memilih satu di
muka) adalah yang membuat kurungan ini tidak longgar di kedua ujung.
`LB_analitik` sendiri **selalu** didominasi `LB_reach` (50/50) — `LB_work` tidak
pernah mengikat, karena dwell 2.0 s terlalu murah dibanding traverse.

🔴 **Rasio kurungan naik dari 1.08 ke 1.62, dan itu TIDAK berarti heuristiknya
memburuk.** Yang terukur naik adalah **jarak antara UB dan sebuah batas yang
melonggar**: `LB_reach` adalah "waktu mencapai tugas terjauh + satu dwell",
sebuah besaran yang praktis **berhenti tumbuh** setelah `n ≈ 20` (40.8 s di
`n = 50` vs 36.0 s di `n = 20`), sementara pekerjaan sebenarnya terus bertambah.
Bagian I memberi kalibrasi yang benar untuk membaca ini: di sana gap sejati
`pose-tour` **turun** terhadap `n` pada 2 gantry, bukan naik. Jadi rasio 1.62 di
`n = 50` adalah pernyataan tentang **batas bawah yang lemah**, bukan tentang
heuristik yang 62% dari optimum. Ini harus ditulis begitu di naskah.

**Perbandingan penjadwal pada instance yang sama persis** (K5.2):

| penjadwal | valid | mean UB | relatif `pose-tour` | wall |
|---|---|---|---|---|
| `pose-tour` | 50/50 | **51.936 s** | — | 89 ms |
| `pose-tour+wide` | 50/50 | 51.941 s | +0.0% | 559 ms |
| `greedy` | 50/50 | 65.838 s | **+26.3%** | 3.2 ms |
| `sequential` | 50/50 | 68.559 s | **+31.5%** | 7.3 ms |
| `fixed` | 50/50 | 288.327 s | **+431.6%** | 3.7 ms |

Jarak `pose-tour` ke baseline **melebar** terhadap `n` (di Bagian I `greedy`
+14.0%, di sini +26.3%): justru di rezim yang exact tidak bisa masuk, heuristik
yang dirancang membayar paling banyak.

**D8 MELESET.** Dugaan: pangsa traverse tetap > 70% di `n = 50`. Terukur:
**59.5%** (min 53.1%, max 65.0%). Dan ia turun **monoton**: 80–85% di `n = 12`,
74–80% di `n = 20`, 66–70% di `n = 30`, 59–60% di `n = 50`.

➜ Konsekuensi untuk naskah: **`p1_g7 §B5` "74–92% makespan adalah gerak gantry"
adalah pernyataan tentang `n ≤ 10`, dan harus dikutip dengan `n`-nya.** Di
`n = 50` sudah 40% waktu adalah kerja. Mekanismenya jelas dan konsisten dengan
§B6: jumlah perhentian nyaris tidak tumbuh (rata-rata 2.62 per gantry), jadi
traverse hampir konstan sementara dwell tumbuh linear terhadap `n`. Ini juga
memprediksi bahwa mutex `r = 0.20` — yang hanya bisa berbiaya di dalam
perhentian — **akan mulai mengikat pada `n` yang lebih besar lagi**, yang tidak
diukur sesi ini dan tidak boleh diklaim.



### B8. `pose-tour+wide` — varian **POST-HOC**, dilaporkan terpisah dan tetap GAGAL

Dirancang **sesudah** membaca tabel §B2, jadi ia **tidak boleh** dihitung sebagai
bukti K1 pada seed K2. Ia tetap dilaporkan karena mekanismenya diturunkan dari
§B3 dan hasilnya menguji §B3 itu sendiri. Dua perubahan, keduanya menyerang
mekanisme yang §B3 ukur:

1. skor set cover mengenakan **durasi perhentian yang ia timbulkan**, bukan hanya
   traverse (`cakupan / (Δtraverse + dur)`), memakai `dur_vec` tervektorisasi;
2. penukaran pose tahap 4 mencari **seluruh 2 376 pose**, bukan tetangga grid ±2.

| himpunan | | mean % | max % | K1 |
|---|---|---|---|---|
| K2 (140) | `pose-tour` (terkunci) | 2.45 | **24.31** | GAGAL |
| K2 (140) | `pose-tour+wide` | **2.01** | **10.63** | **GAGAL** |
| held-out (60) | `pose-tour` | 2.21 | **11.64** | GAGAL |
| held-out (60) | `pose-tour+wide` | **1.82** | **11.64** | **GAGAL** |
| Bagian II (50) | `pose-tour+wide` vs terkunci | **+0.01%** | lebih baik pada 17, lebih **buruk** pada 11 | — |

Yang diperbaikinya **persis** apa yang §B3 ramalkan: pada tiga instance ekor
ber-MR ia memilih pose handover yang sama dengan optimum pada dua di antaranya
(`n10_s3`: 24.31% → 6.80%; `n8_s3`: 22.10% → 3.83%). Instance ekor tanpa MR
(`n8_s4_g1_mr0`, kegagalan batching) **tidak berubah sama sekali**: 10.63% →
10.63%.

Yang **tidak** diperbaikinya: apa pun di Bagian II (nol, dalam derau, sambil
**6× lebih lambat** — 559 ms vs 89 ms), dan max held-out (tetap 11.64%).

➜ **Kesimpulan jujur: `pose-tour+wide` bukan heuristik yang lebih baik, ia
heuristik yang satu bug-nya diperbaiki.** K1 tetap TIDAK TERCAPAI. Yang dipakai
naskah tetap `pose-tour` terkunci, dengan kegagalannya dilaporkan.

### B9. Papan skor §7.2 — sekarang **lima belas meleset, dua tepat**

| # | Dugaan (§A5, ditulis di muka) | Hasil |
|---|---|---|
| **D4** | `pose-tour` mencapai K1 (mean ≤ 5%, max ≤ 10%) | ❌ **MELESET** — mean 2.45% ✓, max 24.31% ✗. Tapi arah prior papan skor **tepat**: mean-nya memang "jauh di bawah 5%". |
| **D5** | `sequential` terburuk telak, > 50% di atas optimum | ❌ **MELESET** — 16.72%, dan bukan yang terburuk (`fixed` 165.54%). Arah prior tepat: **kurang buruk dari dugaan**. |
| **D6** | `greedy` mengalahkan `fixed` | ✅ **TEPAT** — 14.04% vs 165.54%. |
| **D7** | Tur (tahap 3) menyumbang lebih dari perbaikan lokal (tahap 4) | ❌ **MELESET TELAK** — tahap 3 menyumbang **+0.00%** pada 40/40; tahap 4 +0.43%. Arah prior tepat: "§B5 terlalu percaya diri, tahap 4 mungkin penting". |
| **D8** | Pangsa traverse tetap > 70% di `n = 50` | ❌ **MELESET** — **59.5%** (min 53.1, max 65.0). Arah prior tepat: **turun lebih cepat dari dugaan**. |

**Empat dari lima meleset. Total kini 15 meleset, 2 tepat.** Tapi yang lebih
berguna daripada rasionya: **prior papan skor sendiri benar pada empat dari
lima** (D4-mean, D5, D7, D8). Prior itu sekarang punya rekam jejak sendiri dan
layak dipakai sebagai alat kerja, bukan sekadar catatan kaki:

> **Kalau sebuah kendala atau efek belum diukur, tebakan awal yang benar adalah
> "ia lebih longgar / lebih kecil dari yang terasa".**

Dua catatan yang menahan prior itu supaya tidak dipakai berlebihan:

1. **D4 meleset ke arah sebaliknya di bagian max-nya.** Prior "longgar" berlaku
   untuk **dunia** (kendala fisik, geometri, mutex). Ia **tidak** berlaku untuk
   **konstruksi kita sendiri**: heuristik yang kita tulis ternyata **lebih
   buruk**, bukan lebih baik, dari dugaan — dan justru di ekornya. Prior yang
   optimistis tentang alam **bukan** lisensi untuk optimistis tentang kode
   sendiri.
2. Seperti D3 di G7, satu-satunya dugaan tepat (D6) adalah satu-satunya yang bisa
   ditelusuri ke mekanisme yang sudah dipahami sebelumnya (`fixed` tidak punya
   tur sama sekali, dan §B5-G7 sudah mengukur tur sebagai 74–92% biaya).

### B10. Batasan yang tetap berlaku setelah sesi ini

Seluruh `p1_g7 §A4`/`§B7` dan `p1_g8 §A4` masih berlaku. Yang ditambahkan atau
dipertajam sesi ini:

1. 🔴 **Tabrakan struktur gantry–gantry masih belum dimodelkan.** Semua angka
   rugi tetap **BATAS BAWAH**. Ini sekarang lebih penting, bukan kurang: §B6
   mengukur bahwa urutan tur tidak berbiaya — tapi urutan tur adalah persis
   tempat di mana tabrakan gantry–gantry **akan** berbiaya, karena ia satu-
   satunya hal yang bisa membuat dua gantry berkopling dalam **waktu**. Kalimat
   "tahap 3 menyumbang nol" berlaku **pada model yang tidak punya kopling waktu**
   dan tidak boleh dikutip di luar itu.
2. 🔴 **Rasio kurungan Bagian II adalah rasio terhadap BATAS, bukan gap.** `UB/LB
   = 1.65` di `n = 50` **tidak** berarti heuristiknya 65% dari optimum. Ia
   berarti optimumnya ada di suatu tempat di antara keduanya, dan §B7 menunjukkan
   **batasnya yang longgar**, bukan heuristiknya yang buruk.
3. **K4 di Bagian II hanya 79% independen** untuk `pose-tour` (§B4). Di Bagian I
   dan held-out ia 100%.
4. `T_fold = 0.0`, masih tidak pernah diukur. Ia menambah konstanta ke setiap
   traverse. Karena §B6 mengukur jumlah perhentian sebagai 2–4, `T_fold` akan
   menaikkan makespan sekitar `(2..4) × T_fold` — kecil, dan **menguntungkan**
   penjadwal yang berhenti lebih jarang, yaitu justru `pose-tour`.
5. Exact tetap exact hanya **terhadap grid 33 × 72**. Gap yang dilaporkan adalah
   gap terhadap optimum **pada grid itu**.
6. Gerak lengan di dalam satu pose masih 0.

---

## C. Prompt sesi berikutnya — G9

> Rekomendasi: **Opus 5, effort TINGGI.** Alasannya berubah dari G7/G8 dan harus
> dibaca: G8 punya ground truth yang menegur (solver exact) dan gerbang yang
> menegur (`validate_schedule`), dan **tetap** kehilangan 25 menit CPU × 2 karena
> gerbangnya sendiri eksponensial, dan **tetap** hampir melaporkan "tahap 3
> penting" karena §A2 mengatakannya. G9 memodelkan kendala baru — di sana **tidak
> ada** ground truth sampai Anda membangunnya, persis situasi `p1_state §7.1`.

```
Sesi G9 -- SCHED-3: memodelkan tabrakan struktur gantry-gantry, satu-satunya
kopling WAKTU antar gantry yang belum ada di model.

BACA DULU, berurutan:
1. docs/p1_g8_sched2.md   -- SELURUHNYA. B2 (K1 gagal di max, dan kenapa),
                             B3 (ekor = pilihan pose handover, bukan tur),
                             B4 (K4 eksponensial -- BACA SEBELUM menulis
                             evaluasi apa pun), B6 (urutan tur = 0.00%),
                             B9 (papan skor + batas prior), B10 (batasan)
2. docs/p1_g7_sched.md    -- A1 (model, terkunci), B3 (mutex 0.000 s),
                             B5 (74-92% -- perhatikan B8-G8: itu pernyataan
                             n <= 10; di n = 50 pangsanya 59.5%)
3. docs/p1_state.md       -- 5 (terkunci), 7.1, 7.2
4. reachability_gng/sched.py, sched_heur.py, test/verify_sched_exact.py,
   test/eval_sched_heur.py

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.

=== YANG SUDAH SELESAI, JANGAN BANGUN ULANG ===
- p1_state 7.1 langkah 1-4 SEMUA SELESAI. Generator + solver exact (G7),
  heuristik + 3 baseline + kerangka gap + batas bawah (G8).
- solve_exact tetap GROUND TRUTH untuk model TANPA tabrakan.
- pose-tour: mean gap 2.45%, max 24.31%, 9 ms. K1 TIDAK tercapai; itu
  dilaporkan apa adanya, JANGAN diam-diam diperbaiki lalu diklaim tercapai.

=== TUGAS ===
1. Modelkan tabrakan struktur gantry-gantry. Geometrinya sudah ditulis di
   sched.py docstring: pelat mount menyapu r = 0.4 m di y = +-0.36, tumpang
   tindih di y dalam [-0.04, 0.04]. Turunkan kapan dua pose (p1, p2) saling
   mengunci, dan kapan sebuah TRAVERSE memotong pose gantry lain.
2. Ini memperkenalkan kopling WAKTU antar gantry, yang p1_g7 B7.2 catat tidak
   ada di model sekarang. Konsekuensinya: solve_exact yang menyelesaikan tiap
   gantry independen lalu mengambil maksimum TIDAK LAGI EXACT. Bangun ground
   truth baru SEBELUM heuristik apa pun disentuh (7.1).
3. Ukur ulang, pada model baru: (a) apakah mutex r=0.20 masih 0.000 s,
   (b) apakah urutan tur masih menyumbang 0.00% (B6 memperkirakan TIDAK --
   ini dugaan, ukur), (c) berapa banyak makespan yang sebenarnya dibeli
   koordinasi.

=== KUNCI KRITERIA SUKSES SEBELUM SATU BARIS KODE, ke docs/p1_g9_sched3.md A ===
1. Definisi tabrakan sebagai PREDIKAT yang bisa diuji, bukan prosa.
2. Bagaimana ground truth baru dibuktikan exact -- V0-V4 setara.
3. Instance uji, ditetapkan sebelum melihat hasil.
4. Berapa besar kenaikan makespan yang dianggap "koordinasi itu mahal",
   sebagai ANGKA, ditulis sebelum diukur.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- validate_schedule() EKSPONENSIAL dalam tugas per perhentian (B4). Kalau
  evaluasi Anda menggantung, itu gerbangnya, bukan penjadwalnya.
- Urutan tur menyumbang NOL pada model tanpa kopling waktu (B6). Kalau ia
  masih nol SETELAH tabrakan dimodelkan, tabrakannya tidak mengikat.
- Mutex r=0.20 = 0.000 s pada optimum DAN pada heuristik (B3). Jangan
  habiskan sesi di sana.
- Pose handover langka dan mahal (g7 B4, g8 B3). Ekor gap ada di sana.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 15 meleset, 2 tepat. Prior: kendala
  yang belum diukur itu LONGGAR -- TAPI B9 catatan 1: prior itu berlaku untuk
  DUNIA, bukan untuk kode yang Anda tulis sendiri.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
- Akhiri dengan prompt sesi berikutnya (G10).
```
