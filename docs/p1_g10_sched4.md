# P1 / G10 — SCHED-4: menegakkan ground truth terkopel, lalu mengukur

> Sesi G10, 2026-08-14. Melanjutkan [p1_g9_sched3.md](p1_g9_sched3.md).
> G9 **gagal memenuhi kriterianya sendiri** (`p1_g9 §B4`): modelnya benar dan
> terverifikasi, solvernya tidak. Sesi ini menegakkan solvernya lebih dulu, dan
> baru sesudah itu mengukur.
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.**
> §B diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**, bukan dihaluskan. §A tidak ditulis
> ulang belakangan — kesalahannya dibiarkan bisa dibaca (disiplin `p1_g7`,
> `p1_g8`, `p1_g9`).
>
> Sesi ini **sepenuhnya offline**. Tidak ada kamera, gantry, lengan,
> `move_group`. Lengan 4× masih dilepas.
>
> **Rekomendasi model: Opus 5, effort TINGGI, untuk seluruh sesi.** Alasannya
> berbeda lagi dari G7/G8/G9 dan harus dibaca: sesi ini membangun **oracle untuk
> oracle**. W2 adalah enumerator brute force yang menghakimi solver, dan
> **enumerator brute force yang salah mencetak "0 ketidakcocokan" dengan sama
> senangnya** dengan yang benar. G9 punya tiga fase yang gagal senyap; G10 punya
> satu, dan ia yang paling dalam. Karena itu §A3-K1 di bawah dimulai bukan dari
> "bagaimana solver dibuktikan", melainkan dari **"bagaimana W2 sendiri
> dibuktikan"**.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci / terverifikasi di | Dipakai bagaimana |
|---|---|---|
| Model intra-gantry (sumber daya, timeline, SR/MR, mutex, `dur = 2.0 × max(a,b,z)`) | `p1_g7 §A1`, V0–V4 | dibaca apa adanya, **nol perubahan** |
| `T_traverse = max(T_lin, T_rot)` | `p1_state §5.6`, W1a (3.6e-15 s) | dipakai apa adanya |
| Dwell 2.0 s, definisi makespan, handover = SATU tugas dua-lengan | `p1_g7 §A2-K2/K3` | tidak ditawar |
| Peta `cap_g{1,2}_rail160.npz` (33×72, \|P\| = 2376) | `p1_state §3` | satu-satunya oracle kelayakan |
| `sched.solve_exact` | `p1_g7 §B1` | **batas bawah yang sah** (Lemma 3), pembanding W2b |
| Predikat `BLOCK` (`sched_coll.pair_distance`, `blocked`) | `p1_g9 §B1`: W0 (1184 kasus, 0 mismatch), W0b (2.2e-16) | dipakai apa adanya, **nol perubahan** |
| Lintasan traverse A2.3 + sertifikat A2.4 (`Traj`, `first_block`) | `p1_g9 §B2`: W1a/b/c/d | dipakai apa adanya |
| Lemma 1 (parkir aman, margin 0.21 m), Lemma 2 (ambang **0.095 m**), Lemma 3 (terkopel ≥ takterkopel), Lemma 4 | `p1_g9 §A2.2/§A2.5`, W0c | dipakai; Lemma 2 memakai angka yang **sudah dikoreksi** |
| `_repair_ub` + `UB_START_PROBES` | `p1_g9 §B6` probe (rasio 1.126) | **dipakai, tidak dibangun ulang** |
| `gen_real`, `gen_random_small`, `gen_real_crowded` | `p1_g7 §A3`, `p1_g9 §A3-K2` | generator, seed sama dengan G7/G8/G9 |
| `sched_heur.pose_tour` + 3 baseline | `p1_g8 §B2` | dipanggil, **dibungkus, bukan disunting** |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi:**

```
reachability_gng/sched.py          reachability_gng/sched_heur.py
reachability_gng/sched_coll.py     test/verify_sched_exact.py
test/verify_sched_coll.py          test/eval_sched_heur.py
test/eval_sched_coll.py
```

`sched_coll.py` **ikut dibekukan sesi ini**, dan itu keputusan sadar: W0/W0b/
W0c/W1 memverifikasi **berkas itu**, bukan idenya. Menyuntingnya berarti
verifikasi G9 tidak lagi berlaku pada kode yang dijalankan. Yang baru masuk
berkas baru dan mengimpor dari sana.

🔴 **Kalau bug ditemukan di berkas beku:** ia **dilaporkan sebagai temuan**,
diperbaiki di tempatnya, dan pembekuannya dinyatakan gugur secara eksplisit di
§B — tidak ditambal diam-diam di berkas baru.

### A1. Yang dibangun sesi ini, dan yang TIDAK

Dibangun, berurutan:

1. **Gerbang sadar-tunggu** (A2.4) — penerus `validate_schedule()` yang bisa
   memvalidasi jadwal yang mengandung waktu tunggu. Plus **uji mutasi**: gerbang
   yang tidak pernah menolak apa pun bukan gerbang (`p1_g8 §B1`).
2. **`feasible_starts`** (A2.2) — jawaban terhadap `p1_g9` Pertentangan 3.
   Conservative advancement **dalam `s`** (waktu mulai), bukan hanya dalam `t`,
   dengan konstanta Lipschitz yang diturunkan di A2.2 dan bukan ditebak.
3. **`solve_coupled2`** (A2.3) — B&B yang sama bentuknya, ditambah **dive**
   (konstruktor jadwal layak di setiap node) sebagai sumber incumbent.
4. **W2** — enumerator jadwal bersama brute force, **plus buktinya sendiri**
   (A3-K1).
5. **Pengukuran**: K3 pada S1 **dan** S2, K5.4 statistik predikat, K5.6 sapuan
   `c_clear`, ablasi mutex, ablasi urutan tur.

**TIDAK dibangun sesi ini, dan tidak boleh menyelinap masuk:** `T_fold ≠ 0`,
tabrakan lengan–lengan antar gantry, tabrakan lengan–lingkungan, kapabilitas
dinamis, dan **heuristik baru apa pun** sebelum gerbang A3-K2 lulus
(`p1_state §7.1`).

### A2. PENAMBAHAN MODEL DAN ALGORITMA — diturunkan sekarang, dikunci sekarang

#### A2.1 🔺 Pertentangan 0 — dengan prompt G10 sendiri: `(R, p1, p2)` + Pareto `(t1, t2)` **BUKAN statistik cukup**

`p1_g9 §C` menulis **"REKOMENDASI KUAT, dan ini BUKAN selera"**: ganti bentuk
solver jadi DP atas `(R1, p1, R2, p2)` dengan frontier Pareto `(t1, t2)`.
Rekomendasi itu **ditolak di sini, sebelum kode ditulis**, dengan alasan yang
bisa diperiksa. Menuliskannya di §A adalah intinya: kalau saya salah, §B akan
menegur saya dan pertentangannya sudah punya tempat.

**Kenapa ia tidak cukup.** Kendala A2.4 berbunyi *untuk setiap `t`*. Kalau gantry
`g` memulai leg pada `s` dan gantry `h` sedang **di tengah leg** pada
`[s, s+T]`, ujinya menuntut pose `h` **selama** selang itu. Pasangan
`(p_h, t_h)` hanya menyebut **di mana `h` berakhir** dan **kapan ia bebas** — ia
tidak menyebut lintasannya. Dua keadaan dengan `(R, p1, p2, t1, t2)` **identik**
bisa berbeda kelayakannya:

```
h leg A:  (0.80, +90 deg) -> (0.80, 0 deg)      berakhir di p_h, waktu t_h
h leg B:  (0.80, -90 deg) -> (0.80, 0 deg)      berakhir di p_h, waktu t_h  (sama)
```

Selama leg A, `h` menyapu `rot` dari +90°; selama leg B dari −90°. Menurut
Lemma 2 (ambang terkoreksi 0.095 m) hanya salah satunya yang bisa menabrak sapuan
`g` yang bersamaan. Jadi DP itu **tidak sound** (ia akan meloloskan jadwal yang
menabrak) kecuali ia melarang leg tumpang tindih sama sekali — dan melarang
konkurensi antar gantry berarti membuang paralelisme, yaitu membuang seluruh
alasan punya dua gantry.

**Statistik cukup yang minimal**, diturunkan sekarang dan dipakai sebagai
keadaan:

```
(R, g, p_g, t_g, ekor_h)      ekor_h = (p_h_prev, s_h, a_h, p_h, t_h)
```

dengan `g` = gantry yang `t`-nya lebih kecil (aturan ekspansi G9,
dipertahankan), `s_h` = waktu berangkat leg terakhir `h`, `a_h` = waktu tiba.
Kalau `a_h ≤ t_g` maka `h` statis di `p_h` dan ekornya menyusut jadi `p_h` saja.

**Dan dominasi Pareto komponen-per-komponen tidak sah**, yang membunuh separuh
sisa rekomendasinya:

🔒 **LEMMA 6 — "lebih awal" TIDAK selalu lebih baik.** Ambil dua keadaan yang
sama persis kecuali `h` tiba di `p_h` pada `a_h` versus `a_h' > a_h`. Kalau
`p_h` memblokir pose yang dibutuhkan `g`, keadaan dengan `a_h` yang **lebih
awal** memblokir `g` **lebih lama**, sehingga bisa memaksa makespan lebih besar.
Maka `(t1, t2)` terurut komponen-per-komponen bukan relasi dominasi yang sah pada
model ini. ∎
Akibatnya: memoisasi Pareto pada `(t1, t2)` **boleh membuang optimum**. Yang
dipakai sebagai gantinya adalah pemangkasan B&B murni (batas bawah admissible),
yang tidak pernah membuang optimum menurut konstruksi.

➜ **Yang dilakukan sebagai gantinya**, dan alasannya diukur, bukan dipilih:
`p1_g9 §B3` dan `§B6` sudah mengukur **dua** sebab kegagalan G9 yang **berbeda**,
dan masing-masing dapat obatnya sendiri:

| sebab terukur | di mana | obat sesi ini |
|---|---|---|
| himpunan waktu mulai terlalu kasar (tabrakan **di tengah traverse**, obatnya menunda beberapa detik, dan detik itu bukan batas leg apa pun) | `p1_g9 §B6` | **A2.2** `feasible_starts` |
| pencarian tidak lengkap: 1 dari 12 W2b mengembalikan nilai seed-nya sendiri | `p1_g9 §B4` | **A2.3** dive |

#### A2.2 🔒 `feasible_starts` — himpunan waktu mulai, DITURUNKAN

Aksi: gantry `g` dengan lintasan terkomit `A`, berangkat pada `s ≥ t_min`,
traverse `T` ke `q`, lalu menahan `dur`. Lawan `B` sudah terkomit penuh.
Jendela aksi `W = T + dur`. Definisikan

```
f(s) = min over t in [s, s+W] of dist( A_s(t), B(t) )
```

🔒 **LEMMA 7 — `f` adalah `V_POINT`-Lipschitz dalam `s`.**
`A_s(t) = A(t − s)` adalah pergeseran waktu murni: lintasan `g` **dalam jamnya
sendiri** tidak berubah oleh `s`. Ambil `s1 < s2`, `δ = s2 − s1`, dan `t*` yang
mencapai minimum untuk `s1`. Ambil `t' = t* + δ ∈ [s2, s2+W]`. Maka
`A_{s2}(t') = A(t* − s1)` — **titik yang sama persis**. Yang berubah hanya
`B`, dan setiap titik `B` bergerak paling cepat `V_POINT`, jadi
`dist` berubah paling banyak `V_POINT·δ`. Karena itu
`f(s2) ≤ f(s1) + V_POINT·δ`, dan simetrisnya. ∎

```
V_POINT = v_lin + omega * R_MAX = 0.031416 + 0.174533 * 0.455 = 0.110828 m/s
```

Perhatikan: konstantanya `V_POINT`, **bukan** `V_REL = 2 V_POINT` yang dipakai
A2.4 — di sana kedua benda bergerak relatif satu sama lain; di sini lintasan `g`
diam dalam jamnya sendiri dan hanya `B` yang bergerak. Ini konstanta yang
**lebih ketat**, dan itu diturunkan, bukan diambil dari A2.4 karena kebetulan
ada.

🔒 **LANGKAH MAJU YANG SAH.** Kalau `s` terhalang, `first_block` (A2.4, sudah
terverifikasi W1) memberi instan pelanggaran pertama `τ`, dan `d* =
dist(A_s(τ), B(τ)) ≤ c_clear + ε`. Untuk `s' = s + δ`, pada instan `τ + δ`
konfigurasi `g` **identik** dan `B` bergeser ≤ `V_POINT·δ`, jadi

```
dist( A_{s'}(τ+δ), B(τ+δ) )  <=  d* + V_POINT * delta
```

Maka setiap `s'` dengan `δ ≤ (c_clear − d*) / V_POINT` **pasti masih terhalang**
— dilewati dengan sah, bukan dengan harapan. Langkahnya:

```
delta = max( (c_clear - d*) / V_POINT , EPS_S )
```

🔒 **`EPS_S = 0.01 s` DIKUNCI**, dan konsekuensinya ditulis apa adanya: lantai
`EPS_S` bisa **melewati** selang layak yang lebih pendek dari `EPS_S`, jadi waktu
mulai yang dikembalikan boleh **terlambat** sampai `EPS_S` dari yang sejati.
Arah biasnya sama dengan `ε` A2.4: makespan yang dilaporkan adalah **batas atas**.
Besarnya dibatasi dan dihitung sekarang, bukan nanti: paling banyak satu aksi per
tugas plus satu gerak menghindar per gantry, jadi `≤ (n+1)·EPS_S = 0.07 s` pada
`n = 6` — **0.17% dari makespan tipikal 40 s**, dan **dua kali lipat di bawah**
resolusi yang dibutuhkan ambang 5% A3-K3 (2.0 s). `EPS_S` juga 76× di bawah
manuver mengelak termurah yang ada (0.76 s, `p1_g9 §A3-K3`), jadi ia tidak bisa
membalik satu pun keputusan struktural.

🔒 **HIMPUNAN KANDIDAT.** Jangkar (anchors):

```
{ t_min } U { batas leg B yang > t_min } U { B.end_time() }
```

dan untuk **setiap** jangkar, `feasible_starts` menjalankan langkah maju di atas
sampai layak atau melewati cakrawala `max(t_min, B.end_time())`. Hasilnya
himpunan **waktu mulai layak paling awal di atau sesudah setiap jangkar** —
superset sejati dari himpunan G9 (yang memakai jangkar itu **mentah**, tanpa maju
ke titik layak), dan ia memuat waktu mulai layak paling awal secara global.
Setelah `B.end_time()` lawan statis, jadi kelayakan berhenti bergantung pada `s`
dan satu uji menyelesaikan seluruh ekor tak hingga. Berhingga menurut
konstruksi.

🔒 **`K_START = 8`** waktu mulai per aksi, diambil dari yang paling awal.
Kalau tutup ini tersentuh, **dihitung dan dilaporkan sebagai angka** (K5.8) —
karena tutup yang tidak pernah dilaporkan adalah penjarangan yang disembunyikan.

➜ Maka solver tetap **exact TERHADAP himpunan kandidat ini**, bahasa yang
`p1_g7 §A2-K1` sediakan. Yang berubah dari G9: himpunannya sekarang **diturunkan
dengan bukti** dan **diuji oleh W2**, bukan dipilih karena murah.

#### A2.3 🔒 DIVE — dari mana incumbent berasal

`p1_g9 §B4` mengukur kegagalan kelengkapan: 1 dari 12 instance W2b mengembalikan
**persis nilai seed incumbent-nya**, yaitu pencarian tidak pernah menemukan apa
pun. Sebabnya bukan pemangkasan yang salah melainkan **tidak ada mekanisme yang
menghasilkan jadwal lengkap** kecuali dengan mengembangkan sampai `R = 0`.

```
di SETIAP node yang di-pop:
    UB_node = dive(keadaan node)     jadwal LAYAK penuh dari keadaan itu,
                                     dibangun dengan kelanjutan optimal DP
                                     per gantry, diperbaiki oleh feasible_starts
    kalau UB_node < best_m           -> incumbent baru
    kalau UB_node <= node_lb + 1e-9  -> node ini optimal
```

🔒 **LEMMA 8 — aturan berhenti.** Best-first: node yang di-pop punya `node_lb`
minimum di seluruh heap, dan `_split_lb` admissible (Lemma 3), jadi
`node_lb ≤ optimum`. Kalau dive dari node itu menghasilkan jadwal layak bernilai
`node_lb`, maka `optimum ≤ node_lb ≤ optimum`. **Selesai, terbukti optimal.** ∎

⚠️ **Dan justru karena itu dive berbahaya, dan bahayanya dikunci sekarang.**
Dengan tabrakan dimatikan, dive di **akar** langsung mengembalikan optimum
takterkopel = LB, jadi W2b akan lulus **tanpa mesin pencariannya pernah
berjalan** — persis gerbang-yang-tidak-pernah-menyala `p1_g8 §B1`, dalam bentuk
yang lebih halus karena ia terlihat seperti kemenangan.

🔴 **Maka W2b DIJALANKAN DUA KALI**, dan keduanya wajib lulus:
`(i)` dive menyala (jalur normal), `(ii)` **`no_dive = True`** — pencarian harus
menemukan optimum dengan ekspansi saja. Hanya `(ii)` yang menghitung sebagai uji
kelengkapan.

#### A2.4 🔒 GERBANG SADAR-TUNGGU — dan representasi jadwal yang diperlebar

`p1_g9 §B4` Pertentangan 4 mengukur: `validate_schedule()` menjumlahkan traverse
dan dwell **tanpa tempat untuk waktu tunggu**, jadi ia tidak bisa memvalidasi
keluaran model A2.5. Diperbaiki di sini, dan perbaikannya menyingkap sesuatu yang
§A9 G9 lewatkan:

🔺 **Representasi jadwal G7/G8/G9 AMBIGU pada model bertunggu.** Sebuah
perhentian `(pose, start, dur)` tidak menyebut **kapan traverse-nya terjadi**.
A2.5 mengizinkan menahan di pose sekarang **sebelum** berangkat, dan juga
menahan **setelah tiba** sebelum mulai bekerja. Dua penempatan leg yang berbeda
punya jejak tabrakan yang berbeda. Maka:

🔒 **Setiap perhentian membawa `depart` (waktu berangkat) DAN `start` (waktu
dwell mulai)**, dengan

```
depart >= perhentian sebelumnya .start + .dur          (tunggu di pose asal)
start  >= depart + T(pose sebelumnya, pose)            (tunggu di pose tujuan)
```

Gerbangnya (`validate_coupled`) memutar ulang **dari aturan**, bukan dari solver:
`T` dari `ref_traverse` (fungsi independen yang sudah dipakai V0/V4), panjang
dwell dari `ref_stop_slots` (fungsi yang sama, tanpa modifikasi, yang G7 V4 dan
G8 K4 pakai), kelayakan lengan langsung dari array oracle, lalu **kendala A2.4
diputar ulang pada `ε/10`** dari lintasan yang direkonstruksi dari `depart`.

🔴 **UJI MUTASI WAJIB.** Gerbang diadu dengan jadwal yang **sengaja dirusak**,
minimal lima kelas, dan wajib menolak **setiap** mutasi:

| mutasi | apa yang diuji |
|---|---|
| M1 `start` dimajukan 0.5 s | aritmetika traverse/tunggu |
| M2 `depart` dimajukan sampai leg tumpang tindih dengan leg lawan yang menabrak | separuh A2.4 gerbang |
| M3 satu tugas dibuang dari `tasks` | cakupan tugas |
| M4 satu tugas dipindah ke lengan yang tidak menjangkau | kelayakan lengan |
| M5 `dur` dikurangi satu slot | mutex/slot |

Jumlah mutasi yang **lolos** (yaitu gerbang gagal menolak) dilaporkan sebagai
angka, dan wajib **0**.

#### A2.5 Apa yang tidak berubah

Model A2.1–A2.5 `p1_g9` berlaku utuh: geometri, `BLOCK`, lintasan A2.3, kendala
kontinu A2.4 (`ε = 0.005 m`), aturan menunggu, definisi makespan, `c_clear = 0.0`
sebagai default terkunci. Gerak menghindar (`p1_g9` Pertentangan 2) **tetap
bagian model** dan tidak diperdebatkan ulang.

### A3. Kriteria yang DIKUNCI

#### K1 — BAGAIMANA **W2 SENDIRI** DIBUKTIKAN BENAR

Ini pertanyaan pertama prompt G10, dan ia ditaruh pertama karena W2 adalah satu-
satunya bagian sesi ini yang **tidak punya apa pun di atasnya**.

**Bentuk W2 dikunci:** pencarian **tuntas** atas jadwal bersama pada instance
kecil. Dua gantry. Di setiap langkah, gantry dengan `t` lebih kecil bertindak;
aksinya adalah `(pose tujuan ∈ P, himpunan tugas U ⊆ R yang layak di sana atau
U = ∅ untuk gerak menghindar, waktu mulai dari GRID SERAGAM)`. Tanpa B&B, tanpa
DP, tanpa `_split_lb`, tanpa `feasible_starts`, **tanpa Lemma 5 dan tanpa
Lemma 7**. Pemangkasan satu-satunya adalah incumbent (`acc ≥ best` → potong),
yang tidak membuang optimum. Grid waktu mulai `δ` **seragam**, dikunci
`δ = 0.25 s`, dan **dihalfkan jadi 0.125 s** pada instance yang mismatch.

**Lima bukti untuk W2 itu sendiri, semua wajib lulus** — dinamai `P0..P4` supaya
tidak tertukar dengan `W`:

| | Apa yang diadu | Kenapa itu mengikat |
|---|---|---|
| **P0** | lintasan dan `pose_at` versi W2 ditulis ULANG dari teks A2.3 di berkas uji, diadu dengan `sched_coll.Traj.pose_at` pada 2 000 waktu acak | kalau W2 memakai lintasan solver, ia tidak independen pada sumbu yang paling penting |
| **P1** | W2 dengan `BLOCK ≡ False` vs `sched.solve_exact` pada instance kecil yang **sama** | tanpa tabrakan, brute force wajib mereproduksi oracle yang sudah dibuktikan V0–V4 |
| **P2** | setiap jadwal yang W2 kembalikan wajib lolos **gerbang A2.4** (`validate_coupled`) | angka optimal yang menempel pada jadwal tak-tereksekusi tetap jawaban salah (pelajaran V4) |
| **P3** | `W2 ≥ optimum takterkopel` pada setiap instance (Lemma 3) | invarian yang tidak bisa dipenuhi enumerator yang bocor |
| **P4** | monotonisitas grid: `δ = 0.125` tidak boleh memberi nilai **lebih besar** dari `δ = 0.25` | enumerator yang melewatkan cabang biasanya tidak monoton; grid yang lebih rapat adalah superset (0.25 = kelipatan 0.125) |

🔴 **Kalau P0–P4 tidak semuanya lulus, W2 tidak boleh dipakai menghakimi apa
pun**, dan sesi ini melapor GAGAL di titik itu, bukan melanjutkan.

**Apa yang W2 hakimi, dan ke arah mana.** Solver memakai himpunan kandidat
waktu mulai (A2.2); W2 memakai grid seragam. Keduanya **restriksi**, jadi
perbandingannya asimetris dan aturannya dikunci sekarang:

```
solver > W2 + 1e-9   ->  KEGAGALAN SOLVER. Himpunan kandidatnya membuang
                         solusi yang grid rapat menemukan. Dilaporkan sebagai
                         angka dan sebagai instance.
solver < W2 - 1e-9   ->  BUKAN kegagalan: solver memakai waktu mulai di luar
                         grid. Dihitung, dilaporkan, dan diperiksa dengan
                         menghalfkan delta -- kalau W2 turun ke solver, gridnya
                         yang kasar; kalau tidak, itu jadi temuan.
```

#### K2 — PALANG "GROUND TRUTH", SEBAGAI ANGKA

Pertanyaan kedua prompt G10. Dijawab sebagai daftar angka, bukan sebagai
kalimat. `solve_coupled2` boleh disebut **ground truth (terhadap himpunan
kandidat A2.2)** jika dan hanya jika **semua** baris ini lulus:

| | Palang, sebagai angka |
|---|---|
| **Gerbang mutasi** | 5 kelas mutasi × ≥ 3 jadwal = **≥ 15 mutasi, 0 lolos** |
| **P0–P4** | semua lulus (K1) |
| **W2** | **≥ 24 instance**, `n ≤ 3`, `\|P\| ≤ 4`, 2 gantry; `solver > W2` pada **0 dari 24** |
| **W2b (i)** dive menyala | **12 dari 12** identik dengan `solve_exact` sampai **1e-9**, `proved = True` |
| **W2b (ii)** `no_dive` | **12 dari 12** identik sampai **1e-9**, `proved = True` |
| **W3** | **Q1–Q5 lima-limanya**, termasuk **Q3 yang G9 tidak pernah bangun** (`p1_g9 §B4` mencatat W3 lulus atas Q1/Q2/Q4/Q5 saja) |
| **W4′** | **100%** jadwal yang dilaporkan lolos gerbang sadar-tunggu, **0 pelanggaran** |
| **Regresi G9** | `n4_s2_mr1` dan `n4_s9_mr1` tetap `Δ = 0.000` (keduanya sudah terbukti exact, `p1_g9 §B6`) |

🔒 **Palang terpisah, dan JANGAN dicampur dengan yang di atas: "S1 TERJAWAB"**
= 40 dari 40 instance S1 punya `Δ` exact. Patokan konkret dari `p1_g9 §B6`: 27
sudah exact, 13 terkurung ≤ 1.211, 2 di antaranya sudah 1.000. **Menutup ke-13
itu adalah palang yang wajar dan itulah targetnya.** Tapi ia palang
**pengukuran**, bukan palang kebenaran: solver boleh jadi ground truth dan tetap
kehabisan waktu pada instance tertentu. Mencampur keduanya adalah cara termudah
menggeser definisi setelah melihat hasil.

#### K3 — VONIS BIAYA KOORDINASI, DAN APA YANG DILAPORKAN KALAU TIDAK SEMUA TERBUKTI

Pertanyaan ketiga prompt G10. Ambang **tidak digeser** — persis `p1_g9 §A3-K3`,
yang sendiri persis `p1_g8 §A3-K1`:

```
Delta      = makespan_terkopel - makespan_takterkopel      [detik, >= 0, Lemma 3]
Delta%     = Delta / makespan_takterkopel * 100 %

MAHAL              := mean Delta% >= 5.0 %  pada S1
TERUKUR TAPI MURAH := 0 < mean Delta% < 5.0 %  pada S1
GRATIS             := max Delta = 0.000 s pada SELURUH S1
```

🔒 **ATURAN VONIS DUA-SISI — dikunci sekarang, sebelum tahu berapa instance yang
bisa dibuktikan.** `p1_g9 §B6` terpaksa menulis "TIDAK DAPAT DITENTUKAN" karena
ia hanya bisa merata-ratakan subset yang **didefinisikan oleh `Δ = 0`**. Itu
tidak boleh terulang sebagai satu-satunya kemungkinan. Maka:

```
untuk instance terkurung, Delta_lo = 0            (Lemma 3)
                          Delta_hi = UB - LB      (kurungan yang dilaporkan)

mean_lo  = mean atas SELURUH S1 dengan setiap kurungan diambil Delta_lo
mean_hi  = mean atas SELURUH S1 dengan setiap kurungan diambil Delta_hi

kalau mean_lo dan mean_hi jatuh di SISI YANG SAMA dari 5.0 %  -> VONIS SAH
kalau tidak                                                   -> TIDAK DAPAT
                                                                 DITENTUKAN
```

Ini rigor, bukan kelonggaran: `mean_lo` dan `mean_hi` dihitung atas **seluruh 40
instance**, bukan atas subset, jadi tidak ada instance yang dibuang dan tidak ada
subset yang mengukur definisinya sendiri. Kalau vonisnya sah, ia sah **apa pun
nilai `Δ` sejati di dalam kurungan**.

🔴 Ambang 5.0% **dilarang** digeser setelah melihat hasil; instance **dilarang**
dibuang dari rata-rata. S1 dan S2 **tidak pernah** dirata-ratakan bersama.

#### K4 — HIMPUNAN INSTANCE UJI

Seed sama dengan G7/G8/G9. `S1`, `S2`, `S3` **persis** seperti `p1_g9 §A3-K2`
(masing-masing 40 / 40 / 20 instance), dipakai ulang tanpa perubahan supaya
angkanya bisa diadu langsung dengan `p1_g9 §B5/§B6`. `S4` = patologis Q1–Q5.
Baru sesi ini:

| Set | Isi | Peran |
|---|---|---|
| **S5 W2** | `gen_random_small`, `n ∈ {2,3}`, `\|P\| ∈ {3,4}`, 2 gantry, mr 0/1, ≥ 24 instance | satu-satunya bahan bakar W2 |

**Anggaran: `≤ 120 s dinding per instance`**, angka yang sama dengan
`p1_g7 §A2-K1` dan `p1_g9 §A3-K2`, **tidak dilonggarkan**. Instance yang tidak
muat dilaporkan sebagai kurungan dan masuk aturan dua-sisi K3.

#### K5 — YANG DILAPORKAN, TERLEPAS DARI HASILNYA

1. `Δ`, `Δ%` per konfigurasi `(set, n, n_mr)`: mean, median, p90, max, fraksi
   yang mengikat. **S1 dan S2 terpisah.**
2. `mean_lo` / `mean_hi` dan vonis dua-sisi K3, untuk S1 dan S2.
3. Berapa instance selesai lewat Lemma 4, lewat dive, dan lewat ekspansi penuh —
   tiga angka terpisah. Kalau dive menjawab 100%, **itu dikatakan**.
4. Statistik predikat pada grid: fraksi `2376 × 2376` pasangan pose yang `BLOCK`,
   dan fraksi pasangan `(rot1, rot2)` yang memenuhi (N1).
5. **(a)** mutex `r = 0.20` pada model terkopel — masih `0.000 s`?
   **(b)** urutan tur (`nn-only` / `cover-order`) pada model terkopel — masih
   `0.00%`? **(c)** biaya koordinasi = K3.
6. Sapuan `c_clear ∈ {0.00, 0.05, 0.10}` pada S1 dan S2.
7. Waktu dinding `solve_coupled2` vs `solve_exact`; `n` terbesar yang muat 120 s.
8. **Berapa kali tutup `K_START = 8` tersentuh, berapa kali `ACTION_BUDGET`
   tersentuh, berapa kali `EPS_S` menjadi lantai langkah** — tiga angka, karena
   ketiganya adalah penjarangan dan penjarangan yang tidak dilaporkan adalah
   penjarangan yang disembunyikan.
9. Jumlah jadwal yang gagal gerbang, per penjadwal. Jumlah mutasi yang lolos.

### A4. Yang TIDAK dimodelkan — setelah sesi ini

Seluruh `p1_g7 §A4`, `p1_g8 §B10`, `p1_g9 §A4`/`§B9` **masih berlaku**, dan
diulang di sini karena §B akan menggodanya:

1. 🔴 **Tabrakan lengan–lengan antar gantry masih di luar model.** Ujung lengan
   terentang 1.4 m dari sumbu rotasi, **tiga kali** `R_MAX = 0.455` yang
   dimodelkan. **Angka rugi TETAP BATAS BAWAH.**
2. `T_fold = 0.0`, tidak pernah diukur.
3. Gerak lengan di dalam satu pose = 0.
4. Exact hanya terhadap grid 33 × 72, terhadap `c_clear = 0.0`, dan terhadap
   himpunan kandidat waktu mulai A2.2.
5. Konservatisme: `ε = 0.005 m` (A2.4, terukur mengikat 1 dari 250) **dan**
   `EPS_S = 0.01 s` (A2.2, arah diketahui, besar dibatasi ≤ 0.07 s).
6. Lintasan traverse A2.3 tetap **tafsir** offset sebagai waktu mati.

### A5. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor: **17 meleset, 2 tepat** (`p1_g9 §B8`). Dua prior kerja, dan sesi ini
sengaja menguji keduanya:

* **tentang DUNIA:** kendala yang belum diukur itu **LONGGAR** — tapi `p1_g9 §B6`
  baru memberi contoh tandingan pertamanya (D9), pada kelas kendala **eksklusi
  ruang bersama**, bukan kelayakan statis.
* **tentang KODE SENDIRI** (`p1_g9 §B8`, kini 2 dari 2): **lebih lambat, lebih
  rumit, lebih salah dari yang terasa.**

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D14** | W2 menemukan **≥ 1** instance dengan `solver > W2` | **kode sendiri** | prior kode sendiri dipakai secara harfiah: himpunan kandidat A2.2 saya sendiri, dan saya menduga ia masih bocor di suatu tempat |
| **D15** | `feasible_starts` + dive menutup **≥ 8 dari 13** instance S1 yang mengikat jadi exact dalam 120 s | **kode sendiri** | probe `p1_g9 §B6` sudah menutup 2 dari 13 hanya dengan 40 probe seragam; kandidat yang punya bukti seharusnya jauh lebih baik — tapi prior kode sendiri menahan angkanya di 8, bukan 13 |
| **D16** | Vonis K3 pada S1 sah dan berbunyi **TERUKUR TAPI MURAH** (0 < mean `Δ%` < 5%) | dunia | `p1_g9 §B6`: batas atas `Δ` mean 5.13 s pada makespan 31–51 s → `mean_hi` ≈ 3–4%, sudah di bawah 5% **sebelum** dipersempit |
| **D17** | S2 (sempit) **MENGIKAT**: mean `Δ%` ≥ 5% | dunia | D10 G9 yang tidak pernah diukur, diterbitkan ulang tanpa perubahan |
| **D18** | Mutex `r = 0.20` tetap **0.000 s** pada model terkopel | dunia | D12 G9, diterbitkan ulang tanpa perubahan |
| **D19** | Urutan tur tetap **0.00% pada S1**, tapi **> 0 pada S2** | dunia | D11 G9, diterbitkan ulang tanpa perubahan |

D17–D19 adalah dugaan G9 yang **tidak pernah dinilai** karena gerbangnya gagal.
Menerbitkannya ulang **tanpa mengubah kata-katanya** adalah bagian dari
disiplinnya: dugaan yang menghindari penilaian dua sesi berturut-turut adalah
dugaan yang tidak pernah berisiko.

### A6. Berkas

| Berkas | Isi |
|---|---|
| `test/gate_sched_coupled.py` | `validate_coupled` (gerbang sadar-tunggu) + uji mutasi M1–M5 |
| `reachability_gng/sched_coupled.py` | `feasible_starts` (A2.2), `dive` (A2.3), `solve_coupled2` |
| `test/verify_sched_coupled.py` | P0–P4, W2, W2b(i)/(ii), W3 (Q1–Q5), W4′, regresi G9 |
| `test/eval_sched_coupled.py` | K3 S1/S2/S3, K5.1–K5.9, sapuan `c_clear`, ablasi mutex + urutan tur |
| `docs/p1_g10_sched4.md` | dokumen ini |

### A7. Urutan kerja, dan apa yang terjadi kalau waktu habis

```
1. gerbang sadar-tunggu + uji mutasi M1-M5      <- tanpa ini tidak ada yang bisa dinilai
2. feasible_starts + dive + solve_coupled2
3. W2 + P0-P4                                   <- oracle DULU, baru penilaian
4. W2b(i)/(ii), W3 Q1-Q5, W4', regresi G9       <- GERBANG K2: tidak lanjut kalau gagal
5. K3 pada S1, S2, S3 + K5.1-K5.4 + K5.7-K5.9   <- angka utama sesi ini
6. K5.5(a) mutex, K5.6 sapuan c_clear
7. K5.5(b) ablasi urutan tur                    <- perlu pembungkus heuristik
```

🔴 **Apa pun yang tidak tercapai dilaporkan sebagai TIDAK DIUKUR, sebagai kalimat
eksplisit di §B.** Dilarang menuliskan "diperkirakan tidak berubah" untuk sesuatu
yang tidak dijalankan. `p1_g9 §B7` mematuhi aturan ini dengan lima baris TIDAK
DIUKUR; kalau sesi ini harus menulis lima baris lagi, ia menulisnya.

---

## B. Hasil terukur

> §A dikunci 2026-08-14 sebelum `sched_coupled.py` ada (commit `053679a`). Semua
> angka di bawah keluar sesudahnya. Setiap tempat di mana §B bertentangan
> dengan §A ditandai 🔺. **§A TIDAK ditulis ulang.**

### B0. Cara menjalankan ulang

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 test/gate_sched_coupled.py            # gerbang + mutasi M1-M6
python3 test/verify_sched_coupled.py all      # P0-P4, W2, W2b(i)/(ii), W3, W4', REG
python3 test/eval_sched_coupled.py s3         # kontrol nol
python3 test/eval_sched_coupled.py s1         # angka utama
python3 test/eval_sched_coupled.py s2         # probe adversarial
python3 test/eval_sched_coupled.py grid       # K5.4
python3 test/eval_sched_coupled.py report
```

⚠️ **Waktu dinding sesi ini diukur pada mesin yang sedang dibebani hal lain:**
`load average 42` pada 16 core, karena node kamera ROS dari sesi lain
(`depth_cloud`, `color_cloud`, `realsense2_camera`) memakai ~13 core sepanjang
pengukuran. Itu **tidak** mengubah satu pun nilai `Δ` — solver deterministik —
tapi setiap angka detik di K5.7 adalah **batas atas** yang longgar. Disebut di
sini supaya tidak dibandingkan langsung dengan `p1_g7 §B2`.

### B1. 🔴 TIGA BUG NYATA di berkas yang §A0 BEKUKAN. Pembekuannya GUGUR.

§A0 mengunci `sched_coll.py` dan menulis aturannya: *"Kalau bug ditemukan di
berkas beku, ia dilaporkan sebagai temuan, diperbaiki di tempatnya, dan
pembekuannya dinyatakan gugur secara eksplisit di §B."* Aturan itu dipakai tiga
kali, dan dua yang pertama adalah bug yang **mengubah kebenaran**, bukan gaya.

#### B1.1 🔴 `first_block` mengevaluasi DUNIA CERMIN pada separuh panggilan

`pair_distance(l1, r1, l2, r2)` menaruh argumen **pertama** di `y = +0.36` dan
kedua di `y = −0.36`. Argumennya **ber-indeks gantry**, bukan bisa ditukar.
Tetapi `_first_start(tr, q, …, other)` memanggil `first_block(cand, other, …)`
— yaitu *"lintasanku, lintasan lawan"*. **Kalau pemanggilnya gantry 2, seluruh
uji kelayakan dijalankan pada konfigurasi cermin.**

Refleksi `y → −y` memang memetakan gantry 1 ↔ gantry 2, tapi ia **juga**
membalik `rot → −rot`. Menukar argumen tanpa membalik rotasi memberi
konfigurasi yang berbeda. Terukur:

| | jarak XY |
|---|---|
| konfigurasi sebenarnya `(0.80, −70°)` vs `(0.95, +95°)` | **0.000000 m** (menabrak) |
| argumen tertukar, seperti yang dilakukan `_first_start` | **0.188681 m** (dibaca aman) |
| tertukar **dan** rotasi dinegasikan (refleksi yang benar) | 0.000000 m ✓ |

Galat **188.7 mm** pada pasangan itu, dan pencarian acak menemukan **pembalikan
boolean pada kedua arah** di grid nyata (`g1 = (0.884, 125.4°)`,
`g2 = (0.950, −127.6°)`: benar `d = 0.00017`, tertukar `d = 0.00000`).

**Kena di mana:** setiap `_first_start` dan karenanya seluruh `_repair_ub`,
seluruh loop aksi `solve_coupled` G9, dan cabang menghindarnya — kira-kira
separuh dari semua uji kelayakan. **Tidak** kena: `schedule_conflict` dan
`conflict_free` (keduanya sudah mengurutkan berdasar gantry), jadi W0/W0b/W0c/W1
G9 dan gerbang replay-nya semuanya sah.

Perbaikan: dua baris di `first_block`, mengurutkan kedua lintasan berdasar
`Traj.g`, sehingga **semua** pemanggil beres sekaligus.

➜ **Regresi G9 dijalankan ulang setelah perbaikan dan hasilnya identik:**
W0 1184 kasus 0 mismatch, W0b 2.220e-16 m, W0c Lemma 1 = 0.2100 m dan Lemma 2
ambang 0.095 m, W1a 3.553e-15 s, W1b 250 pasangan 197 terhalang **0 terlewat**,
W1c 2.9568 s, W1d 0 blok. Jadi separuh-model G9 berdiri utuh; yang cacat
hanyalah pemakaiannya oleh solver.

#### B1.2 🔴 `_repair_ub` dan `_serial_ub` MENGEMBALIKAN jadwal yang bukan jadwal yang mereka periksa

Cabang "menyingkir" di `_repair_ub` meng-commit leg ke `trajs[g]` tetapi
**tidak pernah** menambahkannya ke `out[g]`. Jadwal yang **dikembalikan**
karenanya kehilangan satu leg yang dimiliki lintasan yang **divalidasi**. Semua
yang di hilir — gerbang, pemutar ulang, optimum yang dilaporkan — melihat gantry
berdiri di tempat yang sudah ia tinggalkan. `_serial_ub` punya cacat yang sama
untuk leg "parkir"-nya, dan di sana lebih tajam lagi: seluruh jadwal gantry
kedua digeser **dengan asumsi** gantry pertama sudah menyingkir.

Tertangkap oleh gerbang sadar-tunggu pada Q3: solver melaporkan makespan 20.5200
dengan `stops[2] = []`, padahal lintasan yang ia periksa berisi satu leg
menghindar. Tanpa gerbang itu, angka tersebut akan masuk §B tanpa ada yang
menegur — dan ia **tidak** akan tertangkap oleh `schedule_conflict` G9 pada
kebanyakan instance, karena jadwal yang kehilangan leg biasanya tetap
"bebas tabrakan" menurut pemutar ulang: gantry yang hilang legnya dianggap diam
di pose lamanya, yang justru pose yang **tidak** menghalangi.

#### B1.3 `hcol[∅]` = `inf` — bug saya sendiri, hari ini, dan gejalanya senyap

`solve_gantry` mengisi `w[A]` hanya untuk `A` tak-kosong dan membaca kasus kosong
dari `h[0] = 0`. Merekonstruksi `h` dari `w` untuk pose di luar `keep` (B3.2)
karena itu **wajib** memulihkan `h[∅] = 0`; lupa melakukannya membuat **setiap**
penerus menghindar bernilai `inf`. Gejalanya: pada Q3, cabang menghindar
mencoba ketiga targetnya, menemukan ketiganya **layak**, dan mendorong **nol**
node. Solver lalu melaporkan `proved = True` atas jawaban yang salah.

➜ Ketiganya ditemukan oleh **uji yang dijalankan**, bukan oleh pembacaan ulang.
Itu tiga dari tiga, dan konsisten dengan pola `p1_g9 §B8`.

### B2. Gerbang sadar-tunggu — SIAP, dan ia menyala

| | Cakupan | Hasil |
|---|---|---|
| kontrol positif | jadwal serialisasi hitung-tangan pada pose saling mengunci | **0 pelanggaran** (harus 0) |
| kontrol negatif | versi konkuren dari jadwal yang sama | ditolak, **A2.4 termasuk** |
| M1–M5 | 3 jadwal nyata Lemma-4 × 5 kelas mutasi | 15 mutasi |
| M6 | 2 optimum takterkopel nyata yang `p1_g9 §B6` ukur sebagai mustahil | ditolak, `A2.4 dilanggar pada t = 8.4995 s` |
| **total** | | **17 mutasi, 0 lolos** |

Dua angka yang membuat Pertentangan 4 `p1_g9` jadi konkret, bukan naratif:

* jadwal legal hitung-tangan itu mengandung **20.520 s waktu tunggu** — besaran
  yang representasi G7/G8 tidak punya tempat untuk menyimpannya;
* `validate_schedule()` G7/G8, dijalankan pada jadwal **legal** yang sama,
  melaporkan **3 "pelanggaran"**. Itu cacatnya, direproduksi sebagai angka.

🔺 **Satu aturan yang §A2.4 lupa tulis, ditambahkan di sini:** perhentian dengan
**nol tugas** adalah gerak menghindar; ia wajib berdwell 0 dan `ref_stop_slots`
tidak dikonsultasikan sama sekali (fungsi itu mengenumerasi penempatan slot dan
tidak punya jawaban untuk nol tugas). §A2.4 mewarisi asumsi G7/G8 bahwa setiap
perhentian membawa kerja; `p1_g9` Pertentangan 2 sudah membatalkannya dan §A
tidak mengikutinya sampai ke gerbang.

Dan **`finish` gantry = akhir DWELL terakhir, bukan akhir gerak terakhir**.
Gantry yang menyingkir setelah selesai tidak memperpanjang makespan
(`p1_g7 §A2-K2`); membaca perhentian terakhir secara buta menggelembungkannya
satu leg penuh, persis pada instance di mana tabrakan mengikat.

### B3. LIMA pertentangan struktural dengan §A — dan W2 menemukan dua di antaranya

#### 🔺 Pertentangan 1 — Lemma 7 BENAR tapi langkahnya DEGENERASI

§A2.2 menurunkan langkah maju `(c_clear − d*) / V_POINT` dan mengunci
`EPS_S = 0.01 s` sebagai lantainya. Lemma 7 tidak dicabut — ia benar. Yang
salah adalah menganggapnya bisa dipakai: **`pair_distance` memotong tumpang
tindih jadi tepat `0.0`** dan tidak melaporkan kedalaman penetrasi. Di dalam
selang terhalang, `d* = 0 = c_clear`, jadi langkahnya **nol**; di ujung selang,
`d*` sudah di dalam pita `ε`, jadi langkahnya **negatif**. Dua-duanya jatuh ke
lantai `EPS_S` dan berjalan merangkak 0.01 s per langkah.

Terukur sebelum ditulis ulang, pada `n4_s4_mr0`: **1547 langkah lantai**,
**8.0 s per dive**, **2 node dalam 74 s**. Yaitu `p1_g9 §B3` — *"yang lambat
adalah PEMERIKSA"* — terulang satu sesi kemudian, **di dalam kode yang ditulis
untuk menghindarinya**, dan sudah disebut sebagai jebakan di prompt G10 sendiri.

Penggantinya mempertahankan jaminannya dan membuang jalannya: satu sapuan
tervektorisasi atas seluruh grid `(waktu mulai × offset jam sendiri)`, dengan
prapenyaring `may_block`, lalu bisection selang layak pertama sampai `EPS_S`.
Lemma 7 tetap terpakai, dengan tugas berbeda: ia yang menjamin penghalusan
sampai `EPS_S` berada **di bawah resolusi uji tabrakannya sendiri**
(`V_POINT × EPS_S = 0.0011 m`, seperlima dari `ε = 0.005 m`).
Hasil: **74 s → 21 s**, lalu 4–14 s setelah Pertentangan 4.

🔒 Dua konstanta yang **tidak** ada di §A dan karena itu disebut: `DS_COARSE =
0.20 s` (kuantum grid waktu mulai sebelum penghalusan) dan `NU_MAX = 4000`.
Konservatismenya searah dengan semua yang lain: jendela layak yang lebih sempit
dari `DS_COARSE` bisa terlangkahi, jadi waktu mulai yang dikembalikan **tidak
pernah lebih awal** dari yang sebenarnya, dan makespan tetap batas atas.

#### 🔺 Pertentangan 2 — gantry TANPA TUGAS tidak bisa bergerak sama sekali

`p1_g9` Pertentangan 2 menulis ruang aksinya adalah `keep_g ∪ safe_g`;
kodenya **meng-iris**, bukan menggabung. Karena `solve_gantry` hanya menyimpan
pose di mana ada himpunan tugas yang layak, gantry **tanpa tugas** menyimpan
tepat pose awalnya dan **tidak bisa pindah**. Ia lalu memblokir gantry lain
selamanya dan instance dilaporkan **tidak layak**.

Terukur pada Q3 — kasus patologis yang `p1_g9 §A3-K1b` tentukan dan `p1_g9`
tidak pernah bangun: makespan terkopel kembali `inf`. Diperbaiki dengan
`PoseCost`, yang menghitung `h[A] = min_k (T(pose, k) + w[A][k])` untuk pose
grid **mana pun**, satu operasi vektor atas `|keep|`.

#### 🔺 Pertentangan 3 — pose menghindar yang AMAN bisa TAK TERCAPAI

Pose "aman universal" (Lemma 1, `|rot| < 31.7°`) cukup sebagai **tujuan**, dan
di situlah jebakannya: **rutenya** bisa terhalang justru ketika rute ke pose
tak-aman tidak. Terukur pada Q3: gantry 2 di `(0.8, +90°)` yang berputar ke
satu-satunya pose aman `rot = 0` menyapu turun melewati `+73°…+45°` dan menabrak
pada **t = 1.553 s**; berputar ke `rot = −135°` justru lewat **atas**, melalui
`+180°`, dan tidak pernah menabrak. Dengan target dibatasi ke pose aman solver
mengembalikan **20.5200**; enumerator brute force menemukan **11.2600**, jadwal
yang gerbang terima.

➜ Urutannya sekarang: pose aman dulu (termurah dulu), lalu **setiap** pose
lain menurut biaya traverse, `EVADE_ATTEMPTS = 60` percobaan dan `EVADE_PUSH =
4` penerus per node.

#### 🔺 Pertentangan 4 — "kembangkan gantry yang tertinggal" adalah HEURISTIK

Aturan ekspansi yang `p1_g9 §A3-K1` kunci — selalu kembangkan gantry dengan
`t_g` lebih kecil — **exact di model takterkopel** (dua jadwal saling bebas,
jadi urutan penyisipannya gratis) dan **heuristik di sini**: gantry yang
tertinggal bisa terhalang oleh yang di depan, dan gerakan yang membebaskannya
bisa berupa **tugas** yang gantry di depan harus kerjakan, bukan sekadar
menghindar.

Terukur pada bahan bakar W2 yang sempit, sebelum diperbaiki: **3 dari 8**
instance mengembalikan `solver > brute force`, sampai **0.96 s**, sambil
melaporkan `proved = True`. Sesudah kedua gantry dikembangkan: **0 dari 8**, dan
solvernya justru **lebih cepat** (7–26 node dan 4–14 s, dari ratusan node dan
24–133 s).

🔴 Ini pertentangan yang paling penting di sesi ini, karena ia satu-satunya yang
**tidak akan pernah ditemukan tanpa W2** — dan W2 adalah persis uji yang G9
tugaskan dan tidak bangun. Ia juga menegur `proved`: bendera itu berarti
*"tertutup terhadap himpunan kandidatnya sendiri"*, bukan *"optimal"*. Bahasa
`p1_g7 §A2-K1` dipakai apa adanya.

#### 🔺 Pertentangan 5 — memeriksa aksi hanya pada JENDELANYA SENDIRI tidak sound

Ditemukan oleh **P2**, yaitu gerbang yang diterapkan pada jadwal **W2 sendiri**.
Ini pertentangan yang paling halus di sesi ini dan satu-satunya yang ada di
**kedua** implementasi sekaligus, jadi tidak ada perbandingan silang yang bisa
melihatnya — hanya gerbang independen yang bisa.

Sebuah aksi diperiksa pada `[s, s + T + dur]` terhadap lintasan lawan **yang
sudah terkomit saat itu**. Sesudah jendela itu, gantrynya **berdiri di `q`** —
dan aksi yang lawan komit **belakangan** bisa menyapu ke sana pada waktu yang
**tidak dicakup jendela mana pun**: bukan jendela kita (sudah lewat), bukan
jendela lawan (ia hanya mencakup geraknya sendiri). Ekor statisnya jatuh di
antara keduanya.

Terukur: W2 mengembalikan jadwal Q4 yang gantry 1-nya masih berputar pada
**t = 9.5243 s** sementara gantry 2 baru saja parkir di `+90°` pada `lin` yang
sama — tabrakan nyata, diterima oleh W2 **dan** secara struktural oleh solver,
dan ditolak gerbang. Nilai Q4 yang salah itu **17.5100**; yang benar
**21.7600**.

Perbaikannya simetris di kedua sisi: setiap uji diperluas ke
`[s, max(s + T + dur, lawan.end_time())]`. Lalu waktu sebelum `s` tercakup oleh
uji milik lawan (gantry kita belum bergerak), waktu di dalamnya tercakup di
sini, dan konfigurasi akhir yang serba-statis tercakup oleh titik ujungnya.

⚠️ Ia **juga** ada di `_first_start` milik berkas beku, dan di sana ia baru
menyala pada `c_clear > 0`: satu jadwal S1 (`n4_s8_mr1`, route `dive-lb`)
ditolak gerbang pada `c_clear = 0.05` **dan** `0.10` sebelum diperbaiki. Pada
`c_clear = 0.0` ia tidak pernah menyala — yaitu bug yang **hanya** terlihat
lewat sapuan sensitivitas yang §A perintahkan karena alasan yang sama sekali
berbeda.

### B4. K5.4 — statistik predikat pada grid, dan (N1) sebagai plafon

| | Angka |
|---|---|
| pasangan pose `2376 × 2376` | 5 645 376 |
| yang `BLOCK` pada `c_clear = 0.00` | **183 836 = 3.2564 %** |
| pasangan `(rot1, rot2)` yang memenuhi (N1) | 1684 / 5184 = **32.48 %** |
| pose yang memblokir **sesuatu** | 1490 / 2376 = **62.71 %** |
| pose yang memblokir **tidak apa pun** (himpunan aman Lemma 1) | **886 / 2376** |
| `c_clear = 0.05` | 334 140 = 5.9188 % (**×1.82**) |
| `c_clear = 0.10` | 528 500 = 9.3616 % (**×2.87**) |

Dua bacaan, dan keduanya jalur data:

1. **Predikat exact-nya sepuluh kali lebih ketat dari plafon aljabarnya.**
   (N1) mengizinkan 32.48% pasangan rotasi; yang benar-benar bertabrakan
   3.26% pasangan pose. Jadi (N1) memang menyaring, tapi menyamakannya dengan
   tabrakan — kesalahan pertama G9 (`p1_g9` Pertentangan 1) — akan
   melebih-lebihkan kendala **sepuluh kali lipat**.
2. **37.3% ruang pose bebas-tabrakan tanpa syarat.** Itu himpunan parkir
   Lemma 1, dan ia besar. Ia sekaligus penjelasan mekanistik kenapa §B5–§B6
   berbentuk seperti itu: selalu ada tempat menyingkir, dan biasanya murah.

### B5. S3 — kontrol nol LULUS 20/20

`gen_real`, `n = 4`, seed 0–9, **satu** gantry, mr 0 dan 1. Tidak ada gantry
kedua, jadi `Δ` wajib nol.

```
20 dari 20 instance:  Delta = +0.000 s   (route 'single'),  gerbang 0 pelanggaran
```

Sama membosankannya dengan `p1_g9 §B5`, dan gunanya sama: kalau satu saja bukan
nol, `solve_coupled2` menambahkan biaya yang tidak berasal dari tabrakan dan
setiap angka `Δ` di bawah tercemar.


### B6. S1 — 🟢 vonis A3-K3 **DAPAT DITENTUKAN**: koordinasi **BUKAN MAHAL**

40 instance, `gen_real`, `n` = 4/6, seed 0–9, 2 gantry, mr 0/1, anggaran 120 s,
`c_clear = 0.0`. **Nol jadwal gagal gerbang.**

| | jumlah | |
|---|---|---|
| `Δ` **EXACT** | **37 / 40** | semuanya `Δ = +0.0000 s` |
| **KURUNGAN** | **3 / 40** | rasio 1.187, 1.187, 1.031 |
| route | `lemma4` 25, `dive-lb` 5, `bnb` 10 | |
| menunggu benar-benar dipakai | **8 / 40**, rata-rata 1.32 s | mekanisme A2.5 hidup |

```
A3-K3 dua-sisi:  mean Delta%  dalam  [0.0000 , 1.0141]  atas SELURUH 40 instance
ambang 5.0 %  ->  VONIS: BUKAN MAHAL (< 5 %)
```

🟢 **Inilah yang G10 beli.** `p1_g9 §B6` terpaksa menulis *"TIDAK DAPAT
DITENTUKAN"* dan melarang mengutip angkanya, karena satu-satunya subset yang
bisa ia rata-ratakan didefinisikan oleh `Δ = 0` dan karenanya mengukur
definisinya sendiri. Aturan dua-sisi A3-K3 — dikunci **sebelum** hasil ini ada —
merata-ratakan atas **seluruh 40** instance pada **kedua** ekstrem kurungan, jadi
tidak ada instance yang dibuang dan tidak ada subset yang bias. Kedua ekstrem
jatuh di sisi yang sama dari 5.0%, jadi vonisnya sah **berapa pun nilai `Δ`
sejati di dalam ketiga kurungan itu**.

Yang **tidak** boleh dikatakan: bahwa koordinasi **GRATIS**. Tiga kurungan
menyisakan `Δ > 0` sebagai kemungkinan, dan ambang GRATIS A3-K3 menuntut
`max Δ = 0.000` pada **seluruh** S1. Batas atasnya `mean Δ% ≤ 1.01%`, jadi:

> Koordinasi gantry–gantry pada instance alami berbiaya antara **0 dan 1.01%**
> makespan. Ia **di bawah** ambang 5% A3-K3, jauh lebih dekat ke mutex
> (`0.000 s`, `p1_g7 §B3`) daripada ke MR (`15.0%`, `p1_g7 §B4`).

Ketiga kurungan itu, dan ketiganya ada di dalam 13 instance yang `p1_g9 §B6`
tinggalkan terbuka:

| instance | LB | UB | rasio | `Δ ≤` |
|---|---|---|---|---|
| `n4_s7_mr0` | 39.304 | 46.660 | 1.187 | 7.356 s |
| `n6_s7_mr0` | 39.304 | 46.654 | 1.187 | 7.350 s |
| `n6_s6_mr1` | 31.346 | 32.333 | 1.031 | 0.986 s |

➜ **10 dari 13 instance yang G9 tinggalkan terkurung sekarang TERBUKTI
`Δ = 0.000` exact.** Palang pengukuran A3-K2 ("menutup ke-13 itu") tidak
tercapai penuh: **3 tersisa**, dan itu dilaporkan sebagai angka, bukan
dihaluskan.

#### 🔺 Dan `p1_g9 §B8` harus DIBALIK: **D9 sebenarnya TEPAT**

`p1_g9 §A5` D9 menduga `Δ = 0.000 s` pada **≥ 80%** instance S1. `p1_g9 §B6`
menilainya **MELESET** pada 67.5% (27/40), dan menaruhnya di papan skor sebagai
**contoh tandingan pertama** terhadap prior *"kendala yang belum diukur itu
LONGGAR"* — kesimpulan yang cukup besar sampai ia ditulis sebagai paragraf
tersendiri.

**Terukur sekarang: 37/40 = 92.5%, dan itu ≥ 80%.** D9 **TEPAT**.

Sebabnya bukan model yang berubah — **`route lemma4` tetap 25 di G9 dan di
G10, angka yang sama persis**, karena jalur itu memakai `schedule_conflict` yang
urutan gantrynya sudah benar sejak awal (§B1.1). Yang berubah adalah berapa
banyak dari 15 sisanya yang **bisa dibuktikan**: G9 membuktikan 2, G10
membuktikan 12. Jadi 67.5% bukan pengukuran tentang dunia; ia pengukuran tentang
**kelemahan solver G9**, dilaporkan sebagai kalau ia tentang dunia.

🔴 Pelajarannya lebih tajam daripada angkanya, dan berlaku ke depan: **sebuah
dugaan yang dinilai dengan solver yang belum tegak tidak dinilai sama sekali.**
`p1_g9 §B4` sudah menulis dengan tinta merah bahwa ground truth-nya belum tegak;
`p1_g9 §B6` tetap menjatuhkan vonis papan skor di atasnya. Itu satu-satunya
tempat di mana disiplin G9 bocor, dan ia bocor ke arah kesimpulan yang paling
menarik.

### B7. S2 — probe adversarial **TIDAK MENGIKAT SAMA SEKALI**, dan itu tentang PROBE-nya

40 instance `gen_real_crowded`, bentuk sama dengan S1. **Nol gagal gerbang.**

```
40 dari 40 EXACT,  Delta = +0.0000 s,  route lemma4 40/40,  GRATIS
wall rata-rata 4.2 s  (bandingkan S1: 18.8 s)
```

🔺 **D17 MELESET TELAK.** Dugaannya: S2 mengikat, `mean Δ% ≥ 5%`. Terukur: ia
**tidak pernah** mengikat, satu kali pun, dan **solver terkopelnya tidak pernah
dijalankan** — 40/40 dijawab Lemma 4.

Sebabnya terbaca dari rancangan probenya, dan ia **kesalahan §A3-K2 `p1_g9`**,
bukan temuan tentang dunia. `gen_real_crowded` mempersempit **kolam simpul**
ke `|x − 0.80| ≤ 0.30` dan menaruh `p0` kedua gantry di `(0.80, rot = 0)`. Itu
membuat tugas-tugasnya **dekat dengan pose awal**, sehingga makespan-nya jatuh
ke 4–9 s (S1: 28–50 s) dan gantrynya **nyaris tidak berputar**. Tapi (N1)
menuntut **kedua** gantry lebih dari 31.7° dari sejajar-rel. Probe itu
memadatkan sumbu yang salah: ia memadatkan `lin`, dan tabrakan menuntut `rot`.

➜ Jadi `p1_g9` D10 dan D17 sekarang keduanya tercatat sebagai **dugaan yang
tidak pernah benar-benar diuji**, karena alatnya tidak bisa mengujinya. Bahan
bakar W2 sesi ini (`gen_small_crowded`, §B8) menunjukkan bentuk probe yang
**benar**: pose aman di `rot = 0` sebagai `p0`, plus pose kerja di `±90°` pada
`lin` yang berdekatan, dan tugas yang **hanya** layak di pose `±90°` itu. Di
sana tabrakan mengikat pada **15 dari 31** instance dan biayanya 2.5–3.9 s.
**G11 harus mengulang S2 dengan generator berbentuk begitu.**

### B8. K5.5 — mutex dan urutan tur, keduanya diuji ULANG di bawah kopling waktu

| | Pertanyaan | `p1_g7`/`p1_g8` | **Terukur di model terkopel** |
|---|---|---|---|
| **K5.5(a)** | apakah mutex `r = 0.20` masih gratis? | `0.000 s`, `p1_g7 §B3` | **mean `+0.0000 s`, max `+0.0000 s`, mengikat pada 0/20**, 19/20 pasang dua-duanya terbukti |
| **K5.5(b)** | apakah urutan tur masih menyumbang nol? | `+0.00%`, `p1_g8 §B6` | **`nn-only` +0.0000%, `cover-order` +0.0000%, lebih buruk pada 0/20** — pada S1 **dan** S2 |

**D18 TEPAT.** **D19 setengah tepat** dan dicatat sebagai **MELESET**: ia benar
untuk S1 dan salah untuk S2, tapi §B7 baru saja menunjukkan S2 tidak mengikat
sama sekali, jadi separuh yang salah itu **belum benar-benar diuji**.

⚠️ **Yang harus dibaca dengan hati-hati:** `p1_g8 §B10.1` memperingatkan bahwa
"tahap 3 menyumbang nol" berlaku **pada model yang tidak punya kopling waktu**.
Sesi ini menjalankan pertanyaan itu **pada model yang punya**, dan jawabannya
tetap nol. Peringatan `p1_g8` karena itu **dicabut sebagian**: ia sekarang sudah
diuji, dan yang tersisa hanyalah bahwa ia diuji pada rezim di mana tabrakan
jarang mengikat (§B6: 37/40 `Δ = 0`). Cara mengujinya sampai tuntas adalah S2
yang benar-benar adversarial (§B7), bukan model yang lebih kaya.

Metodenya disebut supaya bisa diserang: heuristiknya **tidak disentuh** (A0
membekukan `sched_heur`); tiap varian merencanakan seperti biasa, lalu
rencananya **ditempatkan dalam waktu** oleh `sched_coupled.repair_schedule`,
yang mempertahankan setiap pilihan `(pose, himpunan tugas)` dan hanya
menggeser jam. Jadi yang diukur adalah **berapa harga turnya setelah dua gantry
terkopel dalam waktu**, yang persis pertanyaan yang `p1_g8 §B10.1` biarkan
terbuka.

### B9. 🟡 GERBANG A3-K2: **7 dari 8 baris lulus, 1 GAGAL.** Ground truth **BELUM** boleh disebut tegak.

| Palang A3-K2 | Terukur | |
|---|---|---|
| Gerbang mutasi ≥ 15, 0 lolos | **17 mutasi, 0 lolos** | ✅ |
| **P0** model gerak W2 vs `sched_coll.Traj` | 300 lintasan × 200 waktu, maks \|selisih pose\| **3.331e-16**, durasi **0.000e+00 s** | ✅ |
| **P1** W2 tanpa tabrakan vs `sched.solve_exact` | 12 instance, **0 ketidakcocokan** | ✅ |
| **P2** setiap jadwal W2 lewat gerbang | 31 jadwal, **0 ditolak** | ✅ |
| **P3** W2 ≥ optimum takterkopel (Lemma 3) | **0 pelanggaran** | ✅ |
| **P4** grid dihalfkan tidak menaikkan W2 | 8 instance, **0 naik** | ✅ |
| **W2** `solver > W2` pada 0 dari ≥ 24 | **31 instance** (15 mengikat, 2 timeout), **solver > W2 pada 0** | ✅ |
| **W2b(i)** dive menyala | **12 / 12** identik sampai 1e-9, `proved` | ✅ |
| **W2b(ii)** `no_dive` | **6 dari 12 GAGAL** — mengembalikan persis nilai seed-nya | ❌ |
| **W3** Q1–Q5 lima-limanya | **lima-limanya LULUS**, termasuk **Q3 yang G9 tidak pernah bangun** | ✅ |
| **W4′** 100% jadwal lolos gerbang | 12 jadwal, **0 pelanggaran**, 9.9 s tunggu di dalamnya | ✅ |
| **Regresi G9** `n4_s2_mr1`, `n4_s9_mr1` | keduanya `Δ = +0.000000` | ✅ |

🔴 **Vonis: `solve_coupled2` TIDAK boleh disebut ground truth tanpa kualifikasi**,
karena palang A3-K2 menuntut kedelapan barisnya dan W2b(ii) gagal. Itu ditulis
sebagai vonis, bukan catatan kaki, persis seperti `p1_g9 §B4`.

**Apa arti kegagalan itu, tepatnya.** W2b(ii) mematikan dive dan menanyakan
apakah **mesin ekspansinya sendiri** bisa menemukan optimum dalam 90 s ketika
incumbent-nya di-seed 2.0 s di atas. Jawabannya tidak, pada 6 dari 12: ia
mengembalikan persis nilai seed. Jadi **dive-nya menanggung beban**, dan itu
diukur, bukan ditutupi.

**Dan apa yang TIDAK diruntuhkannya**, karena ini menentukan apakah §B6 boleh
dikutip:

1. **W2 menguji solver LENGKAP DENGAN dive-nya**, terhadap enumerator yang tidak
   berbagi satu baris pun dengan pencariannya — 31 instance, 15 di antaranya
   tabrakannya benar-benar mengikat, **0 kali `solver > W2`**. Itu justru bukti
   yang lebih kuat daripada W2b(ii), karena W2b(ii) mematikan tabrakan sama
   sekali.
2. **Nilai `Δ = 0` yang §B6 laporkan tidak bergantung pada kelengkapan
   pencarian.** Ketiga jalurnya adalah bukti tersendiri: `lemma4` (25 instance —
   optimum takterkopel sendiri bebas tabrakan), `dive-lb` (5 — jadwal layak
   menyentuh batas bawah Lemma 3 di akar), dan `UB = LB` di dalam B&B (7). Semua
   bertumpu pada Lemma 3 + Lemma 4 + sertifikat A2.4 yang W1 verifikasi, **tidak
   satu pun pada pencariannya**. Yang tersisa dilaporkan sebagai kurungan.

➜ Jadi **vonis A3-K3 di §B6 sah**, dan **sebutan "ground truth" tidak**. Dua
kalimat yang berbeda, dan sesi ini menuliskannya sebagai dua kalimat.

⚠️ **Batasan W2 sendiri, disebut karena W2 adalah oracle:** pada **9 dari 31**
instance `solver < W2`, dan **menghalfkan gridnya menutup 0 dari 9** — jadi
selisihnya **bukan** grid. Sebab yang paling mungkin, dan G11 harus
mengukurnya: W2 dibatasi `max_evade = 1` per gantry, sementara solver memakai
sampai **3** gerak menghindar (Q3). Artinya W2 **longgar sebagai batas atas**,
dan aturan A3-K1 memang hanya memakainya satu arah (`solver > W2` = kegagalan).
Ia **tidak** boleh dibaca sebagai "optimum sejati".

### B10. K5.6 — sapuan `c_clear`, dan K5.7/K5.8

Sapuan dijalankan pada subset `n = 4` (20 instance per set per nilai), dan itu
disebut sebagai angka, bukan dihaluskan. **Nol jadwal gagal gerbang pada
seluruh sapuan** — tapi hanya setelah Pertentangan 5 diperbaiki juga di
`_first_start`; sebelum itu satu instance (`n4_s8_mr1`) ditolak gerbang pada
`c_clear = 0.05` **dan** `0.10` dan tidak pernah pada `0.00`:

| set | `c_clear` | exact / kurungan | gagal gerbang | `mean Δ%` dua-sisi | vonis |
|---|---|---|---|---|---|
| S1 (n = 4 dan 6) | 0.00 | 37 / 3 (dari 40) | **0** | **[0.0000, 1.0141]** | BUKAN MAHAL |
| S1 (n = 4) | 0.05 | 19 / 1 (dari 20) | **0** | **[0.0000, 1.0526]** | BUKAN MAHAL |
| S1 (n = 4) | 0.10 | 17 / 3 (dari 20) | **0** | **[0.0000, 2.6868]** | BUKAN MAHAL |
| S2 | 0.00 / 0.05 / 0.10 | 40 / 0, 20 / 0, 20 / 0 | **0** | **[0, 0]** | GRATIS |

**Arah dan besarnya:** monoton naik, dan **modest** — batas atas `Δ%` naik
1.01 → 1.05 → 2.69 saat `c_clear` naik 0 → 0.05 → 0.10 m. Vonis A3-K3 **tidak
berubah** pada ketiganya. Bandingkan dengan jalur data K5.4, di mana fraksi
pasangan pose yang `BLOCK` naik ×1.82 dan ×2.87 pada nilai yang sama: kendala
geometrinya hampir tiga kali lipat, biayanya hanya tiga kali lipat dari
sesuatu yang mendekati nol. Itu konsisten dengan §B4: 37.3% ruang pose bebas
tabrakan tanpa syarat, jadi selalu ada tempat menyingkir.

🔒 **`c_clear = 0.0` tetap default terkunci** dan tetap tidak pernah diukur pada
perangkat keras (`p1_state §7.2`). Yang di atas adalah sensitivitas, bukan
kalibrasi.

**K5.7 waktu dinding**, dengan peringatan beban mesin di §B0:
`solve_coupled2` pada S1 rata-rata **18.8 s**, maks **120.1 s** (anggaran
tersentuh pada instance yang terkurung); S2 rata-rata **4.2 s**; S3 **1.1 s**.
`n` terbesar yang muat anggaran 120 s: **`n = 6` pada `\|P\| = 2376`, 2 gantry**
— yaitu **D13 `p1_g9` sekarang terpenuhi**, pada kode yang berbeda.

**K5.8 anggaran yang tersentuh, S1** (40 instance): `action_hit = 35`,
`multi_skipped = 13 640`, `walks = 58 884`, `dives = 62` (`dive_proved = 7`),
`k_start_hit = 0`, `evade_tried = 0`, `eps_s_floor = 0`.
Dua bacaan: (a) `k_start_hit = 0` berarti tutup `K_START = 8` **tidak pernah**
mengikat, jadi ia bukan penjarangan yang aktif; (b) `action_hit = 35` berarti
anggaran aksi **sering** mengikat, dan itulah kenapa 3 instance tersisa sebagai
kurungan. `eps_s_floor = 0` karena mekanismenya sudah diganti (Pertentangan 1),
dan pencacahnya dibiarkan untuk mencatat itu.

### B11. Papan skor §7.2 — dan **satu vonis G9 DIBALIK**

| # | Dugaan (§A5, ditulis di muka) | Hasil |
|---|---|---|
| **D14** | W2 menemukan ≥ 1 instance `solver > W2` | ✅ **TEPAT** — 3 dari 8 pada bahan bakar sempit, sampai 0.96 s, sambil melapor `proved = True` (§B3 Pertentangan 4) |
| **D15** | ≥ 8 dari 13 instance S1 yang mengikat tertutup jadi exact | ✅ **TEPAT** — **10 dari 13** (§B6) |
| **D16** | Vonis S1 sah dan berbunyi TERUKUR TAPI MURAH (`0 < mean Δ% < 5%`) | ❌ **MELESET** — vonisnya **sah** dan **< 5%** ✓, tapi `0 <`-nya **tidak tegak**: mean-nya ada di `[0, 1.01%]` dan GRATIS tidak bisa disingkirkan. Meleset ke arah **lebih longgar** dari dugaan |
| **D17** | S2 mengikat, `mean Δ% ≥ 5%` | ❌ **MELESET TELAK** — 40/40 `Δ = 0`, 40/40 lewat Lemma 4, solver terkopelnya **tidak pernah jalan** (§B7) |
| **D18** | Mutex tetap `0.000 s` di model terkopel | ✅ **TEPAT** — mean dan max `+0.0000 s`, mengikat 0/20 |
| **D19** | Tur `0.00%` di S1, `> 0` di S2 | ❌ **MELESET** — `0.00%` di S1 ✓ dan **juga** `0.00%` di S2; tapi §B7 menunjukkan S2 tidak mengikat, jadi separuh yang salah itu belum benar-benar diuji |

**Tiga tepat, tiga meleset.**

🔺 **Dan `p1_g9 §B8` harus dikoreksi: D9 dinilai MELESET dan sebenarnya TEPAT**
(§B6). Papan skornya karena itu: `p1_g9` berdiri di 17 meleset / 2 tepat; D9
pindah sisi → **16 / 3**; sesi ini menambah 3 / 3 → **19 meleset, 6 tepat**.

Tiga bacaan, dan yang ketiga yang paling berguna:

1. **Prior DUNIA ("kendala yang belum diukur itu LONGGAR") tidak punya contoh
   tandingan lagi.** `p1_g9 §B6` mengangkat D9 sebagai contoh tandingan
   pertamanya dan menulis satu paragraf tentang "kelas kendala kedua". Dengan
   D9 dibalik, dan dengan D16/D17/D19 semuanya meleset **ke arah longgar**,
   prior itu justru **menguat**: sekarang 4 dari 4 dugaan dunia sesi ini meleset
   ke arah kendala lebih longgar.
2. **Prior KODE SENDIRI ("lebih lambat, lebih rumit, lebih salah") dipakai
   secara harfiah di D14 dan D15, dan keduanya TEPAT.** Ini pertama kalinya
   dugaan tentang kode sendiri kena — karena ia ditulis **pesimis dengan
   sengaja**. Prior itu sekarang 4 dari 4 (D4-max G8, D13 G9, D14, D15 di sini)
   dan layak dinaikkan dari catatan kaki jadi metode: *tulis dugaan tentang kode
   sendiri pada sisi pesimisnya, lalu ia bisa dinilai.*
3. 🔴 **Pelajaran metodologis yang paling mahal sesi ini, dan ia bukan tentang
   tabrakan:** `p1_g9 §B4` menyatakan ground truth-nya BELUM tegak, lalu
   `p1_g9 §B6` **tetap menjatuhkan vonis papan skor** di atas solver yang sama.
   Vonis itu salah, dan salahnya ke arah yang paling menggoda — kesimpulan yang
   paling menarik ("prior utama proyek ini punya contoh tandingan pertamanya").
   Aturannya sekarang eksplisit:

   > **Dugaan yang dinilai memakai solver yang belum lulus gerbangnya tidak
   > dinilai sama sekali. Ia ditandai TERTUNDA, bukan MELESET.**

### B12. Batasan setelah sesi ini

Seluruh `p1_g7 §A4`/`§B7`, `p1_g8 §B10`, `p1_g9 §A4`/`§B9` **masih berlaku**
kecuali yang §B di atas cabut secara eksplisit. Yang ditambahkan:

1. 🔴 **`solve_coupled2` BUKAN ground truth menurut palang A3-K2** (§B9,
   W2b(ii)). Yang boleh dikutip: vonis A3-K3 §B6, karena ketiga jalur
   exactness-nya tidak bergantung pada kelengkapan pencarian.
2. 🔴 **Angka rugi TETAP BATAS BAWAH.** Tabrakan **lengan–lengan** antar gantry
   masih di luar model. Ujung lengan terentang 1.4 m dari sumbu rotasi, **tiga
   kali** `R_MAX = 0.455` yang sesi ini modelkan. Sesi ini mengukur bahwa
   kendala **struktur** longgar (≤ 1.01% pada S1); itu **tidak** menyiratkan
   apa pun tentang lengan.
3. **S2 sebagai probe adversarial GAGAL BERFUNGSI** (§B7). `gen_real_crowded`
   memadatkan `lin`, dan (N1) menuntut `rot`. D10 (`p1_g9`) dan D17 karena itu
   **belum pernah benar-benar diuji**.
4. **W2 longgar sebagai batas atas** (`max_evade = 1`, §B9), dan dipakai hanya
   satu arah.
5. Konservatisme yang menumpuk, semuanya searah (makespan yang dilaporkan
   adalah **batas atas**): `ε = 0.005 m` (A2.4), `EPS_S = 0.01 s` dan
   `DS_COARSE = 0.20 s` (A2.2 + Pertentangan 1).
6. `T_fold = 0.0`, masih tidak pernah diukur. `c_clear = 0.0`, tidak pernah
   diukur; sapuannya (§B10) sensitivitas, bukan kalibrasi.
7. Exact tetap exact hanya terhadap grid 33 × 72 dan terhadap himpunan kandidat
   waktu mulai A2.2.
8. Lintasan traverse A2.3 tetap **tafsir** offset sebagai waktu mati.

---

## C. Prompt sesi berikutnya — G11

> Rekomendasi: **Opus 5, effort TINGGI.** Alasannya berbeda lagi, dan harus
> dibaca. G7–G10 semuanya punya struktur yang sama: bangun model, bangun
> oracle, ukur. G11 adalah sesi pertama yang harus **memutuskan apa yang layak
> dibangun berikutnya**, dan pilihan itu tidak punya sinyal error sama sekali —
> membangun hal yang salah dengan sempurna tetap mencetak §B yang rapi. Dua
> kandidatnya berbeda satu orde besarnya (§C tugas 2 vs tugas 3), dan §A G11
> wajib membenarkan pilihannya **sebelum** kode, dengan angka dari G10, bukan
> dengan selera.

```
Sesi G11 -- SCHED-5 / REACH-1: menutup utang G10, lalu lubang model TERBESAR
yang tersisa (tabrakan LENGAN-LENGAN antar gantry).

BACA DULU, berurutan:
1. docs/p1_g10_sched4.md  -- SELURUHNYA. B1 (tiga bug di berkas beku, dan
                             KENAPA pembekuan itu gugur), B3 (LIMA
                             pertentangan struktural -- terutama 4 dan 5, yang
                             dua-duanya ditemukan W2), B6 (vonis S1 SAH),
                             B7 (probe S2 GAGAL BERFUNGSI), B9 (palang
                             A3-K2: 7 dari 8), B11 (D9 G9 DIBALIK -- baca
                             pelajaran metodologisnya), B12 (batasan)
2. docs/p1_g9_sched3.md   -- B1, B2 (predikat + sertifikat, MASIH BERLAKU),
                             B4 (kenapa G9 gagal), B6
3. docs/p1_g8_sched2.md   -- B4, B6, B9, B10
4. docs/p1_g7_sched.md    -- A1, A2-K1 (bahasa "exact TERHADAP himpunan
                             kandidat"), B3, B5
5. reachability_gng/sched.py, sched_coll.py, sched_coupled.py, sched_heur.py,
   test/gate_sched_coupled.py, test/verify_sched_coupled.py,
   test/eval_sched_coupled.py

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE kecuali kalau tugas 3
menuntut pengukuran, dan kalau ya, itu dikunci di A lebih dulu.
CATATAN MESIN: G10 diukur pada mesin dengan load 42/16 karena node kamera ROS
sesi lain masih hidup. Cek `ps` dan bereskan SEBELUM mengukur waktu dinding,
atau nyatakan bebannya seperti p1_g10 B0.

=== YANG SUDAH TEGAK, JANGAN BANGUN ULANG ===
- Predikat BLOCK, lintasan A2.3, sertifikat A2.4: W0/W0b/W0c/W1 G9, dijalankan
  ULANG setelah perbaikan first_block dan hasilnya identik.
- Gerbang sadar-tunggu (test/gate_sched_coupled.py): 17 mutasi, 0 lolos, plus
  kontrol positif. PAKAI INI untuk setiap jadwal yang sesi mana pun laporkan.
- W2 (test/verify_sched_coupled.Brute) + P0-P4. Oracle-nya sendiri terbukti.
- solve_coupled2 dengan dive: W2 31 instance (15 mengikat) 0 kali solver > W2,
  W2b(i) 12/12, W3 Q1-Q5, W4' 12/12, regresi G9 2/2.
- ANGKA UTAMA: pada S1, mean Delta% ada di [0.0000, 1.0141] -> BUKAN MAHAL.
  37/40 terbukti Delta = 0.000 exact. JANGAN hitung ulang; pakai.
- K5.4: 3.2564% dari 2376^2 pasangan pose BLOCK; 886/2376 pose bebas total.

=== UTANG G10 YANG HARUS DIBAYAR ATAU DIBATALKAN SECARA EKSPLISIT ===
U1. W2b(ii) (no_dive) GAGAL 6/12. Dua jalan yang SAMA-SAHNYA, pilih satu di A
    SEBELUM kode:
      (a) perbaiki kelengkapan ekspansi sampai 12/12 tanpa dive; atau
      (b) UBAH KRITERIANYA, dengan argumen tertulis di A, bahwa W2 +
          W2b(i) adalah palang yang benar dan W2b(ii) mengukur konfigurasi
          yang tidak pernah dipakai. Kalau (b), argumennya ditulis SEBELUM
          melihat hasil apa pun, dan "ground truth" baru boleh dipakai
          sesudahnya.
    DILARANG: membiarkannya menggantung seperti p1_g9 membiarkan W2.
U2. S2 TIDAK BERFUNGSI sebagai probe (p1_g10 B7). gen_real_crowded memadatkan
    lin; (N1) menuntut rot. Bentuk yang BENAR sudah ada dan sudah terbukti
    mengikat: verify_sched_coupled.gen_small_crowded -- p0 di pose aman
    rot = 0, pose kerja di +-90 deg pada lin berdekatan, tugas HANYA layak di
    pose +-90 itu. Bangun `gen_real_rotcrowded` berbentuk begitu pada peta
    NYATA, lalu ukur ulang D10/D17/D19-S2. Ini MURAH dan ia membayar utang
    yang sudah dua sesi menghindari penilaian.
U3. Tiga instance S1 masih terkurung (n4_s7_mr0, n6_s7_mr0 rasio 1.187;
    n6_s6_mr1 rasio 1.031). action_hit = 35 menunjuk penyebabnya.
U4. W2 sendiri longgar: max_evade = 1 sementara solver memakai sampai 3.
    Naikkan dan ukur ulang 9 instance `solver < W2`.

=== TUGAS, BERURUTAN. JANGAN LOMPAT. ===
1. U1 dan U2. Keduanya kecil, dan U2 yang menentukan apakah ada rezim di mana
   koordinasi MAHAL sama sekali.
2. K3 ulang pada S2 yang benar. Pertanyaan binernya: apakah ADA rezim di grid
   ini di mana tabrakan struktur melewati ambang 5%? Kalau S2-yang-benar pun
   di bawah 5%, itu temuan yang LEBIH BESAR daripada kalau ia di atas: ia
   berarti tabrakan struktur gantry-gantry BUKAN mekanisme waktu yang dicari
   naskah, dan naskah harus berhenti menggantungkan klaimnya di sana.
3. LUBANG MODEL TERBESAR YANG TERSISA: tabrakan LENGAN-LENGAN antar gantry.
   Ini yang membuat SETIAP angka rugi tetap batas bawah (p1_g10 B12.2).
   Angkanya: ujung lengan terentang 1.4 m dari sumbu rotasi (p1_g2 12), TIGA
   KALI R_MAX = 0.455 m yang G9/G10 modelkan. p1_g2 10 mengukurnya TIDAK
   MENGIKAT (0.00% pasangan target hilang sampai clearance 0.15 m) -- tapi itu
   diukur pada konfigurasi MENJANGKAU, dengan proksi polyline yang MEREMEHKAN
   volume sapuan, dan TIDAK PERNAH pada lengan yang sedang dibawa melintas.
   Struktur A yang sama berlaku: predikat dulu, oracle dulu, solver terakhir.
   Dan predikat lengan JAUH lebih mahal dari predikat struktur -- ukur
   pemeriksanya SEBELUM menjalankan sapuan apa pun (p1_g8 B4, p1_g9 B3,
   p1_g10 B3 Pertentangan 1: TIGA kali sekarang, dan yang ketiga terjadi di
   dalam kode yang ditulis untuk menghindari yang kedua).

=== KUNCI KRITERIA SEBELUM KODE, ke docs/p1_g11_*.md A ===
1. Jalan mana untuk U1, (a) atau (b), dan kenapa.
2. Untuk tugas 3: apa DEFINISI predikat lengan-lengan, sebagai fungsi dari apa.
   Konfigurasi lengan TIDAK ada di model penjadwalan (p1_g7 A4.3: gerak lengan
   di dalam satu pose = 0), jadi predikat yang bergantung pada konfigurasi
   lengan menuntut penambahan model, dan penambahan itu dikunci di A atau ia
   akan menyelinap masuk lewat implementasi.
3. Berapa besar instance yang WAJIB lulus, sebagai angka, sebelum apa pun
   disebut tegak. Palang G10 (A3-K2) adalah contoh yang bisa dipakai ulang.
4. Apa yang dilaporkan kalau lagi-lagi tidak semuanya bisa dibuktikan. Aturan
   dua-sisi p1_g10 A3-K3 BEKERJA -- ia mengubah "TIDAK DAPAT DITENTUKAN" jadi
   vonis yang sah tanpa membuang satu instance pun. Pakai ulang bentuknya.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- Argumen yang ber-INDEKS tidak bisa ditukar. pair_distance(l1,r1,l2,r2)
  menaruh argumen pertama di y = +0.36. Menukarnya tanpa menegasikan rot
  memberi DUNIA CERMIN, dan galatnya 188.7 mm (p1_g10 B1.1).
- Kelayakan aksi HARUS diperiksa sampai ujung gerak terkomit LAWAN, bukan
  hanya sampai ujung jendelanya sendiri; kalau tidak, EKOR STATIS-nya tidak
  pernah diperiksa terhadap apa pun yang dikomit belakangan (Pertentangan 5,
  ditemukan oleh P2 -- gerbang yang diterapkan pada jadwal W2 SENDIRI).
- Konstruktor jadwal harus mengembalikan JADWAL YANG SAMA dengan yang ia
  periksa. _repair_ub G9 tidak (p1_g10 B1.2).
- "Kembangkan gantry yang tertinggal" adalah HEURISTIK di model terkopel,
  exact hanya di model takterkopel (Pertentangan 4).
- Pose menghindar yang AMAN bisa TAK TERCAPAI; rute ke pose tak-aman bisa
  justru bebas (Pertentangan 3).
- h[himpunan kosong] tidak ada di dp.w; merekonstruksinya wajib memulihkan 0
  (p1_g10 B1.3).
- Yang lambat adalah PEMERIKSA. Tiga sesi berturut-turut.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 19 meleset, 6 tepat.
  Prior DUNIA: LONGGAR (dan p1_g10 B11 mengembalikan prior ini setelah D9
  dibalik -- ia tidak punya contoh tandingan).
  Prior KODE SENDIRI: LEBIH LAMBAT, LEBIH RUMIT, LEBIH SALAH. 4 dari 4.
  Tulis dugaan tentang kode sendiri pada sisi PESIMISNYA supaya bisa dinilai.
- 🔴 DUGAAN YANG DINILAI MEMAKAI SOLVER YANG BELUM LULUS GERBANGNYA TIDAK
  DINILAI SAMA SEKALI. Tandai TERTUNDA, bukan MELESET. Ini aturan baru dan ia
  lahir dari kesalahan nyata (p1_g10 B11 catatan 3).
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 punya LIMA, plus satu dengan prompt G10 sendiri (A2.1). Itu bukan aib.
- Akhiri dengan prompt sesi berikutnya (G12).
```
