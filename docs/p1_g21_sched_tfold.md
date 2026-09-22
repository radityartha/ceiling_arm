# P1 / G21 — SCHED di bawah biaya setup TERUKUR (`t_fold` dari g19 B1.2)

> Sesi G21, 2026-09-22. **OFFLINE, nol gerak perangkat keras.**
> Lanjutan [p1_g20_hw.md §D](p1_g20_hw.md). Pertanyaan: semua evaluasi scheduler
> G7–G15 memakai `t_fold = 0.0`, yaitu biaya pindah = traverse saja; g19 §B1.2
> mengukur satu pindah 400 mm ≈ **148 s** dengan traverse hanya **21 s**. Klaim
> g7 §B5 (74–92 % makespan = gerak gantry), g8 §B6 (urutan tur 0.00 %) dan
> g8 §B7 (59.5 % di `n = 50`) belum pernah diuji pada biaya itu.
>
> §A ditulis dan **dikunci sebelum satu pun solve** sesi ini. §B bertentangan
> dengan §A → §B menang, pertentangannya ditulis (🔺), §A tidak diubah.

---

## A. Protokol — DIKUNCI 2026-09-22, SEBELUM satu pun solve

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| Model penjadwalan (stop, slot, mutex `r = 0.20`, MR = satu jendela, makespan K2) | g7 §A1, §A2-K2/K3 | apa adanya |
| `T_traverse = max(T_lin, T_rot)`, offset per sumbu yang bergerak | p1_state §5.6, g7 §A0 | apa adanya; `t_fold` **ditambahkan**, `T_traverse` **tidak** diubah |
| Peta `cap_g{1,2}_rail160.npz` | g7 §A0 | salinan ter-commit `reachability_gng/data/` (`/tmp` terhapus reboot; data/README "Restoring") |
| Ambang K1 gap (mean ≤ 5 %, max ≤ 10 %) | g8 §A3-K1 | dipakai **tanpa diubah** di tiap `t_fold` |
| Himpunan instance g8 K2 (Bagian I 140, Bagian II 50), ablasi D7 (40) | g8 §A3-K2, §B6 | seed/n/gantry/mr **identik** |
| `pose-tour` terkunci (bukan `+wide`) | g8 §B8 | heuristik yang dinilai |
| Tabrakan gantry–gantry **tidak** di model ini | g9 §B6 | semua angka rugi tetap **batas bawah**; subset `Δ` g9 tidak dikutip |

`sched.py` dan `sched_heur.py` **tidak diubah** sesi ini (solver yang sedang
diadu tidak boleh diedit di sesi yang sama, g8 §A6). Yang diubah hanya
**pemeriksa** (A5) dan CLI evaluasi (argumen `--t-fold`), dan tiap perubahan
dibuktikan tidak menggeser angka `t_fold = 0` (C0, A3).

### A1. 🔒 Nilai `t_fold` — DITURUNKAN dari g19 B1.2, rumus tertulis

Sumber: [g19_components.json](results/p1_g19/g19_components.json) /
[g19_summary.txt](results/p1_g19/g19_summary.txt), percobaan 1–10, Δ = 400 mm,
gantry 1, kedua lengan. Semua suku = **median per komponen** atas 10 percobaan
(median tahan terhadap dua pencilan tunggu-45 s `rail_to`, g19 B3 (3)).

| median g19 | nilai | arti |
|---|---|---|
| `retract_move` | **29.02 s** | gerak `return_rest`, = durasi yang KITA perintahkan (30 s) |
| `retract` (dinding) | **48.81 s** | panggil → kedua lengan < 0.5° dari REST (gerak + overhead 19.72) |
| `exec1`, `exec2` | **12.50**, **9.28 s** | eksekusi rencana arm_1 lalu arm_2 (berurutan) |
| `extend` (dinding) | **71.09 s** | rencana arm_1 → eksekusi arm_2 selesai (rencana + saringan ~51) |
| `gap_retract_to_rail` | **4.62 s** | sela skrip retract → rel |
| `gap_rail_to_task` | **2.28 s** (2.275) | sela skrip rel berhenti → rencana arm_1 |
| `traverse` | 21.11 s | **TIDAK** masuk `t_fold` — model memakai `T_traverse` (A1c) |

```
t_fold_0       = 0.0                                                    (lama, G7–G15)
t_fold_FISIK   = retract_move + exec1 + exec2
               = 29.02 + 12.50 + 9.28                       = 50.80 s
t_fold_DINDING = retract + extend + gap_retract_to_rail + gap_rail_to_task
               = 48.81 + 71.09 + 4.62 + 2.28                = 126.80 s
```

Silang-periksa DINDING (bukan dipakai, hanya konsistensi): `T_setup_wall`
median percobaan 2–9 **148.15** − traverse **21.11** = **127.04 s**; selisih
0.24 s = median-jumlah ≠ jumlah-median. Konsisten.

**(a) Mana yang boleh dikutip naskah, dan kenapa.**

| nilai | status di naskah | alasan |
|---|---|---|
| **0** | hanya sebagai "tanpa biaya lengan" — batas bawah | tidak ada lengan di model; ini yang G7/G8 kutip |
| **FISIK 50.80** | ✅ **nilai utama yang boleh dikutip** — "biaya pindah terukur di sel ini, satu Δ, tanpa overhead perangkat lunak" | hanya gerak fisik lengan. Masih terikat **setelan kita**: retract sendi-lurus 30 s yang kita perintah, parameterisasi waktu MoveIt kita, dan extend dua lengan **berurutan** (g19 S11). Jadi ia "fisika sel pada setelan ini", bukan batas bawah fisik |
| **DINDING 126.80** | ⚠️ **tidak** dikutip sebagai sifat sel; dilaporkan sebagai "sistem kita hari ini" | ~76 s darinya (rencana+saringan ~51, overhead `return_rest` ~20, sela ~7) adalah **artefak alat kita** — penyaringan antar-lengan per titik, saringan 3/3, sela skrip — bisa dihapus tanpa menyentuh perangkat keras. Mengutipnya sebagai biaya sel = mengutip bug kita sebagai fisika |

**(b) Yang TIDAK di `t_fold`, disebut supaya tidak dihitung dua kali:** traverse
bridge 21.11 s vs `T_lin(400)` 13.02 s — selisih ~8.1 s adalah artefak rantai
`go_to_absolute` bridge (g19 A1), **tetap di luar** model (model = `T_traverse`
terukur g3, bukan bridge). Jadi DINDING + `T_traverse(400)` = 139.8 s, bukan 148.

**(c) Asumsi yang dibawa nilai ini, tidak diuji sesi ini:** `t_fold` konstan —
tidak bergantung Δ (g19 hanya satu Δ, g19 §C) dan tidak bergantung berapa lengan
yang dipakai di perhentian berikutnya (g19 selalu dua).

🔒 **DILARANG** menambah, mengganti, atau membuang nilai `t_fold` sesudah hasil
pertama terbaca. Tiga nilai ini, dan hanya ini.

### A2. 🔒 Pembebanan `t_fold` — diperiksa di KODE, bukan diasumsikan

Dibaca dari `sched.py` (commit `fe1a285`, tidak berubah sejak G8):

```python
# sched.py traverse_time
t = np.maximum(tl, tr)                         # T_traverse, 0 bila pose sama
return np.where(t > 0, t + t_fold, 0.0) if t_fold else t
# solve_gantry: T = traverse_matrix(inst.poses[g][keep], inst.t_fold)   -- PER GANTRY
# solve_exact : makespan = max_g cost_g[mask_g]                          -- gantry independen
```

Jadi, **menurut kode**:

1. `t_fold` dibebankan **sekali per perubahan pose satu gantry**, termasuk
   keberangkatan pertama dari `p0`; **nol** bila pose tidak berubah
   (`T(p, p) = 0`).
2. Kedua lengan gantry itu diam selama `T_traverse + t_fold` — garis waktu per
   gantry tidak punya kerja di dalam traverse (g7 §A1). ✓ g19 A1 ("menghentikan
   KEDUA lengan").
3. Gantry lain **tidak** dibebani (tidak ada kopling waktu antar gantry, g7 §A4.2).
4. Heuristik membaca `t_fold` lewat `traverse_time` yang sama
   (`GantryView.trow`), dan `lb_analytic`/`lb_subset` juga.

**Ketaksamaan segitiga tetap berlaku** dengan `t_fold ≥ 0` (bukti, supaya bisa
dibantah): `T'(a,b) = T(a,b) + c·[a≠b]`. Untuk `a ≠ c`: `T'(a,c) = T(a,c) + c ≤
T(a,b) + T(b,c) + c ≤ T'(a,b) + T'(b,c)`, karena `a ≠ c` memaksa minimal satu
dari `a≠b`, `b≠c`, dan tiap suku tak-nol membawa `c`-nya sendiri. Jadi DP
(kunjungan ulang tak pernah menguntungkan) dan bukti `LB_subset` (g8 K3) tetap
sah **di atas kertas**. A5 mengujinya, bukan mempercayainya.

⚠️ **Tiga konvensi model yang MENGUNTUNGKAN arah tertentu di bawah `t_fold`
besar — dicatat sekarang, tidak diperbaiki:**

- **(K2) lengan "siap" di `p0` pada t = 0**: perhentian pertama di `p0` tidak
  membayar extend, keberangkatan pertama membayar retract **dan** extend penuh.
  Bias: **menguntungkan `p0`**. Dilaporkan: berapa optimum yang memakai `p0`.
- **gerak lengan di dalam perhentian = 0** (g7 §A4.4): bias **menguntungkan
  perhentian besar** — dan `t_fold` besar justru mendorong ke sana.
- **`t_fold` sama untuk perhentian satu-lengan** (A1c).

### A3. 🔒 Instance dan eksperimen

Peta: `ros2_ws/src/reachability_gng/data/cap_g{1,2}_rail160.npz`. Untuk tiap
`t_fold ∈ {0, 50.80, 126.80}`:

| # | Apa | Himpunan (identik g7/g8) | Isi |
|---|---|---|---|
| **G0** | gerbang exactness (A5) | — | **harus LULUS sebelum E1–E4 dijalankan** |
| **E1** | Bagian I | g8 K2: `n` 4/6/8 × seed 0–9, `n` 10 × seed 0–4; gantry (1,) dan (1,2); mr 0/1 = **140** | `solve_exact` + replay; 4 penjadwal g8 + replay |
| **E2** | Bagian II | `n` 12/16/20/30/50 × seed 0–4, (1,2), mr 0/1 = **50** | 4 penjadwal + gerbang g8 B4 + `LB_analitik` + `LB_subset` (3 × 8) |
| **E3** | ablasi tur (D7) | g8 §B6: `n` 8/12/20/50 × seed 0–4, (1,2), mr 0/1 = **40** | varian `full / nn-only / cover-order / no-refine / neither` |
| **E4** | mutex on/off | `n` 6/8 × seed 0–9, (1,) dan (1,2), mr 0/1 = **80 pasang** | `solve_exact` dengan / tanpa mutex (g7 §B3 — tambahan, lihat A4 catatan) |

**C0 — kontrol `t_fold = 0`:** E1 di `t_fold = 0` harus memberi makespan exact
**dan** makespan 4 penjadwal yang cocok dengan arsip
[g8_part1.json](results/p1_g8/g8_part1.json) sampai 1e-9 pada **140/140**. Gagal
→ peta/kode tidak identik dengan G8, dan tidak ada selisih yang boleh diatribusikan
ke `t_fold`. Sesi berhenti.

Batas exact `n ≤ 10` (g7 §B2); `validate_schedule()` hanya di Bagian I
(`n ≤ 10`), Bagian II lewat gerbang beranggaran g8 §B4 (g8 B4).

### A4. 🔒 Besaran yang dilaporkan — terlepas dari hasilnya

Per `t_fold`, per konfigurasi `(n, G, mr)`:

1. **makespan** exact (E1) dan `pose-tour`.
2. **jumlah pindah** = jumlah perubahan pose pada tiap gantry (termasuk
   keberangkatan dari `p0`), untuk gantry **kritis** dan total; plus berapa
   instance yang optimumnya punya perhentian di `p0`.
3. **pangsa** gantry kritis: `Σ T_traverse` / `pindah × t_fold` / `Σ dwell`
   (tiga suku; jumlahnya = waktu selesai gantry kritis, tidak ada waktu tunggu
   di model ini).
4. **gap** `pose-tour` vs exact — % (ambang K1 g8 dipakai apa adanya) **dan
   detik**, plus selisih jumlah pindah heuristik − exact. ⚠️ Gap % mengecil
   secara mekanis bila `t_fold` menaikkan penyebut dengan pindah yang sama;
   karena itu detik dan selisih-pindah wajib ikut.
5. **kontribusi urutan tur** (E3): penalti mean/max dan "lebih buruk pada k/40"
   per varian, seperti g8 §B6.
6. Bagian II: pangsa (traverse + `t_fold`) `pose-tour` di `n = 50` (klaim g8 §B7)
   dan rasio kurungan UB/LB — **bukan** gap.
7. E4: selisih makespan mutex on − off, jumlah pasang > 0.
8. K4: jumlah jadwal yang gagal replay, per penjadwal, per `t_fold`.

Klaim yang berubah ditulis sebagai **KOREKSI** di g7 §B5, g8 §B6, g8 §B7 (dan
g7 §B3 bila E4 mengubahnya) **dengan nilai `t_fold`-nya**, tidak diganti.

Catatan E4: tidak diminta prompt; ditambahkan **sebelum data** karena
`t_fold` besar mendorong perhentian lebih besar, dan perhentian besar adalah
satu-satunya tempat mutex bisa berbiaya (g8 §B7 paragraf akhir). Klaim
p1_state §6 🔴 (a) (mutex 0.000 s) adalah klaim naskah yang terpapar.

### A5. 🔒 Gerbang G0 — exactness di `t_fold > 0` (tugas 2)

g7 membuktikan exactness **hanya** di `t_fold = 0`. **Temuan kode sebelum
menjalankan apa pun:** `test/verify_sched_exact.py` **tidak mengenal `t_fold`
sama sekali** — `ref_traverse`, `ref_gantry_cost` (V2) dan `validate_schedule`
(V4, dan gerbang K4 g8) semuanya mengganti pose dengan `ref_traverse` saja.
Jadi di `t_fold > 0` V2 akan membandingkan DP ber-`t_fold` dengan brute-force
tanpa-`t_fold`, dan `validate_schedule` akan **menolak setiap jadwal yang
benar**. Menjalankan V0–V4 apa adanya "dengan `t_fold > 0`" tidak menguji
apa-apa.

Perubahan pemeriksa, dikunci sekarang:

- satu fungsi baru `ref_move(inst, g, a, b) = 0 bila a == b, selain itu
  ref_traverse(a, b) + inst.t_fold` — diketik dari teks A2 (butir 1), tidak
  mengimpor `traverse_time`. Dipakai oleh `ref_gantry_cost`, `validate_schedule`,
  dan `gate()` (`eval_sched_heur.py`). Di `t_fold = 0` ia menambah `+ 0.0`,
  jadi aritmetika identik bit per bit.
- V0–V4 dijalankan untuk **setiap** `t_fold ∈ {0, 50.80, 126.80}`:
  V0 `traverse_matrix(·, t_fold)` vs `ref_move`; V1 tidak bergantung `t_fold`
  (durasi perhentian) — dijalankan sekali; V2 96 instance acak per `t_fold`;
  V3 8 patologis dengan jawaban diperbarui (P3: `+ t_fold`; P6: `+ 3·t_fold`,
  tiga pindah; sisanya satu pose, tidak berubah); V4 33 jadwal per `t_fold`.
- **Dua patologis BARU, jawaban ditetapkan SEKARANG:**

| # | Konstruksi | Makespan DIHARAPKAN | Yang diisolasi |
|---|---|---|---|
| **P8** | 1 gantry, pose `p0 = (0, 0)`, `pA = (0.10, 0)` hanya tugas 0 (arm 0), `pB = (0.20, 0)` hanya tugas 1 (arm 0), `pC = (1.00, 0)` kedua tugas (tugas 0 arm 0, tugas 1 arm 1), non-zona | `min( 2·(0.29 + 100/v) + 2·c + 4.0 ,  (0.29 + 1000/v) + c + 2.0 )`, `v = 31.416` → `c = 0`: **10.9462**; `c = 50.80`: **84.9210**; `c = 126.80`: **160.9210**. Titik silang `c* = 23.175` | `t_fold` dibebankan **per pindah**: di `c` besar batching jauh menang atas dekat-dekat |
| **P9** | 2 gantry, tiap gantry punya 1 tugas yang hanya terjangkau di `(0.40, 0)`, `p0 = (0, 0)` | `0.29 + 400/v + c + 2.0` → **15.0224 / 65.8224 / 141.8224** (**maks**, bukan jumlah: tiap gantry membayar `c` sendiri) | `t_fold` per **gantry**, bukan global |

🔴 Kalau G0 gagal dan tidak bisa dijelaskan → sesi melaporkan KEGAGALAN dan
**tidak** menjalankan E1–E4 (g7 A2-K4). Toleransi 1e-9, tidak dilonggarkan.

### A6. 🔒 Papan skor §7.2 — D71–D78, DITULIS SEBELUM DATA

Skor masuk: **40 meleset, 30 tepat.** Prior (p1_state §9, g19/g20): kendala
kelayakan belum diukur → longgar; kode sendiri → lebih buruk (tapi integrasi
g19/g20 lebih murah dari dugaan); besaran kombinatorik yang mengasumsikan
independensi meleset (D65).

| # | Dugaan | Dasar |
|---|---|---|
| **D71** | G0 **LULUS** di ketiga `t_fold` tanpa mengubah `sched.py` — DP tidak bergantung `t_fold = 0` | bukti segitiga A2; `t_fold` hanya menambah konstanta ke tepi tak-nol |
| **D72** | C0: `t_fold = 0` mereproduksi arsip g8 **140/140** (exact dan `pose-tour`) | peta data/ = salinan /tmp (data/README), kode tak berubah sejak `1adc5bf` |
| **D73** | Pada E1 di **FISIK**, rata-rata jumlah pindah gantry kritis exact **≤ 60 %** dari nilainya di `t_fold = 0` | tiap pindah ekstra berbiaya ≥ 51 s vs rel penuh 52.8 s |
| **D74** | Pada E1 di **FISIK**, pangsa `pindah × t_fold` gantry kritis (mean) **≥ 50 %**, dan pangsa gerak gantry total (traverse + `t_fold`) **≥ 90 %** di setiap `n ≤ 10` | g7 §B5 74–92 % pada `t_fold = 0`; `t_fold` 50.8 ≈ 4× traverse tipikal |
| **D75** | `pose-tour` di **FISIK**: K1 tetap **GAGAL** (max gap > 10 %), dan instance max-nya memakai **lebih banyak** pindah daripada exact | prior "kode sendiri lebih buruk"; satu pindah ekstra ≈ 51 s ≈ 30 %+ makespan |
| **D76** | E3: urutan tur (`nn-only`, `cover-order`) tetap **+0.00 % pada 40/40** di FISIK **dan** DINDING | `t_fold` besar → lebih sedikit perhentian → makin sedikit yang bisa diurutkan |
| **D77** | E2: pangsa (traverse + `t_fold`) `pose-tour` di `n = 50`, FISIK, mean **≥ 80 %** (vs 59.5 % g8 §B7) | 2–4 perhentian × (traverse + 51 s) vs dwell ≤ 50 × 2 s / 2 gantry |
| **D78** | E4: mutex berbiaya **> 0 pada ≥ 1** dari 80 pasang di **DINDING** (berlawanan dengan prior "longgar" dan g7 §B3) | `t_fold` 127 s membuat pose-pelarian (g7 Q2) mahal, jadi perhentian besar dengan ≥ 2 tugas zona tak bisa dihindari |

D73–D75 yang menanggung klaim: kalau benar, pembingkaian naskah bergeser dari
"pose mana" ke "**berapa kali pindah**".

### A7. Berkas

| Berkas | Isi |
|---|---|
| `test/verify_sched_exact.py` | + `ref_move`, V0–V4 per `t_fold`, P8/P9 (A5) |
| `test/eval_sched_heur.py` | `gate()` memakai `ref_move`; `--t-fold` di semua subperintah |
| `docs/results/p1_g21/run_g21.py` | driver E1–E4 + dekomposisi (A4); menulis `g21_*.json` ke folder itu |
| `docs/p1_g21_sched_tfold.md` | dokumen ini |

`sched.py`, `sched_heur.py`, peta: **tidak disentuh**.

---

## B. Hasil terukur

> Semua angka di bawah keluar **sesudah** §A dikunci. Data:
> [results/p1_g21/](results/p1_g21/) — `g21_e{1..4}_tf{0,50.8,126.8}.json`,
> tabel lengkap [g21_tables.txt](results/p1_g21/g21_tables.txt)
> (`analyze_g21.py`), driver `run_g21.py`, `launch.sh`.

### B0. Cara menjalankan ulang

```bash
cd ros2_ws/src/reachability_gng
cp data/cap_g*_rail160.npz /tmp/              # V4 memakai path default
python3 test/verify_sched_exact.py            # G0: V1 + V0/V2/V3/V4 x 3 t_fold, ~25 s
cd ../../../docs/results/p1_g21
./launch.sh                                   # E1-E4 x 3 t_fold, 12 proses, ~2 jam dinding
python3 run_g21.py c0                         # kontrol C0 vs arsip g8
python3 analyze_g21.py                        # semua tabel B
python3 torque_offsets.py                     # tugas 4
```

### B1. G0 — exactness di `t_fold > 0`: **LULUS 13/13**, `sched.py` tidak diubah

| | Cakupan per `t_fold` | 0 | 50.80 | 126.80 |
|---|---|---|---|---|
| **V0** `traverse_matrix(·, t_fold)` vs `ref_move` | 1 600 pasangan | 0.0e+00 | 0.0e+00 | 0.0e+00 |
| **V1** bentuk tertutup vs enumerasi slot (tidak bergantung `t_fold`) | 2 700 kasus | 0 ketidakcocokan | — | — |
| **V2** DP vs brute-force | 96 instance | 0 | 0 | 0 |
| **V3** patologis (P1–P9, jawaban A5) | 10 instance | 10/10 | 10/10 | 10/10 |
| **V4** replay jadwal | 33 jadwal | 0 pelanggaran | 0 | 0 |

P8 (`t_fold` per pindah) dan P9 (per gantry) memberi persis nilai A5:
10.9462 / **84.9210** / **160.9210** dan 15.0224 / 65.8224 / 141.8224.

**Kontrol negatif — pemeriksa terbukti BISA gagal** (tanpa ini "LULUS" tidak
berarti apa-apa): solver mutan yang membuang `t_fold` dari `traverse_matrix`
ditangkap V2 (**36/96**), V3 (**4/10**: P3, P6, P8, P9) dan V4 (**16/33**).

➜ **D71 TEPAT.** DP dan segitiga-ketaksamaan **tidak** bergantung pada
`t_fold = 0`; bukti A2 bertahan diuji.

**C0 — kontrol `t_fold = 0` terhadap arsip g8: 700/700 makespan cocok ≤ 1e-9**
(140 exact + 4 × 140 penjadwal), peta `data/` md5 identik dengan salinan
`/tmp`. E2/E3 di `t_fold = 0` juga mereproduksi g8 §B6/§B7 angka per angka
(59.5 % di `n = 50`; `greedy` +26.3 %; ablasi +0.43 / +3.82 %, 15/40).
➜ **D72 TEPAT.** Selisih apa pun di bawah hanya berasal dari `t_fold`.

### B2. 🔺 E1 — Bagian I (140 instance): **yang berubah adalah KELAS masalahnya**

| | `t_fold = 0` | **FISIK 50.80** | DINDING 126.80 |
|---|---|---|---|
| makespan exact, mean | 42.6 s | **121.6 s** | 250.5 s |
| pindah gantry kritis, mean | 1.871 | **1.486** (79.4 %) | 1.486 |
| pindah total per instance (1 / 2 / 3 / 4 / 5) | 10 / 57 / 53 / 16 / 4 | **24 / 97 / 16 / 3 / 0** | identik FISIK |
| tugas per perhentian | 2.28 | 2.63 | 2.61 |
| optimum memakai `p0` sebagai perhentian | 37 / 140 | **69 / 140** | 71 / 140 |
| pangsa gantry kritis: traverse / `t_fold` / dwell | 85.8 / 0.0 / 14.2 % | **32.8 / 61.7 / 5.5 %** | 17.4 / 79.7 / 2.9 % |
| gerak gantry (traverse + `t_fold`), mean per `n` 4/6/8/10 | 89.0 / 86.4 / 84.3 / 81.6 % | **95.8 / 94.7 / 93.9 / 92.6 %** | 97.8 / 97.2 / 96.8 / 96.2 % |
| ↳ minimum per instance | 72.2 % | 87.4 % | 92.5 % |

(tabel per `(n, G, mr)`: [g21_tables.txt](results/p1_g21/g21_tables.txt); residu
`finish − (traverse + fold + dwell)` = 0 pada setiap gantry setiap instance —
model memang tanpa waktu tunggu, A4.3.)

**Temuan utama, tidak diduga §A: optimum di FISIK dan DINDING IDENTIK pada
140/140 instance** — jumlah pindah, `Σ traverse`, dan `Σ dwell` per gantry sama
persis, dan `makespan(126.8) − makespan(50.8) = 76.0 × pindah_kritis` tepat pada
140/140. (Jadwal boleh berbeda di antara optimum seri — `p0` 69 vs 71, tugas per
perhentian 2.63 vs 2.61 — tapi biayanya tidak.) Artinya sejak
`t_fold ≈ 51 s` optimum sudah **leksikografis: minimalkan jumlah pindah dulu,
baru traverse**, dan menaikkan `t_fold` lebih jauh tidak mengubah keputusan
apa pun, hanya harganya. **Kalimat yang boleh dikutip:** *pada biaya pindah
terukur, penjadwalan gantry adalah masalah minimum-jumlah-pindah; traverse hanya
pemecah seri.* Rentang `t_fold` di mana ini berlaku terukur **hanya di dua
titik** (50.8, 126.8); di antara 0 dan 50.8 tidak diukur (A1 melarang menambah
nilai sesudah data).

**D73 MELESET.** Dugaan: pindah FISIK ≤ 60 % dari `t_fold = 0`. Terukur
**79.4 %** (1.871 → 1.486). Sebabnya terbaca di baris "pindah total": di
`t_fold = 0` optimum **sudah** hemat pindah (g8 §B6: 2–4 perhentian) — pada
instance 2-gantry pindah kritis sudah **1.0–1.6**, lantainya 1. Yang bisa
dipotong hanya ekor 3–5 pindah (73 → 19 instance). Arah benar, besaran salah —
dan salahnya ke arah prior "longgar": kendalanya sudah hampir tidak mengikat.

**D74 TEPAT** (dua pembacaan, g20 D66 sebagai preseden): pangsa `t_fold`
**61.7 %** ≥ 50; gerak gantry mean per `n` **92.6–95.8 %** ≥ 90 di setiap `n`.
⚠️ Dibaca per instance, gerak gantry minimum **87.4 %** < 90 — dugaan menulis
"(mean)"; yang dihitung pembacaan mean.

### B3. E1 — gap `pose-tour`: mean MEMBAIK, ekor MEMBURUK dua kali lipat

| `t_fold` | mean % | median | max % | gap detik mean / max | persis optimal | K1 | `greedy` mean / max | `sequential` mean / max |
|---|---|---|---|---|---|---|---|---|
| 0 | 2.454 | 0.000 | 24.311 | 1.13 / 13.26 | 72/140 | **GAGAL** | 14.04 / 56.24 | 16.72 / 56.24 |
| **50.80** | **1.008** | 0.000 | **47.119** | 1.25 / **51.09** | **110/140** | **GAGAL** | 77.83 / 234.70 | 82.02 / 235.07 |
| 126.80 | 1.449 | 0.000 | **68.910** | 3.26 / **127.09** | 109/140 | **GAGAL** | 99.48 / 340.95 | 104.04 / 340.95 |

K4: **0** jadwal gagal replay di ketiga `t_fold` (exact dan 4 penjadwal,
2 100 jadwal). Ambang K1 tidak diubah.

**Ekornya satu kelas, dan kelasnya persis g8 §B3.** Instance > 10 %:
FISIK `n4_s7_g1_mr1` 47.1 %, `n6_s7_g1_mr1` 46.3 %; DINDING + `n6_s4_g1_mr1`
40.7 %. Semuanya 1 gantry, semuanya ber-MR, dan pada semuanya `pose-tour`
memakai **satu pindah lebih banyak** dari exact (kolom `dmv`; gap detik =
`t_fold` + beberapa detik). Ditelusuri pada `n4_s7_g1_mr1` di FISIK:

| | perhentian 1 | perhentian 2 | makespan |
|---|---|---|---|
| exact | **`p0` (0 mm, 0°)**: tugas 3 | 1550 mm, −85°: MR 0 + tugas 1, 2 | 108.43 s (1 pindah) |
| `pose-tour` | 750 mm, −65°: MR 0 + tugas 1, 3 | 1550 mm, −85°: tugas 2 | 159.52 s (2 pindah) |

Tahap 1 memilih pose handover dengan skor cakupan/sisipan: 750 mm menutup
**tiga** tugas — termasuk tugas 3 yang **gratis di `p0`** — dan tidak melihat
bahwa tugas 2 hanya terjangkau di ujung rel. Pose handover yang salah dipilih
(g8 §B3), tetapi harganya kini **satu `t_fold` penuh** (+51 s / +127 s), bukan
+9.55 s traverse. Konvensi K2 (A2 ⚠️) terbaca langsung: optimum menambang
perhentian `p0` yang gratis pada 69/140 instance, `pose-tour` tidak
merencanakannya.

➜ **D75 TEPAT** — K1 tetap GAGAL (max 47.1 %), dan instance max-nya memakai
2 pindah lawan 1. Tapi mekanisme D75 ("kode sendiri lebih buruk") hanya separuh
benar: `pose-tour` **lebih sering** persis optimal (72 → 110/140), dan
**tidak pernah** kalah dari `greedy`/`sequential` di FISIK/DINDING (di
`t_fold = 0`: 8 pasang). Heuristik makin baik di badan dan makin buruk di ekor —
karena biaya satu keputusan salah naik 4×–10×.

**Baseline miopik runtuh**: `greedy` 14 % → **78 %**, `sequential` 17 % →
**82 %** — keduanya membayar `t_fold` per perhentian yang tidak direncanakan.
g8 §B5 "yang membeli makespan adalah tidak mengunjungi pose yang jauh" menjadi
"**tidak berhenti terlalu sering**".

### B4. E3 — urutan tur tetap **+0.00 %** (D76 TEPAT)

| `t_fold` | `nn-only` | `cover-order` | `no-refine` | `neither` |
|---|---|---|---|---|
| 0 | +0.00 / +0.00 %, 0/40 | +0.00 / +0.00 %, 0/40 | +0.43 / +3.82 %, 15/40 | +0.43 / +3.82 %, 15/40 |
| 50.80 | **+0.00 / +0.00 %, 0/40** | **+0.00 / +0.00 %, 0/40** | +0.09 / +1.97 %, 3/40 | +0.09 / +1.97 %, 3/40 |
| 126.80 | **+0.00 / +0.00 %, 0/40** | **+0.00 / +0.00 %, 0/40** | +0.05 / +1.13 %, 3/40 | +0.05 / +1.13 %, 3/40 |

g8 §B6 bertahan di `t_fold` terukur, dan diperkuat: tahap 4 (perbaikan lokal)
juga menyumbang lebih sedikit (15/40 → 3/40), karena perhentian lebih sedikit
(`n = 50`: 3.85 → 2.40 perhentian/gantry).

### B5. E2 — Bagian II (`n` 12…50): klaim g8 §B7 **tidak bertahan**

| `n` | pangsa gerak gantry `pose-tour`, `t_fold` 0 | **FISIK** | DINDING | perhentian/gantry 0 / FISIK | UB/LB mean 0 / FISIK |
|---|---|---|---|---|---|
| 12 | 82.5 % | **91.6 %** | 95.5 % | 2.00 / 1.75 | 1.082 / 1.036 |
| 16 | 79.7 % | **90.0 %** | 94.5 % | 2.10 / 1.70 | 1.167 / 1.130 |
| 20 | 77.1 % | **88.8 %** | 93.7 % | 2.30 / 1.95 | 1.194 / 1.137 |
| 30 | 67.7 % | **84.4 %** | 90.9 % | 2.85 / 2.15 | 1.370 / 1.300 |
| 50 | **59.5 %** | **82.3 %** (75.8–89.0) | 89.0 % | 3.85 / 2.40 | 1.576 / 1.508 |

UB < LB: **0** pada 150 instance; K4 (gerbang beranggaran g8 §B4): 0 gagal.
`LB_subset` lengkap 50/50 di tiap `t_fold`. Jarak baseline ke `pose-tour`
melebar: `greedy` +26.3 % → **+248.6 %** (FISIK) → +354.5 % (DINDING).

➜ **D77 TEPAT** (82.3 % ≥ 80). Pangsa masih **turun** terhadap `n` (91.6 → 82.3 %),
jadi mekanisme g8 §B7 (dwell tumbuh linier, perhentian hampir tetap) tetap
bekerja — hanya dari lantai yang jauh lebih tinggi.

### B6. 🔺 E4 — mutex `r = 0.20` berbiaya untuk PERTAMA KALI (D78 TEPAT)

| `t_fold` | pasang | biaya > 0 | maks | mean |
|---|---|---|---|---|
| 0 | 80 | **0** | 0.000 s | 0.000 s |
| 50.80 | 80 | **1** | **2.000 s** | 0.025 s |
| 126.80 | 80 | **1** | **2.000 s** | 0.025 s |

Pasangnya `n8_s9_g1_mr1`, sama di FISIK dan DINDING. Ditelusuri:

| | perhentian (pose: tugas) | makespan |
|---|---|---|
| `t_fold = 0` | `p0`: 1 tugas · 900 mm −150°: 4 tugas · 1450 mm −80°: 3 tugas (2 pindah) | 56.74 s, mutex 0 |
| FISIK, tanpa mutex | `p0`: 2 · **1450 mm −80°: 6 tugas** (4 slot) | 109.24 s |
| FISIK, dengan mutex | sama, 6 tugas di **5 slot** | **111.24 s** (+2.0) |

Persis rantai sebab yang A6-D78 tulis: `t_fold` membuat pose-pelarian g7 Q2
berbiaya satu pindah (51 s), jadi optimum menumpuk 6 tugas di satu perhentian,
dan di sana mutex menyerialkan satu slot. **Ini dugaan kedua dalam proyek yang
tepat melawan prior "kendala belum diukur itu longgar"** — dan keduanya
(D9 G9, D78) adalah kendala **eksklusi ruang bersama**, persis kelas yang
p1_state §9 pisahkan dari kendala kelayakan. Besarnya kecil (1/80, 2.0 s pada
111 s = 1.8 %) — **batasi** klaim g7 §B3, jangan dibalik: mutex tetap bukan
penggerak makespan, tapi "0.000 s pada setiap pasang" adalah pernyataan
`t_fold = 0`.

### B7. 🔒 D71–D78 DINILAI

| # | Dugaan | Terukur | Vonis |
|---|---|---|---|
| **D71** | G0 lulus di 3 `t_fold`, `sched.py` tak diubah | 13/13; mutan tertangkap V2/V3/V4 | ✅ **TEPAT** |
| **D72** | C0 140/140 | **700/700** makespan ≤ 1e-9 | ✅ **TEPAT** |
| **D73** | pindah kritis FISIK ≤ 60 % dari `t_fold = 0` | **79.4 %** | ❌ **MELESET** |
| **D74** | pangsa `t_fold` ≥ 50 %, gerak gantry ≥ 90 % tiap `n` | 61.7 %; 92.6–95.8 % (mean per `n`) | ✅ **TEPAT** ⚠️ min per instance 87.4 % |
| **D75** | K1 tetap GAGAL, max-gap pakai lebih banyak pindah | max 47.1 %, 2 vs 1 pindah | ✅ **TEPAT** (mekanisme "kode lebih buruk" separuh — badan membaik, B3) |
| **D76** | urutan tur +0.00 % 40/40 di FISIK dan DINDING | 0/40, 0/40 | ✅ **TEPAT** |
| **D77** | pangsa `n = 50` FISIK ≥ 80 % | **82.3 %** | ✅ **TEPAT** |
| **D78** | mutex > 0 pada ≥ 1/80 di DINDING | **1/80**, +2.0 s | ✅ **TEPAT** |

➜ Papan skor §7.2: **40 meleset / 30 tepat → 41 meleset / 37 tepat.** Tujuh
dari delapan tepat — rekor sesi, dan polanya informatif: dugaan yang diturunkan
dari **jalur data kode** (D71, D72: bukti segitiga, md5, commit tak berubah) dan
dari **mekanisme yang sudah diukur** (D75–D78: g8 §B3/§B6/§B7, g7 Q2) tepat;
satu-satunya yang meleset (D73) adalah **besaran** yang ditaksir tanpa melihat
lantainya (pindah kritis 2-gantry sudah ≈ 1 di `t_fold = 0`).

### B8. Tugas 4 (sampingan) — offset torsi `joint_2` dari data yang ada

[torque_offsets.py](results/p1_g21/torque_offsets.py) →
[g21_torque_offsets.json](results/p1_g21/g21_torque_offsets.json). Satu definisi
untuk tiga sesi: prediksi = RNEA mentah rencana yang **dieksekusi** (baris
`torsi RNEA` terakhir sebelum `moveit MOVED`); terukur = puncak \|effort\|
`joint_2` lengan itu di jendela probe file `*_windows.txt` + 3 s (definisi
`g20/analyze.py`). **Silang-periksa: 40/40 rencana g20 = `g20_scored.json`
`j2_meas`, maks \|selisih\| 0.000 N·m.**

| lengan | n | min | median | mean | sd | p95 | **maks** | kemiringan offset/prediksi |
|---|---|---|---|---|---|---|---|---|
| arm_1 | 31 | +1.12 | +4.50 | +4.53 | 1.59 | +6.67 | **+7.06** | +0.49 |
| arm_2 | 30 | +2.53 | +4.40 | +4.37 | 1.11 | +6.13 | +6.43 | +0.39 |
| arm_3 | 10 | +1.79 | +4.96 | +4.92 | 1.81 | +6.97 | **+7.15** | +0.74 |
| arm_4 | 10 | +2.82 | +5.01 | +4.68 | 1.26 | +6.21 | +6.56 | +0.43 |
| **semua** | **81** | +1.12 | +4.49 | +4.54 | 1.41 | +6.69 | **+7.15** | +0.47 |

Offset **naik dengan prediksi** di keempat lengan (kemiringan +0.39…+0.74):
prediksi ≥ 6 N·m (18 rencana) → offset +4.84…**+7.15**, median +6.07. Jadi
offset konstan adalah model yang salah persis di daerah yang menentukan
(prediksi ≈ 7 → rating 14).

Apa yang **akan** dilakukan tiap aturan pada 81 rencana yang ada (in-sample,
bukan validasi):

| aturan | ditolak | lolos tapi terukur > 14 | lolos tapi > 13 | lolos tapi > 12 |
|---|---|---|---|---|
| sekarang: RNEA + 6.6 ≤ 14 | 0 | **1** (g20 #8 arm_3, 14.27) | 8 | 17 |
| RNEA + 7.2 ≤ 14 | 7 | 0 | 3 | 10 |
| RNEA + 6.6 ≤ 13 | 11 | 0 (2 di 13.03, 13.08) | 2 | 6 |
| **RNEA + 7.7 ≤ 14** | 13 | 0 | **0** | 4 |
| linier: RNEA + 2.71 + 0.467·RNEA + 2.53 ≤ 14 (⇔ RNEA ≤ 5.84) | 22 | 0 | 0 | 1 |

**Usulan (keputusan operator, kode penyaring TIDAK diubah):** `+7.7 ≤ 14` =
offset maksimum terukur (7.15) + pelampauan terbesar di atas prediksi
terkoreksi (0.55, g20 #8). Ia satu-satunya aturan konstan yang tidak meloloskan
satu pun rencana > 13 pada data ini, dengan biaya 13/81 rencana ditolak.
`+7.2` **tidak** diusulkan: itu "maksimum yang pernah terlihat" dengan margin
0.05, dan maksimumnya sudah naik sekali (7.06 → 7.15 dari 19 → 81 rencana).
Semua angka in-sample; aturan mana pun wajib dinilai pada rencana **baru**.

⚠️ Tidak mereproduksi g18 B2.4 persis: di sana arm_1 +1.19…+7.06, mean 4.48;
di sini (g18 saja) arm_1 +2.75…+7.06, mean 5.14 — jendela g18 B2.4 lebih
sempit (eksekusi saja). Maksimumnya sama.

### B9. Pertentangan §B lawan §A — G21

G19 tujuh, G20 sembilan.

| # | Pertentangan |
|---|---|
| **(1)** | A5: pemeriksa lama "akan **menolak setiap** jadwal yang benar" di `t_fold > 0`. Terukur: menolak **5/9** jadwal exact yang benar — tepatnya setiap jadwal yang **berpindah**; jadwal yang seluruhnya di `p0` lolos |
| **(2)** | A6-D73 berdasar "tiap pindah ekstra ≥ 51 s ⇒ pindah turun banyak"; lantai pindah (≈ 1 pada 2 gantry di `t_fold = 0`) tidak dipertimbangkan — pindah turun hanya ke 79.4 % |
| **(3)** | A1 mengunci tiga nilai sebagai tiga rezim; terukur FISIK dan DINDING memberi **optimum yang sama** (B2) — secara keputusan hanya **dua** rezim yang teramati |
| **(4)** | A3/E4 "tambahan" diperkirakan tidak berubah; ia memberi hasil yang mengubah klaim g7 §B3 (B6) |
| **(5)** | Tugas 4 (prompt): "g20 **43** rencana"; tahap 1b/2b (3 rencana) tidak punya file jendela → **40**; dan percobaan pertama saya memakai rentang cap waktu log probe — **salah** (baris akhir log tanpa cap waktu, jendela berakhir sebelum lengan terakhir bergerak; silang-periksa menangkap selisih **4.0 N·m**), diganti file jendela batch |
| **(6)** | Tugas 4: g18 B2.4 tidak tereproduksi persis dengan definisi seragam (B8 ⚠️) |
| **(7)** | A0: "`/tmp` terhapus reboot" — `data/README` menyuruh salin ke `/tmp`; V4 memakai path default `/tmp`, jadi pemulihan itu **wajib** untuk G0, bukan opsional |

**G21: TUJUH.**

---

## C. Koreksi yang ditulis ke dokumen lain (dengan nilai `t_fold`-nya)

| Dokumen | Klaim lama | Koreksi (ditambahkan, teks lama tidak dihapus) |
|---|---|---|
| g7 §B5 | "74–92 % makespan adalah gerak gantry" | pada `t_fold = 0` (per instance 72.2–96.0 %). FISIK: per instance **87.4–98.0 %** (mean per `n` 92.6–95.8 %), dan **61.7 %** dari makespan adalah `t_fold` saja — traverse 32.8 % |
| g7 §B3 | mutex 0.000 s pada setiap pasang | pada `t_fold = 0`. FISIK/DINDING: **1/80** pasang +2.0 s (B6) |
| g8 §B2 | `pose-tour` mean 2.45 %, max 24.31 % | pada `t_fold = 0`. FISIK: mean **1.01 %**, max **47.12 %** (+51.09 s = satu `t_fold`); K1 tetap GAGAL |
| g8 §B6 | urutan tur +0.00 % | **bertahan** di FISIK dan DINDING (0/40) |
| g8 §B7 | 59.5 % di `n = 50` | pada `t_fold = 0`. FISIK: **82.3 %** |

## D. Keadaan akhir, dan yang BELUM dikerjakan (Rule 12)

- Nol gerak perangkat keras. `sched.py`, `sched_heur.py`, peta: **tidak
  disentuh**. Diubah: `test/verify_sched_exact.py` (+`ref_move`, V0–V4 per
  `t_fold`, P8/P9), `test/eval_sched_heur.py` (`gate()` via `ref_move`,
  `--t-fold`). Keduanya dibuktikan netral di `t_fold = 0` (C0 700/700).
- **Tidak diukur, disengaja (A1):** `t_fold` di antara 0 dan 50.8 — jadi titik
  di mana optimum berubah rezim **tidak diketahui**; `t_fold` bergantung Δ;
  `t_fold` untuk perhentian satu-lengan; heuristik yang sadar-`t_fold`.
- **Tidak diperbaiki, disengaja:** `pose-tour` tahap 1 tidak merencanakan
  perhentian gratis di `p0` (B3) — perbaikan heuristik sesudah melihat tabel gap
  = varian post-hoc (g8 §B8), butuh set held-out.
- Tabrakan gantry–gantry tetap di luar model ini: **semua angka rugi tetap
  batas bawah** (g9 §B6). `t_fold` besar → lebih sedikit pindah → mungkin lebih
  sedikit konflik g9, **tidak diukur**.
- Penyaring torsi: usulan B8 menunggu keputusan operator **sebelum** sesi
  perangkat keras berikutnya.
- **Belum di-commit** (commit hanya atas permintaan operator).

## E. Prompt sesi berikutnya — G22-HW (salin ke chat BARU)

**Kenapa ini berikutnya.** G21 memberi prediksi model (`t_fold` FISIK 50.80 s,
optimum = minimum jumlah pindah), tapi belum pernah ada **jadwal keluaran
scheduler** yang dijalankan di sel nyata. Klaim inti naskah butuh satu angka:
makespan terukur lawan prediksi. Operator 2026-09-22: batas rating 14 N·m
dipertahankan; **offset penyaring (+6.6 vs +7.7) belum diputuskan** → A8 no. 1.

**Rekomendasi: Opus 5, effort TINGGI** — gantry bergerak dengan empat lengan
dan eksklusi antar-gantry di rel lain untuk pertama kali; kesalahan di sana diam
sampai perangkat keras menabrak.

```
Sesi G22-HW -- jalankan SATU jadwal keluaran scheduler end-to-end di sel NYATA
(empat lengan, dua gantry, gantry BERGERAK) dan bandingkan makespan terukur vs
prediksi model. Repo ceiling_arm, branch feat/rgbd-topo-deploy.

BACA PENUH sebelum menulis kode atau menyentuh hardware:
1. CLAUDE.md (Working Rules; Rule 6 dikecualikan untuk sesi protokol P1, g16 A10).
2. docs/p1_g21_sched_tfold.md -- A1 (t_fold FISIK 50.80 / DINDING 126.80), B2
   (optimum = min jumlah pindah), B3 (ekor pose-tour), B8 (offset torsi), D.
3. docs/p1_g20_hw.md -- B2, B4, C, C1 (error Kortex pasca-sesi: operator cek LED
   keempat lengan SEBELUM bring-up).
4. docs/p1_g19_hw.md -- A1 (bridge terikat T_cmd, bukan T_lin), A5 S12-S17,
   B1.2 (komponen setup), batch.sh/rail_to.py (hanya gantry 1, rel 0.550/0.950).
5. docs/p1_state.md 5.6, 5.7, 7, 8c; docs/p1_g9_sched3.md B6 (tabrakan
   gantry-gantry TIDAK di model scheduler).
6. reachability_gng/sched.py (solve_exact, gen_real, t_fold), sched_heur.py.

=== TUGAS ===
1. Tulis docs/p1_g22_hw.md A DULU, KUNCI sebelum satu pun gerak:
   a. Instance: SATU instance kecil (n <= 6, 2 gantry) yang target-targetnya lolos
      saringan 3/3 plan-only di pose gantry yang dipilih solver. Solve EXACT dengan
      t_fold FISIK dan DINDING; tulis jadwal + makespan prediksi SEBELUM gerak.
      Pose gantry dibatasi ke yang aman: rel <= 1600 mm, rotasi 0 kecuali
      diputuskan lain, jarak antar-gantry dicek predikat BLOCK g9.
   b. Besaran: makespan terukur (jam perekam) vs prediksi; per komponen
      (retract / traverse / extend / dwell / sela) vs model; jumlah pindah.
   c. Model prediksi mana yang dinilai: T_traverse = T_lin (model) DAN T_cmd
      bridge -- keduanya ditulis, karena bridge ~1.6x T_lin (g19).
   d. Palang keselamatan: S1-S23 tetap + rail_to untuk gantry 2 (baru, uji
      plan-only dulu) + origin gantry 2 dikonfirmasi operator.
   e. Dugaan D79+ dikunci SEBELUM data (skor masuk: 41 meleset, 37 tepat).
2. A8 keputusan OPERATOR sebelum data: (1) offset penyaring joint_2: tetap
   RNEA + 6.6 <= 14, atau RNEA + 7.7 <= 14 (usulan g21 B8); batas 14 tetap.
   (2) LED keempat lengan. (3) origin gantry 2. (4) instance yang dipilih.
3. Jalankan per tahap dengan izin operator (plan-only -> satu pindah -> jadwal
   penuh). Nilai apa adanya.
4. Perbarui p1_state.md 6 + 8c + tally 9.

=== ATURAN ===
- 7.1 ground truth dulu; 7.2 UKUR, dugaan dikunci sebelum data.
- DILARANG memilih instance atau t_fold sesudah melihat hasil gerak.
- Kalau B bertentangan dengan A, B menang dan DITULIS. G20 sembilan, G21 tujuh.
- Tanya sebelum setiap gerak perangkat keras. Checkpoint tiap tahap (Rule 10);
  Rule 12.
```

**Alternatif offline (kalau perangkat keras belum siap):** G22-OFF = heuristik
sadar-`t_fold` (rencanakan perhentian gratis di `p0`, minimalkan jumlah pindah
lebih dulu), dinilai pada set **held-out** seed 10–14 karena dirancang sesudah
melihat tabel B3 (preseden g8 §B8). Effort sedang: exact sudah tegak dan
menangkap kesalahan.
