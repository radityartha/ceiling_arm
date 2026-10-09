# P1 / G38-OFF — instans sulit: 3 tugas di TENGAH (zona bersama) + 3 dekat DINDING, rotasi R35 (OFFLINE, nol gerak)

> Sesi 2026-10-09 (sesudah G36b, sesi yang sama). Permintaan operator: titik sasaran instans G22–G36 "terlalu mudah"
> untuk menguji metode → sasaran di area tengah atau dekat dinding. Keputusan operator (ditanya): tengah = pita
> **|y| < 0.15 m** di antara kedua gantry; dinding = **0.10–0.20 m** dari peta D455; **3 tengah + 3 dinding** per instans;
> offline sesi ini. Setelah temuan A0 (tengah mustahil di rotasi 0): **pakai rotasi R35** (|rot| ≤ 35°).
> G37 (penyaring Fix-Start-State, cache geometri, K0 terukur) tetap prasyarat HW; G38 tidak menyentuh HW.
> **§A ditulis dan DIKUNCI SEBELUM pengukuran** (salinan `docs/results/p1_g38/g38_A_locked.md` + sha256). §B menang atas §A.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

Dari cache yang ADA (tanpa komputasi baru), node peta kapabilitas `cap_g{1,2}_rail160` (3 132 node, x 0–2.0, y ±0.6,
z 1.0–1.4; gantry di y = ±0.36):
- Target seed 36 (G32–G36b) semuanya di |y| 0.32–0.46 — di bawah gantry sendiri.
- **Rotasi 0:** 174 node |y| < 0.15 di cache oracle‴ G24 → **0 lolos**, slack terbaik −1.04 N·m, pengikat `joint_2`.
  Node aman dari KEDUA gantry: **0 / 780**. Sel tidak punya zona bersama di rotasi 0 — sebab instans lama "mudah".
- **Rotasi ≠ 0** (cache G29, rot ±30/±60/±90): tuple tengah 472/1 407 ok3; 43/51 node tengah ok di ≥ 1 tuple,
  **39/51 dari kedua gantry**; rot minimum yang membuat ok = 30°.
- Dinding (jarak ke voxel peta g34n 0.10–0.20 m): 180 node (L1); 40 di cache G24, 18 ok. Rintangan dekat kotak tugas:
  rak/dinding samping y ≈ −0.9…−0.6 (x 0.8–1.8) dan rak ujung x ≈ 2.0–2.2.
- Oracle‴ tidak memeriksa lingkungan sama sekali (hanya torsi ∀-cabang, ∀-roll, tilt).

### A1. 🔒 Kolam target

- **TENGAH** = node dengan |y| < 0.15 dan z ≤ 1.32 (oracle⁗ G27: lapis z = 1.40 dibuang).
- **DINDING** = node dengan jarak titik ke voxel peta g34n ∈ [0.10, 0.20) m, |y| ≥ 0.15, z ≤ 1.32 (saling lepas dengan TENGAH).

### A2. 🔒 Predikat tuple dan penarikan

Tuple = (node, g, rel li, rot ri ∈ R35, slot s). **Layak** ⟺ oracle‴ torsi ok (rot 0: cache G24 `ok`; rot ≠ 0: `ok3`
G29/G31 atau dihitung) **DAN lingkungan-∃**: ada ≥ 1 solusi IK oracle (roll mana pun) yang titik akhirnya, dengan lengan
lain REST, gantry lain di 0/0, berjarak ≥ 50 mm (`EnvChecker.check`, geometri lengan tugas saja) dari peta.
Penarikan per seed s ∈ {0…19}: `rng = default_rng(s)`; 3 dari TENGAH lalu 3 dari DINDING, seragam, tanpa duplikat.
Kandidat diterima ⟺ ada tuple layak di antara **40 tuple L1-benar pertama** urut (|rel − x_node|, −|rot|, g, s)
(batas biaya; penolakan = "tidak ditemukan dalam 40", dihitung dan dilaporkan). Tolak > 10 000 per seed → seed gagal.

### A3. 🔒 Penjadwalan

`g31_sched` apa adanya (P1′, rot_cmd 0.9235, lazy-exact), set pose REF.lin × **R35**, dengan dua perubahan di skrip G38
(bukan di `g31_sched`): (1) **p0 g1 = (1.50, 0°)** (PARKIR, B6.6 G36), g2 = (0, 0°); (2) verifikasi lazy mencakup tuple
rot 0 yang belum ada di cache G24 dan **lingkungan-∃** setiap tuple terpakai — gagal → nol, selesaikan ulang sampai
semua tuple terpakai terverifikasi (relaksasi L1 ⊇ layak → optimum terverifikasi = optimum sejati). Kontrol: varian
R10 dengan predikat yang sama pada **5 seed pertama yang tertarik** (biaya; diharapkan (i) GAGAL untuk tugas tengah).

### A4. 🔒 Kriteria (sama dengan G22–G31) + laporan

(i) terselesaikan; (ii) kedua gantry ≥ 1 pindah; (iii) `sched_coll.schedule_conflict` = None. Dilaporkan per seed: tolak
per kolam, gantry + rot yang melayani tiap tugas tengah, makespan, konflik. Gerbang HW (di luar lingkup): `pose_to_g`
S23″ menolak |rot| > 10°, rotasi HW baru diverifikasi ±10° (G30) — jadwal R35 butuh gerbang rotasi baru sebelum HW.

### A5. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D280 | ≥ 15 / 20 seed selesai ditarik (tidak ada seed melewati 10 000 tolak). |
| D281 | Laju tolak kandidat TENGAH **10–40 %**. |
| D282 | Laju tolak kandidat DINDING **40–75 %** (torsi 55 % gagal di cache + lingkungan). |
| D283 | (i) R35 terselesaikan pada **semua** seed yang tertarik; kontrol R10 (i) **gagal** pada 5/5 seed kontrol. |
| D284 | Setiap tugas tengah di jadwal optimum dilayani pada **\|rot\| ≥ 30°**. |
| D285 | (iii) konflik terjadi pada **≥ 30 %** seed (zona bersama). |
| D286 | Lolos (i)–(iii): **≥ 5 / 20** seed. |

sha256 `g38_A_locked.md` = `16254db0…5fb9a0` (dikunci 2026-10-09 15:10, sebelum komputasi baru).

## B. Hasil terukur

### B0. Pemeriksaan + penyimpangan dari §A (DITULIS)

- **K** (`g38_make.py k`): field torsi `_job` G38 **bit-identik** dengan cache G24 (rot 0) dan G29 (rot ≠ 0), 6/6; FK model
  EnvChecker ke target ≤ 0.12 mm → model oracle dan model lingkungan sepakat.
- **B1 — urutan kandidat §A2 salah, diganti (B menang atas A).** §A2 mengurutkan 40 tuple per kandidat menurut
  (|rel − x|, −|rot|). Pangkal lengan duduk **±0.4 m di x dari titik rel** (pelat dudukan, `irm_sweep.base_pose`), dan
  rotasi θ menggeser pangkal ke tengah 0.4·sin θ (≈ 0.23 m di 35°). Urutan terkunci menghabiskan 40 tuple pada pangkal
  0.4 m dari target: **2 lolos dari 693 tuple**, seed 0 belum selesai sesudah 9 min ([log](results/p1_g38/g38_make_order1_killed.log)).
  Dihentikan; urutan diganti ke **jarak xy pangkal–target** (urutan `tuples_for` G29). Cache tuple (693) dipakai lagi:
  hasil per tuple tidak bergantung urutan. Predikat "∃ dalam 40" kini bermakna "∃ di 40 pangkal terdekat".
- **B2 — urutan kerja (bukan hasil):** lazy R35 seed 0 butuh ≥ 24 iterasi / ≥ 19 min (relaksasi L1 lemah di rot ≠ 0).
  `run` diubah: tarik semua seed → R35 semua seed → kontrol R10 (5 seed pertama) di akhir; hasil ditulis per seed.
  Perkiraan total ~10 jam (`setsid nohup`, cache bisa dilanjutkan).

### B1. Penarikan (sementara, diisi sesudah run selesai)

> ⏳ Run berjalan sejak 2026-10-09 15:43 — log `results/p1_g38/g38_make.log`, keluaran `results/p1_g38/g38_candidates.json`.
