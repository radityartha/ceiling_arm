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
