## A′. 🔒 Oracle‴ — ditulis 2026-09-22 SESUDAH B1, SEBELUM oracle‴ dihitung

> Koreksi pasca-data. Yang sudah dilihat: B1 (matriks V, kontrol, diagnosis 11
> miss PC2). Semua yang tidak disebut di sini = §A apa adanya (grid **5°**,
> benih, toleransi IK, margin A2′, A3 lazy, A4, A5, A6–A8).

**A′1. Kemiringan.** Tiap solusi sapuan `s` pada roll ψ diperluas ke **8
kemiringan** target `R(ψ)·Rx(a)·Ry(b)`, `(a, b) ∈ {(±2°, 0), (0, ±2°), (±2°, ±2°)}`
(sumbu alat; = kotak `absolute_x/y_axis_tolerance` 2° probe, termasuk sudut),
Newton 6-D dari `s`, maks 50 iterasi, toleransi A1. Torsi diambil di `s` **dan**
di setiap kemiringan yang konvergen; yang tidak konvergen dihitung dan dilaporkan.
Torsi linear dalam kemiringan kecil → titik sudut/tepi kotak memuat ekstremnya
(asumsi, ditulis).

**A′2. JENUH atas amplop torsi, bukan hitungan solusi.** Ronde r (≥ 2) jenuh ⇔
tidak ada roll grid baru yang mendapat solusi **dan** tidak ada `max |τ_j|`
(6 sendi, termasuk kemiringan) yang naik > **0.01 N·m**. Ronde, benih, maks 4 ronde,
TAK-JENUH = tolak: seperti A1.

**A′3. Kontrol.** NC1, NC2, NC5, NC6, NC7, PC1, PC3 apa adanya (A2). **PC2′**:
`q_final` diproyeksikan (Newton 6-D, dari `q_final`) ke roll grid terdekat tanpa
kemiringan; jarak ke solusi sapuan terdekat pada roll itu < **0.15 rad** (ambang
A2 **tidak** diubah; hanya pengukurannya), harus ≥ 95 % (77/81). ⚠️ 11 dari 81 sudah
terlihat (10 < 0.032, 1 = 2.15) — PC2′ **tidak buta** pada 11 itu.
**Tidak ada A2-fix lagi:** satu kontrol gagal → oracle‴ cacat, G24 berhenti,
prompt G25.

**A′4. Dugaan D102–D106** (D95–D101 tetap, dinilai pada oracle‴ bila datanya ada):

| # | Dugaan | Dasar |
|---|---|---|
| **D102** | NC7 **90/90** | kedua lolos-salah 5.64 vs 5.65 = efek miring (B1-1) |
| **D103** | PC2′ **≥ 78/81** | 10/11 miss sudah < 0.032; sisanya dmin < 0.15 tanpa proyeksi |
| **D104** | TAK-JENUH di V **≤ 1** | kontinum pergelangan tidak menggeser amplop torsi |
| **D105** | V: terima **≤ 2**, TORQUE-UNSAFE **0** | kemiringan hanya bisa menaikkan maks → lebih ketat dari oracle″ (1) — kecuali TAK-JENUH yang kini jenuh |
| **D106** | tarikan: laju terima kandidat **≥ 10 %** (rerata `rejects` per seed ≤ 54) | recall V 1/14 adalah tugas TEPI (G22 memilih tepi); node seragam lebih dalam |

