## A. Protokol — DIKUNCI

### A0. Pengungkapan (sebelum §A)

- G55: solver P1″ TIDAK memodelkan traverse eksklusif; hanya g54_sim. Seleksi g54_sim memilih |rot| ≤ 30 di semua seed (D472 ✗).
- Sebelum §A ditulis SUDAH dijalankan (alat + kontrol, bukan hasil fase):
  1. **K-SIM** `g60_sim.py k`: `fastsim` (g54_sim tanpa penyaringan reservasi) vs g54_sim pada SEMUA 19 jadwal yang pernah
     disimulasikan G54/G55 → **19/19 identik** (semua 0 blok reservasi). [log](results/p1_g60/g60_ksim.log)
  2. **K-DP** `g60_make.py k` (instans terverifikasi G55, tanpa menghitung tuple): (a) 747/747 rantai min-Tr dijalankan
     sendiri = Tr + W; (b) 0 pelanggaran: DP min-Tr ≤ Tr jadwal G55 (62 gantry), LB ≤ sim g54 di semua 31 baris g55_eval
     (sering LB = sim). [log](results/p1_g60/g60_kdp.log)
  3. **Coba search pada instans TERVERIFIKASI saja** (s7 s10 s0, R90 + R35): optimum = jadwal G55 terpilih (283.62 / 278.50 /
     148.16, s10 g2 (1.00, −20) alih-alih (0.85, −20), makespan sama) dengan **LB*ver = makespan (celah 0)**. Relaksasi
     (tuple belum diketahui = layak): LB*rel s7 265.07 (R90) / 266.63 (R35), s10 275.04 (R90). Dugaan di bawah TIDAK
     menebak angka-angka ini.

### A1. 🔒 Pendekatan dan alasannya

**Model** = runner `run_g54` (`g60_sim.fastsim` ≡ g54_sim dengan 0 blok): traverse gantry g mulai hanya bila gantry lain tidak
menjalankan apa pun dan menahan gantry lain selama berjalan; tugas/retract tidak mulai selama gantry lain traverse; traverse yang
menunggu didahulukan. Biaya P1″ (G53 B3): tugas a_t 29.04, retract a_r 45.34 (2.7 bila lengan sudah REST), traverse
l_bar + g·(c_rovh + max(r·T_lin, ROT_CMD·T_rot)).

**Batas bawah terbukti** (untuk SETIAP pasangan urutan event per gantry): semua event gantry g sampai tugas terakhirnya dan
semua traverse gantry lain o saling lepas dalam waktu dan selesai sebelum makespan →
`makespan ≥ W_g + Tr_g + Tr_o` → **LB = Tr_1 + Tr_2 + max(W_1, W_2)**. W hanya bergantung pada (#tugas, #traverse m, stop
pertama di p0 f0); Tr diminimalkan per (himpunan tugas, m, f0) oleh DP = rekursi `solve_gantry_f` + satu indeks m (larik
kelayakan yang sama: reach ok49, larangan (s42), EdgeMatrix (b), tanpa dua stop berurutan di pose sama).

**Pencarian:** semua alokasi (A₁, A₂) × (m₁, f0₁) × (m₂, f0₂) dengan (ii) (m ≥ 1 kedua gantry) → LB + kandidat = rantai
min-Tr kunci itu; dievaluasi `fastsim` berurutan LB naik sampai LB ≥ incumbent; (iii) `schedule_conflict` pada linimasa fastsim
(awal stop = akhir traversenya) — kandidat dengan konflik dilewati. **Sertifikat** LB* = min LB atas SEMUA kunci (berlaku
untuk semua jadwal, bukan hanya kandidat). Pencarian ini EKSAK hanya sejauh rantai min-Tr per kunci adalah yang terbaik untuk
fastsim; celah makespan/LB* dilaporkan apa adanya (= 0 → optimum terbukti).

**Lazy (G46):** jadwal = instans TERVERIFIKASI (+ loop (s42) seperti UB G46). LB* = instans RELAKSASI. Setiap kunci relaksasi
dengan LB < makespan_ver/(1 + 0.03) yang rantainya memakai tuple/stop belum terverifikasi → diverifikasi (+ tetangga G31),
≤ 300 kunci per putaran; berhenti bila tidak ada lagi. Celah akhir = makespan / LB*rel − 1.

**Mengapa ini (paling sederhana yang benar):** penalti saja (mis. LB sebagai objektif) tidak menjamin urutan yang baik untuk
gerbang runner; solver eksak atas penjadwalan waktu-kontinu dua mesin dengan eksklusi parsial butuh B&B atas pasangan urutan
(ruang pose × urutan ≈ 10⁶+ per gantry). DP min-Tr per (A, m, f0) memakai kembali solver G42–G55 tanpa mengubah kelayakan,
LB-nya terbukti, dan evaluasi kandidat = model runner itu sendiri. Kontrol (A2.1) memastikan model = g54_sim.

### A2. 🔒 Uji

1. **K-SIM** (A0.1, sudah) + **K-ULANG:** `fastsim` jadwal tersimpan (lewat `g31_screen.events`) = nilai pencarian (per seed).
2. **Job** `g60_make.py run R90 / R35` seed 0 3 7 8 10 11 12 18 (R35: 0 3 7 8 10 18, seperti G55; s11/s12 juga dicoba R35 —
   G55 tidak). Setiap jadwal: g54_sim PENUH (dengan penyaringan reservasi) = makespan G60 (identik bila 0 blok).
3. Pertanyaan utama: seed mana jadwal R90 dengan |rot| > 35 MENANG atas R35 (makespan eksklusif lebih kecil). Bila ada:
   (iv′) + (iv″) (pola `g55_iv.py`), DRY-CLI mock k = 3 (tol 0.05°), smoke paralel mock `run_g54` → pasangan HW R90 vs R35.
   Bila tidak ada: ditulis sebagai hasil.
4. Jadwal G60 yang lebih baik dari G55 terpilih (> 1 %) → jalur yang sama (iv′)(iv″) → DRY-CLI → smoke, sebagai kandidat
   pengganti R90sel/R35sel.

### A3. 🔒 Gerbang G60

K-SIM 19/19 + K-ULANG semua; tabel 8 seed × {R90, R35}: makespan G60, LB*rel, celah, |rot| maks, vs G55 terpilih; jawaban
pertanyaan A2.3; jadwal baru (bila ada) lolos (iv′), (iv″), DRY-CLI 3/3, smoke 6/6.

### A4. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D520 | Makespan G60 ≤ g54_sim jadwal G55 terpilih di 8/8 seed (R90). |
| D521 | Celah akhir (makespan / LB*rel − 1) ≤ 3 % di ≥ 6/8 seed R90. |
| D522 | TIDAK ada seed di mana jadwal R90 dengan \|rot\| > 35 menang atas R35 lebih dari 1 %. |
| D523 | G60 lebih baik dari G55 terpilih > 1 % di ≥ 2/8 seed. |
| D524 | g54_sim penuh = makespan G60 (0 blok reservasi) di semua jadwal G60. |
| D525 | Jadwal baru yang diuji di mock: (iv′) ∧ (iv″) 100 %, DRY-CLI 3/3, smoke 6/6. |

