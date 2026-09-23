# P1 / G24 — oracle L2 dengan SAPUAN ROLL, tugas ditarik dari himpunan layak-oracle

Lanjutan [p1_g23_oracle_hw.md](p1_g23_oracle_hw.md): oracle′ (IK approach vertikal
5-D dari 8 benih + torsi statis ∀-cabang) benar pada NO-PLAN 23/23 dan
TORQUE-UNSAFE 11/12, tetapi **roll bebas ⇒ solusi IK_V adalah kurva 1-D**, dan
8 benih melewatkan titik tak aman pada ≥ 30 % tuple yang diterima (g23 B3) —
kedua penolakan (iv) G23 persis di sana. Sesi ini mengganti **hanya** enumerasi
solusi (sapuan roll eksplisit) dan, atas keputusan operator, **cara tugas
ditarik** (A5). Model penjadwalan, `sched.py`, peta, `t_fold`, margin A2′
**tidak** diubah.

**Keadaan sel hari ini (operator, 2026-09-22 ±17:40):** lengan **dimatikan**,
operator **tidak di lokasi**, PC hidup. Maka sesi ini dibagi:

- **G24 (sekarang, offline, nol perangkat keras):** A dikunci → oracle″ →
  validasi → instance (i)–(iii).
- **G24b (operator di lokasi):** tahap 0 → saringan (iv) → (kalau lolos) tahap
  1–3 g22. Dugaan tentang (iv) dikunci **sekarang** (A7), sebelum (iv).

---

## A. Protokol — ditulis 2026-09-22 17:47, SEBELUM oracle″ dihitung, SEBELUM gerak apa pun

> 🔒 Seluruh §A dikunci sebelum satu pun nilai oracle″, validasi, atau instance
> dihitung. Yang dihitung sesudahnya masuk §B; kalau §B bertentangan dengan §A,
> §B menang dan pertentangannya ditulis (§B-akhir). Pengukuran **waktu** solver
> pada pose yang bukan anggota set validasi/kontrol (untuk memilih lingkup
> hitung, A3) bukan data oracle dan diizinkan sesudah kunci; ia tidak boleh
> mengubah grid, benih, toleransi, atau margin.

### A0. Yang diwarisi APA ADANYA

- g23 **A1** kecuali enumerasi (model pinocchio `results/p1_g23/reach_dwell_live.urdf`
  sha `f02e7c53…`; `lim` j1/j3 10, j2 **14**, pergelangan 7; offset
  `{6.6, 7.7, 6.6, 1.98, 1.98, 1.98}`; L1 = masker peta kolom rot = 0, slot
  lengan `sched.GANTRY_ARMS`).
- g23 **B0.1 A2′**: margin `m = [0.220, 0.6589, 0.8172, 0.1627, 0.0676, 0]`
  (`g23_calib2.json`) **tetap**, tidak dikalibrasi ulang.
- g22 **A2** (i)–(iv), **A3**, **A4**, **A5** (S1–S27, tahap 0–3), **A6**;
  g22 A8 1/3b/3c. LED / origin g2 / rel bebas **ditanyakan ulang** di G24b.
- Alat G22/G23 apa adanya (`make_instance.py`, `sched_screen.py --plan`,
  `rail_to_g.py`, `run_g22.py`, `js_record.py`); perubahan hanya jalur berkas.

### A1. 🔒 IK_V″ — sapuan roll eksplisit, per `(xyz, rel L, lengan a)`

```
roll grid      ψ_k = 5°·k, k = 0..71  (72 roll).  Target rotasi R(ψ) = diag(1,−1,−1) · Rz(ψ)
                 (diag(1,−1,−1) = quat x=1,w=0 = target probe; Rz = putar di sekitar sumbu alat)
IK per roll    6-D penuh: residu [p_tgt − p ; log3(R(ψ) · Rᵀ)] (LOCAL_WORLD_ALIGNED),
                 DLS λ = 1e-6, langkah 1.0, batas sendi URDF dijepit tiap iterasi
konvergen      ‖e_p‖ < 0.5 mm ∧ ‖e_R‖ < 0.5° ∧ di dalam batas
                 (lebih ketat dari probe 2 mm / 2°: tiap solusi juga solusi IK_V)
beda solusi    dua solusi pada roll yang sama = SAMA ⇔ max_j |Δq_j| < 0.01 rad
penemuan       ronde r memakai benih baru B_r:  |B_1| = 16 (REST + 15 dari default_rng(24)
                 seragam dalam batas), |B_2| = 16, |B_3| = 32, |B_4| = 64 (rng yang sama berlanjut;
                 benih identik untuk setiap tuple → deterministik).
                 Tiap benih × 12 roll jangkar (ψ = 0°, 30°, …, 330°), maks 300 iterasi.
kontinuasi     tiap solusi baru pada roll ψ_k → benih untuk ψ_{k±1}, maks 50 iterasi,
                 berjalan ke dua arah sampai tidak konvergen atau bertemu solusi yang
                 sudah tercatat pada roll itu (cabang sudah dilacak)
JENUH          ronde r tidak menambah satu pun solusi baru (atas ke-72 roll) → berhenti.
                 Minimal 2 ronde. Tidak jenuh sampai ronde 4 (128 benih) → TAK-JENUH
IK_V″          ⇔ ≥ 1 solusi pada ≥ 1 roll
TORQ″          ⇔ JENUH ∧ ∀ roll ∀ solusi s:  ∀ j  |τ_j(s)| + offset_j + m_j ≤ lim_j
                 (τ = gravitasi statis; perencana boleh berhenti di roll mana pun, cabang mana pun)
ORACLE″        = L1 ∧ IK_V″ ∧ TORQ″          (TAK-JENUH = tolak: terima-palsu lebih mahal)
```

- Yang disimpan per tuple: jumlah solusi, roll yang punya solusi, ronde,
  JENUH, dan **maks |τ_j| atas semua solusi** (6) → TORQ″ dan mutan margin/limit
  dihitung dari simpanan tanpa solve ulang.
- **Tidak dimodelkan, sengaja → ORACLE″ tetap syarat PERLU:** kemiringan sumbu
  alat ≤ 2° yang probe izinkan (oracle di tepat vertikal); torsi di antara
  titik grid 5°; lintasan dari keadaan awal (margin A2′ menanggungnya hanya
  untuk rencana dari REST); tabrakan (S18); batas waktu perencana 15 s; varians
  OMPL. Saringan (iv) tetap gerbangnya.
- **Model tereduksi diizinkan** (hanya 6 sendi lengan + rel gantry itu bebas,
  sisanya dikunci di REST / 0) **bila** FK tool_frame dan gravitasi 6 sendi
  sama dengan model penuh ≤ 1e-9 pada 100 konfigurasi acak (dicek, ditulis
  di §B, sebelum hitung).

### A2. 🔒 Validasi + kontrol

Set, semuanya ditetapkan sekarang:

- **V** = 49 rencana G22 B1 (`g22_screen.json`; 14 PLANNED / 12 TORQUE-UNSAFE / 23 NO-PLAN).
- **W** = 3 rencana (iv) G23 (`g23_screen.json`): s0 t2 arm_1 PLANNED (RNEA j2 2.74),
  s0 t4 arm_1 TORQUE-UNSAFE (7.83), s35 t0 arm_2 TORQUE-UNSAFE (7.72).
- **F** = tuple "jatuh" branch_probe G23: pilihan 300 dari tensor `g23_oracle.npz`
  dengan `default_rng(0)` persis `branch_probe.py`, diselesaikan ulang 64 benih
  (kode G23 apa adanya) → yang jatuh (G23: 90).
- **C′** = 81 rencana kalibrasi yang dieksekusi (`g23_calib2.json`, `q_final` terukur).

Dilaporkan (tidak menggerbang): matriks ORACLE″ × {PLANNED, TORQUE, NO-PLAN}
pada V, sebab tolak (L1 / IK_V″ / TAK-JENUH / TORQ″), presisi, recall, dan
perbandingan per-rencana dengan ORACLE′ G23.

**Kontrol yang MENGGERBANG** (gagal = oracle″ cacat → A2-fix, bukan dilonggarkan):

| # | Kontrol | Harus |
|---|---|---|
| **NC1** | V dengan `z := 0.40` | **0 / 49** IK_V″ |
| **NC2** | V, mutan `lim_2 = 5` | **0 / 49** TORQ″ |
| **NC5** | V dihitung dua kali | bit-identik 49/49 |
| **NC6** | W: kedua TORQUE-UNSAFE | **2 / 2 ditolak TORQ″** |
| **NC7** | F | **semua** ditolak TORQ″ |
| **PC1** | C′: IK_V″ | **≥ 95 %** (77/81) punya solusi |
| **PC2** | C′: cabang pendaratan perencana ditemukan — ∃ solusi grid s (roll mana pun) dengan `max_j |s_j − q_final_j| < 0.15 rad` (roll di luar grid ≤ 2.5° + miring ≤ 2°) | **≥ 95 %** |
| **PC3** | tuple V: maks \|τ_2\| oracle″ ≥ maks \|τ_2\| solusi 8-benih G23 − 0.1 N·m (sapuan ⊇ G23, modulo grid) | **≥ 95 %** tuple yang keduanya punya solusi |

**A2-fix — dikunci sekarang, satu kali:** kalau NC6, NC7, PC2, atau PC3 gagal,
grid roll diperhalus ke **2.5°** (144 roll, jangkar tetap tiap 30°) dan
**seluruh** oracle″ + validasi dihitung ulang dari awal. Kalau masih gagal:
oracle″ **cacat**, dilaporkan, **tidak** ada instance, tidak ada (iv). NC1/NC2/NC5
gagal = bug kode → diperbaiki, dihitung ulang, ditulis. Tidak ada perubahan
margin, toleransi, atau benih yang diizinkan.

### A3. 🔒 Lingkup hitung

Oracle″ dihitung pada **setiap tuple L1-benar** kolom rot = 0 di **semua**
node peta (3132 node × 33 rel × 2 slot × 2 gantry; L1-benar: **111 753** — dihitung
dari peta, bukan oracle″). Tuple L1-salah = ORACLE″ salah menurut definisi.
Kalau waktu terukur (A0-catatan) membuat ini > 8 jam pada 15 proses, lingkup
diganti **lazy**: dihitung hanya untuk node yang ditarik A5, nilainya identik
(fungsi deterministik yang sama) — hanya statistik ruang-kerja (A6-D95) yang
hilang, dan itu ditulis.

### A4. 🔒 Keputusan operator — tarikan tugas (prompt 1c), dijawab SEBELUM hitung

| # | Pertanyaan | Jawaban |
|---|---|---|
| **1c** | `gen_real` apa adanya (G23: 2/50 lolos (i)) atau tugas ditarik dari node layak-oracle? | **B — ditarik dari node layak-oracle″** (operator, 2026-09-22) |

### A5. 🔒 Instance — g22 A2 dengan tarikan layak-oracle″

```
untuk seed = 0 … 49 (urut):
  gen_real_oracle(6, seed, 0, (1, 2)):
      loop gen_real APA ADANYA (rng = default_rng(seed), cand = rng.integers(0, 3132),
      max_reject 10 000), SATU-SATUNYA beda: syarat terima kandidat =
        ∃ g ∈ {1,2}, rel L (rot = 0, 33), slot s :  L1 ∧ ORACLE″(node, L, lengan(g, s))
      (gen_real: ∃ pose rot MANA PUN dengan L1). reach/zone/hand dari _gantry_oracles apa adanya.
  R0, R1 = make_instance.restrict_rot0 APA ADANYA (p0 0.55 / 0.00)
  reach[g][:, p, slot] &= ORACLE″(node, L_p, lengan(g, slot))
  (i)–(iii) g22 A2 apa adanya; cadangan (ii) g22 A2 apa adanya, tidak ada yang lain.
  (iv) [G24b] sched_screen APA ADANYA (3/3, --stop-first); instance = seed pertama lolos (i)–(iv)
```

- `sched.py` **tidak** diubah: `gen_real_oracle` ditulis di `results/p1_g24/`
  sebagai salinan loop `gen_real` dengan satu predikat diganti; kesetaraannya
  diuji: dengan predikat L1-apa-pun ia harus menghasilkan `meta['nodes']`
  **identik** dengan `gen_real` untuk seed 0–49 (**kontrol K1**, menggerbang).
- (i) benar menurut konstruksi; tetap dihitung (**kontrol K2**: 50/50, kalau
  tidak = bug).

**Konsekuensi untuk naskah — ditulis SEKARANG, sebelum hasil:**

1. Distribusi instance **bukan** lagi "node peta seragam yang terjangkau L1";
   ia **node seragam yang punya ≥ 1 pose gantry rot = 0 di mana suatu lengan
   mencapainya dengan approach vertikal dan torsi statis aman di SETIAP roll**.
   Itu ruang kerja yang lebih kecil dan lebih "dalam". Setiap angka makespan /
   jumlah pindah G24+ harus dikutip dengan kalimat itu.
2. Angka G22/G23 (instance `gen_real`) dan G24 **tidak** dapat dibandingkan
   satu lawan satu: himpunan tugasnya berbeda menurut konstruksi. Yang sah
   dibandingkan: laju lolos (iv) per rencana, dan kelas penolakan.
3. Hasil scheduler G7–G21 (masker L1) tetap berlaku untuk **model**-nya; G24
   hanya menjawab "jadwal optimum dapat dieksekusi di sel nyata **bila**
   tugasnya dari ruang kerja aman-torsi". Tugas di luar itu (≥ ~30 % ruang
   jangkau arm_1, memori torsi) oleh desain tidak diberikan ke sel ini — itu
   batasan sel, dan ditulis sebagai batasan, bukan disembunyikan.
4. Bias: node dekat bahu/di bawah gantry lebih mungkin layak → jumlah pindah
   bisa lebih kecil dari G22 — atau lebih besar karena tiap node punya lebih
   sedikit pose. Diukur (A6-D96), tidak diasumsikan.

### A6. 🔒 Dugaan offline — D91–D97, DITULIS SEBELUM DATA

Prior (p1_state 9): dugaan dari mekanisme terukur tepat; dugaan tentang **kode
sendiri** dan tentang oracle **menerima** meleset ke arah "terlalu menerima".

| # | Dugaan | Dasar |
|---|---|---|
| **D91** | PC2: cabang pendaratan ditemukan pada **≥ 78 / 81** (≥ 95 %) | kontinuasi melacak kurva utuh; perencana mendarat di titik kurva itu, di sekitar 2.5° roll |
| **D92** | NC6 **2/2** dan NC7 **semua** (G23: 90) ditolak TORQ″ | titik tak aman G23 adalah titik kurva; kurva disapu (B3 g23) |
| **D93** | V: ORACLE″ menerima **≤ 4** rencana, **0** TORQUE-UNSAFE (s44 ditolak) | ∀ atas lebih banyak solusi hanya bisa menolak lebih banyak; s44 = cabang yang terlewat (tidak diverifikasi G23) |
| **D94** | V: IK_V″ ada pada **14 / 14** PLANNED (G23 8-benih: 12/14) | perencana (KDL) menemukannya → solusi ada; 64+ benih + sapuan menemukannya |
| **D95** | Ruang kerja: node dengan ≥ 1 tuple ORACLE″ ≤ **25 %** dari 3132; tuple ORACLE″ ≤ **2 %** dari 111 753 L1-benar | G23: 2.5 % tuple node tugas, dan ≥ 30 % di antaranya jatuh → ≤ ~1.8 % |
| **D96** | (ii)∧(iii) di bawah A5: **≥ 25 / 50** (G22 L1: 35/50) | tugas tetap tersebar di rel 0–2 m, dan tiap node punya lebih sedikit pose layak → pindah tetap diperlukan; sebagian seed kehilangan pindah di salah satu gantry |
| **D97** | Kontrol K1 (kesetaraan loop) lulus bit-identik 50/50 | salinan loop, predikat diganti |

### A7. 🔒 Dugaan (iv) + perangkat keras — D98–D101, dikunci SEKARANG untuk G24b

| # | Dugaan | Dasar |
|---|---|---|
| **D98** | (iv): seed lolos pertama dalam **≤ 5** seed (i)–(iii) pertama | TORQUE dan NO-PLAN kelas statis dibuang oracle″; sisa kegagalan = S18 / varians / miring 2° |
| **D99** | (iv): per rencana PLANNED **≥ 80 %** | sama |
| **D100** | (iv): rencana sesudah pindah PLANNED **≥ 70 %** (G22 0/13, G23 0/2) | tugas yang memaksa pindah kini dari ruang aman-torsi |
| **D101** | (tahap 3) makespan terukur **>** P2-serial | = D90 G23, tidak pernah dinilai; M3 + overhead perangkat lunak |

### A8. 🔒 G24b — perangkat keras (tidak hari ini)

g22 A5 tahap 0–3 apa adanya, dengan g23 A5: `rail_to_g` / `run_g22` **DRY
dulu** (`run_g22 --plan --out-dir --archive --prefix`); `return_rest` per
gantry sebelum tahap 2 bila lengan > 0.5° dari REST (tidak dihitung). Tiap
gerak izin operator. Kalau tidak ada seed lolos (iv): tidak ada gerak,
dilaporkan.

---

## B. Hasil terukur

§A dikunci 17:49 (sha256 teks §A `b07f2a693c331f24`, salinan
[sectionA_locked.md](results/p1_g24/sectionA_locked.md)). Berkas:
[results/p1_g24/](results/p1_g24/) — [oracle2.py](results/p1_g24/oracle2.py),
[validate2.py](results/p1_g24/validate2.py), [make_instance_g24.py](results/p1_g24/make_instance_g24.py).

### B0. Sebelum data oracle″

- **Model tereduksi (A1):** FK + gravitasi vs model penuh, 400 konfigurasi:
  maks selisih **1.8e-15** ✅ ([check_reduced.py](results/p1_g24/check_reduced.py)).
- **Waktu (A3):** 6 tuple di luar V/W/F/C′: 2.7–8.6 s, rerata ~4.6 s →
  111 753 tuple ≈ **9.5 jam** pada 15 proses > 8 jam → lingkup **lazy** (A3).
- **K1:** loop salinan = `gen_real` pada **50/50** seed ✅ (D97).

### B1. Validasi — grid 5° ([log](results/p1_g24/g24_validate.log)) dan A2-fix 2.5° ([log](results/p1_g24/g24_validate_2.5.log)): IDENTIK

| ORACLE″ | PLANNED | TORQUE-UNSAFE | NO-PLAN |
|---|---|---|---|
| terima | **1** (s16 t0) | **0** | **0** |
| tolak | 13 (TORQ″ 7, TAK-JENUH 4, IK_V″ 2) | **12** (TORQ″ 10, TAK-JENUH 2) | **23** (IK_V″ 23) |

Presisi 1/1, recall **1/14** (G23: 4/14). s44 (terima-palsu G23) kini ditolak.
F tereproduksi: 90/300 jatuh (= G23).

| Kontrol | 5° | 2.5° | |
|---|---|---|---|
| NC1 / NC2 / NC5 | 0 / 0 / 49 | sama | ✅ |
| **NC6** (dua penolakan (iv) G23) | **2/2** (j2 maks 8.02 / 6.83) | sama | ✅ |
| **NC7** (F) | **88/90** | **88/90** | ❌ |
| PC1 | 81/81 | sama | ✅ |
| **PC2** | **70/81** | **70/81** | ❌ |
| PC3 | 22/22 | sama | ✅ |

**A2-fix menurut aturan → oracle″ CACAT. Tidak ada instance, tidak ada (iv).**

**Sebab — terukur ([diag_pc2.py](results/p1_g24/diag_pc2.py), [log](results/p1_g24/g24_diag_pc2_grid5.log)), bukan grid:**

1. **Kemiringan ≤ 2° yang tidak dimodelkan (A1) ternyata yang dipakai kontrol.**
   NC7: kedua lolos-salah j2 maks **5.64** vs 64-benih G23 **5.65**, batas statis
   5.64 — IK 5-D G23 menerima miring < 2°, oracle″ tepat vertikal. PC2: pada
   **10/11** miss, `q_final` diproyeksikan ke roll grid terdekat mendarat
   **≤ 0.03 rad** dari solusi sapuan — cabangnya **ditemukan**; jaraknya > 0.15
   rad karena miring 1.2–2.5° di dekat singularitas pergelangan (q5 ≈ 0)
   memutar j4/j6 jauh. 1/11 (g20 #10 arm_2) proyeksinya melompat 2.15 rad —
   **tidak ditentukan**.
2. **Singularitas pergelangan = kontinum, bukan cabang.** Di q5 ≈ 0, j4 + j6
   membentuk keluarga 1-D pada **satu** roll; hitung "solusi berbeda" tidak
   jenuh → **TAK-JENUH 6/49** di V (4 di antaranya PLANNED, juga W s0 t2).
   Kriteria jenuh A1 salah bentuk untuk target approach-vertikal.

➜ **Klaim yang boleh ditulis:** *sapuan roll eksplisit menolak kedua rencana
tak aman yang oracle 8-benih terima (NC6 2/2) dan seluruh TORQUE-UNSAFE (12/12),
tetapi (a) toleransi kemiringan 2° perencana menggeser torsi statis ±0.01 N·m di
batas dan (b) singularitas pergelangan pada approach vertikal membuat enumerasi
"semua solusi" tak terhingga; oracle yang sahih harus memodelkan keduanya.*

### B2. Papan skor — D91–D97

| # | Terukur | |
|---|---|---|
| D91 | PC2 **70/81** | ❌ |
| D92 | NC6 2/2, NC7 **88/90** | ❌ |
| D93 | terima **1**, TORQUE 0 | ✅ |
| D94 | IK_V″ PLANNED **12/14** | ❌ |
| D95 / D96 | tidak ada tarikan | — |
| D97 | K1 50/50 | ✅ |
| D98–D101 | tidak ada (iv) | — |

Ketiga yang meleset lagi tentang **aturan/kode sendiri** (kontrol dan
kriteria jenuh saya). Papan skor G24 total di B′3.

### B3. Pertentangan §B lawan §A — G24

| # | Pertentangan |
|---|---|
| **(1)** | A1 "miring 2° tidak dimodelkan" — tetapi NC7 dan PC2 diam-diam mengandaikannya (B1-1) |
| **(2)** | A1 kriteria JENUH (hitung solusi berbeda) salah bentuk di singularitas pergelangan: kontinum → TAK-JENUH 6/49 |
| **(3)** | A2-fix mengandaikan kegagalan = resolusi grid; 2.5° identik dengan 5° |
| **(4)** | A2 PC2 ambang 0.15 rad mengandaikan miring kecil → Δq kecil; salah di dekat q5 = 0 |
| **(5)** | A3: lingkup penuh 9.5 jam → lazy dipicu (sesuai aturan, dicatat) |

**G24: LIMA.**

### B4. Keputusan operator (sesudah B1, sebelum oracle‴ dihitung)

**Oracle‴ di sesi ini**, dilabeli **koreksi pasca-data** (preseden g23 B0): A′
di bawah dikunci sebelum satu pun nilai oracle‴ dihitung.

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

## B′. Hasil oracle‴ (A′ dikunci 18:12, sha `40dc68a40be5067b`, [sectionA3_locked.md](results/p1_g24/sectionA3_locked.md))

### B′1. Validasi ([log](results/p1_g24/g24_validate3.log), [json](results/p1_g24/g24_validate3.json)) — SEMUA KONTROL LULUS

| ORACLE‴ | PLANNED | TORQUE-UNSAFE | NO-PLAN |
|---|---|---|---|
| terima | **2** (s0 t2, s16 t0) | **0** | **0** |
| tolak | 12 (TORQ‴ 10, IK_V 2) | **12** (TORQ‴ 12) | **23** (IK_V 23) |

Presisi **2/2**, recall 2/14; TAK-JENUH **0/49**. NC1 0, NC2 0, NC5 49/49,
**NC6 2/2** (j2 maks 8.17 / 6.99), **NC7 90/90**, PC1 81/81, **PC2′ 80/81**
(median 0.0013 rad; satu-satunya gagal g20 #10 arm_2, 2.15 — sama seperti B1),
PC3 22/22. W: s0 t2 (PLANNED di G23) kini **diterima**, j2 maks 5.31.

| # | Terukur | |
|---|---|---|
| D102 | NC7 90/90 | ✅ |
| D103 | PC2′ 80/81 (≥ 78) | ✅ |
| D104 | TAK-JENUH 0 | ✅ |
| D105 | terima 2, TORQUE 0 | ✅ |

### B′2. Instance A5 (tarikan layak-oracle‴) — **(i)–(iii) 45 / 50** ([log](results/p1_g24/g24_make_instance.log), [g24_candidates.json](results/p1_g24/g24_candidates.json), cache [g24_oracle3_cache.jsonl](results/p1_g24/g24_oracle3_cache.jsonl))

18:14 → 00:39, 41 gelombang, nol error. Lingkup lazy: **780** node kandidat
berbeda (27 659 tuple L1-benar) dihitung.

| | |
|---|---|
| (i) K2 | **50/50** ✅ |
| (ii)∧(iii) | **45/50**; gagal (ii): s24 (g2 0 tugas), s28, s37, s46, s49 (g1 0 pindah) |
| konflik BLOCK | 0/50 |
| laju terima kandidat | 300 / 890 = **33.7 %** (rerata `rejects` 11.8 per seed) |
| node kandidat layak-oracle‴ | **270 / 780 = 34.6 %** (draw seragam → penaksir fraksi node peta) |
| tuple ORACLE‴ benar | **1 510 / 27 659 = 5.5 %** dari L1-benar |
| makespan model FISIK, 45 lolos | 74.6 – 261.7 s, median 154.3; rerata **3.96 pindah** (G22 L1: 2) |
| jadwal FISIK = DINDING | 36/50 (G22: 50/50) — jadwal beda, jumlah pindah sama |
| TAK-JENUH di cache | 1 / 27 659 |

**Instance untuk (iv) (G24b):** urutan seed apa adanya → **seed 0** dulu (g1 3
pindah, g2 2; 4/2 tugas; FISIK 180.37 s), lalu 1, 2, 3, …

⚠️ **Kemiringan tidak konvergen:** 1 508 / 1 510 tuple yang diterima punya ≥ 1
Newton miring yang gagal; 19 % dari semua (solusi × 8 miring). Amplop torsi
mengabaikan miring itu. NC7 (90/90) dan PC2′ lulus meski begitu, tetapi yang
gagal **tidak** dipilah (batas sendi = perencana juga tak bisa ke sana, atau
Newton divergen dekat singularitas = titik amplop terlewat). Ini sumber
terima-palsu yang paling mungkin untuk (iv).

### B′3. Papan skor G24 (seluruhnya)

| # | | # | |
|---|---|---|---|
| D91 PC2 | ❌ | D102 NC7 | ✅ |
| D92 NC6∧NC7 | ❌ | D103 PC2′ | ✅ |
| D93 V terima ≤ 4 | ✅ | D104 TAK-JENUH ≤ 1 | ✅ |
| D94 IK PLANNED 14/14 | ❌ (12) | D105 terima ≤ 2 | ✅ |
| D95 node ≤ 25 %, tuple ≤ 2 % | ❌ (34.6 %, 5.5 %) | D106 terima ≥ 10 % | ✅ (33.7 %) |
| D96 (ii)∧(iii) ≥ 25 | ✅ (45) | D97 K1 | ✅ |

D98–D101 menunggu (iv) / tahap 3 (G24b). **G24: 4 meleset / 8 tepat →
papan skor 52 meleset / 49 tepat.** Tiga yang meleset tentang kode/kontrol
sendiri; D95 menduga ruang kerja aman **lebih sempit** dari kenyataan (arah
"kendala lebih mengikat", prior §7.2 tua — kebalikan G23).

Pertentangan tambahan: **(6)** A′1 mengandaikan Newton miring konvergen; 19 %
tidak (B′2). **(7)** D95 dinilai dari penaksir lazy, bukan hitungan penuh (A3).
**G24: TUJUH.**

## C. Keadaan akhir (Rule 12)

- **Nol gerak**, lengan mati (operator tidak di lokasi). Tidak ada `ros2` diluncurkan.
- Offline selesai: oracle‴ tervalidasi, 45 instance (i)–(iii). **Tidak** dijalankan:
  saringan (iv), tahap 1–3, D98–D101 (G24b).
- `oracle2.py` default = oracle″ (direproduksi, dicek); `tilt=True, envelope=True` = oracle‴.
- `sched.py` / peta / `make_instance.py` / alat G22 **tidak** diubah. Disk 1.9 GB.

## D. Prompt G24b (salin ke chat BARU saat operator di lokasi)

**Rekomendasi: Opus 5, effort TINGGI** — pertama kali rel gantry 2 bergerak dan
pertama kali `rail_to_g` / `run_g22` hidup; kesalahan di sini menyentuh perangkat keras.

```
Sesi G24b -- saringan (iv) + (kalau lolos) SATU jadwal end-to-end di sel NYATA,
instance dari docs/results/p1_g24/g24_candidates.json. Repo ceiling_arm, branch feat/rgbd-topo-deploy.

BACA PENUH: CLAUDE.md; docs/p1_g24_roll_oracle.md (A, A', B'1-B'3, C);
docs/p1_g22_hw.md A2-A6, A8, B0.3, B1; docs/p1_g23_oracle_hw.md B3, C.

1. Operator: LED 4 lengan tidak merah, origin g2 = home, rel/ruang bebas (tanya ULANG).
2. Tahap 0 g22 A5 (remount_check, ICMP .10-.13, bring-up enable_gantry_bridge:=true,
   4x Actuator count, 7/7 controller). Cek rel terbaca = R1 0.55 / 0.00.
3. (iv) sched_screen --plan pada seed LOLOS (i)-(iii) urut (0,1,2,...), 3/3, --stop-first.
   D98-D101 SUDAH dikunci (g24 A7) -- nilai, jangan tulis ulang.
4. Kalau lolos: rail_to_g DRY tiap traverse, return_rest DRY, run_g22 --plan --out-dir
   --archive --prefix DRY; tahap 2 (rel g2 keluar-kembali, S25); tahap 3. Tiap gerak izin operator.
5. Perbarui p1_state 6 + 8c + tally 9.

ATURAN: B menang atas A dan DITULIS; pgrep -f jangan cocok dengan bash sendiri;
launch python3 -c "...SIG_DFL...execvp" &, cek SigIgn sebelum kill -INT; disk ~1.9 GB,
hapus crash dump move_group milikmu.
```

---

## E. G24b — sel NYATA, 2026-09-23

### E0. Tahap 0 — nol gerak

| Gerbang | Hasil |
|---|---|
| Operator (ditanya ULANG, 2026-09-23) | LED 4/4 tidak merah; origin g2 = home fisik; rel + ruang antar-gantry bebas ✅ |
| pra-cek | nol proses ROS basi; enp112s0 192.168.2.100; `ros2_kortex` **e712295**; disk **4.3 GB** (S27 ✅) |
| `remount_check.py` | **GERBANG LULUS**; ICMP .10–.13 ✅ |
| bring-up (`/tmp/g24b_t1.log`, PID 1098829, SigIgn `0x1001001` = tanpa SIGINT) | **4×** "Actuator count … '6'"; **7/7** controller active; nol FAULT / Kortex exception; 7 spawner "process has died" (= G23) ✅ |
| `/joint_states` (40 pesan) | rel **0.550910 / 0.000094** → R1 0.55 / 0.00 ✅; rotasi g1 0, **g2 0.03°** (G23: 0; < 0.5°) |
| lengan | maks \|q − REST\| arm_1 0.654°, arm_2 0.467°, arm_3 **1.065°** (G23 0.838), arm_4 0.716° |
| validator penilai | **7/7 PASS**; penilai 0 sebelum / 0 sesudah (penghitung melewati argv[0] bash) ✅ |

### E1. 🔒 Saringan (iv) — **seed 1 LOLOS 3/3** ([log](results/p1_g24/g24_screen.log), [json](results/p1_g24/g24_screen.json), sha `b9d870fed33cc3fe`)

| seed | hasil |
|---|---|
| 0 | t1 arm_1 @0.55 PLANNED; retract/traverse g1 0.55→0.75 CLEAR; **t5 arm_1 `(1.2857, 0.3882, 1.4)` @0.75 TORQUE-UNSAFE** — RNEA j2 **7.17** → 14.87 > 14. Batas statis oracle‴ j2 = 14 − 7.7 − 0.659 = 5.64 → **terima-palsu oracle‴** (kelas B′2: miring tak konvergen / cabang terlewat — tidak dipilah) |
| **1** | **PLANNED / PLANNED / PLANNED → LOLOS** (6 tugas × 3; retract 2 × 3 CLEAR; S24 g1 0.55→0.70 min 507.6 mm, g2 0.00→1.45 min **452.7** mm) |

**Instance = seed 1** (A2 mekanis): g1 1 pindah 0.55 → 0.70 (t0, t1 @0.55; t5 @0.70; semua arm_1);
g2 1 pindah **0.00 → 1.45** (t2, t3 @0.00; t4 @1.45; semua arm_3); FISIK = DINDING jadwal sama.

### E2. 🔒 Prediksi (A4) — DIKUNCI SEBELUM GERAK

| | F1 | F2 | max (model) | serial (A3) |
|---|---|---|---|---|
| **P1** FISIK T_lin | 61.86 | 103.24 | **103.24** (= solver) | 165.11 |
| **P2** FISIK T_cmd | 65.13 | 137.36 | 137.36 | **202.49** |
| **P3** DINDING T_lin | 137.86 | 179.24 | 179.24 | 317.11 |
| **P4** DINDING T_cmd | 141.13 | 213.36 | 213.36 | 354.49 |

(Σ T_lin g1 5.065 / g2 46.445; Σ T_cmd 8.333 / 80.556; m = 1 / 1; D = 6.0 / 6.0.)
D101 dinilai terhadap **P2-serial 202.49 s**.

### E3. Papan skor (iv) — D98–D100 (dikunci g24 A7, dinilai di sini)

| # | Terukur | |
|---|---|---|
| D98 | lolos pertama = seed ke-**2** (≤ 5) | ✅ |
| D99 | per rencana PLANNED **20 / 21** = 95 % (≥ 80 %) | ✅ |
| D100 | sesudah pindah PLANNED **6 / 7** = 86 % (≥ 70 %) | ✅ |
| D101 | tahap 3 | ⏳ |

### E4. Tahap 1 (sisa) + tahap 2 — izin operator per gerak

| Langkah | Hasil |
|---|---|
| `return_rest` DRY ([log](results/p1_g24/g24b_rest_dry.log)) | g1 / g2 CLEAR (se-gantry 622.7 / 622.1, antar 546.1 mm), rc 0 |
| **GERAK** `return_rest --move` ([log](results/p1_g24/g24b_rest_move.log)) | g1 0.65° → **0.028°**, torsi puncak 4.47 (t1_a1_j2); g2 1.07° → **0.046°**, 4.70 (t2_a1_j2); rc 0 |
| `rail_to_g` DRY g1 0.55→0.70 / g2 0→1.45 | S12/S23 ✅; S24 CLEAR **507.4 / 539.7** mm; T_cmd 8.28 / 80.55 s |
| penilai + perekam | `reach_dwell_monitor` PID 1109800 (`/tmp/g24b_step_*`), `js_record` PID 1109757 (`/tmp/g24b_js.csv`), keduanya SigIgn tanpa SIGINT |
| `run_g22 --dry` ([wrapper](results/p1_g24/g24b_run.sh)) | 10 event = urutan (iv); awal 0.046°, rel 0.550910 / 0.000094; S26 bersih; "DRY: tidak ada yang dikirim". ⚠ keluar **rc 134** (`terminate called without an active exception`: thread spin rclpy hidup saat interpreter mati) — SESUDAH semua kerja; `finish()` menyalin arsip sebelum `sys.exit`. Alat tidak diubah (A0) |
| **GERAK tahap 2** (S25, operator melihat carriage) g2 0.000094 → 1.45 ([log](results/p1_g24/)) | akhir 1.449416, galat **−0.58 mm**, t_traverse **78.20 s** (T_cmd 80.55, T_lin 46.44), drift lengan 0.149°, rel g1 0.000 mm, torsi lengan puncak 2.29, JTC 0 |
| **GERAK tahap 2** g2 1.449416 → 0 | akhir 0.000555, galat **+0.56 mm**, t_traverse **78.20 s**, drift 0.139°, rel g1 0.000 mm, JTC 0 |
| log launch sesudah gerak | nol ERROR/FAULT baru; satu-satunya kecocokan pola fault = `table1 NOT armed` saat bring-up (baseline, ts sebelum controller aktif) |

➜ **Tahap 2 LULUS** (≤ 2 mm, < 0.5°, nol fault). Gerak rel gantry 2 **pertama di proyek ini**, 1.45 m, dua arah; traverse ≈ T_cmd (bridge debounce, g19) — 1.68× T_lin.

### E5. 🟢 Tahap 3 — JADWAL PENUH seed 1: **6 / 6 tugas sukses, makespan terukur 480.44 s** ([run.json](results/p1_g24/g24b_run.json), [runner.log](results/p1_g24/g24b_runner.log), log per event `g24b_ev*`)

Pertama kali **satu jadwal keluaran scheduler dijalankan utuh di sel nyata**, kedua
rel bergerak (G22 0/35, G23 0/2 lolos saringan). Nol auto-stop S26, nol HALTED,
semua verdict probe `SUCCESS`. t0 = 1790129308.04.

| # | event | mulai (s) | durasi (s) | hasil |
|---|---|---|---|---|
| 0 | t0 arm_1 `(0.9286, 0.3176, 1.0)` @0.55 | 0.02 | 52.20 | success @**52.04** |
| 1 | t1 arm_1 `(1.0, 0.4588, 1.08)` @0.55 | 52.23 | 52.44 | success @**104.49** |
| 2 | retract g1 | 104.69 | 52.71 | rc 0, torsi puncak 7.24 |
| 3 | traverse g1 0.55 → 0.70 | 157.42 | 14.75 | t_traverse **8.01** (T_cmd 8.28, T_lin 5.04), galat −0.13 mm, drift 0.099° |
| 4 | t5 arm_1 `(1.2857, 0.2471, 1.0)` @0.70 | 172.19 | 49.59 | success @**221.59** |
| 5 | t2 arm_3 `(0.2857, −0.3882, 1.08)` @0.00 | 221.79 | 27.73 | success @**249.34** |
| 6 | t3 arm_3 `(0.5, −0.3176, 1.0)` @0.00 | 249.54 | 41.49 | success @**290.84** |
| 7 | retract g2 | 291.05 | 52.76 | rc 0, torsi puncak 6.21 |
| 8 | traverse g2 0.00 → 1.45 | 343.82 | 86.98 | t_traverse **78.25** (T_cmd 80.53, T_lin 46.43), galat −0.54 mm, drift 0.151° |
| 9 | t4 arm_3 `(1.7857, −0.3882, 1.24)` @1.45 | 430.82 | 49.79 | success @**480.44** |

**Terukur lawan prediksi E2 (dikunci sebelum gerak):**

| | terukur | P1 FISIK T_lin | P2 FISIK T_cmd | P3 DINDING T_lin | P4 DINDING T_cmd |
|---|---|---|---|---|---|
| W_1 (g1) | **221.57** | 61.86 | 65.13 | 137.86 | 141.13 |
| W_2 (g2) | **258.65** | 103.24 | 137.36 | 179.24 | 213.36 |
| makespan (serial A3) | **480.44** | 165.11 (×2.91) | 202.49 (×2.37) | 317.11 (×1.52) | 354.49 (×1.36) |

- **Rel:** t_traverse ≈ T_cmd (8.01 / 8.28; 78.25 / 80.53) — lagi bridge debounce g19,
  **bukan** T_lin (×1.59 / ×1.69). Overhead alat `rail_to_g` (saringan S24 + tunggu
  diam 1 s) +6.7 / +8.7 s di atas t_traverse.
- **Retract:** 52.7 / 52.8 s per panggilan (30 s gerak + saringan + overhead) vs
  g19 FISIK 29.02 / DINDING 48.81 — **di atas DINDING**.
- **Tugas:** 27.7–52.4 s per tugas (rencana + saringan + eksekusi + dwell 2 s) vs model
  D = 2.0 s per slot. Ini suku terbesar: Σ tugas **273.2 s** = 57 % makespan; model
  hanya memberinya 12 s.
- **Torsi terukur:** puncak `joint_2` seluruh run **9.62** (t1_a1) / **9.46** (t2_a1)
  / 1.33 / 0.71 N·m — < 12 dan < 14. t5 arm_1 yang disaring RNEA 5.21 + 7.7 = **12.91**
  terukur **≤ 9.62** (puncak run memuat t5) → offset efektif ≤ 4.4 di rencana ini;
  +7.7 (A8-1) konservatif ≥ 3.3 N·m di sini. Tidak dipakai untuk mengubah apa pun.
- **Retract penutup** (tidak dihitung, [log](results/p1_g24/g24b_rest_move_closing.log)):
  g1 170.3° → 0.036°, puncak 4.72; g2 170.4° → 0.045°, 5.27; rc 0 / 0.
- Runner keluar **rc 134** lagi sesudah `SELESAI` (sama seperti DRY, E4); arsip sudah
  tersalin (`finish()` sebelum abort).

### E6. Papan skor G24b — D98–D101

| # | Dugaan | Terukur | |
|---|---|---|---|
| D98 | (iv) lolos dalam ≤ 5 seed | seed ke-**2** | ✅ |
| D99 | per rencana PLANNED ≥ 80 % | **20/21** = 95 % | ✅ |
| D100 | sesudah pindah PLANNED ≥ 70 % | **6/7** = 86 % | ✅ |
| D101 | makespan terukur > P2-serial | **480.44 > 202.49** | ✅ |

**G24b: 0 meleset / 4 tepat → papan skor 52 meleset / 53 tepat.** Keempatnya
ditarik dari mekanisme terukur (oracle‴ membuang kelas statis; overhead perangkat
lunak M3 + debounce rel g19) — pola G21/G24. D101 tepat, tetapi **besarnya**
(×2.37) tidak diduga: suku tugas (rencana 12–30 s tiap tugas), bukan rel, yang
mendominasi.

### E7. Pertentangan §E lawan §A / prompt — G24b

| # | Pertentangan |
|---|---|
| **(1)** | Prompt G24b: "`run_g22 --plan --out-dir --archive --prefix` DRY" — di `run_g22` `--plan` = jalur kandidat; DRY adalah **`--dry`** |
| **(2)** | g22 A5 tahap 1 / g24 A8: `rail_to_g` DRY + `run_g22` DRY "nol gerak" — keduanya **MENOLAK** (S12 / A3-awal) bila lengan > 0.5° dari REST, dan bring-up memberi 0.47–1.07°. DRY yang bermakna menuntut `return_rest --move` **dulu** (gerak, izin operator) |
| **(3)** | `rail_to_g` DRY menyapu dari rel **terukur** ke goal, bukan `frm` jadwal; sapuan jadwal sebenarnya hanya dicakup `sched_screen` (iv) |
| **(4)** | `run_g22` keluar rc **134** (thread spin rclpy) sesudah selesai, DRY maupun gerak — "rc ≠ 0" tidak dapat dipakai sebagai tanda gagal runner |
| **(5)** | Seed 0 ditolak di rencana yang oracle‴ terima (RNEA j2 7.17 vs batas statis 5.64) — terima-palsu yang **diprediksi** B′2 (Newton miring tak konvergen), bukan pertentangan dengan A, dicatat sebagai bukti pertama |

**G24b: EMPAT** (1)–(4).

### E8. Keadaan akhir G24b (Rule 12)

- **Rel dikembalikan** (izin operator): g1 0.70 → **0.549999**, g2 1.45 → **0.000534**,
  lalu — permintaan operator — **g1 0.55 → 0.000503** (galat +0.50 mm, t_traverse
  29.19 s, drift 0.045°). ⚠ Tujuan 0.00 **bukan** nilai jadwal; S13′ dipenuhi lewat
  rencana pemulihan eksplisit [g24b_recovery_plan.json](results/p1_g24/g24b_recovery_plan.json)
  (seed 9001, catatan di dalamnya), `rail_to_g` **apa adanya** via
  [g24b_home_g1.sh](results/p1_g24/g24b_home_g1.sh), DRY dulu (S24 CLEAR 546.0 mm).
  Operator mengonfirmasi fisik sebelumnya: **enkoder g1 0 = home fisik** (carriage
  ~55 cm dari home saat 0.550) — menutup keraguan origin 2026-09-21.
  **Sesi berikut: R1 membaca g1 ≈ 0.00, bukan 0.55** — instance G22–G24 memakai
  p0 = 0.55 dan harus dibuat ulang atau rel dikembalikan dulu.
- Lengan: keempatnya REST (≤ 0.05° sesudah retract penutup, drift traverse ≤ 0.15°).
- **Stack dimatikan:** SIGINT penilai (1109800) + perekam (1109757), lalu launch
  1098829 (SigIgn tanpa SIGINT) → keluar ≤ 30 s; launch meng-eskalasi ke SIGKILL
  untuk `ros2_control_node` / `move_group` / `rviz2` **sesudah** 14 baris *deactivate*
  (= G23); nol FAULT / Kortex exception / `INVALID_USER_SESSION`; `ros2 node list
  --no-daemon` kosong. Crash dump `move_group` sesi ini (196 MB, 11:25) dihapus;
  dump python 09-21 (bukan milik sesi) dibiarkan. Disk **4.1 GB**.
- Arsip: log launch [g24b_launch.log.gz](results/p1_g24/g24b_launch.log.gz); `/tmp/g24b_js.csv`
  (perekam) dan `/tmp/g24b_step_*` (penilai) **tidak** disalin ke repo (ukuran), masih di `/tmp`.
- **Tidak diubah:** `sched.py`, peta, `make_instance.py`, alat G22 (`rail_to_g`,
  `run_g22`, `sched_screen`, `return_rest`). **Baru:** wrapper `g24b_{rest,rail,run,home_g1}.sh`,
  `g24b_recovery_plan.json`. `.claude/settings.local.json` (izin Bash untuk wrapper,
  dibuat operator) — belum ada di `.gitignore`.
- **Belum dikerjakan / tidak diukur:** dekomposisi per tugas rencana vs eksekusi vs dwell
  dari `g24b_ev*_task.log` (hanya durasi per event di E5); torsi puncak **per tugas**
  (hanya puncak seluruh run); M2/M3 koreksi A4; perbandingan dengan g19 per-langkah.
