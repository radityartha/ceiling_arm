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
