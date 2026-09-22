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

