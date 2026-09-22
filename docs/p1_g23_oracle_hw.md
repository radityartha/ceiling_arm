# P1 / G23 — oracle kelayakan L2-torsi untuk scheduler, lalu (kalau lolos) SATU jadwal di sel NYATA

Lanjutan [p1_g22_hw.md](p1_g22_hw.md): jadwal optimum dari oracle **L1** (peta
5 cm, approach ≤ 45°, tanpa torsi) lolos saringan plan-only **0/35**; per rencana
14/49, dan **0/13** di pose sesudah pindah (g22 §B1). Sesi ini mengganti
**hanya** masker `reach` scheduler dengan oracle yang menanyakan hal yang
perencana tanyakan — model penjadwalan, `sched.py`, peta, `t_fold` **tidak**
diubah.

---

## A. Protokol — ditulis 2026-09-22 14:37, SEBELUM oracle dihitung, SEBELUM gerak apa pun

> 🔒 Seluruh §A dikunci sebelum satu pun nilai oracle, margin, validasi, atau
> instance dihitung. Yang dihitung sesudahnya masuk §B; kalau §B bertentangan
> dengan §A, §B menang dan pertentangannya ditulis (§B-akhir).

### A0. Yang diwarisi APA ADANYA

- g22 **§A1** (M1–M3), **§A2** (aturan instance — di sini hanya masker `reach`
  yang diganti, A4), **§A3** (urutan eksekusi serial per gantry), **§A4**
  (prediksi P1–P4 + koreksi M2/M3), **§A5** (S24–S27 + tahap 0–3), **§A6**
  (besaran dilaporkan). Semua palang S1–S27.
- g22 **§A8**: 1 (offset `joint_2` **+7.7**, batas 14), 3b (serial), 3c (tahap 2
  keluar-kembali) berlaku. **2 (LED) dan 3 (origin g2 / rel bebas / ruang bebas)
  ditanyakan ULANG** ke operator sebelum bring-up — itu keadaan fisik hari ini,
  bukan keputusan.
- Alat G22 dipakai apa adanya: `make_instance.py` (R0/R1/`decompose`/`sched_json`),
  `g22_plan.py` (urutan event + S24), `sched_screen.py` (saringan (iv)),
  `rail_to_g.py`, `run_g22.py`, `js_record.py`. Perubahan yang diizinkan hanya
  **jalur berkas** (kandidat G23 ≠ kandidat G22), ditulis di §B.

**Koreksi kutipan sebelum data.** Prompt dan g22 B1 menulis "statis
melebih-lebihkan lintasan ~3×, g17 B1.8". g17 B1.8 menulis **~2×** (10 → 5
pasangan), dan g17 B1.9(b) membatasinya: sebagian adalah varians perencana. Yang
dipakai di sini: **2×, sebagian varians** — dan tidak ada angka di §A yang
bergantung padanya.

### A1. 🔒 Oracle L2-torsi — definisi

Per `(node peta i, pose rel L ∈ 33 nilai rot = 0, lengan a ∈ {arm_1..arm_4})`:

```
ORACLE(i, L, a) = L1(i, L, a)  ∧  IK_V(i, L, a)  ∧  TORQ(i, L, a)
```

- **L1** = masker `reach` peta apa adanya (kolom rot = 0, slot lengan sesuai
  `sched.GANTRY_ARMS`: g1 slot 0/1 = arm_1/arm_2, g2 = arm_3/arm_4). Jadi
  ORACLE ⊆ L1 menurut konstruksi. IK_V ∧ TORQ juga dihitung **tanpa** L1 dan
  jumlah tuple yang L1 tolak tetapi IK_V ∧ TORQ terima dilaporkan (cek peta,
  bukan dipakai).
- **Model**: pinocchio atas `/tmp/reach_dwell_live.urdf` (sha256
  `f02e7c532cdbee70…`) — **berkas yang sama** dengan yang dipakai RNEA
  penyaring probe di G22 (cache `_urdf_path`); disalin ke `results/p1_g23/`.
  Rel lengan itu di `L`, rotasi 0; lengan lain tidak memengaruhi kinematika.
- **IK_V** — sama dengan yang probe minta dari perencana
  (`_goal_constraints`: bola 2 mm, orientasi x/y 2°, roll bebas; target
  `quat x = 1, w = 0` ⇒ sumbu-z `tool_frame` = −z dunia):
  - residu 5-D: posisi `tool_frame` (3) + penyelarasan sumbu-z alat ke −z (2);
    damped least squares, λ = 1e-6, langkah 0.5, **batas sendi URDF dijepit**
    tiap iterasi, maks **300** iterasi;
  - **8 benih tetap**: REST + 7 dari `default_rng(23)` seragam di dalam batas
    sendi (benih yang sama untuk setiap tuple → deterministik);
  - **konvergen** ⇔ ‖galat posisi‖ < **2 mm** ∧ ∠(z_alat, −z) < **2°** ∧ di
    dalam batas sendi. Solusi konvergen dari semua benih dikumpulkan.
- **TORQ** — sama dengan penyaring probe (`_plan_and_screen`), tetapi statis:
  `τ_j` = gravitasi (`computeGeneralizedGravity`) di solusi IK;
  `lim_j` = rating URDF dijepit `--tau-max 12` untuk j ≠ 2 (j1/j3 **10**,
  pergelangan **7**), `joint_2` **14**; offset `{6.6, 7.7, 6.6, 1.98, 1.98, 1.98}`
  (A8-1 g22); **margin `m_j` dari A2**.
  `TORQ ⇔ ∃ solusi konvergen s : ∀ j  |τ_j(s)| + offset_j + m_j ≤ lim_j`
  (keberadaan: perencana boleh sampai di cabang IK mana pun).
- **Tidak dimodelkan, sengaja, dan karena itu ORACLE tetap syarat PERLU, bukan
  cukup:** lintasan dari keadaan awal (tugas ke-2 lengan yang sama tidak mulai
  dari REST), tabrakan diri / adegan / antar-lengan (S18), batas waktu
  perencana 15 s, varians OMPL (g17 B1.9). Saringan (iv) tetap gerbangnya.

### A2. 🔒 Margin `m_j` — ATURAN ditulis sekarang, angkanya dari set kalibrasi yang TERPISAH dari validasi

Tidak ada angka margin yang bisa dibela tanpa data, dan angka yang dipilih
sesudah melihat validasi dilarang. Jadi aturannya dikunci di sini dan dihitung
dari data **g18–g20** (rencana nyata, sesi sebelumnya), **bukan** dari rencana
G22 (itu set validasi, A3):

```
set kalibrasi C = setiap baris 'torsi RNEA+offset' di g18_trial*.log, g19_trial*.log,
                  g20_trial*.log (rencana PLANNED MAUPUN yang ditolak torsi)
   lengan  = awalan sendi 'terketat' baris itu (RNEA probe hanya memuat sendi lengan perencana)
   target  = <trial>.json 'targets'[lengan]
   rel     = g18: 0.550 (g1);  g19: kolom 2 g19_windows.txt per percobaan (g1);
             g20: g1 0.550, g2 0.000
   awal    = REST (setiap rencana kalibrasi mulai dari REST)
untuk c ∈ C:  s*(c) = solusi IK_V konvergen yang meminimalkan max_j (|τ_j| + offset_j) / lim_j
              Δ_j(c) = RNEA_rencana_j(c) − |τ_j(s*(c))|
m_j = max(0, max_{c ∈ C} Δ_j(c))           untuk j = 1..6, tanpa pembulatan, tanpa pengecualian
```

- `max`, bukan kuantil: sama dengan cara +7.7 dipilih (g21 B8), dan pada oracle
  **terima-palsu lebih mahal dari tolak-palsu** (terima-palsu = jadwal yang
  gagal di (iv), itu G22; tolak-palsu = satu pilihan pose lebih sedikit).
- Target kalibrasi yang IK_V-nya **tidak konvergen** = tolak-palsu oracle pada
  target yang **terbukti** dapat direncanakan. Dilaporkan; dikeluarkan dari
  `max`. Kalau > 5 % dari C, IK dianggap **cacat** dan diperbaiki **sebelum**
  validasi dihitung (C adalah set pengembangan; perbaikan ditulis di §B, set
  validasi tidak disentuh sampai itu).
- Berapa pun `m_2` keluar — termasuk besar — dipakai apa adanya.

### A3. 🔒 Validasi — oracle lawan perencana NYATA pada 49 rencana G22 B1

Set validasi V = setiap baris `task tN arm -> [xyz] @rail R: VERDICT` di
`p1_g22/g22_screen.json` (49; 14 PLANNED / 12 TORQUE-UNSAFE / 23 NO-PLAN).
Dihitung **sekali**, sesudah `m_j` tetap.

Dilaporkan: matriks 2×3 (oracle terima/tolak × PLANNED / TORQUE / NO-PLAN);
per penolakan oracle, sebabnya (L1 / IK_V / TORQ); **presisi** = PLANNED ÷
diterima; **recall** = diterima ÷ PLANNED. Tidak ada ambang lulus — validasi
**tidak** menggerbang pemakaian oracle; saringan (iv) yang menggerbang.

**Kontrol wajib — yang ini MENGGERBANG** (gagal = oracle cacat → diperbaiki,
dihitung ulang, ditulis; bukan dilonggarkan):

| # | Kontrol | Harus |
|---|---|---|
| **NC1** | 49 target V dengan `z := 0.40` (1.55 m di bawah bahu, jangkauan 1.0 m) | **0 / 49** diterima (IK_V) |
| **NC2** | mutan `lim_2 = 5` (< offset 7.7) pada V | **0 / 49** diterima (TORQ) |
| **NC3** | mutan **tanpa** sumbu approach (posisi saja = metode g16 B6) pada V | diterima ≥ oracle; selisihnya dilaporkan. Kalau **sama**, bagian vertikal **tidak menggigit** di V — dilaporkan sebagai itu |
| **NC4** | mutan **tanpa** TORQ pada V | diterima ≥ oracle; selisih pada 12 TORQUE-UNSAFE dilaporkan. Sama = bagian torsi tidak menggigit |
| **NC5** | dihitung dua kali | bit-identik |
| **PC1** | rencana kalibrasi C yang **dieksekusi** | IK_V konvergen (A2: > 5 % gagal = cacat) |

### A4. 🔒 Instance — g22 A2 APA ADANYA, masker `reach` := oracle

```
untuk seed = 0 … 49 (urut):
  base = sched.gen_real(6, seed, 0, (1, 2), maps=data/cap_g{1,2}_rail160.npz)   # TIDAK diubah:
         tugas yang ditarik = tugas G22 seed itu (penarikan menolak dengan L1, di dalam gen_real)
  R0, R1  = make_instance.restrict_rot0 APA ADANYA (p0 0.55 / 0.00)
  reach[g][:, p, slot] := reach[g][:, p, slot] ∧ ORACLE(node, L_p, lengan(g, slot))
  (i)–(iii) g22 A2 apa adanya; (iv) sched_screen APA ADANYA (3/3, --stop-first)
  instance = seed pertama yang lolos (i)–(iv). Cadangan (ii) g22 A2 apa adanya, tidak ada yang lain.
```

- **Tugas tidak ditarik ulang** dengan oracle: `gen_real` tetap, dan tugas yang
  tidak layak-oracle di mana pun jatuh di syarat (i) — seed dilewati, dicatat.
  Ini satu-satunya pembacaan "APA ADANYA" yang tidak menyentuh `sched.py`.
- `zone` dan `hand` **tidak** diubah (n_mr = 0; mutex tidak dipakai oleh jadwal
  yang lolos, g21 B6 1/80).
- **p0**: rel terakhir terbaca 0.550910 / 0.000094 (g22 C), tidak pernah
  diperintah sesudahnya → 0.55 / 0.00. Dicek ulang di tahap 0; kalau
  pembulatan grid berbeda, `make_instance` dijalankan ulang dengan nilai terbaca
  (mekanis) **sebelum** (iv).
- (iv) berhenti di seed pertama yang lolos (`--stop-first`, g22 A2). Laju lolos
  (iv) = lolos ÷ seed yang disaring sampai titik itu.

### A5. 🔒 Perangkat keras — g22 A3–A6 apa adanya; alat yang belum pernah hidup DRY dulu

Kalau ada seed lolos (iv): tahap 1 sisa (`rail_to_g` DRY tiap traverse,
`return_rest` DRY, `run_g22 --dry`), lalu **tahap 2** (rel g2 0.000 → rel
pertama jadwal g2 → 0.000, lengan REST, S25) dan **tahap 3** (jadwal penuh,
satu kali). **Tiap gerak izin operator.** Sebelum tahap 2: lengan harus < 0.5°
dari REST (S12; G22 membacanya 0.47–0.84°) — `return_rest` per gantry, DRY →
izin → `--move`, **tidak** dihitung. Kalau tidak ada seed lolos (iv): tidak ada
gerak, dilaporkan.

### A6. 🔒 Papan skor §7.2 — D79–D90, DITULIS SEBELUM DATA APA PUN

Diturunkan dari mekanisme terukur, bukan intuisi, sejauh bisa (prior tally:
dugaan dari jalur data kode / mekanisme terukur tepat 7/7 di G21).

| # | Dugaan | Dasar |
|---|---|---|
| **D79** | `m_2` (A2) **≤ 1.5 N·m** | vel 0.15: suku dinamis kecil; titik akhir lintasan (v = a = 0) = gravitasi statis, dan lintasan dari REST menggantung naik — puncak `joint_2` biasanya di/dekat tujuan. Selisih terbesar datang dari cabang IK berbeda |
| **D80** | PC1: IK_V konvergen pada **≥ 95 %** target kalibrasi yang dieksekusi | target itu terbukti dapat direncanakan dengan kendala yang sama |
| **D81** | V: oracle menerima **≥ 12 / 14** PLANNED | sama seperti D80, di set lain |
| **D82** | V: oracle menolak **≥ 10 / 12** TORQUE-UNSAFE | RNEA rencana 6.97–9.72 (g22 B1 + 7.7 = 14.67–17.42); statis + `m_2` dekat RNEA bila D79 benar |
| **D83** | V: oracle menolak **≥ 15 / 23** NO-PLAN, dan sebab dominannya **IK_V**, bukan TORQ | g22 B1: NO-PLAN di tepi jangkauan dengan approach vertikal wajib |
| **D84** | presisi V **≥ 0.6** (PLANNED ÷ diterima) | D81–D83 bersama |
| **D85** | NC3 menerima **lebih banyak** dari oracle (bagian vertikal menggigit) | L1 memakai approach ≤ 45°, perencana 2° |
| **D86** | (i)∧(ii)∧(iii) di bawah oracle: **≥ 30 / 50** seed | oracle mengecilkan himpunan pose per tugas → lebih banyak pindah (ii ↑), tetapi sebagian tugas kehilangan semua pose (i ↓); rel 1.6 m memberi banyak pose alternatif |
| **D87** | (iv): seed lolos pertama ditemukan dalam **≤ 10** seed layak pertama (laju ≥ 10 %) | per rencana dugaan ≥ 60 % PLANNED (D88) dengan korelasi kuat per seed |
| **D88** | (iv): per rencana PLANNED **≥ 60 %** (G22: 29 %) | oracle membuang kelas NO-PLAN tepi + TORQUE statis |
| **D89** | (iv): rencana sesudah pindah PLANNED **≥ 50 %** (G22: 0/13) | tugas yang memaksa pindah kini dipilih dari pose yang layak-oracle |
| **D90** | (kalau tahap 3 jalan) makespan terukur **>** P2-serial (Σ_g, FISIK, `T_cmd`) | M3 (+rencana/saringan per tugas) dan overhead perangkat lunak ~76 s per pindah (g21 A1) tidak ada di P2 |

---

## B. Hasil terukur

§A dikunci 14:39 (sha256 teks §A `e41011242377f0c0`). Berkas:
[results/p1_g23/](results/p1_g23/) — [oracle.py](results/p1_g23/oracle.py),
[calib.py](results/p1_g23/calib.py).

### B0. 🔴 Aturan margin A2 CACAT — ditemukan di set PENGEMBANGAN, sebelum validasi disentuh

`calib.py` (aturan A2 persis): C = **87** rencana (g18 23, g19 23, g20 41), IK_V
konvergen **87/87** (PC1: 81/81 yang dieksekusi — D80 TEPAT). Tetapi:

| | Δ min | median | p95 | **max = m_j** |
|---|---|---|---|---|
| joint_2 | −3.27 | +0.70 | +5.25 | **+5.91** |
| joint_3 | −1.89 | −0.05 | +0.42 | **+2.06** |
| j1 / j4 / j5 / j6 | | | | 0.22 / 0.30 / 0.08 / 0 |

Dengan `m_2 = 5.91`, statis `joint_2` harus ≤ 14 − 7.7 − 5.91 = **0.39 N·m**.
Oracle terkunci menerima **0 / 81** rencana yang **terbukti** dieksekusi aman —
ia degeneratif; setiap seed akan mati di (i).

**Sebabnya terbaca, bukan diduga:** tiap target punya dua keluarga cabang IK
vertikal — `joint_2` statis rendah (~0.4–2) dan **tinggi** (~6.4–7.3). Contoh
g20 #10 arm_4 `(−0.264, −0.191, 1.16)`: cabang 1.02 / 1.94 / 6.36 / 6.36 / 6.63 /
6.64, RNEA rencana **6.93**. Perencana mendarat di cabang **tinggi**; `s*` A2
(torsi terendah) memilih cabang **rendah**, jadi Δ = pilihan **cabang**, bukan
suku lintasan. Arah sebaliknya juga ada (g18 #9 arm_1: RNEA 0.15, `s*` 3.15).
Ini juga mekanisme yang masuk akal untuk kelas **stokastik** g17 B1.9 (8/10
PLANNED): cabang mana yang ditemukan OMPL. Dugaan itu **tidak** diuji di sini.

**Keputusan operator (sesudah kunci 14:39, sebelum V):** diperbaiki **sebelum** V dihitung, dilabeli
koreksi pasca-set-pengembangan. Set validasi V (G22) belum disentuh.

### B0.1 🔒 A2′ — DIKUNCI sebelum dihitung, sebelum V

```
C' = rencana C yang DIEKSEKUSI (81; rencana yang ditolak tidak punya konfigurasi akhir terukur)
q_akhir(c) = posisi 6 sendi lengan itu di /joint_states, sampel TERAKHIR <= akhir jendela probe
             (<sess>_windows.txt, kolom akhir sama dengan torque_offsets.py g21: g18 3, g19 5, g20 2);
             lengan masih menahan di target saat probe keluar
   sah bila FK(q_akhir) dalam 5 mm dan 5° dari target (kriteria sukses); kalau tidak: dikeluarkan, dilaporkan
Δ_j(c) = RNEA_rencana_j(c) − |τ_j(q_akhir(c))|        (τ = gravitasi statis, model yang sama)
m_j    = max(0, max_{c ∈ C'} Δ_j(c))                   tanpa pembulatan, dipakai apa adanya
TORQ'  ⇔ SETIAP solusi IK_V konvergen s : ∀ j |τ_j(s)| + offset_j + m_j ≤ lim_j
         (perencana boleh mendarat di cabang mana pun yang ada → semua harus aman)
ORACLE' = L1 ∧ IK_V ∧ TORQ'        (IK_V tetap: ≥ 1 solusi konvergen)
```

A3 (V + NC1–NC5), A4, A5 berlaku untuk ORACLE′ apa adanya. D79 dinilai pada
aturan **terkunci** (A2). NC2 dibaca untuk TORQ′ (lim_2 = 5 → setiap cabang
gagal → 0/49). NC4 = tanpa TORQ′.

**Hasil A2′** ([calib2.py](results/p1_g23/calib2.py) → [g23_calib2.json](results/p1_g23/g23_calib2.json)):
81/81 konfigurasi akhir terukur **sah** (FK < 5 mm / 5° dari target). Δ `joint_2`
median **−0.02**, p95 +0.33, maks **+0.66** — puncak lintasan ≈ statis di titik
akhir (suku lintasan kecil pada vel 0.15, persis dasar D79). Margin terpakai:

| | j1 | **j2** | **j3** | j4 | j5 | j6 |
|---|---|---|---|---|---|---|
| `m_j` (A2′) | 0.220 | **0.659** | **0.817** | 0.163 | 0.068 | 0 |
| ⇒ statis maks yang lolos | 3.18 | **5.64** | **2.58** | 4.86 | 4.95 | 5.02 |

### B1. Validasi — ORACLE′ lawan perencana nyata pada 49 rencana G22 ([validate.py](results/p1_g23/validate.py), [log](results/p1_g23/g23_validate.log))

| ORACLE′ | PLANNED | TORQUE-UNSAFE | NO-PLAN |
|---|---|---|---|
| **terima** | **4** | 1 | **0** |
| tolak | 10 (TORQ′ 8, IK_V 2) | **11** (TORQ′ 10, IK_V 1) | **23** (IK_V 23) |

- **Presisi 4/5 = 0.80**, **recall 4/14 = 0.29**. Oracle **terkunci** A2 (m_2 5.91,
  ∃): terima **0/49** — konsisten dengan B0.
- **NO-PLAN 23/23 ditolak oleh IK_V**: tidak ada solusi approach-vertikal
  (< 2 mm, < 2°) dari 8 benih. IK posisi-saja konvergen pada **13/23** di antaranya
  → sebab NO-PLAN G22 memang approach vertikal di tepi jangkauan (g22 B1),
  sekarang terukur.
- **Recall rendah sebabnya ∀-cabang** (A2′): 8 PLANNED ditolak karena **ada**
  cabang tinggi (mis. s40 t4: cabang j2 0.06 / 5.41 / 5.74 — perencana mendarat
  di cabang aman, oracle menuntut semua aman). Arah tolak-palsu, sesuai A2.
  2 PLANNED ditolak IK_V (s7 t4, s16 t5): IK 8-benih melewatkan solusi yang
  KDL temukan — tolak-palsu IK.
- **Satu terima-palsu:** s44 t5 arm_1 `(0.786, 0.318, 1.32)`: semua cabang
  ditemukan j2 statis ≤ 0.56, rencana nyata RNEA **6.45** → cabang yang tidak
  ditemukan 8 benih, atau puncak di tengah lintasan. **Tidak ditentukan.**

**Kontrol:** NC1 (z = 0.40) **0/49** ✅; NC2 (lim_2 = 5) **0/49** ✅; NC5
bit-identik **49/49** ✅; NC4 (tanpa TORQ′) terima 23 vs 5, pada TORQUE-UNSAFE
**11 vs 1** → bagian torsi menggigit ✅; PC1 81/81 ✅. 🔴 **NC3 tidak sesuai A3:**
posisi-saja **4** < oracle 5 — di bawah TORQ′ ∀-cabang, IK posisi-saja
menemukan **lebih banyak cabang**, dan satu cabang tak aman cukup untuk
menolak. Kontrol itu tidak mengisolasi bagian vertikal; versi terisolasi (IK
saja) di atas: 13/23 vs 0/23 → bagian vertikal menggigit.

⚠️ Batas validasi: satu sampel perencana per rencana (g17 B1.9: vonis
stokastik untuk kelas marginal); 49 rencana, 12 dari seed/lengan yang sama
berulang; tugas ke-2 lengan yang sama tidak mulai dari REST.

**D79–D85 dinilai:**

| # | Dugaan | Terukur | |
|---|---|---|---|
| D79 | `m_2` (A2 terkunci) ≤ 1.5 | **5.91** (cabang, B0) — A2′ 0.66 **tidak** dinilai | ❌ |
| D80 | PC1 ≥ 95 % | **81/81** | ✅ |
| D81 | terima ≥ 12/14 PLANNED | **4/14** (∀-cabang) | ❌ |
| D82 | tolak ≥ 10/12 TORQUE | **11/12** | ✅ |
| D83 | tolak ≥ 15/23 NO-PLAN, sebab IK_V | **23/23, semua IK_V** | ✅ |
| D84 | presisi ≥ 0.6 | **0.80** | ✅ |
| D85 | NC3 terima > oracle | **4 < 5** (tertulis); terisolasi IK 13 vs 0 | ❌ |

### B2. Instance (i)–(iii) — **2 / 50** ([make_instance_g23.py](results/p1_g23/make_instance_g23.py), [log](results/p1_g23/g23_make_instance.log), [g23_candidates.json](results/p1_g23/g23_candidates.json))

Oracle′ benar pada **953 / 38 016** tuple (2.5 %) node-tugas × rel × lengan.
Tugas `gen_real` ditarik dengan L1 (A4), jadi **48/50** seed punya ≥ 1 tugas
tanpa pose layak-oracle′ sama sekali → (i) gagal. Lolos (i)–(iii): **seed 0**
(pindah g1/g2 2/1, FISIK 133.64 s, DINDING 285.64 s) dan **seed 35** (3/2,
194.28 / 422.28 s); keduanya jadwal FISIK = DINDING, konflik None. Masker
diverifikasi: tensor = solve langsung, bit-identik, untuk t4 seed 0.

### B3. Tahap 0 + saringan (iv) — **0 / 2**. Jadwal TIDAK dijalankan (A2).

**Tahap 0** (operator: LED tidak merah, origin g2 = home, rel/ruang bebas —
dicek fisik hari ini): `remount_check` **GERBANG LULUS**, ICMP .10–.13 ✅;
launch `/tmp/g23_t1.log` ([arsip](results/p1_g23/g23_launch.log.gz)), PID
1029258, `SigIgn` tanpa SIGINT ✅; **4×** "Actuator count … '6'"; **7/7**
controller active; nol FAULT / Kortex exception. 7 spawner "process has
died" (G22: 3) = "Failed loading controller" pada controller yang sudah aktif.
Rel **0.550910 / 0.000094**, rotasi 0 / 0 → R1 0.55 / 0.00 tetap. Lengan
0.465–0.838° dari REST (sama G22). URDF cache sha `f02e7c53…` = oracle.
`sched_screen.py` dan `run_g22.py` mendapat `--plan` (+ `--out-dir/--archive/
--prefix` di runner); default tak berubah (A0: jalur saja).

**(iv)** ([g23_screen.log](results/p1_g23/g23_screen.log), [json](results/p1_g23/g23_screen.json)):

| seed | langkah | hasil |
|---|---|---|
| 0 | t2 arm_1 `(1.0, 0.388, 1.32)` @0.55 | PLANNED (RNEA j2 2.74) |
| 0 | retract g1 / traverse g1 0.55 → 0.45 | CLEAR / CLEAR 546.1 mm |
| 0 | t4 arm_1 `(0.571, 0.529, 1.32)` @0.45 | **TORQUE-UNSAFE** — RNEA j2 **7.83** → 15.53 |
| 35 | traverse g1 0.55 → 0.40 (lengan sudah REST, retract dilewati) | CLEAR 546.1 mm |
| 35 | t0 arm_2 `(0.143, 0.600, 1.40)` @0.40 | **TORQUE-UNSAFE** — RNEA j2 **7.72** → 15.42 |

Per rencana **1/3** PLANNED; sesudah pindah **0/2**. Yang menolak lagi hanya
rencana lengan; sapuan S24 3/3 CLEAR.

**Kenapa oracle′ menerima keduanya — terukur.** Solve langsung pada target
yang **dibulatkan 4 desimal** (selisih ≤ 0.03 mm) memunculkan cabang j2 **7.93**
(t4 seed 0) dan **6.68** (t0 seed 35) yang 8 benih tidak temukan pada koordinat
node persis; perencana mendarat tepat di cabang itu (RNEA 7.83 ≈ statis 7.93).
Jadi TORQ′ "∀ cabang" hanya sebaik **enumerasi cabangnya**.
[branch_probe.py](results/p1_g23/branch_probe.py): 300 sampel acak dari 953
tuple oracle′-benar, diselesaikan ulang dengan **64** benih → **90 / 300 = 30 %**
jatuh (cabang tak aman ditemukan). Itu **batas bawah** (64 benih pun belum
tentu jenuh).

**Sebab strukturalnya (dari definisi, bukan data):** roll alat bebas ⇒ 5
kendala untuk 6 sendi ⇒ solusi IK_V per target adalah **keluarga 1-D**
(kurva di ruang sendi, per cabang), bukan himpunan titik. Torsi `joint_2`
berubah sepanjang roll, dan perencana boleh berhenti di roll mana pun. "Semua
cabang" dari 8 benih acak = 8 titik di kurva itu. Terima-palsu s44 (B1) cocok
dengan mekanisme yang sama; itu **tidak** diverifikasi terpisah.

➜ **Klaim yang boleh ditulis:** *oracle L2 statis (IK approach vertikal + torsi
statis di SEMUA solusi yang ditemukan, margin terkalibrasi 0.66 N·m) memprediksi
NO-PLAN 23/23 dan TORQUE-UNSAFE 11/12 pada set validasi, tetapi dengan 8 benih
IK ia melewatkan solusi tak aman pada ≥ 30 % tuple yang diterimanya; kedua
jadwal yang lolos (i)–(iii) ditolak di rencana yang jatuh di cabang itu.*
Makespan terukur **tidak diukur** (dua sesi berturut-turut).

### B4. D86–D90 dinilai — papan skor G23

| # | Dugaan | Terukur | |
|---|---|---|---|
| D86 | (i)–(iii) ≥ 30/50 | **2/50** | ❌ |
| D87 | lolos (iv) dalam ≤ 10 seed layak | **0/2** (hanya 2 yang ada) | ❌ |
| D88 | per rencana PLANNED ≥ 60 % | **1/3** | ❌ |
| D89 | sesudah pindah PLANNED ≥ 50 % | **0/2** | ❌ |
| D90 | makespan > P2-serial | tahap 3 tidak jalan | — tidak dinilai |

**G23: 7 meleset / 4 tepat** (D79 ❌, D80 ✅, D81 ❌, D82 ✅, D83 ✅, D84 ✅,
D85 ❌, D86–D89 ❌). Papan skor **48 meleset / 41 tepat**. Polanya: keempat yang
tepat adalah **penolakan** (NO-PLAN, TORQUE, presisi, konvergensi di target
terbukti) — ditarik dari mekanisme G22 yang sudah diukur. Ketujuh yang meleset
semuanya menduga oracle **lebih menerima** dari kenyataannya (D81, D86–D89),
atau menduga aturan saya sendiri benar (D79, D85) — kategori **kode sendiri**
(prior §7.2: "tebak lebih salah") dan kendala **kelayakan yang sudah terbukti
mengikat** (G22), bukan kendala yang belum diukur.

### B5. Pertentangan §B lawan §A — G23

| # | Pertentangan |
|---|---|
| **(1)** | A2: Δ di cabang torsi-terendah `s*` = pilihan **cabang**, bukan suku lintasan → `m_2` 5.91, oracle terkunci menerima 0/81 rencana yang terbukti aman (B0). Diganti A2′ (keputusan operator, sebelum V) |
| **(2)** | A1 TORQ "∃ solusi" → A2′ "∀ solusi": perencana boleh mendarat di cabang mana pun (B0) |
| **(3)** | A3 NC3 "posisi-saja menerima ≥ oracle": di bawah ∀-cabang, posisi-saja menemukan lebih banyak cabang dan menolak lebih banyak (4 < 5); kontrol tidak mengisolasi bagian vertikal (B1) |
| **(4)** | A1 "8 benih tetap, solusi dikumpulkan" mengandaikan cabang diskret; IK_V dengan roll bebas adalah keluarga 1-D, dan 8 benih melewatkan solusi tak aman pada ≥ 30 % tuple yang diterima (B3) — kedua penolakan (iv) persis ini |
| **(5)** | A4 "tugas tidak ditarik ulang, jatuh di (i)": oracle′ benar pada 2.5 % tuple, sehingga (i) membunuh 48/50 seed — "APA ADANYA" hampir mengosongkan himpunan instance (B2) |
| **(6)** | prompt / g22 B1: "~3×, g17 B1.8" — g17 B1.8 menulis ~2× (A0) |
| **(7)** | g22 B0.3: 3 spawner mati; di sini 7 — kelas yang sama (controller tetap 7/7 active), jumlahnya beda (B3) |

**G23: TUJUH.**

---

## C. Keadaan akhir, dan yang BELUM dikerjakan (Rule 12)

- **Nol gerak perangkat keras.** Lengan menggantung 0.47–0.84° dari REST
  seperti saat bring-up; rel **0.550910 / 0.000094**, rotasi 0.
- Stack dimatikan: `kill -INT 1029258` → keluar 12 s; `ros2 node list
  --no-daemon` **kosong**; nol FAULT / Kortex exception / `INVALID_USER_SESSION`;
  14 baris *deactivate*. Crash dump `move_group` sesi ini (177 MB, 15:10) dihapus;
  dump python 12:28 (bukan milik sesi ini) dibiarkan. Disk **2.0 GB** bebas.
- **Belum pernah hidup, masih:** `rail_to_g.py` (termasuk gerak rel gantry 2
  **pertama**, S25), `run_g22.py` (kini dengan `--plan`), perbaikan probe N = 1.
- **Tidak diukur:** makespan terukur vs prediksi, M1–M3, P1–P4, offset +7.7 pada
  rencana baru yang dieksekusi, D90.
- **Diubah di kode bersama:** `docs/results/p1_g22/{sched_screen,run_g22}.py`
  (argumen jalur). `reach_dwell_probe.py` (G22) masih belum di-commit.
  `sched.py` / peta / penyaring **tidak** diubah.

## D. Prompt sesi berikutnya — G24 (salin ke chat BARU)

**Kenapa ini berikutnya.** Oracle′ sudah benar di dua hal yang G22 gagal
(NO-PLAN 23/23, TORQUE 11/12), tetapi ∀-cabang dengan 8 benih acak bukan "semua
solusi": roll bebas membuat solusi IK_V sebuah kurva, dan ≥ 30 % tuple yang
diterima punya titik tak aman di kurva itu — kedua penolakan (iv) G23 persis di
sana. Perbaikannya **menyapu roll secara eksplisit**, bukan menambah benih.
Kedua, penarikan tugas dengan L1 membuat 48/50 seed mati di (i); apakah tugas
ditarik dari himpunan layak-oracle adalah **keputusan protokol**, dikunci
sebelum dihitung.

**Rekomendasi: Opus 5, effort TINGGI** — kesalahan oracle diam sampai perencana
menolak; G23 sudah dua kali (A2, enumerasi) mendapati aturannya sendiri salah
dengan cara yang tidak tampak di angka ringkasan. Bagian offline bisa **sedang**
setelah §A dikunci.

```
Sesi G24 -- oracle L2 dengan SAPUAN ROLL, lalu (kalau lolos) SATU jadwal end-to-end
di sel NYATA. Repo ceiling_arm, branch feat/rgbd-topo-deploy.

BACA PENUH sebelum menulis kode atau menyentuh hardware:
1. CLAUDE.md (Working Rules; Rule 6 dikecualikan untuk sesi protokol P1, g16 A10).
2. docs/p1_g23_oracle_hw.md -- SELURUHNYA. B0 (margin cabang vs lintasan; A2'),
   B1 (validasi: NO-PLAN 23/23 IK_V, recall 4/14, s44), B3 (0/2; cabang yang
   8 benih lewatkan; 30 % dari 300; roll bebas = kurva 1-D), B5.
3. docs/p1_g22_hw.md A1-A6 (protokol eksekusi, belum pernah dipakai), B1, C.
4. docs/p1_state.md 6, 7, 8b, 8c.
5. docs/results/p1_g23/{oracle,calib2,validate,make_instance_g23,branch_probe}.py.

=== TUGAS ===
1. Tulis docs/p1_g24_*.md A DULU, KUNCI sebelum hitung apa pun:
   a. IK_V per roll: roll alat disapu eksplisit (grid yang ditulis SEKARANG,
      mis. 5 deg) -> pose 6-D penuh per roll -> semua solusi per roll (benih
      ganda, kriteria jenuh ditulis sekarang). TORQ'' = aman di SETIAP roll yang
      punya solusi (perencana boleh berhenti di roll mana pun). Margin A2' TETAP
      (m_2 0.659, m_3 0.817) -- jangan dikalibrasi ulang kecuali aturannya
      dikunci dulu.
   b. Validasi ulang pada V G22 (49) + 3 rencana (iv) G23 + tuple jatuh
      branch_probe; kontrol negatif wajib, termasuk: kedua penolakan (iv) G23
      HARUS ditolak TORQ''.
   c. Keputusan tarikan tugas (operator, SEBELUM hitung): gen_real apa adanya
      (G23: 2/50 lolos (i)) atau tugas ditarik dari node layak-oracle (mengubah
      distribusi instance -- tulis konsekuensinya untuk naskah).
   d. Dugaan D91+ dikunci SEBELUM validasi dan SEBELUM saringan (iv).
2. Kalau ada seed lolos (iv): protokol G22 A3-A6 + tahap 2 (rel g2 keluar-
   kembali, S25) + tahap 3, tiap gerak izin operator. rail_to_g / run_g22
   BELUM PERNAH hidup -- DRY dulu (run_g22 --plan --out-dir --archive --prefix).
3. Perbarui p1_state.md 6 + 8c + tally 9.

=== ATURAN ===
- 7.1 ground truth dulu; 7.2 UKUR, dugaan dikunci sebelum data.
- DILARANG memilih instance / t_fold / margin / grid roll sesudah melihat hasil.
- Kalau B bertentangan dengan A, B menang dan DITULIS. G22 tujuh, G23 tujuh.
- Penghitung proses: JANGAN cocok dengan cmdline bash sendiri (pgrep -f cocok
  dengan bash yang menjalankannya -- terjadi lagi di G23).
- Launch: python3 -c "...signal.SIG_DFL...execvp" &, cek SigIgn sebelum kill -INT.
- Disk ~2 GB: crash dump move_group ~180 MB tiap shutdown, hapus milikmu.
```
