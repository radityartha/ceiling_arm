## A. Protokol — ditulis 2026-09-23 SESUDAH A0, SEBELUM uji diagnosis apa pun (langkah 3)

> 🔒 Dikunci sebelum satu pun nilai T0–T3 dihitung. §B menang atas §A dan pertentangannya ditulis.
> **DILARANG** memilih/mengubah aturan oracle sesudah melihat hasilnya pada S16. Aturan koreksi per
> hipotesis dikunci **di sini**; hipotesis mana yang diadopsi diputuskan oleh uji mekanisme A2 (ambang
> terkunci), **bukan** oleh skor aturan pada S16. Skor pada S16 dilaporkan sebagai **dalam-sampel**.

### A1. 🔒 Hipotesis bersaing

Fakta pengikat (A0): RNEA rencana TORQUE 6.43–8.14 > o_j2 5.37–5.63 untuk tuple yang **sama**.
Titik akhir lintasan berparameter waktu diam (v = 0) → RNEA di titik akhir ≈ gravitasi statis.
Maka puncak RNEA > 6.3 berasal dari **(E)** titik akhir di luar himpunan yang oracle‴ enumerasi,
atau **(N)** titik **bukan-akhir** (lintasan / dinamika).

| # | Hipotesis | Kelas | Ramalan pembeda |
|---|---|---|---|
| **H1** | Newton-miring tak konvergen (g24 B′2): titik akhir miring ≤ 2° dengan j2 > 5.64 terlewat | E | kontinuasi miring yang kokoh menaikkan amplop j2 tuple S16 ke ≥ RNEA TORQUE |
| **H2** | Cabang IK terlewat (benih / grid roll): titik akhir tepat-vertikal dengan j2 > 5.64 tidak ditemukan | E | pencarian lebih rapat (benih ×4, roll 1°) menemukan solusi j2 ≥ 6.3 pada target tepat |
| **H3** | Margin m₂ (puncak lintasan − statis akhir) dikalibrasi di z < 1.40; di z = 1.40 **lintasan** dari REST melewati konfigurasi j2 jauh di atas titik akhir | N | maks Ω (A2-T0) < RNEA TORQUE; lintasan proksi dari REST menembus 6.3 jauh lebih sering di z = 1.40 |
| **H4a** | Toleransi posisi 2 mm perencana (oracle di posisi tepat) | E | hanya relaksasi posisi yang membawa maks Ω ke ≥ 6.3 |
| **H4b** | Model: RNEA probe (model penuh, sendi lain `neutral`) ≠ gravitasi oracle (model tereduksi, lengan lain REST) | — | selisih statis di konfigurasi yang sama > 0.01 N·m |

Hipotesis tidak saling eksklusif; A2 menilai masing-masing.

### A2. 🔒 Uji pembeda (offline, pinocchio atas URDF G23 sha `f02e7c53…`, tanpa ROS)

**Himpunan uji S12** = 12 tuple S16 (A0). **Kontrol lapis** = tuple-ok cache di z = 1.32 dan
z ≤ 1.24: 12 tuple per lapis z ∈ {1.00, 1.08, 1.16, 1.24, 1.32}, dipilih `default_rng(27)` dari
tuple-ok cache lapis itu yang **pernah** direncanakan PLANNED (A0) bila ≥ 12, dilengkapi acak
dari tuple-ok lapis itu.

- **T-model (H4b):** 50 konfigurasi acak (`default_rng(27)`) × 4 lengan: |τ₂ probe-model statis −
  τ₂ oracle| maks. H4b benar ⇔ > 0.01.
- **T0 (E vs N) — maks gravitasi j2 atas daerah tujuan perencana.** Ω(tuple) = {q dalam batas:
  ‖p(q) − p_t‖ ≤ 2 mm, ∠(z_alat, −z) ≤ **2.83°** (kerucut yang **memuat** kotak x/y 2°), roll bebas}.
  `G_Ω` = maks |τ₂| atas Ω, SLSQP (scipy) multi-start dari **setiap** solusi oracle‴ tuple itu
  (grid roll, tanpa miring) + 64 benih acak `default_rng(27)` diproyeksikan Newton 6-D (A1 g24,
  roll acak). Ditambah `G_0` (posisi tepat, tepat vertikal, roll bebas — daerah oracle‴ tanpa miring)
  dan `G_t` (posisi tepat, kerucut 2.83°).
  - Per rencana TORQUE `c`: **N-terbukti** ⇔ `G_Ω(tuple) < RNEA(c) − m₂` (m₂ = 0.659 = seluruh selisih puncak − statis-akhir
    teramati di C′, termasuk suku dinamis titik akhir); **E-mungkin** sebaliknya.
  - E-mungkin → atribusi: `G_0 ≥ RNEA − m₂` → **H2**; lain `G_t ≥ RNEA − m₂` → **H1**; lain → **H4a**.
- **T1 (H1 langsung):** per solusi oracle‴ tuple S12, 8 miring A′1 dengan **kontinuasi**
  (0.5° → 1° → 1.5° → 2°, Newton 50 iterasi tiap langkah dari langkah sebelumnya). Dilaporkan:
  miring gagal lama → kini konvergen, dan amplop j2 baru − lama.
- **T2 (H2 langsung):** `oracle2.solve` dengan `ROUNDS = (16,16,32,64,128,256)`, minimal 4 ronde,
  grid roll **1°**, tilt+envelope (A′). Dilaporkan: n_sol, amplop j2 baru − lama.
- **T3 (H3, proksi lintasan).** Proksi = **garis lurus ruang-sendi** REST → q_akhir, 101 titik,
  maks gravitasi statis per sendi `P_j`.
  - **T3a (validasi proksi pada C′, set pengembangan):** 81 rencana dieksekusi, q_akhir terukur:
    laporkan `RNEA₂ − P₂` (median, maks) dan `RNEA₂ − statis_akhir₂` untuk perbandingan.
    Proksi **LAYAK** ⇔ maks (RNEA₂ − P₂) ≤ maks Δ_final (0.659) (proksi tidak memperburuk margin).
  - **T3b:** untuk tiap tuple S12 + kontrol lapis: `P₂` atas **setiap** solusi oracle‴ (grid roll,
    tanpa miring). Laporkan per lapis z: fraksi solusi dengan `P₂ + 7.7 + m₂ᵖ > 14`, `m₂ᵖ` dari
    A3-H3. Dan per rencana S16: apakah `[min P₂, maks P₂]` tuple itu memuat RNEA rencana (PLANNED
    **dan** TORQUE).

**Aturan adopsi (terkunci):**

- **H3 DIADOPSI** ⇔ N-terbukti pada **≥ 6 / 11** rencana TORQUE S16 **dan** T3a LAYAK.
- **H1 DIADOPSI** ⇔ T0 atribusi H1 pada ≥ 3/11, **atau** T1 menaikkan amplop j2 > 0.30 pada ≥ 3/12 tuple.
- **H2 DIADOPSI** ⇔ T0 atribusi H2 pada ≥ 3/11, **atau** T2 menaikkan amplop j2 > 0.30 pada ≥ 3/12 tuple.
- **H4a DIADOPSI** ⇔ atribusi H4a ≥ 3/11. **H4b** ⇔ T-model > 0.01 (bug → diperbaiki dulu, semua diulang).
- Lebih dari satu diadopsi → aturannya **digabung** (konjungsi). Tidak satu pun → oracle **tidak**
  diubah; laporan "tidak ditentukan", G28 = kumpulkan lintasan (A6).

### A3. 🔒 Aturan koreksi per hipotesis (oracle⁗) — dikunci SEKARANG

Semua: oracle‴ apa adanya (grid 5°, benih, toleransi, margin A2′, tilt+envelope) **ditambah**:

- **H3 → PATH:** tuple ditolak kecuali **∀ solusi oracle‴ s (grid roll, tanpa miring)**:
  `∀ j: P_j(REST → s) + offset_j + m_jᵖ ≤ lim_j`, dengan
  `m_jᵖ = max(0, max_{c ∈ C′} (RNEA_j(c) − P_j(REST → q_akhir(c))))` (aturan A2′: maks, tanpa
  pembulatan). ∀-cabang, sama dengan TORQ′. Tidak dimodelkan: start non-REST (tugas ke-2 lengan
  yang sama) — tetap syarat PERLU.
- **H1 → MIRING-KONTINUASI:** amplop A′1 dihitung dengan kontinuasi T1; miring yang tetap gagal
  sesudah kontinuasi **dihitung** dan tuple ditolak bila ada solusi dengan gagal miring **dan**
  j2 statis ≥ batas − 0.5 (titik amplop yang terlewat di dekat batas).
- **H2 → BENIH/ROLL:** T2 (ROUNDS diperluas, roll 1°) menggantikan penemuan g24.
- **H4a → POSISI:** tiap solusi + 6 perpindahan target ±2 mm (±x, ±y, ±z), Newton 6-D 50 iterasi,
  masuk amplop.

**Kontrol yang MENGGERBANG oracle⁗:**

| # | Kontrol | Harus |
|---|---|---|
| **K0** | `oracle2.solve(tilt=True, envelope=True)` default pada **72** tuple cache (60 `default_rng(27)` + 12 S12) | **bit-identik** dengan `g24_oracle3_cache.jsonl` (n_sol, n_roll, rounds, saturated, taumax, tilt_fail, ok) |
| **K1** | oracle⁗ ⊆ oracle‴ (monoton) pada semua tuple yang dihitung | 0 pelanggaran |
| **K2** | oracle⁗ pada 145 rencana diterima-z<1.40 (A0) | menerima **≥ 50 %** tuple distinct-nya (kalau di bawah: aturan terlalu ketat — **dilaporkan**, tidak dilonggarkan) |
| **K3** | oracle⁗ pada C′ (81 dieksekusi aman) | dilaporkan (tuple C′ di rel/node peta bila ada) |

### A4. 🔒 Set VALIDASI — tidak dipakai untuk memilih; prediksi terkunci untuk G28 (plan-only)

**V28** = (a) **setiap** tuple-ok oracle‴ di z = 1.40 dalam cache yang **tidak** pernah direncanakan
(53 − 12 − tuple lain yang pernah direncanakan), dan (b) 20 tuple-ok oracle‴ di z = 1.32 yang tidak
pernah direncanakan, `default_rng(28)`. Per tuple, oracle⁗ memberi terima/tolak **sekarang**
(kalau oracle diubah). G28 merencanakan tiap tuple plan-only **3×** dari REST (rel di L, lengan lain
REST, `_plan_and_screen` apa adanya). Prediksi terkunci: oracle⁗ **tolak** ⇒ ≥ 1/3 TORQUE-UNSAFE;
oracle⁗ **terima** ⇒ 3/3 PLANNED. Presisi/recall oracle⁗ di V28 = hasil G28.

### A5. 🔒 Prediksi laju lolos (iv) — ditulis di §B bila oracle diubah

(a) 46 seed (i)–(iii) G26, jadwal P1′ apa adanya: seed yang **setiap** tuple (tugas, rel, lengan)
jadwalnya diterima oracle⁗ → prediksi lolos (iv). (b) tugas tetap, masker `reach` := oracle⁗,
P1′ di-solve ulang: (i)–(iii) dihitung ulang. Tarik-ulang tugas dari node layak-oracle⁗ (g24 A5)
= G28, bukan sesi ini (butuh oracle⁗ pada node baru; dicatat).

### A6. 🔒 Langkah 4 (sekunder) — P1″, dikunci SEBELUM fit

```
P1'' = P1' (g25_constants.json APA ADANYA) kecuali:
  traverse → c_rovh'' + r · T_cmd(Δ),  r = 0.95017 TETAP;  c_rovh'' = median_i (wall_i − r · T_cmd,i)
             atas traverse jadwal rail_to_g yang dieksekusi (G24b 2 + G26 12 = 14)
  retract dilewati (keberangkatan pertama, lengan REST) → c_skip = median wall retract-dilewati terukur
  c_task, c_ret TIDAK diubah.
Evaluasi: leave-one-seed-out atas 4 seed (G24b s1, G26 s1/s2/s13): konstanta dari 3 seed lain,
prediksi makespan seed ke-4; dilaporkan galat P1' vs P1''.
```

### A7. 🔒 Dugaan D123–D131 — DITULIS SEBELUM T0–T3

Prior (tally): dugaan dari **mekanisme terukur** tepat; tentang **kode/aturan sendiri** meleset;
"kendala lebih longgar" meleset di G26 (D115).

| # | Dugaan | Dasar |
|---|---|---|
| **D123** | T-model: selisih ≤ 1e-6 (H4b salah) | g24 B0 model tereduksi = penuh 1.8e-15; `neutral` untuk jari = acuan oracle |
| **D124** | T0: N-terbukti pada **≥ 8 / 11** rencana TORQUE S16 (H3 diadopsi) | RNEA bimodal pada tuple sama (A0); o_j2 sudah memuat miring sudut; tilt_fail tidak terkonsentrasi di 1.40; C′ Δ_final naik dengan z |
| **D125** | T2: amplop j2 naik ≤ 0.30 pada **≥ 11 / 12** tuple S12 (H2 salah) | g24 NC7 90/90, PC2′ 80/81; PC3 22/22 |
| **D126** | T1: kontinuasi memulihkan ≥ 50 % miring gagal; amplop naik ≤ 0.30 pada ≥ 11/12 (H1 salah) | torsi ≈ linear dalam miring kecil (A′1); sudut kotak sudah dihitung |
| **D127** | T3a LAYAK: maks (RNEA₂ − P₂) di C′ ≤ 0.659 | titik akhir termasuk dalam garis lurus → P₂ ≥ statis akhir |
| **D128** | T3b: fraksi solusi dengan proksi-lintasan tak aman di z = 1.40 **≥ 3×** fraksi di z ≤ 1.24 | lapis teratas = lengan harus naik melewati bahu mendatar dari REST menggantung |
| **D129** | oracle⁗ (bila H3): tuple-ok z = 1.40 tersisa **≤ 15 / 53** | median o_j2 z = 1.40 sudah 5.61 (A0) |
| **D130** | oracle⁗ K2: menerima **≥ 70 %** tuple distinct diterima-z<1.40 | lintasan di z < 1.40 tidak menembus (145/145 aman) |
| **D131** | P1″ LOSO: \|galat\| makespan rerata 4 seed **< 4 %** (P1′: 5.6 %) | bias −23 s/seed ada di suku yang dikalibrasi ulang (g26 B2) |
