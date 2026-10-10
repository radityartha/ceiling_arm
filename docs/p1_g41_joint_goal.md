# P1 / G41 — tujuan tugas = SENDI cabang oracle terpilih + retract akhir gantry (OFFLINE + mock)

> Sesi 2026-10-10 (sesudah G40). NOL gerak HW; mock `ROS_DOMAIN_ID=77`. Instans = 20 set node G38 (R35, p0 g1 PARKIR
> 1.50, g2 0). G40e: (i)–(iii) 6/20, traverse-lingkungan 0/8, plan-only k = 3 **1/6** (seed 8); gagal sisa: lintas-gantry
> lengan 3/7, pulang-lurus diblok rak 3/7, torsi transit 1/7. **§A ditulis dan DIKUNCI SEBELUM pengukuran** (salinan
> `docs/results/p1_g41/g41_A_locked.md` + sha256). §B menang atas §A dan DITULIS. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

- Jadwal G40e 6 seed (`g40e_candidates.json`): tugas ganda satu lengan per stop di seed 0 (g1 stop 1, g2 stop 0), 10
  (g1 stop 0), 19 (g2 stop 0, 2). `g31_screen.events`: retract hanya sebelum traverse; tugas berurutan satu lengan di stop
  yang sama mulai dari ujung tugas sebelumnya; semua event g1 lalu semua event g2 (serial; `run_g32` memakai daftar yang
  sama) — **tidak ada retract sesudah tugas terakhir g1**.
- Log plan-only G40/G40e (B5 G40). `_plan_and_screen` menerima tujuan sendi dict (`_joint_goal`, toleransi 0.2°).
- Pilz PTP (garis sendi lurus) dimuat default oleh `MoveItConfigsBuilder`, tetapi `joint_limits.yaml` 30/32 sendi
  `has_acceleration_limits: false` → Pilz tidak dapat dipakai tanpa mengubah konfigurasi; **tidak** dipakai. Tugas tetap
  OMPL (pipeline default) dengan tujuan sendi; garis lurus di predikat = PROKSI untuk lintasan OMPL (lihat A2).
- Tidak ada perhitungan baru sebelum mengunci (hanya baca kode + jadwal + log).

### A1. 🔒 Predikat tuple (c41) dan tujuan sendi

Tuple (node, g, li, ri, s) layak ⟺ L1 ∧ pose (a) G40 ∧ oracle‴ tepat `torq2` ok (∀ cabang, seperti G40e; tabel G39 TIDAK
dipakai) ∧ **∃ cabang** q ∈ `O2.solve(..., keep_solutions=True)['solutions']` (semua roll), dievaluasi berurutan menurut
max_i |q_i − REST_i| naik (seri = indeks), dan **cabang terpilih = yang PERTAMA lolos semua**:

- (e1) titik akhir = G40 env2: `EnvChecker.check(only=t{g}_)` ≥ 50 mm ∧ bebas-kotak (lambung `t{g}_` vs `column_boxes`).
- (e2) garis pulang lurus `return_rest._rest_line(q)` (60 titik kosinus; dibalik = transit REST → q), titik q ikut,
  lengan pasangan REST, gantry lain TIDAK dimasukkan: `EnvChecker.screen_trajectory` = CLEAR ∧ `InterArmChecker(gantry g)
  .screen_trajectory` = CLEAR (= dua dari tiga penyaring `screen_retract`/`plan_retract`) ∧ bebas-kotak di setiap titik.
- (e3) torsi sepanjang garis: ∀ titik, `O2.gravity(arm, q_pt, lin, rot)` + OFFSET + MARGIN ≤ LIM (aturan `torq2`).

Tujuan tugas = {`{prefix}joint_i`: q_i} cabang terpilih, disimpan di jadwal (`goal` per tugas). Yang tidak berubah: (a), (b)
traverse = G40 (cache tepi G40 disalin, tepi baru ditambahkan ke salinan G41), DP/klaim eksak = G40 A2 dengan (c) →
(c41). (c41) ⊆ (c40e) ⇒ seed yang (i)-nya gagal di G40e tetap gagal (tetap dijalankan, 20 seed).

### A2. 🔒 Perubahan runner/penyaring (plan-only)

- `g31_screen.events(..., final_retract)`: bila True, event `retract` gantry g ditambahkan SESUDAH tugas terakhir g
  (sebelum event gantry berikutnya). `g31_screen.walk`: tugas memakai `ev['goal']` (dict sendi) bila jadwal punya `goal`,
  selain itu pose seperti dulu. Default kedua perubahan = perilaku lama (run_g32/varian lama identik).
- **Celah yang diketahui (TIDAK eksak):** OMPL boleh tidak mengikuti garis lurus (transit torsi/env diperiksa ulang oleh
  penyaring di lintasan sebenarnya, retry 3); tugas ke-2+ satu lengan di stop sama mulai dari ujung tugas sebelumnya, bukan
  REST (titik akhirnya tetap terverifikasi (e1)–(e3)); pasangan lengan di predikat = REST, di runner bisa terjulur (tugas
  lengan lain di stop sama); lintas-gantry di luar (c41). Runner HW (`reach_dwell_probe` CLI `--target`) belum menerima
  tujuan sendi → G41 tidak dapat dieksekusi di HW apa adanya.

### A3. 🔒 Pengukuran berurutan

1. **M1 (permintaan 2, OFFLINE, sebelum jadwal ulang)**: pada setiap (seed, sampel) plan-only G40/G40e yang mencapai akhir
   event g1 (jsonl), keadaan akhir g1 = ujung rencana tugas terakhir g1; (M1a) `return_rest.plan_retract(1, [arm_1,
   arm_2], ..., plan_fn=None)` lurus = CLEAR?; (M1b) semua traverse g2 jadwal itu `sweep_rot(RotCrossChecker)` dengan g1
   REST di pose akhirnya. **Aturan keputusan:** M1 menutup lintas-gantry ⟺ untuk KETIGA gagal lintas-gantry (G40 s8,
   G40e s3, s18) M1a CLEAR ∧ M1b semua CLEAR. Tertutup ⇒ G41 memakai `--final-retract`; tidak ⇒ kendala lintas-gantry di
   penjadwal dirancang dan DITULIS di §B sebelum dijalankan.
2. **K**: (K1) (e2) implementasi G41 vs `plan_retract` lurus pada ≥ 30 keadaan ujung tugas tersimpan (jsonl G40/G40e):
   verdict + min se-gantry dan lingkungan identik (|Δ| ≤ 0.1 mm). (K2) (e3) di titik q = `O2.gravity` (bit-identik dengan
   `taumax` komponen bila q adalah cabang pengikat — cukup: torsi titik q sama dengan `gravity(q)` dipanggil langsung).
   (K3) (c41) pada keadaan ujung yang G40e tolak RETRACT-BLOCKED tidak dapat diulang (rencana ditolak tidak disimpan) —
   diganti: (c41) pada 4 tuple gagal non-lintas G40/G40e (s0 t3, s10 t5, s19 t3 ×2) harus menerima dengan cabang yang
   garisnya CLEAR, atau menolak (dilaporkan).
3. **Jadwal ulang** 20 seed (kriteria (i)–(iii) G38 A4) → `g41_candidates.json`; (iv′) offline untuk tiap seed lolos:
   traverse serial lintas-gantry seperti M1b (g2 vs g1 REST di pose akhir; g1 vs g2 REST di p0) — dilaporkan, tidak
   dipakai untuk optimasi ulang.
4. **Plan-only mock** `g31_screen.py --variant G41 --repeats 3 [--final-retract]` pada semua seed lolos (i)–(iii).
   Lolos ⟺ 3/3 PLANNED. Atribusi = verdict terakhir. Diagnostik: deviasi maks lintasan OMPL dari garis REST → q.

### A4. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D310 | M1 menutup **3/3** gagal lintas-gantry (retract akhir lurus CLEAR dan traverse g2 CLEAR vs g1 REST). |
| D311 | Dari tuple yang G40e verifikasi layak (ok3 ∧ env2) dan dihitung ulang di G41, (c41) menerima **60–90 %**. |
| D312 | (i)–(iii) G41 **4–6 / 20** (≤ 6 pasti, ⊆ G40e). |
| D313 | Makespan median vs G40e (seed lolos keduanya) naik **≤ 5 %**. |
| D314 | Plan-only mock 3/3 PLANNED **≥ 3** seed. |
| D315 | RETRACT-BLOCKED di plan-only = **0** (titik akhir tugas deterministik dan terverifikasi). |
| D316 | TORQUE-UNSAFE transit di plan-only **≤ 1** kejadian. |
| D317 | Biaya (c41) per tuple median **≤ 10 s** CPU. |

## B. Hasil terukur

### B0. M1 (A3.1) — [log](results/p1_g41/g41_m1.log), [JSON](results/p1_g41/g41_m1.json)

**3/3 gagal lintas-gantry TERTUTUP** (G40 s8, G40e s3, s18): retract lurus g1 sesudah tugas terakhirnya CLEAR, semua
traverse g2 CLEAR vs g1 REST di pose akhir (min 106.5 mm, s18 (0.10, +30) → (0.70, −20); di G40e −14.9 mm). Semua
sampel yang mencapai event g2: **17/17** tertutup. ⇒ aturan keputusan: G41 memakai `--final-retract`.

### B1. K (A3.2) — [log](results/p1_g41/g41_k.log)

- **K1:** 40/40 keadaan ujung (pasangan REST) verdict + min env/se-gantry identik dengan `plan_retract` (|Δ| ≤ 0.1 mm).
  Lemah: ke-40 semuanya CLEAR (rencana tersimpan sudah lolos retract_check G36), tidak ada contoh tolak.
- **K2:** torsi titik q bit-identik. **K3:** G40e s0 t3 dan s10 t5 DITERIMA (cabang roll 24 / 36, 169° dari REST;
  89 / 88 cabang lebih dekat ditolak garis-env / garis-kotak); s19 t3 (G40 + G40e) DITOLAK (358/358 cabang: garis-kotak
  298, garis-env 60).
- **Penyimpangan kerja (bukan hasil):** biaya pertama 117–468 s CPU/tuple, 77 % di `InterArmChecker.screen_trajectory`
  yang saya panggil sebelum cek env gagal. Urutan dibuat torsi → env → kotak → se-gantry dengan keluar di gagal pertama
  (konjungsi: cabang terpilih sama; diuji ulang pada 2 tuple K3: hasil identik, 28 s / 98 s).

### B2. Jadwal ulang (A3.3) — [log](results/p1_g41/g41_run.log), [JSON](results/p1_g41/g41_candidates.json)

- **(i)–(iii) 6/20** (seed 0, 3, 8, 10, 18, 19 = himpunan G40e). Makespan = G40e **persis** di 5 seed (stop bisa beda:
  s0 g2 2 stop), s19 **+0.6 %** (381.71 vs 379.43; tuple t3 ditolak). 400 tuple dihitung, 350 ok41; tepi 5 686.
- D311: (c41) menerima **272/274** tuple layak-G40e (99.3 %). D317: CPU/tuple median 16.7 s, p90 34.7, maks 252.
  Cabang terpilih median 79.7° dari REST.
- **(iv′)** traverse serial lintas-gantry: **6/6** CLEAR — [log](results/p1_g41/g41_iv.log).

### B3. Plan-only mock k = 3 `--final-retract` — [log](results/p1_g41/g41_planonly.log), [JSON](results/p1_g41/g41_planonly.json)

Mock domain 77 (launch 2773817, SIG_DFL, SigIgn `0x1001001`, galat spawner = baseline G32/G34).

- **5/6 LOLOS 3/3** (seed 0, 3, 8, 10, 18; G40e: 1/6). Retract akhir g1/g2: CLEAR (lurus) semua. RETRACT-BLOCKED **0**,
  TORQUE-UNSAFE **0**, lintas-gantry **0**.
- **s19 ditolak:** t4 arm_3 @ (0.90, −15) NO-PLAN ×3 (0.6 s). t4 = tugas ke-2 arm_3 di stop itu, berangkat dari ujung t3.
  Tujuan oracle t3 (tuple (1951, 2, 18, 33, 0)) **0.97 mm** dari kotak tumbuh (ujung MoveIt 0.38 mm, deviasi 0.198° =
  toleransi tujuan); move_group: "Invalid states at index [1 2] out of 112", kontak `env_static_map` ↔
  `t2_a1_{left,right}_finger_dist_link`. Dua celah: (1) predikat kotak `> 0` TANPA margin — 0.38–0.97 mm jatuh di pita
  tak-pasti K3 G40 (MoveIt-tabrak ≤ −0.34, MoveIt-bebas ≥ +0.85 mm), melanggar MARGIN = TOLAK (kesalahan saya, A1);
  (2) transit tugas ke-2 satu lengan (ujung t3 → t4) tidak diverifikasi (celah A2 yang sudah diungkap).
- **Diagnostik deviasi** ([log](results/p1_g41/g41_dev.log)): lintasan OMPL vs garis sendi lurus start → tujuan:
  median 0.1°, maks **0.3°** (95/95 ≤ 5°) — garis lurus di predikat praktis = lintasan MoveIt.

### B4. Varian G41b (B menang atas A, DITULIS sebelum dijalankan)

- **Margin kotak** `BOX_MIN` = **10 mm** di ketiga pemakaian kotak: pose (a) (dari `d_box` tabel G40, tanpa hitung ulang),
  (e1) titik akhir, (e2) setiap titik garis. Alasan: > 10× pita tak-pasti K3 dan > perpindahan ujung jari akibat
  toleransi tujuan 0.2° pada lengan 1 m (3.5 mm). Cache: penolakan G40 / G41 tetap penolakan (G41b lebih ketat).
- **(iv″) offline** untuk tiap seed lolos: garis sendi lurus antar-tugas berurutan satu lengan di stop yang sama
  (q_prev → q_next, pasangan di keadaan runner) — env, kotak ≥ BOX_MIN, se-gantry, torsi. Dilaporkan, tidak dipakai untuk
  optimasi. **Kontrol positif:** (iv″) pada rantai G41 s19 t3 → t4 harus menolak.
- Plan-only mock k = 3 `--final-retract` pada semua seed lolos (i)–(iii). Dugaan tambahan (sebelum mengukur): **D318**
  G41b (i)–(iii) ≥ 5/20; **D319** plan-only G41b ≥ 5 seed 3/3; **D320** (iv″) memprediksi verdict plan-only tugas ke-2+
  dengan benar di semua seed.

### B5. Hasil G41b — [run](results/p1_g41/g41b_run.log), [JSON](results/p1_g41/g41b_candidates.json), [plan-only](results/p1_g41/g41b_planonly.log)

- **Penyimpangan kerja (bukan hasil), kesalahan saya:** run G41b pertama membaca `d_box = None` (tidak ada kotak di
  dekat = ∞, bebas) sebagai tak layak → pose (a) g1 5/442; dihentikan pada seed 1, tidak ada tuple dihitung
  ([log INVALID](results/p1_g41/g41b_run1_posebug_INVALID.log)). Diperbaiki (None = bebas), diulang.
- Pose (a) dengan margin 10 mm: R35 g1 **438** (G40 442), g2 **372** (G40 379).
- **(i)–(iii) 5/20** (0, 3, 8, 10, 18); s19 kini (i) GAGAL (tugas 3 tanpa tuple dengan kotak ≥ 10 mm). Makespan = G40e
  persis di kelimanya; stop identik dengan G41, 3 tujuan sendi berganti cabang. 403 tuple, 5 720 tepi.
- (iv′) **5/5**, (iv″) **5/5** ([log](results/p1_g41/g41b_chain.log)). Kontrol positif (iv″) pada G41: menolak tepat s19 t4
  (garis ujung t3 → t4 menembus kotak −6.51 mm) dan menerima 5 seed lain → memprediksi plan-only G41 **6/6** benar
  ([log](results/p1_g41/g41_chain.log)).
- **Plan-only mock k = 3 `--final-retract`: 5/5 LOLOS 3/3** — 90/90 rencana tugas PLANNED pada percobaan PERTAMA (G41:
  95/98), waktu rencana median 4.5 s (maks 12.3); deviasi OMPL dari garis lurus maks 0.3° ([log](results/p1_g41/g41b_dev.log)).
  Mati bersih (launch SIGINT keluar, `node list --no-daemon` 0).

### B6. Papan skor D310–D320

| # | Dugaan | Hasil |
|---|---|---|
| D310 | M1 menutup 3/3 lintas-gantry | ✓ **3/3** (17/17 sampel) |
| D311 | (c41) menerima 60–90 % tuple layak-G40e | ✗ **99.3 %** (272/274) — lebih longgar dari dugaan |
| D312 | (i)–(iii) 4–6/20 | ✓ G41 **6/20** (G41b 5/20) |
| D313 | makespan median naik ≤ 5 % | ✓ **0.0 %** (G41 s19 +0.6 %) |
| D314 | plan-only 3/3 ≥ 3 seed | ✓ G41 **5/6**, G41b **5/5** |
| D315 | RETRACT-BLOCKED = 0 | ✓ **0** (G41 + G41b) |
| D316 | TORQUE-UNSAFE transit ≤ 1 | ✓ **0** |
| D317 | CPU (c41) median ≤ 10 s | ✗ **16.7 s** (p90 34.7, maks 252) |
| D318 | G41b (i)–(iii) ≥ 5/20 | ✓ **5/20** |
| D319 | plan-only G41b ≥ 5 seed 3/3 | ✓ **5/5** |
| D320 | (iv″) memprediksi verdict tugas ke-2+ dengan benar | ✓ 5/5 (dan 6/6 retrospektif di G41) |

**Tally G41: 9 tepat / 2 meleset.**

## C. Keadaan akhir + langkah berikut (Rule 12)

- **Nol gerak HW.** Mock domain 77 naik 11:3x, mati bersih. Kode repo diubah: `g31_screen.py` (`events(...,
  final_retract)`, tujuan sendi `ev['goal']`, flag `--final-retract`; default = perilaku lama, `run_g32` tidak berubah).
  Baru: `results/p1_g41/g41_make.py`, `g41_m1.py`, `g41_dev.py`.
- **Kesimpulan:** tujuan tugas = sendi cabang oracle terpilih + retract akhir gantry menutup ketiga kelas gagal G40e
  (lintas-gantry 3/7 → 0, pulang-lurus 3/7 → 0, torsi transit 1/7 → 0) **tanpa biaya makespan**; plan-only 1/6 → 5/6
  (G41) → **5/5** (G41b). Lintasan OMPL ke tujuan sendi = garis lurus (≤ 0.3°), jadi predikat garis praktis eksak. Sisa
  tunggal G41 (s19) = transit tugas ke-2 dari ujung tugas-1 di 0.4 mm dari kotak; G41b (margin kotak 10 mm) mengeluarkan
  seed itu dari jadwal dan (iv″) offline memprediksi plan-only 11/11 seed benar.
- **Masih terbuka:** (1) 14/20 seed (i) gagal secara struktural (tugas tengah: rotasi aman-torsi mendorong pasangan REST
  ke rak, G40 B3) — tidak tersentuh G41; (2) (iv″) hanya dilaporkan, belum menjadi kendala penjadwal (tugas berurutan satu
  lengan); (3) **runner HW belum bisa tujuan sendi**: `reach_dwell_probe` CLI hanya `--target xyz`, `run_g32` memanggil
  `events()` tanpa `final_retract`; (4) tabel G39 tetap tidak dipakai; (5) biaya (c41) 17 s/tuple median.
- **HW TETAP TERTUTUP:** gerbang rotasi > 10° (HW baru ±10°, G30), origin g2 −35 mm (cek kamera), penyaring
  Fix-Start-State (G37), dan kini juga dukungan tujuan sendi + `final_retract` di runner HW.
- Rule 6: anggaran token sesi (30 k) terlampaui jauh (baca 6 berkas penuh sudah > 30 k) — DITULIS.
