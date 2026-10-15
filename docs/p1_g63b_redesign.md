# P1 / G63b — desain ulang target kalibrasi ekstrinsik D455 (papan catur polos, dudukan di badan gripper)

> Sesi 2026-10-15 (prompt [p1_g63b_redesign_prompt.md](p1_g63b_redesign_prompt.md)): luring + mock domain 77 (izin operator),
> NOL gerak HW, NOL kamera. Lanjutan G63 ([p1_g63_charuco.md](p1_g63_charuco.md) §B1–B3: HW P0 0 observasi).
> **§A DIKUNCI 2026-10-15 malam** atas persetujuan operator (A6) → `results/p1_g63/g63b/g63b_A_locked.md` + sha256.
> Runbook HW: [results/p1_g63/g63b/g63b_runbook.md](results/p1_g63/g63b/g63b_runbook.md). Data/alat: [results/p1_g63/g63b/](results/p1_g63/g63b/).

Tujuan tetap sama dengan G63: ekstrinsik baru kedua D455 (T_wc, 6 DOF) dari sudut papan yang dibawa lengan, acuan = FK sendi
terukur; gerbang G1–G8 G63 kecuali yang diulas di §A4. Yang berubah: **target**, **dudukan**, **lengan per fase**, **detektor**.

## A. Protokol — DIKUNCI 2026-10-15

### A0. Keputusan operator 2026-10-15 (harfiah)

| # | Pertanyaan | Jawaban operator | Konsekuensi |
|---|---|---|---|
| A0.1 | Ukuran cetak maks | **"maksimal cuma bisa cetak A4"** | Satu lembar A4 per papan; sambung lembar tidak dipakai (studi: 2×A4 kotak 70 mm tidak lebih baik, §A8.3). |
| A0.2 | Dudukan | **"ini pake kardus kaku juga kok aman"** → sesudah penjelasan (yang bergerak = sendi jari 1.4°, bukan kardus): **"a"** = kardus kaku di **badan gripper**, bukan jari | Rantai FK tidak lewat sendi jari; G7 (jari ≤ 0.5°) tetap sebagai gerbang keselamatan. Operator: "saya yakin yang sebelumnya tidak geser, saya erat banget pasangnya" — dicatat; data B1.10 G63 = sendi jari membuka, bukan lepas tempel. |
| A0.3 | Massa maks | **"150 gr aman"** | Model torsi memakai 150 g (anggaran) sampai ditimbang; massa terukur menggantikan. |
| A0.4 | Dua sisi | **"2 papan … saya pasang di 2 sisi arm_2. setelah beres kita gunakan di arm_3. jadi papannya tetap 2"** | Rakitan = DUA papan punggung-ke-punggung di badan gripper; fase 1 arm_2, fase 2 rakitan dipindah ke arm_3. Identitas muka dari FK (normal berlawanan, satu kamera tak bisa melihat keduanya). |
| A0.5 | Jumlah gerak | **"a"** = P0 R0 + 3 pose per lengan, lalu bertahap | Sama dengan G63 §A6.2. |
| A0.6 | Jari arm_2 −0.0284 | **"a"** = ukur ulang sekali di bring-up; bila tetap → acuan K0 di `g63_k0`, tercatat | `scripts/k0_check.py` tidak diubah. |
| A0.7 | Ukuran kotak | opsi **"b"** (4×5 @ 42 mm, satu A4, tepi 21 mm) — "bisa tidak?" → bisa (§A8.4) | PDF `g63b/boards/` (raster 42.00 × 41.99 mm). |
| A0.8 | Papan lama di sel | operator **memindahkan ChArUco lama dekat arm_2** (2026-10-15) | Sisa (atas lemari laci, meja, lantai kiri) diminta disingkirkan; penjaga detektor tetap wajib (§A3). |

### A1. Model

Sama dengan G63 §A1 kecuali: papan b ∈ {muka 1, muka 2} dari SATU rakitan per lengan; T_lb rakitan (6 DOF) relatif
`gripper_base_link` = gangguan. **Solver: tiap muka punya T_lb 6-DOF SENDIRI** (nama A2/B2 fase arm_2, A3/B3 fase arm_3;
12 kamera + 24 muka = 36 parameter); T_12 (rot y 180°, asal (4·kotak, 0, 0.070 m terukur)) hanya untuk PRIOR deteksi dan
model tabrakan, tidak dipaksakan di solver (tak bergantung pada kekakuan rakitan). Ukuran kotak = 42.0 mm (operator: cetak
"aman"); angka mm terukur menggantikan bila diberikan di S0.

### A2. Urutan (tiap gerak nyata = satu izin)

Rinci = runbook. S0 (tanpa gerak): preflight, **uji goyang rakitan oleh operator (≤ 1–2 mm)**, papan lama di luar bingkai,
ukuran kotak. S1 bring-up (`octomap_updaters:=false`, `enable_gantry_bridge:=true`, `reach_dwell_monitor` WAJIB hidup), K0 +
jari arm_2 ukur ulang (A0.6), **lepas rakitan → g1 1.50 → 1.30 → pasang lagi** (REST + rakitan tabrak kotak meja MoveIt di
1.40/1.50, §A8.6/A8.9); g1 tetap **1.30 untuk KEDUA fase**. S2 fase arm_2 (P0 → T1 → T2 → T3, tiap tahap plan-only berantai dulu).
S3 pindah rakitan ke arm_3 (attach B, payload fase arm_3), ulang tahap. S4–S6: solve gabungan (`g63b_solve`), gerbang, akhir:
lepas rakitan → baru `return_rest` boleh → g1 1.50 PARK → K0.
**Pulang REST selama papan terpasang hanya via `probe_g63`** (A8.10 G63).

### A3. Deteksi (aturan buang DIDAFTARKAN sebelum data)

Per frame, per kamera, per muka yang menurut FK menghadap kamera (sudut ≤ 70°):
1. ROI = proyeksi pusat muka (FK + incumbent) ± (setengah diagonal + tepi + 0.18 m + 0.05 m) — di luar ROI diabaikan.
2. `findChessboardCornersSB` 3×4, EXHAUSTIVE|ACCURACY; grid ditolak → hull ×1.6 ditutup, ulang maks 3×.
3. Penjaga (semua wajib): residu homografi 12 sudut ≤ **1.0 px** RMS **dan ≤ 1.5 px per sudut** (ditambah luring 10-15
   sesudah e2e seed 4, sebelum data HW — hanya lebih ketat); pelabelan = arah muka (det J dari FK-muka) + paritas
   20 kotak, |skor| ≥ **0.5**; **cincin putih**: 18 titik di tengah tepi 21 mm semua > (hitam+putih)/2.
4. < 12 sudut = frame ditolak (papan polos: semua atau tidak sama sekali). Median 30 frame per pose; std dicatat.
5. Identitas muka: muka hanya dicari bila PRIOR-nya menghadap kamera (det J homografi prior > 0) dan seluruh prior di dalam
   bingkai; muka membelakangi → tidak dicari (mock: 100 % sesuai rencana pose).
Tidak ada pelonggaran sesudah data; frame P0 lama (`hw_20261015b/cap/frames`) = data pengembangan saja.

### A4. Gerbang (ulasan G1–G8)

G1 tetap (≥ 15 FIT/kamera, dua fase/lengan ada). G2 tetap; T_lb ≤ 0.20 m dari `gripper_base_link`. G3 tetap (1.5/3.0/2.0 cm).
G4′ tetap (2.0 cm); G4 lama dilaporkan bila ada pose dua-kamera (kini mungkin: muka berbeda). G5 tetap. G6′ tetap (2 mm).
G7 tetap (MOVED+SUCCESS, torsi ≤ 14, 0 fault, **jari ≤ 0.5°** — kini harus lulus karena jari tidak dibebani). G8 tetap.
**Baru — G9 (diam, A6.1):** sendi 1–3, 5–6 std < 2e-4 rad (tetap); **j4 peak-to-peak ≤ 4.58e-3 rad** (= gerak papan ≤ 1.5 mm
pada tuas terbesar 0.328 m di semua pose terkunci, `j4_limit.json`); drift selama tangkap dinilai tanpa j4.
**Baru — G10 (solve sah):** `g63b_solve` menolak bila rank Jacobian < parameter, residu < 2 × parameter, atau std kamera
(σ ≥ 0.3 px) di atas batas G63 (0.10° / 5 mm) — P0 DIHARAPKAN ditolak (cek proses, bukan kalibrasi).

### A5. Pose (TERKUNCI: `g63b/poses_g63b_v4.json`)

`g63b_poses.py` (= `g63_poses.py` + dua muka + R0 ≤ 40°): visibilitas incumbent per muka, z pusat 0.80–1.40, jarak 0.6–2.8 m,
sudut ≤ 55°, torsi statis ≤ 92 % (150 g, terukur 87 g), tabrakan (rakitan = satu kotak +10 mm), cek retract, oklusi; g1 1.30,
g2 0; seed 63, 8000 sampel. **R0 dilihat ≤ 40° di setiap kamera** (A6.3): arm_2 R0 rgbd2 30.8° / 1.83 m + rgbd 38.5° / 1.16 m;
arm_3 R0 rgbd2 38.8° / 1.14 m + rgbd 31.2° / 1.96 m. Urutan 21 pose per lengan (P0 3 / T1 6 / T2 6 / T3 6), uji-tahan tiap
ke-4 per kategori kamera. Aturan j4 berbasis torsi TIDAK dipakai (tak layak, §A8.8).

### A6. Jawaban operator untuk kunci (2026-10-15 malam, harfiah)

Operator: **"ya saya ikut yang terbaik saja . setuju rekomendasimu. selesaikan semua proses luring sehingga besok tinggal online HW"**
→ rekomendasi diterima:

| # | Butir | Keputusan |
|---|---|---|
| 1 | Aturan diam j4 | (ii): j4 boleh berburu bila gerak papan ≤ 1.5 mm pp → G9 |
| 2 | Rel | g1 **1.30** (kedua fase), digeser TANPA rakitan |
| 3 | R0 | sudut ≤ **40°** di setiap kamera yang melihat R0 → pose v4 |
| 4 | Uji goyang | **belum ada hasil** → gerbang S0 runbook (operator, sebelum gerak pertama; gagal = kencangkan dulu) |
| 5 | Lembar | A4 penuh (operator: "A4 penuh"; usul potong tidak dipakai — model memakai 297 mm) |
| 6 | Meja | operator menawarkan memindah meja → **tidak** (peta env beku memuat meja; pindah = pindai ulang = fase sendiri) |

### A6b. Cadangan JARI (operator 2026-10-15 malam, harfiah)

**"siapkan di jari sekarang. jika besok uji getar gagal, gagal tinggal pakai yang jari. kl jari butuh 1 saja di 1 sisi seperti
charucho tdi cukup kan?"** → jawaban: cukup untuk G1 (FIT gabungan rgbd 17 / rgbd2 17 di `poses_finger_v1.json`), tanpa pose
dua-kamera. **Pemicu:** uji goyang S0 gagal SESUDAH diperkuat (cable tie kedua + ganjal), atau keputusan operator.
**Susunan = G63:** papan A (polos, satu muka) di jari arm_2 → rgbd2 di REST, papan B di jari arm_3 → rgbd, dipasang BERSAMAAN,
link model `tool_frame` (mount `g63b/mount_finger.json` = `mount.json` G63 nyata + asal pola baru; lembar DIASUMSIKAN di tempat &
arah sama dengan ChArUco 10-15 — dicek operator), 43.5 g/papan, **g1 1.40** (G63 A8.13; rencana rel `g63_rail_plan.json`), g2 0.
Alat: `g63_plancheck.py` / `g63_move.py` / `g63_attach` (via `g63b_attach --boards A B`, purge) apa adanya; tangkap
`g63b_capture.py --arm finger` (detektor polos, aturan j4 sama); solve `g63b_solve.py` (papan AJ/BJ). Gerbang = §A4, KECUALI:
**G7 jari ≤ 0.5° tidak bisa dijamin** (P0 G63: jari membuka 1.4° di bawah beban, ±5 mm di papan; model `tool_frame` tidak
mengikuti sendi jari) → bila mode jari dipakai, G7-jari dinilai per blok dan **pelanggaran = berhenti + tanya operator**
(tidak dilonggarkan diam-diam); G6′ R0 tiap blok menangkap pergeseran yang berkembang. Risiko diterima operator dengan memilih
cadangan ini dicatat harfiah saat dipakai.

### A7. Dugaan (dikunci bersama §A)

| # | Dugaan |
|---|---|
| D639 | Papan polos 42 mm terdeteksi (12/12 sudut) pada ≥ 80 % pose terencana per kamera (P0 + tahap), 0 label salah pada pemeriksaan visual semua overlay. |
| D640 | Galat sudut median ≤ 0.5 px pada pose ≤ 2.0 m (std 30 frame). |
| D641 | Jari arm_2/arm_3 berubah ≤ 0.5° sepanjang blok (dudukan di badan gripper) — G7 lulus. |
| D642 | Dengan aturan j4, 0 capture ditolak karena "tidak diam" di P0. |
| D643 | ≥ 1 pose per lengan terlihat kedua kamera (muka berbeda) di daftar terkunci. |
| D644 | Torsi j2 terukur puncak ≤ 13 N·m dengan rakitan (massa terukur) pada semua pose terpilih. |
| D645 | rgbd2 = kamera pembatas: galat rotasi ekstrinsik rgbd2 > rgbd (e2e luring: rgbd2 0.05–0.16°, rgbd 0.04–0.13°). |
| D632–D635, D638 | dibawa dari G63 tanpa perubahan. |

### A8. Temuan luring 2026-10-15

1. **Kalibrasi blur dari kegagalan P0 nyata** (`g63b/plain_study.py`): papan ChArUco lama dirender di atas frame P0 nyata
   (noise temporal terukur 2.3) — blur 1.3–1.6 px mereproduksi marker ≈ 2–7/10 @1.04 m dan 0 @1.44 m (nyata 4/10, 0/10).
2. **Distraktor = bahaya utama:** tanpa penjaga, papan polos memberi **427 label salah** (sub-kisi papan ChArUco lama di
   ruangan / di lengan lain, termasuk grid campuran papan baru + papan lama). Dengan ROI prior + residu H + cincin putih:
   **0 salah** di semua kandidat × cetak (hitam 30/60, putih 230/200) × blur (1.3/2.0) × jarak 0.8–3.0 × miring 0–55°.
3. Kandidat: 3×4 @ 65 mm GUGUR (2×3 sudut < minimum SB); 50 mm tepi 5 mm → cincin tak bisa dicek (ditolak hampir semua);
   2×A4 70 mm tidak lebih baik (papan besar lebih sering menumpuk distraktor).
4. **4×5 @ 42 mm, satu A4, tepi 21 mm**: deteksi 146–178/240 (d ≤ 2.5 m) di latar P0 penuh distraktor (latar polos ≈ 12/12),
   lateral p95 ≤ 1.8 mm s.d. 3 m/55°. PDF diverifikasi raster.
5. Urutan sudut SB sembarang (bisa tercermin/terbalik) → pelabelan WAJIB dari arah muka + paritas (bug alat sendiri
   tertangkap di sintetis sebelum data).
6. **Dudukan TEBAKAN v1 → v2** (`g63b/mount_g63b_guess.json`): lembar mulai di flens → kotak menembus `upper_wrist` −37.5 mm;
   lembar mulai z_g 0.045 → +2.5 mm. **g1 1.40: REST arm_2 + rakitan 49.7 mm dari meja + kotak MoveIt TABRAK** (tidak
   bergantung posisi lembar); **g1 1.30 bersih (111.7 mm)**. Lembar menempel di badan gripper hanya z_g 0.045–0.078 (33 mm) →
   kekakuan dudukan harus dicek operator.
7. **Pose v2** (`g63b/poses_g63b_guess_v2_j4_0.json`, 8000 sampel, g1 1.30, dudukan TEBAKAN, seed 63): arm_2 **163** kandidat
   lolos semua saringan (dari 547 torsi-ok: tabrakan 444, retract 251, oklusi 163), arm_3 **370**; masing-masing R0 + 21 pose.
   **Pose dua-kamera kini ada: arm_2 9, arm_3 8** (R0 keduanya dua-kamera; muka berbeda per kamera) — G63 = 0. Kamera × muka
   seimbang (7–9 per sel). Torsi statis maks 91.7 / 91.9 % (≤ 92 %), jarak kamera 0.88–2.25 m. Plan-check mock BELUM (menunggu
   dudukan nyata; alat `g63_plancheck` pakai `--goal-joints` + `--extra-env`).
8. **Aturan j4 berbasis torsi TIDAK layak:** τ_j4 gravitasi statis di kandidat p5–p95 = **0.009–0.418 N·m** (link distal ringan) —
   ambang 1.5 N·m menolak semua (varian dihentikan, tanpa hasil). Gerak papan akibat j4 berburu pp 3.2e-3 rad (nilai P0) pada
   pose terpilih: **≤ 1.14 mm / ≤ 0.80 px** pp (median 0.75–1.05 mm) → usul (ii) di §A6.1.
9. **Dudukan NYATA v3** (operator 2026-10-15 sore, foto `1–5.jpeg` di root — lokal, TIDAK di-commit): rakitan di arm_2, cable tie
   atas + pita lakban bawah, sejajar **70 mm** kertas-ke-kertas, tepi atas lembar **78 mm di atas pangkal jari** (z_g −0.008),
   A4 penuh, massa 87 g (+8 g cadangan lakban/tie tak ditimbang → 0.095), A → rgbd2, B → rgbd (cocok model REST). Versi pertama
   (hanya cable tie atas) membentuk V 75/115 mm = engsel → lendutan bergantung pose TIDAK tertangkap G6′ → operator menambah
   pita bawah. **Uji goyang (≤ 1–2 mm) belum dikonfirmasi.** `mount_g63b.json`: REST arm_2 g1 1.50 model 35.9 mm (+10 mm kotak)
   vs terukur 65 mm ke meja (model konservatif ≈ 2 cm); g1 1.40 kotak MoveIt TABRAK; **g1 1.30 bersih 120 mm**. Kotak tunggal
   beririsan housing pergelangan secara model (−50 mm) → `g63b_attach.py` menambah `upper_wrist_link` sebagai touch link
   (hanya papan↔lengan sendiri; env/lengan lain tetap penuh). Pose v3 (`run_poses_v3.sh`, `poses_g63b_v3.json`): arm_2 **171**
   kandidat lolos; arm_3 berjalan saat sesi dijeda.
10. **Alat HW + uji ujung-ke-ujung LURING** (tanpa ROS): `g63b_detect.py` (detektor produksi = fungsi `plain_study` tervalidasi
   + muka dipilih dari handedness prior + residu H per sudut ≤ 1.5 px), `g63b_capture.py` (g63_capture + detektor polos +
   rakitan dua muka di satu lengan + nama papan per fase A2/B2/A3/B3), `g63b_solve.py` (g63_solve atas gabungan dua fase).
   `g63b_e2e.py`: pose v3 → render dua muka (lembar A4, latar = frame P0 nyata penuh distraktor, blur 1.3, derau 2.3, 5 frame)
   → deteksi dengan prior incumbent + mount nominal → solve. Kebenaran: kamera digeser 2° / 8 cm, rakitan 3° / 1.5 cm per fase,
   galat FK sistematis per konfigurasi 2 mm / 0.05° + ulang 0.3 mm / 0.01°. **8 seed: label salah 0 (semua); ekstrinsik pulih
   rgbd 0.04–0.11° / 0.1–2.2 mm, rgbd2 0.05–0.16° / 0.5–5.4 mm; G1 FIT 19–23 per kamera; G3 median 0.10–0.29 cm; G4′ ≤ 0.7 cm;
   G6′ tanpa geser ≤ 1.9 mm.** Toleransi uji (0.15° / 5 mm, spesifikasi T3 G63) terlewati tipis di 2/8 seed — keduanya rgbd2
   (0.153° / 4.0 mm; 0.156° / 5.4 mm) → **6/8 LULUS**; batas TIDAK dilonggarkan (D645). Dampak di zona jauh (≈ 3 m): 0.16° ≈ 8 mm
   + 5 mm ≈ 1.3 cm < anggaran demo 5 cm; G3 uji-tahan (pose dekat zona pose) tidak melihat galat zona jauh ini → dicatat.
   **Kontrol positif G6′:** rakitan digeser 3 mm sebelum R0 akhir → 2.88–3.34 mm > 2 mm → ditolak (4/4). Temuan + perbaikan
   selama uji: (a) model uji v1 menarik ulang derau FK 2 mm per kunjungan → alarm palsu G6′ 4.1 mm (salah model uji, bukan alat);
   (b) satu sudut 3.4 px lolos penjaga RMS 1.0 px → penjaga per sudut 1.5 px. Prior muka B wrapper tangkap = model e2e
   (beda 1.7e-16); `g63b_solve` dua berkas = solve e2e (identik). **Belum diuji:** `g63b_capture capture` dengan ROS (hanya di
   mock/HW), aturan diam j4 (keputusan §A6.1).
11. **MOCK domain 77 (izin operator 2026-10-15 "ya boleh"; fake HW, BUKAN data; `octomap_updaters:=false`):**
   (a) **Gladi urutan rel HW:** lepas rakitan dari scene → `return_rest` (per gantry) → `pose_to_g --gantry 1 --plan
   g63b_rail_plan.json --variant G63b 1.30 0` DRY (S13 menerima stop, sapuan CLEAR 380 mm, env 86.6 mm) → `--move` (galat 0.00 mm,
   lengan 0.000°) → `g63b_attach --boards A` OK. (b) **Plan-check berantai arm_2** (`g63b_plancheck.py --board A`, sendi-tujuan,
   `--extra-env` S0c, payload fase arm_2 = rakitan A saja, touch link pergelangan): **27/27 PLANNED** (R0 → P0 → T1 → T2 → T3 →
   REST), margin torsi maks **94.97 %** (j2, A-02), 0 penolakan penjaga 97 %, env min **65.3 mm**. Jebakan: tanpa
   `reach_dwell_monitor` probe = INVALID (bukan lulus) — runbook wajib monitor. (c) **E2E ROS** (`g63b_fakecam` papan polos dari
   TF mock + `probe_g63 --move` + `g63b_capture` + merge + `g63b_solve`), P0 arm_2 5 pose: gerak 5/5 MOVED, deteksi 12 sudut std
   ≤ 0.06 px pada muka yang menghadap; muka membelakangi ditolak dari prior; pemetaan muka = rencana pose; merge A→A2, B→B2.
   R0 rgbd muka B (miring 50°) ditolak penjaga per sudut (1.57–1.61 px) → **usul: R0 dipilih ≤ 40° di kamera mana pun**.
   (d) 🔴 **Cacat `g63_solve` (G63, terkunci, tidak diubah):** 5 pose P0 → 4 observasi FIT, fit pas-sempurna → σ² = 0 → std 0 →
   "tak teramati: []" padahal kamera meleset **9.8 m**. `g63b_solve` kini menambah penjaga (Jacobian numerik rank penuh, residu ≥
   2 × parameter, std dengan σ ≥ 0.3 px): mock P0 → **DITOLAK** (rank 22 < 24); e2e luring dua fase → OK (std 0.025° / 0.6 mm).
   **Akibat untuk HW: P0 = cek PROSES saja; solve bermakna mulai T1+** (sesuai §A3 G63). Mock dimatikan bersih (domain 77 kosong).
12. **Pose v4 + aturan final (sesudah A6), mock 77:** `poses_g63b_v4.json` (= v3 kandidat, R0 ≤ 40°): calon R0 ≤ 40° arm_2 79
   (dua-kamera 1), arm_3 197 (dua-kamera 4); R0 kedua lengan dua-kamera ≤ 38.8°. **Batas j4** (`g63b_j4_limit.py`): tuas maks
   0.328 m (B-19, rgbd2) → **pp ≤ 4.58e-3 rad**; kait `g63_capture.STILL_EXEMPT_PP` (default kosong = G63 sama) + uji unit
   `test_still.py` 5/5 (j4 3.2e-3 diterima, 6.9e-3 ditolak, sendi lain 5e-4 ditolak). **Plan-check v4:** arm_2 **27/27**
   (margin maks 95.80 %, env ≥ 59.7 mm, 0 tolak-97); arm_3 **26/27** (margin maks 95.72 %, env ≥ 108.8 mm) — **B-02 DITOLAK sah**
   (transit: rakitan 41 mm dari forearm arm_4 < 50 mm; MoveIt buta, pasangan dimatikan SRDF) → di HW dilewati, tidak diganti.
   🔴 **Bug ditemukan di gladi pindah rakitan:** `g63_attach --remove` hanya *detach* → MoveIt menyisakan kotak sebagai objek
   DUNIA di posisi terakhir → percobaan pertama arm_3 **0/27** (START-JUMP dengan gripper arm_2; arsip
   `mock/v4_pc_arm_3_STALEWORLD*`). `g63b_attach --remove` kini juga menghapus objek dunia + memverifikasi scene bersih
   ("sisa [] -> OK"); ulang = 26/27. Di HW bug ini akan memblokir seluruh fase arm_3.
13. **Cadangan JARI di mock 77** (`mock/run_finger_mock.sh`): pose `poses_finger_v1.json` (satu muka/lengan, g1 1.40, R0 ≤ 40°):
   arm_2 104 / arm_3 227 kandidat, R0 satu kamera (arm_2 rgbd 37.3° / 1.55 m; arm_3 rgbd2 38.1° / 1.49 m), FIT gabungan rgbd 17 /
   rgbd2 17, torsi statis ≤ 91.9 %. Rel g1 → 1.40 (rencana G63) OK; attach A+B di `tool_frame` OK. **Plan-check arm_2 27/27 (margin
   maks 93.49 %, env ≥ 52.1 mm), arm_3 27/27 (92.25 %, ≥ 77.2 mm)**, masing-masing 1 percobaan ditolak penjaga 97 % sebelum lintasan
   aman. Uji tangkap ROS (`g63b_capture --arm finger`, fakecam polos): A-R0 rgbd 12 sudut, B-R0 rgbd2 12 sudut (kamera = rencana),
   merge AJ/BJ, REST via `g63_move`; mock mati bersih.
