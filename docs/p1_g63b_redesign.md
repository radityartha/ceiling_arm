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

## B. Hasil terukur

### B1. Sesi HW 2026-10-16 (`results/p1_g63/g63b/hw_20261016/`; jawaban operator harfiah di `s0_operator.md`)

1. **S0:** sha §A 8/8 OK; domain 0/77/79/80 kosong; kamera rgbd 241122302297 / rgbd2 234222303079 FW 5.17.0.10; arm .10–.13 ping
   OK; disk 24 GB bebas. Uji goyang: "insya Allah aman" → konfirmasi "a. ya sudah" (≤ 1–2 mm, kembali). Papan lama: "sudah"
   (🔴 satu ChArUco lama masih di lantai kiri-bawah frame rgbd sampai sesudah P0 → disingkirkan; penjaga menahan, 0 label salah).
   Kotak 42.0 mm.
2. **§B menang — rakitan TIDAK bisa dilepas lagi** (operator: "saya tidak bisa lepas2 lagi papannya") → tiga deviasi §A2/runbook
   atas izin operator: (a) rel g1 1.50 → 1.30 DENGAN rakitan (sweep luring `sweep_rail.log`: jarak rakitan–env model naik
   monoton 35.9 → 120.1 mm, kotak MoveIt bersih mulai 1.37; nyata 65 mm di 1.50) — "ya geser saja aman"; (b) fase arm_3 ditunda
   ("arm 3 kita pikir nanti"); (c) parkir akhir g1 **1.30**, bukan 1.50 (REST + rakitan di 1.50 = 35.9 mm model < margin 50).
3. **K0 awal TOLAK:** arm_2 j3 −0.889°, j5 +0.687° (malam: +0.068/+0.019) — operator "nyala matikan tadi malam dan sempet kegeser
   arena pemasangan". Jari arm_2 kini **−0.0004** (bukan −0.0284 → A0.6 tak perlu). **Jalan buntu di 1.50:** `pose_to_g` S12 tolak
   (0.889° > 0.5) dan `g63b_move --pose REST` START-JUMP (start sudah menabrak kotak env MoveIt karena rakitan). **Override larangan
   `return_rest` (operator "a. ya izinkan")** khusus arm_2, dengan cek luring rakitan di lintasan lurus yang sama (`rest_vs_now.log`:
   31.0 → 35.9 mm, monoton menjauh): galat 0.89 → 0.018°, torsi 3.32. K0 LULUS. Rel: lepas scene (`sisa [] -> OK`), DRY S12/S23/S28
   712 mm/G33 115 mm CLEAR → MOVE 1.30041 m (+0.41 mm, 10.6 s, lengan ≤ 0.13°, torsi 2.52). K0 1.30 LULUS. Live URDF = acuan 10-11.
4. **Plan-only P0 berantai 6/6 PLANNED** (margin maks 94.26 %, env ≥ 65.3 mm).
5. **P0 arm_2:** gerak 5/5 MOVED + SUCCESS (R0, A-01, A-02, A-03, R0), torsi puncak **12.00** (A-01) / 10.31 / 6.96 / 5.52 / 10.40;
   pulang REST 7.63 (rencana 1 ditolak GOAL-MISMATCH 3.71 mm di tahap RENCANA, rencana 2 bersih). 0 fault/REJECTED.
6. **🔴 j6 berburu (bukan j4):** A-01 tangkap rc 2 dua kali — j6 std 1.1e-3, pp 3.0e-3 rad (τ gravitasi ≈ 0.02 N·m); sendi lain
   std ~5e-6. Tuas j6 → sudut papan **54.7 mm** semua pose (`j6_lever_check.py`) → 0.16 mm papan. **Deviasi G9 (operator "b saja"):**
   `g63b_capture` membebaskan j6 dengan angka j4 (pp ≤ 4.58e-3 rad ≈ 0.25 mm papan). Sesudahnya tangkap 4/4.
7. **Deteksi:** terencana 6/8 = 75 % (hilang: rgbd muka B di A-02 46°/1.49 m dan R0#2); +1 tak terencana (A-03 rgbd2 B 12/12);
   std sudut ≤ 0.18 px; overlay visual semua deteksi: **0 label salah**. Jari arm_2 −0.0004…−0.0006 rad sepanjang blok.
8. **Solve P0: penjaga G10 TOLAK** (rank 22 < 24) — sesuai dugaan.
9. **🔴 Dudukan bergeser di P0 (G6′ manual, `g6p_manual.py`, PnP rgbd2 muka A, gerak FK dikurangkan):** R0#2 vs R0#1 **5.31 mm /
   1.59°**, R0#3 (tanpa gerak sejak #2) 5.03 mm / 1.52° → perubahan PERMANEN antara R0#1 dan R0#2 (A-01…A-03), bukan derau.
   Bersamaan: rgbd muka B pada pose R0 yang sama turun **30/30 → 12/30 → 11/30** frame lolos (`redetect.py`; gagal = residu H
   3–5 px → lembar B tidak datar lagi). Inspeksi operator: "tidak terlihat ada masalah" → **STOP** (pilihan b): penyebab tak
   terlihat = akan berulang; G6′ (2 mm) tidak mungkin lulus dengan dudukan ini.
10. **Akhir:** arm_2 REST via `g63b_move`, K0 1.30 LULUS (arm_2 0.156°), g1 **1.30** (rakitan masih terpasang di arm_2),
    g2 0. Shutdown SIGINT per PGID, domain 0/77/79/80 kosong. `rec_health` rgbd 71 389 frame min 0.646, rgbd2 71 457 min 0.767,
    **0 < 10 %**. Frame/raw/overlay (ada orang) lokal, tidak di-commit; joint_states .csv.gz tidak di-commit.

### B2. Papan skor (✓/✗ harfiah; hanya P0 arm_2 yang berjalan)

| # | Hasil | Bukti |
|---|---|---|
| D639 | ✗ | 6/8 = 75 % < 80 % (rgbd muka B); 0 label salah ✓ |
| D640 | ✓ | std sudut maks 0.18 px (≤ 1.94 m) |
| D641 | ✓ (P0) | jari arm_2 berubah ≤ 0.0002 rad (0.01°) |
| D642 | ✗ | 2 tangkap ditolak "tidak diam" — j6, bukan j4 (aturan j4 tidak cukup) |
| D643 | ✓ | daftar terkunci arm_2 8 pose dua-kamera; terwujud R0#1 dan A-03 |
| D644 | ✓ (P0) | torsi puncak 12.00 ≤ 13 |
| D645 | tidak dinilai | tidak ada solve sah |
| D632–D635 | tidak teruji | tidak ada solve sah |
| D638 | ✓ | 0 frame < 10 % (kedua kamera) |
| (G6′) | indikasi ✗ | manual 5.3 mm / 1.6° > 2 mm (bukan gerbang resmi: solve ditolak G10) |

**Langkah berikut G63b (luring):** dudukan rakitan tidak kaku pada beban P0 (geser ~5 mm/1.5° + lembar B melengkung) → rancang ulang
dudukan (mis. pelat kaku/akrilik, klem mekanis ke badan gripper, bukan lakban) + uji kekakuan terukur (R0 → pose torsi tinggi → R0
dengan kamera, G6′ ≤ 2 mm) SEBELUM blok kalibrasi; aturan diam j6 masuk §A baru; rakitan masih di arm_2 di g1 1.30.

### B3. Lanjutan HW 2026-10-16 siang — CADANGAN JARI (§A6b; `results/p1_g63/g63b/hw_20261016_finger/`)

1. **Pemicu (operator):** "sudah kita pindah ke gripper saja, sulit kl disini posisinya" → rakitan badan dilepas; gripper ditutup
   lewat Kortex (arm_2 1.004, arm_3 0.963 = nilai 10-15); papan A polos di jari arm_2, B di jari arm_3 (foto `arm2.jpeg`/`arm3.jpeg`
   di root, lokal; latar foto = latar kamera tujuan; klip kertas di 4 sudut). Rel g1 1.30 → **1.40** (stack hidup kembali; operator
   "geser sekrang arm 2. aman saya jamin"; DRY CLEAR, galat −0.00 mm). `g63_k0` jari tertutup LULUS. Payload `mount_finger.json`, attach A+B OK.
2. **Interaksi:** operator minta persetujuan lewat tombol (AskUserQuestion) → izin per BLOK dengan stop otomatis
   (`run_block.sh`: DRY harus PLANNED, gerak harus SUCCESS, torsi ≤ 14, tangkap rc 0, R0 harus terdeteksi).
3. **Data:** arm_2 P0+T1+T2 (23 entri) + arm_3 P0+T1+T2 (19 entri); **semua gerak MOVED+SUCCESS, torsi puncak ≤ 12.41 N.m, 0 fault /
   REJECTED / GOAL-MISMATCH eksekusi**. Plan-only berantai per tahap semua PLANNED (arm_2 6/6, 9/9, 9/9; arm_3 6/6, 9/9, 9/9).
   Lampu ruangan dimatikan sesudah A-01 (operator: silau) → A-01 0 → 12/12. Tak terdeteksi: arm_3 B-01, B-05, B-12 (dekat/jauh).
   Henti otomatis: A-07 rc 3 (j1 berayun ~4 s, lalu tenang → tangkap ulang atas izin); B-R0 rc 1 = crash `g63_capture` KeyError 'B'
   (lookup TF sesaat gagal → prior kosong; tidak ditangani alat) → tangkap ulang atas izin. `rec_health` 0 frame < 10 %.
4. **Jari (G7):** arm_2 0.808 → 0.791 di A-03 (−0.98° > 0.5°) → stop + tanya → operator "tidak apa2 lanjutkan saja"; arm_2 bergerak
   0.775–0.810 dan PULIH di REST (0.8093 sesudah jeda). arm_3 membuka **progresif 0.774 → 0.693 (−4.6°)** tanpa berhenti
   (🔴 saya tidak berhenti untuk bertanya — aturan arm_2 diterapkan diam-diam ke arm_3; operator menegur "kenapa gak diperbaiki dulu").
   Sesudah blok kedua gripper ditutup ulang via GripperCommand 0.96 (stall: arm_3 0.783, arm_2 0.810).
5. 🔴 **Label salah (D639):** B-13 rgbd2 — papan A arm_2 (REST, tengah bingkai) terbaca sebagai papan B (B dekat kamera 0.97 m →
   ROI prior lebar mencakup A). Dua papan polos IDENTIK di bingkai = celah penjaga (§A3 tidak membedakan papan identik). Dibuang dari
   solve dengan alasan tertulis (`ds_finger_all_noB13.json`).
6. **Solve resmi (`g63b_solve`, model kaku `tool_frame`):** arm_2 P0+T1 σ 0.50 px, G6′ resmi (lateral) ≤ 1.66 mm ✓, rgbd2 kurang
   teramati; P0+T1+T2 σ 2.73 px (galat per pose 2–15 mm), G3 uji-tahan rgbd 1.19 / rgbd2 0.98 cm, R0 akhir 2.35 mm; gabungan
   kedua lengan tanpa B-13 σ 5.96 px, R0 arm_3 lateral maks 2.84 mm (G6′ ✗) → **TOLAK** di semua varian.
   "5 mm" G6′ manual sebelumnya = komponen KEDALAMAN PnP (lampu nyala vs mati), bukan lateral; pergeseran R0 tidak sebanding sudut jari.
7. **Analisis luring (eksperimen, BUKAN alat terkunci; skrip + log di folder sesi):** model papan-ikut-jari (M1) tidak membantu;
   lendutan ∝ gravitasi (M2/M3) tidak stabil/overfit. **Offset nol sendi per lengan** (FK pinocchio dari sudut terekam, cocok TF
   ≤ 0.16 mm): arm_2 saja σ 2.73 → 0.65 px, R0 4.1 → 1.6 mm; kedua lengan (j1–j5; j6 setara T_lb → dikunci 0) σ 5.96 → **1.19 px**,
   uji-tahan med/p95 7.0/27.3 mm, R0 5.4/8.4 mm, std kamera 0.09°/1.6 mm & 0.12°/3.7 mm; kamera vs incumbent rgbd 1.45°/8.7 cm,
   rgbd2 1.51°/2.7 cm; offset arm_2 [3.20 0.20 1.72 −2.05 −1.80]°, arm_3 [2.74 −1.86 0.85 −1.35 −0.64]° (±0.07–0.24).
   **Tafsiran:** galat dominan = kinematika lengan (Gen3 Lite tanpa kalibrasi kinematik), bukan dudukan/kamera. j1 ≈ +3° di KEDUA
   lengan mencurigakan (kemungkinan rotasi bersama yang tak dimodelkan: acuan rotasi gantry / dunia) → **ekstrinsik BELUM sah,
   tidak dipasang**; uji-tahan p95 27 mm & R0 5 mm masih di atas gerbang.
8. **Akhir:** semua REST, gripper tertutup, papan TETAP di jari (operator), g1 **1.40**, g2 0; K0 G63 LULUS; shutdown SIGINT per PGID,
   domain kosong. Frame (3.5 GB, ada orang) lokal saja.

### B4. Papan skor akhir (semua blok hari ini; ✓/✗ harfiah)

| # | Hasil | Bukti |
|---|---|---|
| D639 | ✗ | badan: 6/8; jari: tak terdeteksi 4 pose (A-01 lampu nyala, B-01, B-05, B-12) + **1 label salah (B-13)** |
| D640 | ✓ | std sudut ≤ 0.18 px semua deteksi |
| D641 | ✗ | jari arm_2 −1.9°, arm_3 −4.6° (mode jari); badan ✓ 0.01° |
| D642 | ✗ | badan: j6 berburu (2 tolak); jari: A-07 j1 berayun (rc 3) |
| D643 | ✓ (badan) / n.a. (jari, satu muka) | |
| D644 | ✓ | torsi puncak ≤ 12.41 N·m semua gerak |
| D645 | tidak dinilai | tidak ada solve sah |
| D632–D635 | tidak teruji | tidak ada solve sah |
| D638 | ✓ | rec_health 0 frame < 10 % (badan + jari, ±690 k frame) |

**Langkah berikut G63b (luring, data cukup):** (1) penjaga detektor untuk papan identik (B-13); (2) model kinematik: uji apakah j1 ≈ +3°
bersama = acuan rotasi gantry/dunia (tambah rotasi gantry/basis sebagai parameter, bandingkan uji-tahan), validasi silang arm_2↔arm_3
(fit satu lengan, uji lengan lain); (3) bila model tervalidasi → amandemen §A tertulis (model kinematik + gerbang) SEBELUM dipakai;
HW hanya bila data tambahan terbukti perlu. Keadaan sel: papan di jari arm_2/arm_3, g1 1.40.

### B5. G63c LURING 2026-10-16 (NOL HW/kamera/mock; prompt [p1_g63c_offline_prompt.md](p1_g63c_offline_prompt.md); `results/p1_g63/g63c/`)

Data = `hw_20261016_finger/` apa adanya (tidak diambil ulang). Alat terkunci tidak diubah (sha `g63b_A_locked.sha256` 8/8 OK);
semua eksperimen di berkas baru `g63c/` (`g63c_common.py`, `g63c_guard*.py`, `g63c_kin.py`, `g63c_models.py`, `g63c_cross.py`,
`g63c_anchor.py` + log/json). Frame (orang) tidak di-commit.

1. **Penjaga identitas papan (tugas 1).** Pindai median tercatat (`g63c_guard_scan.py`): jarak pusat deteksi ke prior papan sendiri
   (m, pada skala prior; prior SEMUA papan dari FK sudut terekam + `T_wc_live` + `mount_finger`) benar 0.031–**0.139** m, rasio
   jarak ke prior papan lain **≥ 3.8**, skala kotak deteksi/prior 0.97–1.13; **B-13: 0.406 m, rasio 0.0, skala 0.34**. Aturan
   dibuat dari pindaian ini (B-13 TERLIHAT saat merancang — bukan uji buta) lalu dikunci di `g63c_guard.py` SEBELUM uji frame:
   **ID-1** jarak sendiri ≤ 0.18 m (= asumsi galat prior §A3), **ID-2** jarak ke tiap papan lain (prior di depan kamera, pusat di
   bingkai) ≥ 2 × jarak sendiri, **ID-3** skala ∈ [1/1.3, 1.3]. Uji ulang SEMUA frame (`g63c_guard_frames.py`, 43 rekaman × 2
   kamera × 30 frame, detektor terkunci, prior kedua papan, median ≥ 50 % frame seperti capture):
   **B-13 rgbd2 DITOLAK (ID-1 30/30 frame) ✓; 38/38 deteksi benar lain tetap ✓; 0 hilang ✓.** Bonus: 19 deteksi papan LAIN (papan
   di lengan REST) diterima dengan label benar, termasuk papan A di B-13 rgbd2 (29/30) — pusat A/B saat sama-sama terlihat terpisah
   ≥ 229 px. Jebakan: `IMREAD_GRAYSCALE` ≠ `cvtColor(BGR2GRAY)` capture → A-11 rgbd2 14 vs 16/30 frame (palsu "hilang");
   uji final pakai konversi capture. ⚠ Margin ID-1 tipis (0.139 vs 0.18 m) karena prior memuat galat kinematik ≈ 3°: bila prior
   memburuk, ID-1 bisa menolak deteksi benar (gagal aman, bukan label salah).
2. **Model kinematik (tugas 2, `models_all.log`).** Fit FIT kedua lengan (480 residu), nilai HOLDOUT (G3: per kamera med ≤ 15,
   p95 ≤ 30 mm) + **G6′ resmi** (T_lb dibekukan dari R0 pertama, R0 berikut lateral RMS ≤ 2 mm) + rank Jacobian berskala:

   | Model | k | σ px | BIC | rank | G3 rgbd / rgbd2 med/p95 mm | G6′ maks mm | kamera vs incumbent |
   |---|---|---|---|---|---|---|---|
   | M0 kaku | 24 | 5.96 | 1838 | penuh | 15.2/36.4 / 78.8/91.6 ✗ | 1.36 ✓ | 0.56°/6.0, 5.00°/16.9 cm |
   | **Ma** j1–j5 per lengan | 34 | 1.19 | 344 | penuh | 5.5/11.3 / 14.3/27.7 ✓ | 1.61 ✓ | 1.45°/8.7, 1.51°/2.7 cm |
   | Mb_raw Ma + basis B 6 | 40 | 0.60 | — | **38/40** | — | — | null = yaw basis ≡ j1, yaw global (gauge) |
   | **Mb** Ma − j1A + basis B (rx,ry,t) | 38 | 0.60 | −300 | penuh | 7.6/12.3 / 7.8/29.4 ✓ | 1.41 ✓ | 3.18°/10.0, 4.60°/9.7 cm |
   | Mt Ma − j1A + basis B (t) | 36 | 0.84 | 23 | penuh | 11.5/20.5 / 13.9/43.3 ✗ | 1.44 ✓ | |
   | Mc1 j1 bersama saja | 25 | 4.25 | 1518 | penuh | ✗ (48/66 rgbd2) | 1.53 ✓ | j1 +3.51° |
   | Mc2 rotasi gantry saja | 26 | 3.99 | 1463 | penuh | ✗ (66/83 rgbd2) | 1.53 ✓ | g1 −3.40°, g2 −0.06° (TIDAK bersama) |
   | **Md** rot gantry + j2–j5 | 34 | 1.00 | 177 | penuh | 8.6/10.5 / 14.5/26.2 ✓ | 1.51 ✓ | g1 −2.10°, g2 −1.67° |
   | Me j1 bersama + j2–j5 | 33 | 1.21 | 351 | penuh | 6.1/11.4 / 20.1/29.7 ✗ | 1.59 ✓ | j1 +3.02° |

   (c) "rotasi bersama saja" ✗ (σ ≈ 4 px; rotasi gantry yang dipilih data TIDAK sama antar-gantry). (b) dan (d) memberi fit lebih
   baik dari (a) dengan k sama/sedikit lebih — data lebih suka rotasi di sumbu gantry / geser basis daripada j1 murni.
   **Degenerasi (yang dicurigai prompt) TERBUKTI:** (i) yaw basis ≡ offset j1 (eksak); (ii) dengan kamera bebas, rotasi vertikal
   bersama ≡ geser relatif basis B ≈ δ·|o_B − o_A| (3° × 0.94 m ≈ 5 cm) → j1 "+3° di kedua lengan" SETARA dengan "basis arm_3
   tergeser ~(+2.7, +4.2, −1.7) cm + kamera berputar ~3°" — gambar tidak bisa membedakan; (iii) **Ma, Mb, Md sama-sama rank penuh,
   σ ≤ 1.2 px, lulus G3 + G6′, std kamera klaim 0.08–0.12° / 1.5–3.7 mm — tetapi kamera mereka berbeda satu sama lain sampai
   4.11° / 10.3 cm** (`model_spread.log`) = 30–40 × std. Std kecil TIDAK berarti teramati: ketidakpastian model mendominasi;
   uji-tahan (3 pose per kamera) tidak cukup kuat untuk membedakan.
3. **Validasi silang antar-lengan (tugas 3, `cross.log`).** Fit lengan tunggal: j1 bebas → **rank 22/23** (gauge: rotasi lengan di
   sumbu j1 ≡ rotasi kamera) → j1 dikunci. arm_2 saja σ 0.65 px, arm_3 saja σ 0.25 px (22 parameter, 288/216 residu).
   **Kamera tidak sepakat antar-lengan:** rgbd 1.76° / 5.1 cm, rgbd2 1.59° / 9.4 cm; **pose relatif rgbd→rgbd2 (invarian gauge
   dunia) 1.95° / 6.6 cm** vs std 0.10–0.25° / 1.7–4.8 mm ✗. Transfer (kamera dibekukan dari X, fit T_lb + parameter lengan Y):

   | Sumber → target | (a) j1–j5 | (d) rotG + j2–j5 | (b) j1–j5 + basis 5 |
   |---|---|---|---|
   | arm_2 → arm_3 | σ 12.97 px, G3 ✗ (82/112) | σ 9.61, ✗ | σ 0.75, G3 ✗ (rgbd2 15.4/29.9, rgbd 16.7/19.3) |
   | arm_3 → arm_2 | σ 13.65, ✗ | σ 8.89, ✗ (G6′ 2.01 ✗) | σ 2.65, ✗ (rgbd2 22.7/24.2) |

   Varian (b) "lulus hampir" hanya karena basis 6-DOF target menyerap transformasi kaku → yang tersisa diuji = pose relatif dua
   kamera, dan itu gagal. **Tidak ada model yang sah** menurut kriteria tugas 3.
4. **Jangkar lama (`anchor.log`, `anchor_tape.log`).** Kamera sesi jari = incumbent persis (0.000°). Vs incumbent + koreksi G34
   (rel/awan lengan REST): semua kandidat beda 1.26–5.86° / 1.4–24.8 cm; relatif rgbd→rgbd2 2.2–5.3° / 7–15 cm (G34 sendiri
   tak sepakat antar-kamera ~5 cm / 2–3°, memori camera-world-vs-urdf-offset). **Cek pita ARM1-ZONE** (botol di tanda (1.75, 0.55)
   terbaca rgbd2 +G34 di (1.858, 0.654, 0.836) = 15.0 cm): bacaan dipindah ke kamera kandidat → 14.5 (Mb) … 17.9 cm (Ma/Me) —
   **tak satu pun mendekatkan ke tanda ✗** (catatan: bias kedalaman botol +8 cm sepanjang sinar, B64 CAM-6DOF, ikut di angka ini).
5. **Tafsir ulang angka lama:** "R0 5 mm" di B3.7 = residu lateral R0 di bawah model gabungan, BUKAN G6′. G6′ resmi lulus
   (≤ 1.64 mm) di semua model — pergeseran papan antar-R0 kecil; masalahnya identifikasi kamera↔kinematika, bukan geser.
6. **Tidak dijalankan:** LOO per pose (tidak mengubah kesimpulan: validasi antar-lengan sudah gagal); model jari-ikut-papan ulang
   (M1 B3.7 sudah ✗); mock/kamera/HW (tidak perlu).

### B6. Papan skor G63c (✓/✗ harfiah)

| Tugas | Hasil | Bukti |
|---|---|---|
| 1 Penjaga papan identik | ✓ | B-13 ditolak 30/30 frame; 38/38 benar tetap; 0 hilang (aturan dirancang dengan B-13 terlihat — bukan uji buta) |
| 2 Model kinematik (a)/(b)/(c) + degenerasi | ✓ dijalankan / ✗ model unik | (a),(b),(d) lulus G3+G6′, (c) ✗; Mb_raw rank 38/40; kamera antar-model beda ≤ 4.1° / 10 cm ≫ std |
| 3 Validasi silang antar-lengan | ✗ | relatif kamera 1.95° / 6.6 cm; transfer 0/6 lulus G3; cek pita 0/5 membaik; G34 tak sepakat |
| 4 Amandemen §A | tidak dibuat | tidak ada model tervalidasi → B7 (yang kurang + rencana minimal) |

**Ekstrinsik dari data jari TIDAK sah, tidak dipasang.** Kinematika lengan (offset 1–3° per sendi, pilihan model mana pun) dan
kamera tak terpisahkan dengan data ini.

### B7. Yang kurang + rencana minimal (usul, butuh persetujuan operator sebelum §A baru)

**Akar masalah:** jalur "papan di lengan, acuan = FK" hanya sah bila FK benar. Gen3 Lite di sel ini punya galat kinematik
1–3°/sendi (semua model), dan dengan kamera bebas setiap rotasi vertikal per lengan adalah gauge → kamera di "dunia URDF" tidak
bisa ditentukan dari gambar lengan saja. Lebih banyak pose lengan TIDAK menyelesaikan gauge (ii); butuh **jangkar dunia yang
tidak lewat FK sendi**.

**Rencana minimal (NOL gerak lengan, kamera saja):**
1. **S-A papan statis berjangkar pita:** 3–4 papan polos A4 yang sama (4×5 @ 42 mm, penjaga ID dengan prior dari posisi pita)
   ditempel DATAR di permukaan tetap (meja kerja z 0.770, lantai, atas lemari laci) tersebar ≥ 1 m, tiap kamera melihat ≥ 2–3.
   Posisi tiap papan diukur pita dari **titik unting-unting sumbu j1 lengan di REST** (posisi basis hanya bergantung rel + dudukan:
   G34 rel = URDF ≤ 1 cm; pita basis-basis 0.80 = URDF) — bukan dari ujung tool (FK sendi). Ketelitian target ±3 mm / ±0.5°.
2. **Solve:** PnP multi-papan per kamera → T_wc; validasi silang **tinggalkan-satu-papan** (prediksi papan yang ditinggal ≤ G3)
   + kesepakatan dua kamera pada papan yang dilihat keduanya (G4) — semuanya bebas dari kinematika lengan.
3. **S-B (luring, data jari yang ADA):** kamera dibekukan dari S-A → fit offset kinematik per lengan (`g63c_cross.py` mode
   `cams_fixed`) → apakah arm_2/arm_3 konsisten (mis. j1 +3°?) → keputusan terpisah: koreksi kinematik untuk demo/MoveIt (fase
   sendiri, bukan G63).
4. Bila operator menolak papan statis: alternatif = dudukan kaku (klem/pelat) + ≥ 15 pose/lengan/kamera dengan sapuan j1 lebar
   **dan** jangkar statis yang sama — tetap butuh butir 1.

Pertanyaan untuk operator (belum ditanyakan): boleh menempel 3–4 papan A4 polos di meja/lantai/atas laci? siapa mengukur pita
dari titik unting-unting basis? papan di jari arm_2/arm_3 boleh tetap (tidak mengganggu S-A bila di luar bingkai)?
