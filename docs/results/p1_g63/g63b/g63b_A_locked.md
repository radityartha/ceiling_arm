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

