# P1 / G63b — desain ulang target kalibrasi ekstrinsik D455 (papan catur polos, dudukan di badan gripper)

> Sesi 2026-10-15 (prompt [p1_g63b_redesign_prompt.md](p1_g63b_redesign_prompt.md)), **LURING: NOL gerak HW, NOL kamera,
> NOL mock**. Lanjutan G63 ([p1_g63_charuco.md](p1_g63_charuco.md) §B1–B3: HW P0 0 observasi). **§A ini DRAF — belum dikunci.**
> Dikunci (salinan `results/p1_g63/g63b/g63b_A_locked.md` + sha256 + commit) hanya atas persetujuan operator, sesudah dudukan
> NYATA diukur dan plan-check mock. Data/alat: [results/p1_g63/g63b/](results/p1_g63/g63b/).

Tujuan tetap sama dengan G63: ekstrinsik baru kedua D455 (T_wc, 6 DOF) dari sudut papan yang dibawa lengan, acuan = FK sendi
terukur; gerbang G1–G8 G63 kecuali yang diulas di §A4. Yang berubah: **target**, **dudukan**, **lengan per fase**, **detektor**.

## A. Protokol (DRAF)

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
`gripper_base_link` = gangguan, diestimasi; muka 2 = muka 1 · T_12 (rot y 180°, asal (4·kotak, 0, t_rakitan)), t_rakitan
**terukur** (jangka sorong) — bila solver perlu, t_rakitan jadi parameter ke-7 (keputusan di T3b, sebelum data).
Ukuran kotak = **terukur** sesudah cetak. Tiap fase (arm_2, arm_3) punya T_lb sendiri (rakitan dilepas-pasang).

### A2. Urutan (tiap gerak nyata = satu izin)

S0 (tanpa gerak): K0 + jari arm_2 ukur ulang (A0.6), identitas kamera, `rec_health` terus, **papan lama di luar bingkai**
(foto kedua kamera, kamera saja = izin), ukur papan (4/5 kotak, kerataan ≤ 1 mm, t_rakitan, massa, foto dudukan) →
`mount_g63b.json` NYATA → URDF + pose ulang → plan-check mock (sendi-tujuan, `--extra-env`) → kunci §A.
S1 bring-up (`octomap_updaters:=false`, `enable_gantry_bridge:=true`). **Rel g1 1.50 → 1.30 TANPA papan** (REST + papan menabrak
meja di 1.40, §A8.6), lalu pasang rakitan di arm_2 (lengan REST). S2 arm_2 P0 → … S3 lepas rakitan → g1 1.30 → 1.50 PARKIR
(tanpa papan) → pasang rakitan di arm_3 (g2 0; REST arm_3 bersih, §A8.6) → P0 …. S4–S6 sama dengan G63.
**Pulang REST selama papan terpasang hanya via `probe_g63`** (A8.10 G63).

### A3. Deteksi (aturan buang DIDAFTARKAN sebelum data)

Per frame, per kamera, per muka yang menurut FK menghadap kamera (sudut ≤ 70°):
1. ROI = proyeksi pusat muka (FK + incumbent) ± (setengah diagonal + tepi + 0.18 m + 0.05 m) — di luar ROI diabaikan.
2. `findChessboardCornersSB` 3×4, EXHAUSTIVE|ACCURACY; grid ditolak → hull ×1.6 ditutup, ulang maks 3×.
3. Penjaga (semua wajib): residu homografi 12 sudut ≤ **1.0 px** RMS; pelabelan = arah muka (det J dari FK-muka) + paritas
   20 kotak, |skor| ≥ **0.5**; **cincin putih**: 18 titik di tengah tepi 21 mm semua > (hitam+putih)/2.
4. < 12 sudut = frame ditolak (papan polos: semua atau tidak sama sekali). Median 30 frame per pose; std dicatat.
5. Identitas muka: hanya muka dengan normal FK ke kamera ≤ 70° yang dicari; bila kedua muka ≤ 70° (mustahil, normal
   berlawanan) → pose ditolak untuk kamera itu.
Tidak ada pelonggaran sesudah data; frame P0 lama (`hw_20261015b/cap/frames`) = data pengembangan saja.

### A4. Gerbang (ulasan G1–G8)

G1 tetap (≥ 15 FIT/kamera, dua fase/lengan ada). G2 tetap; T_lb ≤ 0.20 m dari `gripper_base_link`. G3 tetap (1.5/3.0/2.0 cm).
G4′ tetap (2.0 cm); G4 lama dilaporkan bila ada pose dua-kamera (kini mungkin: muka berbeda). G5 tetap. G6′ tetap (2 mm).
G7 tetap (MOVED+SUCCESS, torsi ≤ 14, 0 fault, **jari ≤ 0.5°** — kini harus lulus karena jari tidak dibebani). G8 tetap.
**Baru — G9 (diam):** capture ditolak bila sendi bergerak > 2e-4 rad (tetap, tidak dilonggarkan); usul aturan pose j4 §A5.

### A5. Pose

`g63b/g63b_poses.py` (= `g63_poses.py` + dua muka + aturan j4): visibilitas incumbent per muka, z pusat 0.80–1.40,
jarak 0.6–2.8 m, sudut ≤ 55°, torsi statis ≤ 92 % (150 g), tabrakan (rakitan = satu kotak: dua lembar + badan gripper,
+10 mm), cek retract, oklusi; g1 **1.30**, g2 0. **Usul j4** (operator memutuskan, §A6): |τ_j4 statis| ≥ ambang (lihat §A8.7).

### A6. Pertanyaan operator untuk kunci (menunggu)

1. **Aturan diam j4** (§A8.8): (i) tetap 2e-4 rad semua sendi (pose seperti A-02 bisa ditolak lagi) **atau** (ii) usul: sendi
   1–3, 5–6 tetap ≤ 2e-4 rad; j4 boleh berburu bila gerak papan akibat j4 (FK per sampel selama tangkap) ≤ **1.5 mm** pp, sudut =
   median 30 frame dipasangkan dengan median sendi. (ii) = kriteria BARU (bukan pelonggaran diam-diam) → hanya atas keputusan operator.
2. g1 **1.30** untuk fase arm_2 (1.40 tabrak meja di REST dengan rakitan); rel digeser TANPA papan.
3. Lembar dipotong ke tepi 21 mm semua sisi (210 × 252 mm); tepi atas lembar ≥ 45 mm dari flens pergelangan.
4. Dudukan NYATA + ukuran (foto, massa, t_rakitan, posisi lembar) → pose ulang + plan-check mock → kunci.

### A7. Dugaan (DRAF, dikunci bersama §A)

| # | Dugaan |
|---|---|
| D639 | Papan polos 42 mm terdeteksi (12/12 sudut) pada ≥ 80 % pose terencana per kamera (P0 + tahap), 0 label salah pada pemeriksaan visual semua overlay. |
| D640 | Galat sudut median ≤ 0.5 px pada pose ≤ 2.0 m (std 30 frame). |
| D641 | Jari arm_2/arm_3 berubah ≤ 0.5° sepanjang blok (dudukan di badan gripper) — G7 lulus. |
| D642 | Dengan aturan j4, 0 capture ditolak karena "tidak diam" di P0. |
| D643 | ≥ 1 pose per lengan terlihat kedua kamera (muka berbeda) di daftar terkunci. |
| D644 | Torsi j2 terukur puncak ≤ 13 N·m dengan rakitan (massa terukur) pada semua pose terpilih. |
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
