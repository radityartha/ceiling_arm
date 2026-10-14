# P1 / G63 — kalibrasi ekstrinsik kedua D455 dengan papan ChArUco di jari gripper (eye-to-hand), acuan = FK lengan

> Sesi 2026-10-15 (prompt [p1_g63_charuco_prompt.md](p1_g63_charuco_prompt.md)). Sesi pertama = **DISKUSI + §A DRAF, NOL gerak HW,
> NOL kamera**. §A di bawah **BELUM DIKUNCI**; dikunci (salinan `results/p1_g63/g63_A_locked.md` + sha256 + commit) hanya sesudah
> operator menyetujui angka/gerbang di §A6, SEBELUM data kalibrasi dan SEBELUM gerak pertama. Data: [results/p1_g63/](results/p1_g63/).

**Kenapa G63 (tidak diperdebatkan ulang tanpa data baru):** CAM-6DOF (botol + YOLOE + median depth) gagal gerbang, M6/M7 menggeser
kamera 0.13–0.25 m / skala 0.89 ([p1_cam6dof.md](p1_cam6dof.md) B57–B59). B64: permukaan botol terbaca +8.4 cm lebih jauh di KEDUA
kamera (bias material) dan mask ikut box penyangga → botol+depth bukan alat ukur; ray-plane RGB saja rgbd2 0.22 cm, rgbd 5.2 cm di F5 →
ekstrinsik rgbd memang salah. B65: depth rgbd sehat pagi 10-15, kedipan 10-14 tidak tereproduksi (penyebab terbuka); firmware kedua
D455 kini 5.17.0.10. CALIB-CHARUCO lama ([p1_calib_charuco.md](p1_calib_charuco.md)): papan diam satu tempat/satu bidang + acuan
unting → lebih buruk dari incumbent 3–13 cm → G63 butuh **banyak pose 3D tersebar** dan acuan langsung FK (tanpa unting/pita).

## A. Protokol — DRAF (belum dikunci)

### A0. Keputusan operator 2026-10-15 (jawaban sesi diskusi, dicatat harfiah)

| # | Pertanyaan | Jawaban operator | Konsekuensi di protokol |
|---|---|---|---|
| A0.1 | Jenis target | **Papan ChArUco kaku** | RGB saja, PnP / reprojeksi; depth TIDAK dipakai untuk kalibrasi (§A3). |
| A0.2 | Ukuran | **A4, 4×5 kotak @ 50 mm** (pola 200×250 mm) | Dibuat: [boards/](results/p1_g63/boards/) papan **A** (arm_2, ID 0–9) dan **B** (arm_3, ID 10–19), `DICT_4X4_250`, marker 37.5 mm. Commit 0f765db. Verifikasi PDF dirasterisasi ulang: 12/12 sudut, 10/10 marker, kotak 50.00×50.02 mm. Dictionary beda dari papan lama (5X5) → tak tertukar. |
| A0.3 | Cara pasang | **"pakai double tape untuk menempelkan charuco"**; alas **kardus**, ditempel di **jari gripper bagian yang datar** | Papan menempel ke link jari → rantai FK lewat sendi jari (`right_finger_bottom_joint`); gripper TIDAK diperintah selama G63; offset papan→link diestimasi solver (§A3). Risiko: kardus melengkung, jari Gen3 Lite lentur, tape merayap → §A2 cek kerataan + §A4 G6 cek geser. |
| A0.4 | Pengaman tambahan | **"Cek geser saja"** (tanpa tali pengaman) | Pose acuan R0 diulang (§A2). Risiko jatuhnya papan dicatat; papan ringan (kardus ≲ 60 g) — operator menjauh dari bawah lengan saat gerak (aturan biasa). |
| A0.5 | Lengan pembawa | **Dua lengan, 1 per gantry: arm_2 + arm_3** | arm_4 (membawa D435IF) dan arm_1 TETAP REST sepanjang G63. |
| A0.6 | D435IF (327122073095) | Terpasang di gripper **arm_4**; massa/dudukan **belum diketahui**; **tidak dipakai** di G63 | arm_4 tidak digerakkan → model torsi D435IF tidak dibutuhkan G63; TETAPI volumenya tidak ada di URDF → amplop tabrakan konservatif di sekitar gripper arm_4 wajib dalam saringan rencana arm_3 (gantry sama, §A5 T4). Gerak arm_4 di fase mana pun sesudahnya menunggu timbangan + model. |
| A0.7 | Peta env (laci dipindah) | **"env cek nanti"** | Gerbang S0c (kamera saja) sebelum gerak pertama: peta `env_static_map` dibandingkan awan hidup; bila laci di luar peta lama di area jalur → peta diperbarui (fase sendiri) atau pose di sekitar laci dibuang SEBELUM kunci. |

Batas yang tetap (handoff §2): setiap gerak nyata (termasuk pulang/K0/pemulihan) = **satu izin operator per langkah**; kamera saja juga
perlu izin; `rec_health.py` berjalan terus setiap kamera hidup; `octomap_updaters:=false`; torsi > 14 = berhenti, rencana > 97 % = tolak;
penyaring tidak dilonggarkan; launch default / TF default TIDAK diubah tanpa operator; tidak ada skala via TF.

### A1. Model dan besaran

- Frame acuan = **world URDF** (= world MoveIt). Ground truth = FK dari **sendi TERUKUR** (`/joint_states`, baca per nama sampai lengkap;
  dua publisher) termasuk rel + rotasi gantry + sendi jari, median selama tangkapan diam.
- Kamera c ∈ {rgbd, rgbd2}: T_wc (6 DOF) — **yang dikalibrasi**. Intrinsik = `camera_info` hidup (pabrik, sesudah FW 5.17: fx rgbd
  643.544); TIDAK diestimasi. Distorsi dari `camera_info`.
- Papan b ∈ {A, B}: T_lb (6 DOF) papan di frame link jari lengan pembawa — **parameter gangguan**, diestimasi (dudukan tape tidak bisa
  diukur ±1 mm). Ukuran kotak = **terukur** sesudah cetak (bukan nominal 50.0).
- Observasi = piksel sudut ChArUco u_ij. Residu: π(K_c, T_wc⁻¹ · T_wl(q_i) · T_lb · p_j) − u_ij.
- Total parameter 24 (2×6 kamera + 2×6 papan); kedua papan × kedua kamera dalam satu LS bersama.

### A2. Urutan (setiap gerak nyata ditanya, satu langkah per persetujuan)

**S0 — tanpa gerak lengan (sebelum kunci §A):**
- S0a gerbang K0–K3b operator + identitas kamera (serial) + preflight proses/domain (jangan `pkill -f`).
- S0b **uji jangkauan deteksi, kamera saja** (izin kamera): operator memegang papan A tercetak di 1.0/1.5/2.0/2.5/3.0 m dari tiap
  kamera, beberapa kemiringan → catat sudut terdeteksi/marker terdekode per jarak (menguji D630 dan memilih batas jarak pose). Papan
  dipegang tangan = bukan data kalibrasi.
- S0c **verifikasi peta env** (kamera saja): awan D455 vs `env_static_map` (laci dipindah).
- S0d ukur papan sesudah cetak/tempel: 4 kotak (mm), 5 kotak (mm), kerataan (penggaris lurus di diagonal; celah maks mm), massa
  papan+kardus+tape (g), foto dudukan di jari. Gerbang: celah ≤ 1 mm; massa ≤ 100 g (asumsi model torsi §A5).
- `rec_health.py` dua kamera berjalan terus sejak kamera pertama hidup sampai mati (berkas per sesi).

**S1 — bring-up nyata** (`p1_demo_hw/harness/stack_up.sh`, `octomap_updaters:=false`, `enable_gantry_bridge:=true`), K0 terukur
`--rail1 1.50 --rail2 0.00`, monitor torsi + perekam sendi (cek tumbuh), kamera incumbent (launch + koreksi G34).

**S2 — arm_2 / papan A** (g1 tetap PARKIR 1.50, g2 0, rot 0/0 kecuali §A6 butir 4 disetujui):
1. Operator menempel papan A ke jari datar arm_2 saat arm_2 REST; foto; catat sendi jari.
2. Plan-check semua pose arm_2 (plan-only, payload + papan terpasang, §A5) → tabel ke operator → kunci daftar pose.
3. Per pose: izin → gerak (`reach_dwell_probe … --tau-max 14 --move`, wrapper penjaga 97 %) → diam ≥ 2 s → tangkap 30 frame
   RGB kedua kamera + `camera_info` + `/joint_states` + TF (cap waktu per frame) → berikut. **Pose acuan R0** di awal, sesudah tiap
   8 pose, dan di akhir (gerak ke R0 = langkah ber-izin seperti yang lain).
4. Izin → pulang REST arm_2. Operator melepas papan A (atau membiarkan; dicatat).

**S3 — arm_3 / papan B**: sama seperti S2 (g2 0). arm_4 + D435IF tetap REST; amplop D435IF masuk saringan.

**S4 — solve luring** (set FIT saja) → gerbang G2 → evaluasi UJI-TAHAN (G3/G4). Tidak ada penyetelan dari uji-tahan.

**S5 — hanya bila G1–G4 lulus:** kamera diluncurkan ulang dengan arg baru lewat harness BARU (launch default tetap) → G5 TF hidup →
pemeriksaan depth (§A3 D-test, informasional untuk keputusan Tare) + `arm_fk_check --arm-filter 0.03` REST ×3 (kesiapan demo, §A4 G8).

**S6 — akhir**: lengan REST, g1 PARKIR 1.50, K0 terukur, matikan bersih (PID milik sesi, domain kosong, 0 static TF basi), arsip, laporan.

### A3. Pose, solver, uji-tahan, uji depth

**Pose (dibangkitkan alat T4 dari incumbent sebelum kunci; daftar akhir = sesudah plan-check HW):**
- Per lengan ≈ **24 pose** (18 FIT + 6 UJI-TAHAN) + R0 ×≥ 4. Ketinggian pusat papan z ∈ {0.85, 1.10, 1.35} m (cakupan 0.8–1.4), xy
  tersebar di jangkauan lengan dalam FOV kamera sasaran; jarak kamera–papan 0.8–2.8 m (batas atas dari S0b).
- Visibilitas (ekstrinsik incumbent, margin 40 px dari tepi bingkai): ke-12 sudut terproyeksi di dalam bingkai kamera sasaran; sudut
  normal papan–sinar ≤ 55°; diusahakan ≥ 30 % pose terlihat KEDUA kamera (pengikat antar-kamera / G4).
- Keragaman orientasi (wajib untuk keteramatan T_lb): kemiringan papan tersebar ≥ ±25° pada dua sumbu + putar ±30° sekitar normal.
- Zona: rgbd2 — zona meja arm_1 dijangkau arm_2 sejauh mungkin (zona galat +0.15 m), zona arm_2/arm_3; rgbd — lantai/zona arm_3
  (F1–F7 CAM-6DOF), zona arm_2/arm_4. Zona tak terjangkau dicatat sebagai **tak tercakup** (tidak diklaim).

**Deteksi:** marker ID bila terdekode; bila tidak (jauh), fallback sudut chessboard 3×4 dengan pelabelan 180° dipilih dari proyeksi
prediksi FK + incumbent. **Aturan buang terdaftar (sebelum data):** frame ditolak bila < 8 sudut; pelabelan ambigu bila dua kandidat
berselisih < 2× RMS → pose itu ditolak untuk kamera itu (dicatat, tidak diganti). Median sudut per pose dari 30 frame (std dicatat).

**Solver (alat T3):** inisialisasi T_wc = incumbent, T_lb = rata-rata (SE3) dari PnP per gambar (IPPE) × FK; lalu LS polos (Levenberg–
Marquardt, **tanpa loss robust** — titik buruk harus terlihat, sama dengan prinsip CAM-6DOF A0.1) atas semua sudut FIT kedua kamera /
kedua papan. Residu per pose dilaporkan; pose > 3× median RMS ditandai, **tidak dibuang**.

**UJI-TAHAN:** 6 pose per lengan dipilih SEBELUM data dengan aturan indeks (setiap pose ke-4 dalam urutan terkunci, bertingkat per
zona/tinggi). Tidak dipakai untuk fit, inisialisasi, atau memilih apa pun. Besaran utama = **galat lateral** e = |residu piksel| ×
jarak / f (m), dari sudut prediksi FK dengan T_wc baru dan T_lb dari FIT.

**Uji depth (D-test, informasional, BUKAN kalibrasi):** untuk tiap pose, rasio r = d_ukur / d_pred, d_ukur = median depth terdaftar
pada piksel interior **kotak putih** papan (kotak hitam dan tepi dibuang), d_pred dari T_wc baru + FK. Bin 0.5 m, 0.8–3 m, per kamera.
Keputusan terdaftar: bila suatu bin dengan ≥ 3 pose punya |median r − 1| > 0.02 → rekomendasi sesi on-chip calibration / Tare
TERPISAH atas keputusan operator. **Skala via TF dilarang.** Kertas putih matte ≠ botol putih tembus IR (B64) — bias material
diharapkan kecil, tetap diukur.

### A4. Kriteria terima (DRAF — angka diusulkan, disahkan operator di §A6)

| # | Lulus bila |
|---|---|
| G1 | Data per kamera: ≥ 15 pose FIT sah, rentang jarak ≥ 0.8 m, kemiringan papan ≥ 40° pada ≥ 2 sumbu, kedua papan ada |
| G2 | Solver konvergen; sanity vs incumbent: rotasi ≤ 5°, posisi kamera ≤ 0.15 m (= gerbang A0.1 CAM-6DOF); T_lb masuk akal (≤ 0.20 m dari link jari) |
| G3 | UJI-TAHAN per kamera: median e ≤ **1.5 cm**, p95 ≤ **3.0 cm**; tiap zona yang tercakup median ≤ **2.0 cm** |
| G4 | Pose UJI-TAHAN terlihat dua kamera: triangulasi RGB sudut (dua kamera, ekstrinsik baru) vs FK median ≤ **2.0 cm** (mengikat arah sepanjang sinar) |
| G5 | TF hidup = arg ≤ 1 mm / 0.01° |
| G6 | Cek geser R0: perubahan pose papan (PnP pada ekstrinsik tetap, relatif FK) antar-ulangan ≤ **2 mm / 0.3°**; gagal → semua pose sesudah R0 baik terakhir DITOLAK |
| G7 | Keselamatan: tiap gerak MOVED + dwell SUCCESS, torsi terukur ≤ 14, 0 fault/REJECTED; sendi jari berubah ≤ 0.5° sepanjang blok |
| G8 | (Kesiapan DEMO, bukan penerimaan kalibrasi) `arm_fk_check` REST ×3 ekstrinsik baru: tiap lengan terlihat \|t_xy\| ≤ 5 cm DAN ≤ incumbent + 1 cm |

**Keputusan:** G1–G7 lulus → ekstrinsik baru DITERIMA (dipakai lewat harness/arg baru; default launch berubah hanya atas keputusan
operator). G8 gagal dengan G1–G7 lulus → penyebab = depth (bukan ekstrinsik) → keputusan Tare/on-chip terpisah; DEMO-HW tetap menunggu.
Gagal G1–G4 → incumbent tetap; lapor sebabnya. Kriteria tidak diubah sesudah melihat data.

Alasan angka G3: kebutuhan demo ≤ 5 cm total; anggaran ekstrinsik sepertiganya. Lantai galat FK: rel = URDF ≤ 1 cm (G34), FK Gen3 Lite
beberapa mm, deteksi 0.3 px ≈ 1 mm di 2.5 m → 1.5 cm median masih di atas lantai noise. Lebih ketat (mis. 1.0 cm) berisiko gagal
karena galat basis gantry, bukan kamera — operator memilih di §A6.

### A5. Alat luring (sebelum kunci; NOL HW)

| # | Alat | Status |
|---|---|---|
| T1 | `results/p1_g63/make_g63_boards.py` → PDF papan A/B | **SELESAI** 0f765db (verifikasi raster PDF lulus) |
| T2 | Detektor `g63_detect.py`: ChArUco + fallback chessboard + pelabelan FK; keluaran sudut/ID/RMS per frame | belum |
| T3 | Solver `g63_solve.py` (§A3) + **self-test sintetis**: tanpa noise ≤ 0.01° / 1 mm; noise 0.3 px + FK 2 mm ≤ 0.15° / 5 mm; kontrol negatif: tanpa keragaman orientasi → T_lb tak teramati harus TERDETEKSI (kondisi Jacobian), bukan diam | belum |
| T4 | Pembangkit pose `g63_poses.py`: visibilitas (incumbent) + IK + **torsi RNEA dengan payload papan** (massa S0d, asumsi ≤ 100 g di pusat papan, offset j2 +7.7, batas 14, 97 %) + **papan sebagai benda tabrakan** terpasang di link jari (kotak 0.21×0.26×0.01 m + margin) + **amplop D435IF** di gripper arm_4 (kotak konservatif 0.15×0.10×0.10 m sampai diukur). Probe produksi belum mendukung payload/benda terpasang → wrapper sesi / skrip saringan tambahan (keputusan desain di T4, tanpa melonggarkan saringan yang ada). | belum |
| T5 | Harness tangkap `g63_capture.py`: 30 frame RGB + camera_info + joint_states (per nama) + TF, cap waktu, `rec_health.py` paralel | belum |
| T6 | Plan-check MOCK domain 77, `octomap_updaters:=false`, papan + amplop di scene: semua pose arm_2/arm_3 dari K0 (REST, g1 1.50, g2 0) | belum |

### A6. Untuk disetujui operator sebelum kunci

1. Gerbang G3/G4/G6 (1.5/3.0/2.0 cm; 2 mm/0.3°) — atau lebih ketat/longgar sebelum data.
2. ≈ 2 × (24 + 4 R0 + pulang) ≈ **60 gerak nyata**, masing-masing satu izin. Setuju jumlah ini, atau kurangi pose?
3. D435IF: amplop konservatif cukup untuk G63 (arm_4 diam)? Bila bisa, foto + ukur dudukan.
4. Gantry tetap g1 1.50 / g2 0 / rot 0, atau izinkan geser rel untuk cakupan zona (tambahan gerak + galat rel ≤ 1 cm)?
5. Papan B: cetak dan pasang di arm_3 bersamaan (dua papan terpasang) atau bergantian.

### A7. Dugaan (DRAF, dikunci bersama §A)

| # | Dugaan |
|---|---|
| D630 | S0b: sudut papan 50 mm terdeteksi (≥ 8/12) sampai ≥ 2.5 m di kedua kamera; marker 4X4 terdekode hanya ≤ 2.0 m. |
| D631 | Self-test T3: tanpa noise ≤ 0.01° / 1 mm; dengan noise 0.3 px + FK 2 mm ≤ 0.15° / 5 mm; kontrol negatif terdeteksi. |
| D632 | T_wc rgbd baru bergeser ≥ 3 cm (posisi kamera atau ekuivalen lateral di F5) dari incumbent (B64 ray-plane 5.2 cm). |
| D633 | G3 lulus kedua kamera. |
| D634 | rgbd2 zona arm_1: G3 (RGB) lulus TETAPI G8 rgbd2:arm_1 tetap > 5 cm → galat +0.15 m arm_1 bukan ekstrinsik, melainkan depth jarak-jauh (B64: r rgbd2 1.027–1.07 di ≥ 2.5 m). |
| D635 | D-test rgbd2: median r bin 2.5–3.0 m ≥ 1.02. |
| D636 | G6: papan kardus + double tape tidak bergeser > 2 mm / 0.3° dalam satu blok lengan. |
| D637 | Torsi j2 terukur puncak dengan papan ≤ 13 N·m pada semua pose terpilih. |
| D638 | `rec_health`: 0 frame < 10 % valid pada rgbd sepanjang sesi kamera G63 (kedipan 10-14 tidak muncul). |

## B. Hasil terukur

(belum ada — NOL gerak, NOL kamera pada sesi diskusi 2026-10-15)
