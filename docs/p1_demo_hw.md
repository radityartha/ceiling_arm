# P1 / DEMO-HW — suara → YOLOE (objek nyata) → lengan ke ATAS objek (approach-only), HW NYATA

> Sesi 2026-10-12 (prompt [p1_demo_hw_prompt.md](p1_demo_hw_prompt.md)). SCOPE: suara/teks → YOLOE `bottle` → `object_localizer`
> → `target_cli` → `demo_runner` → `reach_dwell_probe` → arm_1 berhenti stand-off 0.25 m di atas centroid botol, dwell, REST.
> Tanpa grasp, tanpa angka akurasi (eval = P2). 1–3 jalan bergerak. g1 PARKIR 1.50, gantry TIDAK diperintah.
> §A dikunci (salinan `results/p1_demo_hw/demo_hw_A_locked.md` + sha256) sebelum kamera dibuka dan sebelum gerak pertama.

## A. Protokol — DIKUNCI

### A0. Gerbang + pengungkapan (sebelum mengunci, NOL gerak)

**Gerbang operator (~17:47):** K0 sel = akhir G57 ✓; K1 operator di e-stop fisik tiap gerak ✓; K2 lampu nyala, satu botol
≤ 0.30 m di meja sekitar (1.75, 0.55), tak ada botol lain ✓ (posisi **kira-kira**, lihat A0.5); K3 tak ada task server /
reach_fusion ✓ — dicek juga `ps`: nol proses ROS, `enp112s0` 192.168.2.100.

| # | Temuan / keputusan | Bukti |
|---|---|---|
| A0.1 | 🔴 **Domain ROS.** Harness DEMO-OFF/G59: kamera + localizer domain 79, runner/CLI 77 (mock). Di HW `demo_runner` (sub `/detected_objects/tracks`, panggil probe → MoveIt) HARUS di domain stack 0 → kamera, `object_localizer`, `voice_web_ui`, `target_cli`, `demo_runner`, monitor, perekam **semua domain 0**. Domain 79 hanya untuk langkah 1 (kamera tanpa stack). Prompt `p1_demo_off.md` §D ("kamera tetap domain 79") **salah** untuk demo bergerak. Harness baru [results/p1_demo_hw/harness/](results/p1_demo_hw/harness/) (`env.sh`: `ROS_DOMAIN_ID=${DEMO_DOMAIN:-0}`); harness lama tidak diubah. | `demo_runner.py:62`; `p1_demo/harness/env*.sh` |
| A0.2 | 🔴 **Ekstrinsik kamera belum dikoreksi G34.** `realsense_dual.launch.py` memakai kalibrasi 2026-07-30; peta `env_static_map` dibangun DENGAN koreksi tetap G34 (`p1_g34/rail_joint4_diff.json`, p_urdf = R·p_cam + t). Tanpa koreksi, botol di URDF (1.75, 0.55, 0.97) terlihat di (1.845, 0.497) [rgbd] / (1.819, 0.482) [rgbd2] → **|dxy| ≈ 0.10–0.11 m** → kriteria 5 cm gagal dan lengan berhenti ~10 cm di samping botol. **Keputusan operator ~17:50: pakai koreksi G34** lewat arg launch di `harness/cam_up.sh` (rgbd t (1.7166, 0.8656, 2.0782) yaw 2.1688; rgbd2 t (−0.1547, −1.0197, 2.0905) yaw −0.9189; roll/pitch tetap — koreksi murni yaw). Default launch TIDAK diubah. Sisa: beda antar-kamera ~5 cm / 2–3° (G34), di bawah z ≈ 1 kurang tepercaya. | memori camera-world-vs-urdf-offset; `realsense_dual.launch.py:163-174` |
| A0.3 | `demo_runner` hardcode `--tau-max 12.0` → opsi `--tau-max` (default 12.0 = argv lama identik `'12.0'`); demo HW `--tau-max 14` (operator 2026-10-12: > 12 bukan gagal, batas keras > 14). | diff `demo_runner.py` |
| A0.4 | `g` hanya lewat `harness/cli.fifo`, ditulis Claude **sesudah** persetujuan operator untuk SATU jalan; satu persetujuan = satu jalan utuh (probe `--move` → dwell → `return_rest --arms arm_1 arm_2 --move` otomatis oleh runner). Suara = Chrome mic ke `voice_web_ui` (https :18080) atau POST `/api/transcript` teks. `g` yang tiba selama misi dibuang `target_cli` (G59 B1.2). | G59 B1 |
| A0.5 | **Posisi botol acuan = kira-kira** (operator tidak tahu titik nol world). Cara ukur yang diberikan: unting dari pusat basis arm_1 (world (1.90, 0.36) saat g1 1.50) ke meja; titik (1.75, 0.55) = **15 cm ke arah arm_2 (−x), 19 cm menjauhi gantry 2 (+y)**. Kriteria H2 dibandingkan ke posisi yang dilaporkan operator sesudah ukur; bila operator tak bisa mengukur, H2 dilaporkan "acuan kasar" (tidak dihitung LULUS). | `p1_demo_off.md` A5 |
| A0.6 | Botol TIDAK ada di `env_static_map` (T11) → pemisah dari botol = stand-off saja. Puncak botol ≤ centroid + 0.15 → ujung tool ≥ 0.10 m di atas puncak bila z track benar. Track z terlalu RENDAH = tool lebih dekat ke botol → H2 z ≥ 0.85 wajib sebelum gerak. | DEMO-OFF A2 |
| A0.7 | `POSE_REFUSALS` tidak memuat START-JUMP / START-UNSCREENED / GOAL-MISMATCH → runner berhenti + `return_rest` (tidak coba lengan lain). Tidak diubah. | G59 A0 |
| A0.8 | `object_localizer` memfusi kedua kamera (`fuse_radius` 0.10) — track = rata-rata; `rgbd_perception.launch.py` tidak menjalankan localizer → `harness/loc_up.sh` (remap `seg/`). | `object_localizer.py:55,125` |

### A1. 🔒 Kriteria

| # | Kriteria | Lulus bila |
|---|---|---|
| H1 | Deteksi stabil | log `seg_router` tanpa `inference failed`; `/rgbd/seg/instance_segmentation_labels` (atau rgbd2) memuat `bottle`; tepat SATU track `bottle` dengan tid sama sepanjang rosbag 20 s |
| H2 | Lokalisasi | track `bottle` \|dxy\| ≤ 5 cm dari posisi botol terukur operator; z ∈ [0.85, 1.05]; di domain 79 (langkah 1) DAN domain 0 dengan stack hidup (langkah 2) |
| H3 | Plan-only | `success arm_1 (arm_1:PLANNED)`; /joint_states sebelum = sesudah (≤ 0.05°, rel ≤ 1 mm) |
| H4 | Jalan bergerak | MOVED → dwell SUCCESS → `return_rest` rc 0 → status `success arm_1`; torsi terukur ≤ 14 N·m semua sendi; 0 gerak kedua tak diminta; t1_linear tetap 1.4999 ± 5 mm, rot ± 0.3° (gantry tak diperintah) |
| H5 | Bersih | lengan REST, K0 akhir LULUS; `ros2 node list --no-daemon` = 0 di domain 0 dan 79; /tmp diarsip |

**Berhenti + amankan (tanya operator):** fault / LED merah / REJECTED / NOT armed sesudah ARMED / torsi > 14 / benturan / kabel /
gerak tak diminta / track z < 0.85 sebelum `g`. MARGIN = TOLAK; penyaring TIDAK dilonggarkan. `failed TORQUE-UNSAFE` = `g` lagi
(dengan persetujuan baru), bukan ubah penyaring.

### A2. 🔒 Urutan

1. Kamera tanpa gerak, domain 79, stack MATI (`DEMO_DOMAIN=79 harness/cam_up.sh` + `loc_up.sh`): H1, H2, rgb (`p1_demo/harness/grab.py`),
   rosbag 20 s (`harness/bag.sh`, topik ringan). Matikan, node 0 di 79.
2. Self-test `env_collision` + `interarm_collision`; `remount_check`; bring-up nyata `harness/stack_up.sh` (G57: SIG_DFL, SigIgn,
   4× '6', 7/7, ARMED, `env_static_map` di scene); K0 `--rail1 1.50 --rail2 0.00`; monitor + perekam (domain 0, cek berkas tumbuh);
   kamera + localizer domain 0 → ulang H1/H2.
3. `voice_web_ui` + `target_cli` (fifo).
4. Runner `EXTRA=--plan-only` → perintah → `g` (persetujuan) → H3.
5. Runner tanpa `--plan-only` → per jalan: tanya → perintah → `g` → H4. 1–3 jalan.
6. K0, matikan bersih (runbook §7), arsip, laporan + gambar arsitektur.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D540 | Langkah 1: H1 lulus (lampu nyala, YOLOE dari `ros2_ws/`); track `bottle` \|dxy\| ≤ 5 cm dari (1.75, 0.55) dengan ekstrinsik terkoreksi. |
| D541 | Track z ∈ [0.90, 1.05] (kamera atap melihat bagian atas botol → centroid permukaan di atas centroid sejati). |
| D542 | Domain 0 dengan stack hidup: track sama ≤ 2 cm dari langkah 1, nol `inference failed`. |
| D543 | Plan-only: arm_1 PLANNED pada `g` pertama (≤ 3 sampel); penyaring terketat = j3 RNEA+offset 88–94 % (mock 90–92 %). |
| D544 | Jalan bergerak 1: MOVED → SUCCESS → REST rc 0 pada `g` pertama atau kedua; perintah → status 60–110 s (mock 70–73 s). |
| D545 | Torsi terukur puncak (semua sendi arm_1, seluruh jalan) ≤ 13.0 N·m, terjadi di TRANSIT (bukan di dwell). |
| D546 | 0 START-JUMP / START-UNSCREENED; 0 gerak tak diminta; gantry diam (H4). |
| D547 | TCP dwell vs p_cmd ≤ 1 cm (monitor `pos_err_max_settled_mm`). |

## B. Hasil terukur

§A sha256 `9cae8ac5…` ([salinan](results/p1_demo_hw/demo_hw_A_locked.md)), commit f3dcb4c.

### B1. Langkah 1 — kamera domain 79, stack MATI (17:52–18:15) — NOL gerak — [gambar + bag](results/p1_demo_hw/hw/step1/)

- Kamera naik dengan ekstrinsik terkoreksi (terverifikasi `ps`: `static_transform_publisher --x 1.7166 … --yaw 2.1688` dan
  `--x -0.1547 … --yaw -0.9189`). 0 `inference failed` (YOLOE dari `ros2_ws/`). rgbd2: peringatan IMU `Motion Module force pause`
  (tidak dipakai).
- **Percobaan 1:** botol abu/biru terdeteksi rgbd (`gray bottle` 0.55) di **(1.166, 0.542, 0.892)** — di depan **arm_2**, bukan arm_1
  (operator meletakkan kira-kira; titik nol world tidak diketahui). Operator diberi gambar proyeksi target
  ([rgbd2_annot.jpg](results/p1_demo_hw/hw/step1/rgbd2_annot.jpg)) dan memindah botol ~58 cm sepanjang meja ke arah arm_1.
- **Percobaan 2:** track kosong. Target tidak masuk bingkai rgbd (proyeksi v = 870 > 720) → hanya rgbd2, dari ~2.9 m. Uji luring
  YOLOE pada frame rgbd2: botol di target conf **0.704 @ imgsz 1280**, tidak terdeteksi @ 512 (≥ 0.25). Kamera diluncurkan ulang
  `seg_imgsz:=1280` (pengaturan detektor, bukan penyaring/kriteria; `cam_up.sh` kini meneruskan `"$@"`). Catatan: SIGINT ke
  wrapper `ros2 run` tidak mematikan anak `object_localizer` → kill PID anak.
- **Percobaan 3/4:** botol kecil gelap lalu botol putih (tinggi 7 cm) di titik target: terdeteksi (`gray`/`white bottle` 0.61–0.79)
  tetapi track **(1.94–1.95, 0.69–0.70, 0.81–0.82)**; tambahan track `cyan bottle` (3.08, −0.34, 1.01) conf 0.32–0.61 (botol di
  luar sel, wastafel belakang; operator menyingkirkan, masih terdeteksi). Depth di mask 2.87–2.93 m vs meja di samping
  2.98–3.03 → depth mengenai botol, bukan meja.
- **Diagnosa (acuan bebas = FK URDF lengan REST, g1 1.4999, g2 0.0008):** titik kamera z 1.15–1.45 di sekitar sumbu tiap lengan,
  median − URDF (+ koreksi radius link 4 cm ke arah pandang):

  | kamera | lengan (jarak) | Δx, Δy (m) |
  |---|---|---|
  | rgbd | arm_4 (dekat) | −0.01, −0.03 |
  | rgbd | arm_2 (~1.3 m) | +0.04, +0.01 |
  | rgbd2 | arm_3 (~1 m) | +0.04, −0.02 |
  | rgbd2 | arm_2 (~2.2 m) | +0.06, +0.02 |
  | rgbd2 | **arm_1 (~3 m, di samping botol)** | **+0.15, +0.02** |

  Galat rgbd2 tumbuh dengan jarak (tilt/skala, G34 hanya mengikat titik lengan dekat). Di zona demo arm_1 → ~12–15 cm.
- **z:** operator mengukur meja **0.77 m**, botol **7 cm** → centroid sejati ≈ 0.80–0.81, puncak 0.84 → z track 0.82 **benar**.
  Batas H2 z [0.85, 1.05] berasumsi meja 0.80–0.87 (voxel peta) + botol ≤ 0.30 m — salah untuk botol pendek. Tidak dilonggarkan.
- **H1:** ✓ (dengan `seg_imgsz:=1280`; satu botol di sel; botol wastafel di luar sel tetap muncul → "tepat satu track" ✗ harfiah).
  **H2: ✗** — xy ~0.20 m dari acuan kasar (1.75, 0.55); galat rgbd2 terukur di arm_1 sendiri +0.15 m; z di bawah 0.85.
  Bag 20 s domain 79 ([bag79](results/p1_demo_hw/hw/step1/bag79)): 35 pesan track.
- **Keputusan operator 18:14:** gerak nyata TIDAK jalan; lanjut **uji rantai plan-only saja** di domain 0 (nol gerak).

### B2. Langkah 2 — self-test + bring-up nyata + kamera domain 0 (18:14–18:18) — NOL gerak — [launch](results/p1_demo_hw/hw/p1demo_launch.log.gz)

- `env_collision --self-test` PASS, `interarm_collision --self-test` LULUS, `remount_check` GERBANG LULUS.
- Operator 18:15: izin bring-up (LED/papan/lengan dicek mata, di e-stop). Bring-up 1 kali (`harness/stack_up.sh`,
  `octomap_updaters:=false`): PID launch 427814, SigIgn `0x1001001`; 4× `Actuator count … '6'`; 5 spawner mati
  (`Failed loading`/`already loaded` — balapan dikenal, kini 5 bukan 1) tetapi `list_controllers` **7/7 active**; table1 NOT armed
  sekali (perintah 0 vs 1500 mm) lalu **ARMED** 3 s kemudian, table2 ARMED; 0 baris `pointcloud_octomap_updater`;
  `env_static_map` di scene (False → publish ulang → True, pola G57). **K0 LULUS** (g1 1.499995 m / −0.25°, g2 0.0008 m / −0.01°,
  lengan ≤ 0.08°). Monitor + perekam domain 0, berkas tumbuh.
- Kamera + localizer domain 0 (`seg_imgsz:=1280`, stack hidup, load 4.1/16): 0 `inference failed`; track #0 `white bottle`
  (1.938–1.947, 0.697–0.703, 0.818–0.821) = langkah 1 ≤ 1 cm; #1 `cyan bottle` (3.10, −0.33, 1.01) tetap ada.

### B3. Langkah 3–4 — rantai plan-only domain 0 (18:18–18:21) — NOL gerak — [run01](results/p1_demo_hw/hw/run01.json), [run02](results/p1_demo_hw/hw/run02.json)

- Operator tidak bisa membuka `voice_web_ui` di Chrome (mencoba `https://192.168.0.101:8080`; layanan di `:18080` pada
  192.168.0.103) → perintah teks lewat POST `/api/transcript` "please bring a bottle" (operator: "perintah sederhana, tidak perlu
  voice"). Jalur `/voice/transcript` → `target_cli._fetch` tetap diuji: "2 known tracks match — defaulting to #0" (white bottle) →
  runner `target -> #0`. 1 pemakai `/reach_fusion/execute` (r2 aman).
- **`g` #1 (18:20:12):** p_cmd (1.941, 0.698, 1.072); arm_1 **TORQUE-UNSAFE** 3/3 (j2 RNEA+offset 14.02–14.04 / 14), arm_2
  **NO-PLAN** 3/3 (jangkauan) → `failed`, 54 s.
- **`g` #2 (18:21:20):** arm_1 sampel 1 j2 14.04 ditolak, sampel 2 **PLANNED** (j2 **13.94 / 14 = 99.6 %**, j3 8.61/10); S18
  622.8 mm, env 107.2 mm (jari), retract lurus se-gantry 622.7 / antar-gantry 863.1 / env 107.2 mm → CLEAR →
  **`success arm_1 (arm_1:PLANNED) 20.27 s`**. 0 START-JUMP / START-UNSCREENED.
- /joint_states sebelum vs sesudah (28 sendi, median 3 s): **0.000°, rel 0.000 mm**.
- Catatan: p_cmd ini dari track yang salah-lokalisasi (B1); titik sebenarnya ~0.15 m lebih dekat ke basis arm_1. Sisa torsi
  0.06 N·m — penyaring meloloskan, tetapi di batas.

### B4. Langkah 5 dilewati; langkah 6 matikan (18:22)

- **Langkah 5 (jalan bergerak) TIDAK dijalankan** — H2 gagal (B1), keputusan operator 18:14. Nol perintah gerak lengan/gantry
  sepanjang sesi.
- K0 akhir LULUS ([txt](results/p1_demo_hw/hw/tmp_logs/p1demo_k0_final.txt)) = K0 awal. Runner, CLI, voice UI, localizer
  (PID anak), kamera, pemegang fifo, monitor, perekam SIGINT per PID (SigIgn tanpa 0x2); launch SIGINT → mati ≤ 60 s;
  `ros2 node list --no-daemon` **0 di domain 0 dan 79**. Crash dump sesi dihapus (`ros2` 18:01 dari `timeout ros2 topic hz`,
  `relay` 18:03 saat kamera 79 dimatikan untuk relaunch, `move_group` 18:22 shutdown); `/tmp/apport_core_*` milik root tidak
  bisa dihapus tanpa sudo. Arsip `/tmp/p1demo_*` → [hw/](results/p1_demo_hw/hw/) (`p1demo_joint_states.csv.gz` 5.7 MB tidak
  di-commit, pola arsip HW).

### B5. Papan skor + kriteria

| # | Kriteria | Hasil |
|---|---|---|
| H1 | Deteksi stabil | ✓ dengan `seg_imgsz:=1280` (512: botol 7 cm di ~2.9 m tidak terdeteksi); ✗ harfiah "tepat satu track" (botol di luar sel, wastafel) |
| H2 | Lokalisasi | **✗** — rgbd2 +0.15 m di zona arm_1 (terukur pada arm_1 vs URDF), z 0.82 < 0.85 (z benar, batas salah asumsi) |
| H3 | Plan-only | ✓ `success arm_1 (arm_1:PLANNED)` pada `g` ke-2; js sebelum = sesudah |
| H4 | Jalan bergerak | — tidak dijalankan (H2) |
| H5 | Bersih | ✓ |

| # | Dugaan | Hasil |
|---|---|---|
| D540 | langkah 1 H1 + \|dxy\| ≤ 5 cm | ✗ (imgsz; galat rgbd2 jarak jauh) |
| D541 | z ∈ [0.90, 1.05] | ✗ (0.82; meja 0.77, botol 7 cm) |
| D542 | domain 0 = langkah 1 ≤ 2 cm, 0 `inference failed` | ✓ (≤ 1 cm) |
| D543 | PLANNED pada `g` pertama, terketat j3 88–94 % | ✗ (`g` ke-2; terketat **j2 99.6 %**) |
| D544 | MOVED → REST | — tidak diuji |
| D545 | torsi terukur ≤ 13.0 | — tidak diuji |
| D546 | 0 START-JUMP, 0 gerak tak diminta, gantry diam | ✓ |
| D547 | TCP dwell ≤ 1 cm | — tidak diuji |

**Tally: 2/5 diuji (3 tidak diuji). Demo bergerak TIDAK tercapai; rantai suara-teks → YOLOE nyata → localizer → CLI → runner →
probe → penyaring terbukti di domain 0 pada HW nyata (plan-only).**

### B6. Temuan

- **F1 🔴 Cakupan kamera zona demo.** Zona arm_1 (g1 PARKIR 1.50) tidak terlihat rgbd (di luar bingkai); hanya rgbd2 dari ~2.9 m.
  Koreksi G34 benar di dekat (≤ 4 cm pada arm_2/3/4) tetapi galat rgbd2 tumbuh dengan jarak: +0.04 (1 m) → +0.06 (2.2 m) →
  **+0.15 m (3 m)**. G34 hanya mengikat titik lengan sampai rel ~1.45 dan z ≥ 1.0. Perbaikan berikut (luring): kalibrasi rgbd2
  memakai arm_1/arm_2 REST di g1 1.50 (+ meja/botol pada z ~0.8) — 6-DOF atau skala depth — dan ulang tabel B1; atau arahkan
  ulang rgbd agar melihat meja depan arm_1.
- **F2 YOLOE kecil-jauh.** Botol 7 cm di ~2.9 m: conf 0.70 @ imgsz 1280, tak terdeteksi @ 512. HW demo harus `seg_imgsz:=1280`
  (beban GPU naik; load CPU 4/16 dengan stack hidup — tidak masalah).
- **F3 Kriteria z §A salah asumsi.** Meja kayu terukur 0.77 m (bukan 0.80–0.87 voxel) dan botol uji 7 cm → centroid ~0.81. Batas
  z berikut harus dari tinggi meja + botol terukur (stand-off 0.25 di atas centroid tetap ≥ 0.20 m di atas puncak botol 7 cm).
- **F4 Torsi j2 di batas pada titik jauh.** p_cmd (1.94, 0.70, 1.07) ≈ 0.38 m dari basis arm_1 dalam xy, j2 99.6–100.3 % di 5/6
  sampel; OMPL hanya sekali menemukan cabang ≤ 14. Lokalisasi yang benar (titik ~0.15 m lebih dekat) kemungkinan lebih ringan —
  belum diuji.
- **F5 Botol di luar sel ikut ter-track** (cyan bottle (3.10, −0.33)); `target_cli` default ke tid terkecil — aman di sini
  (#0 = botol sel) tetapi bergantung urutan. Usul (belum dikerjakan): runner menolak track di luar kotak meja.
- **F6 Akses voice UI dari Chrome** gagal (operator memakai IP/port lain); dokumentasikan URL `https://192.168.0.103:18080`
  (sertifikat swa-tanda) di runbook demo.
- **F7 Harness:** SIGINT ke wrapper `ros2 run` tidak mematikan node anak → selalu kill PID anak (`lib/<pkg>/<node>`).

## C. Arsitektur rantai demo (seperti diuji di HW, domain 0)

```mermaid
flowchart LR
  subgraph Persepsi["Persepsi (domain 0)"]
    D1[D455 rgbd] --> SR
    D2[D455 rgbd2] --> SR[seg_router<br/>YOLOE 'bottle'<br/>imgsz 1280]
    TF["static TF world→cam<br/>(launch + koreksi G34)"] -.-> OL
    SR -->|/rgbd*/seg/instance_*| OL[object_localizer<br/>fusi 2 kamera]
  end
  OL -->|/detected_objects/tracks| CLI
  OL -->|/detected_objects/tracks| RUN
  V[voice_web_ui https:18080<br/>atau POST teks] -->|/voice/transcript| CLI[target_cli<br/>_fetch → #tid]
  OP((operator / fifo 'g')) --> CLI
  CLI -->|/reach_fusion/set_target '#tid'| RUN
  CLI -->|/reach_fusion/execute| RUN[demo_runner<br/>p_cmd = xyz + 0.25 z<br/>arm terdekat dulu]
  RUN -->|subprocess| PR[reach_dwell_probe<br/>RNEA+offset ≤ 14 · S18 · env map · retract]
  PR -->|plan / execute| MG[move_group<br/>+ env_static_map]
  MG --> HW[ros2_control → Kortex arm_1]
  RUN -->|sesudah MOVED| RR[return_rest arm_1 arm_2]
  RUN -->|/reach_fusion/status| CLI
```
