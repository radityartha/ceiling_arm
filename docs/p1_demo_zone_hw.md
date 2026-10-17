# P1 / DEMO-ZONE-HW — teks → YOLOE → rgbd2 → koreksi zona P3 → arm_1 ke ATAS botol (approach-only), HW NYATA

> 2026-10-17 (prompt [p1_demo_zone_hw_prompt.md](p1_demo_zone_hw_prompt.md)). Lanjutan DEMO-HW (H2 gagal: rgbd2 +0.15 m) dengan
> koreksi zona yang DITERIMA hidup ([p1_demo_zone.md](p1_demo_zone.md) §B: C3 2.36 / M1 2.63 cm). Tanpa grasp. Stand-off 0.25 m.
> Alat + log: [results/p1_demo_zone_hw/](results/p1_demo_zone_hw/).

## A. Protokol — DIKUNCI sebelum kamera dibuka dan sebelum bring-up

### A0. Keadaan, keputusan operator, pengungkapan

**Keadaan sel (awal):** papan polos di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack + kamera MATI,
meja di posisi tanda (dicek ulang A2.2).

| # | Temuan / keputusan | Bukti |
|---|---|---|
| A0.1 | **Domain 0 untuk SEMUA** (kamera, localizer rgbd2, node zona, voice UI, CLI, runner, monitor, perekam, tampilan). Runner panggil probe → move_group domain stack (DEMO-HW A0.1). | DEMO-HW A0.1 |
| A0.2 | **Sumber track = `/demo_zone/tracks`.** `loc_cam.sh rgbd2` (localizer rgbd2 saja → `/loc_rgbd2/tracks`) → `zone_correct_node.py` (P3, gerbang kotak track + zona + \|z_trk − 0.805\| ≤ 0.05, botol 7 cm) → `/demo_zone/tracks`. `demo_runner.py --tracks-topic /demo_zone/tracks` (opsi baru; default `/detected_objects/tracks` = lama identik; argparse menolak `--ros-args`). `target_cli` remap `-r /detected_objects/tracks:=/demo_zone/tracks` (`rclpy.init()` = sys.argv). ⚠ peringatan ">1 localizer" CLI memakai `count_publishers` yang TIDAK ikut remap → menghitung topik asli (0 penerbit di harness ini) — tak berbahaya, dicatat. | diff `demo_runner.py`; rclpy `Node.count_publishers` docstring |
| A0.3 | 🔴 **Gerbang papan — operator (tombol, 10:2x): "arm_1 saja".** Runner `--arms arm_1` (opsi baru, default `arm_1 arm_2` = lama identik): hanya arm_1 dicoba, `return_rest --arms arm_1`. arm_2/arm_3 + papan + gripper TIDAK pernah diperintah. Bila arm_1 menolak → `failed`, tidak pindah lengan. Sisa: S18 antar-lengan tidak tahu papan di arm_2 (REST, ~0.8 m dari zona); margin S18 DEMO-HW 622 mm ≫ papan. | AskUserQuestion |
| A0.4 | **p_cmd = (xy terkoreksi, 0.805 + 0.25 = 1.055)**, tool ke bawah. Puncak botol 0.84 → ujung tool ≥ 0.21 m di atas puncak. Botol tidak di `env_static_map` → pemisah = stand-off. Node menulis z = 0.805 (bukan z track), jadi galat z track tidak masuk p_cmd; z track mentah tetap gerbang (Z2). | `zone_correct_node.py` |
| A0.5 | **Torsi:** `--tau-max 14` (operator 2026-10-12). DEMO-HW F4: (1.94, 0.70, 1.07) j2 99.6 %. Titik terkoreksi lebih dekat ke basis arm_1 (g1 1.40 → basis ≈ (1.80, 0.36)): C3 ~0.30 m, M1 ~0.19 m (vs 0.38). Mock plan-only: lihat A0.9. **Penyaring tidak dilonggarkan; MARGIN = TOLAK; TORQUE-UNSAFE = `g` lagi dengan izin baru, bukan ubah penyaring.** | DEMO-HW B3/F4 |
| A0.6 | **Tanda uji — operator (tombol, sesudah mock): "M1 utama".** M1 (1.7497, 0.5493) utama; C3 (1.8486, 0.6786) cadangan HANYA sesudah M1 sukses (izin terpisah). N1 tertutup gripper arm_1 di g1 1.40 (oklusi). Tanda = pita di meja → meja tergeser merusak acuan (DEMO-ZONE-LIVE B2) → `table_check.py` 0 px wajib. | p1_demo_zone §B |
| A0.7 | **Tampilan (permintaan operator):** `view_up.sh` = `rqt_image_view /rgbd2/seg/debug_image` + RViz demo (`demo_zone.rviz`: robot live, rencana, peta env, gambar deteksi, marker `/demo_zone/markers` dari `zone_markers.py`: kotak zona biru, botol terkoreksi hijau, track mentah merah + panah koreksi, p_cmd kuning). Hanya tampil; RViz stack tetap. | harness |
| A0.8 | `g` hanya lewat `$L/cli.fifo`, ditulis SESUDAH izin operator untuk SATU jalan (probe `--move` → dwell → `return_rest --arms arm_1 --move` otomatis). Perintah = POST `/api/transcript` "please bring a bottle". | DEMO-HW A0.4 |
| A0.9 | **Mock 77 (tugas 0c, fake HW, BUKAN data; 10:19–10:24)** — mock REST, g1 1.40 (`pose_to_g` rencana rel G63), g2 0; rekaman mentah LIVE 10-17 diputar ulang ke `/loc_rgbd2/tracks` → node → `/demo_zone/tracks` → CLI (remap) + runner `--plan-only --tracks-topic /demo_zone/tracks`. **Jalan 1 C3** (ARMS default, `#0` lewat fifo): p_cmd **(1.8527, 0.6529, 1.0550)** = xy terkoreksi frame itu (median live (1.8548, 0.6559)), urutan [arm_1 0.298 m, arm_2 0.902 m]; arm_1 TORQUE-UNSAFE j2 **14.51 (104 %)**, 14.45 (103 %), lalu PLANNED j2 13.08 (**93 %**); S18 622.7, env 128.7 mm. **Jalan 2 M1** (`ARMS=arm_1`, POST teks → `_fetch` "understood 'bottle' = track #4" lewat remap → `g`): p_cmd **(1.7258, 0.5383, 1.0550)**, urutan [arm_1] saja; PLANNED percobaan 1, terketat **j3 87 %**, j2 9.83/14 (70 %); S18 596.9, retract lurus CLEAR (se-gantry 597.2 / antar 783.3 / env 138.2 mm). `view_up.sh`: RViz demo + rqt hidup, marker zona/botol/p_cmd/panah tampil ([gambar](results/p1_demo_zone_hw/mock/view_run2.png)); `zone_markers.py` 2 cacat tipe float diperbaiki sebelum kunci. Mock mati bersih (launch 7 s, 0 node 77). | [mock/](results/p1_demo_zone_hw/mock/) |

### A1. 🔒 Kriteria

| # | Kriteria | Lulus bila |
|---|---|---|
| Z1 | Meja | `table_check.py` (kamera domain 0, stack mati) median geser fitur meja **0 px**, kontrol statis ≤ 0.3 px |
| Z2 | Track terkoreksi (botol putih 7 cm di M1; C3 bila dipakai) | rekam 15 s (`rec.sh <tanda>`, domain 0): tepat SATU track diterima di `/demo_zone/tracks`, tid sama, hadir ≥ 90 %; \|median xy terkoreksi − tanda\| ≤ **5 cm**; median z mentah ∈ **[0.79, 0.84]** (= L1–L3 LIVE). Diulang sebelum TIAP `g` bergerak bila botol disentuh. |
| H3 | Plan-only | `success arm_1 (arm_1:PLANNED)`; run JSON p_cmd xy = median Z2 ± 1 cm, z = 1.055 ± 1 mm; /joint_states sebelum = sesudah (≤ 0.05°, rel ≤ 1 mm) |
| H4 | Jalan bergerak | MOVED → dwell SUCCESS → `return_rest` rc 0 → `success arm_1`; torsi terukur ≤ 14 N·m semua sendi arm_1; 0 gerak kedua tak diminta; arm_2/3/4 bergeser ≤ 0.1°; t1_linear 1.400 ± 5 mm, rot ± 0.3°, g2 tak bergerak |
| H5 | Bersih | arm_1 REST, K0 akhir LULUS (`g63_k0 --rail1 1.40 --rail2 0`, jari = nilai awal sesi); `ros2 node list --no-daemon` = 0 di domain 0; /tmp diarsip |

**Info (bukan gerbang):** TCP dwell vs tanda dalam xy = \|p_cmd_xy − tanda\| ⊕ galat TCP (monitor `pos_err_max_settled_mm`).

**Berhenti + amankan (tanya operator):** fault / LED merah / REJECTED / NOT armed sesudah ARMED / torsi > 14 / benturan / kabel /
gerak tak diminta / Z2 ✗ sebelum `g` / orang di zona. Fail loud.

### A2. 🔒 Urutan (tiap gerak / bring-up = SATU izin tombol)

1. Self-test `env_collision --self-test`, `interarm_collision --self-test`, `remount_check` (NOL gerak).
2. Kamera domain 0, stack MATI (`perc_up.sh`, izin kamera + lampu): `table_check.py` → **Z1**; `view_up.sh`.
3. Botol putih 7 cm di **M1** (operator) → rekam 15 s → **Z2**.
4. Bring-up nyata `p1_demo_hw/harness/stack_up.sh` (izin): SigIgn, 4× '6', 7/7 active, ARMED, `env_static_map` di scene;
   K0: `k0_check.py --rail1 1.40 --rail2 0` polos dulu (diharapkan gagal HANYA di jari arm_2/arm_3 tertutup memegang papan)
   → `p1_g63/g63_k0.py --finger-arm2 <terukur> --finger-arm3 <terukur> --rail1 1.40 --rail2 0` LULUS (acuan 10-16: 0.810 /
   0.782 rad; beda > 0.02 dari acuan → stop, tanya); monitor + perekam domain 0 (berkas tumbuh); Z2 ulang singkat (track sama dengan stack hidup).
5. `vwu_up.sh` + `fifo_hold.sh` + `cli_up.sh`; runner `ARMS=arm_1 EXTRA=--plan-only` → POST teks → `g` (izin) → **H3**.
6. Runner tanpa `--plan-only` (`ARMS=arm_1`) → per jalan: izin → POST teks → `g` → **H4**. 1–3 jalan.
7. K0, matikan per PID (runner, CLI, fifo, vwu, view, monitor, perekam, `perc_down.sh`, launch stack anak-dulu bila SigIgn
   SIGINT), arsip `/tmp/p1dzh` → `results/p1_demo_zone_hw/hw/`, laporan §B.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D655 | Z1: `table_check` 0 px (meja tidak digeser sejak 10:09). |
| D656 | Z2 M1: satu track, galat terkoreksi ≤ **3 cm** (live 2.63), z mentah ∈ [0.80, 0.84]. |
| D657 | Stack hidup (domain 0): xy terkoreksi = langkah 3 ≤ 1 cm; 0 `inference failed`. |
| D658 | H3 M1: PLANNED pada `g` pertama (percobaan ≤ 3), terketat j3 85–92 %, j2 ≤ 80 % (mock 87 / 70). |
| D659 | H4 jalan 1: MOVED → SUCCESS → REST rc 0 pada `g` pertama; POST → status 40–90 s. |
| D660 | Torsi terukur puncak arm_1 (seluruh jalan) ≤ **12.0** N·m, terjadi di transit, bukan dwell. |
| D661 | 0 START-JUMP / START-UNSCREENED; 0 gerak tak diminta; arm_2/3/4 + gantry diam (H4). |
| D662 | TCP dwell vs p_cmd ≤ 10 mm (`pos_err_max_settled_mm`). |
| D663 | C3 (bila dijalankan): ≥ 1 TORQUE-UNSAFE sebelum PLANNED, atau terketat j2 ≥ 90 %. |

### A4. 🔒 Amandemen TAS (operator 10:5x, sesudah jalan 1 M1; dikunci SEBELUM data tas)

Operator: "coba cari tas" — tinggi "cek pakai kamera saja lalu tentukan area aman di atasnya", posisi "bebas di meja zona".
Di luar A1–A3 (botol putih 7 cm) → aturan tambahan, A1–A3 tidak diubah.

| # | Aturan |
|---|---|
| T0 | Prompt YOLOE diganti live `ros2 topic pub -1 /seg_prompts std_msgs/String "data: bag"` (tanpa relaunch). Botol diangkat dari meja. Perintah = POST "please bring a bag". |
| T1 | **Tinggi dari kamera.** Bukti botol (§B): z track mentah ≈ PUNCAK benda (0.834 vs puncak 0.84; kamera atap melihat permukaan atas). Rekam 15 s `/loc_rgbd2/tracks` → z_top = median z track tas. **Tolak gerak bila z_top > 1.07** (tas > 0.30 m) atau < 0.79. |
| T2 | Node zona dinyalakan ulang `-p object_height:=2·(z_top − 0.77)` → z_out = z_top → **p_cmd z = z_top + 0.25** (ujung tool 0.25 m di atas permukaan atas terukur; sisa ketidakpastian puncak ≤ ~3 cm → ≥ 0.22 m). Gerbang node (kotak track, zona, \|z_trk − z_out\| ≤ 0.05) TIDAK diubah. Tas tidak di peta env → pemisah = stand-off saja. |
| T3 | Gerbang Zb (15 s): tepat satu track tas diterima di `/demo_zone/tracks`, hadir ≥ 90 %; xy terkoreksi di zona (gerbang node). Galat xy **tidak terukur** (tanpa tanda; P3 di-fit botol putih → bias bahan tas tak diketahui) = info. |
| T4 | Plan-only dulu (H3), lalu gerak per izin (H4, kriteria sama: torsi ≤ 14, dll.). |

| # | Dugaan |
|---|---|
| D664 | YOLOE `bag` mendeteksi tas di zona sebagai satu track stabil (conf ≥ 0.3), tanpa track `bag` lain diterima. |
| D665 | z_top tas ∈ [0.85, 1.00]. |
| D666 | Plan-only tas: PLANNED pada `g` pertama; jalan bergerak MOVED → REST, torsi terukur ≤ 12 N·m. |

## B. Hasil terukur

§A sha256 `c8be80da…` ([salinan](results/p1_demo_zone_hw/demo_zone_hw_A_locked.md)) — dikunci sesudah mock, SEBELUM kamera/bring-up.

**B0. Luring (tugas 0, commit 846e1e3).** `demo_runner.py` `--tracks-topic` + `--arms` (default lama identik: urutan arm_1, arm_2;
argv `return_rest` sama). Harness [p1_demo_zone_hw/harness/](results/p1_demo_zone_hw/harness/). Mock 77 = A0.9. Sesudah kunci, 2
perbaikan harness (bukan gerbang): `perc_up.sh` `set -u` bentrok `setup.bash` (dihapus, 0 proses setengah jalan); `rec.sh` baru
(Z2 domain 0); `js_window.py` (analisis jendela /joint_states).

**B1. Preflight + kamera (10:38–10:45), stack MATI, NOL gerak.** Self-test env PASS, interarm LULUS, remount GERBANG LULUS
([log](results/p1_demo_zone_hw/hw/tmp_logs/selftest.log)). Izin kamera (tombol). 0 node domain 0 sebelum; `perc_up` launch 1726884
(**SigIgn `0x1001007`, SIGINT diabaikan** → anak-dulu), loc 1727183, node 1727184; `seg_router` imgsz **1280** (param), 0
`inference failed` sepanjang sesi; rgbd2 2× peringatan `Left MIPI error` (Hardware Notification) saat start, stream 30 Hz normal
sesudahnya (sesi 09:55 0×). Tampilan `view_up` hidup (RViz demo + rqt); layar :1 sempat screensaver.
- **Z1 `table_check`** (3 bacaan, keluaran hanya di transkrip sesi — tidak tersimpan berkas): meja median **(0, 0) px** 3/3
  (15 fitur NCC ≥ 0.85); statis kabel (0, 0) 3/3; fase kiri-atas **(+0.4, +0.3)**, (+0.3, +0.3), (+0.3, +0.3) px → bacaan 1 > 0.3
  = **✗ harfiah** (0.1 px), 2–3 ✓. Operator diberi tahu, memilih lanjut.
- **Z2 percobaan 1 — BOTOL SALAH:** operator menaruh botol GELAP (`blue bottle` 0.82; gambar debug menunjukkan botol gelap):
  L1–L3 lulus, 4.04 cm — **tidak dihitung** (§A = botol putih), disimpan [info_dark_m1_stackoff/](results/p1_demo_zone_hw/hw/info_dark_m1_stackoff/).
- **Z2 botol putih M1** (10:45, stack mati): `white bottle` #3, 30/30, **3.35 cm** (mentah 14.38), z mentah **0.834** → ✓.

**B2. Bring-up nyata (10:45:37, izin) + K0.** Launch 1729508 SigIgn `0x1001001`; 4× '6'; 0 `pointcloud_octomap_updater`;
`list_controllers` 7/7 active (2 spawner "already loaded" = balapan dikenal); table1 NOT armed sekali → ARMED 2 s kemudian, table2
ARMED. `env_static_map` di scene: kosong → `env_static_map_pub.py --once` → True (pola G57). K0 polos TOLAK hanya jari arm_2
**0.8095** / arm_3 **0.7820** (acuan 10-16 0.810/0.782 ✓) → `g63_k0` **LULUS**. Monitor + perekam domain 0 (CSV tumbuh).
- **Z2 stack hidup:** #4 `white bottle` **3.38 cm**, z 0.834 (beda dari stack mati 0.03 cm).

**B3. H3 plan-only (10:47:52, izin).** POST "please bring a bottle" → CLI (remap) "understood 'bottle' = track #4" → runner `#4`;
`g` → **`success arm_1 (arm_1:PLANNED) 19.64 s`**, percobaan 1, terketat **j3 84 %**, j2 8.55/14 (61 %); S18 582.2, env 155.3 mm;
retract lurus CLEAR (582.3 / 781.3 / 155.2). p_cmd **(1.7166, 0.5375, 1.0550)** vs Z2 (1.7176, 0.5387) = 0.16 cm, z tepat.
/joint_states sebelum−sesudah maks **0.0002°**, rel 0 mm → **H3 ✓** ([run01](results/p1_demo_zone_hw/hw/run01.json)).

**B4. H4 jalan bergerak 1 — M1 (10:49:33–10:50:53, izin).** Runner EXECUTE `--arms arm_1`. p_cmd (1.7167, 0.5383, 1.0550).
⚠ Rencana eksekusi = cabang OMPL lain: prediksi **j2 13.61/14 (97 %)** (plan-only 61 %) — lolos penyaring (tidak dilonggarkan),
dicatat. S18 534.5, env 135.7 mm, retract CLEAR → **MOVED** → dwell **SUCCESS** (time-to-dwell 32.7 s, `pos_err_max_settled`
**0.79 mm**) → `return_rest --arms arm_1` rc 0 (45.1 s, galat 0.036°, torsi 5.18) → **`success arm_1 (arm_1:MOVED) 79.42 s`**.
Terukur: **puncak j2 10.40 N·m** di +31.6 s (ujung transit; dwell 10.15 → beban tahan statis), sendi lain ≤ 1.76; arm_2/3/4 maks
**0.009°**; t1/t2 linear + rotasi **0.000**; 0 START-JUMP/UNSCREENED; 0 gerak kedua → **H4 ✓**
([run02](results/p1_demo_zone_hw/hw/run02.json), [js](results/p1_demo_zone_hw/hw/tmp_logs/js_run02.txt)).
Galat demo terhadap tanda (info): \|p_cmd_xy − M1\| = 3.5 cm ⊕ TCP 0.8 mm.

**B5. Operator: pertanyaan kamera + "cari tas" (10:5x).**
- Jawaban kamera: rgbd2 saja karena rgbd tidak melihat meja arm_1 (DEMO-HW F1) dan P3 di-fit data rgbd2; bagi area per kamera
  (rgbd dekat arm_2/3/4, rgbd2+P3 di meja arm_1) = usul fase baru.
- **A4 TAS dikunci** (sha 67610401, commit 03f47bc) sebelum data. Prompt live `bag` → tidak ada deteksi; operator: **tas di
  rak/lemari, bukan di meja**. Info (NOL gerak): prompt `paper bag,shopping bag,bag` + `loc_cam rgbd` sementara → tas kertas putih
  di atas lemari: rgbd `gray paper bag` 0.29 (1.001, −0.807, 1.094), rgbd2 0.26 (0.956, −0.855, 1.076) — beda 6 cm; plus tas
  coklat lantai (1.565, −0.822, 0.383). Lemari di sisi gantry 2 (y −0.82) = "rak" G32; runner hanya kenal arm_1/arm_2, arm_3
  memegang papan → **gerak ke tas TIDAK dijalankan**; operator: "tas tetap di rak, fase baru". A4 T1–T4 **tidak diuji**.

**B6. Matikan (10:58–11:02).** K0 akhir `g63_k0` **LULUS** (= awal). ⚠ Satu panggilan `kill` memakai pola awk `[r]qt_image_view`
yang cocok dengan teks baris perintah sendiri (`lib\/rqt_image_view`) → shell sendiri ter-TERM (exit 144, jebakan G33) sebelum
`perc_down`; sinyal ke helper sudah terkirim. Lanjut: `perc_down.sh` (11 anak launch SIGINT, 0 sisa), launch stack SIGINT → mati
5 s; **0 node domain 0/77/79**. Crash dump rviz2 mock 10:24 (174 MB, milik sesi) dihapus. Arsip `/tmp/p1dzh` →
[hw/](results/p1_demo_zone_hw/hw/) (`p1dzh_joint_states.csv.gz` 12.8 MB tidak di-commit; frame berisi orang tidak di-commit).

### B7. Papan skor

| # | Kriteria | Hasil |
|---|---|---|
| Z1 | meja 0 px, statis ≤ 0.3 | meja ✓ 0 px 3/3; statis ✗ harfiah bacaan 1 (0.4 px), ✓ bacaan 2–3 |
| Z2 | satu track, ≤ 5 cm, z [0.79, 0.84] | ✓ 3.35 (stack mati) / 3.38 cm (stack hidup), z 0.834 |
| H3 | plan-only | ✓ PLANNED, p_cmd 0.16 cm dari Z2, js 0.0002° |
| H4 | jalan bergerak | ✓ 1 jalan (79.4 s, torsi 10.40, REST rc 0, lengan lain/gantry diam) |
| H5 | bersih | ✓ K0 akhir LULUS, 0 node, arsip |

| # | Dugaan | Hasil |
|---|---|---|
| D655 | Z1 0 px | ✓ meja (statis bacaan 1 0.4 px dicatat di Z1) |
| D656 | Z2 ≤ 3 cm, z [0.80, 0.84] | ✗ 3.35 cm (z 0.834 ✓) |
| D657 | stack hidup = langkah 3 ≤ 1 cm, 0 inference failed | ✓ 0.03 cm, 0 |
| D658 | PLANNED `g` pertama, j3 85–92 %, j2 ≤ 80 % | ✗ j3 **84 %** (di luar rentang 1 poin); PLANNED pertama ✓, j2 61 % ✓ |
| D659 | jalan 1 MOVED → REST `g` pertama, 40–90 s | ✓ 79.4 s |
| D660 | torsi ≤ 12.0, di transit | ✓ 10.40, ujung transit (dwell 10.15) |
| D661 | 0 START-*, 0 gerak tak diminta, lain diam | ✓ |
| D662 | TCP dwell ≤ 10 mm | ✓ 0.79 mm |
| D663 | C3 TORQUE-UNSAFE / j2 ≥ 90 % | — tidak diuji (C3 tidak dijalankan) |
| D664–D666 | tas (A4) | — tidak diuji (tas di rak, di luar zona) |

**Tally: 6 ✓ / 2 ✗ / 4 tidak diuji. DEMO BERGERAK TERCAPAI: teks → YOLOE → rgbd2 → koreksi zona P3 → CLI → runner → probe →
arm_1 0.25 m di atas botol M1 → REST, HW nyata, 1 jalan.**

### B8. Temuan
- **F1 Cabang OMPL beda antara plan-only dan eksekusi:** titik sama, prediksi j2 61 % vs 97 %. Plan-only bukan jaminan margin jalan
  bergerak; penyaring per-rencana tetap yang menjaga. Terukur 10.40 (statis di target ~10.1).
- **F2 Botol salah bisa lolos gerbang:** botol gelap lulus L1–L3 (4.04 cm) — gerbang tidak memeriksa jenis botol; label YOLOE
  (`blue`/`white`) berkedip untuk benda yang sama (log node #3 `blue` → `white`). Cek mata/gambar debug sebelum Z2.
- **F3 Tas di rak** terdeteksi kedua kamera dengan prompt `paper bag` (conf 0.26–0.29, beda 6 cm), di sisi gantry 2 → fase baru:
  arm_3/arm_4, rak = zona tabrakan G32, papan masih di arm_3, runner hanya arm_1/arm_2, tanpa koreksi zona (rgbd dekat).
- **F4 Bagi area per kamera** (usul operator): rgbd untuk zona dekat (arm_2/3/4, G34 ≤ 4 cm), rgbd2 + P3 untuk meja arm_1.
- **F5 Pola kill:** jangan menaruh pola regex yang muncul juga sebagai teks literal di baris perintah (mis. `lib\/rqt_image_view`
  memuat `rqt_image_view`) — pakai skrip berkas atau `comm`.

