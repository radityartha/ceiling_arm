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

## B. Hasil terukur

§A sha256 `c8be80da…` ([salinan](results/p1_demo_zone_hw/demo_zone_hw_A_locked.md)) — dikunci sesudah mock, SEBELUM kamera/bring-up.

