# G63b runbook HW (berlaku: §A `docs/p1_g63b_redesign.md`)

Setiap baris **[GERAK]** = satu izin operator, satu langkah. **[KAMERA]** = izin kamera. **[OPERATOR]** = tindakan fisik operator.
Selebihnya pasif/luring. `G=/home/user1/Documents/ceiling_arm/docs/results/p1_g63`, `B=$G/g63b`, sesi `SES=$B/hw_<tanggal>`.
Pose = `$B/poses_g63b_v4.json` (TERKUNCI bersama §A). Rel g1 **1.30** untuk KEDUA fase; PARK 1.50 hanya di akhir.

Keadaan awal (2026-10-15 malam): rakitan dua papan **TERPASANG di arm_2**, g1 1.50 PARK, g2 0, semua REST, stack/kamera MATI.
⚠ Di g1 1.50/1.40 REST arm_2 + rakitan menabrak kotak env MoveIt → **jangan gerak apa pun sebelum rakitan dilepas (S1.3)**.

## S0 — sebelum gerak
1. Preflight: `git status`, `ps -eo pid,args` (jangan `pkill -f`), domain 0/77/79/80 kosong (`ros2 node list --no-daemon`),
   `df -h /`, identitas kamera (`rs-enumerate-devices -s`: rgbd 241122302297, rgbd2 234222303079, FW 5.17.0.10).
2. **[OPERATOR] uji goyang rakitan**: dorong pelan sisi bawah papan ke samping dan depan-belakang → gerak ≤ 1–2 mm, kembali.
   Gagal → kencangkan (cable tie kedua / ganjal) SEBELUM lanjut. Catat jawaban harfiah.
3. **[OPERATOR] papan catur/ChArUco lama di luar bingkai kedua kamera** (atas lemari laci, meja, lantai kiri; rak dekat arm_2 sudah).
   Meja kerja TIDAK dipindah (peta env beku memuat meja; pindah = pemindaian ulang = fase sendiri).
4. Ukuran cetak (operator: "aman") → `--square-mm A=42.0 B=42.0` kecuali operator memberi angka lain (4 kotak = 168 mm).

## S1 — bring-up + rel
1. `p1_demo_hw/harness/stack_up.sh` (`octomap_updaters:=false`, `enable_gantry_bridge:=true`), 7/7 active, monitor torsi +
   perekam sendi, **`reach_dwell_monitor` hidup** (`ros2 topic info /reach_dwell/target/arm_2` → subscriber ≥ 1; tanpa itu probe
   = INVALID, bukan lulus). Kamera incumbent `p1_demo_hw/harness/cam_up.sh` + `rec_health.py` kedua kamera terus-menerus.
2. K0: `scripts/k0_check.py --rail1 1.5 --rail2 0` → jari arm_2 di batas buka diharapkan ditolak (−0.0284, A0.6): **ukur ulang
   sekali** (perintah buka via `g63_gripper_ros.py`, GripperCommand 0.0) → bila tetap −0.028 ± 0.002 →
   `$G/g63_k0.py --rail1 1.5 --finger-arm2 -0.0284 --finger-arm3 -0.0097` sebagai acuan (dicatat); lain → STOP, tanya.
3. **[OPERATOR] lepas rakitan dari arm_2** (lengan REST). `python3 $B/g63b_attach.py --mount $B/mount_g63b_arm_2.json --boards A --remove`.
4. `python3 $G/../p1_g32/pose_to_g.py --gantry 1 --seed 63 --plan $B/g63b_rail_plan.json --variant G63b 1.30 0` (DRY: S12/S13/S28/G33
   CLEAR) → **[GERAK]** sama + `--move` → K0 `g63_k0 --rail1 1.30 …` LULUS.
5. **[OPERATOR] pasang rakitan lagi di arm_2** (A → rgbd2, B → rgbd di REST, sama seperti foto 4/5).
6. `ros2 param get /robot_state_publisher robot_description` → `$SES/live.urdf`;
   `python3 $G/g63_urdf.py --in $SES/live.urdf --mount $B/mount_g63b_arm_2.json --out $SES/payload_arm_2.urdf --check` (OK);
   `python3 $B/g63b_attach.py --mount $B/mount_g63b_arm_2.json --boards A` → `attached … t1_a2_gripper_base_link -> OK`.

## S2 — fase arm_2 (papan A2/B2), per tahap (P0 dulu; T1–T3 hanya bila tahap sebelumnya lulus)
1. Plan-only berantai tahap itu: `python3 $B/g63b_plancheck.py --board A --poses $B/poses_g63b_v4.json --payload $SES/payload_arm_2.urdf
   --arm arm_2 --out $SES/pc_arm_2_<tahap>.json --stages <tahap> --extra-env $G/s0c_extra_env.npz` → semua PLANNED, margin ≤ 97 %.
2. Per pose (urutan terkunci; R0 di awal tahap): `python3 $B/g63b_move.py --board A --session $SES --poses $B/poses_g63b_v4.json
   --payload $SES/payload_arm_2.urdf --extra-env $G/s0c_extra_env.npz --arm arm_2 --pose <id> --dry` → **[GERAK]** sama tanpa `--dry`.
   Berhenti bila bukan MOVED + dwell SUCCESS, torsi > 14, fault, REJECTED, GOAL-MISMATCH.
3. Tangkap: `python3 $B/g63b_capture.py capture --arm arm_2 --session $SES --pose-id <id> --board A B --role <FIT|HOLDOUT|R0>
   --stage <P0..> --mount $B/mount_g63b.json --save-frames` (frame lokal saja — ada orang di bingkai). Aturan diam: sendi 1–3,5–6
   std < 2e-4 rad; **j4 peak-to-peak ≤ batas §A6.1** (`STILL_EXEMPT_PP` di `g63b_capture`); rc 3 = bergeser → ulang hanya dgn izin.
4. Sesudah tahap: `python3 $B/g63b_capture.py merge --arm arm_2 --session $SES --square-mm A=42.0 B=42.0 --out $SES/ds_arm_2.json
   --poses $B/poses_g63b_v4.json` → `python3 $B/g63b_solve.py --data $SES/ds_arm_2.json --out $SES/res_arm_2_<tahap>.json`.
   **P0 = cek PROSES** (deteksi per kamera, tangkap lengkap, torsi, G6′ R0, jari ≤ 0.5°); solve P0 DIHARAPKAN ditolak penjaga
   (rank/UNDERDETERMINED) — itu benar, bukan kegagalan. Lapor ke operator sebelum tahap berikut.
5. **[GERAK]** pulang REST: `g63b_move.py … --pose REST` (`--dry` dulu). ⚠ **JANGAN `scripts/return_rest.py` selama rakitan terpasang.**

## S3 — pindah rakitan ke arm_3 (papan A3/B3)
1. arm_2 REST. `g63b_attach.py --mount $B/mount_g63b_arm_2.json --boards A --remove` → WAJIB baris
   `hapus dunia+terpasang ['g63_board_A']: sisa [] -> OK` (tanpa itu kotak A tertinggal sebagai objek dunia → arm_3 0/27, mock 10-15). **[OPERATOR] pindahkan rakitan ke arm_3**
   (orientasi sama terhadap gripper; catat muka mana menghadap kamera mana di REST; foto).
2. `g63_urdf.py --in $SES/live.urdf --mount $B/mount_g63b_arm_3.json --out $SES/payload_arm_3.urdf --check`;
   `g63b_attach.py --mount $B/mount_g63b_arm_3.json --boards B` (OK, `t2_a1_gripper_base_link`).
3. B-02 DITOLAK di plan-check mock (rakitan 41 mm dari forearm arm_4) → bila plan-only HW juga menolak: lewati, jangan ganti.
   Ulang S2 dengan `--board B --arm arm_3 --payload $SES/payload_arm_3.urdf`, capture `--arm arm_3`, merge `--arm arm_3` →
   `ds_arm_3.json`; solve gabungan: `g63b_solve.py --data $SES/ds_arm_2.json $SES/ds_arm_3.json --out $SES/res_all.json`.

## S4–S6 — akhir
Solve akhir (FIT saja) → G1–G7 + G9 (§A4), uji-tahan tidak pernah dipakai fit; harness kamera BARU dengan arg baru bila diterima
(launch default tetap); G5 TF hidup; D-test; G8. Akhir: lengan REST (via `g63b_move --pose REST`), **[OPERATOR] lepas rakitan**,
`g63b_attach … --remove`, baru boleh `return_rest`; `pose_to_g --gantry 1 … 1.50 0` (PARK, DRY lalu **[GERAK]**), K0, matikan PID
milik sesi (SIGINT ke launch, cek `node list --no-daemon`), domain kosong, arsip, laporan.
