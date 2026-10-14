# G63 runbook (DRAF — berlaku hanya sesudah §A `docs/p1_g63_charuco.md` DIKUNCI)

Setiap baris bertanda **[GERAK]** = satu izin operator, satu langkah. **[KAMERA]** = izin kamera. Selebihnya pasif/luring.
`G=/home/user1/Documents/ceiling_arm/docs/results/p1_g63`, sesi `SES=$G/hw_<tanggal>`. Bahasa log/keluaran = Indonesia.

## S0 — sebelum gerak (tanpa lengan)
1. Preflight: `git status`, `ps` (jangan `pkill -f`), domain 0/77/79/80 kosong, identitas kamera (`rs-enumerate-devices -s`:
   rgbd 241122302297, rgbd2 234222303079, FW 5.17.0.10 keduanya), D435IF **sudah dilepas** dari arm_4 (foto) — S0e.
2. **[KAMERA]** kamera saja (domain 80) + `rec_health.py` kedua kamera TERUS-MENERUS
   (`docs/results/p1_cam6dof/depth_health_20261015/rec_health.py <ns> <detik> $SES/health_<ns>.jsonl aligned`).
3. S0b uji jangkauan: operator memegang papan A di 1.0/1.5/2.0/2.5/3.0 m per kamera → `g63_detect.py --image <png>` (tanpa prior =
   hanya ChArUco) + catat. Tidak dipakai kalibrasi.
4. S0c peta env vs awan hidup (laci dipindah). Bila laci di luar peta lama di area pose → STOP, peta = fase sendiri.
5. S0d ukur papan: 4 kotak / 5 kotak (mm), celah kerataan (≤ 1 mm), massa (g), foto dudukan → `mount.json` (bukan tebakan) →
   `g63_urdf.py --check` + `g63_poses.py` ulang → T6 mock ulang → **kunci §A** (salinan + sha256 + commit).

## S1 — bring-up nyata
`p1_demo_hw/harness/stack_up.sh` (`octomap_updaters:=false`, `enable_gantry_bridge:=true`), 7/7 active, K0 terukur
`scripts/k0_check.py --rail1 1.5 --rail2 0`, monitor torsi + perekam sendi (cek tumbuh), kamera incumbent
(`p1_demo_hw/harness/cam_up.sh`), `rec_health` tetap jalan. URDF payload dari robot_description HIDUP:
`ros2 param get /robot_state_publisher robot_description` → `g63_urdf.py --in live.urdf --mount mount.json --out $SES/payload.urdf --check`.
Papan ditempel operator saat lengan REST, lalu `g63_attach.py --mount mount.json` (harus `OK`).

## S2/S3 — per lengan, per tahap (P0 dulu; T1–T3 hanya bila tahap sebelumnya lulus)
Per pose dari `poses.json` (urutan terkunci, R0 di awal dan akhir tiap tahap):
1. Plan-only: `probe_g63.py --g63-payload-urdf $SES/payload.urdf --g63-tool-quat <q> --arms <arm> --target <xyz>
   --goal-joints '<json>' --approach 0 --plan-check --tau-max 14 --plan-attempts 8 --trials 1` → PLANNED + margin ≤ 97 %.
2. **[GERAK]** sama + `--move` (tanpa `--plan-check`). Berhenti bila bukan MOVED + dwell SUCCESS, torsi > 14, fault, REJECTED.
3. `g63_capture.py capture --session $SES --pose-id <id> --board <A|B> --role <FIT|HOLDOUT|R0> --stage <P0..> --mount mount.json`
   (rc 0 = lengan diam selama tangkap; rc 3 = bergeser → pose diulang hanya dengan izin).
4. Sesudah tiap tahap: `g63_capture.py merge … --poses poses.json` → `g63_solve.py` (FIT saja) → G6′ (R0) + kelengkapan → lapor ke
   operator sebelum tahap berikut.
5. **[GERAK]** pulang: `scripts/return_rest.py --arms <arm>` (DRY dulu) lalu `--move`.

## S4–S6
Solve akhir, G1–G7 (uji-tahan tidak pernah dipakai fit), harness kamera BARU dengan arg baru bila diterima (launch default tetap),
G5 TF hidup, D-test + G8. Akhir: REST, g1 PARKIR 1.50, K0, `g63_attach.py --remove`, shutdown PID milik sesi, domain kosong, arsip.
