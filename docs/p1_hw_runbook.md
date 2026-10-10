# Runbook HW — P1 blok R180 (kampanye G43+, Fase 6)

> Dari G43–G48 (2026-10-10). Semua langkah bergerak = **tanya operator dulu, satu langkah per persetujuan.** Berhenti dan
> amankan (lengan REST, gantry diam) pada: fault/LED merah, `REJECTED`/`NOT armed` di log bridge, torsi > 14 N·m, benturan
> atau kabel tegang yang dilihat operator, rc ≠ 0 yang tidak dijelaskan. Root = `/home/user1/Documents/ceiling_arm`,
> `R = $ROOT/docs/results`.

## 0. Prasyarat (tanpa gerak)

1. `python3 scripts/remount_check.py` — subnet 192.168.2.x punya IP (`enp112s0`), ICMP .10–.13, tidak ada tab web Kinova,
   tidak ada move_group basi. Bila tanpa IP: `sudo ip addr add 192.168.2.100/24 dev enp112s0` (lihat memori jaringan).
2. Tidak ada stack lain hidup: `ros2 node list --no-daemon` kosong; `ps` tanpa move_group / ros2_control_node.

## 1. Bring-up nyata

```bash
cd $ROOT && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
setsid nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','launch','workcell_moveit_config','my_workcell.launch.py','use_fake_hardware:=false','enable_gantry_bridge:=true','use_sim_time:=false'])" > /tmp/p1hw_launch.log 2>&1 < /dev/null &
```

Gerbang (tunggu, lalu periksa — semuanya harus benar):
- PID launch ASLI di-resolve ulang SESUDAH controller naik (`ps -eo pid,args | awk '$3=="/opt/ros/humble/bin/ros2" && $4=="launch"'`),
  `grep SigIgn /proc/<pid>/status` → bit SIGINT (0x2) TIDAK diset.
- Log: 4× `Actuator count … '6'`; `ros2 control list_controllers` 7/7 active; bridge `table1 ARMED` dan `table2 ARMED`.
- `/table_hw_commands` (perintah bridge) diam selama tidak ada gerak (risiko G36b: tujuan JTC lengan-saja saat perintah rel
  basi ≠ keadaan → rel bisa ter-ramp tanpa saringan).
- Peta lingkungan: `env_static_map` ada di `/get_planning_scene` (launch `env_map:=true` default).

Monitor + perekam:
```bash
setsid nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','run','reachability_gng','reach_dwell_monitor','--ros-args','-p','csv_log:=/tmp/p1hw_mon'])" > /tmp/p1hw_monitor.log 2>&1 < /dev/null &
setsid nohup python3 $R/p1_g22/js_record.py /tmp/p1hw_joint_states.csv > /tmp/p1hw_rec.log 2>&1 < /dev/null &
```

## 2. K0 terukur (tanpa gerak)

`python3 scripts/k0_check.py --rail1 1.50 --rail2 0.00` (g1 PARKIR, g2 home). TOLAK → JANGAN lanjut; pulihkan dengan
`return_rest.py --arms … ` (DRY dulu, lalu `--move` setelah persetujuan) dan `pose_to_g` ke PARK/p0. Nilai yang dicetak
dicatat apa adanya (K0 lisan TIDAK dipakai — G36b).

## 3. Fase 6a — cek origin g2 dengan kamera (tanpa gerak)

Tujuan: membedakan (A) encoder g2 slip sesudah G34 (perlu re-zero) dari (B) stopper home ~35 mm di luar URDF 0 (JANGAN re-zero).
1. Lengan REST, g2 di encoder 0.000 (K0), g1 di PARK 1.50.
2. Kamera: `ros2 launch reachability_gng realsense_dual.launch.py with_color_cloud:=false` (ekstrinsik TIDAK diubah).
3. `python3 $R/p1_g33/capture_cloud.py --joints --seconds 20` → `$R/p1_g48/cal_g2_origin.npz`.
4. `cd $R/p1_g34 && python3 rail_calib.py --capture cal_00.npz cal_06.npz cal_07.npz ../p1_g48/cal_g2_origin.npz
   --config capture capture capture capture --groups g2 --diff --out ../p1_g48/g2_origin.json` — registrasi per tangkapan
   × kamera untuk lengan g2 (`--diff`: latar dibuang memakai cal_06, g2 di 1.45 m). Bandingkan translasi x (arah rel) g2
   tangkapan baru dengan cal_00 / cal_07 (G34, g2 = 0, keterulangan) per kamera.
   **Baseline (G48, sudah dihitung, `results/p1_g48/g2_origin_baseline.json`):** kamera **rgbd** (800 titik, kedua lengan g2,
   yaw ≈ 0): tx = **−0.0916** (cal_00) / **−0.0955** (cal_07) → keterulangan ≈ 4 mm. rgbd2 TIDAK dipakai (hanya satu lengan,
   yaw 15–17° — fit tidak andal).
5. Keputusan pada Δx = tx_baru(rgbd) − (−0.0936) (ketelitian kamera ~4 mm keterulangan, ≤ 0.8 cm residual; prompt meminta
   ±2 mm = DI BAWAH ketelitian kamera, jadi ambang akhir dengan pita ukur):
   - |Δx| ≤ 10 mm → tidak ada slip (B): origin tetap, catat jarak fisik stopper.
   - |Δx| ≥ 25 mm → slip (A): **re-zero HANYA bersama operator** (`scripts/table_keyboard.py` `H` = `ppreset` SEMUA 3 motor
     g2 termasuk rotasi — rotasi harus tepat 0 dulu).
   - di antara → operator mengukur jarak kereta ke stopper dengan pita (bandingkan dengan foto/pita G34 bila ada); tanya.
6. Ambang ±2 mm untuk keputusan akhir = pita ukur fisik, bukan kamera.

## 4. Fase 6b — sweep rotasi bertahap (SETIAP kaki ditanya)

Alat: `bash $R/p1_g48/g48_sweep.sh <g> <kaki> <kaki>` (satu kaki per panggilan; tanpa `--dry` = bergerak). Kaki
`g48_sweep_legs.json`: g1 = ke stasiun (0.65, 0), lalu 0 → +A → 0 → −A → 0 untuk A = 10, 20, 35, 50, 70, lalu 0 → −90 → 0,
0 → −110 → 0, 0 → −125 → 0, lalu PARK (1.50, 0) — 28 kaki; g2 = (0.15, 0), A = 10, 20, 35, lalu (0, 0) — 14 kaki. Gantry
lain DIAM di PARK/home. Setiap kaki: DRY dulu (`… --dry`), lalu `--move` sesudah persetujuan.

Sesudah TIAP kaki (dari JSON `pose_to_g`): |err_deg| ≤ **0.3°**, |err_mm| ≤ 2, `t_traverse` ≤ 1.2 × `T_cmd`, drift lengan
≤ 0.5°, rel/rot lain diam, torsi lengan ≤ 14; operator memeriksa kabel + jarak bebas. **Berhenti** pada pelanggaran mana pun,
`REJECTED`/`NOT armed`/fault, atau permintaan operator → lengan sudah REST; kembalikan rotasi ke 0 dengan kaki balik
(DRY dulu). Catat `t_rot / T_cmd_rot` tiap amplitudo (model `rot_cmd` = 0.9235 hanya terukur di 10°).

Selesai bila g1 lulus sampai −125 dan +70, g2 sampai ±35 (ROT_NEED). Seed yang butuh lebih sedikit (s3, s18: 35°; s7, s10:
70°) bisa dijalankan begitu amplitudonya lulus.

## 5. Fase 6c — `run_g32 --dry` di stack nyata (tanpa gerak)

g1 harus di p0 jadwal: PARK 1.50 = p0 semua kandidat (g42f/g43: p0 g1 1.50, g2 0.00) → tidak perlu pindah.

```bash
python3 -u $R/p1_g32/run_g32.py --seed <S> --plan <PLAN> --variant <V> --final-retract --rot-max 125 --goal-tol-deg 0.05 \
  --dry --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_dry_s<S> \
  --archive $R/p1_g49/hw --prefix p1hw_dry_s<S>_
```
Kandidat: `<PLAN> = $R/p1_g43/g43_candidates_6.json`, `<V> = R180` untuk s0 s3 s7 s10 s18; s8 = `$R/p1_g42/g42f_candidates.json`
`G42`. K0 di dalam runner; rc 134 saat keluar = benign.

## 6. Fase 6d — blok seed HW

Pembanding sama-sesi (G32 B7 (2)): R_max dan R0 di sesi yang SAMA. Urutan usul: s7 (gain terbesar, 70°) → s10 → s0 (125°,
paling akhir) → s3 / s18 / s8. Tiap seed: plan-only NYATA k = 3 (`g31_screen.py --task-cli --final-retract --repeats 3`,
`G47_GOAL_TOL_DEG=0.05`) → `run_g32` di atas tanpa `--dry` (setsid nohup) → pulang:
`bash $R/p1_g47/g47_home.sh <S> <PLAN> <V>` (retrace stop jadwal; pulang langsung ke PARK bisa DITOLAK G33 — G47 B4) → K0.

## 7. Matikan

`return_rest` + PARK (K0 LULUS) → SIGINT ke PID launch ASLI (cek SigIgn), tunggu ≤ 60 s; monitor + perekam SIGINT per PID
(JANGAN `pkill -f`); `ros2 node list --no-daemon` = 0; arsipkan `/tmp/p1hw_*` ke `$R/p1_g49/hw/`, crash dump sesi dihapus.
