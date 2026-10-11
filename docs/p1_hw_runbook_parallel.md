# Runbook HW — P1 runner PARALEL (`run_g54`, kampanye Fase 7c → HW berikut)

> Dari G54 (2026-10-11, mock). **Belum pernah jalan di HW.** Semua langkah bergerak = **tanya operator dulu, satu langkah
> per persetujuan.** Root = `/home/user1/Documents/ceiling_arm`, `R = $ROOT/docs/results`. Bring-up, monitor, perekam, K0,
> cek origin, sweep, dan mematikan = `docs/p1_hw_runbook.md` §0–2, §4, §7 TANPA perubahan; dokumen ini hanya menambah
> yang berbeda untuk runner paralel.

## 0. Apa yang berbeda dari run_g32 (serial)

| | run_g32 (serial) | run_g54 (paralel) |
|---|---|---|
| urutan | semua event g1, lalu semua event g2 | dua thread, satu per gantry, urutan per gantry = jadwal |
| traverse | kapan saja (gantry lain diam karena serial) | **eksklusif**: hanya bila gantry lain tidak menjalankan apa pun; traverse yang menunggu didahulukan |
| tugas | probe → move_group ExecuteTrajectory | probe `--reserve-dir`: rencana nyata disaring vs **reservasi** gantry lain (CrossGantryChecker 50 mm, MARGIN = TOLAK), lalu **FJT langsung** ke `gantry_g_with_arm_controller` (move_group menyerialkan eksekusi — G54 B1) |
| retract | return_rest (garis lurus, cadangan MoveIt) | runner menyaring garis lurus vs reservasi, lalu `return_rest --no-plan` (lurus saja, di dalam reservasi) |
| penjaga | S26 sebelum tiap event | S26 **kontinu 10 Hz** (+ K0, amplop); stop apa pun → **AUTO-STOP KEDUA gantry** (SIGINT semua alat + CancelGoal-semua `/execute_trajectory` + kedua controller) |

## 1. Prasyarat

1. `docs/p1_hw_runbook.md` §0–2: bring-up nyata (`enable_gantry_bridge:=true`), 7/7 controller, bridge ARMED kedua tabel,
   monitor + perekam hidup, K0 LULUS di p0 (g1 PARK 1.50, g2 0).
2. Amplop rotasi = `results/p1_g48/rot_envelope.json` (G51: g1 −125…+90, g2 −35…+95). **Jangan pakai `--rot-max`** di HW: runner
   menolak jadwal di luar amplop sebelum t0. s11 R90 butuh kaki sweep g2 −35 → −85 dulu (runbook §4, tiap kaki ditanya).
3. CPU: jangan ada job offline berat bersamaan (penyaring reservasi jalan di probe dan di proses pekerja runner).

## 2. DRY di stack nyata (tanpa gerak)

```bash
python3 -u $R/p1_g54/run_g54.py --seed <S> --plan <PLAN> --variant <V> --final-retract --goal-tol-deg 0.05 --dry \
  --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_par_dry_s<S> \
  --archive $R/p1_g54/hw --prefix p1hw_par_dry_s<S>_<V>_
```
Harus: K0 LULUS, "traverse di luar: []", rc 0.

## 3. Uji stop-semua di HW (SEBELUM run paralel pertama; tanya operator)

Kontrol negatif G54 A2.4 di HW dengan risiko rendah: jadwal s7 R180, `--inject-stop 2:4` (AUTO-STOP saat g2 akan retract
sementara g1 sedang retract — lengan bergerak ke REST, arah aman). Lulus: rc 3, `post_stop_motion` kedua gantry ≈ 0
(≤ 0.002 rad / 1 mm), lengan g1 berhenti SEBELUM REST (bukti cancel Kortex/JTC bekerja). Lalu pulang: `g47_home.sh` (DRY dulu).
**Belum diuji di HW:** reaksi driver Kortex dan bridge gantry terhadap CancelGoal di tengah gerak (lihat risiko P2).

## 4. Blok seed HW paralel (tiap run ditanya)

Pembanding sama-sesi: R180 (atau R90) dan R35 seed yang sama, keduanya PARALEL, sesi yang sama.

```bash
setsid nohup python3 -u $R/p1_g54/run_g54.py --seed <S> --plan <PLAN> --variant <V> --final-retract --goal-tol-deg 0.05 \
  --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_par_s<S>_<V> \
  --archive $R/p1_g54/hw --prefix p1hw_par_s<S>_<V>_ > /tmp/p1hw_par_s<S>_<V>.log 2>&1 < /dev/null &
```
Sesudah tiap run: `bash $R/p1_g47/g47_home.sh <S> <PLAN> <V> --dry`, lalu tanpa `--dry` sesudah persetujuan → K0.

Urutan usul (prediksi P1″ paralel g54_sim / mock G54 B):
1. s7: `$R/p1_g43/g43_candidates.json R180` vs `$R/p1_g42/g42f_candidates.json G42` (sim −73 s; mock G54 −84.0 s).
2. s10: R180 vs G42 (sim **+53 s**, mock G54 +59.5 s: R180 LEBIH LAMBAT di runner paralel eksklusif-traverse — hasil yang harus diuji, bukan dihindari).
3. s12 R90 (`$R/p1_g52/g52_candidates.json R90`, di dalam amplop) dan s11 R90 (hanya sesudah kaki sweep g2 → −85).

## 5. Risiko terbuka (tanya operator sebelum HW)

| # | Risiko | Mitigasi sekarang |
|---|---|---|
| P1 | Eksekusi FJT langsung melewati pemantauan TEM MoveIt | cek awal 0.01 rad + batas waktu durasi × 3 + 5 s direplikasi; jalur sama dengan return_rest (dipakai di HW sejak G16) |
| P2 | CancelGoal-semua di HW belum diuji: Kortex (FJT cancel) dan bridge gantry (traverse dibatalkan di tengah) | uji §3 dulu; traverse eksklusif → paling banyak satu gantry bergerak di rel saat stop |
| P3 | Dua lengan bergerak bersamaan di dua gantry — beban listrik / getaran struktur langit-langit belum pernah | operator mengamati run pertama; torsi per lengan tetap dijaga (≤ 14 kontinu, rencana ≤ rating) |
| P4 | Reservasi = sampel rencana (waypoint OMPL) + garis lurus retract; galat pelacakan Kortex tidak dimodelkan | margin 50 mm (MARGIN = TOLAK); galat pelacakan HW G51 < beberapa mm |
| P5 | Jadwal R180 dioptimalkan untuk P1′-par TANPA traverse eksklusif dan dengan biaya G25 lama → s10 R180 lebih lambat dari R35 di runner ini | dilaporkan apa adanya; penjadwal dengan model P1″ + traverse eksklusif = pekerjaan berikut |
| P6 | Penjaga "/joint_states basi" (1 s) bisa terpicu palsu bila runner sibuk CPU | penyaring dipindah ke proses pekerja (G54 B2); di HW dua publisher /joint_states digabung per nama |
