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

## 1b. G61 (ditutup 2026-10-14) — yang berubah di run_g54 (`docs/p1_g61_offline.md` B4–B6; mock: A5.1/A5.2 dan A5.3/A5.4 lulus, skor 8/13)

- **Bring-up HW wajib `octomap_updaters:=false`** (runbook §1; G57 B4 OOM). **`--tau-max 14`** (operator 2026-10-12: > 12 bukan
  gagal; batas keras > 14 tetap = penjaga runner AUTO-STOP, retract > 13.5). **`--goal-correct`** (G61 C2: goal tugas di-IK ulang
  untuk pose gantry TERUKUR bila goal nominal menggeser alat ≥ 1 mm; semua penyaring probe + GOAL-MISMATCH 3 mm tetap jalan; G57
  s12 lama ev 5 GOAL-MISMATCH 3.48 mm → 0.01 mm). Batas: koreksi ditolak bila |Δq| > 2° (≈ galat rot > 0.55°) → goal lama → probe
  GOAL-MISMATCH (aman).
- **Pra-cek baru sebelum t0 (juga `--dry`)**: move_group hidup, `/get_planning_scene` ≤ 5 s dengan `env_static_map`, berkas perekam
  tumbuh dalam 3 s (+ URDF untuk `--goal-correct`). Gagal → `AUTO-STOP: pra-cek …` rc 3, tidak ada gerak. Harus dilihat di DRY:
  `pra-cek: /get_planning_scene … objek dunia ['env_static_map']` dan `pra-cek: perekam … +N B / 3 s`.
- **Penjaga kontinu:** move_group mati → AUTO-STOP KEDUA gantry (penjaga 10 Hz; zombie dihitung mati). Mock A5.4: **0.027 s**
  setelah cap waktu kill, 0 CMD sesudah stop, rc 3; kedua alat tugas aktif dalam penyaringan, belum eksekusi lengan. A5.3 DRY
  tanpa move_group ditolak oleh `s26` sebelum t0, rc 3. Ini bukti mock, belum validasi perubahan G61 di HW (G62).
- **HALTED:** penolakan penyaring (NO-PLAN, GOAL-MISMATCH, TORQUE-UNSAFE, INTERARM/ENV-COLLIDE, RETRACT-BLOCKED, START-JUMP,
  TUCK-REFUSED) = "lengan tidak bisa" → dicatat, run lanjut (hasil TIDAK LENGKAP). `EXEC-FAIL` / tanpa catatan probe = "sistem rusak"
  → AUTO-STOP KEDUA gantry (G57 B4: move_group mati → 6/6 EXEC-FAIL sambil rel terus bergerak).
- **Perekam macet > 5 s** → `PERINGATAN perekam` di runner.log + "data perekam TIDAK LENGKAP" di baris SELESAI; run TIDAK dihentikan
  (bukti, bukan keselamatan). Operator: lihat peringatan → putuskan ulang run sesudahnya.

## 2. DRY di stack nyata (tanpa gerak)

```bash
python3 -u $R/p1_g54/run_g54.py --seed <S> --plan <PLAN> --variant <V> --final-retract --goal-tol-deg 0.05 --tau-max 14 --goal-correct --dry \
  --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_par_dry_s<S> \
  --archive $R/p1_g54/hw --prefix p1hw_par_dry_s<S>_<V>_
```
Harus: K0 LULUS, "traverse di luar: []", rc 0.

## 3. Uji stop-semua di HW (SEBELUM run paralel pertama; tanya operator)

Kontrol negatif dengan injeksi BERBASIS KONDISI (G56; injeksi berbasis urutan G54 bergantung linimasa mock): jadwal s7
`$R/p1_g55/g55_selected.json` `R90sel`, `--inject-stop-on-motion 2` → AUTO-STOP pertama kali retract g2 sedang bergerak (> 2° dari
awal event, > 10° dari REST; arah gerak = menuju REST, aman). Lulus: rc 3, `inject` tercatat, lengan g2 berhenti SEBELUM REST dan
diam (perekam), `post_stop_motion` ≈ 0, tidak ada alat diluncurkan sesudah stop. Lalu pulang: `g47_home.sh` (DRY dulu).
Mock G56 B2: lulus percobaan pertama, 0.000°/0.5 s sejak t_stop + 0.5 s.
**Jangan menguji stop saat TRAVERSE di HW** (G56 A0): bridge menyelesaikan kaki `go_to_absolute` yang sudah terkirim lalu mengejar
posisi tahan JTC — rel tidak berhenti seketika (tetap di dalam persegi traverse yang sudah disaring, gantry lain diam).

## 4. Blok seed HW paralel (tiap run ditanya)

Pembanding sama-sesi: R180 (atau R90) dan R35 seed yang sama, keduanya PARALEL, sesi yang sama.

```bash
setsid nohup python3 -u $R/p1_g54/run_g54.py --seed <S> --plan <PLAN> --variant <V> --final-retract --goal-tol-deg 0.05 --tau-max 14 --goal-correct \
  --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_par_s<S>_<V> \
  --archive $R/p1_g54/hw --prefix p1hw_par_s<S>_<V>_ > /tmp/p1hw_par_s<S>_<V>.log 2>&1 < /dev/null &
```
Sesudah tiap run: `bash $R/p1_g47/g47_home.sh <S> <PLAN> <V> --dry`, lalu tanpa `--dry` sesudah persetujuan → K0.

**Diganti oleh G55:** pasangan hari ini = jadwal lama vs terpilih P1″, lihat `docs/p1_hw_day_2026-10-12.md` §2. Usul lama (G54):
1. s7: `$R/p1_g43/g43_candidates.json R180` vs `$R/p1_g42/g42f_candidates.json G42` (sim −73 s; mock G54 −84.0 s).
2. s10: R180 vs G42 (sim **+53 s**, mock G54 +59.5 s: R180 LEBIH LAMBAT di runner paralel eksklusif-traverse — hasil yang harus diuji, bukan dihindari).
3. s12 R90 (`$R/p1_g52/g52_candidates.json R90`, di dalam amplop) dan s11 R90 (hanya sesudah kaki sweep g2 → −85).

## 5. Risiko terbuka (tanya operator sebelum HW)

| # | Risiko | Mitigasi sekarang |
|---|---|---|
| P1 | Eksekusi FJT langsung melewati pemantauan TEM MoveIt | cek awal 0.01 rad + batas waktu durasi × 3 + 5 s direplikasi; jalur sama dengan return_rest (dipakai di HW sejak G16) |
| P2 | CancelGoal-semua di HW belum diuji. Bridge (kode, G56 A0): cancel di tengah traverse TIDAK menghentikan motor AZ — kaki terkirim selesai, lalu bridge mengejar posisi tahan (bisa mundur ≤ 1 kaki) | uji §3 memukul saat retract LENGAN; traverse eksklusif → gerak rel pasca-stop tetap di dalam persegi yang disaring, gantry lain diam |
| P3 | Dua lengan bergerak bersamaan di dua gantry — beban listrik / getaran struktur langit-langit belum pernah | operator mengamati run pertama; torsi per lengan tetap dijaga (≤ 14 kontinu, rencana ≤ rating) |
| P4 | Reservasi = sampel rencana (waypoint OMPL) + garis lurus retract; galat pelacakan Kortex tidak dimodelkan | margin 50 mm (MARGIN = TOLAK); galat pelacakan HW G51 < beberapa mm |
| P5 | Jadwal R180 dioptimalkan untuk P1′-par TANPA traverse eksklusif dan dengan biaya G25 lama → s10 R180 lebih lambat dari R35 di runner ini | dilaporkan apa adanya; penjadwal dengan model P1″ + traverse eksklusif = pekerjaan berikut |
| P6 | Penjaga "/joint_states basi" (1 s) bisa terpicu palsu bila runner sibuk CPU | penyaring dipindah ke proses pekerja (G54 B2); fault log dibaca inkremental (G56); di HW dua publisher /joint_states digabung per nama |
| P8 | Koreksi goal (G61 C2) mengubah goal sendi ≤ 2° dari jadwal | goal baru tetap disaring penuh oleh probe (torsi/antar-lengan/lingkungan/pulang/reservasi/GOAL-MISMATCH); koreksi hanya bila alat bergeser ≥ 1 mm |
| P9 | Penyaring reservasi 0.136 s per keadaan: tugas yang bertemu rencana penuh gantry lain menunggu 10–23 s (G61 B3, s0 HW/sim 1.29) | waktu, bukan keselamatan; usul susutkan reservasi di `MOVED` (belum dikerjakan) |
| P7 | `FAULT_RE` run_g32 tidak menangkap error ros2_control_node generik (G56 B1) | run_g54 diperbaiki; run_g32 (serial) belum — jangan andalkan run_g32 untuk menangkap error itu |
