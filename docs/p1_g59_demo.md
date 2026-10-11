# P1 / G59 — DEMO suara → YOLOE → approach: DEMO-OFF ulang dengan kode SEKARANG + prompt DEMO-HW baru

> Sesi 2026-10-12 (prompt `docs/p1_g58_offline_prompt.md`). SCOPE tetap DEMO-OFF (2026-10-08): demo P1 saja, approach-only
> (stand-off 0.25 m), tanpa grasp, tanpa angka akurasi. NOL gerak HW, stack HW mati, kamera nyata TIDAK dibuka; mock
> `ROS_DOMAIN_ID=77` (fake HW). §A dikunci (salinan `docs/results/p1_g59/g59_A_locked.md` + sha256) sebelum jalan pertama.

## A. Protokol — DIKUNCI

### A0. Pengungkapan (baca kode, sebelum jalan)

| # | Perubahan sejak DEMO-OFF (442a606) yang menyentuh jalur demo | Bawaan? |
|---|---|---|
| C1 | G42 (77e5114) probe `--goal-joints`/`--start-joints` + verdict GOAL-MISMATCH | bawaan-mati (demo memakai `--target`, pose) |
| C2 | **G44 (6ab5370) START-JUMP**: `start_valid` (/check_state_validity) SEBELUM merencana + batas lompatan waypoint 0→1 | **bawaan-HIDUP** — mengubah jalur demo; RETRYABLE di probe; `START-UNSCREENED` (tidak menjawab) = tolak, tidak diulang |
| C3 | G47 (bfc752e) `TASK_GOAL_TOL_DEG` / kirim-ulang `start_valid` | hanya tujuan sendi (demo: pose) / perbaikan C2 |
| C4 | G54 (5c6454b) `--reserve-dir`, `--out-json`, refaktor `_execute` | bawaan-mati; `_execute` = ekor lama `move_to` (G54 A0) |

- `demo_runner.POSE_REFUSALS` tidak memuat START-JUMP / START-UNSCREENED / GOAL-MISMATCH → verdict itu = "mesin", runner
  berhenti dan memanggil `return_rest` (aman: tidak mencoba lengan lain). Tidak diubah (Rule 3) kecuali B menunjukkan perlu.
- `src/reachability_gng/mobileclip_blt.ts` masih TERPOTONG (145 MB vs 599 MB di `ros2_ws/`) → persepsi diluncurkan dari
  `ros2_ws/` (`harness/cam_up.sh` sudah `cd ros2_ws`). `rgbd_perception.launch.py` tidak menjalankan `object_localizer` →
  `harness/loc_up.sh` (remap `seg/`).
- Rantai uji = harness DEMO-OFF disalin ke `docs/results/p1_g59/harness/` (keluaran `p1_g59/mock/`, monitor = monitor mock
  G58 yang sudah hidup). Mock sama dengan G58 (REST, g1 PARKIR 1.50, g2 0; K0 LULUS sebelum mulai).

### A1. 🔒 Jalan (mock, persepsi + suara palsu seperti DEMO-OFF: `fake_tracks` → POST `/api/transcript` → `target_cli` → `g`)

| # | Pos (centroid) | Harapan | DEMO-OFF |
|---|---|---|---|
| J1–J3 | P1 (1.75, 0.55, 0.97) ×3 | arm_1 MOVED → dwell → return_rest rc 0 | 3/3 MOVED, 68–70 s |
| J4 | P1, runner `--plan-only` | arm_1 PLANNED, nol gerak (js sebelum = sesudah) | PLANNED 16.7 s |
| J5 | P3 rak (2.15, −0.15, 0.85) — P+ | kedua lengan DITOLAK, tak ada MOVED | NO-PLAN ×2 |
| J6 | P6 dinding (1.20, 0.85, 1.25) — P+ | kedua lengan DITOLAK | NO-PLAN ×2 (scene MoveIt) |
| J7 | P7 sisi arm_2 (1.25, 0.55, 0.97) | arm_2 TORQUE-UNSAFE, arm_1 NO-PLAN | sama |
| r1 | `g` kedua saat J1 berjalan | "execute DITOLAK … (r1)" | ✅ |
| r2 | J8: subscriber tambahan `/reach_fusion/execute` hidup, `g` | `failed: 2 pemakai … r2`, nol probe | tidak diuji |
| r3 | J9: fake_tracks dimatikan + track kosong diterbitkan, `g` | `failed: … (r3)`, nol probe | tidak diuji |

Plus **cek YOLOE luring (tanpa kamera):** `ultralytics` YOLOE `yoloe-11m-seg.pt`, `set_classes` dengan teks dari
`mobileclip_blt.ts`, inferensi pada gambar bawaan ultralytics (`bus.jpg`, prompt `person, bus`), dijalankan dari `ros2_ws/`
(harus mendeteksi) dan dari `src/reachability_gng/` (harus GAGAL memuat = F4 masih ada).

### A2. 🔒 Gerbang G59

J1–J7 sesuai harapan (P+ J5/J6 tidak MOVED = wajib; MOVED di P+ = GAGAL), r1–r3 lulus, YOLOE luring dari `ros2_ws/` mendeteksi,
mock dimatikan bersih; prompt DEMO-HW (`p1_demo_off.md` §D) diperbarui + langkah cek kamera TANPA gerak untuk hari HW.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D510 | J1–J3: 3/3 MOVED → SUCCESS → REST (jalur bawaan tidak berubah oleh C1/C3/C4). |
| D511 | 0 START-JUMP / START-UNSCREENED di J1–J9 (start REST bebas tabrakan di scene MoveIt). |
| D512 | J5, J6: kedua lengan ditolak (NO-PLAN), sama seperti DEMO-OFF; J7: arm_2 TORQUE-UNSAFE. |
| D513 | r2 dan r3 lulus pada percobaan pertama (nol probe diluncurkan). |
| D514 | Waktu perintah→REST J1–J3 dalam ±15 % DEMO-OFF (68–70 s). |
| D515 | YOLOE luring: dari `ros2_ws/` ≥ 1 deteksi `person` di `bus.jpg`; dari `src/reachability_gng/` gagal memuat mobileclip. |

## B. Hasil terukur

§A sha256 `07582eeb…` ([salinan](results/p1_g59/g59_A_locked.md)). Mock 77 = mock G58 (REST, g1 PARK 1.50, g2 0, K0 LULUS 08:16),
monitor G58 hidup. Harness [g59_run.sh](results/p1_g59/harness/g59_run.sh), [log](results/p1_g59/g59_run.log), per jalan
`results/p1_g59/mock/runNN.json` + log alat, log runner/CLI `results/p1_g59/harness/`. Fake HW + persepsi/suara palsu — BUKAN data.

### B1. Percobaan 1 (08:05, arsip `results/p1_g59/try1/`) — DUA cacat, dihentikan

1. **`demo_runner` salah membaca verdict.** Sejak G47 probe mencetak `arm_1: moveit MOVED  (percobaan rencana: ['PLANNED'])`;
   regex `(?:moveit|rencana:) (\S+)` mengambil kecocokan TERAKHIR = `['PLANNED'])` → J1 (arm_1 MOVED + dwell SUCCESS) dilaporkan
   `failed` dan, karena bukan penolakan pose, runner tidak akan mencoba lengan lain. **D510 ✗** (C3 mengubah jalur demo).
   Perbaikan: `(?:moveit|(?<![a-z] )rencana:) (\S+)` (bukan "percobaan rencana:" / "sepanjang rencana:"; uji 3 baris contoh).
2. **🔴 r1 tembus lewat `target_cli`.** `g` kedua diketik saat J1 berjalan TIDAK sampai ke runner (CLI sedang menunggu status,
   bukan membaca stdin); begitu status J1 datang, `input()` membaca `g` yang terbuffer dan mengirim execute BARU → **run 2
   bergerak otomatis** (arm_1 ke target yang sama, 70.8 s) tanpa konfirmasi baru. Guard r1 runner (sibuk) tidak pernah melihatnya.
   DEMO-OFF S5 menguji r1 dengan execute langsung, bukan lewat CLI. Di HW: operator menekan `g` dua kali = gerak kedua tak diminta.
   **Perbaikan `target_cli._discard_typeahead`** (alasan tertulis di sini): sesudah status/timeout misi, input yang diketik
   selama misi DIBUANG (tty: `tcflush`; pipe: kuras yang terbaca) dan dilaporkan "type g again to run again".
3. Harness: `wait` tanpa PID menunggu proses latar (fifo/`sleep 86400`) → macet sesudah J1 (tidak ada jalan lain); kini `wait $PID`.

### B2. Percobaan 2 (08:17–08:30, sesudah perbaikan)

| # | Pos | Hasil | Waktu | Catatan |
|---|---|---|---|---|
| J1 | P1 | arm_1 MOVED → SUCCESS → REST rc 0 | 72.86 s | `percobaan rencana: ['PLANNED']` |
| J2 | P1 | arm_1 MOVED → SUCCESS → REST | 71.93 s | ['TORQUE-UNSAFE', 'PLANNED'] (cabang j2 15.15/14 ditolak) |
| J3 | P1 | arm_1 MOVED → SUCCESS → REST | 70.33 s | ['TORQUE-UNSAFE', 'PLANNED'] |
| J4 | P1 `--plan-only` | arm_1 PLANNED | 18.92 s | /joint_states sebelum = sesudah (maks 0.000°, rel 0.000 mm) |
| J5 | P3 rak (P+) | arm_1 NO-PLAN, arm_2 NO-PLAN → failed | 92.53 s | tak ada MOVED |
| J6 | P6 dinding (P+) | arm_2 NO-PLAN, arm_1 NO-PLAN → failed | 93.18 s | tak ada MOVED |
| J7 | P7 sisi arm_2 | **arm_2 MOVED → SUCCESS** | 75.95 s | DEMO-OFF: TORQUE-UNSAFE 3/3; kini cabang j3 9.15/10 (91.5 %), j2 9.91/14 |
| r1a | `g` kedua diketik saat J1 | DIBUANG oleh CLI (1 baris), runner tetap 1 run | — | perbaikan B1.2 bekerja |
| r1b | execute langsung saat J2 | runner "execute DITOLAK … (r1)" | — | |
| r2 | subscriber kedua `/reach_fusion/execute` | `failed: 2 pemakai … r2`, nol probe | 0 s | |
| r3 | track hilang sebelum `g` | CLI menolak dulu ("no longer visible") → nol execute; uji langsung runner (plan-only, target #209, track `[]`): `failed: target #209 tidak ada di track sekarang (r3)` | — | kedua lapis teruji |

- Penyaring di P1: RNEA+offset terketat j3 9.04/10 (90.4 %), S18 483–571 mm, env 112.5–115.1 mm (jari), semua CLEAR.
- 0 START-JUMP / START-UNSCREENED di semua log probe → **D511 ✓**. J5/J6 seperti DEMO-OFF, J7 berbeda (cabang IK stokastik:
  OMPL kini menemukan cabang j3 yang lolos; penyaring tidak berubah) → **D512 ✗** (harfiah). r2 + r3 nol probe → **D513 ✓**.
  J1–J3 70.3–72.9 s vs 68–70 (+3–5 %) → **D514 ✓**.

### B3. YOLOE luring (tanpa kamera) — [skrip](results/p1_g59/g59_yoloe.py)

Dari `ros2_ws/` (mobileclip 599 764 649 B): `bus.jpg` → **5 person + 1 bus, 6 mask, 3.6 s** (cuda:0). Dari `src/reachability_gng/`
(145 142 888 B): `get_text_pe` → `RuntimeError PytorchStreamReader … failed finding central directory` = F4 tetap → **D515 ✓**.
Berkas terpotong TIDAK dihapus (keputusan user DEMO-OFF).

### B4. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D510 | J1–J3 3/3 tanpa perubahan jalur | ✗ (regex runner rusak oleh keluaran G47; 3/3 sesudah perbaikan) |
| D511 | 0 START-JUMP/UNSCREENED | ✓ |
| D512 | J5/J6 ditolak, J7 TORQUE-UNSAFE | ✗ (J7 MOVED; J5/J6 ✓) |
| D513 | r2, r3 lulus, nol probe | ✓ |
| D514 | waktu ±15 % DEMO-OFF | ✓ +3–5 % |
| D515 | YOLOE luring | ✓ |

**Tally G59: 4/6. Gerbang G59: LULUS** (P+ J5/J6 tidak MOVED; r1–r3 lulus sesudah perbaikan r1; YOLOE luring dari `ros2_ws/`;
mock mati bersih 08:31: node 0, daemon 77 dihentikan, crash dump rviz2 sesi dihapus). Perubahan kode: `target_cli.py`
(`_discard_typeahead`), `demo_runner.py` (regex verdict). Prompt DEMO-HW baru: [p1_demo_off.md §D](p1_demo_off.md) (diganti).
