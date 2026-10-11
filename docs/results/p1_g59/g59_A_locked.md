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

