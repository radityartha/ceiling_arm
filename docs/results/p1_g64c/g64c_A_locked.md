## A. Protokol — DIKUNCI sebelum kode

### A0. Keputusan operator, definisi, pengungkapan

| # | Isi |
|---|---|
| A0.1 | **Desain C (operator, tombol 2026-10-17 sore).** Penjadwal P1″ memilih per gantry per jadwal mode **REST** (= sistem sekarang, objek/predikat/cache apa adanya) atau **T1**. A (manual) dan B (whole-body) ditolak (§1.2 G64). Biaya mode masuk makespan. |
| A0.2 | **T1** = `results/p1_g64/g64_transit_T.json` q6 = [−0.04589, 1.0234, 2.50309, −1.57319, 1.47987, 1.61665] rad, SAMA untuk keempat lengan (seperti what-if G64; REST juga satu q6 untuk semua lengan). j2 statis mentah 4.04, titik terendah 1.277 m (REST 0.924), radius 0.462 m. |
| A0.3 | **Lipat / buka** = SATU garis sendi lurus kosinus 12-D kedua lengan gantry g (REST↔T1, 61 titik), seperti retract `return_rest` dua lengan. Disaring: torsi statis RNEA + 7.7 + 0.66 ≤ 14 tiap titik tiap lengan, EnvChecker ≥ 50 mm (t{g}_), kotak scene ≥ 10 mm tiap titik, se-gantry CLEAR, antar-gantry (CrossGantryChecker, gantry lain di keadaannya) CLEAR. Tolak = `TUCK-REFUSED`, nol gerak. Lipat/buka hanya di **p0** (rel p0, rot 0). |
| A0.4 | **Mode T1 untuk gantry g** (urutan event): `fold g` di p0 → [retract → traverse] dengan kedua lengan T1 → tugas: garis sendi lurus dari keadaan lengan (T1 atau ujung tugas sebelumnya di stop yang sama) ke tujuan sendi, dwell → retract kedua lengan **ke T1** sebelum tiap traverse → sesudah tugas terakhir: retract ke T1, **traverse pulang ke p0** (T1), `unfold g` di p0. Struktur tugas di stop = REST sekarang (G42 (s42)): REST diganti T1 di tempat REST muncul. "Tugas T1→q→T1" prompt = garis tuple T1↔q (tugas tunggal) dan retract ke T1. |
| A0.5 | **Kenapa pulang ke p0:** alasan T1 = REST di stop gagal (TAS: REST di stop −12 mm, §0.1 G64) → buka di stop terakhir akan gagal tepat di stop yang dibuka T1. Buka hanya di p0 tersertifikasi. Sel berakhir REST di p0 untuk gantry mode T1 (gantry mode REST tetap berakhir di stop terakhir = sekarang). |
| A0.6 | **Mode REST** = rantai G55 R90 (Feas49, EdgeMatrix G46, Stops (s42), solve_lazy46, P1″) apa adanya, cache = SALINAN g55 di `results/p1_g64c/`. Tidak ada satu baris predikat REST yang diubah. |
| A0.7 | **Objek T1 (berkas BARU, g46/g40/g55 tidak ditimpa):** (a) kisi halus `g64c_fine_T1.npz` (= g46 `fine`, 0.00–1.60 m × −180…180°, kedua lengan T1); (b) tabel pose `g64c_pose_T1.json`: kedua lengan T1 di (rel, rot): env ≥ 50 mm ∧ kotak ≥ 10 mm; (c) tuple T1 `_tuple_job49T` = `_tuple_job49` dengan: urutan cabang max\|q − T1\| naik, E1 partner T1 (env ≥ 50, kotak ≥ 10), garis T1→q kosinus 61 titik dengan torsi/env/kotak **tiap titik** (what-if lama cek kotak tiap titik ke-3 — TIDAK dipakai)/se-gantry; cache `g64c_tuple49T_cache.jsonl`; tanpa cermin 180°; (d) stop (s42) T1 = `_stop_job` dengan awal kedua lengan T1 dan retract akhir ke T1, cache `g64c_stopT_cache.jsonl`; (e) EdgeMatrix T1 dari (a). |
| A0.8 | **Solver:** 4 kombinasi (mode g1 × mode g2); per kombinasi F/E/SX gabungan yang membagi per gantry ke objek mode-nya; `solve_lazy46` tidak diubah. Gantry mode T1: DP dengan h[0][r] = T[r, r0] (pulang ke p0, h[0][r0] = 0) + konstanta `c_fold + c_unfold` bila gantry mendapat ≥ 1 tugas (gantry tanpa tugas = mode REST, tidak dilipat). Pilih makespan terkecil; seri → lebih sedikit gantry T1 → REST/REST dulu. Semua kombinasi dan alasan gagalnya dicatat. |
| A0.9 | **Biaya lipat/buka:** diukur di MOCK 77 (fase 4) = waktu dinding event fold / unfold (perintah → selesai), median k = 3, per gantry. Sebelum terukur solver memakai **SEMENTARA** `c_fold = c_unfold = a_r` P1″ = 45.34 s (biaya retract-sebelum-pindah terukur, besaran sama) — DITANDAI sementara di tiap keluaran; gerbang fase 4 = solve ulang dengan nilai terukur. Retract ke T1 dan tugas T1→q memakai `a_r`/`a_t` P1″ yang sama (sementara; diukur mock, selisih dilaporkan). |
| A0.10 | **Amplop rotasi HW** (`p1_g48/rot_envelope.json`, g1 −125…+90, g2 −35…+95): bendera `G64C_ENV=1` menolak (pose_ok = False) kolom di luar amplop untuk KEDUA mode; bawaan 0 (= sistem sekarang, runner yang menolak). K-REST dijalankan dengan 0; DEMO-SYS memakai 1. |
| A0.11 | **Runner (fase 3):** event `fold`/`unfold` (A0.3); `pose_to_g` S12 menerima "REST ATAU T1 ±0.5° kedua lengan" hanya dengan bendera eksplisit `--allow-t1` (bawaan = argv lama, REST saja); retract `return_rest --target T1`; probe `--start-pose T1` (START-JUMP terhadap T1); g31_screen DRY menempatkan T1; reserve.py tahu keadaan T1; g47_home buka T1→REST di p0 sebelum PARK bila lengan di T1. Jadwal kandidat membawa `mode: {"1": "REST"\|"T1", "2": ...}`; tanpa `mode` = REST (kandidat lama bit-identik). |
| A0.12 | **Pengungkapan:** statis saja (dinamika transit = penyaring rencana MoveIt/RNEA di mock/HW). Traverse antar-gantry dengan T1 tidak disaring solver (= sekarang; runner `sweep_rot` menyaring). Memori fold: RNEA meremehkan angkat ×2.74 pada lipat PENUH; G64 §0.7: RNEA + 7.7 benar pada 513 segmen → lipat diukur di HW di tempat dulu. Penyaring tidak dilonggarkan; MARGIN = TOLAK; kotak 10 mm (G41b). |

### A1. 🔒 Kriteria (✓/✗ harfiah)

| # | Fase | Lulus bila |
|---|---|---|
| C1 | 1 sertifikasi | T1 di p0 g1 **1.40** dan **1.50**, g2 **0** (rot 0, gantry lain REST di p0-nya): pose kedua lengan T1 env ≥ 50 mm ∧ kotak ≥ 10 mm; garis lipat 12-D lolos A0.3 seluruhnya (torsi, env, kotak, se-gantry, antar-gantry). 3/3 posisi. |
| C2 | 2 K-REST | 8 seed `g55_selected.json` R90sel lewat kombinasi REST/REST: stop, tugas, tujuan sendi, makespan **bit-identik** 8/8. |
| C3 | 2 TAS | `tasks_synth.json` (p0 g1 1.40 / g2 0, `G64C_ENV=1`): TAS z 1.34 **DITERIMA** dengan mode g2 = T1; M1 tetap DITERIMA; jadwal bersama (M1 + TAS) ada, \|rot\| di amplop HW. |
| C4 | 2 negatif | (1.0, 0, 0.60) **DITOLAK** (semua kombinasi). |
| C5 | 3 runner | Mode REST: argv tiap event `run_g54`/`run_g32` DRY untuk 2 seed lama = sebelum perubahan (diff kosong); kandidat tanpa `mode` = REST. |
| C6 | 4 mock (i) | 2 seed lama mode REST plan-only + eksekusi mock: hasil = G57 (semua tugas PLANNED/SUCCESS, urutan event sama). |
| C7 | 4 mock (ii) | TAS mode T1: plan-only semua PLANNED; eksekusi mock: fold → traverse g2 OLEH SISTEM (run_g54) → tugas SUCCESS → retract T1 → pulang → unfold; 0 TUCK-REFUSED / START-JUMP. |
| C8 | 4 mock (iii) | Kontrol negatif ditolak solver (0 event dijalankan). Waktu fold/unfold terukur (k = 3) + RNEA rencana lipat j2 + 7.7 ≤ 14; solve ulang C3 dengan biaya terukur tetap DITERIMA. |
| C9 | 5 HW | Per gerak satu izin: lipat di tempat 1 lengan → torsi terukur ≤ 14 semua sendi, \|ukur − (RNEA+7.7)\| dilaporkan → buka → kedua lengan → traverse pendek T1 → TAS lewat `run_g54`. Tiap gerak: tidak ada AUTO-STOP, 0 gerak tak diminta. |

**Berhenti + amankan (tanya operator):** fault / LED merah / torsi > 14 terukur / benturan / gerak tak diminta / gerbang ✗. Fail loud.

### A2. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D668 | C1 lulus 3/3; garis lipat j2 statis maks + 7.7 ≈ 11.5 (§1.2 G64: 11.52), env minimum garis ≥ 0.10 m. |
| D669 | C2 8/8 bit-identik (menurut konstruksi). |
| D670 | Solver 4-kombinasi pada 8 seed: mode T1 dipilih **0/8** (lipat + pulang + buka ≥ 100 s > untung). |
| D671 | Tabel pose T1 g2 punya lebih banyak pose layak dari REST (titik terendah +0.35 m). |
| D672 | C3: TAS diterima arm_4 @ g2 rel 0.80–1.30, rot +50…+95, mode g2 T1; M1 arm_1 @ g1 mode REST. |
| D673 | C4: L1 kosong, ditolak seketika (= DEMO-SYS). |
| D674 | Mock: fold/unfold 8–25 s per event; RNEA rencana lipat j2 + 7.7 ≤ 12.5. |
| D675 | Mock TAS: tugas PLANNED ≤ 3 percobaan; tidak ada TORQUE-UNSAFE (tujuan sendi, §0.8 G64). |
| D676 | HW lipat: puncak j2 terukur ≤ 12.5 (prediksi ≤ 11.7); tidak di atas RNEA + 7.7. |
