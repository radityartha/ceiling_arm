## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

- Kode: `reach_dwell_probe.py` (main, `dual_trial`, `move_to`, `_plan_and_screen`, `screen_*`), `run_g32.py`,
  `g31_screen.py`, `g41_make.py`, `g40_make.py`, `sched._dur_table/stop_duration/_reconstruct`, monitor (sumbu approach).
- `g41b_run.log`: tugas PERTAMA yang tak layak per seed gagal (hanya yang pertama dilaporkan solver). G41b gagal **15/20**
  (prompt menulis 14/20 = angka G41; s19 baru gagal di G41b karena margin kotak) → diagnosa mencakup **15** seed.
- Fakta kode: `run_g32` memanggil probe dengan `--arms <arm>` (jalur `dual_trial`, `retract_check=True`, ketiga lengan
  lain disaring); `_plan_and_screen` sudah menerima tujuan dict sendi (`_joint_goal`, toleransi 0.2°); solusi oracle yang
  disimpan = tanpa tilt (tool tegak lurus ke bawah, POS_TOL Newton) — tilt hanya untuk amplop torsi.
- `run_g32` + `rclpy` global executor: thread `Watch` memutar executor global → `Probe` di proses yang sama akan bentrok
  executor. Karena itu uji DRY TIDAK dijalankan di dalam `run_g32` (A2).
- Tidak ada perhitungan baru sebelum mengunci.

### A1. 🔒 Runner HW — perubahan kode

**`scripts/reach_dwell_probe.py`** (default = perilaku lama, bit-identik bila flag baru tidak dipakai):
- `--goal-joints JSON` ({sendi: rad}), hanya dengan `--arms/--quad`; kunci HARUS tepat 6 sendi tiap lengan di `--arms`
  (lain = `ap.error`). Rencana lengan memakai tujuan sendi (subset prefiksnya) lewat `move_to` → `_plan_and_screen`
  (penyaring SAMA: tuck, torsi, antar-lengan ketiga lengan, lingkungan, `retract_check`). Target `--target` TETAP
  diterbitkan ke monitor sebelum gerak (penilaian L2 tidak berubah).
- **GOAL-MISMATCH** (baru, hanya bila tujuan sendi): FK ujung rencana (`tool_frame`, URDF hidup, rel + rotasi dari
  keadaan awal) vs target: posisi > **3.0 mm** atau sumbu approach > **2.0°** → DITOLAK sebelum gerak; masuk `RETRYABLE`
  (sampel OMPL lain bisa mendarat lebih dekat di kotak toleransi sendi 0.2°). FK tidak bisa dihitung → `GOAL-UNSCREENED`
  (TIDAK diulang, menolak). Alasan ambang: target salah-pasang (tujuan dari tugas lain) memberi galat cm; kotak toleransi
  0.2° × 6 sendi pada lengan ~1 m memberi beberapa mm; bar monitor 5 mm.
- `--start-joints JSON`: keadaan awal TERTEMPAT (seleksi, pola `dual_arm_targets`); **hanya tanpa `--move`** dan butuh
  `--plan-check` (lain = `ap.error`) — tidak pernah di jalur yang mengeksekusi.
- DRY `--plan-check`: baris JSON memuat `plans: {arm: {verdict, end: {sendi: rad}}}` (ujung rencana).

**`docs/results/p1_g32/run_g32.py`**: `--final-retract` → `G31S.events(..., final_retract)`; perintah tugas dibangun di
`task_cmd(e, mon, move=True, start=None)` (dipakai jalur `--move` DAN uji DRY); `e['goal']` ada → `--goal-joints=<json>`
DITAMBAHKAN DI AKHIR (tanpa tujuan: argv bit-identik dengan lama). `--dry` juga mencetak perintah tiap event.

**`docs/results/p1_g31/g31_screen.py`**: `walk(..., task_fn=None)`; `--task-cli` → tugas lewat
`run_g32.task_cmd(e, mon, move=False, start=<placed>)` sebagai subproses (CLI probe yang SAMA dengan HW, tanpa `--move`,
`--plan-check`, `--start-joints`), verdict + ujung dari JSON probe. Retract/traverse = walk G41b apa adanya.

### A2. 🔒 Uji (mock domain 77, tanpa `--move`)

- **K-R (offline):** `task_cmd` untuk semua tugas G32 R10 seed 36 dan G24b/G36b varian tanpa tujuan = argv lama
  (ekspresi inline lama disalin ke uji) bit-identik.
- **K-FK (offline):** FK ujung 90 lintasan tugas tersimpan G41b (`g41b_planonly_plans.jsonl`) vs xyz tugas: posisi + sumbu;
  dihitung yang akan GOAL-MISMATCH.
- **K-P (offline):** `--goal-joints` validasi: kunci kurang / lebih / lengan salah / `--start-joints` + `--move` → `ap.error`
  (rc 2) — dijalankan tanpa ROS hidup (parse sebelum `rclpy.init`).
- **DRY-CLI:** `g31_screen.py --plan g41b_candidates.json --variant G41b --final-retract --task-cli --repeats 3` pada
  seed 0 3 8 10 18. Lolos ⟺ 3/3 PLANNED. Verdict dibandingkan dengan plan-only G41b (5/5 3/3).
- **`run_g32 --dry`** (monitor + perekam + `ros2_control_node` hidup, mock di REST p0) satu seed G41b dengan
  `--final-retract`: daftar event = `events(final_retract=True)`, tiap tugas berperintah `--goal-joints`.

### A3. 🔒 (s42) — (iv″) sebagai kendala lazy penjadwal

- Konfigurasi stop c = (g, li, ri, urutan node lengan slot 0, urutan node slot 1) (urutan = indeks tugas naik, seperti
  `events`). (s42)(c) layak ⟺ simulasi stop dari kedua lengan REST di pose (li, ri) (selalu ada retract sebelum traverse;
  p0 = REST), urutan event `events` (j-ke-j, slot 0 lalu 1): tiap transit tugas = garis sendi lurus kosinus 61 titik dari
  keadaan lengan ke tujuan cabang (c41b) tuple-nya, pasangan di keadaan simulasi; lalu retract lurus kedua lengan
  (`_rest_line`) bila tidak REST. Tiap garis: `EnvChecker.screen_trajectory` CLEAR ∧ se-gantry CLEAR ∧ kotak ≥ 10 mm ∧ torsi
  (aturan `torq2`) — = `g41_make.chain` tanpa lintas-gantry, basis `G.base_joints(g, lin, rot)` (gantry lain di 0/0).
  Lintas-gantry tetap di (iv′)/(iv″) pasca-jadwal (dilaporkan).
- Stop dengan 1 tugas: layak tanpa dihitung (= (e2) (c41b) persis, pasangan REST).
- Lazy: konfigurasi belum dihitung = layak (relaksasi); sesudah tuple + tepi konvergen, semua stop jadwal dengan ≥ 2 tugas
  dihitung; yang tolak → kombinasi (U, v, pose) itu dilarang di salinan `_dur_table`/`stop_duration` (VERBATIM kecuali
  larangan); ulang sampai jadwal tidak memakai konfigurasi belum-terverifikasi. Cache `g42_stop_cache.jsonl`.
- **Kontrol positif:** (s42) pada konfigurasi G41 s19 g2 stop berisi t3 → t4 harus TOLAK.
- Jadwal ulang 20 seed (kriteria (i)–(iii) G38 A4) → `g42_candidates.json`; (iv′) + (iv″) untuk tiap seed lolos.

### A4. 🔒 Diagnosa 15 seed gagal (i) G41b

Per seed gagal, untuk SETIAP tugas yang tak layak di kedua gantry (bukan hanya yang pertama): himpunan tuple L1
(`sched._gantry_oracles`, R35, kedua gantry, kedua slot). Atribusi hierarkis: L1 → pose (a) G41b → torsi tepat → (c41b)
(alasan cabang `whys`). Marginal: torsi tepat juga dihitung untuk tuple yang gagal pose (cache yang ada dulu, sisanya
`O2.solve` + `torq2` dihitung); (c41b) dihitung untuk tuple pose-gagal ∧ torsi-ok bila jumlahnya ≤ 600 total (lebih →
sampel acak 600, seed 42, DITULIS). Juga: tugas lolos tuple tetapi tak terjangkau traverse (b). Jenis tugas (TENGAH /
DINDING / …) dilaporkan.

### A5. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D330 | DRY-CLI 5 seed G41b: **5/5 LOLOS 3/3** (= plan-only G41b). |
| D331 | DRY-CLI: tugas PLANNED pada percobaan rencana PERTAMA probe ≥ **95 %**. |
| D332 | K-FK: 0/90 ujung tersimpan melewati 3.0 mm / 2.0° (maks posisi ≤ 3.0 mm). |
| D333 | GOAL-MISMATCH di DRY-CLI = **0**. |
| D334 | G42 (s42): **20/20 seed identik G41b** (5 lolos: stop + makespan sama; 15 gagal (i)). |
| D335 | Kontrol positif s19 TOLAK; semua stop ≥ 2 tugas di 5 seed G41b DITERIMA. |
| D336 | ≥ **90 %** tugas tak layak = jenis TENGAH. |
| D337 | Pada ≥ **80 %** tugas tak layak tidak ada satu predikat yang sendirian mengosongkan: ada tuple gagal HANYA pose dan ada yang gagal HANYA torsi (konjungsi pose × torsi mengikat). |
| D338 | (c41b) (titik akhir / garis) menjadi pengikat terakhir (pose ∧ torsi lolos, semua cabang tolak) pada ≤ **2** tugas. |
| D339 | Tidak ada tugas yang gagal karena traverse (b) saja. |
