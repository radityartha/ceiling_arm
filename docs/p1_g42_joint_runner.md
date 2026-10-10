# P1 / G42 — runner HW siap tujuan sendi + (iv″) jadi kendala penjadwal + diagnosa seed tak terselesaikan (OFFLINE + mock)

> Sesi 2026-10-10 (sesudah G41). NOL gerak HW; mock `ROS_DOMAIN_ID=77`. Dasar = G41b (tujuan sendi cabang oracle, margin
> kotak 10 mm, `--final-retract`): (i)–(iii) 5/20, plan-only mock 5/5 3/3. **§A ditulis dan DIKUNCI SEBELUM pengukuran**
> (salinan `docs/results/p1_g42/g42_A_locked.md`, sha256 `04f936b3…8794c5`, dikunci 2026-10-10 13:16). §B menang atas §A
> dan DITULIS. MARGIN = TOLAK.

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

## B. Hasil terukur

### B0. K offline — [log K-R/K-P](results/p1_g42/g42_k_offline.log), [log K-FK](results/p1_g42/g42_kfk.log)

- **K-R:** 492 tugas tanpa tujuan dari 5 berkas jadwal (G31, G32, G38, G40, G40e): argv `task_cmd` = argv inline lama,
  **0 beda**. **K-P:** 7/7 masukan salah ditolak rc 2 sebelum `rclpy.init` (kunci kurang / lebih / lengan salah / tanpa
  `--arms` / bukan JSON / `--start-joints` + `--move` / tanpa `--plan-check`).
- **K-FK** (`goal_pose_error` asli, URDF hidup sha `f02e7c532cdb` = G32): ujung 90 rencana G41b vs xyz — posisi median
  **1.32 mm**, p90 2.85, maks **3.48 mm**; sumbu maks 0.54°. **8/90 akan GOAL-MISMATCH** (> 3.0 mm) → D332 ✗. Bukan target
  salah-pasang: FK tujuan oracle sendiri vs xyz ≤ **0.38 mm** / 0.05° (30/30; sisa = pembulatan `rail_m` 3 desimal) —
  sebarannya = sampel OMPL di kotak toleransi sendi ±0.2° (A1 sudah menyebutnya). Ambang terkunci 3.0 mm DIPERTAHANKAN
  (MARGIN = TOLAK: ujung 3.5 mm menyisakan 1.5 mm untuk galat lacak di bar monitor 5 mm); retry menanganinya.

### B1. DRY-CLI (mock) + `run_g32 --dry` — [log](results/p1_g42/g42_drycli.log), [JSON](results/p1_g42/g42_drycli.json), [run_g32 --dry](results/p1_g42/g42_rung32_dry.log)

Mock domain 77: launch **2801710** (`SIG_DFL…execvp` + `setsid nohup`, SigIgn `0x1001001` — SIGINT tidak diabaikan), 1
`move_group`, `env_static_map_pub` dari launch, monitor + perekam hidup; URDF hidup sha `f02e7c532cdb`.

- **`g31_screen --task-cli --final-retract` k = 3, seed 0 3 8 10 18: 5/5 LOLOS 3/3** = plan-only G41b (5/5) → D330 ✓.
  **90/90** tugas PLANNED lewat CLI probe HW (`run_g32.task_cmd(move=False)`: `--arms`, `--goal-joints`, `--plan-check`,
  `--start-joints`, tanpa `--move`), rc 0 semua; retract 102/102 CLEAR (lurus), traverse 72/72 CLEAR. Waktu per tugas
  median 20.3 s (maks 42.7; start proses probe + muat penyaring tiap tugas — waktu mock tidak dinilai).
- Percobaan rencana pertama **83/90 (92.2 %)** → D331 ✗ (≥ 95 %). 7 retry: **6 GOAL-MISMATCH** (ujung 3.03–4.03 mm, semua
  lolos di percobaan ke-2 dengan 0.74–2.20 mm) → D333 ✗ (= 0); 1 penolakan SEBELUM cek FK (s10 sampel 2 t2, 42.7 s) —
  alasan tidak terekam (`task_cli` hanya menangkap baris FK; kesalahan saya, celah pencatatan). Pembanding: plan-only G41b
  in-process 90/90 percobaan pertama — bedanya = GOAL-MISMATCH (baru di G42) + 1 sampel OMPL.
- Ujung DITERIMA vs target: median **1.15 mm**, maks **2.88 mm** (sumbu ≤ 0.5°) — di dalam bar perencanaan pose (2 mm)
  untuk median, < 3.0 mm semua.
- **`run_g32 --dry`** seed 0 G41b `--final-retract` (domain **78**, `fake_js` REST + rel 1.5/0, monitor + perekam sendiri;
  `ros2_control_node` = milik mock domain 77 — hanya cek PID hidup; nol gerak, mock tidak disentuh): **16 event** = `events
  (final_retract=True)` (retract akhir ev 7 g1, ev 15 g2), A3 awal 0.000° / rel 1.500 / 0.000, **6/6 perintah tugas berisi
  `--goal-joints`** + `--move --monitor-csv`, tidak dikirim. Keluar rc **134** ("terminate called without an active
  exception") = pola benign warisan `run_g22` (G32 B7 (5)); crash dump python sesi ini dihapus.

### B2. (s42) — [kstop](results/p1_g42/g42_kstop.log), [kstop kotak 0](results/p1_g42/g42_kstop_box0.log), [K-lazy](results/p1_g42/g42_klazy.log), [run](results/p1_g42/g42_run.log), [JSON](results/p1_g42/g42_candidates.json)

- **Kontrol positif (A3):** G41 s19 g2 stop (0.90, −15) t3 → t4 **TOLAK** — tetapi sudah di transit PERTAMA (REST → t3,
  kotak 0.40 mm < 10 mm): tujuan G41 dibuat tanpa margin. Kontrol lebih tajam (B, ditulis sebelum dijalankan): predikat
  G41 sendiri (`BOX_MIN` = 0) → REST → t3 ok (0.40 mm), **t3 → t4 TOLAK, kotak −6.51 mm** = angka `g41_make.chain` persis.
  Stop ≥ 2 tugas G41b: **4/4 diterima** (s0 g1/g2, s10 g1/g2). → D335 ✓.
- **Penyimpangan kerja (bukan hasil), celah saya — K-lazy:** larangan tidak pernah mengikat di G41b (0 stop tolak), jadi
  jalur `forbids` → `_dur_table_f` tak teruji. Uji tambahan (B): predikat G41 + (s42) pada s19 (cache salinan di scratch).
  Jalan pertama: solver menghindari konfigurasi terlarang dengan **memecahnya jadi dua stop berturut di pose SAMA**
  (0.90, −15) {t4} | {t3} — `events()` tidak me-retract bila pose tidak berubah, jadi transit t4 → t3 akan lolos tanpa
  diperiksa ([log v1](results/p1_g42/g42_run_v1_splitgap.log), [K-lazy v1](results/p1_g42/g42_klazy_v1_splitgap.log)).
  **Perbaikan:** `solve_gantry_f` melarang transisi diri (diagonal T = ∞; stop pertama tetap boleh di p0 lewat T0 = baris
  r0). Tanpa larangan (s42) stop gabungan tidak pernah lebih buruk dari pecahan (jangkauan sama, slot max ≤ jumlah), jadi
  tidak ada hasil lain berubah (dicek: run ulang identik). K-lazy ulang: (0.90, −15) dilarang → solver pindah ke
  **(0.95, −15)** {t3, t4} (dihitung (s42): ok), makespan 381.71 → **384.34 s (+0.7 %)**, 4 iterasi. Varian G41 (tanpa
  margin) hanya untuk menguji jalur, BUKAN jadwal sah.
- **Jadwal ulang G42 20 seed:** **20/20 identik G41b** — 5 lolos (0, 3, 8, 10, 18) dengan stop, penugasan, tujuan sendi dan
  makespan sama persis (Δ 0.0 s); 15 gagal (i) dengan tugas pertama tak layak yang sama. (s42) dihitung 4 stop, tolak 0.
  → D334 ✓. **Beda = nol**: (iv″) tidak pernah mengikat pada instans ini; nilainya = menjamin (bukan lagi hanya
  melaporkan) transit tugas berurutan.

### B3. Diagnosa — dua penyimpangan (B, DITULIS sebelum dijalankan)

1. **Sampel (c41b) tidak berstrata.** Kandidat marginal 4 451 > 600 → sampel acak 600 (A4), tetapi strata yang menentukan
   "gagal HANYA pose" (pose ✗ ∧ torsi ✓) hanya **188** tuple dan tersampel **24**. ⇒ strata itu dihitung PENUH (188);
   strata pose ✓ ∧ torsi ✗ tetap sampel (dilaporkan sebagai batas bawah).
2. **🔴 Galat solver warisan G40 (bukan predikat):** `solve_exact_e` (= `sched.solve_exact`) memangkas pilihan gantry tugas i
   dengan `isfinite(costs[g][1 << i])` = tugas i SENDIRIAN langsung dari p0. Sebelum G40 T selalu hingga, jadi itu setara
   "ada tuple". Sejak G40 (b) tepi traverse bisa ∞: tugas i bisa terjangkau lewat stop lain (p0 → A → pose) walau p0 → pose
   langsung ditolak. Diagnosa: 5 dari 18 tugas "tak layak" (s7 t2, s9 t1, s14 t1, s15 t0, s16 t2) MASIH punya 2–3 tuple
   lolos filter (belum terverifikasi = layak di relaksasi) — `ValueError` dilempar sebelum lazy loop memverifikasinya.
   **Perbaikan (G42f):** g layak untuk i ⟺ ∃ A ∋ i dengan `costs[g][A]` hingga; enumerasi alokasi + cek `isfinite(best)`
   tidak berubah. Eksak (enumerasi tetap lengkap atas alokasi layak). Bisa mengubah juga seed yang lolos (gantry yang dulu
   terpangkas untuk satu tugas) ⇒ **20 seed dijalankan ulang** → `g42f_candidates.json`; G42 (pra-perbaikan) tetap catatan
   D334. G40/G40e/G41/G41b memakai pemangkasan lama — **ditandai untuk dibersihkan**, hasilnya tidak diubah di sini.
3. **Seed baru dari G42f** (B, ditulis sebelum dijalankan; s7 sudah terlihat lolos (i)–(iii) di log saat menulis ini):
   (iv′) + (iv″) offline, lalu DRY-CLI mock k = 3 `--final-retract --task-cli` (jalur runner HW, tanpa `--move`) pada
   setiap seed baru yang lolos (i)–(iii). Lolos ⟺ 3/3. Dugaan tambahan: **D340** (iv″) memprediksi verdict DRY-CLI seed baru
   dengan benar di semua seed; **D341** seed baru LOLOS 3/3 ⟺ (iv′) ∧ (iv″) lolos.

### B4. G42f (solver diperbaiki) — [run](results/p1_g42/g42f_run.log), [JSON](results/p1_g42/g42f_candidates.json), [(iv′)](results/p1_g42/g42f_iv.log), [(iv″)](results/p1_g42/g42f_chain.log), [DRY-CLI s7](results/p1_g42/g42f_drycli.log)

- **(i)–(iii) 6/20** (0, 3, **7**, 8, 10, 18); kelima seed lama identik G41b (stop + tujuan + makespan). **s7 baru**:
  makespan **435.00 s**, g1 1 pindah / 2 tugas, g2 4 pindah / 4 tugas (rot −35…+5), 32 iterasi lazy (984 s); (s42) 1 stop
  dihitung, ok. Tuple dihitung 403 → 1 201, tepi 5 720 → 7 456. s14/s15 kini gagal di tugas lain (0 / 4) — verifikasi
  lazy berjalan lebih jauh sebelum kosong. Tidak ada seed yang gagal karena `ValueError('no feasible schedule')`.
- (iv′) **6/6**, (iv″) **6/6** (s7: 9 garis, min env 65.9 mm, antar 105.0 mm, kotak ≥ 42.3 mm).
- **DRY-CLI s7 k = 3: LOLOS 3/3** — 18/18 PLANNED percobaan PERTAMA, ujung ≤ 2.18 mm, retract 21/21 + traverse CLEAR. → D340 ✓
  (prediksi (iv″) benar), D341 ✓ (1/1).
- Catatan HW: s7 g2 berotasi sampai −35° dan ±25° antar-stop — di luar gerbang HW ±10° (G30); hanya plan-only.

### B5. Diagnosa (A4 + B3.1) pada G42f — [log](results/p1_g42/g42f_diag.log), [JSON](results/p1_g42/g42f_diag.json), [laporan](results/p1_g42/g42f_diag_report.log)

14 seed gagal, **19 tugas** tak layak di kedua gantry (pra-perbaikan G42: 15 seed, 18 tugas, [log](results/p1_g42/g42_diag.log)).
Tuple L1 7 667; torsi dihitung untuk 26 (+198 pra-perbaikan) yang belum diketahui; (c41b) marginal 708 (strata pose ✗ ∧
torsi ✓ PENUH: 125).

| | hasil |
|---|---|
| jenis | TENGAH **12**, DINDING **7** (63 % TENGAH) |
| hierarki L1 → pose → torsi → (c41b) | per tugas: pose gagal 73–169 tuple; **SEMUA sisa pose-ok gagal torsi** di 16/19 tugas; (c41b) jadi pengikat terakhir hanya di s15 t4 (2 tuple), s15 t5 (21), s19 t3 (14) |
| **gagal HANYA pose** | **0 di 19/19 tugas** — setiap tuple torsi-ok yang gagal pose JUGA gagal (c41b) (e1-env 75, e1-kotak 53, garis-env 42, G40 env2 15, garis-kotak 3) |
| **gagal HANYA torsi** (pose ✓ ∧ (c41b) ✓, batas bawah: strata ini disampel) | **17/19 tugas**, 256 tuple; sendi pengikat **joint_2 256/256**; lebih dari batas `torq2` (LIM − OFFSET − MARGIN = 5.64 N·m j2) min **+0.36 N·m** (s17 t1), p10 +1.43, median **+2.16**; ≤ +0.5: 1, ≤ +1.0: 5 |
| tuple torsi-ok | hanya **2–49 per tugas**, semuanya di rotasi besar (TENGAH: \|rot\| 25–35°; DINDING: −15…+35) — dan di sana pose (a) gagal: lengan REST **a2 99 / a1 52** dekat peta lingkungan (≥ 50 mm dilanggar), kotak 12 |
| traverse (b) saja | **0** sesudah perbaikan (pra-perbaikan: **5** tugas — galat B3.2) |

**Bacaan:** 17/19 tugas tak terselesaikan karena **torsi joint_2 di rotasi kecil–sedang**, dan rotasi besar yang
menurunkan torsi menaruh lengan REST (tugas atau pasangan) di dalam pita 50 mm peta (rak/tiang) — jadi pengikatnya
konjungsi torsi × geometri, tetapi BUKAN pose (a) sendirian (tuple torsi-ok juga gagal titik akhir/garis (c41b)). Dua tugas
DINDING (s15 t5, s19 t3) terikat geometri (c41b: e1-kotak / garis) walau torsi + pose lolos. Melonggarkan margin torsi
saja (j2 0.659 N·m) membuka **2 tuple** (s17 t1 +0.36, s5 t0 +0.64; s5 masih punya t5 tak layak) — dinding torsi, bukan
artefak margin; dan MARGIN = TOLAK melarang melonggarkannya.

### B6. Papan skor D330–D341

| # | Dugaan | Hasil |
|---|---|---|
| D330 | DRY-CLI 5/5 LOLOS 3/3 | ✓ **5/5** (90/90 PLANNED) |
| D331 | percobaan pertama ≥ 95 % | ✗ **92.2 %** (83/90; 6 GOAL-MISMATCH + 1 tak terekam) |
| D332 | K-FK 0/90 > 3.0 mm | ✗ **8/90** (maks 3.48 mm) |
| D333 | GOAL-MISMATCH DRY-CLI = 0 | ✗ **6** (semua lolos di percobaan ke-2) |
| D334 | G42 20/20 identik G41b | ✓ **20/20** (pra-perbaikan solver) |
| D335 | kontrol positif tolak, stop G41b diterima | ✓ (tolak; −6.51 mm dengan kotak 0; 4/4) |
| D336 | ≥ 90 % tugas tak layak TENGAH | ✗ **63 %** (12/19; G42 pra-perbaikan 13/18 = 72 %) |
| D337 | ≥ 80 % tugas: ada HANYA-pose DAN HANYA-torsi | ✗ **0/19** (HANYA-pose 0 di semua; HANYA-torsi 17/19) |
| D338 | (c41b) pengikat terakhir ≤ 2 tugas | ✗ **3** (s15 t4, s15 t5, s19 t3) |
| D339 | tidak ada tugas gagal karena traverse saja | ✗ **5** pra-perbaikan (galat solver), 0 sesudah |
| D340 | (iv″) memprediksi DRY-CLI seed baru | ✓ 1/1 (s7) |
| D341 | seed baru LOLOS ⟺ (iv′) ∧ (iv″) | ✓ 1/1 |

**Tally G42: 6 tepat / 6 meleset.** Pola: tepat = mekanisme kode yang dibaca (runner, larangan stop, identitas). Meleset =
(1) sebaran ujung OMPL di kotak toleransi sendi (dugaan "beberapa mm" benar, tetapi ekornya melewati 3 mm 7–9 %); (2)
diagnosa: prior G40 B3 ("pose (a) — pasangan REST ke rak") salah sebagai pengikat TUNGGAL — pengikat utama torsi joint_2;
(3) galat solver yang tidak saya cari sebelum mengunci.

## C. Keadaan akhir + langkah berikut (Rule 12)

- **Nol gerak HW.** Mock domain 77 naik 13:19, mati bersih 15:05 (launch SIGINT keluar ~14 s, `node list --no-daemon` 0,
  nol sisa; crash dump python (rc 134) + move_group (shutdown) milik sesi dihapus). Domain 78 (`fake_js` + monitor +
  perekam untuk `run_g32 --dry`) dimatikan via PID.
- **Kode repo diubah:** `scripts/reach_dwell_probe.py` (`--goal-joints`, `--start-joints`, GOAL-MISMATCH /
  GOAL-UNSCREENED, `goal_pose_error`, `move_to(start_joints, target)`, `_plan_end/_plan_tries`, DRY `plans`); `run_g32.py`
  (`task_cmd`, `--final-retract`, `--dry` mencetak perintah); `g31_screen.py` (`walk(task_fn)`, `--task-cli`). Default =
  perilaku lama (K-R 492/492 argv identik; plan-only in-process tidak memakai `target` → tanpa cek FK). Baru:
  `results/p1_g42/g42_make.py` ((s42), solver salinan + perbaikan pemangkasan, diagnosa), `g42_k_offline.py`,
  `g42_kfk.py`, `g42_klazy.py`, `g42_diag_report.py`.
- **Runner HW siap tujuan sendi** (CLI + DRY-CLI mock 6/6 seed 3/3), tetapi **HW TETAP TERTUTUP:** gerbang rotasi > 10°
  (keenam jadwal berotasi ≥ 30° di suatu stop), origin g2 −35 mm (cek kamera), penyaring Fix-Start-State (G37). Tambahan: probe
  `--goal-joints` belum pernah dieksekusi (`--move`) bahkan di mock.
- **Ditandai untuk dibersihkan (Rule 7):** pemangkasan `costs[g][1 << i]` di `g40_make.solve_exact_e` (dipakai G40, G40e,
  G41, G41b) salah sejak tepi (b) bisa ∞; hasil lama tidak diubah — G42f memakai perbaikan. G41b 15 gagal → G42f 14.
- **Celah terbuka:** (1) alasan penolakan sebelum FK tidak terekam di `task_cli`; (2) sebaran ujung 0.2° → 7–9 % retry
  GOAL-MISMATCH (opsi: toleransi tujuan sendi lebih ketat untuk tugas — mengubah permintaan rencana, perlu uji ulang);
  (3) 14/20 seed tetap tak terselesaikan secara STRUKTURAL (torsi joint_2 × geometri rak), bukan oleh predikat runner.
- Rule 6: anggaran token sesi (30 k) terlampaui jauh (membaca 6 berkas wajib saja > 30 k) — DITULIS.
