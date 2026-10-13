# P1 / G61 — OFFLINE: tutup celah G57 (runner sehat + HALTED, koreksi target thd gantry terukur, torsi transit, s0 HW/sim)

> Sesi 2026-10-13, prompt `docs/p1_g61_offline_prompt.md`. **NOL gerak HW** (stack HW mati, sel home sejak G57 17:30). Mock
> `ROS_DOMAIN_ID=77` = fake HW, BUKAN data HW, selalu `octomap_updaters:=false`. **§A DIKUNCI sebelum kode diubah** (salinan
> `docs/results/p1_g61/g61_A_locked.md` + sha256). §B menang atas §A. MARGIN = TOLAK; penyaring tidak dilonggarkan. Arsip
> `docs/results/p1_g61/`.

## A. Protokol — DIKUNCI

### A0. Pengungkapan (sebelum mengunci, NOL gerak)

- Domain 77 kosong (`ros2 node list --no-daemon` 0 baris, 09:5x). Sesi Claude lain hidup sejak 07:55 (kemungkinan CAM-6DOF): berkas
  keadaan dibaca sebelum ditulis, commit hanya berkas G61.
- **Sudah dihitung SEBELUM §A (C2 "ukur dulu")** — [g61_c2_measure.py](results/p1_g61/g61_c2_measure.py) → `g61_c2_measure.json`
  (data HW G57, FK pinocchio URDF live sha `f02e7c53`): 58 traverse (16 run termasuk injeksi + tidak sah): galat rot −0.48…+0.10°,
  |galat| median 0.01°, lin |galat| ≤ 0.94 mm. Hanya 5 traverse |galat rot| ≥ 0.15°, semuanya Δrot besar (+50, +50, +35, +54.9,
  +65°) dan PENDEK (negatif thd arah gerak). 94 tugas ber-goal: galat TCP FK(goal, gantry terukur) vs target ≥ 2.5 mm **4/94**, > 3 mm
  **3/94**, keempatnya s12 lama stop g1 (0.65, +40°) (coba 1 ev 5 3.48 / ev 6 3.10, coba 2 ev 5 3.04 / ev 6 2.72); FK di gantry
  NOMINAL ≤ 0.38 mm (sisa IK oracle). Prediksi FK cocok dengan "ujung rencana" probe (3.48 vs 3.18–3.86; 3.04 vs 3.80/3.35/2.69) →
  galat GOAL-MISMATCH = galat gantry, bukan perencana.
- Sudah terbaca dari kode sebelum §A (bukan hasil): `predict_peak_torque` probe menghitung RNEA (q, v, a) di **SETIAP titik
  lintasan rencana**, bukan statis di ujung. Pernyataan G57 B3 "prediksi probe (statis di ujung, RNEA+offset)" → kemungkinan salah
  baca; diuji di C3 (§B menulis koreksinya bila terbukti). Jalur HW eksekusi = rencana yang sama (FJT langsung, G54 B1), jadi
  lintasan yang disaring = lintasan yang dijalankan.
- Probe TIDAK menyimpan lintasan (hanya puncak RNEA per sendi per percobaan). Lintasan nyata = perekam joint_states (pos, eff).
  Perekam kosong 13:08:49–13:46 (s10 lama tidak sah). Mock tidak mengukur torsi nyata.
- Mock: traverse berakhir TEPAT (err 0.000 mm / 0.0000°, G58) → koreksi C2 tidak pernah terpicu di mock tanpa injeksi.
  `pose_to_g` S13″ menolak target yang bukan stop jadwal → injeksi rotasi butuh salinan jadwal untuk `pose_to_g` saja.
- Runner sekarang: `TIDAK VALID (mesin):` di probe tidak cocok dengan keluaran probe nyata (`TIDAK VALID (mesin, A5, di LUAR
  penyebut):`) → cek itu tidak pernah memicu (sengaja/tidak — tidak diubah kecuali C1(c) membutuhkannya; dicatat). `UNSCREENED`
  di teks probe → AUTO-STOP (sudah ada). move_group mati → `move_to` mengembalikan `EXEC-FAIL` (tunggu server 15 s) → HALTED.

### A1. 🔒 C1 — runner sehat (`run_g54.py`)

(a) **Pra-cek sebelum t0** (juga di `--dry`, sesudah K0): proses move_group hidup (cmdline `moveit_ros_move_group/move_group`,
    semua PID); layanan `/get_planning_scene` menjawab ≤ 5 s; objek dunia `env_static_map` ada di scene. Gagal → `AUTO-STOP:
    pra-cek …`, rc 3, tidak ada alat diluncurkan.
(b) **Penjaga kontinu** (watchdog 10 Hz yang sudah ada, `s26`): PID move_group masuk daftar proses yang wajib hidup (bersama
    monitor/perekam/ros2_control_node) → mati → AUTO-STOP KEDUA gantry (jalur `stop_all` lama).
(c) **Kebijakan HALTED** — dibedakan dari `move_status` lengan di JSON probe:
    - **sistem rusak** → AUTO-STOP KEDUA gantry: `EXEC-FAIL` (server move_group tak ada, FJT ditolak/timeout/error, awal rencana
      > 0.01 rad dari terukur) atau JSON probe hilang / move_status kosong pada rc 0.
    - **lengan tidak bisa** → dicatat, lanjut (= G54 A3): penolakan penyaring yang sehat — `NO-PLAN`, `GOAL-MISMATCH`,
      `TORQUE-UNSAFE`, `INTERARM-COLLIDE`, `ENV-COLLIDE`, `RETRACT-BLOCKED`, `START-JUMP`, `TUCK-REFUSED`. (Penyaring yang tidak bisa
      menyaring = `*-UNSCREENED` → sudah AUTO-STOP.)
    Alasan: HALTED karena penolakan = lengan tidak bergerak, sistem bukti sehat (G57 s12 coba 1: ev 6 sesudahnya sukses); stop di
    situ membuang run tanpa mengurangi risiko. EXEC-FAIL = mesin tidak bisa mengeksekusi apa pun (G57 B4 6/6) atau eksekusi
    menyimpang — melanjutkan = rel bergerak tanpa tugas atau lebih buruk.
(d) **Perekam:** path = argv[1] proses `js_record.py`. Pra-cek: berkas tumbuh dalam 3 s, else AUTO-STOP pra-cek. Selama run:
    tidak tumbuh > 5 s → PERINGATAN sekali per macet di runner.log + `rec['recorder_stall']` (t mulai, t pulih), TIDAK AUTO-STOP.
    Alasan: perekam = bukti, bukan keselamatan (penjaga torsi runner membaca /joint_states sendiri); AUTO-STOP di tengah tugas
    meninggalkan lengan di pose tugas (pemulihan = gerak tambahan yang ditanya) — risiko HW lebih besar dari yang dihilangkan.
    Run dengan macet ditandai "data perekam TIDAK LENGKAP" di baris SELESAI.
- **Default:** (a)–(d) AKTIF tanpa opsi. Tidak bit-identik dengan G54 dalam keadaan rusak (itu tujuannya); dalam keadaan sehat argv
  alat, gerbang, dan urutan event identik (pra-cek ~1–3 s SEBELUM t0, makespan tidak terpengaruh). Alasan tidak opt-in: penjaga
  keselamatan yang harus diingat untuk diaktifkan = kelas kegagalan G57 (`octomap_updaters`).

### A2. 🔒 C2 — koreksi goal-joints thd gantry terukur (opsi (i))

- `run_g54 --goal-correct` (**bawaan MATI** → argv lama bit demi bit). Untuk tiap tugas ber-`goal`, tepat sebelum probe
  diluncurkan (tiap percobaan, termasuk ulang RESERVE-BLOCKED): gantry sendiri terukur m = (lin, rot) dari /joint_states; nominal
  n = (rails[g], rots[g]) (= `to` traverse terakhir / p0, rot 0 — keadaan tempat goal dihitung offline).
  disp = ‖FK(goal, m) − FK(goal, n)‖ (TCP). **disp < 1.0 mm → argv tidak berubah.** disp ≥ 1.0 mm → IK DLS 6-D (gantry tetap di
  m, benih = goal lama) ke pose alat SE3 = FK(goal, n) (posisi + orientasi penuh yang disaring offline). Terima ⟺ galat ≤ 0.05 mm /
  0.01°, dalam batas sendi URDF, max |Δq| ≤ 2°. Diterima → probe diluncurkan dengan goal baru; gagal → goal lama + baris `KOREKSI
  GAGAL` (probe tetap menyaring; kemungkinan GOAL-MISMATCH = aman).
- **Tidak ada penyaring yang dilewati:** probe tetap menjalankan semua penyaring pada rencana nyata ke goal baru (START, torsi
  RNEA, antar-lengan, lingkungan, jalur pulang, reservasi) dan GOAL-MISMATCH vs target 3.0 mm / 2° (gantry dari /joint_states) —
  batas TIDAK diubah. Dicatat per tugas: `er['goal_correct']` = disp, galat IK, max |Δq|, diterapkan/tidak.
- Opsi (ii) traverse koreksi DITOLAK: deadband bridge 0.3° + driver 0.5° (G51) ≥ galat yang mau dikoreksi (0.48°). (iii) tidak ada.
- **Injeksi uji** `run_g54 --inject-rot-error G:K:DEG` (kontrol negatif, mock): traverse gantry G ke stop K diperintahkan ke
  rot + DEG (salinan jadwal hanya untuk S13″ `pose_to_g`); nominal / goal / ekspektasi penjaga (±1°) tetap jadwal asli.
- Satu FK/IK di proses runner (pinocchio, URDF `/tmp/reach_dwell_live.urdf` atau `_urdf_path` probe dari robot_state_publisher);
  URDF tidak tersedia dengan `--goal-correct` → AUTO-STOP pra-cek.

### A3. 🔒 C3 — torsi transit (analisis, NOL perubahan penyaring)

Per tugas G57 yang terekam perekam (run sah + injeksi + s12 coba 1; s10 lama tidak sah tidak terekam): jendela [t_start, t_end]
event, lengan tugas. Ukur: puncak |eff| per sendi + q dan waktu puncak, fase (transit = sebelum lengan diam pertama kali dalam
0.5° dari q akhir; tahan = sesudahnya), torsi tahan median di target. Bandingkan:
1. **probe**: baris `torsi RNEA+offset` percobaan rencana TERAKHIR sebelum `MOVED` (RNEA lintasan + offset j2 7.7);
2. **RNEA rencana** tanpa offset (= ukur − RNEA = offset efektif);
3. **oracle⁗-gaya**: maks gravitasi statis garis lurus ruang-sendi dari keadaan AWAL terukur (REST atau pose tugas sebelumnya) →
   q akhir terukur, + offset + m^p (G27 T3a);
4. **gravitasi sepanjang lintasan TERUKUR** + offset (apakah model statis menjelaskan puncak).
Keluaran: tabel semua tugas + ringkasan; rekomendasi penyaring tertulis (tanpa mengubah `--tau-max 14`, batas keras > 14, offset,
atau penyaring di sesi ini).

### A4. 🔒 C4 — s0 HW/sim 1.287 + traverse banyak (analisis)

- Dekomposisi tiap tugas s0 baru HW dari log probe (CMD → ujung rencana [mulai + rencana], → reservasi CLEAR [penyaring], → MOVED
  [eksekusi], → t_success [dwell]) + perekam (durasi gerak nyata), bandingkan mock G55 s0 (`p1_g54/smoke/g55_s0_*`) dan biaya P1″
  (tugas 29.04 s, G53).
- Traverse semua run G57: dinding HW (t_start → t_end event) vs biaya g54_sim (layar 12.61 + 0.923 × model G25) per traverse;
  selisih s8/s18 lama dipilah: traverse vs tugas vs tunggu gerbang.

### A5. 🔒 Uji mock (domain 77, `octomap_updaters:=false`, monitor + perekam, REST + g1 PARK 1.50, K0 LULUS sebelum tiap run)

1. **Kontrol positif:** smoke paralel s7 baru (`p1_g55/g55_selected.json R90sel`) + s12 lama (`p1_g52/g52_candidates.json R90`)
   dengan kode baru + `--goal-correct`; pulang retrace + K0. Lulus: 6/6, 0 AUTO-STOP, 0 koreksi diterapkan (mock disp 0), 0
   peringatan perekam, argv tugas = argv G58 smoke seed/varian sama (normalisasi path /tmp dan `--tau-max`).
2. **C2 injeksi:** s12 lama `--inject-rot-error 1:1:-0.48` (a) TANPA `--goal-correct` → ev 5 GOAL-MISMATCH (HALTED, runner lanjut =
   kebijakan C1(c)); (b) DENGAN `--goal-correct` → ev 5 koreksi diterapkan, SUCCESS. Pulang + K0 tiap run. Offline: FK goal lama vs
   terkoreksi di g1 (0.65, 39.52°) + semua 94 tugas G57 (berapa terpicu, |Δq|).
3. **C1 pra-cek:** move_group dimatikan (sesudah 4) → `run_g54 --dry` → AUTO-STOP pra-cek rc 3.
4. **C1 kontrol negatif:** smoke s7 baru, SIGKILL PID move_group saat ≥ 1 alat berjalan (tugas) → AUTO-STOP ≤ 2 s sesudah kill
   (cap waktu runner.log vs waktu kill), "TIDAK diluncurkan" / tidak ada CMD baru sesudah stop, rc 3. Lalu stack dimatikan,
   dinyalakan ulang, pulang + K0.
5. Perekam (d): uji mock SIGSTOP perekam 8 s saat smoke (bukan bagian gerbang; opsional bila waktu) → peringatan, tanpa stop.

### A6. 🔒 Gerbang G61

C1: A5.3 + A5.4 lulus, A5.1 tanpa pemicu palsu. C2: A5.2 (a) ditolak + (b) lolos, regresi A5.1 bit-identik. C3: tabel + rekomendasi
tertulis. C4: dekomposisi + penjelasan s0. Smoke 6/6 s7 baru + s12 lama (A5.1). Lalu runbook paralel + prompt hari HW berikut
diperbarui (bring-up `octomap_updaters:=false`, `--tau-max 14`, `--goal-correct`, pra-cek baru).

### A7. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D550 | A5.4: AUTO-STOP ≤ 0.5 s sesudah SIGKILL move_group (penjaga 10 Hz), 0 alat diluncurkan sesudahnya, rc 3. |
| D551 | A5.3: pra-cek menolak `--dry` tanpa move_group, rc 3. |
| D552 | A5.1: kedua smoke 6/6, 0 AUTO-STOP, 0 koreksi, 0 peringatan perekam, argv tugas identik G58 (setelah normalisasi). |
| D553 | A5.2(a): ev 5 GOAL-MISMATCH 3/3 percobaan (ujung 3.0–4.0 mm), runner lanjut, ev 6 tetap SUCCESS (galat ~3.1 mm → mungkin juga GOAL-MISMATCH; dugaan: SUCCESS). |
| D554 | A5.2(b): ev 5 koreksi diterapkan (disp 3.0–4.0 mm), max \|Δq\| ≤ 1°, PLANNED percobaan pertama, ujung ≤ 1.0 mm, SUCCESS. |
| D555 | Offline G57: koreksi terpicu (disp ≥ 1 mm) di ≤ 10/94 tugas, IK diterima di semua yang terpicu, max \|Δq\| ≤ 1°. |
| D556 | C3: puncak j2 terukur ≤ prediksi probe (RNEA lintasan + 7.7) di ≥ 90 % tugas terekam; maks kelebihan ≤ 0.5 N·m. |
| D557 | C3: offset efektif j2 (ukur − RNEA rencana) maks ≤ 8.2 N·m di semua tugas G57. |
| D558 | C3: di tugas dengan puncak j2 > 11 N·m, puncak terjadi di fase TRANSIT ≥ 80 %. |
| D559 | C3: proksi garis lurus dari awal terukur (+ offset + m^p) ≥ puncak j2 terukur di ≥ 80 % tugas. |
| D560 | C4: tiga tugas arm_3 s0: eksekusi (CLEAR → MOVED) = ≥ 60 % selisih durasi tugas vs rata-rata tugas lain run itu. |
| D561 | C4: durasi eksekusi rencana HW / mock (tugas s0 yang sama) median ∈ [0.8, 1.25] — HW tidak lebih lambat secara sistematis; selisih = sampel perencana. |
| D562 | C4: traverse HW − biaya g54_sim > 0 rata-rata dan menjelaskan ≥ 50 % selisih makespan HW − sim di s8 lama dan s18 lama. |

## B. Hasil terukur

(belum)
