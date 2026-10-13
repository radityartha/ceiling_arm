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

§A sha256 `47489ae2…` ([salinan](results/p1_g61/g61_A_locked.md)), commit c2491ff. Kode commit 831db96.

### B0. Perubahan kode — [run_g54.py](results/p1_g54/run_g54.py)

- **C1(a)** `health()` sesudah K0 + amplop, juga di `--dry`: `/get_planning_scene` (komponen WORLD_OBJECT_NAMES) ≤ 5 s,
  `env_static_map` di objek dunia, berkas perekam (argv[1] proses `js_record.py`, relatif thd cwd-nya) tumbuh dalam 3 s; dengan
  `--goal-correct` URDF `/tmp/reach_dwell_live.urdf` harus ada (dibuat lewat `_urdf_path` probe di subproses bila hilang).
- **C1(b)** `s26`: PID move_group (`moveit_ros_move_group/move_group`, di-resolve sebelum t0) wajib hidup. **Tambahan (tidak di
  §A):** pemeriksaan hidup kini `alive()` = `/proc/<pid>/stat` bukan zombie (sebelumnya `os.path.isdir`): proses yang di-SIGKILL
  tetap punya `/proc/<pid>` sampai induknya me-reap → penjaga lama bisa tidak pernah memicu. Berlaku juga untuk monitor/perekam/
  ros2_control_node (perilaku sehat tidak berubah).
- **C1(c)** HALTED: `move_status` semua ∈ `ARM_CANNOT` → "lengan tidak bisa", lanjut; selain itu (EXEC-FAIL, kosong) atau JSON probe
  hilang pada rc 0 → `stop_all('… sistem rusak …')`.
- **C1(d)** `recorder_check()` di watchdog: > 5 s tidak tumbuh → `PERINGATAN perekam` + `rec['recorder_stall']`, baris SELESAI
  diberi "data perekam TIDAK LENGKAP"; tanpa stop.
- **C2** `GoalFix` (pinocchio, IK DLS `log6` frame LOKAL, 200 iterasi, λ 1e-8) + `--goal-correct` + `er['goal_correct']`;
  `--inject-rot-error G:K:DEG` (salinan jadwal `inject_rot_plan.json` hanya untuk `pose_to_g`).
- Bawaan tanpa opsi baru: argv alat identik (dibuktikan A5.1, B5). Yang berubah tanpa opsi = C1 (a)–(d) sesuai §A1.

### B1. C2 luring — [g61_c2_offline.py](results/p1_g61/g61_c2_offline.py) → [JSON](results/p1_g61/g61_c2_offline.json)

`GoalFix` pada 94 tugas ber-goal G57 (nominal vs gantry terukur dari `g61_c2_measure.json`):

| | hasil |
|---|---|
| terpicu (geser alat ≥ 1 mm) | **14/94** (s12 lama ×4, s7 lama, s3 lama, s8 ×3, s3 G60 ×2, s18 lama, s10 lama tidak sah ×2) |
| IK diterima | 14/14 (galat ≤ 0.0001 mm / 0.00000°, dalam batas sendi) |
| \|Δq\| maks | **1.757°** (s12 coba 1 ev 5, rot −0.48°); s12 coba 2 ev 5 1.529° |
| galat FK vs target di gantry terukur | lama > 3 mm **3**, baru > 3 mm **0**; terpicu: baru ≤ 0.22 mm (sisa = sisa IK oracle di nominal) |
| tidak terpicu | goal identik (80/80, galat sama persis) |

s12 lama ev 5 (g1 0.65, +39.52°): goal lama 3.48 mm → koreksi 0.01 mm. **Batas |Δq| ≤ 2° (§A2)** ≈ 3.7°/° rotasi → koreksi hanya
diterima untuk |galat rot| ≲ 0.55°; galat terburuk yang pernah tercatat (G51 0.81°) akan memberi ~3° → `KOREKSI GAGAL` → probe
GOAL-MISMATCH (aman, tidak lolos). Dicatat sebagai batas yang diketahui; tidak diubah sesudah kunci.
→ **D555 ✗** (terpicu 14 > 10, \|Δq\| 1.76 > 1°; IK 14/14 ✓).

### B2. C3 torsi — [g61_c3_extract.py](results/p1_g61/g61_c3_extract.py) + [g61_c3_torque.py](results/p1_g61/g61_c3_torque.py) → [JSON](results/p1_g61/g61_c3_torque.json) (data HW nyata)

88/94 jendela tugas terekam (6 = s10 lama tidak sah, perekam macet), 87 bergerak dengan baris prediksi probe (s12 coba 1 ev 5 HALTED
nol gerak). Puncak j2 tertinggi (semua 13 tugas > 11 N·m; "st" = awal REST atau pose tugas sebelumnya):

| run | ev | lengan | st | puncak j2 | fase | probe (RNEA+7.7) | RNEA rencana | ukur − RNEA | garis lurus (+off+m^p) | tahan |
|---|---|---|---|---|---|---|---|---|---|---|
| s3 G60 | 10 | arm_4 | tugas | **13.22** | transit (0.1 s sebelum tiba) | 12.91 | 5.21 | **+8.01** | 13.02 | 6.23 |
| s8 baru | 10 | arm_4 | REST | 12.57 | transit (0.1 s sebelum tiba) | 12.49 | 4.79 | +7.78 | 12.78 | 6.48 |
| s12 G60 | 2 | arm_1 | REST | 12.51 | transit | 12.98 | 5.28 | +7.23 | 13.06 | 6.09 |
| s10 baru | 12 | arm_3 | tugas | 12.44 | transit (q2 32.6°, tengah) | 13.20 | 5.50 | +6.94 | 13.26 | 8.81 |
| s18 G60 | 10 | arm_4 | REST | 12.41 | transit | 12.48 | 4.78 | +7.63 | 12.78 | 5.84 |
| s12 lama | 10 | arm_3 | REST | 12.14 | transit | 13.01 | 5.31 | +6.83 | 13.08 | 8.37 |
| s12 lama c1 | 10 | arm_3 | REST | 12.12 | transit | 13.00 | 5.30 | +6.82 | 13.08 | 5.70 |
| s12 G60 | 14 | arm_3 | REST | 11.80 | transit | 12.79 | 5.09 | +6.71 | 12.88 | 3.33 |
| s0 baru | 9 | arm_3 | tugas | 11.73 | transit | 12.83 | 5.13 | +6.60 | 12.96 | 8.19 |
| s7 baru | 14 | arm_3 | REST | 11.66 | transit | 12.63 | 4.93 | +6.73 | 12.99 | 4.95 |
| s10 baru | 11 | arm_3 | REST | 11.59 | transit | 12.61 | 4.91 | +6.68 | 12.70 | 7.80 |
| s3 G60 | 13 | arm_3 | REST | 11.45 | transit | 12.53 | 4.83 | +6.62 | 12.63 | 5.77 |
| s8 lama | 18 | arm_3 | REST | 11.15 | transit | 12.53 | 4.83 | +6.32 | 12.94 | 5.85 |

Ringkasan (87 tugas):
- **Koreksi G57 B3 (fail loud):** prediksi probe BUKAN "statis di ujung" — `predict_peak_torque` = RNEA (q, v, a) di setiap titik
  rencana, dan HW mengeksekusi rencana itu sendiri (FJT). Gravitasi model di q terukur saat puncak ≈ RNEA rencana (s10 ev 12: 5.45
  vs 5.50). s10 baru ev 12 terukur 12.44 **di bawah** prediksi 13.20: penyaring benar; yang memotong adalah batas pengawas
  `--tau-max 12` (lebih rendah dari penyaring rating 14) — kini 14 (keputusan operator).
- j2 terukur ≤ prediksi probe **85/87**; kelebihan: s3 G60 ev 10 **+0.31**, s8 baru ev 10 **+0.08** (keduanya arm_4, puncak 0.1 s
  sebelum tiba = deselerasi) → **D556 ✓** (97.7 %, maks 0.31 ≤ 0.5). Sendi lain: 0 kelebihan (j1 margin ≥ 4.73, j3 ≥ 2.90 N·m;
  pergelangan ≤ 1.28).
- Offset efektif j2 (ukur − RNEA rencana): median **5.00**, p95 7.03, maks **8.01** (> 7.7 di 2/87) → **D557 ✓** (≤ 8.2).
  Ukur − gravitasi-di-puncak median 5.09, maks 8.05 → selisih = gesekan/dinamika tak termodel, bukan konfigurasi.
- Fase: puncak di TRANSIT 83/87, tahan 4/87; semua 13 tugas > 11 N·m di transit → **D558 ✓**. Torsi tahan j2 median 4.05 (maks
  10.26) — di bawah gravitasi model; puncak = gerak.
- Garis lurus dari awal TERUKUR + offset + m^p ≥ terukur **86/87** (gagal: s3 G60 ev 10, awal pose tugas sebelumnya) → **D559 ✓**.
  Garis lurus ≈ prediksi probe (rencana OMPL tujuan sendi ≈ garis lurus, G41).
- **Transit tugas → tugas (pertanyaan prompt):** 17/87 tugas berangkat dari pose tugas sebelumnya. oracle⁗ (G27 A3 PATH) dan
  syarat jadwal (e3) G41 hanya memeriksa garis REST ↔ goal → transit tugas → tugas **tidak dimodelkan di penjadwal**. Di runner ia
  tetap disaring probe (RNEA rencana nyata dari pose terukur) → keselamatan tidak bergantung padanya; dampaknya kelayakan (HALTED
  saat run). G57: 0 TORQUE-UNSAFE di 88 tugas. Satu-satunya kelebihan > 0.1 N·m justru tugas non-REST (s3 G60 ev 10).

**Rekomendasi (tidak diterapkan; keputusan operator/sesi berikut):**
1. Offset j2 7.7 → **8.1** (maks terukur 8.01 + 0.1; aturan G22 = maks + lonjakan): menutup 2/87 kelebihan, **0/87** tugas G57
   akan ditolak (prediksi maks 13.26 + 0.4 < 14). Pengetatan, bukan pelonggaran; butuh regenerasi tabel oracle/jadwal (offset
   juga dipakai O2/G41 (e3)) → tidak di sesi ini. Dengan 8.56 (maks + 0.55 gaya G22) 1/87 ditolak (s10 ev 12).
2. Penjadwal: tambah syarat (e3) untuk transit tugas → tugas (garis lurus goal_k → goal_k+1 lengan yang sama, + offset + m^p) agar
   jadwal tidak memuat tugas yang probe akan tolak.
3. `--tau-max 14` + batas keras > 14 tetap (keputusan operator). Pengawas ≥ penyaring menghindari TORQUE-ABORT sesudah gerak yang
   penyaringnya sudah menerima (kelas s10 ev 12).

### B3. C4 waktu — [g61_c4.py](results/p1_g61/g61_c4.py) → [JSON](results/p1_g61/g61_c4.json) (log HW G57 + mock G55/G58/G60)

**s0 baru (HW 190.70 vs sim 148.16):** makespan = rantai g2: retract 2.6 + tunggu traverse g1 29.1 + traverse g2 20.2 + tiga tugas
arm_3 (34.7 + 48.1 + 55.4) ≈ 190. Sim memberi tiap tugas 29.04 → kelebihan arm_3 +51.2, traverse −9.0 → +42.2 ≈ gap +42.54.

| ev | lengan | HW: rencana \| penyaring lain \| **reservasi (N keadaan)** \| eksekusi \| dwell = total | mock G55: reservasi (N) \| eksekusi = total |
|---|---|---|---|
| 2 | arm_2 | 2.4 \| 14.3 \| 1.1 (1) \| 5.0 \| 1.9 = 25.2 | 1.1 (1) \| 5.0 = 24.5 |
| 8 | arm_3 | 4.9 \| 14.0 \| 6.3 (50) \| 7.1 \| 1.9 = 34.7 | 5.7 (49) \| 7.1 = 34.0 |
| 3 | arm_2 | 3.2 \| 15.0 \| 1.2 (1) \| 7.6 \| 1.9 = 29.3 | 1.1 (1) \| 7.6 = 28.7 |
| 9 | arm_3 | 2.6 \| 16.5 \| **14.5 (76)** \| 11.9 \| 1.9 = 48.1 | 1.2 (1) \| 11.9 = 35.7 |
| 4 | arm_2 | 3.2 \| 15.4 \| 1.2 (1) \| 12.4 \| 1.9 = 34.7 | 1.2 (1) \| 12.4 = 33.1 |
| 10 | arm_3 | 3.0 \| 15.3 \| **22.7 (124)** \| 11.7 \| 1.9 = 55.4 | 11.1 (61) \| 11.8 = 42.4 |

(kolom "rencana" = mulai + perencanaan percobaan terakhir; tiap tugas 1 percobaan.)
- **Eksekusi HW = mock** (77 tugas semua run: rasio median **1.0001**, rentang 0.996–1.005) → **D561 ✓** (lintasan sama, FJT,
  fake HW mengikuti waktu rencana). Sumber selisih s0 BUKAN eksekusi: ev 9 / ev 10 memakai **penyaring reservasi 14.5 / 22.7 s**
  karena reservasi g1 saat itu = rencana PENUH tugas arm_2 yang sedang berjalan (76 / 124 titik; reservasi baru menyusut ke keadaan
  terukur di AKHIR event, sesudah dwell + keluar probe — ev 10 disaring 0.5 s sesudah arm_2 `MOVED`, lengan itu sudah diam di
  target). Mock G55 kebetulan bertemu reservasi 1 keadaan / garis retract 61.
- Penyaring reservasi semua tugas: **0.76 + 0.136 s × N keadaan** (HW, n 77, r 0.91; mock 1.02 + 0.114 N, r 0.87). Penyaring lain
  (antar-lengan, lingkungan, retract) median 14.3 s tetap.
- Kelebihan arm_3 s0 vs rata-rata tugas arm_2 (29.7 s): ev 8 +5.0, ev 9 +18.4, ev 10 +25.7; bagian eksekusi (vs rata-rata eksekusi
  arm_2 8.3 s) −1.2 / +3.6 / +3.4 → ≤ 20 %; reservasi +5.1 / +13.3 / +21.5 → **D560 ✗** (eksekusi bukan penyebab; ralat G57 B4
  "durasi eksekusi lintasan panjang").
- **Traverse:** 49 traverse HW vs sim (layar 12.61 + 0.923 × G25): kelebihan rata-rata **+0.02 s** (layar rata-rata 12.35, median
  10.03; gerak/T_cmd median 1.002; gerak/model 0.906; tutup 1.47 s). Per run Σ(traverse HW − model) vs gap makespan HW − sim:
  s8 lama **+10.1 / +60.5**, s18 lama **+12.6 / +72.1** → traverse menjelaskan 17 % / 17 % → **D562 ✗**. Σ(tugas HW − 29.04·n):
  s8 lama +34.3, s18 lama +18.9; sisa = penguatan gerbang paralel (traverse eksklusif menunggu tugas gantry lain yang lebih lama
  dari model).
- Durasi tugas HW per run (rata-rata) **30.7–37.9 s** vs P1″ 29.04 (dikalibrasi runner SERIAL tanpa reservasi); mock 26.4–33.9.

**Usul perbaikan model/alat (tidak diterapkan):**
1. g54_sim: biaya tugas = 29.04 + 0.76 + 0.136 × N_res(gantry lain saat tugas mulai) — N_res = jumlah titik rencana tugas gantry
   lain yang sedang berjalan (≈ 10 × durasi eksekusinya + 1, TOTG 0.1 s) atau 61 untuk retract, 1 bila diam.
2. Runner/probe: menyusutkan reservasi tugas ke keadaan akhir rencana segera sesudah `MOVED` (lengan diam di target selama dwell),
   bukan di akhir event → s0 ev 9/10 −13…−21 s. Perubahan semantik reservasi (keadaan terukur tetap), butuh §A + uji sendiri.
3. Penyaring reservasi 0.136 s/keadaan: B&B / cache geometri seperti G35/G36 (penyaring lain sudah).
