# P1 / DEMO-OFF — suara → YOLOE → approach: integrasi OFFLINE + mock (nol gerak HW)

> Sesi DEMO-OFF, 2026-10-08. Lanjutan [p1_demo_voice_yoloe.md](p1_demo_voice_yoloe.md) (temuan T1–T7, desain §B).
> SCOPE TERKUNCI (user 2026-10-08): demo saja, **approach-only** (stand-off di atas objek), **tanpa grasp**, **tanpa
> angka akurasi**. Pipeline lama `/task/bring_*` tidak dipakai. **§A ditulis sebelum kode apa pun diubah.**

---

## A. 🔒 Kunci sesi

### A0. Keputusan (user, 2026-10-08, sesi ini)

| # | Pertanyaan | Keputusan |
|---|---|---|
| d1 | Backend eksekusi | **Runner kecil** (`docs/results/p1_demo/demo_runner.py`) yang memanggil alat HW-proven sebagai subprocess — pola `run_g32.py`: `reach_dwell_probe.py --arms <arm> --target=… --approach 0 --trials 1 --tau-max 12.0 --move` lalu `return_rest.py --move`. **Tidak** `gantry_reach_executor`, **tidak** `reach_fusion`. |
| d2 | Sumber suara | **Salin `voice_web_ui` saja** ke `ros2_ws/src/sayai_voice_sim` (nama sama → menimpa `install/` yang basi). Tombol `/task/*` dan tombol **STOP** dibuang. |
| d3 | g1 | **PARKIR 1.5 m, gantry DIAM** sepanjang demo (grup arm-only). Nol perintah gantry → T6 hilang. |
| d4 | Objek / prompt | **bottle**, prompt YOLOE `bottle`. |

### A1. Temuan baru (baca kode, sebelum §A dikunci)

| # | Temuan | Bukti |
|---|---|---|
| T8 | 🔴 `target_cli` **tidak** menyambung ke `gantry_reach_executor` (desain §B salah): ia menerbitkan `#tid` ke `/reach_fusion/set_target` dan `Empty` ke `/reach_fusion/execute`, menunggu `/reach_fusion/status`. Pemakainya `reach_fusion.py` (4 lengan, model GNG `/tmp/arm*_model.npz`, env dari `/topo_map/markers` Isaac, plan+execute satu langkah, **tanpa penyaring**). Executor memakai `~/pick` (indeks). | `target_cli.py:48,70,66`; `reach_fusion.py:291-300`; `gantry_reach_executor.py:441` |
| T9 | 🟡 `object_localizer` `/grasp_target` = **label**, bukan `#tid` → `/target_object` tidak bisa dipilih per track. Runner memakai xyz track langsung dari `/detected_objects/tracks`. | `object_localizer.py:244-251,331-338` |
| T10 | 🔴 `voice_web_ui` punya tombol **STOP** yang memanggil `/task/stop` — tanpa task server ia **tidak berbuat apa-apa** (rasa aman palsu). Dibuang (d2). E-stop = fisik. | `sayai_voice_sim/voice_web_ui.py` (checkout lain) baris 100, 247 |
| T11 | 🟡 **Botol tidak ada di peta lingkungan beku** (`env_static_map` direkam tanpa objek) → penyaring env tidak melihat botol. Pemisah dari botol = **stand-off saja**. | `env_static_map_pub.py`; [p1_g33_map.md](p1_g33_map.md) |
| T12 | 🟡 `target_cli` menyerah menunggu status setelah 60 s (`STATUS_TIMEOUT`); satu jalan demo (rencana ≤ 3 percobaan + gerak + dwell + REST) bisa lebih lama → operator bisa menekan `g` lagi saat lengan masih bergerak. | `target_cli.py:31,252` |
| T13 | 🟢 `reach_dwell_probe --arms <1 lengan>` sudah memakai **semua** penyaring: tuck, RNEA + offset per-aktuator (j2 +7.7 ≤ 14), S18 vs **ketiga** lengan lain + lintas-gantry, env (EnvChecker + `env_static_map`), `retract_check=True`, `--plan-attempts 3`; dieksekusi hanya bila semuanya CLEAR (MARGIN ≠ CLEAR = tolak). Jalur ini yang dipakai G24b–G36. | `reach_dwell_probe.py:dual_trial`, `_plan_and_screen`; `run_g32.py:225` |

### A2. Desain terkunci

```
mic browser → voice_web_ui → /voice/transcript ─┐
D455 → rgbd_perception → seg_router(yoloe 'bottle') → object_localizer → /detected_objects/tracks
                                                 ↓
     target_cli (+ /voice/transcript → _fetch) → /reach_fusion/set_target "#tid"   [keyboard tetap]
                       operator 'g' (WAJIB) → /reach_fusion/execute
                                                 ↓
     demo_runner: xyz track #tid → p_cmd = (x, y, z + 0.25) tool menghadap bawah
                  lengan: base arm_1/arm_2 terdekat (xy) dulu, lalu yang lain bila DITOLAK
                  → reach_dwell_probe --arms <arm> --target=p_cmd --approach 0 --move   (semua penyaring, T13)
                  → MOVED: dwell (monitor) → return_rest --arms arm_1 arm_2 --move
                  → /reach_fusion/status "success|failed: <verdict>"
```

- **Stand-off 0.25 m di atas centroid** (T11): botol ≤ 0.30 m → centroid ≤ 0.15 di bawah puncak → ujung tool ≥ 0.10 m
  di atas puncak.
- **Pengaman runner:** (r1) menolak `execute` saat sibuk; (r2) menolak bila ada pemakai lain `/reach_fusion/execute`
  (`reach_fusion` hidup = dua eksekutor); (r3) xyz diambil saat `execute`, bukan saat pilih; track hilang → tolak.
- `target_cli`: perubahan minimum — langganan `/voice/transcript` (param `voice_topic`, default kosong = mati) memanggil
  `_fetch`; `STATUS_TIMEOUT` → param `status_timeout` (T12). `g` tetap satu-satunya pemicu gerak.
- **Tidak** ada penyaring baru; **tidak** ada `move_dual_table`; gantry tidak diperintah sama sekali.

### A3. Kriteria sukses DEMO-OFF

| # | Kriteria | Lulus bila |
|---|---|---|
| S1 | Sumber suara pulih | `colcon build --packages-select sayai_voice_sim` OK; `voice_web_ui` jalan; POST `/api/transcript` → 1 pesan terlihat di `/voice/transcript` |
| S2 | target_cli suara | transcript "please bring a bottle" → `#tid` track botol di `/reach_fusion/set_target`; **nol** `execute` tanpa `g`; keyboard tetap bekerja |
| S3 | Penyaring tersambung | tiap jalan mock mencatat verdict probe per lengan (PLANNED/MOVED atau nama penolakan) + verdict return_rest |
| S4 | Mock 3 posisi | **P-mudah**: MOVED + REST; **P-tepi**: dicatat apa pun hasilnya; **P-rak (x ≈ 2.1)**: **DITOLAK** kedua lengan (bukan MOVED) — kontrol positif. P-rak MOVED = **GAGAL sesi**. |
| S5 | Pengaman runner | `g` kedua saat sibuk → ditolak (r1) |
| S6 | Bersih | mock dimatikan, `ros2 node list --no-daemon` = 0 |
| S7 | Kamera (opsional) | hanya bila D455 bisa dibuka pasif dari jauh; bila tidak → **dilewati, dilaporkan** |

### A4. Dugaan (dikunci sebelum mock)

| # | Dugaan |
|---|---|
| DD1 | P-mudah: lengan terdekat CLEAR dalam ≤ 3 percobaan rencana; perintah → REST < 150 s di mock. |
| DD2 | P-rak: ditolak oleh **ENV-COLLIDE** (rak di peta beku) pada kedua lengan, atau RETRACT-BLOCKED; tidak ada NO-PLAN murni. |
| DD3 | P-tepi: setidaknya satu lengan ditolak (TORQUE-UNSAFE atau NO-PLAN); yang lain mungkin CLEAR. |
| DD4 | Posisi numerik P-mudah/P-tepi/P-rak dipilih dari TF REST mock (g1 = 1.5) dan ditulis di §A5 **sebelum** jalan pertama. |

### A5. 🔒 Posisi uji (ditulis sesudah mock naik + g1 mock diparkir 1.500 m, SEBELUM jalan pertama)

Geometri dari mock (TF) + peta beku `p1_g34/env_static_map.npz` (pusat voxel mentah, tanpa pad):
basis arm_1 (1.900, 0.360, 1.952), arm_2 (1.100, 0.360, 1.952); tool REST z 0.956. **Meja kerja** puncak voxel
z ≈ 0.80–0.87 di x 1.0–2.2, y +0.4..+0.9. **Rak** x 2.0–2.4, y −0.05..−0.6, puncak ≈ 0.9–1.2. **Dinding/rak tinggi**
y ≈ +1.0, x 0.6–1.5, kolom voxel z 0.31–1.93.

> **Temuan T14 (geometri, sebelum jalan):** dengan g1 di PARKIR 1.5 rak berada **di/lewat batas jangkauan** arm_1
> (p_cmd mana pun di/atas rak ≥ 0.94 m dari basis; jangkauan ≈ 1.0 m) → penolakan di rak hampir pasti datang dari IK/
> MoveIt (NO-PLAN), bukan dari penyaring env. Maka ditambah kontrol positif kedua yang **terjangkau**: P-dinding.

| ID | Centroid botol (world) | p_cmd (+0.25 z) | d(p_cmd, voxel) | Jarak ke basis | Urutan lengan | Peran |
|---|---|---|---|---|---|---|
| P1 mudah | (1.75, 0.55, 0.97) | (1.75, 0.55, 1.22) | 0.381 | a1 0.771 / a2 0.997 | arm_1, arm_2 | harus MOVED + REST |
| P2 tepi | (2.30, 0.85, 0.95) | (2.30, 0.85, 1.20) | 0.157 | a1 0.983 / a2 1.499 | arm_1, arm_2 | dicatat |
| P3 rak (P+) | (2.15, −0.15, 0.85) | (2.15, −0.15, 1.10) | **0.046** (di dalam pad 5 cm) | a1 1.024 / a2 1.445 | arm_1, arm_2 | **harus DITOLAK** |
| P4 atas-rak | (2.05, −0.10, 0.90) | (2.05, −0.10, 1.15) | 0.157 | a1 0.937 / a2 1.326 | arm_1, arm_2 | dicatat (margin dekat rak) |
| P5 dinding (P+) | (1.25, 0.98, 1.20) | (1.25, 0.98, 1.45) | **0.010** | a1 1.029 / a2 0.812 | arm_2, arm_1 | **harus DITOLAK** |

Dugaan per posisi (DD5–DD9). **DD2 diganti DD5/DD9 sebelum jalan** — alasannya: `my_workcell.launch.py` memasukkan
`env_static_map` ke scene MoveIt (env_map:=true), jadi goal di dalam rintangan ditolak MoveIt (NO-PLAN) **sebelum**
penyaring env melihat apa pun; penyaring env baru menentukan di kasus MARGIN sepanjang jalur.

| # | Dugaan |
|---|---|
| DD5 | P3: kedua lengan ditolak; arm_1 **NO-PLAN** (goal di luar jangkauan / di dalam kotak rak), arm_2 NO-PLAN. |
| DD6 | P1: arm_1 MOVED ≤ 3 sampel, dwell lulus, return_rest rc 0. |
| DD7 | P2: arm_2 NO-PLAN pasti (1.50 m); arm_1 ditolak (NO-PLAN atau TORQUE-UNSAFE) peluang ≥ 60 %. |
| DD8 | P4: 50/50 — arm_1 MOVED atau ditolak; bila MOVED, jarak env minimum sepanjang jalur dicatat. |
| DD9 | P5: kedua lengan **NO-PLAN** (goal di dalam kotak dinding pada scene MoveIt). |

Persepsi palsu: satu track `bottle` per posisi (tid 101..105), `/detected_objects` + `/detected_objects/tracks`
(latched), 2 Hz. Suara palsu: POST `/api/transcript` "please bring a bottle" ke `voice_web_ui`. `g` lewat stdin
target_cli. Uji r1: satu `execute` tambahan saat P1 berjalan.

---

## B. Hasil

Mock domain 77 (`use_fake_hardware:=true enable_gantry_bridge:=false`, env_map True, g1 mock diparkir 1.500 m lewat
`pose_to_g`, keempat lengan REST). Rantai penuh tiap jalan: `fake_tracks` → POST suara → `target_cli` `_fetch` →
`g` (stdin) → `demo_runner` → probe → (MOVED) dwell → `return_rest`. Data: [results/p1_demo/mock/](results/p1_demo/mock/)
(`runNN.json` + log per alat), harness [results/p1_demo/harness/](results/p1_demo/harness/).

### B1. Per posisi

| Pos | Run | Verdict (urutan lengan) | Sebab sebenarnya (`ik_diag`, roll 12) | Total |
|---|---|---|---|---|
| P1 mudah | 01, 07, 08 | arm_1 **MOVED → SUCCESS** (dwell), return_rest rc 0 ×3 | sampel ke-1 / ke-2 / **ke-3** lolos; sisanya TORQUE-UNSAFE (cabang j2 ≥ 101 %) | 69.6 / 68.0 / 68.8 s |
| P1 plan-only | 10 | arm_1 PLANNED, **nol gerak** (joint_states sebelum = sesudah) | — | 16.7 s |
| P2 tepi | 02 | arm_1 NO-PLAN, arm_2 NO-PLAN | arm_1 kin **0/12** → di luar jangkauan | 93.0 s |
| P3 rak (P+) | 03 | arm_1 NO-PLAN, arm_2 NO-PLAN → **DITOLAK** | kin 0/12 → jangkauan, **bukan** tabrakan | 94.4 s |
| P4 atas-rak | 04 | NO-PLAN, NO-PLAN | kin 0/12 → jangkauan | 92.1 s |
| P5 dinding (P+) | 05 | arm_2 NO-PLAN, arm_1 NO-PLAN → **DITOLAK** | arm_2 kin 0/12 pada 0.812 m → jangkauan, **bukan** dinding | 92.2 s |
| P6 dinding terjangkau (P+, pasca-hoc) | 06 | arm_2 NO-PLAN, arm_1 NO-PLAN → **DITOLAK** | arm_2 kin **12/12**, bebas-tabrakan **0/12** → ditolak scene MoveIt (kotak `env_static_map`) | 96.1 s |
| P7 sisi arm_2 (pasca-hoc) | 09 | arm_2 **TORQUE-UNSAFE** 3/3 (j2 14.50–14.83 / 14 = 104–106 %), arm_1 NO-PLAN | penyaring torsi aktif di rantai demo | 48.0 s |

Penyaring di P1 (run01): RNEA+offset terketat **t1_a1_joint_3 92 %**; S18 antar-lengan min 570.1 mm; env min 112.3 mm
(jari); retract lurus: se-gantry 570.4 mm, antar-gantry 862.6 mm, env 112.3 mm → CLEAR. Run07/08: S18 480–484 mm, env
115.1 mm. Torsi puncak return_rest 0.000 (mock).

**Grid toleransi persepsi** (plan-only, p_cmd P1 ± 5 cm xy, ± 3 cm z, 18 titik): **17/18 PLANNED**, 1 TORQUE-UNSAFE
(1.80, 0.55, 1.19) setelah 3 sampel. Semua rencana lolos memakai **j3 90–92 %**; cabang lain j2 101–112 % → ditolak.

### B2. Kriteria

| # | Hasil |
|---|---|
| S1 | ✅ build `sayai_voice_sim` OK; POST → `/voice/transcript` "please bring a bottle"; `/api/task/stop` → 404 |
| S2 | ✅ suara → `#101` di `/reach_fusion/set_target`; runner terima target, **nol** execute sampai `g` |
| S3 | ✅ verdict tiap lengan + retract + return_rest tercatat per jalan (`runNN.json`, log alat) |
| S4 | ✅ menurut huruf: P1 MOVED + REST 3/3; P3, P5 (dan P6) DITOLAK, tak ada MOVED. ⚠ lihat F2 — P3/P5 ditolak oleh **jangkauan**, bukan penyaring tabrakan |
| S5 | ✅ r1: `execute` kedua saat run 1 → "DITOLAK ... (r1)". ⚠ r2 (reach_fusion hidup) dan r3 (track hilang) **tidak diuji** |
| S6 | ✅ domain 77 dan 79: `node list --no-daemon` = 0 |
| S7 | ⚠ **sebagian** — lihat F4/F5 |

### B3. Dugaan

| # | Hasil |
|---|---|
| DD1 | ✅ P1 ≤ 3 sampel, 68–70 s < 150 s — tetapi run08 baru lolos di sampel **3/3** (F3) |
| DD2 | ✗ (sudah diganti DD5/DD9 sebelum jalan) — tidak ada ENV-COLLIDE di seluruh sesi |
| DD3 | ✅ P2: kedua lengan ditolak |
| DD4 | ✅ §A5 ditulis sebelum jalan pertama |
| DD5 | ✅ label (kedua NO-PLAN); sebab = jangkauan (diduga "jangkauan / kotak rak") |
| DD6 | ✅ 3/3 |
| DD7 | ✅ arm_1 NO-PLAN, arm_2 NO-PLAN |
| DD8 | → ditolak (NO-PLAN, jangkauan) |
| DD9 | ✅ label, ✗ **alasan**: kotak dinding tidak pernah dicapai — P5 di luar jangkauan |

### B4. Temuan

- **F1 — Jangkauan tool-menghadap-bawah ≈ 0.7–0.8 m dari basis, bergantung arah** (0.771 lolos 12/12; 0.812 dan
  0.997 gagal 0/12; 0.681 di satu arah gagal, 0.703 di arah lain lolos). Angka "1.005 m" (selubung link,
  `interarm_collision`) **bukan** jangkauan target approach. Akibat: dengan g1 PARKIR 1.5 zona demo hanya meja di
  depan arm_1 (sekitar P1); rak di luar jangkauan sama sekali.
- **F2 — 🔴 Kontrol positif P3/P5 kosong terhadap tabrakan** (Rule 12). Keduanya ditolak IK sebelum ada geometri yang
  diperiksa. P6 (pasca-hoc) menunjukkan scene MoveIt menolak goal yang menabrak dinding. **Jalur tolak EnvChecker
  (`ENV-COLLIDE`/MARGIN) tidak pernah terpicu di rantai demo** — hanya nilai CLEAR (112–115 mm) yang teramati. Jalur
  tolaknya sendiri teruji di G33/G34 (P+ ev 8) + `--self-test`, tidak diulang di sini.
- **F3 — Torsi ketat.** P1 lolos hanya pada cabang IK j3-berat (j3 terkoreksi 90–92 % rating); OMPL memilih cabang
  j2-berat ≈ separuh sampel (run07 1/2, run08 2/3 ditolak; grid 1/18 habis 3 sampel). Peluang satu `g` gagal di P1 ≈
  0.5³ ≈ 12 % → operator cukup menekan `g` lagi (tiap `g` = 3 sampel baru, penyaring tak dilonggarkan). Di HW: j3
  terukur belum pernah dikalibrasi di pose ini (offset +6.6 dari G16 joint_2).
- **F4 — 🔴 `src/reachability_gng/mobileclip_blt.ts` terpotong** (145 MB vs 599 MB di `ros2_ws/`, 2026-09-21 —
  kemungkinan unduhan parsial saat disk penuh, root 98 %). Ultralytics memuatnya dari **cwd**: `rgbd_perception`
  diluncurkan dari dir itu → tiap frame `YOLOE inference failed ... failed finding central directory` (hanya WARN),
  `/detected_objects/tracks` `[]` selamanya. Diluncurkan dari `ros2_ws/` → bersih. **File tidak dihapus** (keputusan user).
- **F5 — Persepsi nyata tidak teruji.** Kedua D455 terbuka pasif (domain 79), YOLOE `bottle` dimuat di cuda:0,
  `object_localizer` (remap `seg/`) jalan — tetapi lab **gelap** (20:30, frame hitam,
  [rgbd.jpg](results/p1_demo/mock/logs/rgbd.jpg)) → label `{}`, track `[]`. **Rosbag tidak disimpan** (isinya kosong).
  Galat lokalisasi botol nyata di P1 = belum diketahui.
- **F6 — UX penolakan lambat:** lengan yang di luar jangkauan menghabiskan 3 × 15 s OMPL sebelum NO-PLAN → objek di
  luar zona = ~93 s sampai `failed`. Tidak diubah (Rule 2); pra-cek IK bisa ditambah nanti bila mengganggu demo.
- **F7 — `rgbd_perception` tidak menjalankan `object_localizer`**; harus dijalankan terpisah dengan remap
  `/<ns>/instance_segmentation*` → `/<ns>/seg/instance_segmentation*` (seperti `perception.launch.py`) —
  [harness/loc_up.sh](results/p1_demo/harness/loc_up.sh).

## C. Keadaan akhir (Rule 12)

- **Gerak nyata: NOL.** Lengan/gantry nyata tidak diperintah; tidak ada bring-up HW. D455 dibuka **pasif** (domain 79)
  dan ditutup. Mock (domain 77) naik 20:01, mati 20:41 — SIGINT helper lalu launch (SigIgn tanpa SIGINT), `node list
  --no-daemon` 0 di 77 dan 79.
- **Diubah:** `ros2_ws/src/reachability_gng/reachability_gng/target_cli.py` (+param `voice_topic`, `status_timeout`;
  `_on_voice` → `_fetch`).
- **Baru:** `ros2_ws/src/sayai_voice_sim/` (hanya `voice_web_ui`, tombol `/task/*` + STOP palsu dibuang; dibangun
  `--packages-select`); `docs/results/p1_demo/{demo_runner,fake_tracks,ik_diag}.py`, `harness/`, `mock/`.
- **Tidak disentuh:** penyaring (`reach_dwell_probe`, `env_collision`, `interarm_collision`, `return_rest`),
  `reach_fusion`, `gantry_reach_executor`, `object_localizer`, file `mobileclip_blt.ts` terpotong (F4).
- **Terbuka:** persepsi nyata (F5), jalur tolak EnvChecker di rantai demo (F2), r2/r3 (S5), j3 terukur di P1 (F3).
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi multi-langkah dengan mock + kamera; dilaporkan.

## D. Prompt DEMO-HW (salin ke chat BARU)

**Rekomendasi: Opus, effort TINGGI** — gerbang plan-only + mock 3/3 ada (ground truth untuk gerak), tetapi tiga hal
pertama kali di HW dan galatnya diam: lokalisasi botol nyata belum pernah jalan (F5; persepsi meleset > 5 cm keluar
dari grid yang lolos), j3 berjalan di 90–92 % rating terprediksi (F3), dan YOLOE bisa diam-diam mati (F4, hanya WARN).

```
Sesi DEMO-HW -- demo P1 suara -> YOLOE -> approach di HW, 1-3 jalan. Repo ceiling_arm, branch feat/rgbd-deploy.
BACA PENUH: CLAUDE.md; docs/p1_demo_off.md (A2 desain, A5, B1-B4, C); docs/results/p1_demo/demo_runner.py +
harness/*.sh. SCOPE: approach-only (stand-off 0.25 m di atas centroid), tanpa grasp, tanpa angka akurasi. G36b HW
tetap prioritas di atas sesi ini.
FAKTA DEMO-OFF: rantai jalan di mock (P1 (1.75,0.55,0.97) arm_1 MOVED->SUCCESS->REST 3/3, 68-70 s; grid +-5 cm 17/18);
jangkauan tool-bawah ~0.7-0.8 m (F1) -> HANYA meja di depan arm_1 dengan g1 PARKIR 1.5; P1 bisa butuh 3 sampel
(cabang j2 ditolak torsi, F3) -> 'g' lagi bila failed TORQUE-UNSAFE; perception nyata BELUM teruji (lab gelap, F5);
mobileclip_blt.ts di src/reachability_gng TERPOTONG -> luncurkan persepsi dari ros2_ws/ (F4).
GERBANG (tanya operator SEBELUM apa pun): K0 sel = akhir sesi terakhir (lengan REST, g1 PARKIR 1.499, g2 0, rot 0/0,
rak tidak dipindah)? K1 operator di e-stop FISIK selama gerak (voice_web_ui TIDAK punya stop)? K2 lampu lab NYALA;
satu botol <= 0.30 m berdiri di meja di sekitar (1.75, 0.55) world = ~15 cm ke arah x kecil dari basis arm_1,
~19 cm ke +y dari garis gantry 1; tak ada botol lain dalam pandangan? K3 tidak ada voice_command_manager /
real_robot_task_server / reach_fusion hidup (r2)?
0. KUNCI §A di docs/p1_demo_hw.md (kriteria, dugaan) SEBELUM bring-up.
1. Self-test env_collision + interarm; bring-up nyata (enable_gantry_bridge:=true, SIG_DFL + setsid nohup, 7/7,
   ARMED); env_static_map_pub --once True; t1_linear ~1.499; js_record + reach_dwell_monitor.
2. Persepsi (harness/cam_up.sh + loc_up.sh, domain sama dengan HW): cek TIDAK ada 'inference failed' di log;
   /detected_objects/tracks memuat 'bottle' -- catat xyz vs (1.75,0.55,~0.97). |dxy| > 5 cm dari grid yang lolos atau
   z tak masuk akal -> BERHENTI, perbaiki penempatan dulu (jangan perlebar).
3. voice_web_ui (https, Chrome mic); target_cli voice_topic:=/voice/transcript status_timeout:=900.
4. demo_runner --plan-only: ucapkan perintah -> g -> harus 'success arm_1 (arm_1:PLANNED)'; catat semua penyaring.
5. Restart demo_runner TANPA --plan-only: ucapkan -> g -> MOVED -> dwell -> return_rest. Rekam video + waktu
   perintah->selesai + torsi j3 terukur vs prediksi. 1-3 jalan; failed TORQUE-UNSAFE = 'g' lagi, bukan ubah penyaring.
6. Akhir: lengan REST, g1 tetap PARKIR 1.5 (tak pernah diperintah); matikan bersih; laporan + gambar arsitektur.
```
