# P1 / DEMO — suara → YOLOE → approach (persiapan offline, nol gerak)

> Ditulis 2026-10-08. Keputusan (user, 2026-10-08): suara + YOLOE + approach masuk P1 **hanya sebagai demo
> sistem** (1 bagian kecil + 1 gambar), **bukan** kontribusi, **tanpa** angka akurasi, **tanpa grasp**. Evaluasi
> penuh → P2 (suara = sumber "tugas baru" online, [p2_p3_roadmap.md](p2_p3_roadmap.md)). Kontribusi utama P1 tetap
> penjadwalan reach-and-dwell dengan target dari seed (`make_instance`), [p1_state.md](p1_state.md) §5.5.
>
> Dokumen ini hanya hasil membaca kode (tidak ada yang dijalankan). Dua sesi: **DEMO-OFF** (integrasi, mock, nol
> gerak lengan) lalu **DEMO-HW** (sekali jalan nyata). G36b HW tetap prioritas di atas DEMO-HW.

---

## A. Temuan — pipeline yang ada TIDAK siap jalan

| # | Temuan | Bukti |
|---|---|---|
| T1 | 🔴 **Suara tidak tersambung ke YOLOE.** Perintah suara memicu **urutan sendi hardcode** (`/task/bring_bottle` → `take_bottle_demo.launch.py` → `run_take_bottle.py`: grasp + handover arm_4→arm_3→arm_2, sudut sendi tetap). Tidak ada `/detected_objects` di rantai suara. | `sayai_voice_sim/real_robot_task_server.py` TASKS; `workcell_description/scripts/run_take_bottle.py:247` |
| T2 | 🔴 **Paket suara hilang dari repo ini.** `install/sayai_voice_sim` menunjuk ke `ros2_ws/src/sayai_voice_sim` yang **tidak ada**. Sumbernya hanya di checkout lain `/home/user1/ceiling_arm` (branch `siap_demo`, commit terakhir 2026-07-29). `start_voice_demo.sh` juga hanya di sana. | `build/sayai_voice_sim/package.xml` → path tak ada |
| T3 | 🟢 **Jembatan bahasa → objek sudah ada**, tapi lewat keyboard: `target_cli._fetch(sentence)` mem-parse "please bring a teddy bear" → cocokkan label track YOLOE → kirim `#tid` → `g` = eksekusi approach. Cukup diberi masukan `/voice/transcript` (String, diterbitkan `voice_web_ui`, mic browser). | `reachability_gng/target_cli.py:120` |
| T4 | 🟡 **Eksekutor hanya arm_1/arm_2 (gantry_1)**, pilih lengan + konfigurasi 8-DOF berbasis energi, MoveIt `gantry_1_with_arm_<n>`. Ada mode approach-only (`carve_target:=false allow_target_collision:=false`, `box_clearance`). | `gantry_reach_executor.py` docstring; `pick_stack.launch.py` |
| T5 | 🔴 **Eksekutor melewati SEMUA penyaring keselamatan sekarang**: tidak ada EnvChecker / `env_static_map` screen, S18 antar-lengan, RNEA torsi (+7.7 ≤ 14), `retract_check`. Ia bergantung pada octomap live — yang **mati di HW** (tidak ada `collision_cloud`). Dibangun untuk Isaac (`pick_stack` mengandalkan kamera bridge Isaac). | [octomap memory] / `CLAUDE.md` sensors_3d; `pick_stack.launch.py` docstring |
| T6 | 🟡 **Gerak gantry lama vs sekarang bertentangan.** Urutan suara lama memakai `move_dual_table` langsung; tumpukan sekarang = bridge JTC (`enable_gantry_bridge:=true`, wajib untuk gerak lengan apa pun) dan **jangan pernah `move_dual_table` saat ARMED**. g1 kini PARKIR di 1.5 m. | `run_take_bottle.py:86`; [gantry-bridge-debounce], [gantry1-park-far-end] |
| T7 | ❓ Belum pernah tercatat: rantai YOLOE (`rgbd_perception.launch.py` → `seg_router` yoloe → `object_localizer`) jalan di D455 nyata bersama tumpukan G36. G16 sengaja tanpa persepsi. Whisper dimatikan default (driver audio pernah macet D-state) → pakai mic browser. `voice_web_ui` bisa macet (TLS accept). | `docs/p1_g16_hw.md:687`; `start_voice_demo.sh`; [workcell-known-issues] |

**Konsekuensi:** demo ≠ "tinggal jalankan". Perlu satu sesi integrasi offline (DEMO-OFF). Pipeline lama
`/task/bring_*` (hardcode + grasp) **tidak dipakai** untuk demo P1 — ia bukan suara → YOLOE, dan ia grasp.

## B. Desain demo minimum (usulan, dikunci di DEMO-OFF §A)

```
mic browser → voice_web_ui → /voice/transcript ──┐
D455 → rgbd_perception → seg_router(yoloe) → object_localizer → /detected_objects(+/tracks)
                                                  ↓
                    target_cli (+ masukan /voice/transcript) → #tid → konfirmasi operator (g)
                                                  ↓
          gantry_reach_executor PLAN-ONLY (approach-only, stand-off di atas objek, arm_1/arm_2)
                                                  ↓
     SARING dengan alat yang sudah ada: EnvChecker + env_static_map, S18 vs lengan lain, RNEA+7.7≤14, retract_check
                                                  ↓ CLEAR saja
                       eksekusi (bridge JTC) → dwell 2.0 s → pulang REST (return_rest)
```

- Lengan: hanya arm_1/arm_2 (eksekutor sudah begitu); arm_3/arm_4 diam di REST, g2 diam.
- Gantry_1: mulai dari PARKIR 1.5 m atau 0 — diputuskan DEMO-OFF; akhir sesi g1 kembali PARKIR.
- Konfirmasi manusia (`g`) **wajib** sebelum gerak — pipeline suara tidak boleh langsung menggerakkan lengan.
- Yang dilaporkan di paper: 1 gambar arsitektur + 1–3 jalan demo (perintah, objek, lengan terpilih, waktu
  perintah→selesai, foto/video). Bukan statistik.

## C. Model / effort

- **DEMO-OFF: Opus, effort TINGGI** — menyambung eksekutor lama ke penyaring keselamatan; galat di sini diam
  (rencana tak disaring tetap kelihatan CLEAR) dan baru muncul sebagai tabrakan di HW.
- **DEMO-HW: Opus, effort SEDANG–TINGGI** — gerbang plan-only dari DEMO-OFF jadi ground truth; tetap tinggi bila
  DEMO-OFF meninggalkan butir terbuka.

---

## D. Prompt DEMO-OFF (salin ke chat BARU; boleh dijalankan dari jauh — nol gerak lengan/gantry)

```
Sesi DEMO-OFF -- integrasi demo P1 suara -> YOLOE -> approach (OFFLINE + mock, NOL gerak HW). Repo ceiling_arm,
branch feat/rgbd-deploy. BACA PENUH: CLAUDE.md; docs/p1_demo_voice_yoloe.md (A temuan T1-T7, B desain);
docs/p1_g36_retract_branch.md (C keadaan sel); reachability_gng/{target_cli,gantry_reach_executor,seg_router,
object_localizer}.py; launch/{rgbd_perception,pick_stack}.launch.py; scripts/{env_collision,return_rest,
reach_dwell_probe}.py (_plan_and_screen, screen_retract); /home/user1/ceiling_arm/ros2_ws/src/sayai_voice_sim/.
SCOPE TERKUNCI (user 2026-10-08): demo saja, approach-only (stand-off di atas objek), TANPA grasp, TANPA angka
akurasi. Pipeline lama /task/bring_* (sendi hardcode + grasp) TIDAK dipakai.
0. KUNCI §A di dokumen sesi baru (docs/p1_demo_off.md): kriteria sukses, dugaan, keputusan terbuka -- tanya user:
   (a) salin sayai_voice_sim ke ros2_ws/src (hanya voice_web_ui yang perlu) ATAU node kecil baru? (b) g1 mulai di
   PARKIR 1.5 atau 0? (c) objek demo apa + prompt YOLOE apa?
1. Pulihkan sumber suara (T2) sesuai (a); colcon build paket itu saja; voice_web_ui jalan, /voice/transcript terbit.
2. target_cli: tambah masukan /voice/transcript ke _fetch (keyboard tetap jalan; 'g' konfirmasi TETAP wajib).
   Perubahan minimum (Rule 2/3).
3. Sambung penyaring (T5): rencana eksekutor (plan-only) -> EnvChecker+env_static_map, S18 vs lengan lain, RNEA
   +7.7<=14, retract_check -- pakai fungsi yang sudah ada, JANGAN tulis penyaring baru. Eksekusi hanya bila CLEAR;
   MARGIN = TOLAK. Gantry hanya lewat bridge JTC (T6), tidak pernah move_dual_table.
4. Uji MOCK (domain terpisah, setsid nohup, SIG_DFL): /detected_objects palsu (PoseArray + /tracks) di >=3 posisi
   (1 mudah, 1 di tepi jangkauan, 1 dekat rak x~2.1 = kontrol positif harus DITOLAK) -> transcript palsu ->
   fetch -> g -> plan -> saring -> eksekusi mock -> dwell -> REST. Catat verdict tiap penyaring.
5. Bila kamera D455 bisa dibuka dari jauh (pasif, aman): jalankan rgbd_perception + seg_router yoloe SAJA (tanpa
   bring-up lengan), cek /detected_objects pada objek nyata di meja, simpan rosbag pendek untuk replay mock.
6. Laporan §B (apa yang lulus, apa yang tidak, Rule 12), commit, tulis prompt DEMO-HW di §D + model/effort.
ATURAN: NOL gerak lengan/gantry nyata; mock dimatikan bersih (node list --no-daemon = 0); job > 10 menit via
setsid nohup; pkill -f membunuh shell sendiri (pilih PID via ps/awk).
```

## E. Kerangka prompt DEMO-HW (ditulis ulang final oleh DEMO-OFF §D)

```
Sesi DEMO-HW -- demo P1 suara -> YOLOE -> approach di HW, 1-3 jalan. BACA: docs/p1_demo_off.md (B, D).
GERBANG (tanya operator SEBELUM apa pun): K0 sel = akhir sesi terakhir (lengan REST, g1 PARKIR 1.5, g2 0, rot 0/0,
rak tidak dipindah)? K1 operator di e-stop selama gerak? K2 objek demo diletakkan di posisi yang sudah lulus
plan-only DEMO-OFF?
1. Self-test env_collision + interarm; bring-up nyata (enable_gantry_bridge:=true, 7/7, ARMED); env_static_map_pub
   --once True; rgbd_perception + seg_router yoloe; voice_web_ui; js_record.
2. Plan-only nyata untuk tiap objek -> harus CLEAR semua penyaring sebelum gerak pertama.
3. Ucapkan perintah -> fetch -> operator 'g' -> eksekusi -> dwell 2.0 s -> REST. Rekam video + waktu perintah->selesai.
4. Akhir: g1 PARKIR 1.5; matikan bersih; laporan + gambar arsitektur untuk naskah.
```
