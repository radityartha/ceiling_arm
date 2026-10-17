# Prompt G64-C fase 5–6 (HW → laporan) — salin ke sesi baru

```text
Lanjutkan G64-C di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy) mulai FASE 5 (HW NYATA, domain 0). NOL gerak
lengan/rel/gripper tanpa izin. Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin,
operator HADIR di sel dekat e-stop (tanyakan dulu lewat tombol).

BACA PENUH dulu: CLAUDE.md; docs/p1_g64c.md (§A TERKUNCI sha256 712fa8c7… — JANGAN diubah; §B1–B5); docs/p1_g64c_prompt.md
(ATURAN asli); results/p1_g64c/g64c_fold_rnea.json, g64c_fold_times.json, g64c_mock.sh (pola perintah mock = pola HW tanpa
domain 77); harness HW lama: results/p1_demo_hw/harness/stack_up.sh (+ env.sh), results/p1_g57/p1hw_prep.sh / p1hw_run.sh /
p1hw_home.sh; memori g64c-solver-offline, g64-torque-model-verified, ceiling-arm-fold-is-torque-limited, feedback-tau-max-14-operator,
gantry-bridge-status (enable_gantry_bridge:=true WAJIB), move-group-hw-memory-leak (octomap_updaters:=false), ros2-launch-pid-leak,
workcell-known-issues (/joint_states DUA penerbit → gabung per nama ≥ 0.5 s), network-192-168-2-subnet, gantry-bridge-debounce.

KEADAAN (2026-10-17 ~16:40): fase 1–4 ✓ (§B5: C5 hidup, C6 s7/s10 6/6, C7 TAS mock k=3 2/2 0 TUCK, C8 negatif + solve ulang).
Biaya lipat terukur 44.83 s. Jadwal HW = results/p1_g64c/run_c3_cfold44.83_c3/demo_candidates.json (variant DEMO, seed 0):
g1 (1.40, 0°) REST M1 arm_1; g2 mode T1 (0.45, −35°) TAS arm_3 (operator: stop solver, tombol 10-17). RNEA garis lipat: puncak
DI T1, j2 3.815+7.7 = 11.52 ≤ 14; 🔴 j3 2.555+6.6 = 9.16 (+margin 0.82 = 9.97/10) — mock torsi 0, j3 BELUM PERNAH diukur.
Mock 77 MATI. Disk / 100 % (±6 GB bebas): log besar → gz, jangan simpan rosbag.

FASE 5 (C9, urut, berhenti di langkah pertama yang ✗):
 0. Pra-cek tanpa gerak: IP 192.168.2.x ada, tidak ada move_group/ros2_control basi (scan comm, bukan pgrep -f), stack_up.sh
    (use_fake_hardware:=false, enable_gantry_bridge:=true, octomap_updaters:=false) via setsid nohup; controller aktif;
    monitor + perekam (cek file tumbuh); K0 ukur ulang (--rail1 1.40 --rail2 0, sel diharapkan REST). K0 ✗ → tanya.
 1. [izin] lipat di tempat SATU lengan (usul arm_3, g2 p0): return_rest --arms arm_3 --move --no-plan --target T1 --strict
    (DRY dulu). Ukur effort /joint_states selama gerak + tahan ≥ 10 s di T1: j2 DAN j3 puncak + tahan vs RNEA+offset
    (j2 11.52, j3 9.16) → laporkan |ukur − prediksi|. j3 > 10 atau sendi apa pun > 14 → BERHENTI + amankan + tanya.
 2. [izin] buka arm_3: --target REST --strict. Ukur sama.
 3. [izin] kedua lengan g2 lipat (arm_3 arm_4) → tahan → [izin] buka.
 4. [izin] lipat g2 → [izin] traverse pendek T1 (pose_to_g --allow-t1, stop yang ada di jadwal / p0, DRY dulu; ukur j3 saat
    traverse) → [izin] pulang p0 → [izin] buka.
 5. TAS lewat sistem: p1hw-pola plan-only k=3 (g31_screen --task-cli) + run_g54 --dry, lalu [izin] run_g54 --seed 0
    --plan run_c3_cfold44.83_c3/demo_candidates.json --variant DEMO --final-retract --goal-tol-deg 0.05 --tau-max 14
    --goal-correct (setsid nohup) → [izin] pulang g47_home.sh 0 <plan> DEMO (DRY dulu).
 Tiap gerak: tidak ada AUTO-STOP, 0 gerak tak diminta, torsi ≤ 14 terukur. Fault / LED merah / benturan → berhenti, tanya.
FASE 6: §B6 (C9 + j2/j3 ukur vs RNEA, D676), docs/p1_campaign_state.md, memori (g64c-…), dampak paper (P1 lama = mode REST;
mode T1 = perluasan, data lama tetap sah? → tombol operator), DEMO-SYS ulang dengan g64c_demo.py (G64C_ENV=1) bila operator mau,
rekomendasi model/effort.

ATURAN: = p1_g64c_prompt.md (penyaring tidak dilonggarkan, MARGIN = TOLAK, kotak 10 mm, fail loud, setsid nohup, jangan pkill -f /
pola yang muncul di baris perintah sendiri — pakai berkas skrip atau comm, jangan sentuh /home/kubota, commit per bagian hanya
berkas sendiri, Rule 6 checkpoint). Bahasa Indonesia, singkat (caveman). Keadaan sel (tidak disentuh sejak 11:50 10-17): semua
lengan REST, papan DILEPAS, g1 1.40, g2 0, stack + kamera MATI, botol putih di M1, tas kertas di atas lemari sisi gantry 2.
```

Model: Opus, effort **high** (gerak HW pertama ke T1; j3 tanpa cadangan, torsi diam-diam).
