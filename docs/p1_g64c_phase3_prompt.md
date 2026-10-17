# Prompt G64-C fase 3–6 (runner mode T1 → mock 77 → HW) — salin ke sesi baru

```text
Lanjutkan G64-C di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy) mulai FASE 3. NOL gerak lengan/rel/gripper tanpa
izin. Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin.

BACA PENUH dulu: CLAUDE.md; docs/p1_g64c.md (§A TERKUNCI sha256 712fa8c7… — JANGAN diubah; §B1–B2 hasil fase 1–2);
docs/p1_g64c_prompt.md (fase + ATURAN asli, tetap berlaku); results/p1_g64c/g64c_make.py (T1, held_T, cos_line, screen_line_T,
_stop_jobT, mode/record) + g64c_demo.py + run_c3/demo_candidates.json (jadwal TAS: mode {1: REST, 2: T1}, g2 (0.45, −35°)
arm_3); runner: results/p1_g54/run_g54.py, reserve.py, p1_g32/run_g32.py, pose_to_g.py, p1_g31/g31_screen.py (events, DRY),
scripts/return_rest.py, scripts/reach_dwell_probe.py (start / retract / START-JUMP / FORBIDDEN_TUCK), p1_g47/g47_home.sh.
Memori: ceiling-arm-fold-is-torque-limited, feedback-tau-max-14-operator, gantry-bridge-debounce, ros2-launch-pid-leak,
move-group-hw-memory-leak, interarm-collision-srdf-disabled, demo-sys-uses-p1-scheduler.

KEADAAN (2026-10-17 sore): fase 1 ✓ (C1 4/4), fase 2 ✓ kecuali C2 harfiah 7/8 (s11 = drift cache, wrapper ≡ g55 asli — §B2).
🔴 T1 duduk di batas j3: 2.56 + 6.6 + 0.82 = 9.98 / 10 → semua garis T1 tau_excess −0.028. Biaya lipat/buka di solver
SEMENTARA 45.34 s (a_r) — wajib diganti nilai mock terukur (G64C_C_FOLD) lalu solve ulang C3 (gerbang C8).
D670 (4-kombinasi pada 8 seed) — lihat results/p1_g64c/g64c_chain2.log / g64c_combos_*.json (mungkin masih jalan / belum
lengkap; resumable: jalankan ulang g64c_chain2.sh).

FASE 3 (RUNNER) — temuan bacaan sesi lalu, harus ditangani:
 a. return_rest.py menyaring se-gantry / antar-gantry / env SAJA (tanpa torsi RNEA, tanpa kotak) → tambah --target {REST,T1}
    (bawaan REST) + --strict (torsi statis RNEA + 7.7 + 0.66 ≤ 14 tiap titik tiap lengan + kotak scene ≥ 10 mm, = g42 _screen);
    target T1 = garis lurus saja (tanpa cadangan MoveIt), tolak → cetak TUCK-REFUSED, rc 1. argv lama = perilaku lama bit-identik.
    Event: fold = `--target T1 --strict` dari REST di p0; retract mode T1 = `--target T1 --strict`; unfold = `--target REST --strict`
    di p0. return_rest --seconds 30 bawaan → biaya lipat ≈ 30 s + overhead (ukur, jangan tebak).
 b. run_g54 ARM_CANNOT memuat 'TUCK-REFUSED' (= log lalu lanjut, makna lama: FORBIDDEN_TUCK probe). Penolakan fold/unfold/retract-T1
    TIDAK boleh "lanjut" → AUTO-STOP (lengan tidak bergerak, jadwal tak dapat diteruskan). Bedakan sumbernya.
 c. g31_screen.events: tambah mode per gantry (kandidat `mode`, tanpa = REST bit-identik): T1 → [fold] di awal, retract → T1,
    sesudah tugas terakhir: retract T1 → traverse pulang ke p0 → [unfold]. pose_to_g S13 sudah menerima p0 (tak perlu ubah).
 d. pose_to_g S12: `--allow-t1` (eksplisit) → terima REST ATAU T1 ±0.5° kedua lengan; bawaan = lama.
 e. reach_dwell_probe: `--start-pose T1` (START-JUMP terhadap T1; garis tugas dari T1); run_g32.task_cmd meneruskan bila mode T1.
 f. reserve.py: keadaan T1 (reservasi = garis/keadaan terukur, cek bahwa tidak mengasumsikan REST).
 g. g31_screen DRY: menempatkan T1 untuk gantry mode T1. g47_home.sh: bila lengan di T1 → unfold di p0 sebelum PARK.
 Gerbang C5: DRY 2 seed lama (mode REST) argv tiap event = sebelum (diff kosong) + kandidat tanpa `mode` = REST.
FASE 4–6: sesuai §A C6–C9 + p1_g64c_prompt.md (mock 77: regresi 2 seed lama, TAS T1 plan-only + eksekusi oleh run_g54,
kontrol negatif, ukur fold/unfold k=3 + RNEA rencana lipat → G64C_C_FOLD → solve ulang C3; HW: lipat di tempat 1 lengan → ukur
j2 DAN j3 vs RNEA+offset → buka → dua lengan → traverse pendek T1 → TAS lewat run_g54; --tau-max 14, octomap_updaters:=false,
K0 ukur ulang). Lalu §B fase 3–6, state, memori, dampak paper (P1 lama = mode REST), DEMO-SYS ulang dengan g64c_demo.py.

ATURAN: = p1_g64c_prompt.md (penyaring tidak dilonggarkan, MARGIN = TOLAK, kotak 10 mm, fail loud, setsid nohup, jangan pkill -f,
jangan sentuh /home/kubota, commit per bagian hanya berkas sendiri, Rule 6 checkpoint). Bahasa Indonesia, singkat (caveman).
Keadaan sel (tidak disentuh sejak 11:50 10-17): semua lengan REST, papan DILEPAS, g1 1.40, g2 0, stack + kamera + mock MATI,
botol putih di M1, tas kertas di atas lemari sisi gantry 2.
```

Model: Opus, effort **high** (runner menyentuh jalur gerak HW; regresi diam-diam di argv mode REST dan torsi j3 di T1 tanpa cadangan).
