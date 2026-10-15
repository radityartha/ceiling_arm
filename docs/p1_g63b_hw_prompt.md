# Prompt pembuka sesi G63b HW (salin ke sesi baru)

```text
Lanjutkan G63b di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy) sebagai sesi HW: kalibrasi ekstrinsik kedua D455
dengan rakitan dua papan catur polos di badan gripper (fase arm_2 lalu arm_3). §A DIKUNCI 2026-10-15
(docs/results/p1_g63/g63b/g63b_A_locked.md + .sha256 — cek sha dulu; beda = STOP).

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan), docs/p1_campaign_state.md (baris G63b terakhir),
docs/p1_g63b_redesign.md (§A + A8.9–A8.12), docs/results/p1_g63/g63b/g63b_runbook.md (IKUTI langkah demi langkah).
Memori: g63-charuco-gripper-calib.

KEADAAN AWAL: rakitan dua papan TERPASANG di arm_2 (A -> rgbd2, B -> rgbd di REST; 87 g; 70 mm; tepi atas 78 mm di atas
pangkal jari), g1 1.50 PARK, g2 0, semua REST, stack/kamera MATI. ⚠ REST arm_2 + rakitan di g1 1.50/1.40 menabrak kotak env
MoveIt → JANGAN gerak apa pun sebelum rakitan dilepas (runbook S1.3), rel ke 1.30 TANPA rakitan, lalu pasang lagi.

URUTAN (runbook): S0 preflight + [OPERATOR] uji goyang (≤ 1–2 mm; belum pernah dijawab) + papan lama di luar bingkai →
S1 bring-up (octomap_updaters:=false, enable_gantry_bridge:=true, reach_dwell_monitor WAJIB) + K0 + jari arm_2 ukur ulang →
lepas rakitan → pose_to_g g1 1.30 (g63b_rail_plan.json, varian G63b) → pasang → payload dari URDF hidup + g63b_attach →
S2 arm_2 P0 (plan-only berantai dulu; g63b_move --dry lalu satu izin per gerak; g63b_capture --save-frames) → laporan P0
(P0 = cek PROSES; solve P0 DIHARAPKAN ditolak penjaga G10) → T1… hanya bila lulus → S3 pindah ke arm_3 (attach --remove harus
"sisa [] -> OK"; B-02 ditolak di mock → lewati) → S4–S6.

ATURAN: satu izin operator per gerak nyata (termasuk REST/rel/pemulihan); kamera = izin; margin = tolak; penyaring tidak
dilonggarkan; JANGAN scripts/return_rest.py selama rakitan terpasang; torsi > 14 / fault / REJECTED / GOAL-MISMATCH → stop,
amankan, tanya; jangan pkill -f; job panjang setsid nohup; jangan sentuh /home/kubota; frame kamera (ada orang) TIDAK di-commit;
commit per bagian hanya berkas sendiri. Bahasa Indonesia, singkat. Akhir: REST, lepas rakitan, g1 1.50 PARK, K0, shutdown
bersih, laporan + papan skor D639–D645 (+ dibawa D632–D635, D638) + rekomendasi model/effort.
```

Model: Opus, effort **high** (gerak HW; galat dudukan/torsi bisa diam).
