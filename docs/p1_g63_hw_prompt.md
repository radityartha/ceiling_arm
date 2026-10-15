# Prompt pembuka sesi G63-HW (salin ke sesi baru)

```text
Lanjutkan G63 di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy): kalibrasi ekstrinsik kedua D455 dengan papan
ChArUco di gripper arm_2 (papan A) dan arm_3 (papan B), ground truth = FK lengan. Sesi ini = kunci §A lalu HW pilot P0.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan), docs/p1_campaign_state.md (baris G63),
docs/p1_g63_charuco.md (§A + A8.1–A8.14 — temuan alat, WAJIB), docs/results/p1_g63/g63_runbook.md.
Memori: /home/user1/.claude/projects/-home-user1-Documents-ceiling-arm/memory/MEMORY.md (g63-charuco-gripper-calib).

KEADAAN (2026-10-15 sore, commit 12db1ab): gripper arm_2/arm_3 DITUTUP (1.000 / 0.962), papan terpasang (kardus, double tape,
menutup kedua jari; arm_2 → rgbd2, arm_3 → rgbd; kotak TERUKUR 48.43 mm, 43.5 g) → results/p1_g63/mount.json. Lengan REST,
g1 masih 1.50, g2 0. Stack/kamera/mock MATI. Alat luring + mock SELESAI: solver, detektor, URDF payload + penyaring berpapan,
probe_g63, g63_capture, g63_plancheck; plan-check mock dudukan nyata g1 1.40: arm_2 26/27, arm_3 26/27 (poses_real_v1.json).

JEBAKAN (dari A8): papan A di REST g1 1.50 hanya 43 mm dari meja → G63 pakai g1 1.40; pose_to_g dan return_rest TIDAK kenal
papan → lepas papan A sebelum geser rel, pulang REST lewat probe_g63 (bukan return_rest) selama papan terpasang; --target=
(bentuk '='); gripper TIDAK boleh diperintah selama G63.

LANGKAH: (1) minta persetujuan operator G4′ (PnP satu kamera vs FK median ≤ 2 cm) dan G6′ (residu lateral R0 ≤ 2 mm),
konfirmasi D435IF dilepas (S0e) → kunci §A (salinan results/p1_g63/g63_A_locked.md + sha256 + commit, berkas sendiri saja).
(2) HW, SATU izin per langkah: operator lepas papan A → pose_to_g g1 1.50→1.40 (lengan REST) → tempel A lagi → bring-up
(octomap_updaters:=false, enable_gantry_bridge:=true) + K0 terukur rail1 1.40 + monitor/perekam + kamera incumbent + rec_health
terus-menerus → URDF payload dari robot_description HIDUP + g63_attach → plan-only lalu --move pose P0 (R0, 3 pose, R0) per lengan
→ g63_capture tiap pose → merge + solve + G6′ → lapor sebelum tahap berikut.
Fault / LED merah / REJECTED / torsi > 14 / benturan / gerak tak diminta → berhenti, amankan, tulis, tanya.
Margin = tolak; penyaring tidak dilonggarkan. Fail loud. Jangan sentuh /home/kubota. Bahasa Indonesia, jawaban singkat.
Akhir sesi: lepas papan → REST → g1 PARKIR 1.50 → K0 → shutdown PID milik sesi; rekomendasi model + effort.
```

Model: kuat (Opus), effort **high** (gerak HW, galat dudukan/torsi/tabrakan bisa diam).
