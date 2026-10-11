KAMPANYE P1 — FASE 6 (HW NYATA). Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus, effort HIGH.
Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari docs/p1_campaign_state.md.

BACA DULU (penuh): CLAUDE.md; MEMORY.md; docs/p1_campaign_prompt.md (aturan kampanye, bagian "CARA KERJA" + Fase 6);
docs/p1_campaign_state.md; docs/p1_hw_runbook.md (urutan + perintah persis); docs/p1_g48_hw_ready.md §B3 (risiko R1–R9,
DITERIMA operator 2026-10-10); docs/p1_g50_rehearsal11.md (17 jadwal siap HW, sweep v2).

KEADAAN: Gerbang 1–5 LULUS. 17 jadwal R180 siap HW (DRY-CLI 3/3 + smoke mock 6/6, tol tujuan 0.05°): s0 s3 s7 s10 s18
(results/p1_g43/g43_candidates_6.json R180), s2 s11 s14 s16 s17 (results/p1_g43/g43_candidates.json R180), s4 s5 s6 s9 s12 s13
(results/p1_g49/g49_candidates.json R180), s8 (results/p1_g42/g42f_candidates.json G42). ROT_NEED g1 −125…+90°, g2 −35…+95°.
Rotasi HW terverifikasi baru ±10° (results/p1_g48/rot_envelope.json = default alat). Origin g2 ≈ −35 mm belum dicek.
g1 PARKIR 1.50, g2 0.00. Mock MATI. Operator di lokasi.

TUGAS — Fase 6 sesuai runbook, dokumen fase docs/p1_g51_hw.md (§A + dugaan D430+ DIKUNCI sebelum menyentuh HW, salinan +
sha256; §B ditulis; commit per sub-fase; perbarui berkas keadaan sesudah tiap langkah berarti):
- 6a: remount_check → TANYA → bring-up nyata (SIG_DFL + setsid, PID launch asli, SigIgn, 7/7 controller, bridge ARMED,
  /table_hw_commands diam) → monitor + perekam → K0 (k0_check.py --rail1 1.50 --rail2 0.00) → kamera + capture g2 origin
  (TANPA gerak) → rail_calib --diff vs baseline rgbd tx −0.0936 m: |Δx| ≤ 10 mm OK, ≥ 25 mm slip (re-zero HANYA bersama
  operator), di antaranya pita ukur.
- 6b: sweep v2 results/p1_g50/g50_sweep.sh (g1 di lin 0.65, g2 di lin 0.15), SATU kaki per persetujuan, DRY dulu; cek tiap
  kaki (err ≤ 0.3°, t ≤ 1.2 T_cmd, drift ≤ 0.5°, torsi ≤ 14, operator cek kabel); sesudah amplitudo lulus →
  rot_envelope.py widen --evidence <log>. Catat t_rot/T_cmd_rot (model rot_cmd).
- 6c: run_g32 --dry di stack nyata untuk seed yang rotasinya sudah terverifikasi.
- 6d: blok seed HW (R_max vs R0 sama-sesi), plan-only NYATA k=3 dulu, run_g32 --final-retract --goal-tol-deg 0.05,
  pulang results/p1_g47/g47_home.sh (retrace), K0.

ATURAN KERAS: SETIAP gerak nyata ditanya dulu (AskUserQuestion, satu langkah per persetujuan, termasuk pemulihan K0).
Fault / LED merah / REJECTED / NOT armed / torsi > 14 / benturan / kabel → BERHENTI, lengan REST, tulis, tanya. JANGAN
pkill -f (ps|awk lalu kill PID). Jangan move_dual_table saat ARMED. MARGIN = TOLAK. Matikan stack bersih di akhir
(node list --no-daemon 0), arsipkan /tmp/p1hw_* ke results/p1_g51/hw/.
