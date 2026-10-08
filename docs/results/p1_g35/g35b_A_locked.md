### B7. G35b — HW R0 + R10 seed 36 sama-sesi, penyaring dipercepat (2026-10-08)

> Prompt = §D. Gerbang (operator, sebelum apa pun): **K0 sel sama seperti akhir G34b — ya**; **K1 operator di e-stop — ya**;
> **K2 seed tambahan — tidak** (hanya 36). Data: `docs/results/p1_g35/hw/`. **§B7.A DIKUNCI sebelum bring-up**
> (salinan `docs/results/p1_g35/g35b_A_locked.md` + sha256 di bawah); §B7.1+ diisi sesudah.

#### B7.A 🔒 Protokol + dugaan (SEBELUM bring-up)

Urutan = G34b B7 apa adanya (alat tidak diubah; hanya `env_collision.py` G35): self-test → remount_check → bring-up nyata →
plan-only NYATA k=3 (R10, R0) → `run_g32` DRY → R0 → pulang → R10 → pulang → matikan. Fase per event =
`hw_replay.phases()` pada log baru (traverse: S28 → G33; tugas: antar-lengan → lingkungan, termasuk ~1 s tunggu sendi;
retract: `retract g` → `rencana pulang` mencakup S18 + lintas-gantry + lingkungan, dilaporkan terpisah bila log memisah).

| # | Dugaan |
|---|---|
| D232 | Makespan R0 dalam **667.7–678.2 s**, R10 dalam **656.2–673.8 s** (B4). |
| D233 | Δ(R10 − R0) **negatif** (B4: −4.5 … −11.5 s). |
| D234 | Fase lingkungan HW per traverse berotasi (ev 7, ev 12 R10) **≤ 3 s**; per traverse rel-saja ≤ 2 s. |
| D235 | Fase lingkungan HW per tugas **≤ 3 s** (12/12 tugas). |
| D236 | Verdict tiap event = G34b: R0 6/6, R10 6/6 sukses, 14/14 rc 0 per varian, nol auto-stop, semua saring lingkungan CLEAR, ev 8 R10 SUCCESS. |
| D237 | Plan-only NYATA R10 3/3, R0 3/3; semua retract tak-kosong lurus CLEAR (MoveIt tidak terpicu), juga di eksekusi. |
| D238 | Muat EnvChecker di HW dari cache ≤ 0.5 s per proses (tidak ada bangun ulang 4 s; cek di log self-test/alat). |
