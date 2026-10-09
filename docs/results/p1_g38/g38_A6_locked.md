### A6. 🔒 Addendum (G39b, 2026-10-10, SEBELUM plan-only) — plan-only MOCK seed yang lolos (i)–(iii)

Mock domain 77 (`use_fake_hardware:=true enable_gantry_bridge:=false`, env_map:=true), `g31_screen.py --variant R35
--repeats 1 --plan g38_candidates.json` untuk 19 seed, apa adanya (retract_check + 3 percobaan tugas, saring S18 / lintas /
torsi / lingkungan, urutan event g1 lalu g2 seperti runner). Atribusi gagal = verdict terakhir; INTERARM-COLLIDE dengan
pasangan t1_*↔t2_* = **lintas-gantry**. Satu sampel per seed (penyaringan, bukan laju).

| # | Dugaan |
|---|---|
| D295 | PLANNED ujung-ke-ujung **≤ 10 / 19** seed. |
| D296 | **≥ 30 %** seed yang gagal, gagal pada tabrakan lengan **lintas-gantry**. |
| D297 | **≥ 1** seed gagal pada ENV-* (tugas dinding). |
| D298 | Gagal akhir TORQUE-UNSAFE **≤ 2** seed. |

