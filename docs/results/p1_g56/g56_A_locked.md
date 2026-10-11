## A. Protokol — DIKUNCI

### A0. Pengungkapan + analisis P2 (dari kode, sebelum uji)

- **Bridge gantry** (`moving_table_pkg/dual_table_controller.py` `_on_hw_command`): setpoint JTC di-debounce (1 mm / 0.3°) dan
  diteruskan sebagai `go_to_absolute` ke motor AZ yang meramp SENDIRI; target baru hanya dikirim sesudah gerak sebelumnya
  SELESAI. `stop_event` hanya diset oleh layanan `move_dual_table` (op 0 STOP), BUKAN oleh cancel JTC.
  → **Cancel di tengah TRAVERSE di HW tidak menghentikan rel seketika**: kaki yang sudah terkirim diselesaikan, lalu bridge
  mengejar posisi tahan JTC (posisi saat cancel) — bisa MUNDUR sampai satu kaki. Semua titik itu ada di dalam persegi lin × rot
  traverse yang sudah disaring S28/G33, dan gantry lain DIAM (traverse eksklusif) → aman, tetapi "berhenti ≤ 0.5 s" (G54 B4,
  mock) TIDAK berlaku untuk traverse di HW. `move_dual_table` STOP tidak dipakai (memori: JTC yang menahan posisi lama menyeret
  gantry kembali).
- **Lengan Kortex**: cancel JTC → JTC menahan posisi terukur; driver mengalirkan posisi → berhenti cepat (deselerasi).
  Belum terukur di HW → uji stop HW memukul saat **retract lengan**, bukan traverse.
- Injeksi G54 berbasis URUTAN event bergantung linimasa mock (percobaan 1 tidak sah). Untuk HW: injeksi berbasis KONDISI.
- Penjaga G54 membaca SELURUH launch log 10 Hz; di HW log bisa besar → kini inkremental (biner, offset).
- s11 R90 butuh g2 −85; amplop HW g2 −35…+95. EdgeMatrix (env, 50 mm) g2 lin 0.15: 0 → −50/−70/−85/−90 CLEAR, min 301.5 /
  289.4 / 271.6 / 270.8 mm (dihitung sebelum §A).

### A1. 🔒 Perubahan

1. `run_g54 --inject-stop-on-motion G`: AUTO-STOP saat gantry G menjalankan RETRACT yang lengannya sudah bergerak > 2° dari
   awal event dan masih > 10° dari REST.
2. Pemeriksaan fault launch log inkremental.
3. Sweep v3 `results/p1_g56/g56_sweep.sh` + `g56_sweep_legs.json` (g2: (0.15, 0) → −50 → 0 → −70 → 0 → −90 → 0 → (0, 0)) +
   `g56_sweep_plan.json` (salinan G50 + stasiun negatif).

### A2. 🔒 Uji mock (domain 77, monitor + perekam, REST + PARK, K0)

1. **Injeksi gerak:** s7 R180 `--inject-stop-on-motion 2` → rc 3; `inject.moved_deg` > 2, `left_deg` > 10; posisi lengan g2
   konstan (perekam) dari t_stop + 1 s; tidak ada alat diluncurkan sesudah stop.
2. **Kontrol positif fault:** saat smoke berjalan, tambahkan satu baris `[ros2_control_node-1] [ERROR] [ros2_control_node]: G56
   uji` ke launch log mock → AUTO-STOP "launch log: 1 baris fault" ≤ 1 s; rc 3.
3. **Sweep v3:** 8/8 kaki rc 0 di mock (DRY tiap kaki dulu), G33/S28 CLEAR.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D480 | Injeksi gerak lulus pada percobaan pertama (sah: retract g2 sedang bergerak). |
| D481 | Kontrol positif fault: AUTO-STOP ≤ 1 s sesudah baris ditambahkan. |
| D482 | Sweep v3 8/8 rc 0, sapuan G33 min ≥ 200 mm. |

