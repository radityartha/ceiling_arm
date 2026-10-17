### A4. 🔒 Amandemen TAS (operator 10:5x, sesudah jalan 1 M1; dikunci SEBELUM data tas)

Operator: "coba cari tas" — tinggi "cek pakai kamera saja lalu tentukan area aman di atasnya", posisi "bebas di meja zona".
Di luar A1–A3 (botol putih 7 cm) → aturan tambahan, A1–A3 tidak diubah.

| # | Aturan |
|---|---|
| T0 | Prompt YOLOE diganti live `ros2 topic pub -1 /seg_prompts std_msgs/String "data: bag"` (tanpa relaunch). Botol diangkat dari meja. Perintah = POST "please bring a bag". |
| T1 | **Tinggi dari kamera.** Bukti botol (§B): z track mentah ≈ PUNCAK benda (0.834 vs puncak 0.84; kamera atap melihat permukaan atas). Rekam 15 s `/loc_rgbd2/tracks` → z_top = median z track tas. **Tolak gerak bila z_top > 1.07** (tas > 0.30 m) atau < 0.79. |
| T2 | Node zona dinyalakan ulang `-p object_height:=2·(z_top − 0.77)` → z_out = z_top → **p_cmd z = z_top + 0.25** (ujung tool 0.25 m di atas permukaan atas terukur; sisa ketidakpastian puncak ≤ ~3 cm → ≥ 0.22 m). Gerbang node (kotak track, zona, \|z_trk − z_out\| ≤ 0.05) TIDAK diubah. Tas tidak di peta env → pemisah = stand-off saja. |
| T3 | Gerbang Zb (15 s): tepat satu track tas diterima di `/demo_zone/tracks`, hadir ≥ 90 %; xy terkoreksi di zona (gerbang node). Galat xy **tidak terukur** (tanpa tanda; P3 di-fit botol putih → bias bahan tas tak diketahui) = info. |
| T4 | Plan-only dulu (H3), lalu gerak per izin (H4, kriteria sama: torsi ≤ 14, dll.). |

| # | Dugaan |
|---|---|
| D664 | YOLOE `bag` mendeteksi tas di zona sebagai satu track stabil (conf ≥ 0.3), tanpa track `bag` lain diterima. |
| D665 | z_top tas ∈ [0.85, 1.00]. |
| D666 | Plan-only tas: PLANNED pada `g` pertama; jalan bergerak MOVED → REST, torsi terukur ≤ 12 N·m. |

