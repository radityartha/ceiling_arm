# Cek kesehatan depth D455 hidup pasif 2026-10-15 — kedipan rgbd TIDAK tereproduksi

**Kamera saja, domain 80, NOL gerak/controller/move_group/localizer.** Izin operator 2026-10-15 ("oke jalankan") untuk
langkah 1–6 plan kesehatan depth. RGB tidak disimpan (hanya angka kecerahan). Tidak mengubah opsi perangkat, firmware,
launch, TF, atau kode produksi. Lanjutan [diagnosis luring B64](../diagnosis_20261015_offline/report.md).

## Pra-cek
- Domain 0/77/79/80 kosong; tidak ada proses kamera/launch milik siapa pun (termasuk user lain/root; Docker tidak terpasang).
- `rs-enumerate-devices`: rgbd **D455 241122302297 FW 5.16.0.1**, rgbd2 **D455 234222303079 FW 5.17.0.10** — firmware
  berbeda. Perangkat ketiga **D435IF 327122073095 FW 5.15.0.2** terpasang di rantai hub lain (tidak ada di dokumen; tidak
  dipakai launch, tidak disentuh; terakhir dipakai akun ini 2026-10-11 menurut `~/.ros/log`).
- USB: rgbd lewat dua hub 1-port berantai (bus 2 port 2 → 2-2.1.1), rgbd2 langsung (2-1); keduanya 5000M.
- `rs-enumerate-devices -o` hanya mencetak rentang/default (bukan nilai saat ini) — tidak informatif
  ([rs_options_before.txt](rs_options_before.txt)).
- Jaringan sel: hanya 4 lengan (192.168.2.10–13); tidak ada LiDAR/perangkat IR lain terlihat.

## Kondisi dan hasil (30 s per kondisi, depth 848×480×30, warna 1280×720×30 = parameter produksi)

| Kondisi | Topik | rgbd valid median (p10–p90) | rgbd2 valid | frame <10 % valid |
|---|---|---|---|---|
| A rgbd saja, node langsung | depth/image_rect_raw | **0.697** (0.696–0.698), 895 fr | — | 0 |
| A2 sama | aligned_depth_to_color | **0.698** (0.697–0.699), 871 fr | — | 0 |
| B dua node kamera | aligned | **0.700**, 877 fr | 0.781, 858 fr | 0 / 0 |
| C harness incumbent `rgbd_perception` + YOLOE `seg_imgsz:=1280` (GPU 90 %), = setup B61 | aligned | **0.702**, 895 fr | 0.780, 849 fr | 0 / 0 |

Metadata setiap frame: `frame_emitter_mode` 1, `frame_laser_power` 150, `actual_exposure` 31979, AE aktif. Parameter
driver rgbd: emitter_enabled 1, emitter_on_off false, laser_power 150, visual_preset 0, thermal_compensation true,
inter_cam_sync_mode 0, hdr false ([paramsA_rgbd.txt](paramsA_rgbd.txt)). Frame counter skip ≤ 23/30 s.

**Kesimpulan:** sensor rgbd sehat dengan konfigurasi produksi pagi ini (06:30–06:40 JST). Pola B61 kemarin (20:38–20:45 JST:
70 % ↔ 0–1 % ↔ hanya-tekstur, rawdepth 99 % nol pada frame mati) **tidak tereproduksi**, termasuk di bawah beban stack
penuh yang sama. Konfigurasi emitter bukan penyebab (default benar, metadata konstan).

## Yang masih terbuka (ditulis, tidak disimpulkan)
- Penyebab kemarin belum diketahui. Kecerahan RGB median sekarang ≈ kemarin (rgbd 89.5 vs 87.3, rgbd2 102.4 vs 105.2),
  jadi pencahayaan tampak tidak jauh berbeda; IR sekitar tidak terukur. Kandidat sisa: sumber IR luar sementara (orang/
  perangkat di ruangan saat operator bekerja), masalah intermiten kabel/hub rgbd, atau kondisi termal sesudah sesi
  panjang (kamera kemarin dinyalakan ulang 20:34, capture 20:38 — sesi HW sejak ~12:00).
- rgbd mencatat `Asic Temperature value is not valid!` (7× kemarin di jendela 20:41–20:44, 1× hari ini); rgbd2 tidak
  pernah. Waktunya berpasangan ~108 s dan tidak bertepatan dengan jendela capture → indikator sensor suhu/firmware rgbd
  yang tidak stabil, **bukan** bukti penyebab kedipan.
- Data 36 penempatan kampanye lama hanya track JSONL; tidak dapat dipastikan berapa capture yang terkena kedipan.

## Rekomendasi
1. Pada sesi kamera berikut, jalankan perekam validitas+metadata ([rec_health.py](rec_health.py)) **terus-menerus** di
   samping pekerjaan lain, supaya kedipan tertangkap dengan cap waktu (murah: hanya angka per frame).
2. Di lokasi: lihat lensa/jendela emitter rgbd, catat perangkat IR yang menyala di ruangan, dan uji rgbd tanpa rantai hub.
3. Firmware rgbd 5.16 vs rgbd2 5.17: update hanya atas keputusan operator (tidak dilakukan).
4. Tetap berlaku dari B64: botol+depth tidak layak sebagai alat ukur kalibrasi (bias +8 cm terjadi juga di rgbd2 yang
   sehat) → target RGB-saja.

## Berkas
`cam_up.sh`, `rgbd_params.yaml`, `rgbd2_params.yaml`, `rec_health.py`, `summarize.py`, `rgb_brightness.py`;
`rec{A,A2,B,C}_*.jsonl` + `.summary.txt`; `cam*.log`; `paramsA_rgbd.txt`; `rgb_brightness_*`; `owned_C.txt`,
`shutdown.txt`.

## Update firmware rgbd 2026-10-15 06:43–06:47 (izin operator "oke update, samakan saja")
- Firmware resmi `D4XX_FW_Image-5.17.0.10.bin` dari `librealsense.intel.com/Releases/RS4xx/FW/` (sha256 `a7e10cf7…60f1c`,
  1 573 660 B; bertanda tangan, diverifikasi kamera). librealsense ROS 2.58.2.
- Backup flash rgbd 5.16.0.1 sebelum flash: `/home/user1/realsense_fw_backup/rgbd_241122302297_fw_5.16.0.1_backup.bin`
  (2 MiB, sha256 `bd740fbb…0296d`; di luar git). rgbd2 dan D435IF tidak disentuh.
- `rs-fw-update -s 241122302297` rc 0: "successfully updated to FW: 5.17.0.10"; `-l` sesudahnya: kedua D455 5.17.0.10.
- Verifikasi 15 s (kondisi D, rgbd saja aligned): valid **0.700**, 448 frame, 0 frame < 10 %; emitter 1/laser 150;
  `Asic Temperature value is not valid!` **0×** (sebelumnya muncul di 5.16).
- Intrinsik warna sesudah update: fx **643.544** (sebelumnya 643.194), fy **642.759** (642.409), cx/cy identik
  (647.053/365.949) → +0.35 px (0.05 %, ≈ 1–2 mm di 3 m). Dicatat; kalibrasi baru G63 memakai camera_info hidup.
- Shutdown: PID kamera keluar, domain 80 kosong, 0 proses kamera.
