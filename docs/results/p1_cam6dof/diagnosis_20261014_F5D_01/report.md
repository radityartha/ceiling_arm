# Diagnosis pasif F5/D 2026-10-14 — mask, depth dan track lama

**Diagnosis selesai; belum memperbaiki atau menerima kalibrasi, DEMO-HW tetap belum siap.** Operator mengonfirmasi botol165×65 mm yang sama di atas boxD140 mm langsung lantai pada F5=(0.6497,−0.4779), tegak/stabil dan area/e-stop aman. Identitas objek pada citra kedua kamera dikonfirmasi lagi. Data baru diagnosis terpisah dari36 capture kampanye lama; tidak mengganti skor, selector, model, TF atau depthscale.

## Pelaksanaan dan bukti

Hanya kamera incumbent+G34 dan dua localizer terisolasi di domain79 yang diluncurkan. Domain0/79 preflight kosong, inventaris tidak menemukan task server/reach_fusion/controller/move_group. Tidak menghidupkan controller atau mengirim gerak arm/gantry. Run1 dua kamera15 s+warmup3 s selesai rc0:45 snapshot (RGBD24/RGBD2 21) berisi RGB, mask32SC1, depth32FC1, depthmentah16UC1, intrinsics, TF dan semuaevent/track; array disampling maksimal2Hz/kamera, deltaheader disimpan, tidak diklaim sinkronisasi eksak. KeduaTF tersedia pada seluruhsnapshot. LabelsString tidak memilikiheader; asosiasi label berdasarkanreceipt ditandai sebagai keterbatasan.

Run2 RGBD-only: kameraRGBD2 PID925045 ditutupSIGINT setelahUID/cmd/starttick diverifikasi, process sudahkeluar; kameraRGBD, detector danlocalizer tetap berjalan denganparameter yangsama. Botol tidakdipindahkan. Pembanding15 s+warmup3 s selesai rc0,25 snapshot RGBD dan **nol frame RGBD2**. Bukan A/B yang mengisolasi satu mekanismeIR/USB/CPU; kesimpulan hanya apakah mematikan kamera2 memulihkan observasiRGBD pada jendela ini.

- `capture/diagnosis.json`, `capture/events.jsonl`, snapshot NPZ dan JPEG: kondisi dua kamera.
- `capture_rgbd_only/diagnosis.json`, `events.jsonl`: kameraRGBD2 berhenti.
- `audit_dual.json`, `audit_rgbd_only.json`: auditvaliditasdepth danpublishertrack.
- `F5D_pixels.png`: plotRGB/mask/depth/worldz dari snapshot, bukan gambar sintetis.
- `bottle_support_split.json`: partisi pikselmanual sementara untukmenjelaskanpengaruhpenyangga.

## 1. Mask botol memasukkan penyangga — teramati dan identitas dikonfirmasi

Citra RGB/overlay memperlihatkan instance yang mencakup botol **dan boxD**, bukan hanya permukaan botol yang diasumsikan modelcentroid. Operator: “Ya, itu botolF5 danboxDyangsama”. RGBDmaskID3 bboxu417–447,v382–446; RGBD2maskID2 bboxu564–600,v623–719 (menjangkau batas bawahcitra). Kemunculan label “gray bottle” tidak membuktikan semua piksel instance berasal dari botol.

| Snapshot | Median z mask penuh (m) | Partisi bawah sementara | Median z partisi atas/bawah (m) |
|---|---:|---|---|
| RGBD000, mask3 | 0.109216 | v≥422:622/1498 titikvalid (41.5%) | 0.141000 /0.043577 |
| RGBD2 008, mask2 | 0.056985 | v≥683:943/2402 (39.3%) | 0.134155 /−0.032246 |

Kedua median maskpenuh berada **di bawah permukaanpenyangga0.140 m**. Partisi atas menaikkan z masing-masing3.18/7.72 cm, sehingga pikselbagianbawah memberi kontribusi nyata pada biascentroid. **Partisi manual bersifatdiagnostik**, bukan batasGT yangdikonfirmasioperator, perubahanfilter produksi atau bukti zbotol telahbenar; bagianatas masihbelum cocokcentroidsilinder. Tidak menghapus piksel/rival dari kampanye lama atau memasukkan hasilpartisi kefit.

## 2. Depth RGBD sering tidak valid; mematikanRGBD2 tidak memulihkan

Audit memilih instance terbesar hanyauntukstatistikdiagnosis; seluruhsnapshot dipertahankan. Validdepth sesuai rentanglocalizer0.1<d<12 m danfinite.

| Kondisi | Snapshot | Median fraksi depthvalid pada mask | Snapshot mask tanpa depthvalid | Median fraksi depthvalid seluruhcitra |
|---|---:|---:|---:|---:|
| RGBD, dua kamera |24|38.72%|10/24|17.65%|
| RGBD2, dua kamera |21|99.15%|0/21|78.09%|
| RGBD, kamera2mati |25|3.21%|12/25|13.71%|

RGBD track29pesan,16 kosong pada run1; run2 30pesan,18 kosong. **Tidak ada pemulihan setelahkamera2mati** pada15 s pembanding; interferensi antarkamera belumterbukti penyebab, sensor/exposure/emitter/USB/sinkronisasi masihperlu diisolasi. Tidak menyebut metricNaN sebagai0 atau memakai `count_nonzero` untukmenganggap seluruhpikselvalid.

Pada contohpasangan rawdepth/metric bertimestampidentik, rasio depthmetrik/raw **0.001** (p5/p50/p95), sesuai mm→m. Ini menolak dugaan kesalahanfaktor konversi pada pasangan yangdiperiksa, **bukan membuktikan skala fisiksensor, ekstrinsik atau semua kedalaman akurat**. Snapshot headerdelta disimpan; pasangan_RGBD000 RGB/depth/mask tepat sama, RGBD2 008 depth sama/RGB berbeda33.35 ms.

## 3. Localizer menyegarkan deteksi lama walau kamera sudah mati — bug terkonfirmasi

SetelahcameraRGBD2exit, run2 punya **nol RGB/depth/mask baru RGBD2**, tetapi `/loc_rgbd2/tracks` tetap menerbitkan **30/30 pesan nonempty selama14.50 s** (ID4,z≈0.064139 m). TTLdefault1.5 s telah jauhterlewati; bukan bukti30 pengukuran baru. Data/controlPIDs sertaevent tersimpan sehingga temuan dapatdireproduksi tanpa menafsir dari frekuensipublisher saja.

Kode `ros2_ws/src/reachability_gng/reachability_gng/object_localizer.py` menjelaskan sebab: `_on_pair` menyimpan `_dets[ns]` tanpa waktupengukuran; `_publish`(baris452+) memakai ulang `_dets` di setiaptimer; `_update_tracks`(baris386+) mengisi `last=now` dan menaikkan`hits` seolah deteksibaru. Kamera berhenti tidak mengosongkan cache. Karena itu TTL mengukurwaktu pemakaianulang cache, bukanumur pengukuran; `confirm_frames` juga menghitung sikluspublikasi. Temuan bersumber dari **FIT F5/D**, bukanmenyetelparameter memakaiholdout.

**Dampak:** jumlah n30 dalamrekamanlama hanya menghitung pesantrack, tidak memastikan30 pengukuran RGB-D berbeda. Tidak dapat menetapkan berapa bagian tiapcapturelama stale karenaRGB/mask/depth perframe tidakdisimpan semuanya; raw/rival/scorelama tetap utuh. Ini hambatan validitas pengukuran sebelumfit, bukan alasanmengubah kriteria penerimaan atau menyatakan TFkedua kamera pasti rusak.

## Keputusan, penutupan dan langkah berikut

Tidak mengubah kodeproduksi, prompts/conf/filter/centroid, argsTF/defaultlaunch/depthscale; tidak fitulang atau DEMO-HW. Kegagalan kalibrasi lama tetap, tetapi atribusi khusus ekstrinsik/depthscale semakin tidak dapat disimpulkan dari trackdirty/stale. **Prioritas berikut: perbaiki freshness deteksi/konfirmasi per pengukuran baru, verifikasi depthRGBD dan segmentasi botol tanpa penyangga pada FIT, baru kunci kampanye baru sebelum data kalibrasi baru.** Pengujianfix harus membuktikan kamera berhenti→trackkedaluwarsa1.5 s dan1frame tidak menjadi3confirmations lewat3timer, dengan kriteria lama tetap. Jangan melatih/menyetel dariF7/C3/C6holdout.

Shutdown hanyaPIDcamera/localizer sesi terverifikasi. Reviewizin otomatis pertama timeout sebelum perintahdijalankan; retrysekali berhasil. Kamera/nodeSDK exit, seg_router/localizer menunjukkan pengecualianfinalisasi `rcl_shutdown already called` sesudahSIGINT; dicatat, bukan menyebut semuaexitrc0. `shutdown_verified.json`: ownedactive0/zombie0, domain79/0 kosong. Tidak ada geraknyata, K0terukurbaru **tidak dijalankan** karena controller tetapmati; operator mengonfirmasiREST/PARK, K0terukur terakhir berasal dari penutupan kampanye.

Raw70NPZ≈188MiB tetaplokal dan diabaikan git, metadata/JPEG/plot/JSON/log sertahelperdiagnosis diarsip denganSHA (`manifest.json`). Anggaranper-task4000tokenCLAUDE.md terlampaui pada perekam/diagnosis/penutupan, diungkapkan dalamcheckpoint. Modelagen kuat efforthigh direkomendasikan untukfix validitas danHW; medium–high untukanalisis luring.
