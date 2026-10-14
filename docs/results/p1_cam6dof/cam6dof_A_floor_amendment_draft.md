# CAM-6DOF — draf amendemen penyangga lantai 2026-10-14

**DRAF, BELUM DIKUNCI; tidak mengizinkan capture botol lantai atau gerak.** Amendemen diminta operator karena A/B besar/sulit dipindahkan. Dokumen ini melengkapi, tidak menimpa protokol awal `cam6dof_A_locked.md` (commit d081044, sha256 `3d13ea22a89d87bb3ac789bda902a6f2f489f9594d40b166d9426a14a78dc802`). Semua ketentuan asli berlaku kecuali desain penyangga/tinggi lantai yang dirinci di bawah.

## Alasan, waktu, dan batas klaim

Permintaan operator sesudah data incumbent meja18/18 dan gerak penunjuk1–3, tetapi **sebelum data botol lantai apa pun dan sebelum fit/LOO/model-selection**. Data lama, baseline REST×3, peran tanda, hash awal, dan bukti gerak tetap. Hasil kelak dilabel protokol amendemen; tidak diklaim seluruh sesi mengikuti revisi sejak awal. Kepatuhan tinggi protokol asli z0/0.310/0.480 tidak lengkap. Tidak memulai ulang A0/G61/REST baseline/meja.

## Tinggi dan susunan

- Botol fisik tetap satu botol putih H70 mm, diameter60 mm, radius30 mm.
- C100 mm, atas95×70 mm; D140 mm, atas92×50 mm: terukur±2 mm. Data meja tetap T0/TC/TD=0.770/0.870/0.910 m, tanpa tumpukan.
- Tumpukan C+D **245 mm terukur±2 mm**, stabil/rata dengan botol. Nilai245 dipakai, bukan penjumlahan nominal240. Susunan operator: **C di bawah, D di atas**; pertahankan susunan terukur ini, tanpa tambahan benda di antaranya.
- Lantai: tiga tingkat **BASE** (botol langsung permukaan dasar tanda), **D** (box D140), **CD** (tumpukan245). C individual tidak menambah tingkat keempat.
- z world permukaan = z dasar tanda + tinggi penyangga. F3/F4/F6/F7 pada penyangga tetap yang sama **0.070 m±0.002**, kuat menahan C/D; BASE/D/CD=0.070/0.210/0.315 m. F1 sudah dikonfirmasi lantai, BASE/D/CD=0/0.140/0.245. F5 langsung lantai z0 dikonfirmasi; F3 memakai penyangga0.070 m yang sama denganF4/F6; F7 dikonfirmasi pada penyangga0.070 m yang sama. Semua permukaan baru harus diverifikasi sebelum kunci; bila bukan lantai, tinggi dasar diukur±2 mm.
- Posisi pusat alas botol tepat di atas xy tanda. Box/stack tidak digeser mengikuti deteksi kamera. D top lebih sempit5 mm per sisi dari diameter botol; hanya gunakan penempatan yang operator pastikan tegak/stabil. Jika tidak stabil/terhalang, missing dicatat, tidak mengubah posisi/peran atau memakai z palsu.
- Box kecil hanya masuk area penunjuk sesudah arm_3/4 kembali REST dan K0 lulus; setiap gerak termasuk pulang tetap memerlukan izin satu langkah.

| tanda | xy actual (m) | z dasar (m) | BASE / D / CD permukaan world (m) | role rgbd | role rgbd2 |
|---|---|---|---|---|---|
| F1 | 0.2512,−0.4998 | 0 | 0 /0.140 /0.245 | FIT | tidak dipakai |
| F6 | 0.3998,−0.6998 | 0.070 | 0.070 /0.210 /0.315 | FIT | tidak dipakai |
| F4 | 0.5491,−0.6003 | 0.070 | 0.070 /0.210 /0.315 | FIT | FIT dua tingkat terangkat D/CD |
| F5 | 0.6497,−0.4779 | 0 | 0 /0.140 /0.245 | FIT | FIT tiga tingkat |
| F3 | 0.2513,−0.5991 | 0.070 | 0.070 /0.210 /0.315 | UJI-TAHAN | tidak dipakai |
| F7 | 0.6483,−0.5809 | 0.070 | 0.070 /0.210 /0.315 | UJI-TAHAN | UJI-TAHAN + kesepakatan kamera |

Actualxy tanda tersisa diisi hanya dari TCP terukur penunjuk dan konfirmasi label, sebelum capture botol. Tidak mengubah nama/peran mengikuti galat deteksi. BASE F4 tidak ditambahkan keFIT rgbd2 dari pengamatan baru; pengecualian dua tingkat terangkat pada F4 tetap seperti protokol asli.

## Urutan dan penguncian

Penunjuk F1→F6→F4 selesai MOVED/SUCCESS; lanjut F5→F3→F7→REST sesuai urutan asli, per langkah plan-check aktual semua saringan +≤97%, izin operator, satu gerak, verifikasi dwell/TCP/torsi, tanda unting/label/tinggi dasar. Batas measured torque14 tetap. Semua plan-only tidak dianggap eksekusi.

Sebelum data lantai pertama: isi seluruh xy/z aktual dan susunan stack, proj_marks hidup untuk BASE/D/CD kedua kamera memakai tinggi baru (IN bukan jaminan deteksi atau aman), K0 sesudah pulangREST, finalkan amendemen ini sebagai salinan terkunci+sha256+commit dan catat commit di§B/state. Config capture/fit harus menyertakan identitas kedua kunci, tinggi baru dan actualxy; tidak memakai config LA/LB lama. Jangan memilih ulang model/threshold atau titik dari data uji-tahan.

Capture incumbent lantai setelah operator siap/menjauh,15 s kedua kamera, urutan tanda asli F1→F6→F4→F5→F3→F7, masing-masing BASE→D→CD. Capture yang tidak terlihat/missing tetap diungkapkan. Uji-tahan hidup kandidat kelak memakai tinggi amended yang sama; semua tingkat harus lulus. Tidak menghapus data buruk atau mengulang sampai mendapat angka lebih baik.

## Model, gerbang, dan dugaan tetap

Protokol asli A0/A3/A4 utuh: M6 utama; M7 hanya lewat LOO leave-one-mark seluruh tinggi **FIT saja**, perbaikan≥max(0.005 m,15% LOO M6). Tidak memakai holdout untuk fit/inisialisasi/model-choice. Fit polos berbobot, σbottle0.015, tiap arm6 obs1D σ0.03, point-to-plane |nz|≤0.5; tanpa robust loss/pembuangan data buruk. Konvergen, |w|≤5°, |τ|≤0.15 m, |s−1|≤0.06. M7 tidak bisa dipasang melalui TF statis; lapor M7, kandidat hidup tetapM6; skala depth memerlukan keputusan/change tersendiri.

C1: fit sah dan tiap tandaFIT track tunggal≥2 tinggi. C2: holdout HIDUP **semua tinggi**, xy≤0.05 m dan z−(permukaan+offset centroid)≤0.03 m. C3: tiap lengan baseline terlihat REST×3, medianxy≤0.05 m dan≤incumbent+0.01, visibility≥30%. C4: basis botol kedua kamera F7 setelah offsetcentroid, xyagreement≤0.05 m; mentah dilaporkan. C5: TFhidup cocokprediksi≤1 mm/0.01°. C6: tiap gerakMOVED+dwellSUCCESS, TCP≤5 mm,torsi≤14. MARGIN=TOLAK>97%, semua penyaring asli tetap. Defaultlaunch tidak diubah; penerimaan memakai harnessbaru, rgbd incumbent lulus tetapdipakai.

D590–D598 asli dipertahankan harfiah, D590 sudah✗ dari meja. D591–D598 dinilai pada set amendemen dengan perubahan desain diungkapkan, tidak diberi skor untuk bagian belum dijalankan. Jawaban “hanya rgbd2 salah” belum terbukti; incumbent rgbd baselineC3 arm_2=5.3 cm gagal batas5 cm, atribusi belum selesai. DataN1 FIT rgbd2 belum memenuhi C1 menurut checkpointB11/B13, tidak diperbaiki diam-diam lewat amendemen penyangga.

Rentang penyangga baru0.245 m lebih kecil dari0.480 m asli; informasi pemisahan pose/skala dapat berkurang. Keputusan model tetap oleh LOO FIT dan penerimaan ketat; tidak menjanjikan bahwa revisi akan lulus.

## Bukti persiapan sebelum penguncian

Enam actualxy dan alas sudah diisi dari penunjuk+operator, susunanC-bawah/D-atas245 mm dikonfirmasi. Proyeksi hidup `hw_20261014/amended_rgbd{,2}.log/.jpg`: rgbd semua18 titikpermukaan IN; rgbd2F4 BASE OUT(v729), D/CD IN(v708/690), F5 danF7 semuaIN. F1/F6/F3 OUT rgbd2 dan tidak dimasukkan ke perannya. IN hanya geometri bingkai, bukan bukti track tunggal atau galat lokalisasi. Gambar ditampilkan kepadaoperator.

Return khusus sesi `harness/return_rest_cam6dof.py` mempertahankan cosine30 s/60 langkah dan tiga saringan `return_rest.plan_retract`, tanpa fallback atau segmen otomatis; menambah validasi modeltorsi6sendi/finite, margin97%, cek map dalam planning scene, START-CHANGED dan pembatalan>14/effortlenganinvalid/timeout. Target monitorREST dibuat dariFK URDF hidup pada tujuan sendi dan rel/rot terukur; MOVED+dwellSUCCESS sertaK0 harus diverifikasi. DRY hidup dariF7 lulus92.587812% (`return_guard_plan.json`), tanpaeksekusi. Izin pulang masihbelum; REST/K0 belumdijalankan, sehingga draf ini belumdikunci.
