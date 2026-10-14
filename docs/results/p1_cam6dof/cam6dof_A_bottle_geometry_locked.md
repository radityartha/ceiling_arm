# CAM-6DOF — koreksi ukuran botol DIKUNCI 2026-10-14

## Koreksi operator dan batas waktu

Operator melaporkan tinggi botol ternyata165 mm dan diameter65 mm; botol fisik tetap sama pada seluruh18 rekaman meja C3/C6/M1/C5/N1/N2 dan F1. Pernyataan “salah selama ini” memperbaiki ground truth, tidak mengganti objek atau mengestimasi ukurannya dari residu FIT/UJI-TAHAN. H0.070/r0.030 pada versi awal salah untuk botol sesi ini.

**Ukuran operasional: H=0.165 m, diameter=0.065 m, r=0.0325 m.** H adalah tinggi total alas→puncak sesuai pertanyaan operator; nilainya tidak dikoreksi lagi dalam jawaban berikut. Ketelitian numerik±2 mm untuk botol tidak dijawab eksplisit dan tidak dicatat sebagai konfirmasi; angka165/65 adalah laporan operator. Ketelitian±2 mm penyangga tetap seperti pengukuran sebelumnya. Botol fisik tidak berubah sehingga konfirmasi penempatan stabil sebelumnya masih berkaitan dengan objek yang sama; diameter65 membuat overhang D-top50 sebesar7.5 mm per sisi, harus tetap tegak/stabil saat penempatan.

Koreksi ini dikunci **sesudah18 capture meja dan satu F1BASEmissing, sebelum capture berikut/fit/LOO/model-selection**. Seluruh tujuh gerak penunjuk/pulang dan K0 sudah selesai; tidak mengklaim kunci baru mendahului semua data/gerak. NOL gerak atau pengulangan data untuk koreksi ini.

## Identitas protokol yang diwarisi

- Awal `cam6dof_A_locked.md`, commitd081044, sha256 `3d13ea22a89d87bb3ac789bda902a6f2f489f9594d40b166d9426a14a78dc802`.
- Penyangga `cam6dof_A_floor_amendment_locked.md`, commit3b507b9, sha256 `2022dbbc72f93e953daefa5d4d197a7c09aa0502214f1a280ed350c2e7ddff80`.
- Kedua dokumen dan hash lama tidak diubah. **Dokumen ini menggantikan hanya H/r botol pada kedua versi terdahulu.** Semua ketentuan lain tetap. Config capture/fit memuat tiga `protocol_hashes` dan bottle{r0.0325,H0.165}.

## Data lama dan laporan terkoreksi

Semua raw JSONL, RGB/overlay, metadata waktu, median/id/label/saingan, actualxy/tinggi penyangga dan hasil gerak tetap. Tidak mengambil frame baru, menggabungkan ID, memilih track alternatif, mengganti FIT/UJI-TAHAN atau menghapus titik buruk. Selektor labelcontainsbottle/radius0.35 dan rankingtrackterbanyak tetap. Sebelum menulis laporan baru, median/id/n diverifikasi identik dengan laporan lama dan rawfile diberiSHA256.

Angka centroid/z_expected/dz lama berdasarkanH70/r30 **bukan galat sah terhadap botol aktual165/65**. Dihitung ulang dari modelcentroidsilinder yang sama dengan inputH/r baru, ditulis terpisah di `hw_20261014/tracks_inc_corrected_bottle_geometry/`; raw/summarylama tidak ditimpa. Offsetxy model untuk basisC4/targetfit ikut diperbarui; **galatxy mentah** track terhadapmark tidak tergantungH/r dan tidak berubah.

Rekaman F1BASEmissing tetapmissing, tidak diisi dari diagnosis yang lebih baru. N1T0missing dan tigaIDsekuensial N1TD tetap diungkapkan; koreksiH tidak menyatakan masalah C1 selesai. D590✗ (perbandinganxymentah) dan D597✓ (rencana/torsi) tetap. D591–D598 yang memerlukanfit/kandidat tetap belum diuji. Interpretasi “z benar” dari metadata lama tidak dipakai untuk menyimpulkan tilt/skala sesi ini. Data sesi lama di luar CAM-6DOF tidak direkalkulasi tanpa konfirmasi objek/provenance tersendiri.

## Model, gerbang dan penempatan tetap

M6 utama; M7 hanya melalui LOOleave-one-mark semua tinggi **FIT saja**, improvement≥max(0.005 m,15% LOO M6), holdout tidak dipakai fit/init/modelchoice. FitplainLS, σbottle0.015, arm6 obs1D σ0.03, point-to-plane|nz|≤0.5, tidak memakairobustloss/buangbadpoint. Gerbangkonvergen, |w|≤5°,|τ|≤0.15 m,|s−1|≤0.06 tetap. TFstatis tidak menerapkan M7scale; bilaM7terpilih dilaporkan dan M6 tetap kandidat hidup jika sah. Defaultlaunch tidak diubah.

C1 fit sah/tiapFIT tracktunggal≥2 tinggi. C2 holdoutHIDUPsemuatinggi xy≤5 cm,zterhadappermukaan+centroid≤3 cm. C3 REST×3 setiaparmbaselinevisible xy≤5 cm dan tidak lebihburuk>1 cm,visibility≥30%. C4 basisF7kedua kamera setelahcentroidoffset≤5 cmxy. C5 TFhidup cocokprediksi≤1 mm/0.01°. C6 setiapgerakMOVED+dwellSUCCESS,TCP≤5 mm,torsi≤14. MARGIN=TOLAK>97%, semua penyaring tetap.

Meja T0/TC/TD z0.770/0.870/0.910 tetap; lantaiBASE/D140/CD245(Cbawah,Datas), F1/F5z0/0.140/0.245; F3/F4/F6/F7z0.070/0.210/0.315. Actualxy dan peran persis amendemenpenyangga; F4RGBD2 hanyaD/CD,F3holdoutRGBD,F7holdoutkeduakamera. Tinggi botol tidak ditambahkan ke `surf_z`; model menghitung offsetcentroid denganH165.

Botol masihF1; laci telahdipindahkanoperator sehingga sebelumHWbaru peta lingkunganbeku perlu diperiksa/update. Saatpenguncian semuaREST/PARK,K0terakhirLULUS,stack/localizer/monitor/perekam hidup. Capture berikutF1/D hanya setelah operator menempatkanbotol dan menjauh; tidak mengizinkan gerak robotbaru.

## Reproduksi dan bukti

`cam6dof_fit.py` sudah membaca r/H dari config, tidak diubah/diulangA0. Helper `harness/capture_bottle_amended_inc.py` membaca r/H dari config (menghapus konstanta70/30 khususringkasan), memverifikasi ketigahash dan pending-state capture_permitted sebelum membuka perekam. Tidak mengubah detector/selector/radius/centroidformula/bobot. Snapshot config sebelumkoreksi diarsip terpisah. Data asli dan hasilcorrected harus disebut dengan identitasinput masing-masing.
