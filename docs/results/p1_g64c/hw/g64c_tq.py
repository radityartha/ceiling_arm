"""G64-C fase 5 (C9): torsi terukur (effort /joint_states, js_record CSV) per sendi vs RNEA statis + offset. BACA SAJA.
  python3 g64c_tq.py <csv> <t0> <t1> <t_hold_end> <prefix> [<prefix> ...]
gerak = [t0, t1] (perintah -> selesai), tahan = [t1 + 2 s, t_hold_end] (2 s pertama = ekor servo, dibuang).
Prediksi: g64c_fold_rnea.json (puncak garis lipat DI T1, sama keempat lengan): j2 3.815 (+7.7 = 11.52), j3 2.555 (+6.6 = 9.16).
Batas: j3 > 10 atau sendi apa pun > 14 -> BERHENTI (prompt fase 5)."""
import csv, json, sys
csvp, t0, t1, th = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
pre = sys.argv[5:]
RATING = {1: 10, 2: 14, 3: 10, 4: 7, 5: 7, 6: 7}
PRED = {2: (3.815, 7.7), 3: (2.555, 6.6)}
mv, hd, n = {}, {}, 0
with open(csvp) as f:
    for r in csv.DictReader(f):
        try:
            t, e = float(r['t']), r['eff']
        except (ValueError, TypeError):
            continue
        if e == '' or not any(r['joint'].startswith(p) for p in pre):
            continue
        e = abs(float(e))
        if t0 <= t <= t1:
            mv.setdefault(r['joint'], []).append((e, t)); n += 1
        elif t1 + 2.0 <= t <= th:
            hd.setdefault(r['joint'], []).append(e)
out, stop = {}, []
for p in pre:
    for j in range(1, 7):
        nm = f'{p}joint_{j}'
        m, h = mv.get(nm, []), hd.get(nm, [])
        if not m or not h:
            stop.append(f'{nm}: sampel kosong (gerak {len(m)}, tahan {len(h)})'); continue
        pk, tpk = max(m)
        hm, hx = sum(h) / len(h), max(h)
        out[nm] = dict(move_peak=round(pk, 3), move_peak_t=round(tpk - t0, 2), hold_mean=round(hm, 3),
                       hold_max=round(hx, 3), n_move=len(m), n_hold=len(h))
        line = f'{nm}: gerak puncak {pk:6.3f} @ +{tpk - t0:5.1f} s | tahan rata {hm:6.3f} maks {hx:6.3f} | batas {RATING[j]}'
        if j in PRED:
            raw, off = PRED[j]
            line += (f' | RNEA {raw} (+{off} = {raw + off:.2f}): puncak-RNEA {pk - raw:+.3f}, tahan-RNEA {hm - raw:+.3f},'
                     f' puncak-(RNEA+off) {pk - raw - off:+.3f}')
            out[nm].update(rnea=raw, off=off, d_peak=round(pk - raw, 3), d_hold=round(hm - raw, 3),
                           d_peak_off=round(pk - raw - off, 3))
        print(line)
        if j == 3 and max(pk, hx) > 10:
            stop.append(f'{nm} j3 {max(pk, hx):.3f} > 10')
        if max(pk, hx) > 14:
            stop.append(f'{nm} {max(pk, hx):.3f} > 14')
print(json.dumps(out))
print('🔴 BERHENTI: ' + '; '.join(stop) if stop else f'TORSI OK ({n} sampel gerak): j3 <= 10, semua <= 14')
sys.exit(1 if stop else 0)
