import re, csv, collections
W = [(int(a), float(b), float(c), float(d)) for a, b, c, d in (l.split() for l in open('/tmp/g18_windows.txt'))]
ex = []
for l in open('/tmp/g18_t3.log'):
    m = re.search(r'\[(\d+\.\d+)\] \[moveit_ros.trajectory_execution_manager\]: (Starting trajectory execution|Completed trajectory execution)', l)
    if m: ex.append((float(m.group(1)), m.group(2)[0]))
S = collections.defaultdict(list)
for r in csv.DictReader(open('/tmp/g18_step3_samples.csv')): S[r['arm']].append((float(r['t']), float(r['pos_err_mm'])))
REC = list(csv.DictReader(open('/tmp/g18_js_pair1.csv')))
print('--- TUGAS 3: arm_1 selama arm_2 MENGEKSEKUSI ---')
d55 = []
for i, t0, t1, t2 in W:
    e = [x for x in ex if t1 <= x[0] <= t2]
    st = [x[0] for x in e if x[1] == 'S']; co = [x[0] for x in e if x[1] == 'C']
    log = open(f'/tmp/g18_trial{i}.log').read()
    preds = re.findall(r'torsi RNEA\+offset\(per-aktuator\) vs rating: 1=([\d.]+)->[\d.]+/\d+, 2=([\d.]+)->[\d.]+/14, 3=([\d.]+)', log)
    ia = len(re.findall('INTERARM-COLLIDE', log)); dmin = [float(x) for x in re.findall(r'jarak antar-lengan minimum sepanjang rencana: ([\d.]+) mm', log)]
    line = f'pair {i:2d}: interarm COLLIDE={ia} min {min(dmin):.1f} mm'
    if len(st) >= 2 and len(co) >= 2:
        a0, a1 = st[1], co[1]
        w = [p for t, p in S['arm_1'] if a0 <= t <= a1]; pre = [p for t, p in S['arm_1'] if a0 - 5 <= t < a0]
        line += f' | arm_2 exec {a1 - a0:4.1f}s: arm_1 err {min(w):.2f}..{max(w):.2f} mm (5 s sebelum: {min(pre):.2f}..{max(pre):.2f}), drift {max(w) - min(pre):+.2f}'
        pk = collections.defaultdict(float)
        for r in REC:
            t = float(r['t'])
            if a0 <= t <= a1 and r['joint'].startswith('t1_a2_') and r['eff']:
                j = int(r['joint'][-1]); pk[j] = max(pk[j], abs(float(r['eff'])))
        p2, p3 = float(preds[-1][1]), float(preds[-1][2])  # last accepted screen before arm_2 MOVED = arm_2's executed plan
        d55.append((i, p2, pk[2], pk[2] - p2, p3, pk[3], pk[3] - p3))
    print(line)
print('--- D55: arm_2, rencana yang DIEKSEKUSI (prediksi RNEA mentah) vs terukur puncak ---')
for i, p2, m2, d2, p3, m3, d3 in d55:
    print(f'pair {i:2d}: joint_2 pred {p2:5.2f} meas {m2:5.2f} diff {d2:+5.2f} {"IN" if 4.8 <= d2 <= 8.4 else "out"} | joint_3 pred {p3:4.2f} meas {m3:4.2f} diff {d3:+5.2f}')
