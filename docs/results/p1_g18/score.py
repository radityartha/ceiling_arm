import re, json
W = [l.split() for l in open('/tmp/g18_windows.txt')]
W = [(int(a), float(b), float(c), float(d)) for a, b, c, d in W]
starts = [w[1] for w in W] + [9e12]
ev = []
for l in open('/tmp/g18_monitor.log'):
    m = re.search(r'\[(\d+\.\d+)\] \[reach_dwell_monitor\]: (.*)', l)
    if m: ev.append((float(m.group(1)), m.group(2)))
res = {}
for k, (i, t0, t1, t2) in enumerate(W):
    lo, hi = t0, starts[k + 1]
    E = [(t, s) for t, s in ev if lo <= t < hi]
    r = {'pair': i, 'arm_1': None, 'arm_2': None, 'concurrent': None}
    for t, s in E:
        m = re.search(r'>>> (arm_\d): SUCCESS -- held 2.0s, pos max ([\d.]+)mm \(settled ([\d.]+)\).*ori max ([\d.]+)deg.*?(\d+) samples @ ([\d.]+) Hz', s)
        if m and r[m.group(1)] is None:
            r[m.group(1)] = dict(t=round(t - t1, 1), pos_max=float(m.group(2)), settled=float(m.group(3)), ori_max=float(m.group(4)), n=int(m.group(5)), hz=float(m.group(6)))
        if 'CONCURRENT' in s and r['concurrent'] is None: r['concurrent'] = round(t - t1, 1)
    both = r['arm_1'] and r['arm_2']
    r['verdict'] = 'CONCURRENT' if r['concurrent'] is not None else ('STAGGERED' if both else ('PARTIAL' if (r['arm_1'] or r['arm_2']) else 'NEITHER'))
    res[i] = r
    fmt = lambda a: f"{a['pos_max']:.2f}/{a['settled']:.2f}mm {a['ori_max']:.2f}deg n{a['n']}@{a['hz']:.0f}" if a else '-'
    print(f"pair {i:2d} {r['verdict']:10s} conc@{r['concurrent']}s | arm_1 {fmt(r['arm_1'])} | arm_2 {fmt(r['arm_2'])}")
json.dump(res, open('/tmp/g18_scored.json', 'w'), indent=1)
