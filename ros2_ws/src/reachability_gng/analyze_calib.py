#!/usr/bin/env python3
"""Calibration analysis for the J-cost weights/references (gantry_reach_executor).

Reads a pick CSV (the `csv_log` output) and reports the three checks used to
decide whether `w_*` and `ref_*` are well-tuned:

  Tahap 1 (ref):   raw-value distribution per term -> suggested ref = median,
                   plus the value/ref spread under the CURRENT refs (should be
                   ~O(1) and comparable across terms).
  Tahap 2 (w):     weighted contribution share of each term in J (does the
                   priority ranking match intent?).
  Tahap 3 (truth): Spearman rank-correlation between J and traj_energy (does a
                   low J actually mean low mechanical energy?).

The J term for distance uses `ee_dist` (matching the node), NOT the `dist`
column (that one only gates/logs the pool).

  python3 analyze_calib.py /tmp/calib.csv
  python3 analyze_calib.py /tmp/calib.csv --w_hold 6 --ref_hold 4

NOTE ON BIAS: this CSV holds only the CHOSEN candidate per pick. If selection
favoured (say) low-hold configs, median(hold) here is biased low -> the
suggested ref_hold is too small. For an UNBIASED ref, run the picks with equal
weights (all w_*:=1) so selection does not skew the sample (README "Opsi B").
"""
import argparse
import csv
import math

# term -> (csv raw column, default weight, default ref, sign)
# sign is +1 for a cost, -1 for a reward (manip lowers J).
TERMS = {
    'glin':  ('d_gantry_lin', 10.0, 0.5, +1),
    'grot':  ('d_gantry_rot', 10.0, 0.5, +1),
    'arm':   ('d_arm',         1.0, 1.0, +1),
    'dist':  ('ee_dist',      25.0, 0.5, +1),
    'hold':  ('hold',          3.0, 6.5, +1),
    'manip': ('manip',         3.0, 0.1, -1),
}


def _f(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return None
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


def _quantile(xs, q):
    s = sorted(xs)
    if not s:
        return None
    i = q * (len(s) - 1)
    lo = int(math.floor(i))
    hi = int(math.ceil(i))
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (i - lo)


def _spearman(a, b):
    """Spearman rho via Pearson on ranks (no scipy). Ties get average ranks."""
    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(v):
            j = i
            while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r
    ra, rb = ranks(a), ranks(b)
    n = len(a)
    ma, mb = sum(ra) / n, sum(rb) / n
    num = sum((ra[i] - ma) * (rb[i] - mb) for i in range(n))
    da = math.sqrt(sum((x - ma) ** 2 for x in ra))
    db = math.sqrt(sum((x - mb) ** 2 for x in rb))
    return num / (da * db) if da and db else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', help='pick CSV written by csv_log')
    for name, (_col, w, ref, _s) in TERMS.items():
        ap.add_argument(f'--w_{name}', type=float, default=w)
        ap.add_argument(f'--ref_{name}', type=float, default=ref)
    args = ap.parse_args()
    w = {n: getattr(args, f'w_{n}') for n in TERMS}
    ref = {n: getattr(args, f'ref_{n}') for n in TERMS}

    with open(args.csv, newline='') as f:
        rows = [r for r in csv.DictReader(f)]
    if not rows:
        print('empty CSV'); return

    # raw value per term across all logged candidates (skip blank/failed rows)
    vals = {n: [] for n in TERMS}
    for r in rows:
        for n, (col, *_1) in TERMS.items():
            v = _f(r.get(col))
            if v is not None:
                vals[n].append(v)

    print(f'\n=== {len(rows)} rows from {args.csv} ===')

    # --- Tahap 1: ref calibration ------------------------------------------
    print('\n[Tahap 1] raw-value distribution per term  (suggested ref = median)')
    print(f'  {"term":<6} {"n":>4} {"p25":>8} {"median":>8} {"p75":>8} '
          f'{"max":>8} | {"ref_now":>8} {"med/ref":>8}')
    for n in TERMS:
        xs = vals[n]
        if not xs:
            print(f'  {n:<6}  (no data)'); continue
        med = _median(xs)
        mr = med / ref[n] if ref[n] else float('nan')
        flag = '  <-- med/ref far from 1: rescale ref' if (mr > 2 or mr < 0.3) else ''
        print(f'  {n:<6} {len(xs):>4} {_quantile(xs,.25):>8.3f} {med:>8.3f} '
              f'{_quantile(xs,.75):>8.3f} {max(xs):>8.3f} | '
              f'{ref[n]:>8.3f} {mr:>8.2f}{flag}')
    print('  GOAL: med/ref similar (~O(1)) across terms -> refs make terms comparable.')

    # --- Tahap 2: weight / contribution share ------------------------------
    print('\n[Tahap 2] mean weighted contribution |w*value/ref| and its share of J')
    contrib = {}
    for n, (col, *_1) in TERMS.items():
        cs = [abs(w[n] * v / ref[n]) for v in vals[n]] if ref[n] else []
        contrib[n] = sum(cs) / len(cs) if cs else 0.0
    tot = sum(contrib.values()) or 1.0
    print(f'  {"term":<6} {"w":>6} {"ref":>7} {"mean|contrib|":>14} {"share%":>8}')
    for n in sorted(TERMS, key=lambda k: -contrib[k]):
        print(f'  {n:<6} {w[n]:>6.2f} {ref[n]:>7.3f} {contrib[n]:>14.3f} '
              f'{100*contrib[n]/tot:>7.1f}%')
    print('  GOAL: share ranking matches intended priority (dist leads, gantry '
          'next, hold/manip small, arm least).')

    # --- Tahap 3: ground-truth (J vs energy) -------------------------------
    # IMPORTANT: recompute J from the raw per-term columns using the w_*/ref_*
    # given on THIS run's command line, rather than trusting the CSV's stored
    # `J` column. That column was computed with whatever weights/refs were
    # LIVE at collection time (often neutral w=1 to avoid selection bias,
    # while ref_* still held old/uncalibrated defaults) -- it does not
    # necessarily match the candidate weighting you are evaluating now.
    print('\n[Tahap 3] does low J mean low energy?  Spearman(J, traj_energy)')

    def recompute_j(r):
        total = 0.0
        for n, (col, _w, _ref, sign) in TERMS.items():
            v = _f(r.get(col))
            if v is None or not ref[n]:
                return None
            total += sign * w[n] * v / ref[n]
        return total

    succ_rows = [r for r in rows if _f(r.get('success')) == 1.0]
    pairs = []
    for r in succ_rows:
        j = recompute_j(r)
        e = _f(r.get('traj_energy'))
        if j is not None and e is not None:
            pairs.append((j, e))
    if len(pairs) < 3:
        print(f'  only {len(pairs)} rows with recomputable J and finite '
              'traj_energy (need >=3). Run picks with execution/energy '
              'logging enabled.')
    else:
        rho = _spearman([j for j, _e in pairs], [e for _j, e in pairs])
        verdict = ('GOOD: J is a solid energy surrogate' if rho > 0.7 else
                   'WEAK: J only loosely tracks energy' if rho > 0.4 else
                   'BAD: J does not predict energy -- revisit terms/weights')
        print(f'  n={len(pairs)}  rho={rho:.3f}  (J recomputed with the '
              f'w_*/ref_* above)  -> {verdict}')

    # for reference: correlation using the AS-COLLECTED J column (whatever
    # weights/refs were live on the node during logging -- may differ from
    # the w_*/ref_* used above).
    old_pairs = [(_f(r.get('J')), _f(r.get('traj_energy'))) for r in succ_rows]
    old_pairs = [(j, e) for j, e in old_pairs if j is not None and e is not None]
    if len(old_pairs) >= 3:
        rho_old = _spearman([j for j, _e in old_pairs],
                            [e for _j, e in old_pairs])
        print(f'  (for reference: as-collected J column vs traj_energy: '
              f'n={len(old_pairs)} rho={rho_old:.3f})')

    # --- Tahap 4: per-term correlation with energy --------------------------
    # Which raw terms actually track traj_energy, individually? A term with
    # |rho| near 0 is dead weight in J (or has the wrong sign) regardless of
    # how it's weighted; a term with strong |rho| is a good energy proxy and
    # deserves more weight. `manip` is a REWARD in J (sign -1: higher manip ->
    # lower J), so for manip a NEGATIVE rho with energy is the "correct" sign
    # (higher manipulability -> lower energy); for the cost terms a POSITIVE
    # rho is "correct".
    print('\n[Tahap 4] per-term Spearman(term, traj_energy) -- which terms '
          'actually track energy?')
    energy_by_row = [_f(r.get('traj_energy')) for r in succ_rows]
    print(f'  {"term":<6} {"n":>4} {"rho":>7}  expected-sign  verdict')
    term_rhos = []
    for n, (col, _w, _ref, sign) in TERMS.items():
        xs, es = [], []
        for r, e in zip(succ_rows, energy_by_row):
            v = _f(r.get(col))
            if v is not None and e is not None:
                xs.append(v); es.append(e)
        if len(xs) < 3:
            print(f'  {n:<6} {len(xs):>4}     n/a  (need >=3)')
            continue
        rho = _spearman(xs, es)
        term_rhos.append((n, rho))
        want = 'negative' if sign < 0 else 'positive'
        got_ok = (rho < -0.2) if sign < 0 else (rho > 0.2)
        verdict = ('tracks energy (correct sign)' if got_ok else
                   'wrong sign / no signal -- reconsider'
                   if abs(rho) > 0.2 else 'no signal (|rho| small)')
        print(f'  {n:<6} {len(xs):>4} {rho:>7.3f}  {want:<13} {verdict}')
    if term_rhos:
        best = max(term_rhos, key=lambda t: abs(t[1]))
        print(f'  strongest single-term signal: {best[0]} (rho={best[1]:.3f})')


if __name__ == '__main__':
    main()
