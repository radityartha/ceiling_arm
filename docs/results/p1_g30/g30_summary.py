"""G30: 8 legs -> table + D159-D165 scoring inputs (from the rot_to_g JSON records and per-leg launch slices)."""
import glob, json, os, re
HERE = os.path.dirname(os.path.abspath(__file__))
rows = []
for g in (1, 2):
    for k in range(1, 5):
        p = os.path.join(HERE, f'g30_g{g}_leg{k}.log')
        rec = json.loads([l for l in open(p) if l.startswith('{')][-1])
        rc = int(re.search(r'rc=(\d+)', open(p).read()).group(1))
        lp = p + '.launch'
        if os.path.exists(lp):
            L = open(lp).read()
        else:                                     # leg 1 of g1: slice not saved -> from the full launch log window
            import gzip
            full = gzip.open(os.path.join(HERE, 'g30_launch.log.gz'), 'rt').read().splitlines()
            L = '\n'.join(l for l in full if (m := re.search(r'\[(\d{10}\.\d+)\]', l))
                          and rec['t_send'] - 1 <= float(m.group(1)) <= rec['t_stop'] + 3)
        tab = f'table{g}'
        disp = len(re.findall(rf'bridge: {tab} ->', L))
        skip = len(re.findall(r'Already at absolute target', L))
        absm = len(re.findall(r'Absolute move', L))
        bad = len(re.findall(r'(?i)fault|exception|NOT armed|REJECT|timeout', L))
        d = abs(rec['goal_deg'] - rec['start_deg'])
        short = rec['err_deg'] * (rec['goal_deg'] - rec['start_deg']) <= 0
        rows.append(dict(g=g, leg=k, rc=rc, **{x: rec[x] for x in ('start_deg', 'goal_deg', 'end_deg', 'err_deg', 't_rot',
                    'T_cmd', 'T_rot_model', 'arm_drift_deg', 'rail_drift_mm', 'other_rail_drift_mm', 'other_rot_drift_deg',
                    'arm_tau_peak', 'arm_tau_joint', 'sweep_min_mm', 'sweep_arm_mm', 'jtc_error_code')},
                    short_or_zero=short, ratio=round(rec['t_rot'] / rec['T_rot_model'], 2), dispatch=disp, absolute=absm,
                    skipped=skip, bad_lines=bad))
for r in rows:
    print(f"g{r['g']} kaki {r['leg']}: {r['start_deg']:+.2f} -> {r['goal_deg']:+.0f} akhir {r['end_deg']:+.2f} galat {r['err_deg']:+.2f} "
          f"({'kurang/0' if r['short_or_zero'] else 'LEBIH'}); t_rot {r['t_rot']:.3f} s (/T_rot {r['ratio']}); drift lengan "
          f"{r['arm_drift_deg']:.3f}; rel {r['rail_drift_mm']:.3f}/{r['other_rail_drift_mm']:.3f} mm; rot lain {r['other_rot_drift_deg']:.3f}; "
          f"tau {r['arm_tau_peak']:.2f} {r['arm_tau_joint']}; S28 {r['sweep_min_mm']}/{r['sweep_arm_mm']}; kirim {r['dispatch']} "
          f"(lewati {r['skipped']}); buruk {r['bad_lines']}; rc {r['rc']}")
json.dump(rows, open(os.path.join(HERE, 'g30_summary.json'), 'w'), indent=1)
