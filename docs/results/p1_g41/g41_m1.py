"""G41 A3.1 M1 (docs/p1_g41_joint_goal.md): would a retract of gantry 1 AFTER its last task close the cross-gantry
failures of the G40/G40e plan-only runs? OFFLINE (no ROS): replays the saved plans.

Per (variant, seed, sample) that reached the first gantry-2 event: g1 end state = start + end of its last saved task
plan; M1a = return_rest.plan_retract(1, both arms, straight only); M1b = every g2 traverse of that schedule swept
(g29 RotCrossChecker, rect) with g1 REST at its final (lin, rot), g2 arms REST.

    python3 g41_m1.py
"""
import json
import math
import os
import sys
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
for d in ('p1_g31', 'p1_g22', 'p1_g29'):
    sys.path.insert(0, os.path.join(R, d))
import g22_plan as P  # noqa: E402
import g31_screen as G31S  # noqa: E402

RUNS = [('G40', 'g40_planonly.json', 'g40_planonly_plans.jsonl', 'g40_candidates.json'),
        ('G40e', 'g40e_planonly.json', 'g40e_planonly_plans.jsonl', 'g40e_candidates.json')]
CROSS_FAIL = {('G40', 8), ('G40e', 3), ('G40e', 18)}


def main():
    from return_rest import REST, plan_retract
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker, InterArmChecker
        import g29_rot_screen as RS
        chk_cross, chk_same, chk_rot = CrossGantryChecker(), InterArmChecker(gantry='gantry_1'), RS.RotCrossChecker()
    env = G31S.env_checker()
    out, closed = [], {}
    for var, fj, fp, fc in RUNS:
        d40 = os.path.join(R, 'p1_g40')
        cand = {r['seed']: r for r in json.load(open(os.path.join(d40, fc)))}
        plans = [json.loads(x) for x in open(os.path.join(d40, fp))]
        for res in json.load(open(os.path.join(d40, fj))):
            seed = res['seed']
            rec = cand[seed]
            p0 = {g: rec['p0'][str(g)] for g in (1, 2)}
            evs = G31S.events(rec[var]['schedule'], p0)
            k_g2 = min(k for k, e in enumerate(evs) if e['gantry'] == 2)
            trav2 = [e for e in evs if e['gantry'] == 2 and e['kind'] == 'traverse']
            # samples split in the log; a sample reached g2 iff its log has an event line [k >= k_g2]
            samples, cur = [], None
            for ln in res['log']:
                if 'sampel' in ln:
                    cur = []
                    samples.append(cur)
                elif cur is not None:
                    cur.append(ln)
            for si, lines in enumerate(samples):
                if not any(ln.strip().startswith(f'[{k}]') for ln in lines for k in range(k_g2, len(evs))):
                    continue
                rows = [p for p in plans if p['seed'] == seed and p['sample'] == si and p['k'] < k_g2
                        and p.get('traj')]
                last = max(rows, key=lambda p: p['k'])
                st = dict(last['start'])
                st.update(zip(last['traj']['joint_names'], last['traj']['pos'][-1]))
                lines_r = []
                va, _ = plan_retract(1, ['arm_1', 'arm_2'], st, chk_same, chk_cross, env, None,
                                     log=lines_r.append)
                held = dict(st)
                for a in ('arm_1', 'arm_2'):
                    held.update({f'{P.PREFIX[a]}joint_{i}': v for i, v in enumerate(REST, 1)})
                mb = []
                for e in trav2:
                    s = dict(held)
                    s['t2_linear_joint'], s['t2_rotation_joint'] = e['frm']
                    sw = RS.sweep_rot(chk_rot, 2, e['frm'], e['to'], s)
                    mb.append(dict(frm=[e['frm'][0], math.degrees(e['frm'][1])],
                                   to=[e['to'][0], math.degrees(e['to'][1])],
                                   verdict=sw['verdict'], d_mm=round(sw['d'] * 1000, 1), pair=list(sw['pair'])))
                ok = va == 'CLEAR' and all(m['verdict'] == 'CLEAR' for m in mb)
                g1f = (st['t1_linear_joint'], math.degrees(st['t1_rotation_joint']))
                print(f'{var} s{seed} sampel {si + 1}: g1 akhir ({g1f[0]:.2f}, {g1f[1]:+.0f}) task-terakhir t{last["task"]} '
                      f'{last["arm"]}; M1a retract lurus {va}; M1b '
                      + '; '.join(f"({m['frm'][0]:.2f},{m['frm'][1]:+.0f})->({m['to'][0]:.2f},{m['to'][1]:+.0f}) "
                                  f"{m['verdict']} {m['d_mm']}" for m in mb)
                      + f'  -> {"TERTUTUP" if ok else "TIDAK"}' + ('  [gagal lintas-gantry]' if (var, seed) in CROSS_FAIL
                                                                    and si == len(samples) - 1 else ''), flush=True)
                for s_ in lines_r:
                    print('      ' + s_)
                out.append(dict(variant=var, seed=seed, sample=si, g1_final=g1f, m1a=va, m1b=mb, ok=ok))
                if (var, seed) in CROSS_FAIL and si == len(samples) - 1:
                    closed[(var, seed)] = ok
    json.dump(out, open(os.path.join(HERE, 'g41_m1.json'), 'w'), indent=1)
    n = sum(closed.values())
    print(f'\nM1: gagal lintas-gantry tertutup {n}/{len(CROSS_FAIL)} {closed}; semua sampel '
          f'{sum(o["ok"] for o in out)}/{len(out)} tertutup')


if __name__ == '__main__':
    main()
