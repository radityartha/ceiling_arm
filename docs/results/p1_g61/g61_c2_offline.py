"""G61 A5.2 offline: run_g54.GoalFix on every G57 task goal (g61_c2_measure.json: nominal vs measured gantry) -- how many
corrections trigger, does the IK converge, |dq|, and the corrected goal's FK at the measured pose vs the target. OFFLINE.

    python3 g61_c2_offline.py   -> g61_c2_offline.json + stdout
"""
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g54'))
import run_g54 as RG  # noqa: E402

HW = os.path.join(HERE, '..', 'p1_g57', 'hw')
F = RG.GoalFix(RG.URDF)


def tool(goal, g, lin, rot):
    pin, m, d = F.pin, F.m, F.d
    fid = m.getFrameId(sorted(goal)[0].rsplit('joint_', 1)[0] + 'tool_frame')
    pin.framesForwardKinematics(m, d, F._q(goal, g, lin, rot))
    return np.array(d.oMf[fid].translation)


def main():
    meas = json.load(open(os.path.join(HERE, 'g61_c2_measure.json')))
    goals = {}
    for fn in glob.glob(os.path.join(HW, 'p1hw_par_*_run.json')):
        run = os.path.basename(fn)[len('p1hw_par_'):-len('_run.json')]
        for e in json.load(open(fn))['events']:
            if e['kind'] == 'task' and e.get('goal'):
                goals[(run, e['idx'])] = (e['goal'], e['xyz'])
    rows = []
    for t in meas['task']:
        goal, xyz = goals[(t['run'], t['idx'])]
        g2, info = F.fix(goal, t['g'], tuple(t['nom']), tuple(t['meas']))
        e_old = np.linalg.norm(tool(goal, t['g'], *t['meas']) - xyz) * 1000
        e_new = np.linalg.norm(tool(g2, t['g'], *t['meas']) - xyz) * 1000
        rows.append(dict(run=t['run'], idx=t['idx'], arm=t['arm'], err_old_mm=e_old, err_new_mm=e_new, **info))
    trig = [r for r in rows if 'ik_pos_mm' in r]
    s = dict(n=len(rows), triggered=len(trig), applied=sum(r['applied'] for r in rows),
             dq_max_deg=max([r['dq_max_deg'] for r in trig], default=0.0),
             err_new_max_mm_triggered=max([r['err_new_mm'] for r in trig], default=0.0),
             err_old_gt3=sum(r['err_old_mm'] > 3.0 for r in rows), err_new_gt3=sum(r['err_new_mm'] > 3.0 for r in rows),
             untouched_identical=all(r['err_new_mm'] == r['err_old_mm'] for r in rows if 'ik_pos_mm' not in r))
    for r in sorted(trig, key=lambda r: -r['disp_mm']):
        print(f'{r["run"]:22s} ev {r["idx"]:2d} {r["arm"]} disp {r["disp_mm"]:5.2f} mm  target-err {r["err_old_mm"]:5.2f} -> '
              f'{r["err_new_mm"]:5.2f} mm  IK {r["ik_pos_mm"]:.5f} mm {r["ik_rot_deg"]:.5f} deg  |dq| {r["dq_max_deg"]:.3f}  '
              f'{"DITERAPKAN" if r["applied"] else "GAGAL"}')
    print(json.dumps(s, indent=1, default=float))
    json.dump(dict(summary=s, rows=rows), open(os.path.join(HERE, "g61_c2_offline.json"), "w"), indent=1, default=float)


if __name__ == '__main__':
    main()
