"""G61 C2 (docs/p1_g61_offline.md A0/B): how far do the G57 schedule joint goals land from their targets when the
gantry stands where the bridge actually left it (traverse "akhir:"), not where the schedule put it? OFFLINE, HW data.

Per G57 run (results/p1_g57/hw/p1hw_par_*_run.json, every run incl. the invalid / injected ones):
  traverse: err_mm / err_deg of pose_to_g's "akhir:" line (rail dict in run.json)
  task with a goal: FK(goal joints) at (a) the NOMINAL gantry pose of its stop, (b) the MEASURED pose the gantry was
  left at by its last traverse (p0 / rot 0 when it never traversed: K0 measured), vs the scored target xyz;
  (c) the probe's own "ujung rencana vs target" lines (plan end, measured gantry from /joint_states) for comparison.
FK = the probe's goal_pose_error recipe (pinocchio, live URDF /tmp/reach_dwell_live.urdf sha f02e7c53, tool frame).

    python3 g61_c2_measure.py   -> g61_c2_measure.json + stdout table
"""
import glob
import json
import math
import os
import re

import numpy as np
import pinocchio as pin

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.join(HERE, '..', 'p1_g57', 'hw')
URDF = '/tmp/reach_dwell_live.urdf'
TOOL = {'arm_1': 't1_a1_tool_frame', 'arm_2': 't1_a2_tool_frame', 'arm_3': 't2_a1_tool_frame', 'arm_4': 't2_a2_tool_frame'}
M = pin.buildModelFromUrdf(URDF)
D = M.createData()
RE_END = re.compile(r'ujung rencana vs target: ([0-9.]+) mm')


def fk(arm, goal, lin, rot):
    g = int(TOOL[arm][1])
    q = pin.neutral(M)
    for n, v in list(goal.items()) + [(f't{g}_linear_joint', lin), (f't{g}_rotation_joint', rot)]:
        q[M.joints[M.getJointId(n)].idx_q] = v
    pin.framesForwardKinematics(M, D, q)
    return np.array(D.oMf[M.getFrameId(TOOL[arm])].translation)


def main():
    out = dict(traverse=[], task=[])
    for fn in sorted(glob.glob(os.path.join(HW, 'p1hw_par_*_run.json'))):
        run = os.path.basename(fn)[len('p1hw_par_'):-len('_run.json')]
        r = json.load(open(fn))
        k0 = json.load(open(fn.replace('_run.json', '_k0.json'))) if os.path.exists(fn.replace('_run.json', '_k0.json')) else {}
        meas = {}
        evs = sorted(r['events'], key=lambda e: e['t_start'])
        for e in evs:
            g = e['gantry']
            if e['kind'] == 'traverse' and e.get('rail'):
                rl = e['rail']
                meas[g] = (rl['end'][0], math.radians(rl['end'][1]))
                out['traverse'].append(dict(run=run, g=g, idx=e['idx'], goal=rl['goal'], end=rl['end'],
                                            err_mm=rl['err_mm'], err_deg=rl['err_deg'],
                                            d_rot=rl['goal'][1] - rl['start'][1], d_lin=rl['goal'][0] - rl['start'][0]))
            elif e['kind'] == 'task' and e.get('goal'):
                nom = None
                for f in evs:                                # nominal pose of the stop = the last traverse 'to' of g
                    if f['gantry'] == g and f['kind'] == 'traverse' and f['t_start'] < e['t_start']:
                        nom = tuple(f['to'])
                m_ = meas.get(g)
                if nom is None:                              # never traversed: p0, rot 0; measured = K0 medians
                    plan = {x['seed']: x for x in json.load(open(r['plan']))}[r['seed']]
                    nom = (float(plan['p0'][str(g)]), 0.0)
                    rows = k0.get('rows', {})
                    m_ = (rows[f't{g}_linear_joint']['median'], rows[f't{g}_rotation_joint']['median'])
                tgt = np.array(e['xyz'])
                ea = float(np.linalg.norm(fk(e['arm'], e['goal'], *nom) - tgt))
                eb = float(np.linalg.norm(fk(e['arm'], e['goal'], *m_) - tgt))
                logs = sorted(glob.glob(os.path.join(HW, f'p1hw_par_{run}_ev{e["idx"]:02d}_task.log')))
                ends = [float(x) for lf in logs for x in RE_END.findall(open(lf, errors='ignore').read())]
                out['task'].append(dict(run=run, g=g, idx=e['idx'], arm=e['arm'], nom=nom, meas=m_,
                                        d_rot_deg=math.degrees(m_[1] - nom[1]), d_lin_mm=(m_[0] - nom[0]) * 1000,
                                        err_nom_mm=ea * 1000, err_meas_mm=eb * 1000, probe_end_mm=ends,
                                        verdict=e.get('verdict_probe'), reach_xy=None))
    tr = out['traverse']
    er = np.array([t['err_deg'] for t in tr])
    el = np.array([t['err_mm'] for t in tr])
    rot_moves = [t for t in tr if abs(t['d_rot']) > 0.05]
    out['summary'] = dict(
        n_traverse=len(tr), err_deg_min=float(er.min()), err_deg_max=float(er.max()),
        err_deg_abs_p50=float(np.median(abs(er))), err_mm_abs_max=float(abs(el).max()),
        n_rot_moves=len(rot_moves),
        rot_moves_err_deg=sorted(round(t['err_deg'], 3) for t in rot_moves),
        n_task_goal=len(out['task']),
        err_meas_ge_2mm=sum(t['err_meas_mm'] >= 2.0 for t in out['task']),
        err_meas_ge_2p5mm=sum(t['err_meas_mm'] >= 2.5 for t in out['task']),
        err_meas_gt_3mm=sum(t['err_meas_mm'] > 3.0 for t in out['task']),
        err_nom_max_mm=max(t['err_nom_mm'] for t in out['task']),
    )
    print(f'{"run":24s} {"g":>2s} {"ev":>3s} {"arm":6s} {"drot":>7s} {"dlin":>6s} {"nom":>6s} {"meas":>6s}  probe-ends')
    for t in sorted(out['task'], key=lambda t: -t['err_meas_mm']):
        print(f'{t["run"]:24s} {t["g"]:2d} {t["idx"]:3d} {t["arm"]:6s} {t["d_rot_deg"]:+7.3f} {t["d_lin_mm"]:+6.2f} '
              f'{t["err_nom_mm"]:6.2f} {t["err_meas_mm"]:6.2f}  {t["probe_end_mm"]} {t["verdict"]}')
    print('traverse:')
    for t in tr:
        print(f'  {t["run"]:24s} g{t["g"]} ev {t["idx"]:2d} goal {t["goal"]} end {t["end"]} err {t["err_mm"]:+.3f} mm '
              f'{t["err_deg"]:+.3f} deg (drot {t["d_rot"]:+.1f}, dlin {t["d_lin"]:+.3f})')
    print(json.dumps(out['summary'], indent=1))
    json.dump(out, open(os.path.join(HERE, 'g61_c2_measure.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
