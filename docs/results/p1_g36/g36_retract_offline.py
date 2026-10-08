"""G36 B: reach_dwell_probe.screen_retract on every archived ev 10 plan (R10 seed 36), offline, no ROS graph.
Expect the G35b +103.7 deg plan (straight retract COLLIDE -14.09) and g34n s0 (MARGIN 19.64) REFUSED, the rest CLEAR
(= g35b_ev11_check.log).   python3 g36_retract_offline.py > g36_retract_offline.log"""
import glob, json, math, sys, types, warnings
warnings.simplefilter('ignore')
R = '/home/user1/Documents/ceiling_arm'
sys.path.insert(0, R + '/scripts')
import reach_dwell_probe as RDP
from env_collision import EnvChecker
log = types.SimpleNamespace(info=lambda s: None, error=print)
node = types.SimpleNamespace(wait_joints=lambda names: {}, get_logger=lambda: log, _env=EnvChecker())
for f in sorted(glob.glob(R + '/docs/results/p1_g3[34]/*plans*R10.jsonl')):
    for r in map(json.loads, open(f)):
        if r['k'] != 10 or 'traj' not in r:
            continue
        jt = types.SimpleNamespace(joint_names=r['traj']['joint_names'],
                                   points=[types.SimpleNamespace(positions=p) for p in r['traj']['pos']])
        v = RDP.screen_retract(types.SimpleNamespace(joint_trajectory=jt), r['arm'], node, r['start'])
        j1 = math.degrees(r['traj']['pos'][-1][r['traj']['joint_names'].index('t2_a1_joint_1')])
        print(f'{f.split("/")[-1]:45s} s{r["sample"]} {r["arm"]} j1 {j1:+7.1f} -> {v}')
