"""G35b B7.1: the ev 11 retract (arm_3 straight line home from the END of each archived ev 10 plan, R10) screened by the
FROZEN G34 checker (env_collision_old.py) and the G35 one. Offline, no ROS.
    python3 g35b_ev11_check.py <plans.jsonl>... > g35b_ev11_check.log"""
import json, sys, math, warnings, time, importlib.util
warnings.simplefilter('ignore')
R='/home/user1/Documents/ceiling_arm'
sys.path.insert(0, R+'/scripts')
import env_collision as NEW
spec = importlib.util.spec_from_file_location('old', R+'/docs/results/p1_g35/env_collision_old.py')
OLD = importlib.util.module_from_spec(spec); spec.loader.exec_module(OLD)
from return_rest import _rest_line
def end_state(r):
    s = dict(r['start']); t = r['traj']
    s.update(zip(t['joint_names'], t['pos'][-1]))
    return s
ne, oe = NEW.EnvChecker(), OLD.EnvChecker()
for f in sys.argv[1:]:
    for r in map(json.loads, open(f)):
        if r['k'] != 10: continue
        s = end_state(r)
        names = [f't2_a1_joint_{i}' for i in range(1, 7)]
        pts = _rest_line([s[n] for n in names])
        a = ne.screen_trajectory(names, pts, s); b = oe.screen_trajectory(names, pts, s)
        q = [round(math.degrees(s[n]), 1) for n in names]
        print(f.split('/')[-1][-20:], 'sample', r['sample'], 'arm_3 end', q, '| new', a[0], f'{a[1]*1000:.2f}', a[2], a[3], '| old', b[0], f'{b[1]*1000:.2f}', b[2], b[3])
