"""G36 C quick check: old screen_trajectory vs interarm_bb on the first N task plans of one archive."""
import json, sys, time, types, warnings
warnings.simplefilter('ignore')
R = '/home/user1/Documents/ceiling_arm'
sys.path.insert(0, R + '/scripts'); sys.path.insert(0, R + '/docs/results/p1_g36')
from interarm_collision import CrossGantryChecker, InterArmChecker
import interarm_bb as BB
PFX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
f, N = sys.argv[1], int(sys.argv[2])
chk = {1: InterArmChecker(gantry='gantry_1'), 2: InterArmChecker(gantry='gantry_2'), 'x': CrossGantryChecker()}
for c in chk.values():
    c.bb = types.MethodType(BB.screen_trajectory, c)
n = 0
for r in map(json.loads, open(f)):
    if 'traj' not in r or r.get('kind') == 'retract-moveit':
        continue
    g = int(PFX[r['arm']][1]); jn, pts = r['traj']['joint_names'], r['traj']['pos']
    for name, c, kw in (('S18', chk[g], {}), ('lintas', chk['x'], {'only': PFX[r['arm']]})):
        t = time.time(); a = c.screen_trajectory(jn, pts, r['start'], **kw); ta = time.time() - t
        t = time.time(); b = c.bb(jn, pts, r['start'], **kw); tb = time.time() - t
        print(f'{r["arm"]} n {len(pts):3d} {name:6s} old {a[0]} {a[1]*1000:.3f} {a[2]} {a[3]} {ta:.2f}s | bb {b[0]} '
              f'{b[1]*1000:.3f} {b[3]} {tb:.2f}s | {"SAMA" if a == b else "BEDA"}', flush=True)
    n += 1
    if n >= N:
        break
