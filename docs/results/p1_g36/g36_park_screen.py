"""G36 park (operator 2026-10-08): gantry 1 parked at the FAR end between runs, encoder 0 unchanged.
Offline screen of the traverse g1 (0,0) -> (L,0) and back, all arms REST, g2 at (G2,0): S28 sweep_rot + EnvChecker,
the same two screens pose_to_g runs.   python3 g36_park_screen.py > g36_park_screen.log"""
import math, sys, warnings
warnings.simplefilter('ignore')
R = '/home/user1/Documents/ceiling_arm'
sys.path[:0] = [R + '/scripts', R + '/docs/results/p1_g29', R + '/docs/results/p1_g22']
import g22_plan as P
import g29_rot_screen as RS
from env_collision import EnvChecker
chk, env = RS.RotCrossChecker(), EnvChecker()
for g2 in (0.0, 1.35):
    for L in (1.45, 1.50):
        st = {'t1_linear_joint': 0.0, 't2_linear_joint': g2, 't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
        for p in P.PREFIX.values():
            st.update({f'{p}joint_{i}': v for i, v in enumerate(P.REST, 1)})
        sw = RS.sweep_rot(chk, 1, (0.0, 0.0), (L, 0.0), st)
        ev = env.screen_trajectory(['t1_linear_joint', 't1_rotation_joint'], RS.rect_points((0.0, 0.0), (L, 0.0)), st)
        print(f'g2 {g2:.2f}: g1 0 -> {L:.2f}  S28 {sw["verdict"]} {sw["d"]*1000:.1f} mm ({sw["pair"]}, n {sw["n"]})  '
              f'env {ev[0]} {ev[1]*1000:.1f} mm ({ev[2]}, titik {ev[3]})', flush=True)
