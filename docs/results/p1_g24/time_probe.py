"""G24 A3: wall time per tuple on tuples OUTSIDE V/W/F/C' (not G22 task nodes). Time only."""
import json, os, sys, time
import numpy as np
import oracle2 as O2
sys.path.insert(0, '../../../ros2_ws/src/reachability_gng')
from reachability_gng import sched
from reachability_gng.capability import CapabilityMap
c = CapabilityMap.load('../../../ros2_ws/src/reachability_gng/data/cap_g1_rail160.npz')
g22 = {n for r in json.load(open('../p1_g22/g22_candidates.json')) for n in r['nodes']}
r, _, _ = sched._gantry_oracles(c, np.arange(len(c.nodes)))
r = r.reshape(len(c.nodes), 33, 72, 2)[:, :, 36, :]      # rot index of 0
assert abs(c.rot[36]) < 1e-9
rng = np.random.default_rng(7)
idx = [t for t in np.argwhere(r) if t[0] not in g22]
pick = [idx[i] for i in rng.choice(len(idx), 6, replace=False)]
for n, l, s in pick:
    arm = ('arm_1', 'arm_2')[s]
    t = time.time(); res = O2.solve(c.nodes[n], c.lin[l], arm); dt = time.time() - t
    print(f'{dt:6.1f} s  rounds {res["rounds"]}')
