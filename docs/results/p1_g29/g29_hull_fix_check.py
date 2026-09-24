"""B1b: fixed hulls vs mesh on the g29_hull_diag pairs; history (warm vs fresh) with fixed hulls."""
import json, time
import numpy as np, pinocchio as pin
import g29_map as M, g29_rot_screen as G
from interarm_collision import _package_dirs
D = json.load(open('g29_hull_diag.json'))['rows']
c = G.RotCrossChecker()
mg = pin.buildGeomFromUrdf(c.model, G.URDF, pin.GeometryType.COLLISION, _package_dirs())
cfgs = sorted({tuple(r['cfg']) for r in D})
err, cm = [], []
for dx, r1, r2 in cfgs:
    l1, l2 = M.lins(dx); q = c.q_from(G.rest_state(l1, r1, l2, r2))
    c.geom_data = c.geom.createData()
    pin.updateGeometryPlacements(c.model, c.data, c.geom, c.geom_data, q)
    pin.computeDistances(c.model, c.data, c.geom, c.geom_data, q)
    h = {(c.names[cp.first], c.names[cp.second]): c.geom_data.distanceResults[k].min_distance
         for k, cp in enumerate(c.geom.collisionPairs) if k < c.n_arm}
    for r in D:
        if tuple(r['cfg']) == (dx, r1, r2):
            err.append(h[tuple(r['pair'])] - r['m'])
    cm.append(min(v for v in h.values()) - min(r['m'] for r in D if tuple(r['cfg']) == (dx, r1, r2)))
err, cm = np.array(err), np.array(cm)
print(f'hull BENAR - mesh, {len(err)} pasangan: maks {err.max()*1000:+.3f} / min {err.min()*1000:+.3f} mm '
      f'(hull <= mesh wajib: > +0.01 mm pada {int((err > 1e-5).sum())}); min per konfigurasi: maks {cm.max()*1000:+.3f}, '
      f'min {cm.min()*1000:+.3f} mm')
# history: warm vs fresh on 400 random states (all 676 pairs)
rng = np.random.default_rng(7)
names = G.arm_joint_names(); iq = {n: c.model.joints[c.model.getJointId(n)].idx_q for n in names}
S = []
for _ in range(400):
    s = {'t1_linear_joint': rng.uniform(0, 1.6), 't2_linear_joint': rng.uniform(0, 1.6),
         't1_rotation_joint': rng.uniform(-np.pi, np.pi), 't2_rotation_joint': rng.uniform(-np.pi, np.pi)}
    for n in names:
        s[n] = rng.uniform(c.model.lowerPositionLimit[iq[n]], c.model.upperPositionLimit[iq[n]])
    S.append(c.q_from(s))
def allpairs(fresh):
    out = []
    gd = c.geom.createData()
    t = time.time()
    for q in S:
        if fresh:
            gd = c.geom.createData()
        pin.updateGeometryPlacements(c.model, c.data, c.geom, gd, q)
        pin.computeDistances(c.model, c.data, c.geom, gd, q)
        out.append([r.min_distance for r in gd.distanceResults])
    return np.array(out), (time.time() - t) / len(S) * 1000
W, tw = allpairs(False); F, tf = allpairs(True)
print(f'hull BENAR hangat vs segar, 400 keadaan x {W.shape[1]} pasangan: maks |beda| {np.abs(W - F).max()*1000:.6f} mm; '
      f'{tw:.2f} ms/keadaan hangat, {tf:.2f} segar')
