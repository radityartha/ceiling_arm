"""G29 B-extra: history dependence of hull GJK distances (found in N2). Diagnostic, OFFLINE."""
import json, sys, time
import numpy as np
import pinocchio as pin
import g29_rot_screen as G, g22_plan as P
from interarm_collision import CrossGantryChecker

need = G.arm_joint_names() + ['t1_linear_joint', 't2_linear_joint', 't1_rotation_joint', 't2_rotation_joint']


def ev(run, csvp, i):
    d = json.load(open('../' + run)); e = [e for e in d['events'] if e['kind'] == 'traverse'][i]; R = e['rail']
    st = G.js_state_at(csvp, R['t_send'], need); return {k: st[k] for k in need}, R, e['gantry']


def reset(c):
    for r in c.geom_data.distanceRequests:
        r.enable_cached_gjk_guess = False
        r.cached_support_func_guess = np.zeros(2, dtype=np.int32)
        r.cached_gjk_guess = np.array([1.0, 0.0, 0.0])


def pair_d(c, q, k):
    return c.check(q)  # placeholder


A = ev('p1_g24/g24b_run.json', '/tmp/g24b_js.csv', 1)
B = ev('p1_g26/g26_s1_run.json', '/tmp/g26_js.csv', 0)
sw = lambda c, X: P.sweep_screen(c, X[2], X[0][f't{X[2]}_linear_joint'], X[1]['goal'], X[0])
c = CrossGantryChecker(urdf=G.URDF)
print('flags default:', c.geom_data.distanceRequests[0].enable_cached_gjk_guess,
      c.geom_data.distanceRequests[0].gjk_initial_guess)
print('B fresh', round(sw(c, B)[1] * 1000, 3))
c = CrossGantryChecker(urdf=G.URDF); sw(c, A); print('B after A', round(sw(c, B)[1] * 1000, 3))
c = CrossGantryChecker(urdf=G.URDF); reset(c); sw(c, A); reset(c); print('B after A, reset', round(sw(c, B)[1] * 1000, 3))
# which knob? inspect results
c = CrossGantryChecker(urdf=G.URDF); sw(c, A)
dr = c.geom_data.distanceRequests
print('after A: cached_support_func_guess sample', [tuple(dr[k].cached_support_func_guess) for k in range(3)],
      'enable_cached', dr[0].enable_cached_gjk_guess)
c = CrossGantryChecker(urdf=G.URDF); sw(c, A); c.geom_data = c.geom.createData()
print('B after A, fresh geom_data', round(sw(c, B)[1] * 1000, 3))
# per-point history: same state computed after different predecessors
q = None
c = CrossGantryChecker(urdf=G.URDF)
X = B
pts = np.linspace(X[0]['t1_linear_joint'], X[1]['goal'], 56)
base = c.q_from(X[0])
iq = c.model.joints[c.model.getJointId('t1_linear_joint')].idx_q
def dmin(c, q):
    pin.updateGeometryPlacements(c.model, c.data, c.geom, c.geom_data, q)
    pin.computeDistances(c.model, c.data, c.geom, c.geom_data, q)
    return np.array([r.min_distance for r in c.geom_data.distanceResults])
seq = []
for x in pts:
    q = base.copy(); q[iq] = x; seq.append(dmin(c, q))
c2 = CrossGantryChecker(urdf=G.URDF)
fresh = []
for x in pts:
    c2.geom_data = c2.geom.createData(); q = base.copy(); q[iq] = x; fresh.append(dmin(c2, q))
seq, fresh = np.array(seq), np.array(fresh)
D = seq - fresh
print('sweep B: per-pair |seq - fresh| max %.3f mm, pairs affected %d/%d, min-over-pairs seq %.3f fresh %.3f' % (
    np.abs(D).max() * 1000, int((np.abs(D) > 1e-9).any(0).sum()), D.shape[1], seq.min() * 1000, fresh.min() * 1000))
k = np.unravel_index(np.abs(D).argmax(), D.shape)
cp = c.geom.collisionPairs[int(k[1])]
print('  worst pair', c.names[cp.first], c.names[cp.second], 'seq', seq[k] * 1000, 'fresh', fresh[k] * 1000)
# ground truth for the worst pair: mesh (BVH, no hull), and hull with generous GJK iterations
from interarm_collision import _package_dirs
mesh_geom = pin.buildGeomFromUrdf(c.model, G.URDF, pin.GeometryType.COLLISION, _package_dirs())
names = [g.name for g in mesh_geom.geometryObjects]
i, j = names.index(c.names[cp.first]), names.index(c.names[cp.second])
mesh_geom.addCollisionPair(pin.CollisionPair(i, j))
md = mesh_geom.createData()
q = base.copy(); q[iq] = pts[k[0]]
pin.updateGeometryPlacements(c.model, c.data, mesh_geom, md, q)
pin.computeDistances(c.model, c.data, mesh_geom, md, q)
print('  mesh (exact) same pair/state: %.3f mm' % (md.distanceResults[0].min_distance * 1000))
c3 = CrossGantryChecker(urdf=G.URDF)
for r in c3.geom_data.distanceRequests:
    r.gjk_max_iterations = 5000
kk = int(k[1])
d3 = dmin(c3, q)[kk]
print('  hull, gjk_max_iterations 5000: %.3f mm' % (d3 * 1000))
# systematic: reference = hull, fresh data per point, gjk_max_iterations 5000
def ref_d(q):
    cr = CrossGantryChecker(urdf=G.URDF)
    for r in cr.geom_data.distanceRequests:
        r.gjk_max_iterations = 5000
    return dmin(cr, q)
cr = CrossGantryChecker(urdf=G.URDF)
for r in cr.geom_data.distanceRequests:
    r.gjk_max_iterations = 5000
ref = []
for x in pts:
    cr.geom_data = cr.geom.createData()
    for r in cr.geom_data.distanceRequests:
        r.gjk_max_iterations = 5000
    q = base.copy(); q[iq] = x; ref.append(dmin(cr, q))
ref = np.array(ref)
for nm, A_ in (('seq(warm, default iters)', seq), ('fresh(default iters)', fresh)):
    E = A_ - ref
    print('%-26s err max %+.3f / min %+.3f mm; |err|>0.1mm on %d of %d pair-pts; min-over-pairs %.3f vs ref %.3f' % (
        nm, E.max() * 1000, E.min() * 1000, int((np.abs(E) > 1e-4).sum()), E.size, A_.min() * 1000, ref.min() * 1000))
# candidate fix: enable_cached_gjk_guess = False on every request, set once, sequential
cf = CrossGantryChecker(urdf=G.URDF)
for r in cf.geom_data.distanceRequests:
    r.enable_cached_gjk_guess = False
t = time.time(); nocache = np.array([dmin(cf, (lambda x: (lambda q: (q.__setitem__(iq, x), q)[1])(base.copy()))(x)) for x in pts]); tn = time.time() - t
cw = CrossGantryChecker(urdf=G.URDF)
t = time.time(); _ = [dmin(cw, (lambda x: (lambda q: (q.__setitem__(iq, x), q)[1])(base.copy()))(x)) for x in pts]; tw = time.time() - t
t = time.time()
for x in pts:
    cw.geom_data = cw.geom.createData(); q = base.copy(); q[iq] = x; dmin(cw, q)
tf = time.time() - t
E = nocache - ref
print('nocache seq: err max %+.4f / min %+.4f mm; time/pt nocache %.2f ms, warm %.2f ms, fresh-data %.2f ms' % (
    E.max() * 1000, E.min() * 1000, tn / len(pts) * 1000, tw / len(pts) * 1000, tf / len(pts) * 1000))
cz = CrossGantryChecker(urdf=G.URDF)
z2 = np.zeros(2, dtype=np.int32)
out = []
t = time.time()
for x in pts:
    for r in cz.geom_data.distanceRequests:
        r.enable_cached_gjk_guess = False
        r.cached_support_func_guess = z2
    q = base.copy(); q[iq] = x; out.append(dmin(cz, q))
E = np.array(out) - ref
print('reset support hint per call: err max %+.4f / min %+.4f mm, %.2f ms/pt' % (E.max() * 1000, E.min() * 1000, (time.time() - t) / len(pts) * 1000))
print('distance_functors?', [a for a in dir(cz.geom_data) if 'functor' in a.lower() or 'solver' in a.lower()])
