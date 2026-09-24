"""G29 post-lock: hull distance ABOVE mesh distance (up to +18.4 mm, g29_map hull_vs_mesh). Which is wrong?
For the 50 near-margin configurations of g29_map, per ARM-BEARING pair with mesh distance < 150 mm:
  h  = hull GJK (fresh data, = RotCrossChecker)       m  = mesh BVH distance (fresh data)
  hv = hull vertex-vertex (upper bound of the true hull distance)
  mv = mesh vertex-vertex (upper bound of the true mesh distance)
  in = fraction of mesh vertices inside the hull (halfspace test) -- a hull must contain its mesh.
"""
import json
import numpy as np
import pinocchio as pin
from scipy.spatial import cKDTree
import g29_map as M
import g29_rot_screen as G
from interarm_collision import _package_dirs

Z = np.load(M.OUT)
arm = np.minimum(Z['LL'], Z['LS'])
idx = np.argsort(np.abs(arm - 0.05), axis=None)[:50]
cfg = [(float(M.DX[i]), float(M.ROT[a]), float(M.ROT[b])) for i, a, b in zip(*np.unravel_index(idx, arm.shape))]
c = G.RotCrossChecker()
mg = pin.buildGeomFromUrdf(c.model, G.URDF, pin.GeometryType.COLLISION, _package_dirs())
mn = [g.name for g in mg.geometryObjects]
assert mn == c.names
for k in range(c.n_arm):
    cp = c.geom.collisionPairs[k]
    mg.addCollisionPair(pin.CollisionPair(cp.first, cp.second))


def pts_hull(i):
    g = c.geom.geometryObjects[i].geometry
    return np.array([g.points(t) for t in range(g.num_points)]) if hasattr(g, 'num_points') else None


def pts_mesh(i):
    g = mg.geometryObjects[i].geometry
    return np.array([g.vertices(t) for t in range(g.num_vertices)]) if hasattr(g, 'num_vertices') else None


def world(P, M_):
    return (M_.rotation @ P.T).T + M_.translation


# containment per geometry (local frames identical: same URDF placement)
from scipy.spatial import ConvexHull, Delaunay
contain = {}
for i, n in enumerate(c.names):
    H, V = pts_hull(i), pts_mesh(i)
    if H is None or V is None:
        continue
    d = Delaunay(H)
    contain[n] = dict(n_hull=len(H), n_mesh=len(V), inside=float((d.find_simplex(V, tol=1e-9) >= 0).mean()),
                      qhull_vs_coal_vol=float(ConvexHull(V).volume / ConvexHull(H).volume))
bad = {n: v for n, v in contain.items() if v['inside'] < 1.0 or abs(v['qhull_vs_coal_vol'] - 1) > 1e-3}
print(f'geometri hull: {len(contain)}; tidak memuat semua verteks mesh / volume beda > 0.1 %: {len(bad)}')
for n, v in list(bad.items())[:8]:
    print('   ', n, v)

rows = []
for (dx, r1, r2) in cfg:
    l1, l2 = M.lins(dx)
    q = c.q_from(G.rest_state(l1, r1, l2, r2))
    c.geom_data = c.geom.createData()
    pin.updateGeometryPlacements(c.model, c.data, c.geom, c.geom_data, q)
    pin.computeDistances(c.model, c.data, c.geom, c.geom_data, q)
    md = mg.createData()
    pin.updateGeometryPlacements(c.model, c.data, mg, md, q)
    pin.computeDistances(c.model, c.data, mg, md, q)
    for k in range(c.n_arm):
        m = md.distanceResults[k].min_distance
        if m > 0.15:
            continue
        cp = c.geom.collisionPairs[k]
        h = c.geom_data.distanceResults[k].min_distance
        HA, HB = pts_hull(cp.first), pts_hull(cp.second)
        hv = cKDTree(world(HB, c.geom_data.oMg[cp.second])).query(world(HA, c.geom_data.oMg[cp.first]))[0].min() \
            if HA is not None and HB is not None else np.nan
        rows.append(dict(cfg=(dx, r1, r2), pair=(c.names[cp.first], c.names[cp.second]), h=h, m=m, hv=hv))
R = rows
e = np.array([r['h'] - r['m'] for r in R])
g = np.array([r['h'] - r['hv'] for r in R])
print(f'pasangan (mesh < 150 mm) pada 50 konfigurasi: {len(R)}; hull - mesh: maks {np.nanmax(e)*1000:+.2f} mm, '
      f'> 0.5 mm pada {int((e > 5e-4).sum())}; hull GJK - hull vert-vert (harus <= 0): maks {np.nanmax(g)*1000:+.2f} mm, '
      f'> 0.5 mm pada {int((g > 5e-4).sum())}')
for r in sorted(R, key=lambda r: r['m'] - r['h'])[:6]:
    print(f"   h {r['h']*1000:7.2f}  m {r['m']*1000:7.2f}  hv {r['hv']*1000:7.2f}  {r['pair']}")
mins = {}
for r in R:
    k = r['cfg']
    a = mins.setdefault(k, [np.inf, np.inf])
    a[0], a[1] = min(a[0], r['h']), min(a[1], r['m'])
d = np.array([v[0] - v[1] for v in mins.values()])
print(f'min per konfigurasi: hull - mesh maks {d.max()*1000:+.2f}, rerata {d.mean()*1000:+.2f} mm')
json.dump(dict(containment_bad=bad, rows=[dict(r, cfg=list(r['cfg'])) for r in R]),
          open('g29_hull_diag.json', 'w'), indent=1, default=float)
