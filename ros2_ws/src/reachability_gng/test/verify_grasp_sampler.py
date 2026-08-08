"""Geometric verification of grasp_sampler's core against KNOWN dimensions.

    python3 ros2_ws/src/reachability_gng/test/verify_grasp_sampler.py

Deliberately NOT a pytest case (this repo has no formal suite) and deliberately
not an RViz check: a grasp sampler fails silently -- wrong geometry still
produces a tidy arrow that looks entirely reasonable on screen, and the only way
to catch it is to hold the output against dimensions known in advance.
Every fault found while building this node was found here, not by looking.


Synthesises what a wrist depth camera actually returns for a can and a box
standing on a table -- only the camera-facing surface, sampled with probability
proportional to projected area (so grazing bands come out honestly sparse), plus
1 mm range noise -- then checks the top-ranked grasp against the object's true
dimensions rather than against how it looks in RViz.

Objects use published YCB dimensions:
  tomato_soup_can (005)  diameter 0.0677 m, height 0.101 m
  pick cube (polish.add_pick_cube)  0.060 m on a side
  a 0.06 x 0.10 x 0.15 box, so the ONLY graspable width (<= 0.085 m pad gap) is
  the 0.060 m side -- a sampler that picks the wrong axis fails loudly here
"""
import os
import sys

import numpy as np

# Import the geometry core straight from the source tree, so this runs against
# the working copy without needing the workspace rebuilt first.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from reachability_gng.grasp_sampler import (  # noqa: E402
    DEFAULTS, grasp_frame, sample_grasps_from_cloud)

RNG = np.random.default_rng(7)
TABLE_Z = 1.10


def _visible(pts, normals, cam, keep_frac=1.0):
    """Keep only camera-facing samples, with density ~ projected area / range^2
    (what a pinhole depth sensor actually delivers), then add 1 mm range noise."""
    v = cam[None, :] - pts
    d = np.linalg.norm(v, axis=1)
    u = v / d[:, None]
    cos = np.einsum('ij,ij->i', normals, u)
    vis = cos > np.cos(np.radians(85.0))          # grazing cutoff
    w = np.zeros(len(pts))
    w[vis] = cos[vis] / d[vis] ** 2
    if w.sum() <= 0:
        return np.zeros((0, 3))
    p = w / w.sum()
    n = int(keep_frac * vis.sum())
    idx = RNG.choice(len(pts), size=n, replace=True, p=p)
    out = pts[idx]
    out = out + RNG.normal(0.0, 0.001, out.shape)  # 1 mm range noise
    return out


def make_table(cx, cy, cam, half=0.30, n=40000):
    p = np.column_stack([RNG.uniform(cx - half, cx + half, n),
                         RNG.uniform(cy - half, cy + half, n),
                         np.full(n, TABLE_Z)])
    nrm = np.tile([0.0, 0.0, 1.0], (n, 1))
    return _visible(p, nrm, cam, 0.25)


def make_can(cx, cy, cam, radius=0.03385, height=0.101, n=60000):
    """Upright cylinder: side wall + top disc."""
    ns = int(n * 0.75)
    th = RNG.uniform(0, 2 * np.pi, ns)
    z = RNG.uniform(TABLE_Z, TABLE_Z + height, ns)
    side = np.column_stack([cx + radius * np.cos(th), cy + radius * np.sin(th), z])
    side_n = np.column_stack([np.cos(th), np.sin(th), np.zeros(ns)])

    nt = n - ns
    r = radius * np.sqrt(RNG.uniform(0, 1, nt))
    th2 = RNG.uniform(0, 2 * np.pi, nt)
    top = np.column_stack([cx + r * np.cos(th2), cy + r * np.sin(th2),
                           np.full(nt, TABLE_Z + height)])
    top_n = np.tile([0.0, 0.0, 1.0], (nt, 1))
    return _visible(np.vstack([side, top]), np.vstack([side_n, top_n]), cam)


def make_box(cx, cy, cam, dims=(0.06, 0.10, 0.15), n=60000):
    """Axis-aligned box standing on the table; dims = (x, y, z) full lengths."""
    dx, dy, dz = dims
    hx, hy = dx / 2, dy / 2
    faces, norms = [], []
    per = n // 5
    for sign in (-1, 1):                       # +-X faces
        p = np.column_stack([np.full(per, cx + sign * hx),
                             RNG.uniform(cy - hy, cy + hy, per),
                             RNG.uniform(TABLE_Z, TABLE_Z + dz, per)])
        faces.append(p)
        norms.append(np.tile([sign, 0.0, 0.0], (per, 1)))
    for sign in (-1, 1):                       # +-Y faces
        p = np.column_stack([RNG.uniform(cx - hx, cx + hx, per),
                             np.full(per, cy + sign * hy),
                             RNG.uniform(TABLE_Z, TABLE_Z + dz, per)])
        faces.append(p)
        norms.append(np.tile([0.0, sign, 0.0], (per, 1)))
    p = np.column_stack([RNG.uniform(cx - hx, cx + hx, per),      # top
                         RNG.uniform(cy - hy, cy + hy, per),
                         np.full(per, TABLE_Z + dz)])
    faces.append(p)
    norms.append(np.tile([0.0, 0.0, 1.0], (per, 1)))
    return _visible(np.vstack(faces), np.vstack(norms), cam)


def run(name, cloud, obj_xyz, cam, checks, cfg=None):
    cfg = dict(DEFAULTS) if cfg is None else cfg
    out = sample_grasps_from_cloud(cloud, obj_xyz, cam, cfg)
    print(f'\n=== {name} ===')
    print(f'  input {len(cloud)} pts | cluster {out["n_cluster_points"]} '
          f'| plane dropped {out["n_support_filtered"]}')
    print(f'  extent (PCA) = {np.round(out["extent"], 4)}')
    print(f'  {out["message"]}')
    if not out['grasps']:
        print('  RESULT: no grasps -> FAIL')
        return False
    for i, g in enumerate(out['grasps'][:3]):
        print(f'  #{i} width={g["width"]:.4f} anti={g["antipodal"]:.3f} '
              f'grav={g["gravity"]:.3f} score={g["score"]:.3f} '
              f'both_sides={g["both_sides_observed"]} '
              f'axis={np.round(g["axis"], 3)} centre={np.round(g["center"], 4)}')
    top = out['grasps'][0]
    ok = True
    for label, cond, detail in checks(top, out):
        print(f'    [{"PASS" if cond else "FAIL"}] {label}: {detail}')
        ok &= bool(cond)
    return ok


def main():
    results = {}

    # ---- CAN: tomato_soup_can, true diameter 0.0677 m ----------------------
    cx, cy = 1.0, 0.0
    cam = np.array([cx + 0.06, cy, TABLE_Z + 0.101 + 0.24])   # near-nadir wrist
    obj = np.array([cx, cy, TABLE_Z + 0.0505])
    cloud = np.vstack([make_table(cx, cy, cam), make_can(cx, cy, cam)])

    def can_checks(top, out):
        d = 0.0677
        # A single view sees a can's near half plus grazing bands, so the widest
        # observable chord is a hair under the true diameter -- allow that, but
        # not a chord from somewhere else on the cylinder.
        yield ('width vs true diameter 0.0677 m',
               abs(top['width'] - d) <= 0.006,
               f'{top["width"]:.4f} m, err {1000*(top["width"]-d):+.1f} mm')
        yield ('closing axis horizontal (perp. to can axis)',
               abs(top['axis'][2]) <= 0.15,
               f'axis z-component {top["axis"][2]:+.3f}')
        radial = np.linalg.norm(top['center'][:2] - np.array([cx, cy]))
        yield ('grasp centred on the can axis',
               radial <= 0.010, f'{1000*radial:.1f} mm off axis')
        yield ('grasp height within the can body',
               TABLE_Z < top['center'][2] < TABLE_Z + 0.101,
               f'z={top["center"][2]:.4f} (can spans '
               f'{TABLE_Z:.3f}-{TABLE_Z+0.101:.3f})')
    results['can'] = run('CAN  tomato_soup_can  D=0.0677 H=0.101',
                         cloud, obj, cam, can_checks)

    # ---- BOX: 0.06 x 0.10 x 0.15, only the 0.06 axis fits the pads ---------
    cx, cy = 1.0, 0.0
    dims = (0.06, 0.10, 0.15)
    cam = np.array([cx + 0.06, cy, TABLE_Z + dims[2] + 0.24])
    obj = np.array([cx, cy, TABLE_Z + dims[2] / 2])
    cloud = np.vstack([make_table(cx, cy, cam), make_box(cx, cy, cam, dims)])

    def box_checks(top, out):
        yield ('width vs true short side 0.060 m',
               abs(top['width'] - 0.060) <= 0.005,
               f'{top["width"]:.4f} m, err {1000*(top["width"]-0.060):+.1f} mm')
        yield ('closes across X (the 0.06 axis), not Y (0.10, too wide)',
               abs(top['axis'][0]) >= 0.95,
               f'axis={np.round(top["axis"], 3)}')
        off = np.linalg.norm(top['center'][:2] - np.array([cx, cy]))
        yield ('grasp centred on the box', off <= 0.015,
               f'{1000*off:.1f} mm off centre')
    results['box'] = run(f'BOX  {dims[0]}x{dims[1]}x{dims[2]} m',
                         cloud, obj, cam, box_checks)

    # ---- CUBE: the live scene's 0.06 m pick cube ---------------------------
    cx, cy = 1.0, 0.0
    dims = (0.06, 0.06, 0.06)
    cam = np.array([cx + 0.06, cy, TABLE_Z + 0.06 + 0.24])
    obj = np.array([cx, cy, TABLE_Z + 0.03])
    cloud = np.vstack([make_table(cx, cy, cam), make_box(cx, cy, cam, dims)])

    def cube_checks(top, out):
        yield ('width vs true 0.060 m side',
               abs(top['width'] - 0.060) <= 0.005,
               f'{top["width"]:.4f} m, err {1000*(top["width"]-0.060):+.1f} mm')
        yield ('closing axis horizontal', abs(top['axis'][2]) <= 0.15,
               f'axis z-component {top["axis"][2]:+.3f}')
    results['cube'] = run('CUBE  0.06 m (polish.add_pick_cube)',
                          cloud, obj, cam, cube_checks)

    # ---- TWO OBLIQUE VIEWS: both_sides_observed must actually flip true -----
    # The flag is the tier-2/3 escalation trigger, so a flag that is always
    # false is not a conservative default, it is a broken signal: it would send
    # the arm off for extra looks that can never satisfy it. Merging two oblique
    # views 180 deg apart in azimuth genuinely images both sides of the can, so
    # the top candidate must come back true here while the nadir case above
    # comes back false.
    cx, cy = 1.0, 0.0
    top_z = TABLE_Z + 0.101
    obj = np.array([cx, cy, TABLE_Z + 0.0505])
    tilt, dist = np.radians(38.0), 0.25
    clouds, cams = [], []
    for az in (0.0, np.pi):
        cam = obj + dist * np.array([np.sin(tilt) * np.cos(az),
                                     np.sin(tilt) * np.sin(az), np.cos(tilt)])
        cams.append(cam)
        clouds.append(np.vstack([make_table(cx, cy, cam), make_can(cx, cy, cam)]))
    cloud = np.vstack(clouds)

    def two_view_checks(top, out):
        d = 0.0677
        yield ('width vs true diameter 0.0677 m',
               abs(top['width'] - d) <= 0.006,
               f'{top["width"]:.4f} m, err {1000*(top["width"]-d):+.1f} mm')
        yield ('both_sides_observed flips TRUE with both sides imaged',
               top['both_sides_observed'], f'{top["both_sides_observed"]}')
        yield ('antipodality is now a real measurement',
               top['antipodal'] >= 0.85, f'{top["antipodal"]:.3f}')
    results["can_2view"] = run("CAN, TWO OBLIQUE VIEWS (38 deg, az 0 and 180)",
                               cloud, obj, np.array(cams), two_view_checks)

    # ---- grasp_frame sanity: is the emitted pose a valid gripper frame? ----
    print('\n=== grasp_frame algebra ===')
    ok = True
    for axis in ([1, 0, 0], [0, 1, 0], [0.6, 0.8, 0], [0, 0, 1]):
        c = np.array([1.0, 2.0, 1.2])
        p, R = grasp_frame(c, np.array(axis, float), DEFAULTS['finger_offset'])
        orth = np.abs(R.T @ R - np.eye(3)).max()
        det = np.linalg.det(R)
        ax = np.array(axis, float) / np.linalg.norm(axis)
        y_is_axis = abs(abs(float(R[:, 1] @ ax)) - 1.0) < 1e-9
        # local +Z must carry the gripper base to the finger midpoint at c
        recon = np.linalg.norm(p + DEFAULTS['finger_offset'] * R[:, 2] - c)
        good = orth < 1e-9 and abs(det - 1) < 1e-9 and y_is_axis and recon < 1e-9
        ok &= good
        print(f'  axis={axis}: orth_err={orth:.2e} det={det:.6f} '
              f'localY==axis={y_is_axis} fingertip_err={recon:.2e} '
              f'-> {"PASS" if good else "FAIL"}')
    results['grasp_frame'] = ok

    print('\n================ SUMMARY ================')
    for k, v in results.items():
        print(f'  {k:12s} {"PASS" if v else "FAIL"}')
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
