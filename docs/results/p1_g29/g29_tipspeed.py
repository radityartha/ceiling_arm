"""G29 A5 (c): arm speed at 10 deg/s gantry rotation -- TABLE for the operator, no decision. OFFLINE."""
import json, os, sys
import numpy as np
import pinocchio as pin
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g29_rot_screen as G
for p in ('../p1_g22', '../p1_g23', '../p1_g24'):
    sys.path.insert(0, os.path.join(HERE, p))
import make_instance_g24 as MI4

W = np.radians(10.0)
c = G.RotCrossChecker()
q = c.q_from(G.rest_state(0.55, 0.0, 0.0, 0.0))
pin.framesForwardKinematics(c.model, c.data, q)
rows = []
for pfx, g, lin in (('t1_a1_', 1, 0.55), ('t1_a2_', 1, 0.55), ('t2_a1_', 2, 0.0), ('t2_a2_', 2, 0.0)):
    p = c.data.oMf[c.model.getFrameId(f'{pfx}tool_frame')].translation
    rows.append(np.hypot(p[0] - lin, p[1] - (0.36 if g == 1 else -0.36)))
r_tool_rest = max(rows)
a0 = json.load(open(os.path.join(HERE, 'g29_a0.json')))
r_hull_rest = max(v[0] for k, v in a0['radius_z'].items() if k.startswith(G.ARM))
REF = MI4.REF
cache = MI4.load_cache()
rr = []
for (n, g, p, s), d in cache.items():
    if d['ok']:
        x = REF.nodes[n]
        rr.append((np.hypot(x[0] - REF.lin[p], x[1] - (0.36 if g == 1 else -0.36)), n, g, p, s))
rr.sort()
r_ok_max = rr[-1]
extra = {}
f = os.path.join(HERE, 'g29_oracle_rot_cache.jsonl')
if os.path.exists(f):
    for ln in open(f):
        d = json.loads(ln)
        if d['ok3']:
            n, g, p, ri, s = d['key']
            x = REF.nodes[n]
            extra[tuple(d['key'])] = np.hypot(x[0] - REF.lin[p], x[1] - (0.36 if g == 1 else -0.36))
r_rot_max = max(extra.values()) if extra else float('nan')
REACH = 1.005                                      # base -> tool_frame, interarm_collision docstring
tab = [('tool_frame, REST', r_tool_rest), ('titik hull lengan terjauh, REST', r_hull_rest),
       ("tool_frame, tuple layak-oracle''' terjauh (cache rot 0, 1510 tuple)", r_ok_max[0]),
       ("tool_frame, tuple layak-oracle''' terjauh (sampel G29 rot != 0)", r_rot_max),
       ('teoretis z = 1.40 (0.40 + sqrt(1.005^2 - 0.5525^2))', 0.40 + np.sqrt(REACH ** 2 - 0.5525 ** 2)),
       ('teoretis terentang horizontal (0.40 + 1.005)', 0.40 + REACH)]
print(f'omega = 10 deg/s = {W:.4f} rad/s; v = omega * r_h')
out = []
for name, r in tab:
    print(f'  {name:62s} r_h {r*1000:7.1f} mm  v {W*r*1000:6.1f} mm/s')
    out.append(dict(what=name, r_h_m=float(r), v_mm_s=float(W * r * 1000)))
print(f'  (tuple rot 0 terjauh: node {r_ok_max[1]} g{r_ok_max[2]} rel {REF.lin[r_ok_max[3]]:.2f} slot {r_ok_max[4]})')
json.dump(out, open(os.path.join(HERE, 'g29_tipspeed.json'), 'w'), indent=1)
