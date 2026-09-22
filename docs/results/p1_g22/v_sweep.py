"""G22 S24 validation, OFFLINE (URDF /tmp/reach_dwell_live.urdf), zero motion.
X0 REST x REST sweep must be CLEAR (g20 X0-A: 546.1 mm).
X1 NEGATIVE: arm_1 IK'd onto t2_a1_arm_link as it hangs at g2 rail 0.25; the
   sweep 0.00 -> 0.45 has BOTH endpoints clear (>= 50 mm) and must still
   COLLIDE mid-path -- i.e. the screen is a sweep, not an endpoint check.
X2 the same two endpoints checked alone must be CLEAR (proves X1 needs the sweep)."""
import sys; sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
import warnings; warnings.filterwarnings('ignore')
import pinocchio as pin
import g22_plan as P
from interarm_collision import CrossGantryChecker, _ik
c = CrossGantryChecker()
st = {'t1_linear_joint': 0.55, 't2_linear_joint': 0.0, 't1_rotation_joint': 0, 't2_rotation_joint': 0}
for a, p in P.PREFIX.items(): st.update({f'{p}joint_{i}': v for i, v in enumerate(P.REST, 1)})
ok = True
v = P.sweep_screen(c, 2, 0.0, 0.45, st); print('X0 REST sweep g2 0->0.45:', v[0], f'{v[1]*1000:.1f} mm'); ok &= v[0] == 'CLEAR'
s2 = dict(st); s2['t2_linear_joint'] = 0.25
q = c.q_from(s2); pin.forwardKinematics(c.model, c.data, q); pin.updateFramePlacements(c.model, c.data)
w = c.data.oMf[c.model.getFrameId('t2_a1_arm_link')].translation.copy()
qa, conv = _ik(c.model, c.data, 't1_a1_', w, c.q_from(st)); assert conv, 'IK'
placed = dict(st); placed.update({f't1_a1_joint_{i}': qa[c.model.joints[c.model.getJointId(f't1_a1_joint_{i}')].idx_q] for i in range(1, 7)})
v = P.sweep_screen(c, 2, 0.0, 0.45, placed); print('X1 NEG sweep g2 0->0.45:', v[0], f'{v[1]*1000:.1f} mm at k={v[3]}', v[2]); ok &= v[0] == 'COLLIDE'
for x in (0.0, 0.45):
    v = P.sweep_screen(c, 2, x, x, placed); print(f'X2 endpoint {x:.2f} alone:', v[0], f'{v[1]*1000:.1f} mm'); ok &= v[0] == 'CLEAR'
print('V_SWEEP', 'LULUS' if ok else 'GAGAL'); sys.exit(0 if ok else 1)
