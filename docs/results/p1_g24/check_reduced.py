"""G24 A1 check: reduced model == full G23 model (FK tool_frame + 6-joint gravity), 100 random configs per arm."""
import numpy as np
import oracle2 as O2
import oracle as O
import pinocchio as pin

full = pin.buildModelFromUrdf(O.URDF); fd = full.createData()
rng = np.random.default_rng(99)
worst = 0.0
for arm in O.ARMS:
    A = O2.model(arm)
    fid = full.getFrameId(f'{O.PREFIX[arm]}tool_frame')
    ids = [full.getJointId(f'{O.PREFIX[arm]}joint_{i}') for i in range(1, 7)]
    fiq = [full.joints[j].idx_q for j in ids]; fiv = [full.joints[j].idx_v for j in ids]
    frail = full.joints[full.getJointId(f't{O.GANTRY[arm]}_linear_joint')].idx_q
    for _ in range(100):
        q6 = rng.uniform(A['lo'], A['hi']); rail = rng.uniform(0, 1.6)
        qf = O2.q_ref_full(full, pin); qf[fiq] = q6; qf[frail] = rail
        pin.framesForwardKinematics(full, fd, qf)
        Tf = fd.oMf[fid]; gf = np.abs(pin.computeGeneralizedGravity(full, fd, qf)[fiv])
        q = np.zeros(A['m'].nq); q[A['rail']] = rail; q[A['iq']] = q6
        pin.framesForwardKinematics(A['m'], A['d'], q)
        Tr = A['d'].oMf[A['tool']]
        e = max(np.abs(Tf.homogeneous - Tr.homogeneous).max(), np.abs(gf - O2.gravity(arm, q6, rail)).max())
        worst = max(worst, e)
print(f'reduced vs full, 400 configs: max |diff| = {worst:.3e}  ->', 'OK' if worst <= 1e-9 else 'GAGAL')
