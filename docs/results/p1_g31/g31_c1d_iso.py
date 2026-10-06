"""K-C1d isolated (post-lock supplement): hull-of-hull vs hull with BOTH fresh, 2000 N1 states (rng 29)."""
import os, sys, warnings
import numpy as np
warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../../scripts')); sys.path.insert(0, os.path.join(HERE, '../p1_g29'))
import interarm_collision as IC, g29_rot_screen as R
pat, rot = IC.CrossGantryChecker(), R.RotCrossChecker()
rng = np.random.default_rng(29)
names = R.arm_joint_names()
iq = {n: pat.model.joints[pat.model.getJointId(n)].idx_q for n in names}
dd, same_pair = [], 0
for k in range(2000):
    s = {'t1_linear_joint': rng.uniform(0, 1.6), 't2_linear_joint': rng.uniform(0, 1.6)}
    s['t1_rotation_joint'] = 0.0 if k < 500 else rng.uniform(-np.pi, np.pi)
    s['t2_rotation_joint'] = 0.0 if k < 500 else rng.uniform(-np.pi, np.pi)
    for n in names:
        s[n] = rng.uniform(pat.model.lowerPositionLimit[iq[n]], pat.model.upperPositionLimit[iq[n]])
    pat.geom_data = pat.geom.createData()
    dp, pp = pat.check(pat.q_from(s))
    dr, pr, _, _ = rot.check_split(rot.q_from(s))
    dd.append(dr - dp); same_pair += pp == pr
dd = np.abs(dd) * 1000
print(f'K-C1d terisolasi (keduanya segar): maks |hull-dari-hull - hull| {dd.max():.3e} mm, bit-identik {int((dd == 0).sum())}/2000, pasangan sama {same_pair}/2000')
