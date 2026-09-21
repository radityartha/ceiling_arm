import sys, numpy as np
sys.path.insert(0,'/home/user1/Documents/ceiling_arm/scripts')
import reach_dwell_probe as P
from interarm_collision import CrossGantryChecker, _ik
from types import SimpleNamespace as NS
REST=[-0.46352,0.10710,0.12916,-1.38653,-0.17648,1.73885]
class FakeNode:
    def wait_joints(self,w): raise AssertionError('must not read /joint_states')
    def get_logger(self): return NS(error=print, info=print)
node=FakeNode()
c=CrossGantryChecker(); m,d=c.model,c.data
def base(l1=0.55,l2=0.0):
    js={'t1_linear_joint':l1,'t2_linear_joint':l2,'t1_rotation_joint':0.0,'t2_rotation_joint':0.0}
    for p in ['t1_a1_','t1_a2_','t2_a1_','t2_a2_']: js.update({f'{p}joint_{i}':v for i,v in enumerate(REST,1)})
    return js
def joints_at(pfx,xyz,js):
    q,ok=_ik(m,d,pfx,np.array(xyz),c.q_from(js)); assert ok
    return {f'{pfx}joint_{i}':float(q[m.joints[m.getJointId(f"{pfx}joint_{i}")].idx_q]) for i in range(1,7)}
def traj(pfx,jd):
    names=[f'{pfx}joint_{i}' for i in range(1,7)]
    return NS(joint_trajectory=NS(joint_names=names,points=[NS(positions=[jd[n] for n in names])]))
mid=[0.45,0.0,1.30]
ok=True
# V1 arm_3 moves to mid; arm_1 at REST vs PLACED at mid
b=base(); a3=joints_at('t2_a1_',mid,b)
r_rest=P.screen_interarm(traj('t2_a1_',a3),'arm_3',node,['arm_1','arm_2','arm_4'],other_joints=b)
bp=dict(b); bp.update(joints_at('t1_a1_',mid,b))
r_pl=P.screen_interarm(traj('t2_a1_',a3),'arm_3',node,['arm_1','arm_2','arm_4'],other_joints=bp)
print('V1 arm_3->mid, arm_1 REST  :',r_rest[0],round(r_rest[1]*1000,1),r_rest[2]); ok&=r_rest[0]=='CLEAR'
print('V1 arm_3->mid, arm_1 PLACED:',r_pl[0],round(r_pl[1]*1000,1),r_pl[2]); ok&=r_pl[0]=='COLLIDE'
# V2 only-filter: arm_2 and arm_4 collide with each other; arm_1 moves (stays at REST)
b2=base(); pt=[0.0,0.0,1.30]
b2.update(joints_at('t1_a2_',pt,b2)); b2.update(joints_at('t2_a2_',pt,b2))
dall,_=c.check(c.q_from(b2)); print('V2 arm_2 x arm_4 static, unfiltered: %.1f mm'%(dall*1000)); ok&=dall<=0
a1={f't1_a1_joint_{i}':v for i,v in enumerate(REST,1)}
r=P.screen_interarm(traj('t1_a1_',a1),'arm_1',node,['arm_2','arm_3','arm_4'],other_joints=b2)
print('V2 arm_1 moving, filtered to arm_1 pairs:',r[0],round(r[1]*1000,1),r[2]); ok&=r[0]=='CLEAR'
# V3 legacy dual regression = g19 V0: arm_2 -> (L,0.36,1.40), arm_1 REST vs placed
for L in (0.55,0.95):
    b3=base(l1=L); x=[L,0.36,1.40]; a2=joints_at('t1_a2_',x,b3)
    r1=P.screen_interarm(traj('t1_a2_',a2),'arm_2',node,'arm_1',other_joints=b3)
    b3p=dict(b3); b3p.update(joints_at('t1_a1_',x,b3))
    r2=P.screen_interarm(traj('t1_a2_',a2),'arm_2',node,'arm_1',other_joints=b3p)
    print(f'V3 L={L} dual REST {r1[0]} {r1[1]*1000:.1f} | placed {r2[0]} {r2[1]*1000:.1f}'); ok&=r1[0]=='CLEAR' and r2[0]=='COLLIDE'
# V4 incomplete placement -> REFUSE (None)
bi=base(); del bi['t2_linear_joint']
r=P.screen_interarm(traj('t1_a1_',a1),'arm_1',node,['arm_2','arm_3','arm_4'],other_joints=bi)
print('V4 missing t2_linear ->',r); ok&=r is None
print('\nVALIDASI:', 'LULUS' if ok else 'GAGAL')
