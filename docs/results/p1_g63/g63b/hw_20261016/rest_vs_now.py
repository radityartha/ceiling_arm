import sys, os, json, tempfile, numpy as np
G='/home/user1/Documents/ceiling_arm/docs/results/p1_g63'
sys.path.insert(0,G); sys.path.insert(0,'/home/user1/Documents/ceiling_arm/scripts')
import g63_poses as P
mount=json.load(open(G+'/g63b/mount_g63b.json'))
pay=os.path.join(tempfile.mkdtemp(),'p.urdf')
P.g63_urdf.build(G+'/live_urdf_20261011.urdf', mount, pay); P.g63_screens.install(pay)
cell=P.Cell(pay,1.50,0.0); cell.mount=mount
k=json.load(open(G+'/g63b/hw_20261016/k0_1_rail150.json'))['rows']
now=[k[f't1_a2_joint_{i}']['median'] for i in range(1,7)]
for name,q in (('REST',list(P.REST)),('SEKARANG',now)):
    for r in (1.50,1.30):
        cell.base['t1_linear_joint']=r
        ok,info=P.collisions(cell,'arm_2',cell.joints('arm_2',q))
        oMg=cell.env.place(cell.env.q_from(cell.joints('arm_2',q)))
        ib=cell.env.names.index('t1_a2_g63_board_0')
        print(f"{name:8s} g1 {r:.2f}: env {info['env_m']*1000:.1f} mm ({info['worst']['env']}), kotak MoveIt {info['moveit_env_box_hit']}, pusat kotak papan z {oMg[ib].translation[2]:.4f}")
cell.base['t1_linear_joint']=1.50
qa,qb=np.array(now),np.array(P.REST)
for s in np.linspace(0,1,11):
    q=list(qa+(qb-qa)*s); ok,info=P.collisions(cell,'arm_2',cell.joints('arm_2',q))
    print(f"lintasan s={s:.1f}: env papan {info['env_m']*1000:.1f} mm, same {info['same_m']*1000:.0f}, cross {info['cross_m']*1000:.0f}, self {info['self_m']*1000:.0f}")
