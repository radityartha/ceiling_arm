import sys, os, json, tempfile, numpy as np
G='/home/user1/Documents/ceiling_arm/docs/results/p1_g63'
sys.path.insert(0,G); sys.path.insert(0,'/home/user1/Documents/ceiling_arm/scripts')
import g63_poses as P
mount=json.load(open(G+'/g63b/mount_g63b.json'))
pay=os.path.join(tempfile.mkdtemp(),'p.urdf')
P.g63_urdf.build(G+'/live_urdf_20261011.urdf', mount, pay)
P.g63_screens.install(pay)
cell=P.Cell(pay,1.50,0.0); cell.mount=mount
arm='arm_2'
for r in np.round(np.arange(1.50,1.2999,-0.01),3):
    cell.base['t1_linear_joint']=float(r)
    ok,info=P.collisions(cell,arm,cell.joints(arm,list(P.REST)))
    print(f"g1 {r:.2f} ok={ok} env={info['env_m']*1000:6.1f}mm ({info['worst']['env']}) box_hit={info['moveit_env_box_hit']} same={info['same_m']*1000:.0f} cross={info['cross_m']*1000:.0f} self={info['self_m']*1000:.0f}",flush=True)
