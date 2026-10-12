# Synthetic: which height sets still recover the extrinsic? (scratch, not a locked test)
import sys; sys.path.insert(0,'.')
import numpy as np, cam6dof_fit as F
from scipy.spatial.transform import Rotation
def synth(old, marks, holdout, arm_xy, wdeg, tau, s, seed=1, nb=0.005, na=0.003):
    rng=np.random.default_rng(seed); old=np.array(old); R0,t0=F.R_of(old),old[:3]
    Rt=R0@Rotation.from_rotvec(np.radians(wdeg)).as_matrix(); tt=t0+tau; obs=[]
    for mk,(x,y,zs) in marks.items():
        for hz in zs:
            c=np.r_[x,y,hz]+F.centroid_offset(t0,np.r_[x,y,hz],.03,.07); pc=Rt.T@(c-tt)/s+rng.normal(0,nb,3)
            obs.append(dict(name=f'{mk}@{hz}',mark=mk,xy=[x,y],surf_z=hz,track=list(R0@pc+t0),role='holdout' if mk in holdout else 'fit'))
    surf,nrms,axis,Pc={},{},{},[]
    for arm,(ax,ay) in arm_xy.items():
        th=np.linspace(0,2*np.pi,120,endpoint=False); z=np.arange(1.0,1.6,0.005); T,Z=np.meshgrid(th,z)
        S=np.c_[ax+.045*np.cos(T).ravel(),ay+.045*np.sin(T).ravel(),Z.ravel()]; N=np.c_[np.cos(T).ravel(),np.sin(T).ravel(),0*T.ravel()]
        surf[arm],nrms[arm],axis[arm]=S,N,np.array([ax,ay]); vis=((tt-S)*N).sum(1)>0
        Pc.append((S[vis]-tt)@Rt/s+rng.normal(0,na,(vis.sum(),3)))
    arms=F.arm_sets(np.concatenate(Pc),R0,t0,surf,nrms,axis,'')
    return dict(ns='x',old=old.tolist(),bottle=dict(r=.03,H=.07),obs=obs,_arms=arms),Rt,tt
def report(lab,cfg,Rt,tt):
    rep=F.run(cfg,verbose=False); out=[]
    for m in ('M6','M7'):
        d=rep['models'][m]; er=np.degrees(Rotation.from_matrix(Rotation.from_quat(d['quat_xyzw']).as_matrix()@Rt.T).magnitude())
        ho=max(r['res3'] for r in d['rows'].values() if r['role']=='holdout'); out.append(f"{m} rot {er:.2f}° pos {1000*np.linalg.norm(np.array(d['new'][:3])-tt):.0f} mm HO {1000*ho:.0f} mm s {d['s']}")
    print(lab,'|',' | '.join(out),'| select',rep['select'],flush=True)
R2=[-0.1547,-1.0197,2.0905,-2.2369,-0.1285,-0.9189]; R1=[1.7166,0.8656,2.0782,-2.3436,-0.1488,2.1688]
tab=dict(M1=(1.7497,0.5493),C5=(1.6506,0.6503),N1=(1.9389,0.4984),N2=(1.9388,0.6806),C6=(1.9004,0.6004),C3=(1.8486,0.6786))
flo=dict(F1=(0.25,-0.50),F6=(0.40,-0.70),F4=(0.55,-0.60),F5=(0.65,-0.48),F3=(0.25,-0.60),F7=(0.65,-0.58))
a2=dict(arm_1=(1.8993,0.36),arm_2=(1.0993,0.36),arm_3=(0.40,-0.36)); a1=dict(arm_2=(1.0993,0.36),arm_4=(-0.40,-0.36))
for s in (1.0,1.04):
  for lab,hs in (('3 tinggi',(0,.15,.35)),('1 tinggi',(0,))):
    # rgbd2: table (+heights) + floor marks it sees
    m={k:(x,y,tuple(.77+h for h in hs)) for k,(x,y) in tab.items()}; m.update({k:(x,y,hs) for k,(x,y) in flo.items() if k in ('F4','F5','F7')})
    cfg,Rt,tt=synth(R2,m,('C6','C3','F7'),a2,[2.5,0,0],np.array([.02,-.01,.01]),s); report(f'rgbd2 s{s} meja+lantai {lab}',cfg,Rt,tt)
    m={k:(x,y,hs) for k,(x,y) in flo.items()}
    cfg,Rt,tt=synth(R1,m,('F3','F7'),a1,[2.5,0,0],np.array([.02,-.01,.01]),s); report(f'rgbd  s{s} lantai {lab}',cfg,Rt,tt)
