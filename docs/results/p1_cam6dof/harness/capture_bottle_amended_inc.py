#!/usr/bin/env python3
"""Incumbent floor bottle capture after the locked floor-support amendment.

Usage (source ROS/workspace, ROS_DOMAIN_ID=0): <this script> F1 BASE
Operator placement/readiness is required before each invocation. NO motion.
Keeps the original track_median selector/radius, ranking and rival reporting.
"""

import hashlib,json,subprocess,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,'docs/results/p1_cam6dof')
from cam6dof_fit import centroid_offset
mark,level=sys.argv[1:3]
assert mark in ('F1','F6','F4','F5','F3','F7') and level in ('BASE','D','CD')
b=Path('docs/results/p1_cam6dof/hw_20261014')
q=Path('docs/results/p1_cam6dof/cam6dof_A_locked.md')
assert hashlib.sha256(q.read_bytes()).hexdigest()==q.with_suffix('.sha256').read_text().split()[0]
amend=Path('docs/results/p1_cam6dof/cam6dof_A_floor_amendment_locked.md')
assert hashlib.sha256(amend.read_bytes()).hexdigest()==amend.with_suffix('.sha256').read_text().split()[0]
protocol_hashes={'original':hashlib.sha256(q.read_bytes()).hexdigest(),'floor_amendment':hashlib.sha256(amend.read_bytes()).hexdigest()}
correction_state=b/'bottle_height_correction_pending.json'
if correction_state.exists():
    correction=json.loads(correction_state.read_text())
    assert correction.get('capture_permitted') is True, 'Bottle dimensions correction pending; capture blocked'
    geometry=Path('docs/results/p1_cam6dof/cam6dof_A_bottle_geometry_locked.md')
    geometry_hash=hashlib.sha256(geometry.read_bytes()).hexdigest()
    assert geometry_hash==geometry.with_suffix('.sha256').read_text().split()[0]
    protocol_hashes['bottle_geometry_correction']=geometry_hash
plans={ns:json.loads((b/f'plan_{ns}_inc.json').read_text()) for ns in ('rgbd','rgbd2')}
assert len({(p['bottle']['r'],p['bottle']['H']) for p in plans.values()})==1, 'Per-camera bottle dimensions differ'
xy=next(p['marks'][mark] for p in plans.values() if mark in p['marks'])
for p in plans.values():
    if mark in p['marks']:assert p['marks'][mark]==xy
prefix=b/'tracks_inc'/f'{mark}_{level}'
zs=[item['surf_z'] for p in plans.values() for item in p['placements'] if item['mark']==mark and Path(item['file']).name.startswith(f'{mark}_{level}_')]
assert zs and max(zs)==min(zs), 'No matching locked level or mismatched world heights'
z=zs[0]
for p in plans.values():
 assert p['protocol_hashes']==protocol_hashes, 'Plan does not match both locked protocols'
assert not Path(str(prefix)+'_capture.json').exists(),'Placement already captured; stop'
procs=[]; start=time.time()
for ns in plans:
    out=Path(str(prefix)+f'_{ns}.jsonl'); assert not out.exists(),f'Already captured {out}'
    log=open(str(prefix)+f'_{ns}.log','w')
    proc=subprocess.Popen(['python3','-u','docs/results/p1_arm1_zone/track_median.py','15',str(out),f'/loc_{ns}/tracks'],stdout=log,stderr=subprocess.STDOUT)
    procs.append((ns,proc,log))
status={}
for ns,proc,log in procs:status[ns]=proc.wait();log.close()
meta={'mark':mark,'height':level,'surf_z':z,'xy':xy,'operator_ready':True,'start':start,'end':time.time(),'exit_codes':status,'extrinsics':'incumbent launch + G34','duration_per_camera_s':15,'protocol_hashes':protocol_hashes,'bottle':next(iter(plans.values()))['bottle']}
Path(str(prefix)+'_capture.json').write_text(json.dumps(meta,indent=2)+'\n')
assert all(rc==0 for rc in status.values()),status
rep={**meta,'cameras':{}}
for ns,p in plans.items():
    rows=[json.loads(line) for line in Path(str(prefix)+f'_{ns}.jsonl').read_text().splitlines()]
    by={}; all_ids=set()
    for row in rows:
        for o in row['tracks']:
            all_ids.add(o['tid'])
            if 'bottle' in o['label'] and np.hypot(o['x']-xy[0],o['y']-xy[1])<.35:
                by.setdefault(o['tid'],[]).append(o)
    v={'messages':len(rows),'role':(p['roles'].get(mark) if any(item['mark']==mark and Path(item['file']).name.startswith(f'{mark}_{level}_') for item in p['placements']) else 'not_used'),'all_track_ids':sorted(all_ids),'candidate_counts':{str(t):len(L) for t,L in by.items()},'single_track':len(by)==1}
    if by:
        tid=max(by,key=lambda t:len(by[t])); L=by[tid]; X=np.array([[o['x'],o['y'],o['z']] for o in L]); med=np.median(X,axis=0)
        off=centroid_offset(np.array(p['old'][:3]),np.r_[xy,z],p['bottle']['r'],p['bottle']['H'])
        v.update(track_id=tid,n=len(L),labels=sorted({o['label'] for o in L}),median=med.tolist(),xy_error=(med[:2]-xy).tolist(),xy_error_abs_m=float(np.linalg.norm(med[:2]-xy)),model_offset=off.tolist(),z_expected=float(z+off[2]),z_error_m=float(med[2]-z-off[2]),p5_p95=(np.percentile(X,95,axis=0)-np.percentile(X,5,axis=0)).tolist(),conf_median=float(np.median([o['conf'] for o in L])),rivals={str(t):len(V) for t,V in by.items() if t!=tid})
    else:v['note']='No bottle track within locked selector radius; do not fabricate observation'
    rep['cameras'][ns]=v
Path(str(prefix)+'_summary.json').write_text(json.dumps(rep,indent=2)+'\n')
print(json.dumps(rep,indent=2))

