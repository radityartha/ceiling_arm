#!/usr/bin/env python3
"""Read-only audit of passive snapshots; no production selector or fit changes."""
import argparse, json
from pathlib import Path
import numpy as np


def audit(path):
    r=json.loads((path/'diagnosis.json').read_text());rows=[]
    events=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
    cameras={}
    for ns in ('rgbd','rgbd2'):
        fs=[f for f in r['frames'] if f['camera']==ns];local=[]
        for f in fs:
            with np.load(path/f['raw_arrays']) as a:
                assert a['mask'].dtype.kind=='i' and a['mask'].dtype.itemsize==4
                if 'depth' not in a:continue
                d=a['depth'];valid=np.isfinite(d)&(d>.1)&(d<12)
                ids=[int(i) for i in np.unique(a['mask']) if i>1]
                ident=max(ids,key=lambda i:np.count_nonzero(a['mask']==i)) if ids else None
                mask=a['mask']==ident if ident is not None else np.zeros_like(valid)
                item={'raw':f['raw_arrays'],'mask_id':ident,'mask_pixels':int(mask.sum()),'mask_valid_pixels':int((mask&valid).sum()),
                      'mask_valid_fraction':float((mask&valid).sum()/mask.sum()) if mask.any() else None,
                      'global_depth_valid_fraction':float(valid.mean()),'depth_mask_delta_s':f['depth']['delta_mask_s'],
                      'rgb_mask_delta_s':f.get('rgb',{}).get('delta_mask_s'),'mask_receipt_lag_s':f['mask_receipt']-f['mask_stamp']}
                inst=next((i for i in f['instances'] if i['id']==ident),{})
                if 'median_world' in inst:item.update(median_world=inst['median_world'],fraction_z_below_surface=inst['fraction_world_z_below_surface'])
                if 'rawdepth' in a:
                    joint=valid&(a['rawdepth']>0)
                    if joint.any():item['metric_raw_ratio_p5_p50_p95']=np.percentile(d[joint]/a['rawdepth'][joint],[5,50,95]).tolist()
                    item['raw_metric_valid_disagreement_fraction']=float(np.mean((a['rawdepth']>0)!=valid))
                local.append(item)
        tracks=[e['payload'] for e in events if e['camera']==ns and e['kind']=='tracks']
        selected=[]
        for objects in tracks:
            selected.extend(o for o in objects if 'bottle' in o['label'] and np.hypot(o['x']-.6497,o['y']+.4779)<.35)
        vals=[x['mask_valid_fraction'] for x in local if x['mask_valid_fraction'] is not None]
        rec={'snapshots':len(fs),'mask_valid_fraction_p5_p50_p95':np.percentile(vals,[5,50,95]).tolist() if vals else None,
             'mask_zero_valid_snapshots':sum(x['mask_valid_fraction']==0 for x in local),
             'global_depth_valid_fraction_p5_p50_p95':np.percentile([x['global_depth_valid_fraction'] for x in local],[5,50,95]).tolist() if local else None,
             'track_messages':len(tracks),'empty_track_messages':sum(not x for x in tracks),'track_ids':sorted({o['tid'] for o in selected}),
             'median_all_local_bottle_observations_diagnostic_only':np.median([[o[k] for k in ('x','y','z')] for o in selected],axis=0).tolist() if selected else None,
             'rows':local,'note':'All snapshots retained. Largest mask instance used only for quality audit; track aggregate not calibration selector or accepted observation.'}
        cameras[ns]=rec
    return {'capture':str(path),'purpose':r['purpose'],'geometry':r['geometry'],'cameras':cameras}

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('capture');a.add_argument('output');args=a.parse_args()
    r=audit(Path(args.capture));Path(args.output).write_text(json.dumps(r,indent=2)+'\n')
    for ns,c in r['cameras'].items():print(ns,{k:v for k,v in c.items() if k not in ('rows','note')})
