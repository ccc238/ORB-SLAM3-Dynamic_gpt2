#!/usr/bin/env python3
"""Same-run LK/geometric cross tables; creates temporary YAML only under /tmp."""
import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import tempfile
from collections import Counter, defaultdict
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
DATA=Path('/home/aaa/Datasets/TUM')


def read(path):
    with path.open() as f:
        rows=list(csv.DictReader(f))
    assert all(None not in r for r in rows),path
    return rows


def distribution(values):
    a=sorted(float(x) for x in values if math.isfinite(float(x)))
    return {'n':len(a),'median':a[(len(a)-1)//2] if a else None,
            'p90':a[int(.9*(len(a)-1))] if a else None,'max':a[-1] if a else None}


def audit(folder,n):
    rows=read(folder/'lk/features.csv');frames=read(folder/'lk/frames.csv')
    depth=read(folder/'depth/features.csv');epi=read(folder/'epipolar/features.csv')
    dmap={(r['frame_id'],r['feature_id']):r for r in depth}
    emap={(r['frame_id'],r['current_feature_index']):r for r in epi}
    expected=[r for r in depth if int(r['previous_feature_id'])>=0]
    assert len(rows)==len(expected)
    assert len(frames)==n
    cross=defaultdict(Counter);reason=Counter();by_range=defaultdict(Counter)
    for r in rows:
        key=r['frame_id'],r['current_feature_index'];d=dmap[key];e=emap[key]
        assert r['previous_feature_index']==d['previous_feature_id']==e['previous_feature_index']
        for out,original in [('depth_state','depth_state'),('residual_3d_m','residual_3d_m'),('raw_residual_3d_m','raw_residual_3d_m')]:
            assert r[out]==d[original],(key,out)
        assert r['epipolar_state']==e['state'] and r['epipolar_residual_px']==e['epipolar_residual_px']
        for a,b in [('p_x','previous_raw_u'),('p_y','previous_raw_v'),('q_orb_x','current_raw_u'),('q_orb_y','current_raw_v')]:assert r[a]==d[b]
        state=r['lk_check_state'];fb=float(r['fb_error_px']);endpoint=float(r['endpoint_error_px'])
        if state=='SUPPORTED':assert math.isfinite(fb) and fb<=1 and endpoint<=3
        elif state=='DISAGREES':assert math.isfinite(fb) and fb<=1 and endpoint>3
        elif r['lk_check_reason']=='FB_INCONSISTENT':assert fb>1
        else:assert state=='UNVERIFIABLE'
        if r['forward_status']=='0':assert math.isnan(float(r['q_lk_x'])) and math.isnan(endpoint)
        if r['backward_status']=='0':assert math.isnan(float(r['p_back_x'])) and math.isnan(fb)
        cross[r['depth_state']][state]+=1;reason[r['lk_check_reason']]+=1
        residual=float(r['residual_3d_m'])
        group='small_le_0.03m' if residual<=.03 else 'medium_0.03_to_0.08m' if residual<.08 else 'large_ge_0.08m' if math.isfinite(residual) else 'invalid'
        by_range[group][state]+=1
    log=(folder/'run.log').read_text()
    assert log.count('count_descriptors_mappoints_outliers_gray_unchanged=1')==n
    historical=ROOT/'outputs/depth-tracking-audit-v2'/('sitting_static120' if n==120 else 'walking40_gate_on')/'debug/features.csv'
    old=read(historical);fmap={(r['frame_id'],r['feature_id']):r for r in depth}
    checked=0
    for r in old:
        q=fmap.get((r['frame_id'],r['feature_id']))
        if q is None:continue
        for k in ('previous_feature_id','instance_id','previous_depth_m','current_depth_m','previous_raw_u','previous_raw_v','current_raw_u','current_raw_v'):assert r[k]==q[k],(k,r['frame_id'],r['feature_id'])
        checked+=1
    percentages={k:{s:100*v/sum(count.values()) for s,v in count.items()} for k,count in cross.items()}
    return {'frames':n,'matches':len(rows),'states':dict(Counter(r['lk_check_state'] for r in rows)),
            'depth_state_cross_counts':dict(cross),'depth_state_cross_percent':percentages,'residual_range_cross_counts':dict(by_range),'reasons':dict(reason),
            'fb_error_px':distribution(r['fb_error_px'] for r in rows),'endpoint_error_px':distribution(r['endpoint_error_px'] for r in rows),
            'compute_ms':distribution(r['compute_ms'] for r in frames),'record_ms':distribution(r['record_ms'] for r in frames),
            'historical_input_rows_verified':checked,'same_run_geometric_values_unchanged':True,'mutation_checks':n}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    parser.add_argument('--analyze-only',action='store_true');args=parser.parse_args()
    if not args.analyze_only:args.output.mkdir(parents=True,exist_ok=False)
    base=(ROOT/'config/TUM3-instance-association.yaml').read_text();summary={}
    histories=[ROOT/'outputs/depth-tracking-audit-v2'/name/'debug/features.csv' for name in ('sitting_static120','walking40_gate_on')]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in histories}
    def run(tmp,name,sequence,n,changes=None,lifecycle=False):
        folder=args.output/name;folder.mkdir()
        values={'Run.Viewer':0,'SemanticMask.Visualize':0,'SemanticMask.AssociationDebug':1,
                'SemanticMask.OutputDirectory':json.dumps(str((folder/'association').resolve())),
                'Residual3D.Debug':1,'Residual3D.Visualize':0,'Residual3D.OutputDirectory':json.dumps(str((folder/'depth').resolve())),
                'Epipolar.Debug':1,'Epipolar.Visualize':0,'Epipolar.OutputDirectory':json.dumps(str((folder/'epipolar').resolve())),
                'Dynamic.UseLKCheck':0,'LKCheck.Visualize':1,'LKCheck.SaveEvery':10,'LKCheck.MaxImagesPerFrame':4,
                'LKCheck.OutputDirectory':json.dumps(str((folder/'lk').resolve()))}
        values.update(changes or {});text=base
        for k,v in values.items():
            text=re.sub(r'(?m)^'+re.escape(k)+r':.*$','',text)
            if v is not None:text+='\n'+k+': '+str(v)+'\n'
        config=tmp/(name+'.yaml');config.write_text(text);(folder/'effective_settings.txt').write_text(text)
        dataset=DATA/('rgbd_dataset_freiburg3_'+sequence)
        if lifecycle:cmd=[str(ROOT/'build-semantic/depth_pose_lifecycle_test'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(dataset)]
        else:cmd=[str(ROOT/'Examples/RGB-D/rgbd_tum'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(dataset),str(dataset/'associate.txt'),str(folder.resolve()),str(n)]
        env=os.environ.copy();env.pop('DISPLAY',None);env.pop('ORB_RESIDUAL_MATCH_AUDIT_DIR',None)
        with (folder/'run.log').open('w') as log:subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=600)
        return folder
    with tempfile.TemporaryDirectory(prefix='orb-lk-check-',dir='/tmp') as temp:
        tmp=Path(temp)
        for name,seq,n in [('sitting_static120','sitting_static',120),('walking80','walking_xyz',80)]:
            folder=args.output/name
            if not args.analyze_only:folder=run(tmp,name,seq,n)
            summary[name]=audit(folder,n);print(name,json.dumps(summary[name]),flush=True)
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
        if not args.analyze_only:
            for name,value in [('off',1),('missing',None)]:
                f=run(tmp,name,'walking_xyz',3,{'Dynamic.UseLKCheck':value})
                assert not (f/'lk').exists() and '[LKCheckInvariant]' not in (f/'run.log').read_text()
                summary[name]={'no_LK_output_or_compute':True}
            f=run(tmp,'no_match_input','walking_xyz',3,{'Dynamic.Use3DResidual':1,'Dynamic.UseEpipolar':1})
            frames=read(f/'lk/frames.csv');assert all(r['reason']=='NO_MATCH_INPUT' and r['candidates']=='0' for r in frames)
            summary['no_match_input']={'frames':3,'reason':'NO_MATCH_INPUT'}
            f=run(tmp,'no_match_input_without_segmentation','walking_xyz',3,{'Dynamic.Enable':0,'Dynamic.Use3DResidual':1,'Dynamic.UseEpipolar':1})
            assert all(r['reason']=='NO_MATCH_INPUT' and r['candidates']=='0' for r in read(f/'lk/frames.csv'))
            summary['no_match_input_without_segmentation']={'frames':3,'reason':'NO_MATCH_INPUT'}
            for name,changes in [('epipolar_off',{'Dynamic.UseEpipolar':1}),('depth_off',{'Dynamic.Use3DResidual':1})]:
                f=run(tmp,name,'walking_xyz',3,changes);rows=read(f/'lk/features.csv');assert rows
                blank='epipolar_state' if name=='epipolar_off' else 'depth_state';assert all(r[blank]=='' for r in rows)
                summary[name]={'matches':len(rows),'disabled_geometry_fields_blank':True}
            f=run(tmp,'lifecycle','walking_xyz',28,lifecycle=True)
            frames=read(f/'lk/frames.csv');assert len(frames)==28
            for fid in ('16','22'):
                r=next(r for r in frames if r['frame_id']==fid);assert r['candidates']=='0' and r['reason']=='NO_PREVIOUS_FRAME'
            summary['lifecycle']={'frames':28,'reasons':dict(Counter(r['reason'] for r in frames)),'reset_and_dataset_caches_cleared':True}
        for p in histories:assert hashlib.sha256(p.read_bytes()).hexdigest()==before[str(p)]
        summary['historical_feature_csv_sha256_unchanged']=before
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print('PASS LK audit',args.output)


if __name__=='__main__':main()
