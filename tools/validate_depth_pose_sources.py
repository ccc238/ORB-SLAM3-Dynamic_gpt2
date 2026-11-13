#!/usr/bin/env python3
"""Compare Tracking and existing PnP diagnostics, preserving previous evidence.
Usage: python3 tools/validate_depth_pose_sources.py [frames=40] [new_output_directory]
Requires numpy; uses the existing local TUM input/model and temporary /tmp YAML.
"""
import csv
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = Path(sys.argv[2]).resolve() if len(sys.argv)>2 else ROOT/'outputs/depth-pose-source-validation'
COUNT = int(sys.argv[1]) if len(sys.argv)>1 else 40
DATA = Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz')
BASE = (ROOT/'config/TUM3-instance-association.yaml').read_text()
OUT.mkdir(parents=True, exist_ok=True)

def setting(text,key,value):
    text = re.sub(r'(?m)^'+re.escape(key)+r':.*$', '', text)
    return text if value is None else text+'\n'+key+': '+str(value)+'\n'

def rows(path):
    with path.open() as f:
        result=list(csv.DictReader(f))
    assert all(None not in a for a in result), path  # exact header/row column alignment
    return result

def matrix(value):
    return np.array(value['data'], dtype=np.float32 if value['dt']=='f' else np.float64).reshape(value['rows'],value['cols']).astype(np.float64)

def audit(folder, source, count, lifecycle=False):
    frames=rows(folder/'debug/frames.csv')
    features=rows(folder/'debug/features.csv')
    log=(folder/'run.log').read_text()
    assert len(frames)==count
    assert log.count('count_descriptors_mappoints_unchanged=1')==count
    byframe={}
    for a in features:
        byframe.setdefault(a['frame_id'],[]).append(a)
    labels=rows(folder/'association/association_features.csv')
    coords={(a['frame_id'],a['feature_index']):(float(a['u']),float(a['v'])) for a in labels}
    finite=0;max_composition=0.;max_vector=0.;nonidentity=0
    for frame in frames:
        fid=frame['frame_id'];model=json.loads((folder/('debug/motion_'+fid+'.json')).read_text())
        assert frame['pose_source']==source==model['pose_source']
        assert frame['transform_valid']==frame['motion_valid']
        assert frame['transform_reason']==frame['reason']
        assert int(frame['current_frame_id'])==int(fid)==int(model['current_frame_id'])
        if source=='background_pnp':
            assert frame['tracking_inliers']==frame['previous_tracking_inliers']==''
        else:
            assert frame['pnp_inliers']=='0' and frame['background_candidates']=='0'
        valid=frame['motion_valid']=='1'
        if valid:
            previous=frame['previous_frame_id']
            assert int(previous)<int(fid)
            assert float(frame['current_timestamp'])>float(frame['previous_timestamp'])
            T=matrix(model['T_current_previous']);R=matrix(model['R']);t=matrix(model['t_m']).reshape(3)
            assert np.max(np.abs(T[:3,:3]-R))<1e-12 and np.max(np.abs(T[:3,3]-t))<1e-12
            if np.linalg.norm(T-np.eye(4))>1e-6:
                nonidentity+=1
            if source=='tracking':
                p,c=model['previous_tracking'],model['current_tracking']
                assert p['has_pose']==c['has_pose']==p['tracking_ok']==c['tracking_ok']==1
                assert p['frame_id']==int(previous) and c['frame_id']==int(fid)
                assert p['timestamp']==float(frame['previous_timestamp']) and c['timestamp']==float(frame['current_timestamp'])
                assert c['tracking_inliers']>=25
                assert p['map_id']==c['map_id'] and p['map_change']==c['map_change'] and p['big_change']==c['big_change']
                assert p['relocalization_frame']==c['relocalization_frame']
                expected=matrix(c['Tcw'])@np.linalg.inv(matrix(p['Tcw']))
                error=float(np.max(np.abs(T-expected)));max_composition=max(max_composition,error)
                assert error<1e-12
            K=matrix(model['K']);inverse=np.linalg.inv(K)
        for a in byframe.get(fid,[]):
            assert a['pose_source']==source
            residual=float(a['residual_3d_m'])
            if not valid:
                assert a['depth_state']=='UNKNOWN' and math.isnan(residual)
            if math.isfinite(residual):
                finite+=1
                assert valid and a['depth_valid']=='1'
                p=np.array([*coords[(frame['previous_frame_id'],a['previous_feature_id'])],1.])
                c=np.array([*coords[(fid,a['feature_id'])],1.])
                # This real test configuration has zero distortion.
                vector=float(a['current_depth_m'])*(inverse@c)-(R@(float(a['previous_depth_m'])*(inverse@p))+t)
                saved=np.array([float(a['residual_'+axis+'_m']) for axis in 'xyz'])
                error=float(np.max(np.abs(vector-saved)));max_vector=max(max_vector,error)
                assert error<1e-10 and abs(np.linalg.norm(vector)-residual)<1e-10
    if not lifecycle and count>=10:
        assert nonidentity>0 and finite>0
    if lifecycle:
        assert 'ResetActiveMap before sample=16' in log and 'ChangeDataset before sample=22' in log
        assert log.count('reason=NO_PREVIOUS_FRAME')>=3
        assert re.search(r'\[Lifecycle\] sample=10 .* state=(3|4)',log)
        assert all(a['motion_valid']=='0' for a in frames if a['frame_id'] in ('10','11'))
    return {'frames':count,'valid_motion':sum(a['motion_valid']=='1' for a in frames),
            'motion_reasons':dict(Counter(a['reason'] for a in frames)),
            'finite_residuals':finite,'nonidentity_valid_transforms':nonidentity,
            'max_composition_error':max_composition if source=='tracking' and nonidentity else None,'max_residual_vector_error_m':max_vector if finite else None,
            'invariant_audits':count}

cases=[
    ('tracking','tracking',COUNT,{},False),
    ('background_pnp','background_pnp',COUNT,{},False),
    ('default_tracking',None,6,{},False),
    ('tracking_epipolar_off','tracking',6,{'Dynamic.UseEpipolar':1},False),
    ('tracking_quality_rejection','tracking',6,{'Residual3D.TrackingMinInliers':10000},False),
    ('disabled','tracking',5,{'Dynamic.Use3DResidual':1,'Dynamic.UseEpipolar':1},False),
    ('invalid_source','bad',1,{},True),
    ('invalid_source_type',0,1,{},True),
    ('lifecycle','tracking',28,{},False),
]
summary={}
with tempfile.TemporaryDirectory(prefix='orb-depth-pose-',dir='/tmp') as tmp:
    for name,source,count,extra,error in cases:
        folder=OUT/name
        assert not folder.exists(), f'Preserve existing evidence: {folder}'
        folder.mkdir()
        values={'Residual3D.UseDepthDifferenceGate':0,'Dynamic.Use3DResidual':0,'Dynamic.DepthPoseSource':None if source is None else json.dumps(source),
                'Dynamic.UseEpipolar':0,'Run.Viewer':0,'Residual3D.Debug':1,'Residual3D.Visualize':1,
                'Residual3D.OutputDirectory':json.dumps(str(folder/'debug')),
                'SemanticMask.Visualize':0,'SemanticMask.AssociationDebug':1,
                'SemanticMask.OutputDirectory':json.dumps(str(folder/'association')),
                'Epipolar.Debug':0,'Epipolar.Visualize':0,**extra}
        text=BASE
        for key,value in values.items():
            text=setting(text,key,value)
        config=Path(tmp)/(name+'.yaml');config.write_text(text)
        if name=='lifecycle':
            cmd=[str(ROOT/'build-semantic/depth_pose_lifecycle_test'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(DATA)]
        else:
            cmd=[str(ROOT/'Examples/RGB-D/rgbd_tum'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(DATA),str(DATA/'associate.txt'),str(folder),str(count)]
        env=os.environ.copy();env.pop('DISPLAY',None)
        with (folder/'run.log').open('w') as log:
            run=subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=300)
        content=(folder/'run.log').read_text()
        assert (run.returncode!=0)==error,(name,run.returncode,content[-1500:])
        if error:
            assert 'Dynamic.DepthPoseSource' in content
            summary[name]={'expected_error':content.splitlines()[-1]}
        elif name=='disabled':
            assert not (folder/'debug').exists() and '[Residual3D]' not in content
            summary[name]={'frames':count,'disabled':True}
        else:
            summary[name]=audit(folder,'tracking' if source is None else source,count,name=='lifecycle')
            if name=='tracking_quality_rejection':
                assert summary[name]['valid_motion']==0
        print(name,json.dumps(summary[name]),flush=True)
        (OUT/'summary.json').write_text(json.dumps(summary,indent=2))
    a=rows(OUT/'tracking/association/association_features.csv');b=rows(OUT/'background_pnp/association/association_features.csv')
    assert a==b
    assert rows(OUT/'disabled/association/association_features.csv')==[r for r in a if int(r['frame_id'])<5]
    summary['invariants']={'association_features_identical_between_sources':True,'association_features_identical_when_disabled_first5':True}
    left=rows(OUT/'tracking/debug/features.csv');right=rows(OUT/'background_pnp/debug/features.csv')
    keys=['frame_id','feature_id','previous_feature_id','instance_id','previous_depth_m','current_depth_m','depth_valid',
          'previous_depth_valid','current_depth_valid','previous_depth_reason','current_depth_reason','epipolar_residual_px','epipolar_state']
    assert [{k:r[k] for k in keys} for r in left]==[{k:r[k] for k in keys} for r in right]
    summary['invariants']['match_depth_epipolar_inputs_identical_between_sources']=True
    gold=ROOT/'outputs/residual3d-validation/walking_both/debug/features.csv'
    if gold.exists() and COUNT==40:
        original=rows(gold)
        assert len(original)==len(right)
        assert all(all(current[k]==v for k,v in old.items()) for old,current in zip(original,right))
        summary['invariants']['pnp_all_original_feature_fields_identical_to_previous_40frame_evidence']=True
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
print('PASS depth pose source validation',OUT)
