#!/usr/bin/env python3
"""Run the unchanged 40-frame input plus static/longer walking diagnostic audits.
Creates only /tmp temporary YAML; preserves all existing results.
Usage: python3 tools/validate_depth_audit.py [new_output_root]
"""
import csv,json,math,os,re,subprocess,sys,tempfile
from collections import Counter
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else ROOT/'outputs/depth-tracking-audit-v2'
DATA=Path('/home/aaa/Datasets/TUM');ORIGINAL=ROOT/'outputs/residual3d-tracking-test'
BASE=(ROOT/'config/TUM3-instance-association.yaml').read_text()
def read(path):
    with path.open() as f:r=list(csv.DictReader(f))
    assert all(None not in x for x in r),path
    return r

def mat(m):return np.asarray(m['data'],dtype=np.float32 if m['dt']=='f' else np.float64).reshape(m['rows'],m['cols']).astype(float)
def stats(values):
    a=sorted(values)
    return {'n':len(a),'median_m':a[len(a)//2] if a else None,'p90_m':a[int(.9*(len(a)-1))] if a else None}
def audit(folder,gate,n):
    frames=read(folder/'debug/frames.csv');features=read(folder/'debug/features.csv');assert len(frames)==n
    assert (folder/'run.log').read_text().count('count_descriptors_mappoints_unchanged=1')==n
    models={};rebased=0;max_matrix=0.;max_rebase=0.;max_vector=0.;valid=0
    for f in frames:
        model=json.loads((folder/('debug/motion_'+f['frame_id']+'.json')).read_text());models[f['frame_id']]=model
        assert model['depth_difference_gate_enabled']==gate
        assert float(f['max_depth_difference_m'])==.2
        if f['motion_valid']=='1':
            valid+=1;p,c=model['previous_tracking'],model['current_tracking']
            assert p['frame_id']!=c['frame_id'] and p['timestamp']<c['timestamp']
            assert p['map_id']==c['map_id'] and p['map_change']==c['map_change'] and p['big_change']==c['big_change'] and p['relocalization_frame']==c['relocalization_frame']
            assert c['tracking_inliers']>=25
            T=mat(model['T_current_previous']);expected=mat(c['Tcw'])@np.linalg.inv(mat(p['Tcw']))
            error=float(np.max(np.abs(T-expected)));max_matrix=max(max_matrix,error);assert error<1e-12
            if p['rebased']:
                rebased+=1;assert p['original_map_change']!=p['map_change']
                expected=mat(p['frame_to_reference'])@mat(p['reference_Tcw'])
                error=float(np.max(np.abs(expected-mat(p['Tcw']))));max_rebase=max(max_rebase,error);assert error<1e-12
                history=models.get(str(int(p['frame_id'])))
                if history:assert np.max(np.abs(mat(history['current_tracking']['Tcw'])-mat(p['original_Tcw'])))<1e-12
        else:
            assert f['reason']!='OK'
    excluded=0;raw_values=[];kept_values=[];people=[];background=[];paper_offline=[]
    classes={(r['frame_id'],r['instance_id']):r['class_id'] for r in read(folder/'association/association_instances.csv')}
    for f in features:
        raw=float(f['raw_residual_3d_m']);value=float(f['residual_3d_m']);model=models[f['frame_id']]
        reject=f['depth_difference_gate_rejected']=='1';dz=float(f['depth_difference_m'])
        assert reject==(bool(gate) and math.isfinite(dz) and dz>.2)
        if model['motion_valid']==0 or f['depth_valid']=='0' or reject:
            assert f['depth_state']=='UNKNOWN' and math.isnan(value)
        if math.isfinite(raw):
            raw_values.append(raw)
            if dz<=.2:paper_offline.append(raw)
            K=mat(model['K']);inv=np.linalg.inv(K);T=mat(model['T_current_previous'])
            p=np.array([float(f['previous_un_u']),float(f['previous_un_v']),1.])*float(f['previous_depth_m'])
            c=np.array([float(f['current_un_u']),float(f['current_un_v']),1.])*float(f['current_depth_m'])
            vector=inv@c-(T[:3,:3]@(inv@p)+T[:3,3])
            error=abs(float(np.linalg.norm(vector))-raw);max_vector=max(max_vector,error);assert error<1e-10
            if reject:excluded+=1
        if math.isfinite(value):
            assert value==raw;kept_values.append(value)
            if f['instance_id']=='0':background.append(value)
            elif classes.get((f['frame_id'],f['instance_id']))=='0':people.append(value)
    return {'frames':n,'motion_valid_frames':valid,'frame_reasons':dict(Counter(f['reason'] for f in frames)),
        'reference_refreshed_valid_frames':rebased,'raw_residuals':stats(raw_values),'retained_residuals':stats(kept_values),
        'background_retained':stats(background),'person_retained':stats(people),'paper_gate_offline_on_same_raw_data':stats(paper_offline),
        'excluded_finite_samples':excluded,'eligible_depth_gate_rejections':sum(f['depth_difference_gate_rejected']=='1' for f in features),
        'feature_reasons':dict(Counter(f['reason'] for f in features)),
        'max_composition_error':max_matrix,'max_reference_refresh_error':max_rebase,'max_raw_residual_replay_error_m':max_vector,
        'invariant_checks':n}

def setting(text,key,value):return re.sub(r'(?m)^'+re.escape(key)+r':.*$','',text)+'\n'+key+': '+str(value)+'\n'
def lifecycle(tmp):
    folder=OUT/'lifecycle';folder.mkdir()
    values={'Dynamic.Use3DResidual':0,'Dynamic.DepthPoseSource':'"tracking"','Dynamic.UseEpipolar':0,
        'Residual3D.UseDepthDifferenceGate':1,'Residual3D.MaxDepthDifferenceM':.20,
        'Residual3D.Debug':1,'Residual3D.Visualize':0,'Residual3D.OutputDirectory':json.dumps(str(folder/'debug')),
        'Run.Viewer':0,'SemanticMask.Visualize':0,'SemanticMask.AssociationDebug':1,
        'SemanticMask.OutputDirectory':json.dumps(str(folder/'association')),'Epipolar.Debug':0,'Epipolar.Visualize':0}
    text=BASE
    for key,value in values.items():text=setting(text,key,value)
    config=tmp/'lifecycle.yaml';config.write_text(text)
    cmd=[str(ROOT/'build-semantic/depth_pose_lifecycle_test'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(DATA/'rgbd_dataset_freiburg3_walking_xyz')]
    env=os.environ.copy();env.pop('DISPLAY',None)
    with (folder/'run.log').open('w') as log:
        run=subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=300)
    assert run.returncode==0
    result=audit(folder,1,28);log=(folder/'run.log').read_text()
    assert 'ResetActiveMap before sample=16' in log and 'ChangeDataset before sample=22' in log
    frames=read(folder/'debug/frames.csv')
    assert all(r['motion_valid']=='0' for r in frames if r['frame_id'] in ('10','11','16','22'))
    assert re.search(r'\[Lifecycle\] sample=10 .* state=(3|4)',log)
    return result

OUT.mkdir(parents=True,exist_ok=False)
cases=[('walking40_gate_off','walking_xyz',40,0),('walking40_gate_on','walking_xyz',40,1),
       ('sitting_static120','sitting_static',120,1),('walking_xyz200','walking_xyz',200,1)]
summary={}
with tempfile.TemporaryDirectory(prefix='orb-depth-audit-',dir='/tmp') as tmp:
    for name,sequence,n,gate in cases:
        folder=OUT/name;folder.mkdir();data=DATA/('rgbd_dataset_freiburg3_'+sequence)
        values={'Dynamic.Use3DResidual':0,'Dynamic.DepthPoseSource':'"tracking"','Dynamic.UseEpipolar':0,
            'Residual3D.TrackingMinInliers':25,'Residual3D.UseDepthDifferenceGate':gate,'Residual3D.MaxDepthDifferenceM':.20,
            'Residual3D.Debug':1,'Residual3D.Visualize':0,'Residual3D.OutputDirectory':json.dumps(str(folder/'debug')),
            'Run.Viewer':0,'SemanticMask.Visualize':0,'SemanticMask.AssociationDebug':1,
            'SemanticMask.OutputDirectory':json.dumps(str(folder/'association')),'Epipolar.Debug':0,'Epipolar.Visualize':0}
        text=BASE
        for key,value in values.items():text=setting(text,key,value)
        config=Path(tmp)/(name+'.yaml');config.write_text(text)
        command=[str(ROOT/'Examples/RGB-D/rgbd_tum'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(data),str(data/'associate.txt'),str(folder),str(n)]
        env=os.environ.copy();env.pop('DISPLAY',None)
        with (folder/'run.log').open('w') as log:run=subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600)
        assert run.returncode==0,(name,(folder/'run.log').read_text()[-1800:])
        summary[name]=audit(folder,gate,n);print(name,json.dumps(summary[name]),flush=True)
        (OUT/'summary.json').write_text(json.dumps(summary,indent=2))
        if name=='walking40_gate_off':
            # Recover coordinates for the original CSV only after complete per-index input agreement.
            command=[sys.executable,str(ROOT/'tools/inspect_depth_residuals.py'),str(ORIGINAL),str(data),str(OUT/'original_large_residuals'),
                     '--coordinate-features',str(folder/'debug/features.csv'),'--overview']
            subprocess.run(command,cwd=ROOT,check=True)
        subprocess.run([sys.executable,str(ROOT/'tools/inspect_depth_residuals.py'),str(folder/'debug'),str(data),str(folder/'large_residuals')],cwd=ROOT,check=True)
        subprocess.run([sys.executable,str(ROOT/'tools/inspect_depth_residuals.py'),str(folder/'debug'),str(data),str(folder/'retained_large_residuals'),'--only-retained','--per-group','3','--overview'],cwd=ROOT,check=True)
    summary['lifecycle']=lifecycle(Path(tmp));print('lifecycle',json.dumps(summary['lifecycle']),flush=True)
    a=read(OUT/'walking40_gate_off/association/association_features.csv');b=read(OUT/'walking40_gate_on/association/association_features.csv')
    assert a==b
    original=read(ORIGINAL/'features.csv');fresh=read(OUT/'walking40_gate_off/debug/features.csv')
    assert len(original)==len(fresh)
    for p,c in zip(original,fresh):
        for key in ('frame_id','feature_id','previous_feature_id','instance_id','previous_depth_m','current_depth_m','depth_valid'):
            assert p[key]==c[key],(key,p['frame_id'],p['feature_id'])
    summary['invariants']={'original_and_replay_match_depth_label_inputs_equal':True,'gate_on_off_association_features_equal':True}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
print('PASS depth audit',OUT)
