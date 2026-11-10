#!/usr/bin/env python3
"""Validate existing rgbd_tum output. No alternative SLAM executable.
Run after build: python3 tools/validate_instance_association.py
"""
import csv
import json
from pathlib import Path
import re
import subprocess
import time
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/instance-association'
DATA=Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz')
OUT.mkdir(parents=True,exist_ok=True)
def setting(text,key,value):
    p=r'(?m)^'+re.escape(key)+r':.*$'
    line=key+': '+str(value)
    return re.sub(p,line,text) if re.search(p,text) else text+'\n'+line+'\n'
def read(path):
    with path.open() as f: return list(csv.DictReader(f))
summary={}
for name,source,enabled,mask,assoc,frames,resize in [
    ('association','TUM3-instance-association.yaml',1,0,1,12,False),
    ('resized','TUM3-instance-association.yaml',1,0,1,5,True),
    ('selected','TUM3-semantic-selected.yaml',1,1,0,5,False),
    ('original','TUM3-semantic-off.yaml',0,1,0,5,False),
]:
    directory=OUT/name
    text=(ROOT/'config'/source).read_text()
    for key,value in {'Run.Viewer':0,'SemanticMask.Enabled':enabled,'SemanticMask.MaskingEnabled':mask,'SemanticMask.AssociationEnabled':assoc,'SemanticMask.AssociationDebug':assoc,'SemanticMask.Debug':mask*enabled,'SemanticMask.OutputDirectory':'"'+str(directory/'debug')+'"'}.items():
        text=setting(text,key,value)
    if resize:
        text=setting(text,'Camera.newWidth',480)
        text=setting(text,'Camera.newHeight',360)
    config=OUT/(name+'.yaml'); config.write_text(text)
    command=[str(ROOT/'Examples/RGB-D/rgbd_tum'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(DATA),str(DATA/'associate.txt'),str(directory),str(frames)]
    started=time.monotonic()
    with (OUT/(name+'.log')).open('w') as log:
        subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=150)
    runs=read(directory/'RunFrames.csv')
    assert len(runs)==frames and all(r['state']=='2' for r in runs),(name,runs)
    result={'frames':frames,'tracking_states':sorted(set(r['state'] for r in runs)), 'mean_track_ms':sum(float(r['track_ms']) for r in runs)/frames,'wall_seconds':time.monotonic()-started}
    if assoc:
        debug=directory/'debug'
        frame_rows=read(debug/'association_frames.csv')
        features=read(debug/'association_features.csv')
        info=read(debug/'association_instances.csv')
        assert len(frame_rows)==frames
        by_frame={}
        for row in features: by_frame.setdefault(row['frame_id'],[]).append(row)
        tables={}
        for row in info: tables.setdefault(row['frame_id'],{})[int(row['instance_id'])]=row
        persons=background=other=0
        for r in frame_rows:
            fid=r['frame_id']; n=int(r['total_features']); keys=by_frame[fid]
            assert n==len(keys)==int(r['features_before'])==int(r['features_after'])
            assert sum(int(r[c]) for c in ('background_features','instance_features','invalid_features'))==n
            assert [int(k['feature_index']) for k in keys]==list(range(n))
            labels=np.asarray(Image.open(debug/('association_'+fid+'_labels.png')))
            assert labels.shape==((360,480) if resize else (480,640)),labels.shape
            table=tables.get(fid,{})
            assert set(map(int,np.unique(labels)))-{0} <= set(table)
            for k in keys:
                assert k['coordinate_space']=='slam_rgb_raw_pixels'
                u,v=float(k['u']),float(k['v']); label=int(k['instance_id'])
                expected=int(labels[int(np.floor(v)),int(np.floor(u))]) if np.isfinite(u) and np.isfinite(v) and 0<=u<labels.shape[1] and 0<=v<labels.shape[0] else -1
                assert label==expected
                if label>0:
                    if table[label]['class_id']=='0': persons+=1
                    else: other+=1
                if label==0: background+=1
        assert persons>0 and background>0
        result.update(person_features=persons,other_instance_features=other,background_features=background,all_csv_labels_match=True,unchanged_count=True,label_size=list(labels.shape))
    elif mask and enabled:
        checks=read(directory/'debug/frames.csv')
        assert all(int(r['violations'])==0 for r in checks)
        result['mask_violations']=0
    summary[name]=result
    print(name,result,flush=True)
association_runs=read(OUT/'association/RunFrames.csv')
original_runs=read(OUT/'original/RunFrames.csv')
assert [r['keypoints'] for r in association_runs[:len(original_runs)]]==[r['keypoints'] for r in original_runs], 'association-only changed extraction counts versus original'
summary['association']['first_five_counts_match_original']=True
summary['association']['first_five_counts']=[int(r['keypoints']) for r in original_runs]
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
