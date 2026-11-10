#!/usr/bin/env python3
"""Run under xvfb-run to exercise the real Pangolin/OpenCV viewer on TUM RGB-D.
Requires xwininfo, xwd, numpy and Pillow. Does not modify the model or dataset.
"""
import csv
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import time
import numpy as np
from PIL import Image

root=Path(__file__).resolve().parents[1]
data=Path(sys.argv[1] if len(sys.argv)>1 else '/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz')
association=len(sys.argv)>2 and sys.argv[2]=='association'
out=Path(os.environ.get('SEMANTIC_VIEW_OUT', str(root/('outputs/instance-association/live-view' if association else 'outputs/semantic-live-view'))))
out.mkdir(parents=True,exist_ok=True)
base=(root/'config'/('TUM3-instance-association.yaml' if association else 'TUM3-semantic-selected.yaml')).read_text()

def setting(text,key,value):
    pattern=r'(?m)^'+re.escape(key)+r':.*$'
    line=key+': '+str(value)
    return re.sub(pattern,line,text) if re.search(pattern,text) else text+'\n'+line+'\n'

def snapshot(window_id,stem):
    xwd=stem.with_suffix('.xwd')
    subprocess.run(['xwd','-silent','-id',window_id,'-out',str(xwd)],check=True)
    content=xwd.read_bytes()
    fields=struct.unpack('>25I',content[:100])
    header_size,version=fields[:2]
    width,height,order,bpp,stride=fields[4],fields[5],fields[7],fields[11],fields[12]
    assert version==7 and bpp==32,(version,bpp)
    offset=header_size+fields[19]*12
    pixels=np.ndarray((height,width),dtype='<u4' if order==0 else '>u4',buffer=content,offset=offset,strides=(stride,4))
    channels=[]
    for mask in fields[14:17]:
        assert mask
        shift=(mask & -mask).bit_length()-1
        channels.append((((pixels & mask)>>shift)*255//(mask>>shift)).astype(np.uint8))
    rgb=np.stack(channels,axis=-1)
    Image.fromarray(rgb).save(stem.with_suffix('.png'))
    return rgb

results={}
for name,viewer,visualize,count in [('enabled',1,1,30),('window-off',1,0,8),('headless',0,1,5),('dynamic-off',1,1,8)]:
    dynamic=name!='dynamic-off'
    text=setting(base,'Run.Viewer',viewer)
    text=setting(text,'Dynamic.Enable',int(dynamic))
    text=setting(text,'Dynamic.MaskOnly',1)
    text=setting(text,'SemanticMask.Visualize',visualize)
    text=setting(text,'SemanticMask.Debug',0)
    text=setting(text,'SemanticMask.OutputDirectory','"'+str(out/name/'debug')+'"')
    config=out/(name+'.yaml');config.write_text(text)
    argv=[str(root/'Examples/RGB-D/rgbd_tum'),str(root/'Vocabulary/ORBvoc.txt'),str(config),str(data),str(data/'associate.txt'),str(out/name),str(count)]
    env=os.environ.copy()
    if not viewer: env.pop('DISPLAY',None)
    seen=set();captures=[];first_semantic=None;last_capture=0
    started=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        proc=subprocess.Popen(argv,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
        try:
            while proc.poll() is None:
                now=time.monotonic()
                if now-started>90: raise RuntimeError(name+' timed out')
                tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,check=True).stdout
                for title in ('Map Viewer','Current Frame','Semantic Mask'):
                    if 'ORB-SLAM3: '+title in tree: seen.add(title)
                match=re.search(r'(0x[0-9a-fA-F]+) "ORB-SLAM3: Semantic Mask"',tree)
                if match:
                    if first_semantic is None: first_semantic=now
                    if viewer and visualize and now-first_semantic>1.5 and now-last_capture>1.0 and len(captures)<2:
                        pixels=snapshot(match.group(1),out/('viewer-'+str(len(captures)+1)))
                        captures.append(pixels);last_capture=now
                time.sleep(0.1)
            if proc.returncode: raise RuntimeError(name+' exit '+str(proc.returncode)+'; see log')
        finally:
            if proc.poll() is None:
                proc.terminate();proc.wait(timeout=10)
    if viewer:
        assert {'Map Viewer','Current Frame'}.issubset(seen),(name,seen)
        assert ('Semantic Mask' in seen)==bool(visualize and dynamic),(name,seen)
    else: assert not seen,(name,seen)
    if viewer and visualize and dynamic:
        assert len(captures)==2,'did not capture two live images'
        assert captures[0].shape==captures[1].shape and np.any(captures[0]!=captures[1]),'live image did not update'
    with (out/name/'RunFrames.csv').open() as f: frames=list(csv.DictReader(f))
    assert len(frames)==count and all(row['state']=='2' for row in frames),(name,frames)
    assert not (out/name/'debug').exists(),'Debug=0 unexpectedly wrote images'
    results[name]={'frames':len(frames),'state':'OK','windows':sorted(seen),'captures':len(captures),'debug':False,'clean_exit':True}
    print(name,results[name],flush=True)
(out/'gui-summary.json').write_text(json.dumps(results,indent=2)+'\n')
