#!/usr/bin/env python3
"""Serial, synchronous real-data checks; leaves the model and dataset untouched."""
import csv
import json
import pathlib
import statistics
import subprocess
import sys

root = pathlib.Path(__file__).resolve().parents[1]
data = pathlib.Path(sys.argv[1] if len(sys.argv)>1 else '/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz')
out = root/'outputs/semantic-validation'
out.mkdir(parents=True, exist_ok=True)
association = out/'associate-30.txt'
lines = [s for s in (data/'associate.txt').read_text().splitlines() if s.strip() and not s.startswith('#')]
association.write_text('\n'.join(lines[:30])+'\n')
image = data/lines[0].split()[1]
def run(name, args, cwd=root):
    with (out/(name+'.log')).open('w') as log:
        result = subprocess.run([str(x) for x in args],cwd=cwd,stdout=log,stderr=subprocess.STDOUT)
    print(name, 'exit',result.returncode,flush=True)
    if result.returncode: raise RuntimeError('See '+str(out/(name+'.log')))
run('module-test',[root/'build-semantic/semantic_mask_test','/home/aaa/models/yolo/yolo11s-seg.onnx',image,out/'module-debug'])
exe=root/'Examples/RGB-D/rgbd_tum'
voc=root/'Vocabulary/ORBvoc.txt'
for mode in ('off','all','selected'):
    run(mode,[exe,voc,root/f'config/TUM3-semantic-{mode}.yaml',data,association,out/mode])
# Exercise System::TrackRGBD internal resizing (not just a module-sized input).
base=(root/'config/TUM3-semantic-all.yaml').read_text()
cases={
    'resized':base+'\nCamera.newWidth: 320\nCamera.newHeight: 240\n',
    'empty':base.replace('SemanticMask.Confidence: 0.25','SemanticMask.Confidence: 1.0'),
    'fully-blocked':base.replace('SemanticMask.DilationRadius: 0','SemanticMask.DilationRadius: 640'),
}
for name, text in cases.items():
    config=out/(name+'.yaml')
    config.write_text(text.replace('outputs/semantic-validation/all/debug',str(out/name/'debug')))
    run(name,[exe,voc,config,data,association,out/name,5])
# Initialize with a texture fixture, then feed a real-image fully masked frame.
transition_config=out/'transition.yaml'
transition_config.write_text(cases['fully-blocked'].replace('outputs/semantic-validation/all/debug',str(out/'transition/debug')))
run('transition',[root/'build-semantic/semantic_mask_test','/home/aaa/models/yolo/yolo11s-seg.onnx',image,out/'transition-module-debug',voc,transition_config])
# Original four positional arguments with the explicit off switch.
legacy=out/'legacy';legacy.mkdir(exist_ok=True)
config=out/'legacy.yaml'
config.write_text((root/'Examples/RGB-D/TUM3.yaml').read_text()+'\nRun.Viewer: 0\nSemanticMask.Enabled: 0\n')
run('legacy',[exe,voc,config,data,association],legacy)
summary={}
for mode in ('off','all','selected','resized','empty','fully-blocked','legacy'):
    with (out/mode/'RunFrames.csv').open() as f: rows=list(csv.DictReader(f))
    timings=[float(r['track_ms']) for r in rows]
    item={'frames':len(rows),'mean_track_ms':statistics.mean(timings),'median_track_ms':statistics.median(timings),'mean_keypoints':statistics.mean(int(r['keypoints']) for r in rows),'states':sorted(set(r['state'] for r in rows)), 'trajectory_lines':len((out/mode/'CameraTrajectory.txt').read_text().splitlines())}
    debug=out/mode/'debug/frames.csv'
    if debug.exists():
        with debug.open() as f: detections=list(csv.DictReader(f))
        for key in ('instances','blocked_ratio','pre_ms','infer_ms','post_ms','candidates','rejected'):
            item['mean_'+key]=statistics.mean(float(r[key]) for r in detections)
        item['violations']=sum(int(r['violations']) for r in detections)
        if item['violations']: raise RuntimeError('Blocked keypoint invariant violated')
    if mode=='empty' and (item['mean_instances']!=0 or item['mean_blocked_ratio']!=0):
        raise RuntimeError('Expected legitimate empty detections')
    if mode=='fully-blocked' and (item['mean_blocked_ratio']!=1 or item['mean_keypoints']!=0):
        raise RuntimeError('Expected full mask and zero keypoints')
    if mode=='resized':
        # Debug output comes from the image actually handed to Frame, after System resizing.
        png=(out/mode/'debug/0_blocked.png').read_bytes()
        import struct
        if struct.unpack('>II',png[16:24])!=(320,240):
            raise RuntimeError('System resize and mask coordinates disagree')
    summary[mode]=item
with (out/'transition/debug/frames.csv').open() as f: transition=list(csv.DictReader(f))
summary['transition']={'states':[int(r['state']) for r in transition], 'keypoints':[int(r['keypoints']) for r in transition], 'blocked_ratios':[float(r['blocked_ratio']) for r in transition], 'violations':sum(int(r['violations']) for r in transition)}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
