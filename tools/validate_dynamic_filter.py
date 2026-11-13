#!/usr/bin/env python3
"""Ordered conservative-filter validation. Temporary settings live under /tmp only."""
import argparse
import csv
import json
import os
import re
import subprocess
import tempfile
from collections import Counter
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = Path('/home/aaa/Datasets/TUM')


def rows(path):
    with path.open() as f:
        data = list(csv.DictReader(f))
    assert all(None not in r for r in data), path
    return data


def stats(values):
    values = np.array(list(values), dtype=float)
    return dict(n=len(values), median=float(np.median(values)), p90=float(np.percentile(values, 90)), maximum=float(max(values))) if len(values) else {}


def audit(folder):
    run = rows(folder/'RunFrames.csv') if (folder/'RunFrames.csv').exists() else []
    if not run and (folder/'run.log').exists():
        events = re.findall(r'\[Lifecycle\] sample=(\d+) timestamp=([^ ]+) state=(\d+)', (folder/'run.log').read_text())
        run = [{'frame':i,'state':state,'track_ms':'nan'} for i,_,state in events]
    summary = {'frames': len(run), 'tracking_failures': sum(r['state'] != '2' for r in run),
               'track_ms': stats(float(r['track_ms']) for r in run if np.isfinite(float(r['track_ms'])))}
    if not (folder/'filter/frames.csv').exists():
        return summary
    frames = rows(folder/'filter/frames.csv')
    assert len(frames) == len(run) or not run, (folder, len(frames), len(run))
    frame_map = {r['frame_id']: r for r in frames}
    counts = Counter()
    excluded_by_frame = Counter()
    features_by_frame = Counter()
    with (folder/'filter/features.csv').open() as f:
        for r in csv.DictReader(f):
            counts['features'] += 1
            fr = frame_map[r['frame_id']]
            features_by_frame[r['frame_id']] += int(r['candidate'])
            if r['candidate'] == '1':
                assert int(r['instance_id']) > 0 and int(r['previous_index']) >= 0
                assert r['lk'] == 'SUPPORTED' and r['epipolar_state'] == r['depth_state'] == 'SUSPECTED_DYNAMIC'
                assert float(r['epipolar_residual']) >= 3 and float(r['residual3d']) >= .08
            if r['effective_excluded'] == '1':
                assert fr['accepted'] == '1' and r['candidate'] == '1'
                assert r['final_edge'] == r['allow_keyframe_observation'] == r['allow_new_point'] == '0'
                excluded_by_frame[r['frame_id']] += 1
            if fr['filter_mode'] == 'simulate':
                assert r['effective_excluded'] == '0'
                assert r['allow_keyframe_observation'] == r['allow_new_point'] == '1'
    for fr in frames:
        fid = fr['frame_id']
        assert features_by_frame[fid] == int(fr['candidates'])
        assert int(fr['original_inlier_candidates']) + int(fr['original_outlier_candidates']) == int(fr['optimizable_candidates'])
        if fr['accepted'] == '1':
            assert int(fr['final_edges']) == int(fr['pre_edges']) - int(fr['optimizable_candidates'])
            assert excluded_by_frame[fid] == int(fr['candidates'])
        else:
            assert not excluded_by_frame[fid]
            assert fr['final_edges'] == fr['pre_edges'] and fr['final_inliers'] == fr['pre_inliers']
        if fr['filter_mode'] == 'simulate':
            assert fr['accepted'] == fr['executed'] == '0'
            assert fr['simulation_unchanged'] == '1'
            assert float(fr['translation_m']) < 1e-6 and float(fr['rotation_rad']) < 1e-6
    summary.update({k: sum(int(r[k]) for r in frames) for k in ['candidates', 'optimizable_candidates', 'original_outlier_candidates', 'original_inlier_candidates', 'executed', 'accepted']})
    summary['effective_excluded'] = sum(excluded_by_frame.values())
    summary['effective_optimizer_excluded'] = sum(int(r['optimizable_candidates']) for r in frames if r['accepted'] == '1')
    summary['reasons'] = dict(Counter(r['reason'] for r in frames))
    summary['compute_ms'] = stats(float(r['compute_ms']) for r in frames)
    summary['record_ms'] = stats(float(r['record_ms']) for r in frames)
    summary['total_ms'] = stats(float(r['total_ms']) for r in frames)
    summary['remaining_inliers'] = stats(float(r['final_inliers']) for r in frames if r['eligible'] == '1')
    summary['candidate_frames'] = {r['frame_id']: int(r['candidates']) for r in frames if int(r['candidates'])}
    return summary


def transforms(path):
    result = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith('#'):
            continue
        v = list(map(float, line.split()))
        t, x, y, z, qx, qy, qz, qw = v[:8]
        q = np.array([qx,qy,qz,qw]); q /= np.linalg.norm(q)
        xq,yq,zq,wq = q
        rot = np.array([[1-2*(yq*yq+zq*zq),2*(xq*yq-zq*wq),2*(xq*zq+yq*wq)],
                        [2*(xq*yq+zq*wq),1-2*(xq*xq+zq*zq),2*(yq*zq-xq*wq)],
                        [2*(xq*zq-yq*wq),2*(yq*zq+xq*wq),1-2*(xq*xq+yq*yq)]])
        mat = np.eye(4);mat[:3,:3]=rot;mat[:3,3]=[x,y,z]
        result.append((t,mat))
    return result


def metrics(folder, sequence):
    truth = transforms(DATA/('rgbd_dataset_freiburg3_'+sequence)/'groundtruth.txt')
    est = transforms(folder/'CameraTrajectory.txt')
    times = np.array([a[0] for a in truth])
    pairs = []
    for t,mat in est:
        j = int(np.argmin(abs(times-t)))
        if abs(times[j]-t) <= .02:
            pairs.append((t,truth[j][1],mat))
    if len(pairs)<3:
        return {'matched':len(pairs)}
    gt = np.array([a[1][:3,3] for a in pairs]); pred = np.array([a[2][:3,3] for a in pairs])
    gc,pc = gt.mean(axis=0),pred.mean(axis=0)
    u,_,vt = np.linalg.svd((pred-pc).T@(gt-gc))
    sign = np.eye(3);sign[2,2]=np.linalg.det(vt.T@u.T)
    rot = vt.T@sign@u.T
    err = (rot@(pred-pc).T).T+gc-gt
    rel_t,rel_r = [],[]
    pt = np.array([a[0] for a in pairs])
    for i,(t,g,p) in enumerate(pairs):
        j = int(np.argmin(abs(pt-(t+1))))
        if j<=i or abs(pt[j]-(t+1))>.05:
            continue
        gdelta=np.linalg.inv(g)@pairs[j][1];pdelta=np.linalg.inv(p)@pairs[j][2]
        error=np.linalg.inv(gdelta)@pdelta
        rel_t.append(float(np.linalg.norm(error[:3,3])))
        rel_r.append(float(np.degrees(np.arccos(np.clip((np.trace(error[:3,:3])-1)/2,-1,1)))))
    rmse=lambda a: float(np.sqrt(np.mean(np.square(a)))) if len(a) else None
    return {'matched':len(pairs),'ate_se3_rmse_m':rmse(np.linalg.norm(err,axis=1)),
            'rpe_1s_pairs':len(rel_t),'rpe_1s_translation_rmse_m':rmse(rel_t),'rpe_1s_rotation_rmse_deg':rmse(rel_r)}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--stage',choices=['simulate','apply','recheck','full','repeat','analyze'],required=True)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/dynamic-filter-validation')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    base=(ROOT/'config/TUM3-instance-association.yaml').read_text()
    summary_path=args.output/'summary.json'
    summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}

    def run(name, sequence, n, mode, changes=None, lifecycle=False):
        folder=args.output/name
        if folder.exists():
            raise RuntimeError('Refusing to overwrite existing result '+str(folder))
        folder.mkdir()
        settings={'Run.Viewer':0,'SemanticMask.Visualize':0,'SemanticMask.AssociationDebug':0,
                  'SemanticMask.OutputDirectory':json.dumps(str((folder/'semantic').resolve())),
                  'Epipolar.Debug':1,'Epipolar.Visualize':0,'Epipolar.OutputDirectory':json.dumps(str((folder/'epipolar').resolve())),
                  'Residual3D.Debug':1,'Residual3D.Visualize':0,'Residual3D.OutputDirectory':json.dumps(str((folder/'depth').resolve())),
                  'LKCheck.Visualize':0,'LKCheck.OutputDirectory':json.dumps(str((folder/'lk').resolve())),
                  'Dynamic.UseDynamicFilter':1 if mode=='off' else 0,'Dynamic.FilterMode':json.dumps('simulate' if mode=='off' else mode),
                  'Dynamic.FilterOutputDirectory':json.dumps(str((folder/'filter').resolve()))}
        settings.update(changes or {})
        text=base
        for key,value in settings.items():
            text=re.sub(r'(?m)^'+re.escape(key)+r':.*$','',text)
            text+='\n'+key+': '+str(value)+'\n'
        dataset=DATA/('rgbd_dataset_freiburg3_'+sequence)
        with tempfile.TemporaryDirectory(prefix='orb-filter-',dir='/tmp') as tmp:
            config=Path(tmp)/'settings.yaml';config.write_text(text)
            (folder/'effective_settings.txt').write_text(text)
            cmd=[str(ROOT/'build-semantic/depth_pose_lifecycle_test'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(dataset)] if lifecycle else [str(ROOT/'Examples/RGB-D/rgbd_tum'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(dataset),str(dataset/'associate.txt'),str(folder.resolve()),str(n)]
            env=os.environ.copy();env.pop('DISPLAY',None);env.pop('ORB_RESIDUAL_MATCH_AUDIT_DIR',None)
            print('START',name,flush=True)
            with (folder/'run.log').open('w') as log:
                subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1800)
        result=audit(folder)
        if not lifecycle:result['metrics']=metrics(folder,sequence)
        summary[name]=result;summary_path.write_text(json.dumps(summary,indent=2))
        print('PASS',name,json.dumps(result),flush=True)
        return result

    if args.stage=='simulate':
        for mode in ['off','simulate']:
            run('sitting_short_'+mode,'sitting_static',120,mode)
    elif args.stage=='apply':
        assert 'sitting_short_simulate' in summary,'Validate simulate first'
        run('sitting_short_apply','sitting_static',120,'apply')
        for mode in ['off','simulate','apply']:
            run('walking_short_'+mode,'walking_xyz',80,mode)
        run('safety_remaining','walking_xyz',40,'apply',{'Dynamic.FilterMinRemaining':2000})
        run('safety_ratio','walking_xyz',40,'apply',{'Dynamic.FilterMaxRatio':.00001})
        run('safety_coverage','walking_xyz',40,'apply',{'Dynamic.FilterMinCells':12})
        run('forced_rollback','walking_xyz',40,'apply',{'Dynamic.FilterMinFinalInliers':2000})
        run('lifecycle','sitting_static',28,'apply',lifecycle=True)
    elif args.stage=='recheck':
        assert 'walking_short_apply' in summary
        run('walking_apply_recheck','walking_xyz',80,'apply')
        run('forced_rollback_recheck','walking_xyz',40,'apply',{'Dynamic.FilterMinFinalInliers':2000})
        run('safety_remaining_extended','walking_xyz',120,'apply',{'Dynamic.FilterMinRemaining':2000})
    elif args.stage=='full':
        assert 'lifecycle' in summary and 'walking_short_apply' in summary,'Validate short sequences first'
        for sequence,n,label in [('sitting_static',680,'sitting'),('walking_xyz',827,'walking')]:
            for mode in ['off','simulate','apply']:
                run(label+'_full_'+mode,sequence,n,mode)
    elif args.stage=='repeat':
        assert 'walking_full_apply' in summary
        for mode in ['off','apply']:
            run('walking_full_repeat_'+mode,'walking_xyz',827,mode)
    elif args.stage=='analyze':
        for folder in sorted(args.output.iterdir()):
            if folder.is_dir() and (folder/'RunFrames.csv').exists():
                s=audit(folder);sequence='sitting_static' if folder.name.startswith('sitting') else 'walking_xyz'
                s['metrics']=metrics(folder,sequence);summary[folder.name]=s
        summary_path.write_text(json.dumps(summary,indent=2))

if __name__=='__main__':
    main()
