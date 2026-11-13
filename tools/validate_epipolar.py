#!/usr/bin/env python3
"""Verify real RGB-D diagnostic integration. Temporary settings live outside the repository.
Usage: python3 tools/validate_epipolar.py [frames=60] [dataset]
"""
import csv
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 60
DATA = Path(sys.argv[2] if len(sys.argv) > 2 else '/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz')
OUT = ROOT / 'outputs/epipolar-validation'
OUT.mkdir(exist_ok=True)
BASE = (ROOT / 'config/TUM3-instance-association.yaml').read_text()

def setting(text, key, value):
    p = r'(?m)^' + re.escape(key) + r':.*$'
    text = re.sub(p, '', text)
    return text if value is None else text + '\n' + key + ': ' + str(value) + '\n'

def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))

summary = {}
with tempfile.TemporaryDirectory(prefix='orb-epipolar-', dir='/tmp') as temporary:
    for name, switch, n, extra, error in [
        ('enabled', '0', COUNT, {}, False),
        ('disabled', '1', 5, {}, False),
        ('missing', None, 5, {}, False),
        ('invalid', '2', 1, {}, True),
        ('float', '0.0', 1, {}, True),
        ('prerequisite', '0', 1, {'Dynamic.Enable': 0}, True),
        ('resized', '0', 8, {'Camera.newWidth': 480, 'Camera.newHeight': 360}, False),
        ('frequency', '0', 5, {'Epipolar.SaveEvery': 2}, False),
        ('quiet', '0', 5, {'Epipolar.Debug': 0, 'Epipolar.Visualize': 0}, False),
    ]:
        folder = OUT / name
        folder.mkdir(exist_ok=True)
        debug = folder / 'debug'
        assert not debug.exists(), f'{debug} exists; preserve previous evidence before rerunning'
        values = {'Dynamic.UseEpipolar': switch, 'Run.Viewer': 0,
                  'Epipolar.Debug': 1, 'Epipolar.Visualize': 1,
                  'Epipolar.OutputDirectory': '"'+str(debug)+'"',
                  'SemanticMask.AssociationDebug': 1,
                  'SemanticMask.OutputDirectory': '"'+str(folder / 'association')+'"',
                  'SemanticMask.Visualize': 0, **extra}
        text = BASE
        for key, value in values.items():
            text = setting(text, key, value)
        config = Path(temporary) / (name + '.yaml')
        config.write_text(text)
        env = os.environ.copy()
        env.pop('DISPLAY', None)
        cmd = [str(ROOT / 'Examples/RGB-D/rgbd_tum'), str(ROOT / 'Vocabulary/ORBvoc.txt'),
               str(config), str(DATA), str(DATA / 'associate.txt'), str(folder), str(n)]
        with (folder / 'run.log').open('w') as log:
            run = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, env=env, timeout=240)
        log = (folder / 'run.log').read_text()
        assert (run.returncode != 0) == error, (name, run.returncode, log[-1500:])
        if error:
            assert 'Epipolar:' in log, name
            summary[name] = {'expected_error': log.splitlines()[-1]}
            continue
        slam = rows(folder / 'RunFrames.csv')
        assoc = rows(folder / 'association/association_frames.csv')
        assert len(slam) == len(assoc) == n
        assert all(a['features_before'] == a['features_after'] == a['instance_labels_length'] for a in assoc)
        summary[name] = {'frames': n, 'tracking_states': sorted({a['state'] for a in slam}),
                         'feature_counts': [int(a['keypoints']) for a in slam]}
        if switch != '0' or name == 'quiet':
            assert not debug.exists() and '[Epipolar]' not in log
            continue
        frames = rows(debug / 'frames.csv')
        features = rows(debug / 'features.csv')
        by_frame = {}
        association = {}
        for a in rows(folder / 'association/association_features.csv'):
            association.setdefault(a['frame_id'], {})[int(a['feature_index'])] = int(a['instance_id'])
        for f in features:
            by_frame.setdefault(f['frame_id'], []).append(f)
        reasons = {}
        valid = 0
        for f in frames:
            reasons[f['reason']] = reasons.get(f['reason'], 0) + 1
            keys = by_frame[f['frame_id']]
            assert len(keys) == int(f['total_features'])
            assert [int(k['current_feature_index']) for k in keys] == list(range(len(keys)))
            assert int(f['unknown']) + int(f['consistent']) + int(f['suspected']) == len(keys)
            for column, state in [('unknown', 'UNKNOWN'), ('consistent', 'GEOMETRY_CONSISTENT'), ('suspected', 'SUSPECTED_DYNAMIC')]:
                assert int(f[column]) == sum(k['state'] == state for k in keys)
            matched = set()
            for k in keys:
                j, i = int(k['current_feature_index']), int(k['previous_feature_index'])
                assert int(k['current_instance_id']) == association[f['frame_id']][j]
                if i >= 0:
                    assert i not in matched
                    matched.add(i)
                    assert int(k['previous_instance_id']) == association[f['previous_frame_id']][i]
                if k['state'] != 'UNKNOWN':
                    assert f['model_valid'] == '1' and math.isfinite(float(k['epipolar_residual_px']))
                if f['model_valid'] == '0':
                    assert k['state'] == 'UNKNOWN' and math.isnan(float(k['epipolar_residual_px']))
            assert len(matched) == int(f['matches'])
            valid += int(f['model_valid'])
        assert frames[0]['reason'] == 'NO_PREVIOUS_FRAME'
        expected_saved = (n+1)//2 if name == 'frequency' else n
        assert len(frames) == expected_saved
        summary[name].update(model_valid_frames=valid, reasons=reasons,
                             matches=[int(f['matches']) for f in frames],
                             mean_match_ms=sum(float(f['match_ms']) for f in frames)/len(frames),
                             mean_estimate_ms=sum(float(f['estimate_ms']) for f in frames)/len(frames),
                             mean_residual_ms=sum(float(f['residual_ms']) for f in frames)/len(frames),
                             consistent=sum(int(f['consistent']) for f in frames),
                             suspected=sum(int(f['suspected']) for f in frames))
    assert summary['enabled']['model_valid_frames'] > 0, summary['enabled']
    # Same input and unfiltered extraction must yield identical feature counts across switches.
    assert summary['enabled']['feature_counts'][:5] == summary['disabled']['feature_counts'] == summary['missing']['feature_counts']
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
print(json.dumps(summary, indent=2, ensure_ascii=False))
