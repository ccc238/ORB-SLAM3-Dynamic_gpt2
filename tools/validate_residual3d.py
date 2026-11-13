#!/usr/bin/env python3
"""Real RGB-D diagnostic acceptance; temporary YAML only, single-process runs.
Usage: python3 tools/validate_residual3d.py [frames=40] [output_directory]
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
COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 40
OUT = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else ROOT / 'outputs/residual3d-validation'
OUT.mkdir(parents=True, exist_ok=True)
BASE = (ROOT / 'config/TUM3-instance-association.yaml').read_text()
DATA = Path('/home/aaa/Datasets/TUM')

def setting(text, key, value):
    text = re.sub(r'(?m)^' + re.escape(key) + r':.*$', '', text)
    return text if value is None else text + '\n' + key + ': ' + str(value) + '\n'

def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))

def stats(values):
    values = sorted(values)
    return {'n': len(values), 'median_m': values[len(values)//2] if values else None,
            'p90_m': values[int(.9*(len(values)-1))] if values else None}

def supplement(summary, output):
    from collections import Counter
    import struct
    from PIL import Image
    # Classes come from existing association metadata; preserve selected/person settings.
    for name, entry in summary.items():
        folder = output / name
        path = folder / 'debug/features.csv'
        if not path.exists():
            continue
        classes = {(a['frame_id'], a['instance_id']): int(a['class_id'])
                   for a in rows(folder / 'association/association_instances.csv')}
        features = rows(path)
        humans = [a for a in features if classes.get((a['frame_id'], a['instance_id'])) == 0]
        values = [float(a['residual_3d_m']) for a in humans if math.isfinite(float(a['residual_3d_m']))]
        entry['person_residuals'] = stats(values)
        entry['person_states'] = dict(Counter(a['depth_state'] for a in humans))
        entry['feature_reasons'] = dict(Counter(a['reason'] for a in features))
        entry['person_epipolar_consistent_3d_anomaly'] = sum(a['epipolar_consistent_3d_anomaly'] == '1' for a in humans)
        frames = rows(folder / 'debug/frames.csv')
        entry['mean_match_ms'] = sum(float(a['match_ms']) for a in frames)/len(frames)
        entry['mean_estimate_ms'] = sum(float(a['estimate_ms']) for a in frames)/len(frames)
        entry['mean_residual_ms'] = sum(float(a['residual_ms']) for a in frames)/len(frames)
        previous = {a['frame_id']: a['previous_frame_id'] for a in frames}
        labels = {(a['frame_id'], a['feature_index']): a['instance_id']
                  for a in rows(folder / 'association/association_features.csv')}
        for a in features:
            if a['pnp_background_inlier'] != '-1':
                assert labels[(previous[a['frame_id']], a['previous_feature_id'])] == '0'
        if name in ('walking_3d_only', 'walking_both', 'sitting_both'):
            coords = {(a['frame_id'], a['feature_index']): (float(a['u']), float(a['v']))
                      for a in rows(folder / 'association/association_features.csv')}
            models = {}
            errors = []
            for a in features:
                if not math.isfinite(float(a['residual_3d_m'])):
                    continue
                fid = a['frame_id']
                if fid not in models:
                    models[fid] = json.loads((folder / ('debug/motion_'+fid+'.json')).read_text())
                model = models[fid]
                assert model['motion_valid'] == 1
                K = model['K']['data']
                if model['K']['dt'] == 'f':
                    K = [struct.unpack('f', struct.pack('f', v))[0] for v in K]
                R = model['R']['data']
                t = model['t_m']['data']
                def point(uv, z):
                    return [(uv[0]-K[2])*z/K[0], (uv[1]-K[5])*z/K[4], z]
                p = point(coords[(previous[fid], a['previous_feature_id'])], float(a['previous_depth_m']))
                q = point(coords[(fid, a['feature_id'])], float(a['current_depth_m']))
                vector = [q[i]-sum(R[3*i+j]*p[j] for j in range(3))-t[i] for i in range(3)]
                errors.extend(abs(vector[i]-float(a['residual_'+axis+'_m'])) for i, axis in enumerate('xyz'))
            assert max(errors) < 1e-10
            entry['independent_real_residual_vector_max_error_m'] = max(errors)
        if name in ('walking_3d_only', 'walking_both'):
            data = DATA / 'rgbd_dataset_freiburg3_walking_xyz'
            first = next(a for a in (data / 'associate.txt').read_text().splitlines() if a and not a.startswith('#')).split()
            depth_path = data / first[3]
            png = depth_path.read_bytes()
            assert png[24] == 16 and png[25] == 0  # PNG IHDR bit depth / grayscale
            image = Image.open(depth_path)
            coords = {(a['frame_id'], a['feature_index']): (float(a['u']), float(a['v']))
                      for a in rows(folder / 'association/association_features.csv')}
            errors = []
            for a in features:
                if a['frame_id'] != '0':
                    continue
                u, v = coords[('0', a['feature_id'])]
                raw = image.getpixel((int(u), int(v)))
                measured = float(a['current_depth_m'])
                if raw > 0:
                    errors.append(abs(measured-raw/5000.))
                else:
                    assert measured == -1 and a['current_depth_valid'] == '0'
            assert max(errors) < 1e-6
            entry['depth_scaling_check'] = {'positive_samples': len(errors), 'max_abs_error_m': max(errors),
                                          'raw_png_bits': 16, 'factor_raw_per_metre': 5000}
    a = output / 'walking_3d_only/association/association_features.csv'
    b = output / 'walking_both/association/association_features.csv'
    assert a.read_bytes() == b.read_bytes()
    disabled = rows(output / 'disabled/association/association_features.csv')
    enabled = rows(a)
    assert disabled == [r for r in enabled if int(r['frame_id']) < 5]
    left = rows(output / 'walking_3d_only/debug/features.csv')
    right = rows(output / 'walking_both/debug/features.csv')
    excluded = {'epipolar_residual_px', 'epipolar_state', 'epipolar_consistent_3d_anomaly'}
    assert [{k:v for k,v in r.items() if k not in excluded} for r in left] == [{k:v for k,v in r.items() if k not in excluded} for r in right]
    summary['independence_and_invariants'] = {'walking_3d_results_equal_with_epipolar_on_off': True,
        'walking_association_features_equal_with_epipolar_on_off': True,
        'first_five_association_features_equal_with_both_modules_off': True}
    return summary

summary = {}
cases = [
    ('walking_3d_only', 'walking_xyz', COUNT, {'Dynamic.UseEpipolar': 1}, False),
    ('sitting_both', 'sitting_static', COUNT, {'Dynamic.UseEpipolar': 0}, False),
    ('walking_both', 'walking_xyz', COUNT, {'Dynamic.UseEpipolar': 0}, False),
    ('disabled', 'walking_xyz', 5, {'Dynamic.Use3DResidual': 1}, False),
    ('missing', 'walking_xyz', 5, {'Dynamic.Use3DResidual': None}, False),
    ('invalid', 'walking_xyz', 1, {'Dynamic.Use3DResidual': 2}, True),
    ('float', 'walking_xyz', 1, {'Dynamic.Use3DResidual': '0.0'}, True),
    ('string', 'walking_xyz', 1, {'Dynamic.Use3DResidual': '"0"'}, True),
    ('prerequisite', 'walking_xyz', 1, {'Dynamic.Enable': 0}, True),
    ('insufficient_background', 'walking_xyz', 5, {'Residual3D.MinBackground': 10000}, False),
    ('no_depth', 'walking_xyz', 5, {'Residual3D.MinDepthM': 20, 'Residual3D.MaxDepthM': 30}, False),
    ('resized', 'walking_xyz', 5, {'Camera.newWidth': 480, 'Camera.newHeight': 360}, False),
]
with tempfile.TemporaryDirectory(prefix='orb-residual3d-', dir='/tmp') as temporary:
    for name, sequence, count, extra, error in cases:
        folder = OUT / name
        assert not folder.exists(), f'Preserve existing evidence: {folder}'
        folder.mkdir()
        debug = folder / 'debug'
        values = {'Residual3D.UseDepthDifferenceGate': 0, 'Dynamic.DepthPoseSource': '"background_pnp"', 'Dynamic.Use3DResidual': 0, 'Dynamic.UseEpipolar': 1, 'Run.Viewer': 0,
                  'Residual3D.Debug': 1, 'Residual3D.Visualize': 1,
                  'Residual3D.OutputDirectory': '"'+str(debug)+'"',
                  'Epipolar.Debug': 1, 'Epipolar.Visualize': 0,
                  'Epipolar.OutputDirectory': '"'+str(folder / 'epipolar')+'"',
                  'SemanticMask.AssociationDebug': 1, 'SemanticMask.Visualize': 0,
                  'SemanticMask.OutputDirectory': '"'+str(folder / 'association')+'"', **extra}
        text = BASE
        for key, value in values.items():
            text = setting(text, key, value)
        config = Path(temporary) / (name + '.yaml')
        config.write_text(text)
        data = DATA / ('rgbd_dataset_freiburg3_' + sequence)
        env = os.environ.copy()
        env.pop('DISPLAY', None)
        cmd = [str(ROOT / 'Examples/RGB-D/rgbd_tum'), str(ROOT / 'Vocabulary/ORBvoc.txt'),
               str(config), str(data), str(data / 'associate.txt'), str(folder), str(count)]
        with (folder / 'run.log').open('w') as log:
            run = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, env=env, timeout=300)
        log = (folder / 'run.log').read_text()
        assert (run.returncode != 0) == error, (name, run.returncode, log[-1500:])
        if error:
            assert 'Residual3D:' in log
            summary[name] = {'expected_error': log.splitlines()[-1]}
            print(name, summary[name], flush=True)
            continue
        slam = rows(folder / 'RunFrames.csv')
        association = rows(folder / 'association/association_frames.csv')
        assert len(slam) == len(association) == count
        assert all(a['features_before'] == a['features_after'] == a['instance_labels_length'] for a in association)
        summary[name] = {'frames': count, 'feature_counts': [int(a['keypoints']) for a in slam]}
        if values['Dynamic.Use3DResidual'] != 0:
            assert not debug.exists() and '[Residual3D]' not in log and '[Epipolar]' not in log
            print(name, 'PASS disabled', flush=True)
            continue
        frames = rows(debug / 'frames.csv')
        features = rows(debug / 'features.csv')
        assert len(frames) == count
        by_frame = {}
        residuals = {'background_fit': [], 'background_nonfit': [], 'instance': []}
        reasons = {}
        combos = 0
        mapping = {}
        for a in rows(folder / 'association/association_features.csv'):
            mapping.setdefault(a['frame_id'], {})[int(a['feature_index'])] = int(a['instance_id'])
        for f in features:
            by_frame.setdefault(f['frame_id'], []).append(f)
            j, i = int(f['feature_id']), int(f['previous_feature_id'])
            assert int(f['instance_id']) == mapping[f['frame_id']][j]
            if values['Dynamic.UseEpipolar'] == 1:
                assert f['epipolar_residual_px'] == f['epipolar_state'] == ''
            value = float(f['residual_3d_m'])
            if f['depth_valid'] == '0' or i < 0:
                assert f['depth_state'] == 'UNKNOWN' and math.isnan(value)
            if math.isfinite(value):
                assert f['depth_valid'] == '1'
                v = [float(f['residual_'+axis+'_m']) for axis in 'xyz']
                assert abs(math.sqrt(sum(a*a for a in v))-value) < 1e-10
                bucket = 'instance' if int(f['instance_id']) > 0 else 'background_fit' if f['pnp_background_inlier'] == '1' else 'background_nonfit'
                residuals[bucket].append(value)
            if f['pnp_background_inlier'] != '-1':
                assert int(f['instance_id']) == 0
            combos += f['epipolar_consistent_3d_anomaly'] == '1'
        for f, a in zip(frames, association):
            keys = by_frame[f['frame_id']]
            assert len(keys) == int(a['total_features'])
            assert [int(k['feature_id']) for k in keys] == list(range(len(keys)))
            prev = [int(k['previous_feature_id']) for k in keys if int(k['previous_feature_id']) >= 0]
            assert len(prev) == len(set(prev))
            reasons[f['reason']] = reasons.get(f['reason'], 0)+1
            if f['motion_valid'] == '0':
                assert all(k['depth_state'] == 'UNKNOWN' and math.isnan(float(k['residual_3d_m'])) for k in keys)
        if name in ('insufficient_background', 'no_depth'):
            assert all(f['motion_valid'] == '0' for f in frames)
        if name in ('walking_3d_only', 'walking_both', 'sitting_both'):
            assert any(f['motion_valid'] == '1' for f in frames)
        summary[name].update({'motion_reasons': reasons, 'residuals': {k: stats(v) for k, v in residuals.items()},
                              'epipolar_consistent_3d_anomaly': combos})
        print(name, summary[name], flush=True)
        (OUT / 'summary.json').write_text(json.dumps(summary, indent=2))
    assert summary['walking_3d_only']['feature_counts'][:5] == summary['disabled']['feature_counts'] == summary['missing']['feature_counts']
    assert summary['walking_3d_only']['feature_counts'] == summary['walking_both']['feature_counts']
supplement(summary, OUT)
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2))
print('PASS real RGB-D acceptance:', OUT)
