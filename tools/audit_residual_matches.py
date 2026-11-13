#!/usr/bin/env python3
"""Read-only correspondence audit. Replays only to capture Tracking evidence.

Never changes correspondences, depth gates or residual thresholds. Existing run
residuals are kept separate from fresh Tracking associations (BA is asynchronous).
Requires numpy, Pillow and PyYAML; uses no Python OpenCV dependency.
"""
import argparse
import csv
import gzip
import json
import math
import os
import re
import subprocess
import tempfile
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
LUT = np.array([int(i).bit_count() for i in range(256)], dtype=np.uint8)


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def write(path, rows):
    if rows:
        with path.open('w') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


class Loader(getattr(yaml, 'CSafeLoader', yaml.SafeLoader)):
    pass


Loader.add_constructor('tag:yaml.org,2002:opencv-matrix',
                       lambda loader, node: loader.construct_mapping(node, deep=True))


def load(path):
    with gzip.open(path, 'rt') as f:
        return yaml.load(f.read().replace('%YAML:1.0', '%YAML 1.1'), Loader=Loader)


def descriptors(frame):
    m = frame['descriptors']
    return np.asarray(m['data'], dtype=np.uint8).reshape(m['rows'], m['cols'])


def mapping(snapshot, i, j):
    if not snapshot or 'previous' not in snapshot:
        return 'NO_STAGE', [], []
    prev, cur = snapshot['previous']['map_points'], snapshot['current']['map_points']
    a, b = prev[i], cur[j]
    p, c = a['point_id'], b['point_id']
    alt_p = [x['feature_id'] for x in prev if c != 'NONE' and x['point_id'] == c]
    alt_c = [x['feature_id'] for x in cur if p != 'NONE' and x['point_id'] == p]
    if p == c and p != 'NONE':
        state = 'SAME_POINT_UNIQUE' if len(alt_p) == len(alt_c) == 1 else 'SAME_POINT_AMBIGUOUS'
        if a['outlier'] or b['outlier']:
            state += '_OUTLIER'
    elif alt_p or alt_c:
        state = 'DIFFERENT_PAIR_FOR_ENDPOINT'
    elif p == c == 'NONE':
        state = 'BOTH_WITHOUT_POINT'
    else:
        state = 'NO_SHARED_POINT_PAIR'
    return state, alt_p, alt_c


def nearest(query, pool):
    distances = LUT[np.bitwise_xor(pool, query)].sum(axis=1)
    order = np.argsort(distances, kind='stable')[:2]
    a, b = (int(distances[k]) for k in order)
    return int(order[0]), int(order[1]), a, b, a / b if b else math.nan


def boundary(labels, key):
    x, y = int(key[0]), int(key[1])
    value = int(labels[y, x])
    # OpenCV KeyPoint sequence: x,y,size,angle,response,octave,class_id.
    radius = int(math.ceil(15.5 * 1.2 ** int(key[5])))
    patch = labels[max(0, y-radius):y+radius+1, max(0, x-radius):x+radius+1]
    yy, xx = np.nonzero(patch != value)
    distance = min(np.hypot(xx+max(0, x-radius)-x, yy+max(0, y-radius)-y), default=math.inf)
    return value, radius, float(distance), float(np.mean(patch != value))


def analyze(source, replay, dataset, output):
    output.mkdir(exist_ok=False)
    originals, fresh = read(source/'debug/features.csv'), read(replay/'debug/features.csv')
    assert len(originals) == len(fresh)
    # Prove original index/label/depth inputs agree before linking old residuals to replay.
    keys = ('frame_id', 'feature_id', 'previous_feature_id', 'instance_id',
            'previous_depth_m', 'current_depth_m', 'previous_raw_u', 'previous_raw_v',
            'current_raw_u', 'current_raw_v')
    for a, b in zip(originals, fresh):
        assert all(a[k] == b[k] for k in keys), (a['frame_id'], a['feature_id'])
    grouped = defaultdict(list)
    for row in originals:
        if int(row['previous_feature_id']) >= 0:
            grouped[int(row['frame_id'])].append(row)
    classes = {(r['frame_id'], r['instance_id']): r['class_name'] for r in
               read(source/'association/association_instances.csv')}
    files = [line.split() for line in (dataset/'associate.txt').read_text().splitlines()
             if line.strip() and not line.startswith('#')]
    overlaps, evidence, selected = [], [], []
    for fid, rows in grouped.items():
        final = load(replay/f'tracking/final_{fid}.yml.gz')
        stagepath = replay/f'tracking/motion_model_{fid}.yml.gz'
        stage = load(stagepath) if stagepath.exists() else None
        p, c = final['previous'], final['current']
        assert int(c['frame_id']) == fid and int(p['frame_id']) == int(rows[0]['previous_frame_id'])
        assert p['timestamp'] < c['timestamp']
        if stage:
            assert stage['previous']['frame_id'] == p['frame_id'] and stage['current']['frame_id'] == c['frame_id']
        pd, cd = descriptors(p), descriptors(c)
        masks = {}
        for side, frame in [('previous', p), ('current', c)]:
            masks[side] = np.asarray(Image.open(source/f"association/association_{frame['frame_id']}_labels.png"))
        for r in rows:
            i, j = int(r['previous_feature_id']), int(r['feature_id'])
            assert np.max(np.abs(np.array(p['raw_keys'][i][:2]) - [float(r['previous_raw_u']), float(r['previous_raw_v'])])) < 1e-4
            assert np.max(np.abs(np.array(c['raw_keys'][j][:2]) - [float(r['current_raw_u']), float(r['current_raw_v'])])) < 1e-4
            final_state, fp, fc = mapping(final, i, j)
            motion_state, mp, mc = mapping(stage, i, j)
            high = math.isfinite(float(r['residual_3d_m'])) and float(r['residual_3d_m']) >= .08
            base = dict(frame_id=fid, previous_frame_id=p['frame_id'], feature_id=j, previous_feature_id=i,
                        original_residual_3d_m=r['residual_3d_m'], retained_high=int(high),
                        motion_model_relation=motion_state, motion_model_previous_indices=json.dumps(mp),
                        motion_model_current_indices=json.dumps(mc), final_relation=final_state,
                        final_previous_indices=json.dumps(fp), final_current_indices=json.dumps(fc))
            overlaps.append(base)
            if not high:
                continue
            fj, fs, d1, d2, ratio = nearest(pd[i], cd)
            bi, bs, bd1, bd2, bratio = nearest(cd[j], pd)
            assert fj == j and bi == i and d1 < .75*d2 and bd1 < .75*bd2
            row = dict(base, forward_best_index=fj, forward_second_index=fs,
                       forward_best_distance=d1, forward_second_distance=d2, forward_ratio=ratio,
                       reverse_best_index=bi, reverse_second_index=bs, reverse_best_distance=bd1,
                       reverse_second_distance=bd2, reverse_ratio=bratio, mutual_nearest=1,
                       pixel_displacement_px=math.hypot(float(r['current_raw_u'])-float(r['previous_raw_u']),
                                                        float(r['current_raw_v'])-float(r['previous_raw_v'])))
            names = []
            for side, frame, index in [('previous', p, i), ('current', c, j)]:
                label, radius, distance, fraction = boundary(masks[side], frame['raw_keys'][index])
                name = 'background' if label == 0 else classes.get((frame['frame_id'], str(label)), 'UNKNOWN')
                names.append(name)
                row.update({side+'_instance': label, side+'_class': name,
                            side+'_descriptor_patch_radius_px': radius,
                            side+'_nearest_mask_change_px': distance,
                            side+'_patch_other_label_fraction': fraction})
            row['different_semantic_class'] = int(names[0] != names[1])
            row['mask_boundary_within_5px'] = int(min(row['previous_nearest_mask_change_px'], row['current_nearest_mask_change_px']) <= 5)
            evidence.append(row)
        # Limit pictures to prior selected samples, not a new residual-based selection.
        prior = source/'retained_large_residuals/large_residual_matches.csv'
        sample_ids = {(int(x['frame_id']), int(x['feature_id'])) for x in read(prior)}
        for row in [x for x in evidence if x['frame_id'] == fid and (fid, x['feature_id']) in sample_ids]:
            a = min(files, key=lambda a: abs(float(a[0])-p['timestamp']))
            b = min(files, key=lambda a: abs(float(a[0])-c['timestamp']))
            assert abs(float(a[0])-p['timestamp']) < 1e-6 and abs(float(b[0])-c['timestamp']) < 1e-6
            images = [Image.open(dataset/a[1]).convert('RGB'), Image.open(dataset/b[1]).convert('RGB')]
            canvas = Image.new('RGB', (1280, 590), '#171717')
            for k, im in enumerate(images):
                canvas.paste(im, (640*k, 110))
            draw = ImageDraw.Draw(canvas)
            def line(i, j, color, width):
                x, y = p['raw_keys'][i][:2]; u, v = c['raw_keys'][j][:2]
                draw.line((x, y+110, u+640, v+110), fill=color, width=width)
                for qx, qy in [(x, y+110), (u+640, v+110)]:
                    draw.ellipse((qx-5, qy-5, qx+5, qy+5), outline=color, width=2)
            i, j = row['previous_feature_id'], row['feature_id']
            # Tracking alternatives sharing an endpoint, not a filter or dynamic verdict.
            for ai in json.loads(row['final_previous_indices']):
                line(ai, j, 'cyan', 2)
            for aj in json.loads(row['final_current_indices']):
                line(i, aj, 'cyan', 2)
            line(i, j, 'yellow', 2)
            draw.text((8, 5), f"Original frame {p['frame_id']} -> {fid}: feature {i} -> {j}, residual {float(row['original_residual_3d_m']):.6f} m", fill='white')
            draw.text((8, 24), f"Hamming forward {row['forward_best_distance']}/{row['forward_second_distance']} ratio {row['forward_ratio']:.4f}; reverse {row['reverse_best_distance']}/{row['reverse_second_distance']} ratio {row['reverse_ratio']:.4f}; mutual PASS", fill='white')
            draw.text((8, 43), f"Motion-model stage: {row['motion_model_relation']}; final shared-map-point: {row['final_relation']}", fill='white')
            draw.text((8, 62), f"Classes {row['previous_class']} -> {row['current_class']}; mask changes within 5px: {row['mask_boundary_within_5px']}", fill='white')
            draw.text((8, 81), 'Yellow: original diagnostic pair. Cyan: replay Tracking shared-point alternative. NO FEATURES REMOVED.', fill='white')
            canvas.save(output/f'frame_{fid}_feature_{j}.png')
            selected.append(row)
    write(output/'all_pair_overlap.csv', overlaps)
    write(output/'retained_abnormal_matches.csv', evidence)
    write(output/'selected_matches.csv', selected)
    summary = dict(original_input_pairs_verified=len(originals), diagnostic_matches=len(overlaps),
                   retained_high=len(evidence), descriptor_bidirectional_pass=len(evidence),
                   different_class=sum(x['different_semantic_class'] for x in evidence),
                   mask_change_within_5px=sum(x['mask_boundary_within_5px'] for x in evidence),
                   motion_model_high=dict(Counter(x['motion_model_relation'] for x in evidence)),
                   final_high=dict(Counter(x['final_relation'] for x in evidence)),
                   final_all=dict(Counter(x['final_relation'] for x in overlaps)),
                   note='Original residuals + verified identical input indices + fresh replay Tracking associations; not historical Tracking ground truth. Mask proximity is a risk marker, not proof of occlusion.')
    (output/'summary.json').write_text(json.dumps(summary, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--analyze-only', action='store_true')
    parser.add_argument('--resume', action='store_true', help='Reuse complete captures and verified inspection summaries')
    args = parser.parse_args()
    if not args.analyze_only:
        args.output.mkdir(parents=True, exist_ok=args.resume)
    base = (ROOT/'config/TUM3-instance-association.yaml').read_text()
    summary = {}
    for name, seq, count in [('sitting_static120', 'sitting_static', 120), ('walking40_gate_on', 'walking_xyz', 40)]:
        run = args.output/name
        data = Path('/home/aaa/Datasets/TUM')/('rgbd_dataset_freiburg3_'+seq)
        complete = (run/'debug/frames.csv').exists() and len(read(run/'debug/frames.csv')) == count and (run/f'tracking/final_{count-1}.yml.gz').exists()
        if not args.analyze_only and not (args.resume and complete):
            (run/'tracking').mkdir(parents=True)
            values = {'Run.Viewer': 0, 'SemanticMask.Visualize': 0, 'SemanticMask.AssociationDebug': 1,
                      'SemanticMask.OutputDirectory': json.dumps(str((run/'association').resolve())),
                      'Residual3D.OutputDirectory': json.dumps(str((run/'debug').resolve())),
                      'Residual3D.Debug': 1, 'Residual3D.Visualize': 0, 'Epipolar.Debug': 0, 'Epipolar.Visualize': 0}
            text = base
            for k, v in values.items():
                text = re.sub(r'(?m)^'+re.escape(k)+r':.*$', '', text)+'\n'+k+': '+str(v)+'\n'
            with tempfile.TemporaryDirectory(prefix='orb-match-audit-', dir='/tmp') as tmp:
                config = Path(tmp)/'audit.yaml'; config.write_text(text)
                (run/'effective_settings.txt').write_text(text)
                env = os.environ.copy(); env.pop('DISPLAY', None)
                env['ORB_RESIDUAL_MATCH_AUDIT_DIR'] = str((run/'tracking').resolve())
                cmd = [str(ROOT/'Examples/RGB-D/rgbd_tum'), str(ROOT/'Vocabulary/ORBvoc.txt'), str(config), str(data), str(data/'associate.txt'), str(run.resolve()), str(count)]
                with (run/'run.log').open('w') as log:
                    subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=600)
        saved = run/'inspection/summary.json'
        if args.resume and saved.exists():
            summary[name] = json.loads(saved.read_text())
        else:
            summary[name] = analyze(ROOT/'outputs/depth-tracking-audit-v2'/name, run, data, run/'inspection')
        print(name, json.dumps(summary[name]), flush=True)
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
