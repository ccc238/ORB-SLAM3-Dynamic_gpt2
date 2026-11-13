#!/usr/bin/env python3
"""Audit an exported epipolar frame without changing matching or classification.
Requires numpy/Pillow. The line overlay currently supports zero-distortion TUM images.
Example: python3 tools/inspect_epipolar_frame.py outputs/epipolar-inspection 20
"""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def write(path, rows):
    if not rows:
        return
    with path.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def matrix(model, key):
    m = model[key]
    return np.array(m['data'], dtype=float).reshape(m['rows'], m['cols'])


def point(row, prefix):
    return np.array([float(row[prefix+'_u']), float(row[prefix+'_v']), 1.])


def sampson(F, p, q):
    l, m = F @ p, F.T @ q
    return abs(q @ l) / np.sqrt(l[0]**2+l[1]**2+m[0]**2+m[1]**2)


def clipped_line(line, width, height):
    a, b, c = line
    hits = []
    if abs(b) > 1e-12:
        for x in [0, width-1]:
            y = -(a*x+c)/b
            if 0 <= y <= height-1:
                hits.append((x, y))
    if abs(a) > 1e-12:
        for y in [0, height-1]:
            x = -(b*y+c)/a
            if 0 <= x <= width-1:
                hits.append((x, y))
    return hits[:2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('frame', type=int, nargs='?', default=20)
    parser.add_argument('--dataset', type=Path, default=Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz'))
    args = parser.parse_args()
    debug = args.run / 'debug'
    association = args.run / 'association'
    out = args.run / ('audit_frame_'+str(args.frame))
    out.mkdir(parents=True, exist_ok=True)
    all_features = read(debug / 'features.csv')
    features = [r for r in all_features if int(r['frame_id']) == args.frame]
    assert features
    current = str(args.frame)
    previous = features[0]['previous_frame_id']
    model = json.loads((debug / ('model_'+current+'.json')).read_text())
    assert model['model_valid'] == 1
    F = matrix(model, 'F')
    assert F.shape == (3, 3) and np.isfinite(F).all()
    # Straight epilines are drawn on this actual TUM image only after confirming zero distortion.
    assert np.max(np.abs(matrix(model, 'distortion'))) == 0, 'Use an undistorted canvas for nonzero distortion'
    classes = {}
    for r in read(association / 'association_instances.csv'):
        classes[(r['frame_id'], int(r['instance_id']))] = r['class_name']
    assoc = {}
    for r in read(association / 'association_features.csv'):
        if r['frame_id'] in [current, previous]:
            assoc[(r['frame_id'], int(r['feature_index']))] = r
    label_maps = {fid: np.asarray(Image.open(association / ('association_'+fid+'_labels.png'))) for fid in [previous, current]}
    associations = [line.split() for line in (args.dataset / 'associate.txt').read_text().splitlines() if line.strip() and not line.startswith('#')]
    frame_records = read(debug / 'frames.csv')
    times = {r['frame_id']: r['timestamp'] for r in frame_records}
    def load_rgb(fid):
        row = min(associations, key=lambda a: abs(float(a[0])-float(times[fid])))
        assert abs(float(row[0])-float(times[fid])) < 1e-6
        image = Image.open(args.dataset / row[1]).convert('RGB')
        assert image.size == (model['width'], model['height'])
        return image
    images = {fid: load_rgb(fid) for fid in [previous, current]}
    masks, bands, boxes = {}, {}, {}
    for fid in [previous, current]:
        labels = label_maps[fid]
        people = [iid for (f, iid), name in classes.items() if f == fid and name == 'person']
        masks[fid] = np.isin(labels, people)
        # Five-pixel exterior band is an audit candidate set, never a classification change.
        dilated = np.asarray(Image.fromarray(masks[fid].astype(np.uint8)*255).filter(ImageFilter.MaxFilter(11))) > 0
        bands[fid] = dilated & ~masks[fid]
        boxes[fid] = []
        for iid in people:
            yy, xx = np.where(labels == iid)
            if len(xx):
                boxes[fid].append((int(xx.min()), int(yy.min()), int(xx.max()), int(yy.max())))
    def at(mask, xy):
        x, y = math.floor(float(xy[0])), math.floor(float(xy[1]))
        assert 0 <= x < mask.shape[1] and 0 <= y < mask.shape[0]
        return mask[y, x].item()
    def in_person_box(fid, xy):
        return any(x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1 for x0, y0, x1, y1 in boxes[fid])
    background, leakage_candidates, people_green = [], [], []
    error_max = 0.
    for r in features:
        j, i = int(r['current_feature_index']), int(r['previous_feature_index'])
        q_raw = point(r, 'current_raw')
        assert int(r['current_instance_id']) == at(label_maps[current], q_raw)
        assert int(r['current_instance_id']) == int(assoc[(current, j)]['instance_id'])
        if i < 0:
            assert r['state'] == 'UNKNOWN' and r['reason'] == 'NO_MATCH'
            continue
        p_raw = point(r, 'previous_raw')
        assert int(r['previous_instance_id']) == at(label_maps[previous], p_raw)
        assert int(r['previous_instance_id']) == int(assoc[(previous, i)]['instance_id'])
        p, q = point(r, 'previous_undistorted'), point(r, 'current_undistorted')
        assert np.max(np.abs(p-p_raw)) < 1e-5 and np.max(np.abs(q-q_raw)) < 1e-5
        residual = sampson(F, p, q)
        error_max = max(error_max, abs(residual-float(r['epipolar_residual_px'])))
        assert abs(residual-float(r['epipolar_residual_px'])) < 1e-8
        expected = 'GEOMETRY_CONSISTENT' if residual <= 1 else 'SUSPECTED_DYNAMIC' if residual >= 3 else 'UNKNOWN'
        assert r['state'] == expected
        assert r['reason'] == {'GEOMETRY_CONSISTENT': 'LOW_RESIDUAL', 'SUSPECTED_DYNAMIC': 'HIGH_RESIDUAL', 'UNKNOWN': 'AMBIGUOUS_RESIDUAL'}[expected]
        is_background = int(r['previous_instance_id']) == int(r['current_instance_id']) == 0
        assert (int(r['background_ransac_inlier']) >= 0) == is_background
        extra = dict(r)
        extra.update(previous_class=classes.get((previous, int(r['previous_instance_id'])), 'background'),
                     current_class=classes.get((current, int(r['current_instance_id'])), 'background'),
                     residual_recomputed_px=residual)
        if is_background:
            extra.update(previous_in_person_box=in_person_box(previous, p_raw),
                         current_in_person_box=in_person_box(current, q_raw),
                         previous_near_person_mask_5px=bool(at(bands[previous], p_raw)),
                         current_near_person_mask_5px=bool(at(bands[current], q_raw)))
            background.append(extra)
            if extra['previous_in_person_box'] or extra['current_in_person_box'] or extra['previous_near_person_mask_5px'] or extra['current_near_person_mask_5px']:
                leakage_candidates.append(extra)
        if extra['current_class'] == 'person' and r['state'] == 'GEOMETRY_CONSISTENT':
            # Spread samples over the body and full residual range, rather than just selecting smallest residuals.
            l = F @ p
            extra.update(line_a=l[0], line_b=l[1], line_c=l[2],
                         current_point_to_epiline_px=abs(q @ l)/np.hypot(l[0], l[1]))
            people_green.append(extra)
    frame = next(r for r in frame_records if r['frame_id'] == current)
    assert len(background) == int(frame['background'])
    assert sum(int(r['background_ransac_inlier']) for r in background) == int(frame['inliers'])
    people_green.sort(key=lambda r: (int(r['current_instance_id']), float(r['current_raw_v']), float(r['current_raw_u'])))
    selected = [people_green[i] for i in np.linspace(0, len(people_green)-1, min(10, len(people_green)), dtype=int)]
    for idx, r in enumerate(selected, 1):
        r['sample_id'] = idx
    write(out / 'background_estimation_matches.csv', background)
    write(out / 'background_person_risk_candidates.csv', leakage_candidates)
    write(out / 'person_green_samples.csv', selected)
    unknown_reasons = Counter(r['reason'] for r in features if r['state'] == 'UNKNOWN')
    sequence_unknown = Counter(r['reason'] for r in all_features if r['state'] == 'UNKNOWN')
    summary = {'frame': frame, 'F': F.tolist(), 'background_count': len(background),
               'background_all_double_zero': True, 'background_person_risk_candidates': len(leakage_candidates),
               'background_near_person_5px': sum(r['previous_near_person_mask_5px'] or r['current_near_person_mask_5px'] for r in background),
               'person_green_total': len(people_green), 'max_residual_recompute_error_px': error_max,
               'unknown_reasons': dict(unknown_reasons), 'sequence_unknown_reasons': dict(sequence_unknown),
               'model_reasons': dict(Counter(r['reason'] for r in frame_records)),
               'leakage_note': 'Label zero and person box/boundary proximity do not establish true background. Inspect overlays; no manual human ground truth supplied.'}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 13)
    # Ten individual current-frame epilines: l_current = F * x_previous.
    tiles = []
    for r in selected:
        image = images[current].copy()
        draw = ImageDraw.Draw(image)
        l = np.array([r['line_a'], r['line_b'], r['line_c']])
        hits = clipped_line(l, *image.size)
        if len(hits) == 2:
            draw.line(hits, fill=(255, 0, 255), width=2)
        q = point(r, 'current_raw')
        draw.ellipse((q[0]-5, q[1]-5, q[0]+5, q[1]+5), outline=(0, 255, 0), width=2)
        label = f"#{r['sample_id']} prev {r['previous_feature_index']} -> cur {r['current_feature_index']} | r={r['residual_recomputed_px']:.4f}px | line_d={r['current_point_to_epiline_px']:.4f}px"
        tile = Image.new('RGB', (image.width, image.height+34))
        tile.paste(image, (0, 34)); ImageDraw.Draw(tile).text((5, 8), label, font=font, fill='white')
        tile.save(out / f"person_epiline_{r['sample_id']:02d}.png")
        tiles.append(tile)
    if tiles:
        grid = Image.new('RGB', (tiles[0].width*2, tiles[0].height*((len(tiles)+1)//2)))
        for idx, tile in enumerate(tiles):
            grid.paste(tile, ((idx%2)*tile.width, (idx//2)*tile.height))
        grid.save(out / 'person_epilines_grid.png')
    # Actual background estimation set, inliers/outliers, and person segmentation masks.
    panels = []
    for fid, prefix in [(previous, 'previous'), (current, 'current')]:
        image = np.array(images[fid]).copy()
        mask = masks[fid]
        image[mask] = (image[mask]*0.65+np.array([200, 30, 170])*0.35).astype(np.uint8)
        image = Image.fromarray(image); draw = ImageDraw.Draw(image)
        for r in background:
            q = point(r, prefix+'_raw'); color = (0, 255, 255) if int(r['background_ransac_inlier']) else (255, 80, 0)
            draw.ellipse((q[0]-3, q[1]-3, q[0]+3, q[1]+3), outline=color, width=2)
            if r in leakage_candidates:
                draw.text((q[0]+3, q[1]+1), r[prefix+'_feature_index'], font=font, fill='yellow')
        panel = Image.new('RGB', (image.width, image.height+46)); panel.paste(image, (0,46))
        ImageDraw.Draw(panel).text((5,3), f"frame {fid}: background estimation set = {len(background)}\ncyan=inlier orange=outlier magenta=person mask", font=font, fill='white')
        panels.append(panel)
    both = Image.new('RGB', (panels[0].width*2, panels[0].height))
    for i, panel in enumerate(panels): both.paste(panel, (i*panel.width,0))
    both.save(out / 'background_set.png')
    # Save both-frame raw/mask enlargements for every boundary-risk correspondence.
    for r in leakage_candidates:
        if not (r['previous_near_person_mask_5px'] or r['current_near_person_mask_5px']):
            continue
        tiles = []
        for fid, prefix in [(previous, 'previous'), (current, 'current')]:
            raw = images[fid]
            rgb = np.array(raw).copy()
            rgb[masks[fid]] = (rgb[masks[fid]]*0.6+np.array([200,30,170])*0.4).astype(np.uint8)
            q = point(r, prefix+'_raw')
            box = (int(q[0])-30, int(q[1])-30, int(q[0])+31, int(q[1])+31)
            for name, image in [('raw', raw), ('mask', Image.fromarray(rgb))]:
                crop = image.crop(box).resize((244,244))
                cx, cy = (q[0]-box[0])*4, (q[1]-box[1])*4
                ImageDraw.Draw(crop).ellipse((cx-7,cy-7,cx+7,cy+7),outline=(0,255,255),width=2)
                tile = Image.new('RGB',(244,275)); tile.paste(crop,(0,31))
                ImageDraw.Draw(tile).text((2,5),f"{fid} idx {r[prefix+'_feature_index']} {name}",font=font,fill='white')
                tiles.append(tile)
        crops = Image.new('RGB',(976,275))
        for idx, tile in enumerate(tiles): crops.paste(tile,(idx*244,0))
        crops.save(out / f"boundary_cur_{r['current_feature_index']}.png")
    # Yellow subreasons are explicit; original geometry colors and decisions remain untouched.
    canvas = Image.new('RGB', (images[current].width, images[current].height+90))
    canvas.paste(images[current], (0,90)); draw = ImageDraw.Draw(canvas)
    for r in features:
        q = point(r, 'current_raw'); x,y = q[0],q[1]+90
        color = (0,255,0) if r['state']=='GEOMETRY_CONSISTENT' else (255,0,0) if r['state']=='SUSPECTED_DYNAMIC' else (255,255,0)
        draw.ellipse((x-2,y-2,x+2,y+2),outline=color)
        if r['reason']=='AMBIGUOUS_RESIDUAL': draw.rectangle((x-4,y-4,x+4,y+4),outline=(255,255,0))
    draw.text((5,5), 'Geometry diagnostic; features NOT removed\nUNKNOWN: '+str(dict(unknown_reasons))+'\nyellow circle=NO_MATCH; yellow square=AMBIGUOUS_RESIDUAL\nmodel='+frame['reason']+'; see CSV for each reason', font=font, fill='white')
    canvas.save(out / 'unknown_reasons.png')
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
