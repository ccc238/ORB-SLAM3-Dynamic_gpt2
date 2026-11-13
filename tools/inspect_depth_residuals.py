#!/usr/bin/env python3
"""Export original two-frame RGB/depth patches and depth sampling evidence.
For older CSV without pixels, --coordinate-features accepts a deterministic replay;
all feature IDs, matching IDs, labels and both depths MUST agree before using it.
Only registered zero-distortion pinhole input is audited by this tool.
"""
import argparse,csv,json,math
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw

def read(path):
    with path.open() as f:return list(csv.DictReader(f))

def write(path,data):
    if not data:return
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('debug',type=Path);p.add_argument('dataset',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--coordinate-features',type=Path);p.add_argument('--per-group',type=int,default=6)
    p.add_argument('--only-retained',action='store_true',help='Inspect high residuals that remain after the depth gate')
    p.add_argument('--allow-existing',action='store_true',help='Regenerate only this inspection directory; never modify source logs')
    p.add_argument('--overview',action='store_true',help='Also export full-resolution two-frame RGB match overviews')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=args.allow_existing)
    records=read(args.debug/'features.csv');frames=read(args.debug/'frames.csv')
    byframe={r['frame_id']:r for r in frames}
    coordinate_map=None
    if 'current_raw_u' not in records[0]:
        if not args.coordinate_features:raise ValueError('Original CSV has no pixels; provide checked replay coordinates')
        coordinates=read(args.coordinate_features);coordinate_map={(r['frame_id'],r['feature_id']):r for r in coordinates}
        assert len(coordinates)==len(records),'Different input/extractor configuration'
        for r in records:
            q=coordinate_map[(r['frame_id'],r['feature_id'])]
            for key in ('previous_feature_id','instance_id','previous_depth_m','current_depth_m','previous_depth_valid','current_depth_valid'):
                assert q[key]==r[key],(key,r['frame_id'],r['feature_id'])
    association=(args.coordinate_features.parent.parent if coordinate_map is not None else args.debug.parent)/'association/association_instances.csv'
    classes={(r['frame_id'],r['instance_id']):r['class_name'] for r in read(association)} if association.exists() else {}
    label_path=association.parent/'association_features.csv'
    labels={(r['frame_id'],r['feature_index']):r['instance_id'] for r in read(label_path)} if label_path.exists() else {}
    def group(r):return 'background' if r['instance_id']=='0' else 'person' if classes.get((r['frame_id'],r['instance_id']))=='person' else 'other_instance'
    def raw_residual(r):return float(r.get('raw_residual_3d_m',r['residual_3d_m']))
    candidates=[r for r in records if math.isfinite(raw_residual(r)) and raw_residual(r)>=.08]
    if args.only_retained:candidates=[r for r in candidates if math.isfinite(float(r['residual_3d_m'])) and float(r['residual_3d_m'])>=.08]
    selected=[]
    for category in ('background','person','other_instance'):
        selected+=sorted([r for r in candidates if group(r)==category],key=raw_residual,reverse=True)[:args.per_group]
    associations=[x.split() for x in (args.dataset/'associate.txt').read_text().splitlines() if x.strip() and not x.startswith('#')]
    cache={}
    def load(time):
        time=float(time)
        if time not in cache:
            a=min(associations,key=lambda a:abs(float(a[0])-time));assert abs(float(a[0])-time)<1e-6
            rgb=Image.open(args.dataset/a[1]).convert('RGB');depth=np.asarray(Image.open(args.dataset/a[3]),dtype=np.float64)/5000.
            assert rgb.size==(depth.shape[1],depth.shape[0])
            cache[time]=(rgb,depth)
        return cache[time]
    depth_checks=0;all_depth_error=0.
    if coordinate_map is not None:
        # Old logs have no coordinates: verify all recoverable depth samples, not only selected outliers.
        for r in records:
            q=coordinate_map[(r['frame_id'],r['feature_id'])];frame=byframe[r['frame_id']]
            for side in ('previous','current'):
                if side=='previous' and int(r['previous_feature_id'])<0:continue
                time=frame[side+'_timestamp'];_,depth=load(time)
                x,y=int(float(q[side+'_raw_u'])),int(float(q[side+'_raw_v']))
                assert 0<=x<depth.shape[1] and 0<=y<depth.shape[0]
                pixel=float(depth[y,x]);logged=float(r[side+'_depth_m'])
                if pixel<=0:assert logged==-1
                else:
                    error=abs(pixel-logged);assert error<2e-6
                    all_depth_error=max(all_depth_error,error)
                depth_checks+=1
    cards=[];evidence=[]
    for r in selected:
        frame=byframe[r['frame_id']];model=json.loads((args.debug/('motion_'+r['frame_id']+'.json')).read_text())
        K=model['K'];fx,fy=K['data'][0],K['data'][4]
        assert abs(fx-535.4)<.001 and abs(fy-539.2)<.001,'This audit expects the original unscaled TUM3 calibration'
        q=coordinate_map[(r['frame_id'],r['feature_id'])] if coordinate_map is not None else r
        # Config used here is explicitly zero distortion, full resolution; no substitution of undistorted pixels in depth queries.
        for side in ('previous','current'):
            assert abs(float(q[side+'_raw_u'])-float(q[side+'_un_u']))<1e-5 and abs(float(q[side+'_raw_v'])-float(q[side+'_un_v']))<1e-5
        times=[frame['previous_timestamp'],frame['current_timestamp']]
        card=Image.new('RGB',(800,430),'#171717');draw=ImageDraw.Draw(card)
        data=dict(r);data['previous_instance_id']=labels.get((r['previous_frame_id'],r['previous_feature_id']),'UNAVAILABLE')
        data['previous_class_name']=classes.get((r['previous_frame_id'],data['previous_instance_id']),'background' if data['previous_instance_id']=='0' else 'UNAVAILABLE')
        data['class_name']=classes.get((r['frame_id'],r['instance_id']),'background' if r['instance_id']=='0' else 'unknown')
        def matrix(m):return np.asarray(m['data'],dtype=np.float32 if m['dt']=='f' else np.float64).reshape(m['rows'],m['cols']).astype(float)
        inverse=np.linalg.inv(matrix(model['K']));R=matrix(model['R']);t=matrix(model['t_m']).reshape(3)
        p=inverse@np.array([float(q['previous_un_u']),float(q['previous_un_v']),1.])*float(r['previous_depth_m'])
        c=inverse@np.array([float(q['current_un_u']),float(q['current_un_v']),1.])*float(r['current_depth_m'])
        vector=c-(R@p+t);saved=np.array([float(r['residual_'+axis+'_m']) for axis in 'xyz'])
        data['independent_residual_replay_error_m']=abs(float(np.linalg.norm(vector))-raw_residual(r))
        assert data['independent_residual_replay_error_m']<1e-10,'Coordinates do not reproduce the raw residual'
        data['independent_vector_replay_max_error_m']=float(np.max(np.abs(vector-saved))) if np.isfinite(saved).all() else None
        if data['independent_vector_replay_max_error_m'] is not None:
            assert data['independent_vector_replay_max_error_m']<1e-10,'Recovered coordinates do not reproduce the original vector'
        max_error=0
        for column,(side,time) in enumerate(zip(('previous','current'),times)):
            image,depth=load(time);u=float(q[side+'_raw_u']);v=float(q[side+'_raw_v']);x,y=int(u),int(v)
            measured=float(r[side+'_depth_m']);pixel=float(depth[y,x]);error=abs(pixel-measured);max_error=max(max_error,error)
            assert error<2e-6,(r['frame_id'],r['feature_id'],measured,pixel)
            radius=24;box=(x-radius,y-radius,x+radius,y+radius)
            rgb=image.crop(box).resize((288,288),Image.Resampling.NEAREST if hasattr(Image,'Resampling') else Image.NEAREST)
            valid=depth>0;visual=np.clip(depth/8,0,1)
            heat=np.stack([visual,1-np.abs(2*visual-1),1-visual],axis=2);heat[~valid]=0
            depth_image=Image.fromarray((heat*255).astype('uint8')).crop(box).resize((96,288),Image.NEAREST)
            origin=column*400;card.paste(rgb,(origin,70));card.paste(depth_image,(origin+290,70))
            for xx,yy,scale in [(origin,70,6),(origin+290,70,2)]:
                cx=xx+radius*scale+scale/2;cy=yy+radius*6+3
                draw.line((cx-8,cy,cx+8,cy),fill='magenta',width=2);draw.line((cx,cy-8,cx,cy+8),fill='magenta',width=2)
            draw.text((origin+4,365),f'{side}: raw ({u:.4f},{v:.4f})',fill='white')
            draw.text((origin+4,382),f'z_log={measured:.6f} m; PNG/5000={pixel:.6f}',fill='white')
            window=depth[max(0,y-1):y+2,max(0,x-1):x+2]
            data.update({side+'_raw_u':u,side+'_raw_v':v,side+'_un_u':float(q[side+'_un_u']),side+'_un_v':float(q[side+'_un_v']),side+'_depth_pixel_x':x,side+'_depth_pixel_y':y,
                         side+'_png_depth_m':pixel,side+'_depth_error_m':error,
                         side+'_window_min_m':float(window.min()),side+'_window_max_m':float(window.max())})
            np.save(args.output/(f"frame_{r['frame_id']}_feature_{r['feature_id']}_{side}_depth_m.npy"),depth[max(0,y-radius):y+radius,max(0,x-radius):x+radius])
        dz=abs(float(r['current_depth_m'])-float(r['previous_depth_m']));data['paper_0_20m_gate_would_reject']=int(dz>.2)
        data['coordinate_source']='checked deterministic replay' if coordinate_map is not None else 'same-run diagnostic CSV'
        draw.text((5,5),f"frame {r['previous_frame_id']} -> {r['frame_id']}, feature {r['previous_feature_id']} -> {r['feature_id']}, instance {r['instance_id']} ({data['class_name']})",fill='white')
        draw.text((5,25),f"raw 3D r={raw_residual(r):.6f} m; |dz|={dz:.6f} m; gate would reject={dz>.2}",fill='white')
        draw.text((5,45),'RGB crop + depth (fixed 0-8 m scale); magenta: exact sampled pixel; NOT a dynamic verdict',fill='white')
        filename=f"frame_{r['frame_id']}_feature_{r['feature_id']}.png";card.save(args.output/filename);data['image']=filename
        data['raw_pixel_displacement_px']=math.hypot(float(q['current_raw_u'])-float(q['previous_raw_u']),float(q['current_raw_v'])-float(q['previous_raw_v']))
        if args.overview:
            previous_image,_=load(times[0]);current_image,_=load(times[1]);width,height=previous_image.size
            overview=Image.new('RGB',(width*2,height+42),'#171717');overview.paste(previous_image,(0,42));overview.paste(current_image,(width,42));d=ImageDraw.Draw(overview)
            a=(float(q['previous_raw_u']),float(q['previous_raw_v'])+42);b=(float(q['current_raw_u'])+width,float(q['current_raw_v'])+42)
            d.line((*a,*b),fill='yellow',width=2)
            for x,y in (a,b):d.ellipse((x-6,y-6,x+6,y+6),outline='magenta',width=2)
            d.text((5,5),f"Frame {r['previous_frame_id']} -> {r['frame_id']}; feature {r['previous_feature_id']} -> {r['feature_id']}; pixel displacement {data['raw_pixel_displacement_px']:.2f} px",fill='white')
            d.text((5,23),'Diagnostic correspondence audit; high residual is NOT a confirmed dynamic point',fill='white')
            overview_name=filename.replace('.png','_overview.png');overview.save(args.output/overview_name);data['overview_image']=overview_name
        evidence.append(data);cards.append(card.resize((600,322)))
    if cards:
        grid=Image.new('RGB',(1200,322*((len(cards)+1)//2)),'black')
        for i,card in enumerate(cards):grid.paste(card,((i%2)*600,(i//2)*322))
        grid.save(args.output/'contact_sheet.png')
    write(args.output/'large_residual_matches.csv',evidence)
    summary={'frames':len(frames),'frame_reasons':dict(Counter(r['reason'] for r in frames)),
             'high_residual_points':len(candidates),'exported_samples':len(evidence),
             'selection':'retained' if args.only_retained else 'raw_including_excluded','original_coordinates_logged':coordinate_map is None,'all_logged_depth_samples_verified':True if evidence else None,
             'all_recovered_depth_samples_checked':depth_checks,'all_recovered_depth_max_error_m':all_depth_error if depth_checks else None,
             'max_depth_sampling_error_m':max([max(r['previous_depth_error_m'],r['current_depth_error_m']) for r in evidence],default=None),
             'warning':'High residual is not a confirmed dynamic label; crops need visual review.'}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
if __name__=='__main__':main()
