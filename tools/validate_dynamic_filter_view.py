#!/usr/bin/env python3
"""Run the existing rgbd_tum and audit FrameDrawer's real edge snapshot, CSV and PNGs."""
import argparse
import csv
import json
import os
import re
import subprocess
import tempfile
import time
import struct
import numpy as np
from collections import Counter, defaultdict
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
DATA = Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz')


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def audit(folder):
    frames = rows(folder/'view/frames.csv')
    assert frames and 'filter_mode' not in frames[0] and 'enabled' in frames[0] and 'applied' in frames[0]
    original = {r['frame_id']: r for r in rows(folder/'filter/frames.csv')} if (folder/'filter/frames.csv').exists() else {}
    groups = defaultdict(list)
    with (folder/'view/observations.csv').open() as f:
        for r in csv.DictReader(f):
            groups[(r['frame_id'], r['timestamp'])].append(r)
    old_features = defaultdict(dict)
    if (folder/'filter/features.csv').exists():
        with (folder/'filter/features.csv').open() as f:
            for r in csv.DictReader(f):
                old_features[r['frame_id']][r['index']] = r
    screenshots = []
    for fr in frames:
        group = groups[(fr['frame_id'], fr['timestamp'])]
        assert len(group) == int(fr['orb_total'])
        assert [int(r['index']) for r in group] == list(range(len(group)))
        assert sum(int(r['candidate']) for r in group) == int(fr['candidates'])
        excluded = {r['index'] for r in group if r['effective_excluded']=='1'}
        assert len(excluded) == int(fr['actual_excluded'])
        if fr['observations_before'] == 'N/A':
            assert fr['observations_after']=='N/A' and not excluded
            assert all(r['final_edge']=='N/A' for r in group)
        else:
            before = {r['index'] for r in group if r['eligible_before']=='1'}
            after = {r['index'] for r in group if r['eligible_after']=='1'}
            edges = {r['index'] for r in group if r['final_edge']=='1'}
            assert len(before)==int(fr['observations_before'])
            assert len(after)==int(fr['observations_after'])
            assert after == before-excluded == edges
            assert not (excluded & edges)
        if fr['optimization_available']=='0':
            assert fr['final_inliers']=='N/A' and all(r['final_inlier']=='N/A' for r in group)
        else:
            assert sum(r['final_inlier']=='1' for r in group)==int(fr['final_inliers'])
            assert all(r['final_edge']=='1' for r in group if r['final_inlier']=='1')
        if fr['applied']!='1':
            assert not excluded and fr['observations_before']==fr['observations_after']
        if fr['enabled']=='0':
            assert not excluded
        if fr['frame_id'] in original:
            old = original[fr['frame_id']]
            assert old['applied']==fr['applied'] and old['reason']==fr['reason']
            assert old['enabled']==fr['enabled'] and old['actual_excluded']==fr['actual_excluded']
            assert old['candidates']==fr['candidates']
            if fr['observations_before']!='N/A':
                assert old['pre_edges']==fr['observations_before'] and old['final_edges']==fr['observations_after']
            for row in group:
                old_row=old_features[fr['frame_id']][row['index']]
                assert old_row['candidate']==row['candidate']
                if row['final_edge']!='N/A':assert old_row['final_edge']==row['final_edge']
                if row['effective_excluded']=='1':assert old_row['effective_excluded']=='1'
        pngs=list((folder/'view').glob(f"frame_{fr['frame_id']}_*.png"))
        if pngs:
            image=Image.open(pngs[0]).convert('RGB')
            assert image.size==(680,702)
            # Exact center of each red marker must be red in the very renderer used by DrawFrame.
            for row in group:
                if row['effective_excluded']=='1':
                    assert image.getpixel((round(float(row['x'])),round(float(row['y']))))==(255,0,0)
            if fr['applied']=='1' or 'ROLLBACK' in fr['reason']:
                screenshots.append(str(pngs[0].relative_to(ROOT)))
    live_screenshots=[]
    captures=folder/'view/window-capture.json'
    if captures.exists():
        metadata={r['frame_id']:r for r in frames}
        for category,shot in json.loads(captures.read_text()).items():
            assert metadata[shot['frame_id']]['timestamp']==shot['timestamp']
            window=Image.open(folder/'view'/shot['window_png']).convert('RGB')
            rendered=Image.open(folder/'view'/shot['renderer_png']).convert('RGB')
            dx,dy=shot['canvas_offset'];w,h=rendered.size
            assert np.array_equal(np.asarray(window.crop((dx,dy,dx+w,dy+h))),np.asarray(rendered))
            live_screenshots.append(str((folder/'view'/shot['window_png']).relative_to(ROOT)))
    run=rows(folder/'RunFrames.csv')
    assert len(run)==len(frames)
    assert all(a['enabled']==b['enabled'] for a,b in zip(run,frames))
    assert all(abs(float(a['timestamp'])-float(b['timestamp']))<1e-7 for a,b in zip(run,frames))
    return {'frames':len(frames),'candidates':sum(int(r['candidates']) for r in frames),
            'actual_excluded':sum(int(r['actual_excluded']) for r in frames),
            'applied_frames':sum(r['applied']=='1' for r in frames),
            'reasons':dict(Counter(r['reason'] for r in frames)),
            'na_inlier_frames':sum(r['final_inliers']=='N/A' for r in frames),
            'tracking_failures':sum(r['state']!='2' for r in run),
            'screenshots':screenshots,'live_screenshots':live_screenshots}


def run_with_window_capture(cmd, env, log, folder):
    """Capture only this application's Current Frame window, never the desktop."""
    process=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    window=None
    captured={}
    templates={}
    started=time.monotonic()
    try:
        while process.poll() is None:
            if time.monotonic()-started>900:raise TimeoutError("Viewer validation timeout")
            time.sleep(.12)
            if window is None:
                tree=subprocess.run(['xwininfo','-root','-tree'],env=env,capture_output=True,text=True,timeout=5)
                match=re.search(r'(0x[0-9a-f]+) "ORB-SLAM3: Current Frame"',tree.stdout)
                if match:window=match.group(1)
                else:continue
            path=folder/'view/frames.csv'
            if not path.exists():continue
            metadata={r['frame_id']:r for r in rows(path) if r.get('reason') is not None}
            for png in (folder/'view').glob('frame_*.png'):
                if png not in templates:
                    templates[png]=np.asarray(Image.open(png).convert('RGB'))
            if not templates:continue
            shot=subprocess.run(['xwd','-id',window,'-silent'],env=env,capture_output=True,timeout=5)
            if shot.returncode or len(shot.stdout)<100:continue
            header=struct.unpack('>25I',shot.stdout[:100])
            size,_,_,_,width,height,_,byte_order,_,_,_,bits,stride,_,red,green,blue,_,_,ncolors,*_=header
            if byte_order!=0 or bits not in (24,32) or (red,green,blue)!=(0xff0000,0xff00,0xff):continue
            start=size+12*ncolors
            # Some X11 DirectColor captures advertise depth 24 with a 32-bit pixel stride.
            decoder='BGRX' if stride>=4*width else 'BGR'
            raw=Image.frombytes('RGB',(width,height),shot.stdout[start:],'raw',decoder,stride,1)
            array=np.asarray(raw)
            for png,template in list(templates.items())[-8:]:
                fid=png.name.split('_')[1];fr=metadata.get(fid)
                if not fr:continue
                category=('initialization' if fr['final_inliers']=='N/A' else
                    'off' if fr['enabled']=='0' else
                    'filter_applied' if fr['applied']=='1' else
                    'rollback' if 'ROLLBACK' in fr['reason'] else
                    'safety' if fr['reason']=='INSUFFICIENT_REMAINING_INLIERS' else 'other')
                if category in captured or category=='other':continue
                h,w=template.shape[:2]
                if height<h or width<w:continue
                points=np.array([[0,0],[100,100],[240,320],[479,639],[500,20],[600,400]])
                found=None
                for dy in range(height-h+1):
                    for dx in range(width-w+1):
                        if not np.array_equal(array[points[:,0]+dy,points[:,1]+dx],template[points[:,0],points[:,1]]):continue
                        if np.array_equal(array[dy:dy+h,dx:dx+w],template):found=(dx,dy);break
                    if found is not None:break
                if found is not None:
                    destination=folder/'view'/('window_'+category+'.png');raw.save(destination)
                    captured[category]={'frame_id':fid,'timestamp':fr['timestamp'],'canvas_offset':found,
                        'renderer_png':png.name,'window_png':destination.name,'pixel_identical_canvas':True,'xwd_bits':bits,'xwd_stride':stride}
                    (folder/'view/window-capture.json').write_text(json.dumps(captured,indent=2))
    finally:
        if process.poll() is None:process.terminate()
    if process.wait()!=0:raise RuntimeError("RGB-D viewer run failed")
    assert captured, "No live Current Frame screenshot matched the FrameDrawer snapshot"
    expected={'off':'off','on':'filter_applied','legacy':'filter_applied','rollback':'rollback','safety':'safety'}.get(folder.name)
    if expected:assert expected in captured, f"Requested live branch {expected} was not captured"
    return captured


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,default=ROOT/'outputs/dynamic-filter-switch-validation/runs')
    p.add_argument('--case',choices=['off','on','legacy','missing','rollback','safety','all','analyze'],default='all')
    p.add_argument('--frames',type=int,default=120)
    p.add_argument('--viewer',action='store_true')
    p.add_argument('--capture-window',action='store_true',help='X11 Current Frame screenshot matched pixel-for-pixel to saved renderer output')
    args=p.parse_args();args.viewer=args.viewer or args.capture_window;args.output.mkdir(parents=True,exist_ok=True)
    summary_path=args.output/'summary.json'
    summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
    cases=['off','on','rollback','safety','legacy'] if args.case=='all' else [args.case]
    if args.case=='analyze':
        cases=[name for name in ['off','on','rollback','safety','legacy','missing'] if (args.output/name/'view/frames.csv').exists()]
    for name in cases:
        folder=(args.output/name).resolve()
        if args.case!='analyze':
            if folder.exists():raise FileExistsError(folder)
            folder.mkdir()
            settings={'Run.Viewer':int(args.viewer),'SemanticMask.Visualize':0,'SemanticMask.AssociationDebug':0,
                      'SemanticMask.OutputDirectory':str(folder/'semantic'),
                      'Epipolar.Debug':0,'Epipolar.Visualize':0,'Epipolar.OutputDirectory':str(folder/'epipolar'),
                      'Residual3D.Debug':0,'Residual3D.Visualize':0,'Residual3D.OutputDirectory':str(folder/'depth'),
                      'LKCheck.Visualize':0,'LKCheck.OutputDirectory':str(folder/'lk'),
                      'Dynamic.UseDynamicFilter':int(name=='off'),
                      'Dynamic.FilterOutputDirectory':str(folder/'filter'),
                      'Dynamic.FilterViewOutputDirectory':str(folder/'view'),'Dynamic.FilterViewSaveEvery':10}
            if name=='legacy':settings['Dynamic.FilterMode']='simulate' # ignored legacy key; must still really filter
            if name=='rollback':settings['Dynamic.FilterMinFinalInliers']=2000
            if name=='safety':settings['Dynamic.FilterMinRemaining']=2000
            text=(ROOT/'config/TUM3-instance-association.yaml').read_text()
            if name=='missing':
                settings.pop('Dynamic.UseDynamicFilter');text=re.sub(r'(?m)^Dynamic\.UseDynamicFilter:.*$','',text)
            for key,value in settings.items():
                text=re.sub(r'(?m)^'+re.escape(key)+r':.*$','',text)
                text+='\n'+key+': '+json.dumps(value)+'\n'
            (folder/'effective_settings.txt').write_text(text)
            with tempfile.TemporaryDirectory(prefix='orb-filter-view-',dir='/tmp') as tmp:
                config=Path(tmp)/'settings.yaml';config.write_text(text)
                cmd=[str(ROOT/'Examples/RGB-D/rgbd_tum'),str(ROOT/'Vocabulary/ORBvoc.txt'),str(config),str(DATA),str(DATA/'associate.txt'),str(folder),str(args.frames)]
                env=os.environ.copy();env.pop('ORB_RESIDUAL_MATCH_AUDIT_DIR',None)
                if not args.viewer:env.pop('DISPLAY',None)
                print('START',name,flush=True)
                with (folder/'run.log').open('w') as log:
                    if args.capture_window:run_with_window_capture(cmd,env,log,folder)
                    else:subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900,check=True)
        summary[name]=audit(folder)
        if name in ('off','missing'):
            assert not (folder/'filter/frames.csv').exists()
            assert all(r['enabled']=='0' and r['applied']=='0' for r in rows(folder/'view/frames.csv'))
        if name in ('on','legacy'):assert summary[name]['applied_frames']>0, 'No actual filtering exercised'
        if name=='rollback':assert summary[name]['reasons'].get('FINAL_SUPPORT_FAILED_ROLLBACK',0)>0
        if name=='safety':assert summary[name]['reasons'].get('INSUFFICIENT_REMAINING_INLIERS',0)>0
        if name=='legacy':assert 'deprecated' in (folder/'run.log').read_text() and 'ignored' in (folder/'run.log').read_text()
        summary_path.write_text(json.dumps(summary,indent=2))
        print('PASS',name,json.dumps(summary[name]),flush=True)


if __name__=='__main__':main()
