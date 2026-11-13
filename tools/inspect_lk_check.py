#!/usr/bin/env python3
"""Export chosen same-run LK rows with original RGB crops; no re-tracking."""
import argparse
import csv
import math
from pathlib import Path
from PIL import Image,ImageDraw


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('dataset',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--sample',action='append',required=True,help='current_frame:current_feature')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if any(args.output.iterdir()):raise FileExistsError('Refusing to overwrite inspection artifacts')
    with (args.run/'lk/features.csv').open() as f:rows=list(csv.DictReader(f))
    associations=[s.split() for s in (args.dataset/'associate.txt').read_text().splitlines() if s.strip() and not s.startswith('#')]
    selected=[]
    for sample in args.sample:
        fid,j=sample.split(':');r=next(r for r in rows if r['frame_id']==fid and r['current_feature_index']==j)
        images=[]
        for key in ('previous_timestamp','timestamp'):
            a=min(associations,key=lambda a:abs(float(a[0])-float(r[key])));assert abs(float(a[0])-float(r[key]))<1e-6
            images.append(Image.open(args.dataset/a[1]).convert('RGB'))
        assert images[0].size==images[1].size==(640,480)
        canvas=Image.new('RGB',(1280,920),'#171717');draw=ImageDraw.Draw(canvas)
        for k,im in enumerate(images):canvas.paste(im,(640*k,100))
        points=[('p','p_x','p_y',0,'white'),('ORB','q_orb_x','q_orb_y',1,'orange'),('LK','q_lk_x','q_lk_y',1,'cyan'),('back','p_back_x','p_back_y',0,'magenta')]
        def mark(x,y,label,color,scale=1):
            if label=='ORB':draw.rectangle((x-7,y-7,x+7,y+7),outline=color,width=2)
            elif label=='LK':draw.line((x-5,y,x+5,y),fill=color,width=2);draw.line((x,y-5,x,y+5),fill=color,width=2)
            else:draw.ellipse((x-4,y-4,x+4,y+4),outline=color,width=2)
            draw.text((x+9,y+8 if label in ('LK','back') else y-12),label,fill=color)
        for col,(label,kx,ky,side,color) in enumerate(points):
            x,y=float(r[kx]),float(r[ky]);draw.text((col*320+8,600),label+' local RGB crop (5x)',fill=color)
            if not math.isfinite(x+y):
                draw.text((col*320+8,630),'INVALID / NOT COMPUTED',fill='white');continue
            mark(x+640*side,y+100,label,color)
            # Display only: integer crop origin, exact subpixel marker within the crop.
            ox,oy=int(x)-24,int(y)-24
            nearest=Image.Resampling.NEAREST if hasattr(Image,'Resampling') else Image.NEAREST
            crop=images[side].crop((ox,oy,ox+48,oy+48)).resize((240,240),nearest)
            canvas.paste(crop,(col*320+8,625));mark(col*320+8+(x-ox)*5,625+(y-oy)*5,label,color)
        draw.text((8,6),f"Frames {r['previous_frame_id']} -> {fid}; original features {r['previous_feature_index']} -> {j}; {r['lk_check_state']}: {r['lk_check_reason']}",fill='white')
        draw.text((8,25),f"FB={float(r['fb_error_px']):.6f} px; endpoint={float(r['endpoint_error_px']):.6f} px; depth={r['depth_state']}; residual={r['residual_3d_m']} m",fill='white')
        draw.text((8,44),'White circle:p; Orange square:ORB; Cyan cross:LK; Magenta circle:p_back. NO FEATURES REMOVED.',fill='white')
        draw.text((8,63),'Original RGB display; LK was computed on aligned Tracking gray. SUPPORTED is not ground truth.',fill='white')
        draw.text((8,887),'Selected visual inspection only; not an error-rate or dynamic-accuracy measurement.',fill='white')
        canvas.save(args.output/f'frame_{fid}_feature_{j}.png');selected.append(r)
    with (args.output/'selected_matches.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(selected[0]));w.writeheader();w.writerows(selected)


if __name__=='__main__':main()
