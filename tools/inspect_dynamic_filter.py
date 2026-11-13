#!/usr/bin/env python3
"""Render every candidate frame from its own recorded raw feature positions; never rerun evidence."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path)
    p.add_argument('dataset',type=Path)
    args=p.parse_args()
    frames=read(args.run/'filter/frames.csv')
    run=read(args.run/'RunFrames.csv')
    assert [int(r['frame_id']) for r in frames]==list(range(len(run))), 'Requires one fresh-process RGB-D run without global frame-ID reset'
    samples={}
    with (args.dataset/'associate.txt').open() as f:
        for line in f:
            if not line.strip() or line.startswith('#'):continue
            fields=line.split();samples[float(fields[0])]=fields[1]
    selected=defaultdict(list)
    with (args.run/'filter/features.csv').open() as f:
        for r in csv.DictReader(f):
            if r['candidate']=='1':selected[int(r['frame_id'])].append(r)
    output=args.run/'filter/review'
    output.mkdir(exist_ok=True)
    if any(output.iterdir()):raise FileExistsError("Refusing to overwrite inspection artifacts")
    font=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",12)
    with (output/'distribution.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['frame_id','grid_x','grid_y','candidates','effective_excluded'])
        for index,fr in enumerate(frames):
            data=selected[index]
            if not data and index!=0:continue
            image=Image.open(args.dataset/samples[float(run[index]['timestamp'])]).convert('RGB')
            w,h=image.size
            assert (w,h)==(640,480), "Inspection supports the unscaled 640x480 TUM runs in this report"
            draw=ImageDraw.Draw(image)
            cell=defaultdict(lambda:[0,0])
            for row in data:
                x,y=float(row['current_x']),float(row['current_y'])
                excluded=row['effective_excluded']=='1'
                color="red" if excluded else "yellow"
                draw.ellipse((x-6,y-6,x+6,y+6),outline=color,width=2)
                draw.text((x+7,y),row["index"],fill=color,font=font)
                cx=min(3,max(0,int(4*x/w)));cy=min(2,max(0,int(3*y/h)))
                cell[(cx,cy)][0]+=1;cell[(cx,cy)][1]+=excluded
            panel=Image.new("RGB",(w,h+110),"black");panel.paste(image,(0,110));draw=ImageDraw.Draw(panel)
            if fr['filter_mode']=='simulate':title='SIMULATE ONLY - NO FILTERING'
            elif fr['accepted']=='1':title='APPLY ACCEPTED - RED = EFFECTIVELY EXCLUDED'
            else:title='APPLY NOT ACCEPTED - NO EXCLUSION / ROLLBACK'
            lines=[title,'YELLOW = CANDIDATE; labels = original feature indices',
                   f"frame={index} candidates={fr['candidates']} optimizer_candidates={fr['optimizable_candidates']} accepted={fr['accepted']}", fr['reason']]
            for j,line in enumerate(lines):draw.text((8,8+24*j),line,fill="white",font=font)
            panel.save(output/f"frame_{index}.png")
            for cy in range(3):
                for cx in range(4):writer.writerow([index,cx,cy,*cell[(cx,cy)]])
    print('Rendered candidate frames:',sum(bool(x) for x in selected.values()),'->',output)

if __name__=='__main__':main()
