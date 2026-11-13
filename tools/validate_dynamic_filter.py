#!/usr/bin/env python3
"""Switch-only validation entry point; trajectory evaluation helpers are retained."""
from pathlib import Path
import numpy as np
from validate_dynamic_filter_view import main, audit
DATA = Path('/home/aaa/Datasets/TUM')

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



if __name__=='__main__':main()
