#!/usr/bin/env python3
"""Stream-join same-run filter, epipolar, 3D and LK logs without holding feature tables in memory."""
import argparse
import csv
import json
import math
from pathlib import Path


def table(path):
    with path.open() as f:
        yield from csv.DictReader(f)


def key(row,index='current_feature_index'):
    return int(row['frame_id']),int(row[index])


def audit(run):
    frame={int(r['frame_id']):r for r in table(run/'filter/frames.csv')}
    eframes={int(r['frame_id']):r for r in table(run/'epipolar/frames.csv')}
    lk=iter(table(run/'lk/features.csv'));l=next(lk,None)
    count=candidates=0
    features=table(run/'filter/features.csv')
    epis=table(run/'epipolar/features.csv')
    depths=table(run/'depth/features.csv')
    from itertools import zip_longest
    for f,e,d in zip_longest(features,epis,depths):
        assert f is not None and e is not None and d is not None,'Feature table length mismatch'
        k=key(f,'index');assert k==key(e)==key(d,'feature_id'),k
        while l is not None and key(l)<k:l=next(lk,None)
        local=l if l is not None and key(l)==k else None
        assert f['previous_index']==e['previous_feature_index'],k
        assert f['epipolar_state']==e['state'] and f['depth_state']==d['depth_state'],k
        def number(a,b):return (math.isnan(float(a)) and math.isnan(float(b))) or float(a)==float(b)
        assert number(f['epipolar_residual'],e['epipolar_residual_px']) and number(f['residual3d'],d['residual_3d_m']),k
        assert f['previous_depth_valid']==d['previous_depth_valid'] and f['current_depth_valid']==d['current_depth_valid'],k
        assert f['depth_difference_rejected']==d['depth_difference_gate_rejected'],k
        assert int(d['depth_difference_gate_enabled'])==1,k
        assert f['lk']==(local['lk_check_state'] if local else 'UNVERIFIABLE'),k
        assert f['instance_id']==e['current_instance_id']==d['instance_id'],k
        expected=(frame[k[0]]['eligible']=='1' and int(f['instance_id'])>0 and int(e['previous_instance_id'])>0
                  and int(f['previous_index'])>=0 and local is not None and local['lk_check_state']=='SUPPORTED'
                  and eframes[k[0]]['model_valid']=='1' and d['transform_valid']=='1'
                  and d['previous_depth_valid']==d['current_depth_valid']=='1' and d['depth_difference_gate_rejected']=='0'
                  and math.isfinite(float(d['depth_difference_m']))
                  and e['state']==d['depth_state']=='SUSPECTED_DYNAMIC')
        assert (f['candidate']=='1')==expected,(k,'intersection differs',f['reason'])
        if expected:
            assert f['previous_index']==d['previous_feature_id']==local['previous_feature_index'],k
            assert f['previous_frame_id']==e['previous_frame_id']==d['previous_frame_id']==local['previous_frame_id'],k
            assert e['timestamp']==local['timestamp']==d['current_timestamp'],k
        count+=1;candidates+=expected
    return {'features_joined':count,'candidates_verified':candidates,'same_run_evidence_and_complete_intersection':True}


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result={}
    for run in args.run:
        result[run.name]=audit(run);print(run.name,result[run.name],flush=True)
    args.output.write_text(json.dumps(result,indent=2))

if __name__=='__main__':main()
