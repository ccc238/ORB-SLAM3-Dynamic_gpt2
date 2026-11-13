#include "DynamicFilter.h"
#include "Frame.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "SemanticMasker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iomanip>
#include <cstring>
namespace ORB_SLAM3 {
DynamicFilterConfig DynamicFilterConfig::Read(cv::FileStorage& fs) {
    DynamicFilterConfig c;
    auto n=fs["Dynamic.UseDynamicFilter"];
    if(!n.empty()) {
        if(!n.isInt() || (int(n)!=0 && int(n)!=1)) throw std::runtime_error("Dynamic.UseDynamicFilter must be integer 0 (enabled) or 1 (disabled)");
        c.raw=int(n);
    }
    c.enabled=c.raw==0;
    n=fs["Dynamic.FilterMode"];
    if(!n.empty()) {if(!n.isString()) throw std::runtime_error("Dynamic.FilterMode must be string");c.mode=std::string(n);}
    if(c.mode!="simulate" && c.mode!="apply") throw std::runtime_error("Dynamic.FilterMode must be simulate or apply");
    auto number=[&](const char* key,double& value) {auto v=fs[key];if(!v.empty()){if(!v.isReal()&&!v.isInt())throw std::runtime_error(std::string(key)+" must be numeric");value=double(v);}};
    number("Dynamic.FilterMaxRatio",c.maxRatio);
    auto integer=[&](const char* key,int& value){auto v=fs[key];if(!v.empty()){if(!v.isInt())throw std::runtime_error(std::string(key)+" must be integer");value=int(v);}};
    integer("Dynamic.FilterMinRemaining",c.minRemaining);integer("Dynamic.FilterMinFinalInliers",c.minFinalInliers);integer("Dynamic.FilterMinCells",c.minCells);
    n=fs["Dynamic.FilterOutputDirectory"];if(!n.empty()){if(!n.isString())throw std::runtime_error("FilterOutputDirectory must be string");c.output=std::string(n);}
    if(!std::isfinite(c.maxRatio)||c.maxRatio<=0||c.maxRatio>=1||c.minRemaining<3||c.minFinalInliers<30||c.minCells<1||c.minCells>12)throw std::runtime_error("Invalid Dynamic filter safety limits (final inliers >=30)");
    std::cout<<"Dynamic.FilterMode raw="<<(fs["Dynamic.FilterMode"].empty()?"missing":c.mode)<<" ";
    std::cout<<"Dynamic.UseDynamicFilter raw="<<(fs["Dynamic.UseDynamicFilter"].empty()?"missing":std::to_string(c.raw))<<" filter_enabled="<<c.enabled<<" filter_mode="<<c.mode<<std::endl;
    return c;
}
std::vector<DynamicCandidate> ConservativeCandidates(const EpipolarInput& in,const DiagnosticPair& pair,const EpipolarResult& e,const Residual3DResult& d,const LKCheckResult& lk) {
    std::vector<DynamicCandidate> out(in.rawKeys.size());
    std::vector<const LKCheckFeature*> rows(out.size(),nullptr);
    std::set<int> duplicates;
    for(const auto& f:lk.features) if(f.currentIndex>=0&&size_t(f.currentIndex)<out.size()){if(rows[f.currentIndex])duplicates.insert(f.currentIndex);rows[f.currentIndex]=&f;}
    const bool same=e.previousFrameId==long(pair.previous.frameId)&&d.previousFrameId==e.previousFrameId&&lk.previousFrameId==e.previousFrameId&&d.currentFrameId==long(in.frameId)&&lk.frameId==long(in.frameId)&&d.currentTimestamp==in.timestamp&&lk.timestamp==in.timestamp&&d.previousTimestamp==pair.previous.timestamp&&lk.previousTimestamp==pair.previous.timestamp&&pair.matching.previousFrameId==e.previousFrameId&&std::isfinite(e.dt)&&std::abs(e.dt-(in.timestamp-pair.previous.timestamp))<1e-9;
    for(size_t i=0;i<out.size();++i) {
        auto& o=out[i];
        if(i>=in.instances.size()||in.instances[i]<=0){o.reason="NO_CURRENT_INSTANCE";continue;}
        if(!same){o.reason="PAIR_MISMATCH";continue;}
        if(i>=pair.matching.features.size()||i>=e.features.size()||i>=d.features.size()){o.reason="MISSING_EVIDENCE";continue;}
        int p=pair.matching.features[i].previousIndex;
        if(p<0||size_t(p)>=pair.previous.instances.size()||pair.previous.instances[p]<=0){o.reason="NO_PREVIOUS_INSTANCE_MATCH";continue;}
        const auto& ef=e.features[i];const auto& df=d.features[i];auto l=rows[i];
        if(!l||duplicates.count(i)||l->previousIndex!=p||ef.previousIndex!=p||df.previousIndex!=p||ef.previousInstance!=pair.previous.instances[p]){o.reason="MATCH_MISMATCH";continue;}
        if(l->state!=LKCheckState::SUPPORTED){o.reason="LK_NOT_SUPPORTED";continue;}
        if(!e.modelValid||!d.motionValid||!df.previousDepth.valid||!df.currentDepth.valid||df.depthDifferenceRejected||!std::isfinite(df.depthDifference)||!std::isfinite(ef.residual)||!std::isfinite(df.residual)){o.reason="INVALID_GEOMETRY_OR_DEPTH";continue;}
        if(ef.state!=EpipolarState::SUSPECTED_DYNAMIC||df.state!=EpipolarState::SUSPECTED_DYNAMIC){o.reason="GEOMETRY_NOT_BOTH_HIGH";continue;}
        o.candidate=true;o.reason="CONSERVATIVE_INTERSECTION";
    }
    return out;
}
DynamicFilter::DynamicFilter(const DynamicFilterConfig& c):c_(c) {
    EnsureSemanticDirectory(c.output);
    frames_.open(c.output+"/frames.csv");features_.open(c.output+"/features.csv");
    poses_.open(c.output+"/poses.csv");
    poses_<<"frame_id,stage,r0c0,r0c1,r0c2,tx,r1c0,r1c1,r1c2,ty,r2c0,r2c1,r2c2,tz\n";
    if(!poses_||!frames_||!features_)throw std::runtime_error("DynamicFilter: cannot open logs");
    frames_<<std::setprecision(17);features_<<std::setprecision(17);poses_<<std::setprecision(17);
    frames_<<"frame_id,filter_enabled,filter_mode,eligible,candidates,optimizable_candidates,original_outlier_candidates,original_inlier_candidates,pre_edges,pre_inliers,final_edges,final_inliers,safety,reason,executed,accepted,translation_m,rotation_rad,compute_ms,record_ms,total_ms,attempted_edges,attempted_inliers,remaining_cells,simulation_unchanged\n";
    features_<<"frame_id,index,instance_id,previous_index,lk,epipolar_state,epipolar_residual,depth_state,residual3d,candidate,effective_excluded,reason,final_edge,allow_keyframe_observation,allow_new_point,previous_frame_id,current_x,current_y,lk_reason,epipolar_reason,depth_reason,epipolar_model_valid,motion_valid,previous_depth_valid,current_depth_valid,depth_difference_rejected,depth_difference_m\n";
}
void DynamicFilter::Process(Frame& f,const EpipolarInput& in,const DiagnosticPair& pair,const EpipolarResult& e,const Residual3DResult& d,const LKCheckResult& lk,bool eligible,int& trackingInliers,int requiredInliers) {
    if(long(in.frameId)!=long(f.mnId) || in.rawKeys.size()!=size_t(f.N))
        throw std::runtime_error("DynamicFilter: current frame changed during tracking");
    auto start=std::chrono::steady_clock::now();
    const bool simulate=c_.mode=="simulate";
    const auto originalPoints=f.mvpMapPoints;
    const cv::Mat originalDescriptors=simulate?f.mDescriptors.clone():cv::Mat();
    const auto originalDepth=f.mvDepth;
    const auto originalKeys=f.mvKeys,originalUndistorted=f.mvKeysUn;
    std::vector<std::vector<size_t>> originalGrid;
    if(simulate) for(int x=0;x<FRAME_GRID_COLS;++x) for(int y=0;y<FRAME_GRID_ROWS;++y) originalGrid.push_back(f.mGrid[x][y]);
    const std::set<int> originalEdgeSet(f.mvPoseOptimizationIndices.begin(),f.mvPoseOptimizationIndices.end());
    auto candidates=ConservativeCandidates(in,pair,e,d,lk);
    f.mvbDynamicCandidate.assign(f.N,false);f.mvbDynamicExcluded.assign(f.N,false);
    int count=0,optimizable=0,rejected=0,originalIn=0,edges=f.mvPoseOptimizationIndices.size(),preIn=0;
    std::set<int> cells;
    for(int i=0;i<f.N;++i){
        f.mvbDynamicCandidate[i]=eligible&&size_t(i)<candidates.size()&&candidates[i].candidate;
        bool cand=f.mvbDynamicCandidate[i];count+=cand;
        if(f.mvpMapPoints[i]){optimizable+=cand;rejected+=cand&&f.mvbOutlier[i];originalIn+=cand&&!f.mvbOutlier[i];
            if(originalEdgeSet.count(i)&&!f.mvbOutlier[i]&&f.mvpMapPoints[i]->Observations()>0){++preIn;if(!cand){auto p=f.mvKeysUn[i].pt;int x=int(4*p.x/in.size.width),y=int(3*p.y/in.size.height);if(x>=0&&x<4&&y>=0&&y<3)cells.insert(y*4+x);}}}
    }
    const auto pose=f.GetPose();const auto outliers=f.mvbOutlier;const auto priorEdges=f.mvPoseOptimizationIndices;
    const auto priorAudit=f.mPoseObservationAudit;
    int finalEdges=edges,finalIn=preIn,attemptedEdges=-1,attemptedInliers=-1;bool executed=false,accepted=false,safe=false;
    std::string reason="UNSUPPORTED_TRACKING_PATH";
    if(eligible){
        if(!count)reason="NO_CANDIDATES";
        else if(!optimizable)reason="NO_OPTIMIZABLE_CANDIDATES";
        else if(double(optimizable)/std::max(1,edges)>c_.maxRatio || double(originalIn)/std::max(1,preIn)>c_.maxRatio)reason="MAX_EXCLUSION_RATIO";
        else if(preIn-originalIn<c_.minRemaining)reason="INSUFFICIENT_REMAINING_INLIERS";
        else if(int(cells.size())<c_.minCells)reason="INSUFFICIENT_SPATIAL_COVERAGE";
        else {safe=true;reason=c_.mode=="simulate"?"SIMULATE_ONLY":"ACCEPTED";}
    }
    if(safe&&c_.mode=="apply"){
        f.mvbDynamicExcluded=f.mvbDynamicCandidate;
        executed=true;
        const int optimizationResult=Optimizer::PoseOptimization(&f);
        finalEdges=f.mvPoseOptimizationIndices.size();finalIn=0;
        std::set<int> finalCells;
        for(int idx:f.mvPoseOptimizationIndices){
            if(f.mvbDynamicExcluded[idx]) throw std::runtime_error("DynamicFilter: excluded feature entered final optimizer");
            if(!f.mvbOutlier[idx]&&f.mvpMapPoints[idx]->Observations()>0) {
                ++finalIn;
                const auto p=f.mvKeysUn[idx].pt;
                const int x=int(4*p.x/in.size.width),y=int(3*p.y/in.size.height);
                if(x>=0&&x<4&&y>=0&&y<3) finalCells.insert(y*4+x);
            }
        }
        attemptedEdges=finalEdges;attemptedInliers=finalIn;
        if(optimizationResult<0||!f.GetPose().matrix().allFinite()||finalIn<std::max(requiredInliers,c_.minFinalInliers)||int(finalCells.size())<c_.minCells){
            reason=optimizationResult<0?"SOLVER_FAILED_ROLLBACK":"FINAL_SUPPORT_FAILED_ROLLBACK";safe=false;f.SetPose(pose);f.mvbOutlier=outliers;f.mvPoseOptimizationIndices=priorEdges;f.mPoseObservationAudit=priorAudit;f.mvbDynamicExcluded.assign(f.N,false);finalEdges=edges;finalIn=preIn;
        }else {accepted=true;trackingInliers=finalIn;cells=finalCells;}
    }
    if(simulate) {
        bool unchanged=f.mvpMapPoints==originalPoints&&(f.mvDepth.size()==originalDepth.size()&&(originalDepth.empty()||std::memcmp(f.mvDepth.data(),originalDepth.data(),originalDepth.size()*sizeof(float))==0))&&f.mvbOutlier==outliers&&f.mvPoseOptimizationIndices==priorEdges&&f.GetPose().matrix()==pose.matrix()&&f.mvKeys.size()==originalKeys.size()&&f.mvKeysUn.size()==originalUndistorted.size();
        unchanged=unchanged&&f.mDescriptors.size()==originalDescriptors.size()&&f.mDescriptors.type()==originalDescriptors.type();
        if(unchanged&&!originalDescriptors.empty()) unchanged=cv::norm(originalDescriptors,f.mDescriptors,cv::NORM_INF)==0;
        auto equalKey=[](const cv::KeyPoint& a,const cv::KeyPoint& b){return a.pt==b.pt&&a.size==b.size&&a.angle==b.angle&&a.response==b.response&&a.octave==b.octave&&a.class_id==b.class_id;};
        for(size_t i=0;unchanged&&i<originalKeys.size();++i)unchanged=equalKey(originalKeys[i],f.mvKeys[i]);
        for(size_t i=0;unchanged&&i<originalUndistorted.size();++i)unchanged=equalKey(originalUndistorted[i],f.mvKeysUn[i]);
        int cell=0;for(int x=0;unchanged&&x<FRAME_GRID_COLS;++x)for(int y=0;unchanged&&y<FRAME_GRID_ROWS;++y)unchanged=originalGrid[cell++]==f.mGrid[x][y];
        if(!unchanged)throw std::runtime_error("DynamicFilter: simulate changed algorithm data");
    }
    view_=MakeDynamicFilterView(f,priorEdges,priorAudit,true,c_.mode,accepted,executed,reason);
    for(int stage=0;stage<2;++stage){const auto matrix=(stage?f.GetPose():pose).matrix();poses_<<f.mnId<<','<<(stage?"final":"preliminary");for(int y=0;y<3;++y)for(int x=0;x<4;++x)poses_<<','<<matrix(y,x);poses_<<'\n';}
    auto delta=f.GetPose()*pose.inverse();
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();

    std::set<int> finalSet(f.mvPoseOptimizationIndices.begin(),f.mvPoseOptimizationIndices.end());
    cv::Mat picture;if(!lk.currentGray.empty())cv::cvtColor(lk.currentGray,picture,cv::COLOR_GRAY2BGR);
    std::vector<const LKCheckFeature*> lrows(f.N,nullptr);for(const auto& l:lk.features)if(l.currentIndex>=0&&l.currentIndex<f.N)lrows[l.currentIndex]=&l;
    for(int i=0;i<f.N;++i){
        const bool excluded=f.mvbDynamicExcluded[i];
        features_<<f.mnId<<','<<i<<','<<(size_t(i)<in.instances.size()?in.instances[i]:-1)<<','<<(size_t(i)<e.features.size()?e.features[i].previousIndex:-1)<<','<<(lrows[i]?LKCheckStateName(lrows[i]->state):"UNVERIFIABLE")<<','<<(size_t(i)<e.features.size()?EpipolarStateName(e.features[i].state):"UNKNOWN")<<','<<(size_t(i)<e.features.size()?e.features[i].residual:NAN)<<','<<(size_t(i)<d.features.size()?EpipolarStateName(d.features[i].state):"UNKNOWN")<<','<<(size_t(i)<d.features.size()?d.features[i].residual:NAN)<<','<<f.mvbDynamicCandidate[i]<<','<<excluded<<','<<(eligible&&size_t(i)<candidates.size()?candidates[i].reason:"UNSUPPORTED_TRACKING_PATH")<<','<<finalSet.count(i)<<','<<!excluded<<','<<!excluded
                 <<','<<e.previousFrameId<<','<<f.mvKeys[i].pt.x<<','<<f.mvKeys[i].pt.y
                 <<','<<(lrows[i]?lrows[i]->reason:"NO_MATCH")
                 <<','<<(size_t(i)<e.features.size()?e.features[i].reason:"NO_MATCH")
                 <<','<<(size_t(i)<d.features.size()?d.features[i].reason:"NO_MATCH")
                 <<','<<e.modelValid<<','<<d.motionValid
                 <<','<<(size_t(i)<d.features.size()&&d.features[i].previousDepth.valid)
                 <<','<<(size_t(i)<d.features.size()&&d.features[i].currentDepth.valid)
                 <<','<<(size_t(i)<d.features.size()&&d.features[i].depthDifferenceRejected)
                 <<','<<(size_t(i)<d.features.size()?d.features[i].depthDifference:NAN)<<'\n';
        if(f.mvbDynamicCandidate[i]&&!picture.empty()){cv::circle(picture,f.mvKeys[i].pt,4,cv::Scalar(0,255,255),1);if(view_.excludedObservations[i])cv::drawMarker(picture,f.mvKeys[i].pt,{0,0,255},cv::MARKER_TILTED_CROSS,10,2);}
    }
    if(!picture.empty()&&f.mnId%10==0){cv::rectangle(picture,{0,0},{picture.cols-1,58},{0,0,0},cv::FILLED);std::string label=c_.mode=="simulate"?"SIMULATE ONLY - NO FILTERING":accepted?"APPLY: red X=excluded edges; yellow=candidate":"APPLY: NOT APPLIED / ROLLBACK";cv::putText(picture,label,{8,22},cv::FONT_HERSHEY_SIMPLEX,.5,{255,255,255},1);cv::putText(picture,"candidates="+std::to_string(count)+" excluded="+std::to_string(view_.ExcludedCount())+" "+reason,{8,45},cv::FONT_HERSHEY_SIMPLEX,.4,{255,255,255},1);cv::imwrite(c_.output+"/frame_"+std::to_string(f.mnId)+".png",picture);}
    features_.flush();poses_.flush();
    double totalMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    frames_<<f.mnId<<",1,"<<c_.mode<<','<<eligible<<','<<count<<','<<optimizable<<','<<rejected<<','<<originalIn<<','<<edges<<','<<preIn<<','<<finalEdges<<','<<finalIn<<','<<safe<<','<<reason<<','<<executed<<','<<accepted<<','<<delta.translation().norm()<<','<<delta.so3().log().norm()<<','<<ms<<','<<(totalMs-ms)<<','<<totalMs<<','<<attemptedEdges<<','<<attemptedInliers<<','<<cells.size()<<','<<(simulate?1:-1)<<'\n';
    frames_.flush();features_.flush();poses_.flush();
    if(!frames_||!features_||!poses_)throw std::runtime_error("DynamicFilter: log write failed");
    std::cout<<"[DynamicFilter] frame="<<f.mnId<<" mode="<<c_.mode<<" candidates="<<count<<" optimizable="<<optimizable<<" accepted="<<accepted<<" reason="<<reason<<" ms="<<ms<<std::endl;
}
}
