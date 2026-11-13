#include "Residual3D.h"
#include "SemanticMasker.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
namespace ORB_SLAM3 {
namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point start) {return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
void Require(bool ok,const std::string& why) {if(!ok) throw std::runtime_error("Residual3D: "+why);}
bool Finite(cv::Vec3d p) {return std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2]);}
double Coverage(const std::vector<cv::Point2f>& points,cv::Size size) {
    bool cells[16]={}; for(auto p:points) if(p.x>=0&&p.y>=0&&p.x<size.width&&p.y<size.height)
        cells[std::min(3,int(p.y*4/size.height))*4+std::min(3,int(p.x*4/size.width))]=true;
    return std::count(cells,cells+16,true)/16.;
}
DepthSample Sample(const EpipolarInput& input,int i,const Residual3DConfig& cfg) {
    DepthSample s;
    if(i<0 || i>=int(input.depths.size()) || i>=int(input.depthReasons.size())) return s;
    s.meters=input.depths[i]; s.reason=input.depthReasons[i];
    s.valid=s.reason=="OK" && std::isfinite(s.meters)&&s.meters>=cfg.minDepth&&s.meters<=cfg.maxDepth;
    if(!s.valid && s.reason=="OK") s.reason="INVALID_DEPTH";
    return s;
}
}
Residual3DConfig Residual3DConfig::Read(cv::FileStorage& fs,bool seg,bool assoc,bool mask) {
    Residual3DConfig c;
    auto integer=[&](const char* key,int& value) {auto n=fs[key]; if(n.empty()) return; Require(n.isInt(),std::string(key)+" must be integer"); value=int(n);};
    auto number=[&](const char* key,double& value) {auto n=fs[key]; if(n.empty()) return; Require(n.isInt()||n.isReal(),std::string(key)+" must be numeric"); value=double(n); Require(std::isfinite(value),std::string(key)+" must be finite");};
    integer("Dynamic.Use3DResidual",c.configuredValue);
    Require(c.configuredValue==0||c.configuredValue==1,"Dynamic.Use3DResidual accepts only integer 0 or 1");
    c.enabled=(c.configuredValue==0);
    std::cout<<"Dynamic.Use3DResidual raw="<<(fs["Dynamic.Use3DResidual"].empty()?"MISSING":std::to_string(c.configuredValue))<<" effective="<<c.configuredValue<<" enabled="<<c.enabled<<" (0=启用，1=关闭)"<<std::endl;
    if(!c.enabled) return c;
    auto source=fs["Dynamic.DepthPoseSource"];
    if(!source.empty()) {Require(source.isString(),"Dynamic.DepthPoseSource must be string"); source>>c.poseSource;}
    Require(c.poseSource=="tracking"||c.poseSource=="background_pnp","Dynamic.DepthPoseSource accepts tracking or background_pnp only");
    integer("Residual3D.TrackingMinInliers",c.trackingMinInliers);
    Require(c.trackingMinInliers>0,"TrackingMinInliers must be positive");
    std::cout<<"Dynamic.DepthPoseSource="<<c.poseSource<<" TrackingMinInliers="<<c.trackingMinInliers<<" (no automatic fallback)"<<std::endl;
    int depthDifferenceGate=0;integer("Residual3D.UseDepthDifferenceGate",depthDifferenceGate);
    Require(depthDifferenceGate==0||depthDifferenceGate==1,"UseDepthDifferenceGate must be integer 0 or 1 (1=enabled)");
    c.useDepthDifferenceGate=depthDifferenceGate==1;
    number("Residual3D.MaxDepthDifferenceM",c.maxDepthDifferenceM);
    Require(c.maxDepthDifferenceM>0,"MaxDepthDifferenceM must be positive");
    std::cout<<"Residual3D.UseDepthDifferenceGate="<<depthDifferenceGate<<" MaxDepthDifferenceM="<<c.maxDepthDifferenceM<<" m (raw axial depth difference; exclusions audited)"<<std::endl;
    Require(seg&&assoc&&!mask,"requires online segmentation and instance association enabled, semantic feature masking disabled");
    int debug=0,visualize=0;
    integer("Residual3D.Debug",debug); integer("Residual3D.Visualize",visualize);
    Require((debug==0||debug==1)&&(visualize==0||visualize==1),"debug/visualize must be 0 or 1"); c.debug=debug==1; c.visualize=visualize==1;
    integer("Residual3D.MinBackground",c.minBackground); integer("Residual3D.MinInliers",c.minInliers);
    integer("Residual3D.Iterations",c.iterations); integer("Residual3D.DepthRadius",c.depthRadius); integer("Residual3D.SaveEvery",c.every);
    number("Residual3D.MinDepthM",c.minDepth); number("Residual3D.MaxDepthM",c.maxDepth);
    number("Residual3D.DepthJumpM",c.depthJumpM); number("Residual3D.DepthJumpRatio",c.depthJumpRatio);
    number("Residual3D.MatchRatio",c.ratio); number("Residual3D.MaxDt",c.maxDt);
    number("Residual3D.RansacPx",c.ransacPx); number("Residual3D.Confidence",c.confidence);
    number("Residual3D.MinInlierRatio",c.minInlierRatio); number("Residual3D.MinCoverage",c.minCoverage);
    number("Residual3D.MinShapeRatio",c.minShapeRatio); number("Residual3D.MaxReprojectionRms",c.maxReprojectionRms);
    number("Residual3D.LowM",c.lowM); number("Residual3D.HighM",c.highM);
    auto output=fs["Residual3D.OutputDirectory"]; if(!output.empty()) {Require(output.isString(),"output must be string"); output>>c.output;}
    Require(c.minBackground>=6&&c.minInliers>=6&&c.iterations>0&&c.depthRadius>=0&&c.depthRadius<=20&&c.every>0,"invalid counts/radius");
    Require(c.minDepth>0&&c.maxDepth>c.minDepth&&c.depthJumpM>=0&&c.depthJumpRatio>=0,"invalid depth limits");
    Require(c.ratio>0&&c.ratio<1&&c.maxDt>0&&c.ransacPx>0&&c.confidence>0&&c.confidence<1,"invalid matching/PnP parameters");
    Require(c.minInlierRatio>0&&c.minInlierRatio<=1&&c.minCoverage>0&&c.minCoverage<=1&&c.minShapeRatio>0&&c.minShapeRatio<1&&c.maxReprojectionRms>0,"invalid support parameters");
    Require(c.lowM>=0&&c.highM>c.lowM,"require 0 <= LowM < HighM"); return c;
}
DepthSample Residual3DDetector::CheckDepth(double z,cv::Point2f raw,const cv::Mat& depth,const Residual3DConfig& c) {
    DepthSample s; s.meters=z;
    if(!std::isfinite(raw.x)||!std::isfinite(raw.y)||raw.x<0||raw.y<0||raw.x>=depth.cols||raw.y>=depth.rows) {s.reason="DEPTH_OUT_OF_BOUNDS";return s;}
    if(!std::isfinite(z)||z<=0) {s.reason="INVALID_DEPTH";return s;}
    if(z<c.minDepth||z>c.maxDepth) {s.reason="DEPTH_OUT_OF_RANGE";return s;}
    Require(depth.type()==CV_32FC1,"depth consistency input must be metre CV_32FC1");
    const int u=int(raw.x),v=int(raw.y),r=c.depthRadius;
    if(u-r<0||v-r<0||u+r>=depth.cols||v+r>=depth.rows) {s.reason="DEPTH_WINDOW_OUT_OF_BOUNDS";return s;}
    for(int y=v-r;y<=v+r;++y) for(int x=u-r;x<=u+r;++x) {
        double d=depth.at<float>(y,x);
        if(!std::isfinite(d)||d<c.minDepth||d>c.maxDepth) {s.reason="DEPTH_NEIGHBOR_INVALID";return s;}
        if(std::abs(d-z)>c.depthJumpM+c.depthJumpRatio*z) {s.reason="DEPTH_DISCONTINUITY";return s;}
    }
    s.valid=true; s.reason="OK"; return s;
}
cv::Vec3d Residual3DDetector::Backproject(double z,cv::Point2f p,const cv::Mat& calibration) {
    cv::Mat K; calibration.convertTo(K,CV_64F);
    if(K.rows!=3||K.cols!=3||!cv::checkRange(K)||std::abs(cv::determinant(K))<1e-12||!std::isfinite(z)||z<=0||!std::isfinite(p.x)||!std::isfinite(p.y)) return cv::Vec3d::all(NAN);
    cv::Mat ray=K.inv()*(cv::Mat_<double>(3,1)<<p.x,p.y,1.);
    return {z*ray.at<double>(0),z*ray.at<double>(1),z*ray.at<double>(2)};
}
cv::Vec3d Residual3DDetector::Difference(const cv::Vec3d& p,const cv::Vec3d& q,const cv::Mat& R,const cv::Mat& t) {
    if(R.type()!=CV_64FC1||R.rows!=3||R.cols!=3||t.type()!=CV_64FC1||t.total()!=3||!cv::checkRange(R)||!cv::checkRange(t)||!Finite(p)||!Finite(q)) return cv::Vec3d::all(NAN);
    cv::Mat prediction=R*cv::Mat(p)+t.reshape(1,3);
    cv::Vec3d v(prediction.at<double>(0),prediction.at<double>(1),prediction.at<double>(2));
    if(!Finite(v)||v[2]<=0||q[2]<=0) return cv::Vec3d::all(NAN);
    return q-v;
}
Residual3DDetector::Residual3DDetector(const Residual3DConfig& c):config_(c) {
    Require(c.enabled,"disabled detector must not be constructed");
    if(c.debug||c.visualize) EnsureSemanticDirectory(c.output);
    if(c.debug) {
        frames_.open(c.output+"/frames.csv"); features_.open(c.output+"/features.csv");
        Require(frames_.good()&&features_.good(),"cannot open output");
        features_<<"frame_id,feature_id,previous_feature_id,instance_id,epipolar_residual_px,residual_3d_m,previous_depth_m,current_depth_m,depth_valid,epipolar_state,depth_state,reason,previous_depth_valid,current_depth_valid,previous_depth_reason,current_depth_reason,residual_x_m,residual_y_m,residual_z_m,pnp_background_inlier,epipolar_consistent_3d_anomaly,pose_source,previous_frame_id,previous_timestamp,current_timestamp,tracking_inliers,transform_valid,transform_reason,previous_raw_u,previous_raw_v,current_raw_u,current_raw_v,previous_un_u,previous_un_v,current_un_u,current_un_v,raw_residual_3d_m,depth_difference_m,depth_difference_gate_enabled,depth_difference_gate_rejected\n";
        frames_<<"frame_id,previous_frame_id,matches,background_candidates,pnp_inliers,pnp_inlier_ratio,motion_valid,reason,pnp_coverage,pnp_shape_ratio,pnp_reprojection_rms_px,valid_depth_ratio,background_count,background_median_m,background_p90_m,instance_count,instance_median_m,instance_p90_m,fit_background_count,fit_background_median_m,nonfit_background_count,nonfit_background_median_m,match_ms,estimate_ms,residual_ms,pose_source,current_frame_id,previous_timestamp,current_timestamp,previous_tracking_frame_id,current_tracking_frame_id,previous_tracking_timestamp,current_tracking_timestamp,previous_tracking_inliers,tracking_inliers,previous_map_id,map_id,previous_map_change,map_change,previous_big_change,big_change,previous_relocalization_frame,relocalization_frame,transform_valid,transform_reason,previous_pose_rebased,previous_pose_rebase_reason,previous_original_map_change,depth_difference_gate_enabled,max_depth_difference_m,depth_difference_rejected\n";
        frames_<<std::setprecision(17);features_<<std::setprecision(17);
    }
}
DepthRelativeMotion Residual3DDetector::BackgroundPnP(const EpipolarInput& c,const DiagnosticPair& pair) {
    DepthRelativeMotion r;r.poseSource="background_pnp";
    r.currentFrameId=c.frameId;r.currentTimestamp=c.timestamp;r.previousFrameId=pair.matching.previousFrameId;r.previousTimestamp=pair.matching.previousFrameId>=0?pair.previous.timestamp:double(NAN);
    r.backgroundInliers.assign(c.rawKeys.size(),-1);
    const auto& p=pair.previous;
    std::vector<cv::Point3d> object; std::vector<cv::Point2f> pixels; std::vector<int> indices;
    for(size_t j=0;j<c.rawKeys.size();++j) {
        int i=pair.matching.features[j].previousIndex;if(i<0) continue;
        auto previousDepth=Sample(p,i,config_);
        if(p.instances[i]==0&&c.instances[j]==0&&previousDepth.valid) {
            auto xyz=Backproject(previousDepth.meters,p.undistortedKeys[i].pt,p.K); auto uv=c.undistortedKeys[j].pt;
            if(Finite(xyz)&&std::isfinite(uv.x)&&std::isfinite(uv.y)) {object.emplace_back(xyz[0],xyz[1],xyz[2]);pixels.push_back(uv);indices.push_back(j); r.backgroundInliers[j]=0;}
        }
    }
    r.background=object.size();
    auto start=Clock::now();
    auto estimate=[&]() -> std::string {
        if(pair.matching.reason!="OK") return pair.matching.reason;
        if(r.background<config_.minBackground) return "INSUFFICIENT_BACKGROUND";
        cv::Mat rv,tv,inliers,K; c.K.convertTo(K,CV_64F);
        bool ok=cv::solvePnPRansac(object,pixels,K,cv::Mat(),rv,tv,false,config_.iterations,config_.ransacPx,config_.confidence,inliers,cv::SOLVEPNP_EPNP);
        if(!ok||rv.empty()||tv.empty()||!cv::checkRange(rv)||!cv::checkRange(tv)) return "PNP_FAILED";
        r.inliers=inliers.total();r.inlierRatio=double(r.inliers)/r.background;
        if(r.inliers<config_.minInliers||r.inlierRatio<config_.minInlierRatio) return "INSUFFICIENT_PNP_SUPPORT";
        std::vector<cv::Point3d> fit;std::vector<cv::Point2f> obs,prev;
        for(int k=0;k<r.inliers;++k) {
            int b=inliers.at<int>(k); if(b<0||b>=r.background) return "INVALID_PNP_INLIER";
            int j=indices[b];r.backgroundInliers[j]=1;fit.push_back(object[b]);obs.push_back(pixels[b]);prev.push_back(p.undistortedKeys[pair.matching.features[j].previousIndex].pt);
        }
        r.coverage=std::min(Coverage(obs,c.size),Coverage(prev,p.size));
        if(r.coverage<config_.minCoverage) return "INSUFFICIENT_COVERAGE";
        cv::Mat centered(r.inliers,3,CV_64F);cv::Point3d mean;
        for(auto v:fit) mean+=v;
        mean*=1./fit.size();
        for(int k=0;k<r.inliers;++k) {auto v=fit[k]-mean; centered.at<double>(k,0)=v.x;centered.at<double>(k,1)=v.y;centered.at<double>(k,2)=v.z;}
        cv::SVD svd(centered,cv::SVD::NO_UV);r.shapeRatio=svd.w.at<double>(0)>1e-12?svd.w.at<double>(2)/svd.w.at<double>(0):0;
        if(!std::isfinite(r.shapeRatio)||r.shapeRatio<config_.minShapeRatio) return "DEGENERATE_BACKGROUND_SHAPE";
        // Refinement uses only background RANSAC inliers.
        if(!cv::solvePnP(fit,obs,K,cv::Mat(),rv,tv,true,cv::SOLVEPNP_ITERATIVE)||!cv::checkRange(rv)||!cv::checkRange(tv)) return "PNP_REFINEMENT_FAILED";
        cv::Rodrigues(rv,r.R);tv.convertTo(r.t,CV_64F);
        if(!cv::checkRange(r.R)||!cv::checkRange(r.t)||std::abs(cv::determinant(r.R)-1)>1e-6) return "NONFINITE_TRANSFORM";
        std::vector<cv::Point2d> projected;std::vector<cv::Point2d> obs64;for(auto v:obs) obs64.emplace_back(v.x,v.y);
        cv::projectPoints(fit,rv,r.t,K,cv::Mat(),projected);
        double sum=0;
        for(size_t k=0;k<fit.size();++k) {
            auto v=fit[k];cv::Mat pred=r.R*(cv::Mat_<double>(3,1)<<v.x,v.y,v.z)+r.t;
            if(!cv::checkRange(pred)||pred.at<double>(2)<=0) return "NONPOSITIVE_PREDICTION";
            double error=cv::norm(projected[k]-obs64[k]);if(!std::isfinite(error)||error>config_.ransacPx) return "REPROJECTION_UNRELIABLE";sum+=error*error;
        }
        r.reprojectionRms=std::sqrt(sum/fit.size());
        if(r.reprojectionRms>config_.maxReprojectionRms) return "REPROJECTION_UNRELIABLE";
        return "OK";
    };
    try {r.reason=estimate();} catch(const cv::Exception&) {r.reason="PNP_EXCEPTION";}
    r.motionValid=r.reason=="OK";r.estimateMs=Ms(start);
    if(r.motionValid) {r.T_current_previous=cv::Mat::eye(4,4,CV_64F);r.R.copyTo(r.T_current_previous(cv::Rect(0,0,3,3)));r.t.copyTo(r.T_current_previous(cv::Rect(3,0,1,3)));}
    return r;
}

bool Residual3DDetector::RebasePreviousPose(TrackingDepthPose& previous,const TrackingDepthPose& current,
    const cv::Matx44d& latestReferenceTcw,long referenceId,long referenceMapId,bool referenceValid) {
    auto fail=[&](const char* reason) {previous.rebaseReason=reason;return false;};
    if(previous.mapId<0 || previous.mapId!=current.mapId) return fail("MAP_CHANGED");
    if(previous.bigChange!=current.bigChange) return fail("BIG_CHANGE");
    if(previous.relocalizationFrame!=current.relocalizationFrame) return fail("RELOCALIZED");
    if(!previous.hasPose||!previous.trackingOK||current.frameId<=previous.frameId) return fail("PREVIOUS_POSE_UNUSABLE");
    if(!referenceValid||previous.referenceId<0||referenceId!=previous.referenceId||referenceMapId!=current.mapId) return fail("REFERENCE_UNAVAILABLE_OR_CHANGED");
    cv::Matx44d updated=previous.frameToReference*latestReferenceTcw;
    if(!cv::checkRange(cv::Mat(updated))||!cv::checkRange(cv::Mat(latestReferenceTcw))) return fail("REFERENCE_NONFINITE");
    previous.Tcw=updated;previous.referenceTcw=latestReferenceTcw;
    previous.mapChange=current.mapChange;previous.rebased=true;previous.rebaseReason="REFERENCE_REFRESHED";
    return true;
}

DepthRelativeMotion Residual3DDetector::TrackingMotion(const EpipolarInput& c,const DiagnosticPair& pair,
    const TrackingDepthPose& previous,const TrackingDepthPose& current,const Residual3DConfig& cfg) {
    auto start=Clock::now(); DepthRelativeMotion r; r.poseSource="tracking";
    r.previousFrameId=pair.matching.previousFrameId;r.currentFrameId=c.frameId;
    r.previousTimestamp=pair.matching.previousFrameId>=0?pair.previous.timestamp:double(NAN);r.currentTimestamp=c.timestamp;
    r.previousTracking=previous;r.currentTracking=current;
    auto evaluate=[&]() -> std::string {
        if(pair.matching.previousFrameId<0) return "NO_PREVIOUS_FRAME";
        if(pair.matching.reason!="OK") return pair.matching.reason;
        if(previous.frameId!=r.previousFrameId||current.frameId!=r.currentFrameId||
           previous.timestamp!=r.previousTimestamp||current.timestamp!=r.currentTimestamp) return "TRACKING_POSE_PAIR_MISMATCH";
        const double dt=current.timestamp-previous.timestamp;
        if(!std::isfinite(dt)||dt<=0||dt>cfg.maxDt||current.frameId<=previous.frameId) return "TRACKING_TIME_GAP";
        if(!current.trackingOK) return "CURRENT_TRACKING_NOT_OK";
        if(!previous.trackingOK) return "PREVIOUS_TRACKING_NOT_OK";
        if(!current.hasPose||!previous.hasPose) return "TRACKING_POSE_MISSING";
        auto rigid=[](const cv::Matx44d& t) {
            cv::Mat m(t); if(!cv::checkRange(m)) return false;
            cv::Mat R=m(cv::Rect(0,0,3,3));
            return cv::norm(R.t()*R-cv::Mat::eye(3,3,CV_64F))<1e-4&&std::abs(cv::determinant(R)-1)<1e-4&&
                std::abs(t(3,0))+std::abs(t(3,1))+std::abs(t(3,2))+std::abs(t(3,3)-1)<1e-9;
        };
        if(!rigid(current.Tcw)||!rigid(previous.Tcw)) return "TRACKING_POSE_NONFINITE_OR_NONRIGID";
        if(current.mapId<0||previous.mapId<0||current.mapId!=previous.mapId) return "TRACKING_MAP_CHANGED";
        if(current.relocalizationFrame!=previous.relocalizationFrame) return "TRACKING_RELOCALIZED";
        if(current.mapChange!=previous.mapChange||current.bigChange!=previous.bigChange) return "TRACKING_MAP_REVISION_CHANGED";
        if(current.inliers<cfg.trackingMinInliers) return "CURRENT_TRACKING_INLIERS_LOW_OR_UNAVAILABLE";
        cv::Matx44d relative=current.Tcw*previous.Tcw.inv(cv::DECOMP_LU);
        if(!rigid(relative)) return "TRACKING_RELATIVE_TRANSFORM_INVALID";
        r.T_current_previous=cv::Mat(relative).clone();r.R=r.T_current_previous(cv::Rect(0,0,3,3)).clone();r.t=r.T_current_previous(cv::Rect(3,0,1,3)).clone();
        return "OK";
    };
    r.reason=evaluate();r.motionValid=r.reason=="OK";r.estimateMs=Ms(start);return r;
}
Residual3DResult Residual3DDetector::Process(const EpipolarInput& c,const DiagnosticPair& pair,const EpipolarResult* epi,const cv::Mat& image,bool rgb,const DepthRelativeMotion* motion) {
    Residual3DResult r;
    Require(pair.matching.features.size()==c.rawKeys.size(),"shared match size mismatch");
    if(config_.poseSource=="background_pnp") static_cast<DepthRelativeMotion&>(r)=BackgroundPnP(c,pair);
    else if(motion) {
        Require(motion->poseSource=="tracking","wrong motion source");
        static_cast<DepthRelativeMotion&>(r)=*motion;
    } else {
        r.poseSource="tracking";r.reason="TRACKING_POSE_UNAVAILABLE";
        r.currentFrameId=c.frameId;r.currentTimestamp=c.timestamp;r.previousFrameId=pair.matching.previousFrameId;r.previousTimestamp=pair.matching.previousFrameId>=0?pair.previous.timestamp:double(NAN);
    }
    if(r.currentFrameId!=long(c.frameId)||r.previousFrameId!=pair.matching.previousFrameId||r.currentTimestamp!=c.timestamp||
       (r.previousFrameId>=0&&r.previousTimestamp!=pair.previous.timestamp)) {r.motionValid=false;r.reason="MOTION_PAIR_MISMATCH";}
    if(r.motionValid) {
        if(r.T_current_previous.type()!=CV_64FC1||r.T_current_previous.rows!=4||r.T_current_previous.cols!=4||!cv::checkRange(r.T_current_previous)) {
            r.motionValid=false;r.reason="MOTION_TRANSFORM_INVALID";
        } else {r.R=r.T_current_previous(cv::Rect(0,0,3,3)).clone();r.t=r.T_current_previous(cv::Rect(3,0,1,3)).clone();}
    }
    r.features.resize(c.rawKeys.size());const auto& p=pair.previous;int valid=0;
    for(size_t j=0;j<r.features.size();++j) {
        auto& f=r.features[j];f.previousIndex=pair.matching.features[j].previousIndex;
        f.currentDepth=Sample(c,j,config_);if(f.currentDepth.valid) ++valid;
        if(f.previousIndex>=0) {f.previousDepth=Sample(p,f.previousIndex,config_);f.reason="MOTION_UNRELIABLE";}
        else f.reason=pair.matching.reason=="OK"?"NO_MATCH":pair.matching.reason;
        if(j<r.backgroundInliers.size()) f.backgroundInlier=r.backgroundInliers[j];
        if(f.previousDepth.valid && f.currentDepth.valid) {
            f.depthDifference=std::abs(f.currentDepth.meters-f.previousDepth.meters);
            f.depthDifferenceRejected=config_.useDepthDifferenceGate && f.depthDifference>config_.maxDepthDifferenceM;
        }
    }
    r.validDepthRatio=r.features.empty()?0:double(valid)/r.features.size();auto start=Clock::now();
    for(size_t j=0;j<r.features.size();++j) {
        auto& f=r.features[j];if(f.previousIndex<0) continue;
        if(!f.previousDepth.valid) {f.reason="PREVIOUS_"+f.previousDepth.reason;continue;}
        if(!f.currentDepth.valid) {f.reason="CURRENT_"+f.currentDepth.reason;continue;}
        if(!r.motionValid) {f.reason=r.reason;continue;}
        int i=f.previousIndex;
        f.vector=Difference(Backproject(f.previousDepth.meters,p.undistortedKeys[i].pt,p.K),Backproject(f.currentDepth.meters,c.undistortedKeys[j].pt,c.K),r.R,r.t);
        if(!Finite(f.vector)) {f.reason="INVALID_PREDICTION_OR_RESIDUAL";continue;}
        f.rawResidual=cv::norm(f.vector);
        if(f.depthDifferenceRejected) {f.vector=cv::Vec3d::all(NAN);f.reason="CROSS_FRAME_DEPTH_DIFFERENCE";continue;}
        f.residual=f.rawResidual;
        if(!std::isfinite(f.residual)) {f.residual=NAN;f.reason="NONFINITE_RESIDUAL";continue;}
        if(p.instances[i]<0||c.instances[j]<0) {f.reason="INVALID_ASSOCIATION";continue;}
        if(f.residual<=config_.lowM) {f.state=EpipolarState::GEOMETRY_CONSISTENT;f.reason="LOW_3D_RESIDUAL";}
        else if(f.residual>=config_.highM) {f.state=EpipolarState::SUSPECTED_DYNAMIC;f.reason="HIGH_3D_RESIDUAL_GEOMETRIC_ANOMALY";}
        else f.reason="AMBIGUOUS_3D_RESIDUAL";
    }
    r.residualMs=Ms(start);
    if(sequence_++%config_.every==0&&(config_.debug||config_.visualize)) Record(c,pair,r,epi,image,rgb);
    return r;
}
void Residual3DDetector::Record(const EpipolarInput& c,const DiagnosticPair& pair,const Residual3DResult& r,const EpipolarResult* epi,const cv::Mat& image,bool rgb) {
    std::vector<double> bg,instance,fit,nonfit;
    for(size_t j=0;j<r.features.size();++j) {const auto& f=r.features[j];if(!std::isfinite(f.residual)) continue;
        if(c.instances[j]==0) {bg.push_back(f.residual);(f.backgroundInlier==1?fit:nonfit).push_back(f.residual);} else if(c.instances[j]>0) instance.push_back(f.residual);
    }
    auto percentile=[](std::vector<double> values,double q) {if(values.empty()) return double(NAN);std::sort(values.begin(),values.end());return values[size_t(q*(values.size()-1))];};
    std::cout<<"[Residual3D] frame="<<c.frameId<<" pose_source="<<r.poseSource<<" tracking_inliers="<<r.currentTracking.inliers<<" motion_valid="<<r.motionValid<<" reason="<<r.reason<<" pnp_bg="<<r.background<<" pnp_inliers="<<r.inliers<<" pnp_ratio="<<r.inlierRatio<<" depth_valid_ratio="<<r.validDepthRatio<<" bg/instance_median_m="<<percentile(bg,.5)<<'/'<<percentile(instance,.5)<<" (尚未剔除特征; fitted background is not independent validation)"<<std::endl;
    if(config_.debug) {
        frames_<<c.frameId<<','<<pair.matching.previousFrameId<<','<<pair.matching.matches<<','<<r.background<<','<<r.inliers<<','<<r.inlierRatio<<','<<r.motionValid<<','<<r.reason<<','<<r.coverage<<','<<r.shapeRatio<<','<<r.reprojectionRms<<','<<r.validDepthRatio<<','<<bg.size()<<','<<percentile(bg,.5)<<','<<percentile(bg,.9)<<','<<instance.size()<<','<<percentile(instance,.5)<<','<<percentile(instance,.9)<<','<<fit.size()<<','<<percentile(fit,.5)<<','<<nonfit.size()<<','<<percentile(nonfit,.5)<<','<<pair.matching.matchMs<<','<<r.estimateMs<<','<<r.residualMs
            <<','<<r.poseSource<<','<<r.currentFrameId<<','<<r.previousTimestamp<<','<<r.currentTimestamp<<',';
        if(r.poseSource=="tracking") frames_<<r.previousTracking.frameId<<','<<r.currentTracking.frameId<<','
            <<r.previousTracking.timestamp<<','<<r.currentTracking.timestamp<<','<<r.previousTracking.inliers<<','<<r.currentTracking.inliers<<','
            <<r.previousTracking.mapId<<','<<r.currentTracking.mapId<<','<<r.previousTracking.mapChange<<','<<r.currentTracking.mapChange<<','
            <<r.previousTracking.bigChange<<','<<r.currentTracking.bigChange<<','<<r.previousTracking.relocalizationFrame<<','<<r.currentTracking.relocalizationFrame;
        else frames_<<",,,,,,,,,,,,,";
        frames_<<','<<r.motionValid<<','<<r.reason<<','<<r.previousTracking.rebased<<','<<r.previousTracking.rebaseReason<<','<<r.previousTracking.originalMapChange<<','<<config_.useDepthDifferenceGate<<','<<config_.maxDepthDifferenceM<<','<<std::count_if(r.features.begin(),r.features.end(),[](const Residual3DFeature& f){return f.depthDifferenceRejected;})<<'\n';
        for(size_t j=0;j<r.features.size();++j) {
            const auto& f=r.features[j];features_<<c.frameId<<','<<j<<','<<f.previousIndex<<','<<c.instances[j]<<',';
            if(epi&&std::isfinite(epi->features[j].residual)) features_<<epi->features[j].residual;
            features_<<','<<f.residual<<','<<f.previousDepth.meters<<','<<f.currentDepth.meters<<','<<(f.previousDepth.valid&&f.currentDepth.valid)<<',';
            if(epi) features_<<EpipolarStateName(epi->features[j].state);
            features_<<','<<EpipolarStateName(f.state)<<','<<f.reason<<','<<f.previousDepth.valid<<','<<f.currentDepth.valid<<','<<f.previousDepth.reason<<','<<f.currentDepth.reason<<','<<f.vector[0]<<','<<f.vector[1]<<','<<f.vector[2]<<','<<f.backgroundInlier<<',';
            if(epi) features_<<(epi->features[j].state==EpipolarState::GEOMETRY_CONSISTENT&&f.state==EpipolarState::SUSPECTED_DYNAMIC);
            features_<<','<<r.poseSource<<','<<r.previousFrameId<<','<<r.previousTimestamp<<','<<r.currentTimestamp<<',';
            if(r.poseSource=="tracking") features_<<r.currentTracking.inliers;
            features_<<','<<r.motionValid<<','<<r.reason;
            auto pixel=[&](cv::Point2f p) {features_<<','<<p.x<<','<<p.y;};
            if(f.previousIndex>=0) pixel(pair.previous.rawKeys[f.previousIndex].pt);else features_<<",nan,nan";
            pixel(c.rawKeys[j].pt);
            if(f.previousIndex>=0) pixel(pair.previous.undistortedKeys[f.previousIndex].pt);else features_<<",nan,nan";
            pixel(c.undistortedKeys[j].pt);
            features_<<','<<f.rawResidual<<','<<f.depthDifference<<','<<config_.useDepthDifferenceGate<<','<<f.depthDifferenceRejected<<'\n';
        }
        cv::FileStorage model(config_.output+"/motion_"+std::to_string(c.frameId)+".json",cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
        Require(model.isOpened(),"cannot write transform audit");model<<"frame_id"<<double(c.frameId)<<"motion_valid"<<int(r.motionValid)<<"reason"<<r.reason<<"direction"<<"P_current = R * P_previous + t; metres"<<"R"<<r.R<<"t_m"<<r.t<<"K"<<c.K
            <<"T_current_previous"<<r.T_current_previous<<"pose_source"<<r.poseSource
            <<"previous_frame_id"<<double(r.previousFrameId)<<"current_frame_id"<<double(r.currentFrameId)
            <<"current_timestamp"<<r.currentTimestamp<<"depth_difference_gate_enabled"<<int(config_.useDepthDifferenceGate)<<"max_depth_difference_m"<<config_.maxDepthDifferenceM;
        if(std::isfinite(r.previousTimestamp)) model<<"previous_timestamp"<<r.previousTimestamp;
        if(r.poseSource=="tracking") {
            auto pose=[&](const char* name,const TrackingDepthPose& p) {
                model<<name<<"{"<<"frame_id"<<double(p.frameId)<<"has_pose"<<int(p.hasPose)
                    <<"tracking_ok"<<int(p.trackingOK)<<"tracking_inliers"<<p.inliers<<"map_id"<<double(p.mapId)
                    <<"map_change"<<p.mapChange<<"big_change"<<p.bigChange<<"relocalization_frame"<<double(p.relocalizationFrame)
                    <<"Tcw"<<(p.hasPose?cv::Mat(p.Tcw):cv::Mat())
                    <<"original_Tcw"<<(p.hasPose?cv::Mat(p.originalTcw):cv::Mat())<<"original_map_change"<<p.originalMapChange
                    <<"reference_id"<<double(p.referenceId)<<"frame_to_reference"<<(p.referenceId>=0?cv::Mat(p.frameToReference):cv::Mat())
                    <<"reference_Tcw"<<(p.referenceId>=0?cv::Mat(p.referenceTcw):cv::Mat())<<"rebased"<<int(p.rebased)<<"rebase_reason"<<p.rebaseReason;
                if(std::isfinite(p.timestamp)) model<<"timestamp"<<p.timestamp;
                model<<"}";
            };
            pose("previous_tracking",r.previousTracking);pose("current_tracking",r.currentTracking);
        }
        frames_.flush();features_.flush();Require(frames_.good()&&features_.good(),"CSV write failed");
    }
    if(config_.visualize&&!image.empty()) {
        cv::Mat canvas;if(image.channels()==1) cv::cvtColor(image,canvas,cv::COLOR_GRAY2BGR);else if(image.channels()==4) cv::cvtColor(image,canvas,rgb?cv::COLOR_RGBA2BGR:cv::COLOR_BGRA2BGR);else if(rgb) cv::cvtColor(image,canvas,cv::COLOR_RGB2BGR);else canvas=image.clone();
        for(size_t j=0;j<r.features.size();++j) {
            auto point=c.rawKeys[j].pt;if(!std::isfinite(point.x)||!std::isfinite(point.y)) continue;
            const auto& f=r.features[j];cv::Scalar color=f.state==EpipolarState::GEOMETRY_CONSISTENT?cv::Scalar(0,255,0):f.state==EpipolarState::SUSPECTED_DYNAMIC?cv::Scalar(0,0,255):cv::Scalar(0,255,255);
            // Squares mark fitted background; larger circles mark instance tests.
            if(f.backgroundInlier==1) cv::rectangle(canvas,point-cv::Point2f(2,2),point+cv::Point2f(2,2),color,1);
            else cv::circle(canvas,point,c.instances[j]>0?3:2,color,1);
            if(epi&&epi->features[j].state==EpipolarState::GEOMETRY_CONSISTENT&&f.state==EpipolarState::SUSPECTED_DYNAMIC) cv::circle(canvas,point,5,{255,0,255},1);
        }
        cv::copyMakeBorder(canvas,canvas,70,0,0,0,cv::BORDER_CONSTANT,{0,0,0});
        cv::putText(canvas,"3D "+r.poseSource+"; features NOT removed",{5,18},cv::FONT_HERSHEY_SIMPLEX,.45,{255,255,255},1);
        cv::putText(canvas,r.poseSource=="tracking"?"circle: diagnostic test; magenta ring: epi consistent / 3D anomaly":"square: PnP fit; circle: test; magenta ring: epi consistent / 3D anomaly",{5,38},cv::FONT_HERSHEY_SIMPLEX,.38,{255,255,255},1);
        cv::putText(canvas,r.reason,{5,58},cv::FONT_HERSHEY_SIMPLEX,.45,{255,255,255},1);
        Require(cv::imwrite(config_.output+"/residual3d_"+std::to_string(c.frameId)+".png",canvas),"cannot write overlay");
    }
}
}
