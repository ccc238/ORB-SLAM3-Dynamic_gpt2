#include "EpipolarMotionDetector.h"
#include "SemanticMasker.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace ORB_SLAM3 {
namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point a) {return std::chrono::duration<double,std::milli>(Clock::now()-a).count();}
void Require(bool b,const std::string& msg) {if(!b) throw std::runtime_error("Epipolar: "+msg);}
int Integer(cv::FileStorage& fs,const char* key,int fallback) {
    auto n=fs[key]; if(n.empty()) return fallback;
    Require(n.isInt(),std::string(key)+" must be an integer"); return (int)n;
}
bool Flag(cv::FileStorage& fs,const char* key,bool fallback) {
    int n=Integer(fs,key,fallback?1:0); Require(n==0 || n==1,std::string(key)+" must be 0 or 1"); return n==1;
}
void Number(cv::FileStorage& fs,const char* key,double& v) {
    auto n=fs[key]; if(n.empty()) return;
    Require(n.isInt() || n.isReal(),std::string(key)+" must be numeric"); v=(double)n;
    Require(std::isfinite(v),std::string(key)+" must be finite");
}
bool Finite(cv::Point2f p) {return std::isfinite(p.x) && std::isfinite(p.y);}
bool Equal(const cv::Mat& a,const cv::Mat& b) {
    return a.size()==b.size() && a.type()==b.type() && (a.empty() || cv::norm(a,b,cv::NORM_INF)<1e-9);
}
double Coverage(const std::vector<cv::Point2f>& p,cv::Size size) {
    // Occupied 4x4 cells, rather than a bounding box that isolated points inflate.
    bool cells[16]={};
    for(auto q:p) if(q.x>=0 && q.y>=0 && q.x<size.width && q.y<size.height)
        cells[std::min(3,int(q.y*4/size.height))*4+std::min(3,int(q.x*4/size.width))]=true;
    return std::count(cells,cells+16,true)/16.0;
}
cv::Mat Bgr(const cv::Mat& im,bool rgb) {
    cv::Mat out;
    if(im.channels()==1) cv::cvtColor(im,out,cv::COLOR_GRAY2BGR);
    else if(im.channels()==4) cv::cvtColor(im,out,rgb?cv::COLOR_RGBA2BGR:cv::COLOR_BGRA2BGR);
    else if(rgb) cv::cvtColor(im,out,cv::COLOR_RGB2BGR);
    else out=im.clone();
    return out;
}
}
EpipolarConfig EpipolarConfig::Read(cv::FileStorage& fs,bool segmentation,bool association,bool masking) {
    EpipolarConfig c;
    c.configuredValue=Integer(fs,"Dynamic.UseEpipolar",1);
    Require(c.configuredValue==0 || c.configuredValue==1,"Dynamic.UseEpipolar accepts integer 0 or 1 only (0=启用，1=关闭)");
    c.enabled=(c.configuredValue==0);
    std::cout << "Dynamic.UseEpipolar=" << c.configuredValue << " -> Epipolar diagnostic " << (c.enabled?"ENABLED":"DISABLED") << " (0=启用，1=关闭)" << std::endl;
    if(!c.enabled) return c;
    Require(segmentation && association && !masking,"diagnostic requires online segmentation and instance association enabled, semantic feature masking disabled");
    c.debug=Flag(fs,"Epipolar.Debug",false); c.visualize=Flag(fs,"Epipolar.Visualize",false);
    c.drawMatches=Flag(fs,"Epipolar.DrawMatches",true);
    c.every=Integer(fs,"Epipolar.SaveEvery",c.every);
    c.minBackground=Integer(fs,"Epipolar.MinBackground",c.minBackground);
    c.minInliers=Integer(fs,"Epipolar.MinInliers",c.minInliers);
    Number(fs,"Epipolar.MatchRatio",c.ratio); Number(fs,"Epipolar.RansacPx",c.ransacPx);
    Number(fs,"Epipolar.Confidence",c.confidence); Number(fs,"Epipolar.MinInlierRatio",c.minInlierRatio);
    Number(fs,"Epipolar.LowPx",c.lowPx); Number(fs,"Epipolar.HighPx",c.highPx);
    Number(fs,"Epipolar.MaxDt",c.maxDt); Number(fs,"Epipolar.MinCoverage",c.minCoverage);
    Number(fs,"Epipolar.MinDisplacementPx",c.minDisplacementPx); Number(fs,"Epipolar.HomographyRatio",c.homographyRatio);
    if(!fs["Epipolar.OutputDirectory"].empty()) fs["Epipolar.OutputDirectory"] >> c.output;
    Require(c.every>0 && c.minBackground>=8 && c.minInliers>=8,"invalid frequency/background counts");
    Require(c.ratio>0 && c.ratio<1 && c.ransacPx>0 && c.confidence>0 && c.confidence<1,"invalid matching/RANSAC parameters");
    Require(c.minInlierRatio>0 && c.minInlierRatio<=1 && c.minCoverage>0 && c.minCoverage<=1,"invalid support/coverage thresholds");
    Require(c.lowPx>=0 && c.lowPx<c.highPx && c.maxDt>0 && c.minDisplacementPx>=0 && c.homographyRatio>0 && c.homographyRatio<=1,"invalid classification/time/degeneracy thresholds");
    return c;
}
const char* EpipolarStateName(EpipolarState s) {
    switch(s) {case EpipolarState::GEOMETRY_CONSISTENT:return "GEOMETRY_CONSISTENT";
    case EpipolarState::SUSPECTED_DYNAMIC:return "SUSPECTED_DYNAMIC"; default:return "UNKNOWN";}
}
EpipolarMotionDetector::EpipolarMotionDetector(const EpipolarConfig& c):config_(c) {
    Require(c.enabled,"do not construct diagnostic resources when disabled");
    if(c.debug || c.visualize) EnsureSemanticDirectory(c.output);
    if(c.debug) {
        frames_.open(c.output+"/frames.csv"); features_.open(c.output+"/features.csv");
        Require(frames_.good() && features_.good(),"cannot open diagnostic CSV");
        features_ << "frame_id,timestamp,previous_frame_id,current_feature_index,previous_feature_index,current_instance_id,previous_instance_id,epipolar_residual_px,state,reason,background_ransac_inlier,previous_raw_u,previous_raw_v,current_raw_u,current_raw_v,previous_undistorted_u,previous_undistorted_v,current_undistorted_u,current_undistorted_v\n";
        frames_ << "frame_id,timestamp,previous_frame_id,dt,total_features,matches,background,inliers,inlier_ratio,coverage,median_raw_displacement_px,homography_support,model_valid,reason,unknown,consistent,suspected,match_ms,estimate_ms,residual_ms\n";
        frames_ << std::setprecision(17); features_ << std::setprecision(17);
    }
}
void EpipolarMotionDetector::Reset() {previous_=EpipolarInput(); havePrevious_=false; previousImage_.release(); sequence_=0;}
double EpipolarMotionDetector::Residual(const cv::Mat& F,cv::Point2d p,cv::Point2d q) {
    const double invalid=std::numeric_limits<double>::quiet_NaN();
    if(F.rows!=3 || F.cols!=3 || F.type()!=CV_64FC1 || !cv::checkRange(F) ||
       !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(q.x) || !std::isfinite(q.y)) return invalid;
    cv::Matx33d f; for(int y=0;y<3;++y) for(int x=0;x<3;++x) f(y,x)=F.at<double>(y,x);
    double scale=cv::norm(F); if(!std::isfinite(scale) || scale<1e-15) return invalid; f*=1/scale;
    cv::Vec3d a(p.x,p.y,1), b(q.x,q.y,1), fa=f*a, ftb=f.t()*b;
    double den=fa[0]*fa[0]+fa[1]*fa[1]+ftb[0]*ftb[0]+ftb[1]*ftb[1];
    if(!std::isfinite(den) || den<1e-20) return invalid;
    double r=std::abs(b.dot(fa))/std::sqrt(den); return std::isfinite(r)?r:invalid;
}
DiagnosticPair DiagnosticMatcher::Process(const EpipolarInput& c,double ratio,double maxDt) {
    Require(c.instances.size()==c.rawKeys.size() && c.undistortedKeys.size()==c.rawKeys.size(),"invalid diagnostic arrays");
    Require(c.descriptors.empty() || c.descriptors.rows==int(c.rawKeys.size()),"invalid diagnostic descriptors");
    DiagnosticPair pair; pair.previous=previous_;
    auto& r=pair.matching; r.features.resize(c.rawKeys.size());
    const auto& p=pair.previous;
    auto fail=[&](const std::string& why) {r.reason=why; for(auto& f:r.features) f.reason=why; return r;};
    auto match=[&]() -> EpipolarResult {
        if(!havePrevious_) return fail("NO_PREVIOUS_FRAME");
        r.previousFrameId=p.frameId; r.dt=c.timestamp-p.timestamp;
        if(c.size!=p.size || !Equal(c.K,p.K) || !Equal(c.distortion,p.distortion)) return fail("INCOMPATIBLE_INPUT");
        if(!std::isfinite(r.dt) || r.dt<=0 || r.dt>maxDt || c.frameId<=p.frameId) return fail("TIME_GAP");
        if(c.descriptors.empty() || p.descriptors.empty() || c.descriptors.rows<2 || p.descriptors.rows<2) return fail("NO_MATCH");
        if(c.descriptors.type()!=CV_8UC1 || p.descriptors.type()!=CV_8UC1 || c.descriptors.cols!=32 || p.descriptors.cols!=32) return fail("INVALID_DESCRIPTOR");
    auto start=Clock::now();
    cv::BFMatcher matcher(cv::NORM_HAMMING); std::vector<std::vector<cv::DMatch>> forward,backward;
    matcher.knnMatch(p.descriptors,c.descriptors,forward,2); matcher.knnMatch(c.descriptors,p.descriptors,backward,2);
    std::vector<std::pair<int,int>> matches;
    for(size_t i=0;i<forward.size();++i) {
        const auto& a=forward[i]; if(a.size()<2 || !(a[0].distance<ratio*a[1].distance)) continue;
        int j=a[0].trainIdx; const auto& b=backward[j];
        if(b.size()<2 || b[0].trainIdx!=int(i) || !(b[0].distance<ratio*b[1].distance)) continue;
        matches.emplace_back(i,j); auto& f=r.features[j]; f.previousIndex=i; f.previousInstance=p.instances[i]; f.reason="MODEL_UNRELIABLE";
    }
    r.matchMs=Ms(start); r.matches=matches.size();
    if(matches.empty()) return fail("NO_MATCH");
        r.reason="OK"; return r;
    };
    pair.matching=match();
    previous_=c; previous_.descriptors=c.descriptors.clone(); previous_.K=c.K.clone(); previous_.distortion=c.distortion.clone();
    havePrevious_=true; return pair;
}
EpipolarResult EpipolarMotionDetector::Evaluate(const EpipolarInput& c,const DiagnosticPair* pair) {
    EpipolarResult r; r.features.resize(c.rawKeys.size());
    auto fail=[&](const std::string& why) {r.reason=why; for(auto& f:r.features) if(f.previousIndex>=0 || why=="NO_PREVIOUS_FRAME" || why=="INCOMPATIBLE_INPUT" || why=="TIME_GAP") f.reason=why; return r;};
    if(!havePrevious_) return fail("NO_PREVIOUS_FRAME");
    const auto& p=previous_; r.previousFrameId=p.frameId; r.dt=c.timestamp-p.timestamp;
    if(c.size!=p.size || !Equal(c.K,p.K) || !Equal(c.distortion,p.distortion)) return fail("INCOMPATIBLE_INPUT");
    if(!std::isfinite(r.dt) || r.dt<=0 || r.dt>config_.maxDt || c.frameId<=p.frameId) return fail("TIME_GAP");
    if(c.descriptors.empty() || p.descriptors.empty()) return fail("NO_MATCH");
    if(c.descriptors.type()!=CV_8UC1 || p.descriptors.type()!=CV_8UC1 || c.descriptors.cols!=32 || p.descriptors.cols!=32) return fail("INVALID_DESCRIPTOR");
    if(c.descriptors.rows<2 || p.descriptors.rows<2) return fail("NO_MATCH");
    EpipolarResult shared;
    if(pair) shared=pair->matching;
    else { DiagnosticMatcher matcher; matcher.Process(p,config_.ratio,config_.maxDt); shared=matcher.Process(c,config_.ratio,config_.maxDt).matching; }
    r=shared;
    if(r.reason!="OK") return r;
    std::vector<std::pair<int,int>> matches;
    for(size_t j=0;j<r.features.size();++j) if(r.features[j].previousIndex>=0) matches.emplace_back(r.features[j].previousIndex,j);
    auto start=Clock::now();
    std::vector<cv::Point2f> pp,qq; std::vector<size_t> backgroundToMatches;
    for(size_t k=0;k<matches.size();++k) {
        int i=matches[k].first,j=matches[k].second;
        if(p.instances[i]==0 && c.instances[j]==0 && Finite(p.undistortedKeys[i].pt) && Finite(c.undistortedKeys[j].pt)) {
            pp.push_back(p.undistortedKeys[i].pt); qq.push_back(c.undistortedKeys[j].pt); backgroundToMatches.push_back(k);
        }
    }
    r.background=pp.size(); if(r.background<config_.minBackground) return fail("INSUFFICIENT_BACKGROUND");
    start=Clock::now();
    // OpenCV RANSAC owns its local sampler. Do not seed/alter cv::theRNG().
    cv::Mat mask;
    try {r.F=cv::findFundamentalMat(pp,qq,cv::FM_RANSAC,config_.ransacPx,config_.confidence,mask);}
    catch(const cv::Exception&) {r.estimateMs=Ms(start); return fail("MODEL_ESTIMATION_FAILED");}
    if(r.F.rows!=3 || r.F.cols!=3 || mask.total()!=pp.size()) {r.F.release(); r.estimateMs=Ms(start); return fail("MODEL_ESTIMATION_FAILED");}
    r.F.convertTo(r.F,CV_64F);
    if(!cv::checkRange(r.F) || cv::norm(r.F)<1e-15) {r.estimateMs=Ms(start); return fail("MODEL_UNRELIABLE");}
    std::vector<cv::Point2f> inP,inQ; std::vector<double> displacement;
    for(size_t k=0;k<pp.size();++k) {
        auto match=matches[backgroundToMatches[k]];
        bool inside=mask.ptr<uchar>()[k]!=0; r.features[match.second].backgroundInlier=inside?1:0;
        if(inside) {inP.push_back(pp[k]); inQ.push_back(qq[k]); displacement.push_back(cv::norm(c.rawKeys[match.second].pt-p.rawKeys[match.first].pt));}
    }
    r.inliers=inP.size(); r.inlierRatio=double(r.inliers)/r.background;
    r.coverage=std::min(Coverage(inP,c.size),Coverage(inQ,c.size));
    std::string invalid;
    if(r.inliers<config_.minInliers || r.inlierRatio<config_.minInlierRatio) invalid="MODEL_UNRELIABLE";
    else if(r.coverage<config_.minCoverage) invalid="INSUFFICIENT_COVERAGE";
    if(!displacement.empty()) {std::sort(displacement.begin(),displacement.end()); r.medianDisplacement=displacement[displacement.size()/2];}
    if(invalid.empty() && r.medianDisplacement<config_.minDisplacementPx) invalid="LOW_DISPLACEMENT_RISK";
    if(invalid.empty()) {
        cv::Mat hm,H;
        try {H=cv::findHomography(pp,qq,cv::RANSAC,config_.ransacPx,hm,2000,config_.confidence);}
        catch(const cv::Exception&) {invalid="DEGENERACY_CHECK_FAILED";}
        if(!H.empty() && cv::checkRange(H) && hm.total()==pp.size()) r.homographySupport=double(cv::countNonZero(hm))/pp.size();
        else invalid="DEGENERACY_CHECK_FAILED";
        if(r.homographySupport>=config_.homographyRatio) invalid="HOMOGRAPHY_DOMINANCE_RISK";
    }
    r.estimateMs=Ms(start); if(!invalid.empty()) return fail(invalid);
    r.modelValid=true; r.reason="OK"; start=Clock::now();
    for(auto m:matches) {
        auto& f=r.features[m.second];
        // Compute every finite match; invalid associations remain UNKNOWN even if residual is available.
        f.residual=Residual(r.F,p.undistortedKeys[m.first].pt,c.undistortedKeys[m.second].pt);
        if(p.instances[m.first]<0 || c.instances[m.second]<0) {f.reason="INVALID_ASSOCIATION"; continue;}
        if(!std::isfinite(f.residual)) {f.reason="INVALID_RESIDUAL"; continue;}
        if(f.residual<=config_.lowPx) {f.state=EpipolarState::GEOMETRY_CONSISTENT; f.reason="LOW_RESIDUAL";}
        else if(f.residual>=config_.highPx) {f.state=EpipolarState::SUSPECTED_DYNAMIC; f.reason="HIGH_RESIDUAL";}
        else f.reason="AMBIGUOUS_RESIDUAL";
    }
    r.residualMs=Ms(start); return r;
}
EpipolarResult EpipolarMotionDetector::Process(const EpipolarInput& c,const cv::Mat& image,bool rgb,const DiagnosticPair* pair) {
    if(pair) { previous_=pair->previous; havePrevious_=pair->matching.previousFrameId>=0; }
    Require(c.instances.size()==c.rawKeys.size() && c.undistortedKeys.size()==c.rawKeys.size(),"feature/association/undistorted arrays must have identical N");
    Require(c.descriptors.empty() || c.descriptors.rows==int(c.rawKeys.size()),"descriptor rows must equal N");
    Require(c.size.width>0 && c.size.height>0 && c.K.rows==3 && c.K.cols==3 && cv::checkRange(c.K),"invalid image size/calibration");
    EpipolarResult r=Evaluate(c,pair);
    if((config_.debug || config_.visualize) && sequence_%config_.every==0) Record(c,image,rgb,r);
    if(pair) { previous_=EpipolarInput(); havePrevious_=false; }
    else {
        previous_=c; previous_.descriptors=c.descriptors.clone(); previous_.K=c.K.clone(); previous_.distortion=c.distortion.clone();
        havePrevious_=true;
    }
    ++sequence_;
    if(config_.visualize && !image.empty()) previousImage_=Bgr(image,rgb);
    return r;
}
void EpipolarMotionDetector::Record(const EpipolarInput& c,const cv::Mat& image,bool rgb,const EpipolarResult& r) {
    int counts[3]={}; for(const auto& f:r.features) ++counts[int(f.state)];
    std::cout << "[Epipolar] frame=" << c.frameId << " model=" << r.modelValid << " reason=" << r.reason << " matches=" << r.matches << " background=" << r.background << " inliers=" << r.inliers << " UNKNOWN/CONSISTENT/SUSPECTED=" << counts[0] << '/' << counts[1] << '/' << counts[2] << " (geometry diagnostic; features not removed)" << std::endl;
    if(config_.debug) {
        frames_ << c.frameId << ',' << c.timestamp << ',' << r.previousFrameId << ',' << r.dt << ',' << r.features.size() << ',' << r.matches << ',' << r.background << ',' << r.inliers << ',' << r.inlierRatio << ',' << r.coverage << ',' << r.medianDisplacement << ',' << r.homographySupport << ',' << r.modelValid << ',' << r.reason << ',' << counts[0] << ',' << counts[1] << ',' << counts[2] << ',' << r.matchMs << ',' << r.estimateMs << ',' << r.residualMs << '\n';
        for(size_t j=0;j<r.features.size();++j) {
            const auto& f=r.features[j];
            features_ << c.frameId << ',' << c.timestamp << ',' << r.previousFrameId << ',' << j << ',' << f.previousIndex << ',' << c.instances[j] << ',' << f.previousInstance << ',' << f.residual << ',' << EpipolarStateName(f.state) << ',' << f.reason << ',' << f.backgroundInlier;
            if(f.previousIndex>=0) {
                const auto& raw=previous_.rawKeys[f.previousIndex].pt;
                const auto& un=previous_.undistortedKeys[f.previousIndex].pt;
                features_ << ',' << raw.x << ',' << raw.y << ',' << c.rawKeys[j].pt.x << ',' << c.rawKeys[j].pt.y
                          << ',' << un.x << ',' << un.y << ',' << c.undistortedKeys[j].pt.x << ',' << c.undistortedKeys[j].pt.y;
            } else {
                features_ << ",nan,nan," << c.rawKeys[j].pt.x << ',' << c.rawKeys[j].pt.y
                          << ",nan,nan," << c.undistortedKeys[j].pt.x << ',' << c.undistortedKeys[j].pt.y;
            }
            features_ << '\n';
        }
        // Persist this frame's actual model for independent residual/line auditing.
        // Debug only, at the same save frequency; no additional estimation or semantic decisions.
        cv::FileStorage model(config_.output+"/model_"+std::to_string(c.frameId)+".json",
                              cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
        Require(model.isOpened(),"cannot save model audit JSON");
        model << "frame_id" << double(c.frameId) << "previous_frame_id" << double(r.previousFrameId)
              << "model_valid" << int(r.modelValid) << "reason" << r.reason
              << "direction" << "x_current^T F x_previous = 0"
              << "coordinate_space" << "undistorted_pixels"
              << "width" << c.size.width << "height" << c.size.height
              << "K" << c.K << "distortion" << c.distortion << "F" << r.F;
        model.release();
        frames_.flush(); features_.flush(); Require(frames_.good() && features_.good(),"CSV write failed");
    }
    if(config_.visualize && !image.empty()) {
        cv::Mat current=Bgr(image,rgb), canvas;
        bool pair=config_.drawMatches && !previousImage_.empty() && previousImage_.size()==image.size() && r.previousFrameId>=0;
        if(pair) cv::hconcat(previousImage_,current,canvas); else canvas=current;
        int offset=pair?image.cols:0;
        for(size_t j=0;j<r.features.size();++j) {
            if(!Finite(c.rawKeys[j].pt)) continue;
            const auto& f=r.features[j]; cv::Scalar color=f.state==EpipolarState::GEOMETRY_CONSISTENT?cv::Scalar(0,255,0):f.state==EpipolarState::SUSPECTED_DYNAMIC?cv::Scalar(0,0,255):cv::Scalar(0,255,255);
            cv::Point2f q=c.rawKeys[j].pt+cv::Point2f(offset,0); cv::circle(canvas,q,2,color,1);
            if(pair && f.previousIndex>=0 && Finite(previous_.rawKeys[f.previousIndex].pt)) cv::line(canvas,previous_.rawKeys[f.previousIndex].pt,q,color,1);
        }
        cv::copyMakeBorder(canvas,canvas,50,0,0,0,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
        cv::putText(canvas,"Geometry diagnostic; features NOT removed",{5,18},cv::FONT_HERSHEY_SIMPLEX,0.45,{255,255,255},1);
        cv::putText(canvas,r.reason+" bg="+std::to_string(r.background)+" inliers="+std::to_string(r.inliers),{5,38},cv::FONT_HERSHEY_SIMPLEX,0.45,{255,255,255},1);
        Require(cv::imwrite(config_.output+"/epipolar_"+std::to_string(c.frameId)+".png",canvas),"image write failed");
    }
}
}
