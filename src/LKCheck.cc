#include "LKCheck.h"
#include "SemanticMasker.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
namespace ORB_SLAM3 {
namespace {
const double NaN=std::numeric_limits<double>::quiet_NaN();
void Require(bool b,const std::string& s) {if(!b) throw std::runtime_error("LKCheck: "+s);}
int Integer(cv::FileStorage& fs,const char* key,int fallback) {
    auto node=fs[key];if(node.empty()) return fallback;
    Require(node.isInt(),std::string(key)+" must be integer");return int(node);
}
void Number(cv::FileStorage& fs,const char* key,double& value) {
    auto node=fs[key];if(node.empty()) return;
    Require(node.isInt()||node.isReal(),std::string(key)+" must be numeric");value=double(node);
    Require(std::isfinite(value),std::string(key)+" must be finite");
}
bool Finite(cv::Point2f p) {return std::isfinite(p.x)&&std::isfinite(p.y);}
bool Inside(cv::Point2f p,cv::Size s) {return Finite(p)&&p.x>=0&&p.y>=0&&p.x<s.width&&p.y<s.height;}
bool Equal(const cv::Mat& a,const cv::Mat& b) {
    return a.size()==b.size()&&a.type()==b.type()&&(a.empty()||cv::norm(a,b,cv::NORM_INF)<1e-9);
}
double Quantile(std::vector<double> values,double q) {
    if(values.empty()) return NaN;
    std::sort(values.begin(),values.end());return values[size_t(q*(values.size()-1))];
}
}
LKCheckConfig LKCheckConfig::Read(cv::FileStorage& fs) {
    LKCheckConfig c;c.configuredValue=Integer(fs,"Dynamic.UseLKCheck",1);
    Require(c.configuredValue==0||c.configuredValue==1,"Dynamic.UseLKCheck accepts integer 0 or 1 only");
    c.enabled=(c.configuredValue==0);
    std::cout<<"Dynamic.UseLKCheck="<<(fs["Dynamic.UseLKCheck"].empty()?"<missing>":std::to_string(c.configuredValue))
             <<" -> LK check "<<(c.enabled?"ENABLED":"DISABLED")<<" (0=enabled, 1=disabled; audit only)"<<std::endl;
    if(!c.enabled) return c;
    c.window=Integer(fs,"LKCheck.WinSize",c.window);c.maxLevel=Integer(fs,"LKCheck.MaxLevel",c.maxLevel);
    c.iterations=Integer(fs,"LKCheck.Iterations",c.iterations);c.saveEvery=Integer(fs,"LKCheck.SaveEvery",c.saveEvery);
    c.maxImages=Integer(fs,"LKCheck.MaxImagesPerFrame",c.maxImages);
    int visual=Integer(fs,"LKCheck.Visualize",0);Require(visual==0||visual==1,"Visualize accepts 0/1");c.visualize=(visual==1);
    Number(fs,"LKCheck.Epsilon",c.epsilon);Number(fs,"LKCheck.MinEigThreshold",c.minEig);
    Number(fs,"LKCheck.MaxFBErrorPx",c.maxFB);Number(fs,"LKCheck.MaxEndpointErrorPx",c.maxEndpoint);
    Number(fs,"LKCheck.MaxDt",c.maxDt);
    auto output=fs["LKCheck.OutputDirectory"];if(!output.empty()) {Require(output.isString(),"OutputDirectory must be string");output>>c.output;}
    Require(c.window>=3&&c.window%2==1&&c.maxLevel>=0&&c.maxLevel<=8&&c.iterations>0&&c.epsilon>0&&c.minEig>0&&c.maxFB>=0&&c.maxEndpoint>=0&&c.maxDt>0&&c.saveEvery>0&&c.maxImages>=0&&!c.output.empty(),"invalid LK parameters");
    std::cout<<"LKCheck output="<<c.output<<" window="<<c.window<<" levels="<<c.maxLevel
             <<" iterations="<<c.iterations<<" epsilon="<<c.epsilon<<" minEig="<<c.minEig
             <<" maxFB_px="<<c.maxFB<<" maxEndpoint_px="<<c.maxEndpoint<<" flags=0 (experimental starting parameters)"<<std::endl;
    return c;
}
const char* LKCheckStateName(LKCheckState s) {
    switch(s) {case LKCheckState::SUPPORTED:return "SUPPORTED";case LKCheckState::DISAGREES:return "DISAGREES";default:return "UNVERIFIABLE";}
}
LKCheckFeature::LKCheckFeature():p(NaN,NaN),qOrb(NaN,NaN),qLK(NaN,NaN),pBack(NaN,NaN),fbVector(NaN,NaN),endpointVector(NaN,NaN) {}
LKCheck::LKCheck(const LKCheckConfig& c,bool record):config_(c) {
    Require(c.enabled,"disabled LK must not allocate resources");
    if(!record) return;
    EnsureSemanticDirectory(c.output);
    features_.open(c.output+"/features.csv");frames_.open(c.output+"/frames.csv");
    Require(features_.good()&&frames_.good(),"cannot open output CSV");
    features_<<std::setprecision(17)<<"frame_id,previous_frame_id,timestamp,previous_timestamp,previous_feature_index,current_feature_index,p_x,p_y,q_orb_x,q_orb_y,q_lk_x,q_lk_y,p_back_x,p_back_y,forward_status,backward_status,fb_vector_x_px,fb_vector_y_px,endpoint_vector_x_px,endpoint_vector_y_px,fb_error_px,endpoint_error_px,lk_check_state,lk_check_reason,instance_id,previous_instance_id,epipolar_state,depth_state,epipolar_residual_px,residual_3d_m,raw_residual_3d_m\n";
    frames_<<std::setprecision(17)<<"frame_id,previous_frame_id,timestamp,previous_timestamp,candidates,unique_previous_points,forward_success,backward_success,supported,disagrees,unverifiable,fb_median_px,fb_p90_px,endpoint_median_px,endpoint_p90_px,compute_ms,record_ms,reason\n";
}
void LKCheck::Reset() {havePrevious_=false;previousId_=-1;previousGray_.release();previousK_.release();previousDistortion_.release();}
void LKCheck::Invalidate(LKCheckResult& r,const std::string& reason) {
    r.reason=reason;
    for(auto& f:r.features) {
        f.state=LKCheckState::UNVERIFIABLE;f.reason=reason;f.forwardStatus=f.backwardStatus=-1;
        f.qLK=f.pBack=cv::Point2f(NaN,NaN);f.fbVector=f.endpointVector=cv::Point2d(NaN,NaN);f.fbError=f.endpointError=NaN;
    }
}
LKCheckResult LKCheck::Process(const EpipolarInput& c,const DiagnosticPair* pair,const cv::Mat& gray) {
    auto start=std::chrono::steady_clock::now();LKCheckResult r;
    r.frameId=c.frameId;r.timestamp=c.timestamp;
    if(pair) {
        r.previousFrameId=pair->matching.previousFrameId;
        if(r.previousFrameId>=0) r.previousTimestamp=pair->previous.timestamp;
        for(size_t j=0;j<pair->matching.features.size();++j) {
            int i=pair->matching.features[j].previousIndex;if(i<0) continue;
            LKCheckFeature f;f.previousIndex=i;f.currentIndex=j;
            if(size_t(i)<pair->previous.rawKeys.size()) f.p=pair->previous.rawKeys[i].pt;
            if(j<c.rawKeys.size()) f.qOrb=c.rawKeys[j].pt;
            r.features.push_back(f);
        }
    }
    auto run=[&]() {
        if(gray.empty()||gray.type()!=CV_8UC1||gray.size()!=c.size) {Invalidate(r,"INVALID_GRAY_IMAGE");Reset();return;}
        // The two immutable owned matrices also outlive Reset during Track().
        r.currentGray=gray.clone();r.previousGray=previousGray_;
        std::string invalid;
        if(!pair) invalid="NO_MATCH_INPUT";
        else if(!havePrevious_) invalid="NO_PREVIOUS_FRAME";
        else if(r.previousFrameId!=previousId_||pair->previous.frameId!=size_t(previousId_)||pair->previous.timestamp!=previousTime_) invalid="FRAME_IMAGE_MISMATCH";
        else if(previousGray_.size()!=gray.size()||pair->previous.size!=gray.size()||!Equal(c.K,previousK_)||!Equal(c.distortion,previousDistortion_)) invalid="INCOMPATIBLE_INPUT";
        else if(!std::isfinite(c.timestamp)||!std::isfinite(previousTime_)||c.timestamp<=previousTime_||c.timestamp-previousTime_>config_.maxDt||c.frameId<=size_t(previousId_)) invalid="TIME_GAP";
        else if(pair->matching.reason!="OK") invalid=pair->matching.reason;
        else if(r.features.empty()) invalid="NO_MATCH";
        if(!invalid.empty()) Invalidate(r,invalid);
        else {
            r.reason="OK";
            std::map<int,size_t> unique;
            std::vector<cv::Point2f> points;
            for(auto& f:r.features) {
                if(!Finite(f.p)||!Finite(f.qOrb)) {f.reason="NONFINITE";continue;}
                if(!Inside(f.p,gray.size())||!Inside(f.qOrb,gray.size())) {f.reason="OUT_OF_BOUNDS";continue;}
                if(unique.emplace(f.previousIndex,points.size()).second) points.push_back(f.p);
            }
            r.uniquePoints=points.size();
            if(!points.empty()) {
                std::vector<cv::Point2f> forward,back,backStarts;
                std::vector<uchar> sf,sb;std::vector<float> ignoredError;
                std::vector<int> backIndex(points.size(),-1);
                cv::TermCriteria term(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,config_.iterations,config_.epsilon);
                // flags=0; neither direction is initialized by ORB or a pose prediction.
                // ignoredError is OpenCV's mean L1 patch intensity error, NOT displacement.
                cv::calcOpticalFlowPyrLK(previousGray_,r.currentGray,points,forward,sf,ignoredError,cv::Size(config_.window,config_.window),config_.maxLevel,term,0,config_.minEig);
                for(size_t k=0;k<points.size();++k) if(sf[k]&&Inside(forward[k],gray.size())) {backIndex[k]=backStarts.size();backStarts.push_back(forward[k]);}
                if(!backStarts.empty()) cv::calcOpticalFlowPyrLK(r.currentGray,previousGray_,backStarts,back,sb,ignoredError,cv::Size(config_.window,config_.window),config_.maxLevel,term,0,config_.minEig);
                for(auto& f:r.features) {
                    if(f.reason!="NOT_COMPUTED") continue;
                    size_t k=unique.at(f.previousIndex);f.forwardStatus=sf[k]?1:0;
                    if(!sf[k]) {f.reason="FORWARD_FAILED";continue;}
                    if(!Finite(forward[k])) {f.reason="NONFINITE";continue;}
                    if(!Inside(forward[k],gray.size())) {f.reason="OUT_OF_BOUNDS";continue;}
                    f.qLK=forward[k];int b=backIndex[k];f.backwardStatus=sb[b]?1:0;
                    f.endpointVector=cv::Point2d(f.qLK)-cv::Point2d(f.qOrb);f.endpointError=cv::norm(f.endpointVector);
                    if(!sb[b]) {f.reason="BACKWARD_FAILED";continue;}
                    if(!Finite(back[b])) {f.reason="NONFINITE";continue;}
                    if(!Inside(back[b],gray.size())) {f.reason="OUT_OF_BOUNDS";continue;}
                    f.pBack=back[b];f.fbVector=cv::Point2d(f.pBack)-cv::Point2d(f.p);f.fbError=cv::norm(f.fbVector);
                    if(f.fbError>config_.maxFB) {f.reason="FB_INCONSISTENT";continue;}
                    if(f.endpointError>config_.maxEndpoint) {f.state=LKCheckState::DISAGREES;f.reason="ENDPOINT_DISAGREEMENT";}
                    else {f.state=LKCheckState::SUPPORTED;f.reason="WITHIN_THRESHOLDS";}
                }
            }
        }
        previousGray_=r.currentGray;previousId_=c.frameId;previousTime_=c.timestamp;
        previousK_=c.K.clone();previousDistortion_=c.distortion.clone();havePrevious_=true;
    };
    try {run();} catch(const cv::Exception&) {Invalidate(r,"LK_EXCEPTION");Reset();}
    r.computeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return r;
}
void LKCheck::Record(const EpipolarInput& c,const DiagnosticPair* pair,const LKCheckResult& r,const EpipolarResult* epi,const Residual3DResult* depth) {
    if(!features_.is_open()) return;
    auto start=std::chrono::steady_clock::now();
    int nf=0,nb=0,ns=0,nd=0,nu=0,images=0;std::vector<double> fb,endpoint;
    auto isPriority=[&](const LKCheckFeature& f) {
        return (r.previousFrameId==6&&r.frameId==7&&f.previousIndex==32&&f.currentIndex==29)||
               (r.previousFrameId==40&&r.frameId==41&&f.previousIndex==142&&f.currentIndex==94);
    };
    bool pendingPriority=std::any_of(r.features.begin(),r.features.end(),isPriority);
    for(const auto& f:r.features) {
        nf+=f.forwardStatus==1&&Inside(f.qLK,c.size);nb+=f.backwardStatus==1&&Inside(f.pBack,c.size);
        ns+=f.state==LKCheckState::SUPPORTED;nd+=f.state==LKCheckState::DISAGREES;nu+=f.state==LKCheckState::UNVERIFIABLE;
        if(std::isfinite(f.fbError)) fb.push_back(f.fbError);
        if(std::isfinite(f.endpointError)) endpoint.push_back(f.endpointError);
        const EpipolarFeature* e=epi&&size_t(f.currentIndex)<epi->features.size()?&epi->features[f.currentIndex]:nullptr;
        const Residual3DFeature* d=depth&&size_t(f.currentIndex)<depth->features.size()?&depth->features[f.currentIndex]:nullptr;
        int instance=size_t(f.currentIndex)<c.instances.size()?c.instances[f.currentIndex]:-1;
        int previousInstance=pair&&size_t(f.previousIndex)<pair->previous.instances.size()?pair->previous.instances[f.previousIndex]:-1;
        features_<<r.frameId<<','<<r.previousFrameId<<','<<r.timestamp<<','<<r.previousTimestamp<<','<<f.previousIndex<<','<<f.currentIndex<<','
            <<f.p.x<<','<<f.p.y<<','<<f.qOrb.x<<','<<f.qOrb.y<<','<<f.qLK.x<<','<<f.qLK.y<<','<<f.pBack.x<<','<<f.pBack.y<<',';
        if(f.forwardStatus>=0) features_<<f.forwardStatus;
        features_<<',';
        if(f.backwardStatus>=0) features_<<f.backwardStatus;
        features_<<','<<f.fbVector.x<<','<<f.fbVector.y<<','<<f.endpointVector.x<<','<<f.endpointVector.y<<','<<f.fbError<<','<<f.endpointError<<','
            <<LKCheckStateName(f.state)<<','<<f.reason<<','<<instance<<','<<previousInstance<<',';
        if(e) features_<<EpipolarStateName(e->state);
        features_<<',';
        if(d) features_<<EpipolarStateName(d->state);
        features_<<',';
        if(e) features_<<e->residual;
        features_<<',';
        if(d) features_<<d->residual;
        features_<<',';
        if(d) features_<<d->rawResidual;
        features_<<'\n';
        bool priority=isPriority(f);
        if(config_.visualize&&images<config_.maxImages&&!r.previousGray.empty()&&!r.currentGray.empty()&&
           r.previousGray.size()==c.size&&r.currentGray.size()==c.size&&Finite(f.p)&&Finite(f.qOrb)&&
           (priority||!pendingPriority||images<config_.maxImages-1)&&
           (priority||(r.frameId%config_.saveEvery==0))) {
            cv::Mat view(c.size.height+110,c.size.width*2,CV_8UC3,cv::Scalar(20,20,20));
            cv::cvtColor(r.previousGray,view(cv::Rect(0,110,c.size.width,c.size.height)),cv::COLOR_GRAY2BGR);
            cv::cvtColor(r.currentGray,view(cv::Rect(c.size.width,110,c.size.width,c.size.height)),cv::COLOR_GRAY2BGR);
            auto mark=[&](cv::Point2f p,bool current,cv::Scalar color,const char* text) {if(!Inside(p,c.size)) return;
                cv::Point q(cvRound(p.x)+(current?c.size.width:0),cvRound(p.y)+110);
                if(std::string(text)=="ORB") cv::rectangle(view,q-cv::Point(7,7),q+cv::Point(7,7),color,2);
                else if(std::string(text)=="LK") cv::drawMarker(view,q,color,cv::MARKER_CROSS,9,2);
                else cv::circle(view,q,5,color,2);
                cv::putText(view,text,q+cv::Point(9,std::string(text)=="LK"?14:-7),cv::FONT_HERSHEY_SIMPLEX,.4,color,1);};
            mark(f.p,false,{255,255,255},"p");mark(f.qOrb,true,{0,165,255},"ORB");mark(f.qLK,true,{255,255,0},"LK");mark(f.pBack,false,{255,0,255},"back");
            std::ostringstream title;title<<r.previousFrameId<<" -> "<<r.frameId<<" features "<<f.previousIndex<<" -> "<<f.currentIndex<<" | "<<LKCheckStateName(f.state)<<" "<<f.reason;
            cv::putText(view,title.str(),{8,20},cv::FONT_HERSHEY_SIMPLEX,.45,{255,255,255},1);
            title.str("");title<<"FB="<<f.fbError<<" px endpoint="<<f.endpointError<<" px; epi="<<(e?EpipolarStateName(e->state):"OFF")<<" depth="<<(d?EpipolarStateName(d->state):"OFF");
            cv::putText(view,title.str(),{8,42},cv::FONT_HERSHEY_SIMPLEX,.42,{255,255,255},1);
            cv::putText(view,"White:p Orange:ORB Cyan:LK Magenta:back | Independent audit; NO FEATURES REMOVED",{8,64},cv::FONT_HERSHEY_SIMPLEX,.42,{255,255,255},1);
            cv::putText(view,"SUPPORTED is not ground truth; DISAGREES is not a dynamic label",{8,86},cv::FONT_HERSHEY_SIMPLEX,.42,{255,255,255},1);
            cv::imwrite(config_.output+"/frame_"+std::to_string(r.frameId)+"_feature_"+std::to_string(f.currentIndex)+".png",view);++images;
        }
        if(priority) pendingPriority=false;
    }
    double recordMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    frames_<<r.frameId<<','<<r.previousFrameId<<','<<r.timestamp<<','<<r.previousTimestamp<<','<<r.features.size()<<','<<r.uniquePoints<<','<<nf<<','<<nb<<','<<ns<<','<<nd<<','<<nu<<','
           <<Quantile(fb,.5)<<','<<Quantile(fb,.9)<<','<<Quantile(endpoint,.5)<<','<<Quantile(endpoint,.9)<<','<<r.computeMs<<','<<recordMs<<','<<r.reason<<'\n';
    frames_.flush();features_.flush();
}
}
