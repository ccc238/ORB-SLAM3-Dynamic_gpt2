#include "Residual3D.h"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdexcept>
#include <cmath>
using namespace ORB_SLAM3;
void Check(bool b,const char* why) {if(!b) throw std::runtime_error(why);}
Residual3DConfig Config(const std::string& s,bool seg=true,bool assoc=true,bool mask=false) {
    cv::FileStorage fs("%YAML:1.0\n"+s,cv::FileStorage::READ|cv::FileStorage::MEMORY);return Residual3DConfig::Read(fs,seg,assoc,mask);
}
EpipolarInput Input(int frame) {
    EpipolarInput a;a.frameId=frame;a.timestamp=frame/30.;a.size={640,480};a.K=(cv::Mat_<double>(3,3)<<500,0,320,0,500,240,0,0,1);a.distortion=cv::Mat::zeros(4,1,CV_64F);
    cv::Mat R,rv=(cv::Mat_<double>(3,1)<<.01,-.02,.015);cv::Rodrigues(rv,R);
    cv::Mat t=(cv::Mat_<double>(3,1)<<.04,-.02,.03);
    cv::RNG rng(18);a.descriptors=cv::Mat(120,32,CV_8U);rng.fill(a.descriptors,cv::RNG::UNIFORM,0,256);
    for(int i=0;i<120;++i) {
        double z=2.+(i%7)*.27,x=((i%12)-5.5)*.18,y=((i/12)-4.5)*.15;
        cv::Mat q=(cv::Mat_<double>(3,1)<<x,y,z);if(frame) {q=R*q+t;if(i>=100) q.at<double>(0)+=.12;}
        double depth=q.at<double>(2);cv::Point2f uv(500*q.at<double>(0)/depth+320,500*q.at<double>(1)/depth+240);
        a.rawKeys.emplace_back(uv,1);a.undistortedKeys.emplace_back(uv,1);a.depths.push_back(depth);a.depthReasons.push_back("OK");a.instances.push_back(i<100?0:1);
    }return a;
}
int main() {try {
    Check(!Config("").enabled&&!Config("Dynamic.Use3DResidual: 1\n").enabled,"off/missing");auto cfg=Config("Dynamic.Use3DResidual: 0\nDynamic.DepthPoseSource: background_pnp\n");Check(cfg.enabled,"0 on");
    for(auto s:{"2","-1","0.0","\"0\"","[]","{}"}) {bool threw=false;try {Config(std::string("Dynamic.Use3DResidual: ")+s+"\n");}catch(...) {threw=true;}Check(threw,"illegal switch accepted");}
    for(int i=0;i<3;++i) {bool threw=false;try {Config("Dynamic.Use3DResidual: 0\n",i!=0,i!=1,i==2);}catch(...) {threw=true;}Check(threw,"prerequisite");}
    {
        auto trackingCfg=Config("Dynamic.Use3DResidual: 0\n");Check(trackingCfg.poseSource=="tracking"&&trackingCfg.trackingMinInliers==25,"tracking default");
        for(auto value:{"bad","0","[]","background_PnP"}) {
            bool rejected=false;try {Config(std::string("Dynamic.Use3DResidual: 0\nDynamic.DepthPoseSource: ")+value+"\n");}catch(...) {rejected=true;}
            Check(rejected,"invalid pose source accepted");
        }
        auto p=Input(0),c=Input(1);DiagnosticMatcher dm;dm.Process(p,.75,.2);auto pair=dm.Process(c,.75,.2);
        TrackingDepthPose previous,current;
        previous.frameId=0;previous.timestamp=0;previous.mapId=7;previous.mapChange=3;previous.bigChange=2;previous.relocalizationFrame=0;
        previous.inliers=80;previous.hasPose=true;previous.trackingOK=true;
        cv::Mat baseR,baseRv=(cv::Mat_<double>(3,1)<<.2,-.1,.3);cv::Rodrigues(baseRv,baseR);
        for(int y=0;y<3;++y) for(int x=0;x<3;++x) previous.Tcw(y,x)=baseR.at<double>(y,x);
        previous.Tcw(0,3)=1.2;previous.Tcw(1,3)=-.6;previous.Tcw(2,3)=.9;
        cv::Mat relR,relRv=(cv::Mat_<double>(3,1)<<.01,-.02,.015);cv::Rodrigues(relRv,relR);
        cv::Matx44d expected=cv::Matx44d::eye();
        for(int y=0;y<3;++y) for(int x=0;x<3;++x) expected(y,x)=relR.at<double>(y,x);
        expected(0,3)=.04;expected(1,3)=-.02;expected(2,3)=.03;
        current=previous;current.frameId=1;current.timestamp=1/30.;current.inliers=25;current.Tcw=expected*previous.Tcw;
        auto motion=Residual3DDetector::TrackingMotion(c,pair,previous,current,trackingCfg);
        Check(motion.motionValid&&cv::norm(motion.T_current_previous,cv::Mat(expected))<1e-12,"Tcw composition/inverse/metres");
        Residual3DDetector tracking(trackingCfg);auto result=tracking.Process(c,pair,nullptr,cv::Mat(),false,&motion);
        Check(result.motionValid&&result.background==0&&result.inliers==0,"Tracking used PnP statistics");
        double maximum=0;for(int i=0;i<100;++i) maximum=std::max(maximum,result.features[i].residual);
        Check(maximum<1e-5&&std::abs(result.features[110].residual-.12)<1e-5,"Tracking static compensation");
        Check(!tracking.Process(c,pair).motionValid,"missing Tracking auto-fell-back to PnP");
        auto invalid=[&](TrackingDepthPose a,TrackingDepthPose b,const char* why) {
            auto m=Residual3DDetector::TrackingMotion(c,pair,a,b,trackingCfg);Check(!m.motionValid,why);
            auto r=tracking.Process(c,pair,nullptr,cv::Mat(),false,&m);for(auto f:r.features) Check(f.state==EpipolarState::UNKNOWN&&std::isnan(f.residual),"invalid Tracking residual");
        };
        auto bad=current;bad.inliers=24;invalid(previous,bad,"25 threshold");
        bad=current;bad.inliers=-1;invalid(previous,bad,"stale/unavailable inliers");
        bad=current;bad.trackingOK=false;invalid(previous,bad,"failed Tracking");
        bad=previous;bad.trackingOK=false;invalid(bad,current,"previous failed Tracking");
        bad=current;bad.hasPose=false;invalid(previous,bad,"missing pose");
        bad=current;bad.Tcw(0,0)=NAN;invalid(previous,bad,"nonfinite pose");
        bad=current;bad.mapId=8;invalid(previous,bad,"map switch");
        bad=current;bad.mapChange++;invalid(previous,bad,"map coordinate revision");
        bad=current;bad.bigChange++;invalid(previous,bad,"big map change");
        bad=current;bad.relocalizationFrame++;invalid(previous,bad,"relocalization");
        bad=current;bad.frameId=0;bad.timestamp=0;invalid(previous,bad,"same-frame alias");
        invalid(TrackingDepthPose(),current,"cleared/reset cache");
        dm.Reset();auto firstPair=dm.Process(c,.75,.2);auto firstMotion=Residual3DDetector::TrackingMotion(c,firstPair,previous,current,trackingCfg);
        Check(!firstMotion.motionValid&&firstMotion.reason=="NO_PREVIOUS_FRAME","first Tracking frame");
        std::cout<<"PASS Tracking Tcw composition, nonidentity direction, metres, static max_m="<<maximum<<", invalid branches and no fallback\n";
    }
    {
        TrackingDepthPose previous,current;
        previous.frameId=0;previous.hasPose=true;previous.trackingOK=true;previous.mapId=3;previous.mapChange=2;
        previous.bigChange=0;previous.relocalizationFrame=0;previous.referenceId=5;
        previous.frameToReference(0,3)=.3;previous.originalTcw=previous.Tcw;
        current=previous;current.frameId=1;current.mapChange=3;
        cv::Matx44d ref=cv::Matx44d::eye();ref(0,0)=0;ref(0,1)=-1;ref(1,0)=1;ref(1,1)=0;ref(1,3)=.7;
        Check(Residual3DDetector::RebasePreviousPose(previous,current,ref,5,3,true),"reference refresh rejected");
        Check(previous.rebased&&previous.mapChange==3&&std::abs(previous.Tcw(0,3)-.3)<1e-12&&std::abs(previous.Tcw(1,3)-.7)<1e-12,"reference composition order");
        auto old=previous;current.bigChange=1;
        Check(!Residual3DDetector::RebasePreviousPose(previous,current,ref,5,3,true)&&previous.Tcw==old.Tcw,"big change silently rebased");
        current.bigChange=0;Check(!Residual3DDetector::RebasePreviousPose(previous,current,ref,5,3,false),"bad reference accepted");
        auto gateCfg=cfg;gateCfg.useDepthDifferenceGate=true;gateCfg.maxDepthDifferenceM=.20;
        auto p=Input(0),c=Input(1);c.depths[110]+=1;
        DiagnosticMatcher matcher;matcher.Process(p,.75,.2);auto pair=matcher.Process(c,.75,.2);
        Residual3DDetector withGate(gateCfg),withoutGate(cfg);
        auto filtered=withGate.Process(c,pair);auto original=withoutGate.Process(c,pair);
        Check(filtered.motionValid&&filtered.features[110].depthDifferenceRejected,"cross-depth gate not applied");
        Check(filtered.features[110].state==EpipolarState::UNKNOWN&&std::isnan(filtered.features[110].residual)&&std::isfinite(filtered.features[110].rawResidual),"gate exclusion/raw audit");
        Check(!original.features[110].depthDifferenceRejected&&std::isfinite(original.features[110].residual),"disabled gate changed residual");
        std::cout<<"PASS map revision reference refresh, unsafe refresh rejection, depth difference exclusion and raw residual audit\n";
    }
    cv::Mat depth(20,20,CV_32F,cv::Scalar(2));auto check=[&](double z,cv::Point2f uv) {return Residual3DDetector::CheckDepth(z,uv,depth,cfg);};
    Check(check(2,{10,10}).valid,"valid depth");for(double z:{0.,-1.,double(NAN),double(INFINITY),20.}) Check(!check(z,{10,10}).valid,"invalid depth");
    Check(!check(2,{-1,10}).valid&&!check(2,{0,0}).valid,"bounds");depth.at<float>(10,11)=3;Check(check(2,{10,10}).reason=="DEPTH_DISCONTINUITY","edge depth");depth.at<float>(10,11)=NAN;Check(!check(2,{10,10}).valid,"invalid neighbor");
    auto p=Input(0),c=Input(1);DiagnosticMatcher matcher;Residual3DDetector detector(cfg);
    auto first=detector.Process(p,matcher.Process(p,cfg.ratio,cfg.maxDt));Check(!first.motionValid&&std::isnan(first.features[0].residual),"first frame");
    auto pair=matcher.Process(c,cfg.ratio,cfg.maxDt);auto r=detector.Process(c,pair);Check(r.motionValid,"synthetic PnP rejected");
    double maxStatic=0,maxMoveError=0;
    for(int i=0;i<120;++i) {auto f=r.features[i];Check(f.previousIndex==i,"indices");Check(f.backgroundInlier==(i<100?1:-1),"instance entered fit");if(i<100) maxStatic=std::max(maxStatic,f.residual);else maxMoveError=std::max(maxMoveError,std::abs(f.residual-.12));}
    Check(maxStatic<1e-5&&maxMoveError<1e-5,"transform direction/metres/static/moved residual");
    auto rawDifferent=c;
    for(auto& k:rawDifferent.rawKeys) k.pt+=cv::Point2f(19,-8);
    auto unResult=detector.Process(rawDifferent,pair);
    Check(unResult.motionValid&&std::abs(unResult.features[110].residual-r.features[110].residual)<1e-12,"raw distorted pixels used for 3D");
    auto middle=c;for(int i=100;i<120;++i) middle.undistortedKeys[i].pt.x-=float(500*.07/middle.depths[i]);
    auto middleResult=detector.Process(middle,pair);
    Check(middleResult.features[110].state==EpipolarState::UNKNOWN&&std::abs(middleResult.features[110].residual-.05)<1e-5,"middle threshold");
    auto pBefore=p.descriptors.clone(),cBefore=c.descriptors.clone();
    detector.Process(c,pair);
    Check(cv::norm(pBefore,p.descriptors,cv::NORM_INF)==0&&cv::norm(cBefore,c.descriptors,cv::NORM_INF)==0&&c.rawKeys.size()==120&&c.instances.size()==120,"input mutated");
    c.depthReasons[105]="INVALID_DEPTH";c.depths[105]=NAN;r=detector.Process(c,pair);Check(r.features[105].state==EpipolarState::UNKNOWN&&std::isnan(r.features[105].residual),"missing current depth");
    c.depthReasons[106]="DEPTH_DISCONTINUITY";r=detector.Process(c,pair);Check(std::isnan(r.features[106].residual),"edge UNKNOWN");
    auto few=c;std::fill(few.instances.begin(),few.instances.end(),1);r=detector.Process(few,pair);Check(!r.motionValid&&r.reason=="INSUFFICIENT_BACKGROUND","background fallback");
    auto flatPair=pair;for(auto& k:flatPair.previous.undistortedKeys) k.pt={320,240};r=detector.Process(c,flatPair);Check(!r.motionValid,"degenerate PnP");for(auto f:r.features) Check(f.state==EpipolarState::UNKNOWN&&std::isnan(f.residual),"invalid model residual");
    auto mismatched=Input(1);cv::RNG randomPixels(456);
    for(auto& k:mismatched.undistortedKeys) k.pt={randomPixels.uniform(0.f,640.f),randomPixels.uniform(0.f,480.f)};
    auto failedPnP=detector.Process(mismatched,pair);
    Check(!failedPnP.motionValid&&(failedPnP.reason=="PNP_FAILED"||failedPnP.reason=="INSUFFICIENT_PNP_SUPPORT"),"failed PnP accepted");
    for(auto f:failedPnP.features) Check(f.state==EpipolarState::UNKNOWN&&std::isnan(f.residual),"PnP failure reused transform");
    auto absentPrevious=pair;for(auto& reason:absentPrevious.previous.depthReasons) reason="INVALID_DEPTH";
    auto missingPrevious=detector.Process(c,absentPrevious);
    Check(!missingPrevious.motionValid&&missingPrevious.reason=="INSUFFICIENT_BACKGROUND","missing previous depth accepted");
    std::cout<<"PnP failure safety reason="<<failedPnP.reason<<'\n';
    cv::Mat badR=cv::Mat::eye(3,3,CV_64F),badT=(cv::Mat_<double>(3,1)<<0,0,-10);Check(std::isnan(Residual3DDetector::Difference({0,0,2},{0,0,2},badR,badT)[0]),"negative predicted depth");
    matcher.Reset();Check(matcher.Process(c,cfg.ratio,cfg.maxDt).matching.reason=="NO_PREVIOUS_FRAME","reset");
    matcher.Reset();matcher.Process(p,cfg.ratio,cfg.maxDt);auto gap=c;gap.timestamp=2;Check(matcher.Process(gap,cfg.ratio,cfg.maxDt).matching.reason=="TIME_GAP","time gap");
    matcher.Reset();matcher.Process(p,cfg.ratio,cfg.maxDt);p.descriptors.setTo(0);Check(matcher.Process(c,cfg.ratio,cfg.maxDt).matching.matches==120,"cache ownership");
    matcher.Reset();matcher.Process(Input(0),cfg.ratio,cfg.maxDt);auto resized=Input(1);resized.size={320,240};
    Check(matcher.Process(resized,cfg.ratio,cfg.maxDt).matching.reason=="INCOMPATIBLE_INPUT","resize compatibility");
    matcher.Reset();matcher.Process(Input(0),cfg.ratio,cfg.maxDt);auto changed=Input(1);changed.K.at<double>(0,0)=501;
    Check(matcher.Process(changed,cfg.ratio,cfg.maxDt).matching.reason=="INCOMPATIBLE_INPUT","calibration compatibility");
    matcher.Reset();matcher.Process(Input(0),cfg.ratio,cfg.maxDt);auto empty=Input(1);empty.descriptors.release();
    auto noMatch=detector.Process(empty,matcher.Process(empty,cfg.ratio,cfg.maxDt));
    Check(noMatch.reason=="NO_MATCH"&&!noMatch.motionValid&&std::isnan(noMatch.features[0].residual),"no match fallback");
    std::cout<<"PASS residual3d: max_static_m="<<maxStatic<<" max_displacement_error_m="<<maxMoveError<<" (epipolar detector absent)\n";
}catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;} }
