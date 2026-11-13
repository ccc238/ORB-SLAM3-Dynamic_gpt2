#include "EpipolarMotionDetector.h"
#include "SemanticMasker.h"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <stdexcept>
using namespace ORB_SLAM3;
void Check(bool b,const char* m) {if(!b) throw std::runtime_error(m);}
EpipolarConfig Config(const std::string& text,bool seg=true,bool assoc=true,bool mask=false) {
    cv::FileStorage fs("%YAML:1.0\n"+text,cv::FileStorage::READ|cv::FileStorage::MEMORY);
    return EpipolarConfig::Read(fs,seg,assoc,mask);
}
EpipolarInput Input(int frame,bool planar=false) {
    EpipolarInput a; a.frameId=frame; a.timestamp=frame/30.; a.size={640,480};
    a.K=(cv::Mat_<double>(3,3)<<500,0,320,0,500,240,0,0,1);
    a.distortion=cv::Mat::zeros(4,1,CV_64F);
    cv::RNG rng(77); a.descriptors=cv::Mat(120,32,CV_8U); rng.fill(a.descriptors,cv::RNG::UNIFORM,0,256);
    for(int i=0;i<120;++i) {
        double z=planar?4:rng.uniform(2.,8.); double u=rng.uniform(45.,570.),v=rng.uniform(35.,440.);
        if(frame>0) u+=500*0.06/z;
        if(i>=100 && frame>0) v+=16; // instance motion perpendicular to epipolar line
        a.rawKeys.emplace_back(cv::Point2f(u,v),1); a.undistortedKeys.push_back(a.rawKeys.back()); a.instances.push_back(i<100?0:1);
    }
    return a;
}
void Unknown(const EpipolarResult& r,size_t n) {Check(r.features.size()==n,"N mismatch"); for(auto f:r.features) Check(f.state==EpipolarState::UNKNOWN && std::isnan(f.residual),"expected UNKNOWN/NaN");}
int main() {
 try {
    Check(!Config("").enabled && !Config("Dynamic.UseEpipolar: 1\n").enabled,"default/off parsing");
    Check(Config("Dynamic.UseEpipolar: 0\n").enabled,"reverse convention");
    for(auto s:{"2","-1","0.0","\"0\"","[]"}) {bool threw=false; try {Config(std::string("Dynamic.UseEpipolar: ")+s+"\n");} catch(...) {threw=true;} Check(threw,"illegal config accepted");}
    for(int mode=0;mode<3;++mode) {bool threw=false;try {Config("Dynamic.UseEpipolar: 0\n",mode!=0,mode!=1,mode==2);} catch(...) {threw=true;}Check(threw,"prerequisite not enforced");}
    for(auto text:{"Epipolar.LowPx: 3\nEpipolar.HighPx: 3\n","Epipolar.SaveEvery: 0\n","Epipolar.MatchRatio: 1.1\n"}) {bool threw=false;try {Config(std::string("Dynamic.UseEpipolar: 0\n")+text);}catch(...) {threw=true;}Check(threw,"invalid parameters accepted");}
    // A nonsymmetric F from t=(.2,.1,.05), R=Rz(.1); verify direction independently.
    cv::Matx33d R(std::cos(.1),-std::sin(.1),0,std::sin(.1),std::cos(.1),0,0,0,1);
    cv::Matx33d T(0,-.05,.1,.05,0,-.2,-.1,.2,0), K(500,0,320,0,500,240,0,0,1);
    cv::Mat F=cv::Mat(K.inv().t()*T*R*K.inv()).clone();
    cv::Vec3d X(.3,.2,4),Y=R*X+cv::Vec3d(.2,.1,.05), p=K*X,q=K*Y;
    cv::Point2d a(p[0]/p[2],p[1]/p[2]),b(q[0]/q[2],q[1]/q[2]);
    Check(EpipolarMotionDetector::Residual(F,a,b)<1e-10,"F direction/formula");
    Check(EpipolarMotionDetector::Residual(F,b,a)>0.1,"direction test not discriminating");
    cv::Mat previousHomogeneous=(cv::Mat_<double>(3,1)<<a.x,a.y,1);
    cv::Mat line=F*previousHomogeneous; double nx=line.at<double>(0),ny=line.at<double>(1),n=std::hypot(nx,ny);
    cv::Point2d moved=b+cv::Point2d(8*nx/n,8*ny/n);
    double residual=EpipolarMotionDetector::Residual(F,a,moved);
    Check(residual>3,"perpendicular perturbation did not grow");
    cv::Mat homogeneousA=(cv::Mat_<double>(3,1)<<a.x,a.y,1), homogeneousB=(cv::Mat_<double>(3,1)<<moved.x,moved.y,1);
    double squared=cv::sampsonDistance(homogeneousA,homogeneousB,F);
    Check(std::abs(residual-std::sqrt(squared))<1e-9,"sqrt Sampson mismatch");
    Check(std::isnan(EpipolarMotionDetector::Residual(cv::Mat::zeros(3,3,CV_64F),a,b)),"zero denominator");
    auto cfg=Config("Dynamic.UseEpipolar: 0\n"); EpipolarMotionDetector d(cfg);
    auto first=Input(0), second=Input(1); auto original=second.descriptors.clone(); auto labels=second.instances;
    auto raw=second.rawKeys; auto undist=second.undistortedKeys;
    Unknown(d.Process(first),120); auto rngState=cv::theRNG().state; auto result=d.Process(second);
    Check(cv::theRNG().state==rngState,"global RNG modified");
    Check(result.modelValid && result.matches==120 && result.background==100,"synthetic model failed");
    int consistent=0,suspected=0;
    for(int i=0;i<120;++i) {auto f=result.features[i];Check(f.previousIndex==i,"index mapping"); if(f.state==EpipolarState::GEOMETRY_CONSISTENT)++consistent; if(f.state==EpipolarState::SUSPECTED_DYNAMIC)++suspected;Check(i<100?f.backgroundInlier>=0:f.backgroundInlier==-1,"background mapping");}
    Check(consistent>=95 && suspected==20,"classification mismatch");
    Check(cv::norm(second.descriptors,original)==0 && second.instances==labels,"input mutated");
    for(size_t i=0;i<raw.size();++i) Check(raw[i].pt==second.rawKeys[i].pt && undist[i].pt==second.undistortedKeys[i].pt,"keys mutated");
    // IDs are local to each frame; they must never constrain descriptor matching.
    d.Reset(); auto differentIds=Input(0); for(int i=100;i<120;++i) differentIds.instances[i]=42;
    d.Process(differentIds); auto idResult=d.Process(Input(1));
    Check(idResult.matches==120 && idResult.features[110].previousInstance==42 && idResult.features[110].state==EpipolarState::SUSPECTED_DYNAMIC,"cross-frame ID assumption");
    // Distort only raw coordinates, retaining their original indices and the known undistorted pixels.
    d.Reset(); auto distortedPrevious=Input(0),distortedCurrent=Input(1);
    for(auto input:{&distortedPrevious,&distortedCurrent}) {
        input->distortion=(cv::Mat_<double>(4,1)<<0.12,-0.03,0.001,-0.002);
        std::vector<cv::Point3f> rays; for(auto key:input->undistortedKeys) rays.emplace_back((key.pt.x-320)/500,(key.pt.y-240)/500,1);
        std::vector<cv::Point2f> projected; cv::projectPoints(rays,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),input->K,input->distortion,projected);
        for(size_t i=0;i<projected.size();++i) input->rawKeys[i].pt=projected[i];
    }
    d.Process(distortedPrevious);auto distortedResult=d.Process(distortedCurrent);
    Check(distortedResult.modelValid,"distorted raw input model");
    for(int i=0;i<120;++i) Check(std::abs(distortedResult.features[i].residual-result.features[i].residual)<1e-9 && distortedResult.features[i].state==result.features[i].state,"distorted and undistorted coordinates mixed");
    d.Reset(); Unknown(d.Process(second),120);
    // Deep ownership: modifying caller's previous descriptors must not change next match.
    d.Reset(); d.Process(first); first.descriptors.setTo(0); Check(d.Process(second).matches==120,"cache not deep owned");
    d.Reset(); d.Process(Input(0)); auto empty=Input(1); empty.descriptors.release(); Unknown(d.Process(empty),120);
    d.Reset(); d.Process(Input(0)); auto few=Input(1); std::fill(few.instances.begin(),few.instances.end(),1); auto shortResult=d.Process(few);Unknown(shortResult,120);Check(shortResult.reason=="INSUFFICIENT_BACKGROUND","background fallback");
    d.Reset(); auto flat=Input(0); for(auto& k:flat.undistortedKeys) k.pt={100,100}; d.Process(flat);flat.frameId=1;flat.timestamp=1/30.;auto failed=d.Process(flat);Unknown(failed,120);Check(!failed.modelValid,"degenerate model accepted");
    d.Reset(); d.Process(Input(0,true)); auto planar=d.Process(Input(1,true)); Unknown(planar,120);Check(!planar.modelValid,"planar risk accepted");
    d.Reset(); d.Process(Input(0)); auto ambiguous=Input(1);
    for(int i=100;i<120;++i) {ambiguous.rawKeys[i].pt.y-=13.5; ambiguous.undistortedKeys[i]=ambiguous.rawKeys[i];}
    auto middle=d.Process(ambiguous); Check(middle.modelValid,"ambiguous model");
    for(int i=100;i<120;++i) Check(middle.features[i].state==EpipolarState::UNKNOWN && middle.features[i].reason=="AMBIGUOUS_RESIDUAL" && std::isfinite(middle.features[i].residual),"ambiguous residual");
    d.Reset(); d.Process(Input(0)); auto invalidLabel=Input(1); invalidLabel.instances[110]=-1;
    auto invalidResult=d.Process(invalidLabel); Check(invalidResult.features[110].state==EpipolarState::UNKNOWN && invalidResult.features[110].reason=="INVALID_ASSOCIATION","invalid association classified");
    d.Reset(); d.Process(Input(0)); auto badDescriptors=Input(1);badDescriptors.descriptors.convertTo(badDescriptors.descriptors,CV_32F);
    Unknown(d.Process(badDescriptors),120);
    d.Reset(); d.Process(Input(0));auto duplicates=Input(1);duplicates.descriptors.setTo(0);Unknown(d.Process(duplicates),120);
    d.Reset();auto noFeatures=Input(0);noFeatures.rawKeys.clear();noFeatures.undistortedKeys.clear();noFeatures.instances.clear();noFeatures.descriptors.release();Unknown(d.Process(noFeatures),0);
    d.Reset(); auto rotate=Input(0);d.Process(rotate);rotate.frameId=1;rotate.timestamp=1/30.;
    for(size_t i=0;i<rotate.rawKeys.size();++i) {cv::Point2f q=rotate.rawKeys[i].pt;cv::Vec3d v=K*R*K.inv()*cv::Vec3d(q.x,q.y,1);rotate.rawKeys[i].pt=cv::Point2f(v[0]/v[2],v[1]/v[2]);rotate.undistortedKeys[i]=rotate.rawKeys[i];}
    auto rotation=d.Process(rotate);Unknown(rotation,120);Check(!rotation.modelValid,"pure rotation risk accepted");
    d.Reset(); auto still=Input(0);d.Process(still);still.frameId=1;still.timestamp=1/30.;auto low=d.Process(still);Unknown(low,120);
    d.Reset();d.Process(Input(0));auto gap=Input(1);gap.timestamp=2;Check(d.Process(gap).reason=="TIME_GAP","time gap");
    auto next=gap; next.frameId=2;next.timestamp+=1/30.;Check(d.Process(next).previousFrameId==1,"cache recovery");
    d.Reset();d.Process(Input(0));auto resized=Input(1);resized.size={320,240};Unknown(d.Process(resized),120);
    d.Reset();d.Process(Input(0));auto calibration=Input(1);calibration.K.at<double>(0,0)=510;Check(d.Process(calibration).reason=="INCOMPATIBLE_INPUT","calibration reset");
    std::cout << "PASS configuration, prerequisites, F direction, sqrt Sampson, perturbation, model, mapping, per-frame IDs, distorted/raw coordinate separation, tri-state, ownership, immutability, RNG, first/empty/insufficient/degenerate/planar/reset/time/calibration/size\n";
    return 0;
 } catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
