#include "LKCheck.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
using namespace ORB_SLAM3;
void Check(bool b,const char* s) {if(!b) throw std::runtime_error(s);}
LKCheckConfig Config(const std::string& text) {
    cv::FileStorage fs("%YAML:1.0\n"+text,cv::FileStorage::READ|cv::FileStorage::MEMORY);return LKCheckConfig::Read(fs);
}
EpipolarInput Input(int id,cv::Point2f offset={0,0}) {
    EpipolarInput x;x.frameId=id;x.timestamp=id/30.;x.size={640,480};x.K=cv::Mat::eye(3,3,CV_64F);x.distortion=cv::Mat::zeros(4,1,CV_64F);
    for(auto p:std::vector<cv::Point2f>{{140,140},{260,180},{420,280},{300,340}}) {x.rawKeys.emplace_back(p+offset,31);x.undistortedKeys.emplace_back(p+offset+cv::Point2f(20,10),31);x.instances.push_back(0);}
    return x;
}
DiagnosticPair Pair(const EpipolarInput& previous) {
    DiagnosticPair p;p.previous=previous;p.matching.previousFrameId=previous.frameId;p.matching.reason="OK";
    p.matching.features.resize(4);for(int i=0;i<4;++i) p.matching.features[i].previousIndex=i;return p;
}
int main() {try {
    Check(!Config("").enabled&&!Config("Dynamic.UseLKCheck: 1\n").enabled,"missing/1 disabled");
    auto cfg=Config("Dynamic.UseLKCheck: 0\n");Check(cfg.enabled,"0 enabled");
    for(auto v:{"2","-1","0.0","\"0\"","[]","{}","true"}) {
        bool threw=false;try {Config(std::string("Dynamic.UseLKCheck: ")+v+"\n");}catch(...) {threw=true;}Check(threw,"illegal switch accepted");
    }
    for(auto text:{"LKCheck.WinSize: 20","LKCheck.MaxLevel: -1","LKCheck.Epsilon: 0","LKCheck.MinEigThreshold: 0","LKCheck.MaxFBErrorPx: -1"}) {
        bool threw=false;try {Config(std::string("Dynamic.UseLKCheck: 0\n")+text+"\n");}catch(...) {threw=true;}Check(threw,"illegal parameter accepted");
    }
    cv::Mat image(480,640,CV_8U);cv::RNG rng(741);rng.fill(image,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(image,image,{5,5},.8);
    cv::Mat translated;cv::Mat affine=(cv::Mat_<double>(2,3)<<1,0,4,0,1,-3);cv::warpAffine(image,translated,affine,image.size());
    auto p=Input(0),c=Input(1,{4,-3});auto pair=Pair(p);LKCheck checker(cfg,false);
    Check(checker.Process(p,nullptr,image).reason=="NO_MATCH_INPUT","no hidden matcher");
    cv::Mat original=image.clone();auto result=checker.Process(c,&pair,translated);
    Check(result.features.size()==4&&result.uniquePoints==4,"batch candidates");
    for(auto f:result.features) {std::cout<<"translation feature="<<f.currentIndex<<" reason="<<f.reason<<" q="<<f.qLK<<" FB="<<f.fbError<<" endpoint="<<f.endpointError<<'\n';Check(f.state==LKCheckState::SUPPORTED,"known translation supported");Check(f.fbError<.1&&f.endpointError<.1,"translation numeric accuracy");}
    std::cout<<"Known translation (4,-3) px: max errors < 0.1 px\n";
    checker.Reset();checker.Process(p,nullptr,image);auto wrong=c;wrong.rawKeys[0].pt+=cv::Point2f(45,20);
    auto bad=checker.Process(wrong,&pair,translated);
    Check(bad.features[0].state==LKCheckState::DISAGREES&&bad.features[0].fbError<.1&&bad.features[0].endpointError>40,"independent endpoint disagreement");
    Check(cv::norm(image,original,cv::NORM_INF)==0,"input pixels unchanged");
    // Owned previous image survives caller overwrite; undistorted pixels deliberately differ.
    checker.Reset();cv::Mat alias=image.clone();checker.Process(p,nullptr,alias);alias.setTo(0);
    auto owned=checker.Process(c,&pair,translated);Check(owned.features[0].state==LKCheckState::SUPPORTED,"owned raw image cache");
    // Repeated previous index is evaluated once and mapped to both original current indices.
    checker.Reset();checker.Process(p,nullptr,image);auto duplicate=pair;duplicate.matching.features[1].previousIndex=0;auto dupCurrent=c;dupCurrent.rawKeys[1]=dupCurrent.rawKeys[0];
    auto dup=checker.Process(dupCurrent,&duplicate,translated);Check(dup.uniquePoints==3&&dup.features.size()==4&&dup.features[0].qLK==dup.features[1].qLK,"deduplicate source indices");
    cv::Mat blank=cv::Mat::zeros(image.size(),CV_8U);checker.Reset();checker.Process(p,nullptr,blank);
    auto low=checker.Process(c,&pair,blank);for(auto f:low.features) Check(f.state==LKCheckState::UNVERIFIABLE&&f.reason=="FORWARD_FAILED"&&std::isnan(f.qLK.x)&&f.backwardStatus==-1,"low texture fails safely");
    checker.Reset();checker.Process(p,nullptr,image);auto boundary=pair;boundary.previous.rawKeys[0].pt={-1,70};boundary.previous.rawKeys[1].pt={NAN,0};
    auto edge=checker.Process(c,&boundary,translated);Check(edge.features[0].reason=="OUT_OF_BOUNDS"&&edge.features[1].reason=="NONFINITE","invalid coordinates");
    cv::Mat unrelated(image.size(),CV_8U);rng.fill(unrelated,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(unrelated,unrelated,{5,5},.8);
    checker.Reset();checker.Process(p,nullptr,image);auto unstable=checker.Process(c,&pair,unrelated);
    int unstableFB=0;for(auto f:unstable.features) if(std::isfinite(f.fbError)&&f.fbError>cfg.maxFB) {++unstableFB;Check(f.state==LKCheckState::UNVERIFIABLE&&f.reason=="FB_INCONSISTENT","FB must be checked before endpoint");}
    Check(unstableFB>0,"unrelated texture must exercise unstable FB");
    checker.Reset();auto first=checker.Process(c,&pair,translated);Check(first.reason=="NO_PREVIOUS_FRAME","reset first frame");
    checker.Reset();checker.Process(p,nullptr,image);auto gap=c;gap.timestamp=1;Check(checker.Process(gap,&pair,translated).reason=="TIME_GAP","time gap");
    checker.Reset();checker.Process(p,nullptr,image);auto mismatch=pair;mismatch.previous.timestamp=.01;Check(checker.Process(c,&mismatch,translated).reason=="FRAME_IMAGE_MISMATCH","timestamp mismatch");
    checker.Reset();checker.Process(p,nullptr,image);auto resized=c;resized.size={160,120};cv::Mat small;cv::resize(translated,small,resized.size);Check(checker.Process(resized,&pair,small).reason=="INCOMPATIBLE_INPUT","image size mismatch");
    Check(checker.Process(c,&pair,cv::Mat()).reason=="INVALID_GRAY_IMAGE","empty image");
    checker.Reset();checker.Process(p,nullptr,image);auto noMatch=pair;noMatch.matching.features.clear();Check(checker.Process(c,&noMatch,translated).reason=="NO_MATCH","no matches");
    LKCheck::Invalidate(result,"RESET");for(auto f:result.features) Check(f.state==LKCheckState::UNVERIFIABLE&&std::isnan(f.fbError),"reset pending result");
    {
        char directory[]="/tmp/lk-check-visual-XXXXXX";char* path=mkdtemp(directory);Check(path,"temporary visual output");
        auto visual=cfg;visual.output=path;visual.visualize=true;visual.saveEvery=1;visual.maxImages=1;LKCheck recorder(visual);
        auto vinput=Input(7);vinput.instances.resize(30,0);auto vpair=Pair(Input(6));vpair.previous.instances.resize(40,0);
        LKCheckResult vr;vr.frameId=7;vr.previousFrameId=6;vr.timestamp=7/30.;vr.previousTimestamp=6/30.;vr.previousGray=image.clone();vr.currentGray=translated.clone();
        for(int j=0;j<30;++j) {LKCheckFeature f;f.currentIndex=j;f.previousIndex=j==29?32:j;f.p={140,140};f.qOrb={144,137};vr.features.push_back(f);}
        recorder.Record(vinput,&vpair,vr,nullptr,nullptr);
        Check(std::ifstream(visual.output+"/frame_7_feature_29.png").good()&&!std::ifstream(visual.output+"/frame_7_feature_0.png").good(),"priority sample must reserve visual cap");
        vr.frameId=8;vinput.size={320,240};recorder.Record(vinput,&vpair,vr,nullptr,nullptr); // incompatible cached image must not crash visualization
    }
    std::cout<<"PASS LK configuration, translation, independent endpoints, raw coordinates, deduplication, ownership, invalid inputs and lifecycle\n";
    return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}}
