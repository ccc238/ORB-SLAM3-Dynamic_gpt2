#include "SemanticMasker.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "System.h"
#include "CameraModels/Pinhole.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <set>
using namespace ORB_SLAM3;
static void Check(bool value,const char* msg) {if(!value) throw std::runtime_error(msg);}
static void Extract(const cv::Mat& image,const cv::Mat& blocked) {
    cv::Mat gray,valid,desc; cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    cv::bitwise_not(blocked,valid);
    ORBextractor extractor(1000,1.2,8,20,7);
    std::vector<cv::KeyPoint> keys; std::vector<int> lapping={0,0};
    extractor(gray,valid,keys,desc,lapping);
    Check(desc.rows==static_cast<int>(keys.size()),"descriptor row mismatch");
    for(auto& key:keys) Check(!blocked.at<uchar>(cvFloor(key.pt.y),cvFloor(key.pt.x)),"blocked keypoint retained");
    std::cout << "extract size=" << image.size() << " candidates=" << extractor.maskCandidates << " rejected=" << extractor.maskRejected << " keys=" << keys.size() << " violations=0\n";
}
int main(int argc,char** argv) {
 try {
    Check(argc==4 || argc==6,"usage: semantic_mask_test model image output_directory [vocabulary fully_blocked_settings]");
    SemanticConfig defaults;
    Check(defaults.enabled && defaults.mode=="selected" && defaults.classes==std::vector<int>{0},"default must select person only");
    SemanticConfig cfg; cfg.mode="all"; cfg.enabled=true; cfg.model=argv[1]; cfg.output=argv[3]; cfg.debug=true;
    SemanticMasker all(cfg);
    cv::Mat source=cv::imread(argv[2]); Check(!source.empty(),"image load failed");
    cv::Mat saved=source.clone();
    SemanticResult reference=all.Infer(source,false);
    Check(!reference.instances.empty(),"real image should contain detections");
    Check(cv::norm(source,saved,cv::NORM_INF)==0,"input mutated");
    cv::Mat unionMask=cv::Mat::zeros(source.size(),CV_8U);
    std::set<int> ids;
    for(auto& instance:reference.instances) {
        Check(instance.mask.size()==source.size(),"instance size mismatch");
        ids.insert(instance.classId); cv::bitwise_or(unionMask,instance.mask,unionMask);
        for(int y=0;y<source.rows;++y) for(int x=0;x<source.cols;++x) if(instance.mask.at<uchar>(y,x))
            Check(x>=instance.box.x && x<instance.box.x+instance.box.width && y>=instance.box.y && y<instance.box.y+instance.box.height,"mask outside instance box");
    }
    Check(cv::norm(unionMask,reference.blocked,cv::NORM_INF)==0,"all mask union mismatch");
    cv::Mat rgb; cv::cvtColor(source,rgb,cv::COLOR_BGR2RGB);
    auto rgbResult=all.Infer(rgb,true);
    Check(cv::norm(reference.blocked,rgbResult.blocked,cv::NORM_INF)==0,"RGB/BGR mismatch");
    SemanticConfig personConfig; personConfig.enabled=true; personConfig.model=argv[1];
    SemanticMasker person(personConfig); auto personResult=person.Infer(source,false);
    unionMask.setTo(0);
    for(auto& i:reference.instances) if(i.classId==0) cv::bitwise_or(unionMask,i.mask,unionMask);
    Check(cv::norm(unionMask,personResult.blocked,cv::NORM_INF)==0,"default mask must include persons only");
    cfg.mode="selected"; cfg.classes={0,56}; cfg.debug=false;
    SemanticMasker selected(cfg); auto sr=selected.Infer(source,false);
    unionMask.setTo(0);
    for(auto& i:reference.instances) if(i.classId==0 || i.classId==56) cv::bitwise_or(unionMask,i.mask,unionMask);
    Check(cv::norm(unionMask,sr.blocked,cv::NORM_INF)==0,"selected union mismatch");
    cfg.confidence=1; SemanticMasker empty(cfg); auto er=empty.Infer(source,false);
    Check(er.instances.empty() && cv::countNonZero(er.blocked)==0,"empty detection failed");
    std::vector<cv::Size> sizes={source.size(),cv::Size(480,640),cv::Size(321,239),cv::Size(960,320),cv::Size(160,120)};
    int frame=0;
    for(auto size:sizes) {
        cv::Mat image; cv::resize(source,image,size);
        auto result=all.Infer(image,false);
        Extract(image,result.blocked);
        cv::Mat gray,valid,desc; cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY); cv::bitwise_not(result.blocked,valid);
        ORBextractor orb(1000,1.2,8,20,7); std::vector<cv::KeyPoint> keys; std::vector<int> lap={0,0};
        orb(gray,valid,keys,desc,lap);
        all.Record(image,false,result,keys,frame++,0,orb.maskCandidates,orb.maskRejected,0);
    }
    cv::Mat boundary=cv::Mat::zeros(source.size(),CV_8U);
    boundary.colRange(source.cols/2,source.cols).setTo(255);
    Extract(source,boundary);
    cv::Mat portrait; cv::resize(source,portrait,cv::Size(180,640));
    Extract(portrait,cv::Mat::zeros(portrait.size(),CV_8U));
    Extract(source,cv::Mat(source.size(),CV_8U,cv::Scalar(255)));
    Extract(source,cv::Mat::zeros(source.size(),CV_8U));
    // Empty mask (old call) and all-allowed mask must produce identical descriptors.
    cv::Mat checkGray,descOld,descAllowed; cv::cvtColor(source,checkGray,cv::COLOR_BGR2GRAY);
    ORBextractor compare(1000,1.2,8,20,7);
    std::vector<cv::KeyPoint> oldKeys,allowedKeys; std::vector<int> lapping={0,0};
    compare(checkGray,cv::Mat(),oldKeys,descOld,lapping);
    compare(checkGray,cv::Mat(source.size(),CV_8U,cv::Scalar(255)),allowedKeys,descAllowed,lapping);
    Check(oldKeys.size()==allowedKeys.size() && cv::norm(descOld,descAllowed,cv::NORM_INF)==0,"old/no-mask compatibility failed");
    for(size_t i=0;i<oldKeys.size();++i) Check(oldKeys[i].pt==allowedKeys[i].pt,"old/no-mask coordinates changed");
    // Fully blocked RGB-D Frame must leave all aligned arrays empty and safe sentinels.
    cv::Mat gray; cv::cvtColor(source,gray,cv::COLOR_BGR2GRAY);
    cv::Mat K=(cv::Mat_<float>(3,3)<<535,0,320,0,539,240,0,0,1), distortion=cv::Mat::zeros(4,1,CV_32F);
    Pinhole camera(std::vector<float>{535,539,320,240}); ORBextractor orb(1000,1.2,8,20,7);
    Frame frameEmpty(gray,cv::Mat(source.size(),CV_32F,cv::Scalar(1)),0,&orb,nullptr,K,distortion,40,40,&camera,nullptr,IMU::Calib(),cv::Mat::zeros(source.size(),CV_8U));
    Check(frameEmpty.N==0 && frameEmpty.Nleft==-1 && frameEmpty.mDescriptors.empty() && frameEmpty.mvuRight.empty() && frameEmpty.mvDepth.empty(),"empty Frame inconsistency");
    bool rejectedModel=false;
    try { SemanticConfig bad=cfg; bad.model=argv[2]; SemanticMasker wrongModel(bad); }
    catch(const std::exception& e) { rejectedModel=true; std::cout << "expected model error: " << e.what() << '\n'; }
    Check(rejectedModel,"invalid model did not fail");
    std::string badPath=std::string(argv[3])+"/invalid-selected.yaml";
    { std::ofstream bad(badPath); bad << "%YAML:1.0\nSemanticMask.Enabled: 1\nSemanticMask.Model: test.onnx\nSemanticMask.Mode: selected\nSemanticMask.Classes: []\n"; }
    bool rejectedSelection=false;
    try { SemanticConfig::Read(badPath); }
    catch(const std::exception& e) { rejectedSelection=true; std::cout << "expected selected error: " << e.what() << '\n'; }
    Check(rejectedSelection,"selected with explicit empty classes did not fail");
    { std::ofstream defaultFile(badPath); defaultFile << "%YAML:1.0\nSemanticMask.Model: test.onnx\n"; }
    auto loadedDefaults=SemanticConfig::Read(badPath);
    Check(loadedDefaults.enabled && loadedDefaults.mode=="selected" && loadedDefaults.classes==std::vector<int>{0},"YAML defaults must select person only");
    Check(!SemanticConfig::Read(badPath,false).enabled,"other sensors must remain disabled by default");
    { std::ofstream disabled(badPath); disabled << "%YAML:1.0\nSemanticMask.Enabled: 0\n"; }
    Check(!SemanticConfig::Read(badPath).enabled,"explicit disable must override the default");
    rejectedSelection=false;
    try { SemanticConfig bad=cfg; bad.classes.clear(); SemanticMasker invalidSelection(bad); }
    catch(const std::exception& e) { rejectedSelection=true; std::cout << "expected direct config error: " << e.what() << '\n'; }
    Check(rejectedSelection,"direct selected config without classes did not fail");
    if(argc==6) {
        cv::Mat noise(source.size(),CV_8UC3);
        cv::RNG rng(12345); rng.fill(noise,cv::RNG::UNIFORM,0,256);
        Check(all.Infer(noise,false).instances.empty(),"noise fixture unexpectedly detected an object");
        System slam(argv[4],argv[5],System::RGBD,false);
        try {
            cv::Mat depth(source.size(),CV_16U,cv::Scalar(5000));
            slam.TrackRGBD(noise,depth,1.0);
            int initialState=slam.GetTrackingState();
            Check(initialState==2,"noise fixture did not initialize tracking");
            slam.TrackRGBD(source,depth,1.033333);
            int maskedState=slam.GetTrackingState();
            Check(maskedState!=2,"fully blocked frame unexpectedly tracked");
            Check(slam.GetTrackedKeyPointsUn().empty(),"fully blocked frame retained features");
            std::cout << "tracking transition initialized_state=" << initialState << " masked_state=" << maskedState << " keypoints=0\n";
            slam.Shutdown();
        } catch(...) { slam.Shutdown(); throw; }
    }
    std::cout << "PASS real detections=" << reference.instances.size() << " classes="; for(int id:ids) std::cout << id << ' ';
    std::cout << " RGB/BGR, input immutability, all/selected union, empty detections, sizes, zero features\n";
    return 0;
 } catch(const std::exception& e) {std::cerr << "FAIL: " << e.what() << '\n';return 1;}
}
