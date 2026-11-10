#include "SemanticMasker.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "FrameDrawer.h"
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
    Check(argc==4 || argc==6,"usage: semantic_mask_test model image output_directory [vocabulary unfiltered_rgbd_settings]");
    SemanticConfig defaults;
    Check(!defaults.enabled && defaults.maskOnly && defaults.mode=="selected" && defaults.classes==std::vector<int>{0},"default must disable dynamic processing and retain person model policy");
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
    // The live renderer uses an owned, same-frame RGB/mask snapshot and works
    // without a GUI or debug writer. Its binary pane must match the SLAM mask.
    FrameDrawer preview(nullptr);
    Check(!preview.SemanticViewEnabled() && preview.DrawSemanticFrame().empty(),"preview should start disabled");
    preview.EnableSemanticView(true);
    preview.UpdateSemanticFrame(source,false,reference,{},42.0,2);
    cv::Mat panel=preview.DrawSemanticFrame(), expectedBinary;
    cv::cvtColor(reference.blocked,expectedBinary,cv::COLOR_GRAY2BGR);
    Check(panel.size()==cv::Size(source.cols*2,source.rows+48),"preview dimensions incorrect");
    Check(cv::norm(panel(cv::Rect(source.cols,0,source.cols,source.rows)),expectedBinary,cv::NORM_INF)==0,"live binary pane is not the actual mask");
    preview.UpdateSemanticFrame(rgb,true,reference,{},42.0,2);
    Check(cv::norm(panel,preview.DrawSemanticFrame(),cv::NORM_INF)==0,"preview RGB/BGR mismatch");
    panel.setTo(0);
    Check(cv::countNonZero(preview.DrawSemanticFrame().reshape(1))>0,"preview snapshot was modified by reader");
    auto scaled=preview.DrawSemanticFrame(0.5f);
    cv::Mat smallBinary; cv::resize(expectedBinary,smallBinary,source.size()/2,0,0,cv::INTER_NEAREST);
    Check(cv::norm(scaled(cv::Rect(source.cols/2,0,source.cols/2,source.rows/2)),smallBinary,cv::NORM_INF)==0,"preview binary scaling must be nearest-neighbor");
    Check(cv::imwrite(std::string(argv[3])+"/semantic-preview.png",preview.DrawSemanticFrame()),"cannot save preview fixture");
    SemanticResult noRegions; noRegions.blocked=cv::Mat::zeros(source.size(),CV_8UC1);
    preview.UpdateSemanticFrame(source,false,noRegions,{},43.0,1);
    Check(cv::countNonZero(preview.DrawSemanticFrame()(cv::Rect(source.cols,0,source.cols,source.rows)).reshape(1))==0,"empty detection preview must be black");
    preview.EnableSemanticView(false);
    Check(preview.DrawSemanticFrame().empty(),"disabled preview retained stale frame");
    std::cout << "PASS live preview: exact mask, RGB/BGR, snapshot lifetime, nearest-neighbor scaling, empty/disabled\n";
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
        Check(result.instanceLabels.type()==CV_16UC1 && result.instanceLabels.size()==size,"resized/portrait label coordinates incorrect");
        auto sizeIds=QueryInstanceLabels(result.instanceLabels,size,{cv::KeyPoint(size.width/2.f,size.height/2.f,1.f)},result.instanceInfo);
        Check(sizeIds.size()==1 && sizeIds[0]>=0,"size association invalid");
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
    { std::ofstream bad(badPath); bad << "%YAML:1.0\nDynamic.Enable: 1\nSemanticMask.Model: test.onnx\nSemanticMask.Mode: selected\nSemanticMask.Classes: []\n"; }
    bool rejectedSelection=false;
    try { SemanticConfig::Read(badPath); }
    catch(const std::exception& e) { rejectedSelection=true; std::cout << "expected selected error: " << e.what() << '\n'; }
    Check(rejectedSelection,"selected with explicit empty classes did not fail");
    { std::ofstream defaultFile(badPath); defaultFile << "%YAML:1.0\nSemanticMask.Model: test.onnx\n"; }
    auto loadedDefaults=SemanticConfig::Read(badPath);
    Check(!loadedDefaults.enabled && loadedDefaults.maskOnly && loadedDefaults.mode=="selected" && loadedDefaults.classes==std::vector<int>{0},"missing Dynamic parameters must disable processing");
    { std::ofstream disabled(badPath); disabled << "%YAML:1.0\nSemanticMask.Enabled: 0\n"; }
    Check(!SemanticConfig::Read(badPath).enabled,"explicit disable must override the default");
    rejectedSelection=false;
    try { SemanticConfig bad=cfg; bad.classes.clear(); SemanticMasker invalidSelection(bad); }
    catch(const std::exception& e) { rejectedSelection=true; std::cout << "expected direct config error: " << e.what() << '\n'; }
    Check(rejectedSelection,"direct selected config without classes did not fail");
    // Association-only preserves every existing indexed array and uses raw keys.
    SemanticConfig ac=personConfig; ac.maskingEnabled=false; ac.associationEnabled=true; ac.associationDebug=true; ac.output=std::string(argv[3])+"/association";
    SemanticMasker associate(ac); auto ar=associate.Infer(source,false);
    Check(cv::countNonZero(ar.blocked)==0,"association-only still blocks features");
    Check(cv::norm(ar.instanceLabels,reference.instanceLabels,cv::NORM_INF)==0,"selected class policy affected association labels");
    Frame associated(gray,cv::Mat(source.size(),CV_32F,cv::Scalar(1)),42,&orb,nullptr,K,distortion,40,40,&camera);
    Check(orb.maskRejected==0,"unmasked Frame still rejected semantic candidates");
    associated.mnDataset=0;
    int before=associated.N; auto beforeKeys=associated.mvKeys; auto beforeDepth=associated.mvDepth; auto beforeRight=associated.mvuRight; auto beforePoints=associated.mvpMapPoints; cv::Mat beforeDesc=associated.mDescriptors.clone();
    Check(associated.mvInstanceIds==std::vector<int>(before,-1) && !associated.mbInstanceAssociation,"disabled labels must be unqueried");
    associated.AssociateInstances(ar,source.size());
    Check(associated.N==before && associated.mvInstanceIds.size()==static_cast<size_t>(before),"association changed feature count");
    Check(associated.mvDepth==beforeDepth && associated.mvuRight==beforeRight && associated.mvpMapPoints==beforePoints && cv::norm(beforeDesc,associated.mDescriptors,cv::NORM_INF)==0,"association changed indexed arrays");
    for(size_t i=0;i<beforeKeys.size();++i) Check(beforeKeys[i].pt==associated.mvKeys[i].pt,"association changed feature order");
    int bg=0,inside=0,invalid=0,personFeatures=0,otherFeatures=0;
    for(int id:associated.mvInstanceIds) {
        if(id<0) ++invalid; else if(id==0) ++bg; else {
            ++inside; Check(id<=static_cast<int>(ar.instanceInfo.size()),"unknown positive instance ID");
            if(ar.instanceInfo[id-1].classId==0) ++personFeatures; else ++otherFeatures;
        }
    }
    Check(bg+inside+invalid==before && bg>0 && personFeatures>0,"real image must retain background and person features");
    Frame copied(associated); Frame assigned; assigned=associated;
    Check(copied.mvInstanceIds==associated.mvInstanceIds && assigned.mvInstanceIds==associated.mvInstanceIds && copied.mvInstanceInfo.size()==ar.instanceInfo.size() && copied.mbInstanceAssociation,"Frame copy lost association");
    copied.mvInstanceIds[0]=-1; Check(assigned.mvInstanceIds==associated.mvInstanceIds,"copied vector aliases original");
    associate.RecordAssociation(source,false,ar,associated.mvKeys,associated.mvInstanceIds,associated.mnId,42);
    preview.EnableSemanticView(true); preview.UpdateSemanticFrame(source,false,ar,associated.mvKeys,42,2,&associated.mvInstanceIds);
    Check(!preview.DrawSemanticFrame().empty(),"association live view missing");
    Check(cv::countNonZero(er.instanceLabels)==0,"empty detections must have zero labels");
    associated.AssociateInstances(er,source.size());
    Check(associated.mvInstanceIds==std::vector<int>(before,0),"empty detection association not background");
    assigned=frameEmpty; Check(assigned.mvInstanceIds.empty() && !assigned.mbInstanceAssociation,"assignment retained stale labels");
    SemanticConfig dilated=personConfig; dilated.dilation=3; SemanticMasker dm(dilated); auto dr=dm.Infer(source,false);
    Check(cv::norm(dr.instanceLabels,ar.instanceLabels,cv::NORM_INF)==0,"dilation changed association labels");
    SemanticResult controlled; cv::Mat m(4,6,CV_8U,cv::Scalar(255));
    controlled.instances.push_back({cv::Rect2f(0,0,6,4),0,.4f,m});
    controlled.instances.push_back({cv::Rect2f(0,0,6,4),56,.9f,m});
    BuildInstanceLabels(controlled,m.size()); Check(controlled.instanceLabels.at<uint16_t>(1,2)==2,"overlap must prefer highest confidence");
    std::vector<cv::KeyPoint> query;
    for(auto p:std::vector<cv::Point2f>{{2.9f,1.9f},{-0.1f,1},{6,1},{1,4},{NAN,1},{1,INFINITY}}) query.emplace_back(p,1.f);
    auto queried=QueryInstanceLabels(controlled.instanceLabels,m.size(),query,controlled.instanceInfo);
    Check(queried==std::vector<int>({2,-1,-1,-1,-1,-1}),"floor/OOB/nonfinite query failed");
    Frame controlledFrame; controlledFrame.N=query.size(); controlledFrame.mvKeys=query;
    controlledFrame.AssociateInstances(controlled,m.size());
    Check(controlledFrame.mvInstanceIds==queried,"Frame invalid-coordinate association failed");
    controlled.instances[1].confidence=.4f; BuildInstanceLabels(controlled,m.size());
    Check(controlled.instanceLabels.at<uint16_t>(1,2)==1,"equal confidence tie must preserve detection order");
    controlled.instances[1].confidence=.9f; BuildInstanceLabels(controlled,m.size());
    controlled.instanceLabels.at<uint16_t>(1,2)=0;
    Check(QueryInstanceLabels(controlled.instanceLabels,m.size(),query,controlled.instanceInfo)[0]==0,"row/column or floor incorrect");
    bool badType=false,badSize=false,badTable=false;
    try {QueryInstanceLabels(m,m.size(),query,controlled.instanceInfo);} catch(const std::exception&) {badType=true;}
    try {QueryInstanceLabels(controlled.instanceLabels,{3,4},query,controlled.instanceInfo);} catch(const std::exception&) {badSize=true;}
    try {QueryInstanceLabels(controlled.instanceLabels,m.size(),query,{});} catch(const std::exception&) {badTable=true;}
    Check(badType&&badSize&&badTable,"invalid label input must fail");
    // Dynamic.* takes precedence over all historical switches, including debug.
    for(int enable=0;enable<=1;++enable) for(int maskOnly=0;maskOnly<=1;++maskOnly) {
        std::ofstream settings(badPath);
        settings << "%YAML:1.0\nDynamic.Enable: " << enable << "\nDynamic.MaskOnly: " << maskOnly
                 << "\nSemanticMask.Enabled: " << (1-enable)
                 << "\nSemanticMask.MaskingEnabled: 1\nSemanticMask.AssociationEnabled: 0\nSemanticMask.AssociationDebug: 1\n";
        settings.close();
        auto dynamic=SemanticConfig::Read(badPath);
        Check(dynamic.enabled==bool(enable) && dynamic.maskOnly==bool(maskOnly),"Dynamic flags not loaded");
        Check(!dynamic.maskingEnabled && dynamic.associationEnabled==bool(enable) && dynamic.associationDebug==bool(enable),"legacy switches overrode Dynamic gates");
    }
    for(const char* key:{"Dynamic.Enable","Dynamic.MaskOnly"}) {
        {std::ofstream settings(badPath); settings << "%YAML:1.0\n" << key << ": 2\n";}
        bool rejected=false; try {SemanticConfig::Read(badPath);} catch(const std::exception&) {rejected=true;}
        Check(rejected,"invalid Dynamic switch accepted");
    }
    std::cout << "PASS association features_before=" << before << " after=" << before << " background=" << bg << " instance=" << inside << " invalid=" << invalid << " person=" << personFeatures << " other_classes=" << otherFeatures << "; copy, empty, overlap, floor, bounds, shape/type, dilation, selected independence verified\n";
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
            const auto count=slam.GetTrackedKeyPointsUn().size();
            Check(count>0,"online configuration unexpectedly removed all features");
            std::cout << "unfiltered tracking initialized_state=" << initialState << " next_state=" << maskedState << " keypoints=" << count << "\n";
            slam.Shutdown();
        } catch(...) { slam.Shutdown(); throw; }
    }
    std::cout << "PASS real detections=" << reference.instances.size() << " classes="; for(int id:ids) std::cout << id << ' ';
    std::cout << " RGB/BGR, input immutability, all/selected union, empty detections, sizes, zero features\n";
    return 0;
 } catch(const std::exception& e) {std::cerr << "FAIL: " << e.what() << '\n';return 1;}
}
