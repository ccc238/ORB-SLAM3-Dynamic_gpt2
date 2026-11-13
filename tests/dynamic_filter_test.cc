#include "DynamicFilter.h"
#include "FrameDrawer.h"
#include <opencv2/imgcodecs.hpp>
#include <cstdlib>
#include <thread>
#include <atomic>
#include "Frame.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Optimizer.h"
#include "ORBextractor.h"
#include "SemanticMasker.h"
#include <cstdio>
#include "CameraModels/Pinhole.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <algorithm>
using namespace ORB_SLAM3;
void Check(bool b,const char* s){if(!b)throw std::runtime_error(s);}
DynamicFilterConfig Config(std::string s){cv::FileStorage fs("%YAML:1.0\n"+s,cv::FileStorage::READ|cv::FileStorage::MEMORY);return DynamicFilterConfig::Read(fs);}
void SaveView(const std::string& name,const DynamicFilterView& view) {
    const char* root=std::getenv("ORB_FILTER_VIEW_TEST_OUTPUT");
    const std::string folder=std::string(root?root:"/tmp/dynamic-filter-view-test")+"/"+name;
    FrameDrawer drawer(nullptr);drawer.ConfigureDynamicFilterView(folder,1);
    cv::Mat rgb(480,640,CV_8UC3,cv::Scalar(32,48,64));
    drawer.UpdateDynamicFilterFrame(rgb,true,view);
    rgb.setTo(cv::Scalar(255,255,255)); // ownership of input must not escape publication
    cv::Mat snapshot;DynamicFilterView copied;
    Check(drawer.GetDynamicFilterSnapshot(snapshot,copied),"no complete view snapshot");
    Check(snapshot.at<cv::Vec3b>(0,0)==cv::Vec3b(64,48,32),"RGB conversion or snapshot ownership");
    Check(copied.frameId==view.frameId&&copied.beforeIndices==view.beforeIndices,"snapshot state mismatch");
    auto full=drawer.DrawFrame();auto half=drawer.DrawFrame(2.f);
    Check(half.cols==full.cols/2&&half.rows==full.rows/2,"display scaling mismatch");
    for(size_t i=0;i<view.keys.size();++i) {
        const auto p=view.keys[i].pt;const auto pixel=full.at<cv::Vec3b>(cvRound(p.y),cvRound(p.x));
        if(view.excludedObservations[i])Check(pixel==cv::Vec3b(0,0,255),"accepted exclusion missing red cross");
        else if(!view.candidates[i])Check(pixel==cv::Vec3b(150,150,150),"noncandidate not gray");
        else Check(pixel!=cv::Vec3b(0,0,255),"candidate drawn as excluded");
    }
    cv::imwrite(folder+"/current_frame.png",full);
}
int main(){try{
    Check(!Config("").enabled&&!Config("Dynamic.UseDynamicFilter: 1\n").enabled,"default off");
    Check(Config("Dynamic.UseDynamicFilter: 0\n").enabled,"0 enabled");
    for(auto s:{"2","-1","0.0","\"0\"","true","[]","{}"}){bool bad=false;try{Config(std::string("Dynamic.UseDynamicFilter: ")+s+"\n");}catch(...){bad=true;}Check(bad,"strict switch");}
    for(auto s:{"Dynamic.FilterMode: bad","Dynamic.FilterMode: 0","Dynamic.FilterMaxRatio: 1","Dynamic.FilterMinRemaining: 2","Dynamic.FilterMinFinalInliers: 29","Dynamic.FilterMinCells: 13"}){bool bad=false;try{Config(std::string(s)+"\n");}catch(...){bad=true;}Check(bad,"strict limits/mode");}
    const std::string required="Dynamic.Enable: 1\nDynamic.UseEpipolar: 0\nDynamic.Use3DResidual: 0\nDynamic.DepthPoseSource: tracking\nResidual3D.UseDepthDifferenceGate: 1\nDynamic.UseLKCheck: 0\nDynamic.UseDynamicFilter: 0\n";
    auto semanticRead=[](const std::string& settings){
        const std::string path=cv::tempfile(".yaml");
        {std::ofstream file(path);file<<"%YAML:1.0\n"<<settings;}
        try{auto c=SemanticConfig::Read(path);std::remove(path.c_str());return c;}
        catch(...){std::remove(path.c_str());throw;}
    };
    Check(semanticRead(required).dynamicFilter.enabled,"compatible dependencies rejected");
    for(auto missing:std::vector<std::pair<std::string,std::string>>{
        {"Dynamic.Enable: 1","Dynamic.Enable: 0"},
        {"Dynamic.UseEpipolar: 0","Dynamic.UseEpipolar: 1"},
        {"Dynamic.Use3DResidual: 0","Dynamic.Use3DResidual: 1"},
        {"Dynamic.UseLKCheck: 0","Dynamic.UseLKCheck: 1"},
        {"Dynamic.DepthPoseSource: tracking","Dynamic.DepthPoseSource: background_pnp"},
        {"Residual3D.UseDepthDifferenceGate: 1","Residual3D.UseDepthDifferenceGate: 0"}}){
        auto settings=required;settings.replace(settings.find(missing.first),missing.first.size(),missing.second);
        bool rejected=false;try{semanticRead(settings);}catch(...){rejected=true;}Check(rejected,"incompatible dependencies accepted");
    }
    EpipolarInput in;in.frameId=1;in.timestamp=.03;in.size={640,480};
    DiagnosticPair pair;pair.previous.frameId=0;pair.previous.timestamp=0;pair.matching.previousFrameId=0;
    EpipolarResult e;e.previousFrameId=0;e.dt=.03;e.modelValid=true;
    Residual3DResult d;d.currentFrameId=1;d.previousFrameId=0;d.currentTimestamp=.03;d.previousTimestamp=0;d.motionValid=true;
    LKCheckResult lk;lk.frameId=1;lk.previousFrameId=0;lk.timestamp=.03;lk.previousTimestamp=0;
    for(int i=0;i<96;++i){in.rawKeys.emplace_back(cv::Point2f(40+(i%12)*49,40+(i/12)*55),31);in.instances.push_back(i<4?2:0);pair.previous.instances.push_back(7);
        EpipolarFeature ef;ef.previousIndex=i;ef.previousInstance=7;ef.state=EpipolarState::SUSPECTED_DYNAMIC;ef.residual=4;e.features.push_back(ef);pair.matching.features.push_back(ef);
        Residual3DFeature df;df.previousIndex=i;df.state=EpipolarState::SUSPECTED_DYNAMIC;df.residual=.1;df.depthDifference=.1;df.previousDepth.valid=df.currentDepth.valid=true;d.features.push_back(df);
        LKCheckFeature l;l.previousIndex=l.currentIndex=i;l.state=LKCheckState::SUPPORTED;lk.features.push_back(l);
    }
    auto base=ConservativeCandidates(in,pair,e,d,lk);Check(base[0].candidate&&!base[4].candidate,"intersection/different instance ids");
    for(int kind=0;kind<17;++kind){auto ee=e;auto dd=d;auto ll=lk;auto pp=pair;
        switch(kind){case 0:ll.features.clear();break;case 1:ll.features[0].state=LKCheckState::DISAGREES;break;case 2:ll.features[0].state=LKCheckState::UNVERIFIABLE;break;case 3:ee.modelValid=false;break;case 4:dd.motionValid=false;break;case 5:dd.features[0].previousDepth.valid=false;break;case 6:dd.features[0].currentDepth.valid=false;break;case 7:dd.features[0].depthDifferenceRejected=true;break;case 8:dd.features[0].state=EpipolarState::UNKNOWN;break;case 9:ee.features[0].state=EpipolarState::GEOMETRY_CONSISTENT;break;case 10:dd.currentFrameId=9;break;case 11:ll.previousFrameId=9;break;case 12:ll.features[0].previousIndex=9;break;case 13:pp.previous.instances[0]=0;break;case 14:dd.features[0].residual=NAN;break;case 15:ll.features.push_back(ll.features[0]);break;case 16:dd.currentTimestamp=.05;break;}
        Check(!ConservativeCandidates(in,pp,ee,dd,ll)[0].candidate,"missing/unknown/stale/contradictory accepted");
    }
    Map map;
    Pinhole camera(std::vector<float>{500,500,320,240});
    ORBextractor extractor(300,1.2f,8,20,7);
    cv::Mat gray(480,640,CV_8U),depth(480,640,CV_32F,cv::Scalar(3));
    cv::RNG rng(42);rng.fill(gray,cv::RNG::UNIFORM,0,256);
    cv::Mat K=(cv::Mat_<float>(3,3)<<500,0,320,0,500,240,0,0,1);
    cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F);
    Frame f(gray,depth,.03,&extractor,nullptr,K,distortion,40,40,&camera);
    Check(f.N>=96,"fixture feature count");
    f.N=96;f.mnId=1;f.mvKeys=f.mvKeysUn=in.rawKeys;
    f.mDescriptors=f.mDescriptors.rowRange(0,96).clone();f.mvDepth.assign(96,3);
    f.mvInstanceIds=in.instances;f.mvpMapPoints.assign(96,nullptr);
    f.mvuRight.assign(96,-1);f.mvbOutlier.assign(96,false);f.SetPose(Sophus::SE3f());
    for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y)f.mGrid[x][y].clear();
    for(int i=0;i<96;++i){const auto p=f.mvKeysUn[i].pt;f.mGrid[int(p.x/10)][int(p.y/10)].push_back(i);}
    KeyFrame reference(f,&map,nullptr);
    std::vector<std::unique_ptr<MapPoint>> storage;
    for(int i=0;i<96;++i){auto p=in.rawKeys[i].pt;float z=3+.01*(i%7);storage.emplace_back(new MapPoint(Eigen::Vector3f((p.x-320)*z/500,(p.y-240)*z/500,z),&reference,&map));storage.back()->AddObservation(&reference,i);f.mvpMapPoints[i]=storage.back().get();if(i%2)f.mvuRight[i]=p.x-40/z;}
    Optimizer::PoseOptimization(&f);const auto pose=f.GetPose();auto flags=f.mvbOutlier;auto points=f.mvpMapPoints;auto indices=f.mvPoseOptimizationIndices;const auto initialAudit=f.mPoseObservationAudit;
    SaveView("off",MakeDynamicFilterView(f,indices,initialAudit,false,"simulate",false,false,"FILTER_DISABLED"));
    const cv::Mat descriptorsBefore=f.mDescriptors.clone();const auto depthBefore=f.mvDepth;
    std::vector<std::vector<size_t>> gridBefore;
    for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y)gridBefore.push_back(f.mGrid[x][y]);
    DynamicFilterConfig c;c.enabled=true;c.output="/tmp/dynamic-filter-test-simulate";int inliers=96;
    {DynamicFilter filter(c);filter.Process(f,in,pair,e,d,lk,true,inliers,30);
        if(c.output=="/tmp/dynamic-filter-test-simulate"||c.output=="/tmp/dynamic-filter-test-apply"||c.output=="/tmp/dynamic-filter-test-rollback"||c.output=="/tmp/dynamic-filter-test-safety") {
            const auto& v=filter.LastView();
            Check(v.qualificationAvailable&&v.optimizationAvailable,"missing pose measurement");
            const bool applied=c.output=="/tmp/dynamic-filter-test-apply";
            Check(v.beforeIndices.size()==96&&v.afterIndices.size()==size_t(applied?92:96)&&v.ExcludedCount()==(applied?4:0),"view counts disagree with actual edges");
            SaveView(c.output.substr(std::string("/tmp/dynamic-filter-test-").size()),v);
        }
    }
    Check(f.GetPose().matrix()==pose.matrix()&&f.mvbOutlier==flags&&f.mvpMapPoints==points&&f.mvPoseOptimizationIndices==indices&&inliers==96,"simulate altered algorithm state");
    Check(cv::norm(descriptorsBefore,f.mDescriptors,cv::NORM_INF)==0&&depthBefore==f.mvDepth,"simulate changed descriptors or depth");
    int cell=0;for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y)Check(gridBefore[cell++]==f.mGrid[x][y],"simulate changed grid");
    for(int i=0;i<96;++i)Check(f.mvKeys[i].pt==in.rawKeys[i].pt&&f.mvKeysUn[i].pt==in.rawKeys[i].pt,"simulate changed keypoints");
    c.mode="apply";c.output="/tmp/dynamic-filter-test-apply";
    {DynamicFilter filter(c);filter.Process(f,in,pair,e,d,lk,true,inliers,30);
        if(c.output=="/tmp/dynamic-filter-test-simulate"||c.output=="/tmp/dynamic-filter-test-apply"||c.output=="/tmp/dynamic-filter-test-rollback"||c.output=="/tmp/dynamic-filter-test-safety") {
            const auto& v=filter.LastView();
            Check(v.qualificationAvailable&&v.optimizationAvailable,"missing pose measurement");
            const bool applied=c.output=="/tmp/dynamic-filter-test-apply";
            Check(v.beforeIndices.size()==96&&v.afterIndices.size()==size_t(applied?92:96)&&v.ExcludedCount()==(applied?4:0),"view counts disagree with actual edges");
            SaveView(c.output.substr(std::string("/tmp/dynamic-filter-test-").size()),v);
        }
    }
    Check(f.mvPoseOptimizationIndices.size()==92&&inliers==92,"real optimization/support");for(int i:f.mvPoseOptimizationIndices)Check(!f.DynamicExcluded(i),"excluded final edge");
    KeyFrame target(f,&map,nullptr);
    Check(target.DynamicExcluded(0)&&target.GetMapPoint(0)==nullptr,"keyframe copied excluded observation");
    int observations=storage[0]->Observations();storage[0]->AddObservation(&target,0);target.AddMapPoint(storage[0].get(),0);Check(storage[0]->Observations()==observations,"excluded observation reinserted");
    {Frame mixed=f;auto mixedInput=in;
        mixed.mvbDynamicExcluded.assign(96,false);mixed.mvpMapPoints[4]=nullptr;
        mixed.mvInstanceIds[4]=mixedInput.instances[4]=2;mixed.SetPose(pose);
        Optimizer::PoseOptimization(&mixed);int support=95;
        auto mixedConfig=c;mixedConfig.output="/tmp/dynamic-filter-test-mixed-nomap";
        DynamicFilter filter(mixedConfig);filter.Process(mixed,mixedInput,pair,e,d,lk,true,support,30);
        const auto& v=filter.LastView();
        Check(v.accepted&&v.CandidateCount()==5&&v.ExcludedCount()==4&&v.beforeIndices.size()==95&&v.afterIndices.size()==91,"candidate without optimization observation counted as removed edge");
        Check(!v.excludedObservations[4],"no-map candidate must stay yellow");SaveView("mixed-nomap",v);
    }
    flags[10]=true; // This correct observation is reclassified by the extra optimizer, then must be restored on rollback.
    f.mvbDynamicExcluded.clear();f.SetPose(pose);f.mvbOutlier=flags;f.mvPoseOptimizationIndices=indices;f.mPoseObservationAudit=initialAudit;inliers=96;
    c.minFinalInliers=100;c.output="/tmp/dynamic-filter-test-rollback";
    {DynamicFilter filter(c);filter.Process(f,in,pair,e,d,lk,true,inliers,30);
        if(c.output=="/tmp/dynamic-filter-test-simulate"||c.output=="/tmp/dynamic-filter-test-apply"||c.output=="/tmp/dynamic-filter-test-rollback"||c.output=="/tmp/dynamic-filter-test-safety") {
            const auto& v=filter.LastView();
            Check(v.qualificationAvailable&&v.optimizationAvailable,"missing pose measurement");
            const bool applied=c.output=="/tmp/dynamic-filter-test-apply";
            Check(v.beforeIndices.size()==96&&v.afterIndices.size()==size_t(applied?92:96)&&v.ExcludedCount()==(applied?4:0),"view counts disagree with actual edges");
            SaveView(c.output.substr(std::string("/tmp/dynamic-filter-test-").size()),v);
        }
    }
    Check(f.GetPose().matrix()==pose.matrix()&&f.mvbOutlier==flags&&f.mvpMapPoints==points&&f.mvPoseOptimizationIndices==indices&&inliers==96&&!f.DynamicExcluded(0),"rollback incomplete");
    c.minRemaining=100;c.output="/tmp/dynamic-filter-test-safety";{DynamicFilter filter(c);filter.Process(f,in,pair,e,d,lk,true,inliers,30);
        if(c.output=="/tmp/dynamic-filter-test-simulate"||c.output=="/tmp/dynamic-filter-test-apply"||c.output=="/tmp/dynamic-filter-test-rollback"||c.output=="/tmp/dynamic-filter-test-safety") {
            const auto& v=filter.LastView();
            Check(v.qualificationAvailable&&v.optimizationAvailable,"missing pose measurement");
            const bool applied=c.output=="/tmp/dynamic-filter-test-apply";
            Check(v.beforeIndices.size()==96&&v.afterIndices.size()==size_t(applied?92:96)&&v.ExcludedCount()==(applied?4:0),"view counts disagree with actual edges");
            SaveView(c.output.substr(std::string("/tmp/dynamic-filter-test-").size()),v);
        }
    }Check(!f.DynamicExcluded(0)&&inliers==96,"safety failed");
    {DynamicFilter filter(c);filter.Process(f,in,pair,e,d,lk,false,inliers,30);}Check(!f.mvbDynamicCandidate[0],"unsupported path candidate");
    c.minRemaining=50;c.minFinalInliers=30;
    auto noCandidate=in;std::fill(noCandidate.instances.begin(),noCandidate.instances.end(),0);
    c.output="/tmp/dynamic-filter-test-no-candidate";
    {DynamicFilter filter(c);filter.Process(f,noCandidate,pair,e,d,lk,true,inliers,30);}
    Check(f.mvPoseOptimizationIndices==indices&&!f.DynamicExcluded(0),"no candidate reoptimized");
    for(int i=0;i<4;++i)f.mvpMapPoints[i]=nullptr;
    Optimizer::PoseOptimization(&f);auto noOptIndices=f.mvPoseOptimizationIndices;
    c.output="/tmp/dynamic-filter-test-no-observation";
    {DynamicFilter filter(c);filter.Process(f,in,pair,e,d,lk,true,inliers,30);
        if(c.output=="/tmp/dynamic-filter-test-simulate"||c.output=="/tmp/dynamic-filter-test-apply"||c.output=="/tmp/dynamic-filter-test-rollback"||c.output=="/tmp/dynamic-filter-test-safety") {
            const auto& v=filter.LastView();
            Check(v.qualificationAvailable&&v.optimizationAvailable,"missing pose measurement");
            const bool applied=c.output=="/tmp/dynamic-filter-test-apply";
            Check(v.beforeIndices.size()==96&&v.afterIndices.size()==size_t(applied?92:96)&&v.ExcludedCount()==(applied?4:0),"view counts disagree with actual edges");
            SaveView(c.output.substr(std::string("/tmp/dynamic-filter-test-").size()),v);
        }
    }
    Check(f.mvPoseOptimizationIndices==noOptIndices&&!f.DynamicExcluded(0),"no optimizable candidate reoptimized");
    auto resetPair=pair;resetPair.matching.previousFrameId=-1;
    Check(!ConservativeCandidates(in,resetPair,e,d,lk)[0].candidate,"reset reused cached result");
    f.mvpMapPoints=points;f.mvbDynamicExcluded.assign(96,false);f.SetPose(pose);
    Optimizer::PoseOptimization(&f);inliers=96;c.maxRatio=.00001;
    c.output="/tmp/dynamic-filter-test-ratio";
    {DynamicFilter filter(c);filter.Process(f,in,pair,e,d,lk,true,inliers,30);
        if(c.output=="/tmp/dynamic-filter-test-simulate"||c.output=="/tmp/dynamic-filter-test-apply"||c.output=="/tmp/dynamic-filter-test-rollback"||c.output=="/tmp/dynamic-filter-test-safety") {
            const auto& v=filter.LastView();
            Check(v.qualificationAvailable&&v.optimizationAvailable,"missing pose measurement");
            const bool applied=c.output=="/tmp/dynamic-filter-test-apply";
            Check(v.beforeIndices.size()==96&&v.afterIndices.size()==size_t(applied?92:96)&&v.ExcludedCount()==(applied?4:0),"view counts disagree with actual edges");
            SaveView(c.output.substr(std::string("/tmp/dynamic-filter-test-").size()),v);
        }
    }
    Check(!f.DynamicExcluded(0)&&inliers==96,"ratio gate failed");
    c.maxRatio=.2;
    auto clustered=in;
    for(int i=0;i<96;++i){
        cv::Point2f p(200+(i%12),200+(i/12));float z=3+.01*(i%7);
        clustered.rawKeys[i].pt=p;f.mvKeys[i].pt=p;f.mvKeysUn[i].pt=p;
        storage[i]->SetWorldPos(Eigen::Vector3f((p.x-320)*z/500,(p.y-240)*z/500,z));
        f.mvuRight[i]=i%2?p.x-40/z:-1;
    }
    for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y)f.mGrid[x][y].clear();
    for(int i=0;i<96;++i){auto p=f.mvKeysUn[i].pt;f.mGrid[int(p.x/10)][int(p.y/10)].push_back(i);}
    f.SetPose(Sophus::SE3f());Optimizer::PoseOptimization(&f);inliers=96;
    c.output="/tmp/dynamic-filter-test-coverage";
    {DynamicFilter filter(c);filter.Process(f,clustered,pair,e,d,lk,true,inliers,30);}
    Check(!f.DynamicExcluded(0)&&f.mvPoseOptimizationIndices.size()==96&&inliers==96,"coverage gate failed");
    {Frame unoptimized;unoptimized.mnId=99;unoptimized.mTimeStamp=12.;unoptimized.mvKeys=in.rawKeys;
        unoptimized.mvKeysUn=in.rawKeys;unoptimized.mvKeysUn[0].pt+=cv::Point2f(100,100);
        auto v=MakeDynamicFilterView(unoptimized,{},unoptimized.mPoseObservationAudit,false,"simulate",false,false,"INITIALIZATION");
        Check(v.keys[0].pt==in.rawKeys[0].pt&&v.keys[0].pt!=unoptimized.mvKeysUn[0].pt,"overlay must use raw RGB coordinates");
        Check(!v.optimizationAvailable&&!v.qualificationAvailable,"unoptimized frame must be N/A");SaveView("initialization",v);
    }
    {FrameDrawer drawer(nullptr);drawer.ConfigureDynamicFilterView("",0);
        std::atomic<bool> done(false);std::atomic<int> checked(0);
        std::thread producer([&]{for(int i=0;i<200;++i){
            DynamicFilterView v;v.frameId=i;v.timestamp=i*.01;v.keys.emplace_back(cv::Point2f(i,20),1);
            v.candidates={false};v.excludedObservations={false};
            cv::Mat image(48,256,CV_8UC3,cv::Scalar(i,7,9));drawer.UpdateDynamicFilterFrame(image,false,v);
        }done=true;});
        do{cv::Mat image;DynamicFilterView v;if(drawer.GetDynamicFilterSnapshot(image,v)){
            Check(image.at<cv::Vec3b>(0,0)[0]==v.frameId&&v.keys[0].pt.x==v.frameId&&v.timestamp==v.frameId*.01,"mixed frame snapshot");++checked;
        }}while(!done);
        producer.join();Check(checked>0,"snapshot reader never ran");
    }
    std::cout<<"PASS dynamic filter view states, N/A, RGB, scale, real edge counts; dynamic filter rules, simulate invariance, mono/stereo edges, observation guard, rollback and safety\n";
}catch(const std::exception& ex){std::cerr<<"FAIL "<<ex.what()<<'\n';return 1;}}
