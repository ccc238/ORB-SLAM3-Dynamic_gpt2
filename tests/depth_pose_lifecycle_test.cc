// Real RGB-D lifecycle exercise. Output directories/diagnostics come from the supplied existing-style YAML.
#include "System.h"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>
struct Sample {double time;std::string rgb,depth;};
int main(int argc,char** argv) {
    if(argc!=4) {std::cerr<<"usage: depth_pose_lifecycle_test vocabulary settings dataset\n";return 1;}
    try {
        std::string root=argv[3];std::ifstream associations(root+"/associate.txt");
        std::vector<Sample> samples;Sample a;double depthTime;
        while(associations>>a.time>>a.rgb>>depthTime>>a.depth) {samples.push_back(a);if(samples.size()==28) break;}
        if(samples.size()<28) throw std::runtime_error("need 28 RGB-D pairs");
        ORB_SLAM3::System slam(argv[1],argv[2],ORB_SLAM3::System::RGBD,false);
        try {
            for(size_t i=0;i<samples.size();++i) {
                if(i==16) {std::cout<<"[Lifecycle] ResetActiveMap before sample=16\n";slam.ResetActiveMap();}
                if(i==22) {std::cout<<"[Lifecycle] ChangeDataset before sample=22\n";slam.ChangeDataset();}
                const auto& a=samples[i];
                cv::Mat rgb=cv::imread(root+"/"+a.rgb,cv::IMREAD_UNCHANGED),depth=cv::imread(root+"/"+a.depth,cv::IMREAD_UNCHANGED);
                if(rgb.empty()||depth.empty()) throw std::runtime_error("cannot read input");
                if(i==10||i==11) rgb.setTo(0); // actual tracking failure, without changing algorithms
                slam.TrackRGBD(rgb,depth,a.time);
                std::cout<<"[Lifecycle] sample="<<i<<" timestamp="<<std::setprecision(17)<<a.time<<" state="<<slam.GetTrackingState()<<'\n';
            }
            slam.Shutdown();
        } catch(...) {slam.Shutdown();throw;}
        std::cout<<"PASS lifecycle input exercised\n";
    } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
