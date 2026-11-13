#ifndef ORB_SLAM3_DYNAMIC_FILTER_H
#define ORB_SLAM3_DYNAMIC_FILTER_H
#include "LKCheck.h"
#include "DynamicFilterView.h"
namespace ORB_SLAM3 {
class Frame;
struct DynamicFilterConfig {
    bool enabled=false;
    int raw=1;
    std::string mode="simulate",output="outputs/dynamic-filter";
    double maxRatio=.2;
    int minRemaining=50,minFinalInliers=30,minCells=6;
    static DynamicFilterConfig Read(cv::FileStorage&);
};
struct DynamicCandidate {
    bool candidate=false;
    std::string reason="NO_EVIDENCE";
};
std::vector<DynamicCandidate> ConservativeCandidates(const EpipolarInput&,const DiagnosticPair&,
    const EpipolarResult&,const Residual3DResult&,const LKCheckResult&);
class DynamicFilter {
public:
    explicit DynamicFilter(const DynamicFilterConfig&);
    void Process(Frame&,const EpipolarInput&,const DiagnosticPair&,const EpipolarResult&,
        const Residual3DResult&,const LKCheckResult&,bool eligible,int& trackingInliers,int requiredInliers);
    const DynamicFilterView& LastView() const {return view_;}
private:
    DynamicFilterView view_;
    DynamicFilterConfig c_;
    std::ofstream frames_,features_,poses_;
};
}
#endif
