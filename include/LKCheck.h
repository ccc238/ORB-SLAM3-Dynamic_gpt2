#ifndef ORB_SLAM3_LK_CHECK_H
#define ORB_SLAM3_LK_CHECK_H
#include "Residual3D.h"
namespace ORB_SLAM3 {
struct LKCheckConfig {
    int configuredValue=1;
    bool enabled=false,visualize=false;
    int window=21,maxLevel=3,iterations=30,saveEvery=10,maxImages=4;
    double epsilon=.01,minEig=1e-4,maxFB=1,maxEndpoint=3,maxDt=.2;
    std::string output="outputs/lk-check";
    static LKCheckConfig Read(cv::FileStorage&);
};
enum class LKCheckState { SUPPORTED, DISAGREES, UNVERIFIABLE };
const char* LKCheckStateName(LKCheckState);
struct LKCheckFeature {
    int previousIndex=-1,currentIndex=-1,forwardStatus=-1,backwardStatus=-1;
    cv::Point2f p,qOrb,qLK,pBack;
    cv::Point2d fbVector,endpointVector;
    double fbError=std::numeric_limits<double>::quiet_NaN(),endpointError=std::numeric_limits<double>::quiet_NaN();
    LKCheckState state=LKCheckState::UNVERIFIABLE;
    std::string reason="NOT_COMPUTED";
    LKCheckFeature();
};
struct LKCheckResult {
    long frameId=-1,previousFrameId=-1;
    double timestamp=0,previousTimestamp=std::numeric_limits<double>::quiet_NaN(),computeMs=0;
    int uniquePoints=0;
    std::string reason="NO_MATCH_INPUT";
    std::vector<LKCheckFeature> features; // One row per original diagnostic pair, no filtering.
    cv::Mat previousGray,currentGray; // Owned immutable buffers, safe after Track/reset.
};
class LKCheck {
public:
    explicit LKCheck(const LKCheckConfig&,bool record=true);
    LKCheckResult Process(const EpipolarInput&,const DiagnosticPair*,const cv::Mat& gray);
    void Record(const EpipolarInput&,const DiagnosticPair*,const LKCheckResult&,
                const EpipolarResult*,const Residual3DResult*);
    void Reset();
    static void Invalidate(LKCheckResult&,const std::string& reason);
private:
    LKCheckConfig config_;
    bool havePrevious_=false;
    long previousId_=-1;
    double previousTime_=0;
    cv::Mat previousGray_,previousK_,previousDistortion_;
    std::ofstream frames_,features_;
};
}
#endif
