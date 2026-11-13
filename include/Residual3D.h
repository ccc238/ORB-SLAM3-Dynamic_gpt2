#ifndef ORB_SLAM3_RESIDUAL3D_H
#define ORB_SLAM3_RESIDUAL3D_H
#include "EpipolarMotionDetector.h"
namespace ORB_SLAM3 {
struct Residual3DConfig {
    int configuredValue=1;
    std::string poseSource="tracking";
    int trackingMinInliers=25;
    bool useDepthDifferenceGate=false;
    double maxDepthDifferenceM=0.20;
    bool enabled=false, debug=false, visualize=false;
    int minBackground=30,minInliers=20,iterations=200,depthRadius=1,every=1;
    double minDepth=0.2,maxDepth=8,depthJumpM=0.05,depthJumpRatio=0.02;
    double ratio=0.75,maxDt=0.2,ransacPx=2,confidence=0.99,minInlierRatio=0.5;
    double minCoverage=0.15,minShapeRatio=0.001,maxReprojectionRms=2,lowM=0.03,highM=0.08;
    std::string output="outputs/residual3d";
    static Residual3DConfig Read(cv::FileStorage&,bool segmentation,bool association,bool masking);
};
struct DepthSample {
    double meters=std::numeric_limits<double>::quiet_NaN();
    bool valid=false;
    std::string reason="NO_DEPTH";
};
struct Residual3DFeature {
    int previousIndex=-1,backgroundInlier=-1;
    DepthSample previousDepth,currentDepth;
    cv::Vec3d vector=cv::Vec3d::all(std::numeric_limits<double>::quiet_NaN());
    double residual=std::numeric_limits<double>::quiet_NaN();
    double rawResidual=std::numeric_limits<double>::quiet_NaN(),depthDifference=std::numeric_limits<double>::quiet_NaN();
    bool depthDifferenceRejected=false;
    EpipolarState state=EpipolarState::UNKNOWN;
    std::string reason="NO_MATCH";
};
// Immutable value snapshots: Tcw maps world metres into camera metres.
struct TrackingDepthPose {
    cv::Matx44d Tcw=cv::Matx44d::eye();
    long frameId=-1,mapId=-1,relocalizationFrame=-1;
    double timestamp=std::numeric_limits<double>::quiet_NaN();
    int mapChange=-1,bigChange=-1,inliers=-1;
    bool hasPose=false,trackingOK=false;
    cv::Matx44d originalTcw=cv::Matx44d::eye(),frameToReference=cv::Matx44d::eye(),referenceTcw=cv::Matx44d::eye();
    long referenceId=-1;
    int originalMapChange=-1;
    bool rebased=false;
    std::string rebaseReason="NOT_REQUIRED";
};
struct DepthRelativeMotion {
    cv::Mat T_current_previous,R,t; // CV_64F; previous camera -> current camera; metres
    bool motionValid=false;
    std::string poseSource,reason="NO_PREVIOUS_FRAME";
    long currentFrameId=-1,previousFrameId=-1;
    double currentTimestamp=std::numeric_limits<double>::quiet_NaN(),previousTimestamp=std::numeric_limits<double>::quiet_NaN();
    TrackingDepthPose previousTracking,currentTracking;
    int background=0,inliers=0; // background PnP only; never Tracking counts
    double inlierRatio=0,coverage=0,shapeRatio=0,reprojectionRms=std::numeric_limits<double>::quiet_NaN();
    double estimateMs=0;
    std::vector<int> backgroundInliers;
};
struct Residual3DResult : DepthRelativeMotion {
    std::vector<Residual3DFeature> features;
    double validDepthRatio=0,residualMs=0;
};
class Residual3DDetector {
public:
    explicit Residual3DDetector(const Residual3DConfig&);
    static DepthSample CheckDepth(double meters,cv::Point2f raw,const cv::Mat& depth,const Residual3DConfig&);
    static cv::Vec3d Backproject(double depth,cv::Point2f undistorted,const cv::Mat& K);
    static cv::Vec3d Difference(const cv::Vec3d& previous,const cv::Vec3d& current,const cv::Mat& R,const cv::Mat& t);
    static bool RebasePreviousPose(TrackingDepthPose& previous,const TrackingDepthPose& current,
        const cv::Matx44d& latestReferenceTcw,long referenceId,long referenceMapId,bool referenceValid);
    static DepthRelativeMotion TrackingMotion(const EpipolarInput&,const DiagnosticPair&,
        const TrackingDepthPose& previous,const TrackingDepthPose& current,const Residual3DConfig&);
    Residual3DResult Process(const EpipolarInput&,const DiagnosticPair&,const EpipolarResult* epipolar=nullptr,const cv::Mat& image=cv::Mat(),bool rgb=false,const DepthRelativeMotion* motion=nullptr);
    void Reset() { sequence_=0; }
private:
    DepthRelativeMotion BackgroundPnP(const EpipolarInput&,const DiagnosticPair&);
    void Record(const EpipolarInput&,const DiagnosticPair&,const Residual3DResult&,const EpipolarResult*,const cv::Mat&,bool);
    Residual3DConfig config_;
    size_t sequence_=0;
    std::ofstream frames_,features_;
};
}
#endif
