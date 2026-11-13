#ifndef ORB_SLAM3_EPIPOLAR_MOTION_DETECTOR_H
#define ORB_SLAM3_EPIPOLAR_MOTION_DETECTOR_H
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <fstream>
#include <limits>
#include <string>
#include <vector>
namespace ORB_SLAM3 {
struct EpipolarConfig {
    int configuredValue=1; // User convention: 0=启用，1=关闭. Startup only.
    bool enabled=false, debug=false, visualize=false, drawMatches=true;
    bool lkAssistedMatching=false;
    int every=1, minBackground=30, minInliers=20;
    int lkWindow=21, lkMaxLevel=3, lkSearchRadius=8, lkMaxDescriptorDistance=72;
    double ratio=0.75, ransacPx=1.5, confidence=0.99, minInlierRatio=0.5;
    double lkMaxForwardBackward=1.5, lkDescriptorRatio=0.9;
    double lowPx=1, highPx=3, maxDt=0.2, minCoverage=0.15;
    double minDisplacementPx=0.5, homographyRatio=0.9;
    std::string output="outputs/epipolar";
    static EpipolarConfig Read(cv::FileStorage& fs, bool segmentation, bool association, bool masking);
};
enum class EpipolarState { UNKNOWN, GEOMETRY_CONSISTENT, SUSPECTED_DYNAMIC };
const char* EpipolarStateName(EpipolarState state);
struct EpipolarFeature {
    int previousIndex=-1, previousInstance=-1;
    double residual=std::numeric_limits<double>::quiet_NaN();
    EpipolarState state=EpipolarState::UNKNOWN;
    std::string reason="NO_MATCH";
    // -1: not in background set, 0/1: background RANSAC outlier/inlier.
    int backgroundInlier=-1;
    std::string matchSource="NONE";
};
struct EpipolarInput {
    std::vector<cv::KeyPoint> rawKeys, undistortedKeys;
    cv::Mat descriptors, K, distortion;
    std::vector<int> instances;
    std::vector<float> depths; // metres, original feature indexing
    std::vector<std::string> depthReasons;
    cv::Size size;
    unsigned long frameId=0;
    double timestamp=0;
};
struct EpipolarResult {
    std::vector<EpipolarFeature> features; // Always current raw feature indexing.
    long previousFrameId=-1;
    double dt=std::numeric_limits<double>::quiet_NaN();
    cv::Mat F; // x_current^T F x_previous = 0; undistorted pixel coordinates.
    bool modelValid=false;
    std::string reason="NO_PREVIOUS_FRAME";
    int matches=0, background=0, inliers=0;
    double inlierRatio=0, coverage=0, medianDisplacement=0, homographySupport=0;
    int assistedMatches=0;
    double matchMs=0, estimateMs=0, residualMs=0;
};
struct DiagnosticPair {
    EpipolarInput previous;
    EpipolarResult matching;
};
class DiagnosticMatcher {
public:
    DiagnosticMatcher();
    DiagnosticPair Process(const EpipolarInput& current, double ratio, double maxDt);
    DiagnosticPair Process(const EpipolarInput& current, double ratio, double maxDt, const cv::Mat& gray);
    void Configure(const EpipolarConfig& config) { config_=config; }
    void Reset() { previous_=EpipolarInput(); previousGray_.release(); havePrevious_=false; }
private:
    EpipolarConfig config_;
    EpipolarInput previous_;
    cv::Mat previousGray_;
    bool havePrevious_=false;
};
class EpipolarMotionDetector {
public:
    explicit EpipolarMotionDetector(const EpipolarConfig& config);
    EpipolarResult Process(const EpipolarInput& current, const cv::Mat& image=cv::Mat(), bool inputIsRGB=false, const DiagnosticPair* pair=nullptr);
    void Reset();
    static double Residual(const cv::Mat& F, cv::Point2d previous, cv::Point2d current);
private:
    EpipolarResult Evaluate(const EpipolarInput& current, const DiagnosticPair* pair);
    void Record(const EpipolarInput& current, const cv::Mat& image, bool rgb, const EpipolarResult& result);
    EpipolarConfig config_;
    EpipolarInput previous_;
    bool havePrevious_=false;
    size_t sequence_=0;
    cv::Mat previousImage_;
    std::ofstream frames_, features_;
};
}
#endif
