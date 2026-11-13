#ifndef ORB_SLAM3_SEMANTIC_MASKER_H
#define ORB_SLAM3_SEMANTIC_MASKER_H
#include "EpipolarMotionDetector.h"
#include "Residual3D.h"
#include "LKCheck.h"
#include "DynamicFilter.h"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <string>
#include <vector>
namespace ORB_SLAM3 {
std::string DefaultSemanticModelPath();
struct SemanticConfig {
    EpipolarConfig epipolar;
    Residual3DConfig residual3d;
    LKCheckConfig lkcheck;
    DynamicFilterConfig dynamicFilter;
    bool enabled=false, maskOnly=true, debug=false, visualize=true;
    bool maskingEnabled=true, associationEnabled=false, associationDebug=false;
    std::string model=DefaultSemanticModelPath(), mode="selected", output="outputs/semantic";
    std::vector<int> classes{0};
    float confidence=0.25f, iou=0.7f;
    int maxDetections=300, dilation=0;
    int personFeatureBoost=0, personFeatureErode=8;
    // Dynamic.* gates the existing online pipeline; absent parameters disable it.
    static SemanticConfig Read(const std::string& settings);
};
struct SemanticInstance {
    cv::Rect2f box; // Clipped, in the current SLAM image coordinates.
    int classId;
    float confidence;
    cv::Mat mask; // CV_8UC1, full image size, 255 = instance.
};
struct InstanceInfo {
    int instanceId, classId;
    std::string className;
    float confidence;
};
struct SemanticResult {
    std::vector<SemanticInstance> instances; // All detections after class-wise NMS.
    cv::Mat instanceLabels; // CV_16UC1, raw current SLAM RGB coordinates, never dilated.
    std::vector<InstanceInfo> instanceInfo; // Per-frame IDs, not class or tracking IDs.
    cv::Mat blocked; // Selected instances' union, 255 = blocked.
    double preprocessMs=0, inferenceMs=0, postprocessMs=0;
};
// Assign IDs in retained detection order; overlap belongs to highest confidence (stable ties).
void BuildInstanceLabels(SemanticResult& result, cv::Size size);
std::vector<int> QueryInstanceLabels(const cv::Mat& labels, cv::Size expectedSize,
    const std::vector<cv::KeyPoint>& rawKeys, const std::vector<InstanceInfo>& info);
cv::Mat DrawInstanceAssociations(const cv::Mat& image, bool inputIsRGB,
    const SemanticResult& result, const std::vector<cv::KeyPoint>& keys, const std::vector<int>& ids);
class SemanticMasker {
public:
    explicit SemanticMasker(const SemanticConfig& config);
    ~SemanticMasker();
    SemanticResult Infer(const cv::Mat& image, bool inputIsRGB);
    // Checks the invariant even when debug output is disabled. State is Tracking::mState.
    void Record(const cv::Mat& image, bool inputIsRGB, const SemanticResult& result,
                const std::vector<cv::KeyPoint>& keys, double timestamp, int state,
                size_t candidates, size_t rejected, double frameMs);
    bool MaskingEnabled() const;
    bool AssociationEnabled() const;
    void RecordAssociation(const cv::Mat& image, bool inputIsRGB, const SemanticResult& result,
        const std::vector<cv::KeyPoint>& keys, const std::vector<int>& ids,
        unsigned long frameId, double timestamp);
    static const char* ClassName(int classId);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
// Shared by example and debug writer; throws on filesystem errors.
void EnsureSemanticDirectory(const std::string& path);
}
#endif
