#ifndef ORB_SLAM3_SEMANTIC_MASKER_H
#define ORB_SLAM3_SEMANTIC_MASKER_H
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <string>
#include <vector>
namespace ORB_SLAM3 {
std::string DefaultSemanticModelPath();
struct SemanticConfig {
    bool enabled=true, debug=false;
    std::string model=DefaultSemanticModelPath(), mode="selected", output="outputs/semantic";
    std::vector<int> classes{0};
    float confidence=0.25f, iou=0.7f;
    int maxDetections=300, dilation=0;
    static SemanticConfig Read(const std::string& settings, bool defaultEnabled=true);
};
struct SemanticInstance {
    cv::Rect2f box; // Clipped, in the current SLAM image coordinates.
    int classId;
    float confidence;
    cv::Mat mask; // CV_8UC1, full image size, 255 = instance.
};
struct SemanticResult {
    std::vector<SemanticInstance> instances; // All detections after class-wise NMS.
    cv::Mat blocked; // Selected instances' union, 255 = blocked.
    double preprocessMs=0, inferenceMs=0, postprocessMs=0;
};
class SemanticMasker {
public:
    explicit SemanticMasker(const SemanticConfig& config);
    ~SemanticMasker();
    SemanticResult Infer(const cv::Mat& image, bool inputIsRGB);
    // Checks the invariant even when debug output is disabled. State is Tracking::mState.
    void Record(const cv::Mat& image, bool inputIsRGB, const SemanticResult& result,
                const std::vector<cv::KeyPoint>& keys, double timestamp, int state,
                size_t candidates, size_t rejected, double frameMs);
    static const char* ClassName(int classId);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
// Shared by example and debug writer; throws on filesystem errors.
void EnsureSemanticDirectory(const std::string& path);
}
#endif
