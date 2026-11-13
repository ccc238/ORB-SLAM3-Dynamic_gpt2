#ifndef ORB_SLAM3_DYNAMIC_FILTER_VIEW_H
#define ORB_SLAM3_DYNAMIC_FILTER_VIEW_H
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <string>
#include <vector>
#include <limits>
namespace ORB_SLAM3 {
class Frame;
// Instrumentation only. A construction measurement is distinct from a solved pose.
struct PoseObservationAudit {
    long frameId=-1;
    double timestamp=std::numeric_limits<double>::quiet_NaN();
    bool constructed=false,resultAvailable=false;
    std::vector<int> inlierIndices;
};
// One value snapshot, indexed exactly like the original RGB-D raw keypoints.
struct DynamicFilterView {
    long frameId=-1;
    double timestamp=0;
    bool enabled=false,accepted=false,executed=false;
    bool qualificationAvailable=false,optimizationAvailable=false;
    std::string status="SKIPPED",reason="NO_POSE_OPTIMIZATION";
    std::vector<cv::KeyPoint> keys;
    std::vector<bool> candidates,excludedObservations;
    std::vector<int> beforeIndices,afterIndices,finalEdgeIndices,finalInlierIndices;
    int CandidateCount() const;
    int ExcludedCount() const;
};
DynamicFilterView MakeDynamicFilterView(const Frame&,const std::vector<int>& qualification,
    const PoseObservationAudit& initialAudit,bool enabled,
    bool accepted,bool executed,const std::string& reason);
}
#endif
