#include "DynamicFilterView.h"
#include "Frame.h"
#include <algorithm>
#include <set>
#include <stdexcept>
namespace ORB_SLAM3 {
int DynamicFilterView::CandidateCount() const {return std::count(candidates.begin(),candidates.end(),true);}
int DynamicFilterView::ExcludedCount() const {return std::count(excludedObservations.begin(),excludedObservations.end(),true);}
DynamicFilterView MakeDynamicFilterView(const Frame& f,const std::vector<int>& qualification,
    const PoseObservationAudit& initialAudit,bool enabled,
    bool accepted,bool executed,const std::string& reason) {
    DynamicFilterView v;
    v.frameId=f.mnId;v.timestamp=f.mTimeStamp;v.keys=f.mvKeys;
    v.enabled=enabled;v.accepted=accepted;v.executed=executed;v.reason=reason;
    v.status=accepted?"ACCEPTED":(reason.find("ROLLBACK")!=std::string::npos?"ROLLED_BACK":"SKIPPED");
    v.candidates.assign(v.keys.size(),false);v.excludedObservations.assign(v.keys.size(),false);
    if(enabled&&f.mvbDynamicCandidate.size()==v.keys.size())v.candidates=f.mvbDynamicCandidate;
    v.qualificationAvailable=initialAudit.constructed&&initialAudit.frameId==v.frameId&&initialAudit.timestamp==v.timestamp;
    if(v.qualificationAvailable) {
        v.beforeIndices=qualification;
        for(int i:qualification) {
            if(i<0||size_t(i)>=v.keys.size())throw std::runtime_error("Filter view: invalid qualification index");
            if(accepted&&f.DynamicExcluded(i))v.excludedObservations[i]=true;
            else v.afterIndices.push_back(i);
        }
        // No re-evaluation of map points, depth or outlier eligibility here.
        v.finalEdgeIndices=f.mvPoseOptimizationIndices;
        if(v.afterIndices!=v.finalEdgeIndices)
            throw std::runtime_error("Filter view: frozen eligibility/mask differs from actual optimizer edges");
    }
    v.optimizationAvailable=f.mPoseObservationAudit.resultAvailable&&
        f.mPoseObservationAudit.frameId==v.frameId&&f.mPoseObservationAudit.timestamp==v.timestamp;
    if(v.optimizationAvailable)v.finalInlierIndices=f.mPoseObservationAudit.inlierIndices;
    return v;
}
}
