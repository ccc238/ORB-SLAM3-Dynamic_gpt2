#include "FrameDrawer.h"
#include "SemanticMasker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/freetype.hpp>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdexcept>

namespace ORB_SLAM3 {
namespace {
std::string Csv(const std::string& s) {
    std::string out="\"";for(char c:s){if(c=='\"')out+='\"';out+=c;}return out+'\"';
}
std::string Number(bool available,size_t count){return available?std::to_string(count):"N/A";}
void Text(cv::Mat& canvas,const std::string& s,cv::Point p,cv::Scalar color=cv::Scalar(235,235,235)) {
    // A renderer per calling thread: the viewer and tracking file writer can run concurrently.
    struct Fonts {
        cv::Ptr<cv::freetype::FreeType2> chinese=cv::freetype::createFreeType2();
        cv::Ptr<cv::freetype::FreeType2> latin=cv::freetype::createFreeType2();
        Fonts() {
            chinese->loadFontData("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",0);
            latin->loadFontData("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",0);
        }
    };
    thread_local Fonts fonts;
    // DroidSansFallback deliberately lacks Latin digits/letters. Split UTF-8 runs so both
    // Chinese labels and numeric measurements render as glyphs instead of empty boxes.
    for(size_t begin=0;begin<s.size();) {
        const bool latin=static_cast<unsigned char>(s[begin])<128;
        size_t end=begin+1;
        while(end<s.size()&&(static_cast<unsigned char>(s[end])<128)==latin)++end;
        const std::string run=s.substr(begin,end-begin);
        auto font=latin?fonts.latin:fonts.chinese;
        font->putText(canvas,run,p,16,color,-1,cv::LINE_AA,true);
        int baseline=0;p.x+=font->getTextSize(run,16,-1,&baseline).width;
        begin=end;
    }
}
void Validate(const DynamicFilterView& v) {
    if(v.candidates.size()!=v.keys.size()||v.excludedObservations.size()!=v.keys.size())
        throw std::runtime_error("Filter view: inconsistent original feature array lengths");
    std::set<int> before(v.beforeIndices.begin(),v.beforeIndices.end()),after(v.afterIndices.begin(),v.afterIndices.end());
    if(before.size()!=v.beforeIndices.size()||after.size()!=v.afterIndices.size())
        throw std::runtime_error("Filter view: duplicate optimizer index");
    for(int i:before)if(i<0||size_t(i)>=v.keys.size())throw std::runtime_error("Filter view: invalid edge index");
    if(v.qualificationAvailable) {
        if(v.afterIndices!=v.finalEdgeIndices)throw std::runtime_error("Filter view: actual final edge mismatch");
        for(size_t i=0;i<v.keys.size();++i) {
            bool excluded=v.excludedObservations[i];
            if(excluded&&(!v.accepted||!v.enabled||!v.candidates[i]||!before.count(i)))
                throw std::runtime_error("Filter view: invalid effective exclusion");
            if(bool(after.count(i))!=(bool(before.count(i))&&!excluded))
                throw std::runtime_error("Filter view: inconsistent frozen eligibility snapshot");
        }
    } else if(v.ExcludedCount())throw std::runtime_error("Filter view: exclusion without measured eligibility");
    if(v.optimizationAvailable)for(int i:v.finalInlierIndices)
        if(!after.count(i))throw std::runtime_error("Filter view: inlier without final edge");
}
}

void FrameDrawer::ConfigureDynamicFilterView(const std::string& output,int saveEvery) {
    std::lock_guard<std::mutex> writeLock(mDynamicWriterMutex);
    mDynamicOutput=output;mDynamicSaveEvery=saveEvery;
    if(output.empty())return;
    EnsureSemanticDirectory(output);
    mDynamicFrames.open(output+"/frames.csv");mDynamicObservations.open(output+"/observations.csv");
    if(!mDynamicFrames||!mDynamicObservations)throw std::runtime_error("Cannot open dynamic filter view CSV directory: "+output);
    mDynamicFrames<<"frame_id,timestamp,enabled,orb_total,candidates,observations_before,observations_after,actual_excluded,final_inliers,applied,executed,status,reason,optimization_available\n";
    mDynamicObservations<<"frame_id,timestamp,index,x,y,candidate,effective_excluded,eligible_before,eligible_after,final_edge,final_inlier\n";
}

bool FrameDrawer::GetDynamicFilterSnapshot(cv::Mat& image,DynamicFilterView& view) {
    std::lock_guard<std::mutex> lock(mDynamicMutex);
    if(!mbDynamicReady)return false;
    image=mDynamicRGB.clone();view=mDynamicView;return true;
}

void FrameDrawer::UpdateDynamicFilterFrame(const cv::Mat& image,bool inputIsRGB,const DynamicFilterView& view) {
    if(image.empty())return;
    Validate(view);
    cv::Mat bgr;
    if(image.channels()==1)cv::cvtColor(image,bgr,cv::COLOR_GRAY2BGR);
    else if(image.channels()==3){if(inputIsRGB)cv::cvtColor(image,bgr,cv::COLOR_RGB2BGR);else bgr=image.clone();}
    else if(image.channels()==4)cv::cvtColor(image,bgr,inputIsRGB?cv::COLOR_RGBA2BGR:cv::COLOR_BGRA2BGR);
    else throw std::runtime_error("Filter view: unsupported RGB image channels");
    std::lock_guard<std::mutex> writeLock(mDynamicWriterMutex);
    if(mDynamicFrames) {
        mDynamicFrames<<view.frameId<<','<<std::fixed<<std::setprecision(9)<<view.timestamp<<','<<view.enabled<<','
            <<view.keys.size()<<','<<view.CandidateCount()<<','<<Number(view.qualificationAvailable,view.beforeIndices.size())<<','
            <<Number(view.qualificationAvailable,view.afterIndices.size())<<','<<view.ExcludedCount()<<','
            <<Number(view.optimizationAvailable,view.finalInlierIndices.size())<<','<<view.accepted<<','<<view.executed<<','<<view.status<<','<<Csv(view.reason)<<','<<view.optimizationAvailable<<'\n';
        const std::set<int> before(view.beforeIndices.begin(),view.beforeIndices.end()),after(view.afterIndices.begin(),view.afterIndices.end()),
            edges(view.finalEdgeIndices.begin(),view.finalEdgeIndices.end()),inliers(view.finalInlierIndices.begin(),view.finalInlierIndices.end());
        for(size_t i=0;i<view.keys.size();++i) {
            const auto p=view.keys[i].pt;
            mDynamicObservations<<view.frameId<<','<<std::fixed<<std::setprecision(9)<<view.timestamp<<','<<i<<','<<p.x<<','<<p.y<<','
                <<view.candidates[i]<<','<<view.excludedObservations[i]<<','
                <<(view.qualificationAvailable?std::to_string(before.count(i)):"N/A")<<','
                <<(view.qualificationAvailable?std::to_string(after.count(i)):"N/A")<<','
                <<(view.qualificationAvailable?std::to_string(edges.count(i)):"N/A")<<','
                <<(view.optimizationAvailable?std::to_string(inliers.count(i)):"N/A")<<'\n';
        }
        mDynamicFrames.flush();mDynamicObservations.flush();
        if(mDynamicSaveEvery>0&&(view.frameId%mDynamicSaveEvery==0||view.CandidateCount()>0||!view.optimizationAvailable)) {
            std::ostringstream name;name<<mDynamicOutput<<"/frame_"<<view.frameId<<'_'<<std::fixed<<std::setprecision(6)<<view.timestamp<<".png";
            if(!cv::imwrite(name.str(),RenderDynamicFilterFrame(bgr,view)))throw std::runtime_error("Cannot save filter view screenshot");
        }
    }
    // Publish image and all numeric/feature state in one critical section, only after processing this frame.
    std::lock_guard<std::mutex> lock(mDynamicMutex);
    mDynamicRGB=bgr;mDynamicView=view;mbDynamicReady=true;
}

cv::Mat FrameDrawer::RenderDynamicFilterFrame(const cv::Mat& bgr,const DynamicFilterView& v,float scale) {
    Validate(v);
    cv::Mat canvas(bgr.rows+222,std::max(bgr.cols,680),CV_8UC3,cv::Scalar(22,22,22));
    bgr.copyTo(canvas(cv::Rect(0,0,bgr.cols,bgr.rows)));
    // Coordinates come from mvKeys on this exact RGB image, not undistorted mvKeysUn.
    for(size_t i=0;i<v.keys.size();++i)if(!v.candidates[i])cv::circle(canvas,v.keys[i].pt,1,{150,150,150},-1);
    for(size_t i=0;i<v.keys.size();++i)if(v.candidates[i])cv::circle(canvas,v.keys[i].pt,5,{0,255,255},1,cv::LINE_AA);
    for(size_t i=0;i<v.keys.size();++i)if(v.excludedObservations[i])cv::drawMarker(canvas,v.keys[i].pt,{0,0,255},cv::MARKER_TILTED_CROSS,10,2,cv::LINE_AA);
    int y=bgr.rows+23;
    std::ostringstream top;top<<"Frame="<<v.frameId<<"  t="<<std::fixed<<std::setprecision(6)<<v.timestamp<<"  enabled="<<v.enabled<<"  applied="<<v.accepted;
    Text(canvas,top.str(),{8,y});y+=25;
    Text(canvas,"ORB 总数="+std::to_string(v.keys.size())+"   疑似动态候选="+std::to_string(v.CandidateCount()),{8,y});y+=25;
    Text(canvas,"优化观测: 前="+Number(v.qualificationAvailable,v.beforeIndices.size())+"  后="+Number(v.qualificationAvailable,v.afterIndices.size())+"  实际排除="+std::to_string(v.ExcludedCount()),{8,y});y+=25;
    Text(canvas,"最终优化内点="+Number(v.optimizationAvailable,v.finalInlierIndices.size())+"   applied="+std::to_string(v.accepted),{8,y});y+=25;
    const std::string status=!v.enabled?"过滤关闭，实际排除=0":v.accepted?"过滤已接受，实际排除="+std::to_string(v.ExcludedCount()):v.status=="ROLLED_BACK"?"已回退，实际排除=0":"未应用，实际排除=0";
    Text(canvas,status,{8,y});y+=25;
    Text(canvas,v.status+": "+v.reason,{8,y});y+=25;
    cv::circle(canvas,{13,y-5},2,{150,150,150},-1);Text(canvas,"灰点: 非候选（未确认静态）",{25,y});
    cv::circle(canvas,{330,y-5},5,{0,255,255},1);Text(canvas,"黄圈: 疑似动态候选",{342,y});y+=25;
    cv::drawMarker(canvas,{13,y-5},{0,0,255},cv::MARKER_TILTED_CROSS,10,2);Text(canvas,"红叉: 实际退出最终优化的观测；N/A: 无测量结果",{25,y});
    if(scale<=0)throw std::runtime_error("Filter view: invalid display scale");
    if(scale!=1.f)cv::resize(canvas,canvas,cv::Size(),scale,scale,cv::INTER_LINEAR);
    return canvas;
}
}
