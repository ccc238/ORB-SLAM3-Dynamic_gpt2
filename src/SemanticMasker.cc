#include "SemanticMasker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#ifdef ORB_SLAM3_WITH_ONNX
#include <onnxruntime_cxx_api.h>
#endif
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <stdexcept>
#include <sys/stat.h>
#include <cerrno>
#include <cstdlib>
namespace ORB_SLAM3 {
namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point a, Clock::time_point b) {return std::chrono::duration<double,std::milli>(b-a).count();}
void Require(bool ok, const std::string& msg) {if(!ok) throw std::runtime_error("SemanticMask: "+msg);}
template<class T> void ReadValue(cv::FileStorage& fs,const char* key,T& value) {if(!fs[key].empty()) fs[key] >> value;}
void ValidateConfig(const SemanticConfig& c) {
        Require(!c.model.empty(),"Model is required");
        Require(c.mode=="all" || c.mode=="selected","Mode must be all or selected");
        Require(c.mode!="selected" || !c.classes.empty(),"selected requires a nonempty Classes list");
        for(int id:c.classes) Require(id>=0 && id<80,"class_id must be in [0,79]");
        Require(std::isfinite(c.confidence) && c.confidence>=0 && c.confidence<=1,"invalid confidence");
        Require(std::isfinite(c.iou) && c.iou>=0 && c.iou<=1,"invalid NMS IoU");
        Require(c.maxDetections>0 && c.maxDetections<=8400 && c.dilation>=0 && c.dilation<=640,"invalid max detections or dilation radius");
}
cv::Mat RGB(const cv::Mat& image, bool isRGB) {
    Require(!image.empty() && image.depth()==CV_8U, "expected nonempty uint8 image");
    cv::Mat rgb;
    if(image.channels()==1) cv::cvtColor(image,rgb,cv::COLOR_GRAY2RGB);
    else if(image.channels()==3) {if(isRGB) rgb=image; else cv::cvtColor(image,rgb,cv::COLOR_BGR2RGB);}
    else if(image.channels()==4) cv::cvtColor(image,rgb,isRGB?cv::COLOR_RGBA2RGB:cv::COLOR_BGRA2RGB);
    else Require(false,"expected 1, 3 or 4 channels");
    return rgb;
}
}
void EnsureSemanticDirectory(const std::string& path) {
    Require(!path.empty(),"output directory is empty");
    for(size_t i=1;i<=path.size();++i) if(i==path.size() || path[i]=='/') {
        std::string part=path.substr(0,i);
        if(mkdir(part.c_str(),0755)!=0 && errno!=EEXIST) throw std::runtime_error("Cannot create directory: "+part);
        struct stat st; Require(stat(part.c_str(),&st)==0 && S_ISDIR(st.st_mode),"not a directory: "+part);
    }
}
std::string DefaultSemanticModelPath() {
    const char* homeDirectory=std::getenv("HOME");
    return homeDirectory && *homeDirectory ? std::string(homeDirectory)+"/models/yolo/yolo11s-seg.onnx" : "models/yolo/yolo11s-seg.onnx";
}
SemanticConfig SemanticConfig::Read(const std::string& settings, bool defaultEnabled) {
    SemanticConfig c;
    cv::FileStorage fs(settings,cv::FileStorage::READ);
    Require(fs.isOpened(),"cannot read settings: "+settings);
    int enabled=defaultEnabled?1:0,debug=0;
    ReadValue(fs,"SemanticMask.Enabled",enabled); ReadValue(fs,"SemanticMask.Debug",debug);
    c.enabled=enabled!=0; c.debug=debug!=0;
    ReadValue(fs,"SemanticMask.Model",c.model); ReadValue(fs,"SemanticMask.Mode",c.mode);
    ReadValue(fs,"SemanticMask.Classes",c.classes); ReadValue(fs,"SemanticMask.Confidence",c.confidence);
    ReadValue(fs,"SemanticMask.NmsIoU",c.iou); ReadValue(fs,"SemanticMask.MaxDetections",c.maxDetections);
    ReadValue(fs,"SemanticMask.DilationRadius",c.dilation); ReadValue(fs,"SemanticMask.OutputDirectory",c.output);
    if(c.enabled && c.mode=="selected" && fs["SemanticMask.Classes"].empty())
        std::cout << "SemanticMask: Classes not specified; using default [0:person]." << std::endl;
    if(c.enabled) ValidateConfig(c);
    std::cout << "SemanticMask enabled=" << c.enabled << " model=" << c.model << " device=CPU synchronous mode=" << c.mode << " classes=[";
    for(int id:c.classes) std::cout << id << ":" << SemanticMasker::ClassName(id) << " ";
    std::cout << "] confidence=" << c.confidence << " nms_iou=" << c.iou << " max=" << c.maxDetections
              << " dilation_radius=" << c.dilation << " (square kernel; current SLAM image pixels) debug=" << c.debug << " output=" << c.output << std::endl;
    return c;
}
const char* SemanticMasker::ClassName(int id) {
    static const char* names[]={"person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};
    return id>=0 && id<80 ? names[id] : "invalid";
}
struct SemanticMasker::Impl {
    SemanticConfig config;
    std::ofstream log;
    size_t frame=0;
#ifdef ORB_SLAM3_WITH_ONNX
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING,"semantic_mask"};
    Ort::SessionOptions options;
    Ort::Session session{nullptr};
#endif
    explicit Impl(const SemanticConfig& c):config(c) {
        ValidateConfig(c);
#ifdef ORB_SLAM3_WITH_ONNX
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session=Ort::Session(env,c.model.c_str(),options);
        Require(session.GetInputCount()==1 && session.GetOutputCount()==2,"expected exactly 1 input and 2 outputs");
        Ort::AllocatorWithDefaultOptions allocator;
        const std::vector<std::string> names={"images","output0","output1"};
        const std::vector<std::vector<int64_t>> shapes={{1,3,640,640},{1,116,8400},{1,32,160,160}};
        for(int i=0;i<3;++i) {
            auto name=i==0?session.GetInputNameAllocated(0,allocator):session.GetOutputNameAllocated(i-1,allocator);
            auto type=i==0?session.GetInputTypeInfo(0):session.GetOutputTypeInfo(i-1);
            auto tensor=type.GetTensorTypeAndShapeInfo();
            std::cout << "SemanticMask tensor " << name.get() << " shape=[";
            for(auto d:tensor.GetShape()) std::cout << d << " ";
            std::cout << "] type=" << tensor.GetElementType() << std::endl;
            Require(name.get()==names[i] && tensor.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && tensor.GetShape()==shapes[i],"model tensor name/shape/type mismatch at "+names[i]);
        }
#else
        Require(false,"built without ONNX Runtime; configure -DWITH_ONNX_RUNTIME=ON");
#endif
        if(c.debug) {
            EnsureSemanticDirectory(c.output);
            log.open(c.output+"/frames.csv",std::ios::out|std::ios::trunc);
            Require(log.good(),"cannot open frames.csv");
            log << "frame,timestamp,instances,blocked_ratio,keypoints,candidates,rejected,pre_ms,infer_ms,post_ms,frame_ms,state,violations\n";
        }
    }
};
SemanticMasker::SemanticMasker(const SemanticConfig& c):impl_(new Impl(c)) {}
SemanticMasker::~SemanticMasker()=default;
SemanticResult SemanticMasker::Infer(const cv::Mat& image, bool inputIsRGB) {
#ifdef ORB_SLAM3_WITH_ONNX
    auto start=Clock::now();
    cv::Mat rgb=RGB(image,inputIsRGB);
    const double scale=std::min(640.0/image.cols,640.0/image.rows);
    int w=std::max(1,cvRound(image.cols*scale)), h=std::max(1,cvRound(image.rows*scale));
    int left=(640-w)/2, top=(640-h)/2;
    cv::Mat resized,padded;
    cv::resize(rgb,resized,cv::Size(w,h),0,0,cv::INTER_LINEAR);
    cv::copyMakeBorder(resized,padded,top,640-h-top,left,640-w-left,cv::BORDER_CONSTANT,cv::Scalar(114,114,114));
    std::vector<float> input(3*640*640);
    for(int y=0;y<640;++y) for(int x=0;x<640;++x) for(int c=0;c<3;++c)
        input[c*640*640+y*640+x]=padded.at<cv::Vec3b>(y,x)[c]/255.f;
    std::array<int64_t,4> shape={{1,3,640,640}};
    auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
    auto tensor=Ort::Value::CreateTensor<float>(memory,input.data(),input.size(),shape.data(),shape.size());
    const char* inputs[]={"images"}; const char* outputs[]={"output0","output1"};
    auto preEnd=Clock::now();
    auto values=impl_->session.Run(Ort::RunOptions{nullptr},inputs,&tensor,1,outputs,2);
    auto inferEnd=Clock::now();
    const std::vector<std::vector<int64_t>> expected={{1,116,8400},{1,32,160,160}};
    for(size_t i=0;i<2;++i) {
        Require(values[i].IsTensor(),"runtime output is not a tensor");
        auto info=values[i].GetTensorTypeAndShapeInfo();
        Require(info.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && info.GetShape()==expected[i],"runtime output shape/type mismatch");
    }
    const float* det=values[0].GetTensorData<float>();
    const float* proto=values[1].GetTensorData<float>();
    for(size_t i=0;i<116*8400;++i) Require(std::isfinite(det[i]),"nonfinite detection output");
    for(size_t i=0;i<32*160*160;++i) Require(std::isfinite(proto[i]),"nonfinite prototype output");
    struct Candidate {int index,id; float score; cv::Rect2f box;};
    std::vector<Candidate> candidates, kept;
    const auto& config=impl_->config;
    for(int n=0;n<8400;++n) {
        int id=0; float score=det[4*8400+n];
        for(int c=1;c<80;++c) if(det[(4+c)*8400+n]>score) {score=det[(4+c)*8400+n]; id=c;}
        if(score<config.confidence) continue;
        float bw=det[2*8400+n],bh=det[3*8400+n];
        if(bw<=0 || bh<=0) continue;
        candidates.push_back({n,id,score,cv::Rect2f(det[n]-bw/2,det[8400+n]-bh/2,bw,bh)});
    }
    std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.score>b.score;});
    for(const auto& a:candidates) {
        bool suppressed=false;
        for(const auto& b:kept) if(a.id==b.id) {
            float intersection=(a.box & b.box).area();
            if(intersection/(a.box.area()+b.box.area()-intersection)>config.iou) {suppressed=true; break;}
        }
        if(!suppressed) kept.push_back(a);
        if(kept.size()>=static_cast<size_t>(config.maxDetections)) break;
    }
    SemanticResult result;
    result.blocked=cv::Mat::zeros(image.size(),CV_8UC1);
    cv::Mat prototypes(32,160*160,CV_32F,const_cast<float*>(proto));
    for(const auto& a:kept) {
        cv::Mat coeff(1,32,CV_32F);
        for(int c=0;c<32;++c) coeff.at<float>(0,c)=det[(84+c)*8400+a.index];
        cv::Mat logits=coeff*prototypes;
        cv::Mat probability=logits.reshape(1,160).clone();
        for(int y=0;y<160;++y) for(int x=0;x<160;++x) {
            float v=probability.at<float>(y,x);
            probability.at<float>(y,x)=v>=0 ? 1.f/(1.f+std::exp(-v)) : std::exp(v)/(1.f+std::exp(v));
        }
        // Continuous probability: bilinear to input, remove exact integer padding,
        // bilinear to SLAM image, threshold once, then crop in original coordinates.
        cv::Mat inputMask, original, binary;
        cv::resize(probability,inputMask,cv::Size(640,640),0,0,cv::INTER_LINEAR);
        cv::resize(inputMask(cv::Rect(left,top,w,h)),original,image.size(),0,0,cv::INTER_LINEAR);
        cv::compare(original,0.5,binary,cv::CMP_GT);
        const double sx=static_cast<double>(w)/image.cols,sy=static_cast<double>(h)/image.rows;
        cv::Rect2f box((a.box.x-left)/sx,(a.box.y-top)/sy,a.box.width/sx,a.box.height/sy);
        box &= cv::Rect2f(0,0,image.cols,image.rows);
        cv::Mat mask=cv::Mat::zeros(image.size(),CV_8UC1);
        // Pixel-center crop, half-open bounds. No whole-box blocking.
        for(int y=std::max(0,static_cast<int>(std::ceil(box.y)));y<std::min(image.rows,static_cast<int>(std::ceil(box.y+box.height)));++y)
            for(int x=std::max(0,static_cast<int>(std::ceil(box.x)));x<std::min(image.cols,static_cast<int>(std::ceil(box.x+box.width)));++x)
                mask.at<uchar>(y,x)=binary.at<uchar>(y,x);
        if(config.mode=="all" || std::find(config.classes.begin(),config.classes.end(),a.id)!=config.classes.end())
            cv::bitwise_or(result.blocked,mask,result.blocked);
        result.instances.push_back({box,a.id,a.score,mask});
    }
    if(config.dilation>0) cv::dilate(result.blocked,result.blocked,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*config.dilation+1,2*config.dilation+1)));
    result.preprocessMs=Ms(start,preEnd); result.inferenceMs=Ms(preEnd,inferEnd); result.postprocessMs=Ms(inferEnd,Clock::now());
    return result;
#else
    throw std::runtime_error("SemanticMask: ONNX Runtime unavailable");
#endif
}
void SemanticMasker::Record(const cv::Mat& image,bool isRGB,const SemanticResult& r,const std::vector<cv::KeyPoint>& keys,double timestamp,int state,size_t candidates,size_t rejected,double frameMs) {
    size_t violations=0;
    for(const auto& k:keys) {
        int x=cvFloor(k.pt.x),y=cvFloor(k.pt.y);
        if(x<0 || x>=r.blocked.cols || y<0 || y>=r.blocked.rows || r.blocked.at<uchar>(y,x)) ++violations;
    }
    Require(violations==0,"retained keypoint center inside blocked mask");
    if(keys.size()<100 || state!=2) std::cerr << "SemanticMask timestamp=" << std::setprecision(16) << timestamp << " remaining features=" << keys.size() << " tracking_state=" << state << " (mask retained)" << std::endl;
    if(!impl_->config.debug) return;
    double ratio=cv::countNonZero(r.blocked)/static_cast<double>(r.blocked.total());
    auto& log=impl_->log;
    log << impl_->frame << ',' << std::setprecision(16) << timestamp << ',' << r.instances.size() << ',' << ratio << ',' << keys.size() << ',' << candidates << ',' << rejected << ',' << r.preprocessMs << ',' << r.inferenceMs << ',' << r.postprocessMs << ',' << frameMs << ',' << state << ',' << violations << '\n';
    log.flush(); Require(log.good(),"failed to write debug CSV");
    cv::Mat bgr; cv::cvtColor(RGB(image,isRGB),bgr,cv::COLOR_RGB2BGR);
    cv::Mat tint=bgr.clone(),overlay,features;
    tint.setTo(cv::Scalar(0,0,255),r.blocked);
    cv::addWeighted(bgr,0.6,tint,0.4,0,overlay);
    for(const auto& instance:r.instances) {
        cv::rectangle(overlay,instance.box,cv::Scalar(0,255,255),1);
        cv::putText(overlay,std::string(ClassName(instance.classId))+" "+std::to_string(instance.confidence).substr(0,4),instance.box.tl(),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,255,255),1);
    }
    cv::drawKeypoints(overlay,keys,features,cv::Scalar(0,255,0));
    std::string base=impl_->config.output+"/"+std::to_string(impl_->frame++);
    Require(cv::imwrite(base+"_regions.png",overlay) && cv::imwrite(base+"_keypoints.png",features) && cv::imwrite(base+"_blocked.png",r.blocked),"failed to write debug images");
}
}
