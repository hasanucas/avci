// =====================================================================
// yolo_trt.hpp — YOLOv11 TensorRT dedektorunun DIS yuzu.
// NvInfer.h ICERMEZ (tum TRT kodu yolo_trt.cpp icinde).
// =====================================================================
#pragma once

#include "yolo_core.hpp"
#include <memory>
#include <string>
#include <vector>

namespace hybrid {

struct YoloOptions {
    std::string enginePath;
    int   device      = 0;
    float confidence  = 0.35f;   // configs/orin-camera-quality-768.yaml
    float nmsIoU      = 0.5f;
    int   maxDet      = 300;
    bool  keepClass[kYoloNumClasses] = {false, false, true, false};  // sadece 'drone'
    bool  verbose     = false;
    int   warmupIters = 3;
};

struct YoloDetector {
    virtual ~YoloDetector() {}
    // bgr: orijinal kare. out: ORIJINAL kare koordinatlarinda tespitler.
    virtual bool detect(const cv::Mat& bgr, std::vector<Detection>& out) = 0;
    virtual int    inputSize() const = 0;   // motordan okunur (512 / 768 ...)
    virtual double lastMs()    const = 0;
    virtual double inferMs()   const = 0;
};

std::unique_ptr<YoloDetector> makeYoloTensorRT(const YoloOptions& opt, std::string& err);

} // namespace hybrid
