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
    // [SK] Hedef sinif. -1 = OTOMATIK: 4 sinifli UAV modeli -> 2 'drone'
    // (eski sabit keepClass {F,F,T,F} ile birebir), tek sinifli
    // yolo11s-plane -> 0. Baska sinif sayisinda --yolo-class N sart.
    // Filtre ARGMAX'TAN SONRA uygulanir (yoloDecode), yani davranis ayni.
    int   targetClass = -1;
    bool  verbose     = false;
    int   warmupIters = 3;
};

struct YoloDetector {
    virtual ~YoloDetector() {}
    // bgr: orijinal kare. out: ORIJINAL kare koordinatlarinda tespitler.
    virtual bool detect(const cv::Mat& bgr, std::vector<Detection>& out) = 0;
    virtual int    inputSize() const = 0;   // motordan okunur (512 / 768 ...) — dikdortgende GENISLIK
    virtual int    inputH()    const { return inputSize(); }   // [YO] 640x768 motor icin
    virtual int    inputW()    const { return inputSize(); }
    virtual double lastMs()    const = 0;
    virtual double inferMs()   const = 0;
    virtual int    numClasses()  const { return 0; }    // [SK] motordan okunan sinif sayisi
    virtual int    targetClass() const { return -1; }   // [SK] tutulan tek sinif
};

std::unique_ptr<YoloDetector> makeYoloTensorRT(const YoloOptions& opt, std::string& err);

} // namespace hybrid
