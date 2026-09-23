// =====================================================================
// yolo_trt.cpp — YOLOv11 TensorRT dedektoru.
//
// MOTOR DOSYASI IKI BICIMDE OLABILIR:
//   1) ULTRALYTICS export'u: [4 bayt uzunluk][JSON metadata][TRT plani]
//      (models/UAV-YOLOv11m*.engine boyle; ilk baytlar: 'a\x02\x00\x00{"desc...')
//   2) trtexec / ham plan: dogrudan 'ftrt' sihirli baytlariyla baslar
//      (models/ORTrack_ep0300-fp16.engine boyle)
// Ikisi de otomatik taninir. Duz deserialize denemek (1) icin BASARISIZ olur.
//
// OLCUM (kendi benchmark loglarinizdan, Orin, detector_inference_ms ortalama):
//   PyTorch fp16 512: 41.2   -> TensorRT fp16 512: 25.5   (oran ~0.62)
//   PyTorch fp16 768: 68.6   -> TensorRT fp16 768: ~42 TAHMIN (olculmedi)
// trtexec ciktisindaki GPU Compute Time gercek degeri verir.
// =====================================================================
#include "yolo_trt.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace hybrid {
namespace {

using Clock = std::chrono::steady_clock;
inline double msSince(const Clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

class YLogger : public nvinfer1::ILogger {
public:
    explicit YLogger(bool v) : v_(v) {}
    void log(Severity s, const char* m) noexcept override {
        if (s == Severity::kINTERNAL_ERROR || s == Severity::kERROR) std::cerr << "[YOLO][TRT-HATA] " << m << "\n";
        else if (s == Severity::kWARNING) std::cerr << "[YOLO][TRT-UYARI] " << m << "\n";
        else if (v_) std::cout << "[YOLO][TRT] " << m << "\n";
    }
private:
    bool v_;
};

template <class T> struct TrtDel { void operator()(T* p) const noexcept { delete p; } };
template <class T> using TrtPtr = std::unique_ptr<T, TrtDel<T>>;

inline bool cudaOk(cudaError_t e, const char* what, std::string& err) {
    if (e == cudaSuccess) return true;
    err = std::string("CUDA hatasi (") + what + "): " + cudaGetErrorString(e);
    return false;
}

// Ultralytics basligini atla. Doner: plan verisinin baslangic ofseti.
size_t planOffset(const std::vector<char>& buf, std::string& note)
{
    if (buf.size() >= 8 && std::memcmp(buf.data(), "ftrt", 4) == 0) {
        note = "ham TensorRT plani";
        return 0;
    }
    if (buf.size() >= 8) {
        uint32_t n = 0;
        std::memcpy(&n, buf.data(), 4);                 // little-endian
        if (n > 0 && n < 8192 && (size_t)4 + n + 8 <= buf.size() && buf[4] == '{') {
            const size_t off = 4 + (size_t)n;
            if (std::memcmp(buf.data() + off, "ftrt", 4) == 0) {
                note = "ultralytics export (basliк " + std::to_string(n) + " bayt atlandi)";
                return off;
            }
        }
    }
    note = "bilinmeyen bicim; ham plan olarak denenecek";
    return 0;
}

class YoloTrt : public YoloDetector {
public:
    explicit YoloTrt(const YoloOptions& o) : opt_(o), logger_(o.verbose) {}
    ~YoloTrt() override {
        ctx_.reset(); engine_.reset(); runtime_.reset();
        if (stream_) cudaStreamDestroy(stream_);
        if (dIn_)  cudaFree(dIn_);
        if (dOut_) cudaFree(dOut_);
        if (hIn_)  cudaFreeHost(hIn_);
        if (hOut_) cudaFreeHost(hOut_);
    }

    bool open(std::string& err);

    bool detect(const cv::Mat& bgr, std::vector<Detection>& out) override;
    int    inputSize() const override { return size_; }
    double lastMs()    const override { return msTotal_; }
    double inferMs()   const override { return msInfer_; }

private:
    YoloOptions opt_;
    YLogger     logger_;
    TrtPtr<nvinfer1::IRuntime>          runtime_;
    TrtPtr<nvinfer1::ICudaEngine>       engine_;
    TrtPtr<nvinfer1::IExecutionContext> ctx_;
    cudaStream_t stream_ = nullptr;

    std::string inName_, outName_;
    int   size_ = 0, nc_ = 0, anchors_ = 0;
    size_t inElems_ = 0, outElems_ = 0;
    float *dIn_ = nullptr, *dOut_ = nullptr, *hIn_ = nullptr, *hOut_ = nullptr;

    YoloNormalizeLUT lut_;
    cv::Mat          lb_;
    double msTotal_ = 0.0, msInfer_ = 0.0;
};

bool YoloTrt::open(std::string& err)
{
    if (!cudaOk(cudaSetDevice(opt_.device), "cudaSetDevice", err)) return false;

    std::ifstream f(opt_.enginePath, std::ios::binary | std::ios::ate);
    if (!f) { err = "YOLO motoru acilamadi: " + opt_.enginePath; return false; }
    const std::streamsize n = f.tellg();
    if (n <= 0) { err = "YOLO motoru bos: " + opt_.enginePath; return false; }
    f.seekg(0, std::ios::beg);
    std::vector<char> buf((size_t)n);
    if (!f.read(buf.data(), n)) { err = "YOLO motoru okunamadi"; return false; }

    std::string note;
    const size_t off = planOffset(buf, note);
    std::cout << "[YOLO] motor bicimi: " << note << "\n";

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) { err = "TensorRT runtime olusturulamadi"; return false; }
    engine_.reset(runtime_->deserializeCudaEngine(buf.data() + off, buf.size() - off));
    if (!engine_) {
        err = "YOLO motoru deserialize EDILEMEDI: " + opt_.enginePath +
              "\n       TensorRT plani cihaza + surume ozeldir, kopyalanamaz."
              "\n       Cozum: scripts/build_yolo_engine.sh ile ONNX'ten bu Orin'de uretin.";
        return false;
    }
    ctx_.reset(engine_->createExecutionContext());
    if (!ctx_) { err = "TensorRT context olusturulamadi"; return false; }

    // ---- baglantilari motordan OKU (boyutu varsayma)
    const int nio = engine_->getNbIOTensors();
    if (nio != 2) {
        err = "YOLO motoru " + std::to_string(nio) +
              " tensor bildiriyor, 2 bekleniyordu (images, output0)."
              "\n       NMS gomulu (end2end) bir export kullanilmis olabilir; bu kod ham ciktiyi bekler.";
        return false;
    }
    for (int i = 0; i < nio; ++i) {
        const char* nm = engine_->getIOTensorName(i);
        (engine_->getTensorIOMode(nm) == nvinfer1::TensorIOMode::kINPUT ? inName_ : outName_) = nm;
        if (engine_->getTensorDataType(nm) != nvinfer1::DataType::kFLOAT) {
            err = std::string("YOLO tensoru '") + nm + "' FP32 degil."
                  "\n       --fp16 kurun ama G/C tiplerini fp32 birakin.";
            return false;
        }
    }
    const nvinfer1::Dims di = engine_->getTensorShape(inName_.c_str());
    const nvinfer1::Dims dou = engine_->getTensorShape(outName_.c_str());
    if (di.nbDims != 4 || di.d[0] != 1 || di.d[1] != 3 || di.d[2] != di.d[3]) {
        err = "YOLO girisi (1,3,S,S) bicimi degil (dinamik/batch motor desteklenmiyor)."; return false;
    }
    if (dou.nbDims != 3 || dou.d[0] != 1) {
        err = "YOLO ciktisi (1, 4+nc, N) bicimi degil."; return false;
    }
    size_    = (int)di.d[2];
    nc_      = (int)dou.d[1] - 4;
    anchors_ = (int)dou.d[2];
    if (nc_ != kYoloNumClasses) {
        err = "YOLO motoru " + std::to_string(nc_) + " sinif bildiriyor, " +
              std::to_string(kYoloNumClasses) + " bekleniyordu "
              "(airplane, bird, drone, helicopter). Yanlis model dosyasi.";
        return false;
    }
    // Cikti izgara sayisi girdi boyutuyla tutarli mi (stride 8/16/32)?
    const int g = size_ / 8, expect = g * g + (g / 2) * (g / 2) + (g / 4) * (g / 4);
    if (anchors_ != expect) {
        err = "YOLO cikti izgarasi " + std::to_string(anchors_) + ", " +
              std::to_string(expect) + " bekleniyordu (giris " + std::to_string(size_) + ").";
        return false;
    }

    inElems_  = (size_t)3 * size_ * size_;
    outElems_ = (size_t)(4 + nc_) * anchors_;

    if (!cudaOk(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "streamCreate", err)) return false;
    if (!cudaOk(cudaMalloc((void**)&dIn_,  inElems_  * sizeof(float)), "cudaMalloc in",  err)) return false;
    if (!cudaOk(cudaMalloc((void**)&dOut_, outElems_ * sizeof(float)), "cudaMalloc out", err)) return false;
    if (!cudaOk(cudaHostAlloc((void**)&hIn_,  inElems_  * sizeof(float), cudaHostAllocDefault), "hostAlloc in",  err)) return false;
    if (!cudaOk(cudaHostAlloc((void**)&hOut_, outElems_ * sizeof(float), cudaHostAllocDefault), "hostAlloc out", err)) return false;

    if (!ctx_->setTensorAddress(inName_.c_str(),  dIn_) ||
        !ctx_->setTensorAddress(outName_.c_str(), dOut_)) {
        err = "YOLO tensor adresleri baglanamadi"; return false;
    }

    // Isinma: ilk cagri her zaman yavas. Ucus oncesi odenir.
    if (!cudaOk(cudaMemsetAsync(dIn_, 0, inElems_ * sizeof(float), stream_), "memset in", err)) return false;
    for (int i = 0; i < opt_.warmupIters; ++i)
        if (!ctx_->enqueueV3(stream_)) { err = "YOLO isinma cagrisi basarisiz"; return false; }
    if (!cudaOk(cudaStreamSynchronize(stream_), "isinma sync", err)) return false;

    std::cout << "[YOLO] giris " << size_ << "x" << size_
              << "  sinif " << nc_ << "  izgara " << anchors_ << "\n";
    return true;
}

bool YoloTrt::detect(const cv::Mat& bgr, std::vector<Detection>& out)
{
    const auto tAll = Clock::now();
    out.clear();

    const LetterboxInfo li = letterbox(bgr, size_, lb_);
    if (!li.ok) { msTotal_ = msSince(tAll); return false; }
    lut_.apply(lb_, hIn_);

    const auto tInf = Clock::now();
    if (cudaMemcpyAsync(dIn_, hIn_, inElems_ * sizeof(float),
                        cudaMemcpyHostToDevice, stream_) != cudaSuccess) { msTotal_ = msSince(tAll); return false; }
    if (!ctx_->enqueueV3(stream_)) { msTotal_ = msSince(tAll); return false; }
    if (cudaMemcpyAsync(hOut_, dOut_, outElems_ * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_) != cudaSuccess) { msTotal_ = msSince(tAll); return false; }
    if (cudaStreamSynchronize(stream_) != cudaSuccess) { msTotal_ = msSince(tAll); return false; }
    msInfer_ = msSince(tInf);

    yoloPostprocess(hOut_, nc_, anchors_, opt_.confidence, opt_.nmsIoU,
                    opt_.maxDet, opt_.keepClass, li, out);
    msTotal_ = msSince(tAll);
    return true;
}

} // namespace

std::unique_ptr<YoloDetector> makeYoloTensorRT(const YoloOptions& opt, std::string& err)
{
    std::unique_ptr<YoloTrt> d(new YoloTrt(opt));
    if (!d->open(err)) return nullptr;
    return std::unique_ptr<YoloDetector>(d.release());
}

} // namespace hybrid
