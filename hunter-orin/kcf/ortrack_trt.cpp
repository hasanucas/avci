// =====================================================================
// ortrack_trt.cpp — ORTrack (OSTrack/DeiT-tiny) TensorRT backend'i.
//
// Agin ILERI GECISI motorda; kirpma / normalize / Hann / cal_bbox / kutu
// esleme ortrack_core.hpp'de ve Python referansiyla test edilmis durumda
// (test_ortrack_offline, 1444/1444).
//
// OLCUM (kendi benchmark loglarinizdan, Orin, YOLO da koserken):
//     PyTorch fp32 : tracker_inference  41.9 ms  (p95 44.5)  -> UCUSTA KULLANILAMAZ
//     TensorRT fp16: tracker_inference   6.95 ms (p95  7.13) -> bu dosya
//
// MOTOR TASINABILIR DEGILDIR. Plan; GPU mimarisine, TensorRT surumune ve
// surucuye baglidir. Baska makinede uretilmis .engine deserialize edilemez
// (nullptr doner). Cozum: scripts/build_ortrack_engine.sh ile ONNX'ten
// HEDEF ORIN UZERINDE yeniden uret.
// =====================================================================
#include "ortrack_trt.hpp"
#include "ortrack_core.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace ortrack {
using hybrid::BoxF;
namespace {

// ------------------------------------------------------------- yardimcilar
using Clock = std::chrono::steady_clock;

inline double msSince(const Clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

class TrtLogger : public nvinfer1::ILogger {
public:
    explicit TrtLogger(bool verbose) : verbose_(verbose) {}
    void log(Severity s, const char* msg) noexcept override {
        if (s == Severity::kINTERNAL_ERROR || s == Severity::kERROR)
            std::cerr << "[ORT][TRT-HATA] " << msg << "\n";
        else if (s == Severity::kWARNING)
            std::cerr << "[ORT][TRT-UYARI] " << msg << "\n";
        else if (verbose_)
            std::cout << "[ORT][TRT] " << msg << "\n";
    }
private:
    bool verbose_;
};

// TensorRT 10: nesneler delete ile yok edilir (destroy() kaldirildi).
template <class T> struct TrtDelete { void operator()(T* p) const noexcept { delete p; } };
template <class T> using TrtPtr = std::unique_ptr<T, TrtDelete<T>>;

inline bool cudaOk(cudaError_t e, const char* what, std::string& err) {
    if (e == cudaSuccess) return true;
    err = std::string("CUDA hatasi (") + what + "): " + cudaGetErrorString(e);
    return false;
}

// ------------------------------------------------------------ ORTrack TRT
class OrtrackTrtBackend : public OrtrackTracker {
public:
    explicit OrtrackTrtBackend(const TrtOptions& opt) : opt_(opt), logger_(opt.verbose) {}

    ~OrtrackTrtBackend() override {
        // Sira onemli: once context/engine/runtime, sonra CUDA kaynaklari.
        context_.reset(); engine_.reset(); runtime_.reset();
        if (stream_) cudaStreamDestroy(stream_);
        for (void* p : devBufs_) if (p) cudaFree(p);
        for (void* p : hostBufs_) if (p) cudaFreeHost(p);
    }

    bool open(std::string& err);

    // ----------------------------------------------------- TrackerBackend
    bool init(const cv::Mat& frame, const cv::Rect& roi) override {
        return initBox(frame, BoxF::fromRect(roi));
    }
    bool update(const cv::Mat& frame, cv::Rect& box) override {
        BoxF b; float sc, raw;
        const bool ok = track(frame, b, sc, raw);
        if (haveBox_) box = box_.toRect();
        (void)b;
        return ok;
    }

    // -------------------------------------------- hybrid::ManagedTracker
    // Manager KUTUYU TAM HASSASIYETLE surer (cv::Rect yuvarlamasi IoU ve
    // box_scale_drift kararlarini bozardi).
    bool initialize(const cv::Mat& frame, const BoxF& box) override {
        return initBox(frame, box);
    }
    bool track(const cv::Mat& frame, BoxF& box, float& score, float& raw) override;
    void reset() override {
        haveBox_ = false;
        score_ = rawScore_ = -1.0f;
        msTotal_ = msInfer_ = 0.0;
    }
    float  score()    const override { return score_; }
    float  rawScore() const override { return rawScore_; }
    double lastMs()   const override { return msTotal_; }
    double inferMs()  const override { return msInfer_; }
    const char* name() const override { return "ORTRACK"; }

private:
    bool initBox(const cv::Mat& frame, const BoxF& box);
    bool runNetwork(std::string& why);

    static constexpr size_t kTplElems    = 1u * 3u * kTemplateSize * kTemplateSize;
    static constexpr size_t kSrchElems   = 1u * 3u * kSearchSize   * kSearchSize;
    static constexpr size_t kMapElems    = (size_t)kFeatSz * kFeatSz;          // score
    static constexpr size_t kMap2Elems   = 2u * kMapElems;                      // size / offset

    TrtOptions opt_;
    TrtLogger  logger_;

    TrtPtr<nvinfer1::IRuntime>          runtime_;
    TrtPtr<nvinfer1::ICudaEngine>       engine_;
    TrtPtr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_ = nullptr;

    // Cihaz tamponlari (bir kez ayrilir, adresleri bir kez baglanir)
    float* dTpl_ = nullptr; float* dSrch_ = nullptr;
    float* dScore_ = nullptr; float* dSize_ = nullptr; float* dOff_ = nullptr;
    // Sabitlenmis (pinned) host tamponlari — H2D/D2H kopyalari DMA ile
    float* hTpl_ = nullptr; float* hSrch_ = nullptr;
    float* hScore_ = nullptr; float* hSize_ = nullptr; float* hOff_ = nullptr;
    std::vector<void*> devBufs_, hostBufs_;

    NormalizeLUT lut_;
    float        hann_[kMapElems];

    BoxF   box_{};              // son GECERLI kutu (orijinal kare pikselleri)
    bool   haveBox_ = false;
    float  score_ = -1.0f, rawScore_ = -1.0f;
    double msTotal_ = 0.0, msInfer_ = 0.0;
};

bool OrtrackTrtBackend::open(std::string& err)
{
    hannWindow(hann_);

    if (!cudaOk(cudaSetDevice(opt_.device), "cudaSetDevice", err)) return false;

    // ---- motor dosyasini oku
    std::ifstream f(opt_.enginePath, std::ios::binary | std::ios::ate);
    if (!f) { err = "Motor dosyasi acilamadi: " + opt_.enginePath; return false; }
    const std::streamsize n = f.tellg();
    if (n <= 0) { err = "Motor dosyasi bos: " + opt_.enginePath; return false; }
    f.seekg(0, std::ios::beg);
    std::vector<char> plan((size_t)n);
    if (!f.read(plan.data(), n)) { err = "Motor dosyasi okunamadi: " + opt_.enginePath; return false; }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) { err = "TensorRT runtime olusturulamadi"; return false; }

    engine_.reset(runtime_->deserializeCudaEngine(plan.data(), (size_t)n));
    if (!engine_) {
        err = "Motor deserialize EDILEMEDI: " + opt_.enginePath +
              "\n       En olasi sebep: bu plan BASKA bir makinede/TensorRT surumunde uretilmis."
              "\n       TensorRT plani cihaza + surume ozeldir, kopyalanamaz."
              "\n       Cozum: ONNX'ten bu Orin uzerinde yeniden uret:"
              "\n         scripts/build_ortrack_engine.sh";
        return false;
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) { err = "TensorRT execution context olusturulamadi"; return false; }

    // ---- baglantilari DOGRULA (yanlis motoru sessizce kabul etme)
    struct Want { const char* name; bool input; int64_t elems; nvinfer1::Dims4 dims; };
    const Want wants[5] = {
        {"template",   true,  (int64_t)kTplElems,  nvinfer1::Dims4{1, 3, kTemplateSize, kTemplateSize}},
        {"search",     true,  (int64_t)kSrchElems, nvinfer1::Dims4{1, 3, kSearchSize,   kSearchSize}},
        {"score_map",  false, (int64_t)kMapElems,  nvinfer1::Dims4{1, 1, kFeatSz, kFeatSz}},
        {"size_map",   false, (int64_t)kMap2Elems, nvinfer1::Dims4{1, 2, kFeatSz, kFeatSz}},
        {"offset_map", false, (int64_t)kMap2Elems, nvinfer1::Dims4{1, 2, kFeatSz, kFeatSz}},
    };

    const int nio = engine_->getNbIOTensors();
    if (nio != 5) {
        std::ostringstream o;
        o << "Motor " << nio << " tensor bildiriyor, 5 bekleniyordu "
             "(template, search, score_map, size_map, offset_map). Yanlis motor dosyasi.";
        err = o.str(); return false;
    }
    for (const Want& w : wants) {
        bool found = false;
        for (int i = 0; i < nio; ++i)
            if (std::strcmp(engine_->getIOTensorName(i), w.name) == 0) { found = true; break; }
        if (!found) { err = std::string("Motorda '") + w.name + "' tensoru YOK. Yanlis motor dosyasi."; return false; }

        const auto mode = engine_->getTensorIOMode(w.name);
        const bool isIn = (mode == nvinfer1::TensorIOMode::kINPUT);
        if (isIn != w.input) { err = std::string("Tensor '") + w.name + "' giris/cikis yonu ters."; return false; }

        if (engine_->getTensorDataType(w.name) != nvinfer1::DataType::kFLOAT) {
            err = std::string("Tensor '") + w.name + "' FP32 degil."
                  "\n       Motoru --fp16 ile kurun ama G/C tiplerini fp32 birakin"
                  "\n       (--inputIOFormats/--outputIOFormats VERMEYIN).";
            return false;
        }
        const nvinfer1::Dims d = engine_->getTensorShape(w.name);
        if (d.nbDims != 4 || d.d[0] != w.dims.d[0] || d.d[1] != w.dims.d[1] ||
            d.d[2] != w.dims.d[2] || d.d[3] != w.dims.d[3]) {
            std::ostringstream o;
            o << "Tensor '" << w.name << "' sekli (";
            for (int k = 0; k < d.nbDims; ++k) o << d.d[k] << (k + 1 < d.nbDims ? "," : "");
            o << ") beklenen (" << w.dims.d[0] << "," << w.dims.d[1] << ","
              << w.dims.d[2] << "," << w.dims.d[3] << ") ile uyusmuyor."
                 "\n       template=128 / search=256 SABITTIR (DeiT pozisyon token sayisi).";
            err = o.str(); return false;
        }
    }

    // ---- tamponlar
    if (!cudaOk(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "streamCreate", err)) return false;

    auto devAlloc = [&](float** p, size_t elems) {
        if (!cudaOk(cudaMalloc((void**)p, elems * sizeof(float)), "cudaMalloc", err)) return false;
        devBufs_.push_back(*p); return true;
    };
    auto hostAlloc = [&](float** p, size_t elems) {
        if (!cudaOk(cudaHostAlloc((void**)p, elems * sizeof(float), cudaHostAllocDefault),
                    "cudaHostAlloc", err)) return false;
        hostBufs_.push_back(*p); return true;
    };
    if (!devAlloc(&dTpl_,   kTplElems))  return false;
    if (!devAlloc(&dSrch_,  kSrchElems)) return false;
    if (!devAlloc(&dScore_, kMapElems))  return false;
    if (!devAlloc(&dSize_,  kMap2Elems)) return false;
    if (!devAlloc(&dOff_,   kMap2Elems)) return false;
    if (!hostAlloc(&hTpl_,   kTplElems))  return false;
    if (!hostAlloc(&hSrch_,  kSrchElems)) return false;
    if (!hostAlloc(&hScore_, kMapElems))  return false;
    if (!hostAlloc(&hSize_,  kMap2Elems)) return false;
    if (!hostAlloc(&hOff_,   kMap2Elems)) return false;

    // Adresler BIR KEZ baglanir; her karede tekrar set edilmez.
    if (!context_->setTensorAddress("template",   dTpl_)   ||
        !context_->setTensorAddress("search",     dSrch_)  ||
        !context_->setTensorAddress("score_map",  dScore_) ||
        !context_->setTensorAddress("size_map",   dSize_)  ||
        !context_->setTensorAddress("offset_map", dOff_)) {
        err = "TensorRT tensor adresleri baglanamadi"; return false;
    }

    // ---- ISINMA: ilk cagri her zaman yavastir. CH12 kilit aninda
    // tokezlememesi icin burada, ucustan ONCE odenir.
    if (!cudaOk(cudaMemsetAsync(dTpl_,  0, kTplElems  * sizeof(float), stream_), "memset tpl", err)) return false;
    if (!cudaOk(cudaMemsetAsync(dSrch_, 0, kSrchElems * sizeof(float), stream_), "memset srch", err)) return false;
    for (int i = 0; i < opt_.warmupIters; ++i) {
        if (!context_->enqueueV3(stream_)) { err = "TensorRT isinma cagrisi basarisiz"; return false; }
    }
    if (!cudaOk(cudaStreamSynchronize(stream_), "isinma sync", err)) return false;

    return true;
}

bool OrtrackTrtBackend::runNetwork(std::string& why)
{
    const auto t0 = Clock::now();
    if (cudaMemcpyAsync(dSrch_, hSrch_, kSrchElems * sizeof(float),
                        cudaMemcpyHostToDevice, stream_) != cudaSuccess) { why = "H2D search"; return false; }
    if (!context_->enqueueV3(stream_)) { why = "enqueueV3"; return false; }
    if (cudaMemcpyAsync(hScore_, dScore_, kMapElems  * sizeof(float), cudaMemcpyDeviceToHost, stream_) != cudaSuccess ||
        cudaMemcpyAsync(hSize_,  dSize_,  kMap2Elems * sizeof(float), cudaMemcpyDeviceToHost, stream_) != cudaSuccess ||
        cudaMemcpyAsync(hOff_,   dOff_,   kMap2Elems * sizeof(float), cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
        why = "D2H maps"; return false;
    }
    if (cudaStreamSynchronize(stream_) != cudaSuccess) { why = "streamSynchronize"; return false; }
    msInfer_ = msSince(t0);
    return true;
}

bool OrtrackTrtBackend::initBox(const cv::Mat& frame, const BoxF& roi)
{
    reset();
    if (frame.empty() || frame.type() != CV_8UC3) return false;

    // Python initialize(): box = box.clip(W, H) — kirpilmis kutu SAKLANIR.
    const BoxF clipped = roi.clipped((double)frame.cols, (double)frame.rows);
    if (!clipped.ok()) return false;

    const Crop c = sampleTarget(frame, clipped, kTemplateFactor, kTemplateSize);
    if (!c.ok) return false;

    // frame BGR; normalize sirasinda kanal takasi yapilir (RGB duzlemleri).
    lut_.apply(c.patch, hTpl_, /*swapRB=*/true);

    if (cudaMemcpyAsync(dTpl_, hTpl_, kTplElems * sizeof(float),
                        cudaMemcpyHostToDevice, stream_) != cudaSuccess) return false;
    if (cudaStreamSynchronize(stream_) != cudaSuccess) return false;

    box_ = clipped;
    haveBox_ = true;
    return true;
}

bool OrtrackTrtBackend::track(const cv::Mat& frame, BoxF& box, float& score, float& raw)
{
    const auto tAll = Clock::now();
    score = score_; raw = rawScore_;

    // Gecersiz her yolda son gecerli kutuyu dondur (cizim/OSD bozulmasin).
    auto fail = [&](void) -> bool {
        if (haveBox_) box = box_;
        msTotal_ = msSince(tAll);
        return false;
    };

    if (!haveBox_ || frame.empty() || frame.type() != CV_8UC3) return fail();

    const Crop c = sampleTarget(frame, box_, kSearchFactor, kSearchSize);
    if (!c.ok) return fail();                       // kutu tamamen kare disi

    lut_.apply(c.patch, hSrch_, /*swapRB=*/true);

    std::string why;
    if (!runNetwork(why)) {
        static int spam = 0;
        if ((spam++ % 30) == 0) std::cerr << "[ORT] cikarim hatasi: " << why << "\n";
        return fail();
    }

    const Pred p = calBBox(hScore_, hSize_, hOff_, hann_);
    score_    = p.score;
    rawScore_ = p.rawScore;
    score = score_; raw = rawScore_;

    BoxF rawBox{}, out{};
    if (!mapAndClip(p, box_, kSearchSize, c.scale,
                    frame.cols, frame.rows, opt_.minBoxSize, rawBox, out)) return fail();

    // Opsiyonel guven kapisi. Varsayilan 0 = KAPALI -> KCF ile ayni davranis.
    if (opt_.minScore > 0.0f && p.score < opt_.minScore) {
        box_ = out;                                 // upstream gibi kutuyu yine tasi
        box = out;
        msTotal_ = msSince(tAll);
        return false;                               // ama bu kareyi GECERSIZ say
    }

    box_ = out;
    box  = out;
    msTotal_ = msSince(tAll);
    return true;
}

} // namespace

// ------------------------------------------------------------------ fabrika
std::unique_ptr<OrtrackTracker> makeTensorRTTracker(const TrtOptions& opt, std::string& err)
{
    std::unique_ptr<OrtrackTrtBackend> t(new OrtrackTrtBackend(opt));
    if (!t->open(err)) return nullptr;
    return std::unique_ptr<OrtrackTracker>(t.release());
}

const char* tensorrtBuildInfo()
{
    static std::string s = "TensorRT " + std::to_string(NV_TENSORRT_MAJOR) + "." +
                           std::to_string(NV_TENSORRT_MINOR) + "." +
                           std::to_string(NV_TENSORRT_PATCH);
    return s.c_str();
}

} // namespace ortrack
