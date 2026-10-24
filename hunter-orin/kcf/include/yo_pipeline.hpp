// =====================================================================
// yo_pipeline.hpp — [YO] YoloORTrack AsyncPipeline'in C++ portu.
//
// REFERANS: YoloORTrack hybrid/async_pipeline.py (_process). Her karede:
//   1) manager.beginFrame        -> tracker GUNCEL karede (yuksek oncelikli stream)
//   2) tarama bittiyse           -> applyDelayedDetection (+ catch-up)
//   3) manager.finishFrame       -> kayip karari (tespite sans verildikten SONRA)
//   4) bekleyen tarama yoksa     -> detectionRequest + bu karenin KOPYASINI gonder
// Ucusta her an EN FAZLA BIR tarama vardir; kare kuyrugu / gecmis birikmez.
// Biten tarama hemen yenisini baslatir (paket: "each completed scan frees the
// next newest frame") -> YOLO arka planda surekli doner, tracker ~30 fps.
//
// Scanner arayuzu: ThreadedScanner (ucus, ayri thread + SahiDetector) ya da
// testte deterministik sahte tarayici (test_yo_manager.cpp).
// =====================================================================
#pragma once

#include "yo_manager.hpp"
#include "yo_sahi.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace yo {

inline long long steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Tamamlanmis bir taramanin sonucu (py: DetectionBatch)
struct ScanResult {
    bool      ok = false;                // false: karo cagrisi basarisiz -> UYGULANMAZ
    long long frameId = -1;
    long long capNs = 0;
    int       generation = 0;
    long long completedNs = 0;
    std::vector<Detection> dets;         // kare koordinatinda, birlesmis, SAHI sirasinda
    ScanStats stats;
    std::string error;
};

struct Scanner {
    virtual ~Scanner() {}
    virtual void submit(const DetectionRequest& req) = 0;   // tek is; mesgulken cagrilmaz
    virtual bool poll(ScanResult& out) = 0;                 // bitmis sonuc varsa true (bloklamaz)
};

// ---------------------------------------------------------------------
// ThreadedScanner — SahiDetector'u AYRI thread'de kosturur. TensorRT
// context'i ve YOLO CUDA stream'i SADECE bu thread kullanir.
// ---------------------------------------------------------------------
class ThreadedScanner : public Scanner {
public:
    explicit ThreadedScanner(SahiDetector* det) : det_(det) {
        th_ = std::thread([this] { run(); });
    }
    ~ThreadedScanner() override {
        { std::lock_guard<std::mutex> g(m_); stop_ = true; }
        cv_.notify_all();
        if (th_.joinable()) th_.join();
    }
    void submit(const DetectionRequest& req) override {
        { std::lock_guard<std::mutex> g(m_); job_ = req; hasJob_ = true; busy_ = true; }
        cv_.notify_one();
    }
    bool poll(ScanResult& out) override {
        std::lock_guard<std::mutex> g(m_);
        if (!hasResult_) return false;
        out = std::move(result_);
        result_ = ScanResult{};
        hasResult_ = false;
        return true;
    }
    bool busy() const { return busy_.load(); }
    long long scans() const { return scans_.load(); }

private:
    void run() {
        for (;;) {
            DetectionRequest job;
            {
                std::unique_lock<std::mutex> g(m_);
                cv_.wait(g, [this] { return stop_ || hasJob_; });
                if (stop_) return;
                job = job_; hasJob_ = false; job_ = DetectionRequest{};
            }
            ScanResult r;
            r.frameId = job.frameId; r.capNs = job.capNs; r.generation = job.generation;
            try {
                r.ok = job.image && det_->detect(*job.image, r.dets, r.stats);
                if (!r.ok) r.error = "karo cagrisi basarisiz";
            } catch (const std::exception& e) {      // cv::Exception vb.: ucus cokmesin
                r.ok = false; r.dets.clear(); r.error = e.what();
            }
            r.completedNs = steadyNowNs();          // py: perf_counter_ns() tarama sonunda
            ++scans_;
            {
                std::lock_guard<std::mutex> g(m_);
                result_ = std::move(r); hasResult_ = true; busy_ = false;
            }
        }
    }

    SahiDetector* det_;
    std::thread th_;
    std::mutex m_;
    std::condition_variable cv_;
    DetectionRequest job_;
    ScanResult result_;
    bool hasJob_ = false, hasResult_ = false, stop_ = false;
    std::atomic<bool> busy_{false};
    std::atomic<long long> scans_{0};
};

// Bir karenin ozeti (OSD, FRAME, log)
struct StepResult {
    bool      accepted = false;            // false: kare sirasi bozuk, manager surulmedi
    long long frameId = -1, capNs = 0;
    TrackOut  tracked;                     // beginFrame ya da catch-up adimi
    // bu karede biten tarama
    bool      scanDone = false, scanOk = false, scanApplied = false;
    long long scanSrcFrame = -1;
    double    scanAgeMs = NAN;             // sonuc uygulandiginda kaynak karenin yasi
    ScanStats scanStats;
    std::vector<Detection> scanDets;       // cizim/log icin (kare koordinati)
    std::string scanError;
    ApplyInfo apply;
    // gonderim
    bool      submitted = false;
    long long inFlightFrame = -1;
};

// ---------------------------------------------------------------------
class Pipeline {
public:
    Pipeline(TargetManager* m, Scanner* s, std::function<long long()> clock = steadyNowNs)
        : mgr_(m), scanner_(s), clock_(std::move(clock)) {}

    TargetManager& manager() { return *mgr_; }
    bool hasPending() const { return hasPending_; }
    long long pendingFrame() const { return hasPending_ ? pending_.frameId : -1; }

    // frame: GUNCEL kare (bu cagri sirasinda okunur; tarama icin KOPYALANIR).
    StepResult process(const cv::Mat& frame, long long frameId, long long capNs)
    {
        StepResult r;
        r.frameId = frameId; r.capNs = capNs;
        if (!mgr_->beginFrame(frame, frameId, capNs, &r.tracked)) return r;   // py: ValueError
        r.accepted = true;

        ScanResult res;
        if (hasPending_ && scanner_->poll(res)) {
            const DetectionRequest req = pending_;
            hasPending_ = false; pending_ = DetectionRequest{};
            // saat SONUC ALINDIKTAN sonra okunur: completedNs <= now garanti
            const long long now = clock_();
            r.scanDone = true; r.scanOk = res.ok; r.scanSrcFrame = req.frameId;
            r.scanAgeMs = (double)(now - req.capNs) / 1e6;
            r.scanStats = res.stats; r.scanError = res.error;
            if (res.ok) {
                const long long rej = mgr_->rejected();
                mgr_->applyDelayedDetection(frame, frameId, capNs, req,
                                            res.frameId, res.capNs, res.generation,
                                            res.completedNs, now, res.dets, &r.apply);
                r.scanApplied = (mgr_->rejected() == rej);
                if (mgr_->hasCatchup()) r.tracked = mgr_->catchup();
            }
            r.scanDets.swap(res.dets);
        }
        mgr_->finishFrame(capNs);
        if (!hasPending_) {
            DetectionRequest req;
            if (mgr_->detectionRequest(frameId, capNs, req)) {
                req.image = std::make_shared<const cv::Mat>(frame.clone());
                scanner_->submit(req);
                pending_ = req; hasPending_ = true;
                r.submitted = true;
            }
        }
        r.inFlightFrame = pendingFrame();
        return r;
    }

    // Pilot birakma vb. Bekleyen tarama YERINDE kalir; sonucu nesil degistigi
    // icin reddedilir (py: reset/_lose generation += 1).
    void reset(const std::string& reason) { mgr_->reset(reason); }

private:
    TargetManager* mgr_;
    Scanner* scanner_;
    std::function<long long()> clock_;
    bool hasPending_ = false;
    DetectionRequest pending_;
};

} // namespace yo
