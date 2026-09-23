// =====================================================================
// target_manager.hpp — hybrid/manager.py TargetManager'in C++ portu.
//
// YOLO (tespit) + ORTrack (takip) arasindaki TEK HEDEF durum makinesi.
// Modelden bagimsizdir: icinde ne TensorRT ne OpenCV cikarimi vardir,
// sadece mantik. Bu yuzden test_target_manager ile Python referansina
// karsi kare kare dogrulanabilir.
//
// DURUMLAR: SEARCHING -> VERIFYING -> TRACKING -> LOST
//
// PORT KAPSAMI — bilerek daraltildi:
//   Python'daki "playback" dallari (source_timestamp_ms, segment_id,
//   seek) CANLI kamerada ASLA calismaz: canli kaynakta
//   source_timestamp_ms her zaman None ve segment_id her zaman 0'dir.
//   Bu dallar portlanmadi. Geri kalan her karar birebir aynidir ve
//   islem sirasi korunmustur (yorumlarda "py:" satirlari).
//
// KRITIK DAVRANIS: gecersiz tracker ciktisi ANINDA kayip sayilir
// ("invalid_tracker_output"). Bu dogrudur cunku hibritte YOLO yeniden
// yakalayabilir. Manuel ROI modunda (--ortrack, YOLO yok) boyle bir
// kurtarma YOKTUR; orada N kare sabir kullanilir. Iki mod, iki politika.
// =====================================================================
#pragma once

#include "ortrack_core.hpp"

#include <cmath>
#include <deque>
#include <string>
#include <vector>

namespace hybrid {

using ortrack::BoxF;

// ------------------------------------------------------------- yardimcilar
inline double boxArea(const BoxF& b) { return b.w() * b.h(); }

inline double boxIoU(const BoxF& a, const BoxF& b) {
    const double iw = std::max(0.0, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
    const double ih = std::max(0.0, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
    const double inter = iw * ih;
    const double ua = boxArea(a) + boxArea(b) - inter;
    return (ua > 0.0) ? inter / ua : 0.0;
}

enum class State { SEARCHING, VERIFYING, TRACKING, LOST };

inline const char* stateName(State s) {
    switch (s) {
        case State::SEARCHING: return "searching";
        case State::VERIFYING: return "verifying";
        case State::TRACKING:  return "tracking";
        default:               return "lost";
    }
}

struct Detection {
    BoxF   box;
    double score   = 0.0;
    int    classId = -1;
};

// py: ManagerConfig (hybrid/config.py). Varsayilanlar configs/orin-*.yaml ile ayni.
struct ManagerConfig {
    int    detection_interval          = 3;
    int    search_interval             = 2;
    int    confirmation_hits           = 2;
    double confirmation_iou            = 0.2;
    double confirmation_timeout_ms     = 1500.0;
    double verification_iou            = 0.2;
    double detection_accept_confidence = 0.5;
    double tracker_low_score           = 0.2;
    double tracker_recover_score       = 0.3;
    double tracker_score_ema_alpha     = 0.4;
    int    tracker_low_patience        = 3;
    int    verification_miss_patience  = 2;
    double max_box_growth              = 0.0;   // 0 = kapali
    int    verify_interval             = 0;     // 0 = kapali
    int    verify_miss_patience        = 3;
    double verification_timeout_ms     = 2000.0;
    double max_detection_age_ms        = 1000.0;
    double max_tracking_gap_ms         = 1000.0;
    bool   refresh_template            = false;
};

// Manager'in tracker'dan bekledigi sozlesme (ORTrack bunu uygular).
struct ManagedTracker {
    virtual ~ManagedTracker() {}
    virtual bool initialize(const cv::Mat& frame, const BoxF& box) = 0;
    // false = bu karede GECERLI kutu uretilemedi (py: result.valid == False)
    virtual bool track(const cv::Mat& frame, BoxF& box, float& score, float& rawScore) = 0;
    virtual void reset() = 0;
};

struct Event {
    std::string from, to, reason;
};

// ---------------------------------------------------------------------
class TargetManager {
public:
    TargetManager(ManagedTracker* tracker, const ManagerConfig& cfg)
        : tracker_(tracker), cfg_(cfg) {}

    // ---- durum okuyuculari (OSD / log icin)
    State        state()         const { return state_; }
    bool         hasBox()        const { return hasBox_; }
    const BoxF&  box()           const { return box_; }
    bool         hasCandidate()  const { return hasCandidate_; }
    const BoxF&  candidate()     const { return candidate_.box; }
    int          hits()          const { return hits_; }
    int          lowCount()      const { return lowCount_; }
    int          misses()        const { return misses_; }
    int          unverified()    const { return unverified_; }
    bool         weak()          const { return weak_; }
    bool         haveFiltered()  const { return haveFiltered_; }
    double       filteredScore() const { return filteredScore_; }
    int          generation()    const { return generation_; }
    long long    rejected()      const { return rejectedDetections_; }
    const std::vector<Event>& events() const { return events_; }

    // py: begin_frame — takip adimi. Kare ID'si ARTMALI.
    // Doner: bu karede tracker calisti mi (tracked). score/rawScore doldurulur.
    bool beginFrame(const cv::Mat& frame, long long frameId, long long capturedNs,
                    float* outScore = nullptr, float* outRaw = nullptr);

    // py: detection_due
    bool detectionDue() const;

    // py: request_detection — YOLO cagrilmadan HEMEN once.
    int requestDetection();

    // py: apply_detection — YOLO sonucu AYNI kare icin uygulanir.
    // batchFrameId/batchCapturedNs/batchGeneration: requestDetection anindaki degerler.
    void applyDetection(const cv::Mat& frame, long long frameId, long long capturedNs,
                        long long batchFrameId, long long batchCapturedNs, int batchGeneration,
                        long long batchCompletedNs, long long nowNs,
                        const std::vector<Detection>& detections);

    // py: finish_frame — kayip karari YOLO'ya AYNI karede sans verildikten SONRA.
    void finishFrame(long long capturedNs);

    void reset(const std::string& reason);

private:
    void transition(State s, const std::string& reason) {
        if (s == state_) return;
        events_.push_back(Event{stateName(state_), stateName(s), reason});
        state_ = s;
    }
    void lose(const std::string& reason);

    ManagedTracker* tracker_;
    ManagerConfig   cfg_;

    State     state_        = State::SEARCHING;
    bool      hasBox_       = false;
    BoxF      box_{};
    bool      hasCandidate_ = false;
    Detection candidate_{};

    int  hits_ = 0, lowCount_ = 0, misses_ = 0, unverified_ = 0;
    bool haveReferenceArea_ = false;
    double referenceArea_   = 0.0;
    long long candidateNs_ = 0, verifiedNs_ = 0;
    int  generation_ = 0;
    long long lastId_ = -1, lastCaptureNs_ = 0;
    long long processed_ = 0;
    long long lastDetectStep_ = -1000000000LL;
    long long lastDetectionId_ = -1;
    long long rejectedDetections_ = 0;
    std::vector<Event> events_, pendingEvents_;
    bool      haveLastTracking_ = false;
    long long lastTrackingNs_   = 0;
    bool      haveFiltered_ = false;
    double    filteredScore_ = 0.0;
    bool      weak_ = false;
    bool      haveLowSince_ = false;
    long long lowSinceNs_ = 0;
    bool      verificationRequested_ = false;
    bool      scheduledCheck_ = false;
};

// ---------------------------------------------------------------------
inline void TargetManager::lose(const std::string& reason)
{
    tracker_->reset();
    hasBox_ = hasCandidate_ = false;
    hits_ = lowCount_ = misses_ = unverified_ = 0;
    haveReferenceArea_ = false; referenceArea_ = 0.0;
    haveFiltered_ = false; filteredScore_ = 0.0;
    weak_ = false;
    haveLowSince_ = false;
    ++generation_;                 // py: bekleyen dedektor sonuclarini gecersiz kilar
    transition(State::LOST, reason);
}

inline void TargetManager::reset(const std::string& reason)
{
    lose(reason);
    state_ = State::SEARCHING;
    haveLastTracking_ = false;
    lastDetectStep_ = -1000000000LL;
    pendingEvents_.push_back(Event{"", "searching", reason});
}

inline bool TargetManager::beginFrame(const cv::Mat& frame, long long frameId, long long capturedNs,
                                      float* outScore, float* outRaw)
{
    // py: self.events, self.pending_events = self.pending_events, []
    events_.swap(pendingEvents_);
    pendingEvents_.clear();

    // py: kare ID'leri artmali, zaman damgalari monotonik olmali
    if (frameId <= lastId_ || capturedNs < lastCaptureNs_) {
        // Canli yolda bu olmamali; olursa kareyi atla (Python'da exception).
        return false;
    }

    // Canli kaynak: tracking_time_ns == captured_ns
    const long long timeline = capturedNs;
    const double gapMs = haveLastTracking_ ? (double)(timeline - lastTrackingNs_) / 1e6 : 0.0;

    if (hasBox_ && gapMs > cfg_.max_tracking_gap_ms) lose("capture_gap");

    if (!hasBox_ && hasCandidate_ &&
        (double)(timeline - candidateNs_) / 1e6 > cfg_.confirmation_timeout_ms) {
        hasCandidate_ = false;
        hits_ = 0;
        transition(State::SEARCHING, "confirmation_timeout");
    }

    lastId_ = frameId;
    lastCaptureNs_ = capturedNs;
    haveLastTracking_ = true;
    lastTrackingNs_ = timeline;
    verificationRequested_ = scheduledCheck_ = false;
    ++processed_;

    if (!hasBox_) return false;

    BoxF nb{}; float sc = 0.f, raw = 0.f;
    const bool ok = tracker_->track(frame, nb, sc, raw);
    if (outScore) *outScore = sc;
    if (outRaw)   *outRaw   = raw;

    if (!ok || !nb.ok() || !std::isfinite(sc)) {
        lose("invalid_tracker_output");
        return true;
    }
    box_ = nb;

    // py: surüklenen tracker guvenli tepe bildirmeye devam ederken kutusu
    // kareyi yutar -> boyut, YOLO'nun ONAYLADIGI son kutuya karsi denetlenir.
    if (cfg_.max_box_growth > 0.0 && haveReferenceArea_ && referenceArea_ > 0.0) {
        const double scale = std::sqrt(boxArea(box_) / referenceArea_);
        if (!(1.0 / cfg_.max_box_growth <= scale && scale <= cfg_.max_box_growth)) {
            lose("box_scale_drift");
            return true;
        }
    }

    const double alpha = cfg_.tracker_score_ema_alpha;
    filteredScore_ = haveFiltered_ ? alpha * (double)sc + (1.0 - alpha) * filteredScore_ : (double)sc;
    haveFiltered_ = true;

    if (filteredScore_ >= cfg_.tracker_recover_score) {
        weak_ = false;
        lowCount_ = misses_ = 0;
        haveLowSince_ = false;
        transition(State::TRACKING, "tracker_recovered");
    } else if (!weak_ && filteredScore_ < cfg_.tracker_low_score) {
        weak_ = true;
        haveLowSince_ = true; lowSinceNs_ = timeline;
        lowCount_ = misses_ = 0;
        lastDetectStep_ = -1000000000LL;   // ilk dusuk gozlem ANINDA YOLO ister
    }
    if (weak_) {
        ++lowCount_;
        transition(State::VERIFYING, "low_tracker_response");
    }
    return true;
}

inline bool TargetManager::detectionDue() const
{
    if (hasBox_) {
        if (weak_) return processed_ - lastDetectStep_ >= cfg_.detection_interval;
        // py: guvenli tracker YANLIS seye kilitlenmis olabilir; skor bunu SOYLEMEZ.
        return cfg_.verify_interval > 0 && processed_ - lastDetectStep_ >= cfg_.verify_interval;
    }
    if (state_ == State::LOST || state_ == State::VERIFYING) return true;
    return processed_ - lastDetectStep_ >= cfg_.search_interval;
}

inline int TargetManager::requestDetection()
{
    lastDetectStep_ = processed_;
    verificationRequested_ = hasBox_ && weak_;
    scheduledCheck_ = hasBox_ && !weak_ && cfg_.verify_interval > 0;
    if (verificationRequested_) transition(State::VERIFYING, "low_response_verification");
    return generation_;
}

inline void TargetManager::applyDetection(const cv::Mat& frame, long long frameId, long long capturedNs,
                                          long long batchFrameId, long long batchCapturedNs,
                                          int batchGeneration, long long batchCompletedNs,
                                          long long nowNs, const std::vector<Detection>& raw)
{
    // py: kati AYNI-KARE politikasi. Gec gelen / eski nesil sonuc REDDEDILIR.
    if (batchFrameId != frameId || frameId != lastId_ ||
        batchCapturedNs != capturedNs || batchGeneration != generation_ ||
        batchFrameId <= lastDetectionId_ ||
        !(batchCapturedNs <= batchCompletedNs && batchCompletedNs <= nowNs) ||
        (double)(nowNs - batchCapturedNs) / 1e6 > cfg_.max_detection_age_ms) {
        ++rejectedDetections_;
        return;
    }
    lastDetectionId_ = batchFrameId;

    std::vector<Detection> dets;
    dets.reserve(raw.size());
    for (const Detection& d : raw)
        if (std::isfinite(d.score) && d.score >= cfg_.detection_accept_confidence) dets.push_back(d);

    const long long timeline = capturedNs;

    // ---------------- ZATEN KILITLI: dogrulama
    if (hasBox_) {
        const Detection* best = nullptr; double bestIoU = -1.0;
        for (const Detection& d : dets) {
            const double i = boxIoU(box_, d.box);
            if (i >= cfg_.verification_iou && i > bestIoU) { bestIoU = i; best = &d; }
        }
        if (best) {
            misses_ = lowCount_ = unverified_ = 0;
            haveReferenceArea_ = true; referenceArea_ = boxArea(best->box);
            verifiedNs_ = timeline;
            haveLowSince_ = weak_;
            if (weak_) lowSinceNs_ = timeline;
            transition(State::TRACKING, "yolo_verified");
            if (cfg_.refresh_template) {
                box_ = best->box;
                tracker_->initialize(frame, best->box);
            }
        } else if (weak_) {
            ++misses_;
            transition(State::VERIFYING, "low_response_yolo_missing");
        } else if (scheduledCheck_) {
            // py: YOLO recall'u mukemmel degil; TEK kacirma hicbir sey kanitlamaz.
            // Sadece ARDISIK kacirmalar kutuyu dusurur. Bu arada durum tracking kalir.
            ++unverified_;
            if (unverified_ >= cfg_.verify_miss_patience)
                lose("confident_track_without_yolo_support");
        }
        return;
    }

    // ---------------- KILIT YOK: yakalama
    if (dets.empty()) {
        hasCandidate_ = false;
        hits_ = 0;
        transition(State::SEARCHING, "no_detection");
        return;
    }

    const Detection* selected = nullptr;
    if (hasCandidate_) {
        double bestIoU = -1.0;
        for (const Detection& d : dets) {
            const double i = boxIoU(d.box, candidate_.box);
            if (i >= cfg_.confirmation_iou && i > bestIoU) { bestIoU = i; selected = &d; }
        }
    }
    if (selected) {
        ++hits_;
    } else {
        double bestScore = -1.0;
        for (const Detection& d : dets)
            if (d.score > bestScore) { bestScore = d.score; selected = &d; }
        hits_ = 1;
    }

    candidate_ = *selected;
    hasCandidate_ = true;
    candidateNs_ = timeline;
    transition(State::VERIFYING, "candidate_detection");

    if (hits_ >= cfg_.confirmation_hits) {
        const BoxF chosen = selected->box;         // candidate_ uzerine yazilmadan once kopya
        if (!tracker_->initialize(frame, chosen)) {
            // Sablon kirpilamadi (kutu kare disi vb). Adayi dusur, aramaya don.
            hasCandidate_ = false; hits_ = 0;
            transition(State::SEARCHING, "tracker_init_failed");
            return;
        }
        box_ = chosen;
        hasBox_ = true;
        hasCandidate_ = false;
        lowCount_ = misses_ = unverified_ = 0;
        haveReferenceArea_ = true; referenceArea_ = boxArea(chosen);
        verifiedNs_ = timeline;
        haveFiltered_ = false; filteredScore_ = 0.0;
        weak_ = false;
        haveLowSince_ = false;
        transition(State::TRACKING, "confirmed_acquisition");
    }
}

inline void TargetManager::finishFrame(long long capturedNs)
{
    if (!hasBox_ || !weak_ || lowCount_ < cfg_.tracker_low_patience) return;
    const bool timedOut = haveLowSince_ &&
        (double)(capturedNs - lowSinceNs_) / 1e6 >= cfg_.verification_timeout_ms;
    if (misses_ >= cfg_.verification_miss_patience || timedOut)
        lose("low_response_without_yolo_support");
}

} // namespace hybrid
