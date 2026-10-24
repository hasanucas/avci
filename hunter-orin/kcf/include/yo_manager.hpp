// =====================================================================
// yo_manager.hpp — [YO] YoloORTrack TargetManager'in C++ portu (ASENKRON yol).
//
// REFERANS (degistirilmeden kopyalandi): tests/yo_ref/hybrid/manager.py,
// correction.py. Eski hybrid::TargetManager (target_manager.hpp, --hybrid)
// DOKUNULMADAN kalir; bu sinif sadece --yolo-ortrack modunda kullanilir.
//
// Eski porttan farklar (paketin 29 Eylul surumu):
//   * apply_delayed_detection: tarama sonucu, ISTENDIGI kaynak karede
//     degerlendirilir; yakalama o karenin goruntusunden sablon alir ve
//     guncel kareye TEK adim catch-up yapar.
//   * verify_interval: her tamamlanan tarama kilidi dogrular.
//   * detector_absence_grace_ms: hicbir destek kurali kutuyu son YOLO
//     onayindan bu kadar sure gecmeden dusurmez.
//   * tracker_correction center_blend: eslesen tespitte tracker merkezi
//     YOLO merkezine w kadar cekilir (boyut korunur), relocate ile sablon
//     korunur.
//   * detector_average_*, tracker_low_release_ms, clamp_box_growth,
//     acquisition_max_area_frac, detection_verify_confidence (config'te
//     kapali olanlar dahil hepsi portlandi; golden testte hepsi kosuyor).
//
// PORT KAPSAMI: sadece CANLI kaynak (source_timestamp_ms None, segment 0).
// Python'da ValueError firlatan durumlarda (kare sirasi bozuk, sablon
// kirpilamadi, kutu kareye sigmiyor) burada kare/duzeltme ATLANIR — ucus
// kodu cokmez. Bunlarin hicbiri golden senaryolarda olusmaz.
//
// Islem sirasi ve kayan nokta ifadeleri Python ile ayni yazildi
// ("py:" yorumlari). test_yo_manager ile kare kare dogrulanir.
// =====================================================================
#pragma once

#include "target_manager.hpp"   // hybrid::ManagedTracker, Detection, boxIoU, boxArea, BoxF

#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace yo {

using ortrack::BoxF;
using hybrid::Detection;
using hybrid::ManagedTracker;
using hybrid::boxIoU;
using hybrid::boxArea;

enum class State { SEARCHING, VERIFYING, TRACKING, LOST };

inline const char* stateName(State s) {
    switch (s) {
        case State::SEARCHING: return "searching";
        case State::VERIFYING: return "verifying";
        case State::TRACKING:  return "tracking";
        default:               return "lost";
    }
}

// Varsayilanlar = paketin UCUS config'i (configs/orin-siyi-sahi-6-async-768.yaml
// + calistir.sh: --tracker-correction center_blend --correction-weight 0.5
// --detector-grace-ms 1500). Python ManagerConfig varsayilanlari DEGIL.
struct ManagerConfig {
    int    confirmation_hits                  = 2;
    double confirmation_iou                   = 0.2;
    double confirmation_timeout_ms            = 1500.0;
    double verification_iou                   = 0.2;
    double detection_accept_confidence        = 0.4;
    bool   has_verify_confidence              = false;   // py: detection_verify_confidence None
    double detection_verify_confidence        = 0.0;
    double detector_average_release_confidence = 0.0;    // 0 + window 0 = kapali
    double detector_average_window_ms         = 0.0;
    int    detector_average_min_samples       = 2;
    double tracker_low_score                  = 0.4;
    double tracker_recover_score              = 0.5;
    double tracker_score_ema_alpha            = 0.4;
    int    tracker_low_patience               = 3;
    double tracker_low_release_ms             = 0.0;     // 0 = kapali
    int    verification_miss_patience         = 2;
    double max_box_growth                     = 2.5;     // 0 = kapali
    bool   clamp_box_growth                   = false;
    int    verify_interval                    = 1;       // asenkronda: her tarama dogrulama
    int    verify_miss_patience               = 3;
    double verification_timeout_ms            = 2000.0;
    double detector_absence_grace_ms          = 1500.0;
    double acquisition_max_area_frac          = 0.0;     // 0 = kapali
    double max_detection_age_ms               = 1000.0;
    double max_tracking_gap_ms                = 1000.0;
    bool   refresh_template                   = false;
    // tracker_correction
    bool   correction_enabled                 = true;    // mode: center_blend (false = off)
    double correction_weight                  = 0.5;     // detector_weight
    int    correction_application             = 0;       // 0 auto | 1 relocate | 2 reinitialize
};

// Olay (log + test). type: T gecis, Z reset, R reddedilen tespit,
// C duzeltme, K catch-up, B kutu sinirlandi.
struct Event {
    char        type = 'T';
    std::string from, to, reason;      // T: gecis ; Z: reason ; R: reason ; C: application
    // T -> lost kanitlari (py: _lose evidence)
    bool   hasEvidence = false;
    double filtered = NAN, unsupportedMs = NAN, detAvg = NAN, lowMs = NAN, boxScale = NAN;
    int    low = 0, misses = 0, unverified = 0, samples = 0;
    // diger
    long long srcFrame = -1, toFrame = -1;
    double dx = NAN, dy = NAN, value = NAN;
    int    generation = -1;
};

// Tracker adimi (begin_frame ya da catch-up). py: TrackResult
struct TrackOut {
    bool  tracked = false;     // bu karede tracker cagrildi mi
    bool  valid   = false;
    BoxF  box{};
    float score = 0.f, raw = 0.f;
};

// py: DetectionRequest(frame, generation, box, scheduled_check) — kaynak
// kare goruntusu paylasimli tutulur (yakalamada sablon BURADAN alinir).
struct DetectionRequest {
    long long frameId = -1;
    long long capNs   = 0;
    std::shared_ptr<const cv::Mat> image;
    int  generation     = 0;
    bool hasBox         = false;
    BoxF box{};
    bool scheduledCheck = false;
};

// apply_delayed_detection'in ne yaptigi (sadece log/rozet icin; mantiga girmez).
struct ApplyInfo {
    bool rejected = false;
    bool verified = false;          // kilit varken eslesen tespit
    bool missed   = false;          // kilit varken eslesme yok
    bool acquired = false;          // yeni kilit
    bool candidate = false;         // aday guncellendi (henuz kilit yok)
    bool corrected = false;         // center_blend kutuyu oynatti
    double matchIoU = NAN, matchScore = NAN;
    double dx = 0.0, dy = 0.0;      // duzeltme kaymasi (px)
};

// ---------------------------------------------------------------------
class TargetManager {
public:
    TargetManager(ManagedTracker* tracker, const ManagerConfig& cfg)
        : tracker_(tracker), cfg_(cfg) {}

    // ---- okuyucular (OSD / log / test)
    const ManagerConfig& config() const { return cfg_; }
    State        state()          const { return state_; }
    bool         hasBox()         const { return hasBox_; }
    const BoxF&  box()            const { return box_; }
    bool         hasCandidate()   const { return hasCandidate_; }
    const BoxF&  candidate()      const { return candidate_.box; }
    int          hits()           const { return hits_; }
    int          lowCount()       const { return low_; }
    int          misses()         const { return misses_; }
    int          unverified()     const { return unverified_; }
    bool         weak()           const { return weak_; }
    bool         haveFiltered()   const { return haveFiltered_; }
    double       filteredScore()  const { return filtered_; }
    int          generation()     const { return generation_; }
    long long    rejected()       const { return rejected_; }
    bool         hasCatchup()     const { return haveCatchup_; }
    const TrackOut& catchup()     const { return catchup_; }
    const std::vector<Event>& events() const { return events_; }
    // son YOLO onayindan beri gecen sure (ms, verilen zaman cizgisinde)
    double unsupportedMs(long long timelineNs) const { return (double)(timelineNs - verifiedNs_) / 1e6; }
    bool   haveDetAvg()     const { return haveDetAvg_; }
    double detAvg()         const { return detAvg_; }

    // ------------------------------------------------------- py: begin_frame
    // false: kare sirasi/zamani bozuk (py: ValueError) -> kare islenmez.
    bool beginFrame(const cv::Mat& frame, long long frameId, long long capNs, TrackOut* out)
    {
        events_.swap(pendingEvents_);       // py: self.events, self.pending_events = self.pending_events, []
        pendingEvents_.clear();
        if (out) *out = TrackOut{};
        if (frameId <= lastId_ || capNs < lastCaptureNs_) return false;
        const long long timeline = capNs;                       // canli: tracking_time_ns = captured_ns
        if (haveLastTracking_ && timeline < lastTrackingNs_) return false;
        const double gapMs = haveLastTracking_ ? (double)(timeline - lastTrackingNs_) / 1e6 : 0.0;
        if (hasBox_ && gapMs > cfg_.max_tracking_gap_ms) lose("capture_gap", nullptr);
        if (hasCandidate_ && (double)(timeline - candidateNs_) / 1e6 > cfg_.confirmation_timeout_ms) {
            hasCandidate_ = false;
            hits_ = 0;
            if (!hasBox_) transition(State::SEARCHING, "confirmation_timeout");
        }
        lastId_ = frameId; lastCaptureNs_ = capNs;
        haveLastTracking_ = true; lastTrackingNs_ = timeline;
        verificationRequested_ = scheduledCheck_ = false;
        ++processed_;
        const TrackOut t = trackFrame(frame, timeline);
        if (out) *out = t;
        return true;
    }

    // ------------------------------------------------- py: detection_request
    bool detectionRequest(long long frameId, long long capNs, DetectionRequest& req)
    {
        if (frameId != lastId_ || capNs != lastCaptureNs_) return false;   // py: ValueError
        const int gen = requestDetection();
        req.frameId = frameId; req.capNs = capNs;
        req.generation = gen;
        req.hasBox = hasBox_; req.box = box_;
        req.scheduledCheck = scheduledCheck_;
        return true;
    }

    // -------------------------------------------- py: apply_delayed_detection
    // cur*: GUNCEL kare. req: istegin kaynak karesi + o anki hedef geometrisi.
    // batch*: tarama sonucunun etiketi. nowNs: SONUC ALINDIKTAN sonra okunan saat.
    void applyDelayedDetection(const cv::Mat& curFrame, long long curId, long long curNs,
                               const DetectionRequest& req,
                               long long batchFrameId, long long batchCapNs, int batchGeneration,
                               long long batchCompletedNs, long long nowNs,
                               const std::vector<Detection>& raw, ApplyInfo* info = nullptr)
    {
        ApplyInfo dummy;
        ApplyInfo& ai = info ? *info : dummy;
        ai = ApplyInfo{};
        haveCatchup_ = false;                                   // py: self.catchup_result = None
        const cv::Mat* src = req.image.get();
        const bool sameShape = src && src->rows == curFrame.rows && src->cols == curFrame.cols &&
                               src->type() == curFrame.type();
        if (curId != lastId_ || req.frameId > curId || !sameShape ||
            batchFrameId != req.frameId || batchCapNs != req.capNs ||
            batchGeneration != req.generation || req.generation != generation_ ||
            batchFrameId <= lastDetectionId_ ||
            !(req.capNs <= batchCompletedNs && batchCompletedNs <= nowNs) ||
            (double)(nowNs - req.capNs) / 1e6 > cfg_.max_detection_age_ms ||
            !(0.0 <= (double)(curNs - req.capNs) / 1e6 &&
              (double)(curNs - req.capNs) / 1e6 <= cfg_.max_detection_age_ms) ||
            req.hasBox != hasBox_) {
            ++rejected_;
            Event e; e.type = 'R'; e.srcFrame = batchFrameId; e.reason = "stale_async_context";
            events_.push_back(e);
            ai.rejected = true;
            return;
        }
        lastDetectionId_ = batchFrameId;
        std::vector<Detection> dets;
        dets.reserve(raw.size());
        for (const Detection& d : raw) if (std::isfinite(d.score)) dets.push_back(d);
        const Src s{src, req.frameId, req.capNs};
        const Cur c{&curFrame, curId, curNs};
        applyObservations(s, dets, &c, req.hasBox ? &req.box : nullptr, req.scheduledCheck, ai);
    }

    // ------------------------------------------------------ py: finish_frame
    void finishFrame(long long capNs)
    {
        if (!hasBox_) return;
        const long long timeline = capNs;
        if (cfg_.tracker_low_release_ms != 0.0 && weak_ && haveTrackerLowStarted_ &&
            (double)(timeline - trackerLowStartedNs_) / 1e6 >= cfg_.tracker_low_release_ms) {
            lose("tracker_low_score_timeout", &timeline);
            return;
        }
        if (!weak_ || low_ < cfg_.tracker_low_patience) return;
        const bool timedOut = haveLowSince_ &&
            (double)(timeline - lowSinceNs_) / 1e6 >= cfg_.verification_timeout_ms;
        if ((misses_ >= cfg_.verification_miss_patience || timedOut) &&
            unsupportedMs(timeline) >= cfg_.detector_absence_grace_ms)
            lose("low_response_without_detector_support", &timeline);
    }

    // ------------------------------------------------------------ py: reset
    void reset(const std::string& reason)
    {
        lose(reason.c_str(), nullptr);
        state_ = State::SEARCHING;
        haveLastTracking_ = false;
        lastDetectStep_ = -1000000000LL;
        Event e; e.type = 'Z'; e.reason = reason; e.generation = generation_;
        pendingEvents_.push_back(e);
    }

private:
    struct Src { const cv::Mat* image; long long frameId; long long capNs; };
    struct Cur { const cv::Mat* image; long long frameId; long long capNs; };

    void transition(State s, const char* reason) {
        if (s == state_) return;
        Event e; e.type = 'T'; e.from = stateName(state_); e.to = stateName(s); e.reason = reason;
        events_.push_back(e);
        state_ = s;
    }

    // py: request_detection
    int requestDetection() {
        lastDetectStep_ = processed_;
        const bool tracking = hasBox_;
        verificationRequested_ = tracking && weak_;
        scheduledCheck_ = tracking && !weak_ && cfg_.verify_interval != 0;
        if (verificationRequested_) transition(State::VERIFYING, "low_response_verification");
        return generation_;
    }

    // py: _lose(reason, frame=None). timeline: frame verilmisse onun zamani.
    void lose(const char* reason, const long long* timeline) {
        Event ev;                                            // py: evidence (once hesaplanir)
        ev.hasEvidence = true;
        ev.filtered   = haveFiltered_ ? filtered_ : NAN;
        ev.low = low_; ev.misses = misses_; ev.unverified = unverified_;
        ev.unsupportedMs = timeline ? unsupportedMs(*timeline) : NAN;
        ev.detAvg  = haveDetAvg_ ? detAvg_ : NAN;
        ev.samples = (int)detScores_.size();
        ev.lowMs   = (timeline && haveTrackerLowStarted_)
                   ? (double)(*timeline - trackerLowStartedNs_) / 1e6 : NAN;
        ev.boxScale = (hasBox_ && haveReferenceArea_ && referenceArea_ != 0.0)
                    ? std::sqrt(boxArea(box_) / referenceArea_) : NAN;

        tracker_->reset();
        hasBox_ = hasCandidate_ = false;
        hits_ = low_ = misses_ = unverified_ = 0;
        haveReferenceArea_ = false; referenceArea_ = 0.0;
        haveFiltered_ = false; filtered_ = 0.0;
        detScores_.clear(); haveDetAvg_ = false; detAvg_ = 0.0; haveDetStarted_ = false;
        haveTrackerLowStarted_ = false;
        weak_ = false;
        haveLowSince_ = false;
        clamped_ = false;
        ++generation_;                                       // py: bekleyen sonuclari gecersiz kilar
        transition(State::LOST, reason);
        if (!events_.empty() && events_.back().type == 'T' && events_.back().to == "lost") {
            Event& b = events_.back();
            b.hasEvidence = true;
            b.filtered = ev.filtered; b.low = ev.low; b.misses = ev.misses; b.unverified = ev.unverified;
            b.unsupportedMs = ev.unsupportedMs; b.detAvg = ev.detAvg; b.samples = ev.samples;
            b.lowMs = ev.lowMs; b.boxScale = ev.boxScale;
        }
    }

    // py: _track_frame
    TrackOut trackFrame(const cv::Mat& frame, long long timeline)
    {
        TrackOut o;
        if (!hasBox_) return o;
        BoxF nb{}; float sc = 0.f, raw = 0.f;
        const bool ok = tracker_->track(frame, nb, sc, raw);
        o.tracked = true; o.box = nb; o.score = sc; o.raw = raw;
        o.valid = ok && nb.ok() && std::isfinite(sc);
        if (!o.valid) { lose("invalid_tracker_output", nullptr); return o; }
        box_ = nb;
        if (cfg_.max_box_growth != 0.0 && haveReferenceArea_ && referenceArea_ != 0.0) {
            const double scale = std::sqrt(boxArea(box_) / referenceArea_);
            if (!(1.0 / cfg_.max_box_growth <= scale && scale <= cfg_.max_box_growth)) {
                if (!cfg_.clamp_box_growth || unsupportedMs(timeline) >= cfg_.detector_absence_grace_ms) {
                    lose("box_scale_drift", &timeline);
                    return o;
                }
                if (!clamped_) { Event e; e.type = 'B'; e.value = scale; events_.push_back(e); }
                clampGrowth(frame, scale);
            } else {
                clamped_ = false;
            }
        }
        const double alpha = cfg_.tracker_score_ema_alpha;
        filtered_ = haveFiltered_ ? alpha * (double)sc + (1.0 - alpha) * filtered_ : (double)sc;
        haveFiltered_ = true;
        if (filtered_ >= cfg_.tracker_recover_score) {
            weak_ = false;
            low_ = misses_ = 0;
            haveLowSince_ = false;
            haveTrackerLowStarted_ = false;
            transition(State::TRACKING, "tracker_recovered");
        } else if (!weak_ && filtered_ < cfg_.tracker_low_score) {
            weak_ = true;
            haveLowSince_ = true; lowSinceNs_ = timeline;
            haveTrackerLowStarted_ = true; trackerLowStartedNs_ = timeline;
            low_ = misses_ = 0;
            lastDetectStep_ = -1000000000LL;   // py: ilk dusuk gozlem hemen tespit ister
        }
        if (weak_) {
            ++low_;
            transition(State::VERIFYING, "low_tracker_response");
        }
        return o;
    }

    // py: _clamp_growth
    void clampGrowth(const cv::Mat& frame, double scale)
    {
        const double factor = (scale > 1.0 ? cfg_.max_box_growth : 1.0 / cfg_.max_box_growth) / scale;
        const BoxF b = box_;
        const double cx = (b.x1 + b.x2) / 2.0, cy = (b.y1 + b.y2) / 2.0;
        const double hw = (b.x2 - b.x1) * factor / 2.0, hh = (b.y2 - b.y1) * factor / 2.0;
        const BoxF c = BoxF{cx - hw, cy - hh, cx + hw, cy + hh}.clipped((double)frame.cols, (double)frame.rows);
        if (!c.ok()) return;                                  // py: ValueError (olusmaz)
        box_ = c;
        clamped_ = true;
        if (tracker_->canRelocate()) tracker_->relocate(box_);
    }

    // py: _record_detector_score
    bool recordDetectorScore(long long timeline, const std::vector<Detection>& dets, const BoxF& reference)
    {
        const double windowMs = cfg_.detector_average_window_ms;
        if (windowMs == 0.0) return false;
        double score = 0.0; bool any = false;
        for (const Detection& d : dets)
            if (boxIoU(reference, d.box) >= cfg_.verification_iou) {
                if (!any || d.score > score) score = d.score;   // py: max(..., default=0.)
                any = true;
            }
        if (!any) score = 0.0;
        if (!haveDetStarted_) { haveDetStarted_ = true; detStartedNs_ = timeline; }
        detScores_.emplace_back(timeline, score);
        const long long cutoff = timeline - (long long)(windowMs * 1e6);   // py: int(window_ms * 1e6)
        while (!detScores_.empty() && detScores_.front().first < cutoff) detScores_.pop_front();
        double sum = 0.0;
        for (const auto& p : detScores_) sum += p.second;
        detAvg_ = sum / (double)detScores_.size();
        haveDetAvg_ = true;
        if ((double)(timeline - detStartedNs_) >= windowMs * 1e6 &&
            (int)detScores_.size() >= cfg_.detector_average_min_samples &&
            detAvg_ < cfg_.detector_average_release_confidence) {
            lose("detector_confidence_average_low", &timeline);
            return true;
        }
        return false;
    }

    // py: _apply_observations (asenkron yol: current_frame VAR, scheduled_check istekten)
    void applyObservations(const Src& src, const std::vector<Detection>& dets, const Cur* cur,
                           const BoxF* referenceBox, bool scheduledCheck, ApplyInfo& ai)
    {
        const long long srcT = src.capNs;
        if (hasBox_) {
            const BoxF reference = referenceBox ? *referenceBox : box_;
            if (recordDetectorScore(srcT, dets, reference)) return;
            const double vconf = cfg_.has_verify_confidence ? cfg_.detection_verify_confidence
                                                            : cfg_.detection_accept_confidence;
            const Detection* match = nullptr; double bestIoU = 0.0;
            for (const Detection& d : dets) {
                if (!(d.score >= vconf)) continue;
                const double i = boxIoU(reference, d.box);
                if (!(i >= cfg_.verification_iou)) continue;
                if (!match || i > bestIoU) { match = &d; bestIoU = i; }   // py: max -> ILK en buyuk
            }
            if (match) {
                ai.verified = true; ai.matchIoU = bestIoU; ai.matchScore = match->score;
                misses_ = low_ = unverified_ = 0;
                referenceArea_ = boxArea(match->box); haveReferenceArea_ = true;
                verifiedNs_ = srcT;
                if (weak_) { haveLowSince_ = true; lowSinceNs_ = srcT; } else { haveLowSince_ = false; }
                transition(State::TRACKING, "detector_verified");
                if (cfg_.correction_enabled) {
                    applyCorrection(cur ? *cur->image : *src.image,
                                    reference, match->box, src.frameId, ai);
                    return;
                }
                if (cfg_.refresh_template) {
                    if (cur) { acquire(src, match->box, "detector_verified", cur, nullptr, ai); return; }
                    box_ = match->box;
                    tracker_->initialize(*src.image, match->box);
                    return;
                }
            } else if (weak_) {
                ai.missed = true;
                ++misses_;
                transition(State::VERIFYING, "low_response_detector_missing");
            } else if (scheduledCheck) {
                ai.missed = true;
                ++unverified_;
                if (unverified_ >= cfg_.verify_miss_patience &&
                    unsupportedMs(srcT) >= cfg_.detector_absence_grace_ms)
                    lose("confident_track_without_detector_support", &srcT);
            }
            return;
        }
        // ---------------- yakalama: ardisik eslesen tespitler; onaylanana dek tracker yok
        std::vector<Detection> acc;
        for (const Detection& d : dets) if (d.score >= cfg_.detection_accept_confidence) acc.push_back(d);
        if (cfg_.acquisition_max_area_frac != 0.0) {
            const double limit = cfg_.acquisition_max_area_frac * (double)src.image->rows * (double)src.image->cols;
            std::vector<Detection> keep;
            for (const Detection& d : acc) if (boxArea(d.box) <= limit) keep.push_back(d);
            acc.swap(keep);
        }
        if (acc.empty()) {
            hasCandidate_ = false;
            hits_ = 0;
            transition(State::SEARCHING, "no_detection");
            return;
        }
        // py: _confirm
        const Detection* selected = nullptr;
        if (hasCandidate_) {
            double best = 0.0;
            for (const Detection& d : acc) {
                const double i = boxIoU(d.box, candidate_.box);
                if (!(i >= cfg_.confirmation_iou)) continue;
                if (!selected || i > best) { selected = &d; best = i; }
            }
        }
        if (selected) {
            ++hits_;
        } else {
            for (const Detection& d : acc)
                if (!selected || d.score > selected->score) selected = &d;   // py: max -> ILK en buyuk
            hits_ = 1;
        }
        const Detection chosen = *selected;
        candidate_ = chosen; hasCandidate_ = true; candidateNs_ = srcT;
        ai.candidate = true;
        transition(State::VERIFYING, "candidate_detection");
        if (hits_ >= cfg_.confirmation_hits) {
            const double sc = chosen.score;
            acquire(src, chosen.box, "confirmed_acquisition", cur, &sc, ai);
        }
    }

    // py: _acquire — kaynak karede sablon, guncel kareye bir adim catch-up
    void acquire(const Src& src, const BoxF& box, const char* reason, const Cur* cur,
                 const double* detectorScore, ApplyInfo& ai)
    {
        if (!tracker_->initialize(*src.image, box)) {
            // Python'da cokme (sablon kirpilamadi). Ucusta: adayi dusur, aramaya don.
            hasCandidate_ = false; hits_ = 0;
            transition(State::SEARCHING, "tracker_init_failed");
            return;
        }
        box_ = box; hasBox_ = true;
        hasCandidate_ = false;
        low_ = misses_ = unverified_ = 0;
        referenceArea_ = boxArea(box); haveReferenceArea_ = true;
        verifiedNs_ = src.capNs;
        haveFiltered_ = false; filtered_ = 0.0;
        weak_ = false;
        haveLowSince_ = false;
        haveTrackerLowStarted_ = false;
        if (detectorScore) {
            detScores_.clear();
            detScores_.emplace_back(src.capNs, *detectorScore);
            detAvg_ = *detectorScore; haveDetAvg_ = true;
            haveDetStarted_ = true; detStartedNs_ = src.capNs;
        }
        clamped_ = false;
        if (std::string(reason) == "confirmed_acquisition") ai.acquired = true;
        transition(State::TRACKING, reason);
        if (cur && cur->frameId != src.frameId) {
            catchup_ = trackFrame(*cur->image, cur->capNs);
            haveCatchup_ = true;
            Event e; e.type = 'K'; e.srcFrame = src.frameId; e.toFrame = cur->frameId;
            events_.push_back(e);
        }
    }

    // py: TrackerCorrector.apply + blend_center (mode center_blend)
    void applyCorrection(const cv::Mat& frame, const BoxF& reference, const BoxF& detection,
                         long long srcFrameId, ApplyInfo& ai)
    {
        const double w = cfg_.correction_weight;
        const double width = (double)frame.cols, height = (double)frame.rows;
        const BoxF current = box_;
        const double bw = current.x2 - current.x1, bh = current.y2 - current.y1;   // py: current.xywh[2:]
        if (!(0.0 < bw && bw <= width && 0.0 < bh && bh <= height)) return;   // py: ValueError (olusmaz)
        const double tx = (reference.x1 + reference.x2) / 2.0, ty = (reference.y1 + reference.y2) / 2.0;
        const double dx = (detection.x1 + detection.x2) / 2.0, dy = (detection.y1 + detection.y2) / 2.0;
        const double x = std::min(std::max(current.x1 + w * (dx - tx), 0.0), width - bw);
        const double y = std::min(std::max(current.y1 + w * (dy - ty), 0.0), height - bh);
        const BoxF corrected{x, y, x + bw, y + bh};
        const bool applied = !(corrected.x1 == current.x1 && corrected.y1 == current.y1 &&
                               corrected.x2 == current.x2 && corrected.y2 == current.y2);
        std::string method = "none";
        if (applied) {
            if (cfg_.correction_application != 2 && tracker_->canRelocate()) {
                tracker_->relocate(corrected);
                method = "relocate";
            } else {
                tracker_->initialize(frame, corrected);   // DUZELTILMIS GUNCEL kutu, GUNCEL kare
                method = "reinitialize";
            }
        }
        box_ = corrected;
        if (applied) {
            ai.corrected = true;
            ai.dx = corrected.x1 - current.x1; ai.dy = corrected.y1 - current.y1;
            Event e; e.type = 'C'; e.reason = method; e.srcFrame = srcFrameId;
            e.dx = ai.dx; e.dy = ai.dy;
            events_.push_back(e);
        }
    }

    ManagedTracker* tracker_;
    ManagerConfig   cfg_;

    State     state_ = State::SEARCHING;
    bool      hasBox_ = false;       BoxF box_{};
    bool      hasCandidate_ = false; Detection candidate_{};
    int       hits_ = 0, low_ = 0, misses_ = 0, unverified_ = 0;
    bool      haveReferenceArea_ = false; double referenceArea_ = 0.0;
    long long candidateNs_ = 0, verifiedNs_ = 0;
    int       generation_ = 0;
    long long lastId_ = -1, lastCaptureNs_ = 0;
    long long processed_ = 0;
    long long lastDetectStep_ = -1000000000LL;
    long long lastDetectionId_ = -1;
    long long rejected_ = 0;
    std::vector<Event> events_, pendingEvents_;
    bool      haveLastTracking_ = false; long long lastTrackingNs_ = 0;
    bool      haveFiltered_ = false; double filtered_ = 0.0;
    std::deque<std::pair<long long, double>> detScores_;
    bool      haveDetAvg_ = false; double detAvg_ = 0.0;
    bool      haveDetStarted_ = false; long long detStartedNs_ = 0;
    bool      haveTrackerLowStarted_ = false; long long trackerLowStartedNs_ = 0;
    bool      weak_ = false;
    bool      haveLowSince_ = false; long long lowSinceNs_ = 0;
    bool      clamped_ = false;
    bool      verificationRequested_ = false, scheduledCheck_ = false;
    bool      haveCatchup_ = false; TrackOut catchup_{};
};

} // namespace yo
