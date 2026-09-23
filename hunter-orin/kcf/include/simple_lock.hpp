// =====================================================================
// simple_lock.hpp — "bir kez kilitle, karisma" + ASENKRON tespit worker'i.
//                   (--hybrid-kcf)
//
// TASARIM (15 Eylul, ucus oncesi mutabakat):
//   ARAMA : YOLO surekli kosar. Son <lockWindow> sonucun <lockHits> tanesinde
//           AYNI yerde drone varsa -> KCF'i bir kez kur -> KILIT.
//   KILIT : SADECE KCF. Tespit DURUR (thread bosta). Hicbir sey karismaz.
//           Kilit "YOLO onaylamadi" diye ASLA dusmez.
//
// Onceki surumde kilitliyken periyodik kontrol ve "daha iyi hedefe gecis"
// vardi. Kaldirildi: gecisler 11-22 derecelik isinlanmalar uretiyordu
// (1337 karede 5 kez) ve kutunun BUYUMESINI zaten duzeltmiyordu.
//
// KILIDI DUSUREN UC YOL — ucu de TESPIT ONAYINDAN BAGIMSIZ:
//   1) KCF <invalidLimit> kare ust uste gecersiz kutu verir (kare disi).
//   2) Kutu <outsideLimit> kare ust uste guvenli alan disinda kalir.
//      (Drone'u 180 cevirip hedefi kaybetme senaryosu buradan cikar.)
//   3) OPSIYONEL varlik kontrolu: <presencePeriod> karede BIR tespit.
//      Karede HICBIR YERDE drone yoksa sayac artar; <presenceMisses> kez
//      ust uste bos cikarsa kilit duser.
//      DIKKAT: "benim kutumu onaylamadi" DEGIL, "karede drone YOK".
//      YOLO arada kacirsa bile bir yerde drone goruyorsa kilit DURUR.
//      Varsayilan KAPALI (presencePeriod = 0).
//
// BUYUME SINIRI (ref guncellemeli — "B"):
//   Kutu <growthMinPx> esiginin ustundeyken kilit referansinin
//   <growthTrigger> katini asarsa, ayni merkezde referansin <growthReset>
//   katina cekilir ve KCF YENIDEN KURULUR (biriken arka plan modeli silinir).
//   Sonra REFERANS YENI BOYUTA GUNCELLENIR. Boylece gercek yaklasmada kutu
//   basamak basamak buyuyebilir, surukleme ise her basamakta kesilir.
//     50x50 kilit -> 125x125'te tetikler -> 75x75'e doner (yeni ref 75)
//    100x100 kilit -> 250x250'de tetikler -> 150x150'e doner (yeni ref 150)
//
// ASENKRON TESPIT: ana dongu detect()'i HIC beklemez. Kilitliyken tespit
// zaten kosmadigi icin kare periyodu sabit kalir.
// =====================================================================
#pragma once

#include "target_manager.hpp"   // hybrid::BoxF, Detection, boxIoU
#include "yolo_trt.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace slock {

using hybrid::BoxF;
using hybrid::Detection;
using hybrid::boxIoU;

struct Config {
    // --- yakalama: "30 sonucun 20'sinde"
    int    lockWindow   = 30;
    int    lockHits     = 20;
    double lockIoU      = 0.20;   // ardisik tespitler ayni hedef mi
    double minConf      = 0.40;   // tespit kabul esigi

    // --- guvenlik aglari
    int    invalidLimit = 30;     // KCF ardisik gecersiz kutu
    int    outsideLimit = 60;     // kutu guvenli alan DISINDA ardisik kare
    int    presencePeriod = 0;    // 0 = KAPALI. N karede bir varlik kontrolu
    int    presenceMisses = 3;    // ust uste kac bos kontrolde kilit duser

    // --- buyume sinirlamasi
    double growthTrigger = 2.5;   // referansin kac kati tetikler
    double growthReset   = 1.5;   // referansin kac katina cekilir
    double growthMinPx   = 50.0;  // bu boyutun altinda kural UYGULANMAZ
};

enum class State { SEARCH, LOCK };
inline const char* stateName(State s) { return s == State::LOCK ? "locked" : "search"; }

// ---------------------------------------------------------------------
class SimpleLock {
public:
    explicit SimpleLock(const Config& c) : cfg_(c) {}

    State       state()       const { return state_; }
    bool        hasBox()      const { return hasBox_; }
    const BoxF& box()         const { return box_; }
    bool        hasCand()     const { return hasCand_; }
    const BoxF& cand()        const { return cand_; }
    int         hits()        const { return hitCount_; }
    int         window()      const { return (int)win_.size(); }
    long long   lockFrames()  const { return lockFrames_; }
    long long   clamps()      const { return clamps_; }
    int         presenceMiss()const { return presMiss_; }
    const std::string& lastEvent() const { return event_; }
    void clearEvent() { event_.clear(); }

    // Bu karede tespit sonucu ISTIYOR muyuz?
    //   ARAMA : her zaman
    //   KILIT : sadece varlik kontrolu penceresi acikken (varsayilan hic)
    bool detectionWanted() const { return state_ == State::SEARCH || presOpen_; }

    // ---- 1) Kare basi: KCF sonucu (sadece KILIT'te anlamli)
    void onTrackerResult(bool valid, const BoxF& b) {
        if (state_ != State::LOCK) return;
        ++lockFrames_;

        if (valid && b.ok()) { box_ = b; hasBox_ = true; invalidRun_ = 0; }
        else if (++invalidRun_ >= cfg_.invalidLimit) {
            event_ = "kcf_invalid_streak";
            toSearch();
            return;
        }

        checkGrowth();

        if (cfg_.presencePeriod > 0 && !presOpen_ &&
            lockFrames_ - lastPres_ >= cfg_.presencePeriod) {
            presOpen_ = true;
        }
    }

    // ---- 2) Tespit sonucu (asenkron, gec gelmesi SORUN DEGIL)
    void onDetections(const std::vector<Detection>& raw) {
        std::vector<const Detection*> d;
        for (const Detection& x : raw)
            if (x.score >= cfg_.minConf && x.box.ok()) d.push_back(&x);

        if (state_ == State::SEARCH) { searchStep(d); return; }
        if (!presOpen_) return;

        // VARLIK kontrolu: kutuyla KARSILASTIRMA YOK. Sadece "karede drone
        // var mi". Kendi kutumuzun onaylanmasini SART KOSMUYORUZ.
        presOpen_ = false;
        lastPres_ = lockFrames_;
        if (!d.empty()) { presMiss_ = 0; return; }
        if (++presMiss_ >= cfg_.presenceMisses) {
            event_ = "no_drone_in_frame";
            toSearch();
        }
    }

    // ---- 3) Kutu kullanilabilir bolgede mi (guvenli alan)
    void onUsable(bool ok) {
        if (state_ != State::LOCK) return;
        if (ok) { outsideRun_ = 0; return; }
        if (++outsideRun_ >= cfg_.outsideLimit) {
            event_ = "outside_safe_zone";
            toSearch();
        }
    }

    // ---- 4) Kare sonu: KCF (yeniden) kurulmali mi?
    bool pendingInit(BoxF& b) const {
        if (!wantInit_) return false;
        b = initBox_;
        return true;
    }
    void initDone(bool ok) {
        if (!wantInit_) return;
        wantInit_ = false;
        if (!ok) { event_ = "tracker_init_failed"; toSearch(); return; }
        box_    = initBox_;
        hasBox_ = true;
        state_  = State::LOCK;
        // [B] Referans HER kurulumda guncellenir: ilk kilitte YOLO kutusu,
        // her sinirlamada yeni (kucultulmus) boyut. Gercek yaklasmada kutu
        // basamak basamak buyuyebilir; surukleme her basamakta kesilir.
        refW_ = initBox_.w();
        refH_ = initBox_.h();
        hasCand_ = false; hitCount_ = 0; win_.clear();
        invalidRun_ = 0; outsideRun_ = 0;
    }

    void release(const char* why) {
        if (state_ == State::SEARCH && !hasCand_) return;
        event_ = why;
        toSearch();
    }

private:
    void toSearch() {
        state_ = State::SEARCH;
        hasBox_ = false; hasCand_ = false;
        hitCount_ = 0; win_.clear();
        invalidRun_ = 0; outsideRun_ = 0;
        lockFrames_ = 0; lastPres_ = 0; presOpen_ = false; presMiss_ = 0;
        refW_ = refH_ = 0.0;
        wantInit_ = false;
    }

    // "Son <window> sonucun <hits> tanesinde ayni hedef" — KAYAN pencere.
    // Tek bir kacirma sayaci SIFIRLAMAZ; uzak hedefte YOLO arada kacirir.
    void searchStep(const std::vector<const Detection*>& d) {
        const Detection* m = nullptr;
        if (hasCand_) {
            double best = -1.0;
            for (const Detection* x : d) {
                const double i = boxIoU(cand_, x->box);
                if (i >= cfg_.lockIoU && i > best) { best = i; m = x; }
            }
        }
        if (!m && !hasCand_) {
            double best = -1.0;
            for (const Detection* x : d)
                if (x->score > best) { best = x->score; m = x; }
        }
        if (m) { cand_ = m->box; hasCand_ = true; }
        push(m != nullptr);

        // Pencere dolu ve hedef kaybolduysa adayi birak
        const int allowedMiss = cfg_.lockWindow - cfg_.lockHits;
        if (hasCand_ && (int)win_.size() >= cfg_.lockWindow &&
            hitCount_ < cfg_.lockHits - allowedMiss) {
            hasCand_ = false; hitCount_ = 0; win_.clear();
            event_ = "candidate_dropped";
            return;
        }
        if (hasCand_ && hitCount_ >= cfg_.lockHits) {
            initBox_  = cand_;
            wantInit_ = true;
            char b[128];
            std::snprintf(b, sizeof(b), "lock %d/%d  (%.0f,%.0f %.0fx%.0f)",
                          hitCount_, (int)win_.size(),
                          cand_.x1, cand_.y1, cand_.w(), cand_.h());
            event_ = b;
        }
    }

    void checkGrowth() {
        if (!hasBox_ || wantInit_) return;
        if (!(refW_ > 0.0 && refH_ > 0.0)) return;
        const double w = box_.w(), h = box_.h();
        if (std::min(w, h) < cfg_.growthMinPx) return;      // kucukken kural yok
        const double scale = std::sqrt((w * h) / (refW_ * refH_));
        if (scale < cfg_.growthTrigger) return;

        const double nw = refW_ * cfg_.growthReset;
        const double nh = refH_ * cfg_.growthReset;
        const double cx = 0.5 * (box_.x1 + box_.x2);
        const double cy = 0.5 * (box_.y1 + box_.y2);
        initBox_  = BoxF{cx - nw / 2.0, cy - nh / 2.0, cx + nw / 2.0, cy + nh / 2.0};
        wantInit_ = true;
        ++clamps_;
        char b[160];
        std::snprintf(b, sizeof(b),
            "growth_clamp x%.2f  %.0fx%.0f -> %.0fx%.0f  (ref %.0fx%.0f -> %.0fx%.0f)",
            scale, w, h, nw, nh, refW_, refH_, nw, nh);
        event_ = b;
    }

    void push(bool hit) {
        win_.push_back(hit ? 1 : 0);
        if (hit) ++hitCount_;
        while ((int)win_.size() > cfg_.lockWindow) {
            if (win_.front()) --hitCount_;
            win_.pop_front();
        }
    }

    Config cfg_;
    State  state_ = State::SEARCH;
    BoxF   box_{}, cand_{}, initBox_{};
    bool   hasBox_ = false, hasCand_ = false, wantInit_ = false;
    std::deque<int> win_;
    int    hitCount_ = 0, invalidRun_ = 0, outsideRun_ = 0;
    long long lockFrames_ = 0, lastPres_ = 0;
    bool   presOpen_ = false;
    int    presMiss_ = 0;
    double refW_ = 0.0, refH_ = 0.0;
    long long clamps_ = 0;
    std::string event_;
};

// ---------------------------------------------------------------------
// AsyncDetector — YOLO'yu ayri thread'de kosturur. Tek slot, EN TAZE kazanir:
// worker mesgulken gelen istek bekleyeni EZER, kuyruk birikmez. Ana dongu
// detect()'i HIC beklemez.
//
// Thread guvenligi: YoloDetector::detect() SADECE worker thread'inden
// cagrilir; TensorRT context ve CUDA stream'i tek thread kullanir.
// ---------------------------------------------------------------------
class AsyncDetector {
public:
    explicit AsyncDetector(hybrid::YoloDetector* yolo) : yolo_(yolo) {
        th_ = std::thread([this] { run(); });
    }
    ~AsyncDetector() {
        { std::lock_guard<std::mutex> g(m_); stop_ = true; }
        cv_.notify_all();
        if (th_.joinable()) th_.join();
    }

    void request(const cv::Mat& frame) {
        {
            std::lock_guard<std::mutex> g(m_);
            frame.copyTo(pending_);
            hasPending_ = true;
        }
        cv_.notify_one();
    }

    bool poll(std::vector<Detection>& out) {
        std::lock_guard<std::mutex> g(m_);
        if (!hasResult_) return false;
        out.swap(result_);
        result_.clear();
        hasResult_ = false;
        return true;
    }

    bool   busy()     const { return busy_.load(); }
    double lastMs()   const { return lastMs_.load(); }
    long long calls() const { return calls_.load(); }

private:
    void run() {
        cv::Mat work;
        std::vector<Detection> out;
        for (;;) {
            {
                std::unique_lock<std::mutex> g(m_);
                cv_.wait(g, [this] { return stop_ || hasPending_; });
                if (stop_) return;
                pending_.copyTo(work);
                hasPending_ = false;
            }
            busy_ = true;
            out.clear();
            const bool ok = yolo_->detect(work, out);
            lastMs_ = yolo_->lastMs();
            ++calls_;
            busy_ = false;
            {
                std::lock_guard<std::mutex> g(m_);
                result_ = ok ? out : std::vector<Detection>();
                hasResult_ = true;
            }
        }
    }

    hybrid::YoloDetector* yolo_;
    std::thread th_;
    std::mutex  m_;
    std::condition_variable cv_;
    cv::Mat pending_;
    std::vector<Detection> result_;
    bool hasPending_ = false, hasResult_ = false, stop_ = false;
    std::atomic<bool>      busy_{false};
    std::atomic<double>    lastMs_{0.0};
    std::atomic<long long> calls_{0};
};

} // namespace slock
