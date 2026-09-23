// =====================================================================
// tracker_backend.hpp — runtracker'in tek tracker arayuzu.
//
// AMAC: ana dongu hangi tracker'in kostugunu BILMESIN. Dort mod da ayni
// sozlesmeyi kullanir:
//     (bayraksiz)    KCF     , manuel ROI
//     --ortrack      ORTrack , manuel ROI
//     --hybrid       YOLO + ORTrack
//     --hybrid-kcf   YOLO + KCF
//
// KCF IKI arayuzu birden uygular:
//   TrackerBackend         -> manuel ROI modu (cv::Rect)
//   hybrid::ManagedTracker -> hibrit mod, TargetManager surer (BoxF)
// Ayni nesne, ayni durum. reset() iki tabanda da ayni imzada oldugu icin
// tek override ikisini birden karsilar.
//
// GERI UYUMLULUK: bayraksiz calisirken kutu uretimi eskisiyle BIREBIR
// aynidir. Tek eklenen sey score() — artik KCF tepe yanitini dondurur
// (once -1 doniyordu). Sadece rozet/CSV icin; takip mantigi degismedi.
// =====================================================================
#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "kcftracker.hpp"
#include "target_manager.hpp"   // hybrid::ManagedTracker, BoxF

struct TrackerBackend {
    virtual ~TrackerBackend() {}

    // Kilit. false donerse kilit KURULMADI (cagiran trackingActive yapmamali).
    virtual bool init(const cv::Mat& frame, const cv::Rect& roi) = 0;

    // Bir kare isle. false = bu karede gecerli kutu URETILEMEDI.
    // false dondugunde box SON GECERLI kutuyla doldurulur (cizim bozulmasin).
    virtual bool update(const cv::Mat& frame, cv::Rect& box) = 0;

    virtual void reset() = 0;

    // Gozlem icin (OSD / log / CSV). Desteklemeyen backend -1 doner.
    virtual float  score()    const { return -1.0f; }
    virtual float  rawScore() const { return -1.0f; }
    virtual double lastMs()   const { return 0.0; }   // kare basina toplam
    virtual double inferMs()  const { return 0.0; }   // sadece ag ileri gecisi

    virtual const char* name() const = 0;
};

// ---------------------------------------------------------------------
// KCF — hem manuel hem yonetilen mod.
//
// Kurucu parametreleri eski runtracker satirindan birebir alindi:
//     KCFTracker tracker(true, true, true, false);
//     (hog=true, fixed_window=true, multiscale=true, lab=false)
// ---------------------------------------------------------------------
struct KcfTrackerHandle : public TrackerBackend, public hybrid::ManagedTracker {
    virtual ~KcfTrackerHandle() {}
};

class KcfBackend : public KcfTrackerHandle {
public:
    explicit KcfBackend(double minBoxSize = 10.0) : minBox_(minBoxSize) {}

    // ------------------------------------------------- TrackerBackend
    bool init(const cv::Mat& frame, const cv::Rect& roi) override {
        return initBox(frame, hybrid::BoxF::fromRect(roi));
    }

    bool update(const cv::Mat& frame, cv::Rect& box) override {
        hybrid::BoxF b; float sc = 0.f, raw = 0.f;
        const bool ok = track(frame, b, sc, raw);
        if (have_) box = box_.toRect();
        return ok;
    }

    // -------------------------------------- hybrid::ManagedTracker
    bool initialize(const cv::Mat& frame, const hybrid::BoxF& box) override {
        return initBox(frame, box);
    }

    bool track(const cv::Mat& frame, hybrid::BoxF& box,
               float& score, float& raw) override
    {
        const int64 t0 = cv::getTickCount();
        score = raw = peak_;
        if (!kcf_ || !have_ || frame.empty()) return fail(box, t0);

        const cv::Rect r = kcf_->update(frame);
        peak_ = kcf_->peak_value;
        score = raw = peak_;
        ms_ = (double)(cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();

        if (!std::isfinite(peak_)) return fail(box, t0);

        // [SHRINK] KCF daraltilmis kutuyu surer; disariya TAM boyutu bildir.
        hybrid::BoxF nb = hybrid::BoxF::fromRect(r);
        if (!nb.ok()) return fail(box, t0);
        if (shrink_ < 0.999) {
            const double cx = 0.5 * (nb.x1 + nb.x2), cy = 0.5 * (nb.y1 + nb.y2);
            const double hw = 0.5 * nb.w() / shrink_, hh = 0.5 * nb.h() / shrink_;
            nb = hybrid::BoxF{cx - hw, cy - hh, cx + hw, cy + hh};
            if (!nb.ok()) return fail(box, t0);
        }

        // ORTrack yolundaki clip_tracker_box ile AYNI semantik: tamamen kare
        // disi kutu reddedilir, kalan kutu min_box tabaniyla kirpilir. KCF
        // kendi basina "gecersiz" diyemez; dejenere kutuyu BURADA yakaliyoruz.
        if (!nb.clipped((double)frame.cols, (double)frame.rows).ok()) return fail(box, t0);

        const double W = frame.cols, H = frame.rows;
        const double m  = std::min(minBox_, std::min(W, H));
        const double x1 = std::min(std::max(0.0, nb.x1), W - m);
        const double y1 = std::min(std::max(0.0, nb.y1), H - m);
        const double x2 = std::min(std::max(m, nb.x2), W);
        const double y2 = std::min(std::max(m, nb.y2), H);
        hybrid::BoxF out = hybrid::BoxF::fromXYWH(x1, y1,
                               std::max(m, x2 - x1), std::max(m, y2 - y1));
        if (!out.ok()) return fail(box, t0);

        box_ = out;
        box  = out;
        return true;
    }

    void reset() override {
        kcf_.reset();
        have_ = false;
        peak_ = 0.f;
        ms_   = 0.0;
        box_  = hybrid::BoxF{};
    }

    float  score()  const override { return have_ ? peak_ : -1.0f; }
    double lastMs() const override { return ms_; }
    float  padding() const { return pad_; }   // [PAD] tanilama
    const char* name() const override { return "KCF"; }

private:
    bool fail(hybrid::BoxF& box, int64 t0) {
        if (have_) box = box_;
        ms_ = (double)(cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();
        return false;
    }

    // -----------------------------------------------------------------
    // [PAD] UYARLAMALI PADDING — buyuk kutuda KCF'in cokmesini engeller.
    //
    // KCF arama penceresi = kutu x padding (kcftracker.cpp:422,451) ve bu
    // pencere SABIT bir sablona (96 px) indirilip O KAREDEKI icerikle bir
    // filtre EGITILIR. Pencere kareyi tasarsa tasan kisim SIYAH dolar
    // (recttools.hpp:102, BORDER_CONSTANT).
    //
    // Sorun: o siyah bant KAREYE sabittir, hedefe degil. Hedef hareket
    // eder, bant yerinde kalir. Filtre icin en tutarli desen odur; en iyi
    // eslesme her zaman bandin hizalandigi yer olur ve KUTU DONAR.
    // 15 Eylul tezgah olcumu: 355x370 px kutu -> pencere 1704x1776,
    // kare 1920x1080 -> %39 siyah -> kutu 90 kare boyunca hic kimildamadi,
    // hedef kareyi boydan boya gecti (IoU medyani 0.038).
    //
    // Cozum: pencereyi kareye sigdir. Kucuk kutuda min() zaten 4.8'i secer,
    // yani UCUS BOYUTLARINDA TAM NO-OP; sadece dejenere vakayi kurtarir.
    // 1080p'de esik ~225 px kutu yuksekligi.
    //
    // 0.9 pay: kutu tam merkezde olmayabilir. Taban 1.5: KCF'in ayirt
    // edici baglama ihtiyaci var, cok daralirsa arama alani kalmaz.
    // -----------------------------------------------------------------
    static float fitPadding(int frameW, int frameH, int boxW, int boxH) {
        if (boxW <= 0 || boxH <= 0) return 4.8f;
        const double fit = std::min((double)frameW / (double)boxW,
                                    (double)frameH / (double)boxH);
        return (float)std::max(1.5, std::min(4.8, 0.9 * fit));
    }

    // -----------------------------------------------------------------
    // [SHRINK] Buyuk kutuda padding'i DUSURMEK yerine KUTUYU KUCULT.
    //
    // Padding'i 4.8'den 3.0'a cekmek pencereyi sigdirir ama KCF'in
    // geometrisini bozar: Gauss egitim hedefi ve kosinus penceresi hedefin
    // pencerenin 1/4.8'ini kaplamasina gore kurulu. 15 Eylul olcumu:
    //   manuel KCF (kutu 154 px, padding 4.8) -> merkez gurultusu p95 0.46 derece
    //   hibrit    (kutu 346 px, padding 3.0) -> p95 1.21 derece   (2.6 kat)
    //
    // Cozum: KCF'e kutunun <shrink> katini ver, RAPORLARKEN geri buyut.
    // KCF olcegi tek katsayiyla degistirdigi icin oran korunur, dolayisiyla
    // bildirilen kutu (ve bbox yakinlik vekili) DEGISMEZ.
    // 346x322 -> shrink 0.70 -> KCF kutusu 242x225 -> pencere 1162x1080,
    // tam sigiyor ve padding 4.8'de KALIYOR.
    //
    // Taban 0.45: cok kucultursek KCF'in HOG hucrelerine yeterli doku kalmaz.
    // Kucuk kutuda min() 1.0 secer -> UCUS BOYUTLARINDA TAM NO-OP.
    // -----------------------------------------------------------------
    static double fitShrink(int frameW, int frameH, int boxW, int boxH) {
        if (boxW <= 0 || boxH <= 0) return 1.0;
        const double s = std::min((double)frameW / (4.8 * (double)boxW),
                                  (double)frameH / (4.8 * (double)boxH));
        return std::max(0.45, std::min(1.0, s));
    }

    bool initBox(const cv::Mat& frame, const hybrid::BoxF& roi) {
        reset();
        if (frame.empty()) return false;
        cv::Rect r = roi.toRect() & cv::Rect(0, 0, frame.cols, frame.rows);
        if (r.width <= 1 || r.height <= 1) return false;

        // [SHRINK] KCF'e daralttigimiz kutuyu ver; raporlarken 1/shrink ile ac.
        shrink_ = fitShrink(frame.cols, frame.rows, r.width, r.height);
        cv::Rect kr = r;
        if (shrink_ < 0.999) {
            const int nw = std::max(8, (int)std::lround(r.width  * shrink_));
            const int nh = std::max(8, (int)std::lround(r.height * shrink_));
            kr = cv::Rect(r.x + (r.width - nw) / 2, r.y + (r.height - nh) / 2, nw, nh);
            kr &= cv::Rect(0, 0, frame.cols, frame.rows);
            if (kr.width <= 1 || kr.height <= 1) { kr = r; shrink_ = 1.0; }
        }
        pad_ = fitPadding(frame.cols, frame.rows, kr.width, kr.height);

        // KCFTracker'in reset()'i YOK: _scale / _tmpl / _alphaf onceki
        // kilitten kalir. Temiz baslangic icin nesne YENIDEN kuruluyor.
        // Imza: (hog, fixed_window, multiscale, lab, cn, gray, rectTh, padding)
        kcf_.reset(new KCFTracker(true, true, true, false, false, false, 100, pad_));
        kcf_->init(kr, frame);
        box_  = hybrid::BoxF::fromRect(r);   // RAPORLANAN kutu: orijinal boyut
        peak_ = 0.f;
        have_ = true;
        if (shrink_ < 0.999 || pad_ < 4.79f)
            std::cout << "[KCF] kutu " << r.width << "x" << r.height
                      << " -> KCF'e " << kr.width << "x" << kr.height
                      << " (shrink " << shrink_ << ", padding " << pad_
                      << ", pencere " << (int)(kr.width * pad_) << "x"
                      << (int)(kr.height * pad_) << ")\n";
        return true;
    }

    std::unique_ptr<KCFTracker> kcf_;
    hybrid::BoxF box_{};
    bool   have_  = false;
    float  peak_  = 0.f;
    double ms_    = 0.0;
    double minBox_ = 10.0;
    float  pad_    = 4.8f;
    double shrink_ = 1.0;
};

inline std::unique_ptr<KcfTrackerHandle> makeKcfTracker(double minBoxSize = 10.0) {
    return std::unique_ptr<KcfTrackerHandle>(new KcfBackend(minBoxSize));
}
