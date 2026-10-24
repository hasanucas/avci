// =====================================================================
// roi_pointer.hpp — [PTR] GUIDED'da sag cubukla secim kutusu (imlec)
//
// NEDEN: follow_gps takipciyi GUIDED'da ucururken ArduCopter roll/pitch/
// throttle cubuklarini YOK SAYAR. Pilotun sag eli bos kalir. Bu modul o
// cubugu ekrandaki secim kutusuna baglar; CH12 ileri ile o kutuda kilitlenir.
// (Dikkat: yaw HARIC. GUID_OPTIONS=8'de "pilot yaw'ini yok say" biti (4)
//  kapali, sol cubuk GUIDED'da drone'u hala dondurur. Imlec sol cubugu
//  KULLANMAZ.)
//
// KAPI (tek kural, her kare):
//   OFF    FC modu GUIDED degil (ya da --no-pointer). Kutu ESKISI GIBI ortada,
//          cubuk kutuya BAGLI DEGIL -> cubuklar sadece drone'u ucurur.
//   LIVE   FC GUIDED + heartbeat taze + RC taze ve gecerli -> cubuk kutuyu surer.
//   FROZEN Son bilinen mod GUIDED ama heartbeat ya da RC bayat/gecersiz ->
//          kutu YERINDE donar (ne ortaya ziplar ne kayar). Tazelenince LIVE.
//   GUIDED_NOGPS (20) GUIDED DEGILDIR: saldiri sirasinda imlec KAPALI.
//
// MASA DEMOSU (--ptr-demo / --ptr-demo-ch N): GPS fix'siz masada FC GUIDED'a
//   giremez. Demo, FC modu yerine sahte GUIDED kullanir: kanalsiz hep GUIDED,
//   kanalla CHN >1600 "GUIDED", <1350 "degil" (follow_gps'in alip pilotun geri
//   almasini taklit eder). SADECE DISARM'dayken: arm edilince demo kendiliginden
//   kapanir ve gercek kural (FC GUIDED) gecerli olur.
//
// HIZ MODU: cubuk sapmasi -> kutu HIZI. Birakinca kutu oldugu yerde kalir.
//   a = olu banttan sonra 0..1 (bant kenarinda sicrama yok)
//   expo: a' = (1-e)*a + e*a^3   (ortada ince nisan, uca dogru hizli)
//   v = speed * kare_genisligi * a'   [px/s], iki eksende AYNI (izotrop)
// SINIR: kutu merkezi %85 guvenli alanin icinde (disarida kilit zaten
//        hemen duser), kutu kareden tasmaz.
// MANUEL UYUM: imlec hic oynamadiysa uretilen kutu eski updateCenterRoi()
//        ile BIREBIR ayni (tamsayi w/2 - size/2). Test bunu kilitliyor.
//
// OpenCV'ye BAGLI DEGIL -> tests/test_roi_pointer.cpp kamera/FC'siz kosar.
// =====================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ptr {

constexpr int MODE_GUIDED = 4;   // ArduCopter custom_mode. GUIDED_NOGPS = 20 (imlec KAPALI)

struct Config {
    bool   enabled   = true;   // --no-pointer ile kapanir
    int    chX       = 1;      // sag cubuk sag-sol    (Mode 2: CH1 roll)
    int    chY       = 2;      // sag cubuk ileri-geri (Mode 2: CH2 pitch)
    int    center    = 1495;   // RC1/RC2_TRIM (MK15 olcumu)
    int    halfRange = 450;    // 1045..1945
    int    deadband  = 30;     // us — orta civarindaki titreme kutuyu kaydirmasin
    double expo      = 0.6;    // 0 = dogrusal, 1 = tam kubik
    double speed     = 0.5;    // tam sapma: saniyede kare GENISLIGININ bu kadari
    bool   invertX   = false;
    bool   invertY   = false;  // varsayilan: ileri (ArduPilot'ta DUSUK PWM) -> YUKARI
    int    rcStaleMs = 500;    // RC_CHANNELS ~10 Hz istiyoruz
    int    hbStaleMs = 2500;   // HEARTBEAT 1 Hz
    // MASA DEMOSU (ucusta etkisiz: sadece disarm'da calisir)
    bool   demo      = false;  // --ptr-demo / --ptr-demo-ch N
    int    demoCh    = 0;      // 0: hep GUIDED. N: CHN anahtari GUIDED'i taklit eder
};

inline bool pwmValid(int pwm) { return pwm >= 900 && pwm <= 2200; }

// Imlec ve demo anahtari icin KULLANILAMAYACAK kanallar: ucus modu (5), otonom
// kapisi (6), pencere (8), EKF kaynak anahtari (10), ARM (11), kilit (12),
// boyut (13), RSSI (14). Demo anahtari CH11 olsaydi "GUIDED taklidi" ARM ederdi.
inline bool reservedChannel(int ch) {
    return ch < 1 || ch > 16 || ch == 5 || ch == 6 || ch == 8 || ch == 10 ||
           ch == 11 || ch == 12 || ch == 13 || ch == 14;
}

// Demo yalnizca DISARM'dayken. Bayrak unutulup ucusa cikilsa da arm anindan
// itibaren gercek kural gecerli.
inline bool demoActive(const Config& c, bool armed) { return c.demo && !armed; }

// Demo'da kapiya verilecek etkin mod.
//   demoCh == 0 : hep GUIDED
//   demoCh  > 0 : kanal >1600 -> GUIDED, <1350 -> GUIDED degil (0),
//                 ara bolge ya da gecersiz PWM -> onceki (histerezis)
inline int demoEffectiveMode(const Config& c, int demoPwm, int prevMode) {
    if (c.demoCh <= 0)       return MODE_GUIDED;
    if (!pwmValid(demoPwm))  return prevMode;
    if (demoPwm > 1600)      return MODE_GUIDED;
    if (demoPwm < 1350)      return 0;
    return prevMode;
}

enum class Gate { Off, Live, Frozen };

inline const char* gateName(Gate g) {
    switch (g) {
        case Gate::Live:   return "LIVE";
        case Gate::Frozen: return "FROZEN";
        default:           return "OFF";
    }
}

// PWM -> [-1, 1]. Gecersiz PWM -> 0.
inline double stickAxis(int pwm, const Config& c) {
    if (!pwmValid(pwm)) return 0.0;
    const double d  = (double)(pwm - c.center);
    const double ad = std::fabs(d);
    if (ad <= (double)c.deadband) return 0.0;
    const double span = (double)std::max(1, c.halfRange - c.deadband);
    double a = std::min(1.0, (ad - (double)c.deadband) / span);
    const double e = std::min(1.0, std::max(0.0, c.expo));
    a = (1.0 - e) * a + e * a * a * a;
    return d < 0.0 ? -a : a;
}

// fcMode : son FC heartbeat'indeki custom_mode (-1 = hic gelmedi)
// *AgeMs : son mesajdan bu yana gecen sure (-1 = hic gelmedi)
inline Gate gate(const Config& c, int fcMode, int64_t hbAgeMs, int64_t rcAgeMs,
                 int pwmX, int pwmY) {
    if (!c.enabled || fcMode != MODE_GUIDED)   return Gate::Off;
    if (hbAgeMs < 0 || hbAgeMs > c.hbStaleMs)  return Gate::Frozen;
    if (rcAgeMs < 0 || rcAgeMs > c.rcStaleMs)  return Gate::Frozen;
    if (!pwmValid(pwmX) || !pwmValid(pwmY))    return Gate::Frozen;
    return Gate::Live;
}

// GUIDED bitti (OFF'a dusuldu) ve takip yoksa kutu ortaya doner: manuel KCF
// duzeni (pilot drone'u ucurup hedefi ortadaki kutuya getirir). Takip VARSA
// dokunulmaz: takip suruyor, sadece cubuklar drone'a doner.
inline bool shouldRecenter(Gate prev, Gate now, bool tracking) {
    return prev != Gate::Off && now == Gate::Off && !tracking;
}

// Kilit bitince (CH12 geri ya da otomatik dusme) kutu nereye?
//   imlec acikken (LIVE/FROZEN): son takip noktasi -> yeniden kilit kolay
//   kapaliyken                 : eski davranis (orta)
inline bool keepAtLastTrack(Gate g) { return g != Gate::Off; }

class SelectBox {
public:
    // w,h: isleme karesi. safeMarginRatio: runtracker'daki guvenli alan
    // orani (0.075 = %85 alan). Boyutlar runtracker'daki ROI sabitleri.
    void setup(int w, int h, float safeMarginRatio,
               int base, int minSize, int maxSize, int step) {
        w_ = std::max(2, w);
        h_ = std::max(2, h);
        const int mx = (int)(w_ * safeMarginRatio);   // runtracker ile AYNI ifade
        const int my = (int)(h_ * safeMarginRatio);
        minX_ = mx; maxX_ = w_ - mx;
        minY_ = my; maxY_ = h_ - my;
        minS_ = std::max(2, minSize);
        maxS_ = std::max(minS_, maxSize);
        base_ = std::min(maxS_, std::max(minS_, base));
        step_ = std::max(1, step);
        open();
    }

    // CH8 acildi: orta + taban boyut (eskisi gibi)
    void open()      { size_ = base_; recenter(); }
    // Manuel birakma: orta + taban boyut (eskisi gibi)
    void resetBase() { open(); }
    // Ortaya al, boyut KORUNUR. Tamsayi bolme: eski w/2 ile birebir.
    void recenter()  { cx_ = (double)(w_ / 2); cy_ = (double)(h_ / 2); }

    // CH13 adimi. Merkez KORUNUR (manuelde merkez zaten orta).
    void resize(int dir) {
        if (dir == 0) return;
        size_ = std::max(minS_, std::min(size_ + (dir > 0 ? step_ : -step_), maxS_));
        clampCenter();
    }

    void moveTo(double cx, double cy) {
        if (!std::isfinite(cx) || !std::isfinite(cy)) return;
        cx_ = cx; cy_ = cy;
        clampCenter();
    }

    // Hiz modu entegrasyonu. dtSec: gercek kare periyodu (0.1 s'ye kirpilir,
    // takilma sonrasi kutu ziplamasin). Cagiran taraf kapiyi (LIVE) kontrol eder.
    void step(double dtSec, int pwmX, int pwmY, const Config& c) {
        if (!(dtSec > 0.0)) return;                     // sifir / negatif / NaN
        const double dt = std::min(dtSec, 0.1);
        double ax = stickAxis(pwmX, c);
        double ay = stickAxis(pwmY, c);
        if (c.invertX) ax = -ax;
        if (c.invertY) ay = -ay;
        if (ax == 0.0 && ay == 0.0) return;
        const double v = std::max(0.0, c.speed) * (double)w_;   // px/s
        moveTo(cx_ + ax * v * dt, cy_ + ay * v * dt);
    }

    int    x()    const { return (int)std::lround(cx_) - size_ / 2; }
    int    y()    const { return (int)std::lround(cy_) - size_ / 2; }
    int    size() const { return size_; }
    double cx()   const { return cx_; }
    double cy()   const { return cy_; }
    int    minX() const { return minX_; }
    int    maxX() const { return maxX_; }
    int    minY() const { return minY_; }
    int    maxY() const { return maxY_; }

private:
    // Merkez: guvenli alan icinde VE kutu kareden tasmaz. Sinirlar tamsayi:
    // x = round(cx) - size/2 >= 0  ve  x + size <= w  her zaman saglanir.
    static double clampAxis(double v, int safeLo, int safeHi, int size, int extent) {
        const int lo = std::max(safeLo, size / 2);
        const int hi = std::min(safeHi, extent - (size - size / 2));
        if (lo > hi) return (double)(extent / 2);       // kutu cok buyuk: ortada tut
        return std::min((double)hi, std::max((double)lo, v));
    }
    void clampCenter() {
        cx_ = clampAxis(cx_, minX_, maxX_, size_, w_);
        cy_ = clampAxis(cy_, minY_, maxY_, size_, h_);
    }

    int w_ = 2, h_ = 2;
    int minX_ = 0, maxX_ = 2, minY_ = 0, maxY_ = 2;
    int minS_ = 2, maxS_ = 2, base_ = 2, step_ = 1;
    int size_ = 2;
    double cx_ = 1.0, cy_ = 1.0;
};

}  // namespace ptr
