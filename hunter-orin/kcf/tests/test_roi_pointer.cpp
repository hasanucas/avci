// test_roi_pointer — [PTR] GUIDED imleci (roi_pointer.hpp) offline testi.
// Kamera / FC / OpenCV GEREKMEZ.   make tests && ctest   ya da   ./test_roi_pointer
#include "roi_pointer.hpp"
#include <cstdio>
#include <cmath>
#include <limits>
using namespace ptr;
static int P = 0, F = 0;
static void ck(bool c, const char* w) { if (c) ++P; else { ++F; printf("  [FAIL] %s\n", w); } }
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// runtracker'daki ROI sabitleri (1080p isleme: roiK = 1.5)
struct Roi { int base, minS, maxS, step; };
static Roi roiFor(int procH) {
    const double k = (double)procH / 720.0;
    Roi r;
    r.base = std::max(8, (int)lround(50.0 * k));
    r.minS = std::max(6, (int)lround(20.0 * k));
    r.maxS = std::max(r.base, (int)lround(150.0 * k));
    r.step = std::max(1, (int)lround(4.0 * k));
    return r;
}
// ESKI kod (runtracker updateCenterRoi) — birebir kopya
static void oldRect(int w, int h, int roiSize, int& x, int& y) {
    int half = roiSize / 2;
    x = w / 2 - half; y = h / 2 - half;
}
static SelectBox mk(int w, int h) {
    const Roi r = roiFor(h);
    SelectBox s; s.setup(w, h, 0.075f, r.base, r.minS, r.maxS, r.step); return s;
}

int main() {
    Config c;   // varsayilanlar: merkez 1495, olu bant 30, expo 0.6, hiz 0.5

    // --- 1) stickAxis: olu bant, doyma, isaret, gecersiz PWM
    ck(stickAxis(1495, c) == 0.0, "ortada 0 olmali");
    ck(stickAxis(1495 + 30, c) == 0.0 && stickAxis(1495 - 30, c) == 0.0, "olu bant kenari 0 olmali");
    { const double a = stickAxis(1495 + 31, c);
      ck(a > 0.0 && a < 0.005, "olu banttan hemen sonra KUCUK deger (sicrama yok)"); }
    ck(near(stickAxis(1945, c), 1.0, 1e-12), "tam sag +1 olmali");
    ck(near(stickAxis(1045, c), -1.0, 1e-12), "tam sol -1 olmali");
    ck(near(stickAxis(2100, c), 1.0, 1e-12), "asiri PWM 1'e kirpilmali");
    ck(stickAxis(0, c) == 0.0 && stickAxis(850, c) == 0.0 && stickAxis(2500, c) == 0.0,
       "gecersiz PWM 0 olmali");
    ck(near(stickAxis(1495 + 200, c), -stickAxis(1495 - 200, c), 1e-12), "simetrik olmali");
    { Config l = c; l.expo = 0.0;
      ck(near(stickAxis(1495 + 30 + 210, l), 0.5, 1e-12), "expo=0: yarim sapma 0.5"); }
    ck(near(stickAxis(1495 + 30 + 210, c), 0.4 * 0.5 + 0.6 * 0.125, 1e-12), "expo=0.6: yarim sapma 0.275");
    { bool mono = true; double prev = -2.0;
      for (int p = 1045; p <= 1945; ++p) { const double a = stickAxis(p, c); if (a < prev) mono = false; prev = a; }
      ck(mono, "stickAxis monoton artmali"); }

    // --- 2) Kapi
    ck(gate(c, 4, 100, 50, 1500, 1500) == Gate::Live, "GUIDED + taze -> LIVE");
    ck(gate(c, 5, 100, 50, 1500, 1500) == Gate::Off, "LOITER -> OFF");
    ck(gate(c, 2, 100, 50, 1500, 1500) == Gate::Off, "ALTHOLD -> OFF");
    ck(gate(c, 20, 100, 50, 1500, 1500) == Gate::Off, "GUIDED_NOGPS (saldiri) -> OFF");
    ck(gate(c, -1, -1, -1, 1500, 1500) == Gate::Off, "heartbeat hic yok -> OFF");
    ck(gate(c, 4, 3000, 50, 1500, 1500) == Gate::Frozen, "heartbeat bayat -> FROZEN");
    ck(gate(c, 4, 100, 800, 1500, 1500) == Gate::Frozen, "RC bayat -> FROZEN");
    ck(gate(c, 4, 100, -1, 1500, 1500) == Gate::Frozen, "RC hic yok -> FROZEN");
    ck(gate(c, 4, 100, 50, 0, 1500) == Gate::Frozen, "gecersiz PWM -> FROZEN");
    { Config d = c; d.enabled = false;
      ck(gate(d, 4, 100, 50, 1500, 1500) == Gate::Off, "--no-pointer -> OFF"); }

    // --- 3) Gecis kurallari
    ck(shouldRecenter(Gate::Live, Gate::Off, false), "GUIDED bitti, takip yok -> ortaya");
    ck(shouldRecenter(Gate::Frozen, Gate::Off, false), "FROZEN'dan OFF -> ortaya");
    ck(!shouldRecenter(Gate::Live, Gate::Off, true), "takip varken ortaya ALINMAZ (takip surer)");
    ck(!shouldRecenter(Gate::Live, Gate::Frozen, false), "bayatlama ortaya ALMAZ (donar)");
    ck(!shouldRecenter(Gate::Off, Gate::Live, false), "GUIDED'a giriste ziplama yok");
    ck(keepAtLastTrack(Gate::Live) && keepAtLastTrack(Gate::Frozen) && !keepAtLastTrack(Gate::Off),
       "kilit bitince: imlecte son nokta, manuelde orta");

    // --- 4) MANUEL UYUM: imlec oynamadiysa kutu eski kodla BIREBIR
    {   const int dims[][2] = { {1920, 1080}, {1280, 720}, {1281, 721}, {1919, 1079} };
        bool same = true;
        for (auto& d : dims) {
            const int w = d[0], h = d[1];
            const Roi r = roiFor(h);
            SelectBox s = mk(w, h);
            int roiSize = r.base;
            int ox, oy; oldRect(w, h, roiSize, ox, oy);
            if (s.x() != ox || s.y() != oy || s.size() != roiSize) same = false;
            // 40 kez buyut, 80 kez kucult, 10 kez buyut — eski resize ile ayni yol
            const int seq[][2] = { {+1, 40}, {-1, 80}, {+1, 10} };
            for (auto& q : seq) for (int i = 0; i < q[1]; ++i) {
                s.resize(q[0]);
                roiSize += q[0] * r.step;
                roiSize = std::max(r.minS, std::min(roiSize, r.maxS));
                oldRect(w, h, roiSize, ox, oy);
                if (s.x() != ox || s.y() != oy || s.size() != roiSize) same = false;
            }
            s.recenter(); oldRect(w, h, roiSize, ox, oy);
            if (s.x() != ox || s.y() != oy) same = false;        // otomatik dusme yolu
            s.resetBase(); oldRect(w, h, r.base, ox, oy);
            if (s.x() != ox || s.y() != oy || s.size() != r.base) same = false;   // birakma yolu
        }
        ck(same, "manuel yol eski updateCenterRoi ile birebir olmali"); }

    // --- 5) Hiz modu: tam sag 0.1 s -> +96 px (1920 genislik, hiz 0.5)
    {   SelectBox s = mk(1920, 1080);
        s.step(0.1, 1945, 1495, c);
        ck(near(s.cx(), 960.0 + 96.0, 1e-9), "tam sag 0.1 s: +96 px");
        ck(near(s.cy(), 540.0, 1e-12), "dikey kipirdamamali");
        ck(s.x() == 1056 - s.size() / 2, "kutu x merkezle uyumlu"); }
    // ileri (DUSUK PWM) -> YUKARI; --ptr-invert-y ile ters
    {   SelectBox s = mk(1920, 1080);
        s.step(0.05, 1495, 1045, c);
        ck(s.cy() < 540.0, "ileri -> yukari (cy azalmali)");
        Config inv = c; inv.invertY = true;
        SelectBox t = mk(1920, 1080);
        t.step(0.05, 1495, 1045, inv);
        ck(t.cy() > 540.0, "invert-y: ileri -> asagi"); }
    // olu bant icinde 1000 kare -> hic oynamaz
    {   SelectBox s = mk(1920, 1080);
        for (int i = 0; i < 1000; ++i) s.step(1.0 / 30.0, 1495 + 25, 1495 - 25, c);
        ck(s.cx() == 960.0 && s.cy() == 540.0, "olu bantta kutu kaymamali"); }
    // dt kirpma / gecersiz dt
    {   SelectBox s = mk(1920, 1080);
        s.step(5.0, 1945, 1495, c);
        ck(near(s.cx(), 960.0 + 96.0, 1e-9), "dt 0.1 s'ye kirpilmali");
        SelectBox t = mk(1920, 1080);
        t.step(0.0, 1945, 1495, c); t.step(-1.0, 1945, 1495, c);
        t.step(std::numeric_limits<double>::quiet_NaN(), 1945, 1495, c);
        ck(t.cx() == 960.0, "dt <= 0 / NaN hareket ettirmemeli"); }
    // kare hizindan bagimsiz: 30 fps ve 60 fps'te 0.5 s ayni yol
    {   SelectBox a = mk(1920, 1080), b = mk(1920, 1080);
        for (int i = 0; i < 15; ++i) a.step(1.0 / 30.0, 1700, 1495, c);
        for (int i = 0; i < 30; ++i) b.step(1.0 / 60.0, 1700, 1495, c);
        ck(near(a.cx(), b.cx(), 1e-6), "30/60 fps ayni mesafe"); }

    // --- 6) Sinirlar: guvenli alan + kare
    {   SelectBox s = mk(1920, 1080);
        s.moveTo(-500.0, -500.0);
        ck(s.cx() == (double)s.minX() && s.cy() == (double)s.minY(), "sol-ust: guvenli alana kirpilmali");
        ck(s.minX() == 144 && s.minY() == 81, "guvenli alan 1080p: 144/81 (runtracker ile ayni)");
        s.moveTo(1e6, 1e6);
        ck(s.cx() == (double)s.maxX() && s.cy() == (double)s.maxY(), "sag-alt: guvenli alana kirpilmali");
        const int bcx = s.x() + s.size() / 2, bcy = s.y() + s.size() / 2;
        ck(bcx >= s.minX() && bcx <= s.maxX() && bcy >= s.minY() && bcy <= s.maxY(),
           "init kutusunun merkezi (runtracker hesabi) guvenli alanda olmali");
        for (int i = 0; i < 300; ++i) s.step(1.0 / 30.0, 1945, 1945, c);
        ck(s.cx() == (double)s.maxX(), "kenarda tam itince disari tasmamali"); }
    {   // kucuk kare + buyuk kutu: kutu kareden TASMAZ (her konumda)
        SelectBox s; s.setup(200, 160, 0.075f, 150, 20, 150, 4);
        bool inside = true;
        const double pts[][2] = { {-9, -9}, {0, 0}, {100, 80}, {999, 999}, {57.5, 92.5}, {142.5, 67.5} };
        for (auto& p : pts) {
            s.moveTo(p[0], p[1]);
            if (s.x() < 0 || s.y() < 0 || s.x() + s.size() > 200 || s.y() + s.size() > 160) inside = false;
        }
        ck(inside, "kutu hicbir konumda kareden tasmamali"); }
    {   // tek boyutlu kutuda yuvarlama: sag kenarda x + size <= w
        SelectBox s; s.setup(1920, 1080, 0.0f, 75, 20, 225, 6);
        s.moveTo(1e6, 1e6);
        ck(s.x() + s.size() <= 1920 && s.y() + s.size() <= 1080, "tek boyut + kenar: tasma yok");
        s.moveTo(-1e6, -1e6);
        ck(s.x() >= 0 && s.y() >= 0, "tek boyut + sol kenar: tasma yok"); }
    {   SelectBox s = mk(1920, 1080);
        s.moveTo(std::numeric_limits<double>::quiet_NaN(), 300.0);
        ck(s.cx() == 960.0 && s.cy() == 540.0, "NaN hedef yok sayilmali");
        s.moveTo(std::numeric_limits<double>::infinity(), 300.0);
        ck(s.cx() == 960.0 && s.cy() == 540.0, "inf hedef yok sayilmali"); }

    // --- 7) Imlecte boyut degisimi merkezi KORUR, kenarda gerekirse iceri iter
    {   SelectBox s = mk(1920, 1080);
        s.step(0.1, 1945, 1495, c);                 // cx = 1056
        const int before = s.size();
        s.resize(+1);
        ck(s.size() == before + roiFor(1080).step, "buyut: bir adim");
        ck(near(s.cx(), 1056.0, 1e-9), "buyut: merkez yerinde");
        s.open();
        ck(s.cx() == 960.0 && s.size() == roiFor(1080).base, "CH8 yeniden acilinca orta + taban"); }

    // --- 8) Senaryo: GUIDED'da gezdir -> kilit -> birak -> GUIDED biter
    {   SelectBox s = mk(1920, 1080);
        Gate prev = Gate::Off, now = gate(c, 4, 100, 50, 1700, 1300);
        ck(!shouldRecenter(prev, now, false), "GUIDED'a giris: ziplama yok");
        for (int i = 0; i < 20; ++i) s.step(1.0 / 30.0, 1700, 1300, c);   // sag-yukari
        ck(s.cx() > 960.0 && s.cy() < 540.0, "sag-yukari gezdi");
        // kilit (runtracker init) -> takip ... -> CH12 geri: son takip noktasi
        const double lastX = 1300.0, lastY = 400.0;
        if (keepAtLastTrack(now)) s.moveTo(lastX, lastY);
        ck(s.cx() == lastX && s.cy() == lastY, "birakinca son takip noktasinda");
        // pilot modu alir (LOITER), takip yok -> ortaya
        prev = now; now = gate(c, 5, 100, 50, 1500, 1500);
        if (shouldRecenter(prev, now, false)) s.recenter();
        ck(s.cx() == 960.0 && s.cy() == 540.0, "LOITER'a gecince kutu ortada (manuel KCF)"); }

    // --- 9) MASA DEMOSU (--ptr-demo / --ptr-demo-ch N)
    {   const int bad[] = { 0, 5, 6, 8, 10, 11, 12, 13, 14, 17 };
        const int ok[]  = { 1, 2, 3, 4, 7, 9, 15, 16 };
        bool all = true;
        for (int ch : bad) if (!reservedChannel(ch)) all = false;
        for (int ch : ok)  if (reservedChannel(ch))  all = false;
        ck(all, "ayrilmis kanallar: 5/6/8/10/11(ARM)/12/13/14(RSSI) ve aralik disi"); }
    {   Config d = c; d.demo = true;
        ck(demoActive(d, false) && !demoActive(d, true), "demo sadece DISARM'da");
        ck(!demoActive(c, false), "bayraksiz demo yok");
        ck(demoEffectiveMode(d, 0, 0) == MODE_GUIDED, "kanalsiz demo: hep GUIDED");
        d.demoCh = 9;
        ck(demoEffectiveMode(d, 1950, 0) == MODE_GUIDED, "demo anahtari yuksek -> GUIDED");
        ck(demoEffectiveMode(d, 1050, MODE_GUIDED) == 0, "demo anahtari dusuk -> GUIDED degil");
        ck(demoEffectiveMode(d, 1500, MODE_GUIDED) == MODE_GUIDED &&
           demoEffectiveMode(d, 1500, 0) == 0, "ara bolge: onceki korunur (histerezis)");
        ck(demoEffectiveMode(d, 0, MODE_GUIDED) == MODE_GUIDED, "gecersiz PWM: onceki korunur");
        // demo modu kapiya verilince: anahtar yuksek -> LIVE, dusuk -> OFF (orta yolu)
        const int mHi = demoEffectiveMode(d, 1950, 0), mLo = demoEffectiveMode(d, 1050, mHi);
        ck(gate(d, mHi, 0, 50, 1500, 1500) == Gate::Live, "demo yuksek + RC taze -> LIVE");
        ck(gate(d, mLo, 0, 50, 1500, 1500) == Gate::Off, "demo dusuk -> OFF");
        ck(gate(d, mHi, 0, 900, 1500, 1500) == Gate::Frozen, "demo'da da RC bayatsa FROZEN");
        ck(shouldRecenter(gate(d, mHi, 0, 50, 1500, 1500), gate(d, mLo, 0, 50, 1500, 1500), false),
           "demo anahtari dusunce kutu ortaya (pilot geri aldi taklidi)"); }

    printf("\n%s  %d/%d kontrol\n", F ? "KALDI" : "GECTI", P, P + F);
    return F ? 1 : 0;
}
