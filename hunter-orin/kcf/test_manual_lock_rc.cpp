// readRcControl'un MANUEL KILIT (--ch-lock) kismi — runtracker.cpp'den
// AYNEN cikarilan fonksiyon govdesiyle test edilir (kopya DEGIL).
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>

// --- runtracker.cpp'deki global ortam (stub) ---
static std::mutex rcMutex;
static uint16_t   rc_values[18];
static int        g_chBitrate = 0;
static int        g_chLock    = 0;
static int        g_brMin     = 500;
static int        g_brMax     = 6000;

#include "rc_extract.inc"   // runtracker.cpp'den AYNEN cikarilir (driver uretir)

static int PASS = 0;
static std::vector<std::string> FAIL;
static void check(const char* n, bool c, const std::string& d = "") {
    if (c) { ++PASS; std::printf("  ok   %s\n", n); }
    else   { FAIL.push_back(std::string(n)+"  "+d);
             std::printf("  FAIL %s  %s\n", n, d.c_str()); }
}
static void setch(int ch1based, uint16_t v) { rc_values[ch1based-1] = v; }
static bool lockOf() { RcControl c; readRcControl(c); return c.manual_lock; }

int main() {
    // Gecerli taban: CH8/12/13 ortada
    setch(8, 1050); setch(12, 1500); setch(13, 1500);

    std::printf("\nA. --ch-lock VERILMEDI -> buton HEP kapali\n");
    g_chLock = 0;
    setch(9, 1950);
    check("kanal yokken buton false", lockOf() == false);

    std::printf("\nB. CH9 latching: >1600 ACIK, <1350 KAPALI\n");
    g_chLock = 9;
    setch(9, 1050);
    check("bos buton (1050) KAPALI", lockOf() == false);
    setch(9, 1950);
    check("basili buton (1950) ACIK", lockOf() == true);
    setch(9, 1050);
    check("birakinca KAPALI", lockOf() == false);

    std::printf("\nC. Ara bolge (1350-1600) SON DURUMU KORUR\n");
    setch(9, 1950); lockOf();          // ACIK yap
    setch(9, 1500);
    check("ACIK iken ara degerde ACIK kalir", lockOf() == true);
    setch(9, 1050); lockOf();          // KAPALI yap
    setch(9, 1500);
    check("KAPALI iken ara degerde KAPALI kalir", lockOf() == false);

    std::printf("\nD. GECERSIZ RC -> kilit DUSMEZ (en kritik)\n");
    setch(9, 1950); lockOf();          // ACIK
    setch(9, 0);                       // kanal dustu
    check("CH9 = 0 olunca buton ACIK KALIR", lockOf() == true,
          "dusse ucus ortasinda kilit birakilirdi");
    // Tum RC dusunce (erken donus yolu)
    setch(8, 0); setch(12, 0); setch(13, 0);
    check("TUM RC dustugunde de ACIK KALIR", lockOf() == true);
    setch(8, 1050); setch(12, 1500); setch(13, 1500); setch(9, 1950);

    std::printf("\nE. Diger kanallara karismiyor\n");
    g_chLock = 9;
    setch(8, 1950);                    // pencere ACIK
    setch(9, 1950);
    { RcControl c; readRcControl(c);
      check("CH8 pencere hala okunuyor", c.roi_visible == true);
      check("CH9 kilit de ACIK", c.manual_lock == true); }
    setch(12, 1700);                   // kenar: kilitle
    { RcControl c; readRcControl(c);
      check("CH12 kenar tetigi bozulmadi", c.tracker_start == true); }

    std::printf("\nF. Kanal numarasi secilebilir (--ch-lock 7)\n");
    g_chLock = 7;
    setch(7, 1050); setch(9, 1950);
    check("CH9 basili ama kanal 7 secili -> KAPALI", lockOf() == false);
    setch(7, 1950);
    check("CH7 basilinca ACIK", lockOf() == true);

    std::printf("\n==============================================\n");
    if (!FAIL.empty()) {
        std::printf("SONUC: %d gecti, %zu KALDI\n", PASS, FAIL.size());
        for (const auto& f : FAIL) std::printf("   - %s\n", f.c_str());
        return 1;
    }
    std::printf("SONUC: %d/%d GECTI\n", PASS, PASS);
    return 0;
}
