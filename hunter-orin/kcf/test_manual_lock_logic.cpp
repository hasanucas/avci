// SimpleLock manuel kilit (--ch-lock) MANTIK testi. Gercek simple_lock.hpp
// derlenir; OpenCV minimal stub'la saglanir (SimpleLock cv:: kullanmiyor).
#include "simple_lock.hpp"
#include <cstdio>
#include <string>
#include <vector>

static int PASS = 0;
static std::vector<std::string> FAIL;
static void check(const char* n, bool c, const std::string& d = "") {
    if (c) { ++PASS; std::printf("  ok   %s\n", n); }
    else   { FAIL.push_back(std::string(n) + "  " + d);
             std::printf("  FAIL %s  %s\n", n, d.c_str()); }
}

using slock::Config; using slock::SimpleLock; using slock::State;
using hybrid::BoxF; using hybrid::Detection;

static Detection det(double x, double y, double w, double h, double sc) {
    Detection d{}; d.box = BoxF{x, y, x + w, y + h}; d.score = sc; return d;
}
static std::vector<Detection> one(double x, double y, double sc = 0.9) {
    return { det(x, y, 60, 60, sc) };
}

static Config cfgManual() {
    Config c; c.manualOnly = true; c.manualMinHits = 3; return c;
}

int main() {
    std::printf("\nA. MANUEL KILIT — kanit kapisi\n");
    {
        SimpleLock s(cfgManual());
        check("baslangicta ARAMA", s.state() == State::SEARCH);
        check("aday yokken buton kilitlemiyor", !s.forceLock());
        check("engel sebebi 'aday yok'", std::string(s.manualBlockReason()) == "aday yok",
              s.manualBlockReason());

        s.onDetections(one(100, 100));          // 1. kare -> aday kuruldu, hits=1
        check("tek tespitten sonra aday VAR", s.hasCand());
        check("1 karede buton kilitlemiyor (kanit az)", !s.forceLock());
        check("engel sebebi 'kanit az'", std::string(s.manualBlockReason()) == "kanit az",
              s.manualBlockReason());

        s.onDetections(one(101, 101));          // hits=2
        check("2 karede hala kilitlemiyor", !s.forceLock());

        s.onDetections(one(102, 102));          // hits=3 -> yeterli
        check("hits 3'e ulasti", s.hits() >= 3, std::to_string(s.hits()));
        check("3 karede buton KILITLIYOR", s.forceLock());
        BoxF ib{};
        check("kurulum kuyruga girdi (pendingInit)", s.pendingInit(ib));
        check("devredilen kutu ADAY kutusu (son tespit ~102)",
              ib.x1 > 101.0 && ib.x1 < 103.0, std::to_string(ib.x1));
        check("olay 'manual_lock' diyor",
              s.lastEvent().find("manual_lock") != std::string::npos, s.lastEvent());
        s.initDone(true);
        check("initDone sonrasi KILIT", s.state() == State::LOCK);
        check("manuel kilit sayaci 1", s.manualLocks() == 1);
    }

    std::printf("\nB. KILITTE TESPIT DURUYOR\n");
    {
        SimpleLock s(cfgManual());
        for (int i = 0; i < 5; ++i) s.onDetections(one(100 + i, 100));
        s.forceLock(); BoxF ib{}; s.pendingInit(ib); s.initDone(true);
        check("KILIT durumunda", s.state() == State::LOCK);
        check("detectionWanted FALSE -> YOLO thread'i bosta", !s.detectionWanted());
        check("kilitliyken buton tekrar kilitlemiyor (no-op)", !s.forceLock());
        check("engel sebebi bos (zaten kilitli)",
              std::string(s.manualBlockReason()).empty(), s.manualBlockReason());
    }

    std::printf("\nC. OTOMATIK KILIT KAPALI (manualOnly)\n");
    {
        SimpleLock s(cfgManual());               // lockHits 20 / window 30
        for (int i = 0; i < 40; ++i) s.onDetections(one(100, 100));
        check("40 tutarli tespit -> HALA ARAMA (otomatik kilit kapali)",
              s.state() == State::SEARCH, slock::stateName(s.state()));
        BoxF ib{};
        check("kurulum kuyrukta DEGIL", !s.pendingInit(ib));
        check("ama aday ve sayac birikmis", s.hasCand() && s.hits() > 20,
              std::to_string(s.hits()));
        check("buton basilinca ANINDA kilitler", s.forceLock());
    }

    std::printf("\nD. OTOMATIK KILIT ACIK (manualOnly=false) — eski davranis BOZULMADI\n");
    {
        Config c; c.manualOnly = false;          // varsayilan
        SimpleLock s(c);
        for (int i = 0; i < 25; ++i) s.onDetections(one(100, 100));
        BoxF ib{};
        check("20/30 dolunca kurulum KENDILIGINDEN kuyruga girdi", s.pendingInit(ib));
        check("olay 'lock' diyor", s.lastEvent().find("lock") != std::string::npos,
              s.lastEvent());
        s.initDone(true);
        check("otomatik yolla KILIT", s.state() == State::LOCK);
        check("manuel sayac 0 (bu kilit otomatikti)", s.manualLocks() == 0);
    }

    std::printf("\nE. BUTON BIRAKMA -> ARAMA, YOLO yeniden baslar\n");
    {
        SimpleLock s(cfgManual());
        for (int i = 0; i < 5; ++i) s.onDetections(one(100, 100));
        s.forceLock(); BoxF ib{}; s.pendingInit(ib); s.initDone(true);
        check("KILIT", s.state() == State::LOCK);
        s.release("manual_release (buton birakildi)");
        check("release sonrasi ARAMA", s.state() == State::SEARCH);
        check("detectionWanted TRUE -> YOLO yeniden kosuyor", s.detectionWanted());
        check("kutu birakildi", !s.hasBox());
        check("olay kayitli",
              s.lastEvent().find("manual_release") != std::string::npos, s.lastEvent());
        // Tekrar kilitlenebilmeli
        for (int i = 0; i < 5; ++i) s.onDetections(one(400, 300));
        check("yeniden kilitlenebiliyor", s.forceLock());
        check("manuel sayac 2", s.manualLocks() == 2, std::to_string(s.manualLocks()));
    }

    std::printf("\nF. FALSE POSITIVE — tek karelik parlama kilitlenmiyor\n");
    {
        SimpleLock s(cfgManual());
        // Tek karede yuksek skorlu bir sahte tespit. Buton o karede basili.
        s.onDetections(one(800, 50, 0.99));
        check("tek karelik parlamada buton kilitlemiyor", !s.forceLock(),
              "hits=" + std::to_string(s.hits()));
        // Sonraki karelerde sahte tespit YOK -> aday kaniti birikmez
        s.onDetections({});
        s.onDetections({});
        check("tespit kesilince hala kilitlenmiyor", !s.forceLock());
        check("hits 3'un altinda", s.hits() < 3, std::to_string(s.hits()));
    }

    std::printf("\n==============================================\n");
    if (!FAIL.empty()) {
        std::printf("SONUC: %d gecti, %zu KALDI\n", PASS, FAIL.size());
        for (const auto& f : FAIL) std::printf("   - %s\n", f.c_str());
        return 1;
    }
    std::printf("SONUC: %d/%d GECTI\n", PASS, PASS);
    return 0;
}
