// =====================================================================
// test_target_manager.cpp — target_manager.hpp'yi hybrid/manager.py'ye
// karsi KARE KARE dogrular.
//
// manager_golden.txt gen_manager_golden.py ile Python referansindan
// uretildi: tracker cevaplari ve YOLO tespit listeleri ONCEDEN sabit,
// manager'in bunlari NE ZAMAN isteyip NE yapacagi test edilen sey.
//
// TensorRT / CUDA / kamera / FC GEREKTIRMEZ.
//
// Derleme:
//   g++ -O2 -std=c++17 test_target_manager.cpp -o test_target_manager \
//       $(pkg-config --cflags --libs opencv4)
//   ./test_target_manager manager_golden.txt
// =====================================================================
#include "target_manager.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace hybrid;

static int g_pass = 0, g_fail = 0;
static std::string g_scen = "?";

static void check(bool ok, const std::string& what) {
    if (ok) { ++g_pass; return; }
    ++g_fail;
    if (g_fail <= 30) std::cout << "  [FAIL] (" << g_scen << ") " << what << "\n";
}
static bool near(double a, double b, double tol = 1e-9) {
    if (std::isnan(a) && std::isnan(b)) return true;
    if (std::isnan(a) || std::isnan(b)) return false;
    const double d = std::fabs(a - b);
    return d <= tol || d <= tol * std::max(std::fabs(a), std::fabs(b));
}
static std::string f12(double v) { char b[64]; std::snprintf(b, sizeof(b), "%.12g", v); return b; }

// ---------------------------------------------------- senaryo veri yapilari
struct TrkStep { bool valid; double score; BoxF box; };
struct DetStep { std::vector<Detection> dets; bool stale; };

class ScriptedTracker : public ManagedTracker {
public:
    const std::vector<TrkStep>* script = nullptr;
    long long frameId = 0;
    int inits = 0, resets = 0, tracks = 0;

    bool initialize(const cv::Mat&, const BoxF&) override { ++inits; return true; }
    bool track(const cv::Mat&, BoxF& box, float& score, float& raw) override {
        ++tracks;
        const TrkStep& s = (*script)[(size_t)frameId];
        score = (float)s.score; raw = (float)s.score;
        if (!s.valid) return false;
        box = s.box;
        return true;
    }
    void reset() override { ++resets; }
};

// gen_manager_golden.py ile ayni sabitler
static const long long MS = 1000000LL;
static const long long FRAME_DT = 33 * MS;
static const long long DET_DELAY = 30 * MS;

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "manager_golden.txt";
    std::ifstream f(path);
    if (!f) { std::cerr << "golden acilamadi: " << path << "\n"; return 2; }

    std::cout << "=== TargetManager C++ <-> Python referans dogrulamasi ===\n";
    std::cout << "golden: " << path << "\n\n";

    const cv::Mat dummy = cv::Mat::zeros(8, 8, CV_8UC3);

    std::string line;
    int nScen = 0, nFrames = 0;

    while (std::getline(f, line)) {
        std::istringstream h(line);
        std::string tag; h >> tag;
        if (tag != "SCENARIO") continue;

        int n; h >> g_scen >> n;
        ManagerConfig cfg;
        double refreshTpl = 0;
        h >> cfg.detection_interval >> cfg.search_interval >> cfg.confirmation_hits
          >> cfg.confirmation_iou >> cfg.confirmation_timeout_ms >> cfg.verification_iou
          >> cfg.detection_accept_confidence >> cfg.tracker_low_score >> cfg.tracker_recover_score
          >> cfg.tracker_score_ema_alpha >> cfg.tracker_low_patience >> cfg.verification_miss_patience
          >> cfg.max_box_growth >> cfg.verify_interval >> cfg.verify_miss_patience
          >> cfg.verification_timeout_ms >> cfg.max_detection_age_ms >> cfg.max_tracking_gap_ms
          >> refreshTpl;
        cfg.refresh_template = (refreshTpl != 0);
        ++nScen;

        // ---- senaryo girdilerini oku
        std::vector<TrkStep> trk((size_t)n);
        std::vector<DetStep> det((size_t)n);
        int got = 0;
        while (got < 2 * n && std::getline(f, line)) {
            std::istringstream s(line);
            std::string t; s >> t;
            if (t == "IN_TRK") {
                int i, v; double sc, x1, y1, x2, y2;
                s >> i >> v >> sc >> x1 >> y1 >> x2 >> y2;
                trk[(size_t)i] = TrkStep{v != 0, sc, BoxF{x1, y1, x2, y2}};
                ++got;
            } else if (t == "IN_DET") {
                int i, nd, st; s >> i >> nd >> st;
                det[(size_t)i].stale = (st != 0);
                for (int k = 0; k < nd; ++k) {
                    std::getline(f, line);
                    std::istringstream d(line);
                    std::string dt; double x1, y1, x2, y2, sc; int cls;
                    d >> dt >> x1 >> y1 >> x2 >> y2 >> sc >> cls;
                    det[(size_t)i].dets.push_back(Detection{BoxF{x1, y1, x2, y2}, sc, cls});
                }
                ++got;
            }
        }

        // ---- senaryoyu oynat
        ScriptedTracker tracker;
        tracker.script = &trk;
        TargetManager mgr(&tracker, cfg);

        for (int i = 0; i < n; ++i) {
            const long long captured = (long long)i * FRAME_DT
                                     + ((g_scen == "gap" && i >= 40) ? 1500 * MS : 0);
            tracker.frameId = i;
            mgr.beginFrame(dummy, i, captured);
            if (mgr.detectionDue()) {
                const int gen = mgr.requestDetection();
                const long long completed = captured + DET_DELAY;
                const long long now = det[(size_t)i].stale ? captured + 1200 * MS : completed;
                mgr.applyDetection(dummy, i, captured, i, captured, gen, completed, now,
                                   det[(size_t)i].dets);
            }
            mgr.finishFrame(captured);

            // ---- altin satiri oku ve karsilastir
            if (!std::getline(f, line)) { check(false, "golden erken bitti"); break; }
            std::istringstream o(line);
            std::string t; int gi;
            o >> t >> gi;
            if (t != "OUT") { check(false, "OUT bekleniyordu: " + line.substr(0, 40)); break; }

            std::string gState; int gHasBox, gHasCand;
            double bx1, by1, bx2, by2, cx1, cy1, cx2, cy2;
            int gHits, gLow, gMiss, gUnver, gWeak;
            std::string gFilt;
            long long gGen, gRej, gInits, gResets, gTracks;
            o >> gState >> gHasBox >> bx1 >> by1 >> bx2 >> by2
              >> gHasCand >> cx1 >> cy1 >> cx2 >> cy2
              >> gHits >> gLow >> gMiss >> gUnver >> gWeak >> gFilt
              >> gGen >> gRej >> gInits >> gResets >> gTracks;

            const std::string id = "kare " + std::to_string(i) + ": ";
            check(stateName(mgr.state()) == gState,
                  id + "state " + stateName(mgr.state()) + " != " + gState);
            check(mgr.hasBox() == (gHasBox != 0), id + "hasBox farkli");
            if (mgr.hasBox() && gHasBox) {
                check(near(mgr.box().x1, bx1) && near(mgr.box().y1, by1) &&
                      near(mgr.box().x2, bx2) && near(mgr.box().y2, by2),
                      id + "box (" + f12(mgr.box().x1) + "," + f12(mgr.box().y1) + ") != (" +
                      f12(bx1) + "," + f12(by1) + ")");
            }
            check(mgr.hasCandidate() == (gHasCand != 0), id + "hasCandidate farkli");
            if (mgr.hasCandidate() && gHasCand) {
                check(near(mgr.candidate().x1, cx1) && near(mgr.candidate().y1, cy1) &&
                      near(mgr.candidate().x2, cx2) && near(mgr.candidate().y2, cy2),
                      id + "candidate kutusu farkli");
            }
            check(mgr.hits() == gHits, id + "hits " + std::to_string(mgr.hits()) + " != " + std::to_string(gHits));
            check(mgr.lowCount() == gLow, id + "low_count " + std::to_string(mgr.lowCount()) + " != " + std::to_string(gLow));
            check(mgr.misses() == gMiss, id + "misses " + std::to_string(mgr.misses()) + " != " + std::to_string(gMiss));
            check(mgr.unverified() == gUnver, id + "unverified " + std::to_string(mgr.unverified()) + " != " + std::to_string(gUnver));
            check(mgr.weak() == (gWeak != 0), id + "weak farkli");
            const double gf = (gFilt == "nan") ? std::nan("") : std::stod(gFilt);
            const double cf = mgr.haveFiltered() ? mgr.filteredScore() : std::nan("");
            check(near(cf, gf, 1e-9), id + "filtered_score " + f12(cf) + " != " + f12(gf));
            check(mgr.generation() == gGen, id + "generation " + std::to_string(mgr.generation()) + " != " + std::to_string(gGen));
            check(mgr.rejected() == gRej, id + "rejected " + std::to_string(mgr.rejected()) + " != " + std::to_string(gRej));
            check(tracker.inits == gInits, id + "tracker.initialize cagri sayisi " + std::to_string(tracker.inits) + " != " + std::to_string(gInits));
            check(tracker.resets == gResets, id + "tracker.reset cagri sayisi " + std::to_string(tracker.resets) + " != " + std::to_string(gResets));
            check(tracker.tracks == gTracks, id + "tracker.track cagri sayisi " + std::to_string(tracker.tracks) + " != " + std::to_string(gTracks));
            ++nFrames;
        }
        std::getline(f, line);   // END
    }

    // ------------------------------------------------------ kendi tutarlilik
    g_scen = "sanity";
    {
        check(near(boxIoU(BoxF{0, 0, 10, 10}, BoxF{0, 0, 10, 10}), 1.0), "IoU(ayni) != 1");
        check(near(boxIoU(BoxF{0, 0, 10, 10}, BoxF{20, 20, 30, 30}), 0.0), "IoU(ayrik) != 0");
        check(near(boxIoU(BoxF{0, 0, 10, 10}, BoxF{5, 0, 15, 10}), 50.0 / 150.0), "IoU(yarim) yanlis");
        check(near(boxArea(BoxF{0, 0, 4, 5}), 20.0), "boxArea yanlis");
    }

    std::cout << "\nsenaryo=" << nScen << "  karsilastirilan kare=" << nFrames << "\n";
    std::cout << "-----------------------------------------\n";
    std::cout << (g_fail == 0 ? "GECTI  " : "KALDI  ")
              << g_pass << "/" << (g_pass + g_fail) << " kontrol\n";
    return g_fail == 0 ? 0 : 1;
}
