// =====================================================================
// test_yo_manager.cpp — yo::TargetManager + yo::Pipeline'i YoloORTrack
// Python referansina (tests/yo_ref/hybrid/manager.py + correction.py,
// async_pipeline.py cagri sirasi) karsi KARE KARE dogrular.
//
// yo_manager_golden.txt -> gen_yo_manager_golden.py. Senaryolar tracker
// scriptini, tarama surelerini, tespit listelerini ve pilot reset'lerini
// sabitler; manager'in her karede NE YAPTIGI (durum, kutu, aday, skorlar,
// sayaclar, olaylar, tracker cagrilari) karsilastirilir.
//
// TensorRT / CUDA / kamera / FC GEREKTIRMEZ.
//   ./test_yo_manager ../tests/yo_manager_golden.txt
// =====================================================================
#include "yo_pipeline.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using ortrack::BoxF;
using hybrid::Detection;

static int g_pass = 0, g_fail = 0;
static std::string g_scen = "?";
static int g_frame = -1;
static double g_maxDev = 0.0;

static void check(bool ok, const std::string& what) {
    if (ok) { ++g_pass; return; }
    ++g_fail;
    if (g_fail <= 40) std::cout << "  [FAIL] (" << g_scen << " kare " << g_frame << ") " << what << "\n";
}
static double num(const std::string& t) { return std::strtod(t.c_str(), nullptr); }
static bool near(double a, double b) {
    if (std::isnan(a) && std::isnan(b)) return true;
    if (std::isnan(a) || std::isnan(b)) return false;
    const double d = std::fabs(a - b);
    const double rel = d / std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
    if (rel > g_maxDev) g_maxDev = rel;
    return rel <= 1e-9;
}
static std::vector<std::string> split(const std::string& line) {
    std::istringstream s(line); std::vector<std::string> v; std::string t;
    while (s >> t) v.push_back(t);
    return v;
}

// ---- gen_yo_manager_golden.py FakeTracker ile BIREBIR ayni model
struct Step { bool valid; float score; double dx, dy, s; };
static int g_W = 1920, g_H = 1080;

class FakeTracker : public hybrid::ManagedTracker {
public:
    const std::vector<Step>* script = nullptr;
    const int* cur = nullptr;
    bool has = false; BoxF box{};
    int inits = 0, relocs = 0, resets = 0, tracks = 0;

    bool initialize(const cv::Mat&, const BoxF& b) override {
        ++inits; box = b.clipped((double)g_W, (double)g_H); has = true; return true;
    }
    bool canRelocate() const override { return true; }
    bool relocate(const BoxF& b) override { ++relocs; box = b; has = true; return true; }
    void reset() override { ++resets; has = false; }
    bool track(const cv::Mat&, BoxF& out, float& score, float& raw) override {
        ++tracks;
        const Step& st = (*script)[(size_t)*cur];
        const BoxF b = box;
        const double cx = (b.x1 + b.x2) / 2 + st.dx, cy = (b.y1 + b.y2) / 2 + st.dy;
        const double w = (b.x2 - b.x1) * st.s, h = (b.y2 - b.y1) * st.s;
        const double x = cx - w / 2, y = cy - h / 2;
        const double m = std::min(4.0, (double)std::min(g_W, g_H));
        const double x1 = std::min(std::max(0.0, x), g_W - m), y1 = std::min(std::max(0.0, y), g_H - m);
        const double x2 = std::min(std::max(m, x + w), (double)g_W), y2 = std::min(std::max(m, y + h), (double)g_H);
        const BoxF nb = BoxF::fromXYWH(x1, y1, std::max(m, x2 - x1), std::max(m, y2 - y1));
        score = st.score; raw = st.score;
        if (!st.valid) { out = BoxF{}; return false; }
        box = nb; out = nb;
        return true;
    }
};

// ---- deterministik tarayici: istendigi kareden DUR[k] kare sonra biter
class FakeScanner : public yo::Scanner {
public:
    const int* cur = nullptr;
    const std::vector<long long>* cap = nullptr;
    std::vector<int> dur;
    std::map<long long, std::vector<Detection>> dets;
    size_t k = 0;
    bool has = false; yo::DetectionRequest job; int doneAt = 0;

    void submit(const yo::DetectionRequest& r) override {
        job = r; has = true; doneAt = *cur + dur.at(k++);
    }
    bool poll(yo::ScanResult& o) override {
        if (!has || *cur < doneAt) return false;
        has = false;
        o = yo::ScanResult{};
        o.ok = true; o.frameId = job.frameId; o.capNs = job.capNs; o.generation = job.generation;
        o.completedNs = (*cap)[(size_t)*cur] - 1000000LL;
        o.dets = dets.at(job.frameId);
        return true;
    }
};

static std::string evStr(const yo::Event& e) {
    char b[256];
    switch (e.type) {
        case 'T': return "T " + e.from + " " + e.to + " " + e.reason;
        case 'Z': std::snprintf(b, sizeof(b), "Z %s %d", e.reason.c_str(), e.generation); return b;
        case 'R': std::snprintf(b, sizeof(b), "R %lld %s", e.srcFrame, e.reason.c_str()); return b;
        case 'K': std::snprintf(b, sizeof(b), "K %lld %lld", e.srcFrame, e.toFrame); return b;
        case 'C': std::snprintf(b, sizeof(b), "C %s %lld", e.reason.c_str(), e.srcFrame); return b;
        case 'B': return "B";
    }
    return "?";
}

int main(int argc, char** argv)
{
    const std::string path = (argc > 1) ? argv[1] : "yo_manager_golden.txt";
    std::ifstream f(path);
    if (!f) { std::cerr << "golden acilamadi: " << path << "\n"; return 2; }
    std::cout << "=== yo::TargetManager + Pipeline C++ <-> YoloORTrack Python referansi ===\n";

    std::vector<std::string> lines;
    for (std::string l; std::getline(f, l);) lines.push_back(l);
    size_t li = 0;
    int nScen = 0, nFrames = 0, nEvents = 0;

    while (li < lines.size()) {
        std::vector<std::string> h = split(lines[li]);
        if (h.empty() || h[0] != "SCEN") { ++li; continue; }
        g_scen = h[1];
        const int n = std::stoi(h[2]); g_W = std::stoi(h[3]); g_H = std::stoi(h[4]);
        ++li; ++nScen;

        // ---- CFG (gen_yo_manager_golden.py CFG_KEYS sirasi)
        std::vector<std::string> c = split(lines[li++]);
        yo::ManagerConfig cfg;
        size_t p = 1;
        cfg.confirmation_hits = std::stoi(c[p++]);
        cfg.confirmation_iou = num(c[p++]);
        cfg.confirmation_timeout_ms = num(c[p++]);
        cfg.verification_iou = num(c[p++]);
        cfg.detection_accept_confidence = num(c[p++]);
        { const std::string v = c[p++]; cfg.has_verify_confidence = (v != "none");
          cfg.detection_verify_confidence = cfg.has_verify_confidence ? num(v) : 0.0; }
        cfg.detector_average_release_confidence = num(c[p++]);
        cfg.detector_average_window_ms = num(c[p++]);
        cfg.detector_average_min_samples = std::stoi(c[p++]);
        cfg.tracker_low_score = num(c[p++]);
        cfg.tracker_recover_score = num(c[p++]);
        cfg.tracker_score_ema_alpha = num(c[p++]);
        cfg.tracker_low_patience = std::stoi(c[p++]);
        cfg.tracker_low_release_ms = num(c[p++]);
        cfg.verification_miss_patience = std::stoi(c[p++]);
        cfg.max_box_growth = num(c[p++]);
        cfg.clamp_box_growth = (std::stoi(c[p++]) != 0);
        cfg.verify_interval = std::stoi(c[p++]);
        cfg.verify_miss_patience = std::stoi(c[p++]);
        cfg.verification_timeout_ms = num(c[p++]);
        cfg.detector_absence_grace_ms = num(c[p++]);
        cfg.acquisition_max_area_frac = num(c[p++]);
        cfg.max_detection_age_ms = num(c[p++]);
        cfg.max_tracking_gap_ms = num(c[p++]);
        cfg.refresh_template = (std::stoi(c[p++]) != 0);
        cfg.correction_enabled = (c[p++] != "off");
        cfg.correction_weight = num(c[p++]);
        { const std::string a = c[p++];
          cfg.correction_application = (a == "auto") ? 0 : (a == "relocate") ? 1 : 2; }

        // ---- girdiler
        std::vector<long long> cap((size_t)n);
        std::vector<Step> script((size_t)n);
        for (int i = 0; i < n; ++i) {
            std::vector<std::string> t = split(lines[li++]);
            const int idx = std::stoi(t[1]);
            cap[(size_t)idx] = std::stoll(t[2]);
            script[(size_t)idx] = Step{std::stoi(t[3]) != 0, (float)num(t[4]), num(t[5]), num(t[6]), num(t[7])};
        }
        FakeScanner sc;
        { std::vector<std::string> t = split(lines[li++]);
          for (size_t k = 1; k < t.size(); ++k) sc.dur.push_back(std::stoi(t[k])); }
        std::set<int> resets;
        { std::vector<std::string> t = split(lines[li++]);
          for (size_t k = 1; k < t.size(); ++k) resets.insert(std::stoi(t[k])); }
        while (li < lines.size() && lines[li].rfind("DET ", 0) == 0) {
            std::vector<std::string> t = split(lines[li++]);
            const long long src = std::stoll(t[1]); const int nd = std::stoi(t[2]);
            std::vector<Detection>& v = sc.dets[src];
            for (int k = 0; k < nd; ++k) {
                std::vector<std::string> d = split(lines[li++]);
                v.push_back(Detection{BoxF{num(d[1]), num(d[2]), num(d[3]), num(d[4])}, num(d[5]), std::stoi(d[6])});
            }
        }

        // ---- oynat
        int cur = 0;
        FakeTracker tr; tr.script = &script; tr.cur = &cur;
        sc.cur = &cur; sc.cap = &cap;
        yo::TargetManager mgr(&tr, cfg);
        yo::Pipeline pipe(&mgr, &sc, [&]() { return cap[(size_t)cur] + 2000000LL; });
        const cv::Mat img = cv::Mat::zeros(g_H, g_W, CV_8UC3);

        for (int i = 0; i < n; ++i) {
            cur = i; g_frame = i;
            if (resets.count(i)) pipe.reset("pilot_release");
            const yo::StepResult r = pipe.process(img, i, cap[(size_t)i]);
            check(r.accepted, "kare kabul edilmedi");
            ++nFrames;

            std::vector<std::string> F = split(lines[li++]);
            check(F[0] == "F" && std::stoi(F[1]) == i, "F satiri senkron degil");
            size_t q = 2;
            check(F[q++] == yo::stateName(mgr.state()), std::string("durum ") + yo::stateName(mgr.state()) + " != " + F[2]);
            auto cmpBox = [&](bool has, const BoxF& b, const char* what) {
                const bool gh = F[q++] == "1";
                const double x1 = num(F[q++]), y1 = num(F[q++]), x2 = num(F[q++]), y2 = num(F[q++]);
                check(gh == has, std::string(what) + " varligi");
                if (gh && has)
                    check(near(b.x1, x1) && near(b.y1, y1) && near(b.x2, x2) && near(b.y2, y2),
                          std::string(what) + " koordinat");
            };
            cmpBox(mgr.hasBox(), mgr.box(), "kutu");
            cmpBox(mgr.hasCandidate(), mgr.candidate(), "aday");
            check(near(mgr.haveFiltered() ? mgr.filteredScore() : NAN, num(F[q++])), "filtreli skor");
            check(std::stoi(F[q++]) == (int)mgr.weak(), "weak");
            check(std::stoi(F[q++]) == mgr.lowCount(), "low_count");
            check(std::stoi(F[q++]) == mgr.misses(), "misses");
            check(std::stoi(F[q++]) == mgr.unverified(), "unverified");
            check(std::stoi(F[q++]) == mgr.generation(), "generation");
            check(std::stoll(F[q++]) == mgr.rejected(), "rejected");
            check(std::stoi(F[q++]) == mgr.hits(), "hits");
            cmpBox(tr.has, tr.box, "tracker kutusu");
            check(std::stoi(F[q++]) == tr.inits,  "tracker initialize sayisi");
            check(std::stoi(F[q++]) == tr.relocs, "tracker relocate sayisi");
            check(std::stoi(F[q++]) == tr.resets, "tracker reset sayisi");
            check(std::stoi(F[q++]) == tr.tracks, "tracker track sayisi");
            check(std::stoi(F[q++]) == (int)r.tracked.tracked, "tracked");
            check(std::stoi(F[q++]) == (int)(r.tracked.tracked && r.tracked.valid), "tracked valid");
            check(near(r.tracked.tracked ? (double)r.tracked.score : 0.0, num(F[q++])), "tracked skor");
            const std::string ap = F[q++];
            const std::string mine = !r.scanDone ? "-" : (r.scanApplied ? "A" : "R");
            check(ap == mine, "tarama uygulamasi " + mine + " != " + ap);

            // olaylar
            std::vector<std::string> gev;
            while (li < lines.size() && lines[li].rfind("  E ", 0) == 0) gev.push_back(lines[li++]);
            const std::vector<yo::Event>& ev = mgr.events();
            check(ev.size() == gev.size(), "olay sayisi " + std::to_string(ev.size()) + " != " + std::to_string(gev.size()));
            for (size_t k = 0; k < std::min(ev.size(), gev.size()); ++k) {
                std::vector<std::string> g = split(gev[k]);   // E <tip> ...
                const yo::Event& e = ev[k];
                ++nEvents;
                bool ok = g[1].size() == 1 && g[1][0] == e.type;
                if (ok) switch (e.type) {
                    case 'T': ok = g[2] == e.from && g[3] == e.to && g[4] == e.reason; break;
                    case 'Z': ok = g[2] == e.reason && std::stoi(g[3]) == e.generation; break;
                    case 'R': ok = std::stoll(g[2]) == e.srcFrame && g[3] == e.reason; break;
                    case 'C': ok = g[2] == e.reason && std::stoll(g[3]) == e.srcFrame &&
                                   near(e.dx, num(g[4])) && near(e.dy, num(g[5])); break;
                    case 'K': ok = std::stoll(g[2]) == e.srcFrame && std::stoll(g[3]) == e.toFrame; break;
                    case 'B': ok = near(e.value, num(g[2])); break;
                }
                check(ok, "olay " + std::to_string(k) + ": C++ [" + evStr(e) + "] Python [" + gev[k] + "]");
            }
        }
    }
    std::cout << "senaryo " << nScen << "  kare " << nFrames << "  olay " << nEvents
              << "  en buyuk bagil sapma " << g_maxDev << "\n";
    std::cout << "SONUC: " << g_pass << "/" << (g_pass + g_fail) << " kontrol gecti\n";
    return (g_fail == 0 && nScen > 0) ? 0 : 1;
}
