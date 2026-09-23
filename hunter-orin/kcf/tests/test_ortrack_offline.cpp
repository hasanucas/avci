// =====================================================================
// test_ortrack_offline.cpp — ortrack_core.hpp'yi Python referansina karsi dogrular.
//
// golden.txt YoloORTrack/hybrid/geometry.py + adapters/ortrack.py +
// vendor/ORTrack .../head.py kodundan gen_golden.py ile uretildi.
// Bu test TensorRT / CUDA / kamera / FC GEREKTIRMEZ — masaustunde de kosar.
//
// Derleme:
//   g++ -O2 -std=c++17 test_ortrack_offline.cpp -o test_ortrack_offline \
//       $(pkg-config --cflags --libs opencv4)
// Kosma:
//   ./test_ortrack_offline golden.txt
// =====================================================================
#include "ortrack_core.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace ortrack;

// ------------------------------------------------------- sayaclar / raporlama
static int g_pass = 0, g_fail = 0;
static std::string g_section = "?";

static void check(bool ok, const std::string& what) {
    if (ok) { ++g_pass; return; }
    ++g_fail;
    if (g_fail <= 25)
        std::cout << "  [FAIL] (" << g_section << ") " << what << "\n";
}

static bool closeTo(double a, double b, double tol) {
    if (std::isnan(a) || std::isnan(b)) return false;
    const double d = std::fabs(a - b);
    return d <= tol || d <= tol * std::max(std::fabs(a), std::fabs(b));
}

static std::string fmt(double v) {
    char b[64]; std::snprintf(b, sizeof(b), "%.12g", v); return b;
}

// ------------------------------------- gen_golden.py make_frame ile BIREBIR
static cv::Mat makeFrame(int w, int h, uint32_t seed) {
    cv::Mat m(h, w, CV_8UC3);
    for (int y = 0; y < h; ++y) {
        uchar* row = m.ptr<uchar>(y);
        for (int x = 0; x < w; ++x) {
            for (int c = 0; c < 3; ++c) {
                uint32_t v = (uint32_t)x * 2654435761u
                           + (uint32_t)y * 2246822519u
                           + (uint32_t)c * 3266489917u
                           + seed * 668265263u;
                v ^= v >> 13;
                v *= 2654435761u;
                v ^= v >> 16;
                row[x * 3 + c] = (uchar)(v & 255u);
            }
        }
    }
    return m;
}

// ------------------------------------- gen_golden.py checksum ile BIREBIR
static void checksum(const cv::Mat& m, uint64_t& s1, uint64_t& s2, uint64_t& s3) {
    s1 = s2 = s3 = 0;
    uint64_t i = 0;
    for (int y = 0; y < m.rows; ++y) {
        const uchar* row = m.ptr<uchar>(y);
        for (int x = 0; x < m.cols * m.channels(); ++x) {
            const uint64_t b = row[x];
            ++i;
            s1 += b;
            s2 += b * i;
            s3 += b * b * i;
        }
    }
}

// ------------------------------------------------------------------ main
int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "golden.txt";
    std::ifstream f(path);
    if (!f) { std::cerr << "golden dosyasi acilamadi: " << path << "\n"; return 2; }

    std::cout << "=== ORTrack C++ <-> Python referans dogrulamasi ===\n";
    std::cout << "golden: " << path << "\n\n";

    NormalizeLUT lut;
    float hann[kFeatSz * kFeatSz];
    hannWindow(hann);

    // calBBox girdileri arasinda tasinan tampon
    std::vector<float> cbScore, cbSize, cbOff;
    // mapAndClip girdisi (MC_IN satiri MC/MC_INVALID'den ONCE gelir)
    bool  haveMcIn = false;
    int   mcW = 0, mcH = 0, mcOx = 0, mcOy = 0;
    double mcPx = 0, mcPy = 0, mcPw = 0, mcPh = 0, mcScale = 0;
    double mcCx = 0, mcCy = 0, mcBw = 0, mcBh = 0;

    int nST = 0, nCB = 0, nMC = 0, nMCinv = 0, nHann = 0;

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tag; ss >> tag;

        // ------------------------------------------------------ SECTION
        if (tag == "SECTION") { ss >> g_section; continue; }

        // --------------------------------------------------------- HANN
        if (tag == "HANN") {
            int y; ss >> y;
            for (int x = 0; x < kFeatSz; ++x) {
                double v; ss >> v;
                check(closeTo(hann[y * kFeatSz + x], v, 1e-6),
                      "hann[" + std::to_string(y) + "][" + std::to_string(x) + "] " +
                      fmt(hann[y * kFeatSz + x]) + " != " + fmt(v));
            }
            ++nHann;
            continue;
        }

        // ------------------------------------------------- SAMPLE_TARGET
        if (tag == "ST" || tag == "ST_FAIL") {
            int W, H; uint32_t seed;
            double bx, by, bw, bh, factor; int osz;
            ss >> W >> H >> seed >> bx >> by >> bw >> bh >> factor >> osz;

            const cv::Mat frame = makeFrame(W, H, seed);
            const BoxF box = BoxF::fromXYWH(bx, by, bw, bh);
            const Crop c = sampleTarget(frame, box, factor, osz);

            const std::string id = "ST(" + fmt(bx) + "," + fmt(by) + "," +
                                   fmt(bw) + "x" + fmt(bh) + ")";
            if (tag == "ST_FAIL") {
                check(!c.ok, id + " Python hata verdi ama C++ basarili dedi");
                continue;
            }
            if (!c.ok) { check(false, id + " C++ gecersiz dedi, Python basarili"); continue; }

            int side, ox, oy; double scale;
            uint64_t gs1, gs2, gs3;
            double gmean, gstd, g000, g1mq, g2ll;
            ss >> side >> scale >> ox >> oy >> gs1 >> gs2 >> gs3
               >> gmean >> gstd >> g000 >> g1mq >> g2ll;

            check(c.side == side, id + " side " + std::to_string(c.side) + " != " + std::to_string(side));
            check(c.left == ox,   id + " left " + std::to_string(c.left) + " != " + std::to_string(ox));
            check(c.top  == oy,   id + " top "  + std::to_string(c.top)  + " != " + std::to_string(oy));
            check(closeTo(c.scale, scale, 1e-15), id + " scale " + fmt(c.scale) + " != " + fmt(scale));

            uint64_t s1, s2, s3;
            checksum(c.patch, s1, s2, s3);
            check(s1 == gs1 && s2 == gs2 && s3 == gs3,
                  id + " patch checksum farkli (crop/pad/resize ayrisiyor)");

            // Normalize edilmis CHW tensoru: Python zinciriyle ayni mi?
            std::vector<float> chw((size_t)3 * osz * osz);
            lut.apply(c.patch, chw.data(), /*swapRB=*/false);   // golden RGB uzerinde uretildi

            double sum = 0.0;
            for (float v : chw) sum += v;
            const double mean = sum / (double)chw.size();
            double var = 0.0;
            for (float v : chw) { const double d = v - mean; var += d * d; }
            const double sd = std::sqrt(var / (double)chw.size());

            const size_t HW = (size_t)osz * osz;
            const float e000 = chw[0 * HW + 0];
            const float e1mq = chw[1 * HW + (size_t)(osz / 2) * osz + (osz / 4)];
            const float e2ll = chw[2 * HW + HW - 1];

            check(closeTo(mean, gmean, 1e-5), id + " chw.mean " + fmt(mean) + " != " + fmt(gmean));
            check(closeTo(sd,   gstd,  1e-5), id + " chw.std "  + fmt(sd)   + " != " + fmt(gstd));
            check(closeTo(e000, g000, 1e-6), id + " chw[0,0,0] "   + fmt(e000) + " != " + fmt(g000));
            check(closeTo(e1mq, g1mq, 1e-6), id + " chw[1,h/2,w/4] " + fmt(e1mq) + " != " + fmt(g1mq));
            check(closeTo(e2ll, g2ll, 1e-6), id + " chw[2,-1,-1] " + fmt(e2ll) + " != " + fmt(g2ll));
            ++nST;
            continue;
        }

        // ----------------------------------------------------- CAL_BBOX
        if (tag == "CB_IN_SCORE" || tag == "CB_IN_SIZE" || tag == "CB_IN_OFF") {
            std::vector<float>* dst = (tag == "CB_IN_SCORE") ? &cbScore
                                    : (tag == "CB_IN_SIZE")  ? &cbSize : &cbOff;
            dst->clear();
            double v;
            while (ss >> v) dst->push_back((float)v);
            continue;
        }
        if (tag == "CB_OUT") {
            double gcx, gcy, gw, gh, gscore, graw;
            ss >> gcx >> gcy >> gw >> gh >> gscore >> graw;
            const size_t N = kFeatSz * kFeatSz;
            if (cbScore.size() != N || cbSize.size() != 2 * N || cbOff.size() != 2 * N) {
                check(false, "CB girdi boyutu hatali"); continue;
            }
            const Pred p = calBBox(cbScore.data(), cbSize.data(), cbOff.data(), hann);
            check(closeTo(p.cx, gcx, 1e-6), "cal_bbox cx " + fmt(p.cx) + " != " + fmt(gcx));
            check(closeTo(p.cy, gcy, 1e-6), "cal_bbox cy " + fmt(p.cy) + " != " + fmt(gcy));
            check(closeTo(p.w,  gw,  1e-6), "cal_bbox w "  + fmt(p.w)  + " != " + fmt(gw));
            check(closeTo(p.h,  gh,  1e-6), "cal_bbox h "  + fmt(p.h)  + " != " + fmt(gh));
            check(closeTo(p.score,    gscore, 1e-6), "cal_bbox score "     + fmt(p.score)    + " != " + fmt(gscore));
            check(closeTo(p.rawScore, graw,   1e-6), "cal_bbox raw_score " + fmt(p.rawScore) + " != " + fmt(graw));
            ++nCB;
            continue;
        }

        // -------------------------------------------------- MAP + CLIP
        if (tag == "MC_IN") {
            ss >> mcW >> mcH >> mcPx >> mcPy >> mcPw >> mcPh
               >> mcScale >> mcOx >> mcOy >> mcCx >> mcCy >> mcBw >> mcBh;
            haveMcIn = true;
            continue;
        }
        if (tag == "MC" || tag == "MC_INVALID") {
            if (!haveMcIn) { check(false, "MC_IN olmadan MC satiri"); continue; }
            haveMcIn = false;

            Pred p; p.cx = mcCx; p.cy = mcCy; p.w = mcBw; p.h = mcBh;
            const BoxF prev = BoxF::fromXYWH(mcPx, mcPy, mcPw, mcPh);
            BoxF raw{}, out{};
            const bool ok = mapAndClip(p, prev, kSearchSize, mcScale,
                                       mcW, mcH, kMinBoxSize, raw, out);

            if (tag == "MC_INVALID") {
                check(!ok, "MC: Python gecersiz dedi, C++ gecerli dedi");
                ++nMCinv;
                continue;
            }
            double rx1, ry1, rx2, ry2, cx1, cy1, cx2, cy2;
            ss >> rx1 >> ry1 >> rx2 >> ry2 >> cx1 >> cy1 >> cx2 >> cy2;
            if (!ok) { check(false, "MC: C++ gecersiz dedi, Python gecerli"); continue; }

            check(closeTo(raw.x1, rx1, 1e-9), "raw.x1 " + fmt(raw.x1) + " != " + fmt(rx1));
            check(closeTo(raw.y1, ry1, 1e-9), "raw.y1 " + fmt(raw.y1) + " != " + fmt(ry1));
            check(closeTo(raw.x2, rx2, 1e-9), "raw.x2 " + fmt(raw.x2) + " != " + fmt(rx2));
            check(closeTo(raw.y2, ry2, 1e-9), "raw.y2 " + fmt(raw.y2) + " != " + fmt(ry2));
            check(closeTo(out.x1, cx1, 1e-9), "clip.x1 " + fmt(out.x1) + " != " + fmt(cx1));
            check(closeTo(out.y1, cy1, 1e-9), "clip.y1 " + fmt(out.y1) + " != " + fmt(cy1));
            check(closeTo(out.x2, cx2, 1e-9), "clip.x2 " + fmt(out.x2) + " != " + fmt(cx2));
            check(closeTo(out.y2, cy2, 1e-9), "clip.y2 " + fmt(out.y2) + " != " + fmt(cy2));
            ++nMC;
            continue;
        }
    }

    // ------------------------------------------------- ek: kendi tutarlilik
    g_section = "sanity";
    {
        // Banker's rounding: Python round() ile ayni olmali (std::round DEGIL)
        check(pyRound(0.5) == 0.0, "pyRound(0.5) ties-to-even degil");
        check(pyRound(1.5) == 2.0, "pyRound(1.5) ties-to-even degil");
        check(pyRound(2.5) == 2.0, "pyRound(2.5) ties-to-even degil");
        check(pyRound(-0.5) == 0.0, "pyRound(-0.5) ties-to-even degil");
        check(pyRound(-1.5) == -2.0, "pyRound(-1.5) ties-to-even degil");

        // Bos / bozuk kutu cokmemeli
        const cv::Mat fr = makeFrame(320, 240, 5);
        check(!sampleTarget(fr, BoxF{10, 10, 10, 20}, 4.0, 256).ok, "sifir genislik kabul edildi");
        check(!sampleTarget(fr, BoxF{10, 10, 20, 10}, 4.0, 256).ok, "sifir yukseklik kabul edildi");
        check(!sampleTarget(fr, BoxF{5000, 5000, 5020, 5020}, 4.0, 256).ok, "tamamen disari kabul edildi");
        check(!sampleTarget(cv::Mat(), BoxF{0, 0, 10, 10}, 4.0, 256).ok, "bos kare kabul edildi");

        // Cerceve ici kutu -> gecerli, cikti boyutu dogru
        const Crop c = sampleTarget(fr, BoxF::fromXYWH(100, 100, 40, 40), 4.0, 256);
        check(c.ok && c.patch.cols == 256 && c.patch.rows == 256 && c.patch.type() == CV_8UC3,
              "normal kirpma bozuk");

        // mapAndClip: bozuk resize_factor reddedilmeli
        BoxF a{}, b{};
        Pred p; p.cx = 0.5; p.cy = 0.5; p.w = 0.2; p.h = 0.2;
        check(!mapAndClip(p, BoxF::fromXYWH(10, 10, 20, 20), 256, 0.0, 320, 240, 10.0, a, b),
              "resize_factor=0 reddedilmedi");
        check(!mapAndClip(p, BoxF::fromXYWH(10, 10, 20, 20), 256,
                          std::numeric_limits<double>::quiet_NaN(), 320, 240, 10.0, a, b),
              "resize_factor=NaN reddedilmedi");

        // NormalizeLUT: bilinen degerler
        cv::Mat px(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));
        float o[3];
        lut.apply(px, o, false);
        check(closeTo(o[0], (0.0f - 0.485f) / 0.229f, 1e-6), "LUT siyah R yanlis");
        px.setTo(cv::Scalar(255, 255, 255));
        lut.apply(px, o, false);
        check(closeTo(o[2], (1.0f - 0.406f) / 0.225f, 1e-6), "LUT beyaz B yanlis");
        // swapRB gercekten kanal degistiriyor mu
        px.setTo(cv::Scalar(255, 0, 0));   // BGR mavi
        lut.apply(px, o, true);
        check(closeTo(o[2], (1.0f - 0.406f) / 0.225f, 1e-6) &&
              closeTo(o[0], (0.0f - 0.485f) / 0.229f, 1e-6), "swapRB kanal takasi yanlis");
    }

    std::cout << "\nvaka: hann=" << nHann << " satir, sample_target=" << nST
              << ", cal_bbox=" << nCB << ", map_clip=" << nMC
              << " (+" << nMCinv << " gecersiz)\n";
    std::cout << "-----------------------------------------\n";
    std::cout << (g_fail == 0 ? "GECTI  " : "KALDI  ")
              << g_pass << "/" << (g_pass + g_fail) << " kontrol\n";
    return g_fail == 0 ? 0 : 1;
}
