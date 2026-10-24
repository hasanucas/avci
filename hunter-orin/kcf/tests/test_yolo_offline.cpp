// =====================================================================
// test_yolo_offline.cpp — yolo_core.hpp'yi (letterbox / decode / NMS /
// geri tasima) referans uygulamaya karsi dogrular.
// Cikti tensoru duzeni GERCEK ONNX ile teyit edildi (satir 0-3 piksel
// kutu, 4-7 sinif skoru). TensorRT / CUDA GEREKTIRMEZ.
// =====================================================================
#include "yolo_core.hpp"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

using namespace hybrid;
static int g_pass = 0, g_fail = 0;
static std::string g_sec = "?";
static void check(bool ok, const std::string& w) {
    if (ok) { ++g_pass; return; }
    ++g_fail;
    if (g_fail <= 25) std::cout << "  [FAIL] (" << g_sec << ") " << w << "\n";
}
static bool near(double a, double b, double t = 1e-6) {
    const double d = std::fabs(a - b);
    return d <= t || d <= t * std::max(std::fabs(a), std::fabs(b));
}
static std::string f(double v) { char b[64]; std::snprintf(b, sizeof(b), "%.9g", v); return b; }

int main(int argc, char** argv) {
    std::ifstream in((argc > 1) ? argv[1] : "yolo_golden.txt");
    if (!in) { std::cerr << "golden acilamadi\n"; return 2; }
    std::cout << "=== YOLO cekirdek dogrulamasi ===\n\n";

    int nLB = 0, nCase = 0, nDet = 0;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream s(line);
        std::string t; s >> t;
        if (t == "SECTION") { s >> g_sec; continue; }

        // ---------------------------------------------- letterbox
        if (t == "LB") {
            int W, H, S, gTop, gLeft, gNw, gNh; double gr;
            s >> W >> H >> S >> gr >> gTop >> gLeft >> gNw >> gNh;
            cv::Mat src(H, W, CV_8UC3, cv::Scalar(10, 20, 30)), out;
            const LetterboxInfo li = letterbox(src, S, out);
            const std::string id = "LB " + std::to_string(W) + "x" + std::to_string(H) + ": ";
            check(li.ok, id + "basarisiz");
            if (!li.ok) continue;
            check(near(li.r, gr, 1e-15), id + "r " + f(li.r) + " != " + f(gr));
            check(li.top == gTop,   id + "top " + std::to_string(li.top) + " != " + std::to_string(gTop));
            check(li.left == gLeft, id + "left " + std::to_string(li.left) + " != " + std::to_string(gLeft));
            check(out.cols == S && out.rows == S, id + "cikti boyutu " +
                  std::to_string(out.cols) + "x" + std::to_string(out.rows));
            // dolgu degeri 114 mu (kose pikseli, dolgu varsa)
            if (gTop > 0) {
                const cv::Vec3b p = out.at<cv::Vec3b>(0, S / 2);
                check(p[0] == kLetterboxPad && p[1] == kLetterboxPad && p[2] == kLetterboxPad,
                      id + "dolgu 114 degil");
            }
            // tam tersinirlik: letterbox uzayindaki kutu -> orijinal
            BoxF b;
            const bool ok = unletterbox(BoxF{(double)gLeft, (double)gTop,
                                             (double)gLeft + gNw, (double)gTop + gNh}, li, b);
            // Tolerans 1/r px: letterbox nw/nh'yi TAM SAYIYA yuvarlar, bu yuzden
            // tersinirlik tam sayi resize'in izin verdigi kadardir. 1280x720 -> 768'de
            // r=0.6 ve 1280*0.6=768 tam bolundugu icin hata sifir; 333x777 gibi
            // bolunmeyen boyutlarda ~0.15 px kalir. Bu kirpma degil, YUVARLAMA.
            const double tolPx = 1.0 / li.r;
            check(ok && std::fabs(b.x1) <= tolPx && std::fabs(b.y1) <= tolPx &&
                  std::fabs(b.x2 - (double)W) <= tolPx && std::fabs(b.y2 - (double)H) <= tolPx,
                  id + "geri tasima tam kareyi vermiyor: " + f(b.x2) + "x" + f(b.y2));
            ++nLB;
            continue;
        }

        // ---------------------------------------------- decode + NMS
        if (t == "DEC_IN") {
            int c, N, nnz, nc, left, top; double r;
            s >> c >> N >> nnz >> nc >> r >> left >> top;
            std::vector<float> o((size_t)(4 + nc) * N, 0.0f);
            for (int k = 0; k < nnz; ++k) {
                std::getline(in, line);
                std::istringstream a(line);
                std::string at; int i; a >> at >> i;
                for (int row = 0; row < 4 + nc; ++row) { double v; a >> v; o[(size_t)row * N + i] = (float)v; }
            }
            std::getline(in, line);               // DEC_OUT c n
            std::istringstream d(line);
            std::string dt; int dc, n; d >> dt >> dc >> n;
            std::vector<std::vector<double>> want;
            for (int k = 0; k < n; ++k) {
                std::getline(in, line);
                std::istringstream rr(line);
                std::string rt; double x1, y1, x2, y2, sc; int cl;
                rr >> rt >> x1 >> y1 >> x2 >> y2 >> sc >> cl;
                want.push_back({x1, y1, x2, y2, sc, (double)cl});
            }

            LetterboxInfo li;
            li.ok = true; li.r = r; li.left = left; li.top = top;
            li.size = 768; li.origW = 1280; li.origH = 720;
            // [SK] keep dizisi artik YoloTrt::open ile AYNI yoldan uretilir
            // (otomatik hedef sinif). 4 sinifta eski sabit {F,F,T,F} ile ayni olmali.
            const int tgt = yoloResolveTargetClass(nc, -1);
            std::vector<char> keepBuf((size_t)std::max(nc, 1), 0);
            if (tgt >= 0) keepBuf[(size_t)tgt] = 1;
            bool keep[kYoloNumClasses] = {false, false, false, false};
            for (int q = 0; q < kYoloNumClasses && q < nc; ++q) keep[q] = keepBuf[(size_t)q] != 0;
            check(nc == kYoloNumClasses && !keep[0] && !keep[1] && keep[2] && !keep[3],
                  "vaka " + std::to_string(c) + ": otomatik keep eski {F,F,T,F} degil");
            std::vector<Detection> got;
            yoloPostprocess(o.data(), nc, N, 0.35f, 0.5f, 300, keep, li, got);

            const std::string id = "vaka " + std::to_string(c) + ": ";
            check(got.size() == want.size(), id + "tespit sayisi " +
                  std::to_string(got.size()) + " != " + std::to_string(want.size()));
            const size_t m = std::min(got.size(), want.size());
            for (size_t k = 0; k < m; ++k) {
                check(near(got[k].box.x1, want[k][0], 1e-5) && near(got[k].box.y1, want[k][1], 1e-5) &&
                      near(got[k].box.x2, want[k][2], 1e-5) && near(got[k].box.y2, want[k][3], 1e-5),
                      id + "kutu " + std::to_string(k) + " (" + f(got[k].box.x1) + "," +
                      f(got[k].box.y1) + ") != (" + f(want[k][0]) + "," + f(want[k][1]) + ")");
                check(near(got[k].score, want[k][4], 1e-6), id + "skor " + std::to_string(k));
                check(got[k].classId == (int)want[k][5], id + "sinif " + std::to_string(k) +
                      " " + std::to_string(got[k].classId) + " != " + std::to_string((int)want[k][5]));
                check(got[k].classId == YOLO_DRONE, id + "drone olmayan sinif sizdi: " +
                      yoloClassName(got[k].classId));
                ++nDet;
            }
            ++nCase;
        }
    }

    // ------------------------------------------------------------------
    // [SK] Sinif secimi + TEK SINIFLI motor (yolo11s-plane, 640x768, 1x5x10080)
    // ------------------------------------------------------------------
    g_sec = "sinif_secimi";
    {
        struct R { int nc, req, want; };
        const R tab[] = {
            {4, -1, 2}, {1, -1, 0}, {2, -1, -1}, {80, -1, -1}, {0, -1, -1},
            {4, 0, 0}, {4, 3, 3}, {4, 4, -1}, {1, 0, 0}, {1, 1, -1}, {0, 0, -1},
            {80, 5, 5}, {4, -7, 2}};
        for (const R& r : tab)
            check(yoloResolveTargetClass(r.nc, r.req) == r.want,
                  "resolve(nc=" + std::to_string(r.nc) + ", req=" + std::to_string(r.req) + ") = " +
                  std::to_string(yoloResolveTargetClass(r.nc, r.req)) + " != " + std::to_string(r.want));
        check(yoloClassLabel(4, 2) == "drone" && yoloClassLabel(1, 0) == "#0", "sinif etiketi");
    }

    g_sec = "tek_sinif";
    {
        // 640x768 motor: izgara 80*96 + 40*48 + 20*24
        const int inH = 640, inW = 768, nc = 1;
        const int N = (inH / 8) * (inW / 8) + (inH / 16) * (inW / 16) + (inH / 32) * (inW / 32);
        check(N == 10080, "640x768 izgara " + std::to_string(N) + " != 10080");

        // 1080p kare tam kare letterbox: r=0.4, 768x432, ust/alt 104 dolgu
        cv::Mat src(1080, 1920, CV_8UC3, cv::Scalar(1, 2, 3)), lb;
        const LetterboxInfo li = letterboxRect(src, inH, inW, lb);
        check(li.ok && near(li.r, 0.4, 1e-12) && li.left == 0 && li.top == 104 &&
              lb.cols == inW && lb.rows == inH, "1080p -> 640x768 letterbox");

        std::vector<float> o((size_t)(4 + nc) * N, 0.0f);
        auto put = [&](int i, float cx, float cy, float w, float h, float sc) {
            o[(size_t)0 * N + i] = cx; o[(size_t)1 * N + i] = cy;
            o[(size_t)2 * N + i] = w;  o[(size_t)3 * N + i] = h;
            o[(size_t)4 * N + i] = sc;
        };
        put(100,  200.f, 300.f, 20.f, 10.f, 0.62f);   // A: tutulur
        put(101,  201.f, 300.f, 20.f, 10.f, 0.55f);   // A ile IoU>0.5 -> NMS bastirir
        put(5000, 600.f, 200.f, 8.f,  6.f,  0.30f);   // B: conf 0.25 ustu, tutulur
        put(9000, 400.f, 400.f, 30.f, 30.f, 0.25f);   // C: tam esik -> ATILIR (> degil >=)
        put(9001, 100.f, 150.f, 12.f, 12.f, 0.10f);   // D: esik alti

        const int tgt = yoloResolveTargetClass(nc, -1);
        check(tgt == 0, "tek sinifta otomatik hedef " + std::to_string(tgt) + " != 0");
        std::unique_ptr<bool[]> keep(new bool[nc]());
        if (tgt >= 0 && tgt < nc) keep[tgt] = true;
        std::vector<Detection> got;
        yoloPostprocess(o.data(), nc, N, 0.25f, 0.5f, 300, keep.get(), li, got);
        check(got.size() == 2, "tek sinif tespit sayisi " + std::to_string(got.size()) + " != 2");
        if (got.size() == 2) {
            check(near(got[0].score, 0.62f, 1e-6) && got[0].classId == 0, "ilk tespit A");
            // A: letterbox (190,295)-(210,305) -> orijinal ((190-0)/0.4, (295-104)/0.4) ...
            check(near(got[0].box.x1, 475.0, 1e-4) && near(got[0].box.y1, 477.5, 1e-4) &&
                  near(got[0].box.x2, 525.0, 1e-4) && near(got[0].box.y2, 502.5, 1e-4),
                  "A geri tasima (" + f(got[0].box.x1) + "," + f(got[0].box.y1) + "," +
                  f(got[0].box.x2) + "," + f(got[0].box.y2) + ")");
            check(near(got[1].score, 0.30f, 1e-6) && got[1].classId == 0, "ikinci tespit B");
        }

        // ESKI davranisin kaniti: sabit {F,F,T,F} filtresi tek sinifli motorda
        // sinif 0'i atar -> HIC tespit yok (21 Ekim oncesi kod bu yuzden kullanilamazdi).
        bool oldKeep[kYoloNumClasses] = {false, false, true, false};
        std::vector<Detection> old;
        yoloPostprocess(o.data(), nc, N, 0.25f, 0.5f, 300, oldKeep, li, old);
        check(old.empty(), "eski sabit filtre tek sinifta tespit birakti");
    }

    g_sec = "sanity";
    {
        cv::Mat dummy, src(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
        check(!letterbox(cv::Mat(), 768, dummy).ok, "bos kare kabul edildi");
        check(!letterbox(src, 0, dummy).ok, "size=0 kabul edildi");
        LetterboxInfo bad; BoxF b;
        check(!unletterbox(BoxF{0, 0, 10, 10}, bad, b), "gecersiz letterbox kabul edildi");
        // tamamen kare disi kutu reddedilmeli
        LetterboxInfo li; li.ok = true; li.r = 0.6; li.left = 0; li.top = 168;
        li.origW = 1280; li.origH = 720;
        check(!unletterbox(BoxF{-500, -500, -400, -400}, li, b), "kare disi kutu kabul edildi");
    }

    std::cout << "\nletterbox=" << nLB << " vaka, decode=" << nCase
              << " vaka, karsilastirilan tespit=" << nDet << "\n";
    std::cout << "-----------------------------------------\n";
    std::cout << (g_fail == 0 ? "GECTI  " : "KALDI  ")
              << g_pass << "/" << (g_pass + g_fail) << " kontrol\n";
    return g_fail == 0 ? 0 : 1;
}
