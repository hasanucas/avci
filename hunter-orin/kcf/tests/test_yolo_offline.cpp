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
            bool keep[kYoloNumClasses] = {false, false, true, false};
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
