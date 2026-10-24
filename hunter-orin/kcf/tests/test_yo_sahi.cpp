// =====================================================================
// test_yo_sahi.cpp — yo_sahi.hpp + letterboxGeom'u YoloORTrack Python
// referansina (tests/yo_ref/hybrid/slicing.py, adapters/yolo_gpu.py) karsi
// dogrular. yo_sahi_golden.txt -> gen_yo_sahi_golden.py
//
// TensorRT / CUDA / kamera / FC GEREKTIRMEZ.
//   ./test_yo_sahi ../tests/yo_sahi_golden.txt
// =====================================================================
#include "yo_sahi.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) { ++g_pass; return; }
    ++g_fail;
    if (g_fail <= 40) std::cout << "  [FAIL] " << what << "\n";
}
static double num(std::istringstream& s) {          // "nan"/"inf" dahil
    std::string t; s >> t; return std::strtod(t.c_str(), nullptr);
}
static bool same(double a, double b) {               // birebir (round-trip repr)
    if (std::isnan(a) && std::isnan(b)) return true;
    return a == b;
}

int main(int argc, char** argv)
{
    const std::string path = (argc > 1) ? argv[1] : "yo_sahi_golden.txt";
    std::ifstream f(path);
    if (!f) { std::cerr << "golden acilamadi: " << path << "\n"; return 2; }
    std::cout << "=== yo_sahi C++ <-> YoloORTrack Python referansi ===\n";

    int nTile = 0, nLb = 0, nMerge = 0;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream h(line);
        std::string tag; h >> tag;
        if (tag == "TILE") {
            int W, H, r, c, n; h >> W >> H >> r >> c; double ov = num(h); h >> n;
            std::vector<yo::TileRect> t;
            const bool ok = yo::gridTiles(W, H, r, c, ov, t);
            check(ok, "gridTiles basarisiz " + line);
            check((int)t.size() == n, "karo sayisi " + line);
            for (int k = 0; k < n; ++k) {
                std::getline(f, line);
                std::istringstream s(line); std::string tt; int x1, y1, x2, y2;
                s >> tt >> x1 >> y1 >> x2 >> y2;
                if (k < (int)t.size())
                    check(t[k].x1 == x1 && t[k].y1 == y1 && t[k].x2 == x2 && t[k].y2 == y2,
                          "karo " + std::to_string(k) + " " + std::to_string(W) + "x" + std::to_string(H));
            }
            ++nTile;
        } else if (tag == "LB") {
            int w0, h0, ih, iw, nw, nh, tp, bt, lf, rt;
            h >> w0 >> h0 >> ih >> iw; const double g = num(h);
            h >> nw >> nh >> tp >> bt >> lf >> rt;
            const hybrid::LetterboxGeom lg = hybrid::letterboxGeom(w0, h0, ih, iw);
            const std::string id = std::to_string(w0) + "x" + std::to_string(h0) + "->" +
                                   std::to_string(ih) + "x" + std::to_string(iw);
            check(lg.ok, "letterbox ok " + id);
            check(same(lg.r, g), "gain " + id);
            check(lg.nw == nw && lg.nh == nh, "yeni boyut " + id);
            check(lg.top == tp && lg.bottom == bt && lg.left == lf && lg.right == rt, "pad " + id);
            check(lg.nh + lg.top + lg.bottom == ih && lg.nw + lg.left + lg.right == iw, "toplam " + id);
            ++nLb;
        } else if (tag == "MERGE") {
            int W, H, n, ag; h >> W >> H >> n; const double thr = num(h); h >> ag;
            std::vector<hybrid::Detection> glob;
            for (int k = 0; k < n; ++k) {
                std::getline(f, line);
                std::istringstream s(line); std::string tt; int tx, ty; s >> tt >> tx >> ty;
                const double x1 = num(s), y1 = num(s), x2 = num(s), y2 = num(s), sc = num(s);
                int cls; s >> cls;
                std::vector<hybrid::Detection> local{hybrid::Detection{ortrack::BoxF{x1, y1, x2, y2}, sc, cls}};
                yo::globalDetections(local, tx, ty, W, H, glob);
            }
            yo::mergeNms(glob, thr, ag != 0);
            std::getline(f, line);
            std::istringstream o(line); std::string ot; int m; o >> ot >> m;
            check(ot == "OUT", "OUT satiri bekleniyordu");
            check((int)glob.size() == m, "birlesim sayisi (MERGE #" + std::to_string(nMerge) + ") " +
                  std::to_string(glob.size()) + " != " + std::to_string(m));
            for (int k = 0; k < m; ++k) {
                std::getline(f, line);
                std::istringstream s(line); std::string tt; s >> tt;
                const double x1 = num(s), y1 = num(s), x2 = num(s), y2 = num(s), sc = num(s);
                int cls; s >> cls;
                if (k < (int)glob.size()) {
                    const hybrid::Detection& d = glob[(size_t)k];
                    check(same(d.box.x1, x1) && same(d.box.y1, y1) && same(d.box.x2, x2) &&
                          same(d.box.y2, y2) && same(d.score, sc) && d.classId == cls,
                          "birlesim elemani " + std::to_string(k) + " (MERGE #" + std::to_string(nMerge) + ")");
                }
            }
            ++nMerge;
        }
    }
    std::cout << "TILE " << nTile << "  LB " << nLb << "  MERGE " << nMerge << "\n";
    std::cout << "SONUC: " << g_pass << "/" << (g_pass + g_fail) << " kontrol gecti\n";
    return (g_fail == 0 && nTile > 0 && nLb > 0 && nMerge > 0) ? 0 : 1;
}
