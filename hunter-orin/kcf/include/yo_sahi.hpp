// =====================================================================
// yo_sahi.hpp — [YO] SAHI tipi karolu tespit (YoloORTrack paketi portu).
//
// REFERANS: tests/yo_ref/hybrid/slicing.py (degistirilmeden kopyalandi)
//   tile_boxes (grid_rows > 0 yolu) -> gridTiles
//   global_detections                -> globalDetections
//   merge_detections(method="nms")   -> mergeNms
//
// Paket config'i (configs/orin-siyi-sahi-6-async-768.yaml):
//   2 satir x 3 sutun, overlap 0.20, tam kare cagrisi YOK, karo basina
//   UAV-YOLOv11m 640x768 motoru, karolar arasi NMS 0.5 (sinif duyarli).
//   1080p'de karolar ~720x608 / 800x608 -> motor olcegi ~0.96-1.05
//   (tam kare 768 letterbox'ta 0.40'ti: 10 px drone 4 px'e iniyordu).
//
// Ilk kisim SAF C++ (TensorRT yok) -> test_yo_sahi ile Python'a karsi
// dogrulanir. SahiDetector sadece hybrid::YoloDetector arayuzunu kullanir.
// =====================================================================
#pragma once

#include "yolo_trt.hpp"      // hybrid::YoloDetector, Detection, BoxF, boxIoU

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <vector>

namespace yo {

using hybrid::Detection;
using ortrack::BoxF;

struct TileRect { int x1 = 0, y1 = 0, x2 = 0, y2 = 0; };

struct SahiOptions {
    int    rows          = 2;      // grid_rows
    int    cols          = 3;      // grid_columns
    double overlap       = 0.20;   // overlap_ratio
    double mergeIoU      = 0.50;   // merge_iou_threshold (>= ile bastirir)
    bool   classAgnostic = false;  // class_agnostic
};

// ---------------------------------------------------------------------
// slicing.tile_boxes (grid_rows yolu). Esit hucreler, ic sinirlarda
// overlap_ratio'yu saglayacak kadar genisletilir; tam rows*cols karo.
//   pad   = length / count * overlap / (2 * (1 - overlap))
//   edges = i * length // count   (tamsayi bolme)
//   karo  = (floor(edge_i - pad) [i>0], ceil(edge_{i+1} + pad) [son degilse])
// Satir-oncelikli sira (Python list comprehension: once y, sonra x).
// ---------------------------------------------------------------------
inline bool gridAxis(int length, int count, double overlap, std::vector<std::pair<int,int>>& out)
{
    out.clear();
    if (length <= 0 || count <= 0 || !(overlap >= 0.0 && overlap < 1.0)) return false;
    const double pad = (double)length / (double)count * overlap / (2.0 * (1.0 - overlap));
    std::vector<long long> edges((size_t)count + 1);
    for (int i = 0; i <= count; ++i) edges[(size_t)i] = (long long)i * (long long)length / (long long)count;
    for (int i = 0; i < count; ++i) {
        const int a = i ? std::max(0, (int)std::floor((double)edges[(size_t)i] - pad)) : 0;
        const int b = (i + 1 < count) ? std::min(length, (int)std::ceil((double)edges[(size_t)i + 1] + pad))
                                      : length;
        out.emplace_back(a, b);
    }
    return true;
}

inline bool gridTiles(int W, int H, int rows, int cols, double overlap, std::vector<TileRect>& out)
{
    out.clear();
    if (W <= 0 || H <= 0 || rows <= 0 || cols <= 0) return false;
    if (W < cols || H < rows) return false;            // py: ValueError
    std::vector<std::pair<int,int>> ys, xs;
    if (!gridAxis(H, rows, overlap, ys) || !gridAxis(W, cols, overlap, xs)) return false;
    for (const auto& y : ys)
        for (const auto& x : xs)
            out.push_back(TileRect{x.first, y.first, x.second, y.second});
    return true;
}

// ---------------------------------------------------------------------
// slicing.global_detections: karo-yerel kutu -> kare koordinati + kirpma.
// Skor sonlu ve [0,1] icinde olmali; kirpilinca gecersizlesen kutu atilir.
// ---------------------------------------------------------------------
inline void globalDetections(const std::vector<Detection>& local, int x, int y, int W, int H,
                             std::vector<Detection>& out)
{
    for (const Detection& d : local) {
        if (!std::isfinite(d.score) || !(0.0 <= d.score && d.score <= 1.0)) continue;
        if (!d.box.ok()) continue;                                   // py: Box(...) ValueError
        const BoxF g{d.box.x1 + x, d.box.y1 + y, d.box.x2 + x, d.box.y2 + y};
        if (!g.ok()) continue;
        const BoxF c = g.clipped((double)W, (double)H);
        if (!c.ok()) continue;                                       // py: clip -> ValueError
        out.push_back(Detection{c, d.score, d.classId});
    }
}

// ---------------------------------------------------------------------
// slicing.merge_detections(method="nms"): deterministik acgozlu birlesim.
// Sira anahtari: (-score, class_id, x1, y1, x2, y2) — Python sorted ile ayni
// (label class'tan turedigi icin anahtara katkisi yok). Tohum her zaman
// kalanlarin ilki; IoU >= esik (ve sinif ayni ya da agnostic) olanlar silinir.
// ---------------------------------------------------------------------
inline void mergeNms(std::vector<Detection>& dets, double thr, bool agnostic)
{
    if (dets.empty()) return;
    std::vector<size_t> ord(dets.size());
    std::iota(ord.begin(), ord.end(), 0);
    std::stable_sort(ord.begin(), ord.end(), [&](size_t a, size_t b) {
        const Detection& A = dets[a]; const Detection& B = dets[b];
        if (-A.score != -B.score) return -A.score < -B.score;
        if (A.classId != B.classId) return A.classId < B.classId;
        if (A.box.x1 != B.box.x1) return A.box.x1 < B.box.x1;
        if (A.box.y1 != B.box.y1) return A.box.y1 < B.box.y1;
        if (A.box.x2 != B.box.x2) return A.box.x2 < B.box.x2;
        return A.box.y2 < B.box.y2;
    });
    std::vector<Detection> ordered; ordered.reserve(ord.size());
    for (size_t i : ord) ordered.push_back(dets[i]);

    std::vector<double> area(ordered.size());
    for (size_t i = 0; i < ordered.size(); ++i)
        area[i] = (ordered[i].box.x2 - ordered[i].box.x1) * (ordered[i].box.y2 - ordered[i].box.y1);

    std::vector<size_t> remaining(ordered.size());
    std::iota(remaining.begin(), remaining.end(), 0);
    std::vector<Detection> output;
    while (!remaining.empty()) {
        const size_t seed = remaining.front();
        const BoxF& s = ordered[seed].box;
        std::vector<size_t> keep;
        keep.reserve(remaining.size());
        for (size_t k = 1; k < remaining.size(); ++k) {
            const size_t r = remaining[k];
            const BoxF& b = ordered[r].box;
            // numpy: maximum(0, minimum(x2) - maximum(x1)) ... ayni islem sirasi
            const double iw = std::max(0.0, std::min(s.x2, b.x2) - std::max(s.x1, b.x1));
            const double ih = std::max(0.0, std::min(s.y2, b.y2) - std::max(s.y1, b.y1));
            const double inter = iw * ih;
            bool match = inter / (area[seed] + area[r] - inter) >= thr;
            if (!agnostic) match = match && ordered[r].classId == ordered[seed].classId;
            if (!match) keep.push_back(r);
        }
        output.push_back(ordered[seed]);
        remaining.swap(keep);
    }
    dets.swap(output);   // tohum sirasi zaten anahtar sirasi (py: sorted(output) ayni)
}

// ---------------------------------------------------------------------
// Bir tam taramanin istatistigi (log/rozet icin).
// ---------------------------------------------------------------------
struct ScanStats {
    int    tiles = 0;
    int    before = 0, after = 0;      // birlestirme oncesi / sonrasi kutu
    double tileMsSum = 0.0;            // karo cagrilarinin toplam suresi
    double inferMsSum = 0.0;           // sadece GPU (H2D+enqueue+D2H) toplami
    double mergeMs = 0.0;
    double totalMs = 0.0;
};

// ---------------------------------------------------------------------
// SahiDetector — her cagrida TUM karolar (paket: scan_calls=0, focus_size=0).
// Tek YoloDetector ornegi, karolar SIRAYLA (paket: parallel_workers=1).
// SADECE tarama thread'inden cagrilir (TensorRT context tek thread).
// ---------------------------------------------------------------------
class SahiDetector {
public:
    SahiDetector(hybrid::YoloDetector* yolo, const SahiOptions& o) : yolo_(yolo), opt_(o) {}

    const SahiOptions& options() const { return opt_; }

    // bgr: tam kare (BGR, CV_8UC3). out: KARE koordinatinda birlesmis tespitler.
    // false = bir karo cagrisi basarisiz (CUDA/TRT hatasi) -> sonuc KULLANILMAZ.
    bool detect(const cv::Mat& bgr, std::vector<Detection>& out, ScanStats& st)
    {
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();
        out.clear();
        st = ScanStats{};
        if (!yolo_ || bgr.empty() || bgr.type() != CV_8UC3) return false;

        const int W = bgr.cols, H = bgr.rows;
        if (W != lastW_ || H != lastH_) {
            if (!gridTiles(W, H, opt_.rows, opt_.cols, opt_.overlap, tiles_)) return false;
            lastW_ = W; lastH_ = H;
        }
        std::vector<Detection> local;
        for (const TileRect& t : tiles_) {
            const cv::Mat roi = bgr(cv::Rect(t.x1, t.y1, t.x2 - t.x1, t.y2 - t.y1));
            local.clear();
            if (!yolo_->detect(roi, local)) return false;     // karo koordinatinda, kirpilmis
            st.tileMsSum  += yolo_->lastMs();
            st.inferMsSum += yolo_->inferMs();
            globalDetections(local, t.x1, t.y1, W, H, out);
            ++st.tiles;
        }
        st.before = (int)out.size();
        const auto tm = Clock::now();
        mergeNms(out, opt_.mergeIoU, opt_.classAgnostic);
        const auto t1 = Clock::now();
        st.after   = (int)out.size();
        st.mergeMs = std::chrono::duration<double, std::milli>(t1 - tm).count();
        st.totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return true;
    }

    const std::vector<TileRect>& tiles() const { return tiles_; }

private:
    hybrid::YoloDetector* yolo_;
    SahiOptions opt_;
    std::vector<TileRect> tiles_;
    int lastW_ = -1, lastH_ = -1;
};

} // namespace yo
