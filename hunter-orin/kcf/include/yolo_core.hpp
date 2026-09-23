// =====================================================================
// yolo_core.hpp — YOLOv11 (ultralytics) on/son islemesi, SAF C++.
//
// TensorRT'ye BAGIMLI DEGILDIR: letterbox, normalize, cikti cozme, NMS ve
// kutuyu orijinal kareye geri tasima. Boylece TensorRT olmadan da
// derlenip test_yolo_offline ile dogrulanabilir.
//
// MODEL (models/UAV-YOLOv11m-768.onnx metadata'sindan DOGRULANDI):
//   giris  : images  (1, 3, 768, 768)   fp32, RGB, /255, letterbox 114
//   cikis  : output0 (1, 8, 12096)      8 = 4 kutu + 4 SINIF
//   siniflar: {0: airplane, 1: bird, 2: drone, 3: helicopter}
//   end2end=False -> NMS graf ICINDE DEGIL, burada yapilir.
//   12096 = 96^2 + 48^2 + 24^2  (stride 8/16/32 @ 768)
//
// ULTRALYTICS UYUMU — bilerek birebir:
//   * conf = TUM siniflar uzerinde MAX, cls = ARGMAX. Sinif filtresi
//     BUNDAN SONRA uygulanir. Yani en yuksek sinifi 'bird' olan bir kutu,
//     'drone' skoru esigin ustunde olsa bile ATILIR. (non_max_suppression
//     multi_label=False yolu.) Bu davranis KUS AYIRMA icin onemlidir.
//   * letterbox: scaleup=True, center=True, deger 114, round() ties-to-even.
//
// BILINCLI TEK FARK: kutuyu geri tasirken ultralytics'in yeniden hesaplanan
// pad formulu (icinde -0.1 duzeltmesi var) yerine, UYGULANAN letterbox'in
// TAM TERSI kullanilir. Fark <=0.2 px; manager IoU esigi 0.2 oldugu icin
// etkisiz, ama bizimki tam olarak tersinir.
// =====================================================================
#pragma once

#include "target_manager.hpp"   // hybrid::Detection, BoxF, boxIoU

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace hybrid {

// Model sinif indeksleri (ONNX metadata'sindan)
enum YoloClass { YOLO_AIRPLANE = 0, YOLO_BIRD = 1, YOLO_DRONE = 2, YOLO_HELICOPTER = 3 };
static constexpr int kYoloNumClasses = 4;
static constexpr unsigned char kLetterboxPad = 114;

inline const char* yoloClassName(int c) {
    switch (c) {
        case YOLO_AIRPLANE:   return "airplane";
        case YOLO_BIRD:       return "bird";
        case YOLO_DRONE:      return "drone";
        case YOLO_HELICOPTER: return "helicopter";
        default:              return "?";
    }
}

// ---------------------------------------------------------------------
struct LetterboxInfo {
    double r    = 1.0;   // olcek
    int    left = 0, top = 0;
    int    size = 0;     // kare giris kenari
    int    origW = 0, origH = 0;
    bool   ok   = false;
};

// ultralytics LetterBox(new_shape=(size,size), auto=False, scaleFill=False,
//                       scaleup=True, center=True), deger 114.
inline LetterboxInfo letterbox(const cv::Mat& bgr, int size, cv::Mat& out)
{
    LetterboxInfo li;
    if (bgr.empty() || bgr.type() != CV_8UC3 || size <= 0) return li;

    const int W = bgr.cols, H = bgr.rows;
    const double r = std::min((double)size / (double)H, (double)size / (double)W);
    // py: new_unpad = (round(w*r), round(h*r))   -> Python round = ties-to-even
    const int nw = (int)std::nearbyint((double)W * r);
    const int nh = (int)std::nearbyint((double)H * r);
    if (nw <= 0 || nh <= 0 || nw > size || nh > size) return li;

    const double dw = (double)(size - nw) / 2.0;
    const double dh = (double)(size - nh) / 2.0;
    const int top    = (int)std::nearbyint(dh - 0.1);
    const int bottom = (int)std::nearbyint(dh + 0.1);
    const int left   = (int)std::nearbyint(dw - 0.1);
    const int right  = (int)std::nearbyint(dw + 0.1);

    cv::Mat resized;
    if (nw != W || nh != H) cv::resize(bgr, resized, cv::Size(nw, nh));  // INTER_LINEAR
    else                    resized = bgr;

    // BORDER_ISOLATED: resized bir ROI degil ama alisknalik olarak acik
    // birakiliyor; ortrack_core.hpp'deki hata burada da olusamasin.
    cv::copyMakeBorder(resized, out, top, bottom, left, right,
                       cv::BORDER_CONSTANT | cv::BORDER_ISOLATED,
                       cv::Scalar(kLetterboxPad, kLetterboxPad, kLetterboxPad));
    if (out.cols != size || out.rows != size) return li;

    li.r = r; li.left = left; li.top = top; li.size = size;
    li.origW = W; li.origH = H; li.ok = true;
    return li;
}

// BGR uint8 HWC -> RGB float32 CHW, /255 (ultralytics: mean/std YOK).
class YoloNormalizeLUT {
public:
    YoloNormalizeLUT() { for (int p = 0; p < 256; ++p) lut_[p] = (float)p / 255.0f; }
    void apply(const cv::Mat& lb, float* dst) const {
        const int H = lb.rows, W = lb.cols, HW = H * W;
        float* dR = dst; float* dG = dst + HW; float* dB = dst + 2 * HW;
        for (int y = 0; y < H; ++y) {
            const uchar* s = lb.ptr<uchar>(y);
            for (int x = 0; x < W; ++x, s += 3) {
                *dR++ = lut_[s[2]];   // BGR -> R
                *dG++ = lut_[s[1]];
                *dB++ = lut_[s[0]];
            }
        }
    }
private:
    float lut_[256];
};

// ---------------------------------------------------------------------
struct RawDet {
    BoxF   box;      // letterbox piksel uzayinda xyxy
    float  score = 0.f;
    int    cls   = -1;
};

// output0 duzeni: [1][4+nc][N], satir-onceki. out[c*N + i]
//   c=0..3 : cx, cy, w, h   (letterbox piksel)
//   c=4..  : sinif skorlari (sigmoid uygulanmis)
inline void yoloDecode(const float* out0, int numClasses, int numAnchors,
                       float confThres, const bool* keepClass /* numClasses */,
                       std::vector<RawDet>& dets)
{
    dets.clear();
    const int N = numAnchors;
    for (int i = 0; i < N; ++i) {
        // py: conf, j = cls.max(1)  -> TUM siniflar uzerinde argmax
        int   best = 0;
        float bestS = out0[4 * N + i];
        for (int c = 1; c < numClasses; ++c) {
            const float s = out0[(4 + c) * N + i];
            if (s > bestS) { bestS = s; best = c; }
        }
        if (!(bestS > confThres)) continue;             // py: > (>= degil)
        if (keepClass && !keepClass[best]) continue;    // py: sinif filtresi ARGMAX'TAN SONRA

        const float cx = out0[0 * N + i], cy = out0[1 * N + i];
        const float w  = out0[2 * N + i], h  = out0[3 * N + i];
        if (!(w > 0.f) || !(h > 0.f) || !std::isfinite(cx) || !std::isfinite(cy)) continue;
        RawDet d;
        d.box = BoxF{(double)cx - (double)w / 2.0, (double)cy - (double)h / 2.0,
                     (double)cx + (double)w / 2.0, (double)cy + (double)h / 2.0};
        d.score = bestS;
        d.cls = best;
        dets.push_back(d);
    }
}

// Greedy NMS. agnostic=False: farkli siniflar birbirini bastirmaz
// (ultralytics bunu max_wh kaydirmasiyla yapar; burada dogrudan sinif esitligiyle).
inline void yoloNMS(std::vector<RawDet>& dets, float iouThres, int maxDet, int maxNms = 30000)
{
    if (dets.empty()) return;
    std::vector<int> idx(dets.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::stable_sort(idx.begin(), idx.end(),
                     [&](int a, int b) { return dets[a].score > dets[b].score; });
    if ((int)idx.size() > maxNms) idx.resize(maxNms);

    std::vector<char> dead(dets.size(), 0);
    std::vector<RawDet> kept;
    kept.reserve(std::min<size_t>(idx.size(), (size_t)maxDet));
    for (size_t a = 0; a < idx.size(); ++a) {
        const int i = idx[a];
        if (dead[i]) continue;
        kept.push_back(dets[i]);
        if ((int)kept.size() >= maxDet) break;
        for (size_t b = a + 1; b < idx.size(); ++b) {
            const int j = idx[b];
            if (dead[j] || dets[j].cls != dets[i].cls) continue;
            if (boxIoU(dets[i].box, dets[j].box) > iouThres) dead[j] = 1;   // py: > (>= degil)
        }
    }
    dets.swap(kept);
}

// Uygulanan letterbox'in TAM TERSI + orijinal kareye kirpma.
// Gecersiz (sifir alanli) kutu icin false doner.
inline bool unletterbox(const BoxF& in, const LetterboxInfo& li, BoxF& out)
{
    if (!li.ok || !(li.r > 0.0)) return false;
    BoxF b{(in.x1 - li.left) / li.r, (in.y1 - li.top) / li.r,
           (in.x2 - li.left) / li.r, (in.y2 - li.top) / li.r};
    b = b.clipped((double)li.origW, (double)li.origH);
    if (!b.ok()) return false;
    out = b;
    return true;
}

// Cozme + NMS + geri tasima; manager'in bekledigi Detection listesini uretir.
inline void yoloPostprocess(const float* out0, int numClasses, int numAnchors,
                            float confThres, float iouThres, int maxDet,
                            const bool* keepClass, const LetterboxInfo& li,
                            std::vector<Detection>& out)
{
    std::vector<RawDet> raw;
    yoloDecode(out0, numClasses, numAnchors, confThres, keepClass, raw);
    yoloNMS(raw, iouThres, maxDet);
    out.clear();
    out.reserve(raw.size());
    for (const RawDet& d : raw) {
        BoxF b;
        if (!unletterbox(d.box, li, b)) continue;
        out.push_back(Detection{b, (double)d.score, d.cls});
    }
}

} // namespace hybrid
