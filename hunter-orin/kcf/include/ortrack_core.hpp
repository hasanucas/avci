// =====================================================================
// ortrack_core.hpp — ORTrack (OSTrack/DeiT-tiny) on/son isleme, SAF C++
//
// Bu baslik TensorRT'ye BAGIMLI DEGILDIR. Icinde sadece:
//   sample_target  (kare kirpma + sifir dolgu + resize)
//   normalize_chw  (uint8 HWC -> float32 CHW, ImageNet normalize)
//   hann_window    (16x16 pencere)
//   cal_bbox       (upstream CenterPredictor.cal_bbox)
//   map_prediction + clip_tracker_box  (kutuyu orijinal kareye tasi)
// var. Boylece bu mantik Orin'de TensorRT olmadan da derlenip
// test_ortrack_offline ile Python referansina karsi dogrulanabilir.
//
// REFERANS: YoloORTrack/hybrid/geometry.py + hybrid/adapters/ortrack.py +
//           vendor/ORTrack/lib/models/layers/head.py (CenterPredictor.cal_bbox)
// Karar zincirinin TAMAMI Python ile birebir ayni sirada yazildi; kayan
// nokta islem sirasi bilerek korundu (yorumlarda "Python:" satirlari).
//
// DIKKAT — Python round() yarim degerleri CIFTE yuvarlar (banker's rounding).
// C++'ta std::round yarim degerleri SIFIRDAN UZAGA yuvarlar. Bu yuzden
// std::nearbyint kullaniliyor (varsayilan FE_TONEAREST = ties-to-even).
// -ffast-math ile DERLEME: yuvarlama modunu bozar.
// =====================================================================
#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace ortrack {

// ---- Modelin SABIT geometrisi (checkpoint'e gomulu, degistirilemez) ----
// deit_tiny_patch16_224 + CENTER head, stride 16.
static constexpr int    kTemplateSize   = 128;
static constexpr int    kSearchSize     = 256;
static constexpr int    kFeatSz         = kSearchSize / 16;   // = 16
static constexpr double kTemplateFactor = 2.0;
static constexpr double kSearchFactor   = 4.0;
static constexpr double kMinBoxSize     = 10.0;  // upstream clip_box sismesi

// ImageNet normalizasyonu, RGB sirasinda (cfg.DATA.MEAN / STD)
static constexpr float kMeanRGB[3] = {0.485f, 0.456f, 0.406f};
static constexpr float kStdRGB [3] = {0.229f, 0.224f, 0.225f};

// ---------------------------------------------------------------------
// Kutu: xyxy, sag/alt DISLAYICI (exclusive) — Python types.Box ile ayni.
// ---------------------------------------------------------------------
struct BoxF {
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;

    double w()  const { return x2 - x1; }
    double h()  const { return y2 - y1; }

    // Python Box.__post_init__ ile ayni kabul kosulu.
    bool ok() const {
        return std::isfinite(x1) && std::isfinite(y1) &&
               std::isfinite(x2) && std::isfinite(y2) &&
               x2 > x1 && y2 > y1;
    }

    static BoxF fromXYWH(double x, double y, double w, double h) {
        return BoxF{x, y, x + w, y + h};
    }
    static BoxF fromRect(const cv::Rect& r) {
        return BoxF{(double)r.x, (double)r.y,
                    (double)r.x + (double)r.width, (double)r.y + (double)r.height};
    }

    // Ciziim/FRAME icin tam sayi kutu. Python tarafinda karsiligi yok:
    // sadece runtracker'in cv::Rect arayuzune koprudur.
    cv::Rect toRect() const {
        int rx = (int)std::lround(x1);
        int ry = (int)std::lround(y1);
        int rw = (int)std::lround(x2) - rx;
        int rh = (int)std::lround(y2) - ry;
        return cv::Rect(rx, ry, std::max(1, rw), std::max(1, rh));
    }

    // Python Box.clip: kirpar, GECERSIZ olursa ok()==false doner.
    BoxF clipped(double W, double H) const {
        return BoxF{std::max(0.0, x1), std::max(0.0, y1),
                    std::min(W, x2),   std::min(H, y2)};
    }
};

// Python round() = ties-to-even. nearbyint varsayilan kipte aynisini yapar.
inline double pyRound(double v) { return std::nearbyint(v); }

// ---------------------------------------------------------------------
// sample_target — geometry.py ile birebir.
//
// RENK NOTU: bu fonksiyon renk uzayindan BAGIMSIZDIR (sadece kirpar/dolgu
// ekler/olcekler). Python once tum kareyi BGR->RGB cevirip kirpiyor; biz
// BGR kirpip kanal takasini normalize_chw icinde yapiyoruz. Dolgu degeri
// sifir (siyah) iki uzayda da ayni oldugu icin sonuc BIREBIR esittir ve
// kare basina bir tam-kare cvtColor'dan kurtuluruz.
// ---------------------------------------------------------------------
struct Crop {
    bool    ok    = false;
    cv::Mat patch;          // output_size x output_size, CV_8UC3
    double  scale = 0.0;    // resize_factor = output_size / side
    int     left  = 0;      // yuvarlanmis kirpma kokeni (upstream mapping KULLANMAZ)
    int     top   = 0;
    int     side  = 0;
};

inline Crop sampleTarget(const cv::Mat& img, const BoxF& box,
                         double factor, int outputSize)
{
    Crop c;
    if (img.empty() || img.type() != CV_8UC3) return c;

    // Python: x, y, w, h = box.xywh
    const double x = box.x1, y = box.y1;
    const double w = box.w(), h = box.h();
    const double area = w * h;
    if (!std::isfinite(area) || area <= 0.0) return c;

    // Python: side = math.ceil(math.sqrt(w * h) * factor)
    const double sideD = std::ceil(std::sqrt(area) * factor);
    if (!std::isfinite(sideD) || sideD < 1.0 || sideD > 1e7) return c;
    const int side = (int)sideD;

    // Python: left, top = round(x + w/2 - side/2), round(y + h/2 - side/2)
    const int left   = (int)pyRound(x + w / 2.0 - side / 2.0);
    const int top    = (int)pyRound(y + h / 2.0 - side / 2.0);
    const int right  = left + side;
    const int bottom = top  + side;

    // Python: pl, pt = max(0,-left), max(0,-top)
    //         pr, pb = max(right - W + 1, 0), max(bottom - H + 1, 0)   (+1 upstream'den)
    const int pl = std::max(0, -left);
    const int pt = std::max(0, -top);
    const int pr = std::max(right  - img.cols + 1, 0);
    const int pb = std::max(bottom - img.rows + 1, 0);

    const int sx0 = left + pl,  sy0 = top + pt;
    const int sx1 = right - pr, sy1 = bottom - pb;

    // Python'da bos dilim cv2.copyMakeBorder'i PATLATIR (numpy sessizce bos verir).
    // Burada cokme yerine "gecersiz" donuyoruz: kutu tamamen kare disi demektir.
    if (sx1 <= sx0 || sy1 <= sy0) return c;
    if (sx0 < 0 || sy0 < 0 || sx1 > img.cols || sy1 > img.rows) return c;

    const cv::Mat roi = img(cv::Rect(sx0, sy0, sx1 - sx0, sy1 - sy0));

    cv::Mat bordered;
    if (pl || pr || pt || pb) {
        // BORDER_ISOLATED SART. OpenCV C++'ta src bir ROI ise copyMakeBorder
        // dolguyu ANA GORUNTUNUN gercek pikselleriyle doldurur (dokumante
        // davranis). Python'da numpy dilimi ebeveynini bilmedigi icin sabit
        // sifir dolgu olur. ISOLATED olmadan kare KENARLARINDA sessizce farkli
        // arama yamasi olusur -> referanstan ayrisma. Testte yakalandi.
        cv::copyMakeBorder(roi, bordered, pt, pb, pl, pr,
                           cv::BORDER_CONSTANT | cv::BORDER_ISOLATED,
                           cv::Scalar(0, 0, 0));
    } else {
        bordered = roi;
    }
    // Kenarlama sonrasi kare kenar uzunlugu tam olarak "side" olmali.
    if (bordered.cols != side || bordered.rows != side) return c;

    cv::resize(bordered, c.patch, cv::Size(outputSize, outputSize));  // INTER_LINEAR
    c.scale = (double)outputSize / (double)side;
    c.left = left; c.top = top; c.side = side; c.ok = true;
    return c;
}

// ---------------------------------------------------------------------
// normalize_chw — ((p/255) - mean) / std, HWC uint8 -> CHW float32.
//
// 3x256 LUT: bolme/cikarma kare basina degil, bir kez yapilir. Degerler
// Python zinciriyle BIT DUZEYINDE ayni (ayni float32 ifadesi kullanildi).
// swapRB=true iken kaynak BGR kabul edilir, cikis duzlemleri R,G,B olur.
// ---------------------------------------------------------------------
class NormalizeLUT {
public:
    NormalizeLUT() {
        for (int c = 0; c < 3; ++c)
            for (int p = 0; p < 256; ++p)
                // Python: ((t / 255.) - mean) / std   [float32]
                lut_[c][p] = (((float)p / 255.0f) - kMeanRGB[c]) / kStdRGB[c];
    }

    // dst: 3*H*W float, duzlemsel (plane 0 = R, 1 = G, 2 = B)
    void apply(const cv::Mat& patch, float* dst, bool swapRB) const {
        const int H = patch.rows, W = patch.cols;
        const int HW = H * W;
        const int c0 = swapRB ? 2 : 0;   // R kaynak kanali
        const int c1 = 1;                // G
        const int c2 = swapRB ? 0 : 2;   // B
        float* d0 = dst;
        float* d1 = dst + HW;
        float* d2 = dst + 2 * HW;
        for (int y = 0; y < H; ++y) {
            const uchar* src = patch.ptr<uchar>(y);
            for (int x = 0; x < W; ++x, src += 3) {
                *d0++ = lut_[0][src[c0]];
                *d1++ = lut_[1][src[c1]];
                *d2++ = lut_[2][src[c2]];
            }
        }
    }

private:
    float lut_[3][256];
};

// ---------------------------------------------------------------------
// hann_window — ORTrackBase.__init__ ile ayni (torch float32 zinciri).
//   h[i] = .5 * (1 - cos(2*pi/(n+1) * (i+1))),  i = 0..n-1
//   window[y][x] = h[y] * h[x]
// ---------------------------------------------------------------------
inline void hannWindow(float* win /* kFeatSz*kFeatSz */) {
    float h[kFeatSz];
    const float k = (float)(2.0 * M_PI / (double)(kFeatSz + 1));
    for (int i = 0; i < kFeatSz; ++i)
        h[i] = 0.5f * (1.0f - std::cos(k * (float)(i + 1)));
    for (int y = 0; y < kFeatSz; ++y)
        for (int x = 0; x < kFeatSz; ++x)
            win[y * kFeatSz + x] = h[y] * h[x];
}

// ---------------------------------------------------------------------
// cal_bbox — CenterPredictor.cal_bbox, batch 1.
//   response = score_map * hann
//   idx = argmax(response)
//   cx = (idx_x + offset[0][idx]) / feat_sz ; cy = (idx_y + offset[1][idx]) / feat_sz
//   w  = size[0][idx] ; h = size[1][idx]
// score       = max(response)   (Hann'lanmis tepe — mutlak guven DEGIL)
// raw_score   = max(score_map)
// ---------------------------------------------------------------------
struct Pred {
    double cx = 0, cy = 0, w = 0, h = 0;
    float  score = 0.f;
    float  rawScore = 0.f;
    int    idx = -1;
};

inline Pred calBBox(const float* scoreMap /* 256 */,
                    const float* sizeMap  /* 2*256 */,
                    const float* offMap   /* 2*256 */,
                    const float* hann     /* 256 */)
{
    constexpr int N = kFeatSz * kFeatSz;
    Pred p;
    int   best = 0;
    float bestV = -std::numeric_limits<float>::infinity();
    float rawMax = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < N; ++i) {
        const float r = scoreMap[i] * hann[i];   // torch: float32 carpim
        if (r > bestV) { bestV = r; best = i; }  // esitlikte ILK indeks (torch.max ile ayni)
        if (scoreMap[i] > rawMax) rawMax = scoreMap[i];
    }
    const int idxY = best / kFeatSz;
    const int idxX = best % kFeatSz;
    const float f  = (float)kFeatSz;
    // torch: (idx.to(float) + offset) / feat_sz   [float32]
    p.cx = (double)(((float)idxX + offMap[best])     / f);
    p.cy = (double)(((float)idxY + offMap[N + best]) / f);
    p.w  = (double)sizeMap[best];
    p.h  = (double)sizeMap[N + best];
    p.score = bestV;
    p.rawScore = rawMax;
    p.idx = best;
    return p;
}

// ---------------------------------------------------------------------
// map_prediction ("upstream") + off-frame reddi + clip_tracker_box.
//
// Python akisi (ortrack.py track()):
//     raw_box = map_prediction(pred, self.box, search_size, scale, origin, mapping)
//     raw_box.clip(W, H)          # DONUS DEGERI ATILIR — sadece ValueError icin
//     box = clip_tracker_box(raw_box, W, H, min_box_size)
// Yani clip_tracker_box KIRPILMAMIS kutuyu alir. Sira aynen korunmustur.
//
// Doner: true = gecerli. rawOut / boxOut doldurulur.
// ---------------------------------------------------------------------
inline bool mapAndClip(const Pred& p, const BoxF& prev, int searchSize,
                       double resizeFactor, int frameW, int frameH,
                       double minBox, BoxF& rawOut, BoxF& boxOut)
{
    if (!(resizeFactor > 0.0) || !std::isfinite(resizeFactor)) return false;

    const double s = (double)searchSize;
    // Python: [float(v) * size / resize_factor for v in pred]  (islem sirasi korundu)
    const double cx = p.cx * s / resizeFactor;
    const double cy = p.cy * s / resizeFactor;
    const double bw = p.w  * s / resizeFactor;
    const double bh = p.h  * s / resizeFactor;

    // Python (mapping == "upstream"):
    //   x, y, pw, ph = previous.xywh
    //   left = x + pw/2 - size/resize_factor/2
    const double pw = prev.w(), ph = prev.h();
    const double left = prev.x1 + pw / 2.0 - s / resizeFactor / 2.0;
    const double top  = prev.y1 + ph / 2.0 - s / resizeFactor / 2.0;

    // Python: Box.from_xywh((cx + left - w/2, cy + top - h/2, w, h))
    const double rx = cx + left - bw / 2.0;
    const double ry = cy + top  - bh / 2.0;
    BoxF raw{rx, ry, rx + bw, ry + bh};
    if (!raw.ok()) return false;               // Box.__post_init__ -> ValueError

    // Python: raw_box.clip(W, H) — tamamen kare disi kutuyu BURADA reddeder
    if (!raw.clipped((double)frameW, (double)frameH).ok()) return false;

    // Python: clip_tracker_box(raw_box, W, H, minimum)
    const double margin = std::min(minBox, std::min((double)frameW, (double)frameH));
    const double x1 = std::min(std::max(0.0, raw.x1), (double)frameW  - margin);
    const double y1 = std::min(std::max(0.0, raw.y1), (double)frameH - margin);
    const double x2 = std::min(std::max(margin, raw.x2), (double)frameW);
    const double y2 = std::min(std::max(margin, raw.y2), (double)frameH);
    const double ow = std::max(margin, x2 - x1);
    const double oh = std::max(margin, y2 - y1);

    rawOut = raw;
    boxOut = BoxF::fromXYWH(x1, y1, ow, oh);
    return boxOut.ok();
}

} // namespace ortrack
