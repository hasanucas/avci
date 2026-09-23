"""Opsiyonel GERÇEK ZAMANLI hafif görüntü stabilizasyonu.

Klasik iki-geçişli yöntemin (cumsum yörünge + moving average) aksine bu modül
TEK GEÇİŞLİDİR: yörünge tek yönlü EMA ile yumuşatılır, geleceğe bakılmaz ->
canlı RTSP'ye uygundur. Maliyet: 1080p'de CPU'da ~8-15 ms/kare.

Not: Sabit kamera senaryosunda (ekmek fabrikası vb.) genelde GEREKMEZ;
tracker'daki One-Euro kutu yumuşatma yeter. Sarsıntılı kamera (direk, drone)
için config'den açın ve PROFILE logundaki 'stab' süresine bakın.
"""
import logging

import cv2
import numpy as np

log = logging.getLogger("stabilizer")


class Stabilizer:
    def __init__(self, max_corners: int = 200, smoothing_ema: float = 0.85,
                 crop_ratio: float = 0.04):
        self.max_corners = int(max_corners)
        self.ema = float(smoothing_ema)
        self.crop = float(crop_ratio)
        self.prev_gray = None
        self.acc = np.zeros(3, dtype=np.float64)     # birikmiş [dx, dy, da]
        self.smooth = np.zeros(3, dtype=np.float64)  # EMA yörünge

    def apply(self, frame: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.prev_gray is None:
            self.prev_gray = gray
            return frame

        p0 = cv2.goodFeaturesToTrack(self.prev_gray, self.max_corners,
                                     qualityLevel=0.01, minDistance=30,
                                     blockSize=3)
        if p0 is None or len(p0) < 6:
            self.prev_gray = gray
            return frame

        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None)
        if p1 is None:
            self.prev_gray = gray
            return frame
        good0 = p0[st == 1]
        good1 = p1[st == 1]
        if len(good0) < 6:
            self.prev_gray = gray
            return frame

        m, _ = cv2.estimateAffinePartial2D(good0, good1)
        if m is None:
            self.prev_gray = gray
            return frame

        dx, dy = float(m[0, 2]), float(m[1, 2])
        da = float(np.arctan2(m[1, 0], m[0, 0]))
        self.acc += [dx, dy, da]

        # Tek yönlü EMA: smooth = ema*smooth + (1-ema)*acc  (geleceğe bakmaz)
        self.smooth = self.ema * self.smooth + (1.0 - self.ema) * self.acc
        diff = self.smooth - self.acc
        dx_c, dy_c, da_c = dx + diff[0], dy + diff[1], da + diff[2]

        h, w = frame.shape[:2]
        T = np.array([[np.cos(da_c), -np.sin(da_c), dx_c],
                      [np.sin(da_c),  np.cos(da_c), dy_c]], dtype=np.float64)
        out = cv2.warpAffine(frame, T, (w, h))

        # Kenarda oluşan siyah bantları örtmek için hafif zoom
        if self.crop > 0:
            M2 = cv2.getRotationMatrix2D((w / 2, h / 2), 0, 1.0 + self.crop)
            out = cv2.warpAffine(out, M2, (w, h))

        self.prev_gray = gray
        return out
