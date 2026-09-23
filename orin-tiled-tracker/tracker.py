"""Kendi tracker'ımız (harici tracker bağımlılığı YOK).

Bileşenler:
  1) ByteTrack-tarzı İKİ AŞAMALI eşleme:
     - Aşama 1: yüksek güvenli tespitler <-> aktif track'ler (Hungarian, IoU)
     - Aşama 2: eşleşmeyen track'ler <-> düşük güvenli tespitler (occlusion kurtarma)
  2) Kalman filtresi (sabit hız modeli, [cx,cy,alan,oran] durumu):
     ölçüm gürültüsü R bilinçli BÜYÜK -> kutu ölçümündeki titreme filtrelenir.
  3) N-of-M onay penceresi: "son 30 karenin en az 20'sinde tespit" => onaylı.
     Onaysız track'ler ÇİZİLMEZ (false-positive parlamaları ekrana gelmez).
  4) One-Euro filtresi: çıkış kutusunun merkez(x,y) ve boyut(w,h) kanallarını
     ayrı ayrı yumuşatır. Düşük hızda jitter'ı, yüksek hızda lag'i azaltan
     ADAPTİF alçak geçiren filtredir -> "OC-SORT gibi titreme" problemi çözülür.
  5) Kayıp track max_age boyunca Kalman predict ile sürdürülür (ID kararlılığı).
"""
import math
from collections import deque
from typing import List, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment


# ----------------------------- One-Euro filtresi -----------------------------
class OneEuro:
    """1€ filtresi: cutoff frekansı sinyal hızına göre adapte olur.
    min_cutoff küçük -> daha yumuşak; beta büyük -> hızlı harekette az lag."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.05,
                 dcutoff: float = 1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.dcutoff = dcutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        r = 2.0 * math.pi * cutoff * dt
        return r / (r + 1.0)

    def __call__(self, x: float, t: float) -> float:
        if self.x_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dt = max(1e-3, t - self.t_prev)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.dcutoff, dt)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat


# ------------------------------- Kalman kutusu -------------------------------
class KalmanBox:
    """Durum: [cx, cy, s(alan), r(en/boy), vcx, vcy, vs]. Sabit hız modeli."""

    def __init__(self, box: np.ndarray):
        self.x = np.zeros((7, 1), dtype=np.float64)
        cx, cy, s, r = self._to_z(box)
        self.x[:4, 0] = [cx, cy, s, r]
        self.P = np.eye(7) * 10.0
        self.P[4:, 4:] *= 1000.0          # hız başlangıçta belirsiz
        self.F = np.eye(7)
        for i in range(3):
            self.F[i, i + 4] = 1.0
        self.H = np.zeros((4, 7))
        self.H[:4, :4] = np.eye(4)
        self.Q = np.eye(7) * 1e-2         # süreç gürültüsü (küçük)
        self.Q[4:, 4:] *= 1e-2
        self.R = np.eye(4) * 1.0          # ÖLÇÜM gürültüsü: büyük tut ->
        self.R[2:, 2:] *= 10.0            # kutu ölçüm titremesi yumuşar

    @staticmethod
    def _to_z(b) -> Tuple[float, float, float, float]:
        w = b[2] - b[0]
        h = b[3] - b[1]
        return (b[0] + w / 2.0, b[1] + h / 2.0, w * h, w / (h + 1e-9))

    def _to_box(self) -> np.ndarray:
        cx, cy, s, r = self.x[:4, 0]
        w = math.sqrt(max(s * r, 1e-6))
        h = s / (w + 1e-9)
        return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])

    def predict(self) -> np.ndarray:
        # alan negatif hıza düşmesin
        if self.x[2, 0] + self.x[6, 0] <= 0:
            self.x[6, 0] = 0.0
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self._to_box()

    def update(self, box: np.ndarray) -> np.ndarray:
        z = np.array(self._to_z(box), dtype=np.float64).reshape(4, 1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(7) - K @ self.H) @ self.P
        return self._to_box()


# ---------------------------------- Track ------------------------------------
class Track:
    _next_id = 1

    def __init__(self, det: np.ndarray, t: float, cfg: dict):
        self.kf = KalmanBox(det[:4])
        self.cls = int(det[5])
        self.conf = float(det[4])
        self.id = Track._next_id
        Track._next_id += 1
        self.time_since_update = 0
        self.cfg = cfg
        # N-of-M penceresi: 1=tespit var, 0=yok
        self.hits = deque(maxlen=int(cfg["confirm_window"]))
        self.hits.append(1)
        self.confirmed = False
        sm = cfg["smoothing"]
        self.fx = OneEuro(sm["min_cutoff"], sm["beta"])
        self.fy = OneEuro(sm["min_cutoff"], sm["beta"])
        self.fw = OneEuro(sm["min_cutoff"], sm["beta"])
        self.fh = OneEuro(sm["min_cutoff"], sm["beta"])
        self.smooth_box = det[:4].astype(np.float64).copy()
        self.pred_box = det[:4].astype(np.float64).copy()

    def predict(self) -> np.ndarray:
        self.time_since_update += 1
        self.pred_box = self.kf.predict()
        return self.pred_box

    def update(self, det: np.ndarray, t: float) -> None:
        box = self.kf.update(det[:4])
        self.conf = float(det[4])
        self.cls = int(det[5])
        self.time_since_update = 0
        self.hits.append(1)
        self._check_confirm()
        if self.cfg["smoothing"]["enabled"]:
            cx = (box[0] + box[2]) / 2.0
            cy = (box[1] + box[3]) / 2.0
            w = box[2] - box[0]
            h = box[3] - box[1]
            cx = self.fx(cx, t)
            cy = self.fy(cy, t)
            w = max(1.0, self.fw(w, t))
            h = max(1.0, self.fh(h, t))
            self.smooth_box = np.array([cx - w / 2, cy - h / 2,
                                        cx + w / 2, cy + h / 2])
        else:
            self.smooth_box = box

    def mark_missed(self) -> None:
        self.hits.append(0)

    def _check_confirm(self) -> None:
        # "Son M karenin en az N'inde tespit" => onaylı (config: 20/30)
        if sum(self.hits) >= int(self.cfg["confirm_min_hits"]):
            self.confirmed = True


# ------------------------------- yardımcılar ---------------------------------
def _iou_matrix(track_boxes: List[np.ndarray],
                det_boxes: np.ndarray) -> np.ndarray:
    if len(track_boxes) == 0 or len(det_boxes) == 0:
        return np.zeros((len(track_boxes), len(det_boxes)))
    M = np.zeros((len(track_boxes), len(det_boxes)))
    for i, tb in enumerate(track_boxes):
        xx1 = np.maximum(tb[0], det_boxes[:, 0])
        yy1 = np.maximum(tb[1], det_boxes[:, 1])
        xx2 = np.minimum(tb[2], det_boxes[:, 2])
        yy2 = np.minimum(tb[3], det_boxes[:, 3])
        w = np.clip(xx2 - xx1, 0, None)
        h = np.clip(yy2 - yy1, 0, None)
        inter = w * h
        a = (tb[2] - tb[0]) * (tb[3] - tb[1])
        b = (det_boxes[:, 2] - det_boxes[:, 0]) * \
            (det_boxes[:, 3] - det_boxes[:, 1])
        M[i] = inter / (a + b - inter + 1e-9)
    return M


# -------------------------------- ByteTracker --------------------------------
class ByteTracker:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.tracks: List[Track] = []

    def _match(self, tracks: List[Track], dets: np.ndarray,
               iou_thr: float):
        if not tracks or len(dets) == 0:
            return [], list(range(len(tracks))), list(range(len(dets)))
        tb = [t.pred_box for t in tracks]
        M = _iou_matrix(tb, dets[:, :4])
        cost = 1.0 - M
        ri, ci = linear_sum_assignment(cost)   # Hungarian (optimal eşleme)
        matches, ut, ud = [], list(range(len(tracks))), list(range(len(dets)))
        for r, c in zip(ri, ci):
            if M[r, c] >= iou_thr:
                matches.append((r, c))
                ut.remove(r)
                ud.remove(c)
        return matches, ut, ud

    def update(self, dets: np.ndarray, t: float) -> np.ndarray:
        """dets: (N,6) [x1,y1,x2,y2,conf,cls] (fusion sonrası, global koordinat)
        Dönüş: (K,7) [x1,y1,x2,y2,track_id,conf,cls] — SADECE onaylı (N-of-M)
        ve bu karede güncellenmiş, YUMUŞATILMIŞ kutular."""
        cfg = self.cfg
        for tr in self.tracks:
            tr.predict()

        if dets is not None and len(dets):
            high = dets[dets[:, 4] >= cfg["track_high_thresh"]]
            low = dets[(dets[:, 4] < cfg["track_high_thresh"]) &
                       (dets[:, 4] >= cfg["track_low_thresh"])]
        else:
            high = np.zeros((0, 6), dtype=np.float32)
            low = np.zeros((0, 6), dtype=np.float32)

        # --- Aşama 1: yüksek güven <-> tüm track'ler
        m1, ut1, ud1 = self._match(self.tracks, high, cfg["match_iou_thresh"])
        for r, c in m1:
            self.tracks[r].update(high[c], t)

        # --- Aşama 2: eşleşmeyen track'ler <-> düşük güven (kurtarma)
        rem = [self.tracks[i] for i in ut1]
        m2, urem, _ = self._match(rem, low, cfg["match_iou_thresh"])
        for r, c in m2:
            rem[r].update(low[c], t)
        for i in urem:
            rem[i].mark_missed()

        # --- Yeni track'ler: sadece eşleşmeyen YÜKSEK güvenli tespitlerden
        for c in ud1:
            if high[c, 4] >= cfg["new_track_thresh"]:
                self.tracks.append(Track(high[c], t, cfg))

        # --- max_age aşımı: sil (o ana dek predict ile sürdürüldü -> ID korunur)
        self.tracks = [tr for tr in self.tracks
                       if tr.time_since_update <= int(cfg["max_age"])]

        out = []
        for tr in self.tracks:
            if tr.confirmed and tr.time_since_update == 0:
                b = tr.smooth_box
                out.append([b[0], b[1], b[2], b[3],
                            float(tr.id), tr.conf, float(tr.cls)])
        return (np.array(out, dtype=np.float32)
                if out else np.zeros((0, 7), dtype=np.float32))
