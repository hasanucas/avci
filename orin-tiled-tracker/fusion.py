"""Tile'lar arası tespit birleştirme.

Overlap yüzünden aynı nesne birden fazla tile'da tespit edilir; burada
birleştirilir. İki yöntem:
  - NMS : en yüksek güvenli kutuyu tutar, örtüşenleri atar.
  - WBF : örtüşen kutuları güven-ağırlıklı ORTALAR (Weighted Box Fusion).
          Tile sınırında kısmi görünen kutular için lokalizasyon daha iyi.
Varsayılan: sınıf-bilinçli WBF.
"""
import numpy as np


def _iou_one_to_many(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a: (4,), b: (M,4) -> (M,) IoU."""
    xx1 = np.maximum(a[0], b[:, 0])
    yy1 = np.maximum(a[1], b[:, 1])
    xx2 = np.minimum(a[2], b[:, 2])
    yy2 = np.minimum(a[3], b[:, 3])
    w = np.clip(xx2 - xx1, 0, None)
    h = np.clip(yy2 - yy1, 0, None)
    inter = w * h
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / (area_a + area_b - inter + 1e-9)


def nms(dets: np.ndarray, iou_thres: float = 0.5) -> np.ndarray:
    if len(dets) == 0:
        return dets
    order = dets[:, 4].argsort()[::-1]
    dets = dets[order]
    keep = []
    while len(dets):
        keep.append(dets[0])
        if len(dets) == 1:
            break
        ious = _iou_one_to_many(dets[0, :4], dets[1:, :4])
        dets = dets[1:][ious < iou_thres]
    return np.array(keep, dtype=np.float32)


def wbf(dets: np.ndarray, iou_thres: float = 0.5) -> np.ndarray:
    """Weighted Box Fusion (sadeleştirilmiş): IoU >= eşik olan kutular tek
    kümede toplanır; koordinatlar güven-ağırlıklı ortalanır, güven = maksimum.
    """
    if len(dets) == 0:
        return dets
    order = dets[:, 4].argsort()[::-1]
    dets = dets[order]
    used = np.zeros(len(dets), dtype=bool)
    fused = []
    for i in range(len(dets)):
        if used[i]:
            continue
        used[i] = True
        ious = _iou_one_to_many(dets[i, :4], dets[:, :4])
        idx = [i] + [j for j in range(i + 1, len(dets))
                     if (not used[j]) and ious[j] >= iou_thres]
        for j in idx:
            used[j] = True
        cl = dets[idx]
        w = cl[:, 4:5]
        box = (cl[:, :4] * w).sum(axis=0) / w.sum()
        conf = float(cl[:, 4].max())
        fused.append(np.array([box[0], box[1], box[2], box[3],
                               conf, cl[0, 5]], dtype=np.float32))
    return np.stack(fused).astype(np.float32)


def fuse(dets: np.ndarray, method: str = "wbf", iou_thres: float = 0.5,
         class_agnostic: bool = False) -> np.ndarray:
    """dets: (N,6) [x1,y1,x2,y2,conf,cls] -> birleştirilmiş (M,6)."""
    if dets is None or len(dets) == 0:
        return np.zeros((0, 6), dtype=np.float32)
    fn = wbf if method == "wbf" else nms
    if class_agnostic:
        return fn(dets, iou_thres)
    out = []
    for cls in np.unique(dets[:, 5]):
        out.append(fn(dets[dets[:, 5] == cls], iou_thres))
    return np.vstack(out).astype(np.float32) if out else np.zeros((0, 6),
                                                                  np.float32)
