#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_yolo_golden.py — yolo_core.hpp (letterbox/decode/NMS) icin altin deger.

Cikis tensorunun DUZENI gercek ONNX ile dogrulandi (satir 0-3 piksel kutu,
4-7 sinif skoru). Burada decode+NMS mantigi ultralytics semantigiyle
numpy'de yeniden yazilip C++ portuna karsi kullanilir.
"""
import numpy as np, cv2

NC, S = 4, 768
CONF, IOU, MAXDET = 0.35, 0.5, 300
KEEP = [2]                                   # sadece 'drone'
rng = np.random.default_rng(4242)


def ul_letterbox(W, H, size):
    r = min(size / H, size / W)
    nw, nh = round(W * r), round(H * r)
    dw, dh = (size - nw) / 2, (size - nh) / 2
    return r, round(dh - 0.1), round(dw - 0.1), nw, nh


def iou(a, b):
    iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    it = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - it
    return it / ua if ua > 0 else 0.0


def decode_nms(o, r, left, top, W, H):
    cls = o[4:]
    j = cls.argmax(0)                        # ilk maksimum
    s = cls.max(0)
    dets = []
    for i in range(o.shape[1]):
        if not (s[i] > CONF): continue
        if j[i] not in KEEP: continue        # sinif filtresi ARGMAX'TAN SONRA
        cx, cy, w, h = float(o[0,i]), float(o[1,i]), float(o[2,i]), float(o[3,i])
        if not (w > 0 and h > 0): continue
        dets.append([cx-w/2, cy-h/2, cx+w/2, cy+h/2, float(s[i]), int(j[i])])
    order = sorted(range(len(dets)), key=lambda k: -dets[k][4])
    dead = [False]*len(dets); kept = []
    for a in range(len(order)):
        i = order[a]
        if dead[i]: continue
        kept.append(dets[i])
        if len(kept) >= MAXDET: break
        for b in range(a+1, len(order)):
            jj = order[b]
            if dead[jj] or dets[jj][5] != dets[i][5]: continue
            if iou(dets[i][:4], dets[jj][:4]) > IOU: dead[jj] = True
    out = []
    for d in kept:
        x1 = (d[0]-left)/r; y1 = (d[1]-top)/r; x2 = (d[2]-left)/r; y2 = (d[3]-top)/r
        x1, y1 = max(0.0, x1), max(0.0, y1); x2, y2 = min(float(W), x2), min(float(H), y2)
        if not (x2 > x1 and y2 > y1): continue
        out.append([x1, y1, x2, y2, d[4], d[5]])
    return out


lines = []
base = np.load("yolo_out0.npy")               # gercek ONNX ciktisi (8, 12096)
N = base.shape[1]

# ---- letterbox parametreleri (birden fazla kare boyutu)
lines.append("SECTION letterbox")
for (W, H) in [(1280,720),(1920,1080),(640,480),(768,768),(800,600),(1024,768),(1280,721),(333,777)]:
    r, top, left, nw, nh = ul_letterbox(W, H, S)
    lines.append("LB %d %d %d %.17g %d %d %d %d" % (W, H, S, r, top, left, nw, nh))

# ---- decode + NMS vakalari
lines.append("SECTION decode")
W, H = 1280, 720
r, top, left, _, _ = ul_letterbox(W, H, S)
for case in range(12):
    o = base.copy()
    o[4:] = 0.0
    ninj = int(rng.integers(1, 14))
    for _ in range(ninj):
        i = int(rng.integers(0, N))
        cx, cy = float(rng.uniform(60, S-60)), float(rng.uniform(60, S-60))
        w, h = float(rng.uniform(14, 180)), float(rng.uniform(14, 180))
        o[0,i], o[1,i], o[2,i], o[3,i] = cx, cy, w, h
        # bazen 'bird' baskin olsun -> drone skoru yuksek olsa bile ATILMALI
        if rng.random() < 0.3:
            o[4+1, i] = float(rng.uniform(0.6, 0.95))
            o[4+2, i] = float(rng.uniform(0.36, 0.59))
        else:
            o[4+2, i] = float(rng.uniform(0.36, 0.98))
            o[4+0, i] = float(rng.uniform(0.0, 0.3))
        # ustuste binen kopyalar -> NMS testi
        if rng.random() < 0.5:
            k = (i + int(rng.integers(1, 5))) % N
            o[0,k], o[1,k] = cx + rng.uniform(-6, 6), cy + rng.uniform(-6, 6)
            o[2,k], o[3,k] = w * 1.05, h * 0.97
            o[4+2, k] = float(rng.uniform(0.36, 0.98))
    o = o.astype(np.float32)
    res = decode_nms(o, r, top and top or top, left, W, H) if False else decode_nms(o, r, left, top, W, H)
    nz = np.nonzero(o[4:].max(0) > 0.2)[0]
    lines.append("DEC_IN %d %d %d %d %.17g %d %d" % (case, N, len(nz), NC, r, left, top))
    for i in nz:
        lines.append("  A %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g"
                     % (i, o[0,i], o[1,i], o[2,i], o[3,i], o[4,i], o[5,i], o[6,i], o[7,i]))
    lines.append("DEC_OUT %d %d" % (case, len(res)))
    for d in res:
        lines.append("  R %.12g %.12g %.12g %.12g %.12g %d" % tuple(d))

lines.append("SECTION end")
open("yolo_golden.txt","w").write("\n".join(lines)+"\n")
print("yolo_golden.txt: %d satir, 12 decode vakasi, %d tespit"
      % (len(lines), sum(1 for l in lines if l.startswith("  R "))))
