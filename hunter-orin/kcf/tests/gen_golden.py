#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_golden.py — YoloORTrack/hybrid referans kodundan C++ portu icin altin deger uretir.

Cikti: golden.txt -> test_ortrack_offline.cpp okur.
Girdi kareler analitik uint32 hash ile uretilir (C++ tarafinda birebir ayni).
"""
import sys, math
sys.path.insert(0, '.')
import numpy as np

from hybrid_ref.geometry import sample_target, map_prediction, clip_tracker_box
from hybrid_ref.types import Box

RNG = np.random.default_rng(20260910)
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
FEAT = 16
SEARCH = 256


def make_frame(w, h, seed):
    """C++ ile birebir uretilebilen deterministik dokulu kare."""
    with np.errstate(over='ignore'):
        x = np.arange(w, dtype=np.uint32)[None, :, None]
        y = np.arange(h, dtype=np.uint32)[:, None, None]
        c = np.arange(3, dtype=np.uint32)[None, None, :]
        v = (x * np.uint32(2654435761) + y * np.uint32(2246822519)
             + c * np.uint32(3266489917) + np.uint32(seed) * np.uint32(668265263))
        v = v ^ (v >> np.uint32(13))
        v = v * np.uint32(2654435761)
        v = v ^ (v >> np.uint32(16))
        return (v & np.uint32(255)).astype(np.uint8)


def checksum(a):
    """3 parcali agirlikli toplam — C++'ta ayni sekilde hesaplanir, tasma yok."""
    b = a.reshape(-1).astype(np.uint64)
    i = np.arange(1, b.size + 1, dtype=np.uint64)
    s1 = int(b.sum())
    s2 = int((b * i).sum())
    s3 = int((b * b * i).sum())
    return s1, s2, s3


def hann():
    n = FEAT
    a = np.arange(1, n + 1, dtype=np.float32)
    h = (np.float32(.5) * (np.float32(1) - np.cos(np.float32(2 * np.pi / (n + 1)) * a))).astype(np.float32)
    return np.outer(h, h).astype(np.float32)


def cal_bbox(resp_f32, size_map, offset_map):
    flat = resp_f32.reshape(-1)
    idx = int(np.argmax(flat))
    idx_y, idx_x = idx // FEAT, idx % FEAT
    size = size_map.reshape(2, -1)[:, idx]
    off  = offset_map.reshape(2, -1)[:, idx]
    f = np.float32(FEAT)
    cx = np.float32(np.float32(idx_x) + off[0]) / f
    cy = np.float32(np.float32(idx_y) + off[1]) / f
    return float(cx), float(cy), float(size[0]), float(size[1])


def preprocess(patch_rgb):
    t = patch_rgb.astype(np.float32) / np.float32(255.0)
    t = (t - MEAN) / STD
    return np.transpose(t, (2, 0, 1)).astype(np.float32)


out = []
def emit(*p): out.append(" ".join(str(v) for v in p))

# ------------------------------------------------------------------ 1) HANN
w = hann()
emit("SECTION", "hann")
for y in range(FEAT):
    emit("HANN", y, " ".join("%.9g" % v for v in w[y]))

# --------------------------------------------------------- 2) SAMPLE_TARGET
cases = [
    (1280, 720,  615.0,  335.0,  50.0,  50.0, 4.0, 256),
    (1280, 720,  615.0,  335.0,  50.0,  50.0, 2.0, 128),
    (1280, 720,    0.0,    0.0,  20.0,  20.0, 4.0, 256),
    (1280, 720, 1260.0,  700.0,  20.0,  20.0, 4.0, 256),
    (1280, 720,  -30.0,  -30.0,  40.0,  40.0, 4.0, 256),
    (1280, 720, 1270.0,  715.0,  10.0,   5.0, 4.0, 256),
    (1280, 720,  600.0,  300.0,  11.0,  13.0, 4.0, 256),
    (1280, 720,  600.5,  300.5,  11.0,  13.0, 4.0, 256),
    (1280, 720,  640.0,  360.0, 300.0, 200.0, 4.0, 256),
    (1280, 720,  640.0,  360.0,  10.0,  10.0, 4.0, 256),
    (1280, 720,  637.5,  357.5,  25.0,  25.0, 4.0, 256),
    (1920,1080,  100.0,  100.0,  64.0,  48.0, 4.0, 256),
    ( 640, 480,  320.0,  240.0,  50.0,  50.0, 4.0, 256),
]
for i in range(40):
    W, H = 1280, 720
    bw = float(RNG.integers(6, 240)); bh = float(RNG.integers(6, 240))
    bx = float(RNG.integers(-60, W - 10)); by = float(RNG.integers(-60, H - 10))
    cases.append((W, H, bx, by, bw, bh, 4.0, 256))

emit("SECTION", "sample_target")
n_st = 0; n_stfail = 0
for (W, H, bx, by, bw, bh, factor, osz) in cases:
    seed = (int(abs(bx) * 7 + abs(by) * 13 + bw * 3 + bh) % 97) + 1
    frame = make_frame(W, H, seed)
    box = Box.from_xywh((bx, by, bw, bh))
    try:
        patch, scale, origin = sample_target(frame, box, factor, osz)
    except Exception as e:
        emit("ST_FAIL", W, H, seed, "%.9g" % bx, "%.9g" % by, "%.9g" % bw, "%.9g" % bh,
             "%.9g" % factor, osz); n_stfail += 1; continue
    side = math.ceil(math.sqrt(bw * bh) * factor)
    chw = preprocess(patch)
    s1, s2, s3 = checksum(patch)
    emit("ST", W, H, seed, "%.9g" % bx, "%.9g" % by, "%.9g" % bw, "%.9g" % bh,
         "%.9g" % factor, osz, side, "%.17g" % scale, origin[0], origin[1],
         s1, s2, s3,
         "%.9g" % float(chw.mean()), "%.9g" % float(chw.std()),
         "%.9g" % float(chw[0, 0, 0]),
         "%.9g" % float(chw[1, osz // 2, osz // 4]),
         "%.9g" % float(chw[2, osz - 1, osz - 1]))
    n_st += 1

# ------------------------------------------------------------- 3) CAL_BBOX
emit("SECTION", "cal_bbox")
for i in range(30):
    score = RNG.random((FEAT, FEAT)).astype(np.float32)
    size  = RNG.random((2, FEAT, FEAT)).astype(np.float32)
    off   = RNG.random((2, FEAT, FEAT)).astype(np.float32)
    if i == 0: score[:] = 0.1; score[0, 0] = 0.99
    if i == 1: score[:] = 0.1; score[FEAT-1, FEAT-1] = 0.99
    if i == 2: score[:] = 0.1; score[FEAT//2, FEAT//2] = 0.99
    resp = (score * w).astype(np.float32)
    cx, cy, bw_, bh_ = cal_bbox(resp, size, off)
    emit("CB_IN_SCORE", " ".join("%.9g" % v for v in score.reshape(-1)))
    emit("CB_IN_SIZE",  " ".join("%.9g" % v for v in size.reshape(-1)))
    emit("CB_IN_OFF",   " ".join("%.9g" % v for v in off.reshape(-1)))
    emit("CB_OUT", "%.12g" % cx, "%.12g" % cy, "%.12g" % bw_, "%.12g" % bh_,
         "%.12g" % float(resp.max()), "%.12g" % float(score.max()))

# ---------------------------------------------------------- 4) MAP + CLIP
emit("SECTION", "map_clip")
map_cases = []
for i in range(60):
    W, H = 1280, 720
    pbw = float(RNG.integers(8, 200)); pbh = float(RNG.integers(8, 200))
    pbx = float(RNG.integers(-40, W - 10)); pby = float(RNG.integers(-40, H - 10))
    side = math.ceil(math.sqrt(pbw * pbh) * 4.0)
    scale = SEARCH / side
    origin = (round(pbx + pbw/2 - side/2), round(pby + pbh/2 - side/2))
    map_cases.append((W, H, pbx, pby, pbw, pbh, scale, origin,
                      float(RNG.random()), float(RNG.random()),
                      float(RNG.random())*0.9+0.01, float(RNG.random())*0.9+0.01))
map_cases += [
    (1280, 720, 600.0, 300.0, 40.0, 40.0, 256/160, (520, 220),  5.0,  5.0, 0.05, 0.05),
    (1280, 720, 600.0, 300.0, 40.0, 40.0, 256/160, (520, 220), -5.0, -5.0, 0.05, 0.05),
    (1280, 720, 600.0, 300.0, 40.0, 40.0, 256/160, (520, 220),  0.5,  0.5, 0.0,  0.2),
    (1280, 720,   5.0,   5.0, 12.0, 12.0, 256/56,  (-16, -16), 0.02, 0.02, 0.02, 0.02),
]
n_inv = 0
for (W, H, pbx, pby, pbw, pbh, scale, origin, pcx, pcy, pw, ph) in map_cases:
    prev = Box.from_xywh((pbx, pby, pbw, pbh))
    try:
        raw = map_prediction((pcx, pcy, pw, ph), prev, SEARCH, scale, origin, "upstream")
        raw.clip(W, H)
        cb = clip_tracker_box(raw, W, H, 10.0)
        tag = ("MC", "%.12g" % raw.x1, "%.12g" % raw.y1, "%.12g" % raw.x2, "%.12g" % raw.y2,
               "%.12g" % cb.x1, "%.12g" % cb.y1, "%.12g" % cb.x2, "%.12g" % cb.y2)
    except ValueError:
        tag = ("MC_INVALID",); n_inv += 1
    emit("MC_IN", W, H, "%.17g" % pbx, "%.17g" % pby, "%.17g" % pbw, "%.17g" % pbh,
         "%.17g" % scale, origin[0], origin[1],
         "%.17g" % pcx, "%.17g" % pcy, "%.17g" % pw, "%.17g" % ph)
    emit(*tag)

emit("SECTION", "end")
open("golden.txt", "w").write("\n".join(out) + "\n")
print("golden.txt: %d satir | ST=%d (fail=%d)  CB=30  MC=%d (gecersiz=%d)"
      % (len(out), n_st, n_stfail, len(map_cases), n_inv))
