#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_manager_golden.py — hybrid/manager.py TargetManager'i senaryolarla surer
ve kare kare altin iz uretir (manager_golden.txt).

C++ portu (target_manager.hpp) ayni senaryoyu oynatip her karede ayni
durumu uretmek ZORUNDA. Test: test_target_manager.cpp

Senaryo dosyasi tracker cevaplarini ve YOLO tespit listelerini ONCEDEN
belirler; manager'in bunlari NE ZAMAN isteyecegi ve NE yapacagi test
edilen seydir.
"""
import sys
sys.path.insert(0, '.')
import numpy as np

from hybrid_ref.manager import TargetManager
from hybrid_ref.types import Box, Frame, Detection, DetectionBatch, TrackResult
from hybrid_ref.config import ManagerConfig

DUMMY = np.zeros((8, 8, 3), dtype=np.uint8)
MS = 1_000_000            # ns
FRAME_DT = 33 * MS        # ~30 fps
DET_DELAY = 30 * MS       # YOLO gecikmesi (kabul edilir)


class FakeTracker:
    """Senaryodan beslenen tracker. Manager'in NE ZAMAN cagirdigini olcer."""
    def __init__(self, script):
        self.script = script
        self.inits = 0
        self.resets = 0
        self.tracks = 0

    def initialize(self, frame, box):
        self.inits += 1
        return {}

    def track(self, frame):
        self.tracks += 1
        s = self.script[frame.frame_id]['trk']
        if not s['valid']:
            return TrackResult(None, s['score'], s['score'], False, {}, {})
        b = Box(*s['xyxy'])
        return TrackResult(b, s['score'], s['score'], True, {}, {})

    def reset(self):
        self.resets += 1


def cfg_from(**kw):
    c = ManagerConfig()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def build_script(rng, n, mode):
    """Her kare icin: tracker cevabi + o karede YOLO cagrilirsa donecek tespitler."""
    script = []
    cx, cy, w, h = 640.0, 360.0, 60.0, 60.0
    for i in range(n):
        # --- tracker cevabi
        if mode == 'clean':
            score = 0.75 + 0.1 * rng.random()
            valid = True
        elif mode == 'fade':                       # skor yavasca dusuyor
            score = max(0.02, 0.9 - i * 0.02)
            valid = True
        elif mode == 'dropout':                    # arada gecersiz kare
            score = 0.7 + 0.1 * rng.random()
            valid = (i % 17) != 0
        elif mode == 'drift':                      # kutu buyuyor (surüklenme)
            score = 0.8
            valid = True
        elif mode == 'fade_noyolo':                # skor duser, YOLO da bulamaz
            score = max(0.02, 0.9 - i * 0.05)
            valid = True
        elif mode == 'gap':                        # yakalama bosluğu
            score = 0.8
            valid = True
        else:                                      # noisy
            score = float(rng.random())
            valid = rng.random() > 0.1

        cx += float(rng.normal(0, 2.0)); cy += float(rng.normal(0, 2.0))
        if mode == 'drift':
            w *= 1.03; h *= 1.03
        cx = min(max(cx, 60.0), 1220.0); cy = min(max(cy, 60.0), 660.0)
        w = min(max(w, 12.0), 600.0);    h = min(max(h, 12.0), 400.0)
        score = float(np.float32(score))   # motor float32 uretir; referans da float32 gormeli
        trk = {'valid': valid, 'score': score,
               'xyxy': (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)}

        # --- o karede YOLO cagrilirsa donecek tespitler
        dets = []
        if mode == 'clean' or mode == 'fade' or mode == 'drift':
            miss = (i % 23) == 0                   # YOLO bazen kacirir
            if not miss:
                jx, jy = float(rng.normal(0, 6)), float(rng.normal(0, 6))
                dets.append({'xyxy': (cx - w / 2 + jx, cy - h / 2 + jy,
                                      cx + w / 2 + jx, cy + h / 2 + jy),
                             'score': 0.55 + 0.4 * rng.random(), 'cls': 2})
        elif mode == 'fade_noyolo':
            if i < 12:                             # once yakalasin, sonra YOLO sussun
                dets.append({'xyxy': (cx - w/2, cy - h/2, cx + w/2, cy + h/2),
                             'score': 0.8, 'cls': 2})
        elif mode == 'gap':
            dets.append({'xyxy': (cx - w/2, cy - h/2, cx + w/2, cy + h/2),
                         'score': 0.8, 'cls': 2})
        elif mode == 'dropout':
            if (i % 3) != 1:
                dets.append({'xyxy': (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                             'score': 0.7, 'cls': 2})
        else:                                      # noisy: alakasiz kutular da var
            for _ in range(int(rng.integers(0, 3))):
                bx = float(rng.uniform(0, 1100)); by = float(rng.uniform(0, 600))
                bw = float(rng.uniform(20, 160)); bh = float(rng.uniform(20, 160))
                dets.append({'xyxy': (bx, by, bx + bw, by + bh),
                             'score': float(rng.random()), 'cls': 2})
        # ESKI/BAYAT paket testi: bazi karelerde YOLO cok gec doner
        stale = (i % 41) == 40
        script.append({'trk': trk, 'dets': dets, 'stale': stale})
    return script


def run(name, mode, n, cfg, seed):
    rng = np.random.default_rng(seed)
    script = build_script(rng, n, mode)
    tracker = FakeTracker(script)
    mgr = TargetManager(tracker, cfg)

    lines = []
    # senaryo basligi: C++ ayni girdileri oynatabilsin diye
    lines.append("SCENARIO %s %d %s" % (name, n, _cfg_str(cfg)))
    for i, s in enumerate(script):
        t = s['trk']
        lines.append("IN_TRK %d %d %.17g %.17g %.17g %.17g %.17g"
                     % (i, 1 if t['valid'] else 0, t['score'], *t['xyxy']))
        lines.append("IN_DET %d %d %d" % (i, len(s['dets']), 1 if s['stale'] else 0))
        for d in s['dets']:
            lines.append("  DET %.17g %.17g %.17g %.17g %.17g %d"
                         % (*d['xyxy'], d['score'], d['cls']))

    for i, s in enumerate(script):
        captured = i * FRAME_DT
        if mode == 'gap' and i >= 40: captured += 1500 * MS   # >max_tracking_gap_ms
        frame = Frame(frame_id=i, captured_ns=captured, bgr=DUMMY)
        mgr.begin_frame(frame)
        if mgr.detection_due():
            gen = mgr.request_detection()
            completed = captured + DET_DELAY
            now = completed
            if s['stale']:                          # yasi asan paket -> reddedilmeli
                now = captured + 1200 * MS
            dets = [Detection(Box(*d['xyxy']), d['score'], d['cls'], 'drone') for d in s['dets']]
            batch = DetectionBatch(frame.frame_id, frame.captured_ns, gen, completed, dets, {})
            mgr.apply_detection(frame, batch, now_ns=now)
        mgr.finish_frame(frame)

        b = mgr.box
        c = mgr.candidate.box if mgr.candidate else None
        lines.append("OUT %d %s %d %s %d %s %d %d %d %d %d %s %d %d %d %d %d"
                     % (i, mgr.state.value,
                        1 if b else 0, _xyxy(b),
                        1 if c else 0, _xyxy(c),
                        mgr.hits, mgr.low_count, mgr.misses, mgr.unverified,
                        1 if mgr.weak else 0,
                        ("nan" if mgr.filtered_score is None else "%.12g" % mgr.filtered_score),
                        mgr.generation, mgr.rejected_detections,
                        tracker.inits, tracker.resets, tracker.tracks))
    lines.append("END")
    return lines


def _xyxy(b):
    if b is None:
        return "0 0 0 0"
    return "%.12g %.12g %.12g %.12g" % b.xyxy


_CFG_KEYS = ["detection_interval", "search_interval", "confirmation_hits", "confirmation_iou",
             "confirmation_timeout_ms", "verification_iou", "detection_accept_confidence",
             "tracker_low_score", "tracker_recover_score", "tracker_score_ema_alpha",
             "tracker_low_patience", "verification_miss_patience", "max_box_growth",
             "verify_interval", "verify_miss_patience", "verification_timeout_ms",
             "max_detection_age_ms", "max_tracking_gap_ms", "refresh_template"]


def _cfg_str(c):
    out = []
    for k in _CFG_KEYS:
        v = getattr(c, k)
        out.append("%.12g" % v if isinstance(v, float) else str(int(v) if isinstance(v, bool) else v))
    return " ".join(out)


all_lines = []
# orin-video-optimized.yaml degerleri
base = dict(detection_interval=3, search_interval=2, confirmation_hits=2, confirmation_iou=0.2,
            confirmation_timeout_ms=1500, verification_iou=0.2, detection_accept_confidence=0.5,
            tracker_low_score=0.2, tracker_recover_score=0.3, tracker_score_ema_alpha=0.4,
            tracker_low_patience=3, verification_miss_patience=2, verification_timeout_ms=2000,
            max_detection_age_ms=1000, max_tracking_gap_ms=1000, refresh_template=False)

all_lines += run("clean",        "clean",   200, cfg_from(**base), 1)
all_lines += run("fade",         "fade",    120, cfg_from(**base), 2)
all_lines += run("dropout",      "dropout", 200, cfg_from(**base), 3)
all_lines += run("noisy",        "noisy",   300, cfg_from(**base), 4)
all_lines += run("drift_growth", "drift",   150, cfg_from(**dict(base, max_box_growth=2.0)), 5)
all_lines += run("verify_iv",    "clean",   200, cfg_from(**dict(base, verify_interval=10, verify_miss_patience=2)), 6)
all_lines += run("refresh_tpl",  "clean",   150, cfg_from(**dict(base, refresh_template=True)), 7)
all_lines += run("fade_noyolo",  "fade_noyolo", 120, cfg_from(**base), 9)
all_lines += run("gap",          "gap",         90,  cfg_from(**base), 10)
all_lines += run("hits3",        "noisy",   250, cfg_from(**dict(base, confirmation_hits=3)), 8)

open("manager_golden.txt", "w").write("\n".join(all_lines) + "\n")
n_out = sum(1 for l in all_lines if l.startswith("OUT "))
print("manager_golden.txt: %d satir, %d kare izi, 8 senaryo" % (len(all_lines), n_out))
