"""Batched tile inference.

KRİTİK TASARIM KARARI: Tile'lar için thread başına ayrı model çağrısı YAPILMAZ
(GIL + GPU context yarışı + kernel-launch overhead => yavaş). Tüm tile crop'ları
tek listede Ultralytics'e verilir; TensorRT engine dynamic-batch ile TEK
inference çağrısında işler. Orin Nano'da batch=4, tek görüntünün ~2.3 katı
sürer (4 katı değil) — asıl hız kazancı buradan gelir.
"""
import logging
import os
from typing import List

import numpy as np

from tiler import Tile

try:
    import torch
except ImportError:  # engine-only ortamda torch yine de Ultralytics ile gelir
    torch = None

from ultralytics import YOLO

log = logging.getLogger("detector")


class Detector:
    def __init__(self, mcfg: dict):
        self.imgsz = int(mcfg["imgsz"])
        self.conf = float(mcfg["conf_thres"])
        self.iou = float(mcfg["iou_thres"])
        tc = mcfg.get("target_classes") or []
        self.classes = [int(c) for c in tc] if tc else None

        engine = mcfg.get("engine_path", "") or ""
        pt = mcfg.get("pt_path", "") or ""

        self.is_engine = bool(engine) and os.path.exists(engine)
        if self.is_engine:
            log.info("TensorRT engine yükleniyor: %s", engine)
            self.model = YOLO(engine, task="detect")
            self.half = False  # precision engine'e gömülü; predict'e half geçme
        else:
            log.warning(
                "Engine bulunamadı (%s) -> .pt fallback: %s  "
                "[Orin'de 25 FPS için engine ŞART, build_engine.sh çalıştırın]",
                engine, pt)
            self.model = YOLO(pt)
            self.half = bool(mcfg.get("fp16", True))
            if torch is not None and torch.cuda.is_available():
                self.model.to("cuda")
            else:
                self.half = False  # CPU'da half yok
                log.warning("CUDA yok, CPU inference (çok yavaş, sadece test için)")

        # Sınıf isimleri (model-bağımsız etiketleme için)
        self.names = dict(getattr(self.model, "names", {}) or {})

        # Warm-up: ilk inference'lar (CUDA ctx, engine deserialization, autotune)
        # yavaştır; ölçümleri ve ilk kareleri bozmasın.
        try:
            n = max(1, int(mcfg.get("warmup_batch", 4)))
            dummy = [np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)] * n
            for _ in range(2):
                self.model.predict(dummy, imgsz=self.imgsz, conf=0.5,
                                   verbose=False)
            log.info("Detector warm-up tamam (batch=%d, imgsz=%d)", n, self.imgsz)
        except Exception as e:  # warm-up başarısızlığı ölümcül değil
            log.warning("Warm-up atlandı: %s", e)

    def infer_tiles(self, frame: np.ndarray, tiles: List[Tile]) -> np.ndarray:
        """Tüm tile'ları TEK batch olarak modele verir.

        Dönüş: (N, 6) float32 -> [x1, y1, x2, y2, conf, cls] GLOBAL koordinatta.
        """
        crops = [frame[t.y0:t.y1, t.x0:t.x1] for t in tiles]
        # Ultralytics: liste => batch; letterbox/resize/normalize'i kendi yapar
        results = self.model.predict(
            crops, imgsz=self.imgsz, conf=self.conf, iou=self.iou,
            classes=self.classes, verbose=False)

        out = []
        for t, res in zip(tiles, results):
            if res.boxes is None or len(res.boxes) == 0:
                continue
            b = res.boxes.xyxy.cpu().numpy()
            c = res.boxes.conf.cpu().numpy().reshape(-1, 1)
            k = res.boxes.cls.cpu().numpy().reshape(-1, 1)
            # Tile-lokal -> global projeksiyon
            b[:, [0, 2]] += t.x0
            b[:, [1, 3]] += t.y0
            out.append(np.hstack([b, c, k]))

        if not out:
            return np.zeros((0, 6), dtype=np.float32)
        return np.vstack(out).astype(np.float32)
