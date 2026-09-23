"""Kaynak yakalama: dosya + RTSP.

- Jetson'da donanım decode: GStreamer + nvv4l2decoder (Orin Nano decode'u
  donanımda yapar; 11x 1080p30 H.265'e kadar). appsink drop=true max-buffers=1
  => RTSP'de gecikme birikmez, hep EN GÜNCEL kare işlenir.
- Dosya modunda process_every_frame=true ise HER kare sırayla işlenir
  (offline analiz); false ise dosya da "en güncel kare" gibi akar.
- RTSP koparsa otomatik yeniden bağlanır.
- GStreamer açılamazsa (örn. PC'de test) standart cv2.VideoCapture'a düşer.
"""
import logging
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np

log = logging.getLogger("capture")


def build_gst_pipeline(uri: str, latency_ms: int = 100, is_rtsp: bool = True,
                       codec: str = "h264") -> str:
    if is_rtsp:
        depay = ("rtph264depay ! h264parse" if codec == "h264"
                 else "rtph265depay ! h265parse")
        return (f"rtspsrc location={uri} latency={latency_ms} ! {depay} ! "
                f"nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! "
                f"videoconvert ! video/x-raw,format=BGR ! "
                f"appsink drop=true max-buffers=1 sync=false")
    # Dosya: decodebin Jetson'da nvv4l2decoder'ı otomatik seçer (konteyner/codec
    # bağımsız, mp4/mkv/avi çalışır)
    return (f"filesrc location={uri} ! decodebin ! nvvidconv ! "
            f"video/x-raw,format=BGRx ! videoconvert ! "
            f"video/x-raw,format=BGR ! appsink sync=false")


class Capture:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.is_rtsp = (cfg["type"] == "rtsp")
        self.uri = cfg["path"]
        self.codec = cfg.get("codec", "h264")
        # Dosya modunda her kareyi işle (varsayılan); RTSP hep latest-frame
        self.consume = (not self.is_rtsp) and bool(
            cfg.get("process_every_frame", True))
        self.cap: Optional[cv2.VideoCapture] = None
        self.lock = threading.Lock()
        self.latest: Optional[Tuple[int, np.ndarray]] = None
        self.running = False
        self.done = False          # dosya doğal olarak bitti
        self.thread: Optional[threading.Thread] = None
        self.frame_id = 0

    # ------------------------------------------------------------------ open
    def _open(self) -> cv2.VideoCapture:
        if self.cfg.get("use_hw_decode", True):
            gst = build_gst_pipeline(self.uri,
                                     int(self.cfg.get("rtsp_latency_ms", 100)),
                                     self.is_rtsp, self.codec)
            cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                log.info("GStreamer HW decode açıldı")
                return cap
            log.warning("GStreamer HW pipeline açılamadı, "
                        "standart VideoCapture fallback (yavaş olabilir)")
        cap = cv2.VideoCapture(self.uri)
        return cap

    # ----------------------------------------------------------------- start
    def start(self) -> "Capture":
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True,
                                       name="capture")
        self.thread.start()
        return self

    # ------------------------------------------------------------------ loop
    def _loop(self) -> None:
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                self.cap = self._open()
                if not self.cap.isOpened():
                    log.error("Kaynak açılamadı: %s | %.1fs sonra tekrar",
                              self.uri, self.cfg["reconnect_delay_s"])
                    time.sleep(float(self.cfg["reconnect_delay_s"]))
                    continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                if self.is_rtsp:
                    log.warning("RTSP akışı kesildi, yeniden bağlanılıyor...")
                    self.cap.release()
                    self.cap = None
                    time.sleep(float(self.cfg["reconnect_delay_s"]))
                    continue
                log.info("Dosya sonu (%d kare)", self.frame_id)
                self.done = True
                self.running = False
                break

            if self.consume:
                # Her kare işlensin: önceki kare tüketilene kadar bekle
                while self.running:
                    with self.lock:
                        if self.latest is None:
                            break
                    time.sleep(0.001)
                if not self.running:
                    break

            with self.lock:
                self.latest = (self.frame_id, frame)
                self.frame_id += 1

    # ------------------------------------------------------------------ read
    def read(self) -> Optional[Tuple[int, np.ndarray]]:
        """Consumer tarafı: (frame_id, frame) ya da None.
        Dosya+her-kare modunda kareyi 'tüketir' (bir kez verilir)."""
        with self.lock:
            item = self.latest
            if item is not None and self.consume:
                self.latest = None
            return item

    # ------------------------------------------------------------------ stop
    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
