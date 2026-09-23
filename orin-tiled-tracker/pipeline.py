"""Ana orkestrasyon — thread'li pipeline.

    [Capture]──q_pre──▶[Batched Infer]──q_post──▶[Fuse+Track]──q_out──▶[Output]
      (+stab)  drop-old   (TensorRT)     drop-old  (WBF/NMS,     drop-old  callback
                                                    Kalman+1€)             imshow
                                                                           writer
- Her queue bounded (maxsize=2): RTSP'de gecikme BİRİKMEZ; kuyruk doluysa
  ESKİ kare atılır, en güncel kare işlenir (backpressure / frame-drop).
- Uçtan uca FPS ≈ 1 / (en yavaş aşama). PROFILE logu her aşamanın ms
  ortalamasını ve darboğazı yazar.
- Kullanım:  python3 pipeline.py            (config.yaml'ı okur)
             PIPE_CONFIG=baska.yaml python3 pipeline.py
- Ctrl+C ile temiz kapanış; display açıksa 'q' ile de çıkılır.
"""
import logging
import os
import queue
import threading
import time

import cv2
import numpy as np
import yaml

from capture import Capture
from detector import Detector
from fusion import fuse
from stabilizer import Stabilizer
from tiler import Tile, Tiler
from tracker import ByteTracker

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)-10s %(message)s")
log = logging.getLogger("pipeline")


# --------------------------------- Profiler ---------------------------------
class Profiler:
    def __init__(self, every: int = 60):
        self.t = {}
        self.n = 0
        self.every = max(1, int(every))

    def add(self, stage: str, ms: float) -> None:
        self.t.setdefault(stage, []).append(ms)

    def tick(self) -> None:
        self.n += 1
        if self.n % self.every:
            return
        means = {k: float(np.mean(v[-self.every:]))
                 for k, v in self.t.items() if v}
        if not means:
            return
        msg = " | ".join(f"{k}:{m:.1f}ms" for k, m in means.items())
        bott = max(means.values())
        log.info("PROFILE %s || pipeline-FPS~%.1f (darbogaz %.1f ms)",
                 msg, 1000.0 / max(1e-3, bott), bott)


# --------------------------------- Pipeline ----------------------------------
class Pipeline:
    def __init__(self, cfg: dict, frame_callback=None):
        self.cfg = cfg
        self.frame_callback = frame_callback

        self.cap = Capture(cfg["source"])
        tc = cfg["tiler"]
        self.tiler = Tiler(tc["grid_cols"], tc["grid_rows"],
                           tc["overlap_ratio"])
        self.global_pass = bool(tc.get("global_pass", False))
        self.detector = Detector(cfg["model"])
        self.names = self.detector.names
        self.tracker = ByteTracker(cfg["tracker"])
        s = cfg["stabilizer"]
        self.stab = (Stabilizer(s["max_corners"], s["smoothing_ema"],
                                s["crop_ratio"]) if s["enabled"] else None)

        self.q_pre: "queue.Queue" = queue.Queue(maxsize=2)
        self.q_post: "queue.Queue" = queue.Queue(maxsize=2)
        self.q_out: "queue.Queue" = queue.Queue(maxsize=2)
        self.realtime = (cfg["source"]["type"] == "rtsp") or not cfg["source"].get("process_every_frame", True)

        self.running = False
        p = cfg["profiling"]
        self.prof = Profiler(p["report_every"]) if p["enabled"] else None
        self.writer = None
        self._fps_ema = 0.0
        self._last_out_t = None

    # ------------------------------------------------------------- helpers
    def _safe_put(self, q: "queue.Queue", item) -> None:
        """Backpressure: kuyruk doluysa ESKİYİ at, yeniyi koy (latest-wins)."""
        try:
            q.put_nowait(item)
        except queue.Full:
            try:
                q.get_nowait()
            except queue.Empty:
                pass
            try:
                q.put_nowait(item)
            except queue.Full:
                pass

    def _put(self, q, item):
        """Dosya modunda BLOKLA (her kare islensin); RTSP'de en-guncel-kare."""
        if self.realtime:
            self._safe_put(q, item)
            return
        while self.running:
            try:
                q.put(item, timeout=0.2)
                return
            except queue.Full:
                continue

    # ------------------------------------------------------------- threads
    def _feed(self) -> None:
        last_id = -1
        while self.running:
            item = self.cap.read()
            if item is None:
                if self.cap.done:
                    # Dosya bitti: kuyruklar boşalınca kapan
                    if (self.q_pre.empty() and self.q_post.empty()
                            and self.q_out.empty()):
                        log.info("Kaynak bitti, pipeline kapanıyor")
                        self.running = False
                time.sleep(0.002)
                continue
            fid, frame = item
            if fid == last_id:          # RTSP modunda aynı kareyi tekrar işleme
                time.sleep(0.001)
                continue
            last_id = fid
            if self.stab is not None:
                t0 = time.time()
                frame = self.stab.apply(frame)
                if self.prof:
                    self.prof.add("stab", (time.time() - t0) * 1000)
            self._put(self.q_pre, (fid, frame))

    def _infer(self) -> None:
        while self.running:
            try:
                fid, frame = self.q_pre.get(timeout=0.5)
            except queue.Empty:
                continue
            t0 = time.time()
            h, w = frame.shape[:2]
            tiles = list(self.tiler.compute_tiles(w, h))
            if self.global_pass:
                tiles.append(Tile(0, 0, w, h))   # tam kare düşük çöz. ek geçiş
            try:
                dets = self.detector.infer_tiles(frame, tiles)
            except Exception:
                log.exception("Inference hatası (kare %d atlandı)", fid)
                continue
            if self.prof:
                self.prof.add("infer", (time.time() - t0) * 1000)
            self._put(self.q_post, (fid, frame, dets))

    def _track(self) -> None:
        fc = self.cfg["fusion"]
        while self.running:
            try:
                fid, frame, dets = self.q_post.get(timeout=0.5)
            except queue.Empty:
                continue
            t0 = time.time()
            fused = fuse(dets, fc["method"], fc["iou_thres"],
                         fc["class_agnostic"])
            tracks = self.tracker.update(fused, time.time())
            if self.prof:
                self.prof.add("track", (time.time() - t0) * 1000)
            self._put(self.q_out, (fid, frame, tracks))

    def _draw(self, frame: np.ndarray, tracks: np.ndarray) -> np.ndarray:
        vis = frame.copy()
        for tr in tracks:
            x1, y1, x2, y2 = map(int, tr[:4])
            tid, conf, cls = int(tr[4]), float(tr[5]), int(tr[6])
            name = self.names.get(cls, str(cls))
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis, f"#{tid} {name} {conf:.2f}", (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                        cv2.LINE_AA)
        # uçtan uca FPS (EMA) — output kareleri arası süreden
        now = time.time()
        if self._last_out_t is not None:
            inst = 1.0 / max(1e-3, now - self._last_out_t)
            self._fps_ema = (0.9 * self._fps_ema + 0.1 * inst
                             if self._fps_ema else inst)
        self._last_out_t = now
        h, w = vis.shape[:2]
        cv2.putText(vis, f"FPS:{self._fps_ema:5.1f}  obj:{len(tracks)}",
                    (w - 210, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)
        return vis

    def _output(self) -> None:
        oc = self.cfg["outputs"]
        need_vis = bool(oc["display"] or oc["file"]["enabled"])
        while self.running:
            try:
                fid, frame, tracks = self.q_out.get(timeout=0.5)
            except queue.Empty:
                continue
            t0 = time.time()

            # 1) ASIL ÇIKIŞ — kullanıcının RTSP restream koduna temiz arayüz:
            if oc["frame_callback"] and self.frame_callback is not None:
                try:
                    self.frame_callback(fid, frame, tracks)
                except Exception:
                    log.exception("frame_callback hatası")

            # 2) Opsiyonel görselleştirme çıkışları
            if need_vis:
                vis = self._draw(frame, tracks)
                if oc["file"]["enabled"]:
                    if self.writer is None:
                        h, w = frame.shape[:2]
                        self.writer = cv2.VideoWriter(
                            oc["file"]["path"],
                            cv2.VideoWriter_fourcc(*"mp4v"),
                            oc["file"]["fps"], (w, h))
                    self.writer.write(vis)
                if oc["display"]:
                    cv2.imshow("orin-tiled-tracker", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        log.info("'q' basıldı, kapanıyor")
                        self.running = False

            if self.prof:
                self.prof.add("output", (time.time() - t0) * 1000)
                self.prof.tick()

    # ----------------------------------------------------------------- run
    def run(self) -> None:
        self.running = True
        self.cap.start()
        threads = [threading.Thread(target=f, daemon=True, name=n)
                   for f, n in ((self._feed, "feed"),
                                (self._infer, "infer"),
                                (self._track, "track"),
                                (self._output, "output"))]
        for t in threads:
            t.start()
        log.info("Pipeline başladı (Ctrl+C ile çık)")
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            log.info("Ctrl+C alındı, kapanıyor")
            self.running = False
        finally:
            self._stop(threads)

    def _stop(self, threads) -> None:
        self.running = False
        for t in threads:
            t.join(timeout=2.0)
        self.cap.stop()
        if self.writer is not None:
            self.writer.release()
        cv2.destroyAllWindows()
        log.info("Temiz kapanış tamam")


# ------------------------------------ main -----------------------------------
def main() -> None:
    cfg_path = os.environ.get("PIPE_CONFIG", "config.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # =========================================================================
    #  KULLANICI ENTEGRASYON NOKTASI (RTSP restream)
    #  frame : BGR np.ndarray (H x W x 3)  — işlenmiş HAM kare (kutusuz)
    #  tracks: (N, 7) float32 [x1, y1, x2, y2, track_id, conf, cls]
    #          — SADECE onaylı (N-of-M: örn. 30 karede 20) yumuşatılmış kutular
    #  Kutuları kendiniz çizebilir ya da metadata olarak ayrı gönderebilirsiniz.
    #  Orin Nano'da NVENC YOK -> encode CPU'da:
    #    appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast
    #      bitrate=4000 ! h264parse ! rtspclientsink / rtph264pay+udpsink
    # =========================================================================
    def my_restream_callback(fid: int, frame: np.ndarray,
                             tracks: np.ndarray) -> None:
        pass  # <- kendi RTSP restream kodunuzu buraya bağlayın

    Pipeline(cfg, frame_callback=my_restream_callback).run()


if __name__ == "__main__":
    main()
