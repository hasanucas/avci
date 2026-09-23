# Orin Nano Tiled Detection + Tracking — Kurulum ve Kullanım

Model-bağımsız pipeline: özel tile bölme (SAHI değil) → tek batched TensorRT
inference → sınıf-bilinçli WBF/NMS füzyon → kendi ByteTrack-tarzı tracker
(Kalman + 20/30 N-of-M onay + One-Euro yumuşatma) → callback / imshow / dosya.
Hedef: 1080p girişte 25 FPS (alt limit 20).

## Dosyalar
| Dosya | Görev |
|---|---|
| `config.yaml` | Tüm ayarlar (kaynak, model, tile, tracker, çıkışlar) |
| `pipeline.py` | Ana program + thread orkestrasyonu + restream callback noktası |
| `capture.py` | Dosya/RTSP okuma, HW decode, auto-reconnect |
| `tiler.py` | Özel grid+overlap tile üretimi |
| `detector.py` | Batched TensorRT/PT inference, global koordinat projeksiyonu |
| `fusion.py` | Tile'lar arası WBF/NMS birleştirme |
| `tracker.py` | Kendi tracker'ımız (titreme yok, N-of-M onay) |
| `stabilizer.py` | Opsiyonel gerçek zamanlı stabilizasyon |
| `build_engine.sh` | TensorRT engine üretimi + ham hız ölçümü |

## Kurulum (Orin Nano, JetPack 6.2.x)
```bash
# 1) Güç modu (her boot sonrası ya da rc.local'e ekleyin)
sudo nvpmodel -m 0 && sudo jetson_clocks

# 2) PyTorch: NVIDIA'nın JetPack-6 wheel'i (pip'ten DEĞİL)
#    https://developer.nvidia.com/embedded/downloads -> PyTorch for Jetson
# 3) Diğer paketler
pip3 install -r requirements.txt

# 4) Kontroller
python3 -c "import tensorrt; print('TRT', tensorrt.__version__)"   # 10.3.x
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gstreamer  # YES olmalı
python3 -c "import torch; print('CUDA', torch.cuda.is_available())" # True olmalı

# 5) RTSP restream (sizin taraf) için CPU encoder:
sudo apt install gstreamer1.0-plugins-ugly   # x264enc
```

## Engine build ve ham hız ölçümü
```bash
cp /yol/UAV-YOLOv11m.pt .
bash build_engine.sh UAV-YOLOv11m.pt
```
Betik sonda `trtexec --shapes=images:4x3x640x640` ile **saf GPU süresini** ölçer:
- mean **≤ ~35 ms** → Plan A hedefi tutar, devam.
- mean **> 35 ms** → aşağıdaki Plan B/C'ye geç.

## Çalıştırma
```bash
# config.yaml'da source.path ve engine_path'i düzenleyin
python3 pipeline.py
# farklı config: PIPE_CONFIG=fabrika.yaml python3 pipeline.py
```
Her 60 karede log: `PROFILE stab:.. | infer:.. | track:.. | output:.. || pipeline-FPS~..`
→ Uçtan uca FPS ≈ 1000 / (en büyük aşama ms'i). `q` veya Ctrl+C ile çıkış.

## RTSP restream entegrasyonu (ASIL çıkış)
`pipeline.py` sonundaki `my_restream_callback(fid, frame, tracks)` içine kendi
kodunuzu yazın. `frame` = kutusuz BGR kare, `tracks` = (N,7)
`[x1,y1,x2,y2,id,conf,cls]` onaylı+yumuşatılmış kutular. Orin Nano'da **NVENC
yok** → encode CPU'da:
```
appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast \
  bitrate=4000 key-int-max=30 ! h264parse ! rtspclientsink location=rtsp://...
```
x264 1080p30 ~1-2 çekirdek yer; **restream açıkken** FPS'i tekrar ölçün
(encode + inference aynı SoC'de yarışır).

## 25 FPS ayar kademeleri (Plan A/B/C)
| Plan | Model | Precision | Tile | Beklenen | Ne zaman |
|---|---|---|---|---|---|
| **A (başlangıç)** | YOLO11m | FP16 | 2×2 (4) | ~25-27 FPS (sınırda) | önce bunu ölç |
| **B (hız)** | YOLO11m | INT8 | 2×2 (4) | ~30 FPS | A < 25 FPS ise |
| **C (garanti)** | YOLO11s* | FP16 | 2×2 (4) | ~30-35 FPS | INT8 sorunlu/yetersizse |
| **C+ (recall)** | YOLO11s | INT8 | 3×2 (6) | ~28-30 FPS | max küçük-nesne |

\* Kendi verinizle YOLO11s fine-tune → `build_engine.sh yeni_model.pt`.
6 tile için: `grid_cols: 3, grid_rows: 2` (engine zaten batch=6 destekli).

## İnce ayar hızlı rehber
| Belirti | Ayar |
|---|---|
| FPS < 20 | Plan B/C; `global_pass: false`; stabilizasyonu kapat |
| Kutu hâlâ titriyor | `smoothing.min_cutoff: 0.5` (↓); yetmezse `tracker.py` içinde `KalmanBox.R *= 2` |
| Kutu hedefin gerisinde kalıyor (lag) | `smoothing.beta: 0.1-0.2` (↑) |
| Yanıp sönen sahte kutular | `confirm_min_hits: 25` (25/30) veya `conf_thres` ↑ |
| Gerçek nesne geç görünüyor | `confirm_min_hits: 15` (15/30) |
| ID sık değişiyor | `max_age: 45-60` ↑; `match_iou_thresh: 0.15` ↓ |
| Tile sınırında çift kutu | `fusion.iou_thres: 0.4` ↓; `overlap_ratio: 0.25` ↑ |
| Büyük (yakın) nesne kaçıyor | `global_pass: true` (infer'e ~+%20 ms ekler) |
| Kamera sarsıntılı | `stabilizer.enabled: true` → PROFILE'da `stab` ms'ine bak (~8-15 ms bekle) |

## Yeni model (ekmek / araba / …)
1. Yeni `.pt` → `bash build_engine.sh ekmek.pt`
2. `config.yaml`: `engine_path`, `pt_path`, `target_classes` (boş = hepsi)
3. Kodda sıfır değişiklik — drone'a özel hiçbir varsayım yok. Sayım için
   callback'te `len(tracks)` anlık sayı, `set(track_id)` toplam benzersiz sayıdır.

## Sorun giderme
- **GStreamer pipeline açılmıyor** → log'da fallback uyarısı görür, yine çalışır
  (yavaş decode). Kalıcı çözüm: JetPack'in kendi OpenCV'sini kullanın; pip
  `opencv-python` kurduysanız kaldırın.
- **INT8 export hatası** (TRT 10.3 bilinen sorunları) → FP16'da kalın (Plan C).
- **`numpy` / `torch` import hatası** → `pip3 install "numpy<2"`.
- **RAM > 7 GB (jtop)** → tile 4'e düşürün, `display: false`, tarayıcı vb. kapatın.
- **PC'de ön test** (Windows/Linux, Orin'siz): `use_hw_decode: false`,
  engine yerine `.pt` fallback ile mantık test edilir (yavaş ama çalışır).
- **İlk 1-2 sn düşük FPS** → normal (warm-up); PROFILE ortalamasına bakın.
