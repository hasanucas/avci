#!/bin/bash
# =============================================================================
#  TensorRT engine üretimi — ORIN NANO ÜZERİNDE ÇALIŞTIRIN!
#  Engine cihaza + JetPack/TensorRT sürümüne özgüdür; PC'de build edilen
#  engine Orin'de ÇALIŞMAZ. Build süresi 10-25 dk sürebilir, normaldir.
# =============================================================================
set -e

MODEL_PT="${1:-UAV-YOLOv11m.pt}"
IMGSZ=640
MAX_BATCH=6          # 6 tile'a kadar dinamik batch (2x2=4 ve 3x2=6 destekler)

echo "== Güç modu: MAXN + clock kilidi =="
sudo nvpmodel -m 2 || true
sudo jetson_clocks || true

echo "== FP16 engine build: ${MODEL_PT} -> .engine =="
yolo export model="${MODEL_PT}" format=engine half=True \
     dynamic=True batch=${MAX_BATCH} imgsz=${IMGSZ} device=0

# -----------------------------------------------------------------------------
# INT8 (opsiyonel, ~%25-40 ek hız / ~1-3 mAP kayıp riski):
# calib.yaml = hedef sınıflarınızın örneklerini içeren küçük bir dataset yaml'ı
# (Ultralytics data formatı). FP16 yetiyorsa INT8'e gerek yok.
# Not: JetPack 6 / TRT 10.3'te bazı INT8 build'lerde sorun bildirildi;
# başarısız olursa FP16'da kalın.
# -----------------------------------------------------------------------------
# yolo export model="${MODEL_PT}" format=engine int8=True \
#      dynamic=True batch=${MAX_BATCH} imgsz=${IMGSZ} device=0 data=calib.yaml

ENGINE="${MODEL_PT%.pt}.engine"
echo "== Ham inference ölçümü (batch=4, pipeline'sız saf GPU süresi) =="
/usr/src/tensorrt/bin/trtexec --loadEngine="${ENGINE}" \
    --shapes=images:4x3x${IMGSZ}x${IMGSZ} --fp16 --avgRuns=100 || \
    echo "trtexec bulunamadı; ölçümü PROFILE logundan yapın."

echo ""
echo "KARAR: 'GPU Compute Time mean' > 35 ms ise 25 FPS için Plan B/C gerekli"
echo "       (README_KURULUM.md -> Plan A/B/C tablosu)."
echo "Bitti: ${ENGINE}  -> config.yaml'da engine_path'e yazın."
