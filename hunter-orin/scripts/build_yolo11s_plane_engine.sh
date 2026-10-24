#!/usr/bin/env bash
# =====================================================================
# build_yolo11s_plane_engine.sh — SABIT KANAT dedektoru (21 Ekim 2026)
#
#   yolo11s-plane.pt  ->  ONNX 640x768 (fp32)  ->  TensorRT fp16 motoru
#
# Model: yolo11s-sabitkanat-ortrack paketindeki models/yolo11s-plane.pt
#   (upstream berkeduruu/yolo-aircraft-detection S_new720p.pt, GPL-3.0)
#   YOLO11s, TEK SINIF {0: plane}. Cikti (1, 5, 10080).
#   sha256 d2c69ae5d312297406efec94ea9eb994c494bf6b010ee9fd17a460737371b3ee
#
# Paketin hazir .engine dosyasi BASKA bir Orin'de uretildi; TensorRT plani
# cihaza ozeldir. Bu script motoru BU Orin'de, eski UAV motoru ile ayni
# yoldan (trtexec, --fp16, giris/cikis FP32) uretir. runtracker bu motoru
# --yolo-engine ile alir; sinif sayisini motordan okur (21 Ekim degisikligi).
#
# Kullanim (hunter-orin klasorunden):
#   ./scripts/build_yolo11s_plane_engine.sh                 # .pt'yi kendisi arar
#   ./scripts/build_yolo11s_plane_engine.sh /yol/yolo11s-plane.pt
#
# .pt arama sirasi: arguman, models/, ~/Downloads/, ~/Downloads/yolo11s-sabitkanat-ortrack/models/,
# bu scriptin klasoru. Hicbiri yoksa upstream GitHub'dan (sabit commit) indirir;
# her durumda sha256 DOGRULANIR, tutmazsa durur.
#
# Sure: ONNX ~1 dk (CPU), motor ~5-12 dk. Bu sirada runtracker/kcfTracker KAPALI olsun.
# =====================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUNTER_DIR="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$HUNTER_DIR/models"

NAME="yolo11s-plane"
PT_SHA="d2c69ae5d312297406efec94ea9eb994c494bf6b010ee9fd17a460737371b3ee"
PT_URL="https://raw.githubusercontent.com/berkeduruu/yolo-aircraft-detection/85217c512a9abffe67f0c631500bc6aadd7c63f5/models/S_new720p.pt"
IN_H=640
IN_W=768
ONNX="$MODELS_DIR/$NAME-${IN_H}x${IN_W}.onnx"
ENGINE="$MODELS_DIR/$NAME-${IN_H}x${IN_W}.engine"
LOG="$MODELS_DIR/trtexec-$NAME.log"

echo "=== YOLO11s sabit kanat (plane) TensorRT motoru ==="
echo "models : $MODELS_DIR"
mkdir -p "$MODELS_DIR"

if pgrep -x kcfTracker >/dev/null 2>&1; then
    echo "!!! kcfTracker calisiyor. Motor uretimi GPU'yu paylasmamali; once onu kapatin."
    exit 1
fi

# --- trtexec ---
TRTEXEC="$(command -v trtexec || true)"
[ -z "$TRTEXEC" ] && [ -x /usr/src/tensorrt/bin/trtexec ] && TRTEXEC=/usr/src/tensorrt/bin/trtexec
if [ -z "$TRTEXEC" ]; then
    echo "!!! trtexec bulunamadi (Jetson'da genelde /usr/src/tensorrt/bin/trtexec)."
    exit 1
fi
echo "trtexec: $TRTEXEC"

# --- Python ortami (ultralytics + torch + onnx) ---
if ! python3 -c "import torch, ultralytics, onnx" 2>/dev/null; then
    echo "!!! python3'te torch / ultralytics / onnx eksik."
    python3 -c "import torch" 2>/dev/null       || echo "    torch yok (Jetson tekerlegi: setup/tiled_ortam.sh)"
    python3 -c "import ultralytics" 2>/dev/null || echo "    ultralytics yok:  pip3 install ultralytics==8.4.157"
    python3 -c "import onnx" 2>/dev/null        || echo "    onnx yok:         pip3 install onnx"
    exit 1
fi
python3 -c "import torch, ultralytics, onnx; print('python : torch', torch.__version__, '| ultralytics', ultralytics.__version__, '| onnx', onnx.__version__)"

# --- .pt bul ---
PT="${1:-}"
if [ -z "$PT" ]; then
    for c in "$MODELS_DIR/$NAME.pt" \
             "$HOME/Downloads/$NAME.pt" \
             "$HOME/Downloads/yolo11s-sabitkanat-ortrack/models/$NAME.pt" \
             "$SCRIPT_DIR/$NAME.pt"; do
        if [ -f "$c" ]; then PT="$c"; break; fi
    done
fi
if [ -z "$PT" ]; then
    echo ".pt yerelde yok; upstream'den indiriliyor (sabit commit, sha256 dogrulanacak)..."
    TMP_PT="$MODELS_DIR/$NAME.pt.indiriliyor"
    if ! curl -fL --retry 2 -o "$TMP_PT" "$PT_URL"; then
        rm -f "$TMP_PT"
        echo "!!! Indirilemedi. Paketteki models/$NAME.pt dosyasini $MODELS_DIR/ icine koyup tekrar calistirin."
        exit 1
    fi
    mv "$TMP_PT" "$MODELS_DIR/$NAME.pt"
    PT="$MODELS_DIR/$NAME.pt"
fi
[ -f "$PT" ] || { echo "!!! .pt bulunamadi: $PT"; exit 1; }

HAVE="$(sha256sum "$PT" | cut -d' ' -f1)"
if [ "$HAVE" != "$PT_SHA" ]; then
    echo "!!! $PT sha256 TUTMUYOR."
    echo "    beklenen: $PT_SHA"
    echo "    bulunan : $HAVE"
    echo "    Yanlis/bozuk model ile motor uretilmez."
    exit 1
fi
echo "pt     : $PT  (sha256 DOGRULANDI)"
if [ "$(readlink -f "$PT")" != "$(readlink -f "$MODELS_DIR/$NAME.pt")" ]; then
    cp -f "$PT" "$MODELS_DIR/$NAME.pt"
    echo "         -> $MODELS_DIR/$NAME.pt kopyalandi"
fi

# --- Eskileri sakla (uzerine yazma yok) ---
STAMP="$(date +%Y%m%d_%H%M%S)"
for f in "$ONNX" "$ENGINE"; do
    if [ -f "$f" ]; then mv "$f" "$f.$STAMP.bak"; echo "eski -> $f.$STAMP.bak"; fi
done

# --- 1) ONNX: statik 1x3x640x768, fp32, opset 17, simplify yok, NMS yok ---
echo
echo "--- 1/2 ONNX export (CPU, ~1 dk) ---"
YOLO_AUTOINSTALL=false python3 - "$MODELS_DIR/$NAME.pt" "$ONNX" "$IN_H" "$IN_W" <<'PY'
import os, shutil, sys, tempfile
pt, out, h, w = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
from ultralytics import YOLO
with tempfile.TemporaryDirectory(prefix="yolo11s-onnx-") as d:
    src = shutil.copy(pt, os.path.join(d, os.path.basename(pt)))   # .pt'nin yanina yazmasin
    model = YOLO(src, task="detect")
    names = dict(model.model.names)
    print("siniflar:", names)
    if len(names) != 1:
        sys.exit(f"!!! Tek sinif bekleniyordu, model {len(names)} sinif bildiriyor: {names}")
    # nms verilmez (varsayilan None = ham cikti, NMS runtracker'da). 8.4'te nms=False
    # "NMS'siz kafa varsa onu kullan" demek; YOLO11'de yok ama bilerek dokunmuyoruz.
    f = model.export(format="onnx", imgsz=[h, w], opset=17, simplify=False, dynamic=False,
                     batch=1, device="cpu", verbose=False)   # fp32 (varsayilan)
    shutil.move(str(f), out)

import onnx
m = onnx.load(out)
onnx.checker.check_model(m)
dims = lambda v: [d.dim_value for d in v.type.tensor_type.shape.dim]
ins = [(i.name, dims(i)) for i in m.graph.input]
outs = [(o.name, dims(o)) for o in m.graph.output]
n = (h // 8) * (w // 8) + (h // 16) * (w // 16) + (h // 32) * (w // 32)
print("giris :", ins)
print("cikis :", outs)
if ins != [("images", [1, 3, h, w])] or outs != [("output0", [1, 5, n])]:
    sys.exit(f"!!! ONNX bicimi beklenmedik: giris {ins}, cikis {outs} "
             f"(beklenen images [1,3,{h},{w}] -> output0 [1,5,{n}])")
print("ONNX DOGRULANDI")
PY
echo "onnx   : $ONNX"
echo "sha256 : $(sha256sum "$ONNX" | cut -d' ' -f1)"

# --- 2) TensorRT: fp16, G/C FP32 (runtracker FP32 G/C ister) ---
echo
echo "--- 2/2 TensorRT build (~5-12 dk; uzun sessizlik normal) ---"
if command -v nvpmodel >/dev/null; then
    echo "ONERI: build ve olcum icin  sudo nvpmodel -m 2 && sudo jetson_clocks"
fi
"$TRTEXEC" \
    --onnx="$ONNX" \
    --saveEngine="$ENGINE" \
    --fp16 \
    --memPoolSize=workspace:1024M \
    --warmUp=500 \
    --duration=5 \
    --noDataTransfers \
    --useCudaGraph=false 2>&1 | tee "$LOG" | \
    grep -E "GPU Compute Time|Throughput|Engine built|error|Error|FAILED" || true

if [ ! -s "$ENGINE" ]; then
    echo "!!! Motor URETILEMEDI. Tam log: $LOG"
    exit 1
fi

echo
echo "=== HAZIR ==="
echo "motor : $ENGINE"
echo "sha256: $(sha256sum "$ENGINE" | cut -d' ' -f1)"
echo "boyut : $(du -h "$ENGINE" | cut -f1)"
echo "log   : $LOG   (GPU Compute Time satirini not alin)"
echo
echo "runtracker acilisinda beklenen satir:"
echo "  [YOLO] giris 640x768 (HxW)  sinif 1  hedef 0 (#0)  izgara 10080"
echo
echo "Ilk deneme (yerde, sadece tespit; beyin/run_brain.sh ACMAYIN):"
echo "  ./scripts/run_tracker.sh --width 1280 --height 720 --detect-only \\"
echo "      --yolo-engine ../../models/$NAME-${IN_H}x${IN_W}.engine --yolo-conf 0.25"
