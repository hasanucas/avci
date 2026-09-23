#!/usr/bin/env bash
# =====================================================================
# build_ortrack_engine.sh — ORTrack ONNX -> TensorRT fp16 motoru
#
# NEDEN GEREKLI: TensorRT plani (.engine) TASINABILIR DEGILDIR. GPU
# mimarisine, TensorRT surumune ve surucuye baglidir. Baska bir makinede
# uretilmis motor bu Orin'de deserialize EDILEMEZ (runtracker "Motor
# deserialize EDILEMEDI" der). ONNX ise tasinabilir; motor ONDAN uretilir.
#
# ONNX zaten "explicit attention" ile disa aktarilmis durumda (bkz.
# models/ORTrack_ep0300-fp16.engine.json -> onnx.attention = "explicit").
# Bu onemli: TensorRT 10.3 bu omurgadaki SDPA kalibini yanlis derliyor
# (uretici notu: blok 0 ciktisi 3.4 sapiyor). Explicit graf o kalibi
# icermedigi icin trtexec ile yeniden uretim GUVENLIDIR.
#
# Kullanim:
#   ./build_ortrack_engine.sh                    # varsayilan yollar
#   ./build_ortrack_engine.sh /yol/models        # baska models klasoru
# =====================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="${1:-$(dirname "$SCRIPT_DIR")/models}"

ONNX="$MODELS_DIR/ORTrack_ep0300.onnx"
ENGINE="$MODELS_DIR/ORTrack_ep0300-fp16.engine"
SIDECAR="$ENGINE.json"

echo "=== ORTrack TensorRT motoru ==="
echo "models : $MODELS_DIR"

# --- trtexec bul ---
TRTEXEC="$(command -v trtexec || true)"
[ -z "$TRTEXEC" ] && [ -x /usr/src/tensorrt/bin/trtexec ] && TRTEXEC=/usr/src/tensorrt/bin/trtexec
if [ -z "$TRTEXEC" ]; then
    echo "!!! trtexec bulunamadi."
    echo "    Jetson'da genelde: /usr/src/tensorrt/bin/trtexec"
    echo "    Yoksa: sudo apt install tensorrt   (JetPack ile gelir)"
    exit 1
fi
echo "trtexec: $TRTEXEC"

if [ ! -f "$ONNX" ]; then
    echo "!!! ONNX yok: $ONNX"
    echo "    models.zip icindeki ORTrack_ep0300.onnx dosyasini oraya koyun."
    exit 1
fi

# --- ONNX butunlugu (sidecar varsa) ---
if [ -f "$SIDECAR" ] && command -v python3 >/dev/null; then
    WANT="$(python3 -c "import json;print(json.load(open('$SIDECAR'))['onnx']['sha256'])" 2>/dev/null || true)"
    if [ -n "$WANT" ]; then
        HAVE="$(sha256sum "$ONNX" | cut -d' ' -f1)"
        if [ "$WANT" = "$HAVE" ]; then
            echo "ONNX sha256 : DOGRULANDI"
        else
            echo "!!! UYARI: ONNX sha256 sidecar ile UYUSMUYOR."
            echo "    beklenen: $WANT"
            echo "    bulunan : $HAVE"
            echo "    Yanlis/bozuk ONNX ile motor uretmek sessiz takip hatasi demektir."
            read -r -p "    Yine de devam edilsin mi? [e/H] " a
            [ "${a:-H}" = "e" ] || exit 1
        fi
    fi
fi

# --- Saat/guc: olcum ve gecikme tutarliligi icin ---
if command -v nvpmodel >/dev/null; then
    echo
    echo "ONERI: olcum ve ucus oncesi"
    echo "  sudo nvpmodel -m 2 && sudo jetson_clocks"
    echo
fi

# --- Eskisini sakla ---
if [ -f "$ENGINE" ]; then
    BAK="$ENGINE.$(date +%Y%m%d_%H%M%S).bak"
    mv "$ENGINE" "$BAK"
    echo "eski motor -> $BAK"
fi

echo "--- build basliyor (Orin'de ~90-150 sn surer) ---"
"$TRTEXEC" \
    --onnx="$ONNX" \
    --saveEngine="$ENGINE" \
    --fp16 \
    --memPoolSize=workspace:1024M \
    --warmUp=500 \
    --duration=5 \
    --noDataTransfers \
    --useCudaGraph=false 2>&1 | tee "$MODELS_DIR/trtexec-build.log" | \
    grep -E "GPU Compute Time|Throughput|Latency|Engine built|error|Error|FAILED" || true

if [ ! -f "$ENGINE" ]; then
    echo "!!! Motor URETILEMEDI. Tam log: $MODELS_DIR/trtexec-build.log"
    exit 1
fi

echo
echo "=== HAZIR ==="
echo "motor : $ENGINE"
echo "sha256: $(sha256sum "$ENGINE" | cut -d' ' -f1)"
echo "boyut : $(du -h "$ENGINE" | cut -f1)"
echo
echo "Beklenen GPU Compute Time (Orin, fp16): ~7 ms/kare."
echo "Belirgin daha yuksekse once 'sudo jetson_clocks' calistirin."
echo
echo "Test:"
echo "  cd kcf/build && ./kcfTracker --ortrack --ortrack-engine $ENGINE --help"
