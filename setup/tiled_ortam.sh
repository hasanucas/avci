#!/usr/bin/env bash
# =============================================================================
# tiled_ortam.sh — orin-tiled-tracker Python/TensorRT ortami (JetPack 6.2, Py 3.10)
# =============================================================================
# Kanitlanmis tarif (21-22 Eylul 2026, Orin Nano 8GB, L4T R36.4.7):
#   1 apt   2 cuSPARSELt (NVIDIA local repo)   3 torch + torchvision (Jetson wheel)
#   4 ultralytics + scipy + pyyaml   5 onnx + onnxslim   6 onnxruntime-gpu (wheel)
#   7 pip opencv SIL (GStreamer'li sistem cv2 golgelenmesin)   8 numpy<2 + PATH
#
# Tek basina da calisir (or. ekmek fabrikasi Orin'i):   ./setup/tiled_ortam.sh
# Tekrar calistirmak guvenli: kurulu olan adimi atlar.
# Surumler setup/referans_ortam/pip_user.txt'den (yoksa asagidaki varsayilanlar).
# =============================================================================
set -euo pipefail

SETUP_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
PINS="$SETUP_DIR/referans_ortam/pip_user.txt"
W="https://github.com/ultralytics/assets/releases/download/v0.0.0"
TORCH_WHL="$W/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl"
TV_WHL="$W/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl"
ORT_WHL="$W/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl"
CSL="cusparselt-local-tegra-repo-ubuntu2204-0.7.1"
CSL_URL="https://developer.download.nvidia.com/compute/cusparselt/0.7.1/local_installers/${CSL}_1.0-1_arm64.deb"

bas() { echo -e "\n\033[1;36m== $* ==\033[0m"; }
ok()  { echo -e "  \033[32m✓\033[0m $*"; }
bi()  { echo -e "  • $*"; }

# pin <paket> <varsayilan> : referans kayitta varsa "paket==surum", yoksa varsayilan
pin() {
  local v=""
  if [[ -f "$PINS" ]]; then
    v=$(awk -F'==' -v p="$1" 'tolower($1)==tolower(p){print $2; exit}' "$PINS")
  fi
  if [[ -n "$v" ]]; then echo "$1==$v"; else echo "$2"; fi
}
PIP() { python3 -m pip install --user --no-warn-script-location "$@"; }
py()  { python3 -c "$1" >/dev/null 2>&1; }    # sessiz kontrol, 0 = tamam

# numpy 2'ye ziplamasin: bu script'teki TUM pip kurulumlarina kisit
KISIT=$(mktemp /tmp/avci_pip_kisit.XXXXXX)
echo "numpy<2" > "$KISIT"
export PIP_CONSTRAINT="$KISIT"
trap 'rm -f "$KISIT"' EXIT

[[ "$(uname -m)" == aarch64 ]] || { echo "Bu script Jetson (aarch64) icin."; exit 1; }
py 'import sys; sys.exit(sys.version_info[:2] != (3, 10))' || { echo "Python 3.10 gerekli (wheel'ler cp310)."; exit 1; }
if [[ -f "$PINS" ]]; then bi "surumler: setup/referans_ortam/pip_user.txt"
else bi "referans kayit yok -> varsayilan surumler"; fi

# -----------------------------------------------------------------------------
bas "1/8 apt paketleri"
sudo apt-get update -q || bi "apt update hata verdi — devam"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q \
     python3-pip libopenblas-dev gstreamer1.0-plugins-ugly gstreamer1.0-tools
ok "apt tamam"

# -----------------------------------------------------------------------------
bas "2/8 cuSPARSELt (torch 2.5 Jetson wheel'i ister)"
if /sbin/ldconfig -p | grep libcusparseLt >/dev/null; then
  ok "zaten kurulu"
else
  if ! sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q libcusparselt0 libcusparselt-dev 2>/dev/null; then
    bi "apt'te yok -> NVIDIA local repo .deb indiriliyor"
    tmp=$(mktemp -d)
    wget -q --show-progress -O "$tmp/$CSL.deb" "$CSL_URL"
    sudo dpkg -i "$tmp/$CSL.deb"
    sudo cp "/var/$CSL"/cusparselt-*-keyring.gpg /usr/share/keyrings/
    sudo apt-get update -q
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q libcusparselt0 libcusparselt-dev
    rm -rf "$tmp"
  fi
  ok "cuSPARSELt kuruldu"
fi

# -----------------------------------------------------------------------------
bas "3/8 torch + torchvision (CUDA'li Jetson wheel)"
if py 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)'; then
  ok "CUDA'li torch zaten var — atlandi"
else
  bi "indiriliyor (~250 MB, birkac dk; ilerleme gorunmeyebilir)"
  PIP "$TORCH_WHL" "$TV_WHL"
fi
py 'import torchvision' || PIP "$TV_WHL"
python3 -c 'import torch,sys; print("  torch", torch.__version__, "| CUDA:", torch.cuda.is_available()); sys.exit(0 if torch.cuda.is_available() else 1)' \
  || { echo "  ✗ torch CUDA'yi goremiyor — burada dur, ciktiyi kaydet"; exit 1; }

# -----------------------------------------------------------------------------
bas "4/8 ultralytics + scipy + pyyaml"
PIP "$(pin ultralytics 'ultralytics==8.4.157')" "$(pin scipy scipy)" pyyaml
ok "tamam"

# -----------------------------------------------------------------------------
bas "5/8 onnx + onnxslim (engine export sirasinda pip calismasin diye once kurulur)"
PIP "$(pin onnx 'onnx>=1.12,<2')" "$(pin onnxslim 'onnxslim>=0.1.82')"
ok "tamam"

# -----------------------------------------------------------------------------
bas "6/8 onnxruntime-gpu (Jetson wheel; PyPI'da aarch64 GPU surumu yok)"
if py 'import onnxruntime as o,sys; sys.exit(0 if "CUDAExecutionProvider" in o.get_available_providers() else 1)'; then
  ok "zaten kurulu"
else
  python3 -m pip uninstall -y onnxruntime >/dev/null 2>&1 || true   # CPU surumu varsa cakisir
  PIP "$ORT_WHL"
  ok "kuruldu"
fi

# -----------------------------------------------------------------------------
bas "7/8 pip opencv temizligi (sistem cv2 = GStreamer'li)"
python3 -m pip uninstall -y opencv-python opencv-python-headless \
        opencv-contrib-python opencv-contrib-python-headless 2>&1 \
  | grep -i 'successfully uninstalled' | sed 's/^/  /' || true
ok "pip opencv yok"

# -----------------------------------------------------------------------------
bas "8/8 numpy<2 + PATH"
want=$(pin numpy 'numpy<2')
cur=$(python3 -c 'import numpy;print(numpy.__version__)' 2>/dev/null || echo yok)
if [[ "$cur" == 1.* && ( "$want" != numpy==* || "$want" == "numpy==$cur" ) ]]; then
  ok "numpy $cur"
else
  PIP --force-reinstall --no-deps "$want"
  ok "numpy $cur -> $(python3 -c 'import numpy;print(numpy.__version__)')"
fi
if grep -Eq '(HOME\}?|~)/\.local/bin' "$HOME/.bashrc" 2>/dev/null; then
  ok "$HOME/.local/bin PATH'te (.bashrc)"
else
  # shellcheck disable=SC2016  # $HOME .bashrc'de acilsin diye tek tirnak
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
  ok "$HOME/.local/bin PATH'e eklendi (.bashrc) — yeni terminalde gecerli (yolo komutu)"
fi
export PATH="$HOME/.local/bin:$PATH"

# -----------------------------------------------------------------------------
bas "Dogrulama"
if python3 - <<'PY'
import sys
bad = 0
def dene(ad, fn):
    global bad
    try:
        tamam, bilgi = fn()
    except Exception as e:
        tamam, bilgi = False, f"{type(e).__name__}: {str(e)[:100]}"
    print(("  \033[32m✓\033[0m " if tamam else "  \033[31m✗\033[0m ") + f"{ad:12s} {bilgi}")
    bad += not tamam
def f_cv2():   # cv2 torch'tan ONCE (Jetson'da libgomp TLS sorunu olmasin)
    import cv2
    g = [l.strip() for l in cv2.getBuildInformation().splitlines() if "GStreamer" in l]
    gs = bool(g) and "YES" in g[0]
    pip = ".local" in (cv2.__file__ or "")
    return gs and not pip, f"{cv2.__version__} {'GStreamer YES' if gs else 'GStreamer YOK'}{'  (pip opencv golgeliyor!)' if pip else ''}"
def f_numpy():
    import numpy; return numpy.__version__.startswith("1."), numpy.__version__
def f_torch():
    import torch; c = torch.cuda.is_available(); return c, f"{torch.__version__} CUDA={c}"
def f_tv():
    import torchvision; return True, torchvision.__version__
def f_trt():
    import tensorrt; return True, tensorrt.__version__
def f_onnx():
    import onnx; return True, onnx.__version__
def f_ort():
    import onnxruntime as o; p = o.get_available_providers()
    return "CUDAExecutionProvider" in p, f"{o.__version__} ({', '.join(p)})"
def f_ultra():
    import ultralytics; return True, ultralytics.__version__
for ad, fn in [("cv2", f_cv2), ("numpy", f_numpy), ("torch", f_torch), ("torchvision", f_tv),
               ("tensorrt", f_trt), ("onnx", f_onnx), ("onnxruntime", f_ort), ("ultralytics", f_ultra)]:
    dene(ad, fn)
sys.exit(1 if bad else 0)
PY
then
  echo -e "\n  \033[32mtiled ortam hazir.\033[0m  Engine yoksa: cd orin-tiled-tracker && bash build_engine.sh UAV-YOLOv11m.pt"
else
  echo -e "\n  \033[31m✗ olan satir var — ciktiyi kaydet, birlikte bakalim.\033[0m"
  exit 1
fi
