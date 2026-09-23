#!/usr/bin/env bash
# =============================================================================
# kontrol.sh — Orin saglik kontrolu. SADECE OKUR, hicbir seyi degistirmez.
# =============================================================================
#   ./setup/kontrol.sh      her iki Orin'de de guvenli (ucus oncesi dahil)
#   ✓ tamam    ! dikkat (calismaya engel degil)    ✗ duzelt (cikis kodu 1)
# =============================================================================
SETUP_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(dirname "$SETUP_DIR")"
REF="$SETUP_DIR/referans_ortam/ortam.txt"
KUL="${USER:-$(id -un)}"

IYI=0; DIKKAT=0; KOTU=0
bas() { echo -e "\n\033[1;36m== $* ==\033[0m"; }
ok()  { echo -e "  \033[32m✓\033[0m $*"; IYI=$((IYI+1)); }
dk()  { echo -e "  \033[33m!\033[0m $*"; DIKKAT=$((DIKKAT+1)); }
ko()  { echo -e "  \033[31m✗\033[0m $*"; KOTU=$((KOTU+1)); }
bi()  { echo -e "  • $*"; }

l4t() { sed -nE '1s/^# R([0-9]+) \(release\), REVISION: ([0-9.]+).*/R\1.\2/p' /etc/nv_tegra_release 2>/dev/null; }
guc_modu() {
  local ad id
  ad=$( { nvpmodel -q 2>/dev/null || sudo -n nvpmodel -q 2>/dev/null; } | sed -nE 's/^NV Power Mode: *//p' | head -1)
  if [[ -z "$ad" ]]; then
    id=$(sed -nE 's/^pmode:0*([0-9]+).*/\1/p' /var/lib/nvpmodel/status 2>/dev/null)
    [[ -n "$id" ]] && ad=$(sed -nE "s/.*POWER_MODEL[[:space:]]+ID=${id}[[:space:]]+NAME=([A-Za-z0-9_]+).*/\1/p" \
                          /etc/nvpmodel.conf 2>/dev/null | head -1)
  fi
  echo "${ad:-?}"
}
maxn_id() {
  sed -nE 's/.*POWER_MODEL[[:space:]]+ID=([0-9]+)[[:space:]]+NAME=MAXN(_SUPER)?[[:space:]]*>.*/\1/p' \
      /etc/nvpmodel.conf 2>/dev/null | tail -1
}

H=""
for d in "$REPO/hunter-orin" "$REPO/hunter_orin"; do
  if [[ -d "$d" ]]; then H="$d"; break; fi
done
T="$REPO/orin-tiled-tracker"
echo "AVCI kontrol — $(hostname) — $(date '+%F %T')"

# -----------------------------------------------------------------------------
bas "Sistem"
cur=$(l4t); ref=$(sed -n 's/^L4T=//p' "$REF" 2>/dev/null)
if [[ -z "$cur" ]]; then ko "Jetson degil / L4T okunamadi"
elif [[ -z "$ref" ]]; then ok "L4T $cur"
elif [[ "$cur" == "$ref" ]]; then ok "L4T $cur (referansla ayni)"
else dk "L4T $cur — referans $ref (surumler farkli olabilir)"; fi

m=$(guc_modu)
case "$m" in
  MAXN*) ok "guc modu $m" ;;
  "?")   dk "guc modu okunamadi: sudo nvpmodel -q" ;;
  *)     ko "guc modu $m (MAXN degil) -> sudo nvpmodel -m $(maxn_id)   (bu unitede 0 = 15W!)" ;;
esac

mn=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 2>/dev/null)
mx=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null)
if [[ -n "$mn" && "$mn" == "$mx" ]]; then ok "jetson_clocks aktif (CPU $((mx/1000)) MHz sabit)"
else dk "jetson_clocks bu acilista calismamis: sudo jetson_clocks"; fi

if swapon --show=NAME --noheadings 2>/dev/null | grep -qx /swapfile; then ok "swap: /swapfile acik"
elif [[ -f /swapfile ]]; then dk "/swapfile kapali: sudo swapon /swapfile"
else dk "/swapfile yok (engine build icin gerekli)"; fi

if id -nG | grep -qw dialout; then ok "dialout grubu (FC seri port)"
elif id -nG "$KUL" 2>/dev/null | grep -qw dialout; then dk "dialout eklendi ama oturuma yansimamis: reboot"
else ko "dialout grubunda degil: sudo usermod -aG dialout $KUL  (sonra reboot)"; fi

# -----------------------------------------------------------------------------
bas "Ag / FC"
nm=""
if command -v nmcli >/dev/null; then
  while IFS= read -r n; do
    [[ -n "$n" ]] || continue
    if nmcli -g ipv4.addresses con show "$n" 2>/dev/null | grep -q '192\.168\.144\.50/'; then nm="$n"; break; fi
  done < <(nmcli -t -f NAME con show 2>/dev/null)
fi
if [[ -n "$nm" ]]; then ok "NM profili '$nm' (192.168.144.50)"
else dk "kalici saha profili yok (hunter-net) — README > Saha agi"; fi
if ip -br addr show eno1 2>/dev/null | grep -q '192\.168\.144\.50/'; then bi "eno1 su an 192.168.144.50"; fi
if [[ -f /etc/NetworkManager/dispatcher.d/99-eno1-eee-off ]]; then ok "eno1 EEE-off dispatcher"
else dk "EEE-off dispatcher yok: hunter-orin/scripts/setup_orin.sh --net-only"; fi
if [[ -e /dev/ttyACM0 ]]; then bi "/dev/ttyACM0 var (FC bagli)"
else bi "/dev/ttyACM0 yok (FC bagli degil — tezgahta normal)"; fi

# -----------------------------------------------------------------------------
bas "Video (GStreamer / ffmpeg)"
if gst-inspect-1.0 nvv4l2decoder >/dev/null 2>&1; then ok "nvv4l2decoder (donanim decode)"
else ko "nvv4l2decoder yok (JetPack / nvidia-l4t-gstreamer)"; fi
if gst-inspect-1.0 x264enc >/dev/null 2>&1; then ok "x264enc (Orin Nano'da NVENC yok)"
else ko "x264enc yok: sudo apt install gstreamer1.0-plugins-ugly"; fi
if pkg-config --exists gstreamer-rtsp-server-1.0; then ok "gst-rtsp-server (dev)"
else ko "gst-rtsp-server yok: sudo apt install libgstrtspserver-1.0-dev"; fi
if f=$(command -v ffmpeg); then ok "ffmpeg: $f ($(ffmpeg -version 2>/dev/null | head -1 | cut -d' ' -f3))"
else ko "ffmpeg yok: sudo apt install ffmpeg"; fi

# -----------------------------------------------------------------------------
bas "HUNTER (hunter-orin)"
if [[ -z "$H" ]]; then
  dk "hunter-orin klasoru repoda yok"
else
  if [[ -x "$H/kcf/build/kcfTracker" ]]; then ok "kcfTracker ($(date -r "$H/kcf/build/kcfTracker" '+%F %H:%M'))"
  else ko "kcfTracker derlenmemis: cd $H/kcf && mkdir -p build && cd build && cmake .. && make -j\$(nproc)"; fi
  if [[ -f "$H/models/ORTrack_ep0300.onnx" ]]; then
    if [[ -f "$H/models/ORTrack_ep0300-fp16.engine" ]]; then
      ok "ORTrack engine ($(date -r "$H/models/ORTrack_ep0300-fp16.engine" '+%F'))"
    else dk "ORTrack engine yok (--ortrack KCF'e duser): cd $H && ./scripts/build_ortrack_engine.sh  (~2 dk)"; fi
  fi
  if find "$H/models" -maxdepth 1 -name 'UAV-YOLO*.onnx' 2>/dev/null | grep -q .; then
    ye=$(find "$H/models" -maxdepth 1 -name 'UAV-YOLO*.engine' -printf '%f ' 2>/dev/null)
    if [[ -n "$ye" ]]; then ok "hunter YOLO engine: $ye"
    else dk "hunter YOLO engine yok: cd $H && ./scripts/build_yolo_engine.sh"; fi
  fi
  if command -v mavproxy.py >/dev/null || [[ -x "$HOME/.local/bin/mavproxy.py" ]]; then ok "mavproxy.py"
  else ko "MAVProxy yok: pip3 install --user MAVProxy"; fi
fi

# -----------------------------------------------------------------------------
bas "Python"
py=$(timeout 240 python3 - 2>/dev/null <<'PY'
def sat(d, ad, bilgi, cozum=""):
    print("|".join([d, ad, str(bilgi).replace("|", "/").replace("\n", " "), cozum]))
def dene(ad, fn, cozum):
    try:
        tamam, bilgi = fn()
        sat("OK" if tamam else "NO", ad, bilgi, "" if tamam else cozum)
    except Exception as e:
        sat("NO", ad, f"yok ({type(e).__name__})", cozum)
def f_cv2():   # cv2 torch'tan ONCE (Jetson'da libgomp TLS sorunu olmasin)
    import cv2
    g = [l.strip() for l in cv2.getBuildInformation().splitlines() if "GStreamer" in l]
    gs = bool(g) and "YES" in g[0]
    pip = ".local" in (cv2.__file__ or "")
    return gs and not pip, f"{cv2.__version__} {'GStreamer YES' if gs else 'GStreamer YOK'}{' (pip opencv golgeliyor)' if pip else ''}"
def f_numpy():
    import numpy; return numpy.__version__.startswith("1."), numpy.__version__
def f_pymav():
    import pymavlink
    try:
        from importlib.metadata import version; v = version("pymavlink")
    except Exception:
        v = getattr(pymavlink, "__version__", "?")
    return True, v
def f_torch():
    import torch; c = torch.cuda.is_available(); return c, f"{torch.__version__} CUDA={'var' if c else 'YOK'}"
def f_trt():
    import tensorrt; return True, tensorrt.__version__
def f_onnx():
    import onnx; return True, onnx.__version__
def f_ort():
    import onnxruntime as o; p = o.get_available_providers()
    return "CUDAExecutionProvider" in p, f"{o.__version__} ({'CUDA' if 'CUDAExecutionProvider' in p else 'sadece CPU'})"
def f_ultra():
    import ultralytics; return True, ultralytics.__version__
dene("cv2", f_cv2, "pip3 uninstall -y opencv-python opencv-python-headless (import hatasi: sudo apt install nvidia-opencv)")
dene("numpy", f_numpy, 'pip3 install --user --force-reinstall --no-deps "numpy<2"')
dene("pymavlink", f_pymav, "pip3 install --user pymavlink")
dene("torch", f_torch, "./setup/tiled_ortam.sh")
dene("tensorrt", f_trt, "sudo apt install nvidia-jetpack")
dene("onnx", f_onnx, "./setup/tiled_ortam.sh")
dene("onnxruntime", f_ort, "./setup/tiled_ortam.sh")
dene("ultralytics", f_ultra, "./setup/tiled_ortam.sh")
PY
)
if [[ -z "$py" ]]; then ko "python3 kontrolu calismadi"; fi
while IFS='|' read -r d ad bilgi cozum; do
  case "$d" in
    OK) ok "$(printf '%-12s' "$ad") $bilgi" ;;
    NO) ko "$(printf '%-12s' "$ad") $bilgi${cozum:+  -> $cozum}" ;;
  esac
done < <(grep -E '^(OK|NO)\|' <<<"$py")

# -----------------------------------------------------------------------------
bas "orin-tiled-tracker"
if [[ ! -d "$T" ]]; then
  dk "orin-tiled-tracker klasoru repoda yok"
else
  e=$(find "$T" -maxdepth 1 -name '*.engine' -printf '%f %s %TY-%Tm-%Td\n' 2>/dev/null \
      | awk '{printf "%s (%.0f MB, %s)  ", $1, $2/1048576, $3}')
  if [[ -n "$e" ]]; then ok "engine: $e"
  else dk "engine yok: cd $T && bash build_engine.sh UAV-YOLOv11m.pt  (20-40 dk)"; fi
  p=$(find "$T" -maxdepth 1 -name '*.pt' -printf '%f ' 2>/dev/null)
  if [[ -n "$p" ]]; then bi "model: $p"; else dk "orin-tiled-tracker'da .pt yok"; fi
fi

# -----------------------------------------------------------------------------
bas "Git"
if git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1; then
  n=$(git -C "$REPO" status --porcelain 2>/dev/null | wc -l)
  bi "dal: $(git -C "$REPO" branch --show-current 2>/dev/null) | son commit: $(git -C "$REPO" log -1 --format='%h %cs %s' 2>/dev/null)"
  if (( n > 0 )); then bi "commit'lenmemis degisiklik: $n dosya (git status)"; fi
  if ! git -C "$REPO" remote get-url origin >/dev/null 2>&1; then dk "GitHub (origin) tanimli degil"; fi
else
  dk "$REPO bir git reposu degil"
fi

# -----------------------------------------------------------------------------
bas "OZET"
echo -e "  \033[32m✓ $IYI\033[0m   \033[33m! $DIKKAT\033[0m   \033[31m✗ $KOTU\033[0m"
if (( KOTU == 0 )); then echo "  ✗ yok — calismaya hazir."
else echo "  ✗ olanlari duzelt (komutlar satir sonunda)."; fi
exit $(( KOTU > 0 ))
