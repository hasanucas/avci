#!/usr/bin/env bash
# =============================================================================
# ortam_kaydet.sh — CALISAN Orin'in yazilim ortamini kaydeder (sadece okur)
# =============================================================================
#   ./setup/ortam_kaydet.sh [klasor]        varsayilan: setup/referans_ortam
#
# Cikti:
#   ortam.txt        L4T, JetPack, guc modu, surumler, GStreamer eklentileri
#   pip_freeze.txt   tum Python paketleri (kayit)
#   pip_user.txt     pip ile kurulanlar -> kurulum_orin.sh / tiled_ortam.sh surumleri
#                    BURADAN sabitler (numpy, ultralytics, onnx, MAVProxy, pymavlink...)
#   dpkg_secili.txt  nvidia / cuda / tensorrt / opencv / gstreamer apt paketleri
#   ag.txt           Ethernet NetworkManager profilleri (hunter-net)
#
# Ortam bilerek degisince (yeni paket, surum) tekrar calistir ve commit'le:
#   ./setup/ortam_kaydet.sh && git add setup/referans_ortam && git commit -m "ortam kaydi"
# sudo istemez, hicbir seyi degistirmez (sadece hedef klasore yazar).
# =============================================================================
set -u
D="${1:-$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/referans_ortam}"
mkdir -p "$D" || exit 1

l4t() {  # "# R36 (release), REVISION: 4.7, ..." -> R36.4.7
  sed -nE '1s/^# R([0-9]+) \(release\), REVISION: ([0-9.]+).*/R\1.\2/p' /etc/nv_tegra_release 2>/dev/null
}
guc_modu() {  # isim: MAXN_SUPER / 15W / ...
  local ad id
  ad=$(nvpmodel -q 2>/dev/null | sed -nE 's/^NV Power Mode: *//p' | head -1)
  if [[ -z "$ad" ]]; then
    id=$(sed -nE 's/^pmode:0*([0-9]+).*/\1/p' /var/lib/nvpmodel/status 2>/dev/null)
    [[ -n "$id" ]] && ad=$(sed -nE "s/.*POWER_MODEL[[:space:]]+ID=${id}[[:space:]]+NAME=([A-Za-z0-9_]+).*/\1/p" \
                          /etc/nvpmodel.conf 2>/dev/null | head -1)
  fi
  echo "${ad:-?}"
}

echo "Ortam kaydi -> $D"

# --- ortam.txt ---------------------------------------------------------------
{
  echo "# AVCI referans ortam — $(date '+%F %T') — $(hostname) — ${USER:-$(id -un)}"
  echo "# Anahtar satirlar (kurulum_orin.sh / kontrol.sh okur):"
  echo "L4T=$(l4t)"
  echo "JETPACK=$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null)"
  echo "GUC_MODU=$(guc_modu)"
  echo
  echo "## L4T";      head -1 /etc/nv_tegra_release 2>/dev/null || echo "yok (Jetson degil?)"
  echo "## Guc modu"; nvpmodel -q 2>/dev/null || cat /var/lib/nvpmodel/status 2>/dev/null || echo "?"
  echo "## Sistem"
  echo "kernel  : $(uname -r) $(uname -m)"
  echo "python  : $(python3 --version 2>&1)"
  echo "cmake   : $(cmake --version 2>/dev/null | head -1)"
  echo "gcc     : $(gcc -dumpfullversion 2>/dev/null)"
  echo "gst     : $(gst-launch-1.0 --version 2>/dev/null | sed -n 2p)"
  echo "opencv4 : $(pkg-config --modversion opencv4 2>/dev/null) (C++ / pkg-config)"
  echo "cuda    : $(/usr/local/cuda/bin/nvcc --version 2>/dev/null | tail -1)"
  echo "ram     : $(free -m 2>/dev/null | awk '/^Mem:/{print $2" MB"}')"
  echo "swap    :"; swapon --show 2>/dev/null | sed 's/^/  /'
  echo "## GStreamer eklentileri"
  for e in nvv4l2decoder x264enc rtspclientsink h265parse; do
    if gst-inspect-1.0 "$e" >/dev/null 2>&1; then echo "$e: VAR"; else echo "$e: YOK"; fi
  done
  echo "## ffmpeg (PATH sirasiyla)"
  type -ap ffmpeg 2>/dev/null | awk '!s[$0]++' | while read -r f; do
    echo "$f : $("$f" -version 2>/dev/null | head -1)"
  done
  echo "## Python paketleri (import edilen surum + yeri)"
  (timeout 240 python3 - <<'PY' 2>/dev/null
import importlib, sys
def yaz(ad, mod, ek=""):
    try:
        m = importlib.import_module(mod)
        v = str(getattr(m, "__version__", "?"))
        yer = getattr(m, "__file__", "") or ""
        print(f"{ad:12s} {v:34s} {ek(m) if callable(ek) else ek}  [{yer}]")
    except Exception as e:
        print(f"{ad:12s} YOK ({type(e).__name__}: {str(e)[:80]})")
def gst(cv2):
    s = [l.strip() for l in cv2.getBuildInformation().splitlines() if "GStreamer" in l]
    return s[0] if s else "GStreamer ?"
yaz("numpy", "numpy")
yaz("cv2", "cv2", gst)
yaz("torch", "torch", lambda t: f"cuda={t.cuda.is_available()}")
yaz("torchvision", "torchvision")
yaz("tensorrt", "tensorrt")
yaz("onnx", "onnx")
yaz("onnxruntime", "onnxruntime", lambda o: ",".join(o.get_available_providers()))
yaz("ultralytics", "ultralytics")
yaz("scipy", "scipy")
yaz("yaml", "yaml")
yaz("pymavlink", "pymavlink")
yaz("MAVProxy", "MAVProxy")
PY
  ) | grep -E '^(numpy|cv2|torch|torchvision|tensorrt|onnx|onnxruntime|ultralytics|scipy|yaml|pymavlink|MAVProxy) '
} > "$D/ortam.txt"
echo "  ortam.txt ($(wc -l < "$D/ortam.txt") satir)"

# --- pip_freeze.txt ----------------------------------------------------------
# URL'lerdeki olasi kullanici:token kismi silinir
# pip_freeze.txt = hepsi (kayit), pip_user.txt = pip ile ~/.local'e kurulanlar (surum sabitleme bundan)
python3 -m pip freeze 2>/dev/null        | sed -E 's#(https?://)[^/@[:space:]]+@#\1#g' > "$D/pip_freeze.txt"
python3 -m pip freeze --user 2>/dev/null | sed -E 's#(https?://)[^/@[:space:]]+@#\1#g' > "$D/pip_user.txt"
if [[ -s "$D/pip_freeze.txt" ]]; then
  echo "  pip_freeze.txt ($(wc -l < "$D/pip_freeze.txt") paket), pip_user.txt ($(wc -l < "$D/pip_user.txt") paket)"
else
  echo "  ! pip freeze alinamadi (python3-pip kurulu mu?)"
fi

# --- dpkg_secili.txt ---------------------------------------------------------
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 2>/dev/null \
  | grep -Ei 'nvidia|tensorrt|nvinfer|cuda|cudnn|opencv|gstreamer|x264|cusparselt|ffmpeg' \
  | sort -k2 > "$D/dpkg_secili.txt"
echo "  dpkg_secili.txt ($(wc -l < "$D/dpkg_secili.txt") paket)"

# --- ag.txt ------------------------------------------------------------------
{
  echo "## Ethernet NetworkManager profilleri: ad | ipv4.method | adresler | autoneg | arayuz"
  nmcli -t -f NAME,TYPE con show 2>/dev/null | awk -F: '$2=="802-3-ethernet"{print $1}' \
    | while IFS= read -r n; do
        echo "$n | $(nmcli -g ipv4.method,ipv4.addresses,802-3-ethernet.auto-negotiate,connection.interface-name \
                       con show "$n" 2>/dev/null | paste -sd'|' | sed 's/|/ | /g')"
      done
  echo "## eno1"
  ip -br addr show eno1 2>/dev/null || echo "eno1 yok"
  if [[ -f /etc/NetworkManager/dispatcher.d/99-eno1-eee-off ]]; then
    echo "EEE-off dispatcher: VAR"
  else
    echo "EEE-off dispatcher: YOK"
  fi
} > "$D/ag.txt"
echo "  ag.txt"
[[ -n "${AVCI_ICERDEN:-}" ]] || echo "Tamam. Commit'le: git add setup/referans_ortam && git commit -m \"ortam kaydi\""
