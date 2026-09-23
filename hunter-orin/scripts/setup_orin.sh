#!/usr/bin/env bash
# ============================================================================
# HUNTER Orin — SIFIRDAN KURULUM (tek seferlik, birlesik surum)
# ============================================================================
# Test edildi: JetPack 6 (L4T R36.4.x), Ubuntu 22.04 aarch64, GStreamer 1.20.3,
#              OpenCV 4.8 (JetPack), cmake 3.22, Python 3.10, mavlink-router v4
#
# Bu script TEK basina yeni/yedek Orin'i ucusa hazir yapar:
#   APT bagimliliklari + OpenCV kontrol + Python (requirements) +
#   mavlink-router (kaynaktan) + router config /etc'e + mavlink/ header +
#   C++ derleme + GStreamer eklenti dogrulama + eno1 IP + EEE fix + dialout
#
# Bagimliliklar (gercek tarama ile dogrulandi):
#   C++ (CMakeLists): OpenCV, GStreamer(+rtsp-server,+app), OpenMP, pthread, stdc++fs
#   Python: numpy, pymavlink  (torch/model GEREKMEZ — PN teacher saf matematik)
#   GStreamer eklenti: nvv4l2decoder (JetPack), x264enc (plugins-ugly)
#
# Calistirma:  cd hunter-orin && ./scripts/setup_orin.sh
#   --skip-build   C++ derlemeyi atla
#   --net-only     sadece eno1 IP + EEE + dialout (apt/pip/derleme yok)
#   --eno1-ip A.B.C.D   eno1 IP (varsayilan 192.168.144.50)
# ============================================================================
set -euo pipefail

ENO1_IP="192.168.144.50"
SKIP_BUILD=0
NET_ONLY=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build) SKIP_BUILD=1; shift;;
    --net-only)   NET_ONLY=1; shift;;
    --eno1-ip)    ENO1_IP="$2"; shift 2;;
    *) echo "Bilinmeyen arg: $1"; exit 1;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

log()  { echo -e "\n\033[1;36m[SETUP]\033[0m $*"; }
warn() { echo -e "\033[1;33m[UYARI]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[OK]\033[0m $*"; }

echo "=========================================="
echo " HUNTER Orin kurulum (birlesik)"
echo "=========================================="

# --- 0) Ortam kontrolu ---
log "Ortam kontrolu"
if [[ -f /etc/nv_tegra_release ]]; then
  head -1 /etc/nv_tegra_release
else
  warn "/etc/nv_tegra_release yok — bu bir Jetson olmayabilir."
  warn "nvv4l2decoder/JetPack eklentileri OLMAYABILIR. Yine de devam."
fi

if [[ "$NET_ONLY" -eq 0 ]]; then
  # --- 1) APT bagimliliklari ---
  log "[1/7] APT paketleri"
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake git pkg-config ethtool lsof \
    meson ninja-build \
    python3-pip python3-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev libgstrtspserver-1.0-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav
  ok "APT paketleri tamam"

  # --- 2) OpenCV KONTROLU (JetPack ile gelir — KURMA, cakisma yaratir) ---
  log "[2/7] OpenCV kontrolu"
  if pkg-config --exists opencv4; then
    ok "OpenCV bulundu: $(pkg-config --modversion opencv4)"
  else
    warn "OpenCV (opencv4 pkg-config) bulunamadi. JetPack normalde OpenCV ile gelir."
    warn "Cozum: JetPack SDK Manager ile kur ya da 'sudo apt install nvidia-opencv'."
    warn "NOT: apt 'python3-opencv' (4.5) + nvidia-opencv (4.10) cakisip cv2 import'u"
    warn "     kirabilir. C++ tarafi libopencv-dev ile derlenir; tracker'i etkilemez."
  fi

  # --- 3) Python paketleri (requirements.txt veya numpy+pymavlink) ---
  log "[3/7] Python paketleri"
  if [[ -f "$SCRIPT_DIR/requirements.txt" ]]; then
    pip3 install --user -r "$SCRIPT_DIR/requirements.txt"
  elif [[ -f "$REPO_ROOT/requirements.txt" ]]; then
    pip3 install --user -r "$REPO_ROOT/requirements.txt"
  else
    warn "requirements.txt yok — numpy + pymavlink dogrudan kuruluyor"
    pip3 install --user "numpy==1.25.2" "pymavlink==2.4.49"
  fi
  python3 -c "import numpy, pymavlink; print('[OK] numpy', numpy.__version__, '| pymavlink', pymavlink.__version__)"

  # --- 4) mavlink-router (kaynaktan; $HOME'da kalici, varsa atla) ---
  log "[4/7] mavlink-router"
  if command -v mavlink-routerd >/dev/null 2>&1; then
    ok "mavlink-router zaten kurulu: $(mavlink-routerd --version 2>&1 | head -1)"
  else
    MR_DIR="$HOME/mavlink-router"
    if [[ ! -d "$MR_DIR" ]]; then
      git clone https://github.com/mavlink-router/mavlink-router.git "$MR_DIR"
    fi
    pushd "$MR_DIR" >/dev/null
    git submodule update --init --recursive
    meson setup build . --reconfigure || meson setup build .
    ninja -C build
    sudo ninja -C build install
    sudo ldconfig
    popd >/dev/null
    ok "mavlink-router kuruldu: $(mavlink-routerd --version 2>&1 | head -1)"
  fi

  # --- 5) Router config'i /etc'e koy (run_router.sh bunu bekler) ---
  log "[5/7] mavlink-router config -> /etc/mavlink-router/main.conf"
  sudo mkdir -p /etc/mavlink-router
  sudo cp "$REPO_ROOT/config/mavlink-router/main.conf" /etc/mavlink-router/main.conf
  ok "/etc/mavlink-router/main.conf yazildi"

  # --- 6) MAVLink header-only klasoru (CMakeLists ../mavlink bekler) ---
  log "[6/7] MAVLink header-only (../mavlink)"
  if [[ -d "$REPO_ROOT/mavlink/common" ]]; then
    ok "mavlink/ repoda mevcut"
  else
    warn "mavlink/ eksik — c_library_v2 klonlaniyor"
    git clone https://github.com/mavlink/c_library_v2.git "$REPO_ROOT/mavlink"
  fi

  # --- 7) C++ derleme (kcf/build) ---
  if [[ "$SKIP_BUILD" -eq 0 ]]; then
    log "[7/7] C++ derleme (kcf/build)"
    pushd "$REPO_ROOT/kcf" >/dev/null
    mkdir -p build && cd build
    cmake ..
    make -j"$(nproc)"
    if [[ -f ./kcfTracker ]]; then ok "kcfTracker derlendi"; else warn "kcfTracker uretilmedi — cmake/make ciktisina bak"; fi
    popd >/dev/null
  else
    warn "[7/7] Derleme atlandi (--skip-build)"
  fi

  # --- 7b) GStreamer eklenti DOGRULAMA (kritik — yoksa pipeline calismaz) ---
  log "GStreamer eklenti dogrulama"
  for el in nvv4l2decoder x264enc rtph264pay h264parse rtspsrc; do
    if gst-inspect-1.0 "$el" >/dev/null 2>&1; then
      ok "  $el VAR"
    else
      warn "  $el YOK — runtracker pipeline'i bu eleman olmadan calismaz!"
      [[ "$el" == "nvv4l2decoder" ]] && warn "    nvv4l2decoder JetPack ile gelir; 'nvidia-l4t-gstreamer' kurulu mu?"
    fi
  done
fi

# --- 8) AG: eno1 IP ($ENO1_IP) ---
log "Ag: eno1 ($ENO1_IP)"
if command -v nmcli >/dev/null 2>&1 && nmcli -t -f DEVICE device 2>/dev/null | grep -qx eno1; then
  warn "eno1 icin KALICI IP'yi NetworkManager ile ayarlamak istersen:"
  echo "    sudo nmcli con mod <eno1-con> ipv4.addresses $ENO1_IP/24 ipv4.method manual && sudo nmcli con up <eno1-con>"
fi
if ! ip -4 addr show eno1 2>/dev/null | grep -q "$ENO1_IP"; then
  warn "eno1'de $ENO1_IP yok — gecici atama (reboot'ta gider; kalici icin yukaridaki nmcli)"
  sudo ip addr add "$ENO1_IP/24" dev eno1 2>/dev/null || true
  sudo ip link set eno1 up 2>/dev/null || true
else
  ok "eno1 = $ENO1_IP"
fi

# --- 9) EEE fix: RTL8111 <-> SIYI VTX link flap (video donmasinin kok nedeni) ---
log "EEE fix (eno1 802.3az kapatma) — NetworkManager dispatcher"
DISP_SRC=""
for c in "$REPO_ROOT/config/99-eno1-eee-off" "$REPO_ROOT/scripts/99-eno1-eee-off"; do
  [[ -f "$c" ]] && DISP_SRC="$c" && break
done
DISP_DST="/etc/NetworkManager/dispatcher.d/99-eno1-eee-off"
if [[ -n "$DISP_SRC" ]]; then
  sudo install -m 0755 -o root -g root "$DISP_SRC" "$DISP_DST"
  ok "dispatcher kuruldu: $DISP_DST"
else
  warn "repo'da 99-eno1-eee-off yok — dispatcher uretiliyor"
  sudo tee "$DISP_DST" >/dev/null << 'DISP'
#!/bin/bash
# eno1 her UP oldugunda EEE kapat (RTL8111 <-> SIYI VTX 802.3az uyumsuzlugu)
[ "$1" = "eno1" ] && [ "$2" = "up" ] && /usr/sbin/ethtool --set-eee eno1 eee off
DISP
  sudo chmod 0755 "$DISP_DST"
  ok "dispatcher uretildi: $DISP_DST"
fi
sudo ethtool --set-eee eno1 eee off 2>/dev/null && ok "EEE simdi kapatildi" || warn "ethtool --set-eee basarisiz (link down olabilir; up olunca dispatcher halleder)"
if sudo ethtool --show-eee eno1 2>/dev/null | grep -qiE "EEE status:\s*disabled"; then
  ok "EEE durumu: disabled (dogrulandi)"
else
  warn "EEE 'disabled' gorunmuyor — link UP degilken normal; up olunca: sudo ethtool --show-eee eno1"
fi

# --- 10) dialout grubu (/dev/ttyACM0 — FC USB erisimi) ---
log "dialout grubu (FC /dev/ttyACM0 erisimi)"
if id -nG "$USER" | grep -qw dialout; then
  ok "$USER zaten dialout grubunda"
else
  sudo usermod -aG dialout "$USER"
  warn "dialout grubuna eklendin — LOGOUT/LOGIN (veya reboot) gerekli"
fi

# --- OZET ---
echo ""
echo "============================================================"
ok   "Kurulum tamamlandi."
echo "============================================================"
echo "Sonraki adimlar:"
echo "  1) Kamera ag/route:   ./scripts/camera_route.sh"
echo "  2) FC parametreleri:  README (RC_OVERRIDE_TIME=0.3, SYSID_MYGCS=255 ...)"
echo "  3) Calistir (3 terminal):"
echo "       ./scripts/run_router.sh"
echo "       ./scripts/run_tracker.sh"
echo "       ./scripts/run_brain.sh                 # DRY-RUN (motor yok)"
echo "       ./scripts/run_brain.sh --enable-override   # PERVANESIZ bench"
echo ""
echo "Dogrulama:"
echo "  - EEE:     sudo ethtool --show-eee eno1   -> disabled"
echo "  - Build:   ls kcf/build/kcfTracker"
echo "  - Net:     ip -4 addr show eno1 | grep $ENO1_IP"
echo "  - Router:  ls /etc/mavlink-router/main.conf"
echo "  - dialout (login sonrasi):  id -nG | grep dialout"
echo "============================================================"
