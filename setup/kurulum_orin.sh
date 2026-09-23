#!/usr/bin/env bash
# =============================================================================
# kurulum_orin.sh — YENI Orin'i bu repodan calisir hale getirir (tek komut)
# =============================================================================
# On kosul: JetPack 6.2.1 (L4T R36.4.7) kurulu Orin Nano, kullanici avci2, internet.
#   git clone git@github.com:hasanucas/avci.git ~/Desktop/avci
#   cd ~/Desktop/avci && ./setup/kurulum_orin.sh
#     --skip-engine   TensorRT engine uretme (sonra: cd orin-tiled-tracker && bash build_engine.sh ...)
#     --model X.pt    engine modeli (varsayilan UAV-YOLOv11m.pt)
# Sure ~1 saat (engine 20-40 dk). Log: setup/kurulum_<tarih>.log (git'e girmez)
# Tekrar calistirmak guvenli: hazir olan adimlar atlanir.
# CALISAN (ucan) Orin'de calistirma — orada sadece: ./setup/kontrol.sh
#
# Adimlar: 0 on kontrol   1 apt   2 JetPack   3 guc modu + swap   4 hunter-orin
#          5 tiled ortam   6 python son ayar   7 engine'ler (ORTrack + YOLO)   8 kisayollar   9 kontrol
# Bir adim hata verirse durmaz: not alir, sonda ozetler.
# =============================================================================
set -u

SETUP_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(dirname "$SETUP_DIR")"
REF="$SETUP_DIR/referans_ortam"
KUL="${USER:-$(id -un)}"
LOG="$SETUP_DIR/kurulum_$(date +%Y%m%d_%H%M).log"
ANA_PID=$$
MODEL="UAV-YOLOv11m.pt"
SKIP_ENGINE=0
while (( $# )); do
  case "$1" in
    --skip-engine) SKIP_ENGINE=1; shift ;;
    --model) [[ -n "${2:-}" ]] || { echo "Kullanim: --model dosya.pt"; exit 1; }
             MODEL="$2"; shift 2 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1   (--skip-engine | --model X.pt)"; exit 1 ;;
  esac
done
export PATH="$HOME/.local/bin:$PATH"     # yolo, mavproxy.py (pip --user)

HATA=()
bas() { echo -e "\n\033[1;36m== $* ==\033[0m"; }
ok()  { echo -e "  \033[32m✓\033[0m $*"; }
bi()  { echo -e "  • $*"; }
uy()  { echo -e "  \033[33m!\033[0m $*"; }
ha()  { echo -e "  \033[31m✗\033[0m $*"; HATA+=("$*"); }
sor() { local c=""; read -rp "  $* [e/H] " c; [[ "$c" =~ ^[eEyY]$ ]]; }
APT() { sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get -y -q \
          -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold "$@"; }
PIP() { python3 -m pip install --user --no-warn-script-location "$@"; }
pin() {  # pin <paket> <varsayilan>: referans kayittaki surum ya da varsayilan
  local v=""
  if [[ -f "$REF/pip_user.txt" ]]; then
    v=$(awk -F'==' -v p="$1" 'tolower($1)==tolower(p){print $2; exit}' "$REF/pip_user.txt")
  fi
  if [[ -n "$v" ]]; then echo "$1==$v"; else echo "$2"; fi
}
l4t() { sed -nE '1s/^# R([0-9]+) \(release\), REVISION: ([0-9.]+).*/R\1.\2/p' /etc/nv_tegra_release 2>/dev/null; }
guc_modu() {  # isim: MAXN_SUPER / 15W / 25W / 7W
  local ad id
  ad=$( { nvpmodel -q 2>/dev/null || sudo -n nvpmodel -q 2>/dev/null; } | sed -nE 's/^NV Power Mode: *//p' | head -1)
  if [[ -z "$ad" ]]; then
    id=$(sed -nE 's/^pmode:0*([0-9]+).*/\1/p' /var/lib/nvpmodel/status 2>/dev/null)
    [[ -n "$id" ]] && ad=$(sed -nE "s/.*POWER_MODEL[[:space:]]+ID=${id}[[:space:]]+NAME=([A-Za-z0-9_]+).*/\1/p" \
                          /etc/nvpmodel.conf 2>/dev/null | head -1)
  fi
  echo "${ad:-?}"
}
maxn_id() {  # MAXN_SUPER'in (yoksa MAXN'in) ID'si — ISIMLE bulunur (bu unitede 0 = 15W!)
  local n id
  for n in MAXN_SUPER MAXN; do
    id=$(sed -nE "s/.*POWER_MODEL[[:space:]]+ID=([0-9]+)[[:space:]]+NAME=${n}[[:space:]]*>.*/\1/p" \
         /etc/nvpmodel.conf 2>/dev/null | head -1)
    if [[ -n "$id" ]]; then echo "$id"; return 0; fi
  done
  return 1
}
maxn_yap() {
  local id m
  if ! id=$(maxn_id); then uy "nvpmodel.conf'ta MAXN yok — guc moduna dokunulmadi"; return; fi
  m=$(guc_modu)
  if [[ "$m" == MAXN* ]]; then ok "guc modu $m (ID $id)"; return; fi
  echo no | sudo nvpmodel -m "$id" >/dev/null 2>&1   # "reboot?" sorusuna hayir; reboot en sonda
  m=$(guc_modu)
  if [[ "$m" == MAXN* ]]; then ok "guc modu -> $m (ID $id), reboot'ta kalir"
  else uy "guc modu '$m' — reboot sonrasi bak: sudo nvpmodel -q  (MAXN degilse: sudo nvpmodel -m $id)"; fi
}
swap_yap() {
  if [[ ! -f /swapfile ]]; then
    if sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile >/dev/null; then
      ok "/swapfile 8G olusturuldu"
    else ha "/swapfile olusturulamadi"; return; fi
  fi
  if ! grep -Eq '^/swapfile[[:space:]]' /etc/fstab; then
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null && ok "fstab: /swapfile her acilista acik"
  fi
  if swapon --show=NAME --noheadings 2>/dev/null | grep -qx /swapfile; then ok "swap acik: /swapfile"
  elif sudo swapon /swapfile; then ok "swap acildi: /swapfile"
  else ha "swapon /swapfile"; fi
}
nm_profil() {  # 192.168.144.50 adresli NetworkManager profilinin adi
  local n
  command -v nmcli >/dev/null || return 1
  while IFS= read -r n; do
    [[ -n "$n" ]] || continue
    if nmcli -g ipv4.addresses con show "$n" 2>/dev/null | grep -q '192\.168\.144\.50/'; then
      echo "$n"; return 0
    fi
  done < <(nmcli -t -f NAME con show 2>/dev/null)
  return 1
}
python_duzelt() {  # pip opencv sistem cv2'yi golgelemesin + numpy 1.x kalsin
  local v cv
  python3 -m pip uninstall -y opencv-python opencv-python-headless \
          opencv-contrib-python opencv-contrib-python-headless >/dev/null 2>&1
  v=$(python3 -c 'import numpy;print(numpy.__version__)' 2>/dev/null)
  if [[ "$v" == 1.* ]]; then ok "numpy $v"
  elif PIP --force-reinstall --no-deps "$(pin numpy 'numpy<2')" >/dev/null 2>&1; then
    ok "numpy ${v:-yok} -> $(python3 -c 'import numpy;print(numpy.__version__)' 2>/dev/null)"
  else ha "numpy<2 kurulamadi"; fi
  cv=$(python3 -c 'import cv2;print(cv2.__version__, cv2.__file__)' 2>/dev/null)
  if [[ -n "$cv" ]]; then ok "cv2 $cv"
  else ha "python3 'import cv2' calismiyor (JetPack OpenCV: sudo apt install nvidia-opencv)"; fi
}
engine_yap() {  # engine_yap <tiled klasoru>
  local T="$1" mem dock=0
  mem=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
  bi "20-40 dk surer. Uzun sessizlik ve 'Skipping tactic / out of memory' satirlari normal."
  if (( mem < 5000 )); then
    uy "bos RAM ${mem} MB (< 5000) — Firefox / VS Code'u kapat"
    sor "Simdi devam?" || { ha "engine atlandi — sonra: cd $T && bash build_engine.sh $MODEL"; return; }
  fi
  if systemctl is-active --quiet docker 2>/dev/null; then
    sudo systemctl stop docker docker.socket 2>/dev/null && dock=1 && bi "Docker durduruldu (RAM icin; sonra acilir)"
  fi
  (cd "$T" && bash build_engine.sh "$MODEL")
  if find "$T" -maxdepth 1 -name '*.engine' | grep -q .; then
    ok "engine: $(find "$T" -maxdepth 1 -name '*.engine' -printf '%f ')"
  else
    ha "engine uretilemedi (log'a bak). RAM yetmediyse GUI'siz dene: sudo systemctl isolate multi-user.target"
  fi
  if (( dock )); then sudo systemctl start docker && bi "Docker tekrar calisiyor"; fi
  maxn_yap        # build_engine.sh guc modunu degistirdiyse geri al
  python_duzelt   # export sirasinda pip numpy'yi 2'ye cikardiysa geri al
}

# =============================================================================
main() {
  local H="" T="$REPO/orin-tiled-tracker" d cur ref bos f nm eksik trt eng l

  bas "0/9 On kontrol"
  echo "  Repo: $REPO"
  echo "  Log : $LOG"
  if (( EUID == 0 )); then echo "  sudo ile degil, normal kullanici olarak calistir: ./setup/kurulum_orin.sh"; return 1; fi
  if [[ ! -f /etc/nv_tegra_release ]]; then echo "  Bu bir Jetson degil (/etc/nv_tegra_release yok)."; return 1; fi
  if [[ -d "$HOME/Desktop/_repo_yedek" ]]; then
    uy "Bu, repo_hazirla.sh'nin calistigi (ucan) Orin gibi gorunuyor (~/Desktop/_repo_yedek var)."
    uy "Kurulum apt/pip paketlerini degistirir; burada genelde sadece ./setup/kontrol.sh yeterli."
    sor "Yine de devam?" || return 0
  fi
  cur=$(l4t); ref=$(sed -n 's/^L4T=//p' "$REF/ortam.txt" 2>/dev/null)
  if [[ -z "$ref" ]]; then bi "L4T ${cur:-?} (referans kayit yok)"
  elif [[ "$cur" == "$ref" ]]; then ok "L4T $cur (referansla ayni)"
  else
    uy "L4T ${cur:-?} ama referans $ref — torch/onnxruntime wheel'leri farkli JetPack'te tutmayabilir"
    sor "Devam?" || return 0
  fi
  if python3 -c 'import sys; sys.exit(sys.version_info[:2] != (3, 10))' 2>/dev/null; then ok "$(python3 --version 2>&1)"
  else echo "  Python 3.10 gerekli (JetPack 6 / Ubuntu 22.04)."; return 1; fi
  sudo -v || { echo "  sudo yetkisi gerekli."; return 1; }
  ( while kill -0 "$ANA_PID" 2>/dev/null; do sudo -n -v 2>/dev/null; sleep 50; done ) >/dev/null 2>&1 &
  SUDO_PID=$!                             # global: EXIT trap'i main bittikten sonra calisir
  KISIT=$(mktemp /tmp/avci_pip_kisit.XXXXXX); echo "numpy<2" > "$KISIT"
  export PIP_CONSTRAINT="$KISIT"          # tum pip kurulumlari: numpy 2'ye ziplamasin
  trap 'kill "$SUDO_PID" 2>/dev/null; rm -f "$KISIT"' EXIT
  if python3 -c 'import urllib.request as u; u.urlopen("https://github.com", timeout=15)' 2>/dev/null; then ok "internet var"
  else echo "  ✗ internet yok (github.com'a ulasilamadi)."; return 1; fi
  bos=$(df -BG --output=avail / 2>/dev/null | tail -1 | tr -dc '0-9')
  if (( ${bos:-0} < 20 )); then
    uy "diskte ${bos:-?} GB bos (swap 8 GB + paketler ~10 GB ister)"; sor "Devam?" || return 0
  else ok "disk: ${bos} GB bos"; fi
  if [[ "$KUL" != avci2 ]]; then
    f=$(grep -rIl --exclude-dir=.git --exclude-dir=setup --exclude-dir=build --exclude-dir=__pycache__ --exclude='*.md' '/home/avci2' "$REPO" 2>/dev/null)
    if [[ -n "$f" ]]; then
      uy "kullanici '$KUL' (avci2 degil): $(grep -c . <<<"$f") dosyada /home/avci2 yolu var — o kisimlar calismaz:"
      head -5 <<<"$f" | sed "s|^$REPO/|      |"
    fi
  fi
  [[ "$REPO" == "$HOME/Desktop/avci" ]] || uy "repo ~/Desktop/avci degil ($REPO) — Desktop kisayollari yine buraya baglanir"
  for d in "$REPO/hunter-orin" "$REPO/hunter_orin"; do
    if [[ -d "$d" ]]; then H="$d"; break; fi
  done

  # ---------------------------------------------------------------------------
  bas "1/9 apt paketleri"
  sudo dpkg --configure -a || ha "dpkg --configure -a"
  APT update || uy "apt update hata verdi — devam"
  local PAKET=(git curl wget rsync unzip nano htop ethtool build-essential cmake pkg-config
               python3-pip python3-dev libopenblas-dev ffmpeg
               gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
               gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-rtsp
               libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev)
  if APT install "${PAKET[@]}"; then ok "apt paketleri tamam"
  else
    uy "toplu kurulum basarisiz — tek tek deneniyor"
    for d in "${PAKET[@]}"; do APT install "$d" >/dev/null 2>&1 || ha "apt: $d"; done
  fi

  # ---------------------------------------------------------------------------
  bas "2/9 JetPack (CUDA / TensorRT / OpenCV)"
  eksik=()
  python3 -c 'import tensorrt' >/dev/null 2>&1 || eksik+=(tensorrt)
  [[ -x /usr/local/cuda/bin/nvcc ]] || eksik+=(cuda)
  pkg-config --exists opencv4 || eksik+=(opencv)
  if (( ${#eksik[@]} )); then
    bi "eksik: ${eksik[*]} -> nvidia-jetpack kuruluyor (birkac GB, 10-20 dk)"
    APT install nvidia-jetpack || ha "nvidia-jetpack kurulamadi"
  fi
  trt=$(python3 -c 'import tensorrt;print(tensorrt.__version__)' 2>/dev/null)
  if [[ -n "$trt" && -x /usr/local/cuda/bin/nvcc ]] && pkg-config --exists opencv4; then
    ok "TensorRT $trt | CUDA $(/usr/local/cuda/bin/nvcc --version | sed -nE 's/.*release ([0-9.]+).*/\1/p') | OpenCV $(pkg-config --modversion opencv4)"
  else ha "JetPack bilesenleri eksik (tensorrt / cuda / opencv)"; fi

  # ---------------------------------------------------------------------------
  bas "3/9 Guc modu (MAXN_SUPER, isimle) + swap 8G"
  maxn_yap
  swap_yap

  # ---------------------------------------------------------------------------
  bas "4/9 hunter-orin (scripts/setup_orin.sh + MAVProxy)"
  if [[ -z "$H" ]]; then
    ha "hunter-orin klasoru repoda yok"
  else
    if [[ -f "$H/scripts/setup_orin.sh" ]]; then
      if ! bash "$H/scripts/setup_orin.sh"; then
        ha "hunter-orin/scripts/setup_orin.sh hata verdi (log'a bak)"
        bi "ag / EEE / dialout kismi ayrica deneniyor: setup_orin.sh --net-only"
        bash "$H/scripts/setup_orin.sh" --net-only || ha "setup_orin.sh --net-only"
      fi
    else ha "hunter-orin/scripts/setup_orin.sh yok"; fi
    if [[ -x "$H/kcf/build/kcfTracker" ]]; then ok "kcfTracker derli"
    elif [[ -f "$H/kcf/CMakeLists.txt" ]]; then
      bi "kcfTracker yok -> derleniyor (cmake .. && make)"
      if mkdir -p "$H/kcf/build" && (cd "$H/kcf/build" && cmake .. && make -j"$(nproc)") \
         && [[ -x "$H/kcf/build/kcfTracker" ]]; then ok "kcfTracker derlendi"
      else ha "kcfTracker derlenemedi"; fi
    else ha "hunter-orin/kcf/CMakeLists.txt yok"; fi
    # MAVProxy: FC baglantisi (mavlink-router yerine; scripts/run_router_mavproxy.sh, --streamrate=-1)
    if python3 -c 'import MAVProxy, pymavlink' >/dev/null 2>&1 && command -v mavproxy.py >/dev/null; then
      ok "MAVProxy kurulu"
    elif PIP "$(pin MAVProxy MAVProxy)" "$(pin pymavlink pymavlink)"; then ok "MAVProxy kuruldu"
    else ha "MAVProxy kurulamadi (pip)"; fi
    if nm=$(nm_profil); then ok "NM profili '$nm' (192.168.144.50)"
    else bi "kalici saha profili (hunter-net) yok — komutu en sonda (internet isin bitince)"; fi
  fi

  # ---------------------------------------------------------------------------
  bas "5/9 orin-tiled-tracker ortami (setup/tiled_ortam.sh)"
  if [[ -d "$T" ]]; then bash "$SETUP_DIR/tiled_ortam.sh" || ha "tiled_ortam.sh hata verdi (log'a bak)"
  else ha "orin-tiled-tracker klasoru repoda yok"; fi

  # ---------------------------------------------------------------------------
  bas "6/9 Python son ayar (pip opencv yok, numpy<2)"
  python_duzelt

  # ---------------------------------------------------------------------------
  bas "7/9 TensorRT engine'ler (ORTrack + hunter YOLO + tiled $MODEL)"
  # ORTrack (runtracker --ortrack): hunter-orin/models/ORTrack*.onnx'ten, ~2 dk
  if [[ -n "$H" && -f "$H/models/ORTrack_ep0300.onnx" ]]; then
    if [[ -f "$H/models/ORTrack_ep0300-fp16.engine" ]]; then ok "ORTrack engine zaten var"
    elif (( SKIP_ENGINE )); then bi "--skip-engine: ORTrack atlandi. Sonra: cd $H && ./scripts/build_ortrack_engine.sh"
    elif [[ ! -f "$H/scripts/build_ortrack_engine.sh" ]]; then ha "hunter-orin/scripts/build_ortrack_engine.sh yok"
    else
      bi "ORTrack engine uretiliyor (~2 dk)"
      (cd "$H" && bash scripts/build_ortrack_engine.sh)
      if [[ -f "$H/models/ORTrack_ep0300-fp16.engine" ]]; then ok "ORTrack engine uretildi"
      else ha "ORTrack engine uretilemedi (log'a bak) — runtracker --ortrack KCF'e duser"; fi
    fi
  fi
  # hunter YOLO: hunter-orin/models/UAV-YOLO*.onnx'ten (scripts/build_yolo_engine.sh)
  if [[ -n "$H" && -f "$H/scripts/build_yolo_engine.sh" ]] \
     && find "$H/models" -maxdepth 1 -name 'UAV-YOLO*.onnx' 2>/dev/null | grep -q .; then
    if find "$H/models" -maxdepth 1 -name 'UAV-YOLO*.engine' | grep -q .; then ok "hunter YOLO engine zaten var"
    elif (( SKIP_ENGINE )); then bi "--skip-engine: hunter YOLO atlandi. Sonra: cd $H && ./scripts/build_yolo_engine.sh"
    else
      bi "hunter YOLO engine uretiliyor (scripts/build_yolo_engine.sh)"
      (cd "$H" && bash scripts/build_yolo_engine.sh)
      if find "$H/models" -maxdepth 1 -name 'UAV-YOLO*.engine' | grep -q .; then
        ok "hunter YOLO engine: $(find "$H/models" -maxdepth 1 -name 'UAV-YOLO*.engine' -printf '%f ')"
      else ha "hunter YOLO engine uretilemedi — elle: cd $H && ./scripts/build_yolo_engine.sh (kullanimina bak)"; fi
    fi
  fi
  # YOLO (orin-tiled-tracker)
  eng=$(find "$T" -maxdepth 1 -name '*.engine' 2>/dev/null | head -1)
  if (( SKIP_ENGINE )); then bi "--skip-engine: atlandi. Sonra: cd $T && bash build_engine.sh $MODEL"
  elif [[ -n "$eng" ]]; then ok "engine zaten var: $(basename "$eng") (yeniden uretmek icin sil)"
  elif [[ ! -f "$T/$MODEL" ]]; then ha "model yok: $T/$MODEL"
  elif [[ ! -f "$T/build_engine.sh" ]]; then ha "orin-tiled-tracker/build_engine.sh yok"
  else engine_yap "$T"; fi

  # ---------------------------------------------------------------------------
  bas "8/9 Desktop kisayollari (eski yollar ~/Desktop/<proje> calissin)"
  mkdir -p "$HOME/Desktop" "$HOME/recordings"      # recordings: kayit kodlarinin cikti klasoru
  for d in "$H" "$T"; do
    [[ -n "$d" && -d "$d" ]] || continue
    l="$HOME/Desktop/$(basename "$d")"
    if [[ "$(readlink -f "$l" 2>/dev/null)" == "$(readlink -f "$d")" ]]; then ok "$l -> repo"
    elif [[ -e "$l" || -L "$l" ]]; then uy "$l zaten var ve repoya bakmiyor — dokunulmadi"
    elif ln -s "$d" "$l"; then ok "$l -> $d"
    else ha "kisayol olusmadi: $l"; fi
  done

  # ---------------------------------------------------------------------------
  bas "9/9 Kontrol"
  bash "$SETUP_DIR/kontrol.sh" || bi "kontrolde ✗ var — reboot sonrasi tekrar bak (dialout ve guc modu reboot ister)"

  # ---------------------------------------------------------------------------
  bas "OZET"
  if (( ${#HATA[@]} )); then
    echo -e "  \033[31m${#HATA[@]} hata:\033[0m"
    printf '    - %s\n' "${HATA[@]}"
    echo "  Duzeltip tekrar calistirabilirsin (hazir adimlar atlanir). Log: $LOG"
  else
    echo -e "  \033[32mKurulum hatasiz bitti.\033[0m  Log: $LOG"
  fi
  cat <<'EOF'

  Simdi:
    1) sudo reboot                        (dialout grubu + guc modu)
    2) Her acilista: sudo jetson_clocks   (swap artik otomatik acilir)
    3) ./setup/kontrol.sh                 (✗ kalmamali)
    4) Saha agi — internet isin bitince (eno1'den internet aliyorsan keser):
         sudo nmcli con add type ethernet ifname eno1 con-name hunter-net \
              ipv4.method manual ipv4.addresses 192.168.144.50/24 ipv6.method ignore
       eno1'de TEK profil kalsin: nmcli con show -> eno1'deki digerini sil:
         sudo nmcli con delete "<profil adi>"
EOF
  (( ${#HATA[@]} == 0 ))
}

main 2>&1 | tee -a "$LOG"
exit "${PIPESTATUS[0]}"
