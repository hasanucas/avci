#!/usr/bin/env bash
# ============================================================================
# SIYI kamera (192.168.144.25) icin ag route'u. Her boot/replug'da gerekir.
# ============================================================================
# DIKKAT: 192.168.144.x aginda IKI arayuz var:
#   - eno1            = .50  (SIYI VTX / hava unitesi — RTSP cikisi MK15'e)
#   - enx... (dongle) = .10  (SIYI kamera tarafi — RTSP girisi kameradan)
# Kamera (.25) dongle ile ayni subnette; route'u DOGRU arayuze (dongle) eklemeliyiz.
# Eski script "ilk 144.x arayuzu"nu aliyordu ve yanlislikla eno1'i secebiliyordu.
# Bu surum eno1'i (ve kablosuz/sanal arayuzleri) HARIC tutar.
# ============================================================================
CAM_IP="192.168.144.25"
GATEWAY="192.168.144.10"     # kamera tarafi dongle IP'si
VTX_IF="eno1"                # VTX arayuzu — kamera route'u BUNA eklenmez

# 192.168.144.x agindaki arayuzleri bul, VTX arayuzunu (eno1) HARIC tut.
# Kalan(lar)dan ilki kamera dongle'i kabul edilir.
mapfile -t CANDIDATES < <(ip -o -4 addr show | awk '/192\.168\.144\./ {print $2}' | sort -u)

IFACE=""
for c in "${CANDIDATES[@]}"; do
  [[ "$c" == "$VTX_IF" ]] && continue          # eno1'i atla (VTX)
  [[ "$c" == lo || "$c" == wl* || "$c" == docker* || "$c" == l4tbr* ]] && continue
  IFACE="$c"; break
done

if [[ -z "$IFACE" ]]; then
  echo "!!! Kamera dongle arayuzu bulunamadi (192.168.144.x, eno1 disi)."
  echo "    Mevcut 192.168.144.x arayuzler:"
  for c in "${CANDIDATES[@]}"; do echo "      $c"; done
  echo ""
  echo "    Kamera USB-Ethernet adaptoru takili mi? IP'si 192.168.144.10 mu?"
  echo "    Adaptore IP vermek icin (once 'ip -br link' ile adi bul):"
  echo "      sudo ip addr add 192.168.144.10/24 dev <ADAPTOR_ADI>"
  echo "    Sonra bu script'i tekrar calistir."
  exit 1
fi

echo ">>> Kamera dongle arayuzu: $IFACE  (VTX '$VTX_IF' haric tutuldu)"

# Kamera dongle ile kamera ayni /24'te — route teknik olarak gerekmeyebilir,
# ama eski kurulumla uyum icin acik route ekliyoruz (zaten varsa atla).
if ip route get "$CAM_IP" 2>/dev/null | grep -q "dev $IFACE"; then
  echo ">>> $CAM_IP zaten $IFACE uzerinden erisilebilir (route gerekmedi)"
elif ip route | grep -q "$CAM_IP"; then
  echo ">>> Route zaten var: $(ip route | grep "$CAM_IP")"
else
  sudo ip route add "$CAM_IP" dev "$IFACE"
  echo ">>> Route eklendi: $CAM_IP dev $IFACE"
fi

echo ">>> Kamera ping testi..."
if ping -c 2 -W 2 "$CAM_IP" >/dev/null 2>&1; then
  echo "    OK — kamera erisilebilir ($CAM_IP)"
else
  echo "    !!! Kamera ping'e cevap vermiyor. Adaptor IP'sini ($GATEWAY) ve kabloyu kontrol et."
  echo "    Kontrol: ip -4 addr show $IFACE   (192.168.144.10 atanmis mi?)"
fi
