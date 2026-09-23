#!/usr/bin/env bash
# Terminal 1 — ROUTER (MAVProxy sürümü)
#
# NEDEN: mavlink-router (v4-16-g2362c62) bu kurulumda Orin->FC yönünü
# ROUTE EDEMIYOR. Kanit (04 Agustos):
#   * router uzerinden : uplink YOK (EXTRA1=1Hz istegi etkisiz,
#                        ATTITUDE/VFR_HUD orani 1.00 -> 1.00 degismedi)
#                        PARAM_VALUE hic gelmiyor, 3 endpoint'te de ayni
#   * dogrudan seri    : uplink CALISIYOR (oran 1.00 -> 0.10)
#                        PARAM_VALUE aninda geldi (GUID_OPTIONS=8.0)
#   * main.conf'ta filtre YOK; sysid cakismasi da YOK
#     (FC=1, runtracker=254, controller=255)
#   * router log'u "messages to unknown endpoints" sayiyor -> hedefli
#     mesajlarimiz cope gidiyor.
#
# ISTEMCI TARAFI DEGISMEZ: udpin: ile MAVProxy bu portlarda SUNUCU olur,
# runtracker/controller yine udpout ile baglanir. Port haritasi ayni.

set -u

DEV="${1:-/dev/ttyACM0}"
BAUD="${2:-115200}"

echo ">>> /dev/ttyACM* aygitlari:"
ls -l /dev/ttyACM* 2>/dev/null || echo "  YOK — FC bagli mi?"

echo ">>> Port mesgul mu:"
if sudo lsof "$DEV" 2>/dev/null; then
    echo "  !!! TUTULUYOR — once o sureci kapat (mavlink-routerd calisiyor olabilir)"
    exit 1
else
    echo "  bos"
fi

echo ">>> mavlink-routerd calisiyor mu:"
pgrep -a mavlink-routerd && {
    echo "  !!! mavlink-routerd ACIK. Once durdur:  sudo pkill mavlink-routerd"
    exit 1
} || echo "  hayir"

echo ">>> MAVProxy baslatiliyor (CTRL+C ile cik)..."
echo "    14550=runtracker  14551=brain  14552=qgc"

exec mavproxy.py \
    --master="$DEV" --baudrate="$BAUD" \
    --out=udpin:0.0.0.0:14550 \
    --out=udpin:0.0.0.0:14551 \
    --out=udpin:0.0.0.0:14552 \
    --source-system=250 \
    --streamrate=-1 \
    --daemon

# --streamrate=-1 NEDEN SART (MAVProxy kaynagindan dogrulandi):
#   mavproxy.py set_stream_rates():
#       if rate != -1 and settings.streamrate != -1:
#           master.mav.request_data_stream_send(sysid, compid,
#                                               MAV_DATA_STREAM_ALL, rate, 1)
#   Yani -1 DISINDA bir deger verilirse MAVProxy periyodik olarak TUM
#   stream'leri o hiza ceker ve istemcilerin isteklerini EZER.
#   controller_orin ATTITUDE'u 30 Hz istiyor (kontrol dongusu 30 Hz);
#   MAVProxy 10 Hz'e dusurursa teacher BAYAT attitude/gyro ile calisir.
#   -1 ile stream hizlarini istemciler yonetir (controller + runtracker),
#   hicbiri istemezse FC kendi SR* parametrelerini kullanir.
#
# NOT: mavfwd_rate ayari (varsayilan False) SET_ATTITUDE_TARGET'i ETKILEMEZ.
#      O ayar sadece REQUEST_DATA_STREAM'in cikislara yankilanmasini engeller
#      (mavproxy_link.py satir ~1104). Rate kontrolu ile ilgisi yok.
