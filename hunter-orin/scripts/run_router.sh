#!/usr/bin/env bash
# Terminal 1 — mavlink-router. /dev/ttyACM0'i TEK sahip olarak acar.
# FC baska bir ACM'deyse: ls /dev/ttyACM* ile bak, /etc/mavlink-router/main.conf'u duzelt.
echo ">>> /dev/ttyACM* aygitlari:"; ls -l /dev/ttyACM* 2>/dev/null || echo "  YOK — FC bagli mi?"
echo ">>> Port mesgul mu:"; sudo lsof /dev/ttyACM0 2>/dev/null && echo "  !!! TUTULUYOR" || echo "  bos"
echo ">>> mavlink-router baslatiliyor (CTRL+C ile cik)..."
exec mavlink-routerd -c /etc/mavlink-router/main.conf
