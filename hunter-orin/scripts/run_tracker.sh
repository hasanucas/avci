#!/usr/bin/env bash
# Terminal 2 — kcfTracker (runtracker). Router ACIK olmali. --serial KULLANMA.
# FOV: SIYI gercek FOV'u Gazebo'dan farkliysa --hfov/--vfov ile ayarla (README).
ethtool --set-eee eno1 eee off 2>/dev/null || true
RTSP="${RTSP:-rtsp://192.168.144.25:8554/main.264}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$(dirname "$SCRIPT_DIR")/kcf/build"
cd "$BUILD" || { echo "!!! build yok — once derle (README adim 5)"; exit 1; }
exec ./kcfTracker --rtsp "$RTSP" --mavlink-udp 127.0.0.1:14550 --frame-port 9999 "$@"
