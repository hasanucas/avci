#!/usr/bin/env bash
# Terminal 3 — controller_orin (Python beyin). Router + tracker ACIK olmali.
# VARSAYILAN: DRY-RUN (sadece oku+logla, RC_OVERRIDE YOK).
# Motor icin:  ./run_brain.sh --enable-override   (PERVANELER CIKIK!)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$(dirname "$SCRIPT_DIR")/kcf" || exit 1
exec python3 controller_orin.py "$@"
