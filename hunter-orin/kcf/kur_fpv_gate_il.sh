#!/usr/bin/env bash
# HUNTER — fpv_gate_il shim kurulumu (TEK KOMUT)
#
# hc_air.py'nin ihtiyac duydugu shim paketini dogru klasor yapisiyla olusturur.
# Indirme sirasinda iki __init__.py'nin isim cakismasi sorununu tamamen ortadan
# kaldirir — dosyalari tasimana gerek yok, bu script hepsini yazar.
#
# KULLANIM (kcf/ klasorunun icinde calistir):
#     cd ~/Desktop/hunter-orin/kcf
#     bash kur_fpv_gate_il.sh

set -e

if [ ! -f "hc_air.py" ]; then
    echo "!! hc_air.py bu klasorde yok."
    echo "   Bu script'i kcf/ icinde calistir (hc_air.py ve teacher_hc_air.yaml orada olmali)."
    exit 1
fi

mkdir -p fpv_gate_il/teacher

cat > fpv_gate_il/__init__.py << 'HUNTER_EOF_INIT'
"""
fpv_gate_il — HUNTER shim paketi.

hc_air.py orijinalinde büyük bir 'fpv_gate_il' paketinden import ediyor:
    from fpv_gate_il.features import camera_pitch_from_obs
    from fpv_gate_il.teacher.hc_air_config import cfg_float, load_hc_air_config

Bu shim o iki importu — hc_air.py'ye DOKUNMADAN — sağlar. Böylece
    python3 controller_guided.py --teacher hc_air --action-units rad_s
çalışır.

Kurulum: bu klasörü (fpv_gate_il/) kcf/ altına koy:
    kcf/
      controller_guided.py
      hc_air.py
      teacher_hc_air.yaml
      fpv_gate_il/
        __init__.py           <-- bu dosya
        features.py
        teacher/
          __init__.py
          hc_air_config.py
"""
HUNTER_EOF_INIT

cat > fpv_gate_il/teacher/__init__.py << 'HUNTER_EOF_TINIT'
# fpv_gate_il.teacher — HUNTER shim. Bilerek bos.
HUNTER_EOF_TINIT

cat > fpv_gate_il/features.py << 'HUNTER_EOF_FEAT'
"""
fpv_gate_il.features — HUNTER shim.

hc_air.py `camera_pitch_from_obs(obs, default_deg=...)` çağırıyor AMA sonucunu
KULLANMIYOR: _gate_elevation_deg ilk satırda `del camera_pitch_deg` yapıp atıyor.
Gerçek elevation cam_axis_offset_deg + pitch_transfer*pitch_deg'den (YAML) geliyor.
Yani bu fonksiyonun ne döndürdüğü önemsiz; sadece çağrının çalışması gerekiyor.

Yine de doğru davranışı taklit ediyoruz: obs[12] varsa (kamera pitch açısı)
onu döndür, yoksa default_deg. Böylece ileride hc_air bu değeri kullanmaya
başlarsa da doğru çalışır.
"""

from __future__ import annotations

def camera_pitch_from_obs(obs, default_deg: float = -15.0) -> float:
    """
    Kamera montaj pitch açısını döndürür (derece, negatif = yukarı bakıyor).

    obs[12] KASITLI OLARAK OKUNMUYOR — obs düzeni iki sistemde FARKLI:

        observation.yaml (sim) : obs[12] = camera_pitch_deg  (= -15.0)
        HUNTER controller_orin : obs[12] = alt_rel           (irtifa, metre)

    HUNTER'da obs[12]'yi okusaydık irtifayı (örn. 3.0 m) kamera açısı (3°)
    sanacaktık. O yüzden her zaman default_deg dönüyoruz; bu değer
    teacher_hc_air.yaml'daki camera.pitch_default_deg'den (-15.0) geliyor ve
    observation.yaml image.pitch_deg ile AYNI.

    Ayrıca: hc_air bu değeri zaten kullanmıyor — _gate_elevation_deg ilk
    satırında `del camera_pitch_deg` ile atıyor. Yani dönüş değeri şu an
    davranışı etkilemiyor; doğru tutmamızın sebebi ileride kullanılırsa
    sessizce bozulmaması.
    """
    del obs                      # obs[12] okunmuyor — yukarıdaki nota bak
    return float(default_deg)
HUNTER_EOF_FEAT

cat > fpv_gate_il/teacher/hc_air_config.py << 'HUNTER_EOF_CFG'
"""
fpv_gate_il.teacher.hc_air_config — HUNTER shim.

Orijinal hc_air_config.py YAML'ı `Path(__file__).parents[3]/config/...` ile
arıyordu — bu, orijinal derin ağaç yapısına (fpv_gate_il/teacher/...) göreydi.
HUNTER'ın düz kcf/ yapısında o yol tutmaz.

Bu shim YAML'ı ESNEK arar: birkaç olası konumu sırayla dener, ilk bulduğunu
kullanır. Böylece teacher_hc_air.yaml'ı kcf/ altına koyman yeterli.
"""

from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

# Bu dosya: kcf/fpv_gate_il/teacher/hc_air_config.py
# kcf/ = parents[2]
_HERE = Path(__file__).resolve()
_KCF_DIR = _HERE.parents[2]              # kcf/

# YAML'ın olabileceği yerler — sırayla denenir
_CANDIDATES = [
    _KCF_DIR / "teacher_hc_air.yaml",               # kcf/teacher_hc_air.yaml (önerilen)
    _KCF_DIR / "config" / "teacher_hc_air.yaml",    # kcf/config/teacher_hc_air.yaml
    _KCF_DIR.parent / "config" / "teacher_hc_air.yaml",  # hunter-orin/config/...
    _HERE.parent / "teacher_hc_air.yaml",           # shim yanı
]


def _find_config() -> Path:
    # Ortam degiskeni ile override (esneklik)
    env = os.environ.get("HC_AIR_YAML")
    if env:
        p = Path(env)
        if p.exists():
            return p
    for c in _CANDIDATES:
        if c.exists():
            return c
    aranan = "\n  ".join(str(c) for c in _CANDIDATES)
    raise FileNotFoundError(
        "teacher_hc_air.yaml bulunamadı. Şu konumlardan birine koy "
        f"(veya HC_AIR_YAML ortam değişkeni ile yol ver):\n  {aranan}"
    )


def _deep_get(cfg: dict[str, Any], dotted: str, default: Any = None) -> Any:
    node: Any = cfg
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


@lru_cache(maxsize=8)
def load_hc_air_config(path: str | None = None) -> dict[str, Any]:
    cfg_path = Path(path) if path is not None else _find_config()
    with cfg_path.open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    print(f"[HC_AIR] config yüklendi: {cfg_path}")
    return cfg


def cfg_float(cfg: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = _deep_get(cfg, key, default)
    return float(value)
HUNTER_EOF_CFG

echo ">>> Olusturulan dosyalar:"
find fpv_gate_il -type f | sort | sed 's/^/    /'

echo
echo ">>> Dogrulama:"
if python3 -c "import hc_air; hc_air.HCAir()" 2>&1; then
    echo "    OK — hc_air yuklenebiliyor."
else
    echo "    !! hc_air yuklenemedi. teacher_hc_air.yaml bu klasorde mi?"
    exit 1
fi
