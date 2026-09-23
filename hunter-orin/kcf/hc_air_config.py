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
