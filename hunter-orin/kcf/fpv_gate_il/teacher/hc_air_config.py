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

# NOT (31 Agu 2026): hc_air_3 icin dogru dosya teacher_hc_air_3.yaml ve
# --teacher-config ile ACIKCA verilir. Yukaridaki liste SADECE fallback'tir;
# yanlislikla ESKI teacher_hc_air.yaml'i yukleyebilir (output/acro bloklari
# YOK -> acro_rc modu calismaz). Acilista basilan "config yuklendi:" satirini
# HER ZAMAN OKU.


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
    """Teacher config'ini yukle. `hc_air:` sarmalayicisi varsa ACILIR.

    HUNTER NOTU (9 Kas 2026) — `hc_air:` ACMA EKLENDI.
    Bu shim eskiden dokumanin TAMAMINI donduruyordu. Repodaki iki yaml
    sekli var ve ikisi de kullanilir:

      DUZ      teacher_hc_air.yaml / teacher_hc_air_3.yaml
               -> en ustte dogrudan platform:, camera:, yaw: ...
      SARMALI  fpv_gate.yaml / fpv_gate_2.yaml / fpv_gate_4.yaml / fpv_gate_5.yaml
               -> en ustte image:, observation:, bridge:, hc_air:
                  teacher parametreleri `hc_air:` ALTINDA

    Sarmali dosyalarla simdiye kadar sorun cikmamasinin sebebi: hc_air_1.py
    ve hc_air_4.py KENDI load_hc_air_config'ini tanimliyor ve orada acma
    VAR. Bu shim'i import eden tek teacher hc_air_3 idi ve o DUZ dosya
    okuyor -> acma eksikligi hic ortaya cikmadi.

    hc_air_5 (arkadasin 9 Kas paketi) bu shim'i import ediyor AMA SARMALI
    fpv_gate_5.yaml okuyor. Acma olmadan `cfg["platform"]` KeyError verir;
    teacher daha ilk satirda coker. Upstream'in kendi hc_air_config'inde
    bu acma ZATEN VAR (parents[3]/config/fpv_gate.yaml sarmali bir dosya) —
    yani burada eksik olan upstream davranisiydi, yeni bir icat degil.

    GERIYE UYUMLU: duz dosyalarda `hc_air` anahtari YOK, o yuzden onlar
    aynen donmeye devam eder. acro_sender.py'nin `'acro' not in cfg`
    kontrolu de etkilenmez (teacher_hc_air_3.yaml duz ve `acro:` en ustte).
    """
    cfg_path = Path(path) if path is not None else _find_config()
    with cfg_path.open(encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if isinstance(raw, dict) and "hc_air" in raw:
        print(f"[HC_AIR] config yüklendi: {cfg_path}  (hc_air: bloğu açıldı)")
        return raw["hc_air"]
    print(f"[HC_AIR] config yüklendi: {cfg_path}")
    return raw


def cfg_float(cfg: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = _deep_get(cfg, key, default)
    return float(value)
