"""
fpv_gate_il.features — HUNTER shim (hc_air_1 / attitude-hold surumu icin).

hc_air_1.py sunlari import ediyor:
    DEFAULT_CAMERA_HFOV_DEG, DEFAULT_CAMERA_HEIGHT, DEFAULT_CAMERA_WIDTH,
    _camera_vfov_deg, camera_pitch_from_obs

Degerler fpv_gate.yaml'dan (image bolumu) okunur — elle senkron tutmak yok.

=============================================================================
KRITIK FARK: camera_pitch_from_obs obs[12] OKUMAZ
=============================================================================
Gercek features.py obs[12]'yi kamera pitch acisi olarak okuyor:
    pitch = float(obs[12]); if abs(pitch)<1e-6: return default; return pitch

Bu, fpv_gate.yaml obs sozlesmesine gore DOGRU (meta: camera_pitch_deg).
AMA HUNTER'in obs duzeninde obs[12] = alt_rel (IRTIFA, metre):
    controller_orin.py:  obs[12] = fc.alt_rel

hc_air_1 bu degeri _reticle_elev_deg'de KULLANIYOR (eski surum atiyordu),
ve elev dogrudan vz_des'e gidiyor: vz_des = k_pursuit * V * tan(elev).
Okusaydik:
    irtifa  0 m -> abs(0)<1e-6 -> default -15 deg  (tesadufen dogru)
    irtifa  3 m -> "kamera +3 deg ASAGI"  (dogrusu -15 YUKARI) -> 18 deg hata
    irtifa 10 m -> "+10 ASAGI"                                 -> 25 deg hata
    irtifa 50 m -> "+50 ASAGI"                                 -> 65 deg hata
18 deg hata ~8.8 m/s yanlis dikey hiz komutu demek. Ve YERDE dogru calisip
YUKSELDIKCE bozuluyor -> bench'te gorunmez, ucusta ortaya cikar.

Bu yuzden HUNTER'da obs[12] HIC OKUNMUYOR; sabit config degeri donuluyor.
Kamera gimbal'li degil, govdeye SABIT monteli — deger zaten degismez.
"""

from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

_HERE = Path(__file__).resolve()
_KCF_DIR = _HERE.parents[1]          # kcf/

_CANDIDATES = [
    _KCF_DIR / "fpv_gate.yaml",
    _KCF_DIR / "config" / "fpv_gate.yaml",
    _KCF_DIR.parent / "config" / "fpv_gate.yaml",
    _HERE.parent / "fpv_gate.yaml",
    # 31 Agu 2026: yeni yigin (hc_air_4) eski fpv_gate.yaml'in VARLIGINA
    # bagimli kalmasin diye eklendi. Buradan SADECE `image:` blogu okunur
    # (width/height/hfov_deg/pitch_deg) ve o blok iki dosyada BIREBIR AYNI —
    # teacher parametreleri buradan OKUNMAZ. Sira onemli: fpv_gate.yaml
    # varsa o kazanir, yani mevcut davranis DEGISMEZ.
    _KCF_DIR / "fpv_gate_4.yaml",
    _KCF_DIR / "config" / "fpv_gate_4.yaml",
]


@lru_cache(maxsize=1)
def load_observation_config() -> dict:
    env = os.environ.get("FPV_GATE_YAML")
    p = Path(env) if (env and Path(env).exists()) else next(
        (c for c in _CANDIDATES if c.exists()), None)
    if p is None:
        aranan = "\n  ".join(str(c) for c in _CANDIDATES)
        raise FileNotFoundError(
            "fpv_gate.yaml bulunamadi. Su konumlardan birine koy "
            f"(veya FPV_GATE_YAML ile yol ver):\n  {aranan}")
    with p.open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    print(f"[FEATURES] image config: {p}")
    return cfg


def _image_config() -> dict:
    return load_observation_config().get("image", {})


DEFAULT_CAMERA_WIDTH = int(_image_config().get("width", 1280))
DEFAULT_CAMERA_HEIGHT = int(_image_config().get("height", 720))
DEFAULT_CAMERA_HFOV_DEG = float(_image_config().get("hfov_deg", 80.0))
DEFAULT_CAMERA_PITCH_DEG = float(_image_config().get("pitch_deg", -15.0))


def _camera_vfov_deg(hfov_deg: float, aspect: float) -> float:
    """Gercek features.py ile AYNI formul (pinhole)."""
    hfov_rad = np.deg2rad(hfov_deg)
    vfov_rad = 2.0 * np.arctan(np.tan(hfov_rad / 2.0) / aspect)
    return float(np.rad2deg(vfov_rad))


def cam_axis_offset_deg(camera_pitch_deg: float = DEFAULT_CAMERA_PITCH_DEG) -> float:
    return -float(camera_pitch_deg)


def camera_pitch_from_obs(obs, *, default_deg: float = DEFAULT_CAMERA_PITCH_DEG) -> float:
    """
    Kamera montaj pitch acisi (derece, negatif = yukari).
    HUNTER'da obs[12] KASITLI OKUNMUYOR — dosya basindaki nota bak.
    """
    del obs
    return float(default_deg)
