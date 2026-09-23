"""
fpv_gate_il.teacher.camera_geometry — HUNTER shim.

hc_air_4.py (yeni aci surumu) bu modulden SADECE
``normalized_bbox_size_at_range`` import ediyor ve onu SADECE bir yerde,
acilista `terminal.ref_width_m/ref_height_m/ref_range_m` uclusunden mutlak bir
terminal bbox esigi uretmek icin kullaniyor.

=============================================================================
NEDEN OZEL SURUM
=============================================================================
Bu hesap "hedef 0.5 x 0.5 m" varsayimi tasiyor. HUNTER'in degismez ilkesi:

    Hedef boyutu ASLA varsayilmaz. Bbox sadece yakinlik vekilidir;
    ucan sistemde mutlak mesafe/boyut hesabi YOKTUR.

Bu yuzden HUNTER'da terminal esigi geri KILITLENDI: hc_air_4 icinde
``_is_terminal`` once GORECELI kurala bakar (fpv_gate_4.yaml
``terminal.size_ratio``, kilit anindaki bbox'a orani). Mutlak yol sadece
ref_* degerleri > 0 verilirse acilir ve o zaman bile acilista GORUNUR bir
uyari basilir.

Fonksiyon burada tutuluyor cunku:
  * upstream import zinciri kirilmasin (hc_air_4 dosyasi minimal degissin),
  * ileride SITL/offline analizde (ucus SONRASI rekonstruksiyon) lazim olursa
    ayni formul elde olsun — 27 Agu vurus analizindeki 8.7 m/s ve 165-180 m
    rakamlari da bu tur bir offline hesaptan geliyordu.

Ucan sistemde bu fonksiyonun tek cagricisi, ref_* = 0 iken 0.0 dondurur ve
terminal mutlak yolu kapali kalir.
=============================================================================
"""

from __future__ import annotations

import math

# kcf/fpv_gate.yaml image bolumu ile ayni tutulur (SIYI R1 + camera_bridge).
DEFAULT_IMAGE_WIDTH = 1280.0
DEFAULT_IMAGE_HEIGHT = 720.0
DEFAULT_HFOV_DEG = 80.0


def focal_length_px(
    image_width: float = DEFAULT_IMAGE_WIDTH,
    hfov_deg: float = DEFAULT_HFOV_DEG,
) -> float:
    return image_width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))


def vfov_deg(
    image_width: float = DEFAULT_IMAGE_WIDTH,
    image_height: float = DEFAULT_IMAGE_HEIGHT,
    hfov_deg: float = DEFAULT_HFOV_DEG,
) -> float:
    aspect = float(image_width) / max(float(image_height), 1e-9)
    return math.degrees(
        2.0 * math.atan(math.tan(math.radians(hfov_deg) / 2.0) / aspect)
    )


def normalized_bbox_size_at_range(
    width_m: float,
    height_m: float,
    range_m: float,
    *,
    image_width: float = DEFAULT_IMAGE_WIDTH,
    image_height: float = DEFAULT_IMAGE_HEIGHT,
    hfov_deg: float = DEFAULT_HFOV_DEG,
) -> float:
    """Frontal dikdortgen -> normalize ``bbox_size = sqrt(bbox_w * bbox_h)``.

    Girdilerden herhangi biri <= 0 ise 0.0 doner (cagrici terminal mutlak
    yolunu KAPATIR). HUNTER varsayilani budur: fpv_gate_4.yaml icinde
    ``terminal.ref_width_m/ref_height_m/ref_range_m`` hepsi 0.0.
    """
    if (
        width_m <= 0.0
        or height_m <= 0.0
        or range_m <= 0.0
        or image_width <= 0.0
        or image_height <= 0.0
        or hfov_deg <= 0.0
    ):
        return 0.0
    fx = focal_length_px(image_width, hfov_deg)
    fy = focal_length_px(image_height, vfov_deg(image_width, image_height, hfov_deg))
    w_n = (fx * float(width_m) / float(range_m)) / float(image_width)
    h_n = (fy * float(height_m) / float(range_m)) / float(image_height)
    return math.sqrt(max(w_n * h_n, 0.0))
