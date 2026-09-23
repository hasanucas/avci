#!/usr/bin/env python3
"""
HUNTER — config catalisma kontrolu.  (31 Agu 2026)

    cd kcf && python3 check_config_split.py

NEDEN VAR
---------
hc_air_4 calisirken iki AYRI dosyadan ilgili sayilar okuyor:

  fpv_gate.yaml   -> `image:` blogu (features.py shim uzerinden)
                     DEFAULT_CAMERA_WIDTH / HEIGHT / HFOV_DEG
                     Bunlardan camera_vfov_deg turetiliyor ve
                     _ang_y_aim_deg icinde bbox_h ile carpiliyor
                     (irtifa nisan aciSini belirliyor).

  fpv_gate_4.yaml -> `hc_air.camera:` blogu
                     pitch_default_deg, axis_offset_deg

Bu ayrilma BILEREK: features shim'inin aday sirasi degistirilmedi ki
mevcut hc_air_1 davranisi bozulmasin. Ama bedeli su — fpv_gate.yaml'daki
`image:` blogunu degistirirsen hc_air_4 SESSIZCE etkilenir, oysa
`--teacher-config fpv_gate_4.yaml` verdigin icin oyle olmadigini
sanirsin.

Bu betik o catalismayi ARAR. Uctan uca bir test degil; uc satirlik
bir kapi. Config'lere dokunduktan sonra kos.
"""

import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
PROBLEM = []


def load(name):
    p = HERE / name
    if not p.exists():
        return None, p
    return yaml.safe_load(p.read_text(encoding="utf-8")), p


g1, p1 = load("fpv_gate.yaml")
g4, p4 = load("fpv_gate_4.yaml")

if g4 is None:
    print(f"HATA: {p4} yok.")
    sys.exit(1)

if g1 is None:
    print(f"NOT: {p1} yok — features shim fpv_gate_4.yaml'a duser, "
          "catalisma imkansiz. Kontrol gereksiz.")
    sys.exit(0)

print(f"image blogu kaynagi (features shim):  {p1}")
print(f"hc_air parametreleri:                 {p4}")
print()

img1 = (g1 or {}).get("image", {})
img4 = (g4 or {}).get("image", {})

print("image bloklari:")
for k in ("width", "height", "hfov_deg", "pitch_deg"):
    v1, v4 = img1.get(k), img4.get(k)
    same = (v1 == v4)
    print(f"  {k:10s}  fpv_gate={v1!r:>10}  fpv_gate_4={v4!r:>10}  "
          f"{'ok' if same else '<<< FARKLI'}")
    if not same:
        PROBLEM.append(
            f"image.{k}: fpv_gate.yaml={v1} ama fpv_gate_4.yaml={v4}. "
            f"hc_air_4 fpv_gate.yaml'daki {v1} degerini KULLANIR.")

# camera.pitch_default_deg, image.pitch_deg ile ayni olmali
cam4 = (g4.get("hc_air") or {}).get("camera", {})
pd = cam4.get("pitch_default_deg")
ip = img1.get("pitch_deg")
print()
print(f"kamera pitch tutarliligi:")
print(f"  fpv_gate.yaml image.pitch_deg          = {ip}")
print(f"  fpv_gate_4.yaml camera.pitch_default_deg = {pd}   <- hc_air_4 BUNU kullanir")
if pd is not None and ip is not None and abs(float(pd) - float(ip)) > 1e-9:
    PROBLEM.append(
        f"camera.pitch_default_deg={pd} ama image.pitch_deg={ip}. "
        "Kamera govdeye SABIT montaj — ikisi ayni olmali.")

# axis_offset_deg = -pitch_default_deg olmali (kayit: kamera 15 YUKARI)
ax = cam4.get("axis_offset_deg")
print(f"  fpv_gate_4.yaml camera.axis_offset_deg   = {ax}")
if pd is not None and ax is not None and abs(float(ax) + float(pd)) > 1e-9:
    PROBLEM.append(
        f"axis_offset_deg={ax}, pitch_default_deg={pd}. Kayit: kamera 15 "
        "derece YUKARI -> pitch_default_deg=-15.0 ve axis_offset_deg=+15.0.")

print()
print("=" * 62)
if PROBLEM:
    print(f"{len(PROBLEM)} CATALISMA:")
    for x in PROBLEM:
        print("  -", x)
    sys.exit(1)
print("Catalisma yok.")
