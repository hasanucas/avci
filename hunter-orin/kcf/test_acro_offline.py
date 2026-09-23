#!/usr/bin/env python3
"""
HUNTER — yeni teacher'lar + ACRO yolu offline regresyon.  (31 Agu 2026)

FC YOK, pymavlink baglantisi YOK, ucus YOK. Saf hesap dogrulamasi.
test_governor_offline.py ile ayni felsefe: sahaya cikmadan once burasi
YESIL olmali.

    cd kcf && python3 test_acro_offline.py

Kapsam:
  A. Config yukleme      — dogru dosya, dogru anahtarlar, tuzaklar kapali
  B. Hedef boyutu ilkesi — hicbir yerde varsayim/mesafe hesabi yok
  C. Pitch tohum yamasi  — hc_air_4 ilk kare sicramasi YOK
  D. ACRO rate->PWM      — ArduPilot ileri yolu ile tur-donusu tutarli
  E. ACRO thrust->PWM    — hover PWM'i ve monotonluk
  F. Emniyet             — reddedilen komut kontrol dongusunu OLDURMUYOR
  G. Kanal maskesi       — CH5/6/8 override EDILMIYOR
"""

import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

FAIL = []
PASS = 0


def check(name, cond, detail=""):
    global PASS
    if cond:
        PASS += 1
        print(f"  ok   {name}")
    else:
        FAIL.append(f"{name}  {detail}")
        print(f"  FAIL {name}  {detail}")


# =========================================================================
print("\nA. CONFIG YUKLEME")
# =========================================================================
cfg3_path = HERE / "teacher_hc_air_3.yaml"
cfg4_path = HERE / "fpv_gate_4.yaml"
check("teacher_hc_air_3.yaml var", cfg3_path.exists())
check("fpv_gate_4.yaml var", cfg4_path.exists())

cfg3 = yaml.safe_load(cfg3_path.read_text(encoding="utf-8"))
cfg4 = yaml.safe_load(cfg4_path.read_text(encoding="utf-8"))["hc_air"]

check("cfg3 'acro' blogu var", "acro" in cfg3)
check("cfg3 output.mode == body_rate", cfg3["output"]["mode"] == "body_rate",
      f"(geldi: {cfg3['output']['mode']})")

# HUNTER gercek RC kalibrasyonu — mav.parm ile birebir
rc = cfg3["acro"]["rc"]
for ax in ("roll", "pitch", "throttle", "yaw"):
    check(f"acro.rc.{ax} HUNTER kalibrasyonu",
          rc[ax]["min_pwm"] == 1045 and rc[ax]["trim_pwm"] == 1495
          and rc[ax]["max_pwm"] == 1945 and rc[ax]["deadzone"] == 0,
          f"(geldi: {rc[ax]})")
check("acro.rp_expo == 0 (FC: ACRO_RP_EXPO 0)", cfg3["acro"]["rp_expo"] == 0.0)
check("acro.thr_mid_curve == 0.125 (clamp tabani)",
      abs(cfg3["acro"]["thr_mid_curve"] - 0.125) < 1e-9)

# =========================================================================
print("\nB. HEDEF BOYUTU ILKESI")
# =========================================================================
term = cfg4["terminal"]
check("terminal.size_ratio > 0 (goreceli yol acik)", term["size_ratio"] > 0)
check("terminal.ref_width_m == 0 (mutlak yol KAPALI)", term["ref_width_m"] == 0.0)
check("terminal.ref_height_m == 0", term["ref_height_m"] == 0.0)
check("terminal.ref_range_m == 0", term["ref_range_m"] == 0.0)
check("ground_floor KAPALI (FloorMonitor tek otorite)",
      cfg4["ground_floor"]["enabled"] is False)

src4 = (HERE / "hc_air_4.py").read_text(encoding="utf-8")
src3 = (HERE / "hc_air_3.py").read_text(encoding="utf-8")
for nm, s in (("hc_air_3", src3), ("hc_air_4", src4)):
    # R = f*W/bbox turu mutlak mesafe hesabi hicbir yerde olmamali
    check(f"{nm}: 'range_m' calisma aninda kullanilmiyor",
          "range_m" not in s.replace("ref_range_m", "").replace(
              "normalized_bbox_size_at_range", ""),
          "")

# =========================================================================
print("\nC. PITCH TOHUM YAMASI (hc_air_4)")
# =========================================================================
from hc_air_4 import HCAir as HCAir4          # noqa: E402

t4 = HCAir4(config_path=str(cfg4_path))
t4.reset()
check("_pitch_initialized bayragi reset'te False", t4._pitch_initialized is False)


def obs14(ang_x=0.0, ang_y=0.0, bw=0.05, bh=0.05, roll=0.0, pitch=0.0,
          yaw=0.0, valid=1.0, alt=40.0):
    o = np.zeros(14, dtype=np.float64)
    o[2], o[3] = bw, bh
    o[4], o[5] = ang_x, ang_y
    o[6], o[7], o[8] = roll, pitch, yaw
    o[12] = alt          # HUNTER: obs[12] = alt_rel (kamera pitch DEGIL)
    o[13] = valid
    return o


# Burun asagi 20 derece ile angaje ol — eski kusurda ilk kare +18..+25 sicrardi
PITCH_MEAS = -20.0
a0 = t4.compute_action(obs14(pitch=PITCH_MEAS), dt=1.0 / 30.0)
jump = float(a0[1]) - PITCH_MEAS
check("ilk kare pitch sicramasi < 2 derece", abs(jump) < 2.0,
      f"(sicrama {jump:+.2f} deg, cmd {float(a0[1]):+.2f})")
check("_pitch_initialized ilk kareden sonra True", t4._pitch_initialized is True)

# Cruise'a kendi slew'iyle yurudugunu dogrula
for _ in range(60):
    a = t4.compute_action(obs14(pitch=PITCH_MEAS), dt=1.0 / 30.0)
check("2 saniyede cruise'a dogru yurudu", float(a[1]) > float(a0[1]),
      f"(a0 {float(a0[1]):+.2f} -> a {float(a[1]):+.2f})")

# Regresyon: bayrak olmadan eski davranis geri gelir mi (kontrol testi)
t4b = HCAir4(config_path=str(cfg4_path))
t4b.reset()
t4b._pitch_initialized = True          # yamayi ATLA
a_bad = t4b.compute_action(obs14(pitch=PITCH_MEAS), dt=1.0 / 30.0)
check("yama atlanirsa sicrama GERI GELIYOR (kontrol)",
      abs(float(a_bad[1]) - PITCH_MEAS) > 5.0,
      f"(sicrama {float(a_bad[1]) - PITCH_MEAS:+.2f} deg)")

# =========================================================================
print("\nD/E. ACRO RATE + THRUST -> PWM")
# =========================================================================
from fpv_gate_il.teacher.ardupilot_acro_adapter import (   # noqa: E402
    PhysicalRateCommand,
    rp_forward_rate,
)
from fpv_gate_il.teacher.hc_air_acro import (              # noqa: E402
    HCAirAcroConverter,
    vehicle_rate_config_from_dict,
)

conv = HCAirAcroConverter(cfg3)
adapter = conv.adapter

# Tur donusu: istenen rate -> PWM -> ArduPilot ileri yolu -> temsil edilen rate
worst = 0.0
for p_dps in (-40, -20, -5, 0, 5, 20, 40):
    for q_dps in (-40, -10, 0, 10, 40):
        cmd = PhysicalRateCommand(math.radians(p_dps), math.radians(q_dps), 0.0)
        m = adapter.map_command(cmd, adapter.hover_mid_pwm())
        p_rep, q_rep, _, _ = rp_forward_rate(m.roll_pwm, m.pitch_pwm, adapter.config)
        worst = max(worst, abs(p_rep - cmd.p), abs(q_rep - cmd.q))
# TABAN FIZIKSEL: ACRO_RP_RATE=360 deg/s, RC bandi 900 PWM adimi ->
# 1 adim = 0.800 deg/s, yarim adim = 0.400 deg/s. Bunun ALTINA inilemez.
# TemporalRateQuantizer bu hatayi zamana yayiyor (tek karede degil,
# ortalamada kapaniyor) — o yuzden burada tek-kare tabani test ediliyor.
_Q_FLOOR_DPS = 2.0 * cfg3["acro"]["rp_rate_deg_s"] / (
    rc["roll"]["max_pwm"] - rc["roll"]["min_pwm"]) / 2.0
check(f"rate tur donusu tek-kare kuantizasyon tabaninda "
      f"(<= {_Q_FLOOR_DPS:.3f} deg/s)",
      math.degrees(worst) <= _Q_FLOOR_DPS + 1e-6,
      f"(en kotu {math.degrees(worst):.4f} deg/s)")

# PWM tam sayi ve bantta
m = adapter.map_command(PhysicalRateCommand(0.0, 0.0, 0.0), adapter.hover_mid_pwm())
check("notr roll PWM == trim", abs(m.roll_pwm - 1495) <= 1, f"(geldi {m.roll_pwm})")
check("notr pitch PWM == trim", abs(m.pitch_pwm - 1495) <= 1, f"(geldi {m.pitch_pwm})")
check("notr yaw PWM == trim", abs(m.yaw_pwm - 1495) <= 1, f"(geldi {m.yaw_pwm})")

# Thrust: monotonluk + hover
prev = -1.0
mono = True
for f in [i / 50.0 for i in range(51)]:
    pwm = adapter.throttle_pwm_for_thrust(f)
    thr = adapter.thrust_for_throttle_pwm(pwm)
    if thr < prev - 1e-9:
        mono = False
    prev = thr
check("thrust->PWM->thrust monoton", mono)

hover_pwm = adapter.throttle_pwm_for_thrust(0.101)
hover_back = adapter.thrust_for_throttle_pwm(hover_pwm)
check("hover 0.101 tur donusu < 0.005",
      abs(hover_back - 0.101) < 0.005,
      f"(PWM {hover_pwm}, geri {hover_back:.4f})")
check("hover PWM bantta (1045..1945)", 1045 <= hover_pwm <= 1945,
      f"(geldi {hover_pwm})")

# =========================================================================
print("\nF. EMNIYET — reddedilen komut dongusu OLDURMEMELI")
# =========================================================================
# BULGU (31 Agu 2026): MEVCUT konfigde dairesel stick reddi ULASILAMAZ.
#   p_max=q_max=1.20 rad/s, ACRO_RP_RATE=360 deg/s (6.283 rad/s)
#   -> her eksen stick 0.191, yaricap hypot = 0.270  <<  limit 0.85
# Yani emniyet agi bugun OLU KOD. Ama ACRO_RP_RATE dusurulurse
# (interception icin 120 deg/s makul) yaricap 0.810'a cikar ve 110'da
# limiti ASAR. Emniyeti O SENARYODA test ediyoruz — yoksa "test yesil"
# demek "emniyet calisiyor" demek OLMAZDI.
_cfg_tight = yaml.safe_load(cfg3_path.read_text(encoding="utf-8"))
_cfg_tight["acro"]["rp_rate_deg_s"] = 90.0      # yaricap 1.08 > 0.85
conv_tight = HCAirAcroConverter(_cfg_tight)

big = math.radians(60.0)
out = None
try:
    out = conv_tight.rates_to_rc_pwm(big, big, big, 0.101)
except Exception as e:                                     # noqa: BLE001
    check("asiri komut ISTISNA ATMIYOR", False, f"({type(e).__name__}: {e})")
else:
    check("asiri komut ISTISNA ATMIYOR", True)
    check("cikis 4 tam sayi PWM", len(out) == 4 and all(
        isinstance(v, int) and 1045 <= v <= 1945 for v in out), f"(geldi {out})")
    check("cikis 4 tam sayi PWM (dar konfig)", len(out) == 4)
    check("emniyet sayaci arti", (conv_tight.map_error_count
                                  + conv_tight.map_hold_count
                                  + conv_tight.map_neutral_count) > 0,
          f"(err={conv_tight.map_error_count} "
          f"hold={conv_tight.map_hold_count} "
          f"neutral={conv_tight.map_neutral_count})")
    check("emniyet CSV'ye yansiyor",
          "acro_map_err" in conv_tight.last_mapping_debug())

# Normal komut hala calisiyor mu (emniyetten sonra kilitlenme yok)
ok_pwm = conv_tight.rates_to_rc_pwm(math.radians(10.0), 0.0, 0.0, 0.101)
check("emniyetten sonra normal komut calisiyor",
      ok_pwm[0] > 1495, f"(roll PWM {ok_pwm[0]})")

# MEVCUT konfigde red YOK — bunu da acikca dogrula (bulgu kayit altina alinsin)
conv.rates_to_rc_pwm(math.radians(200.0), math.radians(200.0), 0.0, 0.101)
check("mevcut konfigde stick-yaricap reddi ULASILAMAZ (bulgu)",
      conv.map_error_count == 0,
      f"(err={conv.map_error_count})")

# =========================================================================
print("\nG. KANAL MASKESI — CH5/6/8 DOKUNULMAZ")
# =========================================================================
ch = adapter.pack_channels(conv._last_mapped)
check("8 kanal paketleniyor", len(ch) == 8)
check("CH5 override EDILMIYOR (65535)", ch[4] == 65535, f"(geldi {ch[4]})")
check("CH6 override EDILMIYOR (gate!)", ch[5] == 65535, f"(geldi {ch[5]})")
check("CH8 override EDILMIYOR (pencere!)", ch[7] == 65535, f"(geldi {ch[7]})")
check("CH1-4 override EDILIYOR",
      all(v != 65535 for v in ch[:4]), f"(geldi {ch[:4]})")

# =========================================================================
print("\nH. hc_air_3 body_rate yolu bozulmamis")
# =========================================================================
from hc_air_3 import HCAir as HCAir3          # noqa: E402

t3 = HCAir3(config_path=str(cfg3_path))
t3.reset()
check("hc_air_3 varsayilan output_mode body_rate",
      t3.output_mode == "body_rate", f"(geldi {t3.output_mode})")
check("hc_air_3 acro converter kurulmadi", t3._acro_converter is None)
a3 = t3.compute_action(obs14(ang_x=2.0, ang_y=-1.0, pitch=-5.0), dt=1.0 / 30.0)
check("hc_air_3 4 elemanli aksiyon", len(a3) == 4)
check("hc_air_3 cikisi sonlu", all(math.isfinite(float(v)) for v in a3),
      f"(geldi {list(map(float, a3))})")
check("hc_air_3 rate'leri makul bantta (|.| < 3 rad/s)",
      all(abs(float(a3[i])) < 3.0 for i in (0, 1, 3)),
      f"(geldi {[float(a3[i]) for i in (0,1,3)]})")

# =========================================================================
print("\n" + "=" * 62)
if FAIL:
    print(f"SONUC: {PASS} gecti, {len(FAIL)} KALDI")
    for f in FAIL:
        print("   -", f)
    sys.exit(1)
print(f"SONUC: {PASS}/{PASS} GECTI")
