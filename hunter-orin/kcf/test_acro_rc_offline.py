#!/usr/bin/env python3
"""
HUNTER — YENI ACRO yolu (--acro / hc_air_5) offline regresyon.  (9 Kas 2026)

FC YOK, pymavlink baglantisi YOK, ucus YOK. Saf hesap + siralama
dogrulamasi. test_acro_offline.py ile ayni felsefe: sahaya cikmadan once
burasi YESIL olmali.

    cd kcf && python3 test_acro_rc_offline.py

Bu betik ESKI yolu test ETMIYOR — onun testi test_acro_offline.py ve
test_acro_exit_offline.py. Ikisi de hala yesil olmali; bu dosya onlarin
yerine GECMEZ, yanlarina eklenir.

Kapsam:
  A. Paket butunlugu   — dosyalar yerinde, arkadasin dosyalari BIREBIR
  B. Config yukleme    — `hc_air:` acma, output/acro bloklari
  C. Teacher cikisi    — body_rate vs acro_rc, PWM tam sayi ve bantta
  D. Teacher bagi      — sender teacher'in converter'ini kullaniyor
  E. DIKEY SIGORTASI   — hover_thrust_firmware cozumu (SIM 0.33 tuzagi)
  F. PWM governor      — saha merdiveni PWM uzayinda calisiyor
  G. Rate zarfi        — --acro-max-rate / --max-rate TEACHER'a isliyor
  H. Kanal maskesi     — CH5/6/8 override EDILMIYOR
  I. CIKIS SIRASI      — once release, sonra mod (ve release 0, 65535 degil)
  J. Bayrak yonlendirme — --acro ile --action-units acro_pwm catismasi
  K. Shim korumasi     — HUNTER shim'leri EZILMEDI
"""

import math
import subprocess
import sys
from dataclasses import replace
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


def obs14(ang_x=0.0, ang_y=0.0, bw=0.05, bh=0.05, roll=0.0, pitch=0.0,
          yaw=0.0, valid=1.0, alt=40.0):
    o = np.zeros(14, dtype=np.float64)
    o[2], o[3] = bw, bh
    o[4], o[5] = ang_x, ang_y
    o[6], o[7], o[8] = roll, pitch, yaw
    o[12] = alt          # HUNTER: obs[12] = alt_rel (kamera pitch DEGIL)
    o[13] = valid
    return o


PRIV = {'valid': True, 'altitude': 40.0, 'vel_ned': (12.0, 0.0, 0.0)}


# =========================================================================
print("\nA. PAKET BUTUNLUGU")
# =========================================================================
for name in ("hc_air_5.py", "fpv_gate_5.yaml", "acro_rc_sender.py",
             "fpv_gate_il/teacher/los_geometry.py"):
    check(f"{name} var", (HERE / name).exists())

# Arkadasin dosyalari BIREBIR durmali. Degistirmek zorunda kalirsak bu test
# duser ve sebebini yazmaya mecbur kaliriz — sessiz sapma olmaz.
src = (HERE / "hc_air_5.py").read_text(encoding="utf-8")
check("hc_air_5 arkadasin modul yolunu kullaniyor "
      "(fpv_gate_il.teacher.hc_air_acro)",
      "from fpv_gate_il.teacher.hc_air_acro import HCAirAcroConverter" in src)
check("hc_air_5 los_geometry'den import ediyor",
      "from fpv_gate_il.teacher.los_geometry import" in src)
check("hc_air_5 features shim'inden camera_pitch_from_obs aliyor",
      "from fpv_gate_il.features import camera_pitch_from_obs" in src)


# =========================================================================
print("\nB. CONFIG YUKLEME — `hc_air:` ACMA")
# =========================================================================
cfg5_path = HERE / "fpv_gate_5.yaml"
raw5 = yaml.safe_load(cfg5_path.read_text(encoding="utf-8"))
check("fpv_gate_5.yaml SARMALI (en ustte hc_air:)", "hc_air" in raw5)

from fpv_gate_il.teacher.hc_air_config import load_hc_air_config

cfg5 = load_hc_air_config(str(cfg5_path))
check("shim `hc_air:` blogunu ACIYOR (platform en ustte)", "platform" in cfg5,
      f"(anahtarlar: {sorted(cfg5)[:6]}...)")
check("acilan blok output: tasiyor", "output" in cfg5)
check("acilan blok acro: tasiyor", "acro" in cfg5)
check("yaml output.mode == body_rate (bayrak bunu EZER)",
      cfg5["output"]["mode"] == "body_rate",
      f"(geldi: {cfg5['output']['mode']})")

# GERIYE UYUMLULUK: duz dosyalar aynen donmeye devam etmeli.
cfg3 = load_hc_air_config(str(HERE / "teacher_hc_air_3.yaml"))
check("duz teacher_hc_air_3.yaml ETKILENMEDI (platform en ustte)",
      "platform" in cfg3 and "hc_air" not in cfg3)
check("duz dosyada acro: hala gorunuyor (acro_sender kontrolu)",
      "acro" in cfg3)


# =========================================================================
print("\nC. TEACHER CIKISI — body_rate vs acro_rc")
# =========================================================================
import hc_air_5

t_br = hc_air_5.HCAir(config_path=str(cfg5_path))
t_br.reset()
check("bayraksiz output_mode = body_rate", t_br.output_mode == "body_rate",
      f"(geldi {t_br.output_mode})")
check("bayraksiz acro converter KURULMADI", t_br._acro_converter is None)
a_br = t_br.compute_action(obs14(ang_x=2.0, ang_y=-1.0, pitch=-5.0),
                           privileged=PRIV, dt=1.0 / 30.0)
check("body_rate 4 elemanli", len(a_br) == 4)
check("body_rate cikisi sonlu",
      all(math.isfinite(float(v)) for v in a_br), f"(geldi {list(map(float, a_br))})")
check("body_rate rate'leri makul bantta (|.| < 3 rad/s)",
      all(abs(float(a_br[i])) < 3.0 for i in (0, 1, 3)),
      f"(geldi {[float(a_br[i]) for i in (0,1,3)]})")

t_rc = hc_air_5.HCAir(config_path=str(cfg5_path), output_mode="acro_rc")
t_rc.reset()
check("output_mode='acro_rc' kuruldu", t_rc.output_mode == "acro_rc")
check("acro converter KURULDU", t_rc._acro_converter is not None)
check("suggested_control_backend rc_override diyor",
      hc_air_5.HCAir.suggested_control_backend(
          str(cfg5_path), output_mode="acro_rc") == "rc_override")
check("bayraksiz backend set_attitude_target",
      hc_air_5.HCAir.suggested_control_backend(str(cfg5_path))
      == "set_attitude_target")

a_rc = t_rc.compute_action(obs14(ang_x=2.0, ang_y=-1.0, pitch=-5.0),
                           privileged=PRIV, dt=1.0 / 30.0)
check("acro_rc 4 elemanli", len(a_rc) == 4)
check("acro_rc TAM SAYI PWM donduruyor",
      all(abs(float(v) - round(float(v))) < 1e-6 for v in a_rc),
      f"(geldi {list(map(float, a_rc))})")
_rcblk = cfg5["acro"]["rc"]
_lo = min(_rcblk[k]["min_pwm"] for k in ("roll", "pitch", "throttle", "yaw"))
_hi = max(_rcblk[k]["max_pwm"] for k in ("roll", "pitch", "throttle", "yaw"))
check(f"acro_rc PWM'leri [{_lo}, {_hi}] bandinda",
      all(_lo <= float(v) <= _hi for v in a_rc),
      f"(geldi {list(map(float, a_rc))})")
check("last_debug output_mode = acro_rc",
      t_rc.last_debug.get("output_mode") == "acro_rc")
for k in ("body_roll_cmd", "body_pitch_cmd", "body_throttle_cmd",
          "body_yaw_cmd"):
    check(f"last_debug['{k}'] var (CSV icin gövde komutu)",
          k in t_rc.last_debug)


# =========================================================================
print("\nD. TEACHER BAGI — sender teacher'in converter'ini kullaniyor")
# =========================================================================
from acro_rc_sender import AcroRcSender


class FakeMav:
    def __init__(self):
        self.sent = []       # ('override', (8 kanal)) kayitlari

    def rc_channels_override_send(self, sys_id, comp_id, *chans):
        self.sent.append(tuple(int(c) for c in chans))

    def command_long_send(self, *a, **k):
        pass

    def set_mode_send(self, *a, **k):
        pass

    def param_request_read_send(self, *a, **k):
        pass


class FakeConn:
    """Param DONDURMEYEN sahte FC — yaml fallback yolu kosulur."""

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMav()
        self.messages = {}
        self.mode_mapping_called = 0
        self.parm = {}

    def recv_match(self, *a, **k):
        return None

    def mode_mapping(self):
        self.mode_mapping_called += 1
        return {'ACRO': 1, 'STABILIZE': 0, 'ALT_HOLD': 2, 'LOITER': 5,
                'GUIDED_NOGPS': 20}

    def set_mode(self, m):
        pass

    def param_fetch_one(self, name):
        return None

    def wait_heartbeat(self, *a, **k):
        return None


def new_teacher():
    t = hc_air_5.HCAir(config_path=str(cfg5_path), output_mode="acro_rc")
    t.reset()
    return t


def new_sender(teacher, conn=None, **kw):
    kw.setdefault('enabled', True)
    kw.setdefault('hover_override', 0.101)
    # Sahte FC param DONDURMUYOR, yani kalibrasyon kapisi (E2) her setup()'ta
    # keserdi ve asil olcmek istedigimiz seyler (governor, kanal maskesi,
    # cikis sirasi) hic kosmazdi. Kapinin KENDISI E2'de ayrica test ediliyor;
    # burada bilerek kabul ediliyor.
    kw.setdefault('accept_yaml_cal', True)
    return AcroRcSender(conn or FakeConn(), 1, 1, teacher=teacher, **kw)


t1 = new_teacher()
s1 = new_sender(t1)
check("sender teacher'in converter'ini AYNEN kullaniyor "
      "(kopya DEGIL, referans)",
      s1.converter is t1._acro_converter)
check("sender adapter'i converter'in adapter'i",
      s1.adapter is t1._acro_converter.adapter)

# teacher= verilmezse / yanlis modda verilirse LOUD hata
try:
    AcroRcSender(FakeConn(), 1, 1)
    check("teacher= olmadan kurulum REDDEDILIYOR", False, "(istisna yok)")
except ValueError as e:
    check("teacher= olmadan kurulum REDDEDILIYOR", True)
try:
    AcroRcSender(FakeConn(), 1, 1,
                 teacher=hc_air_5.HCAir(config_path=str(cfg5_path)))
    check("body_rate teacher ile kurulum REDDEDILIYOR", False, "(istisna yok)")
except ValueError:
    check("body_rate teacher ile kurulum REDDEDILIYOR", True)


# =========================================================================
print("\nE. DIKEY SIGORTASI — hover_thrust_firmware (SIM 0.33 TUZAGI)")
# =========================================================================
# Bu bolum paketin EN TEHLIKELI tek sayisini kilitliyor. FC param okumasi
# rc kalibrasyonunu, expo'yu ve thr_mid'i duzeltiyor; hover_thrust_firmware'i
# DUZELTMIYOR (load_adapter_from_vehicle onu hic okumuyor).
check("arkadasin yaml'i 0.33 tasiyor (sim degeri)",
      abs(float(cfg5["acro"]["hover_thrust_firmware"]) - 0.33) < 1e-9,
      f"(geldi {cfg5['acro']['hover_thrust_firmware']})")

t2 = new_teacher()
s2 = new_sender(t2, hover_override=0.101)
check("setup() --hover ile GECIYOR", s2.setup() is True)
check("converter.hover_thrust_firmware --hover ile EZILDI",
      abs(s2.converter.hover_thrust_firmware - 0.101) < 1e-9,
      f"(geldi {s2.converter.hover_thrust_firmware})")

# Teacher hover'inda cikan gaz PWM'i, 0.33 ile olanin ALTINDA olmali.
hover_pwm_fixed = s2.hover_pwm
t3 = new_teacher()
s3 = new_sender(t3, hover_override=0.33)
s3.setup()
check("0.33 ile gaz PWM'i BELIRGIN daha yukari "
      f"({s3.hover_pwm} vs {hover_pwm_fixed})",
      s3.hover_pwm - hover_pwm_fixed > 100,
      f"(fark {s3.hover_pwm - hover_pwm_fixed} us)")

# Makul olmayan deger baslatmayi KESMELI (band guided_sender ile ayni).
t4 = new_teacher()
s4 = new_sender(t4, hover_override=0.90)
check("hover 0.90 (band disi) setup()'i REDDEDIYOR", s4.setup() is False)
t5 = new_teacher()
s5 = new_sender(t5, hover_override=0.01)
check("hover 0.01 (band disi) setup()'i REDDEDIYOR", s5.setup() is False)

# --hover YOKKEN: FC okunamiyor -> yaml'a duser ama GURULTULU basar.
# Deger 0.33 ve band ICINDE oldugu icin setup() GECER; emniyet "sessiz
# kalmamak". Bu testin isi o davranisi kilitlemek, yoksa birisi ileride
# sessizce gecirebilir.
t6 = new_teacher()
s6 = new_sender(t6, hover_override=None)
ok6 = s6.setup()
check("--hover yoksa yaml'a duser (band icinde oldugu icin gecer)",
      ok6 is True)
check("...ve kullanilan deger yaml'in 0.33'u",
      abs(s6.converter.hover_thrust_firmware - 0.33) < 1e-9,
      f"(geldi {s6.converter.hover_thrust_firmware})")


# =========================================================================
print("\nE2. KALIBRASYON KAPISI — okunamayan mapping ile CANLI ucus KESILIR")
# =========================================================================
# 9 Kas Orin logunun ortaya cikardigi tuzak. Ayni fiziksel istek (firmware
# thrust 0.101) iki yaml'da bambaska PWM uretiyor:
#     fpv_gate_5.yaml       (rc 1000/1500/2000, thr_mid 0.5  ) -> 1101
#     teacher_hc_air_3.yaml (rc 1045/1495/1945, thr_mid 0.125) -> 1465
# FC, gelen PWM'i KENDI kalibrasyonuyla yorumlar: 1101 -> cubuk %6.2 ->
# gercek thrust 0.0002. Hover isterken gaz NEREDEYSE KAPALI.
from fpv_gate_il.teacher.ardupilot_acro_adapter import (
    ArduPilotAcroRateAdapter, PhysicalRateLimits, RCAxisCal, VehicleRateConfig)


def _adapter(mn, tr, mx, dz, thr_mid):
    c = RCAxisCal(min_pwm=mn, trim_pwm=tr, max_pwm=mx, deadzone=dz,
                  reversed=False)
    return ArduPilotAcroRateAdapter(VehicleRateConfig(
        rp_rate_deg_s=360.0, rp_expo=0.0, yaw_rate_deg_s=202.5, yaw_expo=0.0,
        roll_cal=c, pitch_cal=c, throttle_cal=c, yaw_cal=c,
        roll_ch=1, pitch_ch=2, throttle_ch=3, yaw_ch=4,
        thr_mid_curve=thr_mid), limits=PhysicalRateLimits(1.2, 1.2, 1.0, 0.85))


_sim = _adapter(1000, 1500, 2000, 0, 0.5)      # arkadasin yaml'i
_fc = _adapter(1045, 1495, 1945, 0, 0.125)     # FC'nin gercek kalibrasyonu
_pwm_sim = _sim.throttle_pwm_for_thrust(0.101)
_pwm_fc = _fc.throttle_pwm_for_thrust(0.101)
check("sim yaml 0.101 icin 1101 uretiyor", _pwm_sim == 1101, f"({_pwm_sim})")
check("FC kalibrasyonu 0.101 icin 1465 uretiyor", _pwm_fc == 1465,
      f"({_pwm_fc})")
_actual = _fc.thrust_for_throttle_pwm(_pwm_sim)
check("sim PWM'i FC'de ~sifir thrust demek (tuzagin buyuklugu)",
      _actual < 0.01, f"(gercek thrust {_actual:.4f}, istenen 0.101)")

# CANLI + okuma basarisiz -> KESMELI
t_g1 = new_teacher()
s_g1 = AcroRcSender(FakeConn(), 1, 1, teacher=t_g1, enabled=True,
                    hover_override=0.101)
check("CANLI kosuda okunamayan mapping setup()'i KESIYOR",
      s_g1.setup() is False)

# DRY-RUN + okuma basarisiz -> GECMELI (boru hatti kossun, komut yok)
t_g2 = new_teacher()
s_g2 = AcroRcSender(FakeConn(), 1, 1, teacher=t_g2, enabled=False,
                    hover_override=0.101)
check("DRY-RUN'da ayni durum GECIYOR (gonderilen komut yok)",
      s_g2.setup() is True)

# CANLI + bilerek kabul -> GECMELI
t_g3 = new_teacher()
s_g3 = AcroRcSender(FakeConn(), 1, 1, teacher=t_g3, enabled=True,
                    hover_override=0.101, accept_yaml_cal=True)
check("--acro-accept-yaml-cal ile CANLI kosu GECIYOR (bilerek)",
      s_g3.setup() is True)

ctrl_src_e2 = (HERE / "controller_guided.py").read_text(encoding="utf-8")
check("controller --acro-accept-yaml-cal bayragini sender'a geciriyor",
      "accept_yaml_cal=args.acro_accept_yaml_cal" in ctrl_src_e2)

# Param kadansi saha dersinde kalmali (8 x 0.6, upstream 3 x 3.0 degil)
import acro_rc_sender as ARS
check("param kadansi 8 deneme (saha dersi)", ARS.ACRO_PARAM_ATTEMPTS == 8,
      f"({ARS.ACRO_PARAM_ATTEMPTS})")
check("param kadansi 0.6 s (saha dersi)",
      abs(ARS.ACRO_PARAM_TIMEOUT - 0.6) < 1e-9,
      f"({ARS.ACRO_PARAM_TIMEOUT})")


# =========================================================================
print("\nF. PWM GOVERNOR — saha merdiveni PWM uzayinda")
# =========================================================================
cfgA = None


def pwm_for(action, **kw):
    """Verilen governor ayariyla action PWM'ini gecirip gonderileni dondur."""
    t = new_teacher()
    s = new_sender(t, **kw)
    s.setup()
    s.send(action)
    return s, s.last_pwm


trim = 1500        # fpv_gate_5.yaml acro.rc trim (FC okunamadi -> yaml)
# scale 0 -> eksen trim'e OTURUR
_, pw = pwm_for([1800, 1800, 1600, 1800],
                axis_scale={'roll': 0.0, 'pitch': 1.0, 'yaw': 1.0,
                            'thrust': 1.0})
check("--scale-roll 0 roll'u trim'e oturtuyor", pw[0] == trim,
      f"(geldi {pw[0]})")
check("...pitch'e DOKUNMUYOR", pw[1] == 1800, f"(geldi {pw[1]})")
check("...yaw'a DOKUNMUYOR", pw[3] == 1800, f"(geldi {pw[3]})")

# scale 0.5 -> sapma yariya
_, pw = pwm_for([1800, 1800, 1600, 1800],
                axis_scale={'roll': 0.5, 'pitch': 1.0, 'yaw': 1.0,
                            'thrust': 1.0})
check("--scale-roll 0.5 sapmayi yariya indiriyor (1800 -> 1650)",
      pw[0] == 1650, f"(geldi {pw[0]})")

# negatif yonde de simetrik
_, pw = pwm_for([1200, 1500, 1600, 1500],
                axis_scale={'roll': 0.5, 'pitch': 1.0, 'yaw': 1.0,
                            'thrust': 1.0})
check("--scale-roll 0.5 NEGATIF yonde de simetrik (1200 -> 1350)",
      pw[0] == 1350, f"(geldi {pw[0]})")

# --scale-thrust 0 -> gaz hover PWM'ine KILIT
s, pw = pwm_for([1500, 1500, 1900, 1500],
                hover_override=0.101,
                axis_scale={'roll': 1.0, 'pitch': 1.0, 'yaw': 1.0,
                            'thrust': 0.0})
check("--scale-thrust 0 gazi HOVER PWM'ine kilitliyor",
      pw[2] == s.hover_pwm, f"(geldi {pw[2]}, hover {s.hover_pwm})")

# --thrust-cap -> gaz tavani
s, pw = pwm_for([1500, 1500, 1980, 1500],
                hover_override=0.101, thrust_cap=0.20)
cap_pwm = s.adapter.throttle_pwm_for_thrust(0.20)
check("--thrust-cap gaz PWM'ini tavanda tutuyor", pw[2] <= cap_pwm,
      f"(geldi {pw[2]}, tavan {cap_pwm})")
check("--thrust-cap sayaci arttI", s.thrust_cap_hits >= 1)

# --thrust-dev -> hover +- dev bandi
s, pw = pwm_for([1500, 1500, 1980, 1500],
                hover_override=0.101, thrust_dev=0.02)
hi = s.adapter.throttle_pwm_for_thrust(0.101 + 0.02)
check("--thrust-dev gazi hover+dev bandinda tutuyor", pw[2] <= hi,
      f"(geldi {pw[2]}, band ust {hi})")

# governor YOKKEN hicbir sey degismemeli — en onemli regresyon
s, pw = pwm_for([1712, 1388, 1601, 1455], hover_override=0.101)
check("governor yoksa PWM AYNEN gecer",
      pw == (1712, 1388, 1601, 1455), f"(geldi {pw})")
check("...ve governor sayaci 0", s.pwm_governor_hits == 0)

# PWM kelepcesi: min/max asilmaz
s, pw = pwm_for([9999, -9999, 9999, 0], hover_override=0.101)
check("PWM min/max kelepcesi tutuyor",
      all(1000 <= v <= 2000 for v in pw), f"(geldi {pw})")


# =========================================================================
print("\nG. RATE ZARFI — TEACHER'in converter'ina isliyor")
# =========================================================================
t7 = new_teacher()
s7 = new_sender(t7, hover_override=0.101, acro_max_rate_dps=20.0)
s7.setup()
lim = t7._acro_converter.adapter.limits
want = math.radians(20.0)
check("--acro-max-rate TEACHER'in zarfini daraltti (p)",
      abs(lim.p_max - want) < 1e-6, f"(geldi {lim.p_max:.4f}, istenen {want:.4f})")
check("...q", abs(lim.q_max - want) < 1e-6, f"(geldi {lim.q_max:.4f})")
check("...r", abs(lim.r_max - want) < 1e-6, f"(geldi {lim.r_max:.4f})")

# --max-rate de AYNI zarfa baglandi (sessiz etkisizlik yok)
t8 = new_teacher()
s8 = new_sender(t8, hover_override=0.101, max_rate_dps=15.0)
s8.setup()
check("--max-rate de zarfi daraltiyor (bu yolda kirpacak komut yok)",
      abs(t8._acro_converter.adapter.limits.p_max
          - math.radians(15.0)) < 1e-6,
      f"(geldi {t8._acro_converter.adapter.limits.p_max:.4f})")

# ikisi birden -> KUCUGU gecerli
t9 = new_teacher()
s9 = new_sender(t9, hover_override=0.101, acro_max_rate_dps=30.0,
                max_rate_dps=12.0)
s9.setup()
check("ikisi birden verilirse KUCUGU gecerli (12 deg/s)",
      abs(t9._acro_converter.adapter.limits.p_max
          - math.radians(12.0)) < 1e-6,
      f"(geldi {t9._acro_converter.adapter.limits.p_max:.4f})")

# Zarf GERCEKTEN kirpiyor mu: teacher'dan buyuk rate isteyip PWM'in
# satursayona oturdugunu gor.
t10 = new_teacher()
s10 = new_sender(t10, hover_override=0.101, acro_max_rate_dps=20.0)
s10.setup()
conv = t10._acro_converter
pwm_small = conv.rates_to_rc_pwm(math.radians(10.0), 0.0, 0.0, 0.101)
pwm_big = conv.rates_to_rc_pwm(math.radians(200.0), 0.0, 0.0, 0.101)
pwm_huge = conv.rates_to_rc_pwm(math.radians(400.0), 0.0, 0.0, 0.101)
check("zarf icindeki istek (10 deg/s) tavana OTURMUYOR",
      pwm_small[0] < pwm_big[0], f"(10dps {pwm_small[0]}, 200dps {pwm_big[0]})")
check("zarf ustundeki iki istek AYNI PWM'e oturuyor (kirpildi)",
      pwm_big[0] == pwm_huge[0],
      f"(200dps {pwm_big[0]}, 400dps {pwm_huge[0]})")


# =========================================================================
print("\nH. KANAL MASKESI — CH5/6/8 override EDILMIYOR")
# =========================================================================
t11 = new_teacher()
conn11 = FakeConn()
s11 = new_sender(t11, conn=conn11, hover_override=0.101)
s11.setup()
conn11.mav.sent.clear()
s11.send([1600, 1400, 1500, 1550])
check("send() TEK override paketi gonderdi", len(conn11.mav.sent) == 1,
      f"(geldi {len(conn11.mav.sent)})")
chans = conn11.mav.sent[0]
check("8 kanal alani gonderildi", len(chans) == 8, f"(geldi {len(chans)})")
check("CH1-4 surulüyor",
      chans[0] == 1600 and chans[1] == 1400 and chans[3] == 1550,
      f"(geldi {chans[:4]})")
for ch in (5, 6, 7, 8):
    check(f"CH{ch} = 65535 (YOK SAY — mod/gate/pencere kanallari)",
          chans[ch - 1] == 65535, f"(geldi {chans[ch-1]})")


# =========================================================================
print("\nI. CIKIS SIRASI — ONCE RELEASE, SONRA MOD; release 0 gonderir")
# =========================================================================
# ACRO'nun en olumcul hatasi: override birakilmadan mod degistirmek.
# ALTHOLD o PWM'i pilot cubugu sanar ve hover PWM'i orta cubugun altindaysa
# ALCALIR. Hicbir hesap testi bunu yakalayamaz — siralama testi gerekir.
t12 = new_teacher()
conn12 = FakeConn()
s12 = new_sender(t12, conn=conn12, hover_override=0.101)
s12.setup()

conn12.mav.sent.clear()
s12.release()
check("release() TEK paket gonderdi", len(conn12.mav.sent) == 1)
rel = conn12.mav.sent[0]
check("release 0 gonderiyor, 65535 DEGIL (65535 override'i TUTAR)",
      all(v == 0 for v in rel), f"(geldi {rel})")

conn12.mav.sent.clear()
s12.release_burst(5)
check("release_burst(5) BES paket gonderdi (paket kaybina karsi)",
      len(conn12.mav.sent) == 5, f"(geldi {len(conn12.mav.sent)})")

# Watchdog siralamasi — controller_guided'daki gercek sinif.
import controller_guided as CG


class OrderSender:
    """Release ve mod cagrilarinin SIRASINI kaydeder."""

    def __init__(self):
        self.calls = []

    def release_burst(self, n=5):
        self.calls.append('release')

    def set_mode_fast(self, mode, n=5):
        self.calls.append(f'mode:{mode}')


class FakeFC:
    def __init__(self, mode):
        self.current_mode = mode

    def in_auto(self):
        return self.current_mode == CG.MODE_NUM['acro']

    def mode_name(self):
        for k, v in CG.MODE_NUM.items():
            if v == self.current_mode:
                return k
        return '?'


osender = OrderSender()
wd = CG.ExitModeWatchdog(osender, FakeFC(CG.MODE_NUM['acro']),
                         'althold', acro_path=True)
wd.trigger()
check("watchdog.trigger() ONCE release, SONRA mod gonderiyor",
      osender.calls == ['release', f"mode:{CG.MODE_NUM['althold']}"],
      f"(geldi {osender.calls})")

# armed oldugu surece her donguda tekrar release
osender.calls.clear()
wd.update()
check("watchdog.update() armed iken TEKRAR release ediyor",
      osender.calls and osender.calls[0] == 'release',
      f"(geldi {osender.calls})")

# GUIDED yolunda release OLMAMALI (no-op) — eski davranis bozulmadi
g = OrderSender()
wdg = CG.ExitModeWatchdog(g, FakeFC(CG.MODE_NUM['guided_nogps']),
                          'loiter', acro_path=False)
wdg.trigger()
check("GUIDED yolunda watchdog release ETMIYOR (sadece mod)",
      g.calls == [f"mode:{CG.MODE_NUM['loiter']}"], f"(geldi {g.calls})")


# =========================================================================
print("\nJ. BAYRAK YONLENDIRME")
# =========================================================================
PY = sys.executable


def run_ctrl(args, timeout=60):
    r = subprocess.run([PY, str(HERE / "controller_guided.py")] + args,
                       cwd=str(HERE), capture_output=True, text=True,
                       timeout=timeout)
    return r.returncode, (r.stdout or "") + (r.stderr or "")

rc, out = run_ctrl(['--acro', '--action-units', 'acro_pwm'])
check("--acro + --action-units acro_pwm REDDEDILIYOR", rc != 0, f"(rc={rc})")
check("...sebebi aciklaniyor", "BIRLIKTE" in out or "Birini sec" in out)

rc, out = run_ctrl(['--acro', '--teacher', 'hc_air_5',
                    '--acro-no-tilt-comp'])
check("--acro + --acro-no-tilt-comp REDDEDILIYOR (eski yola ait)", rc != 0,
      f"(rc={rc})")

rc, out = run_ctrl(['--acro', '--teacher', 'hc_air_5', '--gimbal'])
check("--acro + --gimbal REDDEDILIYOR", rc != 0, f"(rc={rc})")

# --acro verildiginde banner ACRO yolunu BASMALI (kamera yok -> cikacak,
# ama banner mod secimi kameradan ONCE basiliyor).
rc, out = run_ctrl(['--acro', '--teacher', 'hc_air_5'])
check("--acro banner'da ACRO yazdiriyor", "ACRO KONTROLC" in out, out[:200])
check("--acro aksiyon birimini acro_rc diye basiyor", "acro_rc" in out)
check("--acro PWM uzayi/expo uyarisini basiyor",
      "PWM uzay" in out and "expo" in out.lower())

rc, out = run_ctrl([])
check("bayraksiz banner GUIDED_NOGPS (varsayilan DEGISMEDI)",
      "GUIDED_NOGPS KONTROLC" in out, "(banner yanlis)")
check("bayraksiz ACRO banner'i BASILMIYOR", "ACRO KONTROLC" not in out)

# NOT: yukaridaki alt-surec kosulari banner'a kadar gidiyor. Teacher kurulumu
# controller'da KAMERA BAGLANTISINDAN SONRA; runtracker olmadigi icin surec
# orada cikiyor. O yuzden teacher secimi/config eslemesi burada DOGRUDAN
# dogrulaniyor (alt-surecle degil) — yoksa test hicbir sey soylemezdi.
import importlib

TEACHER_OM = {'hc_air': False, 'hc_air_1': False, 'hc_air_2': False,
              'hc_air_4': False, 'hc_air_3': True, 'hc_air_5': True}
for name, accepts in TEACHER_OM.items():
    try:
        mod = importlib.import_module(name)
    except Exception as e:
        check(f"{name} import edilebiliyor", False, f"({type(e).__name__})")
        continue
    import inspect as _insp
    sig = _insp.signature(getattr(mod, 'HCAir').__init__)
    got = 'output_mode' in sig.parameters
    check(f"{name} output_mode {'KABUL EDIYOR' if accepts else 'kabul ETMIYOR'}",
          got == accepts, f"(geldi {got})")

# output_mode bilmeyen teacher GERCEKTEN TypeError atiyor mu — controller'in
# kapisi tam bu istisnaya dayaniyor.
import hc_air_1
try:
    hc_air_1.HCAir(config_path=str(HERE / 'fpv_gate.yaml'),
                   output_mode='acro_rc')
    check("hc_air_1 output_mode ile TypeError atiyor", False, "(istisna yok)")
except TypeError:
    check("hc_air_1 output_mode ile TypeError atiyor (controller kapisi "
          "buna dayaniyor)", True)

# Controller'in teacher->config eslemesi hc_air_5'i fpv_gate_5.yaml'a
# baglamis olmali.
ctrl_src = (HERE / "controller_guided.py").read_text(encoding="utf-8")
check("controller _CFG_BY_TEACHER hc_air_5 -> fpv_gate_5.yaml esliyor",
      "'hc_air_5': ['fpv_gate_5.yaml']" in ctrl_src)
check("controller --acro ile output_mode='acro_rc' geciriyor",
      "teacher_kw['output_mode'] = 'acro_rc'" in ctrl_src)
check("controller output_mode'u kurulumdan SONRA da dogruluyor",
      "_om != 'acro_rc'" in ctrl_src)
check("controller acro_rc yolunda sender'a teacher veriyor",
      "sender_kw['teacher'] = teacher" in ctrl_src)
check("controller acro_rc yolunda body=teacher.last_debug geciriyor",
      "body=teacher.last_debug" in ctrl_src)
check("controller CSV etiketi acro_rc",
      "'acro_rc' if acro_rc_path" in ctrl_src)
check("acro_rc_path ACRO emniyet kancalarini ACIYOR (acro_path'e OR'lanmis)",
      "acro_path = acro_rc_path or (args.action_units == 'acro_pwm')"
      in ctrl_src)

# hc_air_3 ile de ACRO yolu kurulabilmeli (senin saha konfigin, A/B icin).
import hc_air_3
t3b = hc_air_3.HCAir(config_path=str(HERE / 'teacher_hc_air_3.yaml'),
                     output_mode='acro_rc')
t3b.reset()
check("hc_air_3 de acro_rc modunda kurulabiliyor (A/B)",
      t3b._acro_converter is not None)
check("hc_air_3 acro konfigi SENIN saha hover'ini tasiyor (0.101)",
      abs(t3b._acro_converter.hover_thrust_firmware - 0.101) < 1e-9,
      f"(geldi {t3b._acro_converter.hover_thrust_firmware})")
s3b = AcroRcSender(FakeConn(), 1, 1, teacher=t3b, enabled=True,
                   hover_override=0.101, accept_yaml_cal=True)
check("hc_air_3 ile sender kurulumu GECIYOR", s3b.setup() is True)
# hc_air_3'un yaml'i FC ile birebir oldugu icin uretilen hover PWM 1465
# olmali — sim yaml'in 1101'i DEGIL. Iki yolun farkini kilitler.
check("hc_air_3 yolunda hover gaz PWM'i 1465 (FC kalibrasyonu)",
      s3b.hover_pwm == 1465, f"(geldi {s3b.hover_pwm})")


# =========================================================================
print("\nK. SHIM KORUMASI — HUNTER shim'leri EZILMEDI")
# =========================================================================
# Arkadasin paketi bu uc dosyanin upstream surumunu de getiriyordu.
# Uzerine yazilsa UCUS bozulurdu; bu test onlari kilitliyor.
feat = (HERE / "fpv_gate_il" / "features.py").read_text(encoding="utf-8")
check("features.py HUNTER shim'i (obs[12] OKUNMUYOR)",
      "HUNTER shim" in feat and "obs[12]" in feat)
from fpv_gate_il.features import camera_pitch_from_obs, DEFAULT_CAMERA_PITCH_DEG
_o = obs14(alt=50.0)
check("camera_pitch_from_obs obs[12]'yi (alt_rel=50) OKUMUYOR",
      abs(camera_pitch_from_obs(_o) - DEFAULT_CAMERA_PITCH_DEG) < 1e-9,
      f"(geldi {camera_pitch_from_obs(_o)}, beklenen {DEFAULT_CAMERA_PITCH_DEG})")

acro_shim = (HERE / "fpv_gate_il" / "teacher"
             / "hc_air_acro.py").read_text(encoding="utf-8")
check("hc_air_acro.py HUNTER emniyeti DURUYOR (_map_with_fallback)",
      "_map_with_fallback" in acro_shim)
check("hc_air_acro.py acro_sender giris noktasi DURUYOR (rates_to_rc_pwm)",
      "def rates_to_rc_pwm" in acro_shim)
check("hc_air_acro.py attempts/timeout DISA ACIK",
      "timeout_per_attempt" in acro_shim)

from fpv_gate_il.teacher.hc_air_acro import HCAirAcroConverter
check("converter map_* sayaclari var (CSV'ye yaziliyor)",
      all(hasattr(t_rc._acro_converter, a) for a in
          ('map_error_count', 'map_hold_count', 'map_neutral_count')))

# Emniyet GERCEKTEN calisiyor mu: dar zarf + dairesel stick reddi.
# Upstream'de bu istisna kontrol dongusunu OLDURUYORDU.
t13 = new_teacher()
s13 = new_sender(t13, hover_override=0.101, acro_max_rate_dps=5.0)
s13.setup()
c13 = t13._acro_converter
before = c13.map_error_count + c13.map_hold_count + c13.map_neutral_count
for _ in range(5):
    out13 = c13.rates_to_rc_pwm(math.radians(400.0), math.radians(400.0),
                               math.radians(400.0), 0.101)
check("dar zarfta asiri komut ISTISNA ATMIYOR (son catki tuttu)",
      all(isinstance(v, int) for v in out13), f"(geldi {out13})")
check("...ve PWM bandinda kaldi", all(1000 <= v <= 2000 for v in out13),
      f"(geldi {out13})")


# =========================================================================
print("\n" + "=" * 62)
if FAIL:
    print(f"SONUC: {PASS} gecti, {len(FAIL)} KALDI")
    for f in FAIL:
        print("   -", f)
    sys.exit(1)
print(f"SONUC: {PASS}/{PASS} GECTI")
