#!/usr/bin/env python3
"""
HUNTER — ACRO cikis sirasi testi (sahte FC).  (31 Agu 2026)

FC YOK, pymavlink YOK, ucus YOK. Sahte bir MAVLink baglantisi ile
AcroSender + ExitModeWatchdog'un GONDERIM SIRASINI dogrular.

    cd kcf && python3 test_acro_exit_offline.py

NEDEN AYRI TEST
---------------
test_acro_offline.py hesabi dogruluyor (rate->PWM, thrust, kanal maskesi).
Bu dosya ZAMANLAMAYI dogruluyor: cikista once release mi gidiyor, mod mu?

Yanlis sira SESSIZ FELAKET: RC_CHANNELS_OVERRIDE moddan bagimsizdir.
ACRO'dan ALTHOLD'a gecerken override birakilmazsa, ALTHOLD o PWM'i pilot
cubugu sanar. Hover thrust 0.101'in PWM karsiligi orta cubugun ALTINDA ->
ALTHOLD alcalma komutu okur. Ekranda "mod = ALTHOLD OK" yazarken drone iner.

Bu hatayi hicbir hesap testi yakalayamaz — sadece sira testi yakalar.
"""

import sys
import types
from pathlib import Path

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


# ---------------------------------------------------------------------------
# pymavlink SAHTESI — gercek kutuphane Orin'de var, burada yok.
# Sadece AcroSender'in dokundugu yuzeyi tasiyor.
# ---------------------------------------------------------------------------
EVENTS = []          # (tur, ayrinti) — gonderim sirasi buraya yazilir

_fake = types.ModuleType("pymavlink")
_mavutil = types.ModuleType("pymavlink.mavutil")


class _MavlinkNS:
    MAV_CMD_DO_SET_MODE = 176
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1


_mavutil.mavlink = _MavlinkNS()
_fake.mavutil = _mavutil
sys.modules.setdefault("pymavlink", _fake)
sys.modules.setdefault("pymavlink.mavutil", _mavutil)


class FakeMav:
    def rc_channels_override_send(self, sys_id, comp_id, *channels):
        if all(c == 0 for c in channels):
            EVENTS.append(("release", channels))
        else:
            EVENTS.append(("override", channels))

    def command_long_send(self, sys_id, comp_id, cmd, conf, p1, p2, *rest):
        if cmd == _MavlinkNS.MAV_CMD_DO_SET_MODE:
            EVENTS.append(("mode", int(p2)))

    def param_request_read_send(self, *a, **k):
        pass


class FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()
        self.messages = {}

    def recv_match(self, **k):
        return None


# ---------------------------------------------------------------------------
print("\nA. AcroSender kurulumu (sahte FC)")
# ---------------------------------------------------------------------------
from acro_sender import AcroSender          # noqa: E402

conn = FakeConn()
sender = AcroSender(
    conn, 1, 1,
    teacher_cfg=str(HERE / "teacher_hc_air_3.yaml"),
    hover_override=0.101,
    enabled=True,
    params_set_manually=True,     # GUID param YAZIMINI atlar (okumayi DEGIL)
    acro_max_rate_dps=20.0,
)
ok = sender.setup()
check("setup() basarili", ok)
# 31 Agu duzeltmesi: params_set_manually ARTIK okumayi engellemiyor.
# Sahte FC param dondurmedigi icin yaml fallback'te kaliyoruz — DOGRU davranis.
check("params_set_manually okumayi ENGELLEMIYOR (denendi, sahte FC verdi yok)",
      sender.converter.config_source == "yaml"
      and sender.converter._vehicle_load_failed is True,
      f"(source={sender.converter.config_source}, "
      f"failed={sender.converter._vehicle_load_failed})")
check("hover 0.101 alindi", abs(sender.hover_motor - 0.101) < 1e-9,
      f"(geldi {sender.hover_motor})")
check("acro_max_rate zarfi daralttI",
      sender.converter.adapter.limits.p_max < 0.36,
      f"(p_max {sender.converter.adapter.limits.p_max:.4f} rad/s)")

# ---------------------------------------------------------------------------
print("\nA2. FC PARAM OKUMASI BASARILI YOL — sahada olacak yol")
# ---------------------------------------------------------------------------
# KAPSAM BOSLUGU (31 Agu 2026): yukaridaki sahte FC param DONDURMUYOR, yani
# hep yaml fallback yolu test ediliyordu. Sahada olacak yol ise BASARILI yol
# ve orada bir tuzak var:
#
#   try_load_from_vehicle basarili olunca  self.adapter = YENI adapter
#   kuruyor ve limits'i yaml rate_limits blogundan TAZELIYOR (1.2/1.2/1.0).
#   Yani __init__'te yaptigimiz --acro-max-rate daraltmasi SILINIYOR.
#   setup() icinde FC okumasindan SONRA tekrar daraltiyoruz — o cagri
#   yuk tasiyor. Silinirse zarf 20 deg/s yerine 68 deg/s olurdu ve bunu
#   kimse fark etmezdi: ekranda "rate zarfi daraltildi" yaziyor olurdu.
#
# Ayrica quantizer YENI adapter'a baglaniyor; daraltma ondan SONRA geliyor.
# Quantizer limits'i kopyalamiyor, adapter'a REFERANS tutuyor (map_command
# icinde self.adapter.validate_command cagiriyor) — o yuzden daraltma
# quantizer'a da geciyor. Bu test onu da kilitliyor.

FC_PARAMS = {
    # mav.parm'dan — gercek FC degerleri
    "ACRO_RP_RATE": 360.0, "ACRO_RP_EXPO": 0.0,
    "ACRO_Y_RATE": 202.5, "ACRO_Y_EXPO": 0.0,
    "RCMAP_ROLL": 1.0, "RCMAP_PITCH": 2.0,
    "RCMAP_THROTTLE": 3.0, "RCMAP_YAW": 4.0,
    "ACRO_THR_MID": 0.0, "MOT_THST_HOVER": 0.125,
}
for _c in (1, 2, 3, 4):
    FC_PARAMS[f"RC{_c}_MIN"] = 1045.0
    FC_PARAMS[f"RC{_c}_TRIM"] = 1495.0
    FC_PARAMS[f"RC{_c}_MAX"] = 1945.0
    FC_PARAMS[f"RC{_c}_DZ"] = 0.0
    FC_PARAMS[f"RC{_c}_REVERSED"] = 0.0


class _ParamMsg:
    def __init__(self, name, value):
        self.param_id = name
        self.param_value = value


class LiveMav(FakeMav):
    def __init__(self, queue):
        self.queue = queue

    def param_request_read_send(self, sys_id, comp_id, name, index):
        nm = name.decode("utf-8") if isinstance(name, bytes) else name
        if nm in FC_PARAMS:
            self.queue.append(_ParamMsg(nm, FC_PARAMS[nm]))


class LiveConn(FakeConn):
    def __init__(self):
        super().__init__()
        self.queue = []
        self.mav = LiveMav(self.queue)

    def recv_match(self, type=None, blocking=False, timeout=None):
        if type == "PARAM_VALUE" and self.queue:
            return self.queue.pop(0)
        return None


live = LiveConn()
sender_live = AcroSender(
    live, 1, 1,
    teacher_cfg=str(HERE / "teacher_hc_air_3.yaml"),
    hover_override=0.101,
    enabled=True,
    acro_max_rate_dps=20.0,
)
ok_live = sender_live.setup()
check("setup() basarili (canli param)", ok_live)
check("config kaynagi = vehicle", sender_live.converter.config_source == "vehicle",
      f"(geldi {sender_live.converter.config_source})")

_lim = sender_live.converter.adapter.limits
import math as _m
check("!! FC okumasi daraltmayi EZMEDI !!",
      abs(_lim.p_max - _m.radians(20.0)) < 1e-9
      and abs(_lim.q_max - _m.radians(20.0)) < 1e-9
      and abs(_lim.r_max - _m.radians(20.0)) < 1e-9,
      f"(p={_lim.p_max:.4f} q={_lim.q_max:.4f} r={_lim.r_max:.4f} rad/s, "
      f"beklenen {_m.radians(20.0):.4f})")

# quantizer ayni adapter'a bagli mi (limits kopyalanmadi mi)
check("quantizer daralmis adapter'a bagli",
      sender_live.converter._quantizer.adapter is sender_live.converter.adapter)

# zarf gercekten uyguluyor mu: 40 deg/s istegi 20'ye kirpilmali
EVENTS.clear()
sender_live.send([_m.radians(40.0), 0.0, 0.07, 0.0], 0.0, 0.0)
_rep = sender_live.converter._last_mapped.represented
check("40 deg/s istegi 20 deg/s'e kirpildi",
      _m.degrees(_rep.p) <= 20.5,
      f"(temsil edilen {_m.degrees(_rep.p):.2f} deg/s)")

# FC'den okunan RC kalibrasyonu yaml ile ayni cikmali (mav.parm dogrulamasi)
_c = sender_live.converter.adapter.config
check("FC RC kalibrasyonu yaml fallback ile AYNI",
      _c.roll_cal.min_pwm == 1045 and _c.roll_cal.trim_pwm == 1495
      and _c.roll_cal.max_pwm == 1945 and _c.roll_cal.deadzone == 0,
      f"(geldi {_c.roll_cal})")
check("thr_mid_curve = clamp(MOT_THST_HOVER,0.125,..) = 0.125",
      abs(_c.thr_mid_curve - 0.125) < 1e-9, f"(geldi {_c.thr_mid_curve})")


# ---------------------------------------------------------------------------
print("\nA3. PAKET DUSEN LINK — udpout dayanikliligi")
# ---------------------------------------------------------------------------
# SAHA DERSI (guided_sender.read_param yorumundan): HUNTER'in udpout linki
# param isteklerini DUSURUYOR. "MOT_THST_HOVER bir kere okunup sonraki
# calistirmada timeout verdi" -> istegi 0.6 saniyede bir tekrarlamak
# zorunda kalinmisti.
#
# Upstream fetch_parameters_strict sabit 3 deneme x 3.0 s kullaniyordu,
# yani 3 saniyede BIR soruyor. Ustelik gruplardan biri eksik kalirsa
# istisna atiliyor, TUM konfig yaml'a dusuyor ve `_vehicle_load_failed`
# yapiskan oldugu icin oturum boyunca bir daha denenmiyor.
# Tek dusen paket = butun oturum yaml fallback'inde.
#
# AcroSender artik 8 x 0.6 s geciyor. Bu test ilk N istegi kasten
# dusuren bir link ile o kadansin GERCEKTEN gectigini dogruluyor.

class FlakyMav(LiveMav):
    """Ilk `drop_first` istegi sessizce yutar."""

    def __init__(self, queue, drop_first):
        super().__init__(queue)
        self.drop_first = int(drop_first)
        self.asked = 0

    def param_request_read_send(self, sys_id, comp_id, name, index):
        self.asked += 1
        if self.asked <= self.drop_first:
            return                      # paket dustu
        super().param_request_read_send(sys_id, comp_id, name, index)


class FlakyConn(LiveConn):
    def __init__(self, drop_first):
        super().__init__()
        self.mav = FlakyMav(self.queue, drop_first)


# MEKANIK: fetch_parameters_strict her denemede TUM pending'i yeniden
# soruyor. Yani dayaniklilik "istek araligi"ndan degil DENEME SAYISINDAN
# geliyor. acro grubu 4 parametre:
#     upstream  3 deneme x 4 = 12 istek, en kotu 9.0 s
#     HUNTER    8 deneme x 4 = 32 istek, en kotu 4.8 s
# Daha cok sans, daha kisa surede. Ilk 12 istegi dusuruyoruz: upstream'in
# butun butcesi tam orada bitiyor, HUNTER 4. turda kurtariyor.
flaky = FlakyConn(drop_first=12)
sender_flaky = AcroSender(
    flaky, 1, 1,
    teacher_cfg=str(HERE / "teacher_hc_air_3.yaml"),
    hover_override=0.101, enabled=True, acro_max_rate_dps=20.0,
)
check("paket dusen linkte setup() basarili", sender_flaky.setup())
check("paket dusen linkte FC'den okundu",
      sender_flaky.converter.config_source == "vehicle",
      f"(geldi {sender_flaky.converter.config_source})")
check("dusen paketlere ragmen daraltma korundu",
      abs(sender_flaky.converter.adapter.limits.p_max - _m.radians(20.0)) < 1e-9,
      f"(p_max {sender_flaky.converter.adapter.limits.p_max:.4f})")

# Kadansin gercekten gectigini kanitla: upstream ayari (3 x 3.0) ayni
# linkte DUSMELI — yoksa test bir sey soylemiyor demektir.
flaky2 = FlakyConn(drop_first=12)
sender_upstream = AcroSender(
    flaky2, 1, 1,
    teacher_cfg=str(HERE / "teacher_hc_air_3.yaml"),
    hover_override=0.101, enabled=True, acro_max_rate_dps=20.0,
    acro_param_attempts=3, acro_param_timeout=3.0,     # UPSTREAM ayari
)
sender_upstream.setup()
check("KONTROL: upstream kadansi (3x3.0) ayni linkte yaml'a DUSUYOR",
      sender_upstream.converter.config_source == "yaml",
      f"(geldi {sender_upstream.converter.config_source} — test disi yok)")


# ---------------------------------------------------------------------------
print("\nB. send() override URETIYOR, CH5/6/8 dokunulmuyor")
# ---------------------------------------------------------------------------
EVENTS.clear()
sender.send([0.1, -0.05, 0.07, 0.02], roll_deg=5.0, pitch_deg=-10.0)
check("bir override paketi gitti",
      len(EVENTS) == 1 and EVENTS[0][0] == "override", f"({EVENTS})")
ch = EVENTS[0][1]
check("CH5 65535 (mod anahtari)", ch[4] == 65535, f"(geldi {ch[4]})")
check("CH6 65535 (gate)", ch[5] == 65535, f"(geldi {ch[5]})")
check("CH8 65535 (pencere)", ch[7] == 65535, f"(geldi {ch[7]})")
check("CH1-4 gercek PWM", all(1045 <= v <= 1945 for v in ch[:4]), f"({ch[:4]})")

# Tilt telafisi gercekten uygulaniyor mu (ACRO'da angle boost KAPALI)
check("tilt boost > 1 (pitch -10, roll 5)", sender.last_tilt_boost > 1.0,
      f"(boost {sender.last_tilt_boost:.4f})")
sender.acro_tilt_comp = False
sender.send([0.0, 0.0, 0.07, 0.0], roll_deg=5.0, pitch_deg=-10.0)
check("--acro-no-tilt-comp boost'u 1.0 yapiyor",
      abs(sender.last_tilt_boost - 1.0) < 1e-9)
sender.acro_tilt_comp = True

# ---------------------------------------------------------------------------
print("\nC. release() 0 gonderiyor, 65535 DEGIL")
# ---------------------------------------------------------------------------
EVENTS.clear()
sender.release()
check("bir release paketi gitti",
      len(EVENTS) == 1 and EVENTS[0][0] == "release", f"({EVENTS})")
check("tum kanallar 0 (65535 override'i TUTARDI)",
      all(v == 0 for v in EVENTS[0][1]), f"({EVENTS[0][1]})")

# ---------------------------------------------------------------------------
print("\nD. CIKIS SIRASI — once release, sonra mod")
# ---------------------------------------------------------------------------
import controller_guided as cg              # noqa: E402


class FakeFC:
    auto_mode = cg.MODE_ACRO
    current_mode = cg.MODE_ACRO

    def in_auto(self):
        return self.current_mode == self.auto_mode

    def mode_name(self):
        for k, v in cg.MODE_NUM.items():
            if v == self.current_mode:
                return k
        return "?"


fc = FakeFC()
wd = cg.ExitModeWatchdog(sender, fc, "althold", acro_path=True)

EVENTS.clear()
wd.trigger()
kinds = [e[0] for e in EVENTS]
check("trigger() hem release hem mod gonderdi",
      "release" in kinds and "mode" in kinds, f"({kinds})")
check("!! ILK release, SONRA mod !!",
      kinds.index("release") < kinds.index("mode"),
      f"(sira: {kinds})")
mode_vals = [e[1] for e in EVENTS if e[0] == "mode"]
check("istenen mod ALTHOLD (2)", mode_vals and mode_vals[0] == 2,
      f"(geldi {mode_vals})")

# armed iken her update tekrar birakmali (paket kaybi sigortasi)
EVENTS.clear()
wd.update()
check("armed iken update() tekrar release ediyor",
      any(e[0] == "release" for e in EVENTS), f"({[e[0] for e in EVENTS]})")

# mod tuttugunda armed duser
fc.current_mode = cg.MODE_NUM["althold"]
wd.update()
check("mod tutunca watchdog disarm", wd.armed is False)

# ---------------------------------------------------------------------------
print("\nE. Yedek zincir — birinci mod TUTMAZSA")
# ---------------------------------------------------------------------------
import time                                  # noqa: E402

fc.current_mode = cg.MODE_ACRO               # otonom modda takili kal
wd2 = cg.ExitModeWatchdog(sender, fc, "loiter", acro_path=True)
wd2.trigger()
wd2.t_stage = time.time() - (cg.EXIT_MODE_TIMEOUT + 0.1)
EVENTS.clear()
wd2.update()
kinds = [e[0] for e in EVENTS]
mode_vals = [e[1] for e in EVENTS if e[0] == "mode"]
check("yedek moda gecildi", bool(mode_vals), f"({kinds})")
check("yedek de once release ediyor",
      "release" in kinds and kinds.index("release") < kinds.index("mode"),
      f"(sira: {kinds})")
check("yedek = stabilize (0)", mode_vals and mode_vals[0] == 0,
      f"(geldi {mode_vals})")

# zincir tukendiginde ACRO yolunda yine de birakmali
wd2.stage = len(wd2.chain) - 1
wd2.t_stage = time.time() - (cg.EXIT_MODE_TIMEOUT + 0.1)
EVENTS.clear()
wd2.update()
check("zincir bitince bile release var",
      any(e[0] == "release" for e in EVENTS), f"({[e[0] for e in EVENTS]})")
check("zincir bitince disarm", wd2.armed is False)

# ---------------------------------------------------------------------------
print("\nF. GUIDED yolu BOZULMADI (regresyon)")
# ---------------------------------------------------------------------------


class FakeGuidedSender:
    def __init__(self):
        self.calls = []

    def set_mode_fast(self, m, n=5):
        self.calls.append(("mode", m))

    def release_burst(self, n=5):
        self.calls.append(("release", n))


gs = FakeGuidedSender()
fcg = FakeFC()
fcg.auto_mode = cg.MODE_GUIDED_NOGPS
fcg.current_mode = cg.MODE_GUIDED_NOGPS
wd3 = cg.ExitModeWatchdog(gs, fcg, "loiter")      # acro_path VARSAYILAN False
wd3.trigger()
check("GUIDED yolunda release HIC cagrilmiyor",
      all(c[0] != "release" for c in gs.calls), f"({gs.calls})")
check("GUIDED yolunda mod komutu gidiyor",
      any(c[0] == "mode" for c in gs.calls), f"({gs.calls})")
wd3.update()
check("GUIDED yolunda update() de release etmiyor",
      all(c[0] != "release" for c in gs.calls), f"({gs.calls})")

# ---------------------------------------------------------------------------
print("\n" + "=" * 62)
if FAIL:
    print(f"SONUC: {PASS} gecti, {len(FAIL)} KALDI")
    for f in FAIL:
        print("   -", f)
    sys.exit(1)
print(f"SONUC: {PASS}/{PASS} GECTI")
