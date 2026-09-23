#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
controller_stab.py — HUNTER / GIMBAL TESTI  (STABILIZE + RC_OVERRIDE)
=====================================================================
Tek is: bbox merkezini goruntu merkezindeki reticle uzerinde tutmak.
Yaklasma fazi YOK, faz makinesi YOK, bbox boyutu KULLANILMIYOR,
S_REF / looming / tau / hiz freni YOK.

controller_orin.py'ye DOKUNULMAZ — buradan import edilir (controller_guided.py
ile ayni desen). Ortak kullanilanlar:
    FrameClient, FCConnection, StateMachine, FlightLogger,
    axis_to_pwm, AXIS_DIR, AXIS_GAIN, SWITCH_HIGH_MIN, CAMERA_*, LOOP_HZ

---------------------------------------------------------------------------
STABILIZE, ACRO DEGILDIR — kontrolcu birebir kopyalanamaz:

  YAW  stick -> yaw HIZI.   psi_dot = K*e  =>  serbest integrator plantin
       icinde, e -> 0.   P (+ hafif D) YETER, KI = 0.

  PITCH stick -> pitch ACISI.  theta = K*(phi - theta)  =>  e_ss = e0/(1+K).
       Serbest integrator YOK. GAIN=0.2, ANGLE_MAX=30 iken K ~ 0.25 kalir,
       yani saf P hatanin ancak ~%20'sini kapatir.  PITCH'te I BASKIN.

  ROLL : gimbalde merkezleme YAPAMAZ (govde yalpalayinca kamera optik ekseni
       etrafinda doner; hedef merkez cevresinde ayni yaricapta dolanir, |e|
       kucumez).  Varsayilan GAIN_ROLL = 0.
---------------------------------------------------------------------------
GAZ (CH3) ASLA OVERRIDE EDILMEZ — pilotta kalir.
  DIKKAT: controller_orin.FCConnection.send_rc_override() CH3'e de PWM basar
  (throttle action=0 -> PWM 1500 = yarim gaz!). Bu yuzden burada CH3'u serbest
  birakan kendi gonderici kullanilir: FCConnStab.send_rpy_override().
---------------------------------------------------------------------------
ISARET KONVANSIYONU (kuru kosuda DOGRULA, sonra --rev-* ile duzelt):
  cx > 0.5  -> hedef SAGDA  -> ex > 0 -> act_yaw   > 0  (saga yaw talebi)
  cy > 0.5  -> hedef ALTTA  -> ey > 0 -> act_pitch > 0  (burun ASAGI talebi)
  Nihai PWM'i AXIS_DIR (AUTO_*_DIR) cevirir; --rev-pitch / --rev-yaw ile tersle.
  NOT: hata cx/cy'den HESAPLANIR (konvansiyonu kesin). runtracker'in ang_x/ang_y
       degerleri sadece LOGLANIR — iki kaynak CSV'de yan yana karsilastirilir.

KULLANIM:
  python3 controller_stab.py --selftest            # donanimsiz yasa testi
  python3 controller_stab.py                       # DRY-RUN (varsayilan, RC yok)
  python3 controller_stab.py --enable-override     # canli (PERVANESIZ!)
  python3 controller_stab.py --gain 0.30
"""

import argparse
import math
import sys
import time

import numpy as np

try:
    from pymavlink import mavutil
except ImportError:
    print("HATA: pymavlink yok. pip3 install --user pymavlink")
    sys.exit(1)

from controller_orin import (
    FrameClient, FCConnection, StateMachine, FlightLogger,
    axis_to_pwm, AXIS_DIR, AXIS_GAIN,
    SWITCH_HIGH_MIN, CAMERA_HOST, CAMERA_PORT, FC_ENDPOINT,
    CAMERA_HFOV, CAMERA_VFOV, LOOP_HZ,
)


# ==========================================================================
# 1) AYARLAR — tek dokunulacak yer
# ==========================================================================

class Cfg:
    # ---- CIKIS GUCU (ana knob) -------------------------------------------
    # act = GAIN * u,  u in [-1,1].  GAIN=0.20 -> action +-0.20 -> +-100 PWM.
    # STABILIZE'da 100 PWM = 0.2*ANGLE_MAX = 6 derece egim (ANGLE_MAX=30).
    GAIN_YAW = 0.20
    GAIN_PITCH = 0.20
    GAIN_ROLL = 0.00          # gimbalde merkezleme yapmaz -> kapali

    MAX_ACTION = 0.30         # sert tavan (GAIN'den bagimsiz guvenlik)
    MAX_SLEW_PER_S = 2.0      # action/saniye; bbox sicramalarina karsi
    RAMP_IN_S = 0.25          # otonoma girerken 0'dan yumusak baslangic

    # ---- PID (normalize hata [-1,1] uzerinde) ----------------------------
    KP_YAW, KI_YAW, KD_YAW = 1.00, 0.00, 0.12
    IL_YAW = 0.40
    # ---- GYRO SONUMLEME (yaw) --------------------------------------------
    # Bbox turevi yerine FC gyro'sundan sonumleme. Gyro ~1000x daha temiz VE
    # video hattinin gecikmesini tasimaz. VARSAYILAN 0 = KAPALI (davranis
    # degismez). Acik-cevrim adim testi yapilmadan ACMA.
    KR_YAW = 0.00
    RATE_SCALE_DPS = 200.0    # normalizasyon: gyro_dps / bu
    KP_PITCH, KI_PITCH, KD_PITCH = 0.50, 3.00, 0.05  # aci planti -> I baskin
    IL_PITCH = 1.00           # pitch'te cikisin tamami I'dan gelir
    KP_ROLL, KI_ROLL, KD_ROLL = 1.00, 0.00, 0.10
    IL_ROLL = 0.40

    I_DEADBAND = 0.015        # |e| bunun altinda integral islemez (surunme yok)

    # ---- FILTRE -----------------------------------------------------------
    FILT_ALPHA = 0.35         # hata EMA (~2 Hz @ 30 fps)

    # ---- ROLL DE-ROTASYONU -----------------------------------------------
    # Govde yatikken piksel hatasini ufka gore dondurur. ILK TESTTE KAPALI.
    # Temel eksen yonleri dogrulanmadan ACMA.
    DEROTATE = False
    DEROTATE_SIGN = +1

    # ---- BILGI / UYARI ----------------------------------------------------
    ANGLE_MAX_DEG = 30.0      # FC'deki ANGLE_MAX ile esitle (sadece uyari icin)


HALF_HFOV = CAMERA_HFOV * 0.5       # 40.0
HALF_VFOV = CAMERA_VFOV * 0.5       # 25.267


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ==========================================================================
# 2) KONTROL YASASI — teacher API'siyle uyumlu (reset / compute_action)
#    -> replay_compare.py ve mevcut analiz araclari calisir.
# ==========================================================================

class LowPass:
    def __init__(self, alpha):
        self.a, self.y = alpha, None

    def reset(self):
        self.y = None

    def __call__(self, x):
        self.y = x if self.y is None else self.y + self.a * (x - self.y)
        return self.y


class AxisPID:
    """normalize hata [-1,1] -> normalize efor u [-1,1]"""

    def __init__(self, kp, ki, kd, i_limit, i_db):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.i_db = i_limit, i_db
        self.reset()

    def reset(self):
        self.i = 0.0
        self.e_prev = None
        self.p = self.d = 0.0

    def update(self, e, dt):
        self.p = self.kp * e
        # turev: yeniden yakalamada kick olmasin diye ilk adimda 0
        self.d = 0.0 if self.e_prev is None else self.kd * (e - self.e_prev) / dt
        self.e_prev = e

        base = self.p + self.d
        raw = base + self.i
        saturated = abs(raw) >= 1.0 and (raw > 0) == (e > 0)
        if abs(e) > self.i_db and not saturated:      # kosullu integrasyon
            self.i = _clip(self.i + self.ki * e * dt, -self.i_limit, self.i_limit)
        return _clip(base + self.i, -1.0, 1.0)


class StabCenterTeacher:
    """
    compute_action(obs14) -> [act_roll, act_pitch, act_throttle, act_yaw]
    act_throttle DAIMA 0.0 ve zaten gonderilmez (CH3 pilotta).

    obs indeksleri controller_orin.py ile birebir:
      0 cx (0..1 normalize)  1 cy   2 w   3 h
      4 ang_x  5 ang_y (runtracker'dan — burada sadece LOGLANIR)
      6 roll_deg  7 pitch_deg  8 yaw_deg
      9..11 rate deg/s   12 alt   13 bbox_valid
    """

    DT = 1.0 / LOOP_HZ        # mpc_teacher konvansiyonu: sabit 30 Hz adim

    def __init__(self, cfg=Cfg):
        self.c = cfg
        self.pid_yaw = AxisPID(cfg.KP_YAW, cfg.KI_YAW, cfg.KD_YAW,
                               cfg.IL_YAW, cfg.I_DEADBAND)
        self.pid_pitch = AxisPID(cfg.KP_PITCH, cfg.KI_PITCH, cfg.KD_PITCH,
                                 cfg.IL_PITCH, cfg.I_DEADBAND)
        self.pid_roll = AxisPID(cfg.KP_ROLL, cfg.KI_ROLL, cfg.KD_ROLL,
                                cfg.IL_ROLL, cfg.I_DEADBAND)
        self.lp_x = LowPass(cfg.FILT_ALPHA)
        self.lp_y = LowPass(cfg.FILT_ALPHA)
        self.reset()

    def reset(self):
        for p in (self.pid_yaw, self.pid_pitch, self.pid_roll):
            p.reset()
        self.lp_x.reset()
        self.lp_y.reset()
        self.prev = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.ramp = 0.0
        self.sat_t = 0.0
        self.dbg = {}

    @staticmethod
    def _angles_from_cxcy(cx, cy):
        """cx,cy 0..1 normalize.  +ang_x = SAGDA, +ang_y = ALTTA (derece)."""
        ax = math.degrees(math.atan(
            math.tan(math.radians(HALF_HFOV)) * (cx - 0.5) * 2.0))
        ay = math.degrees(math.atan(
            math.tan(math.radians(HALF_VFOV)) * (cy - 0.5) * 2.0))
        return ax, ay

    def _shape(self, key, u, gain, dt):
        a = _clip(gain * u, -self.c.MAX_ACTION, self.c.MAX_ACTION)
        a *= self.ramp                                   # yumusak baslangic
        step = self.c.MAX_SLEW_PER_S * dt                # slew limiti
        a = self.prev[key] + _clip(a - self.prev[key], -step, step)
        self.prev[key] = a
        return a

    def compute_action(self, obs, dt=None):
        c = self.c
        dt = self.DT if dt is None else _clip(dt, 1e-3, 0.2)
        cx, cy = float(obs[0]), float(obs[1])
        roll_rad = math.radians(float(obs[6]))
        yaw_rate_dps = float(obs[11])          # ATTITUDE yawspeed, +: saga donus
        valid = float(obs[13]) > 0.5

        if not valid:
            # Normalde bu yola girilmez (kapi bbox['valid'] istiyor).
            for k in self.prev:
                self.prev[k] *= 0.5
            self.dbg = {'state': 'INVALID', 'ex': 0.0, 'ey': 0.0}
            return [self.prev['roll'], self.prev['pitch'], 0.0, self.prev['yaw']]

        self.ramp = min(1.0, self.ramp + dt / max(c.RAMP_IN_S, 1e-3))

        ang_x, ang_y = self._angles_from_cxcy(cx, cy)
        ex_raw = _clip(ang_x / HALF_HFOV, -1.2, 1.2)
        ey_raw = _clip(ang_y / HALF_VFOV, -1.2, 1.2)

        if c.DEROTATE:
            s = c.DEROTATE_SIGN * math.sin(roll_rad)
            co = math.cos(roll_rad)
            ex_raw, ey_raw = ex_raw * co - ey_raw * s, ex_raw * s + ey_raw * co

        ex = self.lp_x(ex_raw)
        ey = self.lp_y(ey_raw)

        u_yaw = self.pid_yaw.update(ex, dt)
        # gyro sonumlemesi: drone zaten saga doniyorsa saga komutu azalt
        r_yaw = -c.KR_YAW * (yaw_rate_dps / c.RATE_SCALE_DPS)
        u_yaw = _clip(u_yaw + r_yaw, -1.0, 1.0)
        u_pitch = self.pid_pitch.update(ey, dt)
        u_roll = self.pid_roll.update(ex, dt) if c.GAIN_ROLL > 0.0 else 0.0

        a_yaw = self._shape('yaw', u_yaw, c.GAIN_YAW, dt)
        a_pitch = self._shape('pitch', u_pitch, c.GAIN_PITCH, dt)
        a_roll = self._shape('roll', u_roll, c.GAIN_ROLL, dt)

        # pitch yetkisi doldu mu? (I tavanda ve hata hala buyuk)
        if abs(self.pid_pitch.i) > 0.97 * c.IL_PITCH and abs(ey) > 0.05:
            self.sat_t += dt
        else:
            self.sat_t = 0.0

        self.dbg = {
            'state': 'TRACK', 'cx': cx, 'cy': cy,
            'ang_x': ang_x, 'ang_y': ang_y,
            'ex_raw': ex_raw, 'ey_raw': ey_raw, 'ex': ex, 'ey': ey,
            'p_yaw': self.pid_yaw.p, 'd_yaw': self.pid_yaw.d,
            'i_yaw': self.pid_yaw.i,
            'p_pitch': self.pid_pitch.p, 'd_pitch': self.pid_pitch.d,
            'i_pitch': self.pid_pitch.i,
            'u_yaw': u_yaw, 'u_pitch': u_pitch, 'u_roll': u_roll,
            'r_yaw': r_yaw, 'yaw_rate': yaw_rate_dps,
            'ramp': self.ramp, 'sat_t': self.sat_t,
        }
        return [a_roll, a_pitch, 0.0, a_yaw]


# ==========================================================================
# 3) FC BAGLANTISI — controller_orin.FCConnection + 2 ekleme
#    (a) HEARTBEAT yakala -> ucus modu / arm durumu bilinsin
#    (b) CH3'u SERBEST birakan RC_OVERRIDE
# ==========================================================================

class FCConnStab(FCConnection):

    def __init__(self, endpoint):
        super().__init__(endpoint)
        self.mode_name = "?"
        self.armed = False
        self.last_hb_time = 0.0

    def poll(self):
        """Base poll() HEARTBEAT'i type filtresiyle YUTUYOR — burada ekleniyor."""
        if self.conn is None:
            return
        while True:
            msg = self.conn.recv_match(
                type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'RC_CHANNELS',
                      'HEARTBEAT'],
                blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == 'ATTITUDE':
                self.roll_deg = math.degrees(msg.roll)
                self.pitch_deg = math.degrees(msg.pitch)
                self.yaw_deg = math.degrees(msg.yaw)
                self.roll_rate = msg.rollspeed
                self.pitch_rate = msg.pitchspeed
                self.yaw_rate = msg.yawspeed
                self.last_attitude_time = time.time()
            elif t == 'GLOBAL_POSITION_INT':
                self.alt_rel = msg.relative_alt / 1000.0
            elif t == 'RC_CHANNELS':
                self.rc_channels = {
                    f'ch{i}': getattr(msg, f'chan{i}_raw') for i in range(1, 17)}
                self.last_rc_time = time.time()
            elif t == 'HEARTBEAT':
                # router'da runtracker/GCS heartbeat'i de var -> sadece autopilot
                if msg.get_srcSystem() != self.target_sys:
                    continue
                if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                try:
                    self.mode_name = mavutil.mode_string_v10(msg)
                except Exception:
                    self.mode_name = str(msg.custom_mode)
                self.armed = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.last_hb_time = time.time()

    def send_rpy_override(self, roll, pitch, yaw):
        """CH1/CH2/CH4 override. CH3 = 0 -> GAZ PILOTTA. AXIS_DIR/GAIN uygulanir."""
        if self.conn is None:
            return
        self.conn.mav.rc_channels_override_send(
            self.target_sys, self.target_comp,
            axis_to_pwm('roll', roll),
            axis_to_pwm('pitch', pitch),
            0,                                   # <<< CH3 SERBEST
            axis_to_pwm('yaw', yaw),
            0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


# ==========================================================================
# 4) DEBUG CSV (PID ici) — FlightLogger formatini bozmadan ek dosya
# ==========================================================================

class DebugLogger:
    HEADER = ['t', 'gate', 'mode_fc', 'trk', 'ramp', 'ch6', 'sm_mode',
              'cx', 'cy', 'ang_x_calc', 'ang_y_calc', 'ang_x_trk', 'ang_y_trk',
              'ex_raw', 'ey_raw', 'ex', 'ey',
              'p_yaw', 'd_yaw', 'i_yaw', 'r_yaw', 'u_yaw', 'yaw_rate', 'ol',
              'p_pitch', 'd_pitch', 'i_pitch', 'u_pitch',
              'act_roll', 'act_pitch', 'act_yaw',
              'pwm_roll', 'pwm_pitch', 'pwm_yaw', 'roll_deg', 'sat_t']

    def __init__(self, path):
        self.path = path
        self.f = open(path, 'w')
        self.f.write(','.join(self.HEADER) + '\n')
        self.n = 0

    def log(self, row):
        self.f.write(','.join(str(x) for x in row) + '\n')
        self.n += 1
        if self.n % 30 == 0:
            self.f.flush()

    def close(self):
        self.f.close()
        print(f"[LOG] debug CSV kapatildi: {self.n} satir -> {self.path}")


# ==========================================================================
# 5) MAIN
# ==========================================================================

def main():
    ap = argparse.ArgumentParser(
        description="HUNTER STABILIZE merkezleme takipcisi (gimbal testi)")
    ap.add_argument('--cam-host', default=CAMERA_HOST)
    ap.add_argument('--cam-port', type=int, default=CAMERA_PORT)
    ap.add_argument('--fc', default=FC_ENDPOINT)
    ap.add_argument('--enable-override', action='store_true',
                    help="RC_OVERRIDE'i GERCEKTEN gonder (PERVANESIZ!). "
                         "Verilmezse DRY-RUN: sadece oku+logla.")
    ap.add_argument('--no-fc', action='store_true')
    ap.add_argument('--log', default=None)
    ap.add_argument('--selftest', action='store_true')
    # cikis gucu
    ap.add_argument('--gain', type=float, default=None, help="yaw+pitch birlikte")
    ap.add_argument('--gain-yaw', type=float, default=None)
    ap.add_argument('--gain-pitch', type=float, default=None)
    ap.add_argument('--gain-roll', type=float, default=None)
    # eksen yonu (controller_orin ile ayni desen)
    ap.add_argument('--rev-roll', action='store_true')
    ap.add_argument('--rev-pitch', action='store_true')
    ap.add_argument('--rev-yaw', action='store_true')
    # kapilar
    ap.add_argument('--require-mode', default='STABILIZE',
                    help="Bu modda degilse override YOK. 'NONE' = kapiyi kapat.")
    ap.add_argument('--derotate', action='store_true',
                    help="roll de-rotasyonu ac (once eksen yonlerini dogrula!)")
    ap.add_argument('--kr-yaw', type=float, default=None,
                    help="gyro sonumleme kazanci (0 = kapali). GERCEK SONUM = GAIN x KR")
    ap.add_argument('--ki-yaw', type=float, default=None,
                    help="yaw integral kazanci. HAREKETLI hedefte geri kalmayi siler.")
    ap.add_argument('--il-yaw', type=float, default=None,
                    help="yaw integral tavani (varsayilan 0.40)")
    ap.add_argument('--kp-pitch', type=float, default=None)
    ap.add_argument('--ki-pitch', type=float, default=None,
                    help="pitch integral kazanci (varsayilan 3.00 — AGRESIF)")
    ap.add_argument('--kd-pitch', type=float, default=None)
    ap.add_argument('--il-pitch', type=float, default=None,
                    help="pitch integral tavani (varsayilan 1.00)")
    ap.add_argument('--max-action', type=float, default=None,
                    help="SERT TAVAN (varsayilan 0.30). GAIN bunu asamaz!")
    ap.add_argument('--filt-alpha', type=float, default=None,
                    help="hata EMA katsayisi (0.35). Buyutmek gecikmeyi azaltir, "
                         "gurultuyu artirir.")
    ap.add_argument('--slew', type=float, default=None,
                    help="action/saniye slew limiti (varsayilan 2.0)")
    # --- ACIK CEVRIM ADIM TESTI (plant kimliklendirme) ---
    ap.add_argument('--openloop-yaw', type=float, default=0.0,
                    help="TAKIP YOK: yaw'a kare dalga bas, plant'i olc. Orn 0.10")
    ap.add_argument('--openloop-pitch', type=float, default=0.0,
                    help="TAKIP YOK: pitch'e kare dalga bas. PITCH YETKISI YAW'DAN "
                         "COK YUKSEK — 0.03-0.05 ve --ol-on 0.5 ile BASLA.")
    ap.add_argument('--ol-on', type=float, default=1.0, help="darbe suresi (s)")
    ap.add_argument('--ol-off', type=float, default=2.0, help="bosluk suresi (s)")
    args = ap.parse_args()

    if args.gain is not None:
        Cfg.GAIN_YAW = Cfg.GAIN_PITCH = args.gain
    if args.gain_yaw is not None:
        Cfg.GAIN_YAW = args.gain_yaw
    if args.gain_pitch is not None:
        Cfg.GAIN_PITCH = args.gain_pitch
    if args.gain_roll is not None:
        Cfg.GAIN_ROLL = args.gain_roll
    if args.derotate:
        Cfg.DEROTATE = True
    if args.kr_yaw is not None:
        Cfg.KR_YAW = args.kr_yaw
    if args.ki_yaw is not None:
        Cfg.KI_YAW = args.ki_yaw
    if args.il_yaw is not None:
        Cfg.IL_YAW = args.il_yaw
    if args.kp_pitch is not None:
        Cfg.KP_PITCH = args.kp_pitch
    if args.ki_pitch is not None:
        Cfg.KI_PITCH = args.ki_pitch
    if args.kd_pitch is not None:
        Cfg.KD_PITCH = args.kd_pitch
    if args.il_pitch is not None:
        Cfg.IL_PITCH = args.il_pitch
    if args.max_action is not None:
        Cfg.MAX_ACTION = args.max_action
    if args.filt_alpha is not None:
        Cfg.FILT_ALPHA = args.filt_alpha
    if args.slew is not None:
        Cfg.MAX_SLEW_PER_S = args.slew
    OL = (abs(args.openloop_yaw) > 1e-6) or (abs(args.openloop_pitch) > 1e-6)

    if args.selftest:
        return selftest()

    if args.rev_roll:
        AXIS_DIR['roll'] *= -1
    if args.rev_pitch:
        AXIS_DIR['pitch'] *= -1
    if args.rev_yaw:
        AXIS_DIR['yaw'] *= -1

    dry_run = not args.enable_override

    print("=" * 72)
    print(" controller_stab.py — STABILIZE merkezleme (gimbal testi)")
    print("=" * 72)
    for axname in ('roll', 'pitch', 'yaw'):
        print(f" [AXIS] {axname:<6} dir={AXIS_DIR[axname]:+d} "
              f"axis_gain={AXIS_GAIN[axname]:.2f}")
    print(f" [GAIN] yaw={Cfg.GAIN_YAW:.2f} pitch={Cfg.GAIN_PITCH:.2f} "
          f"roll={Cfg.GAIN_ROLL:.2f} | DEROTATE={Cfg.DEROTATE}")
    print(f" [PTCH] KP={Cfg.KP_PITCH:.2f} KI={Cfg.KI_PITCH:.2f} "
          f"KD={Cfg.KD_PITCH:.2f} IL={Cfg.IL_PITCH:.2f}")
    print(f" [INFO] PITCH YETKISI = +-{Cfg.GAIN_PITCH * Cfg.ANGLE_MAX_DEG:.1f} deg"
          f"  (GAIN_PITCH x ANGLE_MAX)")
    print(" [INFO] Kamera 15 deg YUKARI — hedefi drone duz iken ~15 deg yukariya")
    print("        koy; dikey hata yetkiyi asarsa merkeze OTURAMAZ.")
    print(f" [YAW ] HIZ knob (GAIN x KP) = {Cfg.GAIN_YAW * Cfg.KP_YAW:.3f}")
    print(f"        FREN knob (GAIN x KR)  = {Cfg.GAIN_YAW * Cfg.KR_YAW:.3f}"
          f"   KR={Cfg.KR_YAW:.2f}")
    print(f"        INTEGRAL knob (GAIN x KI) = {Cfg.GAIN_YAW * Cfg.KI_YAW:.3f}"
          f"   KI={Cfg.KI_YAW:.2f} IL={Cfg.IL_YAW:.2f}")
    print(f"        >> GAIN degisince KR ve KI'yi TERS oranda degistir, yoksa")
    print(f"        >> FREN ve INTEGRAL knob'lari da degisir. Calisan degerler:")
    print(f"        >> HIZ 0.35 / FREN 0.42 / INTEGRAL 0.175")
    print(f"        MAX_ACTION={Cfg.MAX_ACTION:.2f}  FILT={Cfg.FILT_ALPHA:.2f}  "
          f"SLEW={Cfg.MAX_SLEW_PER_S:.1f}/s")
    if Cfg.GAIN_YAW > Cfg.MAX_ACTION:
        print(f"        [!] GAIN_YAW({Cfg.GAIN_YAW:.2f}) > MAX_ACTION"
              f"({Cfg.MAX_ACTION:.2f}) -> tavan kesiyor, --max-action ARTIR")
    if OL:
        _ax = 'yaw' if abs(args.openloop_yaw) > 1e-6 else 'PITCH'
        _am = abs(args.openloop_yaw) or abs(args.openloop_pitch)
        print(f" [OL  ] ACIK CEVRIM ADIM TESTI: {_ax}=+-{_am:.3f}, "
              f"{args.ol_on:.1f}s darbe / {args.ol_off:.1f}s bosluk. TAKIP YOK!")
        print("        Hedefi MERKEZE al, oyle basla. Kare dalga cikacak.")
    print(" [INFO] CH3 (gaz) OVERRIDE EDILMIYOR — pilotta.")
    print(" [MOD ] " + ("DRY-RUN (RC_OVERRIDE YOK)" if dry_run
                        else "!!! RC_OVERRIDE CANLI — PERVANESIZ OLMALI !!!"))
    print("=" * 72)

    teacher = StabCenterTeacher(Cfg)
    teacher.reset()

    cam = FrameClient(args.cam_host, args.cam_port)
    if not cam.connect():
        print("[!] runtracker FRAME'e baglanilamadi.")
        sys.exit(1)

    fc = None
    if not args.no_fc:
        fc = FCConnStab(args.fc)
        if not fc.connect():
            print("[!] FC'ye baglanilamadi (MAVProxy/router 14551 acik mi?).")
            sys.exit(1)
        print("[FC] STABILIZE icin FC parametresi DEGISTIRILMIYOR "
              "(setup_acro_params cagrilmadi).")

    state = StateMachine(cam)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    logger = FlightLogger(args.log or f"stab_{stamp}.csv")
    logger.open()
    dbg = DebugLogger(f"stab_dbg_{stamp}.csv")
    print(f"[LOG] debug CSV: stab_dbg_{stamp}.csv")

    period = 1.0 / LOOP_HZ
    t0 = time.time()
    last_loop = last_print = last_hb = last_warn = time.time()
    was_overriding = False
    slow_frames = 0
    ol_now = 0
    ol_t0 = None

    print("\nCalisiyor... CTRL+C ile cik.\n")
    try:
        while True:
            now = time.time()
            cam.poll()
            if fc is not None:
                fc.poll()
                if now - last_hb >= 1.0:
                    fc.send_heartbeat()
                    last_hb = now

            channels = (fc.rc_channels if fc
                        else {f'ch{i}': 1500 for i in range(1, 17)})
            state.process_rc(channels)

            bbox = cam.get_bbox()
            ch6_high = channels.get('ch6', 1500) > SWITCH_HIGH_MIN
            mode_ok = (args.require_mode.upper() == 'NONE' or fc is None
                       or fc.mode_name.upper() == args.require_mode.upper())
            gate = ((state.mode == 'autonomous') and ch6_high
                    and bbox['valid'] and mode_ok)

            if gate:
                obs = np.zeros(14, dtype=np.float32)
                obs[0] = bbox['cx']
                obs[1] = bbox['cy']
                obs[2] = bbox['w']
                obs[3] = bbox['h']
                obs[4] = bbox['ang_x']
                obs[5] = bbox['ang_y']
                obs[6] = fc.roll_deg if fc else 0.0
                obs[7] = fc.pitch_deg if fc else 0.0
                obs[8] = fc.yaw_deg if fc else 0.0
                obs[9] = math.degrees(fc.roll_rate) if fc else 0.0
                obs[10] = math.degrees(fc.pitch_rate) if fc else 0.0
                obs[11] = math.degrees(fc.yaw_rate) if fc else 0.0
                obs[12] = fc.alt_rel if fc else 0.0
                obs[13] = 1.0

                action = teacher.compute_action(obs)
                if OL:
                    # acik cevrim: takip cikisini EZ, kare dalga bas
                    if ol_t0 is None:
                        ol_t0 = now
                    ph = (now - ol_t0) % (2.0 * (args.ol_on + args.ol_off))
                    pitch_mode = abs(args.openloop_pitch) > 1e-6
                    amp = abs(args.openloop_pitch if pitch_mode
                              else args.openloop_yaw)
                    if ph < args.ol_on:
                        val, ol_now = +amp, 1
                    elif ph < args.ol_on + args.ol_off:
                        val, ol_now = 0.0, 0
                    elif ph < 2 * args.ol_on + args.ol_off:
                        val, ol_now = -amp, -1
                    else:
                        val, ol_now = 0.0, 0
                    action = ([0.0, val, 0.0, 0.0] if pitch_mode
                              else [0.0, 0.0, 0.0, val])
                pwm = (axis_to_pwm('roll', action[0]),
                       axis_to_pwm('pitch', action[1]),
                       0,                                  # CH3 serbest
                       axis_to_pwm('yaw', action[3]))
                if not dry_run and fc is not None:
                    fc.send_rpy_override(action[0], action[1], action[3])
                logger.log('autonomous' if not dry_run else 'auto_dry',
                           obs, action, pwm)
                was_overriding = True
            else:
                if state.mode == 'autonomous':
                    state.mode = 'manual'
                ol_t0 = None
                ol_now = 0
                if was_overriding:
                    if not dry_run and fc is not None:
                        fc.release_burst(5)
                    reason = ("CH6 LOW" if not ch6_high else
                              "hedef kayip" if not bbox['valid'] else
                              f"mod={fc.mode_name if fc else '?'}" if not mode_ok
                              else "state")
                    print(f"[SAFE] {time.strftime('%H:%M:%S')} OVERRIDE KESILDI "
                          f"({reason}) -> MANUAL")
                    was_overriding = False
                elif not dry_run and fc is not None:
                    fc.release_rc_override()
                teacher.reset()
                action = [0.0, 0.0, 0.0, 0.0]
                pwm = (1500, 1500, 0, 1500)

            # --- debug CSV ---
            d = teacher.dbg if gate else {}
            dbg.log([
                f"{now - t0:.4f}", int(gate),
                (fc.mode_name if fc else '-'), bbox.get('state', '-'),
                f"{d.get('ramp', 0):.3f}",
                channels.get('ch6', 0), state.mode,
                f"{bbox.get('cx', 0):.5f}", f"{bbox.get('cy', 0):.5f}",
                f"{d.get('ang_x', 0):.3f}", f"{d.get('ang_y', 0):.3f}",
                f"{bbox.get('ang_x', 0):.3f}", f"{bbox.get('ang_y', 0):.3f}",
                f"{d.get('ex_raw', 0):.5f}", f"{d.get('ey_raw', 0):.5f}",
                f"{d.get('ex', 0):.5f}", f"{d.get('ey', 0):.5f}",
                f"{d.get('p_yaw', 0):.5f}", f"{d.get('d_yaw', 0):.5f}",
                f"{d.get('i_yaw', 0):.5f}", f"{d.get('r_yaw', 0):.5f}",
                f"{d.get('u_yaw', 0):.5f}",
                f"{(math.degrees(fc.yaw_rate) if fc else 0):.3f}", int(ol_now),
                f"{d.get('p_pitch', 0):.5f}", f"{d.get('d_pitch', 0):.5f}",
                f"{d.get('i_pitch', 0):.5f}", f"{d.get('u_pitch', 0):.5f}",
                f"{action[0]:.5f}", f"{action[1]:.5f}", f"{action[3]:.5f}",
                pwm[0], pwm[1], pwm[3],
                f"{fc.roll_deg if fc else 0:.2f}", f"{d.get('sat_t', 0):.2f}",
            ])

            # --- durum yazdir ---
            if now - last_print >= 1.0:
                tag = "DRY" if dry_run else "LIVE"
                mn = fc.mode_name if fc else '-'
                arm = "ARM" if (fc and fc.armed) else "disarm"
                print(f"[{tag}|{state.mode:10s}] mod={mn:<10s} {arm} "
                      f"CH6={channels.get('ch6', 0)} "
                      f"TRK={bbox.get('state', 'idle'):8s} "
                      f"BBOX={'Y' if bbox.get('valid') else 'N'} gate={int(gate)}")
                if gate:
                    print(f"   ex={d.get('ex', 0):+.3f} ey={d.get('ey', 0):+.3f}"
                          f" | ang_calc=({d.get('ang_x', 0):+.1f},"
                          f"{d.get('ang_y', 0):+.1f})"
                          f" ang_trk=({bbox.get('ang_x', 0):+.1f},"
                          f"{bbox.get('ang_y', 0):+.1f})")
                    print(f"   {'[OL] ' if OL else ''}"
                          f"act r={action[0]:+.3f} p={action[1]:+.3f} "
                          f"y={action[3]:+.3f} -> PWM "
                          f"[{pwm[0]},{pwm[1]},CH3-serbest,{pwm[3]}]")
                    if d.get('sat_t', 0) > 1.5 and now - last_warn > 5:
                        last_warn = now
                        print(f"   [UYARI] pitch yetkisi {d['sat_t']:.1f}s dolu."
                              f" GAIN_PITCH'i artir veya hedefi boresight'a "
                              f"yaklastir.")
                if slow_frames > 5:
                    print(f"   [UYARI] {slow_frames} yavas dongu — sabit "
                          f"dt={teacher.DT:.3f}s varsayimi bozuluyor.")
                    slow_frames = 0
                last_print = now

            elapsed = time.time() - last_loop
            if elapsed > period * 1.6:
                slow_frames += 1
            time.sleep(max(0.0, period - elapsed))
            last_loop = time.time()

    except KeyboardInterrupt:
        print("\n\nCikis sinyali...")
    finally:
        if not dry_run and fc is not None:
            fc.release_burst(5)
        logger.close()
        dbg.close()
        cam.close()
        print("Kapatildi.")
    return 0


# ==========================================================================
# 6) SELFTEST — donanimsiz. P vs PI farkini ve yetki sinirini gosterir.
# ==========================================================================

def selftest():
    dt = 1.0 / LOOP_HZ
    ANGLE_MAX = Cfg.ANGLE_MAX_DEG
    YAW_RATE_MAX = 200.0      # PILOT_Y_RATE
    TAU_PITCH = 0.15          # STABILIZE aci dongusu gecikmesi
    CAM_UP = 15.0             # kamera 15 derece YUKARI

    auth = Cfg.GAIN_PITCH * ANGLE_MAX
    print(f"LOOP_HZ={LOOP_HZ}  HFOV={CAMERA_HFOV}  VFOV={CAMERA_VFOV:.3f}")
    print(f"pitch yetkisi = +-{auth:.1f} derece\n")

    for name, tgt_az, tgt_el in (
            ("yaw  12 deg", 12.0, 15.0),
            ("pitch +5 deg  (yetki ICINDE)", 0.0, 20.0),
            ("pitch +10 deg (yetki DISINDA)", 0.0, 25.0)):
        t = StabCenterTeacher(Cfg)
        t.reset()
        yaw = pitch = 0.0
        az = el = 0.0
        print(f"--- {name} ---")
        for i in range(int(6.0 / dt)):
            az = tgt_az - yaw
            el = tgt_el - (pitch + CAM_UP)
            cx = 0.5 + 0.5 * math.tan(math.radians(az)) / \
                math.tan(math.radians(HALF_HFOV))
            cy = 0.5 - 0.5 * math.tan(math.radians(el)) / \
                math.tan(math.radians(HALF_VFOV))
            obs = np.zeros(14, dtype=np.float32)
            obs[0], obs[1], obs[13] = cx, cy, 1.0
            a = t.compute_action(obs)
            yaw += a[3] * YAW_RATE_MAX * dt                        # +act = saga
            pitch += ((-a[1] * ANGLE_MAX) - pitch) * (dt / TAU_PITCH)  # +act = burun asagi
            if i % int(1.0 / dt) == 0:
                print(f"  t={i*dt:.1f}s  az={az:+6.2f}  el={el:+6.2f}  "
                      f"ex={t.dbg['ex']:+.3f} ey={t.dbg['ey']:+.3f} "
                      f"i_p={t.dbg['i_pitch']:+.3f} act_p={a[1]:+.3f}")
        print(f"  SON: az={az:+.2f}  el={el:+.2f}\n")

    print("Beklenen:")
    print("  - yaw ve yetki-icindeki pitch -> hata ~0")
    print(f"  - yetki disindaki pitch -> (istenen - {auth:.1f}) kadar kalir.")
    print("  Bu bug degil, fiziksel sinir: GAIN_PITCH x ANGLE_MAX kadar egilir.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
