#!/usr/bin/env python3
"""
HUNTER — ACRO + RC_CHANNELS_OVERRIDE tasima katmani.  (31 Agu 2026)

GuidedSender'in ACRO kardesi. KONTROL MANTIGI ICERMEZ. Teacher (hc_air_3,
output.mode=body_rate) rad/s govde hizi + thrust uretir; bu katman onu
ArduPilot ACRO ileri yolunu tersine cozerek TAM SAYI RC PWM'e cevirir ve
RC_CHANNELS_OVERRIDE ile gonderir.

    sender = AcroSender(conn, sys, comp, teacher_cfg='kcf/teacher_hc_air_3.yaml')
    sender.setup()          # hover + ACRO/RC paramlari (FC'den)
    sender.enter_guided()   # ACRO moduna gecer (isim GuidedSender API'si ile ayni)
    sender.send(action, roll_deg, pitch_deg)
    sender.release()        # override BIRAK — cikista SART

=============================================================================
NEDEN AYRI DOSYA, NEDEN GuidedSender'DAN TUREDI
=============================================================================
controller_guided.py sender'i su API ile kullaniyor: setup / enter_guided /
send / send_neutral / set_mode_fast / last_rates / last_thrust. AcroSender
GuidedSender'dan turedigi icin controller tarafinda sadece SENDER SECIMI
degisiyor, akis degismiyor. Governor bayraklari (--scale-*, --max-rate,
--rate-scale, --thrust-dev, --thrust-cap, --scale-thrust) MIRAS ALINAN ayni
alanlardan okunuyor -> ucuncu bir bayrak seridi OLUSMUYOR.

27 Agu devir notundaki "bayrak seritleri" tablosuna eklenen satir:

    | Bayrak                  | rate  | attitude | acro (BU DOSYA) |
    | --scale-roll/pitch/yaw  | var   | ETKISIZ  | VAR             |
    | --max-rate, --rate-scale| var   | ETKISIZ  | VAR             |
    | --att-*-max             | -     | var      | ETKISIZ         |
    | --scale-thrust/-dev/-cap| var   | var      | VAR             |

=============================================================================
ACRO'NUN GUIDED'DAN 3 GERCEK FARKI
=============================================================================
1. ANGLE BOOST KAPALI.
   mode_acro.cpp: set_throttle_out(thr, /*apply_angle_boost=*/false, ...).
   GUIDED'da acik. Yani egildikce dikey bileseni FC telafi ETMEZ; bu katman
   ekler:  motor_cmd = motor / max(cos_pitch*cos_roll, TILT_COS_MIN)
   --acro-no-tilt-comp ile kapatilir (bench).
   NOT: guided_sender teacher'in KENDI tilt_comp'unu SOYUYOR (action
   biriminde, 0.07 tabanli, yanlis olcek). Burada da soyuyoruz ve dogrusunu
   motor biriminde ekliyoruz — cift telafi YOK.

2. GAZ CUBUGU DOGRUDAN PILOT THROTTLE.
   Motor thrust -> PWM cevrimi ArduPilot'un kendi kubik egrisinden geciyor
   (pilot_thrust_for_pwm). Egri ortasi thr_mid_curve; FC'de
   clamp(MOT_THST_HOVER, 0.125, 0.6875) = 0.125 (senin gercek hover'in
   0.101 OLSA BILE clamp tabani 0.125). Cevrim tam sayi PWM tablosundan
   EN YAKIN degeri seciyor — tahmin yok.

3. FAILSAFE BASKA.
   GUID_TIMEOUT burada YOK. Orin olurse FC 3 saniye eski override'i tutar
   sonra RC FS'e duser. Bu yuzden cikista release() SART ve controller
   her cikista release_burst() atiyor.

=============================================================================
DEGISMEYEN ILKELER
=============================================================================
* Hedef boyutu varsayimi yok, mesafe hesabi yok — bu katman gorus verisine
  hic dokunmuyor.
* Tek degisken disiplini: ilk ACRO ucusunda --acro-max-rate ile dusuk tavan.
* Kanal haritasi FC'den (RCMAP_*). CH5/6/8/11/12/13 override EDILMEZ
  (pack_channels o alanlara 65535 = "yok say" yaziyor).
"""

import math

import numpy as np
from pymavlink import mavutil

from guided_sender import (
    GuidedSender,
    HOVER_ACTION,
    HOVER_AP_FLOOR,
    HOVER_SANE_MAX,
    HOVER_SANE_MIN,
    MODE_ACRO,
    TILT_COS_MIN,
)
from fpv_gate_il.teacher.hc_air_acro import HCAirAcroConverter
from fpv_gate_il.teacher.hc_air_config import load_hc_air_config

# ACRO'da action birimi -> motor birimi egimi. guided_sender RATE yolundaki
# THRUST_GAIN (0.30) ile AYNI tutuldu: teacher ayni teacher, dikey kanalin
# anlami degismedi. Tek fark angle boost telafisi (asagida, ayri adim).
THRUST_GAIN_ACRO = 0.30


class AcroSender(GuidedSender):
    """ACRO + RC_CHANNELS_OVERRIDE. Kontrol mantigi YOK."""

    def __init__(self, conn, target_sys, target_comp,
                 teacher_cfg=None,
                 acro_tilt_comp=True,
                 acro_max_rate_dps=None,
                 acro_param_attempts=8,
                 acro_param_timeout=0.6,
                 **kwargs):
        """
        teacher_cfg      : teacher_hc_air_3.yaml yolu (acro: blogu icin).
                           None ise hc_air_config shim'i arar.
        acro_tilt_comp   : ACRO'da angle boost kapali oldugu icin dikey
                           telafiyi BU KATMAN ekler. False = bench.
        acro_max_rate_dps: adapter'in p/q/r kelepcesini deg/s cinsinden
                           daraltir (--acro-max-rate). --max-rate'ten FARKLI:
                           --max-rate teacher komutunu kirpar, bu adapter'in
                           fiziksel zarfini daraltir. Ikisi birlikte calisir.
        """
        kwargs.setdefault('action_units', 'rad_s')
        kwargs.setdefault('thrust_gain', THRUST_GAIN_ACRO)
        if kwargs['action_units'] != 'rad_s':
            raise ValueError(
                "AcroSender sadece action_units='rad_s' ile calisir "
                f"(geldi: {kwargs['action_units']}). ACRO yolu govde hizi "
                "bekler; attitude_deg teacher'lari GUIDED ile ucar.")
        super().__init__(conn, target_sys, target_comp, **kwargs)

        self.acro_tilt_comp = bool(acro_tilt_comp)
        self.acro_param_attempts = int(acro_param_attempts)
        self.acro_param_timeout = float(acro_param_timeout)
        self.acro_max_rate_dps = acro_max_rate_dps
        self.teacher_cfg = teacher_cfg

        cfg = load_hc_air_config(teacher_cfg)
        if 'acro' not in cfg:
            raise KeyError(
                "teacher config'inde 'acro:' blogu YOK. hc_air_3 icin dogru "
                "dosya teacher_hc_air_3.yaml — ESKI teacher_hc_air.yaml'da "
                "bu blok bulunmuyor. --teacher-config ile dogrusunu ver.")
        self.converter = HCAirAcroConverter(cfg)

        if self.acro_max_rate_dps is not None:
            self._narrow_rate_limits(float(self.acro_max_rate_dps))

        # Loglama / durum
        self.last_pwm = (0, 0, 0, 0)
        self.last_tilt_boost = 1.0
        self.last_attitude = (0.0, 0.0, 0.0)   # controller uyumlulugu
        self.override_active = False

    # ------------------------------------------------------------------
    # KURULUM
    # ------------------------------------------------------------------

    def _narrow_rate_limits(self, dps):
        """--acro-max-rate: adapter fiziksel zarfini daralt (deg/s)."""
        from fpv_gate_il.teacher.ardupilot_acro_adapter import PhysicalRateLimits
        lim = self.converter.adapter.limits
        r = math.radians(float(dps))
        self.converter.adapter.limits = PhysicalRateLimits(
            p_max=min(lim.p_max, r),
            q_max=min(lim.q_max, r),
            r_max=min(lim.r_max, r),
            max_rp_stick_radius=lim.max_rp_stick_radius,
        )
        print(f"[ACRO] rate zarfi daraltildi: p/q/r <= {dps:.1f} deg/s "
              f"({r:.3f} rad/s)")

    def setup(self):
        """Hover'i belirle + ACRO/RC paramlarini FC'den yukle."""
        print("[ACRO] Kurulum basliyor...")

        # --- 1. Hover thrust (guided_sender ile AYNI kapilar) ---
        if self.hover_override is not None:
            self.hover_motor = float(self.hover_override)
            print(f"[ACRO] hover = {self.hover_motor:.3f}  (--hover ile ELLE)")
        else:
            val = self.read_param('MOT_THST_HOVER')
            if val is None:
                print("[ACRO] HATA: MOT_THST_HOVER okunamadi. --hover ile ver.")
                return False
            self.hover_motor = float(val)
            print(f"[ACRO] hover = {self.hover_motor:.3f}  (FC'den)")

        if not (HOVER_SANE_MIN <= self.hover_motor <= HOVER_SANE_MAX):
            print(f"[ACRO] HATA: hover={self.hover_motor:.3f} makul bandin "
                  f"({HOVER_SANE_MIN}-{HOVER_SANE_MAX}) DISINDA.")
            return False
        if self.hover_override is None and abs(self.hover_motor - 0.35) < 1e-4:
            print("[ACRO] HATA: MOT_THST_HOVER = 0.350 = OGRENILMEMIS "
                  "fabrika degeri. AltHold'da ogren ya da --hover ile ver.")
            return False
        if abs(self.hover_motor - HOVER_AP_FLOOR) < 1e-4:
            print(f"[ACRO] NOT: hover = {HOVER_AP_FLOOR} = ArduPilot ALT "
                  "TABANI. Gercek hover bunun ALTINDA olabilir "
                  "(saha olcumu 0.101-0.102). mini-G1 ile dogrula.")

        # --- 2. ACRO/RC paramlari FC'den ---
        # 31 Agu 2026 DUZELTME: burada eskiden `params_set_manually` ise
        # master=None geciliyordu ve okuma HIC DENENMIYORDU. Bu YANLISTI:
        #   --params-set-manually'nin anlami "GUID_OPTIONS/GUID_TIMEOUT'u
        #   ELLE YAZDIM, sen YAZMAYA calisma" (guided_sender.setup adim 4).
        #   YAZMA ile OKUMA ayri seyler. ACRO/RCMAP/RC*_MIN..MAX okumasi
        #   FC'yi hic degistirmiyor, sadece rate->PWM haritasini dogruluyor.
        # Eski hali sessiz tuzakti: --params-set-manually ile ucarsan ACRO
        # yaml fallback'ine duserdin ve bunu ancak log satirindan anlardin.
        # Okuma zaten basarisizliga karsi korumali (try_load_from_vehicle
        # istisnayi yakalar, yaml'da kalir, GURULTULU basar) — denemenin
        # maliyeti yok.
        # Deneme kadansi read_param'in SAHADA KANITLANMIS ritmine cekildi:
        # 8 x 0.6 s (upstream 3 x 3.0 s idi).
        #
        # MEKANIK: fetch_parameters_strict her denemede TUM pending'i yeniden
        # soruyor, yani dayaniklilik DENEME SAYISINDAN geliyor. 4 parametrelik
        # acro grubu icin:
        #     upstream  3 deneme x 4 = 12 istek, en kotu 9.0 s
        #     HUNTER    8 deneme x 4 = 32 istek, en kotu 4.8 s
        # Daha cok sans, ustelik daha kisa surede (FC gercekten yoksa
        # acilisi 9 s degil 5 s bekletiyor).
        #
        # Neden onemli: gruplardan biri eksik kalirsa istisna atiliyor, TUM
        # konfig yaml'a dusuyor ve _vehicle_load_failed yapiskan oldugu icin
        # oturum boyunca bir daha DENENMIYOR.
        src = self.converter.try_load_from_vehicle(
            self.conn,
            attempts=self.acro_param_attempts,
            timeout_per_attempt=self.acro_param_timeout,
        )
        if src != 'vehicle':
            print("[ACRO] " + "!" * 58)
            print("[ACRO] !! UYARI: rate->PWM konfigi FC'den OKUNAMADI, "
                  "yaml fallback kullaniliyor.")
            print("[ACRO] !! teacher_hc_air_3.yaml acro: blogu senin gercek "
                  "RC kalibrasyonunla (1045/1495/1945, DZ 0) dolu, ama")
            print("[ACRO] !! FC'de bir sey degistiyse BU DEGERLER ESKIR. "
                  "Ucmadan once yukaridaki [ACRO] satirlarini kontrol et.")
            print("[ACRO] " + "!" * 58)

        if self.acro_max_rate_dps is not None:
            self._narrow_rate_limits(float(self.acro_max_rate_dps))

        cap = self.converter.adapter.capability_report()
        print(f"[ACRO] FC zarf: {cap}")
        print(f"[ACRO] tilt telafisi (angle boost yerine): "
              f"{'ACIK' if self.acro_tilt_comp else 'KAPALI'}")

        rmap = self.converter.rc_channel_map
        print(f"[ACRO] override kanallari: roll=CH{rmap[0]} pitch=CH{rmap[1]} "
              f"throttle=CH{rmap[2]} yaw=CH{rmap[3]}  "
              f"(CH5/6/8/11/12/13 DOKUNULMAZ)")
        for ch in rmap:
            if ch in (5, 6, 8):
                print(f"[ACRO] HATA: RCMAP kanal {ch} HUNTER'da mod/gate/"
                      f"pencere kanali. FC RCMAP_* ayarini kontrol et.")
                return False

        self.ready = True
        return True

    def enter_guided(self):
        """Isim GuidedSender API'si ile ayni; gercekte ACRO'ya gecer."""
        if not self.ready:
            print("[ACRO] HATA: setup() cagrilmadan enter_guided() olmaz")
            return False
        ok = self._set_mode(MODE_ACRO, 'ACRO')
        self.in_guided = ok
        return ok

    def reset(self):
        self.converter.reset()

    # ------------------------------------------------------------------
    # GONDERIM
    # ------------------------------------------------------------------

    def _acro_tilt_boost(self, roll_deg, pitch_deg):
        """ACRO'da angle boost KAPALI — dikey bileseni burada telafi et."""
        if not self.acro_tilt_comp:
            self.last_tilt_boost = 1.0
            return 1.0
        cos_tilt = (math.cos(math.radians(pitch_deg))
                    * math.cos(math.radians(roll_deg)))
        boost = 1.0 / max(cos_tilt, TILT_COS_MIN)
        self.last_tilt_boost = boost
        return boost

    def send(self, action, roll_deg, pitch_deg):
        """
        action: [roll_rate, pitch_rate, thrust_cmd, yaw_rate] — rad/s + teacher
                thrust birimi (hc_air_3, output.mode=body_rate).
        Donus: (roll_rate, pitch_rate, yaw_rate, motor_thrust) — loglamak icin,
               GuidedSender.send() ile AYNI imza.
        """
        roll_a, pitch_a, thr_a, yaw_a = (float(action[0]), float(action[1]),
                                         float(action[2]), float(action[3]))

        # --- GOVERNOR: guided_sender.send() ile BIREBIR AYNI seritler ---
        roll_r, pitch_r, yaw_r = self._rates_to_rad_s(roll_a, pitch_a, yaw_a)
        roll_r *= self.rate_scale * self.axis_scale['roll']
        pitch_r *= self.rate_scale * self.axis_scale['pitch']
        yaw_r *= self.rate_scale * self.axis_scale['yaw']
        if self.max_rate_dps is not None:
            lim = math.radians(self.max_rate_dps)
            g = (max(-lim, min(lim, roll_r)),
                 max(-lim, min(lim, pitch_r)),
                 max(-lim, min(lim, yaw_r)))
            if any(abs(a - b) > 1e-9 for a, b in zip(g, (roll_r, pitch_r, yaw_r))):
                self.governor_hits += 1
            roll_r, pitch_r, yaw_r = g
        pre = (roll_r, pitch_r, yaw_r)
        roll_r, pitch_r, yaw_r = self._clip_action_limits(roll_r, pitch_r, yaw_r)
        if any(abs(a - b) > 1e-6 for a, b in zip(pre, (roll_r, pitch_r, yaw_r))):
            self.clip_count += 1

        # --- DIKEY: guided_sender ile ayni yol + ACRO tilt telafisi ---
        thr_stripped = self._strip_tilt_comp(thr_a, roll_deg, pitch_deg)
        motor = self._action_thrust_to_motor(thr_stripped)
        motor_cmd = motor * self._acro_tilt_boost(roll_deg, pitch_deg)
        # thrust_cap tilt boost'tan SONRA da gecerli olmali (bench korumasi
        # her seyi ezer — guided_sender'daki sirayla ayni mantik).
        if self.thrust_cap is not None and motor_cmd > self.thrust_cap:
            self.thrust_cap_hits += 1
            motor_cmd = float(self.thrust_cap)
        motor_cmd = float(min(max(motor_cmd, 0.0), 1.0))

        # --- RATE + THRUST -> TAM SAYI PWM ---
        pwm = self.converter.rates_to_rc_pwm(roll_r, pitch_r, yaw_r, motor_cmd)

        self.last_rates = (roll_r, pitch_r, yaw_r)
        self.last_thrust = motor
        self.last_pwm = pwm

        if self.enabled and self.conn is not None:
            try:
                self.converter.adapter.send(self._mav_target(), self.converter._last_mapped)
                self.override_active = True
            except Exception as e:
                print(f"[ACRO] gonderim hatasi: {e}")

        return roll_r, pitch_r, yaw_r, motor

    def send_neutral(self):
        """Sifir rate + hover thrust."""
        return self.send([0.0, 0.0, HOVER_ACTION, 0.0], 0.0, 0.0)

    # ------------------------------------------------------------------
    # BIRAKMA — CIKISTA SART
    # ------------------------------------------------------------------

    def _mav_target(self):
        """adapter.send()/release() bir 'master' bekliyor (target_system/
        target_component + mav). FCConnection.conn zaten oyle."""
        return self.conn

    def release(self):
        """
        Tum override'i BIRAK.

        KRITIK: 0 gonderilir, 65535 DEGIL. 65535 = 'bu alani yok say'
        demektir ve mevcut override'i TUTAR. controller_orin.py'de tam
        bu hata yasanmisti (release etmiyordu, sadece RC_OVERRIDE_TIME
        doluyordu). adapter.release() 1..8 kanala 0 yaziyor.
        """
        if not self.enabled or self.conn is None:
            self.override_active = False
            return
        try:
            self.converter.adapter.release(self._mav_target())
            self.override_active = False
        except Exception as e:
            print(f"[ACRO] release hatasi: {e}")

    def release_burst(self, n=5):
        """Paket kaybina karsi n kez birak. Kesme yolunda bloklamaz."""
        for _ in range(max(1, n)):
            self.release()

    def debug(self):
        d = dict(self.converter.last_mapping_debug())
        d['acro_tilt_boost'] = float(self.last_tilt_boost)
        return d
