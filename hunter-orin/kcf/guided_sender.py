#!/usr/bin/env python3
"""
HUNTER — GUIDED_NOGPS taşıma katmanı.

RC_OVERRIDE yerine SET_ATTITUDE_TARGET gönderir. KONTROL MANTIĞI İÇERMEZ —
teacher ne üretiyorsa onu FC'ye taşır. Teacher-agnostik: hem PNTeacher
(normalize aksiyon) hem HCAir (rad/s aksiyon) ile çalışır.

ArduCopter 4.6.3 kaynağından doğrulanmış davranış:
  * type_mask=128 (ATTITUDE_IGNORE) → FC quaternion'ı zaten sıfırlıyor
    (GCS_Mavlink.cpp: `if (attitude_ignore) attitude_quat.zero();`)
    → mode_guided.cpp `angle_control_run()` saf gövde-rate yoluna giriyor.
  * Rate bitleri YA ÜÇÜ BİRDEN geçerli YA ÜÇÜ BİRDEN ignore olmalı.
    Karışıksa FC `mode_guided.init(true)` çağırıp modu RESETLİYOR. 128 üçünü
    de geçerli bırakır.
  * GUID_OPTIONS bit3 (=8) → thrust alanı 0..1 mutlak thrust. Yoksa climb-rate
    olarak yorumlanır ve baro z-controller pitch komutuyla kavga eder.

Kullanım:
    sender = GuidedSender(conn, sys_id, comp_id, action_units='rad_s')
    sender.setup()                       # param oku + doğrula + GUID_OPTIONS
    sender.enter_guided()                # mod: GUIDED_NOGPS
    sender.send(action, roll_deg, pitch_deg)   # her kontrol adımında
    sender.exit_to_acro()                # pilota devir
"""

import math
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# SABİTLER
# ---------------------------------------------------------------------------

MODE_ACRO          = 1
MODE_GUIDED_NOGPS  = 20

# type_mask: bit7 (128) = attitude ignore. Rate bitleri (0,1,2) ve throttle
# biti (6) TEMİZ → hepsi geçerli. Bu değeri değiştirme (yukarıdaki nota bak).
TYPE_MASK_RATES_ONLY = 128

# Teacher throttle konvansiyonu — HEM PNTeacher HEM HCAir aynı:
#   mpc_teacher1.py: self.hover_thrust = 0.07, clip(-0.2, 0.5)
#   teacher_hc_air.yaml: hover_thrust: 0.07, thrust_cmd_min/max: -0.2/0.5
HOVER_ACTION       = 0.07
THRUST_GAIN        = 0.30    # RATE modu: action birimi → motor birimi eğimi

# ATTITUDE-HOLD modu (hc_air_1). fpv_gate.yaml satır 56:
#   "Motor map: hover_motor + (action - 0.07) * 0.45 -> 0.85 = motor 0.68"
# Rate modundan FARKLI eğim; thrust aralığı da geniş ([-0.30, 0.75]).
THRUST_GAIN_ATT    = 0.45

# type_mask: RATE bitleri set (yok say) -> attitude quaternion GEÇERLİ.
# Referans: gazebo_drone_env._send_attitude_target_quaternion
TYPE_MASK_ATTITUDE = (1 | 2 | 4)   # BODY_ROLL/PITCH/YAW_RATE_IGNORE
TILT_COS_MIN       = 0.5     # teacher_tilt_comp ile aynı taban

# MOT_THST_HOVER akıl sağlığı bandı. Dışındaysa BAŞLAMA.
# (ArduPilot iç sınırları 0.125–0.6875; üst tarafta daha dar tutuyoruz çünkü
#  0.35 = öğrenilmemiş fabrika değeri, onu yakalamak istiyoruz. ALT sınır
#  0.05: 8S interceptor tabana [0.125] dayanacak kadar güçlü çıktı —
#  14 Ağu öğrenme uçuşu; 0.15'lik eski taban FC'nin öğrendiği değeri
#  reddediyordu. --hover ile taban altı gerçek değer de verilebilmeli.)
HOVER_SANE_MIN     = 0.05
HOVER_SANE_MAX     = 0.50
# ArduPilot MOT_THST_HOVER alt kırpması. Öğrenilen değer TAM buysa öğrenici
# tabana DAYANMIŞ demektir: gerçek hover <= 0.125, tam değeri bilinmiyor.
HOVER_AP_FLOOR     = 0.125

# Normalize aksiyon → rad/s dönüşümü için ACRO rate paramları okunamazsa
ACRO_RP_RATE_FALLBACK = 360.0    # deg/s
ACRO_Y_RATE_FALLBACK  = 202.5    # deg/s

# Orin ölürse FC'nin kendiliğinden level'a dönmesi için (saniye, aralık 0.1–5)
GUID_TIMEOUT_DEFAULT = 0.5

# ---- V-TRIM (voltaj bazlı hover düzeltmesi) ----
# 20 Ağu 2026 hover-voltaj süpürmesi (4 pencere, sabit 0.102 komut):
#   FC'nin MOT_BAT_ kompanzasyonu DEVREDE (PWM -6.3 µs/V) ama fizik ~-10 istiyor
#   -> kalan açık ~0.003-0.004 hover birimi / V. Saha kuralı (+0.005 H / 5 dk)
#   aynı sayıyı veriyordu. Bu trim o kalan açığı yazılımda kapatır:
#       H_eff = H0 + vtrim * (vref - V_filt)
#   vref = mini-G1 hover kalibrasyonunun yapıldığı voltaj (ELLE girilir).
# NEDEN 5 sn EMA: gaz vuruşlarında V anlık 24'e sarkıyor (20 Ağu logu).
#   Filtresiz trim bu sarkmaları kovalayıp hover'ı pompalar. 5 sn sabiti
#   sag'ları yutar, gerçek deşarj eğimini (0.26 V/dk) rahat takip eder.
# KLEMP: trim deltası [-0.005, +0.020] dışına ÇIKAMAZ — vref yanlış girilse
#   bile hover en fazla bu kadar oynar (asimetrik: deşarj yönü geniş).
VTRIM_TAU       = 5.0      # s, EMA zaman sabiti
VTRIM_DELTA_MIN = -0.005   # taze batarya / yanlış vref koruması
VTRIM_DELTA_MAX = +0.020   # ~5-6 V deşarja yeter (0.0033 * 6 = 0.020)


class GuidedSender:
    """SET_ATTITUDE_TARGET taşıma katmanı. Kontrol mantığı YOK."""

    def __init__(self, conn, target_sys, target_comp,
                 action_units='rad_s',
                 hover_override=None,
                 guid_timeout=GUID_TIMEOUT_DEFAULT,
                 thrust_gain=THRUST_GAIN,
                 enabled=False,
                 params_set_manually=False,
                 thrust_cap=None,
                 rate_scale=1.0,
                 axis_scale=None,
                 max_rate_dps=None,
                 thrust_dev=None,
                 vtrim=0.0,
                 vref=None,
                 att_roll_max=None,
                 att_pitch_max=None,
                 att_yaw_off_max=None):
        """
        conn          : açık mavutil bağlantısı (FCConnection.conn)
        target_sys/comp: FCConnection'dan
        action_units  : 'rad_s'      → HCAir (doğrudan geçir)
                        'normalized' → PNTeacher (-1..+1, ACRO_*_RATE ile ölçekle)
        hover_override: None ise FC'den MOT_THST_HOVER okunur
        enabled       : False = DRY-RUN (hesapla, logla, GÖNDERME)
        """
        if action_units not in ('rad_s', 'normalized', 'attitude_deg'):
            raise ValueError(
                "action_units 'rad_s' | 'normalized' | 'attitude_deg' olmalı, "
                f"geldi: {action_units}")

        self.conn = conn
        self.target_sys = target_sys
        self.target_comp = target_comp
        self.action_units = action_units
        self.hover_override = hover_override
        self.guid_timeout = guid_timeout
        # attitude-hold modunda farkli egim (bkz. THRUST_GAIN_ATT notu)
        if action_units == 'attitude_deg' and thrust_gain == THRUST_GAIN:
            self.thrust_gain = THRUST_GAIN_ATT
        else:
            self.thrust_gain = float(thrust_gain)
        self.enabled = enabled
        # PARAM_VALUE router filtresine takiliyorsa param OKUMA/YAZMA calismaz.
        # Bu bayrakla kullanici paramlari elle set ettigini beyan eder.
        self.params_set_manually = params_set_manually
        # KAPALI ALAN / GIMBAL GUVENLIGI: motor thrust ust siniri (0..1).
        # None = sinir yok (SAHA icin dogru).
        # Gimbal'de dikey dongu OTELEMEYLE kapandigi ve gimbal buna izin
        # vermedigi icin thrust surekli doygun kalir. 05 Agu logu: %47.7 gaz,
        # surenin %95'i tavanda. Gercek hover ~0.25 ise bu 1.9x hover demek —
        # pervaneli gimbal testi icin fazla. Cap ile motorlari donmeye yetecek
        # ama rig'i zorlamayacak seviyede tutuyoruz.
        # ALT SINIR UYARISI: 0.15'in altina inme. Gimbal'de donus motorlar arasi
        # FARKTAN gelir; collective cok dusukse motorlar 0'a dayanir, fark icin
        # yer kalmaz ve yaw/pitch tepkisi kaybolur.
        self.thrust_cap = float(thrust_cap) if thrust_cap is not None else None
        self.thrust_cap_hits = 0
        # GIMBAL GOZLEM OLCEGI: tum gövde rate komutlarini carpar (1.0 = degisiklik yok).
        #
        # NEDEN AYRI BIR DUGME: donus HIZINI thrust DEGIL, rate komutu belirler.
        # ArduCopter'in rate kontrolcusu komutlanan hizi hedefler ve tutturur.
        # 05 Agu logu: yaw komutu 0.954 rad/s = 55 deg/s tavanda -> drone 55 deg/s
        # dondu. Bu sistemin DOGRU calismasi. Thrust'i kismak yetkiyi azaltir ama
        # kontrolcu yine o hiza ulasmaya calisir; gozlem icin yavaslatmaz.
        # Yavaslatmak icin dogru yol rate komutunu olceklemektir.
        #
        # SAHADA 1.0 OLMALI — olcek kucukse teacher'in tuning'i bozulur
        # (kazanclar Gazebo'da 1.0 ile dogrulandi).
        self.rate_scale = float(rate_scale)

        # ---- SAHA GOVERNOR (kademeli yetki merdiveni) ----
        # rate_scale'den FARKI: rate_scale gimbal GOZLEM hilesiydi (global,
        # "sahada 1.0"). Governor ise SAHA icin: eksenleri TEK TEK acarak
        # otonomiyi kademeli test etmenin resmi yolu. Etkin olcek =
        # rate_scale * axis_scale[eksen]. 0.0 = eksen tamamen KAPALI
        # (rate komutu 0; thrust'ta hover'a kilit = sifir-yetki hover testi).
        # Merdiven bitince hepsi 1.0/None olmali — teacher tuning'i o
        # degerlerle dogrulandi.
        sc = {'roll': 1.0, 'pitch': 1.0, 'yaw': 1.0, 'thrust': 1.0}
        if axis_scale:
            for k, v in axis_scale.items():
                if k not in sc:
                    raise ValueError(f"axis_scale bilinmeyen eksen: {k}")
                sc[k] = float(v)
        self.axis_scale = sc
        # Olcekten BAGIMSIZ sert rate tavani (deg/s, uc rate eksenine birden).
        # Olcek komutu kucultur ama satursayonda tavani degistirmez; bu tavan
        # "asla su hizdan fazla donme" garantisidir. None = ek tavan yok
        # (action_limits yine gecerli).
        self.max_rate_dps = None if max_rate_dps is None else abs(float(max_rate_dps))
        # Ucusa uygun dikey kelepce: motor thrust hover ± thrust_dev bandinda
        # kalir. Band hover'i ICERIR -> drone her zaman askida kalabilir.
        # thrust_cap'ten FARKI: cap mutlak ust sinir (bench, hover'in ALTINA
        # inebilir -> ucamaz); dev ise hover etrafinda simetrik band (saha).
        self.thrust_dev = None if thrust_dev is None else abs(float(thrust_dev))

        # ---- V-TRIM (bkz. VTRIM_* sabit notu) ----
        # vtrim = 0 -> TAMAMEN KAPALI (varsayılan; davranış eski kodla birebir).
        # vtrim > 0 -> vref ZORUNLU (setup() reddeder). Voltaj gelene kadar
        # trim pasiftir (H_eff = H0) — sessizce tahmin YOK.
        self.vtrim = max(0.0, float(vtrim))
        self.vref = None if vref is None else float(vref)
        self.v_filt = None            # EMA durumu; ilk örnekle dolar
        self.vtrim_clamp_hits = 0     # delta kaç kez klempe dayandı
        self.last_batt_vf = 0.0       # log için (0.0 = henüz veri yok)
        self.last_heff = 0.0          # log için; setup() hover ile doldurur
        self.governor_hits = 0     # max_rate/thrust_dev kac kez kirpti

        # ---- ATTITUDE-HOLD gimbal sinirlari (sadece action_units='attitude_deg') ----
        # NEDEN GEREKLI: attitude modunda FC'ye "roll 40 derece OLSUN" diyoruz.
        # Gimbal ekseni KILITLIYSE bu asla gerceklesemez; hata kapanmaz, teacher
        # daha da bastirir, komut tavana yapisir. 07 Agu logu tam bunu gosterdi:
        #   cmd_roll  : -39.97 ... +39.98  (limit +-40, TAVANDA)
        #   obs_roll  :  -1.39 ...  +0.39  (drone neredeyse hic yatmadi)
        # Rate modunda bu kadar kotu degildi ("su hizda don" gimbal'de zararsiz);
        # attitude modu bir HEDEF ACI istedigi icin kilitli eksende kacak yapar.
        #
        # att_roll_max = 0.0  -> roll komutu SIFIRLANIR (kilitli eksen icin dogrusu)
        # None                -> sinir yok (SAHA icin dogrusu; teacher tuning'i bozulmasin)
        self.att_roll_max = None if att_roll_max is None else abs(float(att_roll_max))
        self.att_pitch_max = None if att_pitch_max is None else abs(float(att_pitch_max))
        # yaw MUTLAK basliktir; olceklenemez. Bunun yerine mevcut yaw'a gore
        # OFSET sinirlanir: yaw_des = yaw_meas + clamp(yaw_des - yaw_meas, +-limit)
        self.att_yaw_off_max = (None if att_yaw_off_max is None
                                else abs(float(att_yaw_off_max)))
        self.att_limit_hits = 0
        self.last_yaw_meas = None      # controller her adimda gunceller

        self.hover_motor = None          # setup() dolduracak
        self.acro_rp_rate = ACRO_RP_RATE_FALLBACK
        self.acro_y_rate = ACRO_Y_RATE_FALLBACK
        self.ready = False
        self.in_guided = False

        # Son gönderilen değerler — logger/OSD için
        self.last_rates = (0.0, 0.0, 0.0)
        self.last_attitude = (0.0, 0.0, 0.0)   # attitude modunda roll/pitch/yaw deg
        self.last_thrust = 0.0
        self.last_tilt_strip = 0.0
        self.clip_count = 0        # action_limits'e kac kez kirpildi

    # -----------------------------------------------------------------------
    # PARAMETRE YARDIMCILARI
    # -----------------------------------------------------------------------

    def read_param(self, name, timeout=5.0):
        """
        PARAM_REQUEST_READ → PARAM_VALUE. Bulamazsa None.

        UDP KAYIP DAYANIKLILIGI: baglanti udpout, yani istek de cevap da
        kaybolabilir. Tek atisla beklemek yetmiyor (sahada MOT_THST_HOVER bir
        kere okunup sonraki calistirmada timeout verdi). recovery_logger'daki
        dersle ayni: istegi PERIYODIK TEKRARLA.
        """
        def _ask():
            try:
                self.conn.mav.param_request_read_send(
                    self.target_sys, self.target_comp, name.encode('ascii'), -1)
                return True
            except Exception as e:
                print(f"[GUIDED] {name} istenemedi: {e}")
                return False

        if not _ask():
            return None

        t0 = time.time()
        last_ask = t0
        while time.time() - t0 < timeout:
            if time.time() - last_ask > 0.6:      # istek dustuyse yeniden sor
                _ask()
                last_ask = time.time()
            msg = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode('ascii', errors='ignore')
            if pid.strip('\x00') == name:
                return float(msg.param_value)
        print(f"[GUIDED] {name} okunamadı ({timeout:.0f}s timeout, "
              f"{int((time.time()-t0)/0.6)} deneme)")
        return None

    def set_param(self, name, value, verify=True, timeout=3.0):
        """Parametre yaz, istenirse geri okuyarak doğrula."""
        try:
            self.conn.mav.param_set_send(
                self.target_sys, self.target_comp,
                name.encode('ascii'), float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        except Exception as e:
            print(f"[GUIDED] {name} yazılamadı: {e}")
            return False
        if not verify:
            time.sleep(0.05)
            return True
        # Yazim da UDP — kaybolabilir. 3 deneme, her denemede yeniden yaz+oku.
        for attempt in range(3):
            got = self.read_param(name, timeout=2.0)
            if got is not None and abs(got - float(value)) < 1e-3:
                return True
            if got is not None:
                print(f"[GUIDED] {name} doğrulanamadı: istendi={value} okundu={got}")
            if attempt < 2:
                print(f"[GUIDED] {name} tekrar deneniyor ({attempt+2}/3)...")
                try:
                    self.conn.mav.param_set_send(
                        self.target_sys, self.target_comp,
                        name.encode('ascii'), float(value),
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                except Exception:
                    pass
        return False

    # -----------------------------------------------------------------------
    # KURULUM
    # -----------------------------------------------------------------------

    def setup(self):
        """
        Hover'ı belirle, akıl sağlığı kapısından geçir, GUID paramlarını yaz.
        Başarısızsa False döner — ÇAĞIRAN TARAF BAŞLAMAMALI.
        """
        print("[GUIDED] Kurulum başlıyor...")

        # --- 0. V-TRIM ön kontrol: vref'siz vtrim YOK ---
        # Sessiz bir varsayılan uydurmak yerine yüksek sesle reddediyoruz:
        # vref, mini-G1 hover kalibrasyonunun voltajıdır ve ancak SEN bilirsin.
        if self.vtrim > 0 and self.vref is None:
            print("[GUIDED] HATA: --vtrim verildi ama --vref YOK.")
            print("[GUIDED]   vref = hover'ı (mini-G1) bulduğun andaki batarya voltajı.")
            print("[GUIDED]   Örn: --vtrim 0.0033 --vref 32.8")
            return False

        # --- 1. Hover thrust ---
        if self.hover_override is not None:
            self.hover_motor = float(self.hover_override)
            print(f"[GUIDED] hover = {self.hover_motor:.3f}  (--hover ile ELLE verildi)")
            # FC'de ÖĞRENİLMİŞ bir değer varken elle ezmek TEHLİKELİ.
            # Tipik hata: kapalı alanda --hover 0.35 ile test edip, sahada
            # bayrağı kaldırmayı unutmak. Burada yüksek sesle uyarıyoruz.
            fc_val = self.read_param('MOT_THST_HOVER', timeout=2.0)
            if fc_val is not None:
                learned = abs(fc_val - 0.35) >= 1e-4
                if learned and abs(fc_val - self.hover_motor) > 0.01:
                    print("[GUIDED] " + "!" * 58)
                    print(f"[GUIDED] !! DİKKAT: FC'de ÖĞRENİLMİŞ hover = {fc_val:.3f} var,")
                    print(f"[GUIDED] !! sen --hover {self.hover_motor:.3f} ile bunu EZİYORSUN.")
                    print("[GUIDED] !! Uçacaksan --hover bayrağını KALDIR, FC'ninki doğru.")
                    print("[GUIDED] !! (pervanesiz bench için sorun değil)")
                    print("[GUIDED] " + "!" * 58)
                elif not learned:
                    print("[GUIDED]   (FC'de hover hâlâ öğrenilmemiş: 0.350 — "
                          "uçuş öncesi AltHold'da öğren)")
        else:
            val = self.read_param('MOT_THST_HOVER')
            if val is None:
                print("[GUIDED] HATA: MOT_THST_HOVER okunamadı. "
                      "--hover <deger> ile elle ver veya FC bağlantısını kontrol et.")
                return False
            self.hover_motor = val
            print(f"[GUIDED] hover = {self.hover_motor:.3f}  (FC'den MOT_THST_HOVER)")

        # --- 2. AKIL SAĞLIĞI KAPISI ---
        if not (HOVER_SANE_MIN <= self.hover_motor <= HOVER_SANE_MAX):
            print(f"[GUIDED] HATA: hover={self.hover_motor:.3f} makul bandın "
                  f"({HOVER_SANE_MIN}–{HOVER_SANE_MAX}) DIŞINDA.")
            return False

        # ÖĞRENİLMEMİŞ FABRİKA DEĞERİ KONTROLÜ.
        # AP_MOTORS_THST_HOVER_DEFAULT = 0.35f. Öğrenme 10s zaman sabitli bir
        # filtre olduğu için öğrenilmiş değer pratikte ASLA tam 0.35 olmaz
        # (0.2873 gibi çıkar). Tam 0.35 görüyorsak hover hiç öğrenilmemiştir.
        # Bu bandın İÇİNDE kaldığı için üstteki kapı yakalamaz — ayrı bakıyoruz.
        if self.hover_override is None and abs(self.hover_motor - 0.35) < 1e-4:
            print("[GUIDED] HATA: MOT_THST_HOVER = 0.350 — bu ArduPilot'un "
                  "ÖĞRENİLMEMİŞ fabrika değeri.")
            print("[GUIDED]   Senin drone'unun gerçek hover'ı bu değil; bu değerle "
                  "uçarsan dikey kanal baştan hatalı olur.")
            print("[GUIDED]   Çözüm: AltHold'da 1 dk düz as — çubuk TAM ORTADA, "
                  "<5° eğim, |vz|<0.6 m/s.")
            print("[GUIDED]           MOT_HOVER_LEARN=2 (varsayılan) EEPROM'a yazar.")
            print("[GUIDED]   Gerçekten 0.35 hover'lı bir gövde ise: --hover 0.35 ile "
                  "bilerek geç.")
            return False

        # TABANA YAPIŞMA UYARISI (0.35 tuzağının kardeşi, ama REDDETMEZ):
        # Öğrenilen değer TAM 0.125 = ArduPilot alt kırpması. Filtre tabana
        # dayanmış -> gerçek hover <= 0.125, ne kadar altında bilinmiyor.
        # 14 Ağu öğrenme uçuşunda yaşandı (8S güç fazlalığı). Değer kullanılabilir
        # ama dikey kanal hover farkı kadar YUKARI meyilli olur.
        if abs(self.hover_motor - HOVER_AP_FLOOR) < 1e-4:
            print("[GUIDED] " + "!" * 58)
            print("[GUIDED] !! hover = 0.125 — ArduPilot'un ALT KIRPMASI. Öğrenici")
            print("[GUIDED] !! tabana dayanmış: gerçek hover <= 0.125, tam değer")
            print("[GUIDED] !! bilinmiyor. Doğrulama: .bin logda CTUN.ThO hover")
            print("[GUIDED] !! ortalaması, ya da G1'de tırmanma hızından geri hesap.")
            print("[GUIDED] !! Gerçek değer belliyse: --hover <deger> ile ver.")
            print("[GUIDED] " + "!" * 58)

        # Ne kadar dikey otorite kaldığını uçuş öncesi göster
        if self.rate_scale != 1.0:
            print("[GUIDED] " + "=" * 58)
            print(f"[GUIDED] RATE OLCEGI: x{self.rate_scale:.2f}  (gimbal gozlem modu)")
            print(f"[GUIDED]   yaw tavani {math.degrees(0.954):.0f} d/s -> "
                  f"{math.degrees(0.954*self.rate_scale):.0f} d/s")
            print("[GUIDED]   SAHADA 1.0 YAP — kucuk olcek teacher tuning'ini bozar.")
            print("[GUIDED] " + "=" * 58)
        if self.thrust_cap is not None:
            print("[GUIDED] " + "=" * 58)
            print(f"[GUIDED] THRUST TAVANI AKTIF: motor <= {self.thrust_cap:.2f} "
                  f"(%{100*self.thrust_cap:.0f} gaz)")
            print("[GUIDED]   Kapali alan / gimbal testi icin. SAHADA KALDIR —")
            print("[GUIDED]   bu tavanla drone HAVALANAMAZ.")
            print("[GUIDED] " + "=" * 58)
        gov_active = (any(abs(v - 1.0) > 1e-9 for v in self.axis_scale.values())
                      or self.max_rate_dps is not None
                      or self.thrust_dev is not None)
        if gov_active:
            lim = (math.radians(self.max_rate_dps) if self.max_rate_dps is not None
                   else float('inf'))
            print("[GUIDED] " + "=" * 58)
            print("[GUIDED] SAHA GOVERNOR AKTIF (kademeli yetki merdiveni):")
            print(f"[GUIDED]   yetki: roll={self.axis_scale['roll']:.2f} "
                  f"pitch={self.axis_scale['pitch']:.2f} "
                  f"yaw={self.axis_scale['yaw']:.2f} "
                  f"thrust={self.axis_scale['thrust']:.2f}   (0 = eksen KAPALI)")
            print(f"[GUIDED]   etkin rate tavani (deg/s): "
                  f"R={math.degrees(min(self.ACTION_LIMIT_ROLL, lim)):.0f} "
                  f"P={math.degrees(min(self.ACTION_LIMIT_PITCH, lim)):.0f} "
                  f"Y={math.degrees(min(self.ACTION_LIMIT_YAW, lim)):.0f}")
            td = (f"hover ± {self.thrust_dev:.3f}" if self.thrust_dev is not None
                  else "yok (tam aralik)")
            print(f"[GUIDED]   thrust bandi: {td}")
            print("[GUIDED]   MERDIVEN BITINCE hepsi 1.0 / limitsiz olmali —")
            print("[GUIDED]   teacher tuning'i o degerlerle dogrulandi.")
            print("[GUIDED] " + "=" * 58)
        self.last_heff = self.hover_motor    # V gelene kadar log'da H0 görünür
        if self.vtrim > 0:
            print("[GUIDED] " + "=" * 58)
            print(f"[GUIDED] V-TRIM AKTIF: H_eff = {self.hover_motor:.3f} "
                  f"+ {self.vtrim:.4f} × ({self.vref:.1f} − V_filt)")
            print(f"[GUIDED]   delta klempi [{VTRIM_DELTA_MIN:+.3f}, "
                  f"{VTRIM_DELTA_MAX:+.3f}]  ·  EMA {VTRIM_TAU:.0f} s")
            print("[GUIDED]   Voltaj (SYS_STATUS) gelene kadar trim PASİF (H_eff=H0).")
            print(f"[GUIDED]   vref={self.vref:.1f} V = mini-G1 kalibrasyon voltajın "
                  "OLMALI — değilse yanlış.")
            print("[GUIDED] " + "=" * 58)
        motor_min = self._action_thrust_to_motor(-0.2)
        motor_max = self._action_thrust_to_motor(0.5)
        print(f"[GUIDED]   motor bandı: [{motor_min:.3f}, {motor_max:.3f}]  "
              f"→ dikey ivme [{(motor_min/self.hover_motor - 1)*9.81:+.1f}, "
              f"{(motor_max/self.hover_motor - 1)*9.81:+.1f}] m/s²")
        # Banner hesabi _action_thrust_to_motor'u cagirdi -> sayaclar kirlendi.
        # Ucus sayimi temiz baslasin.
        self.thrust_cap_hits = 0
        self.governor_hits = 0
        self.vtrim_clamp_hits = 0

        # --- 3. Normalize aksiyon kullanılacaksa ACRO rate'leri oku ---
        if self.action_units == 'attitude_deg':
            print("[GUIDED] ATTITUDE-HOLD modu: quaternion gönderilir "
                  "(type_mask=7, rate'ler ignore)")
            print(f"[GUIDED]   thrust eğimi = {self.thrust_gain} (rate modunda 0.30)")
            print("[GUIDED]   tilt_comp ÇIKARILMIYOR (referans env ile aynı)")
        if self.action_units == 'normalized':
            rp = self.read_param('ACRO_RP_RATE')
            y = self.read_param('ACRO_Y_RATE')
            if rp is not None and rp > 0:
                self.acro_rp_rate = rp
            else:
                print(f"[GUIDED] ACRO_RP_RATE okunamadı → varsayılan {self.acro_rp_rate}")
            if y is not None and y > 0:
                self.acro_y_rate = y
            else:
                print(f"[GUIDED] ACRO_Y_RATE okunamadı → varsayılan {self.acro_y_rate}")
            print(f"[GUIDED] normalize→rad/s ölçeği: RP={self.acro_rp_rate:.1f} °/s, "
                  f"Y={self.acro_y_rate:.1f} °/s")

        # --- 4. GUID parametreleri ---
        # DİKKAT: SITL env'indeki FS_THR_ENABLE=0 / SIM_RC_FAIL=0 BİLEREK ALINMADI.
        #         Onlar sim'in kendi kendine arm olması içindi; gerçek uçuşta
        #         failsafe'i kapatmak tehlikeli.
        if self.params_set_manually:
            print("[GUIDED] " + "=" * 58)
            print("[GUIDED] --params-set-manually: GUID param yazimi ATLANDI.")
            print("[GUIDED] SORUMLULUK SENDE — Mission Planner'dan DOGRULA:")
            print("[GUIDED]     GUID_OPTIONS = 8      (thrust mutlak; 0 ise thrust")
            print("[GUIDED]                            climb-rate sanilir, TEHLIKELI)")
            print(f"[GUIDED]     GUID_TIMEOUT = {self.guid_timeout}    (Orin susarsa level'a doner)")
            print("[GUIDED] " + "=" * 58)
        elif not self.enabled:
            # DRY-RUN: FC'ye YAZMA. controller_orin'in ACRO param davranışıyla aynı.
            print("[GUIDED] DRY-RUN: GUID_OPTIONS / GUID_TIMEOUT yazılmadı (FC'ye yazım yok)")
        else:
            if not self.set_param('GUID_OPTIONS', 8):
                print("[GUIDED] " + "!" * 58)
                print("[GUIDED] HATA: GUID_OPTIONS=8 yazilamadi/dogrulanamadi.")
                print("[GUIDED]   Bu param OLMADAN thrust climb-rate sanilir ve")
                print("[GUIDED]   baro z-controller pitch komutuyla kavga eder.")
                print("[GUIDED]   ")
                print("[GUIDED]   MUHTEMEL SEBEP: ArduPilot ARM iken parametre")
                print("[GUIDED]   yazimini reddedebilir. DISARM edip tekrar dene.")
                print("[GUIDED]   ")
                print("[GUIDED]   KALICI COZUM: Mission Planner / QGC'den elle")
                print("[GUIDED]   GUID_OPTIONS = 8 yaz, bir daha ugrasma.")
                print("[GUIDED] " + "!" * 58)
                return False
            print("[GUIDED] GUID_OPTIONS=8  (thrust = mutlak 0..1)")

            if self.set_param('GUID_TIMEOUT', self.guid_timeout):
                print(f"[GUIDED] GUID_TIMEOUT={self.guid_timeout}s  "
                      f"(Orin susarsa FC level'a döner, rate=0)")
            else:
                print("[GUIDED] Uyarı: GUID_TIMEOUT yazılamadı — varsayılan (3s) geçerli")

        self.ready = True
        mode_str = "AKTİF" if self.enabled else "DRY-RUN (gönderim YOK)"
        print(f"[GUIDED] Kurulum tamam — {mode_str}")
        return True

    # -----------------------------------------------------------------------
    # THRUST DÖNÜŞÜMÜ
    # -----------------------------------------------------------------------

    @staticmethod
    def _teacher_tilt_comp(roll_deg, pitch_deg):
        """
        Teacher'ın kendi eklediği tilt trim'i (action biriminde).
        HCAir.teacher_tilt_comp ve mpc_teacher1 satır 259 ile AYNI formül.
        """
        cos_tilt = math.cos(math.radians(pitch_deg)) * math.cos(math.radians(roll_deg))
        return (1.0 / max(cos_tilt, TILT_COS_MIN) - 1.0) * HOVER_ACTION

    def _strip_tilt_comp(self, thrust_action, roll_deg, pitch_deg):
        """
        Teacher tilt_comp'u ÇIKAR.

        Teacher ACRO için yazıldı; ACRO'da angle boost KAPALI
        (mode_acro.cpp: set_throttle_out(..., false, ...)) o yüzden teacher
        kendi kompanzasyonunu ekliyordu. GUIDED'da angle boost AÇIK
        (set_throttle_out(..., true, ...)) ve FC bunu motor biriminde,
        yani fiziksel olarak DOĞRU ölçekte yapıyor. Teacher'ınki action
        biriminde (0.07 tabanlı) olduğu için zaten ~15x eksik ölçekliydi.
        → Teacher'ınkini at, işi FC'ye bırak.
        """
        tilt = self._teacher_tilt_comp(roll_deg, pitch_deg)
        self.last_tilt_strip = tilt
        return float(thrust_action) - tilt

    def update_voltage(self, v, dt):
        """
        Batarya voltajı örneği (SYS_STATUS'tan, controller her döngü çağırır).
        5 sn EMA — gaz vuruşu sag'larını yutar (bkz. VTRIM_TAU notu).
        vtrim kapalıyken de çağrılabilir; zararsız (sadece filtre döner).
        """
        v = float(v)
        if not (5.0 <= v <= 60.0):        # saçma örnek (sensör glitch) -> atla
            return
        if self.v_filt is None:
            self.v_filt = v               # ilk örnek: filtreyi doğrudan doldur
        else:
            a = min(1.0, max(0.0, float(dt)) / VTRIM_TAU)
            self.v_filt += (v - self.v_filt) * a
        self.last_batt_vf = self.v_filt

    def _hover_eff(self):
        """
        Etkin hover: H0 + klempli V-trim deltası.
        vtrim=0 veya henüz voltaj yoksa AYNEN hover_motor döner
        (eski davranışla bit-bit aynı).
        """
        if self.vtrim <= 0 or self.v_filt is None:
            return self.hover_motor
        delta = self.vtrim * (self.vref - self.v_filt)
        if delta < VTRIM_DELTA_MIN or delta > VTRIM_DELTA_MAX:
            self.vtrim_clamp_hits += 1
            delta = max(VTRIM_DELTA_MIN, min(VTRIM_DELTA_MAX, delta))
        return self.hover_motor + delta

    def _action_thrust_to_motor(self, thrust_action):
        """
        Teacher throttle aksiyonu → motor thrust (0..1).

        Teacher'ın 0.07'si RC stick ofsetiydi, %7 motor DEĞİL. Hover'ı
        gerçek MOT_THST_HOVER'a çiviliyoruz, sapmaları gain ile ölçekliyoruz.
        V-trim aktifse merkez H_eff'tir (thrust_dev bandı da onunla kayar —
        trim bandın dışına itilemez, band trimle birlikte yürür).
        """
        h = self._hover_eff()
        self.last_heff = h
        dev = (float(thrust_action) - HOVER_ACTION) * self.thrust_gain
        # GOVERNOR: dikey yetki. 0.0 = hover'a kilit (sifir-yetki hover testi).
        dev *= self.axis_scale['thrust']
        # GOVERNOR: hover ± thrust_dev bandi (ucusa uygun — hover bandin icinde)
        if self.thrust_dev is not None:
            if abs(dev) > self.thrust_dev:
                self.governor_hits += 1
            dev = max(-self.thrust_dev, min(self.thrust_dev, dev))
        motor = h + dev
        motor = float(min(max(motor, 0.0), 1.0))
        # thrust_cap EN SON — bench/rig korumasi her seyi ezer
        if self.thrust_cap is not None and motor > self.thrust_cap:
            self.thrust_cap_hits += 1
            motor = self.thrust_cap
        return motor

    # Referans env.step() komutu bu tavanlara KIRPIYOR (gazebo_drone_env satir ~471:
    # "Rates rad/s; clip to HCAir command caps"). teacher_hc_air.yaml action_limits.
    # Teacher'in KENDI ic limitleri bazi kanallarda BUNDAN GENIS:
    #     pitch ic=0.4006 > env=0.3770  -> kirpilmazsa Gazebo'da hic gorulmemis
    #                                      bir oran gercek drone'a gider (%6 fazla).
    # Gercek logda (05 Agu) cmd_pitch 0.4004'e ciktigi OLCULDU -> kirpma SART.
    ACTION_LIMIT_ROLL  = 1.884955592153876
    ACTION_LIMIT_PITCH = 0.3769911184307752
    ACTION_LIMIT_YAW   = 1.0602875205865552

    def _clip_action_limits(self, roll_r, pitch_r, yaw_r):
        c = lambda v, lim: float(min(max(v, -lim), lim))
        return (c(roll_r,  self.ACTION_LIMIT_ROLL),
                c(pitch_r, self.ACTION_LIMIT_PITCH),
                c(yaw_r,   self.ACTION_LIMIT_YAW))

    def _rates_to_rad_s(self, roll_a, pitch_a, yaw_a):
        """Aksiyon birimine göre gövde rate'lerini rad/s'ye çevir."""
        if self.action_units == 'rad_s':
            return float(roll_a), float(pitch_a), float(yaw_a)
        # normalized (-1..+1) → ACRO stick eşdeğeri → rad/s
        k_rp = math.radians(self.acro_rp_rate)
        k_y = math.radians(self.acro_y_rate)
        return float(roll_a) * k_rp, float(pitch_a) * k_rp, float(yaw_a) * k_y

    # -----------------------------------------------------------------------
    # MOD YÖNETİMİ
    # -----------------------------------------------------------------------

    def _set_mode(self, custom_mode, name, timeout=3.0):
        if not self.enabled:
            print(f"[GUIDED] (dry-run) mod değişimi atlandı: {name}")
            return True
        try:
            self.conn.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode, 0, 0, 0, 0, 0)
        except Exception as e:
            print(f"[GUIDED] {name} moduna geçilemedi: {e}")
            return False

        # HEARTBEAT ile gerçekten geçti mi doğrula (ACK'e güvenme)
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if hb is None:
                continue
            if hb.get_srcSystem() != self.target_sys:
                continue
            if hb.custom_mode == custom_mode:
                print(f"[GUIDED] mod = {name} ✓")
                return True
        print(f"[GUIDED] UYARI: {name} moduna geçiş doğrulanamadı")
        return False

    def enter_guided(self):
        if not self.ready:
            print("[GUIDED] HATA: setup() çağrılmadan enter_guided() olmaz")
            return False
        ok = self._set_mode(MODE_GUIDED_NOGPS, 'GUIDED_NOGPS')
        self.in_guided = ok
        return ok

    def exit_to_acro(self):
        """Pilota devir (doğrulamalı). Normal/planlı çıkış için."""
        ok = self._set_mode(MODE_ACRO, 'ACRO')
        self.in_guided = False
        return ok

    def set_mode_fast(self, custom_mode, n=5):
        """
        BLOKLAMAYAN mod komutu — güvenlik kesme yolu.

        release_burst(5)'in karşılığı. _set_mode() heartbeat doğrulaması için
        3 sn'ye kadar bloklayabilir; kesme anında bu KABUL EDİLEMEZ (30 Hz
        döngü durur, pilot bekler). Burada komutu n kez ateşleyip çıkıyoruz —
        paket kaybına karşı güvence, doğrulama YOK.

        Doğrulamayı çağıran taraf yapar (bkz. ExitModeWatchdog): heartbeat'ten
        modu izler, tutmazsa yedek moda geçer.
        """
        if custom_mode != MODE_GUIDED_NOGPS:
            self.in_guided = False
        if not self.enabled or self.conn is None:
            print(f"[GUIDED] (dry-run) mod komutu atlandı: {custom_mode}")
            return
        for _ in range(max(1, n)):
            try:
                self.conn.mav.command_long_send(
                    self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode, 0, 0, 0, 0, 0)
            except Exception:
                pass

    def exit_to_acro_fast(self, n=5):
        """Geriye uyumluluk: ACRO'ya bloklamadan geç."""
        self.set_mode_fast(MODE_ACRO, n)

    # -----------------------------------------------------------------------
    # GÖNDERİM
    # -----------------------------------------------------------------------

    @staticmethod
    def _euler_to_quaternion(roll_rad, pitch_rad, yaw_rad):
        """ZYX (yaw-pitch-roll) -> [w, x, y, z]. Referans env ile ayni."""
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]

    @staticmethod
    def _wrap_deg(d):
        return (float(d) + 180.0) % 360.0 - 180.0

    def _apply_att_limits(self, roll_d, pitch_d, yaw_d, yaw_meas_deg):
        """Gimbal test sinirlari. SAHADA hepsi None olmali (sinir yok)."""
        hit = False
        if self.att_roll_max is not None:
            lim = self.att_roll_max
            if abs(roll_d) > lim:
                hit = True
            roll_d = max(-lim, min(lim, roll_d))
        if self.att_pitch_max is not None:
            lim = self.att_pitch_max
            if abs(pitch_d) > lim:
                hit = True
            pitch_d = max(-lim, min(lim, pitch_d))
        if self.att_yaw_off_max is not None and yaw_meas_deg is not None:
            off = self._wrap_deg(yaw_d - yaw_meas_deg)
            lim = self.att_yaw_off_max
            if abs(off) > lim:
                hit = True
            off = max(-lim, min(lim, off))
            yaw_d = self._wrap_deg(yaw_meas_deg + off)
        if hit:
            self.att_limit_hits += 1
        return roll_d, pitch_d, yaw_d

    def send_attitude(self, action, yaw_meas_deg=None):
        """
        ATTITUDE-HOLD gönderimi (hc_air_1 / attitude-hold teacher).

        action = [roll_des_deg, pitch_des_deg, thrust, yaw_des_deg]
        yaw MUTLAK basliktir (hc_air_1: yaw_des = wrap(yaw_meas + offset)).

        RATE modundan iki KRITIK fark:
          1) type_mask: rate bitleri IGNORE, quaternion GECERLI
          2) tilt_comp CIKARILMAZ — referans env yorumu birebir:
             "Unlike rate control, tilt_comp is NOT stripped —
              caller should add pitch feedforward to thrust."
             (hc_air_1 pitch_thrust_gain ile FF'i kendi ekliyor)
        """
        roll_d, pitch_d, thr_a, yaw_d = (float(action[0]), float(action[1]),
                                         float(action[2]), float(action[3]))
        roll_d, pitch_d, yaw_d = self._apply_att_limits(
            roll_d, pitch_d, yaw_d, yaw_meas_deg)
        quat = self._euler_to_quaternion(math.radians(roll_d),
                                         math.radians(pitch_d),
                                         math.radians(yaw_d))
        thrust = self._action_thrust_to_motor(thr_a)   # tilt strip YOK

        self.last_attitude = (roll_d, pitch_d, yaw_d)
        self.last_rates = (0.0, 0.0, 0.0)
        self.last_thrust = thrust
        self.last_tilt_strip = 0.0

        if self.enabled and self.conn is not None:
            try:
                self.conn.mav.set_attitude_target_send(
                    0,
                    self.target_sys, self.target_comp,
                    TYPE_MASK_ATTITUDE,
                    quat,
                    0.0, 0.0, 0.0,
                    thrust)
            except Exception as e:
                print(f"[GUIDED] attitude gönderim hatası: {e}")
        return roll_d, pitch_d, yaw_d, thrust

    def send(self, action, roll_deg, pitch_deg):
        """
        Teacher aksiyonunu FC'ye gönder.

        action: [roll, pitch, throttle, yaw]  (teacher sırası — HCAir ve
                PNTeacher aynı sırayı kullanıyor; throttle 3. eleman)
        roll_deg/pitch_deg: FC'den anlık tutum (tilt strip için)

        Dönüş: (roll_rate, pitch_rate, yaw_rate, thrust) — loglamak için.
        """
        if self.action_units == 'attitude_deg':
            return self.send_attitude(action, yaw_meas_deg=self.last_yaw_meas)

        roll_a, pitch_a, thr_a, yaw_a = (float(action[0]), float(action[1]),
                                         float(action[2]), float(action[3]))

        roll_r, pitch_r, yaw_r = self._rates_to_rad_s(roll_a, pitch_a, yaw_a)
        # GOVERNOR 1/2: eksen yetkisi (etkin olcek = rate_scale * axis_scale)
        roll_r  *= self.rate_scale * self.axis_scale['roll']
        pitch_r *= self.rate_scale * self.axis_scale['pitch']
        yaw_r   *= self.rate_scale * self.axis_scale['yaw']
        # GOVERNOR 2/2: olcekten bagimsiz sert tavan (deg/s)
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
        thr_stripped = self._strip_tilt_comp(thr_a, roll_deg, pitch_deg)
        thrust = self._action_thrust_to_motor(thr_stripped)

        self.last_rates = (roll_r, pitch_r, yaw_r)
        self.last_thrust = thrust

        if self.enabled and self.conn is not None:
            try:
                self.conn.mav.set_attitude_target_send(
                    0,
                    self.target_sys, self.target_comp,
                    TYPE_MASK_RATES_ONLY,
                    [1.0, 0.0, 0.0, 0.0],   # FC sıfırlıyor (attitude_ignore)
                    roll_r, pitch_r, yaw_r,
                    thrust)
            except Exception as e:
                print(f"[GUIDED] gönderim hatası: {e}")

        return roll_r, pitch_r, yaw_r, thrust

    def send_neutral(self):
        """Sıfır rate + hover thrust. Hedef kaybında / devir öncesi."""
        return self.send([0.0, 0.0, HOVER_ACTION, 0.0], 0.0, 0.0)
