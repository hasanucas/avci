#!/usr/bin/env python3
"""
HUNTER — ACRO + RC_CHANNELS_OVERRIDE tasima katmani, PWM GECIRMELI.  (9 Kas 2026)

acro_sender.py'nin KARDESI, YERINE DEGIL. Ikisi de ACRO'ya gecip CH1-4'u
RC_CHANNELS_OVERRIDE ile suruyor; fark rate->PWM cevriminin NEREDE yapildigi:

    acro_sender.py     teacher rad/s verir  -> SENDER cevirir
                       (hc_air / hc_air_3, output.mode=body_rate)
                       dikey kanal guided_sender'in sahada dogrulanmis
                       yolundan gecer: hover_motor + dev*thrust_gain,
                       --scale-thrust/--thrust-dev/--thrust-cap governorlari,
                       ACRO'da angle boost kapali oldugu icin tilt telafisi.

    acro_rc_sender.py  teacher TAM SAYI PWM verir -> SENDER sadece GONDERIR
    (BU DOSYA)         (hc_air_5, output.mode=acro_rc)
                       cevrimi arkadasin HCAirAcroConverter'i teacher'in
                       ICINDE yapar; dikey kanal da onun
                       hover_thrust_firmware + thrust_scale yolundan gecer.

Neden ayri dosya: ikisi ayni anda YASAMALI. acro_sender sahada bench'e kadar
gelmis, incelenmis bir yol; hc_air_5 ise arkadasin yeni paketi ve dikey
kanali BASKA bir egriden geciyor. Birini digerinin uzerine yazmak tek
degisken disiplinini bozardi — iki yol yan yana durur, bayrak secer.

    sender = AcroRcSender(conn, sys, comp, teacher=teacher)
    sender.setup()          # teacher'in converter'ini FC paramlariyla tazeler
    sender.enter_guided()   # ACRO'ya gecer (isim GuidedSender API'si ile ayni)
    sender.send(action, roll_deg, pitch_deg, body=teacher.last_debug)
    sender.release()        # override BIRAK — cikista SART

=============================================================================
GOVERNOR BU YOLDA NE YAPAR — DURUST TABLO
=============================================================================
Teacher PWM'i KENDISI uretiyor, yani rate uzayindaki kelepceler artik
"komutu kirpma" degil. Sessizce etkisiz kalmasinlar diye her biri bu yolda
ANLAMLI bir ise baglandi:

| Bayrak                    | Bu yolda ne yapar                              |
|---------------------------|------------------------------------------------|
| --acro-max-rate           | TEACHER'in converter zarfini daraltir (rad/s). |
|                           | TAM: cevrim bu zarfin icinde yapilir.          |
| --max-rate                | AYNI zarfa baglandi (ikisinin KUCUGU gecerli). |
|                           | GUIDED yolunda teacher komutunu kirpiyordu;    |
|                           | burada kirpacak komut yok, tavan zarftan gelir.|
| --scale-roll/pitch/yaw    | PWM uzayinda trim etrafinda olcek.             |
|                           | YAKLASIK — asagidaki EXPO notuna bak.          |
| --scale-thrust            | Gaz PWM'i hover PWM'ine dogru olceklenir.      |
|                           | 0 = gaz HOVER PWM'ine KILITLI.                 |
| --thrust-cap              | Gaz PWM ust siniri (bench).                    |
| --thrust-dev              | Gaz, hover +- dev bandinda tutulur (saha).     |
| --hover                   | ZORUNLU GIBI DAVRAN. Teacher'in dikey cikisi   |
|                           | acro.hover_thrust_firmware'den geliyor ve O    |
|                           | DEGER FC'DEN OKUNMUYOR. fpv_gate_5.yaml 0.33   |
|                           | tasiyor (sim); HUNTER'in olculmus hover'i      |
|                           | 0.101-0.102. Bkz. _resolve_hover_fw().         |
| --att-*-max, --gimbal     | ATTITUDE yoluna ait — controller REDDEDER.     |
| --acro-no-tilt-comp       | ESKI yola ait — controller REDDEDER. Bu yolda  |
|                           | tilt telafisi teacher'in icinde.               |

EXPO NOTU (ucmadan once oku): ACRO_RP_EXPO = 0.3 oldugu icin
rate = f(cubuk) DOGRUSAL DEGIL. Yani --scale-roll 0.5 PWM uzayinda
cubugu yariya indirir ama rate'i TAM YARIYA indirmez (expo kucuk
cubukta rate'i daha da kisar, yani gercek azalma yaridan FAZLA olur).
Yetki azaltmak icin dogru yondedir, ama "yarim rate" diye okuma.
KESIN rate tavani istiyorsan --acro-max-rate kullan: o zarfi
dogrudan rad/s cinsinden daraltir.

=============================================================================
CIKIS — acro_sender.py ile AYNI TEHLIKE, AYNI SIRA
=============================================================================
RC_CHANNELS_OVERRIDE MODDAN BAGIMSIZDIR. ACRO'dan ALTHOLD'a gecsen bile
override hala CH1-4'u suruyor ve yeni mod onu PILOT CUBUGU sanar. Hover
PWM'i orta cubugun altindaysa ALTHOLD "pilot alcalmak istiyor" diye okur
ve drone iner — ekranda mod dogru yazarken.

Bu yuzden sira KESIN: once release, sonra mod. Release 0 gonderir, 65535
DEGIL (65535 override'i TUTAR; controller_orin.py'de tam bu hata yasandi).
Siralama controller_guided.py'deki ExitModeWatchdog'da, acro_path=True ile
zaten kurulu — bu dosya onun release()/release_burst() sozlesmesini saglar.
"""

import math
from dataclasses import replace

from guided_sender import (
    GuidedSender,
    HOVER_AP_FLOOR,
    HOVER_SANE_MAX,
    HOVER_SANE_MIN,
    MODE_ACRO,
)

# Teacher'in converter'ini FC paramlariyla tazelerken kullanilan kadans.
#
# NEDEN TEACHER'IN KENDI METODU DEGIL: HCAir.try_load_acro_from_vehicle(master)
# hic kwarg almiyor, yani upstream'in sabit 3 deneme x 3.0 s ayarinda kaliyor.
# HUNTER'in udpout linki param isteklerini dusuruyor (guided_sender.read_param
# yorumundaki saha dersi: "MOT_THST_HOVER bir kere okunup sonraki calistirmada
# timeout verdi"). Gruplardan biri eksik kalirsa TUM konfig yaml'a duser ve
# _vehicle_load_failed yapiskan oldugu icin oturum boyunca bir daha denenmez.
# Bu yuzden converter'in HUNTER surumundeki attempts/timeout_per_attempt
# parametrelerini DOGRUDAN kullaniyoruz: 8 x 0.6 s = 32 istek, en kotu 4.8 s
# (upstream 12 istek, en kotu 9.0 s).
ACRO_PARAM_ATTEMPTS = 8
ACRO_PARAM_TIMEOUT = 0.6


class AcroRcSender(GuidedSender):
    """ACRO + RC_CHANNELS_OVERRIDE. Teacher'in verdigi PWM'i GECIRIR."""

    def __init__(self, conn, target_sys, target_comp,
                 teacher=None,
                 acro_max_rate_dps=None,
                 acro_param_attempts=ACRO_PARAM_ATTEMPTS,
                 acro_param_timeout=ACRO_PARAM_TIMEOUT,
                 accept_yaml_cal=False,
                 **kwargs):
        """
        teacher          : output_mode='acro_rc' ile kurulmus HCAir (hc_air_5).
                           Cevrimi yapan converter ONUN icinde; bu katman
                           ayri bir adapter KURMAZ, teacher'inkini kullanir
                           (tek dogruluk kaynagi).
        acro_max_rate_dps: teacher'in converter zarfini deg/s cinsinden
                           daraltir. --max-rate ile birlesik (kucugu gecer).
        """
        # Taban sinif action_units'i dogruluyor. Bu yolda taban sinifin rate
        # yolu (send/_rates_to_rad_s/_action_thrust_to_motor) HIC CALISMIYOR —
        # send() tamamen override edildi. 'rad_s' sadece dogrulamayi gecmek
        # icin; dikey kanal da dahil her sey teacher'in icinde hesaplandi.
        kwargs.setdefault('action_units', 'rad_s')
        if kwargs['action_units'] != 'rad_s':
            raise ValueError(
                "AcroRcSender action_units='rad_s' ile kurulur (geldi: "
                f"{kwargs['action_units']}). Bu katman teacher'in TAM SAYI "
                "PWM cikisini gecirir; attitude_deg teacher'lari GUIDED ile "
                "ucar.")
        super().__init__(conn, target_sys, target_comp, **kwargs)

        self.teacher = teacher
        self.acro_max_rate_dps = acro_max_rate_dps
        self.acro_param_attempts = int(acro_param_attempts)
        self.acro_param_timeout = float(acro_param_timeout)
        # FC'den okunamayan mapping ile canli ucusa BILEREK izin
        # (--acro-accept-yaml-cal). Bkz. _gate_unverified_mapping().
        self.accept_yaml_cal = bool(accept_yaml_cal)

        self.converter = self._teacher_converter(teacher)
        self.adapter = self.converter.adapter

        # Dikey referanslar — PWM uzayindaki governor bunlara gore calisir.
        # hover_thrust_firmware teacher'in hover'ina (platform.hover_thrust)
        # KARSILIK GELEN firmware thrust'i; converter o eslemeyi kuruyor.
        self.hover_thrust_fw = float(self.converter.hover_thrust_firmware)
        self.hover_pwm = int(self.adapter.throttle_pwm_for_thrust(self.hover_thrust_fw))
        # GuidedSender alanlari log/durum satiri icin doldurulur. Bu yolda
        # thrust'i teacher belirliyor, hover_motor SADECE bilgi.
        self.hover_motor = float(self.converter.hover_thrust_teacher)

        # Loglama / durum
        self.last_pwm = (0, 0, 0, 0)
        self.last_pwm_raw = (0, 0, 0, 0)     # teacher'dan cikan, governor ONCESI
        self.last_tilt_boost = 1.0           # bu yolda telafi teacher'in icinde
        self.last_attitude = (0.0, 0.0, 0.0)  # controller uyumlulugu
        self.override_active = False
        self.pwm_governor_hits = 0

    # ------------------------------------------------------------------
    # TEACHER BAGI
    # ------------------------------------------------------------------

    @staticmethod
    def _teacher_converter(teacher):
        """Teacher'in ICINDEKI HCAirAcroConverter'i al.

        ARKADASIN DOSYASINA DOKUNMAMAK ICIN ozel alana bakiyoruz. HCAir'in
        public yuzeyinde converter'i veren bir metot YOK (sadece
        try_load_acro_from_vehicle ve channel_limits). Alternatif, hc_air_5.py'ye
        iki satir accessor eklemekti; o zaman dosya arkadasin gonderdiginden
        SAPARDI ve sonraki paket birlestirmesi elle catisma cozmek olurdu.
        Bu dosya hc_air_5.py'yi BIREBIR birakir.

        Bedeli: upstream alani yeniden adlandirirsa buras kirilir. Bu yuzden
        test_acro_rc_offline.py bu bagi KILITLIYOR — kirilirsa testte,
        ucusta degil, ortaya cikar.
        """
        if teacher is None:
            raise ValueError(
                "AcroRcSender teacher= ister. ACRO PWM'i teacher uretiyor; "
                "teacher olmadan gonderecek bir sey yok.")
        conv = getattr(teacher, '_acro_converter', None)
        if conv is None:
            mode = getattr(teacher, 'output_mode', '?')
            raise ValueError(
                f"teacher'da acro converter YOK (output_mode={mode!r}). "
                "hc_air_5'i output_mode='acro_rc' ile kur — controller "
                "--acro bayragi bunu yapar. output.mode=body_rate ile "
                "kurulmus teacher rad/s uretir, PWM degil.")
        return conv

    # ------------------------------------------------------------------
    # KURULUM
    # ------------------------------------------------------------------

    def _rate_envelope_dps(self):
        """--acro-max-rate ve --max-rate'in KUCUGU (deg/s) ya da None."""
        vals = [v for v in (self.acro_max_rate_dps, self.max_rate_dps)
                if v is not None]
        return min(float(v) for v in vals) if vals else None

    def _narrow_rate_limits(self, dps):
        """Teacher'in converter zarfini daralt (deg/s).

        try_load_from_vehicle BASARILI olursa adapter'i YENI bir nesneyle
        degistiriyor ve limits'i yaml'in rate_limits blogundan tazeliyor —
        yani bu daraltmayi SILIYOR. Bu yuzden setup() icinde FC okumasindan
        SONRA tekrar cagrilir. Acilista iki 'rate zarfi daraltildi' satiri
        gorursen DOGRU calisiyor; tek satir gorursen bir sey ters
        (31 Agu notu, acro_sender.py ile ayni tuzak).
        """
        from fpv_gate_il.teacher.ardupilot_acro_adapter import PhysicalRateLimits
        lim = self.adapter.limits
        r = math.radians(float(dps))
        self.adapter.limits = PhysicalRateLimits(
            p_max=min(lim.p_max, r),
            q_max=min(lim.q_max, r),
            r_max=min(lim.r_max, r),
            max_rp_stick_radius=lim.max_rp_stick_radius,
        )
        print(f"[ACRO-RC] rate zarfi daraltildi: p/q/r <= {dps:.1f} deg/s "
              f"({r:.3f} rad/s)")

    def setup(self):
        """Teacher'in converter'ini FC paramlariyla tazele + kanal haritasini dogrula."""
        print("[ACRO-RC] Kurulum basliyor...")
        print(f"[ACRO-RC] cevrim TEACHER icinde (output_mode="
              f"{getattr(self.teacher, 'output_mode', '?')}) — bu katman "
              f"sadece RC_CHANNELS_OVERRIDE tasiyor")

        env = self._rate_envelope_dps()
        if env is not None:
            self._narrow_rate_limits(env)

        # --- ACRO/RC paramlari FC'den (teacher'in converter'ina) ---
        # Converter'i DOGRUDAN cagiriyoruz, teacher'in sarmalayicisini degil:
        # sarmalayici attempts/timeout gecirmiyor (dosya basindaki nota bak).
        try:
            src = self.converter.try_load_from_vehicle(
                self.conn,
                attempts=self.acro_param_attempts,
                timeout_per_attempt=self.acro_param_timeout,
            )
        except TypeError:
            # Upstream converter (attempts kwarg'i olmayan surum) — yine dene,
            # sadece kadans upstream'in sabit 3 x 3.0 s ayarinda kalir.
            print("[ACRO-RC] NOT: converter attempts/timeout kabul etmiyor "
                  "(upstream surum) — kadans 3 x 3.0 s")
            src = self.converter.try_load_from_vehicle(self.conn)

        # FC okumasi adapter'i DEGISTIRDI -> referanslari tazele, zarfi
        # tekrar daralt (yukaridaki _narrow_rate_limits notu).
        self.adapter = self.converter.adapter
        if env is not None:
            self._narrow_rate_limits(env)

        # --- DIKEY KANAL: hover_thrust_firmware'i COZ (FC OKUMASI BUNU DUZELTMEZ) ---
        if not self._resolve_hover_fw():
            return False

        # --- KALIBRASYON KAPISI: okunamayan mapping ile CANLI UCMA ---
        if not self._gate_unverified_mapping(src):
            return False

        print(f"[ACRO-RC] FC zarf: {self.adapter.capability_report()}")
        print(f"[ACRO-RC] dikey zinciri: teacher hover {self.hover_motor:.3f} "
              f"-> firmware {self.hover_thrust_fw:.3f} -> gaz PWM "
              f"{self.hover_pwm}  (thrust_scale={self.converter.thrust_scale})")

        rmap = self.converter.rc_channel_map
        print(f"[ACRO-RC] override kanallari: roll=CH{rmap[0]} pitch=CH{rmap[1]} "
              f"throttle=CH{rmap[2]} yaw=CH{rmap[3]}  "
              f"(CH5/6/8/11/12/13 DOKUNULMAZ)")
        for ch in rmap:
            if ch in (5, 6, 8):
                print(f"[ACRO-RC] HATA: RCMAP kanal {ch} HUNTER'da mod/gate/"
                      f"pencere kanali. FC RCMAP_* ayarini kontrol et.")
                return False

        self._print_governor_summary()
        self.ready = True
        return True

    def _gate_unverified_mapping(self, src):
        """FC'den okunamayan rate/thrust->PWM mapping'i ile CANLI ucusu KES.

        =================================================================
        9 KAS 2026 — SAHA LOGUNUN ORTAYA CIKARDIGI TUZAK
        =================================================================
        Orin kosusunda ayni fiziksel istek iki yaml'da bambaska PWM uretti:

            istenen firmware thrust 0.101
              fpv_gate_5.yaml  (rc 1000/1500/2000, thr_mid 0.5)   -> 1101
              teacher_hc_air_3.yaml (rc 1045/1495/1945, thr_mid .125) -> 1465

        364 us fark. Ve asil mesele PWM'in kendisi degil, FC'nin onu NASIL
        YORUMLADIGI: FC kendi gercek kalibrasyonunu kullanir. 1101'i
        1045/1495/1945 + thr_mid 0.125 ile yorumlayinca cubuk %6.2 cikiyor,
        yani GERCEK thrust 0.0002 — hover isterken gaz NEREDEYSE KAPALI.
        ACRO angajmaninda drone duser.

        Ekranda ise `firmware 0.101 -> gaz PWM 1101` yaziyor; sayi makul
        gorunuyor. Klasik sessiz tuzak.

        NEDEN ESKI YOLDA UYARI YETIYORDU: acro_sender teacher_hc_air_3.yaml
        okuyor ve o dosyanin acro: blogu mav.parm'dan ELLE doldurulmus,
        FC ile birebir. Orada yaml fallback GUVENLI. fpv_gate_5.yaml ise
        arkadasin SIM degerlerini tasiyor — ayni fallback burada guvenli
        DEGIL. Risk farkli, kapi da farkli.

        KURAL: canli komut gonderiyorsak (--enable-override) ortada bir FC
        VAR; kalibrasyonunu okuyamadiysak mapping DOGRULANMAMIS demektir ve
        dogrulanmamis bir gaz mapping'i ucagi dusuren seydir. Kes.
        Dry-run'da sadece uyar — orada gonderilen komut yok, boru hattini
        kostursun.
        """
        if src == 'vehicle':
            return True

        print("[ACRO-RC] " + "!" * 56)
        print("[ACRO-RC] !! rate/thrust->PWM konfigi FC'den OKUNAMADI "
              "(yaml fallback).")
        print(f"[ACRO-RC] !! Bu mapping ile hover icin uretilecek gaz PWM: "
              f"{self.hover_pwm}")
        print("[ACRO-RC] !! FC o PWM'i KENDI kalibrasyonuyla yorumlar. "
              "Yaml FC ile ayni degilse")
        print("[ACRO-RC] !! istedigin thrust ile giden thrust ALAKASIZ olur "
              "(9 Kas: 0.101 isteyip 0.0002 gonderme).")
        print("[ACRO-RC] " + "!" * 56)

        if not self.enabled:
            print("[ACRO-RC] DRY-RUN: komut gonderilmiyor, devam ediliyor. "
                  "Canli kosuda bu KESER.")
            return True

        if self.accept_yaml_cal:
            print("[ACRO-RC] --acro-accept-yaml-cal verildi: BILEREK devam "
                  "ediliyor. Yaml'in FC ile birebir oldugunu DOGRULADIN.")
            return True

        print("[ACRO-RC] BASLATILMIYOR — canli ucusta dogrulanmamis mapping "
              "kabul edilmiyor. Uc yol var:")
        print("[ACRO-RC]   1) Param linkini duzelt. Dogru cozum bu: acilista "
              "'config source: vehicle (MAVLink params)' gormelisin.")
        print("[ACRO-RC]      MAVProxy --streamrate=-1 mi, PARAM_VALUE router "
              "filtresine takiliyor mu, baud/port dogru mu?")
        print("[ACRO-RC]   2) --teacher hc_air_3 --teacher-config "
              "kcf/teacher_hc_air_3.yaml ile uc. O dosyanin acro: blogu")
        print("[ACRO-RC]      mav.parm'dan elle dolduruldu (1045/1495/1945, "
              "dz 0, thr_mid 0.125, hover_fw 0.101) — fallback GUVENLI.")
        print("[ACRO-RC]   3) Yaml'in FC ile birebir oldugundan EMINSEN "
              "--acro-accept-yaml-cal ver.")
        return False

    def _resolve_hover_fw(self):
        """hover_thrust_firmware'i coz ve converter'a YAZ. False = baslatma.

        =================================================================
        NEDEN AYRI BIR KAPI — FC OKUMASI BU DEGERI DUZELTMEZ
        =================================================================
        load_adapter_from_vehicle FC'den sunlari okuyor ve yaml'i EZIYOR:
            ACRO_RP_RATE / ACRO_RP_EXPO / ACRO_Y_RATE / ACRO_Y_EXPO
            RCMAP_* ve RC{n}_MIN/TRIM/MAX/DZ/REVERSED
            thr_mid_curve  (ACRO_THR_MID, yoksa clamp(MOT_THST_HOVER,.125,.6875))
        Yani arkadasin yaml'indaki rc kalibrasyonu, expo ve thr_mid sahada
        KENDILIGINDEN dogruya cekiliyor.

        AMA hover_thrust_firmware ve thrust_scale FC'de KARSILIGI OLMAYAN,
        yalnizca yaml'dan gelen degerler. teacher_thrust_to_firmware:
            firmware = hover_thrust_firmware
                     + (thrust_cmd - platform.hover_thrust) * thrust_scale
        yani teacher hover'inda cikti DOGRUDAN hover_thrust_firmware'dir.

        fpv_gate_5.yaml 0.33 tasiyor (sim degeri). HUNTER'in sahada olculmus
        hover'i 0.101-0.102 (27 Agu, mini-G1). Senin RC kalibrasyonun ve
        thr_mid=0.125 ile:
            0.33  -> gaz PWM 1667  (cubuk %69)
            0.101 -> gaz PWM 1465  (cubuk %47)
        202 us fark = teacher hover isterken ~3.3x hover thrust. Ilk ACRO
        angajmaninda sert tirmanis. Ve bu SESSIZ olurdu: acilista "config
        source: vehicle" yazar, rc/expo/thr_mid dogru okunmus gorunur,
        tek bozuk sayi yaml'dan gelmeye devam ederdi.

        Kapilar guided_sender/acro_sender ile AYNI sabitleri kullaniyor,
        yani saha ritmi degismiyor: --hover 0.101 ne yapiyorsa burada da
        onu yapar.
        """
        yaml_fw = float(self.converter.hover_thrust_firmware)
        src = None

        if self.hover_override is not None:
            fw = float(self.hover_override)
            src = "--hover ile ELLE"
        else:
            val = self.read_param('MOT_THST_HOVER')
            if val is not None:
                fw = float(val)
                src = "FC MOT_THST_HOVER"
            else:
                fw = yaml_fw
                src = "yaml fpv_gate_5.yaml acro.hover_thrust_firmware"
                print("[ACRO-RC] " + "!" * 56)
                print("[ACRO-RC] !! MOT_THST_HOVER OKUNAMADI ve --hover "
                      "verilmedi.")
                print("[ACRO-RC] !! Dikey kanal arkadasin yaml degerine "
                      f"({yaml_fw:.3f}) dusuyor.")
                print("[ACRO-RC] !! Bu deger SIM icin; HUNTER'in olculmus "
                      "hover'i 0.101-0.102. --hover ile VER.")
                print("[ACRO-RC] " + "!" * 56)

        if not (HOVER_SANE_MIN <= fw <= HOVER_SANE_MAX):
            print(f"[ACRO-RC] HATA: hover_thrust_firmware={fw:.3f} makul "
                  f"bandin ({HOVER_SANE_MIN}-{HOVER_SANE_MAX}) DISINDA. "
                  f"Kaynak: {src}")
            return False
        if self.hover_override is None and abs(fw - 0.35) < 1e-4:
            print("[ACRO-RC] HATA: MOT_THST_HOVER = 0.350 = OGRENILMEMIS "
                  "fabrika degeri. AltHold'da ogren ya da --hover ile ver.")
            return False

        # Yaml ile cozulen deger ciddi ayriliyorsa SAYIYI goster — "config
        # source: vehicle" satirina guvenip bu farki atlamak kolay.
        self.converter.hover_thrust_firmware = fw
        self.hover_thrust_fw = fw
        self.hover_pwm = int(self.adapter.throttle_pwm_for_thrust(fw))
        if abs(fw - yaml_fw) > 0.02:
            yaml_pwm = int(self.adapter.throttle_pwm_for_thrust(yaml_fw))
            print(f"[ACRO-RC] dikey DUZELTILDI: yaml {yaml_fw:.3f} "
                  f"(PWM {yaml_pwm}) → {fw:.3f} (PWM {self.hover_pwm})  "
                  f"fark {yaml_pwm - self.hover_pwm:+d} us   kaynak: {src}")
        else:
            print(f"[ACRO-RC] dikey: hover_thrust_firmware {fw:.3f} "
                  f"({src})")

        if abs(fw - HOVER_AP_FLOOR) < 1e-4:
            print(f"[ACRO-RC] NOT: hover = {HOVER_AP_FLOOR} = ArduPilot ALT "
                  "TABANI. Gercek hover bunun ALTINDA olabilir "
                  "(saha olcumu 0.101-0.102). mini-G1 ile dogrula.")
        return True

    def _print_governor_summary(self):
        """Hangi kelepce ACIK — sessiz etkisizlige karsi."""
        env = self._rate_envelope_dps()
        sc = self.axis_scale
        parts = []
        if env is not None:
            parts.append(f"rate zarfi <= {env:.1f} deg/s (TAM)")
        eff = {k: self.rate_scale * sc[k] for k in ('roll', 'pitch', 'yaw')}
        if any(abs(v - 1.0) > 1e-9 for v in eff.values()):
            parts.append(
                "PWM olcegi r={roll:.2f} p={pitch:.2f} y={yaw:.2f} "
                "(YAKLASIK — expo)".format(**eff))
        if abs(sc['thrust']) < 1e-9:
            parts.append(f"gaz HOVER PWM'ine KILITLI ({self.hover_pwm})")
        elif abs(sc['thrust'] - 1.0) > 1e-9:
            parts.append(f"gaz olcegi {sc['thrust']:.2f}")
        if self.thrust_dev is not None:
            parts.append(f"gaz bandi hover +- {self.thrust_dev:.3f}")
        if self.thrust_cap is not None:
            parts.append(f"gaz tavani {self.thrust_cap:.3f}")
        if parts:
            print("[ACRO-RC] governor: " + " · ".join(parts))
        else:
            print("[ACRO-RC] governor: TAM YETKI (hicbir kelepce yok)")

    def enter_guided(self):
        """Isim GuidedSender API'si ile ayni; gercekte ACRO'ya gecer."""
        if not self.ready:
            print("[ACRO-RC] HATA: setup() cagrilmadan enter_guided() olmaz")
            return False
        ok = self._set_mode(MODE_ACRO, 'ACRO')
        self.in_guided = ok
        return ok

    def reset(self):
        self.converter.reset()

    # ------------------------------------------------------------------
    # PWM UZAYINDA GOVERNOR
    # ------------------------------------------------------------------

    @staticmethod
    def _scale_about(pwm, pivot, scale, lo, hi):
        """pivot etrafinda olcekle, sonra [lo, hi]'ya kelepce. Tam sayi doner.

        reversed eksende de dogru: pivot trim PWM'i, olcek sapmayi kucultur,
        isaret neyse o kalir.
        """
        v = float(pivot) + (float(pwm) - float(pivot)) * float(scale)
        return int(round(min(max(v, float(lo)), float(hi))))

    def _apply_pwm_governor(self, roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm):
        """Saha merdivenini PWM uzayinda uygula. Rate zarfi AYRICA daraltildi."""
        cfg = self.adapter.config
        sc = self.axis_scale
        out = [int(roll_pwm), int(pitch_pwm), int(throttle_pwm), int(yaw_pwm)]

        # --- SON KAPI: her kanal KENDI RC bandina kelepcelenir ---
        # Teacher zaten band icinde PWM uretiyor (adapter cevrimi oyle
        # kuruluyor), yani normalde burasi hicbir seyi degistirmez. Yine de
        # KOSULSUZ duruyor: buradan cikan sayi dogrudan
        # RC_CHANNELS_OVERRIDE'a gidiyor. Bant disi bir deger (bozuk config,
        # teacher'da NaN->int, ileride bir regresyon) FC'ye ham gider ve
        # negatif bir sayi uint16 paketlemesinde bambaska bir PWM'e doner.
        # Olcek kelepceleri KOSULLU oldugu icin (scale==1 ise atlanir)
        # governor'a guvenmek yetmez — 9 Kas offline testi tam bunu yakaladi.
        for idx, cal in ((0, cfg.roll_cal), (1, cfg.pitch_cal),
                         (3, cfg.yaw_cal)):
            out[idx] = int(min(max(out[idx], int(cal.min_pwm)),
                               int(cal.max_pwm)))

        for idx, (cal, key) in enumerate(((cfg.roll_cal, 'roll'),
                                          (cfg.pitch_cal, 'pitch'))):
            eff = self.rate_scale * sc[key]
            if abs(eff - 1.0) > 1e-9:
                out[idx] = self._scale_about(out[idx], cal.trim_pwm, eff,
                                             cal.min_pwm, cal.max_pwm)
        eff_yaw = self.rate_scale * sc['yaw']
        if abs(eff_yaw - 1.0) > 1e-9:
            out[3] = self._scale_about(out[3], cfg.yaw_cal.trim_pwm, eff_yaw,
                                       cfg.yaw_cal.min_pwm, cfg.yaw_cal.max_pwm)

        # --- DIKEY ---
        tcal = cfg.throttle_cal
        thr_lo, thr_hi = int(tcal.min_pwm), int(tcal.max_pwm)

        # --scale-thrust: hover PWM'ine dogru olcek. 0 = hover'a KILIT
        # (sifir-yetki hover testi, G1 merdiveni).
        if abs(sc['thrust'] - 1.0) > 1e-9:
            out[2] = self._scale_about(out[2], self.hover_pwm, sc['thrust'],
                                       thr_lo, thr_hi)

        # --thrust-dev: hover +- dev bandi (saha). Band hover'i ICERIR ->
        # drone her zaman askida kalabilir.
        if self.thrust_dev is not None:
            lo = self.adapter.throttle_pwm_for_thrust(
                max(0.0, self.hover_thrust_fw - self.thrust_dev))
            hi = self.adapter.throttle_pwm_for_thrust(
                min(1.0, self.hover_thrust_fw + self.thrust_dev))
            lo, hi = min(lo, hi), max(lo, hi)
            out[2] = int(min(max(out[2], lo), hi))

        # --thrust-cap: mutlak ust sinir (bench). Her seyi EZER — guided_sender
        # ve acro_sender'daki sirayla ayni mantik.
        if self.thrust_cap is not None:
            cap_pwm = int(self.adapter.throttle_pwm_for_thrust(
                min(1.0, max(0.0, self.thrust_cap))))
            if out[2] > cap_pwm:
                self.thrust_cap_hits += 1
                out[2] = cap_pwm

        out[2] = int(min(max(out[2], thr_lo), thr_hi))

        if any(int(a) != int(b) for a, b in
               zip(out, (roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm))):
            self.pwm_governor_hits += 1
        return out[0], out[1], out[2], out[3]

    # ------------------------------------------------------------------
    # GONDERIM
    # ------------------------------------------------------------------

    def send(self, action, roll_deg=0.0, pitch_deg=0.0, body=None):
        """
        action: [roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm] — TAM SAYI us.
                hc_air_5'in output_mode='acro_rc' cikisi.
        body  : teacher.last_debug (istege bagli). body_*_cmd alanlarindan
                CSV'ye gercek gövde komutu yazilir; yoksa converter'in
                'temsil edilen' rate'leri kullanilir.
        roll_deg/pitch_deg: BU YOLDA KULLANILMAZ. Tilt telafisi teacher'in
                icinde (platform.tilt_cos_min). Imza GuidedSender ile ayni
                kalsin diye duruyor — controller akisi degismiyor.

        Donus: (roll_rate, pitch_rate, yaw_rate, thrust_cmd) — loglamak icin,
               GuidedSender.send() ile AYNI imza.
        """
        raw = (int(round(float(action[0]))), int(round(float(action[1]))),
               int(round(float(action[2]))), int(round(float(action[3]))))
        roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm = self._apply_pwm_governor(*raw)

        # --- CSV/durum icin gövde komutu ---
        # Teacher'in last_debug'i body_*_cmd'yi veriyor (cevrim ONCESI,
        # gercek komut). Yoksa converter'in temsil ettigi rate'lere dusuyoruz.
        rates = (0.0, 0.0, 0.0)
        thrust = float(self.hover_motor)
        if body:
            try:
                rates = (float(body['body_roll_cmd']),
                         float(body['body_pitch_cmd']),
                         float(body['body_yaw_cmd']))
                thrust = float(body['body_throttle_cmd'])
            except (KeyError, TypeError, ValueError):
                body = None
        if not body:
            mapped = getattr(self.converter, '_last_mapped', None)
            if mapped is not None:
                rep = mapped.represented
                rates = (float(rep.p), float(rep.q), float(rep.r))
            thrust = float(self.adapter.thrust_for_throttle_pwm(throttle_pwm))

        self.last_rates = rates
        self.last_thrust = thrust
        self.last_pwm = (roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm)
        self.last_pwm_raw = raw

        if self.enabled and self.conn is not None:
            try:
                self.adapter.send(self._mav_target(), self._mapped_for(
                    roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm))
                self.override_active = True
            except Exception as e:
                print(f"[ACRO-RC] gonderim hatasi: {e}")

        return rates[0], rates[1], rates[2], thrust

    def _mapped_for(self, roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm):
        """adapter.send()/pack_channels() icin MappedRateCommand hazirla.

        pack_channels sadece dort PWM alanini okuyor, ama adapter.send()
        imzasi MappedRateCommand bekliyor ve RCMAP eslemesini (CH5/6/8'e
        65535 = 'yok say' yazma dahil) O yapiyor. Elle 8 kanal paketlemek
        o sahada dogrulanmis eslemeyi KOPYALAMAK olurdu; onun yerine
        teacher'in urettigi komutun PWM alanlarini degistiriyoruz.
        """
        mapped = getattr(self.converter, '_last_mapped', None)
        if mapped is None:
            # Ilk kare / teacher henuz cevirmedi: notr komut uret, PWM'leri ez.
            mapped = self.adapter.neutral_command(int(throttle_pwm))
        return replace(
            mapped,
            roll_pwm=int(roll_pwm),
            pitch_pwm=int(pitch_pwm),
            throttle_pwm=int(throttle_pwm),
            yaw_pwm=int(yaw_pwm),
        )

    def send_neutral(self):
        """Cubuklar trim'de, gaz HOVER PWM'inde."""
        cfg = self.adapter.config
        return self.send([cfg.roll_cal.trim_pwm, cfg.pitch_cal.trim_pwm,
                          self.hover_pwm, cfg.yaw_cal.trim_pwm])

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
        demektir ve mevcut override'i TUTAR. adapter.release() 1..8
        kanala 0 yaziyor.
        """
        if not self.enabled or self.conn is None:
            self.override_active = False
            return
        try:
            self.adapter.release(self._mav_target())
            self.override_active = False
        except Exception as e:
            print(f"[ACRO-RC] release hatasi: {e}")

    def release_burst(self, n=5):
        """Paket kaybina karsi n kez birak. Kesme yolunda bloklamaz."""
        for _ in range(max(1, n)):
            self.release()

    def debug(self):
        d = dict(self.converter.last_mapping_debug())
        d['acro_pwm_gov'] = float(self.pwm_governor_hits)
        d['acro_hover_pwm'] = float(self.hover_pwm)
        return d
