#!/usr/bin/env python3
"""
HUNTER — GUIDED_NOGPS kontrolcüsü (yeni buton haritası)

controller_orin.py'ye DOKUNMAZ — ondan import eder. Güvenlik mantığı,
FrameClient, FlightLogger, StateMachine TEK KOPYA kalır.

KANAL HARİTASI (yeni sistem)
  CH1-4  Roll/Pitch/Throttle/Yaw   — pilot (GUIDED_NOGPS'te FC yok sayar)
  CH5    Uçuş modları              — FLTMODE_CH=5, FC yönetir. ORIN OKUMAZ.
  CH6    Otonom kapısı             — LOW=kapalı · MID=güvenlik · HIGH=otonom
  CH8    Pencere aç/kapa           — runtracker.cpp (latching)
  CH9    Kilitle/bırak             — runtracker.cpp (latching)
  CH11   Arm/disarm                — RC11_OPTION=153, FC (latching ŞART)
  CH12   boşta
  CH13   ROI boyut                 — runtracker.cpp (yaylı slider)

ÇIKIŞ MODU
  --exit-mode stabilize   kapalı alan testi (GPS gerekmez)
  --exit-mode loiter      saha (GPS'li, olduğu yerde durur)

  Loiter GPS/pozisyon ister. Kapalı alanda FC bu mod değişimini REDDEDER ve
  drone GUIDED_NOGPS'te KALIR (pilot çubukları ölü!). Bu yüzden watchdog var:
  istenen mod süresi içinde tutmazsa yedek moda (STABILIZE/ACRO — ikisi de
  pozisyon istemez) otomatik düşer.

KULLANIM
  # kapalı alan, dry-run
  python3 controller_guided.py --exit-mode stabilize
  # kapalı alan, canlı (PERVANESİZ)
  python3 controller_guided.py --exit-mode stabilize --enable-override
  # saha (tam yetki)
  python3 controller_guided.py --exit-mode loiter --enable-override

SAHA MERDİVENİ (kademeli yetki — her uçuşta tek değişken):
  # G1 — sıfır yetki hover: rates=0 + hover thrust (thrust merkezi doğru mu?)
  python3 controller_guided.py --exit-mode loiter --enable-override \\
      --scale-roll 0 --scale-pitch 0 --scale-yaw 0 --scale-thrust 0
  # G2 — sadece yaw %30, sert tavan 20°/s (hedefe burun dönmesi)
  python3 controller_guided.py --exit-mode loiter --enable-override \\
      --scale-roll 0 --scale-pitch 0 --scale-thrust 0 \\
      --scale-yaw 0.3 --max-rate 20
  # G3 — pitch'i ekle, dikey hâlâ dar bantta
  python3 controller_guided.py --exit-mode loiter --enable-override \\
      --scale-roll 0 --scale-yaw 0.5 --scale-pitch 0.2 \\
      --scale-thrust 0.3 --max-rate 20 --thrust-dev 0.05
"""

import argparse
import importlib
import math
import os
import sys
import time

import numpy as np

try:
    from pymavlink import mavutil
except ImportError:
    print("HATA: pymavlink yok. pip3 install --user pymavlink")
    sys.exit(1)

# --- controller_orin'den ORTAK parçalar (o dosya DEĞİŞTİRİLMEDİ) ---
from controller_orin import (
    CAMERA_HOST, CAMERA_PORT, FC_ENDPOINT, SWITCH_HIGH_MIN, LOOP_HZ,
    FrameClient, FlightLogger, StateMachine, FCConnection, write_run_config,
)
from guided_sender import GuidedSender, MODE_ACRO, MODE_GUIDED_NOGPS
# 31 Agu 2026: FloorMonitor ayri dosyada. Sebep floor_monitor.py basliginda:
# ACRO yamasini pre-patch snapshot uzerine kurdum ve bu dosyadaki taban
# sigortasini EZDIM. Ayri dosyada olursa bir daha ezilmez.
from floor_monitor import FloorMonitor, floor_preflight_check

# ---------------------------------------------------------------------------
# MOD NUMARALARI (ArduCopter 4.6.3 FLTMODE @Values'tan doğrulandı)
# ---------------------------------------------------------------------------
MODE_NUM = {
    'stabilize': 0,
    'acro': 1,
    'althold': 2,
    'loiter': 5,
    'guided_nogps': 20,
}

# Yedek mod: pozisyon kestirimi İSTEMEZ, her koşulda kabul edilir.
# STABILIZE kendi kendine dengeler → acil durumda ACRO'dan güvenli.
FALLBACK_PRIMARY = 'stabilize'
FALLBACK_SECOND = 'acro'

# Otonoma girerken GUIDED_NOGPS teyidi için tanınan süre
GUIDED_ENGAGE_TIMEOUT = 1.5      # s
# Çıkışta istenen mod tutmazsa yedeğe düşme süresi
EXIT_MODE_TIMEOUT = 0.7          # s


class GuidedFlightLogger(FlightLogger):
    """
    FlightLogger + guided kolonlari.

    Neden gerekli: FlightLogger.log() son 4 kolonu `str(int(x))` ile yaziyor —
    PWM tamsayilari icin dogru, ama guided'da oraya rad/s geliyor ve
    1.131 rad/s -> "1" olarak EZILIRDI. Burada ondalik korunuyor ve kolon
    isimleri gercege uyduruluyor.
    """

    # RATE modu   : cmd_* = rad/s
    # ATTITUDE modu: cmd_* = DERECE (roll/pitch/yaw setpoint)
    # Kolon isimleri notrleştirildi; hangi mod olduğu 'mode' kolonundan belli.
    HEADER = FlightLogger.HEADER[:-4] + [
        'cmd_roll', 'cmd_pitch', 'cmd_yaw', 'cmd_thrust',
    ]

    def log(self, mode, obs, action, cmd):
        if self.file is None:
            return
        t = time.time() - self.t0
        row = [f"{t:.4f}", mode]
        row += [f"{float(x):.6f}" for x in obs]
        row += [f"{float(x):.6f}" for x in action]
        row += [f"{float(x):.6f}" for x in cmd]      # <-- int() YOK
        self.file.write(','.join(row) + '\n')
        self.row_count += 1
        if self.row_count % 30 == 0:
            self.file.flush()


class GuidedFCConnection(FCConnection):
    """
    FCConnection + HEARTBEAT (mevcut uçuş modu).

    Neden gerekli: ArduCopter SET_ATTITUDE_TARGET'ı guided'da DEĞİLKEN
    sessizce yutuyor (GCS_Mavlink.cpp: `if (!in_guided_mode()) return;`).
    Yani modu bilmezsek "boşluğa gönderiyor" olabiliriz ve fark etmeyiz.
    Ayrıca pilot CH5 ile kaçış yaparsa ANINDA görmemiz gerekiyor.
    """

    def __init__(self, endpoint):
        super().__init__(endpoint)
        self.current_mode = None       # custom_mode (int)
        self.last_hb_time = 0.0
        # Referans implementasyon (gazebo_drone_env._drain_mavlink) LOCAL_POSITION_NED'den
        # UC bileseni birden aliyor: drone_vel_ned = [vx, vy, vz]. Biz de ayni.
        self.vel_ned = (0.0, 0.0, 0.0)   # m/s, NED (vz: +asagi)
        self.vel_valid = False
        self.vel_source = None           # 'LOCAL_POSITION_NED' | 'GLOBAL_POSITION_INT'
        self.last_vel_time = 0.0

    def poll(self):
        super().poll()                 # ATTITUDE + GLOBAL_POSITION_INT + RC_CHANNELS
        if self.conn is None:
            return
        # BUG DUZELTMESI (FC=? sorunu):
        # super().poll() `while True: recv_match(type=[3 tip], blocking=False)`
        # ile soketi KOMPLE bosaltiyor ve eslesmeyen mesajlari (HEARTBEAT dahil)
        # ATIYOR. Bu yuzden burada recv_match ile HEARTBEAT aramak BOS sokete
        # bakmak demekti -> current_mode hep None -> "FC=?" -> in_guided() hep
        # False -> otonom hicbir zaman teyit edilemiyordu.
        #
        # Cozum: pymavlink her ayristirdigi mesaji tipine gore conn.messages
        # sozlugunde saklar (post_message). recv_match onu atsa bile kayit
        # duruyor. Oradan okuyoruz — controller_orin.py'ye dokunmadan.
        hb = self.conn.messages.get('HEARTBEAT')
        if hb is not None and hb.get_srcSystem() == self.target_sys:
            self.current_mode = hb.custom_mode
            self.last_hb_time = time.time()

        # HIZ (vel_ned) — hc_air'in governor + dikey PI'i BUNU ister.
        #
        # REFERANSA HIZALANDI: gazebo_drone_env._drain_mavlink LOCAL_POSITION_NED
        # okuyor (satir ~990: drone_vel_ned = [msg.vx, msg.vy, msg.vz]). Ilk
        # entegrasyonumda GLOBAL_POSITION_INT.vz kullanmistim — calisir ama
        # referanstan sapma; env'in kendi yorumu GPS-free'de bu mesajlarin farkli
        # davrandigini not ediyor ("GPS-free: NED z may stay ~0").
        #
        # LOCAL_POSITION_NED zaten MAV_DATA_STREAM_POSITION grubunda geliyor
        # (ArduCopter GCS_Mavlink.cpp STREAM_POSITION_msgs), controller_orin
        # o stream'i zaten istiyor -> ek istek GEREKMEZ.
        #
        # NED konvansiyonu: vz +asagi. hc_air kendi icinde vz_up = -vel[2] yapiyor.
        lpn = self.conn.messages.get('LOCAL_POSITION_NED')
        if lpn is not None:
            self.vel_ned = (float(lpn.vx), float(lpn.vy), float(lpn.vz))  # m/s
            self.vel_valid = True
            self.vel_source = 'LOCAL_POSITION_NED'
            self.last_vel_time = time.time()
        else:
            # YEDEK: LOCAL_POSITION_NED hic gelmiyorsa (EKF origin yok vb.)
            gpi = self.conn.messages.get('GLOBAL_POSITION_INT')
            if gpi is not None:
                self.vel_ned = (gpi.vx / 100.0, gpi.vy / 100.0, gpi.vz / 100.0)
                self.vel_valid = True
                self.vel_source = 'GLOBAL_POSITION_INT'
                self.last_vel_time = time.time()

    # -----------------------------------------------------------------
    # OTONOM MOD KONTROLU
    #
    # 31 Agu 2026: iki tasima yolu var, otonom modu FARKLI:
    #     GUIDED yolu (rad_s / attitude_deg) -> GUIDED_NOGPS (20)
    #     ACRO yolu   (acro_pwm)             -> ACRO (1)
    # auto_mode main() icinde BIR KEZ set edilir. Varsayilan GUIDED_NOGPS,
    # yani mevcut davranis DEGISMEZ.
    #
    # Bu uc yerde kullaniliyor ve UCUNDE de dogru mod sart:
    #   1) watchdog.update()  — cikis modu tuttu mu
    #   2) pending -> engaged — otonom teyidi
    #   3) 4. kapi            — pilot CH5 ile kacti mi
    # ACRO yolunda GUIDED_NOGPS'e bakmak bunlarin UCUNU de kilitlerdi:
    # pending asla teyit edilmez, otonom hic baslamazdi.
    # -----------------------------------------------------------------
    auto_mode = MODE_GUIDED_NOGPS

    def in_auto(self):
        return self.current_mode == self.auto_mode

    def in_guided(self):
        """Geriye uyumluluk adi. Gercekte 'otonom modda miyiz' demek."""
        return self.in_auto()

    def mode_name(self):
        for k, v in MODE_NUM.items():
            if v == self.current_mode:
                return k
        return f"mode{self.current_mode}"


class ExitModeWatchdog:
    """
    Otonomdan çıkışta modu güvenceye alır — BLOKLAMADAN.

    trigger()  : çıkış kenarında istenen modu ateşler (5 paket, bekleme yok)
    update()   : her döngü çağrılır. Mod hâlâ otonom modsa ve süre dolduysa
                 yedek moda düşer. Yedek de tutmazsa ikinci yedeğe.

    Neden kademeli: Loiter pozisyon ister; kapalı alanda / GPS kaybında FC
    reddeder. Reddedilirse drone GUIDED'da kalır ve pilot ÇUBUKLARI ÖLÜDÜR.
    STABILIZE ve ACRO pozisyon istemez, her zaman kabul edilir.

    =====================================================================
    ACRO YOLU (31 Agu 2026) — TEHLIKE SIRALAMASI TERS
    =====================================================================
    GUIDED yolunda pilotu kurtaran sey MOD DEGISIMIDIR: GUIDED'dan cikinca
    cubuklar canlanir. ACRO yolunda ise mod degisimi TEK BASINA YETMEZ,
    hatta TEHLIKELIDIR:

      RC_CHANNELS_OVERRIDE MODDAN BAGIMSIZDIR. ACRO'dan ALTHOLD'a gecsen
      bile override hala CH1-4'u suruyor. ALTHOLD o PWM'i PILOT CUBUGU
      sanar. Hover thrust 0.101'in PWM karsiligi orta cubugun ALTINDA —
      yani ALTHOLD "pilot alcalmak istiyor" diye okur ve PILOT_SPEED_DN
      ile ALCALIR. Mod degistirdik diye rahatlarken drone iner.

    Bu yuzden ACRO yolunda sira KESIN:
        1. release  (0 gonder, 65535 DEGIL — 65535 override'i TUTAR)
        2. mod komutu
    ve armed oldugu SURECE her donguda tekrar release edilir; tek paket
    kaybi 3 saniye boyunca eski PWM'in surmesi demek.

    ACRO yolunda "hicbir mod tutmadi" da felaket DEGIL: override birakildiysa
    zaten ACRO'dayiz ve ACRO pilotun kendi modu. Mesaj ona gore.
    =====================================================================
    """

    def __init__(self, sender, fc, primary, acro_path=False):
        self.sender = sender
        self.fc = fc
        self.primary = primary
        self.acro_path = bool(acro_path)
        chain = [primary]
        for m in (FALLBACK_PRIMARY, FALLBACK_SECOND):
            if m not in chain:
                chain.append(m)
        self.chain = chain
        self.armed = False
        self.stage = 0
        self.t_stage = 0.0

    def _release(self):
        """ACRO yolunda override birak. GUIDED yolunda no-op."""
        if self.acro_path:
            self.sender.release_burst(5)

    def _fire(self, name):
        # SIRA ONEMLI: once birak, sonra mod. Ters olursa override yeni modda
        # pilot cubugu gibi davranir (yukaridaki nota bak).
        self._release()
        self.sender.set_mode_fast(MODE_NUM[name], n=5)

    def trigger(self):
        self.armed = True
        self.stage = 0
        self.t_stage = time.time()
        self._fire(self.chain[0])

    def update(self):
        if not self.armed:
            return
        # Armed oldugu SURECE tekrar birak — paket kaybina karsi.
        self._release()
        if self.fc.current_mode is None:
            return                              # heartbeat henüz yok, bekle
        if not self.fc.in_auto():
            print(f"[EXIT] Mod = {self.fc.mode_name()} ✓ pilot kumandada")
            self.armed = False
            return
        if time.time() - self.t_stage < EXIT_MODE_TIMEOUT:
            return
        self.stage += 1
        if self.stage < len(self.chain):
            nm = self.chain[self.stage]
            print(f"[EXIT] !! '{self.chain[self.stage-1]}' TUTMADI "
                  f"(FC reddetti — pozisyon kestirimi yok olabilir). "
                  f"Yedek moda geçiliyor: {nm}")
            self.t_stage = time.time()
            self._fire(nm)
        else:
            if self.acro_path:
                self._release()
                print("[EXIT] Hiçbir yedek mod tutmadı — ACRO'da kalındı. "
                      "Override BIRAKILDI, çubuklar CANLI. "
                      "İstersen CH5 ile mod değiştir.")
            else:
                print("[EXIT] !!! HİÇBİR MOD TUTMADI — CH5 mod anahtarını ELLE OYNAT !!!")
            self.armed = False


def main():
    p = argparse.ArgumentParser(description="HUNTER GUIDED_NOGPS kontrolcüsü")
    p.add_argument('--cam-host', default=CAMERA_HOST)
    p.add_argument('--cam-port', type=int, default=CAMERA_PORT)
    p.add_argument('--fc', default=FC_ENDPOINT)
    p.add_argument('--teacher', default='mpc_teacher1',
                   help="Kontrolcü modülü (.py'siz)")
    p.add_argument('--action-units',
                   choices=['normalized', 'rad_s', 'attitude_deg', 'acro_pwm'],
                   default='normalized',
                   help="normalized = PNTeacher (-1..+1) | rad_s = hc_air/hc_air_3 "
                        "(gövde hızı, GUIDED + type_mask=128) | attitude_deg = "
                        "hc_air_1/2/4 (tutum açısı, GUIDED + QUATERNION) | "
                        "acro_pwm = hc_air/hc_air_3 (gövde hızı, ACRO + "
                        "RC_CHANNELS_OVERRIDE — acro_sender.py)")
    # --- ACRO YOLU (--action-units acro_pwm) ---
    p.add_argument('--acro-max-rate', type=float, default=None,
                   help="ACRO adapter fiziksel zarfını daraltır (deg/s). "
                        "--max-rate teacher KOMUTUNU kırpar, bu ADAPTER'i "
                        "kısıtlar; ikisi birlikte çalışır. İlk ACRO uçuşunda "
                        "20 gibi düşük bir değerle gir.")
    p.add_argument('--acro-no-tilt-comp', action='store_true',
                   help="ACRO'da angle boost KAPALI olduğu için acro_sender "
                        "dikey telafiyi kendi ekler. Bu bayrak onu kapatır — "
                        "SADECE PERVANESİZ BENCH.")
    p.add_argument('--teacher-config', default=None,
                   help="Teacher YAML yolu (hc_air_1 için fpv_gate.yaml). "
                        "Verilmezse kcf/ içinde aranır.")
    p.add_argument('--exit-mode', choices=['stabilize', 'acro', 'althold', 'loiter'],
                   default='loiter',
                   help="Otonomdan çıkınca dönülecek mod. VARSAYILAN=loiter (SAHA). "
                        "Kapalı alan testinde: --exit-mode stabilize")
    p.add_argument('--hover', type=float, default=None,
                   help="MOT_THST_HOVER elle (verilmezse FC'den okunur)")
    p.add_argument('--guid-timeout', type=float, default=0.5)
    p.add_argument('--rate-scale', type=float, default=1.0,
                   help="Tum rate komutlarini carpar (gimbal gozlem icin). "
                        "0.3 = 3 kat yavas donus. SAHADA 1.0 BIRAK.")
    p.add_argument('--thrust-cap', type=float, default=None,
                   help="Motor thrust ust siniri 0..1 (kapali alan/gimbal testi). "
                        "Onerilen 0.15-0.25. SAHADA KULLANMA — drone havalanamaz.")
    # --- SAHA MERDIVENI (kademeli yetki) — governor guided_sender'da, ---
    # --- teacher'dan BAGIMSIZ (PNTeacher/HCAir ikisine de uygulanir).  ---
    p.add_argument('--scale-roll',   type=float, default=1.0,
                   help="Roll rate yetkisi 0..1. 0 = eksen KAPALI (komut 0). "
                        "rate-scale ile CARPILIR.")
    p.add_argument('--scale-pitch',  type=float, default=1.0,
                   help="Pitch rate yetkisi 0..1.")
    p.add_argument('--scale-yaw',    type=float, default=1.0,
                   help="Yaw rate yetkisi 0..1.")
    p.add_argument('--scale-thrust', type=float, default=1.0,
                   help="Dikey yetki 0..1. 0 = thrust HOVER'a KILITLI "
                        "(sifir-yetki hover testi: rates=0 + hover).")
    p.add_argument('--max-rate', type=float, default=None,
                   help="Tum rate eksenlerine SERT tavan (deg/s), olcekten "
                        "bagimsiz. Orn 20 = hicbir eksen 20 deg/s'i gecemez.")
    p.add_argument('--thrust-dev', type=float, default=None,
                   help="Motor thrust'in hover'dan max sapmasi (orn 0.05). "
                        "Hover bandin ICINDE -> ucusa uygun. thrust-cap'ten "
                        "farki: cap hover'in altina inebilir (bench), bu inemez.")
    # --- ATTITUDE-HOLD gimbal sınırları (SAHADA KULLANMA) ---
    p.add_argument('--gimbal', action='store_true',
                   help="GIMBAL TEST ön ayarı: roll=0 (kilitli eksen), "
                        "pitch ±8°, yaw ofset ±15°. SAHADA KULLANMA — "
                        "teacher tuning'i bozulur.")
    p.add_argument('--att-roll-max', type=float, default=None,
                   help="attitude modunda roll |max| derece. 0 = roll KAPALI "
                        "(gimbal roll ekseni kilitliyse doğrusu budur).")
    p.add_argument('--att-pitch-max', type=float, default=None,
                   help="attitude modunda pitch |max| derece.")
    p.add_argument('--att-yaw-off-max', type=float, default=None,
                   help="attitude modunda yaw'ın mevcut başlıktan |max| sapması (derece).")
    p.add_argument('--params-set-manually', action='store_true',
                   help="GUID_OPTIONS/GUID_TIMEOUT'u Mission Planner'dan ELLE yazdim, "
                        "kod yazmaya calismasin. PARAM_VALUE router filtresine "
                        "takiliyorsa gerekir.")
    # --- V-TRIM (20 Agu yamasi) ---
    # guided_sender'da mekanizma DURUYOR (update_voltage + _hover_eff) ama
    # controller_guided bu bayraklari gecmiyordu, yani ULASILAMAZ haldeydi.
    # Varsayilan 0.0 = TAMAMEN KAPALI, davranis birebir eski kodla ayni.
    #   H_eff = H0 + vtrim * (vref - V_filt),  delta klemp [-0.005, +0.020]
    p.add_argument('--vtrim', type=float, default=0.0,
                   help="Voltaj-hover trim egimi (1/V). 0 = KAPALI (varsayilan). "
                        "Saha olcumu ~0.0033. --vref ZORUNLU.")
    p.add_argument('--vref', type=float, default=None,
                   help="mini-G1 hover kalibrasyonunun yapildigi batarya voltaji. "
                        "--vtrim ile birlikte ZORUNLU.")
    p.add_argument('--floor-alt', type=float, default=20.0,
                   help="TABAN SIGORTASI (m). Irtifa taban+5'i asinca kurulur; "
                        "sonra taban altinda 0.25 s kalirsa otonom KESILIR ve "
                        "cikis modu komutlanir. 0 = kapali. VARSAYILAN 20.")
    p.add_argument('--skip-floor-preflight', action='store_true',
                   help="Yerde alt_rel~0 kontrolunu ATLA. Baro kayikken taban "
                        "sigortasi ISE YARAMAZ — bu bayragi ancak ne yaptigini "
                        "biliyorsan kullan.")
    p.add_argument('--enable-override', action='store_true',
                   help="GERÇEKTEN komut gönder + FC param yaz (PERVANESİZ bench!)")
    p.add_argument('--log', default=None)
    args = p.parse_args()

    # Governor bayrak dogrulamasi — saha yazim hatasi korumasi.
    # Negatif olcek EKSEN YONUNU TERSLER (sessiz felaket), >1 yetki ARTIRIR;
    # ikisi de merdivenin amaci degil. Kabul: 0..1.
    for _nm, _v in (('--scale-roll', args.scale_roll),
                    ('--scale-pitch', args.scale_pitch),
                    ('--scale-yaw', args.scale_yaw),
                    ('--scale-thrust', args.scale_thrust)):
        if not (0.0 <= _v <= 1.0):
            print(f"HATA: {_nm} {_v} gecersiz — 0.0 ile 1.0 arasi olmali "
                  f"(0=kapali, 1=tam yetki)")
            sys.exit(1)
    if args.max_rate is not None and args.max_rate <= 0:
        print(f"HATA: --max-rate {args.max_rate} gecersiz — pozitif deg/s olmali")
        sys.exit(1)
    if args.thrust_dev is not None and args.thrust_dev < 0:
        print(f"HATA: --thrust-dev {args.thrust_dev} gecersiz — >= 0 olmali")
        sys.exit(1)

    dry_run = not args.enable_override

    # --gimbal ön ayarı: elle verilmemiş olanları doldurur (elle verilen kazanır)
    if args.gimbal:
        if args.att_roll_max is None:
            args.att_roll_max = 0.0      # roll ekseni KİLİTLİ → komut gönderme
        if args.att_pitch_max is None:
            args.att_pitch_max = 8.0
        if args.att_yaw_off_max is None:
            args.att_yaw_off_max = 15.0

    print("=" * 70)
    print(" HUNTER — GUIDED_NOGPS KONTROLCÜSÜ")
    print("=" * 70)
    print(" CH5=uçuş modu(FC) · CH6=otonom · CH8=pencere · CH9=kilit · CH11=arm")
    print(f" Çıkış modu: {args.exit_mode.upper()}"
          f"   (yedek: {FALLBACK_PRIMARY} → {FALLBACK_SECOND})")
    if args.exit_mode == 'loiter':
        print(" >>> SAHA MODU (varsayılan). Loiter GPS/pozisyon ister.")
        print(" >>> Kapalı alandaysan: Ctrl-C, --exit-mode stabilize ile başlat.")
        print(" >>> (yine de unutursan Loiter reddedilir ve 0.7s'de stabilize'a düşer)")
    else:
        print(f" >>> KAPALI ALAN / ELLE SEÇİLDİ: {args.exit_mode.upper()}")
    print(f" Aksiyon birimi: {args.action_units}"
          + ("  → QUATERNION attitude-hold" if args.action_units == 'attitude_deg'
             else "  → gövde rate"))
    print(" NOT: AUTO_*_DIR / AXIS_GAIN / RC_DZ bu yolda DEVREDE DEĞİL (RC baypas).")
    print(f" MOD: {'DRY-RUN (komut GÖNDERİLMİYOR)' if dry_run else '!!! KOMUT CANLI — PERVANESİZ !!!'}")
    print("=" * 70)

    # --- Kamera ---
    cam = FrameClient(args.cam_host, args.cam_port)
    if not cam.connect():
        print("[!] runtracker FRAME server'a bağlanılamadı")
        sys.exit(1)

    # --- FC ---
    fc = GuidedFCConnection(args.fc)
    if not fc.connect():
        print("[!] FC'ye bağlanılamadı")
        sys.exit(1)

    # --- Sender ---
    # ACRO yolu (31 Agu 2026): AcroSender GuidedSender'DAN TUREDI, ayni API'yi
    # (setup/enter_guided/send/send_neutral/set_mode_fast/last_rates/
    # last_thrust) tasiyor. Bu yuzden asagidaki akisin GERI KALANI DEGISMEDI —
    # sadece hangi sinifin kuruldugu degisiyor. Governor bayraklari ayni
    # alanlardan okundugu icin ucuncu bir bayrak seridi OLUSMUYOR.
    acro_path = (args.action_units == 'acro_pwm')
    if acro_path:
        from acro_sender import AcroSender
        if args.att_roll_max is not None or args.att_pitch_max is not None \
                or args.att_yaw_off_max is not None or args.gimbal:
            print("[!] --att-*-max / --gimbal ATTITUDE yoluna aittir, "
                  "acro_pwm ile ETKISIZDIR. ACRO'da kelepce icin "
                  "--acro-max-rate ve --max-rate kullan.")
            sys.exit(1)
        SenderClass = AcroSender
        sender_kw = dict(teacher_cfg=None,        # asagida doldurulur
                         acro_tilt_comp=not args.acro_no_tilt_comp,
                         acro_max_rate_dps=args.acro_max_rate)
        print(" TASIMA: ACRO + RC_CHANNELS_OVERRIDE  (acro_sender.py)")
    else:
        SenderClass = GuidedSender
        sender_kw = dict(att_roll_max=args.att_roll_max,
                         att_pitch_max=args.att_pitch_max,
                         att_yaw_off_max=args.att_yaw_off_max)
        print(" TASIMA: GUIDED_NOGPS + SET_ATTITUDE_TARGET  (guided_sender.py)")

    sender_units = 'rad_s' if acro_path else args.action_units
    sender = None      # asagida teacher config cozuldukten SONRA kurulur
    _sender_common = dict(action_units=sender_units,
                            hover_override=args.hover,
                            guid_timeout=args.guid_timeout,
                            enabled=not dry_run,
                            params_set_manually=args.params_set_manually,
                            thrust_cap=args.thrust_cap,
                            rate_scale=args.rate_scale,
                            axis_scale={'roll': args.scale_roll,
                                        'pitch': args.scale_pitch,
                                        'yaw': args.scale_yaw,
                                        'thrust': args.scale_thrust},
                            max_rate_dps=args.max_rate,
                            thrust_dev=args.thrust_dev,
                            vtrim=args.vtrim,
                            vref=args.vref)

    # --- Kontrolcü ---
    try:
        tmod = importlib.import_module(args.teacher)
        # Teacher sinif adi modulden module DEGISIYOR:
        #   mpc_teacher1 -> PNTeacher
        #   hc_air       -> HCAir
        # controller_orin sadece HybridTeacher/PNTeacher ariyordu; HCAir'i
        # bulamayip "no attribute 'PNTeacher'" hatasi veriyordu.
        TeacherClass = None
        for cname in ('HybridTeacher', 'PNTeacher', 'HCAir', 'Teacher'):
            TeacherClass = getattr(tmod, cname, None)
            if TeacherClass is not None:
                break
        if TeacherClass is None:
            # Son care: compute_action metodu olan ilk sinifi bul
            import inspect
            for _, obj in inspect.getmembers(tmod, inspect.isclass):
                if hasattr(obj, 'compute_action'):
                    TeacherClass = obj
                    break
        if TeacherClass is None:
            raise AttributeError(
                f"'{args.teacher}' icinde compute_action'i olan bir sinif yok. "
                f"Beklenen adlar: HybridTeacher / PNTeacher / HCAir / Teacher")
    except Exception as e:
        print(f"[!] Kontrolcü '{args.teacher}' yüklenemedi: {e}")
        sys.exit(1)
    # hc_air_1 config'ini parents[3]/config/fpv_gate.yaml diye ariyor — HUNTER'in
    # duz kcf/ yapisinda tutmaz. Constructor config_path kabul ediyor, onu
    # kullaniyoruz (arkadasin dosyasina DOKUNMADAN).
    #
    # 31 Agu 2026 — CONFIG TUZAGI KAPATILDI.
    # Eski hali teacher'dan BAGIMSIZ olarak once fpv_gate.yaml'i seciyordu.
    # hc_air_3 ile bu SESSIZ FELAKET olurdu: hc_air_3 teacher_hc_air_3.yaml
    # bekler (output/acro bloklari orada), fpv_gate.yaml'da o bloklar YOK ve
    # ustelik anahtar isimleri farkli -> ya coker ya YANLIS parametrelerle
    # ucar. Artik esleme TEACHER ADINA gore ACIK:
    _CFG_BY_TEACHER = {
        'hc_air':   ['teacher_hc_air.yaml'],                  # eski rate
        'hc_air_1': ['fpv_gate.yaml'],                        # senin aci
        'hc_air_2': ['fpv_gate_2.yaml'],                      # arkadasin aci
        'hc_air_3': ['teacher_hc_air_3.yaml'],                # YENI rate
        'hc_air_4': ['fpv_gate_4.yaml'],                      # YENI aci
    }
    teacher_cfg = args.teacher_config
    if teacher_cfg is None:
        here = os.path.dirname(os.path.abspath(__file__))
        cands = _CFG_BY_TEACHER.get(args.teacher)
        if cands is None:
            cands = ['fpv_gate.yaml', 'teacher_hc_air.yaml']  # PN vb.
        for cand in cands:
            for pth in (os.path.join(here, cand),
                        os.path.join(here, 'config', cand)):
                if os.path.exists(pth):
                    teacher_cfg = pth
                    break
            if teacher_cfg:
                break
        if teacher_cfg is None and args.teacher in _CFG_BY_TEACHER:
            print(f"[!] '{args.teacher}' icin config bulunamadi: "
                  f"{cands}. --teacher-config ile ACIKCA ver.")
            print("[!] BASKA bir yaml'a DUSMUYORUM — 27 Agu 'sessizce yanlis "
                  "config' tuzagi bilerek kapatildi.")
            sys.exit(1)
    try:
        teacher = (TeacherClass(config_path=teacher_cfg) if teacher_cfg
                   else TeacherClass())
        if teacher_cfg:
            print(f"[CTRL] teacher config: {teacher_cfg}")
    except TypeError:
        teacher = TeacherClass()      # config_path kabul etmeyen teacher'lar
    teacher.reset()
    print(f"[CTRL] Kontrolcü hazır: {args.teacher}.{TeacherClass.__name__}")

    # --- Sender kurulumu (teacher config COZULDUKTEN sonra) ---
    # AcroSender teacher_hc_air_3.yaml'in `acro:` blogunu okumak zorunda,
    # o yuzden kurulum buraya kadar bekletildi.
    if acro_path:
        sender_kw['teacher_cfg'] = teacher_cfg
    sender = SenderClass(fc.conn, fc.target_sys, fc.target_comp,
                         **_sender_common, **sender_kw)
    if not sender.setup():
        print("[!] Sender kurulumu BAŞARISIZ — başlatılmıyor.")
        sys.exit(1)

    # PRIVILEGED KIME GIDER — modul adina gore ACIK karar (tahmin yok):
    #   hc_air ailesi ('hc_air', 'hc_air_1', ...) privileged DICT'i bilerek
    #     alir (vel_ned/altitude -> governor + dikey PI).
    #   mpc_teacher1 (PN) privileged'SIZ cagrilir. Imzasinda 'privileged'
    #     OLMASI yaniltici: SITL doneminden kalan PrivilegedState OBJESI
    #     bekler (.valid / .get_horizontal_speed) — dict gidince patlar
    #     (14 Agu bench: AttributeError). Dict'i objeye uydursak bile PN'in
    #     GPS yolu SESSIZCE ACILIRDI; dogrulanmis konfig GPS-free =
    #     privileged'siz cagri (controller_orin ile birebir ayni).
    use_privileged = args.teacher.startswith('hc_air')
    print("[CTRL] privileged: "
          + ("AKTIF (hc_air ailesi)" if use_privileged
             else "KAPALI — GPS-free cagri (controller_orin ile ayni)"))

    if args.vtrim > 0:
        print(f"[VTRIM] ACIK: egim {args.vtrim} 1/V, vref {args.vref} V. "
              f"H_eff = {args.hover} + vtrim*(vref - V_filt), "
              f"delta klemp [-0.005, +0.020]")
    else:
        print(f"[VTRIM] KAPALI — hover ELLE sabit "
              f"({args.hover if args.hover else 'FC MOT_THST_HOVER'}). "
              f"Batarya dustukce +0.005 trim'i SEN yapacaksin.")

    log_path = args.log or time.strftime("guided_%Y%m%d_%H%M%S.csv")
    logger = GuidedFlightLogger(log_path)
    logger.open()
    write_run_config(log_path, vars(args))

    # Otonom modu SEC — in_auto() bunu kullanir (watchdog + pending teyidi +
    # 4. kapi). ACRO yolunda GUIDED_NOGPS'e bakmak ucunu de kilitlerdi.
    fc.auto_mode = MODE_ACRO if acro_path else MODE_GUIDED_NOGPS
    print(f"[CTRL] otonom mod = {'ACRO' if acro_path else 'GUIDED_NOGPS'}")

    state = StateMachine(cam)
    watchdog = ExitModeWatchdog(sender, fc, args.exit_mode, acro_path=acro_path)

    # --- TABAN SIGORTASI ---
    floor = FloorMonitor(args.floor_alt)
    if floor.enabled:
        print(f"[FLOOR] Taban sigortasi: {args.floor_alt:.0f} m "
              f"(kurulum {args.floor_alt + FloorMonitor.ARM_MARGIN_M:.0f} m ustu, "
              f"tutma {FloorMonitor.DWELL_S:.2f} s)")
        # ON UCUS: drone YERDE dururken alt_rel ~0 olmali. 31 Agu masa
        # kosusunda 16.5 m okuyordu; oyle bir kayikla 20 m taban gercekte
        # 3.5 m eder ve sigorta ISE YARAMAZ.
        for _ in range(60):
            fc.poll()
            if fc.last_attitude_time > 0 or fc.alt_rel != 0.0:
                break
            time.sleep(0.05)
        uygun, mesaj = floor_preflight_check(fc.alt_rel)
        if not uygun:
            print("[FLOOR] " + "!" * 58)
            print("[FLOOR] " + mesaj.replace("\n", "\n[FLOOR] "))
            print("[FLOOR] " + "!" * 58)
            if not args.skip_floor_preflight:
                print("[FLOOR] BASLATILMIYOR. Duzelt, ya da bilerek "
                      "--skip-floor-preflight ver.")
                sys.exit(1)
            print("[FLOOR] --skip-floor-preflight verildi, DEVAM EDILIYOR. "
                  "Taban sigortasina GUVENME.")
        else:
            print("[FLOOR] on ucus: " + mesaj)
    else:
        print("[FLOOR] Taban sigortasi KAPALI (--floor-alt 0). "
              "Irtifa korumasi YOK.")

    dt = 1.0 / LOOP_HZ
    last_hb = 0.0
    last_status = 0.0
    last_action = [0.0, 0.0, 0.0, 0.0]

    engaged = False          # FC GERÇEKTEN GUIDED_NOGPS'te ve komut gönderiyoruz
    pending = False          # GUIDED_NOGPS komutu atıldı, teyit bekleniyor
    pilot_escape = False     # pilot CH5 ile kaçtı → CH6 kapanıp açılana dek girme
    t_pending = 0.0

    try:
        while True:
            now = time.time()
            cam.poll()
            fc.poll()
            if now - last_hb >= 1.0:
                fc.send_heartbeat()
                last_hb = now

            channels = fc.rc_channels
            state.process_rc(channels)
            watchdog.update()

            bbox = cam.get_bbox()
            ch6_high = channels.get('ch6', 1500) > SWITCH_HIGH_MIN
            if not ch6_high:
                pilot_escape = False     # kaçış kilidi CH6 kapanınca açılır
                floor.reset()            # taban kilidi de CH6 kapat-ac ile acilir
            # ÜÇLÜ KAPI (controller_orin ile AYNI)
            want_auto = (state.mode == 'autonomous') and ch6_high and bbox['valid']

            # ---------- GİRİŞ KENARI ----------
            # pilot_escape kilidi: pilot CH5 ile kaçtıysa CH6 hâlâ HIGH diye
            # GUIDED_NOGPS'i geri komutlama — pilotla mod savaşı olur.
            if want_auto and not engaged and not pending and not pilot_escape:
                if dry_run:
                    # DRY-RUN: mod komutu gonderilmiyor, dolayisiyla FC asla
                    # GUIDED_NOGPS'e gecmez ve teyit gelmez. Teyidi sart kossak
                    # kapali alan testinde kapilari/aksiyonlari hic goremezdik.
                    # Burada SIMULE ediyoruz — komut yine GONDERILMIYOR.
                    print("[GUIDED] (dry-run) otonom SIMULE ediliyor "
                          "— FC modu degismez, komut gonderilmez")
                    engaged = True
                else:
                    if acro_path:
                        print("[ACRO] Otonom istendi → ACRO komutlanıyor")
                        sender.set_mode_fast(MODE_ACRO, n=5)
                    else:
                        print("[GUIDED] Otonom istendi → GUIDED_NOGPS komutlanıyor")
                        sender.set_mode_fast(MODE_GUIDED_NOGPS, n=5)
                    pending = True
                    t_pending = now

            if pending:
                _mname = 'ACRO' if acro_path else 'GUIDED_NOGPS'
                if fc.in_auto():
                    print(f"[{_mname}] {_mname} TEYİTLENDİ ✓ komut akışı başlıyor")
                    engaged = True
                    pending = False
                elif now - t_pending > GUIDED_ENGAGE_TIMEOUT:
                    print(f"[{_mname}] !! {_mname} TUTMADI (arm değil / FC reddetti) "
                          "— otonom İPTAL")
                    pending = False
                    engaged = False
                    # Override baslamadi ama emin ol.
                    if acro_path:
                        sender.release_burst(3)

            # ---------- 4. KAPI: FC gerçekten guided'da mı ----------
            # Pilot CH5 ile kaçtıysa burada ANINDA yakalanır.
            if engaged and not dry_run and not fc.in_auto():
                # ACRO yolunda BURASI EN KRITIK NOKTA. Pilot CH5 ile kacti,
                # FC artik baska modda — ama RC_OVERRIDE hala CH1-4'u suruyor
                # ve yeni mod onu PILOT CUBUGU sanar. Once birak, sonra konus.
                if acro_path:
                    sender.release_burst(5)
                print(f"[SAFE] {time.strftime('%H:%M:%S')} FC otonom moddan ÇIKTI "
                      f"(mod={fc.mode_name()}) — muhtemelen pilot CH5'i oynattı. "
                      + ("Override BIRAKILDI. " if acro_path else "")
                      + "Komut akışı DURDURULDU, tekrar giriş için CH6'yı kapatıp aç.")
                engaged = False
                pilot_escape = True

            # ---------- KOMUT / KESME ----------
            if want_auto and engaged:
                obs = np.zeros(14, dtype=np.float32)
                obs[0] = bbox['cx'];    obs[1] = bbox['cy']
                obs[2] = bbox['w'];     obs[3] = bbox['h']
                obs[4] = bbox['ang_x']; obs[5] = bbox['ang_y']
                obs[6] = fc.roll_deg;   obs[7] = fc.pitch_deg
                obs[8] = fc.yaw_deg
                obs[9] = math.degrees(fc.roll_rate)
                obs[10] = math.degrees(fc.pitch_rate)
                obs[11] = math.degrees(fc.yaw_rate)
                obs[12] = fc.alt_rel
                obs[13] = 1.0

                if use_privileged:
                    # Referans eslemesi (gazebo_drone_env -> hc_air._parse_privileged):
                    #   valid    <- drone_state_valid
                    #   altitude <- latest_altitude
                    #   vel_ned  <- drone_vel_ned  (UC bilesen, sifirlanmis DEGIL)
                    privileged = {
                        'valid': fc.vel_valid if fc else False,
                        'altitude': fc.alt_rel if fc else 0.0,
                        'vel_ned': fc.vel_ned if fc else (0.0, 0.0, 0.0),
                    }
                    action = teacher.compute_action(obs, privileged=privileged)
                else:
                    # PN / mpc_teacher1: privileged'SIZ = GPS-free.
                    # (Karar yukarida, use_privileged aciklamasinda.)
                    action = teacher.compute_action(obs)
                last_action = list(action)

                # ---------- V-TRIM VOLTAJ BESLEMESI ----------
                # FCConnection SYS_STATUS'tan batt_v'yi zaten okuyor; sender'a
                # beslenmiyordu, yani --vtrim verilse bile filtre hic dolmazdi.
                #
                # vtrim=0 (VARSAYILAN) iken bu cagri zaten zararsizdi — olcup
                # dogruladim, _action_thrust_to_motor cikislari bit-bit ayni.
                # Yine de vtrim kapaliyken HIC CALISMASIN: sicak yolda yeni
                # kod calismazsa "eski davranisla ayni mi" sorusu hic sorulmaz.
                # --vtrim vermedigin surece bu satirin altina inilmez.
                if sender.vtrim > 0 and fc.batt_v is not None:
                    sender.update_voltage(fc.batt_v, dt)

                # ---------- TABAN SIGORTASI ----------
                # Komut GONDERILMEDEN once degerlendir: tetiklerse o kare
                # hic gonderilmez. Yas = telemetrinin bayatligi; bayatsa
                # FloorMonitor karar VERMEZ (yanlis alarm korumasi).
                _yas = (now - fc.last_attitude_time) if fc.last_attitude_time else 99.0
                if floor.update(fc.alt_rel, _yas, now):
                    print(f"[FLOOR] !!! {time.strftime('%H:%M:%S')} TABAN "
                          f"SIGORTASI: irtifa {fc.alt_rel:.1f} m < "
                          f"{args.floor_alt:.0f} m ({FloorMonitor.DWELL_S:.2f} s) "
                          f"— OTONOM KESILDI")
                    engaged = False
                    if acro_path:
                        sender.release_burst(5)
                    watchdog.trigger()
                    teacher.reset()
                    pilot_escape = True     # CH6 kapat-ac gerekir
                    continue

                sender.last_yaw_meas = fc.yaw_deg    # attitude modu yaw ofset siniri icin
                sender.send(action, fc.roll_deg, fc.pitch_deg)
                cmd = (sender.last_attitude if args.action_units == 'attitude_deg'
                       else sender.last_rates)
                tag = ('guided_att' if args.action_units == 'attitude_deg'
                       else ('acro' if acro_path else 'guided'))
                logger.log(tag if not dry_run else tag + '_dry', obs, action,
                           tuple(cmd) + (sender.last_thrust,))
            else:
                if engaged:
                    reason = ("CH6 LOW" if not ch6_high else
                              ("hedef kayip" if not bbox['valid'] else "state"))
                    print(f"[SAFE] {time.strftime('%H:%M:%S')} OTONOM KESILDI ({reason}) "
                          f"→ {args.exit_mode.upper()}")
                    engaged = False
                    # ACRO yolu: MOD DEGISIMINDEN ONCE override'i BIRAK.
                    # 65535 degil 0 gonderilir (adapter.release). Birakilmazsa
                    # FC 3 sn eski PWM'i tutar -> pilot cubuklari OLU kalir.
                    # controller_orin'de tam bu hata yasanmisti.
                    if acro_path:
                        sender.release_burst(5)
                    watchdog.trigger()
                    teacher.reset()

            # ---------- DURUM ----------
            if now - last_status >= 1.0:
                last_status = now
                a = last_action
                r, pt, y = sender.last_rates
                mode_s = fc.mode_name() if fc.current_mode is not None else "?"
                print(f"[{time.strftime('%H:%M:%S')}] {state.mode:10s} FC={mode_s:12s} "
                      f"CH6={channels.get('ch6',0)} bbox={'OK' if bbox['valid'] else '--'}")
                if engaged:
                    if args.action_units == 'attitude_deg':
                        ar, ap, ay = sender.last_attitude
                        print(f"             └─ ATT roll={ar:+6.1f}° pitch={ap:+6.1f}° "
                              f"yaw={ay:+6.1f}° thrust={sender.last_thrust:.3f} "
                              f"| ang_x={bbox.get('ang_x',0):+.1f} "
                              f"ang_y={bbox.get('ang_y',0):+.1f}")
                    elif acro_path:
                        pw = sender.last_pwm
                        d = sender.debug()
                        print(f"             └─ rad/s [{r:+.3f} {pt:+.3f} {y:+.3f}] "
                              f"→ PWM R{pw[0]} P{pw[1]} T{pw[2]} Y{pw[3]} "
                              f"thrust={sender.last_thrust:.3f} "
                              f"boost={sender.last_tilt_boost:.3f} "
                              f"| ang_x={bbox.get('ang_x',0):+.1f} "
                              f"ang_y={bbox.get('ang_y',0):+.1f}")
                        if d.get('acro_map_err', 0):
                            print(f"             └─ !! ACRO map reddi: "
                                  f"err={d['acro_map_err']:.0f} "
                                  f"hold={d.get('acro_map_hold',0):.0f} "
                                  f"neutral={d.get('acro_map_neutral',0):.0f}")
                    else:
                        print(f"             └─ ACTION r={a[0]:+.3f} p={a[1]:+.3f} "
                              f"t={a[2]:+.3f} y={a[3]:+.3f} → rad/s "
                              f"[{r:+.3f} {pt:+.3f} {y:+.3f}] "
                              f"thrust={sender.last_thrust:.3f} "
                              f"| ang_x={bbox.get('ang_x',0):+.1f} "
                              f"ang_y={bbox.get('ang_y',0):+.1f}")

            time.sleep(max(0.0, dt - (time.time() - now)))

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl-C")
    finally:
        if acro_path and sender is not None:
            # Her kapanista — istisna/Ctrl-C dahil. Idempotent.
            sender.release_burst(5)
        if engaged or pending:
            watchdog.trigger()
            t0 = time.time()
            while watchdog.armed and time.time() - t0 < 3.0:
                fc.poll()
                watchdog.update()
                time.sleep(0.05)
        logger.close()
        cam.close()
        print("Kapatıldı.")


if __name__ == "__main__":
    main()
