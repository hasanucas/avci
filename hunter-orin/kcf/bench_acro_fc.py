#!/usr/bin/env python3
"""
HUNTER — ACRO tasima katmani MASA TESTI.  (31 Agu 2026)

SADECE ROUTER GEREKIR. Kamera yok, tracker yok, teacher yok, kontrol
dongusu yok. Bu betik yalnizca su soruyu cevapliyor:

    Orin -> MAVProxy -> FC yolunda ACRO param okumasi ve RC override
    calisiyor mu?

NEDEN AYRI
----------
Tam donguyu tek seferde acarsan bir sey tutmadiginda sebep 5 yerden biri
olabilir: kamera, tracker, teacher, tasima, FC. Bu betik tasimayi TEK
BASINA test ediyor; burasi yesil olursa geri kalan hata yuzeyi kuculuyor.

PERVANELER CIKIK OLMALI. Betik motor calistirmiyor, arm etmiyor, mod
degistirmiyor (--set-acro vermezsen), ama --send ile RC override
gonderiyor. Override + arm = motor doner.

KULLANIM (sirayla)
------------------
  1) python3 bench_acro_fc.py
       Sadece OKUR. Hicbir sey gondermez. FC param okumasi calisiyor mu.

  2) python3 bench_acro_fc.py --send 10
       10 saniye NOTR override gonderir. DIKKAT: notr komutta CH1/CH2/CH4
       = 1495 = cubuk merkezi, yani o uc kanal AYIRT EDICI OLMAZ. Tek
       gercek kanit CH3 (hover 1465 != merkez 1495). Betik bunu artik
       kendisi soyluyor.

  3) python3 bench_acro_fc.py --send 20 --sweep
       ASIL TEST. Roll +-15 deg/s, yaw +-10 deg/s taranir; CH1 ve CH4
       merkezden ayrilir, boylece o zincirler de KANITLANIR. CH2 sabit
       kalir (kanallarin karismadiginin kontrolu).

  4) python3 bench_acro_fc.py --send 10 --release-test
       Ortada birakip yeniden alir — CH1-4 pilota donuyor mu.
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

try:
    from pymavlink import mavutil
except ImportError:
    print("HATA: pymavlink yok. pip3 install --user pymavlink")
    sys.exit(1)

from acro_sender import AcroSender
from guided_sender import HOVER_ACTION
# 31 Agu 2026: baglanti icin KENDI yolumu yazmistim, TUTMADI.
# controller_orin.FCConnection SAHADA CALISAN desen — aynen kullan.
# Neden kendi yolum tutmadi, asagida "1. BAGLANTI" bolumunde yazili.
from controller_orin import FCConnection

# mav.parm'dan BEKLENEN degerler. FC'den okunan bunlarla karsilastirilir.
# FC'de bir sey degistirirsen (ornegin ACRO_RP_RATE) BURAYI DA guncelle —
# burasi "FC'den ne bekliyorum"un yazili kaydi.
BEKLENEN = {
    'ACRO_RP_RATE': 360.0,
    'ACRO_RP_EXPO': 0.0,
    'ACRO_Y_RATE': 202.5,
    'ACRO_Y_EXPO': 0.0,
    'RCMAP_ROLL': 1, 'RCMAP_PITCH': 2, 'RCMAP_THROTTLE': 3, 'RCMAP_YAW': 4,
    'RC_MIN': 1045, 'RC_TRIM': 1495, 'RC_MAX': 1945, 'RC_DZ': 0,
    'thr_mid_curve': 0.125,
}
HOVER_PWM_BEKLENEN = 1465          # thrust 0.101 -> CH3


def ayir(baslik):
    print()
    print("=" * 64)
    print(baslik)
    print("=" * 64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fc', default='udpout:127.0.0.1:14551',
                    help="MAVProxy cikisi (run_router_mavproxy.sh 14551=brain)")
    ap.add_argument('--hover', type=float, default=0.101)
    ap.add_argument('--send', type=float, default=0.0,
                    help="Kac saniye override gonderilecek (0 = sadece oku)")
    ap.add_argument('--sweep', action='store_true',
                    help="Roll'u +-15 deg/s tara (CH1 hareket etmeli)")
    ap.add_argument('--release-test', action='store_true',
                    help="Ortada birak-yeniden al")
    ap.add_argument('--set-acro', action='store_true',
                    help="ACRO moduna gec (VARSAYILAN: mod degistirme)")
    ap.add_argument('--acro-max-rate', type=float, default=20.0)
    args = ap.parse_args()

    ayir("HUNTER — ACRO TASIMA MASA TESTI")
    print("  PERVANELER CIKIK MI?  Bu betik arm etmiyor ama override")
    print("  gonderiyor. Override + arm = motor doner.")
    print()
    print(f"  FC       : {args.fc}")
    print(f"  hover    : {args.hover}")
    print(f"  gonderim : {'YOK (sadece oku)' if args.send <= 0 else f'{args.send:.0f} s'}")

    # --- Baglan ---
    #
    # NEDEN controller_orin.FCConnection KULLANILIYOR
    # ----------------------------------------------
    # Ilk yazdigimda `mavutil.mavlink_connection(...)` + `wait_heartbeat()`
    # kullandim ve MAVProxy acikken bile "HEARTBEAT YOK" verdi. Iki sebep,
    # ikisi de FCConnection.connect() icinde COZULMUS durumda:
    #
    # 1. ILK PAKETI BIZ ATMALIYIZ.
    #    run_router_mavproxy.sh cikislari `--out=udpin:0.0.0.0:14551`.
    #    `udpin:` = MAVProxy SUNUCU. Bir istemci kendisine paket
    #    gonderene kadar o istemcinin adresini BILMIYOR, dolayisiyla
    #    hicbir sey gondermiyor. `wait_heartbeat()` sadece OKUR, yazmaz.
    #    -> sonsuza kadar bekler. FCConnection once heartbeat_send()
    #    atiyor, adres boylece ogreniliyor.
    #
    # 2. LINKTE BIRDEN COK HEARTBEAT VAR.
    #    MAVProxy kendisi --source-system=250 ile, runtracker de ayri
    #    sysid ile heartbeat atiyor. `wait_heartbeat()` ilk gelene
    #    kilitlenir ve GCS'e "autopilot" muamelesi yapabilir. FCConnection
    #    `hb.autopilot != MAV_AUTOPILOT_INVALID` suzgeciyle GERCEK
    #    autopilot'u ayikliyor ve target_sys/comp'u ondan aliyor.
    #
    # Ayrica _request_streams() ile ATTITUDE/RC_CHANNELS akislarini
    # istiyor — `--streamrate=-1` yuzunden bunu istemcinin yapmasi sart.
    ayir("1. BAGLANTI")
    fc = FCConnection(args.fc)
    if not fc.connect():
        print()
        print("  !! BAGLANTI YOK. Sirayla bak:")
        print("     - scripts/run_router_mavproxy.sh acik mi (Terminal 1)")
        print("     - ls /dev/ttyACM*   FC USB'de mi")
        print("     - pgrep -a mavlink-routerd   acik kalmis olabilir, pkill")
        print("     - 14551'i baska bir surec tutuyor olabilir:")
        print("         sudo lsof -i :14551")
        return 1
    conn = fc.conn
    print(f"  ok  autopilot sys={fc.target_sys} comp={fc.target_comp}")

    # --- Sender kur (gonderim KAPALI) ---
    ayir("2. PARAM OKUMASI  (asil soru burasi)")
    sender = AcroSender(
        conn, fc.target_sys, fc.target_comp,
        teacher_cfg=os.path.join(HERE, 'teacher_hc_air_3.yaml'),
        hover_override=args.hover,
        enabled=(args.send > 0),          # --send yoksa HICBIR SEY gonderilmez
        acro_max_rate_dps=args.acro_max_rate,
    )
    if not sender.setup():
        print("  !! setup() DUSTU — yukaridaki [ACRO] satirlarina bak.")
        return 1

    kaynak = sender.converter.config_source
    print()
    if kaynak == 'vehicle':
        print("  >>> FC'den OKUNDU. Asil test burasi ve GECTI.")
    else:
        print("  >>> !!! yaml FALLBACK. FC param okumasi CALISMADI.")
        print("      Ucmadan once bunu coz. Bakilacaklar:")
        print("        - MAVProxy --streamrate=-1 ile mi acik")
        print("        - mavlink-routerd acik kalmis olabilir (pkill)")
        print("        - baska bir istemci 14551'i tutuyor olabilir")

    # --- Okunani mav.parm beklentisiyle karsilastir ---
    ayir("3. OKUNAN DEGERLER vs BEKLENEN (mav.parm)")
    c = sender.converter.adapter.config
    satirlar = [
        ('ACRO_RP_RATE', c.rp_rate_deg_s, BEKLENEN['ACRO_RP_RATE']),
        ('ACRO_RP_EXPO', c.rp_expo, BEKLENEN['ACRO_RP_EXPO']),
        ('ACRO_Y_RATE', c.yaw_rate_deg_s, BEKLENEN['ACRO_Y_RATE']),
        ('ACRO_Y_EXPO', c.yaw_expo, BEKLENEN['ACRO_Y_EXPO']),
        ('thr_mid_curve', c.thr_mid_curve, BEKLENEN['thr_mid_curve']),
    ]
    fark = 0
    for ad, okunan, beklenen in satirlar:
        ok = abs(float(okunan) - float(beklenen)) < 1e-6
        fark += 0 if ok else 1
        print(f"  {ad:16s} okunan={float(okunan):9.4f}  "
              f"beklenen={float(beklenen):9.4f}  {'ok' if ok else '<<< FARKLI'}")

    for ad, cal in (('roll', c.roll_cal), ('pitch', c.pitch_cal),
                    ('throttle', c.throttle_cal), ('yaw', c.yaw_cal)):
        ok = (cal.min_pwm == BEKLENEN['RC_MIN']
              and cal.trim_pwm == BEKLENEN['RC_TRIM']
              and cal.max_pwm == BEKLENEN['RC_MAX']
              and cal.deadzone == BEKLENEN['RC_DZ'])
        fark += 0 if ok else 1
        print(f"  RC_{ad:12s} min={cal.min_pwm} trim={cal.trim_pwm} "
              f"max={cal.max_pwm} dz={cal.deadzone} rev={cal.reversed}  "
              f"{'ok' if ok else '<<< FARKLI'}")

    if fark:
        print()
        print(f"  !! {fark} deger beklenenden FARKLI.")
        print("     Ya FC'de bir sey degisti (o zaman BEKLENEN sozlugunu ve")
        print("     teacher_hc_air_3.yaml acro: blogunu guncelle), ya da")
        print("     yanlis arac okunuyor. Ucma, once coz.")

    # --- Hover PWM ---
    ayir("4. HOVER GAZ KARSILIGI")
    a = sender.converter.adapter
    hpwm = a.throttle_pwm_for_thrust(args.hover)
    print(f"  thrust {args.hover:.3f}  ->  CH3 = {hpwm}   "
          f"(trim 1495 = thrust {a.thrust_for_throttle_pwm(1495):.4f})")
    if args.hover == 0.101 and hpwm != HOVER_PWM_BEKLENEN:
        print(f"  !! BEKLENEN {HOVER_PWM_BEKLENEN} idi. Kalibrasyon farkli okunmus.")
    else:
        print("  ok  Asagida FC'nin bu degeri gordugu DOGRULANACAK.")
    print()
    print(f"  NOT: teacher aksiyon birimi != motor thrust. Notr gaz aksiyonu")
    print(f"       HOVER_ACTION={HOVER_ACTION} ve sender onu hover_motor'a")
    print(f"       ({args.hover}) civiliyor. Bu betik notr aksiyon gonderiyor.")
    print()
    print("  ALTHOLD olu bandi (THR_DZ=100): 1395 .. 1595")
    print(f"  {hpwm} bu bandin {'ICINDE (birakilmazsa alcalmaz)' if 1395 <= hpwm <= 1595 else 'DISINDA — birakilmazsa ALCALIR'}")

    if args.send <= 0:
        ayir("BITTI — hicbir sey gonderilmedi")
        print("  Sirada:  python3 bench_acro_fc.py --send 10")
        return 0 if (kaynak == 'vehicle' and fark == 0) else 1

    # --- Gonderim ---
    if args.set_acro:
        ayir("5. ACRO MODUNA GECIS")
        sender.enter_guided()      # isim GuidedSender API'si; ACRO'ya gecer
        time.sleep(0.5)

    ayir(f"{'6' if args.set_acro else '5'}. OVERRIDE GONDERIMI — {args.send:.0f} s")
    print("  Mission Planner GEREKMIYOR — betik kendi dogruluyor.")
    print()
    # -----------------------------------------------------------------
    # KENDINI DOGRULAMA
    #
    # Betik eskiden sadece paketi GONDERDIGINI biliyordu; FC'nin KABUL
    # ETTIGINI bilmiyordu. Ikisi ayri sey: override reddedilse bile
    # gonderim sayaci artardi ve ekranda her sey yolunda gorunurdu.
    # (31 Agu 2026 ilk masa kosusunda tam bu oldu: 285 kare gonderildi,
    #  "map reddi 0" yazdi, ama override'in FC'ye GECTIGINE dair tek bir
    #  kanit yoktu. Kullaniciya "Mission Planner'a bak" demek zorunda
    #  kaldim; oysa kanit zaten linkte akiyordu.)
    #
    # ArduPilot RC_CHANNELS telemetrisinde override UYGULANMIS degerleri
    # bildiriyor. FCConnection.poll() bunu fc.rc_channels'a yaziyor.
    # -----------------------------------------------------------------
    print("  Taban cizgisi aliniyor (override YOK, 1.5 s)...")
    taban = {}
    tb0 = time.time()
    while time.time() - tb0 < 1.5:
        fc.poll()
        if fc.last_rc_time > 0:
            taban = dict(fc.rc_channels)
        time.sleep(0.02)
    if not taban:
        print("  !! RC_CHANNELS gelmiyor — kumanda ACIK MI?")
        print("     Dogrulama yapilamayacak; gonderim yine de surecek.")
    else:
        print("  taban:  " + "  ".join(
            f"CH{i}={taban.get(f'ch{i}', 0)}" for i in (1, 2, 3, 4, 5, 6, 8)))
    print()

    DOKUNULMAZ = [5, 6, 8, 11, 12, 13]
    esles = {'ch1': 0, 'ch2': 0, 'ch3': 0, 'ch4': 0}
    orneklem = 0
    kirlenen = {}

    # -----------------------------------------------------------------
    # GECIKMEYE DUYARLI ESLESTIRME (31 Agu 2026)
    #
    # Ilk --sweep kosusunda CH1 %99.8 ama CH4 %90.4 cikti. Once "yaw
    # zinciri zayif" gibi gorundu; degildi. CH4'teki 18 sapmanin
    # HEPSI egimin TERSINE idi — yani FC raporu gonderdigimiz degeri
    # GERIDEN takip ediyordu. Telemetri gecikmesi, paket kaybi degil.
    #
    # Biz 30 Hz gonderiyoruz, RC_CHANNELS ~20 Hz (ve seri hat 115200
    # baud'da daha da dusebiliyor). Yani elimizdeki en taze FC raporu
    # 50-200 ms bayat. Hizli degisen bir sinyalde bu birkac PWM eder:
    #     roll  (periyot 6 s): tepe egim 19.6 PWM/s
    #     yaw   (periyot 4 s): tepe egim 34.9 PWM/s   <- 1.8 kat
    # CH4'un daha kotu gorunmesinin TEK sebebi buydu.
    #
    # Cozum toleransi gevsetmek DEGIL (o gercek hatalari da gizlerdi):
    # son GECMIS penceresindeki gonderimlerden HERHANGI BIRINE uyuyor
    # mu diye bakiyoruz. Gecikme kadar geriye bakiyor, hassasiyet
    # kaybetmiyoruz.
    # -----------------------------------------------------------------
    GECMIS = 12                      # ~400 ms @ 30 Hz
    gecmis = []                      # [(pwm4), ...] en yeni sonda
    rc_sayac, rc_t0, rc_son_t = 0, None, 0.0

    t0 = time.time()
    n = 0
    birakildi = False
    try:
        while time.time() - t0 < args.send:
            t = time.time() - t0

            if args.release_test and not birakildi and t > args.send / 2:
                print()
                print("  >>> BIRAKILIYOR — CH1-4 simdi PILOT cubuklarina donmeli")
                sender.release_burst(5)
                birakildi = True
                tr = time.time()
                while time.time() - tr < 2.0:
                    fc.poll()
                    time.sleep(0.02)
                geri = dict(fc.rc_channels)
                sp = sender.last_pwm
                dondu = any(abs(geri.get(f'ch{i+1}', 0) - sp[i]) > 3
                            for i in range(4))
                print("      FC simdi: " + "  ".join(
                    f"CH{i+1}={geri.get(f'ch{i+1}', 0)}" for i in range(4)))
                print("      " + ("ok  cubuklara DONDU" if dondu
                                  else "!! hala override degerinde"))
                print("  >>> yeniden aliniyor")
                t0 += 2.0
                continue

            # --sweep: roll VE yaw'i tara. Yaw da olmali, yoksa CH4
            # taban cizgisinde kalir ve "ayirt edici degil" cikar.
            # Pitch bilerek disarida: CH2'yi sabit birakip taramanin
            # kanallari karistirmadigini gorebilelim.
            roll_r = math.radians(15.0) * math.sin(2 * math.pi * t / 6.0) \
                if args.sweep else 0.0
            yaw_r = math.radians(10.0) * math.cos(2 * math.pi * t / 4.0) \
                if args.sweep else 0.0
            # DIKKAT: ucuncu eleman TEACHER AKSIYON birimi, motor thrust
            # DEGIL. HOVER_ACTION (0.07) = teacher'in notr gazi; sender bunu
            # hover_motor'a (--hover) civiliyor. Buraya 0.101 yazmak
            # "hover'in 0.031 ustunde" demek olurdu ve CH3 1465 yerine
            # 1477 cikardi. Ilk yazdigimda tam bu hatayi yaptim.
            sender.send([roll_r, 0.0, HOVER_ACTION, yaw_r], 0.0, 0.0)
            n += 1

            gecmis.append(tuple(sender.last_pwm))
            if len(gecmis) > GECMIS:
                gecmis.pop(0)

            fc.poll()
            # RC_CHANNELS gercek hizini olc — gecikme butcesi buradan cikar
            if fc.last_rc_time > rc_son_t:
                rc_son_t = fc.last_rc_time
                if rc_t0 is None:
                    rc_t0 = fc.last_rc_time
                else:
                    rc_sayac += 1

            if taban and fc.last_rc_time > 0 and t > 0.5:
                gercek = fc.rc_channels
                orneklem += 1
                for i, ad in enumerate(('ch1', 'ch2', 'ch3', 'ch4')):
                    g = gercek.get(ad, 0)
                    if any(abs(g - h[i]) <= 3 for h in gecmis):
                        esles[ad] += 1
                for k in DOKUNULMAZ:
                    ad = f'ch{k}'
                    if ad in taban and abs(gercek.get(ad, 0) - taban[ad]) > 20:
                        kirlenen.setdefault(ad, (taban[ad], gercek.get(ad, 0)))

            if n % 30 == 0:
                sp = sender.last_pwm
                d = sender.debug()
                g = fc.rc_channels
                print(f"  t={t:5.1f}s  GONDERDIK   CH1={sp[0]} CH2={sp[1]} "
                      f"CH3={sp[2]} CH4={sp[3]}")
                print(f"            FC GORUYOR  CH1={g.get('ch1',0)} "
                      f"CH2={g.get('ch2',0)} CH3={g.get('ch3',0)} "
                      f"CH4={g.get('ch4',0)}"
                      + (f"   !! map_err={d['acro_map_err']:.0f}"
                         if d.get('acro_map_err', 0) else ""))
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        print("\n  CTRL+C")
    finally:
        print()
        print("  >>> BIRAKILIYOR (release_burst x5, 0 gonderiliyor)")
        sender.release_burst(5)
        time.sleep(0.3)
        sender.release_burst(5)

    # --- Birakma FC tarafinda gerceklesti mi ---
    time.sleep(0.5)
    tb = time.time()
    while time.time() - tb < 1.5:
        fc.poll()
        time.sleep(0.02)
    son = dict(fc.rc_channels)

    ayir("SONUC")
    d = sender.debug()
    print(f"  gonderilen kare  : {n}")
    print(f"  map reddi        : {d.get('acro_map_err', 0):.0f}")
    print(f"  son gecerli tutma: {d.get('acro_map_hold', 0):.0f}")
    print(f"  notr'e dusme     : {d.get('acro_map_neutral', 0):.0f}")

    hata = 0
    if not taban:
        print()
        print("  ?? RC_CHANNELS gelmedi — DOGRULAMA YAPILAMADI.")
        print("     Kumandayi ac ve tekrar kos; yoksa override'in FC'ye")
        print("     GECTIGINI bilmiyoruz, sadece GONDERILDIGINI biliyoruz.")
        hata = 1
    else:
        if rc_t0 is not None and rc_sayac > 0:
            hz = rc_sayac / max(rc_son_t - rc_t0, 1e-6)
            print()
            print(f"  RC_CHANNELS telemetri hizi: {hz:.1f} Hz "
                  f"(gonderim 30 Hz) -> rapor ~{1000.0/max(hz,1e-6):.0f} ms bayat")
            if hz < 8:
                print("    ?? Dusuk. Seri hat 115200 baud'da dolmus olabilir.")
                print("       Ucusu engellemez ama bu betigin dogrulama")
                print("       cozunurlugunu dusurur.")

        print()
        print(f"  --- OVERRIDE FC'YE GECTI MI ({orneklem} orneklem, "
              f"gecikme penceresi {GECMIS} kare) ---")
        # AYIRT EDICILIK KONTROLU (31 Agu 2026 eklendi)
        #
        # Gonderdigimiz deger taban cizgisiyle AYNIYSA, "eslesme %100"
        # HICBIR SEY KANITLAMAZ: override uygulansa da uygulanmasa da FC
        # ayni sayiyi bildirir. Ilk basarili masa kosusunda tam bu oldu —
        # notr komutta CH1/CH2/CH4 = 1495 gonderiliyordu ve pilot cubuklari
        # da 1495'teydi. Uc kanal "%100 ok" yazdi ama sifir bilgi tasiyordu.
        # Tek gercek kanit CH3'tu (taban 1495, gonderilen 1465).
        #
        # Artik ayirt edici olmayan kanal ACIKCA isaretleniyor ve
        # hukumde sayilmiyor.
        ayirt_eden = 0
        for i, ad in enumerate(('ch1', 'ch2', 'ch3', 'ch4')):
            oran = 100.0 * esles[ad] / max(orneklem, 1)
            gonderilen = sender.last_pwm[i]
            tabanda = taban.get(ad, -999)
            ayirt = abs(gonderilen - tabanda) > 5
            if not ayirt:
                print(f"    CH{i+1}  eslesme %{oran:5.1f}   "
                      f"?? AYIRT EDICI DEGIL (gonderilen {gonderilen} == "
                      f"taban {tabanda})")
                continue
            ayirt_eden += 1
            ok = oran > 90.0
            hata += 0 if ok else 1
            print(f"    CH{i+1}  eslesme %{oran:5.1f}   "
                  + ("ok  (KANIT)" if ok else "<<< GECMEDI"))

        if ayirt_eden == 0:
            hata += 1
            print()
            print("    !! HICBIR KANAL AYIRT EDICI DEGIL — bu kosu override'in")
            print("       gectigini KANITLAMIYOR. --sweep ile tekrar kos.")
        elif ayirt_eden < 3:
            print()
            print(f"    NOT: sadece {ayirt_eden} kanal kanit tasiyor. Roll ve")
            print("         yaw zincirini de kanitlamak icin: --sweep")

        print()
        print("  --- DOKUNULMAZ KANALLAR ---")
        if kirlenen:
            hata += 1
            for ad, (t_, g_) in kirlenen.items():
                print(f"    !! {ad.upper()} DEGISTI: {t_} -> {g_}  "
                      "(bu kanala DOKUNULMAMALIYDI)")
        else:
            print("    ok  CH5/6/8/11/12/13 taban cizgisinde kaldi")

        print()
        print("  --- BIRAKMA ---")
        # "Gonderilenden farkli mi" YETMEZ: pilot cubugu tesadufen
        # gonderdigimiz degere yakin olabilir (notr komutta CH1/2/4 = trim,
        # cubuk da ortadaysa fark 0 cikar ve "birakilmadi" sanilir).
        # Dogru soru: TABAN CIZGISINE dondu mu.
        sp = sender.last_pwm
        print("    taban   : " + "  ".join(
            f"CH{i+1}={taban.get(f'ch{i+1}', 0)}" for i in range(4)))
        print("    FC simdi: " + "  ".join(
            f"CH{i+1}={son.get(f'ch{i+1}', 0)}" for i in range(4)))
        takili = []
        for i in range(4):
            ad = f'ch{i+1}'
            simdi = son.get(ad, 0)
            tabanda = abs(simdi - taban.get(ad, -999)) <= 5
            gonderilende = abs(simdi - sp[i]) <= 3
            if gonderilende and not tabanda:
                takili.append(i + 1)
        if takili:
            hata += 1
            print(f"    !! CH{takili} hala override degerinde, tabana DONMEDI.")
            print("       FC 3 sn sonra kendi birakir; cubuklari oynatip")
            print("       tekrar kos. Donmuyorsa release paketi gitmiyor.")
        else:
            print("    ok  CH1-4 taban cizgisine dondu")

    ayir("HUKUM")
    if kaynak != 'vehicle' or fark:
        print("  DUSTU — param okumasi ya da degerler sorunlu (bolum 2-3).")
        return 1
    if hata:
        print("  DUSTU — override FC'ye tam gecmedi ya da yanlis kanala")
        print("  dokundu. Yukaridaki '<<<' ve '!!' satirlarina bak.")
        return 1
    print("  GECTI — tasima katmani calisiyor:")
    print("    * FC param okumasi         ok")
    print("    * okunan degerler mav.parm ok")
    print("    * override FC'ye gecti     ok")
    print("    * dokunulmaz kanallar      ok")
    print("    * birakma                  ok")
    print()
    print("  Sirada: tam dongu (router + tracker + controller_guided).")
    return 0


if __name__ == '__main__':
    sys.exit(main())
