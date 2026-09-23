#!/usr/bin/env python3
"""
test_governor_offline.py — SAHA GOVERNOR + ACRO BAND regresyon testi.

FC / kamera / router GEREKMEZ — saf matematik. Orin'de uçuş öncesi çalıştır:

    cd ~/Desktop/hunter-orin/kcf && python3 test_governor_offline.py

Tüm satırlar OK basmalı. TEK FAIL varsa UÇMA — önce düzelt.

Neyi doğrular:
  1) Governor bayrakları VERİLMEDİĞİNDE davranış eski kodla BİREBİR
     (geriye uyumluluk — mevcut Gazebo doğrulaması geçerli kalır)
  2) scale=0 → eksen tamamen kapalı / thrust hover'a kilitli (G1 testi)
  3) --max-rate sert tavanı hiçbir eksende aşılmıyor
  4) --thrust-dev bandı hover'ı içeriyor ve iki yönde de kelepçeliyor
  5) --thrust-cap (bench) her şeyi ezmeye devam ediyor
  6) rate_scale × axis_scale çarpımı doğru
  7) send_neutral() governor'dan etkilenmiyor (0 rate + hover her zaman)
  8) ACRO axis_to_pwm band kelepçesi: default 500 = eski davranış,
     dar band küçük komutlara dokunmuyor, büyükleri kesiyor
  9) V-TRIM: kapalıyken bit-bit eski davranış; açıkken H_eff formülü,
     delta klempi iki yönde, 5s EMA gaz-sag'ı yutuyor
 10) V-TRIM güvenliği: --vtrim var --vref yok → setup() REDDEDER
 11) TABAN SİGORTASI (FloorMonitor): kurulum marjı, 0.25s tutma,
     bayat veri reddi, tek atım + kilit, reset
"""
import math
import sys

sys.path.insert(0, '.')

from guided_sender import GuidedSender
import controller_orin as co

HOVER = 0.30
fails = []


def check(name, cond, detail=""):
    tag = "OK " if cond else "FAIL"
    print(f"  [{tag}] {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        fails.append(name)


def mk(**kw):
    """conn=None + enabled=False + --hover ile FC'siz sender kur."""
    s = GuidedSender(None, 1, 1, action_units='rad_s',
                     hover_override=HOVER, enabled=False, **kw)
    if not s.setup():
        print("  [FAIL] setup() False dondu — test kurulamadi")
        sys.exit(1)
    return s


print("== 1. Governor KAPALI -> eski davranisla birebir ==")
s = mk()
r = s.send([0.3, 0.5, 0.07, 1.0], 0.0, 0.0)
check("pitch action_limit kirpmasi (0.5 -> 0.377)",
      abs(r[1] - 0.3769911184307752) < 1e-9, f"{r[1]:.4f}")
check("roll degismedi", abs(r[0] - 0.3) < 1e-9)
check("yaw degismedi", abs(r[2] - 1.0) < 1e-9)
check("thrust = hover (action 0.07)", abs(r[3] - HOVER) < 1e-9, f"{r[3]:.3f}")

print("== 2. SIFIR YETKI (G1): tum scale 0 ==")
s = mk(axis_scale={'roll': 0, 'pitch': 0, 'yaw': 0, 'thrust': 0})
r = s.send([0.3, 0.5, 0.5, 1.0], 10.0, -20.0)   # tilt'li durumda bile
check("rate'ler 0", all(abs(x) < 1e-12 for x in r[:3]))
check("thrust hover'a KILITLI", abs(r[3] - HOVER) < 1e-9, f"{r[3]:.3f}")

print("== 3. --max-rate 20 deg/s sert tavan ==")
s = mk(max_rate_dps=20.0)
r = s.send([1.884, 0.3, 0.07, 1.06], 0.0, 0.0)
lim = math.radians(20.0)
check("hicbir eksen 20 d/s'i gecmiyor",
      all(abs(x) <= lim + 1e-9 for x in r[:3]),
      f"{[round(math.degrees(x), 1) for x in r[:3]]} d/s")
check("governor_hits sayiyor", s.governor_hits >= 1, f"{s.governor_hits}")

print("== 4. --thrust-dev 0.05 hover bandi ==")
s = mk(thrust_dev=0.05)
r = s.send([0, 0, 0.5, 0], 0.0, 0.0)
check("ust kelepce hover+0.05", abs(r[3] - (HOVER + 0.05)) < 1e-9, f"{r[3]:.3f}")
r = s.send([0, 0, -0.2, 0], 0.0, 0.0)
check("alt kelepce hover-0.05", abs(r[3] - (HOVER - 0.05)) < 1e-9, f"{r[3]:.3f}")

print("== 5. --thrust-cap (bench) hala her seyi ezer ==")
s = mk(thrust_dev=0.05, thrust_cap=0.20)
r = s.send([0, 0, 0.5, 0], 0.0, 0.0)
check("cap 0.20 kazandi", abs(r[3] - 0.20) < 1e-9, f"{r[3]:.3f}")

print("== 6. rate_scale x axis_scale carpimi ==")
s = mk(rate_scale=0.5, axis_scale={'yaw': 0.5})
r = s.send([0, 0, 0.07, 1.0], 0.0, 0.0)
check("yaw 1.0 rad/s -> 0.25", abs(r[2] - 0.25) < 1e-9, f"{r[2]:.3f}")

print("== 7. send_neutral governor'dan ETKILENMEZ ==")
s = mk(axis_scale={'roll': 0, 'pitch': 0, 'yaw': 0, 'thrust': 0},
       max_rate_dps=15.0, thrust_dev=0.03)
r = s.send_neutral()
check("neutral = (0,0,0,hover)",
      all(abs(x) < 1e-12 for x in r[:3]) and abs(r[3] - HOVER) < 1e-9,
      f"thrust={r[3]:.3f}")

print("== 8. ACRO axis_to_pwm band kelepcesi ==")
co.AXIS_GAIN.update({'roll': 1.0, 'pitch': 1.0, 'throttle': 1.0, 'yaw': 1.0})
co.AXIS_BAND['pitch'] = 500
base = co.axis_to_pwm('pitch', 1.0)          # dir=-1 -> -1.0 -> 1000
check("band=500 (default) = eski davranis", base == 1000, f"{base}")
co.AXIS_BAND['pitch'] = 120
check("band=120 alt kelepce [1380]", co.axis_to_pwm('pitch', 1.0) == 1380,
      f"{co.axis_to_pwm('pitch', 1.0)}")
check("band=120 ust kelepce [1620]", co.axis_to_pwm('pitch', -1.0) == 1620,
      f"{co.axis_to_pwm('pitch', -1.0)}")
check("kucuk komuta DOKUNMUYOR (0.1 -> 1450)",
      co.axis_to_pwm('pitch', 0.1) == 1450, f"{co.axis_to_pwm('pitch', 0.1)}")
co.AXIS_BAND['pitch'] = 500

print("== 9. V-TRIM ==")
# 9a. KAPALI (varsayilan) -> voltaj beslense bile bit-bit eski davranis
s = mk()
s.update_voltage(20.0, 100.0)          # cok dusuk V bile etkisiz olmali
r = s.send([0, 0, 0.07, 0], 0.0, 0.0)
check("vtrim kapali: dusuk V'de bile thrust = hover", abs(r[3] - HOVER) < 1e-12,
      f"{r[3]:.4f}")
check("vtrim kapali: h_eff = H0", abs(s.last_heff - HOVER) < 1e-12)
# 9b. ACIK ama voltaj YOK -> trim pasif (H_eff = H0)
s = mk(vtrim=0.005, vref=33.0)
r = s.send([0, 0, 0.07, 0], 0.0, 0.0)
check("V gelmeden trim pasif", abs(r[3] - HOVER) < 1e-12, f"{r[3]:.4f}")
# 9c. formul: V=29, vref=33 -> delta = 0.005*4 = 0.020 (ust klemp SINIRINDA)
s.update_voltage(29.0, 100.0)          # dt>>tau -> filtre direkt oturur
r = s.send([0, 0, 0.07, 0], 0.0, 0.0)
check("H_eff = H0 + vtrim*(vref-V)", abs(r[3] - (HOVER + 0.020)) < 1e-9,
      f"{r[3]:.4f}")
# 9d. asiri dusuk V -> delta ust klempte KALIR (0.020), sayac artar
s.update_voltage(20.0, 100.0)
r = s.send([0, 0, 0.07, 0], 0.0, 0.0)
check("ust klemp +0.020 asilmiyor", abs(r[3] - (HOVER + 0.020)) < 1e-9,
      f"{r[3]:.4f}")
check("klemp sayaci artti", s.vtrim_clamp_hits >= 1, f"{s.vtrim_clamp_hits}")
# 9e. taze batarya (V > vref) -> alt klemp -0.005
s2 = mk(vtrim=0.005, vref=30.0)
s2.update_voltage(33.0, 100.0)         # delta = 0.005*(-3) = -0.015 -> klemp
r = s2.send([0, 0, 0.07, 0], 0.0, 0.0)
check("alt klemp -0.005", abs(r[3] - (HOVER - 0.005)) < 1e-9, f"{r[3]:.4f}")
# 9f. EMA gaz-sag reddi: 33 -> tek 24V ornegi (dt=1) filtreyi 31.2'ye ceker
s3 = mk(vtrim=0.005, vref=33.0)
s3.update_voltage(33.0, 100.0)
s3.update_voltage(24.0, 1.0)           # alpha = 1/5 = 0.2
check("EMA: 33V + tek 24V ornegi -> 31.2 (sag'i yutuyor)",
      abs(s3.v_filt - 31.2) < 1e-9, f"{s3.v_filt:.2f}")
r = s3.send([0, 0, 0.07, 0], 0.0, 0.0)
check("sag aninda hover sicramiyor (+0.009)",
      abs(r[3] - (HOVER + 0.005 * 1.8)) < 1e-9, f"{r[3]:.4f}")
# 9g. thrust_dev bandi H_eff ile birlikte kayar (trim bandin disina itilmez)
s4 = mk(vtrim=0.005, vref=33.0, thrust_dev=0.05)
s4.update_voltage(29.0, 100.0)         # H_eff = HOVER + 0.020
r = s4.send([0, 0, 0.5, 0], 0.0, 0.0)
check("dev bandi merkez H_eff (ust = H_eff+0.05)",
      abs(r[3] - (HOVER + 0.020 + 0.05)) < 1e-9, f"{r[3]:.4f}")
# 9h. sacma voltaj ornegi (glitch) filtreye GIRMEZ
s5 = mk(vtrim=0.005, vref=33.0)
s5.update_voltage(33.0, 100.0)
s5.update_voltage(0.4, 100.0)          # 65535mV kacagi vb.
check("glitch voltaj (0.4V) yok sayildi", abs(s5.v_filt - 33.0) < 1e-9,
      f"{s5.v_filt:.2f}")

print("== 10. V-TRIM guvenligi: vref'siz vtrim REDDEDILIR ==")
s6 = GuidedSender(None, 1, 1, action_units='rad_s',
                  hover_override=HOVER, enabled=False, vtrim=0.005)
check("setup() False dondu", s6.setup() is False)

print("== 11. TABAN SIGORTASI (FloorMonitor) ==")
from floor_monitor import FloorMonitor   # 31 Agu: ayri dosyaya tasindi
fm = FloorMonitor(20.0)                # arm 25 m, tutma 0.25 s, tazelik 0.5 s
check("floor 20 -> enabled", fm.enabled)
check("floor 0 -> disabled", not FloorMonitor(0.0).enabled)
# 11a. yerden engage: alt 1 m -> KURULMAZ, TETIKLEMEZ (bench senaryosu)
trip = any(fm.update(1.0, 0.0, t * 0.033) for t in range(60))
check("alcakta kurulmadan tetik YOK", not trip and not fm.armed)
# 11b. 30 m'ye cikinca kurulur
fm.update(30.0, 0.0, 10.0)
check("taban+5 ustunde KURULDU", fm.armed)
# 11c. kisa dip (< 0.25 s) tetiklemez
t0 = 20.0
tripped = fm.update(18.0, 0.0, t0) or fm.update(18.0, 0.0, t0 + 0.1)
tripped = tripped or fm.update(22.0, 0.0, t0 + 0.2)   # geri cikti
check("0.2 s'lik dip tetiklemedi", not tripped and not fm.latched)
# 11d. bayat veri (age > 0.5) tabanin altinda bile karar YOK
tripped = any(fm.update(5.0, 1.0, 30.0 + i * 0.1) for i in range(20))
check("bayat veriyle tetik YOK", not tripped and not fm.latched)
# 11e. surekli taban alti -> TAM BIR kez True, sonra kilit
trips = [fm.update(18.0, 0.0, 40.0 + i * 0.05) for i in range(30)]
check("0.25 s sonra tetikledi", any(trips))
check("TEK atim (sonra kilit)", sum(trips) == 1 and fm.latched)
check("kilitliyken tekrar tetik YOK",
      not fm.update(5.0, 0.0, 60.0) and fm.latched)
# 11f. reset (CH6 kapat-ac) -> kilit acilir, kurulum sifirlanir
fm.reset()
check("reset: kilit acildi + kurulum sifir", not fm.latched and not fm.armed)
# 11g. W0 gercek profili: 188 -> 108 m dalis TABANA DEGMEZ (yanlis alarm yok)
fm2 = FloorMonitor(20.0)
w0_alts = [188 - i * (80.0 / 99) for i in range(100)]     # 49 s'lik dalisin ozeti
trip = any(fm2.update(a, 0.0, i * 0.5) for i, a in enumerate(w0_alts))
check("20 Agu W0 profili (min 108 m) yanlis alarm URETMEZ",
      fm2.armed and not trip)

print()
if fails:
    print(f"!!! {len(fails)} FAIL: {fails}")
    print("!!! UCMA — once duzelt.")
    sys.exit(1)
print("HEPSI OK — governor/band/v-trim/taban matematigi dogru, ucusa hazir.")
