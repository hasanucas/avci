#!/usr/bin/env python3
"""
HUNTER — guided uçuş log analizcisi.

Kullanım:
    python3 analiz_guided.py guided_YYYYMMDD_HHMMSS.csv

Her eksenin isaret dogrulugunu, saturasyonu ve thrust dagilimini ozetler.
Pervaneli teste gecmeden ONCE her logu buradan gecir.
"""
import csv
import sys
import statistics as st
from collections import Counter


def col(rows, c):
    return [float(r[c]) for r in rows]


def sign_check(name, ang, cmd, expect_same):
    """ang ile cmd arasi isaret korelasyonu dogru mu."""
    pairs = [(a, c) for a, c in zip(ang, cmd) if abs(a) > 2.0]  # deadband disi
    if not pairs:
        return f"  {name}: yeterli veri yok (hedef hep merkezde)"
    same = sum(1 for a, c in pairs if (a > 0) == (c > 0))
    frac = same / len(pairs)
    if expect_same:
        ok = frac > 0.8
        yon = "ayni yon" if ok else "TERS YON!"
    else:
        ok = frac < 0.2
        yon = "ters yon (dogru)" if ok else "AYNI YON — HATA!"
    flag = "✅" if ok else "🔴"
    return f"  {flag} {name}: ang>0 iken cmd ayni-isaret {100*frac:.0f}% ({yon})"


def main():
    if len(sys.argv) < 2:
        print("kullanim: python3 analiz_guided.py <log.csv>")
        return
    rows = list(csv.DictReader(open(sys.argv[1])))
    g = [r for r in rows if r['mode'] in ('guided', 'guided_dry')]
    if not g:
        print("guided satiri yok (sadece manual?).")
        return

    t = col(g, 't')
    print(f"=== {sys.argv[1]} ===")
    print(f"guided sure: {t[-1]-t[0]:.1f} s, {len(g)} satir")
    dry = g[0]['mode'] == 'guided_dry'
    if dry:
        print("(DRY-RUN — komut gonderilmedi)")

    # TEACHER TESPITI — iki teacher kanallari TERS kullaniyor:
    #   mpc_teacher1 : pitch = DIKEY NISAN (-kp*ang_y), thrust = irtifa koruma
    #   hc_air       : pitch = HIZ regulatoru (governor), thrust = DIKEY NISAN
    # Bu yuzden pitch<->ang_y korelasyon testi SADECE mpc_teacher1 icin gecerli.
    # hc_air'de pitch'in ang_y ile dogrudan iliskisi YOKTUR; o testi uygulamak
    # yanlis alarm verir.
    th = col(g, 'cmd_thrust')
    thr_span = max(th) - min(th)
    # hc_air ipucu: thrust genis bantta oynar (dikey nisan orada) ve act_throttle
    # hover 0.07'den cok uzaktir. mpc_teacher1'de thrust neredeyse sabittir.
    at_mean = st.mean(col(g, 'act_throttle'))
    is_hcair = at_mean > 0.15

    print("\n--- ISARET DOGRULAMASI (gimbal'de donusle kapanan kanallar) ---")
    print(f"  teacher tahmini: {'hc_air (pitch=hiz, thrust=dikey nisan)' if is_hcair else 'mpc_teacher1 (pitch=dikey nisan)'}")
    print(sign_check("yaw  (ang_x)", col(g, 'obs_ang_x'), col(g, 'cmd_yaw_rad_s'), True))
    if is_hcair:
        print("  --  pitch(ang_y): hc_air'de pitch DIKEY NISAN YAPMAZ (hiz regulatoru)")
        print("      -> bu test uygulanmadi. Dikey nisan THRUST kanalinda ve")
        print("         gimbal'de otelemeyle kapandigi icin DOGRULANAMAZ.")
    else:
        print("Kamera 15° YUKARI: ang_y<0 (hedef yukarida) -> burun asagi pitch beklenir")
        print(sign_check("pitch(ang_y)", col(g, 'obs_ang_y'), col(g, 'cmd_pitch_rad_s'), False))

    # DONUS gerceklesti mi — gimbal'de kapali cevrimin KANITI
    print("\n--- DRONE GERCEKTEN DONDU MU (kapali cevrim kaniti) ---")
    for c, ad in [('obs_yaw_deg', 'yaw'), ('obs_pitch_deg', 'pitch'), ('obs_roll_deg', 'roll')]:
        v = col(g, c)
        span = max(v) - min(v)
        flag = "✅" if span > 3.0 else "⚠"
        print(f"  {flag} {ad:6s}: {min(v):+7.1f}° .. {max(v):+7.1f}°  (aralik {span:5.1f}°)")

    print("\n--- SATURASYON ---")
    # DOGRU esikler: teacher_hc_air.yaml'daki ETKIN limit = min(ic_limit, action_limit).
    # Ilk surumde action_limits kullanmistim; teacher'in IC limitleri bazi kanallarda
    # daha dar oldugu icin gercek doygunlugu KACIRIYORDU (yaw %29'u %0 gostermisti).
    lims = {
        'cmd_roll_rad_s':  min(1.0367255756846316, 1.884955592153876),
        'cmd_pitch_rad_s': min(0.40055306333269863, 0.3769911184307752),
        'cmd_yaw_rad_s':   min(0.9542587685278996, 1.0602875205865552),
    }
    for c, lim in lims.items():
        v = col(g, c)
        sat = sum(1 for x in v if abs(x) > 0.98 * lim)
        flag = "🔴" if sat > 0.2 * len(v) else ("⚠" if sat > 0.05 * len(v) else "✅")
        print(f"  {flag} {c:18s}: %{100*sat/len(v):.0f} tavanda "
              f"(ort={st.mean(v):+.3f} min={min(v):+.3f} max={max(v):+.3f})")

    print("\n--- THRUST ---")
    th = col(g, 'cmd_thrust')
    at = col(g, 'act_throttle')
    print(f"  cmd_thrust  : ort={st.mean(th):.3f} min={min(th):.3f} max={max(th):.3f}")
    print(f"  act_throttle: ort={st.mean(at):.3f} (teacher ham; hover ~0.07 olmali)")
    span = max(th) - min(th)
    hi = max(th)
    sat = sum(1 for x in th if x > hi - 0.005)
    if is_hcair and sat > 0.5 * len(th):
        print(f"  ℹ thrust %{100*sat/len(th):.0f} tavanda ({hi:.3f}) — GIMBAL'DE BEKLENEN.")
        print("    hc_air dikey nisani OTELEMEYLE kapanir (tirman -> hedef karede")
        print("    asagi kayar -> hata kapanir). Gimbal tirmanmaya izin vermedigi")
        print("    icin hata hic kapanmaz ve PI doyar. HATA DEGIL.")
        print("    -> Bu kanal ancak serbest ucusta/SITL'de dogrulanabilir.")
    elif span < 0.03:
        print(f"  ⚠ thrust neredeyse SABIT (aralik {span:.3f}) — dikey kanal etkisiz.")
        print("    Muhtemel sebep: --hover degeri gercek hover'dan uzak, ya da")
        print("    vz beslemesi yok (governor/PI devre disi).")

    print("\n--- IMU CANLI MI ---")
    for c in ['obs_pitch_deg', 'obs_pitch_rate', 'obs_yaw_rate']:
        v = col(g, c)
        s = st.pstdev(v)
        note = "sabit (gimbal'de kilitli)" if s < 0.5 else "degisiyor (serbest?)"
        print(f"  {c:16s}: std={s:.2f}  {note}")

    print("\n--- bbox ---")
    bv = col(g, 'obs_bbox_valid')
    print(f"  valid: %{100*sum(bv)/len(bv):.0f}")


if __name__ == "__main__":
    main()
