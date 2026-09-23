#!/usr/bin/env python3
"""
PARAM_VALUE teshis araci — HUNTER kodundan BAGIMSIZ.

Amac: "param okunamiyor" sorununun nerede oldugunu kesinlestirmek.
Hicbir HUNTER modulu import etmez; sadece pymavlink.

KULLANIM
  python3 param_test.py                      # brain endpoint (14551)
  python3 param_test.py --port 14552         # qgc endpoint
  python3 param_test.py --port 14550         # runtracker endpoint
  python3 param_test.py --tcp 5760           # TCP endpoint
  python3 param_test.py --param MOT_THST_HOVER

OKUMA
  * "PARAM_VALUE geldi" -> yol calisiyor, sorun HUNTER kodunda
  * Hicbir PARAM_VALUE yok ama HEARTBEAT/ATTITUDE var
      -> router bu endpointte msgid 22'yi engelliyor VEYA FC cevap vermiyor
  * Bir endpointte calisip digerinde calismiyorsa -> router filtresi kesin
"""

import argparse
import time
from collections import Counter

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=14551, help="UDP port (default 14551=brain)")
    ap.add_argument('--tcp', type=int, default=None, help="TCP port kullan (orn 5760)")
    ap.add_argument('--param', default='GUID_OPTIONS')
    ap.add_argument('--secs', type=float, default=12.0)
    ap.add_argument('--quiet', action='store_true',
                    help="Param istemeden ONCE tum stream'leri KIS. ArduPilot "
                         "link doluyken PARAM_VALUE'yu sessizce dusurur "
                         "(HAVE_PAYLOAD_SPACE); bu onu test eder.")
    ap.add_argument('--serial', default=None,
                    help="Router'i BYPASS et, dogrudan seri porta bagla "
                         "(orn /dev/ttyACM0). ONCE ROUTER'I DURDUR!")
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--sysid', type=int, default=255)
    ap.add_argument('--compid', type=int, default=190)
    args = ap.parse_args()

    if args.serial:
        ep = args.serial
    elif args.tcp:
        ep = f"tcp:127.0.0.1:{args.tcp}"
    else:
        ep = f"udpout:127.0.0.1:{args.port}"
    print(f"=== PARAM TESTI ===\nEndpoint : {ep}\nParam    : {args.param}\n"
          f"Kimlik   : sys={args.sysid} comp={args.compid}\n")

    if args.serial:
        conn = mavutil.mavlink_connection(ep, baud=args.baud,
                                          source_system=args.sysid,
                                          source_component=args.compid)
    else:
        conn = mavutil.mavlink_connection(ep, source_system=args.sysid,
                                          source_component=args.compid)
    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    print("Autopilot heartbeat bekleniyor...")
    tgt_sys = tgt_comp = None
    t0 = time.time()
    while time.time() - t0 < 10:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            tgt_sys, tgt_comp = hb.get_srcSystem(), hb.get_srcComponent()
            print(f"  OK: autopilot sys={tgt_sys} comp={tgt_comp}\n")
            break
    if tgt_sys is None:
        print("  HATA: autopilot heartbeat YOK. Router calisiyor mu, port dogru mu?")
        return

    # --- 0) UPLINK TESTI (v2 — birikimden ETKILENMEZ) ---
    # v1 HATALIYDI: iki olcum arasindaki sleep soket tamponunda mesaj biriktirip
    # ikinci olcumu sisiriyordu (2 Hz istendi, 15.5 Hz "olculdu").
    # v2: ATTITUDE'u TEK BASINA degil, VFR_HUD'a ORANLA olcuyoruz.
    #     Birikme ikisini de ayni etkiler -> oran bozulmaz.
    #     ATTITUDE = EXTRA1, VFR_HUD = EXTRA2. Sadece EXTRA1'i kisiyoruz.
    def drain():
        while conn.recv_match(blocking=False) is not None:
            pass

    def ratio(sec=5.0):
        drain()                       # ONCE tamponu bosalt — kritik
        c = Counter()
        t = time.time()
        while time.time() - t < sec:
            m = conn.recv_match(blocking=True, timeout=0.3)
            if m is not None:
                c[m.get_type()] += 1
        a, v = c.get('ATTITUDE', 0), c.get('VFR_HUD', 0)
        return a, v, (a / v if v else float('nan'))

    print("[0] UPLINK TESTI (mesajlarimiz FC'ye ulasiyor mu)")
    a0, v0, r0 = ratio()
    print(f"    once        : ATTITUDE={a0} VFR_HUD={v0}  oran={r0:.2f}")

    for _ in range(4):
        conn.mav.request_data_stream_send(
            tgt_sys, tgt_comp, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1)
        time.sleep(0.15)
    time.sleep(1.0)

    a1, v1, r1 = ratio()
    print(f"    EXTRA1=1Hz  : ATTITUDE={a1} VFR_HUD={v1}  oran={r1:.2f}")

    # KARAR MANTIGI (v3):
    # Oncelik oran yontemi (VFR_HUD referansi varsa) — global hiz degisimlerine
    # bagisik. Ama VFR_HUD gelmiyorsa (MAVProxy --streamrate=-1 ile stream'leri
    # ISTEMCILER belirliyor; VFR_HUD=EXTRA2'yi runtracker istiyor, o kapaliysa
    # gelmez) payda 0 olur ve oran nan cikar. O durumda MUTLAK ATTITUDE sayimina
    # duseriz — her olcum oncesi drain() yaptigimiz icin bu da guvenilir.
    if v0 > 0 and v1 > 0:
        uplink_ok = r1 < 0.5 * r0
        yontem = "oran (VFR_HUD referansli)"
    elif a0 > 0:
        uplink_ok = a1 < 0.5 * a0
        yontem = "mutlak ATTITUDE sayimi (VFR_HUD yok)"
    else:
        uplink_ok = False
        yontem = "veri yok"

    print(f"    yontem: {yontem}")
    if uplink_ok:
        print(f"    ✅ UPLINK CALISIYOR — ATTITUDE {a0}->{a1} dustu, istek FC'ye ULASTI")
    else:
        print("    ❌ UPLINK YOK — ATTITUDE hizi degismedi, istek FC'ye ULASMIYOR")
        print("       Mod komutu ve SET_ATTITUDE_TARGET de ulasmaz demektir.")

    for _ in range(4):                # eski hizi geri koy
        conn.mav.request_data_stream_send(
            tgt_sys, tgt_comp, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
        time.sleep(0.15)
    drain()
    print()

    # --- 1) Tek param oku ---
    if args.quiet:
        print("[0b] --quiet: tum stream'ler kisiliyor (link bosalsin)...")
        for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                    mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER):
            for _ in range(2):
                conn.mav.request_data_stream_send(tgt_sys, tgt_comp, sid, 0, 0)
                time.sleep(0.05)
        time.sleep(2.0)
        while conn.recv_match(blocking=False) is not None:
            pass
        print("     link bosaltildi, param isteniyor\n")

    print(f"[1] PARAM_REQUEST_READ '{args.param}' gonderiliyor (0.5s'de bir tekrar)...")
    seen = Counter()
    got = None
    t0 = time.time()
    last_ask = 0.0
    while time.time() - t0 < args.secs:
        if time.time() - last_ask > 0.5:
            conn.mav.param_request_read_send(tgt_sys, tgt_comp,
                                             args.param.encode('ascii'), -1)
            last_ask = time.time()
        m = conn.recv_match(blocking=True, timeout=0.3)   # FILTRE YOK — hepsini gor
        if m is None:
            continue
        t = m.get_type()
        seen[t] += 1
        if t == 'PARAM_VALUE':
            pid = m.param_id
            if isinstance(pid, bytes):
                pid = pid.decode('ascii', 'ignore')
            pid = pid.strip('\x00')
            if got is None:
                print(f"    >>> PARAM_VALUE GELDI: {pid} = {m.param_value}")
            if pid == args.param:
                got = m.param_value
                break

    print(f"\n[2] {args.secs:.0f} saniyede gorulen mesaj tipleri:")
    for t, n in seen.most_common(12):
        print(f"      {t:28s} {n}")

    print("\n=== SONUC ===")
    print(f"  Uplink (Orin->FC): {'CALISIYOR' if uplink_ok else 'YOK'}")
    if got is not None:
        print(f"  ✅ {args.param} = {got}")
        print("  -> Bu endpointte param yolu CALISIYOR.")
        print("     HUNTER kodu ayni endpointte basarisizsa sorun kodda.")
    elif seen.get('PARAM_VALUE'):
        print(f"  ⚠ PARAM_VALUE geliyor ama '{args.param}' gelmedi.")
        print("     Param adi yanlis olabilir ya da bu firmware'de yok.")
    elif seen:
        print("  ❌ HIC PARAM_VALUE gelmedi (baska mesajlar geliyor).")
        print("     -> Yol tek yonlu calisiyor. Iki olasilik:")
        print("        a) mavlink-router bu endpointte msgid 22'yi engelliyor")
        print("           (/etc/mavlink-router/main.conf -> AllowMsgIdOut / BlockMsgIdOut)")
        print("        b) FC istegi almiyor/cevaplamiyor")
        if not uplink_ok:
            print("     >>> UPLINK YOK oldugu icin (a) degil (b): isteklerimiz")
            print("         FC'ye hic ulasmiyor. Router routing/config sorunu.")
            print("     >>> KESIN TEST: router'i DURDUR, sonra")
            print("         python3 param_test.py --serial /dev/ttyACM0")
            print("         Orada calisiyorsa suclu ROUTER.")
        else:
            print("     Uplink calisiyor ama param cevabi yok -> FC tarafi.")
    else:
        print("  ❌ Hicbir mesaj yok — endpoint/router sorunu.")


if __name__ == "__main__":
    main()
