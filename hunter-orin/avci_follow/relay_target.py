#!/usr/bin/env python3
"""
relay_target.py — laptop side of the Orin architecture (Asama 1).

The brain runs on the chaser's Orin (chase_orin.py). The laptop's only job is
to be the "radar": take the TARGET drone's own packets from its radio and
push them, byte-for-byte, into the CHASER's radio. ArduPilot's MAVLink routing
forwards those packets out of the chaser FC's USB port to the Orin.

    HEDEF FC --SiK--> laptop COM(hedef) --SiK--> TAKIPCI FC --USB--> Orin
                      (this script: forward only, sends NO flight commands)

Forwarded (from the target's system id only): GLOBAL_POSITION_INT (position +
velocity), GPS_RAW_INT (fix), HEARTBEAT (mode). Nothing else — keeps the
half-duplex chaser radio free. Both radios are logged per message (link_log).

Latency: the chaser radio's round trip is measured exactly as follow_gps does
it in laptop mode — a side-effect-free REQUEST_MESSAGE ping (--rtt-ping-s,
default 1 s) timed to its COMMAND_ACK, on the laptop's clock -> cmd_lat CSV.
With the Orin's own USB RTT (its cmd_lat), the extra hop the target data now
takes is ~ (radio RTT + USB RTT) / 2. The ping changes nothing on the FC.

REQUIREMENT: the two drones need different system ids (target
SYSID_THISMAV=2). Otherwise neither the chaser FC nor the Orin can tell them
apart; the relay refuses to start.

  python relay_target.py --target-port COM5 --chaser-port COM14
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from chase_core import has_fix, haversine_m
from chaser_env import ChaserEnv
from follow_gps import Vehicle
from link_log import LinkLogger

RELAY_SYSID = 250  # Vehicle() connects with source_system=250; ACKs come back to it

FORWARD_TYPES = ("GLOBAL_POSITION_INT", "GPS_RAW_INT", "HEARTBEAT")


class TargetForwarder:
    """Forwards selected target packets, unchanged, through write_fn."""

    def __init__(self, write_fn, *, types=FORWARD_TYPES, pos_max_hz: float = 0.0):
        self.write_fn = write_fn
        self.target_sysid = None          # set once the target link is up
        self.types = frozenset(types)
        self.pos_min_dt = (1.0 / pos_max_hz) if pos_max_hz and pos_max_hz > 0 else 0.0
        self._last_pos_t = None
        self.lock = threading.Lock()
        self.sent = 0
        self.bytes = 0
        self.by_type = {}
        self.skipped_foreign = 0
        self.decimated = 0
        self.errors = 0

    def on_msg(self, msg, t_mono: float) -> bool:
        if self.target_sysid is None:
            return False
        try:
            if msg.get_srcSystem() != self.target_sysid:
                self.skipped_foreign += 1   # crosstalk / radio's own packets
                return False
            typ = msg.get_type()
        except Exception:
            return False
        if typ not in self.types:
            return False
        if typ == "GLOBAL_POSITION_INT" and self.pos_min_dt > 0:
            if self._last_pos_t is not None and (t_mono - self._last_pos_t) < self.pos_min_dt - 1e-3:
                self.decimated += 1
                return False
            self._last_pos_t = t_mono
        buf = msg.get_msgbuf()
        if not buf:
            return False
        try:
            self.write_fn(bytes(buf))
        except Exception:
            self.errors += 1
            return False
        with self.lock:
            self.sent += 1
            self.bytes += len(buf)
            self.by_type[typ] = self.by_type.get(typ, 0) + 1
        return True


def locked_writer(conn):
    """The chaser link is written by two threads (forwarding + stream
    re-requests); serialize so frames never interleave on the serial port."""
    lock = threading.Lock()
    raw_write = conn.write

    def write(buf):
        with lock:
            return raw_write(buf)

    conn.write = write  # mavlink sends (stream requests) go through it too
    return write


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target-port", required=True, help="hedef drone radyosu, or: COM5")
    ap.add_argument("--chaser-port", required=True, help="takipci drone radyosu, or: COM14")
    ap.add_argument("--target-baud", type=int, default=57600)
    ap.add_argument("--chaser-baud", type=int, default=57600)
    ap.add_argument("--target-hz", type=float, default=10.0,
                    help="hedeften istenen konum hizi (Hz) - Orin'e bu hizda gider")
    ap.add_argument("--chaser-hz", type=float, default=4.0,
                    help="takipciden istenen konum hizi (Hz). Laptopta SADECE izleme icin; "
                         "dusuk tut, yarim-dubleks radyoda hedef aktarimina yer kalsin.")
    ap.add_argument("--fwd-max-hz", type=float, default=0.0,
                    help="aktarilan konum hizina ust sinir (Hz). 0 = gelen her paket. "
                         "Takipci radyosu bogulursa 5 gibi bir deger ver.")
    ap.add_argument("--status-s", type=float, default=1.0, help="durum satiri araligi (s)")
    ap.add_argument("--rtt-ping-s", type=float, default=1.0,
                    help="takipci radyosunun RTT'si icin bu araliklarla zararsiz "
                         "REQUEST_MESSAGE ping'i (s). follow_gps'teki ile ayni yontem. "
                         "0 = kapali.")
    ap.add_argument("--no-log", action="store_true", help="link loglamayi kapat")
    ap.add_argument("--log-dir", default="logs", help="log klasoru")
    a = ap.parse_args()

    link_log = LinkLogger(a.log_dir, meta={"script": "relay_target", "target_hz": a.target_hz,
                                           "chaser_hz": a.chaser_hz, "fwd_max_hz": a.fwd_max_hz,
                                           "rtt_ping_s": a.rtt_ping_s},
                          enabled=not a.no_log, with_relay=False, with_cmd=a.rtt_ping_s > 0)

    ids = {"chaser": None}  # known once the chaser link is up

    def _chaser_on_msg(name, msg, t_mono):
        link_log.note_message(name, msg, t_mono)
        # RTT: only ACKs from the chaser addressed to us (never the Orin's/MAVProxy's)
        if ids["chaser"] is not None and msg.get_type() == "COMMAND_ACK":
            tgt = getattr(msg, "target_system", 0) or 0
            if msg.get_srcSystem() == ids["chaser"] and tgt in (0, RELAY_SYSID):
                link_log.cmd_lat.note_ack(msg, t_mono)

    chaser = Vehicle("TAKIPCI", a.chaser_port, a.chaser_baud, a.chaser_hz,
                     on_msg=_chaser_on_msg)
    ids["chaser"] = chaser.sysid
    fwd = TargetForwarder(locked_writer(chaser.conn), pos_max_hz=a.fwd_max_hz)

    def _target_on_msg(name, msg, t_mono):
        link_log.note_message(name, msg, t_mono)
        fwd.on_msg(msg, t_mono)

    target = Vehicle("HEDEF  ", a.target_port, a.target_baud, a.target_hz,
                     on_msg=_target_on_msg)

    if target.sysid == chaser.sysid:
        link_log.close()
        sys.exit(f"[X] Iki drone da sysid={target.sysid}. Hedefte SYSID_THISMAV=2 yap "
                 f"(Mission Planner > Full Parameter List > yaz > yeniden baslat). "
                 f"Ayni sysid ile takipci FC ve Orin iki drone'u ayiramaz.")
    fwd.target_sysid = target.sysid
    # Same ping as follow_gps (ChaserEnv.ping_rtt): read-only on the FC.
    pinger = ChaserEnv(chaser, verbose_speed=False, verbose_param=False,
                       cmd_latency=link_log.cmd_lat) if a.rtt_ping_s > 0 else None
    print(f"\n[i] Aktarim acik: hedef sysid={target.sysid} -> takipci radyosu "
          f"(takipci sysid={chaser.sysid}). Tipler: {', '.join(FORWARD_TYPES)}")
    print("[i] Bu script takipciye UCUS KOMUTU GONDERMEZ; beyin Orin'de (chase_orin.py).")
    if pinger is not None:
        print(f"[i] Takipci radyosu RTT: her {a.rtt_ping_s:g} s zararsiz ping "
              f"(laptop modundaki ile ayni yontem) -> cmd_lat CSV")
    print("    Ctrl-C ile cik.\n")

    last_status = last_req = last_ping = time.monotonic()
    last_sent = 0
    try:
        while True:
            time.sleep(0.2)
            now = time.monotonic()
            if now - last_req > 5.0:  # re-request streams (lossy links drop them)
                try:
                    target._request_streams()
                    chaser._request_streams()
                except Exception:
                    pass
                last_req = now
            if pinger is not None and now - last_ping >= a.rtt_ping_s:
                try:
                    pinger.ping_rtt(now)
                except Exception:
                    pass
                last_ping = now
            if now - last_status < a.status_s:
                continue
            dt = now - last_status
            last_status = now
            ts, cs = target.snapshot(), chaser.snapshot()
            with fwd.lock:
                sent = fwd.sent
            rate = (sent - last_sent) / dt if dt > 0 else 0.0
            last_sent = sent
            d = (f"{haversine_m(cs['lat'], cs['lon'], ts['lat'], ts['lon']):6.0f} m"
                 if has_fix(ts) and has_fix(cs) else "     ? m")
            sep = (f"{ts['alt_amsl'] - cs['alt_amsl']:+5.1f} m"
                   if ts["alt_amsl"] is not None and cs["alt_amsl"] is not None else "   ? m")
            rtt = ""
            if pinger is not None:
                n_rtt, n_lost, med = link_log.cmd_lat.summary()
                rtt = (f" | rtt {med:4.0f} ms" if med is not None else " | rtt    - ms") + \
                      f" (n={n_rtt} kayip={n_lost})"
            print(f"  HEDEF fix={ts['fix']}/{ts['nsat']:<2} {ts['hz']:4.1f}Hz yas={ts['age']:5.2f}s | "
                  f"TAKIPCI mod={cs['mode']:<8} fix={cs['fix']}/{cs['nsat']:<2} {cs['hz']:4.1f}Hz "
                  f"yas={cs['age']:5.2f}s | d={d} ayrim={sep} | aktarim {rate:4.1f}/s "
                  f"(toplam {sent}, seyreltme {fwd.decimated}, hata {fwd.errors}){rtt}", flush=True)
    except KeyboardInterrupt:
        print("\n[i] Durduruldu.")
    finally:
        print(f"[i] aktarilan={fwd.sent} paket ({fwd.bytes} bayt) {fwd.by_type} "
              f"yabanci-atlanan={fwd.skipped_foreign}")
        link_log.close()


if __name__ == "__main__":
    main()
