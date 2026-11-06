#!/usr/bin/env python3
"""
relay_radio.py — Victus side of the RADIO layout (3DR on the chaser's Orin).

    target source --> Victus: THIS SCRIPT --3DR--> Orin USB (chase_orin.py) --USB--> chaser FC
                                          <--3DR-- chaser status (~2 Hz) + chase_orin events

The laptop is only the "radar": it takes the TARGET's own packets from a
source and pushes them, byte-for-byte, into the 3DR whose other end is on the
chaser's Orin. Nothing here ever reaches the chaser FC (the Orin never passes
radio packets on), and by default nothing at all goes back to the target.

Sources (--target, any pymavlink connection string):
  udpin:0.0.0.0:14550   Talon / own target via Kirmizi or the other team's
                        laptop over Ethernet (Mission Planner MAVLink forward)
  COM12                 own target drone's radio directly on this laptop
                        (two 433 MHz radios side by side desense each other;
                        prefer the target radio on Kirmizi + UDP)

Forwarded (from the target's system id only): GLOBAL_POSITION_INT,
GPS_RAW_INT, HEARTBEAT. The target sysid is the first vehicle heartbeat in
the source (or --target-sysid). It may equal the chaser's: the Orin reads
target and chaser from different links.

PASSIVE by default: zero bytes written to the source. With --request-hz N
(own target only) the script asks the target for N Hz position.

Radio round trip: TIMESYNC every --rtt-ping-s (default 1 s), answered by
the Orin; logged in cmd_lat_*.csv (command=TIMESYNC). The status line also
shows the chaser's mode / fix / distance / separation from the Orin's
downlink, and chase_orin's events are printed as they arrive.

  python relay_radio.py --target udpin:0.0.0.0:14550 --radio COM5
  python relay_radio.py --target COM12 --radio COM5 --request-hz 10
"""
from __future__ import annotations

import argparse
import collections
import sys
import threading
import time

from pymavlink import mavutil

from chase_core import has_fix, haversine_m
from link_log import LinkLogger
from mav_hub import VehicleState, install_write_lock, is_vehicle_heartbeat, rx_forever
from relay_target import TargetForwarder

RELAY_SYSID = 250
TIMESYNC_ID = 111
RTT_LOST_S = 2.0


class RadioRelay:
    """All of the relay's logic, driven by two RX threads + a main loop.
    Kept free of argparse/threads so the tests can drive it directly."""

    def __init__(self, target_conn, radio_conn, *, target_sysid: int = 0,
                 request_hz: float = 0.0, fwd_max_hz: float = 0.0, link_log=None,
                 out=None):
        self.tconn = target_conn
        self.rconn = radio_conn
        self._out = out or (lambda s: print(s, flush=True))
        self.radio_lock = install_write_lock(radio_conn)
        self.target_lock = install_write_lock(target_conn)
        self.request_hz = float(request_hz or 0.0)
        self.link_log = link_log
        self.target = VehicleState("HEDEF", int(target_sysid or 0))
        self.target_locked = bool(target_sysid)
        self.chaser = VehicleState("TAKIPCI", 0)
        self.chaser_locked = False
        self.fwd = TargetForwarder(self._radio_write, pos_max_hz=fwd_max_hz)
        if self.target_locked:
            self.fwd.target_sysid = int(target_sysid)
        self.target_others = collections.Counter()   # other systems after the lock
        self.prelock = 0                # packets seen before the target heartbeat
        self.radio_others = collections.Counter()
        self.target_writes = 0          # bytes-producing sends to the SOURCE (0 if passive)
        self.radio_rx = 0
        self.events = collections.deque(maxlen=50)   # (t_mono, src, text)
        self._ts_lock = threading.Lock()
        self._ts_out = None             # ns stamp of the outstanding TIMESYNC
        self.rtt_ms = collections.deque(maxlen=20)
        self.rtt_n = self.rtt_lost = 0

    # ------------------------------------------------ writes
    def _radio_write(self, buf: bytes):
        r = self.rconn.write(buf)
        if isinstance(r, int) and r < 0:
            raise OSError("radyo yazilamadi")
        return r

    # ------------------------------------------------ source side (RX thread)
    def on_target(self, msg, t_mono: float) -> None:
        src = msg.get_srcSystem()
        typ = msg.get_type()
        if not self.target_locked and typ == "HEARTBEAT" and is_vehicle_heartbeat(msg):
            self.target.sysid = int(src)
            self.fwd.target_sysid = int(src)
            self.target_locked = True
            self._out(f"\n[HEDEF] sysid={src} kilitlendi (kaynaktaki ilk arac, tip={msg.type})")
        if self.target_locked and src == self.target.sysid:
            self.target.handle(msg, t_mono)
            self.fwd.on_msg(msg, t_mono)
        elif not self.target_locked:
            self.prelock += 1
        else:
            self.target_others[src] += 1
        if self.link_log is not None:
            self.link_log.note_message("HEDEF", msg, t_mono)

    def request_target_streams(self) -> bool:
        """Own target only (--request-hz): ask for position at request_hz."""
        if self.request_hz <= 0 or not self.target_locked:
            return False
        tsys = self.target.sysid
        for msg_id, hz in ((33, self.request_hz), (24, 1.0)):
            self.tconn.mav.command_long_send(
                tsys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
            self.target_writes += 1
        self.tconn.mav.request_data_stream_send(
            tsys, 1, mavutil.mavlink.MAV_DATA_STREAM_POSITION, int(self.request_hz), 1)
        self.target_writes += 1
        return True

    # ------------------------------------------------ radio side (RX thread)
    def on_radio(self, msg, t_mono: float) -> None:
        self.radio_rx += 1
        src = msg.get_srcSystem()
        typ = msg.get_type()
        if typ == "TIMESYNC" and int(getattr(msg, "tc1", 0)) != 0:
            self._note_timesync(int(msg.ts1), t_mono)
        elif typ == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode("ascii", errors="replace")
            text = str(text).rstrip("\x00")
            who = "TAKIPCI FC" if (self.chaser_locked and src == self.chaser.sysid) else "ORIN"
            self.events.append((t_mono, src, text))
            self._out(f"  >>> [{who}] {text}")
        if not self.chaser_locked and typ == "HEARTBEAT" and is_vehicle_heartbeat(msg):
            self.chaser.sysid = int(src)
            self.chaser_locked = True
        if self.chaser_locked and src == self.chaser.sysid:
            self.chaser.handle(msg, t_mono)
        else:
            self.radio_others[src] += 1
        if self.link_log is not None:
            self.link_log.note_message("TAKIPCI", msg, t_mono)

    # ------------------------------------------------ radio RTT (TIMESYNC)
    def ping(self, t_mono: float) -> None:
        now_ns = time.monotonic_ns()
        with self._ts_lock:
            prev = self._ts_out
            if prev is not None and (now_ns - prev) > RTT_LOST_S * 1e9:
                self.rtt_lost += 1
                if self.link_log is not None:
                    self.link_log.cmd_lat.note_lost(TIMESYNC_ID, t_mono, "no reply in 2s")
                prev = None
            if prev is not None:
                return  # one in flight at a time (unambiguous matching)
            self._ts_out = now_ns
        self.rconn.mav.timesync_send(0, now_ns)

    def _note_timesync(self, ts1: int, t_mono: float) -> None:
        now_ns = time.monotonic_ns()
        with self._ts_lock:
            if self._ts_out is None or ts1 != self._ts_out:
                return  # not ours / stale
            self._ts_out = None
        rtt = (now_ns - ts1) / 1e6
        self.rtt_ms.append(rtt)
        self.rtt_n += 1
        if self.link_log is not None:
            self.link_log.cmd_lat.note_rtt(TIMESYNC_ID, rtt, t_mono)

    # ------------------------------------------------ display
    def status_line(self, fwd_rate: float) -> str:
        ts, cs = self.target.snapshot(), self.chaser.snapshot()
        if self.target_locked:
            h = (f"HEDEF s{self.target.sysid} fix={ts['fix']}/{ts['nsat']:<2} "
                 f"{ts['hz']:4.1f}Hz yas={min(ts['age'], 99.0):5.2f}s")
        else:
            h = "HEDEF bekleniyor (kaynakta arac heartbeat'i yok)"
        if self.chaser_locked:
            c = (f"TAKIPCI mod={cs['mode']:<8} fix={cs['fix']}/{cs['nsat']:<2} "
                 f"{cs['hz']:3.1f}Hz yas={min(cs['age'], 99.0):5.2f}s")
        else:
            c = "TAKIPCI: Orin'den veri yok"
        geo = ""
        if has_fix(ts) and has_fix(cs):
            d = haversine_m(cs["lat"], cs["lon"], ts["lat"], ts["lon"])
            geo = f" | d={d:6.0f} m ayrim={ts['alt_amsl'] - cs['alt_amsl']:+5.1f} m"
        r = sorted(self.rtt_ms)
        rtt = f"rtt {r[len(r) // 2]:4.0f} ms" if r else "rtt    - ms"
        with self.fwd.lock:
            sent = self.fwd.sent
        return (f"  {h} | {c}{geo} | aktarim {fwd_rate:4.1f}/s (toplam {sent}, "
                f"hata {self.fwd.errors}) | {rtt} (n={self.rtt_n} kayip={self.rtt_lost})")


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True,
                    help="hedef kaynagi: udpin:0.0.0.0:14550 (Kirmizi/ELRS/Talon) veya COM12")
    ap.add_argument("--target-baud", type=int, default=57600, help="kaynak seri ise baud")
    ap.add_argument("--radio", required=True, help="Orin'e giden 3DR'nin yer ucu, or: COM5")
    ap.add_argument("--radio-baud", type=int, default=57600)
    ap.add_argument("--target-sysid", type=int, default=0,
                    help="hedef sysid. 0 = kaynaktaki ilk arac heartbeat'i (varsayilan)")
    ap.add_argument("--request-hz", type=float, default=0.0,
                    help="SADECE kendi hedefin: kaynaktan bu hizda konum iste (Hz). "
                         "0 = PASIF, kaynaga tek bayt gitmez (Talon/baskasinin ucagi icin sart).")
    ap.add_argument("--fwd-max-hz", type=float, default=0.0,
                    help="aktarilan konum hizina ust sinir (Hz). 0 = gelen her paket.")
    ap.add_argument("--rtt-ping-s", type=float, default=1.0,
                    help="radyo RTT icin TIMESYNC araligi (s). 0 = kapali (radyoya sadece hedef gider).")
    ap.add_argument("--status-s", type=float, default=1.0, help="durum satiri araligi (s)")
    ap.add_argument("--no-log", action="store_true", help="loglamayi kapat")
    ap.add_argument("--log-dir", default="logs", help="log klasoru")
    return ap


def main(argv=None, *, clock=time, connect=None):
    a = build_parser().parse_args(argv)
    connect = connect or (lambda dev, baud: mavutil.mavlink_connection(
        dev, baud=baud, source_system=RELAY_SYSID, source_component=191))
    link_log = LinkLogger(a.log_dir,
                          meta={"script": "relay_radio", "target": a.target, "radio": a.radio,
                                "request_hz": a.request_hz, "fwd_max_hz": a.fwd_max_hz,
                                "rtt_ping_s": a.rtt_ping_s},
                          enabled=not a.no_log, with_relay=False, with_cmd=a.rtt_ping_s > 0)
    try:
        tconn = connect(a.target, a.target_baud)
        rconn = connect(a.radio, a.radio_baud)
    except OSError as e:
        link_log.close()
        sys.exit(f"[X] Port acilamadi: {e}  (COM numarasi? baska program tutuyor mu? "
                 f"UDP 14550 icin guvenlik duvari kurali?)")
    relay = RadioRelay(tconn, rconn, target_sysid=a.target_sysid, request_hz=a.request_hz,
                       fwd_max_hz=a.fwd_max_hz, link_log=link_log)
    state = {"run": True}
    running = lambda: state["run"]  # noqa: E731
    reopen_t = not str(a.target).lower().startswith(("udp", "tcp"))
    for conn, fn, name, reopen, lock in (
            (tconn, relay.on_target, "HEDEF", reopen_t, relay.target_lock),
            (rconn, relay.on_radio, "RADYO", True, relay.radio_lock)):
        threading.Thread(target=rx_forever, args=(conn, fn, running),
                         kwargs=dict(name=name, reopen=reopen, write_lock=lock),
                         daemon=True).start()

    mode = (f"kaynaktan {a.request_hz:g} Hz konum isteniyor" if a.request_hz > 0
            else "PASIF: kaynaga hicbir sey gonderilmez")
    print(f"\n[i] Hedef kaynagi: {a.target}  ->  3DR {a.radio} @ {a.radio_baud} -> Orin")
    print(f"[i] {mode}. Tipler: GLOBAL_POSITION_INT, GPS_RAW_INT, HEARTBEAT.")
    print("[i] Bu script UCUS KOMUTU GONDERMEZ; beyin Orin'de (chase_orin.py --target-link ...).")
    if a.rtt_ping_s > 0:
        print(f"[i] Radyo RTT: her {a.rtt_ping_s:g} s TIMESYNC (Orin cevaplar) -> cmd_lat CSV")
    print("    Ctrl-C ile cik.\n")

    last_status = last_req = last_ping = clock.monotonic()
    last_sent = 0
    try:
        while True:
            clock.sleep(0.2)
            now = clock.monotonic()
            if a.request_hz > 0 and now - last_req > 5.0:
                try:
                    relay.request_target_streams()
                except Exception:
                    pass
                last_req = now
            if a.rtt_ping_s > 0 and now - last_ping >= a.rtt_ping_s:
                try:
                    relay.ping(now)
                except Exception:
                    pass
                last_ping = now
            if now - last_status < a.status_s:
                continue
            dt = now - last_status
            last_status = now
            with relay.fwd.lock:
                sent = relay.fwd.sent
            rate = (sent - last_sent) / dt if dt > 0 else 0.0
            last_sent = sent
            print(relay.status_line(rate), flush=True)
    except KeyboardInterrupt:
        print("\n[i] Durduruldu.")
    finally:
        state["run"] = False
        print(f"[i] aktarilan={relay.fwd.sent} paket ({relay.fwd.bytes} bayt) {relay.fwd.by_type} "
              f"| kaynaga yazilan={relay.target_writes} | kilit oncesi={relay.prelock} "
              f"kaynakta hedef disi={dict(relay.target_others)} "
              f"| radyo RTT n={relay.rtt_n} kayip={relay.rtt_lost}")
        link_log.close()
    return relay


if __name__ == "__main__":
    main()
