#!/usr/bin/env python3
"""
mav_hub — the Orin's MAVLink connections, with one view per vehicle.

Two layouts are supported:

RADIO mode (field layout from 24 Oct 2026): the target's packets arrive on
their OWN link — the chaser's 3DR plugged into the Orin's USB — and the
chaser FC is reached over its USB through MAVProxy:

    Victus (relay_radio.py) --3DR--> Orin USB  /dev/serial/by-id/...  target only
    chaser FC --USB--> MAVProxy --UDP 14551--> Orin                   chaser only

    Each vehicle is read from its own link only, so the two may share a
    system id (a Talon is usually sysid 1, same as the chaser). The target's
    sysid is locked onto the first vehicle heartbeat seen on the radio
    (or given). On the radio link the Orin only transmits small things:
    TIMESYNC replies (Victus measures the radio round trip), the chaser's own
    HEARTBEAT / GLOBAL_POSITION_INT / GPS_RAW_INT / STATUSTEXT rate-limited
    (downlink_hz), and short STATUSTEXT events from chase_orin. Radio
    packets are NEVER passed to the FC.

LEGACY mode (radio soldered on the FC): ONE link carries both vehicles,
the laptop's relay_target.py injects the target's packets into the chaser's
radio, ArduPilot routes them out of USB, and the vehicles are told apart by
system id only (they must differ).

Pitfalls handled here (both modes):
  * pymavlink locks `target_system` onto the FIRST vehicle heartbeat it sees
    (read-only property). PinnedLink addresses the chaser explicitly, always.
  * The RX thread is the only reader of a link; parameter read-backs wait on
    it (wait_param) instead of reading the link from a second thread.
  * Several threads write (heartbeat, main loop, downlink): writes are
    serialized per link.
  * A USB serial link that drops (connector, re-enumeration) is re-opened by
    its RX thread; meanwhile the target goes stale and the brain holds.
"""
from __future__ import annotations

import collections
import threading
import time
from typing import Callable, Optional

from pymavlink import mavutil

# ArduCopter mode name -> number (mavutil ships number -> name).
COPTER_MODE_IDS = {name: num for num, name in mavutil.mode_mapping_acm.items()}

_NON_VEHICLE_TYPES = {
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_TYPE_GIMBAL,
    mavutil.mavlink.MAV_TYPE_ADSB,
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
}

# Downlink (radio mode): chaser message types sent to the ground and the
# minimum spacing between two of the same type (s). GPI uses downlink_hz.
_DOWNLINK_MIN_DT = {"HEARTBEAT": 0.9, "GPS_RAW_INT": 1.9, "STATUSTEXT": 0.2}
EVENT_MIN_DT = 0.3          # chase_orin event STATUSTEXTs (non-forced)
STATUSTEXT_MAX = 50         # MAVLink STATUSTEXT text field


def is_vehicle_heartbeat(msg) -> bool:
    try:
        return (msg.type not in _NON_VEHICLE_TYPES
                and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID)
    except AttributeError:
        return False


def install_write_lock(conn):
    """Serialize writes on a link (several threads send). Returns the lock."""
    lock = getattr(conn, "_hub_lock", None)
    if lock is not None:
        return lock
    lock = threading.Lock()
    if not hasattr(conn, "write"):
        return lock
    raw_write = conn.write

    def locked_write(buf):
        with lock:
            return raw_write(buf)

    conn.write = locked_write
    conn._hub_locked = True
    conn._hub_lock = lock
    return lock


def rx_forever(conn, handle: Callable, running: Callable[[], bool], *, name: str,
               reopen: bool = False, write_lock=None, retry_s: float = 1.0,
               log: Callable[[str], None] | None = None) -> None:
    """The ONLY reader of `conn`: recv -> handle(msg, t_mono) until running()
    is False. On a link error it waits retry_s and, with reopen=True, tries
    conn.reset() (pymavlink serial re-open of the same device path — use a
    /dev/serial/by-id path so re-enumeration still resolves)."""
    log = log or (lambda s: print(s, flush=True))
    dead = False
    while running():
        try:
            msg = conn.recv_match(blocking=True, timeout=0.5)
        except Exception as e:  # unplugged / dead port
            if not dead:
                log(f"\n[{name}] link hatasi: {e}" + (" - yeniden aciliyor" if reopen else ""))
                dead = True
            time.sleep(retry_s)
            reset = getattr(conn, "reset", None) if reopen else None
            if callable(reset):
                try:
                    if write_lock is not None:
                        with write_lock:
                            ok = reset()
                    else:
                        ok = reset()
                except Exception:
                    ok = False
                if ok:
                    log(f"[{name}] link yeniden acildi")
                    dead = False
            continue
        if msg is None:
            continue
        dead = False
        if msg.get_type() == "BAD_DATA":
            continue
        handle(msg, time.monotonic())


class VehicleState:
    """Per-vehicle view with the same snapshot() contract as follow_gps.Vehicle."""

    def __init__(self, name: str, sysid: int, *, print_statustext: bool = False):
        self.name = name
        self.sysid = int(sysid)
        self.print_statustext = print_statustext
        self.lock = threading.Lock()
        self.lat = self.lon = None
        self.alt_amsl = self.alt_rel = None
        self.vx = self.vy = self.vz = 0.0
        self.fix = self.nsat = 0
        self.mode = "?"
        self.armed = False
        self.pos_mono: Optional[float] = None
        self.hb_mono: Optional[float] = None
        self.last_raw = None
        self.roll = self.pitch = self.yaw = None
        self.att_mono: Optional[float] = None
        self.pos_times = collections.deque(maxlen=40)
        self.new_pos = threading.Event()   # set on every position (wakes the loop)
        self._params: dict = {}
        self._param_cv = threading.Condition()

    # --- called by an RX thread only ---
    def handle(self, msg, t_mono: float) -> None:
        t = msg.get_type()
        if t == "PARAM_VALUE":
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", errors="ignore")
            pid = pid.rstrip("\x00")
            with self._param_cv:
                self._params[pid] = (float(msg.param_value), t_mono)
                self._param_cv.notify_all()
            return
        with self.lock:
            if t == "GLOBAL_POSITION_INT":
                self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
                self.alt_amsl = msg.alt / 1000.0
                self.alt_rel = msg.relative_alt / 1000.0
                self.vx, self.vy, self.vz = msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0
                self.pos_mono = t_mono
                self.pos_times.append(t_mono)
                self.last_raw = msg.get_msgbuf()
                self.new_pos.set()
            elif t == "GPS_RAW_INT":
                self.fix, self.nsat = msg.fix_type, msg.satellites_visible
            elif t == "HEARTBEAT" and is_vehicle_heartbeat(msg):
                self.mode = mavutil.mode_string_v10(msg)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.hb_mono = t_mono
            elif t == "ATTITUDE":
                self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
                self.att_mono = t_mono
            elif t == "STATUSTEXT" and self.print_statustext:
                print(f"\n[{self.name} FC] {msg.text}", flush=True)

    # --- readers ---
    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            ts = list(self.pos_times)
            hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) >= 3 and ts[-1] > ts[0] else 0.0
            age = (now - self.pos_mono) if self.pos_mono is not None else 1e9
            return dict(lat=self.lat, lon=self.lon, alt_amsl=self.alt_amsl,
                        alt_rel=self.alt_rel, vx=self.vx, vy=self.vy, vz=self.vz,
                        fix=self.fix, nsat=self.nsat, mode=self.mode, armed=self.armed,
                        age=age, hz=hz, raw=self.last_raw, pos_mono=self.pos_mono,
                        roll=self.roll, pitch=self.pitch, yaw=self.yaw,
                        att_age=(now - self.att_mono) if self.att_mono is not None else None,
                        hb_age=(now - self.hb_mono) if self.hb_mono is not None else None)

    def wait_param(self, name: str, *, since_mono: float, timeout: float):
        deadline = time.monotonic() + float(timeout)
        with self._param_cv:
            while True:
                rec = self._params.get(name)
                if rec is not None and rec[1] >= since_mono:
                    return rec[0]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._param_cv.wait(remaining)


class PinnedLink:
    """Looks like a mavutil connection to ChaserEnv, but always addresses one
    system id — immune to pymavlink locking target_system onto the target."""

    def __init__(self, conn, sysid: int, compid: int = 1):
        self._conn = conn
        self.mav = conn.mav
        self.target_system = int(sysid)
        self.target_component = int(compid)

    def recv_match(self, *a, **k):
        raise RuntimeError("PinnedLink is send-only; the hub RX thread owns reading")


class ChaserHandle:
    """What ChaserEnv receives: a pinned send link + the RX-thread param waiter."""

    def __init__(self, link: PinnedLink, state: VehicleState):
        self.conn = link
        self.wait_param = state.wait_param
        self.sysid = link.target_system


class MavHub:
    def __init__(self, conn, *, chaser_sysid: int = 1, target_sysid: int = 2,
                 on_msg: Optional[Callable] = None, print_statustext: bool = True,
                 target_conn=None, downlink_hz: float = 2.0):
        self.radio = target_conn is not None
        if not self.radio and int(chaser_sysid) == int(target_sysid):
            raise ValueError("chaser and target must have different system ids "
                             "(set SYSID_THISMAV=2 on the target)")
        self.conn = conn
        self._fc_lock = install_write_lock(conn)
        self.target_conn = target_conn
        self._radio_lock = install_write_lock(target_conn) if self.radio else None
        self.chaser = VehicleState("TAKIPCI", chaser_sysid, print_statustext=print_statustext)
        self.target = VehicleState("HEDEF", int(target_sysid or 0))
        # radio mode with target_sysid 0: lock onto the first vehicle on the radio
        self.target_locked = (not self.radio) or bool(target_sysid)
        self.on_msg = on_msg
        self.others = collections.Counter()        # FC link: sysid -> count
        self.radio_others = collections.Counter()  # radio link: sysid -> count (after lock)
        self.radio_prelock = 0                     # radio packets before the target lock
        self.radio_rx = 0
        self.downlink_hz = float(downlink_hz or 0.0)
        self._dl_last: dict = {}
        self._ev_last = -1e9
        self.downlink_sent = self.downlink_errors = 0
        self.timesync_replies = 0
        self.events_sent = self.events_dropped = 0
        self._running = False
        self._threads: list = []

    @classmethod
    def connect(cls, device: str, *, baud: int = 115200, source_system: int = 251,
                source_component: int = 191, target_device: str | None = None,
                target_baud: int = 57600, **kw) -> "MavHub":
        # Our own sysid (251) differs from the laptop relay's (250) and
        # MAVProxy's (250 in run_router_mavproxy.sh) so ACK routing and the
        # FC's GCS bookkeeping (MAV_GCS_SYSID) can single us out.
        conn = mavutil.mavlink_connection(device, baud=baud, source_system=source_system,
                                          source_component=source_component)
        tconn = None
        if target_device:
            tconn = mavutil.mavlink_connection(target_device, baud=target_baud,
                                               source_system=source_system,
                                               source_component=source_component)
            # Bytes the radio delivered while nobody had the port open are old
            # positions: drop them so the first fixes we see are live.
            try:
                tconn.port.reset_input_buffer()
            except Exception:
                pass
        return cls(conn, target_conn=tconn, **kw)

    # kept for callers/tests of the legacy API
    @staticmethod
    def _install_write_lock(conn) -> None:
        install_write_lock(conn)

    # ------------------------------------------------ threads
    def start(self, *, heartbeat_s: float = 1.0) -> None:
        self._running = True
        self.send_heartbeat()  # first packet: lets a MAVProxy udpin output learn us
        running = lambda: self._running  # noqa: E731
        specs = [
            (rx_forever, (self.conn, self.dispatch, running),
             dict(name="HUB", reopen=False, retry_s=0.2)),
            (self._hb_loop, (heartbeat_s,), {}),
        ]
        if self.radio:
            specs.append((rx_forever, (self.target_conn, self.dispatch_radio, running),
                          dict(name="RADYO", reopen=True, write_lock=self._radio_lock,
                               retry_s=1.0)))
        for fn, args, kw in specs:
            th = threading.Thread(target=fn, args=args, kwargs=kw, daemon=True)
            th.start()
            self._threads.append(th)

    def stop(self) -> None:
        self._running = False

    def _hb_loop(self, period: float) -> None:
        while self._running:
            time.sleep(period)
            try:
                self.send_heartbeat()
            except Exception:
                pass

    # ------------------------------------------------ dispatch
    def dispatch(self, msg, t_mono: float) -> None:
        """FC link (MAVProxy / USB)."""
        src = msg.get_srcSystem()
        if src == self.chaser.sysid:
            self.chaser.handle(msg, t_mono)
            name = "TAKIPCI"
            if self.radio:
                self._maybe_downlink(msg, t_mono)
        elif not self.radio and src == self.target.sysid:
            self.target.handle(msg, t_mono)
            name = "HEDEF"
        else:
            self.others[src] += 1
            name = "TAKIPCI"  # logged with its src_sys column in the chaser file
        self._note(name, msg, t_mono)

    def dispatch_radio(self, msg, t_mono: float) -> None:
        """Radio link (3DR on the Orin): the target only. Never forwarded to the FC."""
        self.radio_rx += 1
        src = msg.get_srcSystem()
        typ = msg.get_type()
        if typ == "TIMESYNC" and int(getattr(msg, "tc1", 1)) == 0:
            self._reply_timesync(msg)
        if not self.target_locked and typ == "HEARTBEAT" and is_vehicle_heartbeat(msg):
            self.target.sysid = int(src)
            self.target_locked = True
            print(f"\n[HUB] hedef sysid={src} kilitlendi (radyoda ilk arac)", flush=True)
        if self.target_locked and src == self.target.sysid:
            self.target.handle(msg, t_mono)
        elif not self.target_locked:
            self.radio_prelock += 1
        else:
            self.radio_others[src] += 1
        self._note("HEDEF", msg, t_mono)

    def _note(self, name, msg, t_mono):
        if self.on_msg is not None:
            try:
                self.on_msg(name, msg, t_mono)
            except Exception:
                pass

    # ------------------------------------------------ radio transmit
    def _radio_write(self, buf: bytes) -> bool:
        try:
            r = self.target_conn.write(buf)
        except Exception:
            r = -1
        if r is not None and isinstance(r, int) and r < 0:
            self.downlink_errors += 1
            return False
        return True

    def _reply_timesync(self, msg) -> None:
        try:
            self.target_conn.mav.timesync_send(max(1, time.monotonic_ns()), int(msg.ts1))
            self.timesync_replies += 1
        except Exception:
            self.downlink_errors += 1

    def _maybe_downlink(self, msg, t_mono: float) -> None:
        """Chaser FC packet -> ground, byte-for-byte, rate-limited."""
        if self.downlink_hz <= 0:
            return
        typ = msg.get_type()
        if typ == "GLOBAL_POSITION_INT":
            min_dt = max(0.0, 1.0 / self.downlink_hz - 0.02)
        else:
            min_dt = _DOWNLINK_MIN_DT.get(typ)
            if min_dt is None:
                return
            if typ == "HEARTBEAT" and not is_vehicle_heartbeat(msg):
                return
        last = self._dl_last.get(typ)
        if last is not None and (t_mono - last) < min_dt:
            return
        buf = msg.get_msgbuf()
        if not buf:
            return
        self._dl_last[typ] = t_mono
        if self._radio_write(bytes(buf)):
            self.downlink_sent += 1

    def send_event(self, text: str, *, severity: int = 6, force: bool = False,
                   now: float | None = None) -> bool:
        """Short status text to the ground (Victus prints it). Radio mode only."""
        if not self.radio or self.downlink_hz <= 0:
            return False
        now = time.monotonic() if now is None else float(now)
        if not force and (now - self._ev_last) < EVENT_MIN_DT:
            self.events_dropped += 1
            return False
        self._ev_last = now
        raw = str(text).encode("ascii", "replace")[:STATUSTEXT_MAX]
        try:
            self.target_conn.mav.statustext_send(int(severity), raw)
        except Exception:
            self.downlink_errors += 1
            return False
        self.events_sent += 1
        return True

    # ------------------------------------------------ FC sends
    def send_heartbeat(self) -> None:
        self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def request_streams(self, *, pos_hz: float, att_hz: float) -> None:
        """Rates on OUR port (USB) only."""
        tsys = self.chaser.sysid
        for msg_id, hz in ((33, pos_hz), (24, 1.0), (30, att_hz)):
            if hz and hz > 0:
                self.conn.mav.command_long_send(
                    tsys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def set_chaser_mode(self, name: str, timeout: float = 4.0) -> bool:
        if name not in COPTER_MODE_IDS:
            print(f"[HUB] bilinmeyen mod: {name}", flush=True)
            return False
        self.conn.mav.set_mode_send(self.chaser.sysid,
                                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                    COPTER_MODE_IDS[name])
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.2)
            if self.chaser.mode == name:
                print(f"[TAKIPCI] mod = {name}", flush=True)
                return True
        print(f"[TAKIPCI] mod {name} dogrulanamadi (su an: {self.chaser.mode})", flush=True)
        return False

    def chaser_handle(self) -> ChaserHandle:
        return ChaserHandle(PinnedLink(self.conn, self.chaser.sysid, 1), self.chaser)

    def wait_ready(self, timeout: float) -> tuple[bool, bool]:
        """(chaser heartbeat seen, target position seen) within timeout."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            c_ok = self.chaser.hb_mono is not None
            t_ok = self.target.pos_mono is not None
            if c_ok and t_ok:
                return True, True
            time.sleep(0.2)
        return self.chaser.hb_mono is not None, self.target.pos_mono is not None
