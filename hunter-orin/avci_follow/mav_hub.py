#!/usr/bin/env python3
"""
mav_hub — ONE MAVLink connection carrying TWO vehicles (Orin side).

On the Orin the chaser FC is reached over USB through MAVProxy. The laptop
(relay_target.py) injects the TARGET drone's own packets into the chaser's
telemetry radio; ArduPilot's MAVLink routing forwards those (non-addressed)
packets out of its other ports, so the Orin sees both vehicles on the same
link, told apart only by system id:

    sysid 1  chaser FC (our own vehicle)          -> commands go here
    sysid 2  target FC (forwarded by the chaser)  -> position source only

Two pitfalls this module exists to handle:
  * pymavlink locks `target_system` onto the FIRST vehicle heartbeat it sees
    (read-only property). With two vehicles on one link that may be the
    target, and every command would silently go to the wrong drone.
    PinnedLink addresses the chaser explicitly, always.
  * The RX thread is the only reader of the link; parameter read-backs wait
    on it (wait_param) instead of reading the link from a second thread.
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


def is_vehicle_heartbeat(msg) -> bool:
    try:
        return (msg.type not in _NON_VEHICLE_TYPES
                and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID)
    except AttributeError:
        return False


class VehicleState:
    """Per-sysid view with the same snapshot() contract as follow_gps.Vehicle."""

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

    # --- called by the hub RX thread only ---
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
                 on_msg: Optional[Callable] = None, print_statustext: bool = True):
        if int(chaser_sysid) == int(target_sysid):
            raise ValueError("chaser and target must have different system ids "
                             "(set SYSID_THISMAV=2 on the target)")
        self.conn = conn
        self._install_write_lock(conn)
        self.chaser = VehicleState("TAKIPCI", chaser_sysid, print_statustext=print_statustext)
        self.target = VehicleState("HEDEF", target_sysid)
        self.on_msg = on_msg
        self.others = collections.Counter()   # sysid -> count (MAVProxy, laptop, ...)
        self._running = False
        self._threads: list = []

    @classmethod
    def connect(cls, device: str, *, baud: int = 115200, source_system: int = 251,
                source_component: int = 191, **kw) -> "MavHub":
        # Our own sysid (251) differs from the laptop relay's (250) so the FC's
        # routing table never confuses who should get a COMMAND_ACK back.
        conn = mavutil.mavlink_connection(device, baud=baud, source_system=source_system,
                                          source_component=source_component)
        return cls(conn, **kw)

    @staticmethod
    def _install_write_lock(conn) -> None:
        """Several threads send (heartbeat, main loop); serialize the writes."""
        if getattr(conn, "_hub_locked", False) or not hasattr(conn, "write"):
            return
        lock = threading.Lock()
        raw_write = conn.write

        def locked_write(buf):
            with lock:
                return raw_write(buf)

        conn.write = locked_write
        conn._hub_locked = True

    # ------------------------------------------------ threads
    def start(self, *, heartbeat_s: float = 1.0) -> None:
        self._running = True
        self.send_heartbeat()  # first packet: lets a MAVProxy udpin output learn us
        for fn, arg in ((self._rx_loop, None), (self._hb_loop, heartbeat_s)):
            th = threading.Thread(target=fn, args=(() if arg is None else (arg,)), daemon=True)
            th.start()
            self._threads.append(th)

    def stop(self) -> None:
        self._running = False

    def _rx_loop(self) -> None:
        while self._running:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as e:  # link hiccup: keep going
                print(f"\n[HUB] link hatasi: {e}", flush=True)
                time.sleep(0.2)
                continue
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            self.dispatch(msg, time.monotonic())

    def _hb_loop(self, period: float) -> None:
        while self._running:
            time.sleep(period)
            try:
                self.send_heartbeat()
            except Exception:
                pass

    def dispatch(self, msg, t_mono: float) -> None:
        src = msg.get_srcSystem()
        if src == self.chaser.sysid:
            self.chaser.handle(msg, t_mono)
            name = "TAKIPCI"
        elif src == self.target.sysid:
            self.target.handle(msg, t_mono)
            name = "HEDEF"
        else:
            self.others[src] += 1
            name = "TAKIPCI"  # logged with its src_sys column in the chaser file
        if self.on_msg is not None:
            try:
                self.on_msg(name, msg, t_mono)
            except Exception:
                pass

    # ------------------------------------------------ sends
    def send_heartbeat(self) -> None:
        self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def request_streams(self, *, pos_hz: float, att_hz: float) -> None:
        """Rates on OUR port (USB) only; the radio port keeps the laptop's."""
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
