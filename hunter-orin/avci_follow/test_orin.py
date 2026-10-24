#!/usr/bin/env python3
"""
Offline test for the Orin architecture (Asama 1) — real MAVLink frames, no
hardware.

  1) hub: two vehicles on one link are split by sysid; GCS heartbeats never
     change the chaser's mode; other systems are only counted
  2) PinnedLink: every command goes to the chaser (sysid 1) even though
     pymavlink's target_system locked onto the target (sysid 2)
  3) param read-back through the RX thread (no second reader on the link)
  4) mode set / stream requests addressed to the chaser
  5) relay: target packets forwarded byte-for-byte (MAVLink1 and 2), foreign
     sysids and other types dropped, optional decimation
  6) round trip: target frames -> relay -> (chaser FC routing) -> Orin parser
     -> hub, interleaved with the chaser's own frames and MAVProxy's
  7) orchestrator loop (virtual clock): dry-run, real run pinned to sysid 1
     with WP_SPD confirmation, pilot takeover, stale hold, target missing,
     operator cancel; chase_state CSV written
  8) live pymavlink UDP link to a MAVProxy stand-in, RX thread running: the
     target's heartbeat arrives first (pymavlink locks onto it), commands
     still reach sysid 1, param read-back through the live thread
  9) relay_target.main() with fake radios: refuses equal sysids; forwards the
     target's packets unchanged; its own traffic is only the harmless RTT ping;
     radio RTT logged on the laptop with ACKs meant for the Orin ignored

Run:  python3 test_orin.py
"""
from __future__ import annotations

import builtins
import contextlib
import io
import math
import os
import sys
import tempfile
import threading
import time

sys.path.insert(0, ".")

from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as ml2  # noqa: E402

import chase_orin  # noqa: E402
from chaser_env import ChaserEnv  # noqa: E402
from mav_hub import ChaserHandle, MavHub, PinnedLink  # noqa: E402
from relay_target import TargetForwarder  # noqa: E402
from test_loop import HOME_LAT, HOME_LON, FakeTime, World  # noqa: E402

PASS = FAIL = 0
GUIDED, LOITER = 4, 5


def check(name, cond, detail=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += (not cond)
    print(f"  {'[OK]  ' if cond else '[FAIL]'} {name}{('  — ' + detail) if detail else ''}")


# ---------------------------------------------------------------- frame helpers
def enc(sysid, compid=1):
    return ml2.MAVLink(None, srcSystem=sysid, srcComponent=compid)


def frame(e, msg, v1=False):
    return bytes(msg.pack(e, force_mavlink1=v1))


def parse(buf):
    return ml2.MAVLink(None).parse_buffer(bytes(buf)) or []


def gpi(e, lat, lon, alt_amsl, rel, vn, ve, vd=0.0):
    return e.global_position_int_encode(1000, int(round(lat * 1e7)), int(round(lon * 1e7)),
                                        int(alt_amsl * 1000), int(rel * 1000),
                                        int(vn * 100), int(ve * 100), int(vd * 100), 0)


def heartbeat(e, custom_mode, mav_type=ml2.MAV_TYPE_QUADROTOR,
              autopilot=ml2.MAV_AUTOPILOT_ARDUPILOTMEGA):
    return e.heartbeat_encode(mav_type, autopilot,
                              ml2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | ml2.MAV_MODE_FLAG_SAFETY_ARMED,
                              custom_mode, ml2.MAV_STATE_ACTIVE)


def one(e, msg, v1=False):
    return parse(frame(e, msg, v1))[0]


class FakeConn:
    """Stands in for a mavutil connection: real MAVLink encoder, records bytes,
    applies PARAM_SETs to a fake FC param store."""

    def __init__(self):
        self.written = bytearray()
        self.mav = ml2.MAVLink(self, srcSystem=251, srcComponent=191)
        self.target_system = 2        # pymavlink locked onto the TARGET's heartbeat
        self.target_component = 1
        self.params = {}
        self._p = ml2.MAVLink(None)

    def write(self, buf):
        self.written += bytes(buf)
        for m in self._p.parse_buffer(bytes(buf)) or []:
            if m.get_type() == "PARAM_SET":
                pid = m.param_id.decode() if isinstance(m.param_id, bytes) else m.param_id
                self.params[pid.rstrip("\x00")] = float(m.param_value)

    def recv_match(self, *a, **k):
        raise AssertionError("only the hub RX thread may read the link")

    def sent(self, typ=None):
        msgs = parse(self.written)
        return [m for m in msgs if typ is None or m.get_type() == typ]


# ---------------------------------------------------------------- 1-4 hub
def test_hub():
    print("1) hub: one link, two vehicles, split by sysid")
    conn = FakeConn()
    hub = MavHub(conn, chaser_sysid=1, target_sysid=2, print_statustext=False)
    e1, e2, e255, e191 = enc(1), enc(2), enc(255, 190), enc(1, 191)
    hub.dispatch(one(e1, gpi(e1, HOME_LAT, HOME_LON, 940.0, 40.0, 0.5, 0.0)), 10.0)
    hub.dispatch(one(e2, gpi(e2, 39.87, 32.74, 945.0, 45.0, -15.0, 1.0)), 10.0)
    c, t = hub.chaser.snapshot(), hub.target.snapshot()
    check("chaser position from sysid 1", abs(c["lat"] - HOME_LAT) < 1e-7 and c["alt_rel"] == 40.0)
    check("target position from sysid 2", abs(t["lat"] - 39.87) < 1e-7 and t["vx"] == -15.0)
    hub.dispatch(one(e1, heartbeat(e1, GUIDED)), 10.1)
    hub.dispatch(one(e191, heartbeat(e191, 0, ml2.MAV_TYPE_GCS, ml2.MAV_AUTOPILOT_INVALID)), 10.2)
    hub.dispatch(one(e2, heartbeat(e2, LOITER)), 10.2)
    check("chaser mode from its heartbeat", hub.chaser.mode == "GUIDED", hub.chaser.mode)
    check("GCS-type heartbeat on sysid 1 ignored for mode", hub.chaser.mode == "GUIDED")
    check("target mode kept separate", hub.target.mode == "LOITER", hub.target.mode)
    hub.dispatch(one(e1, e1.attitude_encode(1, 0.1, -0.05, math.radians(90), 0, 0, 0)), 10.3)
    hub.dispatch(one(e2, e2.attitude_encode(1, 0.0, 0.0, math.radians(-45), 0, 0, 0)), 10.3)
    check("chaser yaw captured (90°)", abs(hub.chaser.yaw - math.pi / 2) < 1e-6)
    check("target attitude does not leak into chaser", abs(hub.chaser.yaw - math.pi / 2) < 1e-6)
    hub.dispatch(one(e255, heartbeat(e255, 0, ml2.MAV_TYPE_GCS, ml2.MAV_AUTOPILOT_INVALID)), 10.4)
    check("other systems only counted", hub.others.get(255) == 1, str(dict(hub.others)))
    check("new_pos event raised on target fix", hub.target.new_pos.is_set())

    print("2) PinnedLink: commands pinned to the chaser")
    env = ChaserEnv(hub.chaser_handle(), verbose_speed=False)
    env.send_position_target_global(lat_deg=HOME_LAT, lon_deg=HOME_LON, alt_rel_m=30.0)
    env.set_guided_horizontal_speed_mps(12.0, min_interval_s=0.0, min_delta_mps=0.0)
    pos = conn.sent("SET_POSITION_TARGET_GLOBAL_INT")
    cmd = conn.sent("COMMAND_LONG")
    check("conn.target_system is the TARGET (the pitfall)", conn.target_system == 2)
    check("position target addressed to sysid 1", pos and all(m.target_system == 1 for m in pos),
          str([m.target_system for m in pos]))
    check("DO_CHANGE_SPEED addressed to sysid 1", cmd and all(m.target_system == 1 for m in cmd))
    check("sent from our own sysid 251", all(m.get_srcSystem() == 251 for m in pos + cmd))
    try:
        hub.chaser_handle().conn.recv_match()
        check("PinnedLink refuses to read", False)
    except RuntimeError:
        check("PinnedLink refuses to read", True)

    print("3) param read-back via the RX thread")
    conn.written.clear()

    def fc_reply():
        time.sleep(0.05)
        hub.dispatch(one(e2, e2.param_value_encode(b"WP_SPD", 5.0, 9, 1, 0)), time.monotonic())
        hub.dispatch(one(e1, e1.param_value_encode(b"WP_SPD", 20.0, 9, 1, 0)), time.monotonic())

    threading.Thread(target=fc_reply, daemon=True).start()
    got = env._get_param("WP_SPD", timeout=2.0)
    check("chaser's WP_SPD read (target's reply ignored)", got == 20.0, f"got={got}")
    req = conn.sent("PARAM_REQUEST_READ")
    check("request addressed to sysid 1", req and req[0].target_system == 1)

    print("4) mode + stream requests addressed to the chaser")
    conn.written.clear()

    def fc_mode():
        time.sleep(0.1)
        hub.dispatch(one(e1, heartbeat(e1, LOITER)), time.monotonic())

    threading.Thread(target=fc_mode, daemon=True).start()
    ok = hub.set_chaser_mode("LOITER", timeout=2.0)
    sm = conn.sent("SET_MODE")
    check("set_chaser_mode confirmed", ok)
    check("SET_MODE -> sysid 1, LOITER", sm and sm[0].target_system == 1 and sm[0].custom_mode == LOITER)
    conn.written.clear()
    hub.request_streams(pos_hz=10, att_hz=10)
    iv = [m for m in conn.sent("COMMAND_LONG")
          if m.command == ml2.MAV_CMD_SET_MESSAGE_INTERVAL]
    check("GPI/GPS_RAW/ATTITUDE intervals -> sysid 1",
          sorted(int(m.param1) for m in iv) == [24, 30, 33] and all(m.target_system == 1 for m in iv))
    conn.written.clear()
    hub.send_heartbeat()
    hb = conn.sent("HEARTBEAT")
    check("our heartbeat is GCS type (never mistaken for a vehicle)",
          hb and hb[0].type == ml2.MAV_TYPE_GCS)
    try:
        MavHub(FakeConn(), chaser_sysid=1, target_sysid=1)
        check("equal sysids refused", False)
    except ValueError:
        check("equal sysids refused", True)


# ---------------------------------------------------------------- 5 relay
def test_relay():
    print("5) relay: byte-for-byte forwarding of target packets")
    out = bytearray()
    fwd = TargetForwarder(lambda b: out.extend(b))
    e1, e2 = enc(1), enc(2)
    t_gpi_v2 = frame(e2, gpi(e2, 39.87, 32.74, 945.0, 45.0, -15.0, 1.0))
    check("nothing forwarded before target sysid known", not fwd.on_msg(parse(t_gpi_v2)[0], 0.0))
    fwd.target_sysid = 2
    out.clear()
    check("GPI (MAVLink2) forwarded", fwd.on_msg(parse(t_gpi_v2)[0], 0.0))
    check("  ...byte-identical", bytes(out) == t_gpi_v2)
    out.clear()
    t_gpi_v1 = frame(e2, gpi(e2, 39.87, 32.74, 945.0, 45.0, -15.0, 1.0), v1=True)
    fwd.on_msg(parse(t_gpi_v1)[0], 1.0)
    check("GPI (MAVLink1) forwarded byte-identical", bytes(out) == t_gpi_v1)
    out.clear()
    for m in (heartbeat(e2, LOITER),
              e2.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 14)):
        fwd.on_msg(one(e2, m), 2.0)
    check("HEARTBEAT + GPS_RAW_INT forwarded",
          [m.get_type() for m in parse(out)] == ["HEARTBEAT", "GPS_RAW_INT"])
    out.clear()
    fwd.on_msg(one(e2, e2.statustext_encode(6, b"hello")), 3.0)
    fwd.on_msg(one(e1, gpi(e1, HOME_LAT, HOME_LON, 940.0, 40.0, 0, 0)), 3.0)
    check("other types and foreign sysids dropped", len(out) == 0 and fwd.skipped_foreign == 1)

    dec = TargetForwarder(lambda b: None, pos_max_hz=5.0)
    dec.target_sysid = 2
    n = sum(dec.on_msg(parse(t_gpi_v2)[0], 10.0 + 0.1 * i) for i in range(10))
    check("--fwd-max-hz 5 halves a 10 Hz stream", n == 5 and dec.decimated == 5,
          f"forwarded={n} decimated={dec.decimated}")


# ---------------------------------------------------------------- 6 round trip
def test_round_trip():
    print("6) round trip: target -> relay -> chaser FC -> Orin hub")
    radio = []  # forwarded frames, one per packet (the FC routes whole packets)
    fwd = TargetForwarder(lambda b: radio.append(bytes(b)))
    fwd.target_sysid = 2
    e1, e2, e255 = enc(1), enc(2), enc(255, 190)
    lat_t, lon_t = 39.8659321, 32.7401234
    for i in range(5):  # target stream arriving at the laptop
        fwd.on_msg(parse(frame(e2, gpi(e2, lat_t, lon_t, 947.25, 47.25, -14.5, 2.25)))[0], 0.1 * i)
    fwd.on_msg(parse(frame(e2, heartbeat(e2, LOITER)))[0], 0.5)
    # the chaser FC routes those bytes out of USB, interleaved with its own
    usb = bytearray(frame(e1, heartbeat(e1, GUIDED)))
    own = [frame(e1, gpi(e1, HOME_LAT, HOME_LON, 940.0, 40.0, 0.3, -0.2)),
           frame(e255, heartbeat(e255, 0, ml2.MAV_TYPE_GCS, ml2.MAV_AUTOPILOT_INVALID))]
    for i, f in enumerate(radio):  # interleave at packet boundaries
        usb += f
        if i < len(own):
            usb += own[i]
    hub = MavHub(FakeConn(), chaser_sysid=1, target_sysid=2, print_statustext=False)
    for m in parse(usb):  # what mavutil yields on the Orin
        hub.dispatch(m, 20.0)
    t, c = hub.target.snapshot(), hub.chaser.snapshot()
    check("target lat/lon exact after two hops", abs(t["lat"] - lat_t) < 1.5e-7
          and abs(t["lon"] - lon_t) < 1.5e-7, f"{t['lat']:.7f},{t['lon']:.7f}")
    check("target alt + velocity exact", t["alt_amsl"] == 947.25 and t["vx"] == -14.5
          and t["vy"] == 2.25, f"alt={t['alt_amsl']} v=({t['vx']},{t['vy']})")
    check("target mode arrives too", t["mode"] == "LOITER", t["mode"])
    check("chaser state unpolluted", abs(c["lat"] - HOME_LAT) < 1e-7 and c["mode"] == "GUIDED")


# ---------------------------------------------------------------- 7 orchestrator
class _FakeEvent:
    def __init__(self, clock):
        self.clock = clock

    def wait(self, timeout=None):
        self.clock.sleep(min(timeout or 0.1, 0.1))  # a target fix every 0.1 s
        return True

    def clear(self):
        pass

    def set(self):
        pass


class _FakeState:
    def __init__(self, fn, clock, sysid, params):
        self.fn, self.sysid, self.params = fn, sysid, params
        self.new_pos = _FakeEvent(clock)

    def snapshot(self):
        return self.fn()

    def wait_param(self, name, *, since_mono, timeout):
        return self.params.get(name)


class FakeHub:
    def __init__(self, world, clock, *, target_ok=True):
        self.world = world
        self.conn = FakeConn()
        self.chaser = _FakeState(world.chaser, clock, 1, self.conn.params)
        self.target = _FakeState(world.target, clock, 2, {})
        self.others = {255: 12, 250: 3}
        self.on_msg = None
        self.mode_calls = []
        self.target_ok = target_ok
        self.stopped = False

    def start(self):
        pass

    def stop(self):
        self.stopped = True

    def request_streams(self, **k):
        pass

    def wait_ready(self, timeout):
        return True, self.target_ok

    def set_chaser_mode(self, name, timeout=4.0):
        self.mode_calls.append(name)
        self.world.chaser_mode = name
        return True

    def chaser_handle(self):
        return ChaserHandle(PinnedLink(self.conn, 1, 1), self.chaser)


def run_orin(argv, *, budget_s=10.0, world_kw=None, answer="evet", target_ok=True):
    clock = FakeTime(budget_s)
    world = World(clock, **(world_kw or {}))
    hub = FakeHub(world, clock, target_ok=target_ok)
    a = chase_orin.build_parser().parse_args(argv)
    saved = builtins.input
    builtins.input = lambda *x: answer
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            code = chase_orin.Orchestrator(a, hub, clock=clock).run()
    finally:
        builtins.input = saved
    return code, buf.getvalue(), hub


def test_orchestrator():
    print("7) orchestrator loop (virtual clock)")
    code, txt, hub = run_orin(["--dry-run", "--no-log"])
    check("dry-run: clean exit, no traceback", code == 0 and "Traceback" not in txt)
    check("dry-run: head-on -> intercept", "beh=intercept" in txt)
    check("dry-run: nothing sent, mode untouched", not hub.conn.written and hub.mode_calls == [])
    check("dry-run: heading from yaw", "yon=yaw" in txt)

    logdir = tempfile.mkdtemp()
    code, txt, hub = run_orin(["--set-mode", "--posvel", "--log-dir", logdir])
    pos = hub.conn.sent("SET_POSITION_TARGET_GLOBAL_INT")
    dcs = [m for m in hub.conn.sent("COMMAND_LONG") if m.command == ml2.MAV_CMD_DO_CHANGE_SPEED]
    check("real: GUIDED then LOITER on Ctrl-C", hub.mode_calls == ["GUIDED", "LOITER"],
          str(hub.mode_calls))
    check("real: WP_SPD=20 set and confirmed", hub.conn.params.get("WP_SPD") == 20.0
          and "WP_SPD ceiling confirmed" in txt)
    check("real: carrots sent, all to sysid 1", len(pos) > 20 and all(m.target_system == 1 for m in pos),
          f"n={len(pos)}")
    check("real: DO_CHANGE_SPEED to sysid 1", dcs and all(m.target_system == 1 for m in dcs))
    check("real: posvel ff = target velocity", abs(pos[-1].vx + 15.0) < 1e-6, f"vx={pos[-1].vx}")
    check("real: hub stopped on exit", hub.stopped)
    files = os.listdir(logdir)
    st = [f for f in files if f.startswith("chase_state_")]
    check("chase_state CSV written", len(st) == 1, str(files))
    if st:
        lines = [r for r in open(os.path.join(logdir, st[0])).read().splitlines()
                 if r and not r.startswith("#")]
        hdr = lines[0].split(",")
        row = dict(zip(hdr, lines[-1].split(",")))
        check("  ...one row per tick with chaser attitude + both GPS",
              len(lines) > 20 and row["c_yaw"] != "" and row["t_lat"] != "" and row["heading_src"] == "yaw",
              f"rows={len(lines) - 1} yaw={row.get('c_yaw')}")

    code, txt, hub = run_orin(["--set-mode", "--no-log"], budget_s=12.0,
                              world_kw=dict(pilot_mode_at=8.0))
    check("pilot takeover: exits, mode NOT overridden",
          "Kontrol kumandada" in txt and hub.mode_calls == ["GUIDED"], str(hub.mode_calls))

    code, txt, hub = run_orin(["--set-mode", "--no-log"], budget_s=12.0,
                              world_kw=dict(stop_fix_at=5.0))
    pos = hub.conn.sent("SET_POSITION_TARGET_GLOBAL_INT")
    check("stale target: chaser held in place", "yerinde tutuluyor" in txt
          and pos[-1].lat_int == int(round(HOME_LAT * 1e7))
          and pos[-1].lon_int == int(round(HOME_LON * 1e7)))

    code, txt, hub = run_orin(["--set-mode", "--no-log"], target_ok=False)
    check("target missing: clear diagnosis, nothing commanded",
          code == 2 and "relay_target.py" in txt and hub.mode_calls == [] and not hub.conn.written)

    code, txt, hub = run_orin(["--set-mode", "--no-log"], answer="hayir")
    check("operator cancels: nothing commanded", code == 0 and "Iptal" in txt
          and hub.mode_calls == [] and not hub.conn.written)


# ---------------------------------------------------------------- 8 live UDP
def test_udp_loopback():
    print("8) live UDP link (MAVProxy stand-in), hub RX thread running")
    port = 14700 + os.getpid() % 200
    mp = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=255)
    hub = MavHub.connect(f"udpout:127.0.0.1:{port}", source_system=251,
                         chaser_sysid=1, target_sysid=2, print_statustext=False)
    hub.start(heartbeat_s=0.2)
    try:
        hb = mp.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        check("hub's first heartbeat reaches MAVProxy (it learns our address)",
              hb is not None and hb.get_srcSystem() == 251)
        e1, e2 = enc(1), enc(2)
        mp.write(frame(e2, heartbeat(e2, LOITER)))   # target FIRST: pymavlink locks onto it
        time.sleep(0.05)
        mp.write(frame(e1, heartbeat(e1, GUIDED)))
        for _ in range(5):
            mp.write(frame(e1, gpi(e1, HOME_LAT, HOME_LON, 940.0, 40.0, 0.0, 0.0)))
            mp.write(frame(e2, gpi(e2, 39.87, 32.74, 945.0, 45.0, -15.0, 0.0)))
            time.sleep(0.02)
        c_ok, t_ok = hub.wait_ready(3.0)
        check("both vehicles seen by the live RX thread", c_ok and t_ok)
        check("pymavlink really locked onto the target (pitfall reproduced)",
              hub.conn.target_system == 2, f"target_system={hub.conn.target_system}")
        check("states split correctly over the live link",
              hub.chaser.mode == "GUIDED" and hub.target.mode == "LOITER"
              and abs(hub.target.snapshot()["lat"] - 39.87) < 1e-7)
        env = ChaserEnv(hub.chaser_handle(), verbose_speed=False)
        env.send_position_target_global(lat_deg=HOME_LAT, lon_deg=HOME_LON, alt_rel_m=30.0)
        got = mp.recv_match(type="SET_POSITION_TARGET_GLOBAL_INT", blocking=True, timeout=3)
        check("command over UDP still addressed to sysid 1",
              got is not None and got.target_system == 1,
              f"target_system={getattr(got, 'target_system', None)}")

        def fc_reply():
            time.sleep(0.1)
            mp.write(frame(e1, e1.param_value_encode(b"WP_SPD", 20.0, 9, 1, 0)))

        threading.Thread(target=fc_reply, daemon=True).start()
        v = env._get_param("WP_SPD", timeout=2.0)
        check("param read-back through the live RX thread", v == 20.0, f"v={v}")
    finally:
        hub.stop()
        mp.close()


# ---------------------------------------------------------------- 9 relay main
class _RelayConn:
    """Fake radio port: real MAVLink encoder (relay sends as sysid 250)."""

    def __init__(self):
        self.written = []
        self.mav = ml2.MAVLink(self, srcSystem=250, srcComponent=0)
        self.target_system = 1
        self.target_component = 1

    def write(self, buf):
        self.written.append(bytes(buf))


class _RelayVehicle:
    sysids = {"TAKIPCI": 1, "HEDEF": 2}
    made = {}

    def __init__(self, name, port, baud, stream_hz=10.0, on_msg=None, att_hz=0.0):
        key = name.strip().upper()
        self.name, self.on_msg = name, on_msg
        self.sysid = _RelayVehicle.sysids[key]
        self.conn = _RelayConn()
        _RelayVehicle.made[key] = self

    def _request_streams(self):
        pass

    def snapshot(self):
        return dict(lat=HOME_LAT, lon=HOME_LON, alt_amsl=940.0, alt_rel=40.0, fix=3, nsat=15,
                    hz=10.0, age=0.05, mode="LOITER", vx=0.0, vy=0.0, vz=0.0)


def run_relay(sysids, feed, extra=("--no-log",)):
    import relay_target
    _RelayVehicle.sysids = dict(sysids)
    _RelayVehicle.made = {}

    class Clock(FakeTime):
        def sleep(self, dt):
            feed(_RelayVehicle.made, self)
            super().sleep(dt)

    saved = (relay_target.Vehicle, relay_target.time, sys.argv)
    relay_target.Vehicle, relay_target.time = _RelayVehicle, Clock(3.0)
    sys.argv = ["relay_target.py", "--target-port", "T", "--chaser-port", "C"] + list(extra)
    buf = io.StringIO()
    exit_msg = None
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            relay_target.main()
    except SystemExit as e:
        exit_msg = str(e.code)
    finally:
        relay_target.Vehicle, relay_target.time, sys.argv = saved
    return buf.getvalue(), exit_msg, _RelayVehicle.made


def _split_chaser_writes(written):
    """Forwarded target frames (sysid 2) vs the relay's own frames."""
    fwd, own = [], []
    for w in written:
        m = parse(w)[0]
        (fwd if m.get_srcSystem() == 2 else own).append((w, m))
    return fwd, own


def test_relay_main():
    print("9) relay_target.main(): wiring with fake radios")
    _txt, exit_msg, made = run_relay({"TAKIPCI": 1, "HEDEF": 1}, lambda m, c: None)
    check("equal sysids -> refuses to start, says SYSID_THISMAV=2",
          exit_msg is not None and "SYSID_THISMAV=2" in exit_msg)
    check("  ...and wrote nothing to the chaser radio", not made["TAKIPCI"].conn.written)

    e1, e2 = enc(1), enc(2)
    sent = []
    acked = [0]

    def feed(m, clock):
        if "HEDEF" not in m:
            return
        f_t = frame(e2, gpi(e2, 39.87, 32.74, 945.0, 45.0, -15.0, 0.0))
        f_c = frame(e1, gpi(e1, HOME_LAT, HOME_LON, 940.0, 40.0, 0.0, 0.0))  # crosstalk
        f_s = frame(e2, e2.statustext_encode(6, b"x"))
        for f in (f_t, f_c, f_s):
            m["HEDEF"].on_msg(m["HEDEF"].name, parse(f)[0], 0.0)
        sent.append(f_t)
        # the chaser FC answers each new ping: first an ACK meant for the Orin
        # (sysid 251, must be ignored), then the one addressed to the laptop (250)
        pings = [x for x in _split_chaser_writes(m["TAKIPCI"].conn.written)[1]
                 if x[1].get_type() == "COMMAND_LONG"]
        ch = m["TAKIPCI"]
        while acked[0] < len(pings):
            acked[0] += 1
            cmd = ml2.MAV_CMD_REQUEST_MESSAGE
            for tgt, dt in ((251, 0.01), (250, 0.08)):
                ack = one(e1, e1.command_ack_encode(cmd, 0, 0, 0, tgt, 0))
                ch.on_msg(ch.name, ack, clock.monotonic() + dt)

    logdir = tempfile.mkdtemp()
    txt, exit_msg, made = run_relay({"TAKIPCI": 1, "HEDEF": 2}, feed,
                                    extra=("--log-dir", logdir))
    fwd, own = _split_chaser_writes(made["TAKIPCI"].conn.written)
    check("clean exit on Ctrl-C", exit_msg is None and "Durduruldu" in txt)
    check("chaser radio got exactly the target's position packets",
          [w for w, _m in fwd] == sent and len(sent) > 5, f"fwd={len(fwd)} sent={len(sent)}")
    only_ping = all(m.get_type() == "COMMAND_LONG" and m.command == ml2.MAV_CMD_REQUEST_MESSAGE
                    and int(m.param1) == ml2.MAVLINK_MSG_ID_SYSTEM_TIME
                    and m.target_system == 1 and m.get_srcSystem() == 250 for _w, m in own)
    check("relay's own traffic = only the harmless REQUEST_MESSAGE ping, to sysid 1",
          only_ping and len(own) >= 2, f"own={[m.get_type() for _w, m in own]}")
    files = os.listdir(logdir)
    cl = [f for f in files if f.startswith("cmd_lat_")]
    rtts = []
    if cl:
        for r in open(os.path.join(logdir, cl[0])).read().splitlines():
            if r and not r.startswith("#") and ",rtt," in r:
                rtts.append(float([x for x in r.split(",") if x][-1]))
    check("cmd_lat CSV: radio RTT logged on the laptop, Orin's ACK ignored",
          len(rtts) >= 2 and all(abs(x - 80.0) < 1.0 for x in rtts), f"rtt_ms={rtts}")
    check("status line shows live RTT", "rtt   80 ms" in txt)

    _txt, _e, made = run_relay({"TAKIPCI": 1, "HEDEF": 2}, lambda m, c: None,
                               extra=("--log-dir", tempfile.mkdtemp(), "--rtt-ping-s", "0"))
    check("--rtt-ping-s 0 -> relay writes nothing of its own",
          not made["TAKIPCI"].conn.written)


def main():
    print("=" * 68)
    print("Orin architecture test (Asama 1) — real MAVLink frames, no hardware")
    print("=" * 68)
    test_hub()
    test_relay()
    test_round_trip()
    test_orchestrator()
    test_udp_loopback()
    test_relay_main()
    print("-" * 68)
    print(f"PASS={PASS}  FAIL={FAIL}")
    print("-" * 68)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
