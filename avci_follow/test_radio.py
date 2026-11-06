#!/usr/bin/env python3
"""
Offline test for the RADIO layout (3DR on the chaser's Orin) — real MAVLink
frames, no hardware.

  1) hub, radio mode: target read ONLY from the radio link, chaser ONLY from
     the FC link, even with the same sysid (Talon = 1 = chaser); target sysid
     auto-locks on the first vehicle heartbeat (GCS heartbeats never lock);
     crosstalk sysids ignored; legacy mode still refuses equal sysids
  2) TIMESYNC: requests answered on the radio (ts1 echoed, from 251), replies
     never answered
  3) downlink: chaser HEARTBEAT / GPI / GPS_RAW / STATUSTEXT to the ground,
     byte-identical, rate-limited; ATTITUDE, gimbal heartbeats and the target
     never sent; --downlink-hz 0 -> radio silent
  4) events: 50-char STATUSTEXT, rate limit, force; nothing in legacy mode
  5) rx_forever: a dying serial link is re-opened (reset) and reading resumes
  6) relay_radio.RadioRelay: PASSIVE (0 bytes to the source); target lock and
     byte-for-byte forwarding of the three types only; --request-hz sends only
     stream requests; TIMESYNC RTT matched / stale ignored / lost counted;
     chaser status + Orin events from the downlink
  7) chase_orin orchestrator in radio mode (virtual clock): banner, events once
     per state change, stale hold, target-missing diagnosis, pilot takeover
  8) signals: SIGHUP/SIGTERM mid-chase -> LOITER hand-back (stdout detached on
     SIGHUP), second signal ignored, nohup's SIG_IGN respected
  9) LIVE end to end: fake Talon (sysid 1) --UDP--> relay_radio --virtual 3DR
     pair (ptys)--> MavHub <--UDP--> fake FC (sysid 1); downlink + RTT back on
     the laptop; commands reach only the FC; radio drop -> target stale ->
     resumes; the Talon receives nothing

Run:  python3 test_radio.py
"""
from __future__ import annotations

import builtins
import contextlib
import io
import os
import select
import signal
import sys
import threading
import time

sys.path.insert(0, ".")

from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as ml2  # noqa: E402

import chase_orin  # noqa: E402
from chaser_env import ChaserEnv  # noqa: E402
from mav_hub import MavHub, rx_forever  # noqa: E402
from relay_radio import RadioRelay  # noqa: E402
from test_loop import HOME_LAT, HOME_LON, FakeTime, World  # noqa: E402
from test_orin import (  # noqa: E402
    FakeConn, FakeHub, PilotWorld, enc, frame, gpi, heartbeat, one, parse)

PASS = FAIL = 0
GUIDED, LOITER = 4, 5
TALON = (39.9012345, 32.7654321, 1012.5, 112.5, -18.25, 3.5)   # lat lon amsl rel vn ve


def check(name, cond, detail=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += (not cond)
    print(f"  {'[OK]  ' if cond else '[FAIL]'} {name}{('  — ' + detail) if detail else ''}")


def gcs_hb(e):
    return heartbeat(e, 0, ml2.MAV_TYPE_GCS, ml2.MAV_AUTOPILOT_INVALID)


def plane_hb(e, mode=10):
    return heartbeat(e, mode, ml2.MAV_TYPE_FIXED_WING)


def radio_hub(downlink_hz=2.0, target_sysid=0):
    fc, rad = FakeConn(), FakeConn()
    hub = MavHub(fc, chaser_sysid=1, target_sysid=target_sysid, target_conn=rad,
                 print_statustext=False, downlink_hz=downlink_hz)
    return hub, fc, rad


# ---------------------------------------------------------------- 1 split by link
def test_split():
    print("1) hub radio mode: one vehicle per link, same sysid allowed")
    hub, _fc, _rad = radio_hub()
    eT, eC, eV, e7 = enc(1), enc(1), enc(250, 191), enc(7)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        hub.dispatch_radio(one(eT, gpi(eT, *TALON)), 1.0)
        check("GPI before any vehicle heartbeat ignored",
              hub.target.pos_mono is None and not hub.target_locked)
        hub.dispatch_radio(one(eV, gcs_hb(eV)), 1.1)
        check("Victus GCS heartbeat does not lock", not hub.target_locked)
        hub.dispatch_radio(one(eT, plane_hb(eT)), 1.2)
        check("Talon heartbeat locks the target (sysid 1 = chaser's)",
              hub.target_locked and hub.target.sysid == 1)
        hub.dispatch_radio(one(eT, gpi(eT, *TALON)), 1.3)
        hub.dispatch(one(eC, gpi(eC, HOME_LAT, HOME_LON, 940.0, 40.0, 0.3, -0.2)), 1.3)
        hub.dispatch(one(eC, heartbeat(eC, GUIDED)), 1.3)
        hub.dispatch_radio(one(e7, gpi(e7, 41.0, 29.0, 100.0, 10.0, 0, 0)), 1.4)
        hub.dispatch(one(enc(2), gpi(enc(2), 41.0, 29.0, 100.0, 10.0, 0, 0)), 1.4)
    t, c = hub.target.snapshot(), hub.chaser.snapshot()
    check("target = Talon, exact", abs(t["lat"] - TALON[0]) < 1e-7 and t["vx"] == -18.25
          and t["alt_amsl"] == 1012.5, f"{t['lat']:.7f} vx={t['vx']}")
    check("chaser = FC link only, exact", abs(c["lat"] - HOME_LAT) < 1e-7 and c["mode"] == "GUIDED")
    check("target mode from the plane heartbeat, kept apart",
          hub.target.mode != "?" and hub.target.mode != hub.chaser.mode, hub.target.mode)
    check("crosstalk sysid 7 on the radio ignored", hub.radio_others.get(7) == 1
          and abs(t["lat"] - TALON[0]) < 1e-7)
    check("FC-link sysid 2 never becomes the target", hub.others.get(2) == 1)
    check("lock announced", "hedef sysid=1 kilitlendi" in buf.getvalue())
    hub2, _f, _r = radio_hub(target_sysid=9)
    hub2.dispatch_radio(one(eT, plane_hb(eT)), 1.0)
    check("--target-sysid 9: sysid 1 plane not accepted", hub2.target.sysid == 9
          and hub2.radio_others.get(1) == 1)
    try:
        MavHub(FakeConn(), chaser_sysid=1, target_sysid=1)
        check("legacy mode still refuses equal sysids", False)
    except ValueError:
        check("legacy mode still refuses equal sysids", True)


# ---------------------------------------------------------------- 2 timesync
def test_timesync():
    print("2) TIMESYNC on the radio")
    hub, _fc, rad = radio_hub()
    eV = enc(250, 191)
    hub.dispatch_radio(one(eV, eV.timesync_encode(0, 123456789)), 2.0)
    rep = rad.sent("TIMESYNC")
    check("request answered once", len(rep) == 1)
    check("  ...ts1 echoed, tc1 set, from sysid 251",
          rep and rep[0].ts1 == 123456789 and rep[0].tc1 > 0 and rep[0].get_srcSystem() == 251)
    rad.written.clear()
    hub.dispatch_radio(one(eV, eV.timesync_encode(5, 42)), 2.1)
    check("a reply (tc1 != 0) is not answered", not rad.written)


# ---------------------------------------------------------------- 3 downlink
def _feed_fc(hub, t0=10.0, secs=3.0, eC=None):
    eC = eC or enc(1)
    frames = {}
    n = int(round(secs / 0.1))
    for i in range(n):
        t = t0 + 0.1 * i
        f = frame(eC, gpi(eC, HOME_LAT, HOME_LON, 940.0, 40.0, 0.3, -0.2))
        frames.setdefault("gpi", f)
        hub.dispatch(parse(f)[0], t)
        hub.dispatch(one(eC, eC.attitude_encode(1, 0, 0, 1.0, 0, 0, 0)), t)
        if i % 10 == 0:
            hub.dispatch(one(eC, heartbeat(eC, GUIDED)), t)
            hub.dispatch(one(eC, eC.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 15)), t)
            hub.dispatch(one(enc(1, 154), heartbeat(enc(1, 154), 0, ml2.MAV_TYPE_GIMBAL)), t)
    return frames


def test_downlink():
    print("3) downlink: chaser status to the ground")
    hub, _fc, rad = radio_hub(downlink_hz=2.0)
    eT = enc(1)
    hub.dispatch_radio(one(eT, plane_hb(eT)), 9.0)
    hub.dispatch_radio(one(eT, gpi(eT, *TALON)), 9.0)
    rad.written.clear()
    eC = enc(1)
    frames = _feed_fc(hub, eC=eC)
    hub.dispatch(one(eC, eC.statustext_encode(4, b"EKF3 IMU0 is using GPS")), 13.0)
    sent = parse(rad.written)
    types = [m.get_type() for m in sent]
    n_gpi, n_hb, n_raw = types.count("GLOBAL_POSITION_INT"), types.count("HEARTBEAT"), \
        types.count("GPS_RAW_INT")
    check("GPI at ~2 Hz (3 s of a 10 Hz stream -> 6)", n_gpi == 6, f"n={n_gpi}")
    check("HEARTBEAT 1 Hz, vehicle only (gimbal dropped)", n_hb == 3
          and all(m.type == ml2.MAV_TYPE_QUADROTOR for m in sent if m.get_type() == "HEARTBEAT"),
          f"n={n_hb}")
    check("GPS_RAW_INT every ~2 s", n_raw == 2, f"n={n_raw}")
    check("ATTITUDE never sent", "ATTITUDE" not in types)
    check("FC STATUSTEXT forwarded", "STATUSTEXT" in types)
    check("all from the chaser FC (sysid 1)", all(m.get_srcSystem() == 1 for m in sent))
    first = next(m for m in sent if m.get_type() == "GLOBAL_POSITION_INT")
    check("byte-identical to the FC's frame", bytes(first.get_msgbuf()) == frames["gpi"])
    check("the target is never echoed down",
          all(abs(m.lat / 1e7 - HOME_LAT) < 1e-6 for m in sent if m.get_type() == "GLOBAL_POSITION_INT"))
    hub0, _f0, rad0 = radio_hub(downlink_hz=0.0)
    _feed_fc(hub0)
    check("--downlink-hz 0 -> nothing on the radio", not rad0.written)


# ---------------------------------------------------------------- 4 events
def test_events():
    print("4) chase_orin events")
    hub, _fc, rad = radio_hub()
    check("first event sent", hub.send_event("x" * 80, now=20.0))
    st = rad.sent("STATUSTEXT")
    txt = st[0].text if st else ""
    check("  ...truncated to 50 chars, from 251", len(txt.rstrip("\x00")) == 50
          and st[0].get_srcSystem() == 251)
    check("second within 0.3 s dropped", not hub.send_event("y", now=20.1) and hub.events_dropped == 1)
    check("forced event always sent", hub.send_event("cikis", now=20.15, force=True))
    check("after 0.3 s sent again", hub.send_event("z", now=20.5))
    leg = MavHub(FakeConn(), chaser_sysid=1, target_sysid=2)
    check("legacy hub: send_event is a no-op", not leg.send_event("x", force=True)
          and not leg.conn.written)
    hub0, _f, rad0 = radio_hub(downlink_hz=0.0)
    check("--downlink-hz 0: no events", not hub0.send_event("x", force=True) and not rad0.written)


# ---------------------------------------------------------------- 5 reconnect
class _FlakyConn:
    """Serves msgs[:k], then raises (unplugged) until reset() succeeds on its
    2nd call, then serves the rest."""

    def __init__(self, msgs, k):
        self.msgs, self.k = list(msgs), k
        self.i = 0
        self.resets = 0
        self.alive = True

    def recv_match(self, **kw):
        if self.i == self.k and self.alive and self.resets == 0:
            self.alive = False
        if not self.alive:
            raise OSError("device reports readiness to read but returned no data")
        if self.i < len(self.msgs):
            self.i += 1
            return self.msgs[self.i - 1]
        time.sleep(0.005)
        return None

    def reset(self):
        self.resets += 1
        if self.resets >= 2:
            self.alive = True
            return True
        return False


def test_reconnect():
    print("5) rx_forever: a dying serial link is re-opened")
    eT = enc(1)
    msgs = [one(eT, gpi(eT, *TALON)) for _ in range(6)]
    got, logs = [], []
    conn = _FlakyConn(msgs, 2)
    state = {"run": True}

    def handle(m, t):
        got.append(m)
        if len(got) == len(msgs):
            state["run"] = False

    th = threading.Thread(target=rx_forever, args=(conn, handle, lambda: state["run"]),
                          kwargs=dict(name="RADYO", reopen=True, retry_s=0.01,
                                      write_lock=threading.Lock(), log=logs.append), daemon=True)
    th.start()
    th.join(3.0)
    check("all messages delivered across the drop", len(got) == len(msgs), f"got={len(got)}")
    check("reset retried until the device came back", conn.resets == 2, f"resets={conn.resets}")
    check("one error line, one reopened line",
          sum("link hatasi" in x for x in logs) == 1 and sum("yeniden acildi" in x for x in logs) == 1,
          str(logs))
    conn2 = _FlakyConn(msgs, 1)
    state2 = {"run": True}
    th2 = threading.Thread(target=rx_forever, args=(conn2, lambda m, t: None, lambda: state2["run"]),
                           kwargs=dict(name="HUB", reopen=False, retry_s=0.01, log=lambda s: None),
                           daemon=True)
    th2.start()
    time.sleep(0.1)
    state2["run"] = False
    th2.join(1.0)
    check("reopen=False (FC/UDP link): reset never called", conn2.resets == 0)


# ---------------------------------------------------------------- 6 relay unit
def test_relay_unit():
    print("6) relay_radio.RadioRelay")
    out = []
    tconn, rconn = FakeConn(), FakeConn()
    relay = RadioRelay(tconn, rconn, out=out.append)
    eT, eMP, e3 = enc(1), enc(255, 190), enc(3)
    relay.on_target(one(eT, gpi(eT, *TALON)), 0.9)
    relay.on_target(one(eMP, gcs_hb(eMP)), 1.0)
    check("MP's GCS heartbeat in the stream does not lock", not relay.target_locked)
    check("packets before the lock counted apart, not forwarded",
          relay.prelock == 2 and not rconn.written)
    f_hb = frame(eT, plane_hb(eT))
    f_gpi = frame(eT, gpi(eT, *TALON))
    f_raw = frame(eT, eT.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 14))
    for f in (f_hb, f_gpi, f_raw):
        relay.on_target(parse(f)[0], 1.1)
    relay.on_target(one(eT, eT.attitude_encode(1, 0, 0, 0, 0, 0, 0)), 1.2)
    relay.on_target(one(e3, gpi(e3, 41.0, 29.0, 100.0, 10.0, 0, 0)), 1.2)
    check("Talon (sysid 1) locked", relay.target_locked and relay.target.sysid == 1)
    check("radio got exactly HB + GPI + GPS_RAW, byte-identical",
          bytes(rconn.written) == f_hb + f_gpi + f_raw)
    check("other vehicle (sysid 3) not forwarded", relay.target_others.get(3) == 1)
    check("PASSIVE: zero bytes to the source", not tconn.written and relay.target_writes == 0)
    check("passive: request_target_streams refuses", relay.request_target_streams() is False
          and not tconn.written)

    t2, r2 = FakeConn(), FakeConn()
    rq = RadioRelay(t2, r2, request_hz=10.0, out=lambda s: None)
    rq.on_target(parse(f_hb)[0], 1.0)
    rq.request_target_streams()
    sent = t2.sent()
    ok = (all(m.get_type() in ("COMMAND_LONG", "REQUEST_DATA_STREAM") for m in sent)
          and all(m.target_system == 1 for m in sent)
          and sorted(int(m.param1) for m in sent if m.get_type() == "COMMAND_LONG") == [24, 33])
    check("--request-hz: only interval/stream requests, to the target", ok and len(sent) == 3,
          str([m.get_type() for m in sent]))

    print("   TIMESYNC RTT")
    rconn.written.clear()
    relay.ping(5.0)
    ts = rconn.sent("TIMESYNC")
    check("ping = TIMESYNC request (tc1=0)", len(ts) == 1 and ts[0].tc1 == 0)
    rconn.written.clear()
    relay.ping(5.2)
    check("no second ping while one is in flight", not rconn.written)
    e251 = enc(251, 191)
    relay.on_radio(one(e251, e251.timesync_encode(77, ts[0].ts1 + 1)), 5.3)
    check("reply with a foreign ts1 ignored", relay.rtt_n == 0)
    relay.on_radio(one(e251, e251.timesync_encode(77, ts[0].ts1)), 5.3)
    check("matching reply -> one RTT sample", relay.rtt_n == 1 and relay.rtt_ms[-1] >= 0)
    relay._ts_out = time.monotonic_ns() - int(3e9)
    rconn.written.clear()
    relay.ping(9.0)
    check("unanswered > 2 s -> lost, new ping sent", relay.rtt_lost == 1
          and len(rconn.sent("TIMESYNC")) == 1)

    print("   downlink view")
    eC = enc(1)
    relay.on_radio(one(eC, heartbeat(eC, GUIDED)), 6.0)
    relay.on_radio(one(eC, gpi(eC, HOME_LAT, HOME_LON, 1009.0, 40.0, 0.0, 0.0)), 6.0)
    relay.on_radio(one(eC, eC.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 16)), 6.0)
    relay.on_radio(one(e251, e251.statustext_encode(6, b"intercept d=412m")), 6.1)
    relay.on_radio(one(eC, eC.statustext_encode(4, b"EKF3 IMU0 is using GPS")), 6.1)
    line = relay.status_line(10.0)
    check("chaser locked from the downlink", relay.chaser_locked and relay.chaser.sysid == 1)
    check("status line: chaser mode, distance, separation",
          "mod=GUIDED" in line and "d=" in line and "ayrim= +3.5 m" in line, line)
    check("Orin event printed", any("[ORIN] intercept d=412m" in s for s in out), str(out[-3:]))
    check("chaser FC text labelled as FC", any("[TAKIPCI FC] EKF3" in s for s in out))


# ---------------------------------------------------------------- 7 orchestrator
class RadioFakeHub(FakeHub):
    def __init__(self, world, clock, *, target_ok=True, radio_rx=0):
        super().__init__(world, clock, target_ok=target_ok)
        self.events = []
        self.radio_others = {250: 12}
        self.radio_rx = radio_rx

    def send_event(self, text, *, severity=6, force=False):
        self.events.append((text, severity, force))
        return True


def run_radio(argv, *, budget_s=10.0, world_kw=None, answer="evet", target_ok=True,
              clock=None, radio_rx=0, world_cls=World):
    clock = clock or FakeTime(budget_s)
    world = world_cls(clock, **(world_kw or {}))
    hub = RadioFakeHub(world, clock, target_ok=target_ok, radio_rx=radio_rx)
    if hasattr(world, "attach"):
        world.attach(hub)
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


RADIO = ["--target-link", "/dev/serial/by-id/usb-fake-if00-port0"]


def test_orchestrator():
    print("7) chase_orin in radio mode (virtual clock)")
    chase_orin._SIGNAL["name"] = None
    a = chase_orin.resolve_args(chase_orin.build_parser().parse_args(RADIO))
    b = chase_orin.resolve_args(chase_orin.build_parser().parse_args([]))
    c = chase_orin.resolve_args(chase_orin.build_parser().parse_args(RADIO + ["--target-sysid", "5"]))
    check("target sysid: radio auto(0) / legacy 2 / explicit kept",
          a.target_sysid == 0 and b.target_sysid == 2 and c.target_sysid == 5)

    code, txt, hub = run_radio(RADIO + ["--dry-run", "--no-log"])
    ev = [e[0] for e in hub.events]
    check("dry-run: clean exit, intercept", code == 0 and "beh=intercept" in txt
          and "Traceback" not in txt)
    check("banner names the radio + auto sysid", "hedef  : radyo /dev/serial/by-id/usb-fake" in txt
          and "oto (radyodaki ilk arac)" in txt)
    check("events: start, behaviour, exit", ev[0].startswith("chase_orin DRY-RUN")
          and any(e.startswith("intercept d=") for e in ev) and ev[-1] == "chase_orin cikis (Ctrl-C)",
          str(ev))
    check("one event per state change (not per tick)", len(ev) <= 6, f"n={len(ev)}")
    check("dry-run: nothing sent to the FC", not hub.conn.written and hub.mode_calls == [])

    code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log"])
    ev = hub.events
    check("real: GUIDED then LOITER", hub.mode_calls == ["GUIDED", "LOITER"], str(hub.mode_calls))
    check("real: 'takip basladi' event", any("GUIDED: takip basladi" in e[0] for e in ev))
    check("real: forced exit event says LOITER, warning severity",
          ev[-1][0] == "chase_orin cikis (Ctrl-C) -> LOITER" and ev[-1][1] == 4 and ev[-1][2])
    pos = hub.conn.sent("SET_POSITION_TARGET_GLOBAL_INT")
    check("real: commands to sysid 1 on the FC link", len(pos) > 20
          and all(m.target_system == 1 for m in pos))

    code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log"], budget_s=12.0,
                               world_kw=dict(stop_fix_at=5.0))
    bek = [e for e in hub.events if e[0].startswith("BEKLE")]
    check("stale target: held, ONE 'BEKLE' event (severity 4)",
          "yerinde tutuluyor" in txt and len(bek) == 1 and bek[0][1] == 4
          and "bayat" in bek[0][0], str(bek))

    code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log", "--exit-on-pilot"], budget_s=12.0,
                               world_kw=dict(pilot_mode_at=8.0))
    check("--exit-on-pilot (old behaviour): mode untouched, event says pilot",
          hub.mode_calls == ["GUIDED"] and hub.events[-1][0] == "chase_orin cikis: pilot (ALT_HOLD)",
          str(hub.events[-1]))

    code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log"], budget_s=16.0,
                               world_cls=PilotWorld,
                               world_kw=dict(schedule=[(0.0, "GUIDED"), (4.0, "LOITER"),
                                                       (8.0, "GUIDED")]))
    ev = hub.events
    pa = [i for i, e in enumerate(ev) if e[0].startswith("chase_orin DURAKLADI")]
    rs = [i for i, e in enumerate(ev) if e[0].startswith("chase_orin GUIDED: takip devam")]
    check("pilot LOITER -> GUIDED: ONE 'DURAKLADI' (warning, forced) and ONE 'takip devam' event",
          len(pa) == 1 and ev[pa[0]] == ("chase_orin DURAKLADI: pilot (LOITER)", 4, True)
          and len(rs) == 1 and ev[rs[0]] == ("chase_orin GUIDED: takip devam (#1)", 6, True),
          str([ev[i] for i in pa + rs]))
    check("  ...nothing else reported while paused (no BEKLE / behaviour spam)",
          len(pa) == 1 and len(rs) == 1 and rs[0] == pa[0] + 1, f"pause@{pa} resume@{rs}")
    check("  ...first tick after the resume announces its behaviour again",
          rs and rs[0] + 1 < len(ev) and " d=" in ev[rs[0] + 1][0]
          and not ev[rs[0] + 1][0].startswith("BEKLE"),
          str(ev[rs[0] + 1]) if rs and rs[0] + 1 < len(ev) else "no event after the resume")
    check("  ...mode never overridden; Ctrl-C afterwards still hands back LOITER",
          hub.mode_calls == ["GUIDED", "LOITER"] and ev[-1][0] == "chase_orin cikis (Ctrl-C) -> LOITER",
          f"{hub.mode_calls} / {ev[-1][0]}")

    class _InterruptMidSwitch(RadioFakeHub):
        def set_chaser_mode(self, name, timeout=4.0):
            super().set_chaser_mode(name, timeout)
            if name == "GUIDED":
                raise KeyboardInterrupt  # SIGTERM lands while the switch is confirming

    class _GuidedRefused(RadioFakeHub):
        def set_chaser_mode(self, name, timeout=4.0):
            self.mode_calls.append(name)
            return False                  # FC stays in LOITER

    for cls, want, label in ((_InterruptMidSwitch, ["GUIDED", "LOITER"],
                              "signal mid GUIDED switch -> still handed back to LOITER"),
                             (_GuidedRefused, ["GUIDED"],
                              "GUIDED refused -> no LOITER sent, mode left alone")):
        saved_cls = globals()["RadioFakeHub"]
        globals()["RadioFakeHub"] = cls
        try:
            code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log"])
        finally:
            globals()["RadioFakeHub"] = saved_cls
        check(label, hub.mode_calls == want, f"{hub.mode_calls} / {hub.events[-1][0]}")

    code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log"], target_ok=False)
    check("target missing: radio diagnosis (relay_radio.py, no packets)",
          code == 2 and "relay_radio.py" in txt and "radyodan HIC paket gelmedi" in txt
          and "250" in txt and hub.mode_calls == [] and not hub.conn.written)
    code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log"], target_ok=False, radio_rx=40)
    check("  ...packets but no target -> says so", "radyodan 40 paket geldi" in txt)


# ---------------------------------------------------------------- 8 signals
class _SigClock(FakeTime):
    def __init__(self, budget_s, at, sig, times=1):
        super().__init__(budget_s)
        self.at, self.sig, self.left = at, sig, times

    def sleep(self, dt):
        if self.left and self.t - 1000.0 > self.at:
            self.left -= 1
            os.kill(os.getpid(), self.sig)
        super().sleep(dt)


def test_signals():
    print("8) signals: SIGHUP / SIGTERM mid-chase")
    saved = {s: signal.getsignal(s) for s in (signal.SIGHUP, signal.SIGTERM)}
    real_out, real_err = sys.stdout, sys.stderr
    try:
        for sig, name in ((signal.SIGHUP, "SIGHUP"), (signal.SIGTERM, "SIGTERM")):
            chase_orin._SIGNAL["name"] = None
            signal.signal(signal.SIGHUP, signal.SIG_DFL)
            chase_orin.install_signal_handlers()
            clock = _SigClock(30.0, 5.0, sig, times=2)   # second signal must be ignored
            code, txt, hub = run_radio(RADIO + ["--set-mode", "--no-log"], clock=clock)
            sys.stdout, sys.stderr = real_out, real_err
            check(f"{name}: GUIDED -> LOITER hand-back", hub.mode_calls == ["GUIDED", "LOITER"],
                  str(hub.mode_calls))
            check(f"{name}: exit event names the signal",
                  hub.events[-1][0] == f"chase_orin cikis ({name}) -> LOITER", hub.events[-1][0])
            check(f"{name}: stopped near t=5 s, not at the 30 s budget", clock.t - 1000.0 < 7.0,
                  f"t={clock.t - 1000.0:.1f}")
            if sig == signal.SIGHUP:
                check("SIGHUP: output after it went to /dev/null, not the dead terminal",
                      "Durduruldu" not in txt and "d=" in txt)
        chase_orin._SIGNAL["name"] = None
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        chase_orin.install_signal_handlers()
        check("nohup's SIG_IGN for SIGHUP respected", signal.getsignal(signal.SIGHUP) == signal.SIG_IGN)
        check("SIGTERM still handled", callable(signal.getsignal(signal.SIGTERM)))
    finally:
        sys.stdout, sys.stderr = real_out, real_err
        for s, h in saved.items():
            signal.signal(s, h)
        chase_orin._SIGNAL["name"] = None


# ---------------------------------------------------------------- 9 live e2e
class PtyPair:
    """Two ptys bridged by a thread = a virtual 3DR pair. drop=True loses bytes."""

    def __init__(self):
        import pty
        import tty
        self.m1, self.s1 = pty.openpty()
        self.m2, self.s2 = pty.openpty()
        for s in (self.s1, self.s2):
            tty.setraw(s)
        self.a, self.b = os.ttyname(self.s1), os.ttyname(self.s2)
        self.drop = False
        self.run = True
        self.bytes = [0, 0]   # a->b, b->a
        threading.Thread(target=self._bridge, daemon=True).start()

    def _bridge(self):
        while self.run:
            r, _w, _x = select.select([self.m1, self.m2], [], [], 0.05)
            for fd in r:
                try:
                    d = os.read(fd, 4096)
                except OSError:
                    return
                if self.drop:
                    continue
                if fd == self.m1:
                    os.write(self.m2, d)
                    self.bytes[0] += len(d)
                else:
                    os.write(self.m1, d)
                    self.bytes[1] += len(d)

    def close(self):
        self.run = False


def test_live_e2e():
    print("9) LIVE: fake Talon -> relay_radio -> virtual 3DR -> MavHub <-> fake FC")
    base = 15000 + os.getpid() % 2000
    fc_port, src_port = base, base + 1
    pair = PtyPair()
    fake_fc = mavutil.mavlink_connection(f"udpin:127.0.0.1:{fc_port}", source_system=1)
    hub = MavHub.connect(f"udpout:127.0.0.1:{fc_port}", source_system=251, chaser_sysid=1,
                         target_sysid=0, target_device=pair.b, target_baud=57600,
                         downlink_hz=5.0, print_statustext=False)
    tconn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{src_port}", source_system=250)
    rconn = mavutil.mavlink_connection(pair.a, baud=57600, source_system=250)
    out = []
    relay = RadioRelay(tconn, rconn, out=out.append)
    radio_types = []
    orig_on_radio = relay.on_radio

    def on_radio(m, t):
        radio_types.append(m.get_type())
        orig_on_radio(m, t)

    state = {"run": True}
    running = lambda: state["run"]  # noqa: E731
    for conn, fn in ((tconn, relay.on_target), (rconn, on_radio)):
        threading.Thread(target=rx_forever, args=(conn, fn, running),
                         kwargs=dict(name="T", reopen=False, log=lambda s: None),
                         daemon=True).start()
    talon = mavutil.mavlink_connection(f"udpout:127.0.0.1:{src_port}", source_system=1)
    fc_got = []
    try:
        hub.start(heartbeat_s=0.2)
        assert fake_fc.recv_match(type="HEARTBEAT", blocking=True, timeout=3) is not None
        eT, eC = enc(1), enc(1)

        def tick(i, send_talon=True):
            if send_talon:
                if i % 10 == 0:
                    talon.write(frame(eT, plane_hb(eT)))
                    talon.write(frame(eT, eT.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 14)))
                talon.write(frame(eT, gpi(eT, *TALON)))
            if i % 10 == 0:
                fake_fc.write(frame(eC, heartbeat(eC, GUIDED)))
                fake_fc.write(frame(eC, eC.gps_raw_int_encode(0, 3, 0, 0, 0, 0, 0, 0, 0, 16)))
            fake_fc.write(frame(eC, gpi(eC, HOME_LAT, HOME_LON, 1009.0, 40.0, 0.3, -0.2)))
            if i % 3 == 0:
                relay.ping(time.monotonic())
            while True:
                m = fake_fc.recv_match(blocking=False)
                if m is None:
                    break
                fc_got.append(m)
            time.sleep(0.05)

        for i in range(40):
            tick(i)
        t, c = hub.target.snapshot(), hub.chaser.snapshot()
        check("target locked on the Talon (sysid 1, same as chaser)",
              hub.target_locked and hub.target.sysid == 1)
        check("target exact over UDP + virtual 3DR", t["lat"] is not None
              and abs(t["lat"] - TALON[0]) < 1.5e-7 and t["vx"] == -18.25 and t["fix"] == 3,
              f"{t['lat']}")
        check("chaser exact from the FC link, unmixed",
              c["lat"] is not None and abs(c["lat"] - HOME_LAT) < 1e-7 and c["mode"] == "GUIDED")
        rc = relay.chaser.snapshot()
        check("laptop sees the chaser via the downlink",
              relay.chaser_locked and rc["mode"] == "GUIDED" and rc["lat"] is not None
              and abs(rc["lat"] - HOME_LAT) < 1e-7)
        check("radio RTT measured on the laptop", relay.rtt_n >= 3
              and max(relay.rtt_ms) < 500, f"n={relay.rtt_n} max={max(relay.rtt_ms or [0]):.1f}ms")
        line = relay.status_line(10.0)
        check("status line shows distance + separation", "d=" in line and "ayrim=" in line, line)

        env = ChaserEnv(hub.chaser_handle(), verbose_speed=False)
        env.send_position_target_global(lat_deg=HOME_LAT, lon_deg=HOME_LON, alt_rel_m=30.0)
        hub.send_event("intercept d=412m", force=True)
        for i in range(40, 50):
            tick(i)
        sp = [m for m in fc_got if m.get_type() == "SET_POSITION_TARGET_GLOBAL_INT"]
        check("command reached the FC, addressed to sysid 1", sp and sp[0].target_system == 1)
        check("FC got only the Orin's own packets (no radio traffic)",
              fc_got and all(m.get_srcSystem() == 251 for m in fc_got),
              str(sorted({m.get_srcSystem() for m in fc_got})))
        check("radio never carried a command",
              "SET_POSITION_TARGET_GLOBAL_INT" not in radio_types and "COMMAND_LONG" not in radio_types)
        check("Orin event shown on the laptop", any("[ORIN] intercept d=412m" in s for s in out))

        pair.drop = True
        for i in range(50, 72):
            tick(i)
        age_drop = hub.target.snapshot()["age"]
        pair.drop = False
        for i in range(72, 82):
            tick(i)
        age_back = hub.target.snapshot()["age"]
        check("radio drop -> target goes stale (brain would hold)", age_drop > 0.8,
              f"age={age_drop:.2f}s")
        check("radio back -> target fresh again", age_back < 0.3, f"age={age_back:.2f}s")
        time.sleep(0.2)
        back = talon.recv_match(blocking=False)
        check("the Talon received nothing (relay passive)",
              back is None and relay.target_writes == 0, str(back))
        check("bytes flowed both ways on the virtual 3DR", pair.bytes[0] > 0 and pair.bytes[1] > 0,
              str(pair.bytes))
    finally:
        state["run"] = False
        hub.stop()
        pair.close()
        time.sleep(0.15)
        for x in (fake_fc, tconn, rconn, talon):
            try:
                x.close()
            except Exception:
                pass


def main():
    print("=" * 68)
    print("RADIO layout test (3DR on the Orin) — real MAVLink frames, no hardware")
    print("=" * 68)
    test_split()
    test_timesync()
    test_downlink()
    test_events()
    test_reconnect()
    test_relay_unit()
    test_orchestrator()
    test_signals()
    test_live_e2e()
    print("-" * 68)
    print(f"PASS={PASS}  FAIL={FAIL}")
    print("-" * 68)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
