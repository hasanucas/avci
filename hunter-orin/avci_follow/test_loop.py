#!/usr/bin/env python3
"""
Loop-level offline test: runs the REAL follow_gps.main() end to end.

test_offline.py / test_wiring.py prove the planner and the MAVLink adapter in
isolation; neither ever executed the main loop — which is exactly where a
KeyError crash hid. Here the radios are replaced by scripted fake vehicles and
time by a virtual clock (10 Hz ticks, no real waiting), then main() runs:

  A. dry-run, head-on target  -> intercept chosen, nothing sent, no crash
  B. real run + posvel        -> WP_SPD set & confirmed (no link read race),
                                 carrots + DO_CHANGE_SPEED sent, feed-forward
                                 = TARGET velocity, Ctrl-C -> LOITER
  C. pilot takes over         -> loop exits, mode NOT overridden
  D. target goes stale        -> chaser held in place (not flown to old carrot)
  E. unexpected exception     -> loop stops, chaser still put in LOITER

Run:  python3 test_loop.py
"""
from __future__ import annotations

import builtins
import contextlib
import io
import math
import sys
import time as _real_time

sys.path.insert(0, ".")

import chase_core  # noqa: E402
import follow_gps  # noqa: E402
from pymavlink import mavutil  # noqa: E402

HOME_LAT, HOME_LON, HOME_AMSL = 39.8626103, 32.7359613, 900.0
PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += (not cond)
    print(f"  {'[OK]  ' if cond else '[FAIL]'} {name}{('  — ' + detail) if detail else ''}")


def offset(n, e):
    return (HOME_LAT + n / 111320.0,
            HOME_LON + e / (111320.0 * math.cos(math.radians(HOME_LAT))))


# ---------------------------------------------------------------- virtual clock
class FakeTime:
    """Replaces follow_gps's `time` module: sleep() advances virtual time."""

    def __init__(self, budget_s):
        self.t = 1000.0
        self.budget = budget_s
        self.raised = False

    def monotonic(self):
        return self.t

    def time(self):
        return 1.7e9 + self.t

    def perf_counter(self):
        return _real_time.perf_counter()

    def strftime(self, *a):
        return _real_time.strftime(*a)

    def sleep(self, dt):
        self.t += max(0.0, float(dt))
        if not self.raised and self.t - 1000.0 > self.budget:
            self.raised = True
            raise KeyboardInterrupt  # the operator's Ctrl-C


# ---------------------------------------------------------------- scripted world
class World:
    def __init__(self, clock, *, n0=400.0, vn=-15.0, ve=0.0, stop_fix_at=None,
                 pilot_mode_at=None):
        self.clock = clock
        self.t0 = clock.monotonic()
        self.n0, self.vn, self.ve = n0, vn, ve
        self.stop_fix_at = stop_fix_at
        self.pilot_mode_at = pilot_mode_at
        self.chaser_mode = "LOITER"

    def target(self):
        now = self.clock.monotonic()
        t = now - self.t0
        if self.stop_fix_at is not None:
            t = min(t, self.stop_fix_at)
        t_fix = math.floor(t / 0.1) * 0.1           # 10 Hz fixes
        lat, lon = offset(self.n0 + self.vn * t_fix, self.ve * t_fix)
        pos_mono = self.t0 + t_fix
        return dict(lat=lat, lon=lon, alt_amsl=HOME_AMSL + 45.0, alt_rel=45.0,
                    vx=self.vn, vy=self.ve, vz=0.0, fix=3, nsat=15, mode="LOITER",
                    armed=True, age=now - pos_mono, hz=10.0, raw=b"", pos_mono=pos_mono,
                    roll=None, pitch=None, yaw=None, att_age=None)

    def chaser(self):
        now = self.clock.monotonic()
        mode = self.chaser_mode
        if self.pilot_mode_at is not None and now - self.t0 > self.pilot_mode_at:
            mode = "ALT_HOLD"  # pilot flipped the RC switch
        return dict(lat=HOME_LAT, lon=HOME_LON, alt_amsl=HOME_AMSL + 40.0, alt_rel=40.0,
                    vx=0.0, vy=0.0, vz=0.0, fix=3, nsat=16, mode=mode, armed=True,
                    age=0.05, hz=10.0, raw=b"", pos_mono=now - 0.05,
                    roll=0.0, pitch=0.0, yaw=math.pi / 2, att_age=0.1)


class FakeMav:
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.pos = []        # set_position_target_global_int args
        self.cmds = []       # command_long args
        self.params = {}     # FC-side param store (PARAM_SET applied)

        outer = self

        class _M:
            def set_position_target_global_int_send(self, *a):
                outer.pos.append(a)

            def set_position_target_local_ned_send(self, *a):
                pass

            def command_long_send(self, *a):
                outer.cmds.append(a)

            def param_set_send(self, sysid, comp, name, value, ptype):
                outer.params[name.decode() if isinstance(name, bytes) else name] = float(value)

            def param_request_read_send(self, *a):
                pass

            def request_data_stream_send(self, *a):
                pass

        self.mav = _M()

    def recv_match(self, *a, **k):
        raise AssertionError("main thread must never read the link (RX thread owns it)")


class FakeVehicle:
    world = None
    made = []

    def __init__(self, name, port, baud, stream_hz=10.0, on_msg=None, att_hz=0.0):
        self.name = name
        self.is_chaser = name.strip().upper().startswith("TAKIPCI")
        self.sysid = 1 if self.is_chaser else 2
        self.conn = FakeMav()
        self.mode_calls = []
        FakeVehicle.made.append(self)

    @property
    def mode(self):
        return self.snapshot()["mode"]

    def snapshot(self):
        return FakeVehicle.world.chaser() if self.is_chaser else FakeVehicle.world.target()

    def set_mode(self, name, timeout=4.0):
        self.mode_calls.append(name)
        if self.is_chaser:
            FakeVehicle.world.chaser_mode = name
        return True

    def wait_param(self, name, *, since_mono, timeout):
        return self.conn.params.get(name)

    def _request_streams(self):
        pass

    @staticmethod
    def home_amsl(s):
        return chase_core.chaser_home_amsl(s)


def run_main(argv, *, budget_s=10.0, world_kw=None, tick_hook=None):
    clock = FakeTime(budget_s)
    FakeVehicle.world = World(clock, **(world_kw or {}))
    FakeVehicle.made = []
    saved = (follow_gps.Vehicle, follow_gps.time, builtins.input, sys.argv,
             follow_gps.TrackController)
    follow_gps.Vehicle = FakeVehicle
    follow_gps.time = clock
    builtins.input = lambda *a: "evet"
    sys.argv = ["follow_gps.py", "--target-port", "T", "--chaser-port", "C",
                "--no-rec", "--no-log"] + list(argv)
    if tick_hook is not None:
        follow_gps.TrackController = tick_hook
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            follow_gps.main()
    finally:
        (follow_gps.Vehicle, follow_gps.time, builtins.input, sys.argv,
         follow_gps.TrackController) = saved
    target = next(v for v in FakeVehicle.made if not v.is_chaser)
    chaser = next(v for v in FakeVehicle.made if v.is_chaser)
    return out.getvalue(), target, chaser


def dcs_speeds(chaser):
    return [c[5] for c in chaser.conn.cmds if c[2] == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED]


# ---------------------------------------------------------------- scenarios
def test_dry_run():
    print("A) dry-run, head-on target")
    txt, _t, c = run_main(["--dry-run"])
    check("no crash / traceback", "Traceback" not in txt and "Beklenmeyen" not in txt)
    check("loop ran (status lines printed)", txt.count(" d=") >= 20, f"lines={txt.count(' d=')}")
    check("head-on -> beh=intercept", "beh=intercept" in txt)
    check("dry-run sends nothing", not c.conn.pos and not c.conn.cmds and not c.conn.params,
          f"pos={len(c.conn.pos)} cmds={len(c.conn.cmds)}")
    check("dry-run never touches the mode", c.mode_calls == [], str(c.mode_calls))


def test_real_run_posvel():
    print("B) real run + posvel, Ctrl-C at the end")
    txt, t, c = run_main(["--set-mode", "--posvel", "--alt-offset", "-8"])
    check("no crash / traceback", "Traceback" not in txt and "Beklenmeyen" not in txt)
    check("GUIDED set, then LOITER on Ctrl-C", c.mode_calls == ["GUIDED", "LOITER"],
          str(c.mode_calls))
    check("WP_SPD raised to 20 + WP_ACC", c.conn.params.get("WP_SPD") == 20.0
          and c.conn.params.get("WP_ACC") == 4.0, str(c.conn.params))
    check("WP_SPD read-back confirmed (RX-thread waiter, no link race)",
          "WP_SPD ceiling confirmed" in txt)
    check("carrot position targets sent", len(c.conn.pos) >= 20, f"n={len(c.conn.pos)}")
    check("DO_CHANGE_SPEED sent", len(dcs_speeds(c)) >= 1, f"n={len(dcs_speeds(c))}")
    last = c.conn.pos[-1]
    # args: (t, sys, comp, frame, mask, lat, lon, alt, vx, vy, vz, ...)
    check("addressed to the chaser (sysid 1)", last[1] == 1, f"sys={last[1]}")
    check("posvel feed-forward = TARGET velocity (-15,0), not chaser's (0,0)",
          abs(last[8] + 15.0) < 1e-6 and abs(last[9]) < 1e-6, f"vx={last[8]} vy={last[9]}")
    check("head-on -> beh=intercept", "beh=intercept" in txt)


def test_pilot_takeover():
    print("C) pilot flips the RC switch mid-chase")
    txt, _t, c = run_main(["--set-mode"], budget_s=12.0, world_kw=dict(pilot_mode_at=8.0))
    check("loop exits on mode change", "Kontrol kumandada" in txt)
    check("mode NOT overridden (no LOITER after takeover)", c.mode_calls == ["GUIDED"],
          str(c.mode_calls))
    check("says control is with the pilot", "Kontrol pilotta" in txt)


def test_stale_hold():
    print("D) target telemetry goes stale")
    txt, _t, c = run_main(["--set-mode"], budget_s=12.0, world_kw=dict(stop_fix_at=5.0))
    check("stale detected", "bayat" in txt)
    check("chaser held in place while stale", "yerinde tutuluyor" in txt)
    last = c.conn.pos[-1]
    lat_i, lon_i = int(round(HOME_LAT * 1e7)), int(round(HOME_LON * 1e7))
    check("last setpoint = chaser's own position (hold), not an old carrot",
          last[5] == lat_i and last[6] == lon_i, f"lat={last[5]} lon={last[6]}")
    check("hold keeps current altitude (40 m)", abs(last[7] - 40.0) < 1e-6, f"alt={last[7]}")


def test_exception_safety():
    print("E) unexpected exception inside the loop")

    class Boom(chase_core.TrackController):
        n = 0

        def tick(self, ts, cs, now):
            Boom.n += 1
            if Boom.n > 5:
                raise RuntimeError("simulated bug")
            return super().tick(ts, cs, now)

    txt, _t, c = run_main(["--set-mode"], tick_hook=Boom)
    check("error reported", "Beklenmeyen hata" in txt and "simulated bug" in txt)
    check("chaser still put in LOITER (was GUIDED)", c.mode_calls == ["GUIDED", "LOITER"],
          str(c.mode_calls))


def main():
    print("=" * 68)
    print("follow_gps main-loop test (fake vehicles, virtual clock)")
    print("=" * 68)
    test_dry_run()
    test_real_run_posvel()
    test_pilot_takeover()
    test_stale_hold()
    test_exception_safety()
    print("-" * 68)
    print(f"PASS={PASS}  FAIL={FAIL}")
    print("-" * 68)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
