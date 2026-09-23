#!/usr/bin/env python3
"""
Offline wiring/integration test for the assembled follow_gps stack — no hardware.

test_offline.py proves the vendored planner logic. This proves the parts
follow_gps.py adds around it actually connect:
  1. ChaserEnv emits the right MAVLink on a fake connection:
     - configure_guided_speed_limits -> 4x PARAM_SET (WP_SPD/UP/DN/ACC) + DO_CHANGE_SPEED
     - send_position_target_global -> GLOBAL_RELATIVE_ALT_INT, lat/lon *1e7, pos-only mask
     - posvel on -> velocity fields populated (feed-forward)
  2. The cross-vehicle altitude math (gate_alt = target_amsl - sep_bias - chaser_home
     + alt_offset) and the --min-alt floor behave.
  3. A head-on scenario driven end-to-end (pipeline -> apply_planner_output ->
     ChaserEnv) selects INTERCEPT and lands a position target on the fake link.

Run:  python3 test_wiring.py
"""
from __future__ import annotations

import math
import sys

sys.path.insert(0, ".")

from pymavlink import mavutil

from fpv_gate_il.gate_gps import GateGpsSample
from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.coords import latlonalt_to_local
from fpv_gate_il.trajectory.mav_bridge import (
    CarrotDirSlew,
    CatchupSpeedLatch,
    PositionSetpointHold,
    SpeedCommandSlew,
    apply_planner_output,
    traj_hold_alt_m,
)
from fpv_gate_il.trajectory.pipeline import TrajectoryPipeline
from fpv_gate_il.trajectory.radar import RadarMeasurement
from fpv_gate_il.trajectory.types import BehaviorMode, DroneState, NedVelocity

from chaser_env import ChaserEnv

HOME_LAT = 39.8626103
HOME_LON = 32.7359613
PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
    else:
        FAIL += 1
    print(f"  {'[OK]  ' if cond else '[FAIL]'} {name}{('  — ' + detail) if detail else ''}")


class FakeMav:
    """Records the mavlink sends ChaserEnv makes."""

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.pos_int = []       # set_position_target_global_int_send calls
        self.local_ned = []     # set_position_target_local_ned_send calls
        self.cmd_long = []      # command_long_send calls
        self.param_set = []     # param_set_send calls

        class _Mav:
            def __init__(self, outer):
                self.o = outer

            def set_position_target_global_int_send(self, *args):
                self.o.pos_int.append(args)

            def set_position_target_local_ned_send(self, *args):
                self.o.local_ned.append(args)

            def command_long_send(self, *args):
                self.o.cmd_long.append(args)

            def param_set_send(self, *args):
                self.o.param_set.append(args)

            def param_request_read_send(self, *args):
                pass

        self.mav = _Mav(self)

    def recv_match(self, **kw):
        return None  # verify_wp_spd handles a timeout gracefully


class FakeVehicle:
    def __init__(self):
        self.conn = FakeMav()


def cfg_default(**over):
    cfg = TrajectoryConfig(home_lat_deg=HOME_LAT, home_lon_deg=HOME_LON, home_alt_m=0.0)
    for k, v in over.items():
        setattr(cfg, k, v)
    return cfg


def latlon_offset(lat, lon, dn, de):
    return (lat + dn / 111320.0, lon + de / (111320.0 * math.cos(math.radians(lat))))


# ---------------------------------------------------------------- 1. speed setup MAVLink
def test_speed_setup():
    print("1) ChaserEnv.configure_guided_speed_limits MAVLink")
    veh = FakeVehicle()
    env = ChaserEnv(veh, verbose_param=False)
    env.configure_guided_speed_limits(speed_mps=20.0, accel_mps2=4.0)
    names = [p[2].decode() if isinstance(p[2], bytes) else p[2] for p in veh.conn.param_set]
    check("PARAM_SET WP_SPD sent", "WP_SPD" in names, str(names))
    check("PARAM_SET WP_ACC sent", "WP_ACC" in names)
    check("4 params set (SPD/UP/DN/ACC)", len(veh.conn.param_set) == 4, f"n={len(veh.conn.param_set)}")
    # exactly one DO_CHANGE_SPEED kicked
    ds = [c for c in veh.conn.cmd_long if c[2] == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED]
    check("DO_CHANGE_SPEED kicked once", len(ds) == 1, f"n={len(ds)}")
    if ds:
        # command_long_send(sys, comp, cmd, conf, p1..p7): p1=type(1 gs), p2=speed
        check("DO_CHANGE_SPEED groundspeed=20", abs(ds[0][5] - 20.0) < 1e-6, f"got {ds[0][5]}")


# ---------------------------------------------------------------- 2. position target MAVLink
def test_position_target():
    print("2) ChaserEnv.send_position_target_global MAVLink (pos-only + posvel)")
    veh = FakeVehicle()
    env = ChaserEnv(veh)
    env.send_position_target_global(lat_deg=HOME_LAT, lon_deg=HOME_LON, alt_rel_m=42.0,
                                    yaw_rad=0.3)
    check("one position target sent", len(veh.conn.pos_int) == 1)
    a = veh.conn.pos_int[-1]
    # args: (time, sys, comp, frame, mask, lat_int, lon_int, alt, vx,vy,vz, ax,ay,az, yaw, yawrate)
    check("frame = GLOBAL_RELATIVE_ALT_INT",
          a[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    check("lat scaled *1e7", a[5] == int(round(HOME_LAT * 1e7)), f"got {a[5]}")
    check("alt passed through", abs(a[7] - 42.0) < 1e-6, f"got {a[7]}")
    mask = a[4]
    vx_ignore = mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    check("pos-only: VX ignored (posvel off)", bool(mask & vx_ignore))
    check("velocity fields zero (posvel off)", a[8] == 0.0 and a[9] == 0.0)

    # posvel on
    env.enable_posvel(True)
    env.set_feedforward_velocity(3.0, -1.5, 0.2)
    env.send_position_target_global(lat_deg=HOME_LAT, lon_deg=HOME_LON, alt_rel_m=42.0,
                                    yaw_rad=0.0)
    b = veh.conn.pos_int[-1]
    check("posvel on: VX NOT ignored", not bool(b[4] & vx_ignore))
    check("posvel on: vx=3.0 ve=-1.5 in message",
          abs(b[8] - 3.0) < 1e-6 and abs(b[9] + 1.5) < 1e-6, f"vx={b[8]} vy={b[9]}")


# ---------------------------------------------------------------- 3. altitude math + floor
def test_altitude_math():
    print("3) cross-vehicle altitude math + --min-alt floor")
    # Chaser: home_amsl = alt_amsl - alt_rel = 900 - 30 = 870.
    cs = dict(alt_amsl=900.0, alt_rel=30.0)
    chome = cs["alt_amsl"] - cs["alt_rel"]
    check("chaser home AMSL = 870", abs(chome - 870.0) < 1e-6, f"got {chome}")
    # Target at 950 AMSL, sep_bias 2, alt_offset -8:
    # gate_alt = 950 - 2 - 870 + (-8) = 70.  (target 80 m above chaser home, minus 8, minus bias 2)
    ts = dict(alt_amsl=950.0)
    sep_bias, alt_offset = 2.0, -8.0
    gate_alt = (ts["alt_amsl"] - sep_bias - chome) + alt_offset
    check("gate_alt = 70 (target rel 80, -bias2, -8)", abs(gate_alt - 70.0) < 1e-6, f"got {gate_alt}")

    # --alt-match: commanded altitude == gate_alt (no clearance/cap)
    cfg_m = cfg_default(traj_alt_match_above_m=-1e9)
    cmd = traj_hold_alt_m(gate_alt, cfg_m, range_m=150.0)
    check("alt-match: commanded == gate_alt (70)", abs(cmd - 70.0) < 1e-6, f"got {cmd}")

    # min-alt floor: target very low so commanded would dip below floor -> bump
    min_alt = 15.0
    gate_low = (880.0 - 0.0 - chome) + (-8.0)  # 880-870-8 = 2 m -> below floor
    preview = traj_hold_alt_m(gate_low, cfg_m, range_m=150.0)
    if preview < min_alt:
        gate_low += (min_alt - preview)
    cmd_floor = traj_hold_alt_m(gate_low, cfg_m, range_m=150.0)
    check("min-alt floor lifts commanded to >= 15", cmd_floor >= min_alt - 1e-6, f"got {cmd_floor}")


# ---------------------------------------------------------------- 4. end-to-end drive
def test_end_to_end_intercept():
    print("4) head-on scenario: pipeline -> apply_planner_output -> ChaserEnv")
    cfg = cfg_default()
    pipe = TrajectoryPipeline(cfg)
    pipe.soft_reset_guidance(realign=False)
    veh = FakeVehicle()
    env = ChaserEnv(veh)
    slew = SpeedCommandSlew.from_config(cfg)
    hold = PositionSetpointHold.from_config(cfg)
    dslew = CarrotDirSlew.from_config(cfg)
    catch = CatchupSpeedLatch()
    slew.reset(0.0)

    clat, clon, calt = HOME_LAT, HOME_LON, 40.0
    last_out = None
    for i in range(18):
        t = i * 0.5
        tn = 300.0 - 15.0 * t   # target closes from 300 m North, flying South
        tlat, tlon = latlon_offset(HOME_LAT, HOME_LON, tn, 0.0)
        meas = RadarMeasurement(timestamp_s=t, latitude_deg=tlat, longitude_deg=tlon,
                                altitude_up_m=40.0, ground_speed_mps=15.0, course_rad=math.pi)
        c_ned = latlonalt_to_local(clat, clon, calt, home_lat_deg=cfg.home_lat_deg,
                                   home_lon_deg=cfg.home_lon_deg, home_alt_m=cfg.home_alt_m)
        drone = DroneState(position=c_ned, velocity=NedVelocity(0, 0, 0),
                           timestamp_s=t, speed_mps=0.0, heading_rad=0.0)
        last_out = pipe.update(meas, drone)

    check("behavior = INTERCEPT (cuts in front)", last_out.behavior is BehaviorMode.INTERCEPT,
          f"got {last_out.behavior.value if last_out.behavior else None}")

    # now drive the chaser for a few ticks
    tlat, tlon = latlon_offset(HOME_LAT, HOME_LON, 180.0, 0.0)
    gate = GateGpsSample(lat=tlat, lon=tlon, alt=40.0, recv_mono=0.0)
    for k in range(20):
        t = 9.0 + k * 0.1
        c_ned = latlonalt_to_local(clat, clon, calt, home_lat_deg=cfg.home_lat_deg,
                                   home_lon_deg=cfg.home_lon_deg, home_alt_m=cfg.home_alt_m)
        drone = DroneState(position=c_ned, velocity=NedVelocity(min(15, k), 0, 0),
                           timestamp_s=t, speed_mps=float(min(15, k)), heading_rad=0.0)
        env.refresh_from_snapshot({"lat": clat, "lon": clon, "fix": 3})
        apply_planner_output(env, last_out, gate=gate, drone=drone, config=cfg,
                             speed_slew=slew, pos_hold=hold, dir_slew=dslew,
                             catchup=catch, now_s=t)
    check("position targets reached the link", len(veh.conn.pos_int) >= 1,
          f"n={len(veh.conn.pos_int)}")
    ds = [c for c in veh.conn.cmd_long if c[2] == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED]
    check("DO_CHANGE_SPEED reached the link", len(ds) >= 1, f"n={len(ds)}")
    # sent altitude should be finite and near target (40 default clearance/cap region)
    alt_sent = veh.conn.pos_int[-1][7]
    check("commanded altitude sane", math.isfinite(alt_sent) and 0 < alt_sent < 120,
          f"alt={alt_sent:.1f}")


def test_cmd_latency():
    print("5) command->response latency tracker (RTT + lost + spd_start)")
    import os
    import tempfile
    from link_log import CmdLatencyTracker

    d = tempfile.mkdtemp()
    tr = CmdLatencyTracker(os.path.join(d, "cmd.csv"), {"t": "x"}, enabled=True)
    DCS = mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
    REQ = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE

    # RTT: command then ACK 120 ms later
    tr.mark_command(DCS, 100.0)

    class Ack:
        def get_type(self):
            return "COMMAND_ACK"
        command = DCS

    tr.note_ack(Ack(), 100.120)
    check("RTT sample recorded", tr.rtt_count == 1, f"n={tr.rtt_count}")

    # unmatched ACK is ignored (no outstanding)
    tr.note_ack(Ack(), 100.5)
    check("stray ACK ignored", tr.rtt_count == 1, f"n={tr.rtt_count}")

    # lost: a command with no ACK, then a later same-command send -> rtt_lost
    tr.mark_command(REQ, 200.0)
    tr.mark_command(REQ, 203.0)
    check("lost ACK detected after timeout", tr.lost_count == 1, f"n={tr.lost_count}")

    # spd_start: arm at gs=10 target=20, gs moves past +0.5
    tr.arm_speed(from_gs=10.0, target=20.0, t_mono=300.0)
    tr.note_gs(10.1, 300.05)  # below threshold, no log
    tr.note_gs(10.7, 300.18)  # crosses +0.5 -> spd_start
    tr.close()

    rows = [r for r in open(os.path.join(d, "cmd.csv")).read().splitlines()
            if r and not r.startswith("#") and "event" not in r]
    kinds = [r.split(",")[2] for r in rows]
    check("CSV has rtt row", "rtt" in kinds, str(kinds))
    check("CSV has rtt_lost row", "rtt_lost" in kinds)
    check("CSV has spd_start row", "spd_start" in kinds)
    # the rtt row carries ~120 ms
    rtt_row = next(r for r in rows if r.split(",")[2] == "rtt")
    lat = float(rtt_row.split(",")[4])
    check("rtt latency ~120 ms", abs(lat - 120.0) < 1.0, f"got {lat}")


def main():
    print("=" * 68)
    print("follow_gps wiring/integration test (no hardware)")
    print("=" * 68)
    test_speed_setup()
    test_position_target()
    test_altitude_math()
    test_end_to_end_intercept()
    test_cmd_latency()
    print("-" * 68)
    print(f"PASS={PASS}  FAIL={FAIL}")
    print("-" * 68)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
