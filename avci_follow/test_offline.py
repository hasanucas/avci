#!/usr/bin/env python3
"""
Offline regression for the vendored trajectory planner + the ChaserEnv contract.

No hardware, no MAVLink, no serial. This proves, before any field run:
  1. The vendored fpv_gate_il.trajectory package imports with no Gazebo dep.
  2. The NED frame we build for the chaser round-trips through coords.
  3. A head-on target selects INTERCEPT (chaser cuts in front).
  4. A crossing target selects FOLLOW (chaser trails on course).
  5. apply_planner_output drives exactly the 4 env methods ChaserEnv implements,
     emits a GUIDED position target, and ramps DO_CHANGE_SPEED toward the peak.
  6. traj_hold_alt_m altitude modes: file-default clearance, --alt-match, offset.

Run:  python3 test_offline.py
"""
from __future__ import annotations

import math
import sys

sys.path.insert(0, ".")

from fpv_gate_il.gate_gps import GateGpsSample
from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.coords import latlonalt_to_local, local_to_latlonalt
from fpv_gate_il.trajectory.mav_bridge import (
    CarrotDirSlew,
    CatchupSpeedLatch,
    PositionSetpointHold,
    SpeedCommandSlew,
    apply_planner_output,
    gate_sample_to_radar,
    traj_hold_alt_m,
)
from fpv_gate_il.trajectory.radar import RadarMeasurement
from fpv_gate_il.trajectory.pipeline import TrajectoryPipeline
from fpv_gate_il.trajectory.types import (
    BehaviorMode,
    DroneState,
    NedPosition,
    NedVelocity,
)

# Ankara-ish origin so we exercise real cos(lat) scaling, not the Canberra default.
HOME_LAT = 39.8626103
HOME_LON = 32.7359613

PASS = 0
FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    tag = "[OK]  " if cond else "[FAIL]"
    if cond:
        PASS += 1
    else:
        FAIL += 1
    print(f"  {tag} {name}{('  — ' + detail) if detail else ''}")


# ---------------------------------------------------------------- mock env
class MockEnv:
    """Records the 4 methods apply_planner_output / configure call.

    This is the exact duck-typed contract ChaserEnv must satisfy — the offline
    test and the real adapter share one interface.
    """

    def __init__(self, chaser_lat, chaser_lon, chaser_alt_rel):
        self.chaser_lat = chaser_lat
        self.chaser_lon = chaser_lon
        self.chaser_alt_rel = chaser_alt_rel
        self.pos_cmds: list[tuple] = []
        self.vel_cmds: list[tuple] = []
        self.speed_cmds: list[float] = []

    # --- the four methods apply_planner_output touches ---
    def get_drone_state(self) -> dict:
        return {
            "global_valid": True,
            "lat_deg": self.chaser_lat,
            "lon_deg": self.chaser_lon,
        }

    def send_position_target_global(self, *, lat_deg, lon_deg, alt_rel_m, yaw_rad=None):
        self.pos_cmds.append((lat_deg, lon_deg, alt_rel_m, yaw_rad))

    def send_velocity_ned(self, *, vn_mps, ve_mps, vd_mps, yaw_rad=None):
        self.vel_cmds.append((vn_mps, ve_mps, vd_mps, yaw_rad))

    def set_guided_horizontal_speed_mps(self, speed_mps, *, min_interval_s=0.05, min_delta_mps=0.15):
        self.speed_cmds.append(float(speed_mps))


def make_cfg(**over) -> TrajectoryConfig:
    cfg = TrajectoryConfig(home_lat_deg=HOME_LAT, home_lon_deg=HOME_LON, home_alt_m=0.0)
    for k, v in over.items():
        setattr(cfg, k, v)
    return cfg


def drone_at(cfg, lat, lon, alt_rel, vn=0.0, ve=0.0, vd=0.0, t=0.0) -> DroneState:
    ned = latlonalt_to_local(lat, lon, alt_rel,
                             home_lat_deg=cfg.home_lat_deg,
                             home_lon_deg=cfg.home_lon_deg,
                             home_alt_m=cfg.home_alt_m)
    return DroneState(position=ned, velocity=NedVelocity(vn, ve, vd),
                      timestamp_s=t, speed_mps=math.hypot(vn, ve),
                      heading_rad=math.atan2(ve, vn))


def latlon_offset(lat, lon, dnorth_m, deast_m):
    """Move a lat/lon by NED metres (flat-earth, matches coords.py)."""
    dlat = dnorth_m / 111320.0
    dlon = deast_m / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


# ---------------------------------------------------------------- 1. frame round-trip
def test_frame_roundtrip():
    print("1) NED frame round-trip (chaser lat/lon -> NED -> lat/lon)")
    cfg = make_cfg()
    lat, lon = latlon_offset(HOME_LAT, HOME_LON, 137.0, -88.0)
    ned = latlonalt_to_local(lat, lon, 42.0,
                             home_lat_deg=cfg.home_lat_deg, home_lon_deg=cfg.home_lon_deg,
                             home_alt_m=cfg.home_alt_m)
    check("north≈137 m", abs(ned.north_m - 137.0) < 0.5, f"got {ned.north_m:.2f}")
    check("east≈-88 m", abs(ned.east_m + 88.0) < 0.5, f"got {ned.east_m:.2f}")
    check("down = -alt_up", abs(ned.down_m + 42.0) < 1e-6, f"got {ned.down_m:.2f}")
    rlat, rlon, ralt = local_to_latlonalt(ned, home_lat_deg=cfg.home_lat_deg,
                                          home_lon_deg=cfg.home_lon_deg, home_alt_m=cfg.home_alt_m)
    check("lat round-trips", abs(rlat - lat) < 1e-9, f"d={abs(rlat-lat):.2e}")
    check("lon round-trips", abs(rlon - lon) < 1e-9, f"d={abs(rlon-lon):.2e}")
    check("alt round-trips", abs(ralt - 42.0) < 1e-9)


# ---------------------------------------------------------------- behavior scenarios
def run_scenario(cfg, target_start_ned, target_vel_ned, chaser_ned_latlon, *, n=16, dt=0.5):
    """Feed n moving-target fixes into the pipeline; return the final PlannerOutput.

    target_start_ned: (north, east) of target at t=0
    target_vel_ned:   (vn, ve) target velocity, m/s
    chaser_ned_latlon:(lat, lon, alt_rel) fixed chaser pose
    """
    pipe = TrajectoryPipeline(cfg)
    pipe.soft_reset_guidance(realign=False)
    tn0, te0 = target_start_ned
    vn, ve = target_vel_ned
    clat, clon, calt = chaser_ned_latlon
    out = None
    for i in range(n):
        t = i * dt
        tn = tn0 + vn * t
        te = te0 + ve * t
        tlat, tlon = latlon_offset(cfg.home_lat_deg, cfg.home_lon_deg, tn, te)
        meas = RadarMeasurement(
            timestamp_s=t, latitude_deg=tlat, longitude_deg=tlon, altitude_up_m=40.0,
            ground_speed_mps=math.hypot(vn, ve),
            course_rad=math.atan2(ve, vn) if (vn or ve) else None,
        )
        drone = drone_at(cfg, clat, clon, calt, t=t)
        out = pipe.update(meas, drone)
    return pipe, out


def test_headon_intercept():
    print("2) Head-on target -> INTERCEPT (chaser cuts in front)")
    cfg = make_cfg()
    # target 300 m North of chaser, flying South (toward chaser) at 15 m/s.
    pipe, out = run_scenario(cfg, target_start_ned=(300.0, 0.0),
                             target_vel_ned=(-15.0, 0.0),
                             chaser_ned_latlon=(HOME_LAT, HOME_LON, 40.0))
    check("behavior = INTERCEPT", out.behavior is BehaviorMode.INTERCEPT,
          f"got {out.behavior.value if out.behavior else None}")
    check("has a path with waypoints", out.path is not None and len(out.path.waypoints) >= 1)
    check("target course ~South (180°)",
          out.target is not None and abs(abs(math.degrees(out.target.course_rad)) - 180.0) < 20.0,
          f"got {math.degrees(out.target.course_rad):.0f}°" if out.target else "no target")


def test_crossing_follow():
    print("3) Crossing target -> FOLLOW (chaser trails on course)")
    cfg = make_cfg()
    # target 300 m North, flying East (perpendicular / crossing) at 15 m/s.
    pipe, out = run_scenario(cfg, target_start_ned=(300.0, 0.0),
                             target_vel_ned=(0.0, 15.0),
                             chaser_ned_latlon=(HOME_LAT, HOME_LON, 40.0))
    check("behavior = FOLLOW", out.behavior is BehaviorMode.FOLLOW,
          f"got {out.behavior.value if out.behavior else None}")
    check("target course ~East (90°)",
          out.target is not None and abs(math.degrees(out.target.course_rad) - 90.0) < 20.0,
          f"got {math.degrees(out.target.course_rad):.0f}°" if out.target else "no target")


# ---------------------------------------------------------------- apply_planner_output contract
def test_apply_output_drives_env():
    print("4) apply_planner_output drives the 4-method env contract")
    cfg = make_cfg()
    # Far target (>1000 m) so the speed schedule allows the peak; slow closing
    # so range stays far through the whole ramp.
    pipe, out = run_scenario(cfg, target_start_ned=(1500.0, 0.0),
                             target_vel_ned=(-5.0, 0.0),
                             chaser_ned_latlon=(HOME_LAT, HOME_LON, 40.0))
    env = MockEnv(HOME_LAT, HOME_LON, 40.0)
    slew = SpeedCommandSlew.from_config(cfg)
    hold = PositionSetpointHold.from_config(cfg)
    dslew = CarrotDirSlew.from_config(cfg)
    catch = CatchupSpeedLatch()
    slew.reset(0.0)
    # drive several control ticks (mimics loop re-send between GPS fixes)
    last_speed = None
    # gate tracks the (far) target; ~1460 m North after the closing scenario.
    tlat, tlon = latlon_offset(HOME_LAT, HOME_LON, 1460.0, 0.0)
    gate = GateGpsSample(lat=tlat, lon=tlon, alt=40.0, recv_mono=0.0)
    # ~14 s of control ticks at 10 Hz so the 3 m/s² slew can reach the peak;
    # fake the airframe accelerating (drone.speed_mps) so the lead cap opens up.
    for k in range(140):
        t = 8.0 + k * 0.1
        drone = drone_at(cfg, HOME_LAT, HOME_LON, 40.0, vn=min(20.0, k * 0.5), t=t)
        last_speed = apply_planner_output(
            env, out, gate=gate, drone=drone, config=cfg,
            last_speed_cmd=last_speed, speed_slew=slew, pos_hold=hold,
            dir_slew=dslew, catchup=catch, now_s=t,
        )
    check("emitted >=1 position target", len(env.pos_cmds) >= 1, f"n={len(env.pos_cmds)}")
    check("emitted >=1 DO_CHANGE_SPEED", len(env.speed_cmds) >= 1, f"n={len(env.speed_cmds)}")
    check("far target: speed ramps toward peak (>15 m/s)",
          max(env.speed_cmds) > 15.0, f"max={max(env.speed_cmds):.1f}")
    check("speed never exceeds peak cap", max(env.speed_cmds) <= cfg.max_speed_mps + 1e-6,
          f"max={max(env.speed_cmds):.1f} cap={cfg.max_speed_mps}")
    alt_sent = env.pos_cmds[-1][2]
    check("position altitude finite & sane", math.isfinite(alt_sent) and 0 < alt_sent < 200,
          f"alt={alt_sent:.1f} m")

    # Near-range taper: within deadline_range_m the schedule floors at
    # close_approach_speed_mps (this is the intended "slow down when close").
    from fpv_gate_il.trajectory.mav_bridge import speed_for_gate_range_m
    v_far = speed_for_gate_range_m(1500.0, cfg)
    v_near = speed_for_gate_range_m(150.0, cfg)
    check("schedule: far(1500m) -> peak 20", abs(v_far - cfg.max_speed_mps) < 1e-6, f"got {v_far:.1f}")
    check("schedule: near(150m) -> floor 10", abs(v_near - cfg.close_approach_speed_mps) < 1e-6,
          f"got {v_near:.1f}")


# ---------------------------------------------------------------- altitude modes
def test_altitude_modes():
    print("5) traj_hold_alt_m altitude modes")
    # target at 30 m (below match_above=50): default -> 30 + clearance(10 far) = 40
    cfg = make_cfg()
    # far clearance applies only beyond traj_alt_near_range_m (default 1000 m)
    a_far = traj_hold_alt_m(30.0, cfg, range_m=1500.0)
    a_near = traj_hold_alt_m(30.0, cfg, range_m=100.0)
    check("default: gate30 far(>1000m) -> 40 (clearance +10)", abs(a_far - 40.0) < 1e-6, f"got {a_far}")
    check("default: gate30 near -> 35 (clearance +5)", abs(a_near - 35.0) < 1e-6, f"got {a_near}")
    a_high = traj_hold_alt_m(70.0, cfg, range_m=800.0)
    check("default: gate70 (>=50) -> match 70", abs(a_high - 70.0) < 1e-6, f"got {a_high}")

    # --alt-match: set match_above very low -> always return gate alt unchanged
    cfg_m = make_cfg(traj_alt_match_above_m=-1e9)
    am = traj_hold_alt_m(30.0, cfg_m, range_m=100.0)
    check("--alt-match: gate30 -> 30 (no clearance/cap)", abs(am - 30.0) < 1e-6, f"got {am}")

    # offset folded into gate.alt under --alt-match: gate(30) + offset(-8) = 22
    off = -8.0
    am_off = traj_hold_alt_m(30.0 + off, cfg_m, range_m=100.0)
    check("--alt-match + offset -8: gate30 -> 22", abs(am_off - 22.0) < 1e-6, f"got {am_off}")


def main():
    print("=" * 68)
    print("follow_gps offline regression")
    print("=" * 68)
    test_frame_roundtrip()
    test_headon_intercept()
    test_crossing_follow()
    test_apply_output_drives_env()
    test_altitude_modes()
    print("-" * 68)
    print(f"PASS={PASS}  FAIL={FAIL}")
    print("-" * 68)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
