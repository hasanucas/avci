"""Bridge trajectory planner I/O to MAVLink GUIDED setpoints."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

from fpv_gate_il.gate_gps import GateGpsSample, bearing_ned_rad

# VENDOR NOTE: In the standalone follow_gps package the trajectory planner is
# driven by a duck-typed pymavlink adapter (chaser_env.ChaserEnv), not the
# Gazebo SITL env. `apply_planner_output(env, ...)` only calls four methods on
# `env` (get_drone_state / send_position_target_global / send_velocity_ned /
# set_guided_horizontal_speed_mps), so the concrete type is irrelevant at
# runtime. `from __future__ import annotations` (above) makes the `env:
# GazeboDroneTrackingEnv` annotation a lazy string that is never evaluated, so
# guarding the import under TYPE_CHECKING keeps the file byte-for-byte
# behaviourally identical while removing the heavy Gazebo dependency.
if TYPE_CHECKING:  # pragma: no cover
    from fpv_gate_il.gazebo_drone_env import GazeboDroneTrackingEnv
from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.coords import distance_m, local_to_latlonalt
from fpv_gate_il.trajectory.radar import RadarMeasurement
from fpv_gate_il.trajectory.types import (
    BehaviorMode,
    DroneState,
    FollowPhase,
    NedPosition,
    NedVelocity,
    Path,
    PlannerOutput,
    RealignPhase,
    TargetState,
    Waypoint,
)

_EPS = 1e-9


def _wrap_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _wrap_180(angle_deg: float) -> float:
    return float((angle_deg + 180.0) % 360.0 - 180.0)


def _unit_ne(dn: float, de: float) -> tuple[float, float]:
    n = math.hypot(dn, de)
    if n <= _EPS:
        return 1.0, 0.0
    return dn / n, de / n


@dataclass
class PositionSetpointHold:
    """Ignore micro-moves of the GUIDED position target (reduces pitch chop)."""

    last: NedPosition | None = None
    min_retarget_m: float = 30.0

    @classmethod
    def from_config(cls, config: TrajectoryConfig) -> PositionSetpointHold:
        return cls(min_retarget_m=float(config.setpoint_retarget_min_m))

    def reset(self) -> None:
        self.last = None

    def hold(self, desired: NedPosition) -> NedPosition:
        if self.last is None or distance_m(self.last, desired) >= self.min_retarget_m:
            self.last = desired
        return self.last


@dataclass
class SpeedCommandSlew:
    """
    Rate-limit GUIDED speed commands.

    Default ~3 m/s² ≈ +0.15 m/s every 50 ms — stays under WP_ACC=4 so
    DO_CHANGE_SPEED does not race far ahead of the airframe.
    """

    value_mps: float = 0.0
    last_t_s: float | None = None
    slew_up_mps2: float = 2.0
    slew_dn_mps2: float = 2.5
    lead_mps: float = 3.0

    @classmethod
    def from_config(cls, config: TrajectoryConfig) -> SpeedCommandSlew:
        return cls(
            slew_up_mps2=float(config.speed_cmd_slew_up_mps2),
            slew_dn_mps2=float(config.speed_cmd_slew_dn_mps2),
            lead_mps=float(config.speed_cmd_lead_mps),
        )

    def reset(self, value_mps: float = 0.0) -> None:
        self.value_mps = float(value_mps)
        self.last_t_s = None

    def step(
        self,
        target_mps: float,
        *,
        now_s: float | None = None,
        actual_mps: float | None = None,
    ) -> float:
        now = float(time.monotonic() if now_s is None else now_s)
        target = max(0.0, float(target_mps))
        if actual_mps is not None and self.lead_mps >= 0.0:
            # Do not ask Ardu for more than it can reach soon (WP_ACC limited).
            target = min(target, max(0.0, float(actual_mps)) + float(self.lead_mps))
        if self.last_t_s is None:
            self.last_t_s = now
            # First tick: one 100 ms quantum from current value (often seeded GS).
            dt = 0.1
        else:
            dt = max(1e-3, now - self.last_t_s)
            self.last_t_s = now

        max_up = float(self.slew_up_mps2) * dt
        max_dn = float(self.slew_dn_mps2) * dt
        dv = target - self.value_mps
        if dv > max_up:
            dv = max_up
        elif dv < -max_dn:
            dv = -max_dn
        self.value_mps = max(0.0, self.value_mps + dv)
        self.last_t_s = now
        return self.value_mps


@dataclass
class CarrotDirSlew:
    """Rate-limit GUIDED aim heading (deg/s and lateral-accel caps)."""

    n: float = 1.0
    e: float = 0.0
    last_t_s: float | None = None
    max_rate_deg_s: float = 15.0
    lat_accel_mps2: float = 3.5
    # +1 = clockwise / right, -1 = CCW / left. None = shortest wrap.
    turn_sign: float | None = None
    turn_unlock_deg: float = 35.0
    _seeded: bool = False

    @classmethod
    def from_config(cls, config: TrajectoryConfig) -> CarrotDirSlew:
        return cls(
            max_rate_deg_s=float(config.carrot_max_rate_deg_s),
            lat_accel_mps2=float(config.carrot_lat_accel_mps2),
            turn_unlock_deg=float(config.realign_turn_unlock_deg),
        )

    def reset(self) -> None:
        self.n, self.e = 1.0, 0.0
        self.last_t_s = None
        self.turn_sign = None
        self._seeded = False

    def step(
        self,
        want_n: float,
        want_e: float,
        *,
        now_s: float | None = None,
        drone: DroneState | None = None,
    ) -> tuple[float, float]:
        now = float(time.monotonic() if now_s is None else now_s)
        wn, we = _unit_ne(float(want_n), float(want_e))

        if not self._seeded:
            self._seeded = True
            self.last_t_s = now
            if drone is not None and drone.speed_mps >= 1.0:
                vn = float(drone.velocity.vn_mps)
                ve = float(drone.velocity.ve_mps)
                if math.hypot(vn, ve) > _EPS:
                    self.n, self.e = _unit_ne(vn, ve)
                    return self.n, self.e
            self.n, self.e = wn, we
            return self.n, self.e

        if self.last_t_s is None:
            dt = 0.1
        else:
            dt = max(1e-3, now - float(self.last_t_s))
        self.last_t_s = now

        cur = math.atan2(self.e, self.n)
        des = math.atan2(we, wn)
        err = _wrap_pi(des - cur)
        sign = self.turn_sign
        if sign is not None and abs(float(sign)) > _EPS:
            s = 1.0 if float(sign) >= 0.0 else -1.0
            unlock = math.radians(max(1.0, float(self.turn_unlock_deg)))
            # Near ±180° wrap can pick the long way; keep the latched side
            # until the remaining error is small.
            if abs(err) > unlock and err * s < 0.0:
                err += s * 2.0 * math.pi

        max_rate = math.radians(max(0.1, float(self.max_rate_deg_s)))
        if drone is not None and self.lat_accel_mps2 > 0.0:
            v = max(float(drone.speed_mps), drone.velocity.horizontal_speed_mps)
            if v > 1.0:
                max_rate = min(max_rate, float(self.lat_accel_mps2) / v)
        step = max(-max_rate * dt, min(max_rate * dt, err))
        ang = cur + step
        self.n = math.cos(ang)
        self.e = math.sin(ang)
        return self.n, self.e


def resolve_realign_turn_sign(
    *,
    ang_x_deg: float | None,
    ang_y_deg: float | None,
    xt_signed_m: float | None,
    config: TrajectoryConfig,
) -> float:
    """Latch realign yaw side: +1 right/CW, -1 left/CCW.

    Side exit (|ang_x| large and ≥ |ang_y|) follows last bbox ang_x.
    Top/bottom or weak ang_x uses signed XT (turn toward the course line).
    If XT is also small, ``realign_turn_default_sign``.
    """
    ax = 0.0
    ay = 0.0
    if ang_x_deg is not None and math.isfinite(float(ang_x_deg)):
        ax = float(ang_x_deg)
    if ang_y_deg is not None and math.isfinite(float(ang_y_deg)):
        ay = float(ang_y_deg)
    lat_min = max(0.0, float(config.realign_turn_lateral_deg))
    if abs(ax) >= lat_min and abs(ax) >= abs(ay):
        return 1.0 if ax > 0.0 else -1.0

    xt_min = max(0.0, float(config.realign_turn_xt_m))
    if xt_signed_m is not None and math.isfinite(float(xt_signed_m)):
        xt = float(xt_signed_m)
        if abs(xt) >= xt_min:
            # Right of course (XT>0) → turn left to intercept.
            return -1.0 if xt > 0.0 else 1.0

    d = float(config.realign_turn_default_sign)
    if d >= 0.0:
        return 1.0
    return -1.0


def realign_dive_arresting(drone: DroneState, config: TrajectoryConfig) -> bool:
    """True when REALIGN must hold heading so GUIDED can arrest a sink first."""
    limit = max(0.0, float(config.realign_arrest_vz_mps))
    if limit <= 0.0:
        return False
    vz_up = -float(drone.velocity.vd_mps)
    return vz_up < -limit


def gate_course_yaw_deg(target: TargetState | None) -> float | None:
    """Live gate course yaw (deg from North toward East), or None if unknown."""
    if target is None:
        return None
    vn = float(target.velocity.vn_mps)
    ve = float(target.velocity.ve_mps)
    if math.hypot(vn, ve) < 0.5:
        return None
    return math.degrees(math.atan2(ve, vn))


def heading_error_to_gate_course_deg(
    drone: DroneState,
    target: TargetState | None,
) -> float:
    """Absolute yaw error (deg) from nose to live gate course; 0 if unknown."""
    want = gate_course_yaw_deg(target)
    if want is None:
        return 0.0
    heading = math.degrees(float(drone.heading_rad))
    return abs(_wrap_180(want - heading))


def realign_brake_ready(
    drone: DroneState,
    target: TargetState | None,
    config: TrajectoryConfig,
    *,
    now_s: float,
    brake_since_s: float | None,
) -> bool:
    """True when GUIDED velocity stop/go can hand off to FOLLOW carrot.

    Stay in BRAKE until slow and not diving. GO along the gate course happens
    inside BRAKE; PATH starts once the nose is roughly on course (or timeout).
    """
    max_s = max(0.5, float(config.realign_brake_max_s))
    elapsed = 0.0 if brake_since_s is None else max(0.0, float(now_s) - float(brake_since_s))
    gs = max(float(drone.speed_mps), drone.velocity.horizontal_speed_mps)
    slow = gs <= float(config.realign_speed_mps) * 1.15
    aligned = heading_error_to_gate_course_deg(drone, target) <= float(
        config.realign_brake_exit_course_deg
    )
    vz_ok = not realign_dive_arresting(drone, config)
    if elapsed >= max(6.0, 2.0 * max_s):
        return True
    if not vz_ok or not slow:
        return False
    return bool(aligned or elapsed >= max_s)


def realign_brake_stopping(drone: DroneState, config: TrajectoryConfig) -> bool:
    """True while BRAKE must command v=0 (still fast or diving)."""
    gs = max(float(drone.speed_mps), drone.velocity.horizontal_speed_mps)
    stop_gs = float(config.realign_speed_mps) * 1.05
    return bool(gs > stop_gs or realign_dive_arresting(drone, config))


def realign_brake_velocity_cmd(
    drone: DroneState,
    output: PlannerOutput,
    config: TrajectoryConfig,
) -> tuple[float, float, float, float | None]:
    """GUIDED NED velocity for REALIGN BRAKE.

    STOP: ``(0, 0, vd)`` — vd is up (negative down) while sinking.
    GO: along live gate course at ``realign_brake_go_mps``, yaw = course.

    Returns ``(vn, ve, vd, yaw_rad_or_none)``.
    """
    vz_up = -float(drone.velocity.vd_mps)
    sink = max(0.0, -vz_up)
    climb = max(0.0, float(config.realign_brake_climb_mps))
    vd = 0.0
    if sink > 0.4 and climb > 0.0:
        vd = -min(climb, 0.4 + sink)

    if realign_brake_stopping(drone, config):
        return 0.0, 0.0, vd, None

    want = gate_course_yaw_deg(output.target)
    if want is None:
        return 0.0, 0.0, vd, None
    yaw = math.radians(want)
    v_go = max(1.0, float(config.realign_brake_go_mps))
    return math.cos(yaw) * v_go, math.sin(yaw) * v_go, vd, yaw


def traj_hold_alt_m(
    gate_alt_m: float,
    config: TrajectoryConfig,
    *,
    range_m: float | None = None,
) -> float:
    """
    GUIDED hold altitude until visual track lock.

    Gate below ``traj_alt_match_above_m`` → fly ``gate + clearance``, but not
    above the match threshold (gate 45 → 50). Gate at/above threshold → gate alt.

    Clearance is ``traj_alt_clearance_m`` far out; at/under
    ``traj_alt_near_range_m`` it drops to ``traj_alt_clearance_near_m``.
    """
    gate_alt = float(gate_alt_m)
    match_above = float(config.traj_alt_match_above_m)
    far_clearance = max(0.0, float(config.traj_alt_clearance_m))
    near_clearance = max(0.0, float(config.traj_alt_clearance_near_m))
    near_range = max(0.0, float(config.traj_alt_near_range_m))
    if gate_alt >= match_above:
        return gate_alt
    clearance = far_clearance
    if (
        near_range > 0.0
        and range_m is not None
        and math.isfinite(float(range_m))
        and float(range_m) <= near_range
    ):
        clearance = near_clearance
    return min(gate_alt + clearance, match_above)


def gate_sample_to_radar(
    sample: GateGpsSample,
    *,
    timestamp_s: float,
) -> RadarMeasurement:
    return RadarMeasurement(
        timestamp_s=float(timestamp_s),
        latitude_deg=float(sample.lat),
        longitude_deg=float(sample.lon),
        altitude_up_m=float(sample.alt),
    )


def drone_dict_to_state(state: dict[str, Any], *, timestamp_s: float) -> DroneState:
    pos = state.get("pos_ned")
    vel = state.get("vel_ned")
    if pos is None:
        pos = (0.0, 0.0, 0.0)
    if vel is None:
        vel = (0.0, 0.0, 0.0)
    att = state.get("att_rad")
    yaw = float(att[2]) if att is not None else 0.0
    gs = float(state.get("groundspeed_m_s") or 0.0)
    if gs <= 0.0:
        gs = float(math.hypot(float(vel[0]), float(vel[1])))
    return DroneState(
        position=NedPosition(float(pos[0]), float(pos[1]), float(pos[2])),
        velocity=NedVelocity(float(vel[0]), float(vel[1]), float(vel[2])),
        timestamp_s=float(timestamp_s),
        speed_mps=gs,
        heading_rad=yaw,
    )


def active_waypoint(
    path: Path,
    drone_pos: NedPosition,
    *,
    flyby_radius_m: float = 120.0,
) -> Optional[Waypoint]:
    """
    Next waypoint not yet reached; intermediates use fly-by radius.

    Switching early (≫ stop radius) keeps GUIDED from hard-braking into a
    join WP and then yaw-slamming toward the final.
    """
    if not path.waypoints:
        return None
    flyby = max(5.0, float(flyby_radius_m))
    for wp in path.waypoints[:-1]:
        if distance_m(drone_pos, wp.position) > flyby:
            return wp
    return path.waypoints[-1]


def _remaining_polyline(
    path: Path,
    drone_pos: NedPosition,
    *,
    flyby_radius_m: float,
) -> list[NedPosition]:
    """Drone → remaining WPs (skip intermediates inside fly-by)."""
    if not path.waypoints:
        return []
    flyby = max(5.0, float(flyby_radius_m))
    pts: list[NedPosition] = [drone_pos]
    for i, wp in enumerate(path.waypoints):
        if i < len(path.waypoints) - 1 and distance_m(drone_pos, wp.position) <= flyby:
            continue
        pts.append(wp.position)
    return pts


def path_remaining_length_m(
    path: Path,
    drone_pos: NedPosition,
    *,
    flyby_radius_m: float = 120.0,
) -> float:
    """Horizontal path length from drone along remaining waypoints."""
    pts = _remaining_polyline(path, drone_pos, flyby_radius_m=flyby_radius_m)
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        total += distance_m(a, b)
    return float(total)


def direction_carrot_setpoint(
    drone_pos: NedPosition,
    dir_n: float,
    dir_e: float,
    *,
    lookahead_m: float,
    down_m: float | None = None,
) -> NedPosition:
    """GUIDED aim a fixed lookahead along a unit NE direction (never a stop WP)."""
    look = max(1.0, float(lookahead_m))
    un, ue = _unit_ne(dir_n, dir_e)
    z = float(drone_pos.down_m if down_m is None else down_m)
    return NedPosition(
        drone_pos.north_m + look * un,
        drone_pos.east_m + look * ue,
        z,
    )


def fly_aim_direction_ne(
    drone_pos: NedPosition,
    gate_pos: NedPosition,
    ref_pos: NedPosition | None,
    *,
    capture_m: float = 80.0,
    max_offset_from_los_deg: float = 25.0,
) -> tuple[float, float]:
    """
    Unit NE aim for the rabbit carrot.

    Prefer engagement ``ref`` while it is still ahead of the drone and not too
    far off the live gate LOS. Once the ref is captured / behind, or missing,
    aim straight at the gate. Result is always forward toward the gate half-plane
    when possible (no reverse WP).
    """
    los_n = gate_pos.north_m - drone_pos.north_m
    los_e = gate_pos.east_m - drone_pos.east_m
    los_un, los_ue = _unit_ne(los_n, los_e)
    if ref_pos is None:
        return los_un, los_ue

    rn = ref_pos.north_m - drone_pos.north_m
    re = ref_pos.east_m - drone_pos.east_m
    r_dist = math.hypot(rn, re)
    # Past / on engagement: fly gate, never reverse to a behind-ref.
    if r_dist <= max(1.0, float(capture_m)) or (rn * los_un + re * los_ue) <= 0.0:
        return los_un, los_ue

    ru_n, ru_e = _unit_ne(rn, re)
    # Keep ref aim inside a cone around gate LOS (avoids early join snap).
    max_off = math.radians(max(0.0, float(max_offset_from_los_deg)))
    if max_off > 0.0:
        dot = max(-1.0, min(1.0, ru_n * los_un + ru_e * los_ue))
        ang = math.acos(dot)
        if ang > max_off + _EPS:
            # Rotate LOS toward ref by at most max_off.
            cross = los_un * ru_e - los_ue * ru_n
            sign = 1.0 if cross >= 0.0 else -1.0
            ca, sa = math.cos(sign * max_off), math.sin(sign * max_off)
            return (
                los_un * ca - los_ue * sa,
                los_un * sa + los_ue * ca,
            )
    return ru_n, ru_e


def follow_centerline_seat_ned(
    drone_pos: NedPosition,
    gate_pos: NedPosition,
    target: TargetState,
    *,
    course_aim_xt_m: float = 12.0,
    min_ahead_m: float = 40.0,
    xt_null_lookahead_m: float = 80.0,
) -> NedPosition | None:
    """
    On-course GUIDED setpoint for FOLLOW XT-null (point *on the line*).

    Returns None when |XT| is already within ``course_aim_xt_m`` (caller
    should fly along û / gate). Lookahead shrinks with XT so the seat point
    stays near enough for a steep approach — direction-carrots along û were
    freezing a parallel offset (traj_diag_20260915_150955, XT≈39 m).
    """
    vn = float(target.velocity.vn_mps)
    ve = float(target.velocity.ve_mps)
    sp = math.hypot(vn, ve)
    if sp <= _EPS:
        return None
    un, ue = vn / sp, ve / sp
    dn = drone_pos.north_m - gate_pos.north_m
    de = drone_pos.east_m - gate_pos.east_m
    xt = abs(dn * ue - de * un)
    along = dn * un + de * ue
    xt_lim = max(0.0, float(course_aim_xt_m))
    if xt <= xt_lim:
        return None
    min_ahead = max(1.0, float(min_ahead_m))
    look_cap = max(min_ahead, float(xt_null_lookahead_m), 30.0)
    # Prefer ≥45° approach: look ≤ XT (floored for forward progress).
    look = max(20.0, min(look_cap, float(xt)))
    if along > 1.0:
        aim_along = max(0.0, along - look)
    elif along < -1.0:
        aim_along = along + look
    else:
        aim_along = 0.0
    if aim_along < along + 20.0:
        aim_along = along + 20.0
    return NedPosition(
        gate_pos.north_m + aim_along * un,
        gate_pos.east_m + aim_along * ue,
        float(gate_pos.down_m),
    )


def follow_aim_direction_ne(
    drone_pos: NedPosition,
    gate_pos: NedPosition,
    ref_pos: NedPosition | None,
    target: TargetState,
    *,
    course_aim_xt_m: float = 12.0,
    standoff_m: float = 50.0,
    min_ahead_m: float = 40.0,
    capture_m: float = 80.0,
    max_offset_from_los_deg: float = 25.0,
    join_pos: NedPosition | None = None,
    xt_null_lookahead_m: float = 80.0,
) -> tuple[float, float]:
    """
    FOLLOW rabbit aim — always on the course line and ahead of the drone.

    - XT large: null cross-track like INTERCEPT — aim at a *nearby* on-line
      point ahead of the drone. Lookahead shrinks with XT so the approach
      angle stays steep (~30°) — a fixed 80 m look at XT≈16° yielded a
      shallow parallel cruise (traj_diag_20260915_144617).
    - Huge XT: may use join WP only if it lies ahead of the drone along û
      and toward the gate (never a behind WP → no spin / reverse carrot).
    - XT small and behind: aim along target course û.
    - Ahead on course: gate LOS.

    ``standoff_m`` is retained for API compatibility; trail station is a path
    final hint, not the large-XT aim target.
    """
    del standoff_m  # path/final geometry; large-XT aim uses nearby centerline
    vn = float(target.velocity.vn_mps)
    ve = float(target.velocity.ve_mps)
    sp = math.hypot(vn, ve)
    xt_lim = max(0.0, float(course_aim_xt_m))
    min_ahead = max(1.0, float(min_ahead_m))
    look_cap = max(min_ahead, float(xt_null_lookahead_m), 30.0)

    if sp > _EPS:
        un, ue = vn / sp, ve / sp
        dn = drone_pos.north_m - gate_pos.north_m
        de = drone_pos.east_m - gate_pos.east_m
        xt_signed = dn * ue - de * un
        xt = abs(xt_signed)
        along = dn * un + de * ue

        if xt > xt_lim:
            if join_pos is not None and xt > max(200.0, 10.0 * xt_lim):
                jn = join_pos.north_m - drone_pos.north_m
                je = join_pos.east_m - drone_pos.east_m
                along_join = jn * un + je * ue
                los_n = gate_pos.north_m - drone_pos.north_m
                los_e = gate_pos.east_m - drone_pos.east_m
                if (
                    along_join > min_ahead
                    and jn * los_n + je * los_e > 0.0
                    and math.hypot(jn, je) > _EPS
                ):
                    return _unit_ne(jn, je)
            # Steep XT-null: look ≈ XT / tan(30°) so residual offset does not
            # collapse into a parallel asymptote.
            adapt = float(xt) / max(math.tan(math.radians(30.0)), 1e-6)
            look = max(min_ahead, min(look_cap, adapt))
            if along > 1.0:
                aim_along = max(0.0, along - look)
            elif along < -1.0:
                aim_along = along + look
            else:
                aim_along = 0.0
            if aim_along < along + min_ahead:
                aim_along = along + min_ahead
            center_n = gate_pos.north_m + aim_along * un
            center_e = gate_pos.east_m + aim_along * ue
            return _unit_ne(
                center_n - drone_pos.north_m,
                center_e - drone_pos.east_m,
            )
        if along <= 0.0:
            return un, ue
        return fly_aim_direction_ne(
            drone_pos,
            gate_pos,
            None,
            capture_m=capture_m,
            max_offset_from_los_deg=max_offset_from_los_deg,
        )

    aim_ref = join_pos if join_pos is not None else ref_pos
    return fly_aim_direction_ne(
        drone_pos,
        gate_pos,
        aim_ref,
        capture_m=capture_m,
        max_offset_from_los_deg=0.0,
    )


def intercept_aim_direction_ne(
    drone_pos: NedPosition,
    gate_pos: NedPosition,
    ref_pos: NedPosition | None,
    target: TargetState | None,
    *,
    align_xt_m: float = 15.0,
    capture_m: float = 80.0,
    max_offset_from_los_deg: float = 25.0,
    lookahead_m: float = 80.0,
) -> tuple[float, float]:
    """
    INTERCEPT rabbit aim: null cross-track on the course line, then gate.

    While |XT| is large, aim at a live centerline point (not pure LOS, which
    under-corrects laterally on a nearly head-on geometry). Once seated, aim
    at the gate / engagement ref for punch-through.
    """
    if target is None:
        return fly_aim_direction_ne(
            drone_pos,
            gate_pos,
            ref_pos,
            capture_m=capture_m,
            max_offset_from_los_deg=max_offset_from_los_deg,
        )
    vn = float(target.velocity.vn_mps)
    ve = float(target.velocity.ve_mps)
    sp = math.hypot(vn, ve)
    if sp <= _EPS:
        return fly_aim_direction_ne(
            drone_pos,
            gate_pos,
            ref_pos,
            capture_m=capture_m,
            max_offset_from_los_deg=max_offset_from_los_deg,
        )
    un, ue = vn / sp, ve / sp
    dn = drone_pos.north_m - gate_pos.north_m
    de = drone_pos.east_m - gate_pos.east_m
    xt = abs(dn * ue - de * un)
    along = dn * un + de * ue
    align = max(0.0, float(align_xt_m))
    if xt > align:
        look = max(30.0, float(lookahead_m))
        if along > 1.0:
            aim_along = max(0.0, along - look)
        elif along < -1.0:
            aim_along = along + look
        else:
            aim_along = 0.0
        cn = gate_pos.north_m + aim_along * un - drone_pos.north_m
        ce = gate_pos.east_m + aim_along * ue - drone_pos.east_m
        if math.hypot(cn, ce) > _EPS:
            return _unit_ne(cn, ce)
    return fly_aim_direction_ne(
        drone_pos,
        gate_pos,
        ref_pos,
        capture_m=capture_m,
        max_offset_from_los_deg=max_offset_from_los_deg,
    )


def path_carrot_setpoint(
    path: Path,
    drone_pos: NedPosition,
    *,
    lookahead_m: float,
    flyby_radius_m: float = 120.0,
    gate_pos: NedPosition | None = None,
    capture_m: float = 80.0,
    max_offset_from_los_deg: float = 25.0,
) -> Optional[NedPosition]:
    """
    Sliding GUIDED aim ``lookahead_m`` ahead.

    Prefer gate-aware fly-aim (engagement ref while ahead, else live gate) so
    the setpoint is never a reachable stop and never behind the drone toward
    the gate. Falls back to polyline walk when no gate is provided.
    """
    look = max(1.0, float(lookahead_m))
    if gate_pos is not None:
        ref = path.final_position
        if ref is None and path.waypoints:
            ref = path.waypoints[-1].position
        dn, de = fly_aim_direction_ne(
            drone_pos,
            gate_pos,
            ref,
            capture_m=capture_m,
            max_offset_from_los_deg=max_offset_from_los_deg,
        )
        return direction_carrot_setpoint(
            drone_pos, dn, de, lookahead_m=look, down_m=gate_pos.down_m
        )

    pts = _remaining_polyline(path, drone_pos, flyby_radius_m=flyby_radius_m)
    if len(pts) < 2:
        base = path.waypoints[-1].position if path.waypoints else drone_pos
        return NedPosition(base.north_m + look, base.east_m, base.down_m)

    remaining = look
    last_dir_n, last_dir_e = 1.0, 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        dn = b.north_m - a.north_m
        de = b.east_m - a.east_m
        seg = math.hypot(dn, de)
        if seg > _EPS:
            last_dir_n, last_dir_e = dn / seg, de / seg
        if seg >= remaining - _EPS:
            if seg <= _EPS:
                return NedPosition(b.north_m, b.east_m, b.down_m)
            frac = remaining / seg
            return NedPosition(
                a.north_m + frac * dn,
                a.east_m + frac * de,
                b.down_m,
            )
        remaining -= seg

    final = pts[-1]
    if math.hypot(last_dir_n, last_dir_e) <= _EPS:
        last_dir_n, last_dir_e = 1.0, 0.0
    return NedPosition(
        final.north_m + remaining * last_dir_n,
        final.east_m + remaining * last_dir_e,
        final.down_m,
    )


def _lerp_speed(cruise: float, floor: float, dist_m: float, taper_m: float, end_m: float) -> float:
    """Linearly taper cruise→floor as distance goes from taper_m down to end_m."""
    span = max(1.0, float(taper_m) - float(end_m))
    alpha = (float(dist_m) - float(end_m)) / span
    alpha = min(1.0, max(0.0, alpha))
    return float(floor) + alpha * (float(cruise) - float(floor))


def speed_for_gate_range_m(range_m: float, config: TrajectoryConfig) -> float:
    """
    Peak (max_speed) until ``speed_decel_start_range_m``, then ramp to
    close_approach by ``deadline_range_m`` (e.g. 1000→30 … 400→10).
    """
    peak = float(config.max_speed_mps)
    close = float(config.close_approach_speed_mps)
    start = float(config.speed_decel_start_range_m)
    end = float(config.deadline_range_m)
    if start < end:
        start = end
    r = max(0.0, float(range_m))
    if r >= start:
        return peak
    if r <= end:
        return close
    return _lerp_speed(peak, close, r, start, end)


def _cross_track_to_target_course_m(drone: DroneState, target: TargetState) -> float:
    """Horizontal distance from drone to the infinite line of target course."""
    vn = float(target.velocity.vn_mps)
    ve = float(target.velocity.ve_mps)
    speed = math.hypot(vn, ve)
    if speed < _EPS:
        return distance_m(drone.position, target.position)
    un, ue = vn / speed, ve / speed
    dn = drone.position.north_m - target.position.north_m
    de = drone.position.east_m - target.position.east_m
    return abs(dn * ue - de * un)


def _gate_horizontal_speed_mps(target: TargetState | None) -> float:
    if target is None:
        return 0.0
    return float(
        math.hypot(float(target.velocity.vn_mps), float(target.velocity.ve_mps))
    )


def closing_speed_mps(drone: DroneState, target: TargetState) -> float:
    """Positive when range is shrinking (m/s); negative when opening."""
    dn = float(target.position.north_m) - float(drone.position.north_m)
    de = float(target.position.east_m) - float(drone.position.east_m)
    dist = math.hypot(dn, de)
    if dist <= _EPS:
        return 0.0
    un, ue = dn / dist, de / dist
    rv_n = float(target.velocity.vn_mps) - float(drone.velocity.vn_mps)
    rv_e = float(target.velocity.ve_mps) - float(drone.velocity.ve_mps)
    # d(range)/dt = û · v_rel; closing = −range_rate.
    return float(-(rv_n * un + rv_e * ue))


@dataclass
class CatchupSpeedLatch:
    """Hysteresis for INTERCEPT opening-range catch-up speed floor."""

    active: bool = False
    since_s: float | None = None
    last_open_s: float | None = None

    def reset(self) -> None:
        self.active = False
        self.since_s = None
        self.last_open_s = None

    def update(
        self,
        closing_mps: float,
        *,
        range_m: float,
        config: TrajectoryConfig,
        now_s: float,
    ) -> bool:
        lead = float(config.catchup_lead_mps)
        if lead <= 0.0:
            self.reset()
            return False

        min_r = max(0.0, float(config.catchup_min_range_m))
        if float(range_m) < min_r:
            # Inside visual / near band: drop latch so schedule can soft-land.
            self.reset()
            return False

        enter = float(config.catchup_enter_closing_mps)
        exit_c = float(config.catchup_exit_closing_mps)
        if exit_c < enter:
            exit_c = enter
        hold = max(0.0, float(config.catchup_hold_s))
        now = float(now_s)
        closing = float(closing_mps)

        if closing <= enter:
            if not self.active:
                self.since_s = now
            self.active = True
            self.last_open_s = now
            return True

        if not self.active:
            return False

        elapsed = 0.0 if self.since_s is None else max(0.0, now - float(self.since_s))
        last_open = self.last_open_s
        quiet = 0.0 if last_open is None else max(0.0, now - float(last_open))
        # Clear only after sustained closing and minimum latch time.
        if closing >= exit_c and elapsed >= hold and quiet >= hold:
            self.reset()
            return False
        return True


def follow_speed_for_output(
    output: PlannerOutput,
    *,
    config: TrajectoryConfig,
) -> float:
    """
    FOLLOW speed: far JOIN capped, near trail matched to gate + lead.

    Catch-up: while still far behind the soft standoff or off-line, allow
    peak so we can seat on the line. Once near station and on-line, match
    gate + lead (adapts if the gate is fast).
    """
    peak = float(config.max_speed_mps)
    gate_sp = _gate_horizontal_speed_mps(output.target)
    lead = max(0.0, float(config.follow_trail_match_lead_mps))
    match = min(peak, gate_sp + lead)
    # Almost-stopped gate: still allow a crawl, not a freeze at 0.
    if match < 1.0:
        match = min(peak, max(1.0, lead))

    join_cap = float(config.follow_align_speed_mps)
    if join_cap <= 0.0:
        join_cap = peak
    join_cap = min(peak, join_cap)

    xt = float(output.xt_m)
    along = float(output.along_track_m)
    station = max(1.0, float(config.follow_standoff_m))
    align_xt = max(0.0, float(config.follow_align_xt_m))
    need_catch_up = xt > align_xt or along < -station

    if output.follow_phase is FollowPhase.CHASE:
        if need_catch_up:
            return float(peak)
        return float(min(peak, max(match, gate_sp + lead)))

    # JOIN / ALIGN / unknown FOLLOW phase.
    speed = join_cap
    brake_xt = max(0.0, float(config.follow_trail_brake_xt_m))
    brake_along = max(
        float(config.follow_standoff_m),
        float(config.follow_trail_brake_along_m),
    )

    if brake_xt > 0.0 and xt <= brake_xt and along > -brake_along:
        behind = max(0.0, -along)
        if behind >= station:
            # behind: brake_along → station maps join_cap → match.
            speed = _lerp_speed(join_cap, match, behind, brake_along, station)
        elif along <= 0.0:
            # Inside standoff: match gate+lead. Never peak while XT is still
            # open — that raced past the gate at ~16 m offset (diag 144617).
            speed = match
        else:
            # Ahead of gate: do not keep pulling away — ≤ gate speed.
            speed = min(match, max(gate_sp, 1.0))
    return float(min(peak, speed))


def desired_speed_for_output(
    output: PlannerOutput,
    *,
    drone: DroneState,
    config: TrajectoryConfig,
    catchup: CatchupSpeedLatch | None = None,
    now_s: float | None = None,
) -> float:
    """
    Raw planner speed target before slew limiting.

    INTERCEPT: gate-range schedule (peak → close_approach by deadline_range).
    Off-line XT does not skip that taper — only the far XT boost
    (``speed_decel_max_xt_m`` at ``speed_xt_boost_min_range_m``) can hold peak.
    When range is opening (catch-up latch), floor at gate_speed + lead.
    FOLLOW: far JOIN cap, then gate-adaptive trail match (see
    ``follow_speed_for_output``). The 400 m → 10 m/s taper is INTERCEPT-only.
    """
    range_m = float(output.distance_to_target_m)
    peak = float(config.max_speed_mps)
    follow = output.behavior is BehaviorMode.FOLLOW

    if follow:
        speed = follow_speed_for_output(output, config=config)
        if catchup is not None:
            catchup.reset()
    else:
        speed = speed_for_gate_range_m(range_m, config)
        # Far + badly off-line: keep peak to join before the decel band.
        # Inside speed_decel_start_range_m the range schedule always wins.
        xt_lim = float(config.speed_decel_max_xt_m)
        xt_min_range = float(config.speed_xt_boost_min_range_m)
        if (
            xt_lim > 0.0
            and output.target is not None
            and range_m >= xt_min_range
        ):
            xt = _cross_track_to_target_course_m(drone, output.target)
            if xt >= xt_lim:
                speed = peak

        if (
            catchup is not None
            and output.target is not None
            and not output.realign_active
        ):
            now = float(time.monotonic() if now_s is None else now_s)
            closing = closing_speed_mps(drone, output.target)
            if catchup.update(
                closing,
                range_m=range_m,
                config=config,
                now_s=now,
            ):
                gate_sp = _gate_horizontal_speed_mps(output.target)
                lead = max(0.0, float(config.catchup_lead_mps))
                floor = min(peak, gate_sp + lead)
                speed = max(speed, floor)

    carrot_on = float(config.path_carrot_lookahead_m) > 0.0
    path = output.path
    if path is not None and path.waypoints:
        flyby = float(config.wp_flyby_radius_m)
        if carrot_on:
            end_taper = float(config.path_carrot_end_taper_m)
            # End taper is INTERCEPT-oriented; skip for FOLLOW.
            if end_taper > 0.0 and not follow:
                rem = path_remaining_length_m(
                    path, drone.position, flyby_radius_m=flyby
                )
                if rem < end_taper:
                    floor = float(config.close_approach_speed_mps)
                    speed = min(
                        speed,
                        _lerp_speed(peak, floor, rem, end_taper, 0.0),
                    )
        else:
            wp = active_waypoint(path, drone.position, flyby_radius_m=flyby)
            if wp is not None and wp is not path.waypoints[-1]:
                taper = float(config.wp_approach_taper_m)
                approach_floor = float(config.wp_approach_min_speed_mps)
                d_wp = distance_m(drone.position, wp.position)
                if d_wp < taper:
                    join_cap = _lerp_speed(speed, approach_floor, d_wp, taper, flyby)
                    speed = min(speed, max(approach_floor, join_cap))

    if output.realign_active:
        realign_cap = float(config.realign_speed_mps)
        if realign_cap > 0.0:
            speed = min(speed, realign_cap)
        if catchup is not None:
            catchup.reset()

    return float(speed)


def apply_planner_output(
    env: GazeboDroneTrackingEnv,
    output: PlannerOutput,
    *,
    gate: GateGpsSample,
    drone: DroneState,
    config: TrajectoryConfig,
    last_speed_cmd: float | None = None,
    speed_slew: SpeedCommandSlew | None = None,
    pos_hold: PositionSetpointHold | None = None,
    dir_slew: CarrotDirSlew | None = None,
    catchup: CatchupSpeedLatch | None = None,
    now_s: float | None = None,
) -> float:
    """
    Send GUIDED setpoints for the current planner output.

    TRACK_PLAN → rabbit carrot (always ahead toward gate / engagement ref)
    + gate-range speed schedule. No path → live gate position (fail-safe).

    REALIGN BRAKE sends GUIDED velocity (v=0 stop, then along gate course)
    and does not emit carrot / DO_CHANGE_SPEED until PATH.

    When ``speed_slew`` is provided, DO_CHANGE_SPEED is ramped (~WP_ACC).
    When ``dir_slew`` is provided, aim heading is rate-limited.
    When ``pos_hold`` is provided, small carrot jitters are ignored.

    Returns the commanded horizontal speed actually sent (after slew).
    """
    del last_speed_cmd  # kept for call-site compatibility
    if (
        output.realign_active
        and output.realign_phase is RealignPhase.BRAKE
    ):
        if catchup is not None:
            catchup.reset()
        vn, ve, vd, yaw = realign_brake_velocity_cmd(drone, output, config)
        env.send_velocity_ned(
            vn_mps=vn,
            ve_mps=ve,
            vd_mps=vd,
            yaw_rad=yaw,
        )
        return float(math.hypot(vn, ve))

    state = env.get_drone_state()
    target_speed = desired_speed_for_output(
        output,
        drone=drone,
        config=config,
        catchup=catchup,
        now_s=now_s,
    )
    actual_gs = float(drone.speed_mps)

    if speed_slew is not None:
        speed = speed_slew.step(target_speed, now_s=now_s, actual_mps=actual_gs)
    else:
        speed = target_speed

    env.set_guided_horizontal_speed_mps(
        speed,
        min_interval_s=float(config.speed_cmd_min_interval_s),
        min_delta_mps=float(config.speed_cmd_min_delta_mps),
    )

    path = output.path
    look = float(config.path_carrot_lookahead_m)
    if output.realign_active:
        rl = float(config.realign_carrot_lookahead_m)
        if rl > 0.0:
            look = rl
    cmd_pos: NedPosition | None = None
    aim_n = aim_e = 0.0

    gate_ned: NedPosition | None = None
    if output.target is not None:
        gate_ned = output.target.position
    elif path is not None and path.final_position is not None:
        gate_ned = path.final_position

    saved_retarget: float | None = None
    if pos_hold is not None and output.realign_active:
        saved_retarget = float(pos_hold.min_retarget_m)
        rt = float(config.realign_setpoint_retarget_min_m)
        if rt > 0.0:
            pos_hold.min_retarget_m = rt

    try:
        seat_bypass_hold = False
        if path is not None and path.waypoints and look > 0.0 and gate_ned is not None:
            ref = path.final_position or path.waypoints[-1].position
            join = path.waypoints[0].position if len(path.waypoints) >= 2 else None
            max_off = float(config.carrot_max_offset_from_los_deg)
            if output.realign_active:
                max_off = min(max_off, float(config.realign_max_offset_from_los_deg))
                ref = None
            follow_seat: NedPosition | None = None
            if output.behavior is BehaviorMode.FOLLOW and output.target is not None:
                # JOIN: steer at intermediate; on-trail: course û / final.
                aim_ref = (
                    join
                    if (
                        join is not None
                        and output.follow_phase in (FollowPhase.JOIN, FollowPhase.ALIGN)
                        and not output.realign_active
                    )
                    else ref
                )
                if not output.realign_active:
                    follow_seat = follow_centerline_seat_ned(
                        drone.position,
                        gate_ned,
                        output.target,
                        course_aim_xt_m=float(config.follow_course_aim_xt_m),
                        min_ahead_m=float(config.follow_aim_min_ahead_m),
                        xt_null_lookahead_m=max(
                            80.0, float(config.path_carrot_lookahead_m) * 0.3
                        ),
                    )
                if follow_seat is not None:
                    # Point *on the line* — not a û-direction rabbit (that
                    # froze XT≈39 m parallel for ~100 s in diag 150955).
                    aim_n, aim_e = _unit_ne(
                        follow_seat.north_m - drone.position.north_m,
                        follow_seat.east_m - drone.position.east_m,
                    )
                else:
                    aim_n, aim_e = follow_aim_direction_ne(
                        drone.position,
                        gate_ned,
                        aim_ref,
                        output.target,
                        course_aim_xt_m=float(config.follow_course_aim_xt_m),
                        standoff_m=float(config.follow_standoff_m),
                        min_ahead_m=float(config.follow_aim_min_ahead_m),
                        capture_m=float(config.carrot_ref_capture_m),
                        max_offset_from_los_deg=max_off,
                        join_pos=None if output.realign_active else join,
                        xt_null_lookahead_m=max(
                            80.0, float(config.path_carrot_lookahead_m) * 0.3
                        ),
                    )
            else:
                aim_n, aim_e = intercept_aim_direction_ne(
                    drone.position,
                    gate_ned,
                    ref,
                    output.target,
                    align_xt_m=float(config.intercept_align_xt_m),
                    capture_m=float(config.carrot_ref_capture_m),
                    max_offset_from_los_deg=max_off,
                    lookahead_m=max(
                        80.0, float(config.path_carrot_lookahead_m) * 0.3
                    ),
                )
            arrest = False
            if output.realign_active:
                arrest = realign_dive_arresting(drone, config)
                output.realign_arrest = arrest
                if arrest:
                    vn = float(drone.velocity.vn_mps)
                    ve = float(drone.velocity.ve_mps)
                    if math.hypot(vn, ve) > _EPS:
                        aim_n, aim_e = _unit_ne(vn, ve)
            if dir_slew is not None and follow_seat is None:
                saved_rate = float(dir_slew.max_rate_deg_s)
                saved_sign = dir_slew.turn_sign
                saved_lat = float(dir_slew.lat_accel_mps2)
                if output.realign_active:
                    dir_slew.max_rate_deg_s = max(
                        saved_rate, float(config.realign_carrot_rate_deg_s),
                    )
                    lat_rl = float(config.realign_carrot_lat_accel_mps2)
                    if lat_rl > 0.0:
                        dir_slew.lat_accel_mps2 = lat_rl
                    ts = float(output.realign_turn_sign)
                    dir_slew.turn_sign = None if arrest else (
                        ts if abs(ts) > _EPS else None
                    )
                else:
                    dir_slew.turn_sign = None
                try:
                    aim_n, aim_e = dir_slew.step(
                        aim_n, aim_e, now_s=now_s, drone=drone,
                    )
                finally:
                    dir_slew.max_rate_deg_s = saved_rate
                    dir_slew.turn_sign = saved_sign
                    dir_slew.lat_accel_mps2 = saved_lat
            elif dir_slew is not None and follow_seat is not None:
                # Snap slew state to the seat bearing so later û-chase is clean.
                dir_slew.n, dir_slew.e = float(aim_n), float(aim_e)
                dir_slew._seeded = True
                dir_slew.turn_sign = None
            output.carrot_dir_n = float(aim_n)
            output.carrot_dir_e = float(aim_e)
            if follow_seat is not None:
                cmd_pos = follow_seat
                seat_bypass_hold = True
            else:
                cmd_pos = direction_carrot_setpoint(
                    drone.position,
                    aim_n,
                    aim_e,
                    lookahead_m=look,
                    down_m=gate_ned.down_m,
                )
        elif path is not None and path.waypoints and look > 0.0:
            cmd_pos = path_carrot_setpoint(
                path,
                drone.position,
                lookahead_m=look,
                flyby_radius_m=float(config.wp_flyby_radius_m),
            )
        elif path is not None and path.waypoints:
            wp = active_waypoint(
                path,
                drone.position,
                flyby_radius_m=float(config.wp_flyby_radius_m),
            )
            if wp is not None:
                cmd_pos = wp.position

        range_m = float(output.distance_to_target_m)

        if cmd_pos is None:
            if not state.get("global_valid"):
                return float(speed)
            lat = float(gate.lat)
            lon = float(gate.lon)
            alt_cmd = traj_hold_alt_m(float(gate.alt), config, range_m=range_m)
            bearing = bearing_ned_rad(
                float(state["lat_deg"]),
                float(state["lon_deg"]),
                lat,
                lon,
            )
            env.send_position_target_global(
                lat_deg=lat,
                lon_deg=lon,
                alt_rel_m=alt_cmd,
                yaw_rad=bearing,
            )
            return speed

        if pos_hold is not None:
            if seat_bypass_hold and cmd_pos is not None:
                # Always retarget onto the live centerline seat.
                pos_hold.last = cmd_pos
            else:
                cmd_pos = pos_hold.hold(cmd_pos)

        lat, lon, _alt = local_to_latlonalt(
            cmd_pos,
            home_lat_deg=config.home_lat_deg,
            home_lon_deg=config.home_lon_deg,
            home_alt_m=config.home_alt_m,
        )
        alt_cmd = traj_hold_alt_m(float(gate.alt), config, range_m=range_m)
        yaw = None
        if state.get("global_valid"):
            yaw = bearing_ned_rad(
                float(state["lat_deg"]),
                float(state["lon_deg"]),
                lat,
                lon,
            )
        env.send_position_target_global(
            lat_deg=lat,
            lon_deg=lon,
            alt_rel_m=alt_cmd,
            yaw_rad=yaw,
        )
        return speed
    finally:
        if pos_hold is not None and saved_retarget is not None:
            pos_hold.min_retarget_m = saved_retarget
