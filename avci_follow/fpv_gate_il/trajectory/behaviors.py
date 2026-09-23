"""INTERCEPT vs FOLLOW candidate generation and cost selection.

Goal: reach engagement *quickly*.
  INTERCEPT — get ahead on course (head-on) when the target is closing / low aspect.
  FOLLOW    — join behind when aspect is high (crossing) so ahead would be a long detour.

Do not optimize for shortest path or slowest cruise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from fpv_gate_il.trajectory.config import CostWeights, TrajectoryConfig
from fpv_gate_il.trajectory.coords import distance_m, horizontal_distance_m
from fpv_gate_il.trajectory.path_planner import PathPlanner
from fpv_gate_il.trajectory.types import (
    BehaviorCandidate,
    BehaviorMode,
    DroneState,
    NedPosition,
    Path,
    TargetState,
)

_EPS = 1e-9
_MIN_SPEED_FOR_COURSE = 0.5  # m/s — bare minimum to prefer velocity over LOS
_LATERAL_JOIN_M = 15.0  # join course line if lateral offset exceeds this


def target_course_reliable(target: TargetState, min_speed_mps: float) -> bool:
    """True when horizontal target speed is trusted for INTERCEPT/FOLLOW."""
    v = target.velocity.as_array()
    return float(math.hypot(v[0], v[1])) >= float(min_speed_mps)


def _horizontal_unit(vec: np.ndarray) -> np.ndarray:
    h = np.array([float(vec[0]), float(vec[1]), 0.0], dtype=float)
    n = float(np.linalg.norm(h))
    if n <= _EPS:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return h / n


def course_unit(
    target: TargetState,
    drone: DroneState,
    p_t: NedPosition,
) -> np.ndarray:
    """Unit course in horizontal NED (toward target motion, or drone→target)."""
    v = target.velocity.as_array()
    speed = float(math.hypot(v[0], v[1]))
    if speed >= _MIN_SPEED_FOR_COURSE:
        return _horizontal_unit(v)
    return _horizontal_unit(p_t.as_array() - drone.position.as_array())


def predict_target_position(target: TargetState, t_s: float) -> NedPosition:
    p = target.position.as_array() + target.velocity.as_array() * float(t_s)
    return NedPosition.from_array(p)


def engagement_horizon_s(config: TrajectoryConfig, t_400_s: float) -> float:
    """Clamp CV prediction used for engagement geometry."""
    raw = max(0.0, float(t_400_s))
    return min(raw, float(config.engagement_predict_horizon_s))


def closing_aspect_deg(target: TargetState, drone: DroneState) -> float:
    """
    Aspect between target course and direction toward the drone.

    0°  = course points at the drone (head-on closing).
    90° = pure cross / abeam.
    """
    v = target.velocity.as_array()
    if float(math.hypot(v[0], v[1])) < _MIN_SPEED_FOR_COURSE:
        return 0.0
    u = _horizontal_unit(v)
    to_drone = drone.position.as_array() - target.position.as_array()
    to_drone[2] = 0.0
    n = float(np.linalg.norm(to_drone))
    if n <= _EPS:
        return 0.0
    to_drone = to_drone / n
    c = float(np.clip(np.dot(u, to_drone), -1.0, 1.0))
    return float(math.degrees(math.acos(c)))


def target_course_unit_ne(target: TargetState) -> tuple[float, float] | None:
    """Horizontal unit course from target velocity, or None if too slow."""
    vn = float(target.velocity.vn_mps)
    ve = float(target.velocity.ve_mps)
    speed = math.hypot(vn, ve)
    if speed < _MIN_SPEED_FOR_COURSE:
        return None
    return vn / speed, ve / speed


def signed_cross_track_to_course_m(drone: DroneState, target: TargetState) -> float:
    """Signed XT to the infinite target-course line (m). Right of û is positive."""
    u = target_course_unit_ne(target)
    if u is None:
        return 0.0
    un, ue = u
    dn = drone.position.north_m - target.position.north_m
    de = drone.position.east_m - target.position.east_m
    return float(un * de - ue * dn)


def cross_track_to_course_m(drone: DroneState, target: TargetState) -> float:
    """Horizontal distance from drone to the infinite target-course line."""
    u = target_course_unit_ne(target)
    if u is None:
        return distance_m(drone.position, target.position)
    return abs(signed_cross_track_to_course_m(drone, target))


def along_track_m(drone: DroneState, target: TargetState) -> float:
    """
    Signed along-track of drone relative to gate (m).

    Positive = ahead of gate on course; negative = behind (FOLLOW station).
    """
    return point_along_track_m(drone.position, target)


def point_along_track_m(pos: NedPosition, target: TargetState) -> float:
    """Signed along-track of ``pos`` relative to the live gate (m)."""
    u = target_course_unit_ne(target)
    if u is None:
        return 0.0
    un, ue = u
    dn = pos.north_m - target.position.north_m
    de = pos.east_m - target.position.east_m
    return float(dn * un + de * ue)


def course_error_deg(drone: DroneState, target: TargetState) -> float:
    """Absolute heading error between drone ground track and target course."""
    u = target_course_unit_ne(target)
    if u is None:
        return 180.0
    un, ue = u
    if drone.speed_mps > _MIN_SPEED_FOR_COURSE or drone.velocity.horizontal_speed_mps > _MIN_SPEED_FOR_COURSE:
        dn = float(drone.velocity.vn_mps)
        de = float(drone.velocity.ve_mps)
    else:
        # Hovering: use nose as weak proxy.
        dn = math.cos(float(drone.heading_rad))
        de = math.sin(float(drone.heading_rad))
    n = math.hypot(dn, de)
    if n <= _EPS:
        return 180.0
    dn, de = dn / n, de / n
    c = max(-1.0, min(1.0, dn * un + de * ue))
    return float(math.degrees(math.acos(c)))


def follow_geometry_ok(
    drone: DroneState,
    target: TargetState,
    config: TrajectoryConfig,
) -> tuple[bool, float, float, float]:
    """
    Return (aligned, xt_m, along_track_m, course_err_deg).

    Aligned = on-course, behind the gate, and chasing roughly the same heading.
    Behind-min shrinks with the adaptive trail so a short standoff can still align.
    """
    xt = cross_track_to_course_m(drone, target)
    along = along_track_m(drone, target)
    cerr = course_error_deg(drone, target)
    trail_along = follow_trail_along_m(drone, target, config)
    # Need to sit behind the gate; allow a shorter behind-band when trail shrank.
    nom_behind = float(config.follow_behind_min_m)
    if trail_along < 0.0:
        soft_behind = max(5.0, min(nom_behind, -float(trail_along) * 0.5))
    else:
        soft_behind = max(5.0, float(config.follow_standoff_min_m) * 0.5)
    ok = (
        xt <= float(config.follow_align_xt_m)
        and along <= -soft_behind
        and cerr <= float(config.follow_align_course_deg)
    )
    return ok, float(xt), float(along), float(cerr)


def follow_trail_along_m(
    drone: DroneState,
    target: TargetState,
    config: TrajectoryConfig,
) -> float:
    """
    Signed along-track for the FOLLOW final (relative to live gate).

    Soft nominal trail at ``-follow_standoff_m``. If the drone is already closer,
    shrink so the point stays **ahead of the drone** on the course line (never
    behind the aircraft). Prefers staying just behind the gate when possible.

    When the drone is already ahead of the live gate, keep the classic
    behind-gate trail (``-standoff``). GUIDED carrot aims stay ahead of the
    airframe; the path final still marks the behind-gate station to rejoin.
    """
    along = along_track_m(drone, target)
    nom = max(1.0, float(config.follow_standoff_m))
    min_st = min(nom, max(1.0, float(config.follow_standoff_min_m)))
    min_ahead = max(1.0, float(config.follow_aim_min_ahead_m))

    # Drone ahead of gate: station remains behind the live gate.
    if along > 0.0:
        return float(-nom)

    ideal = -nom
    floor = along + min_ahead  # must stay ahead of drone along û
    trail = max(ideal, floor)
    if trail > 0.0 and along < 0.0:
        # Crossing the gate to stay ahead — prefer just behind gate if that
        # is still in front of the aircraft.
        just_behind = -min_st
        if just_behind > along:
            trail = just_behind
        else:
            trail = floor
    return float(trail)


def los_heading_error_deg(drone: DroneState, gate: NedPosition) -> float:
    """Absolute error between drone ground track (or nose) and gate LOS (deg)."""
    dn = float(gate.north_m) - float(drone.position.north_m)
    de = float(gate.east_m) - float(drone.position.east_m)
    n_los = math.hypot(dn, de)
    if n_los <= _EPS:
        return 0.0
    dn, de = dn / n_los, de / n_los
    if (
        drone.speed_mps > _MIN_SPEED_FOR_COURSE
        or drone.velocity.horizontal_speed_mps > _MIN_SPEED_FOR_COURSE
    ):
        vn = float(drone.velocity.vn_mps)
        ve = float(drone.velocity.ve_mps)
    else:
        vn = math.cos(float(drone.heading_rad))
        ve = math.sin(float(drone.heading_rad))
    n_v = math.hypot(vn, ve)
    if n_v <= _EPS:
        return 180.0
    vn, ve = vn / n_v, ve / n_v
    c = max(-1.0, min(1.0, vn * dn + ve * de))
    return float(math.degrees(math.acos(c)))


def intercept_realign_ok(
    drone: DroneState,
    target: TargetState,
    config: TrajectoryConfig,
) -> tuple[bool, float]:
    """INTERCEPT REALIGN: pointing at the live gate closely enough."""
    err = los_heading_error_deg(drone, target.position)
    ok = err <= float(config.realign_course_deg)
    return ok, float(err)


def intercept_geometry_ok(
    drone: DroneState,
    target: TargetState,
    config: TrajectoryConfig,
) -> tuple[bool, float, float, float]:
    """
    INTERCEPT seat check: on course line and pointed at the gate.

    Return (seated, xt_m, along_track_m, los_err_deg).
    """
    xt = cross_track_to_course_m(drone, target)
    along = along_track_m(drone, target)
    los_err = los_heading_error_deg(drone, target.position)
    ok = (
        xt <= float(config.intercept_align_xt_m)
        and los_err <= float(config.intercept_align_los_deg)
    )
    return ok, float(xt), float(along), float(los_err)


def centerline_point_ned(
    target: TargetState,
    along_m: float,
) -> NedPosition | None:
    """Point on the live gate course line at signed along-track ``along_m``."""
    u = target_course_unit_ne(target)
    if u is None:
        return None
    un, ue = u
    g = target.position
    return NedPosition(
        g.north_m + float(along_m) * un,
        g.east_m + float(along_m) * ue,
        g.down_m,
    )


def intercept_centerline_final(
    drone: DroneState,
    target: TargetState,
    config: TrajectoryConfig,
    *,
    lookahead_m: float = 80.0,
) -> NedPosition:
    """
    Live on-line aim for INTERCEPT after engagement is past / abandoned.

    Large XT → point on the course line toward the gate (null cross-track).
    Small XT → live gate (punch-through on centerline).
    """
    along = along_track_m(drone, target)
    xt = cross_track_to_course_m(drone, target)
    align = float(config.intercept_align_xt_m)
    if xt <= align:
        return target.position
    look = max(30.0, float(lookahead_m))
    if along > 1.0:
        aim_along = max(0.0, along - look)
    elif along < -1.0:
        aim_along = along + look
    else:
        aim_along = 0.0
    pt = centerline_point_ned(target, aim_along)
    return pt if pt is not None else target.position



def final_retreats_from_target(
    drone: DroneState,
    target: TargetState,
    final: NedPosition,
) -> bool:
    """True if final lies opposite the live target relative to the drone."""
    to_t = target.position.as_array() - drone.position.as_array()
    to_f = final.as_array() - drone.position.as_array()
    to_t[2] = 0.0
    to_f[2] = 0.0
    if float(np.linalg.norm(to_t)) <= _EPS or float(np.linalg.norm(to_f)) <= _EPS:
        return False
    return float(np.dot(to_t, to_f)) < 0.0


def engagement_final_position(
    p_t: NedPosition,
    u_hat: np.ndarray,
    standoff_m: float,
    *,
    ahead: bool = True,
) -> NedPosition:
    """
    Course-aligned standoff at altitude of ``p_t``.

    ahead=True  → p_t + û * R  (INTERCEPT / head-on)
    ahead=False → p_t - û * R  (FOLLOW / behind)
    """
    u = _horizontal_unit(u_hat)
    sign = 1.0 if ahead else -1.0
    final = p_t.as_array() + sign * u * float(standoff_m)
    final[2] = p_t.down_m
    return NedPosition.from_array(final)


def project_onto_course_line(
    point: NedPosition,
    line_point: NedPosition,
    u_hat: np.ndarray,
) -> NedPosition:
    """Horizontal projection of ``point`` onto line through ``line_point`` || û."""
    u = _horizontal_unit(u_hat)
    w = point.as_array() - line_point.as_array()
    w[2] = 0.0
    s = float(np.dot(w, u))
    proj = line_point.as_array() + s * u
    proj[2] = line_point.down_m
    return NedPosition.from_array(proj)


def _wrap_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _heading_mismatch_rad(drone: DroneState, toward: NedPosition) -> float:
    dn = toward.north_m - drone.position.north_m
    de = toward.east_m - drone.position.east_m
    if math.hypot(dn, de) <= _EPS:
        return 0.0
    desired = math.atan2(de, dn)
    if drone.speed_mps > _MIN_SPEED_FOR_COURSE or drone.velocity.horizontal_speed_mps > _MIN_SPEED_FOR_COURSE:
        current = drone.velocity.course_rad
    else:
        current = float(drone.heading_rad)
    return abs(_wrap_pi(desired - current))


@dataclass
class BehaviorSelector:
    config: TrajectoryConfig

    def propose_intercept_final(
        self,
        target: TargetState,
        drone: DroneState,
        t_400_s: float,
    ) -> NedPosition:
        """Head-on: ahead of target on course at clamped engagement time."""
        t_eng = engagement_horizon_s(self.config, t_400_s)
        p_t = predict_target_position(target, t_eng)
        u = course_unit(target, drone, p_t)
        return engagement_final_position(
            p_t, u, float(self.config.deadline_range_m), ahead=True
        )

    def propose_follow_final(
        self,
        target: TargetState,
        drone: DroneState,
        t_400_s: float,
    ) -> NedPosition:
        """
        On-course FOLLOW aim on the *live* gate: soft trail behind the gate,
        shrunk so the point stays ahead of the drone (see
        ``follow_trail_along_m``).

        Does not CV-extrapolate the gate (unlike INTERCEPT). Predicting the
        gate then placing a trail behind that future point lands ahead of the
        live gate and looks like an intercept cut-in.
        """
        del t_400_s  # FOLLOW geometry is live; horizon is INTERCEPT-only.
        p_t = target.position
        trail_along = follow_trail_along_m(drone, target, self.config)
        u = course_unit(target, drone, p_t)
        u_h = _horizontal_unit(u)
        final = p_t.as_array() + u_h * float(trail_along)
        final[2] = p_t.down_m
        return NedPosition.from_array(final)

    def propose_course_join_intermediate(
        self,
        target: TargetState,
        drone: DroneState,
        t_400_s: float,
        final: NedPosition,
    ) -> Optional[NedPosition]:
        """
        Merge onto the course line, then proceed along-track to the final.

        Not a 90° L-join: the merge point sits *along-track* from the foot of
        the perpendicular so the approach angle to the line is
        ``course_merge_angle_deg`` (~30°).
        """
        t_eng = engagement_horizon_s(self.config, t_400_s)
        p_t = predict_target_position(target, t_eng)
        u = course_unit(target, drone, p_t)
        foot = project_onto_course_line(drone.position, final, u)
        foot = NedPosition(foot.north_m, foot.east_m, final.down_m)
        lateral = horizontal_distance_m(drone.position, foot)
        if lateral < _LATERAL_JOIN_M:
            return None

        u_h = _horizontal_unit(u)
        foot_to_final = final.as_array() - foot.as_array()
        foot_to_final[2] = 0.0
        s_final = float(np.dot(foot_to_final, u_h))

        alpha = math.radians(float(self.config.course_merge_angle_deg))
        alpha = max(math.radians(15.0), min(math.radians(60.0), alpha))
        along = lateral / max(_EPS, math.tan(alpha))

        if s_final < _LATERAL_JOIN_M and s_final > -_LATERAL_JOIN_M:
            lead = max(150.0, min(800.0, along))
            join_arr = final.as_array() - u_h * lead
            join_arr[2] = final.down_m
            join = NedPosition.from_array(join_arr)
        else:
            room = abs(s_final) - _LATERAL_JOIN_M
            if room < _LATERAL_JOIN_M:
                return None
            along = min(along, room)
            join_arr = foot.as_array() + u_h * math.copysign(along, s_final)
            join_arr[2] = final.down_m
            join = NedPosition.from_array(join_arr)

        if distance_m(join, final) < _LATERAL_JOIN_M:
            return None
        if distance_m(drone.position, join) < _LATERAL_JOIN_M:
            return None
        return join

    def propose_follow_intermediate(
        self,
        target: TargetState,
        drone: DroneState,
        t_400_s: float,
        final: NedPosition,
    ) -> Optional[NedPosition]:
        return self.propose_course_join_intermediate(target, drone, t_400_s, final)

    def build_candidate(
        self,
        behavior: BehaviorMode,
        target: TargetState,
        drone: DroneState,
        t_400_s: float,
        *,
        path_planner: PathPlanner | None = None,
        blended_final: NedPosition | None = None,
    ) -> BehaviorCandidate:
        planner = path_planner or PathPlanner(self.config)
        if blended_final is not None:
            final = blended_final
        elif behavior is BehaviorMode.INTERCEPT:
            final = self.propose_intercept_final(target, drone, t_400_s)
        else:
            final = self.propose_follow_final(target, drone, t_400_s)

        intermediate = self.propose_course_join_intermediate(
            target, drone, t_400_s, final
        )

        path = planner.build_path(
            drone,
            final,
            behavior=behavior,
            t_400_s=t_400_s,
            intermediate=intermediate,
        )
        metrics = self.compute_metrics(path, drone, target, t_400_s)
        cost = self.score(path, metrics=metrics)
        path.cost = cost
        return BehaviorCandidate(behavior=behavior, path=path, cost=cost, metrics=metrics)

    def compute_metrics(
        self,
        path: Path,
        drone: DroneState,
        target: TargetState,
        t_400_s: float,
    ) -> dict[str, float]:
        final = path.final_position or drone.position
        length = PathPlanner(self.config).path_length_m(path.waypoints, drone.position)
        available = float(t_400_s) - float(self.config.time_margin_s)
        slack = available - float(path.travel_time_s)
        t_eng = engagement_horizon_s(self.config, t_400_s)
        if path.behavior is BehaviorMode.FOLLOW:
            p_t = target.position
            standoff = float(self.config.follow_standoff_m)
        else:
            p_t = predict_target_position(target, t_eng)
            standoff = float(self.config.deadline_range_m)
        final_err = abs(distance_m(final, p_t) - standoff)
        retreat = 1.0 if final_retreats_from_target(drone, target, final) else 0.0
        return {
            "path_length_m": length,
            "travel_time_s": float(path.travel_time_s),
            "deadline_slack_s": float(slack),
            "heading_mismatch_rad": _heading_mismatch_rad(drone, final),
            "final_error_m": float(final_err),
            "retreat": retreat,
            "feasible": 1.0 if path.feasible else 0.0,
        }

    def score(
        self,
        path: Path,
        *,
        weights: CostWeights | None = None,
        metrics: dict[str, float] | None = None,
    ) -> float:
        """Scalar cost from path metrics + feasibility / retreat."""
        w = weights or CostWeights.from_config(self.config)
        m = metrics
        if m is None:
            m = {
                "path_length_m": 0.0,
                "travel_time_s": float(path.travel_time_s),
                "deadline_slack_s": float(path.t_400_s)
                - float(self.config.time_margin_s)
                - float(path.travel_time_s),
                "heading_mismatch_rad": 0.0,
                "final_error_m": 0.0,
                "retreat": 0.0,
                "feasible": 1.0 if path.feasible else 0.0,
            }

        slack = float(m.get("deadline_slack_s", 0.0))
        slack_penalty = max(0.0, -slack) ** 2

        cost = (
            w.travel_time * float(m.get("travel_time_s", path.travel_time_s))
            + w.path_length * float(m.get("path_length_m", 0.0))
            + w.deadline_slack * slack_penalty
            + w.heading_mismatch * float(m.get("heading_mismatch_rad", 0.0))
            + w.final_error * float(m.get("final_error_m", 0.0))
        )
        if float(m.get("retreat", 0.0)) >= 0.5:
            cost += w.retreat_penalty
        if not path.feasible or float(m.get("feasible", 1.0)) < 0.5:
            cost += w.infeasible_penalty
        return float(cost)

    def select(
        self,
        intercept: BehaviorCandidate,
        follow: BehaviorCandidate,
        *,
        previous: Optional[BehaviorMode] = None,
        previous_switch_time_s: Optional[float] = None,
        now_s: Optional[float] = None,
        course_reliable: bool = True,
        aspect_deg: float | None = None,
    ) -> BehaviorCandidate:
        """
        Pick engagement side for *fast* positioning — but stick once chosen.

        FOLLOW is sticky: once entered, stay FOLLOW unless the FOLLOW path
        retreats / is insane while INTERCEPT is sane. INTERCEPT may still
        switch to FOLLOW after ``behavior_min_hold_s`` with cost margin +
        aspect hysteresis. Feasibility flicker alone must not flip modes.
        """
        if not course_reliable:
            return intercept

        aspect = 90.0 if aspect_deg is None else float(aspect_deg)
        follow_min = float(self.config.follow_min_aspect_deg)
        hyst = max(0.0, float(self.config.behavior_aspect_hysteresis_deg))

        # Aspect hysteresis relative to the current commitment.
        if previous is BehaviorMode.FOLLOW:
            follow_ok_aspect = aspect >= (follow_min - hyst)
        elif previous is BehaviorMode.INTERCEPT:
            follow_ok_aspect = aspect >= (follow_min + hyst)
        else:
            follow_ok_aspect = aspect >= follow_min

        def _sane(c: BehaviorCandidate) -> bool:
            return float(c.metrics.get("retreat", 0.0)) < 0.5

        def _preferred() -> BehaviorCandidate:
            if not follow_ok_aspect:
                if _sane(intercept) or not _sane(follow):
                    return intercept
                return follow
            ic = intercept.cost if _sane(intercept) else intercept.cost + 1.0e6
            fc = follow.cost if _sane(follow) else follow.cost + 1.0e6
            return intercept if ic <= fc else follow

        preferred = _preferred()

        if previous is None or now_s is None or previous_switch_time_s is None:
            return preferred

        prev_cand = intercept if previous is BehaviorMode.INTERCEPT else follow
        alt_cand = follow if previous is BehaviorMode.INTERCEPT else intercept

        # Escape hatch: previous retreats / insane and alternative is sane.
        if not _sane(prev_cand) and _sane(alt_cand):
            return alt_cand

        # FOLLOW sticky: do not leave for INTERCEPT on cost/aspect alone.
        if previous is BehaviorMode.FOLLOW:
            return follow

        hold = float(self.config.behavior_min_hold_s)
        if float(now_s) - float(previous_switch_time_s) < hold:
            return prev_cand

        if preferred.behavior is previous:
            return preferred

        # Major cost win required to flip INTERCEPT → FOLLOW after the hold.
        margin = max(0.0, float(self.config.behavior_switch_cost_margin))
        if preferred.cost + 1e-9 < float(prev_cand.cost) - margin:
            return preferred
        return prev_cand
