"""Speed planning: trapezoidal / triangular profiles separate from geometry.

Goal: reach final position by T_400 without unnecessary max-speed cruise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.coords import distance_m
from fpv_gate_il.trajectory.types import NedPosition, Path, Waypoint

_EPS = 1e-9


def calculate_minimum_travel_time(
    distance_m: float,
    current_speed_mps: float,
    max_speed_mps: float,
    acceleration_mps2: float,
    deceleration_mps2: float,
    *,
    final_speed_mps: float = 0.0,
) -> float:
    """
    Minimum time to cover ``distance_m`` under accel/cruise/decel limits.

    Profile: accelerate → (optional cruise at max_speed) → decelerate to
    ``final_speed_mps``. If distance is short, triangular (no cruise).

    ``current_speed_mps`` is clamped to ``[0, max_speed]`` (conservative).
    """
    distance = max(0.0, float(distance_m))
    if distance <= _EPS:
        return 0.0

    accel = max(float(acceleration_mps2), _EPS)
    decel = max(float(deceleration_mps2), _EPS)
    max_speed = max(float(max_speed_mps), _EPS)
    v0 = min(max(float(current_speed_mps), 0.0), max_speed)
    vf = min(max(float(final_speed_mps), 0.0), max_speed)

    # Pure deceleration covers the whole segment (v0 > vf).
    if v0 > vf + _EPS:
        d_only_dec = (v0 * v0 - vf * vf) / (2.0 * decel)
        if distance <= d_only_dec + _EPS:
            v_end_sq = max(0.0, v0 * v0 - 2.0 * decel * distance)
            v_end = math.sqrt(v_end_sq)
            return (v0 - v_end) / decel

    # Pure acceleration covers the whole segment (vf > v0).
    if vf > v0 + _EPS:
        d_only_acc = (vf * vf - v0 * v0) / (2.0 * accel)
        if distance <= d_only_acc + _EPS:
            v_end_sq = v0 * v0 + 2.0 * accel * distance
            v_end = math.sqrt(max(0.0, v_end_sq))
            return (v_end - v0) / accel

    # Constant speed (v0 ≈ vf > 0) with no accel need beyond cruise.
    if abs(v0 - vf) <= _EPS and abs(v0 - max_speed) <= _EPS:
        return distance / max_speed

    # Peak speed for a triangular profile (accel then decel, no cruise).
    v_peak_sq = (
        2.0 * accel * decel * distance + decel * v0 * v0 + accel * vf * vf
    ) / (accel + decel)
    v_peak = math.sqrt(max(0.0, v_peak_sq))

    if v_peak <= max_speed + _EPS:
        # If peak is below current speed, motion is decelerate-dominated.
        if v_peak < v0 - _EPS:
            v_end_sq = max(0.0, v0 * v0 - 2.0 * decel * distance)
            v_end = math.sqrt(v_end_sq)
            return max(0.0, (v0 - v_end) / decel)
        t_acc = max(0.0, (v_peak - v0) / accel)
        t_dec = max(0.0, (v_peak - vf) / decel)
        return t_acc + t_dec

    # Trapezoid: accel to max_speed, cruise, decel to vf.
    d_acc = max(0.0, (max_speed * max_speed - v0 * v0) / (2.0 * accel))
    d_dec = max(0.0, (max_speed * max_speed - vf * vf) / (2.0 * decel))
    d_cruise = distance - d_acc - d_dec
    if d_cruise < 0.0:
        # Numerical edge: fall back to triangular at vmax boundary.
        t_acc = max(0.0, (max_speed - v0) / accel)
        t_dec = max(0.0, (max_speed - vf) / decel)
        return t_acc + t_dec

    t_acc = max(0.0, (max_speed - v0) / accel)
    t_dec = max(0.0, (max_speed - vf) / decel)
    t_cruise = d_cruise / max_speed
    return t_acc + t_cruise + t_dec


def cruise_speed_for_deadline(
    distance_m: float,
    current_speed_mps: float,
    available_time_s: float,
    max_speed_mps: float,
    acceleration_mps2: float,
    deceleration_mps2: float,
    *,
    final_speed_mps: float = 0.0,
) -> float:
    """
    Lowest cruise speed that still meets ``available_time_s``, capped at max.

    If even max_speed cannot meet the deadline, return max_speed (caller
    marks path infeasible via min travel time check).
    """
    distance = max(0.0, float(distance_m))
    max_speed = max(float(max_speed_mps), _EPS)
    available = float(available_time_s)

    if distance <= _EPS:
        return 0.0
    if available <= _EPS:
        return max_speed

    t_at_max = calculate_minimum_travel_time(
        distance,
        current_speed_mps,
        max_speed,
        acceleration_mps2,
        deceleration_mps2,
        final_speed_mps=final_speed_mps,
    )
    if t_at_max > available:
        return max_speed

    # Binary search lowest v_cruise in [0, max_speed] that meets the time.
    lo = 0.0
    hi = max_speed
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        # Effective max for the profile is the candidate cruise speed.
        t_mid = calculate_minimum_travel_time(
            distance,
            current_speed_mps,
            max(mid, _EPS),
            acceleration_mps2,
            deceleration_mps2,
            final_speed_mps=final_speed_mps,
        )
        if t_mid <= available:
            hi = mid
        else:
            lo = mid
    return hi


@dataclass
class SpeedPlanner:
    config: TrajectoryConfig

    def annotate_path_speeds(
        self,
        path: Path,
        *,
        current_speed_mps: float,
        t_400_s: float,
        start: NedPosition,
        time_margin_s: float | None = None,
    ) -> Path:
        """
        Fill desired_speed / arrival times on waypoints.

        TRACK commanded speed is ``track_cruise_mps`` (fast engagement).
        We do *not* search for the slowest cruise that still meets T_400 —
        that produced ~1 m/s crawls when slack was large.
        """
        if not path.waypoints:
            return path

        cfg = self.config
        margin = float(cfg.time_margin_s if time_margin_s is None else time_margin_s)
        available = max(0.0, float(t_400_s) - margin)
        final_speed = float(cfg.close_approach_speed_mps)

        # Total polyline length from start through waypoints.
        length = 0.0
        cursor = start
        for wp in path.waypoints:
            length += distance_m(cursor, wp.position)
            cursor = wp.position

        cruise = min(float(cfg.max_speed_mps), float(cfg.track_cruise_mps))
        # If even cruise cannot meet the deadline, bump toward max_speed.
        t_at_cruise = calculate_minimum_travel_time(
            length,
            current_speed_mps,
            cruise,
            float(cfg.max_acceleration_mps2),
            float(cfg.max_deceleration_mps2),
            final_speed_mps=final_speed,
        )
        if available > _EPS and t_at_cruise > available:
            cruise = min(
                float(cfg.max_speed_mps),
                max(
                    cruise,
                    cruise_speed_for_deadline(
                        length,
                        current_speed_mps,
                        available,
                        float(cfg.max_speed_mps),
                        float(cfg.max_acceleration_mps2),
                        float(cfg.max_deceleration_mps2),
                        final_speed_mps=final_speed,
                    ),
                ),
            )

        annotated: list[Waypoint] = []
        cursor = start
        speed_cursor = max(0.0, float(current_speed_mps))
        t_cursor = 0.0
        n_wp = len(path.waypoints)
        for i, wp in enumerate(path.waypoints):
            seg = distance_m(cursor, wp.position)
            is_final = i == n_wp - 1
            # Profile may decelerate into final_speed on last segment for timing,
            # but GUIDED setpoint stays at cruise until near the final.
            seg_end_speed = final_speed if is_final else cruise
            seg_max = max(cruise, final_speed, _EPS)
            dt = calculate_minimum_travel_time(
                seg,
                speed_cursor,
                seg_max,
                float(cfg.max_acceleration_mps2),
                float(cfg.max_deceleration_mps2),
                final_speed_mps=seg_end_speed,
            )
            t_cursor += dt
            annotated.append(
                Waypoint(
                    position=wp.position,
                    desired_speed_mps=float(cruise),
                    estimated_arrival_time_s=float(t_cursor),
                )
            )
            cursor = wp.position
            speed_cursor = seg_end_speed

        travel = calculate_minimum_travel_time(
            length,
            current_speed_mps,
            float(cfg.max_speed_mps),
            float(cfg.max_acceleration_mps2),
            float(cfg.max_deceleration_mps2),
            final_speed_mps=final_speed,
        )
        feasible = travel <= available + 1e-6
        notes = path.notes
        if not feasible and "travel_time_exceeds_T400_margin" not in notes:
            notes = (notes + "; " if notes else "") + "travel_time_exceeds_T400_margin"

        return replace(
            path,
            waypoints=annotated,
            travel_time_s=float(travel),
            t_400_s=float(t_400_s),
            feasible=feasible,
            notes=notes,
        )

    def close_approach_speed_mps(self) -> float:
        return float(self.config.close_approach_speed_mps)
