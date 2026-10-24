"""Path planning with timed waypoints and 400 m deadline feasibility.

Path is not only geometry: each waypoint carries desired_speed and
estimated_arrival_time. Final waypoint arrival should be ≤ T_400 − margin.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.coords import distance_m
from fpv_gate_il.trajectory.speed_planner import (
    SpeedPlanner,
    calculate_minimum_travel_time,
)
from fpv_gate_il.trajectory.types import (
    BehaviorMode,
    DroneState,
    NedPosition,
    Path,
    Waypoint,
)

_EPS = 1e-6
_MIN_SEGMENT_M = 5.0


@dataclass
class PathPlanner:
    config: TrajectoryConfig

    def build_path(
        self,
        drone: DroneState,
        final_position: NedPosition,
        *,
        behavior: BehaviorMode,
        t_400_s: float,
        intermediate: Optional[NedPosition] = None,
    ) -> Path:
        """
        Build a simple path: [optional intermediate] → final.

        Annotate each waypoint with desired_speed and estimated_arrival_time.
        Mark feasible iff min travel time ≤ t_400_s − TIME_MARGIN.
        """
        points: list[NedPosition] = []
        if intermediate is not None:
            d_start = distance_m(drone.position, intermediate)
            d_end = distance_m(intermediate, final_position)
            if d_start >= _MIN_SEGMENT_M and d_end >= _MIN_SEGMENT_M:
                points.append(intermediate)
        points.append(final_position)

        waypoints = [
            Waypoint(position=p, desired_speed_mps=0.0, estimated_arrival_time_s=0.0)
            for p in points
        ]
        length = self.path_length_m(waypoints, drone.position)
        cfg = self.config
        v0 = float(drone.speed_mps) if drone.speed_mps > 0.0 else drone.velocity.horizontal_speed_mps
        final_speed = float(cfg.close_approach_speed_mps)
        travel = calculate_minimum_travel_time(
            length,
            v0,
            float(cfg.max_speed_mps),
            float(cfg.max_acceleration_mps2),
            float(cfg.max_deceleration_mps2),
            final_speed_mps=final_speed,
        )
        available = float(t_400_s) - float(cfg.time_margin_s)
        feasible = travel <= available + _EPS

        path = Path(
            waypoints=waypoints,
            behavior=behavior,
            feasible=feasible,
            travel_time_s=float(travel),
            t_400_s=float(t_400_s),
            final_position=final_position,
            notes="" if feasible else "travel_time_exceeds_T400_margin",
        )
        return SpeedPlanner(cfg).annotate_path_speeds(
            path,
            current_speed_mps=v0,
            t_400_s=float(t_400_s),
            start=drone.position,
        )

    def blend_final(
        self,
        previous_final: Optional[NedPosition],
        new_final: NedPosition,
    ) -> NedPosition:
        """EMA / lerp to reduce radar-noise zigzag between replans.

        Large horizontal jumps snap to ``new_final`` so a one-frame KF
        velocity spike cannot poison ``last_final`` for many cycles.
        """
        if previous_final is None:
            return new_final
        dn = float(new_final.north_m) - float(previous_final.north_m)
        de = float(new_final.east_m) - float(previous_final.east_m)
        jump = math.hypot(dn, de)
        snap_m = max(0.0, float(self.config.final_pos_blend_snap_m))
        if snap_m > 0.0 and jump >= snap_m:
            return new_final
        alpha = min(1.0, max(0.0, float(self.config.final_pos_blend)))
        prev = previous_final.as_array()
        nxt = new_final.as_array()
        blended = (1.0 - alpha) * prev + alpha * nxt
        return NedPosition.from_array(blended)

    def path_length_m(self, waypoints: list[Waypoint], start: NedPosition) -> float:
        if not waypoints:
            return 0.0
        total = 0.0
        cursor = start
        for wp in waypoints:
            total += distance_m(cursor, wp.position)
            cursor = wp.position
        return float(total)
