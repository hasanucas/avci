"""Target trajectory prediction.

v1: constant-velocity samples over a horizon.
Architecture leaves room for constant-acceleration / EKF later.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.types import DroneState, NedPosition, TargetState

_EPS = 1e-9


@dataclass
class TrajectorySample:
    time_s: float  # seconds from "now" (target.timestamp)
    position: NedPosition


class TrajectoryPredictor:
    """Predict future target positions from filtered state."""

    def __init__(self, config: TrajectoryConfig) -> None:
        self.config = config

    def sample_cv(
        self,
        target: TargetState,
        *,
        horizon_s: float | None = None,
        dt_s: float | None = None,
    ) -> list[TrajectorySample]:
        """
        Constant-velocity samples at 0, dt, 2dt, ... horizon.

        future_position = position + velocity * t
        """
        horizon = float(self.config.predict_horizon_s if horizon_s is None else horizon_s)
        dt = float(self.config.predict_dt_s if dt_s is None else dt_s)
        if horizon < 0.0:
            return []
        dt = max(dt, _EPS)

        samples: list[TrajectorySample] = []
        t = 0.0
        # Inclusive endpoint within floating tolerance.
        while t <= horizon + 1e-12:
            samples.append(TrajectorySample(time_s=t, position=self.position_at(target, t)))
            t += dt
        return samples

    def estimate_t_400(
        self,
        target: TargetState,
        drone: DroneState,
        *,
        deadline_range_m: float | None = None,
        horizon_s: float | None = None,
    ) -> float | None:
        """
        Estimate time until distance(drone, target) == deadline_range_m.

        Uses relative geometry with constant velocities:

            || (p_t - p_d) + (v_t - v_d) * t || = R

        Not the naive (distance - R) / target_speed.

        Returns seconds from now, or None if the relative trajectory never
        reaches the deadline range within the prediction horizon.
        """
        radius = float(
            self.config.deadline_range_m if deadline_range_m is None else deadline_range_m
        )
        horizon = float(
            self.config.t_400_horizon_s if horizon_s is None else horizon_s
        )
        radius = max(0.0, radius)

        r0 = target.position.as_array() - drone.position.as_array()
        v_rel = target.velocity.as_array() - drone.velocity.as_array()
        d0 = float(np.linalg.norm(r0))

        if d0 <= radius + _EPS:
            return 0.0

        # Quadratic: a t^2 + b t + c = 0 for ||r0 + v t||^2 = R^2
        a = float(np.dot(v_rel, v_rel))
        b = float(2.0 * np.dot(r0, v_rel))
        c = float(np.dot(r0, r0) - radius * radius)

        if a <= _EPS:
            # Relative velocity ~0 → range fixed; already handled d0<=R.
            return None

        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None

        sqrt_disc = math.sqrt(disc)
        roots = [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)]
        roots.sort()

        # Prefer first future contact while range is non-increasing (closing).
        closing: list[float] = []
        any_in_horizon: list[float] = []
        for raw_t in roots:
            if raw_t < -1e-6:
                continue
            t = max(0.0, float(raw_t))
            if t > horizon + 1e-9:
                continue
            any_in_horizon.append(t)
            r_t = r0 + v_rel * t
            # d(range)/dt ∝ r·v; <=0 means entering / grazing the sphere.
            if float(np.dot(r_t, v_rel)) <= _EPS:
                closing.append(t)

        if closing:
            return closing[0]
        if any_in_horizon:
            return any_in_horizon[0]
        return None

    def position_at(
        self,
        target: TargetState,
        t_s: float,
    ) -> NedPosition:
        """CV position of target at t seconds from now."""
        p = target.position.as_array() + target.velocity.as_array() * float(t_s)
        return NedPosition.from_array(p)
